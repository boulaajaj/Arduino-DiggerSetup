// =============================================================================
// X.BUS Telemetry Probe — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// PURPOSE: Determine if the XC E10 ESC's X.BUS wire provides fast enough
// telemetry for closed-loop PID control (dual-loop: inner current + outer RPM).
//
// This sketch:
//   1. Listens on Serial (USART1, D0 = RX) for X.BUS data from the ESC
//   2. Auto-scans common baud rates to find the right one
//   3. Dumps raw hex bytes to debug serial for analysis
//   4. Attempts to decode Spektrum X-Bus ESC telemetry packets
//   5. Measures packet arrival rate (Hz) and inter-packet gap (ms)
//   6. Reports RPM, current, voltage, temperature if decoding succeeds
//
// SERIAL ARCHITECTURE (UNO Q Zephyr LLEXT firmware):
//   Serial  = USART1 on D0 (RX) / D1 (TX) — used for X.BUS input
//   Serial1 = LPUART1 on PG8 (RX) / PG7 (TX) — available for 2nd ESC (JMISC)
//   Debug   = SoftwareSerial TX on D8 — debug output to USB-serial adapter
//
//   NOTE: Serial (USART1/D0) is shared with the on-board debug probe that
//   provides Serial Monitor via USB. When X.BUS is connected to D0, Serial
//   Monitor is unavailable. For bench testing, unplug X.BUS from D0 to
//   use Serial Monitor directly (change DEBUG_USE_SOFTSERIAL to false).
//
// WIRING:
//   ESC X.BUS Yellow wire -> D0 (USART1 RX) via 5V->3.3V voltage divider
//   ESC X.BUS Brown wire  -> GND (common ground)
//   ESC X.BUS Red wire    -> NOT CONNECTED (BEC+, we don't need it)
//   Debug output: D8 -> USB-to-serial adapter RX
//   Debug GND:    GND -> USB-to-serial adapter GND
//
//   Voltage divider (5V -> 3.3V):
//     Yellow ──[10kΩ]──┬──[6.8kΩ]── GND
//                      └──> D0 (USART1 RX)
//
//   ⚠️  3.3V WARNING: The UNO Q uses 3.3V logic. If X.BUS outputs 5V,
//   you MUST use a voltage divider. Connecting 5V directly to D0 will
//   damage the STM32U585.
//
// BOARD: Arduino UNO Q [ABX00162]
// FQBN:  arduino:zephyr:unoq
// Debug Serial: 115200 baud (via SoftwareSerial on D8, or Serial Monitor on D0)
// =============================================================================

// -----------------------------------------------------------------------------
// Debug output configuration
// Set to true when X.BUS is connected to D0 (use bit-bang TX on D8)
// Set to false for bench testing without X.BUS (use Serial Monitor on D0)
// -----------------------------------------------------------------------------
#define DEBUG_USE_SOFTSERIAL true

#define DEBUG_TX_PIN 8  // D8 — bit-bang TX for debug output

// -----------------------------------------------------------------------------
// Minimal TX-only software serial (no library needed)
// Bit-bangs UART TX at 115200 baud on a GPIO pin.
// Only used for debug output — no RX, no interrupt blocking.
// -----------------------------------------------------------------------------
class SoftTX : public Print {
public:
  void begin(unsigned long baud) {
    _bitTimeUs = 1000000UL / baud;
    pinMode(DEBUG_TX_PIN, OUTPUT);
    digitalWrite(DEBUG_TX_PIN, HIGH); // idle state
    delayMicroseconds(100);
  }

  virtual size_t write(uint8_t b) {
    noInterrupts();
    // Start bit
    digitalWrite(DEBUG_TX_PIN, LOW);
    delayMicroseconds(_bitTimeUs);
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
      digitalWrite(DEBUG_TX_PIN, (b >> i) & 1);
      delayMicroseconds(_bitTimeUs);
    }
    // Stop bit
    digitalWrite(DEBUG_TX_PIN, HIGH);
    delayMicroseconds(_bitTimeUs);
    interrupts();
    return 1;
  }

private:
  unsigned long _bitTimeUs;
};

static SoftTX softDebug;

// Route debug output to bit-bang TX or Serial Monitor
#if DEBUG_USE_SOFTSERIAL
  #define DBG softDebug
  #define XBUS_SERIAL Serial   // X.BUS reads from USART1 (D0)
#else
  #define DBG Serial           // Debug goes to Serial Monitor (D0)
  #define XBUS_SERIAL Serial1  // X.BUS reads from LPUART1 (PG8) — for bench test
#endif

// -----------------------------------------------------------------------------
// Baud rates to try (most likely first)
// -----------------------------------------------------------------------------
const long BAUD_RATES[] = {115200, 100000, 19200, 57600, 38400, 9600};
const int  NUM_BAUDS    = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// -----------------------------------------------------------------------------
// Spektrum X-Bus ESC telemetry packet (address 0x20)
// 16 bytes total, big-endian
// -----------------------------------------------------------------------------
struct XBusESCPacket {
  uint16_t rpm;         // Resolution: 10 RPM per LSB
  uint16_t voltage;     // Resolution: 0.01V per LSB
  uint16_t fetTemp;     // Resolution: 0.1°C per LSB
  uint16_t current;     // Resolution: 10mA per LSB (0.01A)
  uint16_t becTemp;     // Resolution: 0.1°C per LSB
  uint8_t  becCurrent;  // Resolution: 100mA per LSB
  uint8_t  becVoltage;  // Resolution: 0.05V per LSB
  uint8_t  throttle;    // Resolution: 0.5% per LSB
  uint8_t  powerOut;    // Resolution: 0.5% per LSB
};

// -----------------------------------------------------------------------------
// Receive buffer
// -----------------------------------------------------------------------------
const int BUF_SIZE = 256;
uint8_t rxBuf[BUF_SIZE];
int     rxLen = 0;

// -----------------------------------------------------------------------------
// Timing / stats
// -----------------------------------------------------------------------------
unsigned long lastPacketUs   = 0;
unsigned long packetCount    = 0;
unsigned long lastReportUs   = 0;
unsigned long scanStartUs    = 0;
int           currentBaudIdx = 0;
bool          baudLocked     = false;
unsigned long bytesReceived  = 0;

// How long to wait at each baud rate before moving to the next (ms)
const unsigned long SCAN_DWELL_MS = 3000;

// Report interval (ms)
const unsigned long REPORT_INTERVAL_MS = 1000;

// -----------------------------------------------------------------------------
// Phase
// -----------------------------------------------------------------------------
enum Phase { PHASE_SCAN, PHASE_RAW_DUMP, PHASE_DECODE };
Phase phase = PHASE_SCAN;

// After locking baud, dump raw for this long before attempting decode
const unsigned long RAW_DUMP_DURATION_MS = 5000;
unsigned long rawDumpStartMs = 0;

// Packet pattern detection
int consecutiveValidPackets = 0;
int detectedPacketLen = 0;

// =============================================================================
// Setup
// =============================================================================
void setup() {
  DBG.begin(115200);
  delay(1000); // Give debug serial time to initialize

  DBG.println();
  DBG.println("==============================================");
  DBG.println("  X.BUS Telemetry Probe — XC E10 ESC");
  DBG.println("==============================================");
  DBG.println();
#if DEBUG_USE_SOFTSERIAL
  DBG.println("Mode: SoftwareSerial debug on D8");
  DBG.println("X.BUS input: Serial (USART1) on D0");
#else
  DBG.println("Mode: Serial Monitor debug on D0");
  DBG.println("X.BUS input: Serial1 (LPUART1) on PG8");
#endif
  DBG.println();
  DBG.println("Wiring: ESC X.BUS Yellow -> D0 (via 5V->3.3V divider)");
  DBG.println("        ESC X.BUS Brown  -> GND");
  DBG.println("        ESC X.BUS Red    -> NOT CONNECTED");
  DBG.println();
  DBG.println("Make sure ESC is powered on (battery connected).");
  DBG.println();

  startBaudScan();
}

// =============================================================================
// Baud rate scanning
// =============================================================================
void startBaudScan() {
  currentBaudIdx = 0;
  baudLocked = false;
  phase = PHASE_SCAN;
  tryNextBaud();
}

void tryNextBaud() {
  if (currentBaudIdx >= NUM_BAUDS) {
    DBG.println();
    DBG.println("!! No data received at any baud rate.");
    DBG.println("!! Check wiring: Yellow->D0, Brown->GND, ESC powered on.");
    DBG.println("!! Restarting scan in 5 seconds...");
    DBG.println();
    delay(5000);
    currentBaudIdx = 0;
    tryNextBaud();
    return;
  }

  long baud = BAUD_RATES[currentBaudIdx];
  DBG.print("Trying baud rate: ");
  DBG.print(baud);
  DBG.println(" ...");

  XBUS_SERIAL.end();
  XBUS_SERIAL.begin(baud);
  // Flush any garbage
  while (XBUS_SERIAL.available()) XBUS_SERIAL.read();

  bytesReceived = 0;
  rxLen = 0;
  scanStartUs = micros();
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // --- Read all available bytes from X.BUS ---
  while (XBUS_SERIAL.available()) {
    uint8_t b = XBUS_SERIAL.read();
    bytesReceived++;

    if (rxLen < BUF_SIZE) {
      rxBuf[rxLen++] = b;
    }

    if (phase == PHASE_RAW_DUMP || phase == PHASE_DECODE) {
      lastPacketUs = nowUs;
    }
  }

  // --- Phase: Baud scanning ---
  if (phase == PHASE_SCAN) {
    unsigned long elapsed = (nowUs - scanStartUs) / 1000;
    if (elapsed >= SCAN_DWELL_MS) {
      if (bytesReceived > 10) {
        // Got data at this baud rate!
        DBG.print(">> Data detected at ");
        DBG.print(BAUD_RATES[currentBaudIdx]);
        DBG.print(" baud (");
        DBG.print(bytesReceived);
        DBG.println(" bytes received)");
        DBG.println();

        baudLocked = true;
        phase = PHASE_RAW_DUMP;
        rawDumpStartMs = nowMs;
        packetCount = 0;
        rxLen = 0;

        DBG.println("--- RAW HEX DUMP (5 seconds) ---");
        DBG.println("Look for repeating patterns to identify packet structure.");
        DBG.println();
      } else {
        if (bytesReceived > 0) {
          DBG.print("   (only ");
          DBG.print(bytesReceived);
          DBG.println(" bytes — likely noise, skipping)");
        } else {
          DBG.println("   (no data)");
        }
        currentBaudIdx++;
        tryNextBaud();
      }
    }
    return;
  }

  // --- Phase: Raw hex dump ---
  if (phase == PHASE_RAW_DUMP) {
    // Print accumulated bytes as hex
    if (rxLen > 0) {
      for (int i = 0; i < rxLen; i++) {
        if (rxBuf[i] < 0x10) DBG.print("0");
        DBG.print(rxBuf[i], HEX);
        DBG.print(" ");
        if ((i + 1) % 16 == 0) DBG.println();
      }
      rxLen = 0;
    }

    if (nowMs - rawDumpStartMs >= RAW_DUMP_DURATION_MS) {
      DBG.println();
      DBG.println();
      DBG.println("--- END RAW DUMP ---");
      DBG.println();
      DBG.println("Switching to packet decode mode...");
      DBG.println("Looking for Spektrum X-Bus ESC packets (0x20 address).");
      DBG.println();
      phase = PHASE_DECODE;
      rxLen = 0;
      packetCount = 0;
      lastReportUs = nowUs;
    }
    return;
  }

  // --- Phase: Decode ---
  if (phase == PHASE_DECODE) {
    // Try to find and decode packets in the buffer
    tryDecodePackets(nowUs);

    // Periodic report
    if ((nowUs - lastReportUs) / 1000 >= REPORT_INTERVAL_MS) {
      lastReportUs = nowUs;
      printReport(nowUs);
    }
  }
}

// =============================================================================
// Packet decoding
//
// Strategy: look for repeating patterns. Spektrum X-Bus ESC packets are
// 16 bytes at address 0x20. But XC may use a different format.
//
// We try multiple approaches:
//   1. Look for Spektrum header byte patterns
//   2. Look for any repeating byte at regular intervals
//   3. Fall back to timing-based packet detection (gaps between bursts)
// =============================================================================

unsigned long lastByteUs = 0;
unsigned long gapSumUs   = 0;
int           gapCount   = 0;
int           burstLen   = 0;
int           burstLens[32];
int           burstIdx   = 0;

void tryDecodePackets(unsigned long nowUs) {
  if (rxLen < 2) return;

  // --- Approach 1: Spektrum X-Bus ESC format ---
  // Look for 0x20 (ESC address) potentially followed by data
  for (int i = 0; i < rxLen - 15; i++) {
    if (rxBuf[i] == 0x20) {
      // Possible Spektrum ESC packet starting at rxBuf[i]
      // Try to decode 16 bytes starting here
      XBusESCPacket pkt;
      uint8_t *p = &rxBuf[i + 1]; // skip address byte

      // Spektrum uses big-endian
      pkt.rpm      = ((uint16_t)p[0] << 8) | p[1];
      pkt.voltage  = ((uint16_t)p[2] << 8) | p[3];
      pkt.fetTemp  = ((uint16_t)p[4] << 8) | p[5];
      pkt.current  = ((uint16_t)p[6] << 8) | p[7];
      pkt.becTemp  = ((uint16_t)p[8] << 8) | p[9];
      pkt.becCurrent = p[10];
      pkt.becVoltage = p[11];
      pkt.throttle   = p[12];
      pkt.powerOut   = p[13];

      // Sanity check: values should be in reasonable ranges
      float rpmVal = pkt.rpm * 10.0f;
      float voltVal = pkt.voltage * 0.01f;
      float curVal = pkt.current * 0.01f;
      float fetTempVal = pkt.fetTemp * 0.1f;

      bool plausible = (voltVal >= 0.0f && voltVal <= 60.0f)    // battery voltage
                    && (curVal >= 0.0f && curVal <= 300.0f)       // current
                    && (fetTempVal >= -20.0f && fetTempVal <= 200.0f); // temperature

      if (plausible) {
        packetCount++;
        unsigned long gap = 0;
        if (lastPacketUs > 0) {
          gap = nowUs - lastPacketUs;
        }
        lastPacketUs = nowUs;

        DBG.print("[PKT #");
        DBG.print(packetCount);
        DBG.print("] ");
        DBG.print("RPM=");
        DBG.print(rpmVal, 0);
        DBG.print("  V=");
        DBG.print(voltVal, 2);
        DBG.print("V  I=");
        DBG.print(curVal, 2);
        DBG.print("A  FET=");
        DBG.print(fetTempVal, 1);
        DBG.print("C  Thr=");
        DBG.print(pkt.throttle * 0.5f, 1);
        DBG.print("%");
        if (gap > 0) {
          DBG.print("  gap=");
          DBG.print(gap / 1000.0f, 1);
          DBG.print("ms (");
          DBG.print(1000000.0f / gap, 1);
          DBG.print("Hz)");
        }
        DBG.println();

        // Remove consumed bytes
        int consumed = i + 16;
        memmove(rxBuf, rxBuf + consumed, rxLen - consumed);
        rxLen -= consumed;
        return;
      }
    }
  }

  // --- Approach 2: Timing-based burst detection ---
  // If we can't find Spektrum packets, detect packet boundaries by gaps.
  // Serial data arrives in bursts with quiet periods between them.
  // Track burst lengths to determine packet size.
  if (rxLen >= 64) {
    // We've accumulated a lot without finding packets.
    // Dump as hex with gap markers for manual analysis.
    DBG.print("[RAW ");
    DBG.print(rxLen);
    DBG.print("B] ");
    int printLen = min(rxLen, 64);
    for (int i = 0; i < printLen; i++) {
      if (rxBuf[i] < 0x10) DBG.print("0");
      DBG.print(rxBuf[i], HEX);
      DBG.print(" ");
    }
    if (rxLen > 64) DBG.print("...");
    DBG.println();

    // Keep last 32 bytes in case packet straddles the boundary
    if (rxLen > 32) {
      memmove(rxBuf, rxBuf + rxLen - 32, 32);
      rxLen = 32;
    }
  }
}

// =============================================================================
// Periodic report
// =============================================================================
void printReport(unsigned long nowUs) {
  DBG.println();
  DBG.println("--- STATUS ---");
  DBG.print("Baud: ");
  DBG.print(BAUD_RATES[currentBaudIdx]);
  DBG.print("  |  Packets decoded: ");
  DBG.print(packetCount);
  DBG.print("  |  Total bytes: ");
  DBG.println(bytesReceived);

  if (packetCount >= 2) {
    DBG.println(">> X.BUS telemetry is WORKING.");
    DBG.println(">> Check the gap/Hz values above to assess update rate.");
    DBG.println(">> For dual-loop PID: need >20Hz for outer RPM loop.");
  } else if (bytesReceived > 100) {
    DBG.println(">> Receiving data but can't decode as Spektrum X-Bus.");
    DBG.println(">> This may be a proprietary XC format.");
    DBG.println(">> Check the RAW hex dump above for repeating patterns.");
    DBG.println(">> Try different packet sizes or contact XC Technology.");
  } else {
    DBG.println(">> Very little data. Check wiring and ESC power.");
  }
  DBG.println("--------------");
  DBG.println();
}
