// =============================================================================
// X.BUS Telemetry Probe — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// PURPOSE: Determine if the XC E10 ESC's X.BUS wire provides fast enough
// telemetry for closed-loop PID control (dual-loop: inner current + outer RPM).
//
// This sketch:
//   1. Listens on Serial1 (D0 = RX) for X.BUS data from the ESC
//   2. Auto-scans common baud rates to find the right one
//   3. Dumps raw hex bytes to USB Serial for analysis
//   4. Attempts to decode Spektrum X-Bus ESC telemetry packets
//   5. Measures packet arrival rate (Hz) and inter-packet gap (ms)
//   6. Reports RPM, current, voltage, temperature if decoding succeeds
//
// WIRING:
//   ESC X.BUS Yellow wire -> D0 (Serial1 RX) via 5V->3.3V voltage divider
//   ESC X.BUS Brown wire  -> GND (common ground)
//   ESC X.BUS Red wire    -> NOT CONNECTED (BEC+, we don't need it)
//
//   Voltage divider (5V -> 3.3V):
//     Yellow ──[10kΩ]──┬──[6.8kΩ]── GND
//                      └──> D0 (Serial1 RX)
//
//   ⚠️  3.3V WARNING: The UNO Q uses 3.3V logic. If X.BUS outputs 5V,
//   you MUST use a voltage divider. Connecting 5V directly to D0 will
//   damage the STM32U585.
//
//   NOTE: If unsure whether X.BUS is 5V or 3.3V, measure with a multimeter
//   first. If the ESC's BEC+ (red wire) is 5V, assume data is 5V too.
//
// BOARD: Arduino UNO Q [ABX00162]
// FQBN:  arduino:zephyr:unoq
// USB Serial: 115200 baud (monitoring output)
// =============================================================================

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
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {} // Wait for USB serial (max 3s)

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  X.BUS Telemetry Probe — XC E10 ESC");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("Wiring: ESC X.BUS Yellow -> D0 (via 5V->3.3V divider)");
  Serial.println("        ESC X.BUS Brown  -> GND");
  Serial.println("        ESC X.BUS Red    -> NOT CONNECTED");
  Serial.println();
  Serial.println("Make sure ESC is powered on (battery connected).");
  Serial.println();

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
    Serial.println();
    Serial.println("!! No data received at any baud rate.");
    Serial.println("!! Check wiring: Yellow->D0, Brown->GND, ESC powered on.");
    Serial.println("!! Restarting scan in 5 seconds...");
    Serial.println();
    delay(5000);
    currentBaudIdx = 0;
    tryNextBaud();
    return;
  }

  long baud = BAUD_RATES[currentBaudIdx];
  Serial.print("Trying baud rate: ");
  Serial.print(baud);
  Serial.println(" ...");

  Serial1.end();
  Serial1.begin(baud);
  // Flush any garbage
  while (Serial1.available()) Serial1.read();

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
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
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
        Serial.print(">> Data detected at ");
        Serial.print(BAUD_RATES[currentBaudIdx]);
        Serial.print(" baud (");
        Serial.print(bytesReceived);
        Serial.println(" bytes received)");
        Serial.println();

        baudLocked = true;
        phase = PHASE_RAW_DUMP;
        rawDumpStartMs = nowMs;
        packetCount = 0;
        rxLen = 0;

        Serial.println("--- RAW HEX DUMP (5 seconds) ---");
        Serial.println("Look for repeating patterns to identify packet structure.");
        Serial.println();
      } else {
        if (bytesReceived > 0) {
          Serial.print("   (only ");
          Serial.print(bytesReceived);
          Serial.println(" bytes — likely noise, skipping)");
        } else {
          Serial.println("   (no data)");
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
        if (rxBuf[i] < 0x10) Serial.print("0");
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
        if ((i + 1) % 16 == 0) Serial.println();
      }
      rxLen = 0;
    }

    if (nowMs - rawDumpStartMs >= RAW_DUMP_DURATION_MS) {
      Serial.println();
      Serial.println();
      Serial.println("--- END RAW DUMP ---");
      Serial.println();
      Serial.println("Switching to packet decode mode...");
      Serial.println("Looking for Spektrum X-Bus ESC packets (0x20 address).");
      Serial.println();
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

        Serial.print("[PKT #");
        Serial.print(packetCount);
        Serial.print("] ");
        Serial.print("RPM=");
        Serial.print(rpmVal, 0);
        Serial.print("  V=");
        Serial.print(voltVal, 2);
        Serial.print("V  I=");
        Serial.print(curVal, 2);
        Serial.print("A  FET=");
        Serial.print(fetTempVal, 1);
        Serial.print("C  Thr=");
        Serial.print(pkt.throttle * 0.5f, 1);
        Serial.print("%");
        if (gap > 0) {
          Serial.print("  gap=");
          Serial.print(gap / 1000.0f, 1);
          Serial.print("ms (");
          Serial.print(1000000.0f / gap, 1);
          Serial.print("Hz)");
        }
        Serial.println();

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
    Serial.print("[RAW ");
    Serial.print(rxLen);
    Serial.print("B] ");
    int printLen = min(rxLen, 64);
    for (int i = 0; i < printLen; i++) {
      if (rxBuf[i] < 0x10) Serial.print("0");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    if (rxLen > 64) Serial.print("...");
    Serial.println();

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
  Serial.println();
  Serial.println("--- STATUS ---");
  Serial.print("Baud: ");
  Serial.print(BAUD_RATES[currentBaudIdx]);
  Serial.print("  |  Packets decoded: ");
  Serial.print(packetCount);
  Serial.print("  |  Total bytes: ");
  Serial.println(bytesReceived);

  if (packetCount >= 2) {
    Serial.println(">> X.BUS telemetry is WORKING.");
    Serial.println(">> Check the gap/Hz values above to assess update rate.");
    Serial.println(">> For dual-loop PID: need >20Hz for outer RPM loop.");
  } else if (bytesReceived > 100) {
    Serial.println(">> Receiving data but can't decode as Spektrum X-Bus.");
    Serial.println(">> This may be a proprietary XC format.");
    Serial.println(">> Check the RAW hex dump above for repeating patterns.");
    Serial.println(">> Try different packet sizes or contact XC Technology.");
  } else {
    Serial.println(">> Very little data. Check wiring and ESC power.");
  }
  Serial.println("--------------");
  Serial.println();
}
