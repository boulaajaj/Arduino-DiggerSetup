// ═══════════════════════════════════════════════════════════════
// X.BUS Master — XC E10 ESC Protocol Test
// ═══════════════════════════════════════════════════════════════
//
// Implements the official XC-ESC Technology X.BUS protocol to:
//   1. Send throttle commands to ESCs via broadcast (func 0x50)
//   2. Receive telemetry (RPM, current, voltage, temp) from one ESC
//   3. Display decoded telemetry on Serial Monitor
//
// Protocol: XC X.BUS (modbus-like, master-slave, half-duplex)
//   - 115200 baud, 8N1, little-endian, non-inverted UART
//   - Master sends 0x0F frames, slave responds with 0xF0 frames
//   - ESC never transmits without being polled
//
// Wiring (simplified half-duplex for Nano R4):
//
//                      5V (or 3.3V)
//                       |
//                      [3K-10K pull-up]
//                       |
//   Arduino D1 (TX) --[1K]--+-- ESC X.BUS Yellow wire
//                            |
//   Arduino D0 (RX) --------+
//
//   ESC X.BUS Brown  --> GND (common ground)
//   ESC X.BUS Red    --> NOT CONNECTED (BEC power — never connect both ESCs)
//
// IMPORTANT: D0/D1 are shared with USB Serial. Disconnect X.BUS from D0
// before uploading. Reconnect after upload for testing.
//
// Board: Arduino Nano R4
// FQBN:  arduino:renesas_uno:nanor4
// Port:  COM8

// ═══════════════════════════════════════════════════════════════
// [CONFIG]
// ═══════════════════════════════════════════════════════════════

// Number of ESCs on the bus (our setup: 2, addresses 0 and 1)
const uint8_t NUM_ESCS = 2;

// Which ESC to request telemetry from (alternates each cycle)
uint8_t telemetryTarget = 0;

// Control loop period (ms) — protocol spec says 10-50ms acceptable
const uint32_t CONTROL_PERIOD_MS = 20;  // 50 Hz

// Telemetry response timeout (us) — spec says 2ms for broadcast
const uint32_t TELEM_TIMEOUT_US = 3000;  // 3ms with margin

// Throttle values for bench testing (0 = neutral, safe)
int16_t throttle[2] = {0, 0};  // -1000 to +1000 (x0.1%)

// Debug print interval
const uint32_t PRINT_INTERVAL_MS = 500;


// ═══════════════════════════════════════════════════════════════
// [PROTOCOL] — X.BUS frame construction and parsing
// ═══════════════════════════════════════════════════════════════

// Frame headers
const uint8_t HEADER_MASTER = 0x0F;
const uint8_t HEADER_SLAVE  = 0xF0;

// Addresses
const uint8_t ADDR_BROADCAST = 0xFF;

// Function codes
const uint8_t FUNC_READ_REG    = 0x10;
const uint8_t FUNC_WRITE_REG   = 0x11;
const uint8_t FUNC_THROTTLE    = 0x50;
const uint8_t FUNC_RESTART     = 0xA0;
const uint8_t FUNC_CLEAR_FLAGS = 0xA1;

// Telemetry response length (fixed for func 0x50)
const uint8_t TELEM_DATA_LEN = 17;
// Total response frame: header(1) + SAdr(1) + extra(1) + func(1) + len(1)
//                       + data(17) + checksum(1) = 23 bytes
const uint8_t TELEM_FRAME_LEN = 23;

// Telemetry data structure
struct ESCTelemetry {
  int16_t  rpmHz;           // Output RPM in Hz (multiply by 30 for mech RPM with 4-pole motor)
  int16_t  busCurrent;      // x0.1A
  int16_t  phaseCurrent;    // x0.1A
  uint16_t status;          // Bitfield (see XBUS-PROTOCOL.md)
  int16_t  receivedThrottle;  // x0.1%
  int16_t  outputThrottle;  // x0.1%
  int16_t  busVoltage;      // x0.1V
  int16_t  escTempRaw;      // Actual = raw - 40 degC
  uint8_t  motorTemp;       // Spare/reserved
  bool     valid;           // True if successfully parsed
  uint32_t timestamp;  // micros() when received
};

ESCTelemetry telem[NUM_ESCS];

// TX buffer (max frame: header + addr + extra + func + len + 33 data + checksum = 38)
uint8_t txBuf[40];
// RX buffer
uint8_t rxBuf[40];
int rxLen = 0;


// ═══════════════════════════════════════════════════════════════
// [FRAMING] — Build and send X.BUS frames
// ═══════════════════════════════════════════════════════════════

// Compute checksum: sum of SAdr + Extra byte, low 8 bits
// NOTE: The spec literally says "sum from SAdr to Extra byte (inclusive)".
// This seems too narrow — if it doesn't work, try summing all bytes
// from SAdr through end of data. Both variants are implemented below.
uint8_t checksumNarrow(uint8_t sadr, uint8_t extra) {
  return (sadr + extra) & 0xFF;
}

uint8_t checksumFull(const uint8_t *frame, int len) {
  // Sum everything from byte 1 (SAdr) to byte len-2 (last data byte)
  // frame[0] = header, frame[len-1] = checksum position
  uint8_t sum = 0;
  for (int i = 1; i < len - 1; i++) {
    sum += frame[i];
  }
  return sum;
}

// Encode throttle value into 2-byte little-endian format
// value: -1000 to +1000 (x0.1%), isBrake: false=throttle, true=brake
void encodeThrottle(int16_t value, bool isBrake, uint8_t *out) {
  uint16_t encoded = ((uint16_t)(value & 0x3FFF) << 2) | (isBrake ? 1 : 0);
  out[0] = encoded & 0xFF;        // Low byte
  out[1] = (encoded >> 8) & 0xFF;  // High byte
}

// Send broadcast throttle command and request telemetry from targetESC
void sendThrottleCommand(const int16_t *values, uint8_t numESCs, uint8_t targetESC) {
  int dataLen = numESCs * 2;
  int frameLen = 6 + dataLen;  // header + SAdr + extra + func + len + data + checksum

  txBuf[0] = HEADER_MASTER;
  txBuf[1] = ADDR_BROADCAST;
  txBuf[2] = targetESC;       // Extra byte: ESC to respond with telemetry
  txBuf[3] = FUNC_THROTTLE;
  txBuf[4] = dataLen;

  for (uint8_t i = 0; i < numESCs; i++) {
    encodeThrottle(values[i], false, &txBuf[5 + i * 2]);
  }

  // Try full checksum first (sum of all bytes from SAdr through data)
  txBuf[frameLen - 1] = checksumFull(txBuf, frameLen);

  // Flush any stale RX data before sending
  while (Serial.available()) Serial.read();

  // Send the frame
  Serial.write(txBuf, frameLen);
  Serial.flush();  // Wait for TX to complete before switching to RX
}

// Send a read-register command to a specific ESC (point-to-point)
void sendReadRegister(uint8_t escAddr, uint8_t regAddr) {
  txBuf[0] = HEADER_MASTER;
  txBuf[1] = escAddr;
  txBuf[2] = 0x00;           // Extra: ignored in service control
  txBuf[3] = FUNC_READ_REG;
  txBuf[4] = 1;              // Length: 1 register address
  txBuf[5] = regAddr;
  txBuf[6] = checksumFull(txBuf, 7);

  while (Serial.available()) Serial.read();
  Serial.write(txBuf, 7);
  Serial.flush();
}

// Send restart command to a specific ESC or all (0xFF)
void sendRestart(uint8_t target) {
  txBuf[0] = HEADER_MASTER;
  txBuf[1] = ADDR_BROADCAST;
  txBuf[2] = target;         // Extra: target ESC or 0xFF for all
  txBuf[3] = FUNC_RESTART;
  txBuf[4] = 0;              // Length: 0
  txBuf[5] = checksumFull(txBuf, 6);

  Serial.write(txBuf, 6);
  Serial.flush();
}


// ═══════════════════════════════════════════════════════════════
// [PARSING] — Receive and decode telemetry responses
// ═══════════════════════════════════════════════════════════════

// Wait for and parse a telemetry response frame
// Returns true if a valid frame was received
bool receiveTelemetry(uint8_t expectedESC) {
  uint32_t startUs = micros();
  rxLen = 0;

  // Wait for response bytes within timeout
  while ((micros() - startUs) < TELEM_TIMEOUT_US) {
    while (Serial.available() && rxLen < (int)sizeof(rxBuf)) {
      rxBuf[rxLen++] = Serial.read();
    }

    // Check if we have enough bytes for a complete frame
    if (rxLen >= TELEM_FRAME_LEN) break;
  }

  // Skip our own TX echo (half-duplex: we'll see our transmitted bytes)
  // Look for the slave header (0xF0) in the received data
  int frameStart = -1;
  for (int i = 0; i < rxLen; i++) {
    if (rxBuf[i] == HEADER_SLAVE) {
      frameStart = i;
      break;
    }
  }

  if (frameStart < 0 || (rxLen - frameStart) < TELEM_FRAME_LEN) {
    return false;  // No valid frame found
  }

  uint8_t *frame = &rxBuf[frameStart];

  // Verify frame structure
  if (frame[1] != ADDR_BROADCAST) return false;  // SAdr should be 0xFF
  if (frame[3] != FUNC_THROTTLE)  return false;  // FuncCode should be 0x50
  if (frame[4] != TELEM_DATA_LEN) return false;  // Data length should be 17

  // Verify checksum
  uint8_t expectedSum = checksumFull(frame, TELEM_FRAME_LEN);
  if (frame[TELEM_FRAME_LEN - 1] != expectedSum) {
    // Try narrow checksum as fallback
    uint8_t narrowSum = checksumNarrow(frame[1], frame[2]);
    if (frame[TELEM_FRAME_LEN - 1] != narrowSum) {
      return false;  // Checksum mismatch with both methods
    }
  }

  // Parse telemetry data (starts at frame[5], little-endian)
  const uint8_t *d = &frame[5];
  ESCTelemetry *t = &telem[expectedESC];

  t->rpmHz            = (int16_t)(d[0]  | (d[1]  << 8));
  t->busCurrent       = (int16_t)(d[2]  | (d[3]  << 8));
  t->phaseCurrent     = (int16_t)(d[4]  | (d[5]  << 8));
  t->status           = (uint16_t)(d[6] | (d[7]  << 8));
  t->receivedThrottle = (int16_t)(d[8]  | (d[9]  << 8));
  t->outputThrottle   = (int16_t)(d[10] | (d[11] << 8));
  t->busVoltage       = (int16_t)(d[12] | (d[13] << 8));
  t->escTempRaw       = (int16_t)(d[14] | (d[15] << 8));
  t->motorTemp        = d[16];
  t->valid            = true;
  t->timestamp        = micros();

  return true;
}


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — Telemetry display
// ═══════════════════════════════════════════════════════════════

uint32_t lastPrintMs = 0;
uint32_t totalPolls = 0;
uint32_t goodFrames = 0;
uint32_t badFrames = 0;

// Print status bits in human-readable format
void printStatus(uint16_t status) {
  if (status & (1 << 0))  Serial.print(" OVERVOLT");
  if (status & (1 << 1))  Serial.print(" UNDERVOLT");
  if (status & (1 << 2))  Serial.print(" CURLIMIT");
  if (status & (1 << 3))  Serial.print(" BUS_MODE");
  if (status & (1 << 4))  Serial.print(" THR_LOST");
  if (status & (1 << 5))  Serial.print(" THR_NOTZERO");
  if (status & (1 << 6))  Serial.print(" ESC_OVERTEMP");
  if (status & (1 << 7))  Serial.print(" MOT_OVERTEMP");
  if (status & (1 << 9))  Serial.print(" SENSORED");
  if (status & (1 << 11)) Serial.print(" CAP_CHARGED");
  if (status & (1 << 13)) Serial.print(" POWER_ON");
  if (status & (1 << 14)) Serial.print(" BRAKING");
}

void printTelemetry() {
  uint32_t now = millis();
  if ((now - lastPrintMs) < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;

  Serial.print("# Polls:");
  Serial.print(totalPolls);
  Serial.print(" Good:");
  Serial.print(goodFrames);
  Serial.print(" Bad:");
  Serial.println(badFrames);

  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    ESCTelemetry *t = &telem[i];
    if (!t->valid) {
      Serial.print("# ESC");
      Serial.print(i);
      Serial.println(": no data");
      continue;
    }

    uint32_t age = (micros() - t->timestamp) / 1000;

    Serial.print("# ESC");
    Serial.print(i);
    Serial.print(": RPM=");
    Serial.print(t->rpmHz * 30);  // Convert Hz to mechanical RPM (4-pole)
    Serial.print(" I_bus=");
    Serial.print(t->busCurrent * 0.1f, 1);
    Serial.print("A I_ph=");
    Serial.print(t->phaseCurrent * 0.1f, 1);
    Serial.print("A V=");
    Serial.print(t->busVoltage * 0.1f, 1);
    Serial.print("V T_esc=");
    Serial.print(t->escTempRaw - 40);
    Serial.print("C Thr_in=");
    Serial.print(t->receivedThrottle * 0.1f, 1);
    Serial.print("% Thr_out=");
    Serial.print(t->outputThrottle * 0.1f, 1);
    Serial.print("% age=");
    Serial.print(age);
    Serial.print("ms");
    printStatus(t->status);
    Serial.println();
  }
  Serial.println();
}

// Print raw RX bytes for debugging (call when telemetry parse fails)
void printRawRX() {
  if (rxLen == 0) return;
  Serial.print("# RAW RX (");
  Serial.print(rxLen);
  Serial.print("B): ");
  for (int i = 0; i < min(rxLen, 32); i++) {
    char hex[4];
    snprintf(hex, sizeof(hex), "%02X ", rxBuf[i]);
    Serial.print(hex);
  }
  if (rxLen > 32) Serial.print("...");
  Serial.println();
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

uint32_t lastControlMs = 0;
bool firstReport = true;

void setup() {
  Serial.begin(115200);

  // Wait for USB serial on Nano R4 (up to 2 seconds, then continue)
  uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart) < 2000) {}

  Serial.println();
  Serial.println("# ═══════════════════════════════════════════");
  Serial.println("# X.BUS Master — XC E10 ESC Protocol Test");
  Serial.println("# ═══════════════════════════════════════════");
  Serial.println("#");
  Serial.println("# Protocol: XC-ESC Technology X.BUS V1.0");
  Serial.println("# Baud: 115200, 8N1, little-endian, non-inverted");
  Serial.println("# ESCs: 2 (addresses 0 and 1)");
  Serial.println("# Throttle: 0% (safe — bench test mode)");
  Serial.println("#");
  Serial.println("# WIRING:");
  Serial.println("#   D1 (TX) --[1K]--+-- ESC X.BUS Yellow");
  Serial.println("#   D0 (RX) --------+");
  Serial.println("#   Pull-up: 3K-10K to 5V on bus");
  Serial.println("#   ESC Brown -> GND, Red -> NOT CONNECTED");
  Serial.println("#");
  Serial.println("# NOTE: USB Serial is shared with D0/D1.");
  Serial.println("# Debug output may be garbled when X.BUS is connected.");
  Serial.println("# Disconnect X.BUS Yellow from D0 to read this output.");
  Serial.println("#");
  Serial.println("# Commands via Serial Monitor:");
  Serial.println("#   t0 <val>  — set ESC0 throttle (-1000 to 1000)");
  Serial.println("#   t1 <val>  — set ESC1 throttle (-1000 to 1000)");
  Serial.println("#   r         — read register 0x04 (protocol type)");
  Serial.println("#   x         — restart all ESCs");
  Serial.println("#");

  // Initialize telemetry structs
  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    telem[i].valid = false;
  }

  lastControlMs = millis();
}

void loop() {
  uint32_t now = millis();

  // === Control loop at fixed period ===
  if ((now - lastControlMs) >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    totalPolls++;

    // Send throttle to all ESCs, request telemetry from one
    sendThrottleCommand(throttle, NUM_ESCS, telemetryTarget);

    // Wait for and parse telemetry response
    bool got = receiveTelemetry(telemetryTarget);
    if (got) {
      goodFrames++;
      if (firstReport) {
        Serial.println("# >>> FIRST TELEMETRY RECEIVED! Protocol is working. <<<");
        firstReport = false;
      }
    } else {
      badFrames++;
      // Print raw bytes on first few failures for debugging
      if (badFrames <= 10 || badFrames % 100 == 0) {
        printRawRX();
      }
    }

    // Alternate telemetry target between ESCs each cycle
    telemetryTarget = (telemetryTarget + 1) % NUM_ESCS;
  }

  // === Debug output ===
  printTelemetry();

  // === Serial commands (only works when X.BUS is disconnected) ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("t0 ")) {
      throttle[0] = constrain(cmd.substring(3).toInt(), -1000, 1000);
      Serial.print("# ESC0 throttle set to ");
      Serial.println(throttle[0]);
    } else if (cmd.startsWith("t1 ")) {
      throttle[1] = constrain(cmd.substring(3).toInt(), -1000, 1000);
      Serial.print("# ESC1 throttle set to ");
      Serial.println(throttle[1]);
    } else if (cmd == "r") {
      Serial.println("# Reading register 0x04 (ProtocolType) from ESC 0...");
      sendReadRegister(0, 0x04);
      delay(15);  // 10ms timeout per spec + margin
      printRawRX();
    } else if (cmd == "x") {
      Serial.println("# Restarting all ESCs...");
      sendRestart(0xFF);
    }
  }
}
