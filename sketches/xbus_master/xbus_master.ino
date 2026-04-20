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

// Forward-declared struct for curvatureDrive return (normalized wheel speeds).
// Must be above any function that references it, or Arduino's auto-prototype
// inserts a prototype before the struct definition.
struct WS { float left; float right; };


// ═══════════════════════════════════════════════════════════════
// [CONFIG]
// ═══════════════════════════════════════════════════════════════

// Number of ESCs on the bus (both tracks)
const uint8_t NUM_ESCS = 2;

// Which ESC to request telemetry from (alternates each cycle)
uint8_t telemetryTarget = 0;

// Control loop period (ms) — protocol spec says 10-50ms acceptable
const uint32_t CONTROL_PERIOD_MS = 10;  // 100 Hz

// Telemetry response timeout (us) — spec says 2ms for broadcast.
// Half-duplex bus: RX buffer also contains our own TX echo (~8 bytes),
// so we need to wait for TX(8) + response(23) = 31 bytes at 115200 baud = 2.7ms.
const uint32_t TELEM_TIMEOUT_US = 5000;  // 5ms = comfortable margin

// Joystick config — Y axis on A0 = throttle, X axis on A1 = steering
const int JOYSTICK_Y_PIN   = A0;
const int JOYSTICK_X_PIN   = A1;
const int ADC_RESOLUTION   = 14;        // Nano R4 14-bit ADC
const int ADC_MAX          = 16383;     // 2^14 - 1
const int ADC_CENTER       = 8192;      // ADC mid-scale (~2.5V)
const int ADC_DEADBAND     = 400;       // ±400 counts = small dead zone around center
const int  STEER_INVERT    = 0;         // set to 1 if steering feels reversed

// curvatureDrive tuning (same values as rc_test.ino V5.0)
const float PIVOT_THRESHOLD = 0.10f;    // Below 10% throttle: allow pivot turns
const float PIVOT_SPEED_CAP = 0.45f;    // Max track speed during pivot (45%)
const float REVERSE_LIMIT   = 0.35f;    // Max reverse (straight-line only)

// Plant characterization test — set to true to replace joystick input with a
// scripted throttle schedule for measuring step response. Normally false.
const bool AUTOMATED_TEST = false;

// Closed-loop RPM mode — ON by default (no RC switch available)
bool     rpmMode = true;
int16_t  rpmTargetL = 0;                // target RPM per track
int16_t  rpmTargetR = 0;
// Bench safety cap on commanded RPM target — raise once vehicle is loaded.
// 10,000 mech RPM is reached at about 45% throttle based on measured data.
const int16_t RPM_TARGET_CAP = 10000;

// Stick → RPM shaping
// Expo curve: soft near center (fine control), aggressive toward full stick.
const float EXPO_LINEAR = 0.35f;
const float EXPO_CUBIC  = 0.65f;

// ─── Controller constants (see docs/PLANT-CHARACTERIZATION.md) ───────────
// Layer 1 — stick → target: target below MIN_TARGET_RPM → snap to zero.
const int16_t MIN_TARGET_RPM      = 500;

// Layer 2 — feed-forward table: measured RPM vs throttle on tracks-only load.
// Keyed on TARGET RPM; table returns throttle (x0.1%, 0..1000). Separate
// tables per ESC because ESC1 reads 5–15% higher RPM at the same throttle.
// Table points are the mean-of-5-runs numbers from staircase_5runs.csv.
// 15% throttle is the first reliable region (below that is dead-zone edge).
struct FFPoint { int16_t rpm; int16_t throttle; };

const FFPoint FF_TABLE_L[] = {
  {    0,   0},
  { 2774, 150},  // 15%
  { 4154, 200},  // 20%
  { 5714, 250},  // 25%
  { 6827, 300},  // 30%
  { 8120, 350},  // 35%
  { 9003, 400},  // 40%
  { 9554, 450},  // 45%
  {10377, 500},  // 50%
};
const uint8_t FF_TABLE_L_N = sizeof(FF_TABLE_L) / sizeof(FF_TABLE_L[0]);

// ESC1 reaches more RPM per throttle unit, so its FF table has larger RPM
// values at each throttle row (derived from same 5-run test, right motor).
const FFPoint FF_TABLE_R[] = {
  {    0,   0},
  { 2700, 150},  // 15% — approximated; refine on next capture
  { 4400, 200},  // 20%
  { 6100, 250},  // 25%
  { 7200, 300},  // 30%
  { 8400, 350},  // 35%
  { 9300, 400},  // 40%
  { 9900, 450},  // 45%
  {10800, 500},  // 50%
};
const uint8_t FF_TABLE_R_N = sizeof(FF_TABLE_R) / sizeof(FF_TABLE_R[0]);

// Layer 3 — slow trim
const uint32_t TRIM_UPDATE_MS     = 1000;  // 1 Hz correction
const int16_t  TRIM_DEADBAND_RPM  = 500;   // |error| below this = no change
const int16_t  TRIM_STEP          = 10;    // per-update adjustment (x0.1% throttle)
const int16_t  TRIM_CLAMP         = 50;    // absolute cap on accumulated trim (5%)
int16_t trimL = 0, trimR = 0;              // accumulated trim per ESC
uint32_t lastTrimMs = 0;

// Output safety
const int16_t MAX_THROTTLE_DELTA  = 10;    // 1% slew limit per 10ms cycle
const int16_t DEAD_ZONE_MAX       = 60;    // 0..60 in output = always zero (below 6% dead zone)
const int16_t OUTPUT_CAP          = 500;   // bench cap on FINAL throttle output (50%)

// Drive-feel tuning
const float STRAIGHT_LINE_THRESHOLD = 0.05f;  // |zRotation| below this = "going straight" — applies reverse limiter

// BUS_MODE watchdog (XC flowchart: resend Restart if an ESC drops out of BUS_MODE)
const uint32_t BUS_MODE_TIMEOUT_MS      = 2000;  // No ESC reported BUS_MODE for this long → trigger restart
const uint32_t BUS_MODE_RESTART_RATE_MS = 3000;  // Minimum spacing between Restart broadcasts

// Non-blocking read-register ('r' debug command) timing
const uint32_t READ_REG_TIMEOUT_US = 10000;  // 10 ms per spec + margin

// Step response test state
bool     stepActive   = false;
uint32_t stepStartUs  = 0;
int16_t  stepValue    = 0;
const uint32_t STEP_DURATION_MS = 500;

// Throttle values for bench testing (updated from joystick in loop)
int16_t throttle[2] = {0, 0};  // -1000 to +1000 (x0.1%)
int rawJoyY = 0;                // Raw ADC reading for debug display
int rawJoyX = 0;                // Raw ADC reading for debug display

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
// Observed: ESC sends DataLen=0x11 (17) in header; actual on-wire frame is 22 bytes.
// 5 header bytes (0xF0 + SAdr + Extra + Func + Len) + 16 data bytes + 1 checksum = 22.
// DataLen=17 in the header counts 16 real data bytes + the trailing checksum.
// So we parse only 16 data bytes; byte 17 is the checksum (verified separately).
const uint8_t TELEM_DATA_LEN      = 17;  // as reported by the ESC in the Length field
const uint8_t TELEM_PAYLOAD_BYTES = 16;  // actual telemetry payload bytes (d[0..15])
const uint8_t TELEM_FRAME_LEN     = 22;  // total on-wire bytes (header + payload + checksum)

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
  while (Serial1.available()) Serial1.read();

  // Send the frame
  Serial1.write(txBuf, frameLen);
  Serial1.flush();  // Wait for TX to complete before switching to RX
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

  while (Serial1.available()) Serial1.read();
  Serial1.write(txBuf, 7);
  Serial1.flush();
}

// Send restart command to a specific ESC or all (0xFF)
void sendRestart(uint8_t target) {
  txBuf[0] = HEADER_MASTER;
  txBuf[1] = ADDR_BROADCAST;
  txBuf[2] = target;         // Extra: target ESC or 0xFF for all
  txBuf[3] = FUNC_RESTART;
  txBuf[4] = 0;              // Length: 0
  txBuf[5] = checksumFull(txBuf, 6);

  Serial1.write(txBuf, 6);
  Serial1.flush();
}

// Send clear-flags command (broadcast) — clears latched safety flags
// like THR_NOTZERO so the ESC will accept throttle commands.
void sendClearFlags(uint8_t target) {
  txBuf[0] = HEADER_MASTER;
  txBuf[1] = ADDR_BROADCAST;
  txBuf[2] = target;
  txBuf[3] = FUNC_CLEAR_FLAGS;
  txBuf[4] = 0;
  txBuf[5] = checksumFull(txBuf, 6);

  Serial1.write(txBuf, 6);
  Serial1.flush();
}


// ═══════════════════════════════════════════════════════════════
// [PARSING] — Receive and decode telemetry responses
// ═══════════════════════════════════════════════════════════════

// Wait for and parse a telemetry response frame
// Returns true if a valid frame was received
bool receiveTelemetry(uint8_t expectedESC) {
  uint32_t startUs = micros();
  rxLen = 0;

  // Wait for response bytes within timeout.
  // Half-duplex bus: RX buffer also contains our TX echo. Exit early only
  // when we have located the slave header 0xF0 AND enough bytes follow it
  // to cover a full telemetry frame.
  while ((micros() - startUs) < TELEM_TIMEOUT_US) {
    while (Serial1.available() && rxLen < (int)sizeof(rxBuf)) {
      rxBuf[rxLen++] = Serial1.read();
    }

    // Check full header signature (0xF0 + 0xFF SAdr + 0x50 Func), not just 0xF0.
    // Half-duplex RX sees our TX echo first, and the encoded throttle high byte
    // can be 0xF0 for values near -1000 — matching 0xF0 alone would false-trigger.
    bool haveFullFrame = false;
    for (int i = 0; i + TELEM_FRAME_LEN <= rxLen; i++) {
      if (rxBuf[i]     == HEADER_SLAVE   &&
          rxBuf[i + 1] == ADDR_BROADCAST &&
          rxBuf[i + 3] == FUNC_THROTTLE) {
        haveFullFrame = true;
        break;
      }
    }
    if (haveFullFrame) break;
  }

  // Skip our own TX echo (half-duplex). Match full header signature to avoid
  // false-triggering on an echoed 0xF0 that happens to appear inside a TX byte.
  int frameStart = -1;
  for (int i = 0; i + TELEM_FRAME_LEN <= rxLen; i++) {
    if (rxBuf[i]     == HEADER_SLAVE   &&
        rxBuf[i + 1] == ADDR_BROADCAST &&
        rxBuf[i + 3] == FUNC_THROTTLE) {
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

  // Parse 16 payload bytes (d[0..15]). Byte d[16] / frame[21] is the checksum,
  // already verified above — do not read it as data.
  t->rpmHz            = (int16_t)(d[0]  | (d[1]  << 8));
  t->busCurrent       = (int16_t)(d[2]  | (d[3]  << 8));
  t->phaseCurrent     = (int16_t)(d[4]  | (d[5]  << 8));
  t->status           = (uint16_t)(d[6] | (d[7]  << 8));
  t->receivedThrottle = (int16_t)(d[8]  | (d[9]  << 8));
  t->outputThrottle   = (int16_t)(d[10] | (d[11] << 8));
  t->busVoltage       = (int16_t)(d[12] | (d[13] << 8));
  t->escTempRaw       = (int16_t)(d[14] | (d[15] << 8));
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
uint32_t cycleTimeSumUs = 0;  // rolling sum of cycle times (TX+RX)
uint32_t cycleTimeMaxUs = 0;  // peak cycle time since last print
uint32_t cycleTimeN     = 0;

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
  uint32_t elapsedMs = now - lastPrintMs;
  if (elapsedMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;

  // Use the actual elapsed wall time — loop slip or serial backpressure can
  // stretch the print interval, and a constant divisor would misreport Hz.
  // elapsedMs is guaranteed >= PRINT_INTERVAL_MS by the gate above, so no /0 guard needed.
  uint32_t avgCycleUs  = cycleTimeN ? (cycleTimeSumUs / cycleTimeN) : 0;
  uint32_t maxCycleUs  = cycleTimeMaxUs;
  float    effectiveHz = (float)cycleTimeN * 1000.0f / (float)elapsedMs;
  cycleTimeSumUs = 0; cycleTimeN = 0; cycleTimeMaxUs = 0;

  Serial.print("# Polls:");
  Serial.print(totalPolls);
  Serial.print(" Good:");
  Serial.print(goodFrames);
  Serial.print(" Bad:");
  Serial.print(badFrames);
  Serial.print(" JoyY=");
  Serial.print(rawJoyY);
  Serial.print(" JoyX=");
  Serial.print(rawJoyX);
  Serial.print(" ThrL=");
  Serial.print(throttle[0]);
  Serial.print(" ThrR=");
  Serial.print(throttle[1]);
  Serial.print(" loop_Hz=");
  Serial.print(effectiveHz, 1);
  Serial.print(" cycle_avg=");
  Serial.print(avgCycleUs);
  Serial.print("us max=");
  Serial.print(maxCycleUs);
  Serial.print("us");
  if (rpmMode) {
    Serial.print(" [RPM tgtL=");
    Serial.print(rpmTargetL);
    Serial.print(" tgtR=");
    Serial.print(rpmTargetR);
    Serial.print("]");
  }
  Serial.println();

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
uint32_t lastBusModeMs = 0;   // last time we saw BUS_MODE=1 on any ESC
uint32_t lastRestartMs = 0;   // last time we broadcast Restart
bool firstReport = true;

// Non-blocking 'r' command state (register read)
bool     readRegPending  = false;
uint32_t readRegStartUs  = 0;

// Exponential stick curve — preserves sign, soft near zero, steep at ±1.
// x in [-1..+1], y in [-1..+1]. Same shape as rc_test.ino.
float expoCurve(float x) {
  float a = fabsf(x);
  float sign = (x >= 0.0f) ? 1.0f : -1.0f;
  return sign * (EXPO_LINEAR * a + EXPO_CUBIC * a * a * a);
}

// Generic axis reader: raw ADC → signed deflection with deadband, scaled to ±maxOut
int16_t scaleAxis(int raw, int16_t maxOut) {
  int deflection = raw - ADC_CENTER;
  if (abs(deflection) < ADC_DEADBAND) return 0;
  if (deflection > 0) deflection -= ADC_DEADBAND;
  else                deflection += ADC_DEADBAND;
  const int usable = ADC_CENTER - ADC_DEADBAND;
  int32_t scaled = (int32_t)deflection * maxOut / usable;
  return (int16_t)constrain(scaled, -maxOut, maxOut);
}

// Joystick sampling is done directly in the control loop (single authoritative
// path: analogRead() → scaleAxis() → expoCurve() → curvatureDrive).

// ─── curvatureDrive (WPILib DifferentialDrive.curvatureDriveIK) ───
// At speed: steering ONLY slows the inner track, outer holds throttle.
// At standstill: arcade-style pivot, capped by PIVOT_SPEED_CAP.
// Inputs/outputs in normalized -1..+1 range.
WS curvatureDrive(float xSpeed, float zRotation) {
  xSpeed    = constrain(xSpeed,    -1.0f, 1.0f);
  zRotation = constrain(zRotation, -1.0f, 1.0f);

  float left, right;
  if (fabsf(xSpeed) < PIVOT_THRESHOLD) {
    // Pivot — arcade with cap
    float capped = constrain(zRotation, -PIVOT_SPEED_CAP, PIVOT_SPEED_CAP);
    left  = xSpeed - capped;
    right = xSpeed + capped;
  } else {
    // Curvature — steering scales by |speed|; outer never exceeds throttle
    left  = xSpeed - fabsf(xSpeed) * zRotation;
    right = xSpeed + fabsf(xSpeed) * zRotation;
  }

  // Desaturate: scale down proportionally if either exceeds ±1
  float m = max(fabsf(left), fabsf(right));
  if (m > 1.0f) {
    left /= m;
    right /= m;
  }
  return {left, right};
}

void setup() {
  Serial.begin(115200);    // USB CDC — debug output to Serial Monitor
  Serial1.begin(115200);   // Hardware UART on D0/D1 — X.BUS bus

  analogReadResolution(ADC_RESOLUTION);  // 14-bit ADC on Nano R4

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
  Serial.println("# NOTE: Nano R4 USB Serial (`Serial`) is independent of D0/D1.");
  Serial.println("# X.BUS runs on `Serial1` (D0 RX / D1 TX) and does NOT share");
  Serial.println("# the USB connection, so the Serial Monitor stays readable");
  Serial.println("# while the bus is live — commands below are fully usable.");
  Serial.println("#");
  Serial.println("# Commands via Serial Monitor (work while X.BUS is connected):");
  Serial.println("#   t0 <val>  — set ESC0 throttle (-1000 to 1000)");
  Serial.println("#   t1 <val>  — set ESC1 throttle (-1000 to 1000)");
  Serial.println("#   s  <val>  — step response: slam both ESCs to val for 500ms");
  Serial.println("#   m         — toggle closed-loop RPM mode");
  Serial.println("#   r         — read register 0x04 (protocol type)");
  Serial.println("#   x         — restart all ESCs");
  Serial.println("#");

  // Initialize telemetry structs
  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    telem[i].valid = false;
  }

  // Give ESCs a moment to finish booting, then kick them into BUS_MODE
  // by broadcasting Restart. Per XC flowchart, this activates X.BUS control.
  delay(500);
  Serial.println("# Sending Restart (0xA0) broadcast to activate BUS_MODE...");
  while (Serial1.available()) Serial1.read();
  sendRestart(0xFF);
  delay(1000);  // ESC reboots and comes back in BUS mode — give it a full second

  // Clear any latched safety flags (e.g. THR_NOTZERO) before the test starts.
  // Sent twice with a small gap — reliable even if the first one races the reboot.
  Serial.println("# Clearing safety flags (0xA1) broadcast...");
  sendClearFlags(0xFF);
  delay(100);
  sendClearFlags(0xFF);
  delay(100);

  lastControlMs = millis();
}

// ═══════════════════════════════════════════════════════════════
// [CONTROLLER] — FF + slow trim + slew limit (see docs/PLANT-CHARACTERIZATION.md)
// ═══════════════════════════════════════════════════════════════

// Feed-forward lookup: target RPM → throttle (linear interp over table).
// Handles signed targets — table is for positive RPM; negate both sides for reverse.
int16_t feedForward(int16_t targetRpm, const FFPoint *tbl, uint8_t n) {
  int16_t sign = (targetRpm < 0) ? -1 : 1;
  int16_t rpm  = targetRpm * sign;

  if (rpm <= tbl[0].rpm) return 0;
  if (rpm >= tbl[n - 1].rpm) return tbl[n - 1].throttle * sign;

  for (uint8_t i = 1; i < n; i++) {
    if (rpm < tbl[i].rpm) {
      int16_t rpm_lo = tbl[i - 1].rpm;
      int16_t rpm_hi = tbl[i].rpm;
      int16_t thr_lo = tbl[i - 1].throttle;
      int16_t thr_hi = tbl[i].throttle;
      int32_t interp = thr_lo + (int32_t)(thr_hi - thr_lo) * (rpm - rpm_lo) / (rpm_hi - rpm_lo);
      return (int16_t)(interp * sign);
    }
  }
  return 0;  // unreachable
}

// Apply final-output safety: slew limit and dead-zone skip. `desired` is
// what the controller wants; `current` is what we last sent. Returns the
// new throttle value to command, honoring ≤1% change per cycle and the
// 0..60 dead-zone skip.
int16_t applyFinalOutput(int16_t desired, int16_t current) {
  // Cap the target to the bench output cap
  desired = constrain(desired, -OUTPUT_CAP, OUTPUT_CAP);

  // Slew limit: max MAX_THROTTLE_DELTA change per cycle
  int16_t delta = desired - current;
  if (delta >  MAX_THROTTLE_DELTA) delta =  MAX_THROTTLE_DELTA;
  if (delta < -MAX_THROTTLE_DELTA) delta = -MAX_THROTTLE_DELTA;
  int16_t out = current + delta;

  // Dead-zone skip: if we would command a small non-zero value, send 0 instead.
  // This keeps us from wasting throttle in the 1..6% band where nothing happens.
  if (out > 0 && out <= DEAD_ZONE_MAX) out = 0;
  if (out < 0 && out >= -DEAD_ZONE_MAX) out = 0;

  return out;
}


void loop() {
  uint32_t now = millis();

  // === Non-blocking 'r' register read completion ===
  // Debug-only path: we collect the register response across loop iterations
  // instead of delay()ing. While pending, we suppress the control-loop branch
  // below (not the whole loop()) so serial commands and prints still flow.
  // At 10 ms timeout this skips at most one control cycle — well inside the
  // ESC's own THR_LOST grace window.
  if (readRegPending) {
    while (Serial1.available() && rxLen < (int)sizeof(rxBuf)) {
      rxBuf[rxLen++] = Serial1.read();
    }
    if ((micros() - readRegStartUs) >= READ_REG_TIMEOUT_US) {
      printRawRX();
      readRegPending = false;
      lastControlMs = now;  // resync so the next control cycle fires on schedule
    }
  }

  // === Control loop at fixed period (skipped while register read is pending) ===
  if (!readRegPending && (now - lastControlMs) >= CONTROL_PERIOD_MS) {
    uint32_t cycleStartUs = micros();
    lastControlMs = now;
    totalPolls++;

    // --- AUTOMATED TEST: scripted throttle schedule for plant characterization.
    // Ignores joystick. Both ESCs receive the same throttle. CSV log captures
    // every cycle so we can measure step response (how long until RPM reaches
    // steady state and what steady-state RPM each throttle produces).
    if (AUTOMATED_TEST) {
      static uint32_t testStartMs = 0;
      if (testStartMs == 0) testStartMs = now;
      uint32_t tMs = now - testStartMs;

      // Dead-zone verification test — hold each low throttle for 3s with 2s rest.
      // Runs 3%, 4%, 5%, 6%, 7%, 8%. Twice for consistency.
      // 5-second zero baseline up front so ESCs fully arm.
      const uint32_t BASELINE_MS = 5000;
      const uint32_t HOLD_MS     = 3000;
      const uint32_t REST_MS     = 2000;
      const uint32_t SLOT_MS     = HOLD_MS + REST_MS;  // 5000 ms per throttle level
      // throttle values tested (x0.1%) — 3% to 8% in 1% increments
      const int16_t LEVELS[] = {30, 40, 50, 60, 70, 80};
      const uint32_t N_LEVELS = sizeof(LEVELS) / sizeof(LEVELS[0]);
      const uint32_t N_RUNS   = 2;  // repeat the full 3%→8% sweep

      int16_t schedThrottle;
      if (tMs < BASELINE_MS) {
        schedThrottle = 0;
      } else {
        uint32_t off      = tMs - BASELINE_MS;
        uint32_t totalRun = SLOT_MS * N_LEVELS;
        uint32_t runIdx   = off / totalRun;
        if (runIdx >= N_RUNS) {
          schedThrottle = 0;
        } else {
          uint32_t within  = off % totalRun;
          uint32_t slot    = within / SLOT_MS;       // which level 0..5
          uint32_t slotOff = within % SLOT_MS;
          if (slotOff < HOLD_MS) {
            schedThrottle = LEVELS[slot];
          } else {
            schedThrottle = 0;  // rest between levels
          }
        }
      }

      throttle[0] = schedThrottle;
      throttle[1] = schedThrottle;
      rpmTargetL  = schedThrottle;  // logged in CSV so we can see the command
      rpmTargetR  = schedThrottle;
      rawJoyY     = 0;
      rawJoyX     = 0;

      sendThrottleCommand(throttle, NUM_ESCS, telemetryTarget);
      bool gotT = receiveTelemetry(telemetryTarget);
      if (gotT) goodFrames++; else badFrames++;

      // Cycle timing
      uint32_t cycleUs = micros() - cycleStartUs;
      cycleTimeSumUs += cycleUs;
      cycleTimeN++;
      if (cycleUs > cycleTimeMaxUs) cycleTimeMaxUs = cycleUs;

      // CSV log (same format as live run)
      uint32_t tNowUs2 = micros();
      int32_t actL_c = telem[0].valid ? (int32_t)telem[0].rpmHz * 30 : 0;
      int32_t actR_c = telem[1].valid ? (int32_t)telem[1].rpmHz * 30 : 0;
      uint32_t ageL_c = telem[0].valid ? (tNowUs2 - telem[0].timestamp) / 1000 : 9999;
      uint32_t ageR_c = telem[1].valid ? (tNowUs2 - telem[1].timestamp) / 1000 : 9999;
      Serial.print("CSV,");
      Serial.print(now); Serial.print(',');
      Serial.print(rawJoyY); Serial.print(',');
      Serial.print(rawJoyX); Serial.print(',');
      Serial.print(rpmTargetL); Serial.print(',');
      Serial.print(actL_c); Serial.print(',');
      Serial.print(throttle[0]); Serial.print(',');
      Serial.print(ageL_c); Serial.print(',');
      Serial.print(rpmTargetR); Serial.print(',');
      Serial.print(actR_c); Serial.print(',');
      Serial.print(throttle[1]); Serial.print(',');
      Serial.println(ageR_c);

      telemetryTarget = (telemetryTarget + 1) % NUM_ESCS;
      return;  // Skip the normal joystick/closed-loop path entirely
    }

    // --- Compute throttle commands ---
    // Stage 1: Read axes, normalize to -1..+1, apply expo for feel
    rawJoyY = analogRead(JOYSTICK_Y_PIN);
    rawJoyX = analogRead(JOYSTICK_X_PIN);
    float normY = scaleAxis(rawJoyY, 1000) / 1000.0f;
    float normX = scaleAxis(rawJoyX, 1000) / 1000.0f;
    float xSpeed    = expoCurve(normY);
    float zRotation = expoCurve(STEER_INVERT ? normX : -normX);

    // Reverse limit (only when going straight)
    if (fabsf(zRotation) < STRAIGHT_LINE_THRESHOLD && xSpeed < -REVERSE_LIMIT) {
      xSpeed = -REVERSE_LIMIT;
    }

    // Stage 2: curvatureDrive → normalized wheel speeds
    WS ws = curvatureDrive(xSpeed, zRotation);

    if (stepActive) {
      // Step response override: both tracks slammed to stepValue (bypasses all shaping)
      throttle[0] = stepValue;
      throttle[1] = stepValue;
      if ((micros() - stepStartUs) / 1000 > STEP_DURATION_MS) {
        stepActive = false;
        Serial.println("# step response complete");
      }
    } else if (rpmMode) {
      // ── Layer 1: wheel speeds → target RPM ──────────────────────────
      rpmTargetL = (int16_t)(ws.left  * RPM_TARGET_CAP);
      rpmTargetR = (int16_t)(ws.right * RPM_TARGET_CAP);
      if (abs(rpmTargetL) < MIN_TARGET_RPM) rpmTargetL = 0;
      if (abs(rpmTargetR) < MIN_TARGET_RPM) rpmTargetR = 0;

      // Reset trim on direction reversal — the error would otherwise be huge
      // and wrong-signed during the zero-crossing, saturating the trim.
      static int16_t prevSignL = 0, prevSignR = 0;
      int16_t signL = (rpmTargetL > 0) - (rpmTargetL < 0);
      int16_t signR = (rpmTargetR > 0) - (rpmTargetR < 0);
      if (signL != prevSignL) trimL = 0;
      if (signR != prevSignR) trimR = 0;
      prevSignL = signL;
      prevSignR = signR;

      // ── Layer 2: feed-forward ───────────────────────────────────────
      int16_t ffL = feedForward(rpmTargetL, FF_TABLE_L, FF_TABLE_L_N);
      int16_t ffR = feedForward(rpmTargetR, FF_TABLE_R, FF_TABLE_R_N);

      // ── Layer 3: slow trim update (1 Hz, separate from 100 Hz control) ──
      if ((now - lastTrimMs) >= TRIM_UPDATE_MS) {
        lastTrimMs = now;

        int32_t actualL = telem[0].valid ? (int32_t)telem[0].rpmHz * 30 : 0;
        int32_t actualR = telem[1].valid ? (int32_t)telem[1].rpmHz * 30 : 0;

        // Only trim when we're actually commanding motion — below the
        // target-RPM threshold, leave trim at 0 so rest-state is clean.
        if (rpmTargetL == 0) {
          trimL = 0;
        } else {
          int32_t errL = (int32_t)rpmTargetL - actualL;
          if      (errL >  TRIM_DEADBAND_RPM) trimL += TRIM_STEP;
          else if (errL < -TRIM_DEADBAND_RPM) trimL -= TRIM_STEP;
          trimL = constrain(trimL, (int16_t)-TRIM_CLAMP, (int16_t)TRIM_CLAMP);
        }
        if (rpmTargetR == 0) {
          trimR = 0;
        } else {
          int32_t errR = (int32_t)rpmTargetR - actualR;
          if      (errR >  TRIM_DEADBAND_RPM) trimR += TRIM_STEP;
          else if (errR < -TRIM_DEADBAND_RPM) trimR -= TRIM_STEP;
          trimR = constrain(trimR, (int16_t)-TRIM_CLAMP, (int16_t)TRIM_CLAMP);
        }
      }

      // ── Combine FF + trim → desired, then apply slew + dead-zone skip ──
      int16_t desiredL = (rpmTargetL == 0) ? 0 : ffL + trimL * signL;
      int16_t desiredR = (rpmTargetR == 0) ? 0 : ffR + trimR * signR;
      throttle[0] = applyFinalOutput(desiredL, throttle[0]);
      throttle[1] = applyFinalOutput(desiredR, throttle[1]);

    } else {
      // Open-loop: wheel speeds → scaled throttle directly (no RPM target)
      int16_t desiredL = (int16_t)(ws.left  * OUTPUT_CAP);
      int16_t desiredR = (int16_t)(ws.right * OUTPUT_CAP);
      throttle[0] = applyFinalOutput(desiredL, throttle[0]);
      throttle[1] = applyFinalOutput(desiredR, throttle[1]);
    }

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
      // Track BUS_MODE (bit 3) for watchdog
      if (telem[telemetryTarget].status & (1 << 3)) {
        lastBusModeMs = now;
      }
    } else {
      badFrames++;
      // Print raw bytes on first few failures for debugging
      if (badFrames <= 10 || badFrames % 100 == 0) {
        printRawRX();
      }
    }

    // Cycle time measurement (TX + telemetry wait + parse)
    uint32_t cycleUs = micros() - cycleStartUs;
    cycleTimeSumUs += cycleUs;
    cycleTimeN++;
    if (cycleUs > cycleTimeMaxUs) cycleTimeMaxUs = cycleUs;

    // Step response per-cycle log (RPM every cycle while step active)
    if (stepActive && telem[telemetryTarget].valid) {
      uint32_t tMs = (micros() - stepStartUs) / 1000;
      Serial.print("# step t=");
      Serial.print(tMs);
      Serial.print("ms ESC");
      Serial.print(telemetryTarget);
      Serial.print(" RPM=");
      Serial.print(telem[telemetryTarget].rpmHz * 30);
      Serial.print(" Thr_out=");
      Serial.println(telem[telemetryTarget].outputThrottle * 0.1f, 1);
    }

    // Per-cycle CSV log — one row every 10 ms (100 Hz).
    // Columns: t_ms, joyY, joyX, tgtL, actL, thrL, ageL_ms, tgtR, actR, thrR, ageR_ms
    // "age" is how old the last valid telemetry for that ESC is — tells you
    // when a row reflects stale data vs. a fresh reading.
    uint32_t tNowUs = micros();
    int32_t actL_csv = telem[0].valid ? (int32_t)telem[0].rpmHz * 30 : 0;
    int32_t actR_csv = telem[1].valid ? (int32_t)telem[1].rpmHz * 30 : 0;
    uint32_t ageL_csv = telem[0].valid ? (tNowUs - telem[0].timestamp) / 1000 : 9999;
    uint32_t ageR_csv = telem[1].valid ? (tNowUs - telem[1].timestamp) / 1000 : 9999;
    Serial.print("CSV,");
    Serial.print(now); Serial.print(',');
    Serial.print(rawJoyY); Serial.print(',');
    Serial.print(rawJoyX); Serial.print(',');
    Serial.print(rpmTargetL); Serial.print(',');
    Serial.print(actL_csv); Serial.print(',');
    Serial.print(throttle[0]); Serial.print(',');
    Serial.print(ageL_csv); Serial.print(',');
    Serial.print(rpmTargetR); Serial.print(',');
    Serial.print(actR_csv); Serial.print(',');
    Serial.print(throttle[1]); Serial.print(',');
    Serial.println(ageR_csv);

    // Alternate telemetry target between ESCs each cycle
    telemetryTarget = (telemetryTarget + 1) % NUM_ESCS;

    // === BUS_MODE watchdog ===
    // If we haven't seen any ESC report BUS_MODE=1 for 2s, resend Restart.
    // Rate-limit to once every 3s so we don't spam.
    if ((now - lastBusModeMs) > BUS_MODE_TIMEOUT_MS &&
        (now - lastRestartMs) > BUS_MODE_RESTART_RATE_MS) {
      Serial.println("# BUS_MODE lost — broadcasting Restart (0xA0)...");
      sendRestart(0xFF);
      lastRestartMs = now;
    }
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
      readRegPending = true;
      readRegStartUs = micros();
      rxLen = 0;  // start fresh so the poll collects only the register response
    } else if (cmd == "x") {
      Serial.println("# Restarting all ESCs...");
      sendRestart(0xFF);
    } else if (cmd.startsWith("s ")) {
      // Step response test: "s 150" slams throttle to +150 for STEP_DURATION_MS
      stepValue = constrain(cmd.substring(2).toInt(), -OUTPUT_CAP, OUTPUT_CAP);
      stepStartUs = micros();
      stepActive = true;
      Serial.print("# STEP RESPONSE: throttle=");
      Serial.print(stepValue);
      Serial.print(" for ");
      Serial.print(STEP_DURATION_MS);
      Serial.println("ms");
    } else if (cmd == "m") {
      rpmMode = !rpmMode;
      Serial.print("# RPM closed-loop mode: ");
      Serial.println(rpmMode ? "ON" : "OFF");
    }
  }
}
