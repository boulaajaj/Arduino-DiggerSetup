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
const int16_t THROTTLE_MAX = 200;       // Cap at 20% for bench safety (out of 1000)
const int16_t STEER_MAX    = 150;       // Steering authority (x0.1%) — not used with curvatureDrive
const int  STEER_INVERT    = 0;         // set to 1 if steering feels reversed

// curvatureDrive tuning (same values as rc_test.ino V5.0)
const float PIVOT_THRESHOLD = 0.10f;    // Below 10% throttle: allow pivot turns
const float PIVOT_SPEED_CAP = 0.45f;    // Max track speed during pivot (45%)
const float REVERSE_LIMIT   = 0.35f;    // Max reverse (straight-line only)

// Closed-loop RPM mode — ON by default (no RC switch available)
bool     rpmMode = true;
int16_t  rpmTargetL = 0;                // smoothed target RPM (what PID chases)
int16_t  rpmTargetR = 0;
const int16_t RPM_MAX = 27000;          // motor no-load max (~2500KV × 11V)
const float   KP_RPM  = 0.02f;          // proportional gain (throttle units per RPM err)

// Stick → RPM shaping
// Expo curve: soft near center (fine control), aggressive toward full stick.
// y = EXPO_LINEAR * x + EXPO_CUBIC * x^3   with EXPO_LINEAR + EXPO_CUBIC = 1
const float EXPO_LINEAR = 0.35f;        // linear term weight
const float EXPO_CUBIC  = 0.65f;        // cubic term weight (more = softer center)

// First-order lag on target RPM — instant stick response with smooth ramp.
// Think of this as "how long to reach 63% of the new target".
const float TAU_ACCEL = 0.30f;          // seconds to ramp UP
const float TAU_DECEL = 0.50f;          // seconds to ramp DOWN (gentler)

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
// Observed: ESC sends DataLen=0x11 (17) in header but actual frame is 22 bytes.
// Interpretation: DataLen counts 16 data bytes + 1 checksum.
const uint8_t TELEM_DATA_LEN = 17;
const uint8_t TELEM_FRAME_LEN = 22;

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

    bool haveFullFrame = false;
    for (int i = 0; i + TELEM_FRAME_LEN <= rxLen; i++) {
      if (rxBuf[i] == HEADER_SLAVE) { haveFullFrame = true; break; }
    }
    if (haveFullFrame) break;
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
  if ((now - lastPrintMs) < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;

  uint32_t avgCycleUs   = cycleTimeN ? (cycleTimeSumUs / cycleTimeN) : 0;
  uint32_t maxCycleUs   = cycleTimeMaxUs;
  float    effectiveHz  = (cycleTimeN * 1000.0f) / (float)PRINT_INTERVAL_MS;
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
  long scaled = (long)deflection * maxOut / usable;
  return (int16_t)constrain(scaled, -maxOut, maxOut);
}

int16_t readJoystickThrottle() {
  rawJoyY = analogRead(JOYSTICK_Y_PIN);
  return scaleAxis(rawJoyY, THROTTLE_MAX);
}

int16_t readJoystickSteer() {
  rawJoyX = analogRead(JOYSTICK_X_PIN);
  int16_t s = scaleAxis(rawJoyX, STEER_MAX);
  return STEER_INVERT ? -s : s;
}

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
  if (m > 1.0f) { left /= m; right /= m; }
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
  Serial.println("# NOTE: USB Serial is shared with D0/D1.");
  Serial.println("# Debug output may be garbled when X.BUS is connected.");
  Serial.println("# Disconnect X.BUS Yellow from D0 to read this output.");
  Serial.println("#");
  Serial.println("# Commands via Serial Monitor:");
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
  delay(200);  // ESC reboots and comes back in BUS mode

  lastControlMs = millis();
}

void loop() {
  uint32_t now = millis();

  // === Control loop at fixed period ===
  if ((now - lastControlMs) >= CONTROL_PERIOD_MS) {
    uint32_t cycleStartUs = micros();
    lastControlMs = now;
    totalPolls++;

    // --- Compute throttle commands ---
    // Stage 1: Read axes, normalize to -1..+1, apply expo for feel
    rawJoyY = analogRead(JOYSTICK_Y_PIN);
    rawJoyX = analogRead(JOYSTICK_X_PIN);
    float normY = scaleAxis(rawJoyY, 1000) / 1000.0f;
    float normX = scaleAxis(rawJoyX, 1000) / 1000.0f;
    float xSpeed    = expoCurve(normY);
    float zRotation = expoCurve(STEER_INVERT ? normX : -normX);

    // Reverse limit (only when going straight)
    if (fabsf(zRotation) < 0.05f && xSpeed < -REVERSE_LIMIT) {
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
      // Stage 3: wheel speeds → static RPM target → smoothed RPM target
      float rpmStaticL = ws.left  * RPM_MAX;
      float rpmStaticR = ws.right * RPM_MAX;

      static float rpmSmoothL = 0.0f, rpmSmoothR = 0.0f;
      const float dts = CONTROL_PERIOD_MS / 1000.0f;
      bool  accelL = fabsf(rpmStaticL) > fabsf(rpmSmoothL);
      bool  accelR = fabsf(rpmStaticR) > fabsf(rpmSmoothR);
      float alphaL = dts / (dts + (accelL ? TAU_ACCEL : TAU_DECEL));
      float alphaR = dts / (dts + (accelR ? TAU_ACCEL : TAU_DECEL));
      rpmSmoothL += (rpmStaticL - rpmSmoothL) * alphaL;
      rpmSmoothR += (rpmStaticR - rpmSmoothR) * alphaR;
      rpmTargetL = (int16_t)rpmSmoothL;
      rpmTargetR = (int16_t)rpmSmoothR;

      // Stage 4: P controller — throttle driven by RPM error
      int32_t actualL = telem[0].valid ? (int32_t)telem[0].rpmHz * 30 : 0;
      int32_t actualR = telem[1].valid ? (int32_t)telem[1].rpmHz * 30 : 0;
      throttle[0] = constrain((int32_t)((rpmTargetL - actualL) * KP_RPM), -THROTTLE_MAX, THROTTLE_MAX);
      throttle[1] = constrain((int32_t)((rpmTargetR - actualR) * KP_RPM), -THROTTLE_MAX, THROTTLE_MAX);
    } else {
      // Open-loop: wheel speeds → scaled throttle directly (no RPM target)
      throttle[0] = constrain((int16_t)(ws.left  * THROTTLE_MAX), -THROTTLE_MAX, THROTTLE_MAX);
      throttle[1] = constrain((int16_t)(ws.right * THROTTLE_MAX), -THROTTLE_MAX, THROTTLE_MAX);
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

    // Alternate telemetry target between ESCs each cycle
    telemetryTarget = (telemetryTarget + 1) % NUM_ESCS;

    // === BUS_MODE watchdog ===
    // If we haven't seen any ESC report BUS_MODE=1 for 2s, resend Restart.
    // Rate-limit to once every 3s so we don't spam.
    if ((now - lastBusModeMs) > 2000 && (now - lastRestartMs) > 3000) {
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
      delay(15);  // 10ms timeout per spec + margin
      printRawRX();
    } else if (cmd == "x") {
      Serial.println("# Restarting all ESCs...");
      sendRestart(0xFF);
    } else if (cmd.startsWith("s ")) {
      // Step response test: "s 150" slams throttle to +150 for STEP_DURATION_MS
      stepValue = constrain(cmd.substring(2).toInt(), -THROTTLE_MAX, THROTTLE_MAX);
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
