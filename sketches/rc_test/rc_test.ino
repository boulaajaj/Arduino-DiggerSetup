// =============================================================================
// Tank Mixer V2.1 — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// Dual-Loop PID Control — Inner Current (Feedforward) + Outer RPM (Feedback)
//
// Migrated from V2.0 (single-loop current-only PID).
//
// V2.1 adds:
//   - X.BUS serial telemetry from ESC (RPM, current, voltage, temp)
//   - Dual-loop PID: inner current loop detects load spikes instantly,
//     outer RPM loop confirms actual speed and fine-tunes compensation
//   - Hall sensor tap fallback for RPM if X.BUS is too slow
//   - RPM source auto-detection (X.BUS vs hall sensor)
//
// Retained from V2.0:
//   - Exponential response curve (pow 2.5) for fine low-stick control
//   - Inertia simulation (spring-damper model — heavy machine feel)
//   - Soft power limits (tanh saturation, no hard clamps)
//   - Anti-runaway failsafe (2s at max boost → neutral, latch until centered)
//   - 3-mode override switch (RC only / RC+joy / 50-50 blend)
//
// Hardware:
//   MCU:       STM32U585 (Arm Cortex-M33, 160MHz, 786KB RAM, 2MB flash)
//   ADC:       14-bit (0-16383), 3.3V reference
//   Logic:     3.3V — DO NOT connect 5V signals directly to any pin!
//   ESCs:      XC E10 Sensored Brushless 140A (x2)
//   Motors:    XC E3665 2500KV Sensored Brushless (x2)
//   Battery:   OVONIC 3S LiPo 15000mAh 130C 11.1V
//   RC:        Radiolink RC6GS V3 + R7FG gyro receiver
//   Joystick:  Genie 101174GT dual-axis (5V, needs divider to 3.3V)
//   Current:   CS7581 Hall-effect current sensor (x2)
//   X.BUS:     ESC telemetry wire (Yellow=data via 5V->3.3V divider)
//
// Inputs:
//   D0  <- ESC X.BUS Left     (Serial1 RX, via 5V->3.3V divider)
//   D2  <- RC CH1 -> Left motor   (servo PWM 1000-2000us, already mixed)
//   D4  <- RC CH2 -> Right motor  (servo PWM 1000-2000us, already mixed)
//   D5  <- Motor Hall Tap LEFT    (RPM fallback, via 5V->3.3V divider)
//   D6  <- Motor Hall Tap RIGHT   (RPM fallback, via 5V->3.3V divider)
//   D7  <- RC CH5 Override        (servo PWM, 3-position switch)
//   A0  <- Joystick Y axis        (analog 0-3.3V via divider, throttle)
//   A1  <- Joystick X axis        (analog 0-3.3V via divider, steering)
//   A2  <- CS7581 current LEFT    (analog, hall-effect)
//   A3  <- CS7581 current RIGHT   (analog, hall-effect)
//
// Outputs:
//   D9  -> Left track ESC    (servo PWM 1000-2000us)
//   D10 -> Right track ESC   (servo PWM 1000-2000us)
//
// Override modes (CH5):
//   LOW  (<1250us)      Mode 1: RC only, joystick disabled
//   MID  (1250-1750us)  Mode 2: Joystick active, RC overrides when non-neutral
//   HIGH (>1750us)      Mode 3: 50/50 blend of RC + joystick
//
// =============================================================================

#include <Servo.h>

// =============================================================================
// RPM Source Selection
//
// Set ONE of these to 1 to choose the RPM source. Start with XBUS.
// If X.BUS is too slow or doesn't work, switch to HALL.
// =============================================================================
#define RPM_SOURCE_XBUS  1    // ESC X.BUS serial telemetry on D0 (Serial1)
#define RPM_SOURCE_HALL  0    // Direct motor hall sensor tap on D5/D6

// X.BUS baud rate — determined by xbus_probe sketch (change after probing)
#define XBUS_BAUD        115200

// =============================================================================
// Pin Assignments
// =============================================================================
const uint8_t PIN_RC_LEFT     = 2;
const uint8_t PIN_RC_RIGHT    = 4;
const uint8_t PIN_RC_OVERRIDE = 7;
const uint8_t PIN_JOY_Y       = A0;   // Via 5V->3.3V voltage divider
const uint8_t PIN_JOY_X       = A1;   // Via 5V->3.3V voltage divider
const uint8_t PIN_CUR_LEFT    = A2;   // CS7581 left motor
const uint8_t PIN_CUR_RIGHT   = A3;   // CS7581 right motor
const uint8_t PIN_ESC_LEFT    = 9;
const uint8_t PIN_ESC_RIGHT   = 10;

#if RPM_SOURCE_HALL
const uint8_t PIN_HALL_LEFT   = 5;    // Motor hall tap, via 5V->3.3V divider
const uint8_t PIN_HALL_RIGHT  = 6;    // Motor hall tap, via 5V->3.3V divider
#endif

// =============================================================================
// ADC (14-bit, 3.3V reference)
// =============================================================================
const int   ADC_MAX    = 16383;
const int   ADC_CENTER = 8192;
const float ADC_VOLTS  = 3.3f;

// =============================================================================
// Joystick
// =============================================================================
const int   JOY_DEADBAND = 480;    // ~3% of 14-bit range
const float EXP_CURVE    = 2.5f;   // Expo exponent (1=linear, 2.5=excavator)

// =============================================================================
// RC
// =============================================================================
const int RC_DEADBAND      = 50;
const int SERVO_CENTER     = 1500;
const int SERVO_MIN        = 1000;
const int SERVO_MAX        = 2000;
const int MODE_LOW_THRESH  = 1250;
const int MODE_HIGH_THRESH = 1750;

// =============================================================================
// Power Limits — soft, not hard
// =============================================================================
const int   POWER_LIMIT_PCT = 50;
const float SOFT_LIMIT_RANGE = 500.0f * POWER_LIMIT_PCT / 100.0f; // 250us

// =============================================================================
// Inertia Simulation
//
// Makes output feel heavy. Spring-damper model.
//   VIRTUAL_MASS:    Higher = more sluggish
//   FRICTION_COEFF:  Higher = stops sooner
//   RESPONSE_FORCE:  Higher = tracks stick faster
// =============================================================================
const float VIRTUAL_MASS   = 3.0f;
const float FRICTION_COEFF = 8.0f;
const float RESPONSE_FORCE = 20.0f;

// =============================================================================
// CS7581 Current Sensor
//
// Adjust to match your CS7581 variant:
//   CUR_ZERO_V:    Output voltage at 0A (typically Vcc/2)
//   CUR_SENS_MV_A: Sensitivity in mV per Amp
//   CUR_DIVIDER:   Voltage divider ratio (1.0 = no divider)
// =============================================================================
const float CUR_ZERO_V    = 1.65f;
const float CUR_SENS_MV_A = 20.0f;
const float CUR_DIVIDER   = 1.0f;

// =============================================================================
// Dual-Loop PID — Inner Current (Feedforward) + Outer RPM (Feedback)
//
// INNER LOOP — Current-Based Feedforward (runs every main loop iteration):
//   Current sensors detect load changes INSTANTLY. When a track hits mud or
//   an obstacle, current spikes BEFORE RPM drops. The inner loop sees the
//   spike and pre-boosts power to compensate before the operator feels it.
//
// OUTER LOOP — RPM-Based Feedback (runs when new RPM data arrives):
//   RPM measurement (X.BUS or hall sensor) tells us actual motor speed.
//   This closes the loop: did the feedforward actually maintain speed?
//   If not, the outer loop applies a trim correction.
//
// Together: current predicts the disturbance, RPM confirms the recovery.
// The operator feels nothing — the system absorbed it before it was
// perceptible to a human.
// =============================================================================

// --- Inner loop (current feedforward) ---
const float CURRENT_PER_UNIT = 0.25f;  // Expected A per us of output from center
const float FF_MAX_BOOST     = 50.0f;  // Max us the feedforward can add
const float FF_KP            = 0.3f;   // Proportional: immediate correction
const float FF_KI            = 0.05f;  // Integral: persistent load
const float FF_KD            = 0.01f;  // Derivative: dampen oscillation
const float FF_I_MAX         = 30.0f;  // Integral windup limit (us)

// --- Outer loop (RPM feedback) ---
const float RPM_MAX_TRIM     = 40.0f;  // Max us the RPM loop can add/subtract
const float RPM_KP           = 0.15f;  // Proportional — gentler than inner loop
const float RPM_KI           = 0.03f;  // Integral — slow drift correction
const float RPM_KD           = 0.005f; // Derivative — smooth transitions
const float RPM_I_MAX        = 25.0f;  // Integral windup limit (us)

// Stick-to-RPM mapping (calibrate on your machine!)
// At full stick (250us delta), target RPM = MAX_TARGET_RPM.
// Linear mapping: targetRPM = (absCommandDelta / SOFT_LIMIT_RANGE) * MAX_TARGET_RPM
const float MAX_TARGET_RPM   = 5000.0f;  // RPM at full stick (adjust after testing)

// Hard current safety cutoff — if current exceeds this, stop the motor
// regardless of compensation. Last-resort safety net.
const float CURRENT_HARD_LIMIT = 100.0f;  // Amps — force neutral

// Anti-runaway: if total boost maxed out for this long, track is stuck.
const unsigned long RUNAWAY_TIME_US = 2000000UL; // 2 seconds

// =============================================================================
// Timing
// =============================================================================
const unsigned long FAILSAFE_TIMEOUT_US = 500000UL;
const unsigned long SERIAL_INTERVAL_US  = 50000UL;   // 20 Hz telemetry

// =============================================================================
// Servo Objects
// =============================================================================
Servo escLeft;
Servo escRight;

// =============================================================================
// RC Interrupt State
// =============================================================================
volatile unsigned long riseTime[3]   = {0, 0, 0};
volatile int           pulseWidth[3] = {1500, 1500, 1000};
volatile unsigned long pulseTime[3]  = {0, 0, 0};

const uint8_t CH_LEFT  = 0;
const uint8_t CH_RIGHT = 1;
const uint8_t CH_OVR   = 2;

void isrLeft() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_LEFT) == HIGH) {
    riseTime[CH_LEFT] = now;
  } else {
    unsigned long pw = now - riseTime[CH_LEFT];
    if (pw >= 800 && pw <= 2200) {
      pulseWidth[CH_LEFT] = pw;
      pulseTime[CH_LEFT]  = now;
    }
  }
}

void isrRight() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_RIGHT) == HIGH) {
    riseTime[CH_RIGHT] = now;
  } else {
    unsigned long pw = now - riseTime[CH_RIGHT];
    if (pw >= 800 && pw <= 2200) {
      pulseWidth[CH_RIGHT] = pw;
      pulseTime[CH_RIGHT]  = now;
    }
  }
}

void isrOverride() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_OVERRIDE) == HIGH) {
    riseTime[CH_OVR] = now;
  } else {
    unsigned long pw = now - riseTime[CH_OVR];
    if (pw >= 800 && pw <= 2200) {
      pulseWidth[CH_OVR] = pw;
      pulseTime[CH_OVR]  = now;
    }
  }
}

// =============================================================================
// Hall Sensor RPM — Interrupt-Driven (FALLBACK)
//
// Counts rising edges on motor hall sensor taps. RPM calculated from
// interval between edges (better resolution at low RPM).
//
// 4-pole motor, 2 pole pairs → 4 edges per mechanical revolution
// (2 hall lines × 2 pole pairs, counting RISING on each).
// =============================================================================
#if RPM_SOURCE_HALL

const int HALL_EDGES_PER_REV = 4;  // 2 hall lines × 2 pole pairs

volatile unsigned long hallLastEdgeL = 0;
volatile unsigned long hallIntervalL = 0;
volatile unsigned long hallLastEdgeR = 0;
volatile unsigned long hallIntervalR = 0;

void isrHallLeft() {
  unsigned long now = micros();
  unsigned long interval = now - hallLastEdgeL;
  if (interval > 50) {  // Debounce: ignore edges <50us apart (~300k eRPM)
    hallIntervalL = interval;
    hallLastEdgeL = now;
  }
}

void isrHallRight() {
  unsigned long now = micros();
  unsigned long interval = now - hallLastEdgeR;
  if (interval > 50) {
    hallIntervalR = interval;
    hallLastEdgeR = now;
  }
}

float hallRpmFromInterval(volatile unsigned long &interval,
                          volatile unsigned long &lastEdge,
                          unsigned long nowUs) {
  noInterrupts();
  unsigned long iv = interval;
  unsigned long le = lastEdge;
  interrupts();

  // If no edge for 500ms, motor is stopped
  if (iv == 0 || (nowUs - le) > 500000UL) return 0.0f;

  // RPM = 60,000,000 / (interval_us * edges_per_rev)
  return 60000000.0f / ((float)iv * HALL_EDGES_PER_REV);
}

#endif // RPM_SOURCE_HALL

// =============================================================================
// X.BUS Telemetry — Serial (PRIMARY)
//
// Reads ESC telemetry from the X.BUS wire on Serial1 (D0 = RX).
// Attempts to decode Spektrum X-Bus ESC packets (address 0x20, 16 bytes).
// If the format is different, raw bytes are buffered for analysis.
//
// Data provided per packet:
//   - RPM (10 RPM resolution)
//   - Voltage (0.01V resolution)
//   - Motor current (10mA resolution)
//   - FET temperature (0.1°C resolution)
//   - BEC current, BEC voltage, throttle %, power output %
// =============================================================================
#if RPM_SOURCE_XBUS

const int XBUS_BUF_SIZE = 64;
uint8_t xbusBuf[XBUS_BUF_SIZE];
int     xbusLen = 0;

// Decoded telemetry (updated when a valid packet arrives)
float xbusRpmL       = 0.0f;   // RPM from left ESC
float xbusVoltageL   = 0.0f;   // Battery voltage (V)
float xbusCurrentL   = 0.0f;   // Motor current (A)
float xbusFetTempL   = 0.0f;   // FET temperature (°C)
float xbusThrottleL  = 0.0f;   // Throttle command (%)
unsigned long xbusLastPacketUs = 0;
unsigned long xbusPacketCount  = 0;
unsigned long xbusPacketGapUs  = 0;  // Time between last two packets

// For diagnostics: track how fast packets arrive
float xbusHz = 0.0f;

// Try to decode a Spektrum X-Bus ESC packet at the given offset.
// Returns number of bytes consumed (0 if no valid packet found).
int xbusTryDecode(uint8_t *buf, int len, unsigned long nowUs) {
  // Look for address byte 0x20 (Spektrum ESC sensor)
  for (int i = 0; i <= len - 16; i++) {
    if (buf[i] == 0x20) {
      uint8_t *p = &buf[i + 1];

      // Decode big-endian fields
      uint16_t rawRpm     = ((uint16_t)p[0] << 8) | p[1];
      uint16_t rawVoltage = ((uint16_t)p[2] << 8) | p[3];
      uint16_t rawFetTemp = ((uint16_t)p[4] << 8) | p[5];
      uint16_t rawCurrent = ((uint16_t)p[6] << 8) | p[7];

      float rpm     = rawRpm * 10.0f;
      float voltage = rawVoltage * 0.01f;
      float current = rawCurrent * 0.01f;
      float fetTemp = rawFetTemp * 0.1f;
      float throttle = p[12] * 0.5f;

      // Sanity check
      bool plausible = (voltage >= 0.0f && voltage <= 60.0f)
                    && (current >= 0.0f && current <= 300.0f)
                    && (fetTemp >= -20.0f && fetTemp <= 200.0f);

      if (plausible) {
        xbusRpmL      = rpm;
        xbusVoltageL  = voltage;
        xbusCurrentL  = current;
        xbusFetTempL  = fetTemp;
        xbusThrottleL = throttle;

        if (xbusLastPacketUs > 0) {
          xbusPacketGapUs = nowUs - xbusLastPacketUs;
          if (xbusPacketGapUs > 0) {
            xbusHz = 1000000.0f / (float)xbusPacketGapUs;
          }
        }
        xbusLastPacketUs = nowUs;
        xbusPacketCount++;

        return i + 16;  // Bytes consumed
      }
    }
  }

  // No valid packet found. If buffer is getting full, discard old bytes
  // but keep last 16 in case a packet straddles the boundary.
  if (len > 32) return len - 16;

  return 0;
}

void xbusRead(unsigned long nowUs) {
  // Read all available bytes from Serial1 (non-blocking)
  while (Serial1.available() && xbusLen < XBUS_BUF_SIZE) {
    xbusBuf[xbusLen++] = Serial1.read();
  }

  if (xbusLen >= 16) {
    int consumed = xbusTryDecode(xbusBuf, xbusLen, nowUs);
    if (consumed > 0) {
      memmove(xbusBuf, xbusBuf + consumed, xbusLen - consumed);
      xbusLen -= consumed;
    }
  }
}

float xbusGetRpmLeft(unsigned long nowUs) {
  // If no packet for 500ms, consider data stale
  if (xbusLastPacketUs == 0 || (nowUs - xbusLastPacketUs) > 500000UL) {
    return 0.0f;
  }
  return xbusRpmL;
}

#endif // RPM_SOURCE_XBUS

// =============================================================================
// Runtime State
// =============================================================================

// Inertia simulation (per motor)
float velocityL = 0.0f;
float velocityR = 0.0f;
float positionL = 0.0f;
float positionR = 0.0f;

// Inner loop (current feedforward) state — per motor
float ffIntegralL  = 0.0f;
float ffIntegralR  = 0.0f;
float ffPrevErrorL = 0.0f;
float ffPrevErrorR = 0.0f;
float ffBoostL     = 0.0f;
float ffBoostR     = 0.0f;

// Outer loop (RPM feedback) state — per motor
float rpmIntegralL  = 0.0f;
float rpmIntegralR  = 0.0f;
float rpmPrevErrorL = 0.0f;
float rpmPrevErrorR = 0.0f;
float rpmTrimL      = 0.0f;   // RPM loop correction (us)
float rpmTrimR      = 0.0f;

// Anti-runaway tracking (per motor)
unsigned long runawayStartL = 0;
unsigned long runawayStartR = 0;
bool runawayActiveL  = false;
bool runawayActiveR  = false;
bool runawayTrippedL = false;
bool runawayTrippedR = false;

// Current sensor moving average (8 samples)
const int CUR_AVG_N = 8;
float curBufL[CUR_AVG_N];
float curBufR[CUR_AVG_N];
int   curBufIdx = 0;
float curAvgL = 0.0f;
float curAvgR = 0.0f;

// Measured RPM (from whichever source is active)
float measuredRpmL = 0.0f;
float measuredRpmR = 0.0f;

// Timing (all in micros for 20kHz+ resolution)
unsigned long prevLoopUs   = 0;
unsigned long prevSerialUs = 0;

// =============================================================================
// Helpers
// =============================================================================

int deadbandRC(int value) {
  return (abs(value - SERVO_CENTER) <= RC_DEADBAND) ? SERVO_CENTER : value;
}

int deadbandJoy(int value) {
  return (abs(value - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : value;
}

// 14-bit ADC -> servo range with exponential curve
int joyToServoExpo(int adcValue) {
  float norm = (float)(adcValue - ADC_CENTER) / (float)ADC_CENTER;
  norm = constrain(norm, -1.0f, 1.0f);
  float sign = (norm >= 0.0f) ? 1.0f : -1.0f;
  float curved = sign * powf(fabsf(norm), EXP_CURVE);
  return SERVO_CENTER + (int)(curved * 500.0f);
}

// Read CS7581 and convert to Amps
float readCurrent(uint8_t pin) {
  int raw = analogRead(pin);
  float voltage = (float)raw * ADC_VOLTS / (float)ADC_MAX;
  float sensorV = voltage / CUR_DIVIDER;
  return (sensorV - CUR_ZERO_V) * 1000.0f / CUR_SENS_MV_A;
}

void updateCurrentAvg() {
  curBufL[curBufIdx] = readCurrent(PIN_CUR_LEFT);
  curBufR[curBufIdx] = readCurrent(PIN_CUR_RIGHT);
  curBufIdx = (curBufIdx + 1) % CUR_AVG_N;

  float sumL = 0.0f, sumR = 0.0f;
  for (int i = 0; i < CUR_AVG_N; i++) {
    sumL += curBufL[i];
    sumR += curBufR[i];
  }
  curAvgL = sumL / CUR_AVG_N;
  curAvgR = sumR / CUR_AVG_N;
}

// ---------------------------------------------------------------------------
// Inner Loop — Current-Based Feedforward
//
// Detects load spikes INSTANTLY via current sensors. When measured current
// exceeds expected current for the commanded output level, resistance is
// detected. The feedforward BOOSTS power to push through BEFORE RPM drops.
//
// This is the fast loop — runs every main loop iteration at 20kHz+.
// ---------------------------------------------------------------------------
float feedforwardCompensate(float commandDelta, float measuredAmps,
                            float &integral, float &prevError, float dtSec) {
  float absCommand = fabsf(commandDelta);
  if (absCommand < 5.0f) {
    integral *= 0.9f;
    prevError = 0.0f;
    return 0.0f;
  }

  float expectedAmps = absCommand * CURRENT_PER_UNIT;
  float absMeasured = fabsf(measuredAmps);

  float error = absMeasured - expectedAmps;
  if (error < 0.0f) {
    integral *= 0.95f;
    prevError = 0.0f;
    return 0.0f;
  }

  // Adaptive gain: reduce at higher speeds for stability
  float speedFactor = absCommand / SOFT_LIMIT_RANGE;
  float adaptScale = 1.0f / (1.0f + speedFactor);

  float pTerm = FF_KP * adaptScale * error;

  integral += FF_KI * adaptScale * error * dtSec;
  integral = constrain(integral, 0.0f, FF_I_MAX);

  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = FF_KD * adaptScale * (error - prevError) / dtSec;
  }
  prevError = error;

  float boost = pTerm + integral + dTerm;

  // Soft ceiling using tanh
  boost = FF_MAX_BOOST * tanhf(boost / FF_MAX_BOOST);

  return max(boost, 0.0f);
}

// ---------------------------------------------------------------------------
// Outer Loop — RPM-Based Feedback
//
// Measures actual motor speed and compares to target RPM derived from
// stick position. Applies a TRIM correction on top of the feedforward.
//
// This loop runs at whatever rate RPM data arrives:
//   - X.BUS: ~22-45 Hz (one new value every 22-44ms)
//   - Hall sensor: continuous (kHz+ edge rate)
//
// The trim is ADDED to the feedforward boost. It can be positive (need
// more power to reach target speed) or negative (feedforward over-
// compensated and actual speed exceeded target).
// ---------------------------------------------------------------------------
float rpmFeedbackTrim(float commandDelta, float targetRpm, float actualRpm,
                      float &integral, float &prevError, float dtSec) {
  float absCommand = fabsf(commandDelta);
  if (absCommand < 5.0f || targetRpm < 10.0f) {
    integral *= 0.9f;
    prevError = 0.0f;
    return 0.0f;
  }

  // Error = target - actual (positive means we need MORE power)
  float error = targetRpm - actualRpm;

  // Normalize error relative to target for consistent gain behavior
  float normError = error / max(targetRpm, 100.0f);

  float pTerm = RPM_KP * normError * RPM_MAX_TRIM;

  integral += RPM_KI * normError * dtSec;
  integral = constrain(integral, -RPM_I_MAX, RPM_I_MAX);

  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = RPM_KD * ((normError - prevError) / dtSec);
    dTerm *= RPM_MAX_TRIM;
  }
  prevError = normError;

  float trim = pTerm + integral + dTerm;

  // Soft ceiling
  trim = RPM_MAX_TRIM * tanhf(trim / RPM_MAX_TRIM);

  return trim;
}

// ---------------------------------------------------------------------------
// Anti-runaway failsafe
//
// If total boost (feedforward + RPM trim) has been near max for RUNAWAY_TIME,
// the track is physically stuck. Go neutral to prevent motor burnout.
// Latches until stick returns to neutral (deliberate restart).
// ---------------------------------------------------------------------------
bool checkRunaway(float totalBoost, float maxBoost, unsigned long nowUs,
                  unsigned long &startTime, bool &active, bool &tripped) {
  if (tripped) return true;

  float threshold = maxBoost * 0.85f;
  if (totalBoost >= threshold) {
    if (!active) {
      active = true;
      startTime = nowUs;
    } else if (nowUs - startTime >= RUNAWAY_TIME_US) {
      tripped = true;
      return true;
    }
  } else {
    active = false;
  }
  return false;
}

void resetRunawayIfNeutral(float target, bool &tripped,
                           float &ffIntegral, float &ffBoost,
                           float &rpmIntegral, float &rpmTrim) {
  if (fabsf(target) < 5.0f && tripped) {
    tripped = false;
    ffIntegral = 0.0f;
    ffBoost = 0.0f;
    rpmIntegral = 0.0f;
    rpmTrim = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// Soft power scaling with tanh saturation (no hard clamps)
// ---------------------------------------------------------------------------
int scalePowerSoft(float delta) {
  if (fabsf(delta) < 0.5f) return SERVO_CENTER;
  float softened = SOFT_LIMIT_RANGE * tanhf(delta / SOFT_LIMIT_RANGE);
  return SERVO_CENTER + (int)(softened + 0.5f);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

#if RPM_SOURCE_XBUS
  Serial1.begin(XBUS_BAUD);
#endif

  analogReadResolution(14);

  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),    isrRight,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVERRIDE), isrOverride, CHANGE);

#if RPM_SOURCE_HALL
  pinMode(PIN_HALL_LEFT, INPUT);
  pinMode(PIN_HALL_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_LEFT),  isrHallLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_RIGHT), isrHallRight, RISING);
#endif

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  for (int i = 0; i < CUR_AVG_N; i++) {
    curBufL[i] = 0.0f;
    curBufR[i] = 0.0f;
  }

  prevLoopUs = micros();

  Serial.println("Tank Mixer V2.1 — Dual-Loop PID");
#if RPM_SOURCE_XBUS
  Serial.print("RPM source: X.BUS on Serial1 @ ");
  Serial.print(XBUS_BAUD);
  Serial.println(" baud");
#elif RPM_SOURCE_HALL
  Serial.println("RPM source: Hall sensor tap on D5/D6");
#endif
}

// =============================================================================
// Main Loop — targets 20,000+ iterations/sec
//
// No delay(), no blocking reads, no pulseIn(). RC is interrupt-driven.
// X.BUS is polled non-blocking via Serial1.available().
// All timing via micros() for sub-millisecond resolution.
//
// Pipeline:
//   1. RC snapshot (interrupt-driven, atomic)
//   2. Joystick read (14-bit ADC, ~2-5us)
//   3. Current sensors (CS7581, moving average)
//   4. X.BUS / Hall RPM read (non-blocking)
//   5. Failsafe check
//   6. Expo curve + tank mix
//   7. Override mode selection
//   8. Inner loop: current feedforward (fast, every iteration)
//   9. Outer loop: RPM feedback trim (uses latest RPM value)
//  10. Anti-runaway failsafe
//  11. Soft power limit (tanh saturation)
//  12. Inertia simulation (spring-damper)
//  13. Servo output
//  14. Telemetry (20 Hz)
// =============================================================================
void loop() {
  unsigned long nowUs = micros();
  unsigned long dtUs  = nowUs - prevLoopUs;
  if (dtUs == 0) return;
  prevLoopUs = nowUs;
  float dtSec = (float)dtUs * 0.000001f;

  // --- 1. Atomic RC snapshot from ISRs ---
  noInterrupts();
  int rawRCLeft    = pulseWidth[CH_LEFT];
  int rawRCRight   = pulseWidth[CH_RIGHT];
  int rawOverride  = pulseWidth[CH_OVR];
  unsigned long lastLeftTime  = pulseTime[CH_LEFT];
  unsigned long lastRightTime = pulseTime[CH_RIGHT];
  interrupts();

  // --- 2. Joystick (14-bit ADC, non-blocking ~2-5us each) ---
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // --- 3. Current sensors (moving average) ---
  updateCurrentAvg();

  // --- 4. RPM source read ---
#if RPM_SOURCE_XBUS
  xbusRead(nowUs);
  measuredRpmL = xbusGetRpmLeft(nowUs);
  // TODO: Right ESC X.BUS (needs 2nd serial or multiplexing)
  // For now, mirror left RPM as estimate for right motor
  measuredRpmR = measuredRpmL;
#elif RPM_SOURCE_HALL
  measuredRpmL = hallRpmFromInterval(hallIntervalL, hallLastEdgeL, nowUs);
  measuredRpmR = hallRpmFromInterval(hallIntervalR, hallLastEdgeR, nowUs);
#endif

  // --- 5. Failsafe: RC signal lost ---
  bool rcLost = (nowUs - lastLeftTime  > FAILSAFE_TIMEOUT_US)
             && (nowUs - lastRightTime > FAILSAFE_TIMEOUT_US);

  int rcLeft  = rcLost ? SERVO_CENTER : deadbandRC(rawRCLeft);
  int rcRight = rcLost ? SERVO_CENTER : deadbandRC(rawRCRight);

  // --- 6. Joystick -> expo curve -> tank mix ---
  int joyThrottle = joyToServoExpo(deadbandJoy(rawJoyY));
  int joySteering = joyToServoExpo(deadbandJoy(rawJoyX));
  int joySteerOff = joySteering - SERVO_CENTER;
  int joyLeft     = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyRight    = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // --- 7. Override mode selection ---
  int left, right;

  if (rawOverride < MODE_LOW_THRESH) {
    left  = rcLeft;
    right = rcRight;
  } else if (rawOverride <= MODE_HIGH_THRESH) {
    bool rcActive = (rcLeft != SERVO_CENTER) || (rcRight != SERVO_CENTER);
    if (rcActive && !rcLost) {
      left  = rcLeft;
      right = rcRight;
    } else {
      left  = joyLeft;
      right = joyRight;
    }
  } else {
    left  = (rcLeft  + joyLeft)  / 2;
    right = (rcRight + joyRight) / 2;
  }

  // --- 8. Base command delta (us from center) ---
  float deltaL = (float)(left  - SERVO_CENTER);
  float deltaR = (float)(right - SERVO_CENTER);

  // --- 9. Hard current safety cutoff ---
  bool hardCutoffL = fabsf(curAvgL) >= CURRENT_HARD_LIMIT;
  bool hardCutoffR = fabsf(curAvgR) >= CURRENT_HARD_LIMIT;

  // --- 10. Inner loop: current feedforward ---
  if (hardCutoffL) {
    deltaL = 0.0f;
    ffIntegralL = 0.0f;
    ffBoostL = 0.0f;
    rpmIntegralL = 0.0f;
    rpmTrimL = 0.0f;
  } else {
    ffBoostL = feedforwardCompensate(deltaL, curAvgL,
                                     ffIntegralL, ffPrevErrorL, dtSec);
    float sign = (deltaL >= 0.0f) ? 1.0f : -1.0f;
    deltaL += sign * ffBoostL;
  }

  if (hardCutoffR) {
    deltaR = 0.0f;
    ffIntegralR = 0.0f;
    ffBoostR = 0.0f;
    rpmIntegralR = 0.0f;
    rpmTrimR = 0.0f;
  } else {
    ffBoostR = feedforwardCompensate(deltaR, curAvgR,
                                     ffIntegralR, ffPrevErrorR, dtSec);
    float sign = (deltaR >= 0.0f) ? 1.0f : -1.0f;
    deltaR += sign * ffBoostR;
  }

  // --- 11. Outer loop: RPM feedback trim ---
  if (!hardCutoffL && measuredRpmL > 0.0f) {
    float absDelta = fabsf((float)(left - SERVO_CENTER));
    float targetRpmL = (absDelta / SOFT_LIMIT_RANGE) * MAX_TARGET_RPM;

    rpmTrimL = rpmFeedbackTrim(deltaL, targetRpmL, measuredRpmL,
                               rpmIntegralL, rpmPrevErrorL, dtSec);

    float sign = (deltaL >= 0.0f) ? 1.0f : -1.0f;
    deltaL += sign * rpmTrimL;
  }

  if (!hardCutoffR && measuredRpmR > 0.0f) {
    float absDelta = fabsf((float)(right - SERVO_CENTER));
    float targetRpmR = (absDelta / SOFT_LIMIT_RANGE) * MAX_TARGET_RPM;

    rpmTrimR = rpmFeedbackTrim(deltaR, targetRpmR, measuredRpmR,
                               rpmIntegralR, rpmPrevErrorR, dtSec);

    float sign = (deltaR >= 0.0f) ? 1.0f : -1.0f;
    deltaR += sign * rpmTrimR;
  }

  // --- 12. Anti-runaway failsafe ---
  float totalBoostL = ffBoostL + fabsf(rpmTrimL);
  float totalBoostR = ffBoostR + fabsf(rpmTrimR);
  float maxTotalBoost = FF_MAX_BOOST + RPM_MAX_TRIM;

  bool failL = checkRunaway(totalBoostL, maxTotalBoost, nowUs,
                            runawayStartL, runawayActiveL, runawayTrippedL);
  bool failR = checkRunaway(totalBoostR, maxTotalBoost, nowUs,
                            runawayStartR, runawayActiveR, runawayTrippedR);

  if (failL) deltaL = 0.0f;
  if (failR) deltaR = 0.0f;

  // --- 13. Soft power limit (tanh saturation, no hard clamps) ---
  int scaledL = scalePowerSoft(deltaL);
  int scaledR = scalePowerSoft(deltaR);

  // --- 14. Inertia simulation (heavy machine feel) ---
  float targetL = (float)(scaledL - SERVO_CENTER);
  float targetR = (float)(scaledR - SERVO_CENTER);

  // Reset runaway latch if stick at neutral
  resetRunawayIfNeutral(targetL, runawayTrippedL,
                        ffIntegralL, ffBoostL, rpmIntegralL, rpmTrimL);
  resetRunawayIfNeutral(targetR, runawayTrippedR,
                        ffIntegralR, ffBoostR, rpmIntegralR, rpmTrimR);

  // Spring force (pulls toward target) + friction (opposes velocity)
  float forceL = RESPONSE_FORCE * (targetL - positionL);
  float forceR = RESPONSE_FORCE * (targetR - positionR);
  float fricL  = -FRICTION_COEFF * velocityL;
  float fricR  = -FRICTION_COEFF * velocityR;

  velocityL += (forceL + fricL) / VIRTUAL_MASS * dtSec;
  velocityR += (forceR + fricR) / VIRTUAL_MASS * dtSec;
  positionL += velocityL * dtSec;
  positionR += velocityR * dtSec;

  // Snap to zero near neutral (prevents micro-drift)
  if (fabsf(targetL) < 1.0f && fabsf(positionL) < 2.0f && fabsf(velocityL) < 5.0f) {
    positionL = 0.0f; velocityL = 0.0f;
  }
  if (fabsf(targetR) < 1.0f && fabsf(positionR) < 2.0f && fabsf(velocityR) < 5.0f) {
    positionR = 0.0f; velocityR = 0.0f;
  }

  int outLeft  = SERVO_CENTER + (int)(positionL + 0.5f);
  int outRight = SERVO_CENTER + (int)(positionR + 0.5f);

  // --- 15. Write to ESCs ---
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // --- 16. Serial telemetry (20 Hz, non-blocking) ---
  if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
    prevSerialUs = nowUs;
    Serial.print("D2=");    Serial.print(rawRCLeft);
    Serial.print("  D4=");  Serial.print(rawRCRight);
    Serial.print("  D7=");  Serial.print(rawOverride);
    Serial.print("  A0=");  Serial.print(rawJoyY);
    Serial.print("  A1=");  Serial.print(rawJoyX);
    Serial.print("  IL=");  Serial.print(curAvgL, 1);
    Serial.print("  IR=");  Serial.print(curAvgR, 1);
    Serial.print("  FFl="); Serial.print(ffBoostL, 1);
    Serial.print("  FFr="); Serial.print(ffBoostR, 1);
    Serial.print("  RPMl="); Serial.print(measuredRpmL, 0);
    Serial.print("  RPMr="); Serial.print(measuredRpmR, 0);
    Serial.print("  TRl="); Serial.print(rpmTrimL, 1);
    Serial.print("  TRr="); Serial.print(rpmTrimR, 1);
    Serial.print("  L=");   Serial.print(outLeft);
    Serial.print("  R=");   Serial.print(outRight);
#if RPM_SOURCE_XBUS
    Serial.print("  XHz="); Serial.print(xbusHz, 1);
    Serial.print("  XV=");  Serial.print(xbusVoltageL, 1);
    Serial.print("  XT=");  Serial.print(xbusFetTempL, 1);
#endif
    Serial.println();
  }
}
