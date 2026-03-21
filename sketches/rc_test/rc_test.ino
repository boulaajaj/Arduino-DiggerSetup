// =============================================================================
// Tank Mixer V2.2 — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// Dual-Loop PID with Runtime Control Mode Selector (RC CH4)
//
// V2.2 adds:
//   - RC CH4 (D3) as a 3-position control mode selector:
//       LOW  = RAW         — direct stick-to-ESC, no processing layers
//       MID  = RPM_ONLY    — outer RPM PID loop only (speed regulation)
//       HIGH = FULL_STACK  — inner current FF + outer RPM + expo + inertia
//     This lets you A/B/C test on the field by flipping one switch.
//
// V2.1 features (all retained):
//   - X.BUS serial telemetry from ESC (RPM, current, voltage, temp)
//   - Dual-loop PID: inner current loop (feedforward) + outer RPM (feedback)
//   - Hall sensor tap fallback for RPM if X.BUS is too slow
//   - Exponential response curve (pow 2.5) for fine low-stick control
//   - Inertia simulation (spring-damper model — heavy machine feel)
//   - Soft power limits (tanh saturation, no hard clamps)
//   - Anti-runaway failsafe (2s at max boost → neutral, latch until centered)
//   - 3-mode override switch CH5 (RC only / RC+joy / 50-50 blend)
//
// Hardware:
//   MCU:       STM32U585 (Arm Cortex-M33, 160MHz, 786KB RAM, 2MB flash)
//   ADC:       14-bit (0-16383), 3.3V reference
//   Logic:     3.3V — DO NOT connect 5V signals directly to any pin!
//   ESCs:      XC E10 Sensored Brushless 140A (x2)
//   Motors:    XC E3665 2500KV Sensored Brushless (x2)
//   Battery:   OVONIC 3S LiPo 15000mAh 130C 11.1V
//   RC:        Radiolink RC6GS V3 + R7FG gyro receiver (6CH)
//   Joystick:  Genie 101174GT dual-axis (5V, needs divider to 3.3V)
//   Current:   CS7581 Hall-effect current sensor (x2)
//   X.BUS:     ESC telemetry wire (Yellow=data via 5V->3.3V divider)
//
// Inputs:
//   D0  <- ESC X.BUS Left     (Serial1 RX, via 5V->3.3V divider)
//   D2  <- RC CH1 -> Left motor   (servo PWM 1000-2000us, already mixed)
//   D3  <- RC CH4 Control mode    (servo PWM, 3-position switch)
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
// Control modes (CH4 — 3-position switch):
//   LOW  (<1250us)      RAW:        Direct pass-through, no processing
//   MID  (1250-1750us)  RPM_ONLY:   Outer RPM PID only (speed regulation)
//   HIGH (>1750us)      FULL_STACK: Dual-loop PID + expo + inertia + soft limits
//
// Override modes (CH5 — 3-position switch):
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
const uint8_t PIN_RC_CTRLMODE = 3;    // RC CH4 — control mode selector
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
// Control Mode — selected by RC CH4 at runtime
// =============================================================================
enum ControlMode {
  MODE_RAW,         // Direct pass-through — no PID, no expo, no inertia
  MODE_RPM_ONLY,    // Outer RPM PID loop only (speed regulation, no smoothing)
  MODE_FULL_STACK   // Inner current FF + outer RPM + expo + inertia + soft limits
};

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
// Inertia Simulation (FULL_STACK mode only)
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
// =============================================================================
const float CUR_ZERO_V    = 1.65f;
const float CUR_SENS_MV_A = 20.0f;
const float CUR_DIVIDER   = 1.0f;

// =============================================================================
// Dual-Loop PID — Inner Current (Feedforward) + Outer RPM (Feedback)
//
// INNER LOOP — Current-Based Feedforward (FULL_STACK only):
//   Current sensors detect load changes INSTANTLY. When a track hits mud or
//   an obstacle, current spikes BEFORE RPM drops. Pre-boosts power before
//   the operator feels it.
//
// OUTER LOOP — RPM-Based Feedback (RPM_ONLY and FULL_STACK):
//   Measures actual motor speed, compares to target RPM from stick position.
//   Applies trim correction to maintain consistent track speed.
//
// RAW mode: neither loop runs. Direct stick → ESC.
// =============================================================================

// --- Inner loop (current feedforward) — FULL_STACK only ---
const float CURRENT_PER_UNIT = 0.25f;
const float FF_MAX_BOOST     = 50.0f;
const float FF_KP            = 0.3f;
const float FF_KI            = 0.05f;
const float FF_KD            = 0.01f;
const float FF_I_MAX         = 30.0f;

// --- Outer loop (RPM feedback) — RPM_ONLY and FULL_STACK ---
const float RPM_MAX_TRIM     = 40.0f;
const float RPM_KP           = 0.15f;
const float RPM_KI           = 0.03f;
const float RPM_KD           = 0.005f;
const float RPM_I_MAX        = 25.0f;

// Stick-to-RPM mapping (calibrate on your machine!)
const float MAX_TARGET_RPM   = 5000.0f;

// Hard current safety cutoff — always active in ALL modes
const float CURRENT_HARD_LIMIT = 100.0f;

// Anti-runaway — active in RPM_ONLY and FULL_STACK
const unsigned long RUNAWAY_TIME_US = 2000000UL;

// =============================================================================
// Timing
// =============================================================================
const unsigned long FAILSAFE_TIMEOUT_US = 500000UL;
const unsigned long SERIAL_INTERVAL_US  = 50000UL;

// =============================================================================
// Servo Objects
// =============================================================================
Servo escLeft;
Servo escRight;

// =============================================================================
// RC Interrupt State — 4 channels now (added CH4 for control mode)
// =============================================================================
volatile unsigned long riseTime[4]   = {0, 0, 0, 0};
volatile int           pulseWidth[4] = {1500, 1500, 1000, 1500};
volatile unsigned long pulseTime[4]  = {0, 0, 0, 0};

const uint8_t CH_LEFT  = 0;
const uint8_t CH_RIGHT = 1;
const uint8_t CH_OVR   = 2;
const uint8_t CH_CTRL  = 3;

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

void isrCtrlMode() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_CTRLMODE) == HIGH) {
    riseTime[CH_CTRL] = now;
  } else {
    unsigned long pw = now - riseTime[CH_CTRL];
    if (pw >= 800 && pw <= 2200) {
      pulseWidth[CH_CTRL] = pw;
      pulseTime[CH_CTRL]  = now;
    }
  }
}

// =============================================================================
// Hall Sensor RPM — Interrupt-Driven (FALLBACK)
// =============================================================================
#if RPM_SOURCE_HALL

const int HALL_EDGES_PER_REV = 4;

volatile unsigned long hallLastEdgeL = 0;
volatile unsigned long hallIntervalL = 0;
volatile unsigned long hallLastEdgeR = 0;
volatile unsigned long hallIntervalR = 0;

void isrHallLeft() {
  unsigned long now = micros();
  unsigned long interval = now - hallLastEdgeL;
  if (interval > 50) {
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

  if (iv == 0 || (nowUs - le) > 500000UL) return 0.0f;
  return 60000000.0f / ((float)iv * HALL_EDGES_PER_REV);
}

#endif // RPM_SOURCE_HALL

// =============================================================================
// X.BUS Telemetry — Serial (PRIMARY)
// =============================================================================
#if RPM_SOURCE_XBUS

const int XBUS_BUF_SIZE = 64;
uint8_t xbusBuf[XBUS_BUF_SIZE];
int     xbusLen = 0;

float xbusRpmL       = 0.0f;
float xbusVoltageL   = 0.0f;
float xbusCurrentL   = 0.0f;
float xbusFetTempL   = 0.0f;
float xbusThrottleL  = 0.0f;
unsigned long xbusLastPacketUs = 0;
unsigned long xbusPacketCount  = 0;
unsigned long xbusPacketGapUs  = 0;
float xbusHz = 0.0f;

int xbusTryDecode(uint8_t *buf, int len, unsigned long nowUs) {
  for (int i = 0; i <= len - 16; i++) {
    if (buf[i] == 0x20) {
      uint8_t *p = &buf[i + 1];

      uint16_t rawRpm     = ((uint16_t)p[0] << 8) | p[1];
      uint16_t rawVoltage = ((uint16_t)p[2] << 8) | p[3];
      uint16_t rawFetTemp = ((uint16_t)p[4] << 8) | p[5];
      uint16_t rawCurrent = ((uint16_t)p[6] << 8) | p[7];

      float rpm     = rawRpm * 10.0f;
      float voltage = rawVoltage * 0.01f;
      float current = rawCurrent * 0.01f;
      float fetTemp = rawFetTemp * 0.1f;
      float throttle = p[12] * 0.5f;

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
        return i + 16;
      }
    }
  }
  if (len > 32) return len - 16;
  return 0;
}

void xbusRead(unsigned long nowUs) {
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
  if (xbusLastPacketUs == 0 || (nowUs - xbusLastPacketUs) > 500000UL) {
    return 0.0f;
  }
  return xbusRpmL;
}

#endif // RPM_SOURCE_XBUS

// =============================================================================
// Runtime State
// =============================================================================

// Control mode (updated each loop from CH4)
ControlMode ctrlMode = MODE_FULL_STACK;
ControlMode prevCtrlMode = MODE_FULL_STACK;

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
float rpmTrimL      = 0.0f;
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

// Timing
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

// 14-bit ADC -> servo range with exponential curve (FULL_STACK only)
int joyToServoExpo(int adcValue) {
  float norm = (float)(adcValue - ADC_CENTER) / (float)ADC_CENTER;
  norm = constrain(norm, -1.0f, 1.0f);
  float sign = (norm >= 0.0f) ? 1.0f : -1.0f;
  float curved = sign * powf(fabsf(norm), EXP_CURVE);
  return SERVO_CENTER + (int)(curved * 500.0f);
}

// 14-bit ADC -> servo range LINEAR (RAW and RPM_ONLY modes)
int joyToServoLinear(int adcValue) {
  float norm = (float)(adcValue - ADC_CENTER) / (float)ADC_CENTER;
  norm = constrain(norm, -1.0f, 1.0f);
  return SERVO_CENTER + (int)(norm * 500.0f);
}

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

// Reset all PID and inertia state — called on mode transitions
void resetAllState() {
  ffIntegralL = 0.0f;  ffIntegralR = 0.0f;
  ffPrevErrorL = 0.0f; ffPrevErrorR = 0.0f;
  ffBoostL = 0.0f;     ffBoostR = 0.0f;

  rpmIntegralL = 0.0f;  rpmIntegralR = 0.0f;
  rpmPrevErrorL = 0.0f; rpmPrevErrorR = 0.0f;
  rpmTrimL = 0.0f;      rpmTrimR = 0.0f;

  velocityL = 0.0f; velocityR = 0.0f;
  positionL = 0.0f; positionR = 0.0f;

  runawayActiveL = false;  runawayActiveR = false;
  runawayTrippedL = false; runawayTrippedR = false;
}

// ---------------------------------------------------------------------------
// Inner Loop — Current-Based Feedforward (FULL_STACK only)
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
  boost = FF_MAX_BOOST * tanhf(boost / FF_MAX_BOOST);
  return max(boost, 0.0f);
}

// ---------------------------------------------------------------------------
// Outer Loop — RPM-Based Feedback (RPM_ONLY and FULL_STACK)
// ---------------------------------------------------------------------------
float rpmFeedbackTrim(float commandDelta, float targetRpm, float actualRpm,
                      float &integral, float &prevError, float dtSec) {
  float absCommand = fabsf(commandDelta);
  if (absCommand < 5.0f || targetRpm < 10.0f) {
    integral *= 0.9f;
    prevError = 0.0f;
    return 0.0f;
  }

  float error = targetRpm - actualRpm;
  float normError = error / max(targetRpm, 100.0f);

  float pTerm = RPM_KP * normError * RPM_MAX_TRIM;

  integral += RPM_KI * normError * dtSec;
  integral = constrain(integral, -RPM_I_MAX, RPM_I_MAX);

  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = RPM_KD * ((normError - prevError) / dtSec) * RPM_MAX_TRIM;
  }
  prevError = normError;

  float trim = pTerm + integral + dTerm;
  trim = RPM_MAX_TRIM * tanhf(trim / RPM_MAX_TRIM);
  return trim;
}

// ---------------------------------------------------------------------------
// Anti-runaway failsafe (RPM_ONLY and FULL_STACK)
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
                           float &ffInt, float &ffBst,
                           float &rpmInt, float &rpmTrm) {
  if (fabsf(target) < 5.0f && tripped) {
    tripped = false;
    ffInt = 0.0f;
    ffBst = 0.0f;
    rpmInt = 0.0f;
    rpmTrm = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// Soft power scaling with tanh saturation (FULL_STACK only)
// ---------------------------------------------------------------------------
int scalePowerSoft(float delta) {
  if (fabsf(delta) < 0.5f) return SERVO_CENTER;
  float softened = SOFT_LIMIT_RANGE * tanhf(delta / SOFT_LIMIT_RANGE);
  return SERVO_CENTER + (int)(softened + 0.5f);
}

// ---------------------------------------------------------------------------
// Hard power clamp (RAW and RPM_ONLY — simple clamp at POWER_LIMIT_PCT)
// ---------------------------------------------------------------------------
int clampPower(int value) {
  int maxDelta = 500 * POWER_LIMIT_PCT / 100;
  int delta = value - SERVO_CENTER;
  delta = constrain(delta, -maxDelta, maxDelta);
  return SERVO_CENTER + delta;
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
  pinMode(PIN_RC_CTRLMODE, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CTRLMODE), isrCtrlMode, CHANGE);
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

  Serial.println("Tank Mixer V2.2 — Dual-Loop PID + Mode Selector");
  Serial.println("CH4: LOW=RAW  MID=RPM_ONLY  HIGH=FULL_STACK");
#if RPM_SOURCE_XBUS
  Serial.print("RPM source: X.BUS @ ");
  Serial.print(XBUS_BAUD);
  Serial.println(" baud");
#elif RPM_SOURCE_HALL
  Serial.println("RPM source: Hall sensor tap D5/D6");
#endif
}

// =============================================================================
// Main Loop — targets 20,000+ iterations/sec
//
// Pipeline varies by control mode:
//
// RAW:        Stick → deadband → linear map → hard clamp → ESC
// RPM_ONLY:   Stick → deadband → linear map → RPM PID trim → hard clamp → ESC
// FULL_STACK: Stick → deadband → expo → tank mix → mode select →
//             current FF → RPM trim → anti-runaway → soft limit →
//             inertia sim → ESC
//
// Hard current safety cutoff (100A) is ALWAYS active in all modes.
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
  int rawCtrlMode  = pulseWidth[CH_CTRL];
  unsigned long lastLeftTime  = pulseTime[CH_LEFT];
  unsigned long lastRightTime = pulseTime[CH_RIGHT];
  interrupts();

  // --- 2. Determine control mode from CH4 ---
  if (rawCtrlMode < MODE_LOW_THRESH) {
    ctrlMode = MODE_RAW;
  } else if (rawCtrlMode <= MODE_HIGH_THRESH) {
    ctrlMode = MODE_RPM_ONLY;
  } else {
    ctrlMode = MODE_FULL_STACK;
  }

  // Reset PID/inertia state on mode transitions to prevent jumps
  if (ctrlMode != prevCtrlMode) {
    resetAllState();
    prevCtrlMode = ctrlMode;
  }

  // --- 3. Joystick (14-bit ADC, non-blocking ~2-5us each) ---
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // --- 4. Current sensors (always read — needed for hard safety cutoff) ---
  updateCurrentAvg();

  // --- 5. RPM source read (always read — RPM_ONLY and FULL_STACK use it) ---
#if RPM_SOURCE_XBUS
  xbusRead(nowUs);
  measuredRpmL = xbusGetRpmLeft(nowUs);
  measuredRpmR = measuredRpmL;  // TODO: 2nd serial for right ESC
#elif RPM_SOURCE_HALL
  measuredRpmL = hallRpmFromInterval(hallIntervalL, hallLastEdgeL, nowUs);
  measuredRpmR = hallRpmFromInterval(hallIntervalR, hallLastEdgeR, nowUs);
#endif

  // --- 6. Failsafe: RC signal lost ---
  bool rcLost = (nowUs - lastLeftTime  > FAILSAFE_TIMEOUT_US)
             && (nowUs - lastRightTime > FAILSAFE_TIMEOUT_US);

  int rcLeft  = rcLost ? SERVO_CENTER : deadbandRC(rawRCLeft);
  int rcRight = rcLost ? SERVO_CENTER : deadbandRC(rawRCRight);

  // --- 7. Joystick processing (expo in FULL_STACK, linear otherwise) ---
  int joyThrottle, joySteering;
  if (ctrlMode == MODE_FULL_STACK) {
    joyThrottle = joyToServoExpo(deadbandJoy(rawJoyY));
    joySteering = joyToServoExpo(deadbandJoy(rawJoyX));
  } else {
    joyThrottle = joyToServoLinear(deadbandJoy(rawJoyY));
    joySteering = joyToServoLinear(deadbandJoy(rawJoyX));
  }

  int joySteerOff = joySteering - SERVO_CENTER;
  int joyLeft     = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyRight    = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // --- 8. Override mode selection (CH5 — same in all control modes) ---
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

  // --- 9. Hard current safety cutoff (ALWAYS active, all modes) ---
  bool hardCutoffL = fabsf(curAvgL) >= CURRENT_HARD_LIMIT;
  bool hardCutoffR = fabsf(curAvgR) >= CURRENT_HARD_LIMIT;

  if (hardCutoffL) left  = SERVO_CENTER;
  if (hardCutoffR) right = SERVO_CENTER;

  // =====================================================================
  // MODE: RAW — Direct pass-through, no processing layers
  //
  // Stick → deadband → linear → hard clamp → ESC
  // No PID, no expo, no inertia. Old-school direct control.
  // =====================================================================
  if (ctrlMode == MODE_RAW) {
    int outLeft  = clampPower(left);
    int outRight = clampPower(right);

    escLeft.writeMicroseconds(outLeft);
    escRight.writeMicroseconds(outRight);

    // Telemetry
    if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
      prevSerialUs = nowUs;
      Serial.print("MODE=RAW");
      Serial.print("  D2=");   Serial.print(rawRCLeft);
      Serial.print("  D4=");   Serial.print(rawRCRight);
      Serial.print("  D7=");   Serial.print(rawOverride);
      Serial.print("  A0=");   Serial.print(rawJoyY);
      Serial.print("  A1=");   Serial.print(rawJoyX);
      Serial.print("  IL=");   Serial.print(curAvgL, 1);
      Serial.print("  IR=");   Serial.print(curAvgR, 1);
      Serial.print("  RPMl="); Serial.print(measuredRpmL, 0);
      Serial.print("  RPMr="); Serial.print(measuredRpmR, 0);
      Serial.print("  L=");    Serial.print(outLeft);
      Serial.print("  R=");    Serial.println(outRight);
    }
    return;
  }

  // =====================================================================
  // MODE: RPM_ONLY — Outer RPM PID loop only
  //
  // Stick → deadband → linear → RPM PID trim → hard clamp → ESC
  // Speed regulation without the smoothing layers.
  // =====================================================================
  if (ctrlMode == MODE_RPM_ONLY) {
    float deltaL = (float)(left  - SERVO_CENTER);
    float deltaR = (float)(right - SERVO_CENTER);

    // RPM feedback trim (outer loop only, no current feedforward)
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

    // Anti-runaway (RPM trim only, no feedforward boost)
    bool failL = checkRunaway(fabsf(rpmTrimL), RPM_MAX_TRIM, nowUs,
                              runawayStartL, runawayActiveL, runawayTrippedL);
    bool failR = checkRunaway(fabsf(rpmTrimR), RPM_MAX_TRIM, nowUs,
                              runawayStartR, runawayActiveR, runawayTrippedR);

    if (failL) deltaL = 0.0f;
    if (failR) deltaR = 0.0f;

    // Reset runaway on neutral
    if (fabsf(deltaL) < 5.0f && runawayTrippedL) {
      runawayTrippedL = false;
      rpmIntegralL = 0.0f; rpmTrimL = 0.0f;
    }
    if (fabsf(deltaR) < 5.0f && runawayTrippedR) {
      runawayTrippedR = false;
      rpmIntegralR = 0.0f; rpmTrimR = 0.0f;
    }

    int outLeft  = clampPower(SERVO_CENTER + (int)(deltaL + 0.5f));
    int outRight = clampPower(SERVO_CENTER + (int)(deltaR + 0.5f));

    escLeft.writeMicroseconds(outLeft);
    escRight.writeMicroseconds(outRight);

    // Telemetry
    if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
      prevSerialUs = nowUs;
      Serial.print("MODE=RPM");
      Serial.print("  D2=");    Serial.print(rawRCLeft);
      Serial.print("  D4=");    Serial.print(rawRCRight);
      Serial.print("  D7=");    Serial.print(rawOverride);
      Serial.print("  A0=");    Serial.print(rawJoyY);
      Serial.print("  A1=");    Serial.print(rawJoyX);
      Serial.print("  IL=");    Serial.print(curAvgL, 1);
      Serial.print("  IR=");    Serial.print(curAvgR, 1);
      Serial.print("  RPMl=");  Serial.print(measuredRpmL, 0);
      Serial.print("  RPMr=");  Serial.print(measuredRpmR, 0);
      Serial.print("  TRl=");   Serial.print(rpmTrimL, 1);
      Serial.print("  TRr=");   Serial.print(rpmTrimR, 1);
      Serial.print("  L=");     Serial.print(outLeft);
      Serial.print("  R=");     Serial.print(outRight);
#if RPM_SOURCE_XBUS
      Serial.print("  XHz=");   Serial.print(xbusHz, 1);
#endif
      Serial.println();
    }
    return;
  }

  // =====================================================================
  // MODE: FULL_STACK — All layers active
  //
  // Stick → deadband → expo curve → tank mix → mode select →
  // current feedforward → RPM trim → anti-runaway → soft power limit →
  // inertia simulation → ESC
  //
  // This is the premium control experience: disturbances are absorbed
  // before the operator feels them, tracks feel heavy and deliberate.
  // =====================================================================

  float deltaL = (float)(left  - SERVO_CENTER);
  float deltaR = (float)(right - SERVO_CENTER);

  // --- Inner loop: current feedforward ---
  if (hardCutoffL) {
    ffIntegralL = 0.0f; ffBoostL = 0.0f;
    rpmIntegralL = 0.0f; rpmTrimL = 0.0f;
    deltaL = 0.0f;
  } else {
    ffBoostL = feedforwardCompensate(deltaL, curAvgL,
                                     ffIntegralL, ffPrevErrorL, dtSec);
    float sign = (deltaL >= 0.0f) ? 1.0f : -1.0f;
    deltaL += sign * ffBoostL;
  }

  if (hardCutoffR) {
    ffIntegralR = 0.0f; ffBoostR = 0.0f;
    rpmIntegralR = 0.0f; rpmTrimR = 0.0f;
    deltaR = 0.0f;
  } else {
    ffBoostR = feedforwardCompensate(deltaR, curAvgR,
                                     ffIntegralR, ffPrevErrorR, dtSec);
    float sign = (deltaR >= 0.0f) ? 1.0f : -1.0f;
    deltaR += sign * ffBoostR;
  }

  // --- Outer loop: RPM feedback trim ---
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

  // --- Anti-runaway failsafe ---
  float totalBoostL = ffBoostL + fabsf(rpmTrimL);
  float totalBoostR = ffBoostR + fabsf(rpmTrimR);
  float maxTotalBoost = FF_MAX_BOOST + RPM_MAX_TRIM;

  bool failL = checkRunaway(totalBoostL, maxTotalBoost, nowUs,
                            runawayStartL, runawayActiveL, runawayTrippedL);
  bool failR = checkRunaway(totalBoostR, maxTotalBoost, nowUs,
                            runawayStartR, runawayActiveR, runawayTrippedR);

  if (failL) deltaL = 0.0f;
  if (failR) deltaR = 0.0f;

  // --- Soft power limit (tanh saturation) ---
  int scaledL = scalePowerSoft(deltaL);
  int scaledR = scalePowerSoft(deltaR);

  // --- Inertia simulation (heavy machine feel) ---
  float targetL = (float)(scaledL - SERVO_CENTER);
  float targetR = (float)(scaledR - SERVO_CENTER);

  resetRunawayIfNeutral(targetL, runawayTrippedL,
                        ffIntegralL, ffBoostL, rpmIntegralL, rpmTrimL);
  resetRunawayIfNeutral(targetR, runawayTrippedR,
                        ffIntegralR, ffBoostR, rpmIntegralR, rpmTrimR);

  float forceL = RESPONSE_FORCE * (targetL - positionL);
  float forceR = RESPONSE_FORCE * (targetR - positionR);
  float fricL  = -FRICTION_COEFF * velocityL;
  float fricR  = -FRICTION_COEFF * velocityR;

  velocityL += (forceL + fricL) / VIRTUAL_MASS * dtSec;
  velocityR += (forceR + fricR) / VIRTUAL_MASS * dtSec;
  positionL += velocityL * dtSec;
  positionR += velocityR * dtSec;

  if (fabsf(targetL) < 1.0f && fabsf(positionL) < 2.0f && fabsf(velocityL) < 5.0f) {
    positionL = 0.0f; velocityL = 0.0f;
  }
  if (fabsf(targetR) < 1.0f && fabsf(positionR) < 2.0f && fabsf(velocityR) < 5.0f) {
    positionR = 0.0f; velocityR = 0.0f;
  }

  int outLeft  = SERVO_CENTER + (int)(positionL + 0.5f);
  int outRight = SERVO_CENTER + (int)(positionR + 0.5f);

  // --- Write to ESCs ---
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // --- Telemetry ---
  if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
    prevSerialUs = nowUs;
    Serial.print("MODE=FULL");
    Serial.print("  D2=");    Serial.print(rawRCLeft);
    Serial.print("  D4=");    Serial.print(rawRCRight);
    Serial.print("  D7=");    Serial.print(rawOverride);
    Serial.print("  A0=");    Serial.print(rawJoyY);
    Serial.print("  A1=");    Serial.print(rawJoyX);
    Serial.print("  IL=");    Serial.print(curAvgL, 1);
    Serial.print("  IR=");    Serial.print(curAvgR, 1);
    Serial.print("  FFl=");   Serial.print(ffBoostL, 1);
    Serial.print("  FFr=");   Serial.print(ffBoostR, 1);
    Serial.print("  RPMl=");  Serial.print(measuredRpmL, 0);
    Serial.print("  RPMr=");  Serial.print(measuredRpmR, 0);
    Serial.print("  TRl=");   Serial.print(rpmTrimL, 1);
    Serial.print("  TRr=");   Serial.print(rpmTrimR, 1);
    Serial.print("  L=");     Serial.print(outLeft);
    Serial.print("  R=");     Serial.print(outRight);
#if RPM_SOURCE_XBUS
    Serial.print("  XHz=");   Serial.print(xbusHz, 1);
    Serial.print("  XV=");    Serial.print(xbusVoltageL, 1);
    Serial.print("  XT=");    Serial.print(xbusFetTempL, 1);
#endif
    Serial.println();
  }
}
