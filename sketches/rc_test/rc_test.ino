// =============================================================================
// Tank Mixer V2.0 — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// Migrated from V1.2 (Nano V3 / ATmega328P).
//
// Changes from V1.2:
//   - Target: Arduino UNO Q [ABX00162] (STM32U585, Cortex-M33, 160MHz)
//   - FQBN: arduino:zephyr:unoq
//   - 3.3V logic — joystick needs voltage divider (5V → 3.3V)
//   - 14-bit ADC (16384 steps) for joystick and current sensors
//   - attachInterrupt() replaces AVR PCINT
//   - 20,000+ loops/sec — zero blocking, micros()-based timing
//   - Exponential response curve for fine low-stick control
//   - Inertia simulation — makes it feel heavy (smooth ramp, gradual stop)
//   - CS7581 hall-effect current sensors (A2, A3) — one per motor
//   - Closed-loop PID load compensation with adaptive gains
//   - Soft power limits — exponential saturation, no hard clamps
//   - Anti-runaway failsafe — if PID compensation diverges, go neutral
//   - Power limit raised to 50%
//
// Hardware:
//   MCU:       STM32U585 (Arm Cortex-M33, 160MHz, 786KB RAM, 2MB flash)
//   ADC:       14-bit (0–16383), 3.3V reference
//   Logic:     3.3V — DO NOT connect 5V signals directly to any pin!
//   ESCs:      XC E10 Sensored Brushless 140A (x2)
//   Motors:    XC E3665 2500KV Sensored Brushless (x2)
//   Battery:   OVONIC 3S LiPo 15000mAh 130C 11.1V
//   RC:        Radiolink RC6GS V3 + R7FG gyro receiver
//   Joystick:  Genie 101174GT dual-axis (5V, needs divider to 3.3V)
//   Current:   CS7581 Hall-effect current sensor (x2)
//
// Inputs:
//   D2  <- RC CH1 -> Left motor   (servo PWM 1000-2000us, already mixed)
//   D4  <- RC CH2 -> Right motor  (servo PWM 1000-2000us, already mixed)
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
const float SOFT_LIMIT_RANGE = 500.0f * POWER_LIMIT_PCT / 100.0f; // 250us from center

// =============================================================================
// Inertia Simulation
//
// Makes output feel heavy — like driving something with real mass.
// Simple spring-damper model: stick pulls output, friction slows it.
//
//   VIRTUAL_MASS:    Higher = more sluggish to start/stop
//   FRICTION_COEFF:  Higher = stops sooner, less coasting
//   RESPONSE_FORCE:  Higher = tracks stick more aggressively
//
// The friction is velocity-dependent: at low speed, friction dominates
// and the output stops cleanly. At high speed, it takes longer to stop
// (momentum). This gives the natural "heavy machine" feel.
// =============================================================================
const float VIRTUAL_MASS   = 3.0f;
const float FRICTION_COEFF = 8.0f;
const float RESPONSE_FORCE = 20.0f;

// =============================================================================
// CS7581 Current Sensor
//
// Adjust these to match your CS7581 variant and wiring:
//   CUR_ZERO_V:    Sensor output voltage at 0A
//   CUR_SENS_MV_A: Sensitivity in mV per Amp (from datasheet)
//   CUR_DIVIDER:   Voltage divider ratio if sensor output exceeds 3.3V
//                  (e.g. 0.66 for a 2:1 divider). 1.0 = no divider.
// =============================================================================
const float CUR_ZERO_V    = 1.65f;  // 0A output (typically Vcc/2)
const float CUR_SENS_MV_A = 20.0f;  // mV per Amp
const float CUR_DIVIDER   = 1.0f;   // Voltage divider ratio

// =============================================================================
// PID Load Compensation — Adaptive Gains
//
// The PID watches motor current and adjusts the output to compensate for
// load variations. Heavier load (bigger rider, uphill) draws more current
// — the PID scales output to keep behavior consistent.
//
// Adaptive: gains scale based on measured load level (curAvg / CURRENT_NOMINAL).
// Low load = higher gains (responsive). High load = lower gains (stable).
//
// CURRENT_NOMINAL: Expected current at normal operation. Used as the
//                  reference point for adaptive gain scaling.
//
// Anti-runaway failsafe: if PID correction stays maxed out for too long,
// the system can't compensate — something unexpected is happening.
// Go to neutral rather than letting it diverge.
// =============================================================================
const float CURRENT_NOMINAL    = 30.0f;  // Amps — baseline "normal" load
const float CURRENT_SOFT_LIMIT = 60.0f;  // PID starts reducing above this
const float CURRENT_HARD_LIMIT = 100.0f; // Safety cutoff
const float PID_KP             = 0.015f;
const float PID_KI             = 0.005f;
const float PID_KD             = 0.002f;
const float PID_I_MAX          = 0.5f;   // Anti-windup clamp

// Anti-runaway: if PID reduction stays above this for RUNAWAY_TIME, failsafe
const float RUNAWAY_THRESHOLD  = 0.85f;  // 85% reduction = nearly stalled
const unsigned long RUNAWAY_TIME_US = 2000000UL; // 2 seconds

// =============================================================================
// Timing
// =============================================================================
const unsigned long FAILSAFE_TIMEOUT_US = 500000UL; // 500ms in micros
const unsigned long SERIAL_INTERVAL_US  = 50000UL;  // 50ms = 20 Hz telemetry

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
// Runtime State
// =============================================================================

// Inertia simulation (per motor)
float velocityL = 0.0f;
float velocityR = 0.0f;
float positionL = 0.0f;
float positionR = 0.0f;

// PID state (per motor)
float pidIntegralL  = 0.0f;
float pidIntegralR  = 0.0f;
float pidPrevErrorL = 0.0f;
float pidPrevErrorR = 0.0f;

// Anti-runaway tracking (per motor)
unsigned long runawayStartL = 0;
unsigned long runawayStartR = 0;
bool runawayActiveL = false;
bool runawayActiveR = false;
bool runawayTrippedL = false;
bool runawayTrippedR = false;

// Current sensor moving average (8 samples)
const int CUR_AVG_N = 8;
float curBufL[CUR_AVG_N];
float curBufR[CUR_AVG_N];
int   curBufIdx = 0;
float curAvgL = 0.0f;
float curAvgR = 0.0f;

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

// 14-bit ADC -> servo range with exponential curve.
// Fine control at low stick, full authority at max.
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

// Update current moving average (called every loop)
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
// PID current limiter with adaptive gains
//
// Returns reduction factor: 0.0 = no reduction, 1.0 = full stop.
//
// Adaptive scaling: at low load, gains are higher (snappy response).
// At high load, gains are lower (smoother, more stable).
// The ratio curAvg/CURRENT_NOMINAL drives this — if you're pulling 2x
// the nominal current, gains halve to keep behavior stable.
// ---------------------------------------------------------------------------
float pidCurrentLimit(float currentAmps, float &integral, float &prevError, float dtSec) {
  if (fabsf(currentAmps) >= CURRENT_HARD_LIMIT) {
    integral = PID_I_MAX;
    return 1.0f;
  }

  float absCur = fabsf(currentAmps);

  if (absCur <= CURRENT_SOFT_LIMIT) {
    integral *= 0.95f;
    prevError = 0.0f;
    return 0.0f;
  }

  // Adaptive gain scaling: lower gains under heavy load for stability
  float loadRatio = max(absCur, CURRENT_NOMINAL) / CURRENT_NOMINAL;
  float adaptScale = 1.0f / loadRatio;  // e.g. 2x load = 0.5x gains

  float error = absCur - CURRENT_SOFT_LIMIT;

  float pTerm = PID_KP * adaptScale * error;

  integral += PID_KI * adaptScale * error * dtSec;
  integral = constrain(integral, 0.0f, PID_I_MAX);

  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = PID_KD * adaptScale * (error - prevError) / dtSec;
  }
  prevError = error;

  return constrain(pTerm + integral + dTerm, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Anti-runaway failsafe
//
// If the PID is pushing reduction near max (RUNAWAY_THRESHOLD) for longer
// than RUNAWAY_TIME, it means the compensation can't handle the situation.
// Rather than letting the PID keep fighting a losing battle (which could
// cause unpredictable output), trip the failsafe and go neutral.
//
// The failsafe latches until the operator returns stick to neutral,
// which resets it. This forces a deliberate restart.
// ---------------------------------------------------------------------------
bool checkRunaway(float reduction, unsigned long nowUs,
                  unsigned long &startTime, bool &active, bool &tripped) {
  if (tripped) return true;

  if (reduction >= RUNAWAY_THRESHOLD) {
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

// Reset runaway latch when stick returns to neutral
void resetRunawayIfNeutral(float target, bool &tripped, float &integral) {
  if (fabsf(target) < 5.0f && tripped) {
    tripped = false;
    integral = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// Soft power scaling with exponential saturation
//
// Instead of a hard clamp at OUTPUT_MIN/OUTPUT_MAX, the output follows
// a soft curve that asymptotically approaches the limit. Small inputs
// pass through nearly linearly; large inputs compress smoothly.
//
//   output = limit * tanh(input / limit)
//
// This means you never get an abrupt cutoff — the output just gets
// progressively harder to push further. The PID reduction multiplies
// the effective limit, making the soft ceiling lower under load.
// ---------------------------------------------------------------------------
int scalePowerSoft(int value, float reductionFactor) {
  float delta = (float)(value - SERVO_CENTER);
  float effectiveLimit = SOFT_LIMIT_RANGE * (1.0f - reductionFactor);
  if (effectiveLimit < 1.0f) return SERVO_CENTER;

  // tanh soft saturation: linear near zero, asymptotic near limit
  float normalized = delta / effectiveLimit;
  float softened = effectiveLimit * tanhf(normalized);

  return SERVO_CENTER + (int)(softened + 0.5f);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(14);

  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),    isrRight,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVERRIDE), isrOverride, CHANGE);

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  for (int i = 0; i < CUR_AVG_N; i++) {
    curBufL[i] = 0.0f;
    curBufR[i] = 0.0f;
  }

  prevLoopUs = micros();
}

// =============================================================================
// Main Loop — targets 20,000+ iterations/sec
//
// No delay(), no blocking reads, no pulseIn(). RC is interrupt-driven.
// All timing via micros() for sub-millisecond resolution.
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

  // --- 4. Failsafe: RC signal lost ---
  bool rcLost = (nowUs - lastLeftTime  > FAILSAFE_TIMEOUT_US)
             && (nowUs - lastRightTime > FAILSAFE_TIMEOUT_US);

  int rcLeft  = rcLost ? SERVO_CENTER : deadbandRC(rawRCLeft);
  int rcRight = rcLost ? SERVO_CENTER : deadbandRC(rawRCRight);

  // --- 5. Joystick -> expo curve -> tank mix ---
  int joyThrottle = joyToServoExpo(deadbandJoy(rawJoyY));
  int joySteering = joyToServoExpo(deadbandJoy(rawJoyX));
  int joySteerOff = joySteering - SERVO_CENTER;
  int joyLeft     = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyRight    = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // --- 6. Override mode selection ---
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

  // --- 7. PID load compensation (adaptive gains) ---
  float reductionL = pidCurrentLimit(curAvgL, pidIntegralL, pidPrevErrorL, dtSec);
  float reductionR = pidCurrentLimit(curAvgR, pidIntegralR, pidPrevErrorR, dtSec);

  // --- 8. Anti-runaway failsafe ---
  // If PID can't compensate (reduction stuck near max), go neutral.
  // Latches until stick returns to neutral — forces deliberate restart.
  bool failL = checkRunaway(reductionL, nowUs, runawayStartL, runawayActiveL, runawayTrippedL);
  bool failR = checkRunaway(reductionR, nowUs, runawayStartR, runawayActiveR, runawayTrippedR);

  // --- 9. Soft power scaling (exponential saturation, no hard clamps) ---
  if (failL) left  = SERVO_CENTER;
  if (failR) right = SERVO_CENTER;
  left  = scalePowerSoft(left,  reductionL);
  right = scalePowerSoft(right, reductionR);

  // --- 10. Inertia simulation (heavy machine feel) ---
  float targetL = (float)(left  - SERVO_CENTER);
  float targetR = (float)(right - SERVO_CENTER);

  // Reset runaway latch if stick at neutral
  resetRunawayIfNeutral(targetL, runawayTrippedL, pidIntegralL);
  resetRunawayIfNeutral(targetR, runawayTrippedR, pidIntegralR);

  // Spring force (pulls toward stick) + friction (opposes velocity)
  float forceL = RESPONSE_FORCE * (targetL - positionL);
  float forceR = RESPONSE_FORCE * (targetR - positionR);
  float fricL  = -FRICTION_COEFF * velocityL;
  float fricR  = -FRICTION_COEFF * velocityR;

  // F = ma
  float accelL = (forceL + fricL) / VIRTUAL_MASS;
  float accelR = (forceR + fricR) / VIRTUAL_MASS;

  velocityL += accelL * dtSec;
  velocityR += accelR * dtSec;
  positionL += velocityL * dtSec;
  positionR += velocityR * dtSec;

  // Snap to zero near neutral (prevents micro-drift)
  if (fabsf(targetL) < 1.0f && fabsf(positionL) < 2.0f && fabsf(velocityL) < 5.0f) {
    positionL = 0.0f; velocityL = 0.0f;
  }
  if (fabsf(targetR) < 1.0f && fabsf(positionR) < 2.0f && fabsf(velocityR) < 5.0f) {
    positionR = 0.0f; velocityR = 0.0f;
  }

  // Final output — no hard constrain, soft saturation already handled it
  int outLeft  = SERVO_CENTER + (int)(positionL + 0.5f);
  int outRight = SERVO_CENTER + (int)(positionR + 0.5f);

  // --- 11. Write to ESCs ---
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // --- 12. Serial telemetry (20 Hz, non-blocking) ---
  if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
    prevSerialUs = nowUs;
    Serial.print("D2=");   Serial.print(rawRCLeft);
    Serial.print("  D4="); Serial.print(rawRCRight);
    Serial.print("  D7="); Serial.print(rawOverride);
    Serial.print("  A0="); Serial.print(rawJoyY);
    Serial.print("  A1="); Serial.print(rawJoyX);
    Serial.print("  IL="); Serial.print(curAvgL, 1);
    Serial.print("  IR="); Serial.print(curAvgR, 1);
    Serial.print("  RL="); Serial.print(reductionL, 2);
    Serial.print("  RR="); Serial.print(reductionR, 2);
    Serial.print("  L=");  Serial.print(outLeft);
    Serial.print("  R=");  Serial.println(outRight);
  }
}
