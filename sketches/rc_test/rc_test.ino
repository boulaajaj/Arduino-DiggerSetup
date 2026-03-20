// =============================================================================
// Tank Mixer V2.0 — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// Migrated from V1.2 (Nano V3 / ATmega328P).
//
// Changes from V1.2:
//   - Target: Arduino UNO Q [ABX00162] (STM32U585, Cortex-M33, 160MHz)
//   - FQBN: arduino:zephyr:unoq
//   - 3.3V logic — joystick needs voltage divider (5V -> 3.3V)
//   - 14-bit ADC (16384 steps) for joystick and current sensors
//   - attachInterrupt() replaces AVR PCINT
//   - 20,000+ loops/sec — zero blocking, micros()-based timing
//   - Exponential response curve for fine low-stick control
//   - Inertia simulation — makes it feel heavy (smooth ramp, gradual stop)
//   - CS7581 hall-effect current sensors (A2, A3) — one per motor
//   - Closed-loop PID load compensation: if a track hits resistance,
//     BOOST power to push through and maintain consistent speed
//   - Adaptive PID gains: scale with load for stable behavior
//   - Soft power limits — tanh() saturation, no hard clamps
//   - Anti-runaway failsafe — if compensation maxes out for too long,
//     something is physically stuck; go neutral to prevent damage
//   - Power limit raised to 50%
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
// PID Load Compensation — "Cruise Control for Tracks"
//
// The PID measures current to estimate load on each motor. When a track
// encounters resistance (mud, cold rubber, obstacle, uphill), it draws
// more current. The PID BOOSTS the output to push through and maintain
// consistent track speed.
//
// How it works:
//   - We estimate "expected current" from the commanded output level.
//     At 50% stick, we expect ~CURRENT_PER_UNIT * 50% = some baseline.
//   - If measured current exceeds expected, the track is fighting resistance.
//   - The PID computes a BOOST (extra us added to the ESC command).
//   - The boost is bounded by COMP_MAX_BOOST — the PID can't add more
//     than this, no matter what.
//
// Adaptive gains: at higher output (faster speed), gains are reduced
// to prevent oscillation. At low speed (precise work), gains are higher
// for responsive compensation.
//
// CURRENT_PER_UNIT: Expected amps per unit of output (us from center).
//                   Calibrate this on your machine: measure current at
//                   known output levels and compute the ratio.
//
// COMP_MAX_BOOST: Maximum extra us the PID can add. This is the ceiling
//                 that prevents runaway compensation. Set conservatively.
// =============================================================================
const float CURRENT_PER_UNIT = 0.25f;  // Expected A per us of output from center
const float COMP_MAX_BOOST   = 50.0f;  // Max us the PID can add (safety ceiling)
const float COMP_KP          = 0.3f;   // Proportional: immediate correction
const float COMP_KI          = 0.05f;  // Integral: persistent load needs persistent boost
const float COMP_KD          = 0.01f;  // Derivative: dampen oscillation
const float COMP_I_MAX       = 30.0f;  // Integral windup limit (in us)

// Hard current safety cutoff — if current exceeds this, stop the motor
// regardless of compensation. This is SEPARATE from the PID; it's a
// last-resort safety net.
const float CURRENT_HARD_LIMIT = 100.0f;  // Amps — force neutral

// Anti-runaway: if compensation has been at COMP_MAX_BOOST for this long
// continuously, the track is likely jammed/stalled. Go neutral.
const unsigned long RUNAWAY_TIME_US = 2000000UL; // 2 seconds

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

// PID compensation state (per motor)
float compIntegralL  = 0.0f;
float compIntegralR  = 0.0f;
float compPrevErrorL = 0.0f;
float compPrevErrorR = 0.0f;
float compBoostL     = 0.0f;  // Current boost being applied (us)
float compBoostR     = 0.0f;

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
// PID Load Compensation — BOOSTS power to push through resistance
//
// Compares measured current against expected current for the commanded
// output level. If measured > expected, the track is fighting resistance.
// The PID adds a boost (extra us) to help it push through.
//
// Returns boost in us (0 to COMP_MAX_BOOST). Always positive (or zero).
// The sign of the boost matches the direction of travel (added to the
// output delta from center, not subtracted).
//
// commandDelta: how far the output is from center (us). Used to compute
//               expected current and to scale gains adaptively.
// measuredAmps: actual current from CS7581 (absolute value used).
// ---------------------------------------------------------------------------
float pidCompensate(float commandDelta, float measuredAmps,
                    float &integral, float &prevError, float dtSec) {
  float absCommand = fabsf(commandDelta);
  if (absCommand < 5.0f) {
    // Near neutral — no compensation needed, decay integral
    integral *= 0.9f;
    prevError = 0.0f;
    return 0.0f;
  }

  // Expected current for this output level
  float expectedAmps = absCommand * CURRENT_PER_UNIT;
  float absMeasured = fabsf(measuredAmps);

  // Error = how much MORE current than expected (resistance detected)
  float error = absMeasured - expectedAmps;
  if (error < 0.0f) {
    // Drawing less than expected — no resistance, decay integral
    integral *= 0.95f;
    prevError = 0.0f;
    return 0.0f;
  }

  // Adaptive gain scaling: at higher speeds, reduce gains for stability
  float speedFactor = absCommand / SOFT_LIMIT_RANGE;  // 0.0 to ~1.0
  float adaptScale = 1.0f / (1.0f + speedFactor);     // 1.0 at stop, 0.5 at full

  // PID terms
  float pTerm = COMP_KP * adaptScale * error;

  integral += COMP_KI * adaptScale * error * dtSec;
  integral = constrain(integral, 0.0f, COMP_I_MAX);

  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = COMP_KD * adaptScale * (error - prevError) / dtSec;
  }
  prevError = error;

  float boost = pTerm + integral + dTerm;

  // Soft ceiling on boost using tanh — approaches COMP_MAX_BOOST smoothly
  boost = COMP_MAX_BOOST * tanhf(boost / COMP_MAX_BOOST);

  return max(boost, 0.0f);
}

// ---------------------------------------------------------------------------
// Anti-runaway failsafe
//
// If boost has been at or near COMP_MAX_BOOST for RUNAWAY_TIME, the track
// is physically stuck and the PID can't help. Go neutral to prevent motor
// burnout or erratic behavior.
//
// Latches until stick returns to neutral (deliberate restart).
// ---------------------------------------------------------------------------
bool checkRunaway(float boost, unsigned long nowUs,
                  unsigned long &startTime, bool &active, bool &tripped) {
  if (tripped) return true;

  float threshold = COMP_MAX_BOOST * 0.85f;
  if (boost >= threshold) {
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
                           float &integral, float &boost) {
  if (fabsf(target) < 5.0f && tripped) {
    tripped = false;
    integral = 0.0f;
    boost = 0.0f;
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

  // --- 7. Base power scaling (proportional, through soft saturation) ---
  float deltaL = (float)(left  - SERVO_CENTER);
  float deltaR = (float)(right - SERVO_CENTER);

  // --- 8. PID load compensation — BOOST to push through resistance ---
  //
  // Measured current > expected current = resistance detected.
  // PID adds extra us in the direction of travel to compensate.
  //
  // Hard safety cutoff: if current exceeds CURRENT_HARD_LIMIT on either
  // motor, force it to neutral immediately (motor stall / short / jam).
  bool hardCutoffL = fabsf(curAvgL) >= CURRENT_HARD_LIMIT;
  bool hardCutoffR = fabsf(curAvgR) >= CURRENT_HARD_LIMIT;

  if (hardCutoffL) {
    deltaL = 0.0f;
    compIntegralL = 0.0f;
    compBoostL = 0.0f;
  } else {
    compBoostL = pidCompensate(deltaL, curAvgL,
                               compIntegralL, compPrevErrorL, dtSec);
    // Add boost in the direction of travel
    float sign = (deltaL >= 0.0f) ? 1.0f : -1.0f;
    deltaL += sign * compBoostL;
  }

  if (hardCutoffR) {
    deltaR = 0.0f;
    compIntegralR = 0.0f;
    compBoostR = 0.0f;
  } else {
    compBoostR = pidCompensate(deltaR, curAvgR,
                               compIntegralR, compPrevErrorR, dtSec);
    float sign = (deltaR >= 0.0f) ? 1.0f : -1.0f;
    deltaR += sign * compBoostR;
  }

  // --- 9. Anti-runaway failsafe ---
  // Compensation stuck at max for too long = track jammed/stalled.
  // Go neutral and latch until stick returns to center.
  bool failL = checkRunaway(compBoostL, nowUs,
                            runawayStartL, runawayActiveL, runawayTrippedL);
  bool failR = checkRunaway(compBoostR, nowUs,
                            runawayStartR, runawayActiveR, runawayTrippedR);

  if (failL) deltaL = 0.0f;
  if (failR) deltaR = 0.0f;

  // --- 10. Soft power limit (tanh saturation, no hard clamps) ---
  int scaledL = scalePowerSoft(deltaL);
  int scaledR = scalePowerSoft(deltaR);

  // --- 11. Inertia simulation (heavy machine feel) ---
  float targetL = (float)(scaledL - SERVO_CENTER);
  float targetR = (float)(scaledR - SERVO_CENTER);

  // Reset runaway latch if stick at neutral
  resetRunawayIfNeutral(targetL, runawayTrippedL, compIntegralL, compBoostL);
  resetRunawayIfNeutral(targetR, runawayTrippedR, compIntegralR, compBoostR);

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

  // --- 12. Write to ESCs ---
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // --- 13. Serial telemetry (20 Hz, non-blocking) ---
  if (nowUs - prevSerialUs >= SERIAL_INTERVAL_US) {
    prevSerialUs = nowUs;
    Serial.print("D2=");   Serial.print(rawRCLeft);
    Serial.print("  D4="); Serial.print(rawRCRight);
    Serial.print("  D7="); Serial.print(rawOverride);
    Serial.print("  A0="); Serial.print(rawJoyY);
    Serial.print("  A1="); Serial.print(rawJoyX);
    Serial.print("  IL="); Serial.print(curAvgL, 1);
    Serial.print("  IR="); Serial.print(curAvgR, 1);
    Serial.print("  BL="); Serial.print(compBoostL, 1);
    Serial.print("  BR="); Serial.print(compBoostR, 1);
    Serial.print("  L=");  Serial.print(outLeft);
    Serial.print("  R=");  Serial.println(outRight);
  }
}
