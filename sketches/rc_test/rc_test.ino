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
//   - Exponential response curve — fine control at low stick, full authority at max
//   - Inertia simulation — physics-based "heavy machine" feel replaces EMA
//   - Current sensor inputs (A2, A3) — one per motor/ESC
//   - PID current limiter — soft power reduction when current exceeds threshold
//   - Soft power limits — progressive reduction, no hard clamp
//   - Power limit raised to 50%
//
// Hardware:
//   MCU:       STM32U585 (Arm Cortex-M33, 160MHz, 786KB RAM, 2MB flash)
//   ADC:       14-bit (0–16383), 3.3V reference
//   Logic:     3.3V — DO NOT connect 5V signals directly to any pin!
//   ESCs:      XC E10 Sensored Brushless 140A (×2)
//   Motors:    XC E3665 2500KV Sensored Brushless (×2)
//   Battery:   OVONIC 3S LiPo 15000mAh 130C 11.1V
//   RC:        Radiolink RC6GS V3 + R7FG gyro receiver
//   Joystick:  Genie 101174GT dual-axis (5V, needs divider to 3.3V)
//
// Inputs:
//   D2  <- RC CH1 → Left motor   (servo PWM 1000-2000μs, already mixed)
//   D4  <- RC CH2 → Right motor  (servo PWM 1000-2000μs, already mixed)
//   D7  <- RC CH5 Override        (servo PWM, 3-position switch)
//   A0  <- Joystick Y axis        (analog 0-3.3V via divider, throttle)
//   A1  <- Joystick X axis        (analog 0-3.3V via divider, steering)
//   A2  <- Current sensor LEFT    (analog, hall-effect)
//   A3  <- Current sensor RIGHT   (analog, hall-effect)
//
// Outputs:
//   D9  -> Left track ESC    (servo PWM 1000-2000μs)
//   D10 -> Right track ESC   (servo PWM 1000-2000μs)
//
// Override modes (CH5):
//   LOW  (<1250μs)      Mode 1: RC only, joystick disabled
//   MID  (1250-1750μs)  Mode 2: Joystick active, RC overrides when non-neutral
//   HIGH (>1750μs)      Mode 3: 50/50 blend of RC + joystick
//
// =============================================================================

#include <Servo.h>

// =============================================================================
// Pin Assignments
// =============================================================================
const uint8_t PIN_RC_LEFT     = 2;    // RC CH1 (left motor)
const uint8_t PIN_RC_RIGHT    = 4;    // RC CH2 (right motor)
const uint8_t PIN_RC_OVERRIDE = 7;    // RC CH5 (mode switch)
const uint8_t PIN_JOY_Y       = A0;   // Joystick throttle (via 5V→3.3V divider)
const uint8_t PIN_JOY_X       = A1;   // Joystick steering (via 5V→3.3V divider)
const uint8_t PIN_CUR_LEFT    = A2;   // Current sensor — left motor
const uint8_t PIN_CUR_RIGHT   = A3;   // Current sensor — right motor
const uint8_t PIN_ESC_LEFT    = 9;    // ~PWM output
const uint8_t PIN_ESC_RIGHT   = 10;   // ~PWM output

// =============================================================================
// ADC Configuration (14-bit, 3.3V reference)
// =============================================================================
const int  ADC_MAX         = 16383;   // 14-bit max
const int  ADC_CENTER      = 8192;    // midpoint
const float ADC_VOLTS      = 3.3f;    // reference voltage

// =============================================================================
// Joystick Tuning
// =============================================================================
const int  JOY_DEADBAND    = 480;     // ~3% of 14-bit range (dead zone at center)

// Exponential curve exponent: higher = more fine control at low stick.
// 1.0 = linear, 2.0 = quadratic, 2.5 = excavator-like, 3.0 = very fine
const float EXP_CURVE      = 2.5f;

// =============================================================================
// RC Tuning
// =============================================================================
const int  RC_DEADBAND      = 50;     // ±50μs around 1500 = neutral
const int  SERVO_CENTER     = 1500;
const int  SERVO_MIN        = 1000;
const int  SERVO_MAX        = 2000;
const int  MODE_LOW_THRESH  = 1250;   // CH5 below = Mode 1
const int  MODE_HIGH_THRESH = 1750;   // CH5 above = Mode 3

// =============================================================================
// Power Limits
// =============================================================================
const int  POWER_LIMIT_PCT  = 50;     // Max output as % of full range
const int  OUTPUT_MIN       = SERVO_CENTER - (500L * POWER_LIMIT_PCT / 100); // 1250
const int  OUTPUT_MAX       = SERVO_CENTER + (500L * POWER_LIMIT_PCT / 100); // 1750

// =============================================================================
// Inertia Simulation — physics-based "heavy machine" feel
//
// Instead of simple EMA smoothing, this simulates a mass with friction.
// The virtual mass resists sudden changes (heavy feel), friction brings
// the system to rest (no creep), and applied force is proportional to
// how far the stick is from the current output (spring-like tracking).
//
// Tune VIRTUAL_MASS higher for heavier/slower response.
// Tune FRICTION higher for quicker stops (less coasting).
// Tune RESPONSE_FORCE for how aggressively it tracks the stick.
// =============================================================================
const float VIRTUAL_MASS    = 3.0f;   // kg equivalent — higher = more sluggish
const float FRICTION_COEFF  = 8.0f;   // drag — higher = stops faster
const float RESPONSE_FORCE  = 20.0f;  // spring constant — higher = tracks stick faster

// =============================================================================
// Current Sensor Configuration
//
// Configure for your sensor (e.g. ACS758LCB-100B):
//   - Bidirectional: 0A = Vcc/2 (1.65V at 3.3V supply, or 2.5V at 5V supply)
//   - Sensitivity: 20mV/A (ACS758-100B)
//   - If sensor runs on 5V and outputs >3.3V, you NEED a voltage divider!
//
// Adjust these values to match your specific sensor and wiring:
// =============================================================================
const float CUR_ZERO_V      = 1.65f;  // Voltage at 0A (Vcc/2 for bidirectional)
const float CUR_SENS_MV_A   = 20.0f;  // mV per Amp (from sensor datasheet)
const float CUR_DIVIDER     = 1.0f;   // Voltage divider ratio (1.0 = no divider)

// =============================================================================
// PID Current Limiter
//
// When measured motor current exceeds CURRENT_SOFT_LIMIT, the PID starts
// reducing the power output progressively. At CURRENT_HARD_LIMIT, output
// is forced to zero. Between soft and hard, power is reduced smoothly.
//
// This is a single-sided PID: it only acts when current exceeds the
// soft limit. Below the limit, no reduction is applied (PID output = 0).
// =============================================================================
const float CURRENT_SOFT_LIMIT = 60.0f;  // Amps — PID starts reducing power
const float CURRENT_HARD_LIMIT = 100.0f; // Amps — force stop (safety cutoff)
const float PID_KP           = 0.015f;   // Proportional — immediate response
const float PID_KI           = 0.005f;   // Integral — eliminate steady-state error
const float PID_KD           = 0.002f;   // Derivative — dampen oscillation
const float PID_I_MAX        = 0.5f;     // Anti-windup clamp for integral term

// =============================================================================
// Timing
// =============================================================================
const long FAILSAFE_TIMEOUT = 500;     // Neutral if RC lost for this many ms
const int  SERIAL_INTERVAL  = 50;      // Serial output every N ms (20 Hz)

// =============================================================================
// Servo Objects
// =============================================================================
Servo escLeft;
Servo escRight;

// =============================================================================
// RC Interrupt State (written by ISR, read by loop)
// =============================================================================
volatile unsigned long riseTime[3]   = {0, 0, 0};
volatile int           pulseWidth[3] = {1500, 1500, 1000};
volatile unsigned long pulseTime[3]  = {0, 0, 0};

const uint8_t CH_LEFT  = 0;
const uint8_t CH_RIGHT = 1;
const uint8_t CH_OVR   = 2;

// =============================================================================
// ISR Handlers — one per RC channel, using attachInterrupt()
//
// On UNO Q (STM32U585), all digital pins support attachInterrupt().
// No need for AVR-specific PCINT register manipulation.
// =============================================================================
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

// Inertia simulation state (per motor)
float velocityL = 0.0f;   // Current "velocity" of left output
float velocityR = 0.0f;   // Current "velocity" of right output
float positionL = 0.0f;   // Current output position (0 = center)
float positionR = 0.0f;

// PID current limiter state (per motor)
float pidIntegralL = 0.0f;
float pidIntegralR = 0.0f;
float pidPrevErrorL = 0.0f;
float pidPrevErrorR = 0.0f;

// Current sensor moving average (8-sample window)
const int CUR_AVG_SAMPLES = 8;
float curSamplesL[CUR_AVG_SAMPLES];
float curSamplesR[CUR_AVG_SAMPLES];
int   curSampleIdx = 0;
float curAvgL = 0.0f;
float curAvgR = 0.0f;

// Timing
unsigned long prevLoopTime   = 0;
unsigned long prevSerialTime = 0;

// =============================================================================
// Helpers
// =============================================================================

int deadbandRC(int value) {
  return (abs(value - SERVO_CENTER) <= RC_DEADBAND) ? SERVO_CENTER : value;
}

int deadbandJoy(int value) {
  return (abs(value - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : value;
}

// Convert 14-bit ADC to servo range with exponential curve.
// The expo curve gives fine control at low stick (for precise maneuvering)
// and full authority at max stick (for travel speed).
//
// input: 0–16383 (14-bit ADC)
// output: 1000–2000 (servo μs)
int joyToServoExpo(int adcValue) {
  float normalized = (float)(adcValue - ADC_CENTER) / (float)(ADC_CENTER);
  normalized = constrain(normalized, -1.0f, 1.0f);

  // Apply exponential curve: preserve sign, raise magnitude to power
  float sign = (normalized >= 0.0f) ? 1.0f : -1.0f;
  float curved = sign * powf(fabsf(normalized), EXP_CURVE);

  return SERVO_CENTER + (int)(curved * 500.0f);
}

// Read current sensor and convert to Amps.
// ADC reads voltage after optional divider, then converts to current.
float readCurrent(uint8_t pin) {
  int raw = analogRead(pin);
  float voltage = (float)raw * ADC_VOLTS / (float)ADC_MAX;
  float sensorV = voltage / CUR_DIVIDER;  // Undo divider to get actual sensor output
  return (sensorV - CUR_ZERO_V) * 1000.0f / CUR_SENS_MV_A;
}

// Update moving average for current readings
void updateCurrentAvg() {
  curSamplesL[curSampleIdx] = readCurrent(PIN_CUR_LEFT);
  curSamplesR[curSampleIdx] = readCurrent(PIN_CUR_RIGHT);
  curSampleIdx = (curSampleIdx + 1) % CUR_AVG_SAMPLES;

  float sumL = 0.0f, sumR = 0.0f;
  for (int i = 0; i < CUR_AVG_SAMPLES; i++) {
    sumL += curSamplesL[i];
    sumR += curSamplesR[i];
  }
  curAvgL = sumL / CUR_AVG_SAMPLES;
  curAvgR = sumR / CUR_AVG_SAMPLES;
}

// PID current limiter — returns a reduction factor (0.0 = no reduction, 1.0 = full stop).
// Only activates when current exceeds CURRENT_SOFT_LIMIT.
float pidCurrentLimit(float currentAmps, float &integral, float &prevError, float dtSec) {
  // Hard limit safety cutoff
  if (fabsf(currentAmps) >= CURRENT_HARD_LIMIT) {
    integral = PID_I_MAX;  // Saturate integral for fast recovery
    return 1.0f;           // Full reduction
  }

  float absCurrent = fabsf(currentAmps);

  // Below soft limit — no reduction, but decay integral toward zero
  if (absCurrent <= CURRENT_SOFT_LIMIT) {
    integral *= 0.95f;  // Gradual integral decay
    prevError = 0.0f;
    return 0.0f;
  }

  // Above soft limit — PID computes reduction
  float error = absCurrent - CURRENT_SOFT_LIMIT;

  // Proportional
  float pTerm = PID_KP * error;

  // Integral with anti-windup
  integral += PID_KI * error * dtSec;
  integral = constrain(integral, 0.0f, PID_I_MAX);

  // Derivative (on error, not measurement, for simplicity)
  float dTerm = 0.0f;
  if (dtSec > 0.0f) {
    dTerm = PID_KD * (error - prevError) / dtSec;
  }
  prevError = error;

  float reduction = pTerm + integral + dTerm;
  return constrain(reduction, 0.0f, 1.0f);
}

// Scale power proportionally and apply soft current-based reduction.
// reductionFactor: 0.0 = full power, 1.0 = zero power
int scalePowerSoft(int value, float reductionFactor) {
  long delta = value - SERVO_CENTER;  // -500 to +500
  float effectivePct = POWER_LIMIT_PCT * (1.0f - reductionFactor);
  return SERVO_CENTER + (int)(delta * effectivePct / 100.0f);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Configure 14-bit ADC resolution (UNO Q supports 14-bit on ADC1)
  analogReadResolution(14);

  // RC input pins
  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  // Attach interrupts — UNO Q supports CHANGE on all digital pins
  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),    isrRight,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVERRIDE), isrOverride, CHANGE);

  // ESC outputs
  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  // Initialize current sensor arrays
  for (int i = 0; i < CUR_AVG_SAMPLES; i++) {
    curSamplesL[i] = 0.0f;
    curSamplesR[i] = 0.0f;
  }

  prevLoopTime = millis();
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
  unsigned long now = millis();
  unsigned long dt  = now - prevLoopTime;
  if (dt == 0) return;
  prevLoopTime = now;
  float dtSec = (float)dt / 1000.0f;

  // ---------------------------------------------------------------------------
  // 1. Read RC values from ISR (atomic snapshot)
  // ---------------------------------------------------------------------------
  noInterrupts();
  int rawRCLeft    = pulseWidth[CH_LEFT];
  int rawRCRight   = pulseWidth[CH_RIGHT];
  int rawOverride  = pulseWidth[CH_OVR];
  unsigned long lastLeftTime  = pulseTime[CH_LEFT];
  unsigned long lastRightTime = pulseTime[CH_RIGHT];
  interrupts();

  // ---------------------------------------------------------------------------
  // 2. Read Joystick (14-bit, 0–16383)
  // ---------------------------------------------------------------------------
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // ---------------------------------------------------------------------------
  // 3. Read Current Sensors (moving average)
  // ---------------------------------------------------------------------------
  updateCurrentAvg();

  // ---------------------------------------------------------------------------
  // 4. Failsafe — go neutral if RC signal lost
  // ---------------------------------------------------------------------------
  unsigned long nowMicros = micros();
  bool rcLost = (nowMicros - lastLeftTime  > (unsigned long)FAILSAFE_TIMEOUT * 1000)
             && (nowMicros - lastRightTime > (unsigned long)FAILSAFE_TIMEOUT * 1000);

  int rcLeft  = rcLost ? SERVO_CENTER : deadbandRC(rawRCLeft);
  int rcRight = rcLost ? SERVO_CENTER : deadbandRC(rawRCRight);
  int override = rawOverride;

  // ---------------------------------------------------------------------------
  // 5. Joystick → Expo Curve → Tank Mix
  //    14-bit ADC with exponential response curve for excavator-like feel.
  //    Fine control at low stick, full authority at max.
  // ---------------------------------------------------------------------------
  int joyThrottle   = joyToServoExpo(deadbandJoy(rawJoyY));
  int joySteering   = joyToServoExpo(deadbandJoy(rawJoyX));
  int joySteerOff   = joySteering - SERVO_CENTER;
  int joyLeft       = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyRight      = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 6. Override Mode Selection
  // ---------------------------------------------------------------------------
  int left, right;

  if (override < MODE_LOW_THRESH) {
    // Mode 1 — RC only
    left  = rcLeft;
    right = rcRight;

  } else if (override <= MODE_HIGH_THRESH) {
    // Mode 2 — Joystick with RC override
    bool rcActive = (rcLeft != SERVO_CENTER) || (rcRight != SERVO_CENTER);
    if (rcActive && !rcLost) {
      left  = rcLeft;
      right = rcRight;
    } else {
      left  = joyLeft;
      right = joyRight;
    }

  } else {
    // Mode 3 — 50/50 blend
    left  = (rcLeft  + joyLeft)  / 2;
    right = (rcRight + joyRight) / 2;
  }

  // ---------------------------------------------------------------------------
  // 7. PID Current Limiter
  //    Computes per-motor power reduction based on measured current.
  //    Soft limit: progressively reduce power above CURRENT_SOFT_LIMIT.
  //    Hard limit: force stop at CURRENT_HARD_LIMIT.
  // ---------------------------------------------------------------------------
  float reductionL = pidCurrentLimit(curAvgL, pidIntegralL, pidPrevErrorL, dtSec);
  float reductionR = pidCurrentLimit(curAvgR, pidIntegralR, pidPrevErrorR, dtSec);

  // ---------------------------------------------------------------------------
  // 8. Power Scaling with Soft Current Limits
  //    Proportional scaling (50%) further reduced by PID current limiter.
  //    No wasted stick travel — every bit of movement produces output.
  // ---------------------------------------------------------------------------
  left  = scalePowerSoft(left,  reductionL);
  right = scalePowerSoft(right, reductionR);

  // ---------------------------------------------------------------------------
  // 9. Inertia Simulation — "Heavy Machine" Physics
  //
  //    Simulates a mass on a spring with friction. The stick position is where
  //    the spring wants to pull the output. The virtual mass resists instant
  //    changes, and friction prevents coasting.
  //
  //    This replaces the V1.2 asymmetric EMA with a more natural feel:
  //    - Heavy ramp-up (mass resists acceleration)
  //    - Controllable coast-down (friction, not abrupt stop)
  //    - No overshoot at neutral (friction dominates at low velocity)
  // ---------------------------------------------------------------------------
  float targetL = (float)(left  - SERVO_CENTER);
  float targetR = (float)(right - SERVO_CENTER);

  // Spring force: pulls output toward stick position
  float forceL = RESPONSE_FORCE * (targetL - positionL);
  float forceR = RESPONSE_FORCE * (targetR - positionR);

  // Friction force: opposes velocity (decelerates)
  float frictionL = -FRICTION_COEFF * velocityL;
  float frictionR = -FRICTION_COEFF * velocityR;

  // F = ma → a = F/m
  float accelL = (forceL + frictionL) / VIRTUAL_MASS;
  float accelR = (forceR + frictionR) / VIRTUAL_MASS;

  // Integrate velocity and position
  velocityL += accelL * dtSec;
  velocityR += accelR * dtSec;
  positionL += velocityL * dtSec;
  positionR += velocityR * dtSec;

  // Snap to zero if close enough and target is neutral (prevents micro-drift)
  if (fabsf(targetL) < 1.0f && fabsf(positionL) < 2.0f && fabsf(velocityL) < 5.0f) {
    positionL = 0.0f;
    velocityL = 0.0f;
  }
  if (fabsf(targetR) < 1.0f && fabsf(positionR) < 2.0f && fabsf(velocityR) < 5.0f) {
    positionR = 0.0f;
    velocityR = 0.0f;
  }

  int outLeft  = constrain(SERVO_CENTER + (int)(positionL + 0.5f), OUTPUT_MIN, OUTPUT_MAX);
  int outRight = constrain(SERVO_CENTER + (int)(positionR + 0.5f), OUTPUT_MIN, OUTPUT_MAX);

  // ---------------------------------------------------------------------------
  // 10. Write to ESCs
  // ---------------------------------------------------------------------------
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // ---------------------------------------------------------------------------
  // 11. Serial Telemetry (rate-limited, 20 Hz)
  //     Format: key=value pairs for live_plot.py parsing
  // ---------------------------------------------------------------------------
  if (now - prevSerialTime >= (unsigned long)SERIAL_INTERVAL) {
    prevSerialTime = now;
    Serial.print("D2=");   Serial.print(rawRCLeft);
    Serial.print("  D4="); Serial.print(rawRCRight);
    Serial.print("  D7="); Serial.print(rawOverride);
    Serial.print("  A0="); Serial.print(rawJoyY);
    Serial.print("  A1="); Serial.print(rawJoyX);
    Serial.print("  IL="); Serial.print(curAvgL, 1);
    Serial.print("  IR="); Serial.print(curAvgR, 1);
    Serial.print("  L=");  Serial.print(outLeft);
    Serial.print("  R=");  Serial.println(outRight);
  }
}
