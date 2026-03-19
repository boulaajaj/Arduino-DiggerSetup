// =============================================================================
// Tank Mixer V2.0 — Arduino Nano R4 (Renesas RA4M1)
// =============================================================================
//
// Migrated from V1.2 (Nano V3 / ATmega328P).
//
// Changes from V1.2:
//   - Hardware interrupts via attachInterrupt() (replaces PCINT)
//   - 14-bit ADC (16384 steps) for joystick — smoother exponential curve
//   - Exponential response curve — instant, excavator-like feel
//   - No acceleration ramp — response is immediate
//   - Smooth deceleration ramp (300ms) — safe stop, no jerking
//   - Power limit raised to 50%
//
// RC inputs (D2, D4) are LEFT/RIGHT motor signals — the transmitter
// already does the tank mixing. Passed through directly (no second mix).
//
// Joystick inputs (A0, A1) are raw throttle/steering and DO get
// tank-mixed by this code.
//
// Inputs:
//   D2  <- RC CH1 → Left motor   (servo PWM 1000-2000us, already mixed)
//   D4  <- RC CH2 → Right motor  (servo PWM 1000-2000us, already mixed)
//   D7  <- RC CH5 Override        (servo PWM, 3-position switch)
//   A0  <- Joystick Y axis        (analog 0-5V, throttle)
//   A1  <- Joystick X axis        (analog 0-5V, steering)
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

// -----------------------------------------------------------------------------
// Pin Assignments
// -----------------------------------------------------------------------------
const uint8_t PIN_RC_LEFT     = 2;    // RC CH1 (left motor)
const uint8_t PIN_RC_RIGHT    = 4;    // RC CH2 (right motor)
const uint8_t PIN_RC_OVERRIDE = 7;    // RC CH5 (mode switch)
const uint8_t PIN_JOY_Y       = A0;   // Joystick throttle
const uint8_t PIN_JOY_X       = A1;   // Joystick steering
const uint8_t PIN_ESC_LEFT    = 9;
const uint8_t PIN_ESC_RIGHT   = 10;

// -----------------------------------------------------------------------------
// Tuning Constants
// -----------------------------------------------------------------------------
const int  RC_DEADBAND      = 50;     // +/-50us around 1500 = neutral
const int  JOY_DEADBAND     = 480;    // ~3% of 16384 range (was 30/1024)
const int  JOY_CENTER       = 8192;   // 14-bit midpoint (was 512)
const int  JOY_MAX          = 16383;  // 14-bit max
const int  SERVO_CENTER     = 1500;
const int  SERVO_MIN        = 1000;
const int  SERVO_MAX        = 2000;
const int  POWER_LIMIT_PCT  = 50;     // Max output as % of full range
const int  OUTPUT_MIN       = SERVO_CENTER - (500L * POWER_LIMIT_PCT / 100); // 1250
const int  OUTPUT_MAX       = SERVO_CENTER + (500L * POWER_LIMIT_PCT / 100); // 1750
const int  MODE_LOW_THRESH  = 1250;   // CH5 below = Mode 1
const int  MODE_HIGH_THRESH = 1750;   // CH5 above = Mode 3
const long SMOOTH_TAU_DOWN  = 300;    // Deceleration ramp (ms) — safe stop
const long FAILSAFE_TIMEOUT = 500;    // Neutral after this long without RC (ms)
const int  SERIAL_INTERVAL  = 50;     // Serial output every N ms (20 Hz)

// Exponential curve exponent — higher = more fine control at low stick
// 2.0 = quadratic (good balance), 3.0 = cubic (very fine low-end)
const float EXP_CURVE       = 2.5;

// -----------------------------------------------------------------------------
// Servo Objects
// -----------------------------------------------------------------------------
Servo escLeft;
Servo escRight;

// -----------------------------------------------------------------------------
// Interrupt State (written by ISR, read by loop)
// -----------------------------------------------------------------------------
volatile unsigned long riseTime[3]   = {0, 0, 0};
volatile int           pulseWidth[3] = {1500, 1500, 1000};
volatile unsigned long pulseTime[3]  = {0, 0, 0};

// Channel indices
const uint8_t CH_LEFT  = 0;  // D2 — RC left motor
const uint8_t CH_RIGHT = 1;  // D4 — RC right motor
const uint8_t CH_OVR   = 2;  // D7 — Override switch

// -----------------------------------------------------------------------------
// ISR Handlers — one per RC channel, using attachInterrupt()
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Runtime State
// -----------------------------------------------------------------------------
float         smoothLeft     = SERVO_CENTER;
float         smoothRight    = SERVO_CENTER;
unsigned long prevLoopTime   = 0;
unsigned long prevSerialTime = 0;

// =============================================================================
// Helpers
// =============================================================================

int deadbandRC(int value) {
  return (abs(value - SERVO_CENTER) <= RC_DEADBAND) ? SERVO_CENTER : value;
}

int deadbandJoy(int value) {
  return (abs(value - JOY_CENTER) <= JOY_DEADBAND) ? JOY_CENTER : value;
}

// Map 14-bit joystick ADC to servo range (1000-2000us)
int joyToServo(int adcValue) {
  return map(adcValue, 0, JOY_MAX, SERVO_MIN, SERVO_MAX);
}

// Exponential power scaling — full stick travel maps to POWER_LIMIT_PCT
// with an exponential curve for fine control at low inputs.
//
// Input:  servo value 1000-2000 (center 1500)
// Output: servo value with exponential curve applied, limited to power cap
//
// At 10% stick: ~0.3% output (barely moving — fine positioning)
// At 50% stick: ~17.7% output (moderate speed)
// At 100% stick: 50% output (full allowed power)
int expScalePower(int value) {
  float delta = (float)(value - SERVO_CENTER) / 500.0f;  // -1.0 to +1.0
  float sign = (delta >= 0) ? 1.0f : -1.0f;
  float magnitude = fabs(delta);

  // Apply exponential curve: out = sign * |in|^EXP_CURVE
  float curved = sign * powf(magnitude, EXP_CURVE);

  // Scale to power limit
  return SERVO_CENTER + (int)(curved * 500.0f * POWER_LIMIT_PCT / 100.0f);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  // 14-bit ADC resolution (Nano R4 supports up to 14 bits)
  analogReadResolution(14);

  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  // Hardware interrupts — Nano R4 supports attachInterrupt on all digital pins
  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),    isrRight,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVERRIDE), isrOverride, CHANGE);

  prevLoopTime = millis();
}

// =============================================================================
// Main Loop — no delays, no blocking
// =============================================================================
void loop() {
  unsigned long now = millis();
  unsigned long dt  = now - prevLoopTime;
  if (dt == 0) return;
  prevLoopTime = now;

  // ---------------------------------------------------------------------------
  // 1. Read RC values from ISR (atomic)
  // ---------------------------------------------------------------------------
  noInterrupts();
  int rawRCLeft    = pulseWidth[CH_LEFT];
  int rawRCRight   = pulseWidth[CH_RIGHT];
  int rawOverride  = pulseWidth[CH_OVR];
  unsigned long lastLeftTime  = pulseTime[CH_LEFT];
  unsigned long lastRightTime = pulseTime[CH_RIGHT];
  interrupts();

  // ---------------------------------------------------------------------------
  // 2. Read Joystick (14-bit: 0-16383)
  // ---------------------------------------------------------------------------
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // ---------------------------------------------------------------------------
  // 3. Failsafe — go neutral if RC signal lost
  // ---------------------------------------------------------------------------
  unsigned long nowMicros = micros();
  bool rcLost = (nowMicros - lastLeftTime  > (unsigned long)FAILSAFE_TIMEOUT * 1000)
             && (nowMicros - lastRightTime > (unsigned long)FAILSAFE_TIMEOUT * 1000);

  int rcLeft  = rcLost ? SERVO_CENTER : deadbandRC(rawRCLeft);
  int rcRight = rcLost ? SERVO_CENTER : deadbandRC(rawRCRight);
  int override = rawOverride;

  // ---------------------------------------------------------------------------
  // 4. Joystick → Tank Mix
  //    Joystick outputs raw throttle/steering, so WE do the tank mix here.
  // ---------------------------------------------------------------------------
  int joyThrottle   = joyToServo(deadbandJoy(rawJoyY));
  int joySteering   = joyToServo(deadbandJoy(rawJoyX));
  int joySteerOff   = joySteering - SERVO_CENTER;
  int joyLeft       = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyRight      = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 5. Override Mode Selection
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
  // 6. Exponential Power Scaling (50%, instant response)
  //    Replaces linear scaling + EMA ramp-up. The exponential curve itself
  //    provides fine control at low inputs — no delay needed.
  // ---------------------------------------------------------------------------
  left  = expScalePower(left);
  right = expScalePower(right);

  // ---------------------------------------------------------------------------
  // 7. Deceleration-Only Smoothing
  //    Accelerating: INSTANT — no ramp, no delay, direct to target
  //    Decelerating: 300ms EMA ramp — smooth, safe stop
  // ---------------------------------------------------------------------------
  float targetDistL = fabs((float)(left  - SERVO_CENTER));
  float smoothDistL = fabs(smoothLeft  - (float)SERVO_CENTER);
  if (targetDistL >= smoothDistL) {
    // Accelerating or holding — instant response
    smoothLeft = (float)left;
  } else {
    // Decelerating — smooth ramp down
    float alphaL = (float)dt / (SMOOTH_TAU_DOWN + (float)dt);
    smoothLeft += alphaL * ((float)left - smoothLeft);
  }

  float targetDistR = fabs((float)(right - SERVO_CENTER));
  float smoothDistR = fabs(smoothRight - (float)SERVO_CENTER);
  if (targetDistR >= smoothDistR) {
    smoothRight = (float)right;
  } else {
    float alphaR = (float)dt / (SMOOTH_TAU_DOWN + (float)dt);
    smoothRight += alphaR * ((float)right - smoothRight);
  }

  int outLeft  = constrain((int)(smoothLeft  + 0.5f), OUTPUT_MIN, OUTPUT_MAX);
  int outRight = constrain((int)(smoothRight + 0.5f), OUTPUT_MIN, OUTPUT_MAX);

  // ---------------------------------------------------------------------------
  // 8. Write to ESCs
  // ---------------------------------------------------------------------------
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // ---------------------------------------------------------------------------
  // 9. Serial Output (rate-limited)
  // ---------------------------------------------------------------------------
  if (now - prevSerialTime >= (unsigned long)SERIAL_INTERVAL) {
    prevSerialTime = now;
    Serial.print("D2=");   Serial.print(rawRCLeft);
    Serial.print("  D4="); Serial.print(rawRCRight);
    Serial.print("  D7="); Serial.print(rawOverride);
    Serial.print("  A0="); Serial.print(rawJoyY);
    Serial.print("  A1="); Serial.print(rawJoyX);
    Serial.print("  L=");  Serial.print(outLeft);
    Serial.print("  R=");  Serial.println(outRight);
  }
}
