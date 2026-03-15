// =============================================================================
// Tank Mixer V1 — Arduino Nano V3 (ATmega328P)
// =============================================================================
//
// Reads 3 RC channels + 2 joystick axes, applies override logic, mixes to
// tank drive, smooths output with EMA, and drives two ESCs via servo PWM.
//
// Inputs:
//   D2  <- RC CH1 Throttle   (servo PWM, 1000-2000us)
//   D4  <- RC CH2 Steering   (servo PWM, 1000-2000us)
//   D7  <- RC CH5 Override   (servo PWM, 3-position switch)
//   A0  <- Joystick Y axis   (analog 0-5V, throttle)
//   A1  <- Joystick X axis   (analog 0-5V, steering)
//
// Outputs:
//   D9  -> Left track ESC    (servo PWM, 1000-2000us)
//   D10 -> Right track ESC   (servo PWM, 1000-2000us)
//
// Override modes (CH5):
//   LOW  (<1250us)  Mode 1: RC only, joystick disabled
//   MID  (1250-1750us)  Mode 2: Joystick active, RC overrides when non-neutral
//   HIGH (>1750us)  Mode 3: 50/50 blend of RC + joystick
//
// Note: pulseIn() is blocking — D4 is read first to minimize missed pulses.
//       Migrate to attachInterrupt() on Nano R4 for non-blocking reads.
//
// =============================================================================

#include <Servo.h>

// -----------------------------------------------------------------------------
// Pin Assignments
// -----------------------------------------------------------------------------
const int PIN_RC_THROTTLE  = 2;
const int PIN_RC_STEERING  = 4;
const int PIN_RC_OVERRIDE  = 7;
const int PIN_JOY_Y        = A0;
const int PIN_JOY_X        = A1;
const int PIN_ESC_LEFT     = 9;
const int PIN_ESC_RIGHT    = 10;

// -----------------------------------------------------------------------------
// Tuning Constants
// -----------------------------------------------------------------------------
const int  RC_DEADBAND      = 50;     // +/-50us around 1500 = neutral
const int  JOY_DEADBAND     = 30;     // +/-30 around 512 = neutral
const int  JOY_CENTER       = 512;
const int  SERVO_CENTER     = 1500;   // Neutral servo position (us)
const int  SERVO_MIN        = 1000;   // Full reverse (us)
const int  SERVO_MAX        = 2000;   // Full forward (us)
const int  MODE_LOW_THRESH  = 1250;   // CH5 below this = Mode 1
const int  MODE_HIGH_THRESH = 1750;   // CH5 above this = Mode 3
const long SMOOTH_TAU       = 800;    // EMA time constant (ms)
const long FAILSAFE_TIMEOUT = 500;    // Go neutral after this long without RC (ms)
const long PULSE_TIMEOUT    = 25000;  // pulseIn timeout (us)

// -----------------------------------------------------------------------------
// Servo Objects
// -----------------------------------------------------------------------------
Servo escLeft;
Servo escRight;

// -----------------------------------------------------------------------------
// Runtime State
// -----------------------------------------------------------------------------
float         smoothLeft    = SERVO_CENTER;
float         smoothRight   = SERVO_CENTER;
unsigned long prevLoopTime  = 0;
unsigned long lastRCTime    = 0;       // Last time we got a valid RC pulse
int           holdThrottle  = SERVO_CENTER;
int           holdSteering  = SERVO_CENTER;
int           holdOverride  = SERVO_MIN;

// =============================================================================
// Helpers
// =============================================================================

// Snap RC value to center if within deadband. Preserves 0 (no signal).
int deadbandRC(int value) {
  if (value == 0) return 0;
  return (abs(value - SERVO_CENTER) <= RC_DEADBAND) ? SERVO_CENTER : value;
}

// Snap joystick value to center if within deadband.
int deadbandJoy(int value) {
  return (abs(value - JOY_CENTER) <= JOY_DEADBAND) ? JOY_CENTER : value;
}

// Convert joystick ADC (0-1023) to servo range (1000-2000us).
int joyToServo(int adcValue) {
  return map(adcValue, 0, 1023, SERVO_MIN, SERVO_MAX);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEERING, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  prevLoopTime = millis();
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
  unsigned long now = millis();
  unsigned long dt  = now - prevLoopTime;
  prevLoopTime = now;

  // ---------------------------------------------------------------------------
  // 1. Read Inputs
  //    D4 (steering) is read first to avoid pulseIn blocking on the Nano V3.
  // ---------------------------------------------------------------------------
  int rawSteering = pulseIn(PIN_RC_STEERING, HIGH, PULSE_TIMEOUT);
  int rawThrottle = pulseIn(PIN_RC_THROTTLE, HIGH, PULSE_TIMEOUT);
  int rawOverride = pulseIn(PIN_RC_OVERRIDE, HIGH, PULSE_TIMEOUT);
  int rawJoyY     = analogRead(PIN_JOY_Y);
  int rawJoyX     = analogRead(PIN_JOY_X);

  // ---------------------------------------------------------------------------
  // 2. Failsafe — hold last valid RC values when signal drops
  // ---------------------------------------------------------------------------
  if (rawThrottle > 0) { holdThrottle = rawThrottle; lastRCTime = now; }
  if (rawSteering > 0)   holdSteering = rawSteering;
  if (rawOverride > 0)   holdOverride = rawOverride;

  int rcThrottle = deadbandRC(holdThrottle);
  int rcSteering = deadbandRC(holdSteering);
  int override   = holdOverride;

  bool rcLost = (now - lastRCTime > FAILSAFE_TIMEOUT);
  if (rcLost) {
    rcThrottle = SERVO_CENTER;
    rcSteering = SERVO_CENTER;
  }

  // ---------------------------------------------------------------------------
  // 3. Joystick Processing
  // ---------------------------------------------------------------------------
  int joyThrottle = joyToServo(deadbandJoy(rawJoyY));
  int joySteering = joyToServo(deadbandJoy(rawJoyX));

  // ---------------------------------------------------------------------------
  // 4. Override Mode Selection
  // ---------------------------------------------------------------------------
  int throttle, steering;

  if (override < MODE_LOW_THRESH) {
    // Mode 1 — RC only: joystick completely ignored
    throttle = rcThrottle;
    steering = rcSteering;

  } else if (override <= MODE_HIGH_THRESH) {
    // Mode 2 — Joystick with RC override: RC takes priority when non-neutral
    bool rcActive = (rcThrottle != SERVO_CENTER) || (rcSteering != SERVO_CENTER);
    if (rcActive && !rcLost) {
      throttle = rcThrottle;
      steering = rcSteering;
    } else {
      throttle = joyThrottle;
      steering = joySteering;
    }

  } else {
    // Mode 3 — Blended: 50% RC + 50% joystick
    throttle = (rcThrottle + joyThrottle) / 2;
    steering = (rcSteering + joySteering) / 2;
  }

  // ---------------------------------------------------------------------------
  // 5. Tank Mix
  //    Left  = Throttle + Steering offset
  //    Right = Throttle - Steering offset
  // ---------------------------------------------------------------------------
  int steerOffset = steering - SERVO_CENTER;
  int left  = constrain(throttle + steerOffset, SERVO_MIN, SERVO_MAX);
  int right = constrain(throttle - steerOffset, SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 6. EMA Smoothing (800ms time constant)
  // ---------------------------------------------------------------------------
  float alpha = (float)dt / (SMOOTH_TAU + (float)dt);
  smoothLeft  += alpha * (left  - smoothLeft);
  smoothRight += alpha * (right - smoothRight);
  int outLeft  = constrain((int)smoothLeft,  SERVO_MIN, SERVO_MAX);
  int outRight = constrain((int)smoothRight, SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 7. Write to ESCs
  // ---------------------------------------------------------------------------
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // ---------------------------------------------------------------------------
  // 8. Serial Output (for live_plot.py)
  // ---------------------------------------------------------------------------
  Serial.print("D4=");    Serial.print(rawSteering > 0 ? rawSteering : holdSteering);
  Serial.print("  D2=");  Serial.print(rawThrottle > 0 ? rawThrottle : holdThrottle);
  Serial.print("  D7=");  Serial.print(rawOverride > 0 ? rawOverride : holdOverride);
  Serial.print("  A0=");  Serial.print(rawJoyY);
  Serial.print("  A1=");  Serial.print(rawJoyX);
  Serial.print("  L=");   Serial.print(outLeft);
  Serial.print("  R=");   Serial.println(outRight);

  delay(20);
}
