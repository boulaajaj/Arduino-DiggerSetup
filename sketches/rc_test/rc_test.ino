// =============================================================================
// Tank Mixer V1.1 — Arduino Nano V3 (ATmega328P)
// =============================================================================
//
// Non-blocking RC reading via Pin Change Interrupts (PCINT).
// All 3 RC channels are decoded simultaneously in hardware ISR.
// Loop runs at full speed for responsive mixing and smooth ESC output.
//
// Inputs:
//   D2  <- RC CH1 Throttle   (PCINT18, servo PWM 1000-2000us)
//   D4  <- RC CH2 Steering   (PCINT20, servo PWM 1000-2000us)
//   D7  <- RC CH5 Override   (PCINT23, servo PWM 3-position switch)
//   A0  <- Joystick Y axis   (analog 0-5V, throttle)
//   A1  <- Joystick X axis   (analog 0-5V, steering)
//
// Outputs:
//   D9  -> Left track ESC    (servo PWM 1000-2000us)
//   D10 -> Right track ESC   (servo PWM 1000-2000us)
//
// Tank mix (both outputs are motors driving tracks):
//   Throttle up   -> both tracks forward
//   Throttle down -> both tracks backward
//   Steer right   -> left track faster, right track slower
//   Steer left    -> right track faster, left track slower
//
// Override modes (CH5):
//   LOW  (<1250us)      Mode 1: RC only, joystick disabled
//   MID  (1250-1750us)  Mode 2: Joystick active, RC overrides when non-neutral
//   HIGH (>1750us)      Mode 3: 50/50 blend of RC + joystick
//
// =============================================================================

#include <Servo.h>

// -----------------------------------------------------------------------------
// Pin Assignments (D2, D4, D7 are all on Port D -> PCINT2)
// -----------------------------------------------------------------------------
const uint8_t PIN_RC_THROTTLE = 2;   // PD2, PCINT18
const uint8_t PIN_RC_STEERING = 4;   // PD4, PCINT20
const uint8_t PIN_RC_OVERRIDE = 7;   // PD7, PCINT23
const uint8_t PIN_JOY_Y       = A0;
const uint8_t PIN_JOY_X       = A1;
const uint8_t PIN_ESC_LEFT    = 9;
const uint8_t PIN_ESC_RIGHT   = 10;

// -----------------------------------------------------------------------------
// Tuning Constants
// -----------------------------------------------------------------------------
const int  RC_DEADBAND      = 50;     // +/-50us around 1500 = neutral
const int  JOY_DEADBAND     = 30;     // +/-30 around 512 = neutral
const int  JOY_CENTER       = 512;
const int  SERVO_CENTER     = 1500;
const int  SERVO_MIN        = 1000;
const int  SERVO_MAX        = 2000;
const int  MODE_LOW_THRESH  = 1250;   // CH5 below = Mode 1
const int  MODE_HIGH_THRESH = 1750;   // CH5 above = Mode 3
const long SMOOTH_TAU       = 800;    // EMA time constant (ms)
const long FAILSAFE_TIMEOUT = 500;    // Neutral after this long without RC (ms)
const int  SERIAL_INTERVAL  = 50;     // Serial output every N ms (20 Hz)

// -----------------------------------------------------------------------------
// Servo Objects
// -----------------------------------------------------------------------------
Servo escLeft;
Servo escRight;

// -----------------------------------------------------------------------------
// Interrupt State (written by ISR, read by loop)
// -----------------------------------------------------------------------------
volatile unsigned long riseTime[3]  = {0, 0, 0};     // Rising edge timestamp
volatile int           pulseWidth[3] = {1500, 1500, 1000}; // Decoded pulse width
volatile unsigned long pulseTime[3] = {0, 0, 0};     // Last valid pulse timestamp
volatile uint8_t       prevPortD    = 0;

// Channel indices
const uint8_t CH_THR = 0;
const uint8_t CH_STR = 1;
const uint8_t CH_OVR = 2;

// Port D bit masks for each channel
const uint8_t MASK_THR = (1 << 2);  // PD2
const uint8_t MASK_STR = (1 << 4);  // PD4
const uint8_t MASK_OVR = (1 << 7);  // PD7

// -----------------------------------------------------------------------------
// Pin Change Interrupt — decodes all 3 RC channels simultaneously
// -----------------------------------------------------------------------------
ISR(PCINT2_vect) {
  unsigned long now = micros();
  uint8_t pins    = PIND;
  uint8_t changed = pins ^ prevPortD;
  prevPortD = pins;

  // D2 — Throttle
  if (changed & MASK_THR) {
    if (pins & MASK_THR) {
      riseTime[CH_THR] = now;
    } else {
      unsigned long pw = now - riseTime[CH_THR];
      if (pw >= 800 && pw <= 2200) {
        pulseWidth[CH_THR] = pw;
        pulseTime[CH_THR]  = now;
      }
    }
  }

  // D4 — Steering
  if (changed & MASK_STR) {
    if (pins & MASK_STR) {
      riseTime[CH_STR] = now;
    } else {
      unsigned long pw = now - riseTime[CH_STR];
      if (pw >= 800 && pw <= 2200) {
        pulseWidth[CH_STR] = pw;
        pulseTime[CH_STR]  = now;
      }
    }
  }

  // D7 — Override
  if (changed & MASK_OVR) {
    if (pins & MASK_OVR) {
      riseTime[CH_OVR] = now;
    } else {
      unsigned long pw = now - riseTime[CH_OVR];
      if (pw >= 800 && pw <= 2200) {
        pulseWidth[CH_OVR] = pw;
        pulseTime[CH_OVR]  = now;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Runtime State
// -----------------------------------------------------------------------------
float         smoothLeft    = SERVO_CENTER;
float         smoothRight   = SERVO_CENTER;
unsigned long prevLoopTime  = 0;
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

int joyToServo(int adcValue) {
  return map(adcValue, 0, 1023, SERVO_MIN, SERVO_MAX);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEERING, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  // Enable Pin Change Interrupts on Port D (D0-D7)
  prevPortD = PIND;
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT20) | (1 << PCINT23);

  prevLoopTime = millis();
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
  unsigned long now = millis();
  unsigned long dt  = now - prevLoopTime;
  if (dt == 0) return;  // Skip if called faster than 1ms
  prevLoopTime = now;

  // ---------------------------------------------------------------------------
  // 1. Read RC values from ISR (atomic read with interrupts disabled)
  // ---------------------------------------------------------------------------
  noInterrupts();
  int rawThrottle  = pulseWidth[CH_THR];
  int rawSteering  = pulseWidth[CH_STR];
  int rawOverride  = pulseWidth[CH_OVR];
  unsigned long lastThrTime = pulseTime[CH_THR];
  unsigned long lastStrTime = pulseTime[CH_STR];
  interrupts();

  // ---------------------------------------------------------------------------
  // 2. Read Joystick (analog, fast — ~100us per read)
  // ---------------------------------------------------------------------------
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // ---------------------------------------------------------------------------
  // 3. Failsafe — go neutral if RC signal lost
  // ---------------------------------------------------------------------------
  unsigned long nowMicros = micros();
  bool rcLost = (nowMicros - lastThrTime > (unsigned long)FAILSAFE_TIMEOUT * 1000)
             && (nowMicros - lastStrTime > (unsigned long)FAILSAFE_TIMEOUT * 1000);

  int rcThrottle = rcLost ? SERVO_CENTER : deadbandRC(rawThrottle);
  int rcSteering = rcLost ? SERVO_CENTER : deadbandRC(rawSteering);
  int override   = rawOverride;

  // ---------------------------------------------------------------------------
  // 4. Joystick Processing
  // ---------------------------------------------------------------------------
  int joyThrottle = joyToServo(deadbandJoy(rawJoyY));
  int joySteering = joyToServo(deadbandJoy(rawJoyX));

  // ---------------------------------------------------------------------------
  // 5. Override Mode Selection
  // ---------------------------------------------------------------------------
  int throttle, steering;

  if (override < MODE_LOW_THRESH) {
    // Mode 1 — RC only
    throttle = rcThrottle;
    steering = rcSteering;

  } else if (override <= MODE_HIGH_THRESH) {
    // Mode 2 — Joystick with RC override
    bool rcActive = (rcThrottle != SERVO_CENTER) || (rcSteering != SERVO_CENTER);
    if (rcActive && !rcLost) {
      throttle = rcThrottle;
      steering = rcSteering;
    } else {
      throttle = joyThrottle;
      steering = joySteering;
    }

  } else {
    // Mode 3 — 50/50 blend
    throttle = (rcThrottle + joyThrottle) / 2;
    steering = (rcSteering + joySteering) / 2;
  }

  // ---------------------------------------------------------------------------
  // 6. Tank Mix
  //    Both ESCs drive track motors (not servos).
  //    Throttle controls forward/backward. Steering is differential.
  //    1500 = stop, >1500 = forward, <1500 = backward.
  // ---------------------------------------------------------------------------
  int steerOffset = steering - SERVO_CENTER;
  int left  = constrain(throttle + steerOffset, SERVO_MIN, SERVO_MAX);
  int right = constrain(throttle - steerOffset, SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 7. EMA Smoothing (800ms time constant)
  // ---------------------------------------------------------------------------
  float alpha = (float)dt / (SMOOTH_TAU + (float)dt);
  smoothLeft  += alpha * (left  - smoothLeft);
  smoothRight += alpha * (right - smoothRight);
  int outLeft  = constrain((int)(smoothLeft  + 0.5f), SERVO_MIN, SERVO_MAX);
  int outRight = constrain((int)(smoothRight + 0.5f), SERVO_MIN, SERVO_MAX);

  // ---------------------------------------------------------------------------
  // 8. Write to ESCs
  // ---------------------------------------------------------------------------
  escLeft.writeMicroseconds(outLeft);
  escRight.writeMicroseconds(outRight);

  // ---------------------------------------------------------------------------
  // 9. Serial Output (rate-limited to avoid TX buffer blocking)
  // ---------------------------------------------------------------------------
  if (now - prevSerialTime >= SERIAL_INTERVAL) {
    prevSerialTime = now;
    Serial.print("D2=");   Serial.print(rawThrottle);
    Serial.print("  D4="); Serial.print(rawSteering);
    Serial.print("  D7="); Serial.print(rawOverride);
    Serial.print("  A0="); Serial.print(rawJoyY);
    Serial.print("  A1="); Serial.print(rawJoyX);
    Serial.print("  L=");  Serial.print(outLeft);
    Serial.print("  R=");  Serial.println(outRight);
  }
}
