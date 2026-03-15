// =============================================================================
// Tank Mixer V1.2 — Arduino Nano V3 (ATmega328P)
// =============================================================================
//
// Non-blocking RC reading via Pin Change Interrupts (PCINT).
//
// RC inputs (D2, D4) are treated as LEFT/RIGHT motor signals — the
// transmitter already does the tank mixing internally. These are passed
// through to the ESCs directly (no second mix applied).
//
// Joystick inputs (A0, A1) are raw throttle/steering and DO get tank-mixed
// by this code before output.
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
// Pin Assignments (D2, D4, D7 are all on Port D -> PCINT2)
// -----------------------------------------------------------------------------
const uint8_t PIN_RC_LEFT     = 2;    // PD2, PCINT18  — RC CH1 (left motor)
const uint8_t PIN_RC_RIGHT    = 4;    // PD4, PCINT20  — RC CH2 (right motor)
const uint8_t PIN_RC_OVERRIDE = 7;    // PD7, PCINT23  — RC CH5 (mode switch)
const uint8_t PIN_JOY_Y       = A0;   // Joystick throttle
const uint8_t PIN_JOY_X       = A1;   // Joystick steering
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
const int  POWER_LIMIT_PCT  = 40;     // Max output as % of full range
const int  OUTPUT_MIN       = SERVO_CENTER - (500L * POWER_LIMIT_PCT / 100); // 1300
const int  OUTPUT_MAX       = SERVO_CENTER + (500L * POWER_LIMIT_PCT / 100); // 1700
const int  MODE_LOW_THRESH  = 1250;   // CH5 below = Mode 1
const int  MODE_HIGH_THRESH = 1750;   // CH5 above = Mode 3
const long SMOOTH_TAU_UP    = 800;    // EMA time constant for acceleration (ms)
const long SMOOTH_TAU_DOWN  = 400;    // EMA time constant for deceleration (ms)
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
volatile unsigned long riseTime[3]   = {0, 0, 0};
volatile int           pulseWidth[3] = {1500, 1500, 1000};
volatile unsigned long pulseTime[3]  = {0, 0, 0};
volatile uint8_t       prevPortD     = 0;

// Channel indices
const uint8_t CH_LEFT  = 0;  // D2 — RC left motor
const uint8_t CH_RIGHT = 1;  // D4 — RC right motor
const uint8_t CH_OVR   = 2;  // D7 — Override switch

// Port D bit masks
const uint8_t MASK_LEFT  = (1 << 2);  // PD2
const uint8_t MASK_RIGHT = (1 << 4);  // PD4
const uint8_t MASK_OVR   = (1 << 7);  // PD7

// -----------------------------------------------------------------------------
// Pin Change Interrupt — decodes all 3 RC channels simultaneously
// -----------------------------------------------------------------------------
ISR(PCINT2_vect) {
  unsigned long now = micros();
  uint8_t pins    = PIND;
  uint8_t changed = pins ^ prevPortD;
  prevPortD = pins;

  // D2 — Left motor
  if (changed & MASK_LEFT) {
    if (pins & MASK_LEFT) {
      riseTime[CH_LEFT] = now;
    } else {
      unsigned long pw = now - riseTime[CH_LEFT];
      if (pw >= 800 && pw <= 2200) {
        pulseWidth[CH_LEFT] = pw;
        pulseTime[CH_LEFT]  = now;
      }
    }
  }

  // D4 — Right motor
  if (changed & MASK_RIGHT) {
    if (pins & MASK_RIGHT) {
      riseTime[CH_RIGHT] = now;
    } else {
      unsigned long pw = now - riseTime[CH_RIGHT];
      if (pw >= 800 && pw <= 2200) {
        pulseWidth[CH_RIGHT] = pw;
        pulseTime[CH_RIGHT]  = now;
      }
    }
  }

  // D7 — Override switch
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

int joyToServo(int adcValue) {
  return map(adcValue, 0, 1023, SERVO_MIN, SERVO_MAX);
}

// Scale full input range (1000-2000) proportionally to limited output range.
// 50% stick → 20% output, 100% stick → 40% output. No wasted travel.
int scalePower(int value) {
  long delta = value - SERVO_CENTER;  // -500 to +500
  return SERVO_CENTER + (int)(delta * POWER_LIMIT_PCT / 100);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
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
  // 2. Read Joystick
  // ---------------------------------------------------------------------------
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);

  // ---------------------------------------------------------------------------
  // 3. Failsafe — go neutral if RC signal lost
  // ---------------------------------------------------------------------------
  unsigned long nowMicros = micros();
  bool rcLost = (nowMicros - lastLeftTime  > (unsigned long)FAILSAFE_TIMEOUT * 1000)
             && (nowMicros - lastRightTime > (unsigned long)FAILSAFE_TIMEOUT * 1000);

  // RC values are already left/right motor signals from the transmitter.
  // Apply deadband to detect neutral (for Mode 2 override logic).
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
  //    RC signals are already mixed (left/right), joystick is tank-mixed above.
  //    Both are now in the same domain: left motor value + right motor value.
  // ---------------------------------------------------------------------------
  int left, right;

  if (override < MODE_LOW_THRESH) {
    // Mode 1 — RC only: pass RC signals straight through
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
    // Mode 3 — 50/50 blend (both in left/right domain)
    left  = (rcLeft  + joyLeft)  / 2;
    right = (rcRight + joyRight) / 2;
  }

  // ---------------------------------------------------------------------------
  // 6. Power Scaling (40% proportional)
  //    Maps full input range to 40% output range — no wasted stick travel.
  //    100% stick → 40% output, 50% stick → 20% output, etc.
  // ---------------------------------------------------------------------------
  left  = scalePower(left);
  right = scalePower(right);

  // ---------------------------------------------------------------------------
  // 7. Asymmetric EMA Smoothing
  //    Accelerating (away from center): 800ms — gentle ramp up
  //    Decelerating (toward center):    400ms — quick stop
  // ---------------------------------------------------------------------------
  float targetDistL = abs(left  - SERVO_CENTER);
  float smoothDistL = abs(smoothLeft  - SERVO_CENTER);
  long tauL = (targetDistL > smoothDistL) ? SMOOTH_TAU_UP : SMOOTH_TAU_DOWN;
  float alphaL = (float)dt / (tauL + (float)dt);
  smoothLeft += alphaL * (left - smoothLeft);

  float targetDistR = abs(right - SERVO_CENTER);
  float smoothDistR = abs(smoothRight - SERVO_CENTER);
  long tauR = (targetDistR > smoothDistR) ? SMOOTH_TAU_UP : SMOOTH_TAU_DOWN;
  float alphaR = (float)dt / (tauR + (float)dt);
  smoothRight += alphaR * (right - smoothRight);

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
  if (now - prevSerialTime >= SERIAL_INTERVAL) {
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
