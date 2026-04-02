// ═══════════════════════════════════════════════════════════════
// Digger Control V3.1 — Arduino Nano R4
// ═══════════════════════════════════════════════════════════════
//
// Dual-input tank mixer for 3-ton ride-on excavator.
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Signal flow:
//   RC Input ──┐
//              ├─► Mixer ─► Spin Limiter ─► Reverse Limiter
//   Joystick ─┘         ─► Soft Limit ─► Inertia ─► ESC
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [RC]         RC input — ISR (D2/D3) + non-blocking poll (D4/D7)
//   [JOYSTICK]   ADC input — deadband, expo curve, tank mix
//   [MIXER]      Override switch — selects RC vs joystick
//   [DYNAMICS]   Spin/reverse limiter, soft power cap, inertia
//   [OUTPUT]     ESC servo PWM
//   [DEBUG]      10 Hz serial CSV telemetry
//
// Pin map:
//   D2  ← RC CH1 (left track)       [interrupt]
//   D3  ← RC CH4 (control mode)     [interrupt]
//   D4  ← RC CH2 (right track)      [non-blocking poll]
//   D7  ← RC CH5 (override switch)  [non-blocking poll]
//   A0  ← Joystick Y (throttle)     [14-bit ADC]
//   A1  ← Joystick X (steering)     [14-bit ADC]
//   D9  → Left ESC                   [Servo PWM]
//   D10 → Right ESC                  [Servo PWM]
//
// Reference: docs/CONTROL-RESEARCH.md

#include <Servo.h>
#include "types.h"


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — All tunable constants
// ═══════════════════════════════════════════════════════════════

// Pins
const uint8_t PIN_RC_CH1 = 2;   // Left track (interrupt)
const uint8_t PIN_RC_CH4 = 3;   // Control mode (interrupt)
const uint8_t PIN_RC_CH2 = 4;   // Right track (polled)
const uint8_t PIN_RC_CH5 = 7;   // Override switch (polled)
const uint8_t PIN_JOY_Y  = A0;  // Throttle
const uint8_t PIN_JOY_X  = A1;  // Steering
const uint8_t PIN_ESC_L  = 9;   // Left ESC
const uint8_t PIN_ESC_R  = 10;  // Right ESC

// Servo PWM range
const int SVC   = 1500;  // Center (neutral)
const int SVMIN = 1000;  // Full reverse
const int SVMAX = 2000;  // Full forward

// ADC
const int ADC_CENTER = 8192;  // 14-bit midpoint

// Deadbands
const int RC_DEADBAND  = 50;   // RC pulse (us)
const int JOY_DEADBAND = 480;  // Joystick ADC (~5.9% of travel)

// Override switch thresholds (CH5 pulse width)
const int OVR_LO = 1400;  // Below → RC only
const int OVR_HI = 1600;  // Above → Joystick only

// Expo curve: output = LINEAR*|x| + SQUARE*x^2 (must sum to 1.0)
const float EXPO_LINEAR = 0.3f;  // More low-speed authority for tight spaces
const float EXPO_SQUARE = 0.7f;  // Smooth high-speed range

// Power range
const float SOFT_RANGE = 400.0f;  // Max servo offset from center (us)

// Inertia — asymmetric exponential filter
// Tuned for ~50 lb machine, 4-5 ft long. Smooth but nimble.
const float TAU_ACCEL = 0.3f;  // Accel (s) — 63% in 0.3s, 95% in ~0.9s
const float TAU_DECEL = 0.5f;  // Decel/coast (s) — 63% in 0.5s, 95% in ~1.5s

// Spin turn safety — power cap during counter-rotation
const float SPIN_LIMIT = 0.35f;  // 35% at full pivot

// Reverse speed limit — percentage of forward max
const float REVERSE_LIMIT = 0.35f;  // 35% max reverse

// Failsafe
const unsigned long FAILSAFE_US = 500000UL;  // 0.5s per-channel timeout

// Debug
const unsigned long PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output


// ═══════════════════════════════════════════════════════════════
// [RC] — RC input: ISR for D2/D3, non-blocking poll for D4/D7
// ═══════════════════════════════════════════════════════════════
//
// D2/D3 have hardware interrupt support — ISR fires on every edge.
// D4/D7 do NOT support attachInterrupt on Nano R4 (confirmed).
// Instead of blocking pulseIn(), we use PulseReader (types.h) which
// samples pin state every loop iteration and tracks edges.
//
// All channels have independent failsafe — if a channel hasn't
// received a valid pulse within FAILSAFE_US, it returns neutral.

// Interrupt channels (D2, D3)
RCChannel rc1 = {SVC, 0, false};  // Left track
RCChannel rc4 = {SVC, 0, false};  // Control mode

volatile unsigned long isr1Rise = 0, isr4Rise = 0;
volatile int isr1Pw = SVC, isr4Pw = SVC;
volatile unsigned long isr1Time = 0, isr4Time = 0;

void isrCh1() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_CH1) == HIGH) {
    isr1Rise = now;
  } else {
    unsigned long pw = now - isr1Rise;
    if (pw >= 800 && pw <= 2200) { isr1Pw = pw; isr1Time = now; }
  }
}

void isrCh4() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_CH4) == HIGH) {
    isr4Rise = now;
  } else {
    unsigned long pw = now - isr4Rise;
    if (pw >= 800 && pw <= 2200) { isr4Pw = pw; isr4Time = now; }
  }
}

// Polled channels (D4, D7) — non-blocking, zero delay
PulseReader poll2, poll5;

// Snapshot: copy ISR data + evaluate all failsafes with FRESH timestamp
void rcSnapshot() {
  unsigned long now = micros();  // Fresh — never stale

  // Copy interrupt data atomically
  noInterrupts();
  rc1.pw = isr1Pw;  rc1.lastOk = isr1Time;
  rc4.pw = isr4Pw;  rc4.lastOk = isr4Time;
  interrupts();

  // Per-channel failsafe
  rc1.valid = (now - rc1.lastOk) < FAILSAFE_US;
  rc4.valid = (now - rc4.lastOk) < FAILSAFE_US;

  // Polled channels use their own timestamps (always fresh)
  poll2.valid = (now - poll2.lastOk) < FAILSAFE_US;
  poll5.valid = (now - poll5.lastOk) < FAILSAFE_US;
}

int rcDeadband(int pw) {
  return (abs(pw - SVC) <= RC_DEADBAND) ? SVC : pw;
}

// Safe accessors — return neutral if channel has no signal
int rcLeft()     { return rc1.valid ? rcDeadband(rc1.pw) : SVC; }
int rcRight()    { return poll2.valid ? rcDeadband(poll2.pw) : SVC; }
int rcOverride() { return poll5.valid ? poll5.pw : SVC; }


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve, tank mix
// ═══════════════════════════════════════════════════════════════
//
// 14-bit ADC. Double-read clears mux crosstalk between channels.
// Expo gives fine control at low stick, full range at high stick.
// Tank mix: throttle +/- steering → left/right.

float expoCurve(float x) {
  float a = fabsf(x);
  return EXPO_LINEAR * a + EXPO_SQUARE * a * a;
}

int joyDeadband(int adc) {
  return (abs(adc - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : adc;
}

int joyToServo(int adc) {
  float norm = constrain((float)(adc - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float sign = (norm >= 0) ? 1.0f : -1.0f;
  return SVC + (int)(sign * expoCurve(norm) * SOFT_RANGE);
}

// Cached joystick state — updated at ADC_INTERVAL, not every loop.
// This keeps the ADC blocking (~800us) from starving the PulseReader.
JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, SVC, SVC};
unsigned long lastAdcTime = 0;
const unsigned long ADC_INTERVAL = 10000UL;  // 10ms = 100Hz (plenty for joystick)

void updateJoystick(unsigned long now) {
  if ((now - lastAdcTime) < ADC_INTERVAL) return;
  lastAdcTime = now;

  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  cachedJoy.rawY = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  cachedJoy.rawX = analogRead(PIN_JOY_X);

  int throttle = joyToServo(joyDeadband(cachedJoy.rawY));
  int steer    = joyToServo(joyDeadband(cachedJoy.rawX));
  int offset   = steer - SVC;
  cachedJoy.left  = constrain(throttle + offset, SVMIN, SVMAX);
  cachedJoy.right = constrain(throttle - offset, SVMIN, SVMAX);
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════
//
// CH5 LOW  → RC only (Jason full control)
// CH5 MID  → RC priority (joystick if RC idle)
// CH5 HIGH → Joystick only (Malaki full control)

MixerOutput mixInputs(int rcL, int rcR, int ovr, int joyL, int joyR) {
  MixerOutput out;
  if (ovr < OVR_LO) {
    out.left = rcL;  out.right = rcR;
  } else if (ovr > OVR_HI) {
    out.left = joyL; out.right = joyR;
  } else {
    bool rcActive = (rcL != SVC) || (rcR != SVC);
    if (rcActive) { out.left = rcL;  out.right = rcR; }
    else          { out.left = joyL; out.right = joyR; }
  }
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [DYNAMICS] — Spin/reverse limiter, soft limit, inertia
// ═══════════════════════════════════════════════════════════════
//
// Processing order:
//   1. Spin limiter   — reduce power during counter-rotation
//   2. Reverse limiter — cap backward speed to REVERSE_LIMIT
//   3. Soft limit      — tanh saturation caps max deviation
//   4. Inertia         — asymmetric exponential (heavy feel)

float posL = 0, posR = 0;  // Smoothed servo offset from center

float fastTanh(float x) {
  float a = fabsf(x);
  return constrain(x / (1.0f + a + 0.28f * a * a), -1.0f, 1.0f);
}

int softLimit(float deviation) {
  if (fabsf(deviation) < 0.5f) return SVC;
  return SVC + (int)(SOFT_RANGE * fastTanh(deviation / SOFT_RANGE) + 0.5f);
}

// Spin limiter: graduated power reduction during counter-rotation.
// Full pivot (equal opposing) → SPIN_LIMIT. Gentle curve → ~100%.
void applySpinLimit(int &left, int &right) {
  float dL = (float)(left - SVC);
  float dR = (float)(right - SVC);
  if (dL * dR < 0) {
    float minM = min(fabsf(dL), fabsf(dR));
    float maxM = max(fabsf(dL), fabsf(dR));
    float spinRatio = (maxM > 1.0f) ? (minM / maxM) : 0.0f;
    float scale = 1.0f - spinRatio * (1.0f - SPIN_LIMIT);
    left  = SVC + (int)(dL * scale);
    right = SVC + (int)(dR * scale);
  }
}

// Reverse limiter: cap backward speed to REVERSE_LIMIT of forward max.
// "Backward" = servo value below SVC (1500). Forward is above.
void applyReverseLimit(int &left, int &right) {
  float maxReverse = SOFT_RANGE * REVERSE_LIMIT;
  if (left < SVC)  left  = max(left,  SVC - (int)maxReverse);
  if (right < SVC) right = max(right, SVC - (int)maxReverse);
}

// Inertia: asymmetric exponential filter.
// TAU_ACCEL when speeding up (shorter = faster response).
// TAU_DECEL when slowing down (longer = more coast).
void applyInertia(float targetL, float targetR, float dts) {
  bool accelL = fabsf(targetL) > fabsf(posL) + 1.0f;
  bool accelR = fabsf(targetR) > fabsf(posR) + 1.0f;
  float alphaL = dts / (dts + (accelL ? TAU_ACCEL : TAU_DECEL));
  float alphaR = dts / (dts + (accelR ? TAU_ACCEL : TAU_DECEL));
  posL += (targetL - posL) * alphaL;
  posR += (targetR - posR) * alphaR;
  if (fabsf(targetL) < 1.0f && fabsf(posL) < 3.0f) posL = 0;
  if (fabsf(targetR) < 1.0f && fabsf(posR) < 3.0f) posR = 0;
}


// ═══════════════════════════════════════════════════════════════
// [OUTPUT] — ESC servo PWM (non-blocking)
// ═══════════════════════════════════════════════════════════════

Servo escL, escR;
int outL = SVC, outR = SVC;

void outputInit() {
  escL.attach(PIN_ESC_L);
  escR.attach(PIN_ESC_R);
  escL.writeMicroseconds(SVC);
  escR.writeMicroseconds(SVC);
}

void outputWrite() {
  outL = constrain(SVC + (int)(posL + 0.5f), SVMIN, SVMAX);
  outR = constrain(SVC + (int)(posR + 0.5f), SVMIN, SVMAX);
  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);
}


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV telemetry
// ═══════════════════════════════════════════════════════════════
//
// Columns: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,CH1ok,CH2ok

unsigned long prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V3.3 ===");
    Serial.println("# CSV: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,CH1ok,CH2ok");
  }
}

void debugPrint(unsigned long now, const JoystickState &js) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;
  char buf[100];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
          rc1.pw, poll2.pw, rc4.pw, poll5.pw,
          js.rawY, js.rawX, outL, outR,
          (int)rc1.valid, (int)poll2.valid);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

unsigned long prevUs = 0;

void setup() {
  analogReadResolution(14);

  // Interrupt RC channels
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH4, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), isrCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), isrCh4, CHANGE);

  // Polled RC channels (non-blocking)
  poll2.init(PIN_RC_CH2);
  poll5.init(PIN_RC_CH5);

  outputInit();
  debugInit();
  prevUs = micros();
}

void loop() {
  unsigned long now = micros();
  float dts = (float)(now - prevUs) * 1e-6f;
  if (dts <= 0) return;
  prevUs = now;

  // 1. Read inputs
  poll2.poll();              // Non-blocking RC edge detection (~4us)
  poll5.poll();              // Non-blocking RC edge detection (~4us)
  rcSnapshot();              // Copy ISR data + failsafe (~5us)
  updateJoystick(now);       // ADC only every 10ms (~800us when active, 0 otherwise)

  // 2. Mix (override selects authority)
  MixerOutput mix = mixInputs(rcLeft(), rcRight(), rcOverride(),
                              cachedJoy.left, cachedJoy.right);

  // 3. Dynamics pipeline
  applySpinLimit(mix.left, mix.right);
  applyReverseLimit(mix.left, mix.right);
  int scL = softLimit((float)(mix.left  - SVC));
  int scR = softLimit((float)(mix.right - SVC));
  applyInertia((float)(scL - SVC), (float)(scR - SVC), dts);

  // 4. Output
  outputWrite();

  // 5. Debug
  debugPrint(now, cachedJoy);
}
