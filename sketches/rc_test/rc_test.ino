// ═══════════════════════════════════════════════════════════════
// Digger Control V5.0 — Curvature Drive
// ═══════════════════════════════════════════════════════════════
//
// Dual-input controller for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Both RC and joystick send raw throttle + steering (no pre-mix).
// Arduino applies curvatureDrive() — at speed, turning only slows
// the inner track (outer holds speed). At standstill, full pivot.
//
// Signal flow:
//   RC (S.BUS) ──► curvatureDrive ──┐
//                                   ├─► Mixer ─► Soft Limit
//   Joystick ──► curvatureDrive ───┘          ─► Inertia ─► ESC
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [DRIVE]      curvatureDrive — proven FRC algorithm (WPILib)
//   [RC]         S.BUS input — raw throttle + steering via Serial1
//   [JOYSTICK]   ADC input — deadband, expo curve
//   [MIXER]      Override switch — selects RC vs joystick
//   [DYNAMICS]   Soft power cap, inertia
//   [OUTPUT]     ESC servo PWM
//   [DEBUG]      10 Hz serial CSV telemetry
//
// Pin map:
//   D0  ← S.BUS (all RC channels, Serial1 RX via NPN inverter)
//   A0  ← Joystick Y (throttle)     [14-bit ADC]
//   A1  ← Joystick X (steering)     [14-bit ADC]
//   D9  → Left ESC                   [Servo PWM]
//   D10 → Right ESC                  [Servo PWM]
//
// S.BUS wiring:
//   R7FG S.BUS signal ──[1K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► D0 (Serial1 RX)

#include <Servo.h>
#include "sbus.h"
#include "types.h"


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — All tunable constants
// ═══════════════════════════════════════════════════════════════

// Pins
const uint8_t PIN_JOY_Y  = A0;  // Throttle
const uint8_t PIN_JOY_X  = A1;  // Steering
const uint8_t PIN_ESC_L  = 9;   // Left ESC
const uint8_t PIN_ESC_R  = 10;  // Right ESC

// S.BUS channel mapping (0-indexed, RC6GS V3 with no transmitter mix)
const uint8_t SBUS_CH_THR   = 1;  // CH2 = throttle (forward/back)
const uint8_t SBUS_CH_STEER = 0;  // CH1 = steering (left/right)
const uint8_t SBUS_CH_MODE  = 3;  // CH4 = control mode (3-pos switch)
const uint8_t SBUS_CH_OVR   = 4;  // CH5 = override switch (3-pos switch)

// S.BUS value range (raw 172-1811, center ~992)
const int SBUS_MIN = 172;
const int SBUS_MAX = 1811;

// Servo PWM range
const int SVC   = 1500;  // Center (neutral)
const int SVMIN = 1000;  // Full reverse
const int SVMAX = 2000;  // Full forward

// ADC
const int ADC_CENTER = 8192;  // 14-bit midpoint

// Deadbands
const int RC_DEADBAND  = 50;   // RC mapped pulse (us)
const int JOY_DEADBAND = 480;  // Joystick ADC (~5.9% of travel)

// Override switch thresholds (mapped to PWM-equivalent)
const int OVR_LO = 1400;  // Below → RC only
const int OVR_HI = 1600;  // Above → 50/50 blend (RC + joystick)

// Expo curve blend weights — smooth low-end, no ESC deadband jump
const float EXPO_LINEAR = 0.4f;   // Linear component (low-end response)
const float EXPO_CUBIC  = 0.6f;   // Cubic component (high-end precision)

// Power range
const float SOFT_RANGE = 400.0f;  // Max servo offset from center (us)

// Inertia — asymmetric exponential filter
// Tuned for ~50 lb machine, 4-5 ft long. Smooth but nimble.
const float TAU_ACCEL = 0.3f;  // Accel (s) — 63% in 0.3s, 95% in ~0.9s
const float TAU_DECEL = 0.5f;  // Decel/coast (s) — 63% in 0.5s, 95% in ~1.5s

// Curvature drive — pivot threshold
const float PIVOT_THRESHOLD = 0.10f;  // Below 10% throttle: allow pivot turns
const float PIVOT_SPEED_CAP = 0.45f;  // Max track speed during pivot (45%)

// Reverse speed limit — percentage of forward max (straight-line only)
const float REVERSE_LIMIT = 0.35f;  // 35% max reverse

// Debug
const unsigned long PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output


// ═══════════════════════════════════════════════════════════════
// [DRIVE] — curvatureDrive: proven FRC algorithm (WPILib)
// ═══════════════════════════════════════════════════════════════
//
// Inputs:  xSpeed (-1 to 1, throttle), zRotation (-1 to 1, steering)
// Returns: left/right wheel speeds (-1 to 1)
//
// At speed: rotation is proportional to |throttle|. The outer track
//   NEVER exceeds throttle — mathematically impossible. Inner track
//   only slows down.
// At standstill: allowTurnInPlace enables arcade-style counter-rotation
//   for pivot turns.
// Desaturation: if either wheel exceeds ±1, both scale down
//   proportionally — preserves turn ratio without clipping.
//
// Reference: WPILib DifferentialDrive.curvatureDriveIK()
//   https://github.com/wpilibsuite/allwpilib

struct WheelSpeeds {
  float left;
  float right;
};

WheelSpeeds curvatureDrive(float xSpeed, float zRotation) {
  xSpeed    = constrain(xSpeed, -1.0f, 1.0f);
  zRotation = constrain(zRotation, -1.0f, 1.0f);

  bool allowTurnInPlace = (fabsf(xSpeed) < PIVOT_THRESHOLD);

  float leftSpeed, rightSpeed;

  if (allowTurnInPlace) {
    // Pivot mode: arcade-style, capped for safety
    float cappedRotation = constrain(zRotation, -PIVOT_SPEED_CAP, PIVOT_SPEED_CAP);
    leftSpeed  = xSpeed - cappedRotation;
    rightSpeed = xSpeed + cappedRotation;
  } else {
    // Curvature mode: rotation scaled by |speed|
    // Outer track = xSpeed, inner track = xSpeed - |xSpeed|*rotation
    // Outer NEVER exceeds xSpeed. Inner only slows.
    leftSpeed  = xSpeed - fabsf(xSpeed) * zRotation;
    rightSpeed = xSpeed + fabsf(xSpeed) * zRotation;
  }

  // Desaturate: scale both down proportionally if either exceeds ±1
  float maxMag = max(fabsf(leftSpeed), fabsf(rightSpeed));
  if (maxMag > 1.0f) {
    leftSpeed  /= maxMag;
    rightSpeed /= maxMag;
  }

  return {leftSpeed, rightSpeed};
}

// Convert normalized wheel speeds (-1..1) to servo microseconds
MixerOutput wheelSpeedsToServo(WheelSpeeds ws) {
  MixerOutput out;
  out.left  = SVC + (int)(ws.left  * SOFT_RANGE);
  out.right = SVC + (int)(ws.right * SOFT_RANGE);
  out.left  = constrain(out.left,  SVMIN, SVMAX);
  out.right = constrain(out.right, SVMIN, SVMAX);
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [RC] — S.BUS input: raw throttle + steering on D0 via Serial1
// ═══════════════════════════════════════════════════════════════
//
// RC6GS V3 sends raw throttle on CH2 and raw steering on CH1
// (no transmitter tank mix — Arduino handles all mixing).
// S.BUS delivers all 16 channels + failsafe + frame-lost flags
// over a single wire at 143Hz via NPN inverter on D0.

bfs::SbusRx sbusRx(&Serial1);
bfs::SbusData sbusData;
bool sbusValid = false;
unsigned long sbusLastFrame = 0;
const unsigned long SBUS_TIMEOUT = 100000UL;  // 100ms

int sbusToServo(int raw) {
  return map(constrain(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
}

int rcDeadband(int pw) {
  return (abs(pw - SVC) <= RC_DEADBAND) ? SVC : pw;
}

int rcThrottle() { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_THR]))   : SVC; }
int rcSteering() { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_STEER])) : SVC; }
int rcOverride() { return sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR]) : SVMIN; }

// RC → curvatureDrive → servo values
MixerOutput rcDrive() {
  float xSpeed    = (float)(rcThrottle() - SVC) / SOFT_RANGE;
  float zRotation = (float)(rcSteering() - SVC) / SOFT_RANGE;

  // Apply reverse limit for straight-line reverse
  if (fabsf(zRotation) < 0.05f && xSpeed < -REVERSE_LIMIT) {
    xSpeed = -REVERSE_LIMIT;
  }

  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
  return wheelSpeedsToServo(ws);
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve → curvatureDrive
// ═══════════════════════════════════════════════════════════════
//
// 14-bit ADC. Double-read clears mux crosstalk between channels.
// Expo gives fine control at low stick, full range at high stick.

float expoCurve(float x) {
  float a = fabsf(x);
  return EXPO_LINEAR * a + EXPO_CUBIC * a * a * a;
}

int joyDeadband(int adc) {
  return (abs(adc - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : adc;
}

// Cached joystick state — updated at ADC_INTERVAL, not every loop.
JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, SVC, SVC};
unsigned long lastAdcTime = 0;
const unsigned long ADC_INTERVAL = 10000UL;  // 10ms = 100Hz

void updateJoystick(unsigned long now) {
  if ((now - lastAdcTime) < ADC_INTERVAL) return;
  lastAdcTime = now;

  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  cachedJoy.rawY = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  cachedJoy.rawX = analogRead(PIN_JOY_X);

  // Normalize joystick to -1..1 with expo curve
  float normY = constrain((float)(joyDeadband(cachedJoy.rawY) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float normX = constrain((float)(joyDeadband(cachedJoy.rawX) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float signY = (normY >= 0) ? 1.0f : -1.0f;
  float signX = (normX >= 0) ? 1.0f : -1.0f;
  float xSpeed    = signY * expoCurve(normY);
  float zRotation = -(signX * expoCurve(normX));  // Invert: joystick left = turn left

  // Apply reverse limit for straight-line reverse
  if (fabsf(zRotation) < 0.05f && xSpeed < -REVERSE_LIMIT) {
    xSpeed = -REVERSE_LIMIT;
  }

  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
  MixerOutput out = wheelSpeedsToServo(ws);
  cachedJoy.left  = out.left;
  cachedJoy.right = out.right;
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════
//
// RC ALWAYS has control. The switch controls joystick authority:
//   CH5 LOW  → RC only (joystick disabled)
//   CH5 MID  → Both active, RC overrides joystick when RC non-neutral
//   CH5 HIGH → 50/50 blend (RC + joystick averaged)

MixerOutput mixInputs(int rcL, int rcR, int ovr, int joyL, int joyR) {
  MixerOutput out;
  if (ovr < OVR_LO) {
    out.left = rcL;  out.right = rcR;
  } else if (ovr > OVR_HI) {
    out.left  = SVC + ((rcL - SVC) + (joyL - SVC)) / 2;
    out.right = SVC + ((rcR - SVC) + (joyR - SVC)) / 2;
  } else {
    bool rcActive = (rcL != SVC) || (rcR != SVC);
    if (rcActive) { out.left = rcL;  out.right = rcR; }
    else          { out.left = joyL; out.right = joyR; }
  }
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [DYNAMICS] — Soft limit, inertia
// ═══════════════════════════════════════════════════════════════
//
// curvatureDrive handles all steering dynamics. This section only
// applies output saturation and inertia smoothing.

float posL = 0, posR = 0;

float fastTanh(float x) {
  float a = fabsf(x);
  return constrain(x / (1.0f + a + 0.28f * a * a), -1.0f, 1.0f);
}

int softLimit(float deviation) {
  float val = SOFT_RANGE * fastTanh(deviation / SOFT_RANGE);
  return SVC + (int)(val >= 0.0f ? val + 0.5f : val - 0.5f);
}

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
  outL = constrain(SVC + (int)(posL >= 0.0f ? posL + 0.5f : posL - 0.5f), SVMIN, SVMAX);
  outR = constrain(SVC + (int)(posR >= 0.0f ? posR + 0.5f : posR - 0.5f), SVMIN, SVMAX);
  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);
}


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV telemetry
// ═══════════════════════════════════════════════════════════════
//
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,FS,Lost

unsigned long prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V5.0 — Curvature Drive ===");
    Serial.println("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,FS,Lost");
  }
}

void debugPrint(unsigned long now, const JoystickState &js) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  int rcT = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_THR])   : SVC;
  int rcS = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_STEER]) : SVC;
  int rc4 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_MODE])  : SVC;
  int rc5 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR])   : SVMIN;

  char buf[100];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
          rcT, rcS, rc4, rc5,
          js.rawY, js.rawX, outL, outR,
          sbusData.failsafe, sbusData.lost_frame);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

unsigned long prevUs = 0;

void setup() {
  analogReadResolution(14);
  sbusRx.Begin();
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
  if (sbusRx.Read()) {
    sbusData = sbusRx.data();
    sbusLastFrame = now;
    sbusValid = !sbusData.failsafe;
  }
  if ((now - sbusLastFrame) > SBUS_TIMEOUT) sbusValid = false;
  updateJoystick(now);

  // 2. RC lockout — no RC signal means immediate stop
  MixerOutput mix;
  if (!sbusValid) {
    mix.left = SVC;  mix.right = SVC;
    posL = 0;  posR = 0;
  } else {
    MixerOutput rc = rcDrive();
    mix = mixInputs(rc.left, rc.right, rcOverride(),
                    cachedJoy.left, cachedJoy.right);
  }

  // 3. Dynamics — soft limit + inertia only (curvatureDrive handles steering)
  int scL = softLimit((float)(mix.left  - SVC));
  int scR = softLimit((float)(mix.right - SVC));
  applyInertia((float)(scL - SVC), (float)(scR - SVC), dts);

  // 4. Output
  outputWrite();

  // 5. Debug
  debugPrint(now, cachedJoy);
}
