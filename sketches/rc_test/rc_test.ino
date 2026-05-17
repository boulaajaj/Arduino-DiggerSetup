// ═══════════════════════════════════════════════════════════════
// Digger Control V7 — GL10 FOC + Speed-Adaptive Steering
// ═══════════════════════════════════════════════════════════════
//
// Dual-input controller for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Hardware shift (2026-04-25): swapped from E10/E3665 to GL10 ESC
// + GL540L motor. The GL10's FOC handles motor acceleration
// compensation and smoothness internally — Arduino's job shrinks
// to: input mixing, override switch, gear caps.
// V7.2 removed the Arduino-side inertia filter; the ESC's own
// Acceleration + Drag Force settings own command smoothing now.
// V7.6 removed the reverse-direction beeper entirely — audible
// alerts will return as a battery-aware system after the planned
// Nano R4 → UNO R4 migration (see GitHub issue tracker).
// Telemetry (X.BUS Read Register) is scaffolded in types.h but
// not polled — sbusUart (SCI0) is consumed by S.BUS.
//
// Signal flow:
//   RC (S.BUS) ──► curvatureDrive ──┐
//                                   ├─► Mixer ─► gear cap ─► PWM
//   Joystick ──► curvatureDrive ───┘
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [DRIVE]      curvatureDrive — symmetric add + desaturate, smoothstep blend into pivot
//   [RC]         S.BUS input — raw throttle + steering via sbusUart (SCI0)
//   [JOYSTICK]   ADC input — deadband, per-axis expo curve
//   [GEAR]       RC CH4 → Eco 40% / Normal 65% / Turbo 100% wheel-speed cap
//   [MIXER]      Override switch — selects RC vs joystick
//   [OUTPUT]     ESC servo PWM
//   [DEBUG]      10 Hz serial CSV telemetry
//
// Pin map:
//   A0  ← Joystick Y (throttle)        [14-bit ADC]
//   A1  ← Joystick X (steering)        [14-bit ADC]
//   A4  (unused — sbusUart TX on SCI0, S.BUS is RX-only)
//   A5  ← S.BUS RX (sbusUart on SCI0 via NPN inverter)
//   D8     (reserved — future battery-aware beeper, V8/UNO R4)
//   D9  → Left ESC                      [Servo PWM]
//   D10 → Right ESC                     [Servo PWM]
//   D0/D1 → Serial1 hardware UART (currently unused)
//   USB-C → USB CDC Serial (debug + firmware upload)
//
// S.BUS wiring (unchanged inverter circuit, now lands on A5):
//   R7FG S.BUS signal ──[1K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► A5 (sbusUart RX)

#include <Arduino.h>
#include <Servo.h>
#include "sbus.h"
#include "types.h"


// Second hardware UART on A4=TX(pin 18) / A5=RX(pin 19) via SCI0.
// S.BUS only needs RX; TX is unused. Named `sbusUart` (not Serial2)
// to avoid the macro collision with the core's pre-declared _UART2_.
UART sbusUart(18, 19);


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — All tunable constants
// ═══════════════════════════════════════════════════════════════

// Pins
const uint8_t PIN_JOY_Y  = A0;  // Throttle
const uint8_t PIN_JOY_X  = A1;  // Steering
const uint8_t PIN_ESC_L  = 9;   // Left ESC PWM (50 Hz, 1000-2000 us)
const uint8_t PIN_ESC_R  = 10;  // Right ESC PWM
// D8 reserved for the future battery-aware beeper (deferred to V8 / UNO R4)

// S.BUS channel mapping (0-indexed). Confirmed by live capture while
// the operator moved each control independently on the RC6GS V3:
// trigger → ch 1, wheel → ch 0.
const uint8_t SBUS_CH_THR   = 1;  // trigger → throttle (forward/back)
const uint8_t SBUS_CH_STEER = 0;  // wheel   → steering (left/right)
const uint8_t SBUS_CH_GEAR  = 3;  // CH4 = gear selector (3-pos switch)
const uint8_t SBUS_CH_OVR   = 4;  // CH5 = override switch (3-pos switch)

// S.BUS value range (raw 172-1811, center ~992)
const int SBUS_MIN = 172;
const int SBUS_MAX = 1811;

// Servo PWM range (matches GL10's standard 50 Hz, 1-2 ms input spec)
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

// Expo curve blend weights — output = LINEAR*|x| + CUBIC*|x|^3.
// Throttle keeps the smoother (more cubic) curve so launch feel is gentle.
// Steering uses a more linear curve so partial joystick deflection
// produces real turn authority — operator feedback was that the joystick
// pivot felt underpowered before reaching full lock.
const float EXPO_THROTTLE_LINEAR = 0.4f;
const float EXPO_THROTTLE_CUBIC  = 0.6f;
const float EXPO_STEER_LINEAR    = 0.7f;
const float EXPO_STEER_CUBIC     = 0.3f;

// Power range — full PWM authority (1000-2000 us = ±500 us from SVC)
const float SOFT_RANGE = 500.0f;  // Max servo offset from center (us)

// Gear scaling — RC CH4 selects speed cap. 3-position switch:
//   LOW  → 60% wheel speed cap   (training / tight spaces)
//   MID  → 70% wheel speed cap   (normal driving)
//   HIGH → 100% wheel speed cap  (full throttle authority)
// Failsafe: when S.BUS is invalid, gearScale stays at LOW for safety.
const float GEAR_LOW_SCALE  = 0.40f;  // Eco
const float GEAR_MID_SCALE  = 0.65f;  // Normal
const float GEAR_HIGH_SCALE = 1.00f;  // Turbo

// Eco gets +5pp authority on reverse and pivot caps so the operator can
// still maneuver in tight spaces. Forward stays at GEAR_LOW_SCALE (40%).
//   reverse:  0.625 × 0.40 = 0.25 effective (vs 0.20 unboosted)
//   pivot:    0.725 × 0.40 = 0.29 effective (vs 0.24 unboosted)
const float REVERSE_LIMIT_LOW   = 0.625f;
const float PIVOT_SPEED_CAP_LOW = 0.725f;

// Gear state — declared here so curvatureDrive() and rcDrive() can read
// it for the Eco-only conditional caps above. updateGear() in [GEAR]
// owns the writes.
float gearScale  = GEAR_LOW_SCALE;
Gear  currentGear = GEAR_LOW;

// Curvature drive — pivot/curvature blend band.
// |xSpeed| <= START: pure pivot (counter-rotate at PIVOT_SPEED_CAP)
// |xSpeed| >= END:   pure curvature (outer holds xSpeed, inner slows)
// Between: smoothstep blend so the operator doesn't feel a mode jump
// when transitioning from rolling-turn into pivot-in-place.
const float PIVOT_BLEND_START = 0.05f;
const float PIVOT_BLEND_END   = 0.30f;
const float PIVOT_SPEED_CAP   = 0.60f;  // pivot rotation cap (~60% wheel power)

// RC input gains — neutral baseline (1.0 = no scaling). Stick travel
// maps directly to curvatureDrive, which already handles inner-track
// slowdown and pivot/curvature blend. Earlier non-unity values were
// band-aids compensating for the flipped-ESC steering bug; with the
// root cause fixed, the gains return to neutral.
const float RC_THROTTLE_GAIN = 1.00f;
const float RC_STEERING_GAIN = 1.00f;

// Reverse speed limit — percentage of forward max (straight-line only).
// 1.0 = no reverse cap (full 100% reverse authority). The previous 0.35
// cap was for the older E10/E3665 hardware; GL10 + GL540L can take
// full reverse without trouble.
const float REVERSE_LIMIT = 0.50f;  // reverse capped at 50% of forward max

// Debug
const uint32_t PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output


// ═══════════════════════════════════════════════════════════════
// [DRIVE] — curvatureDrive: proven FRC algorithm (WPILib)
// ═══════════════════════════════════════════════════════════════

WheelSpeeds curvatureDrive(float xSpeed, float zRotation) {
  xSpeed    = constrain(xSpeed, -1.0f, 1.0f);
  zRotation = constrain(zRotation, -1.0f, 1.0f);

  // Pivot output: counter-rotate the tracks, capped to PIVOT_SPEED_CAP.
  // Eco gear uses a looser cap so the operator keeps usable pivot
  // authority even with the 40% forward-speed scaling.
  float pivotCap = (currentGear == GEAR_LOW) ? PIVOT_SPEED_CAP_LOW : PIVOT_SPEED_CAP;
  float cappedRotation = constrain(zRotation, -pivotCap, pivotCap);
  float pivotL = xSpeed - cappedRotation;
  float pivotR = xSpeed + cappedRotation;

  // Curvature output: symmetric add — whatever we subtract from the
  // inner track we add to the outer. Average wheel speed stays at
  // xSpeed through the turn, so the vehicle does not slow down.
  //
  //   inner = xSpeed * (1 - |z|)   ← slows
  //   outer = xSpeed * (1 + |z|)   ← speeds up by the same delta
  //
  // When the outer would exceed ±1.0 (high throttle + sharp turn),
  // desaturate by scaling BOTH wheels equally so the differential
  // ratio is preserved and only the average is trimmed. This is the
  // same pattern WPILib's ArcadeDrive uses with desaturateOutputs=true.
  float boost = 1.0f + fabsf(zRotation);
  float slow  = 1.0f - fabsf(zRotation);
  float curvL, curvR;
  if (zRotation > 0) {
    curvL = xSpeed * slow;   // turn LEFT: left is inner (subtract)
    curvR = xSpeed * boost;  // and outer gets the matching add
  } else {
    curvL = xSpeed * boost;
    curvR = xSpeed * slow;   // turn RIGHT: right is inner (subtract)
  }
  float peak = fmaxf(fabsf(curvL), fabsf(curvR));
  if (peak > 1.0f) {
    float k = 1.0f / peak;
    curvL *= k;
    curvR *= k;
  }

  // Smoothstep blend from pivot → curvature as |xSpeed| grows. Zero
  // slope at both endpoints means the operator never feels a mode
  // boundary — the rolling-turn smoothly becomes a pivot when they
  // back off the throttle, and vice versa.
  float t = (fabsf(xSpeed) - PIVOT_BLEND_START) / (PIVOT_BLEND_END - PIVOT_BLEND_START);
  t = constrain(t, 0.0f, 1.0f);
  t = t * t * (3.0f - 2.0f * t);

  return {
    pivotL * (1.0f - t) + curvL * t,
    pivotR * (1.0f - t) + curvR * t
  };
}

ServoOutput wheelSpeedsToServo(WheelSpeeds ws) {
  ServoOutput out;
  out.left  = SVC + (int)(ws.left  * SOFT_RANGE);
  out.right = SVC + (int)(ws.right * SOFT_RANGE);
  out.left  = constrain(out.left,  SVMIN, SVMAX);
  out.right = constrain(out.right, SVMIN, SVMAX);
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [RC] — S.BUS input on sbusUart / SCI0 (A5 RX, NPN inverter)
// ═══════════════════════════════════════════════════════════════

bfs::SbusRx sbusRx(&sbusUart);
bfs::SbusData sbusData;
bool sbusValid = false;
uint32_t sbusLastFrame = 0;
const uint32_t SBUS_TIMEOUT = 100000UL;  // 100ms

int sbusToServo(int raw) {
  return map(constrain(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
}

int rcDeadband(int pw) {
  return (abs(pw - SVC) <= RC_DEADBAND) ? SVC : pw;
}

int rcThrottle() { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_THR]))   : SVC; }
int rcSteering() { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_STEER])) : SVC; }
int rcOverride() { return sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR]) : SVMIN; }

ServoOutput rcDrive() {
  float xSpeed    = (float)(rcThrottle() - SVC) / SOFT_RANGE;
  float zRotation = (float)(rcSteering() - SVC) / SOFT_RANGE;
  // Apply tunable input gains, then clamp to the curvatureDrive domain.
  xSpeed    = constrain(xSpeed    * RC_THROTTLE_GAIN, -1.0f, 1.0f);
  zRotation = constrain(zRotation * RC_STEERING_GAIN, -1.0f, 1.0f);
  float revLimit = (currentGear == GEAR_LOW) ? REVERSE_LIMIT_LOW : REVERSE_LIMIT;
  if (xSpeed < -revLimit) xSpeed = -revLimit;
  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
  ws = applyGear(ws);
  return wheelSpeedsToServo(ws);
}


// ═══════════════════════════════════════════════════════════════
// [GEAR] — RC CH4 → speed cap (Eco 40% / Normal 65% / Turbo 100%)
// ═══════════════════════════════════════════════════════════════
//
// updateGear() is defined here (after [RC]) so it can read sbusValid /
// sbusData directly. The gearScale and currentGear globals it writes
// are declared up in [CONFIG] so the drive functions and curvatureDrive
// can read them for the Eco-only conditional caps.

void updateGear() {
  if (!sbusValid) {
    gearScale = GEAR_LOW_SCALE;
    currentGear = GEAR_LOW;
    return;
  }
  int rc4 = sbusToServo(sbusData.ch[SBUS_CH_GEAR]);
  if (rc4 < OVR_LO) {
    gearScale = GEAR_LOW_SCALE;
    currentGear = GEAR_LOW;
  } else if (rc4 > OVR_HI) {
    gearScale = GEAR_HIGH_SCALE;
    currentGear = GEAR_HIGH;
  } else {
    gearScale = GEAR_MID_SCALE;
    currentGear = GEAR_MID;
  }
}

WheelSpeeds applyGear(WheelSpeeds ws) {
  return {ws.left * gearScale, ws.right * gearScale};
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve
// ═══════════════════════════════════════════════════════════════

float expoCurve(float x, float linearW, float cubicW) {
  float a = fabsf(x);
  return linearW * a + cubicW * a * a * a;
}

int joyDeadband(int adc) {
  return (abs(adc - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : adc;
}

JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, 0.0f, 0.0f};
ServoOutput   cachedJoyOut = {SVC, SVC};
uint32_t      lastAdcTime = 0;
const uint32_t ADC_INTERVAL = 10000UL;  // 10 ms = 100 Hz

void updateJoystick(uint32_t now) {
  if ((now - lastAdcTime) < ADC_INTERVAL) return;
  lastAdcTime = now;

  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  cachedJoy.rawY = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  cachedJoy.rawX = analogRead(PIN_JOY_X);

  float normY = constrain((float)(joyDeadband(cachedJoy.rawY) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float normX = constrain((float)(joyDeadband(cachedJoy.rawX) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float signY = (normY >= 0) ? 1.0f : -1.0f;
  float signX = (normX >= 0) ? 1.0f : -1.0f;
  cachedJoy.xSpeed    = signY * expoCurve(normY, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC);
  cachedJoy.zRotation = signX * expoCurve(normX, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC);

  float xSpeed = cachedJoy.xSpeed;
  float zRotation = cachedJoy.zRotation;
  float revLimit = (currentGear == GEAR_LOW) ? REVERSE_LIMIT_LOW : REVERSE_LIMIT;
  if (xSpeed < -revLimit) xSpeed = -revLimit;
  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
  ws = applyGear(ws);
  cachedJoyOut = wheelSpeedsToServo(ws);
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════

ServoOutput mixInputs(int rcL, int rcR, int ovr, int joyL, int joyR) {
  ServoOutput out;
  if (ovr < OVR_LO) {
    out.left = rcL;  out.right = rcR;
  } else if (ovr > OVR_HI) {
    out.left  = SVC + ((rcL - SVC) + (joyL - SVC)) / 2;
    out.right = SVC + ((rcR - SVC) + (joyR - SVC)) / 2;
  } else {
    bool rcActive = (rcL != SVC) || (rcR != SVC);
    if (rcActive) {
      out.left = rcL;
      out.right = rcR;
    } else {
      out.left = joyL;
      out.right = joyR;
    }
  }
  return out;
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

void outputWrite(int left, int right) {
  outL = constrain(left,  SVMIN, SVMAX);
  outR = constrain(right, SVMIN, SVMAX);
  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);
}


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV telemetry
// ═══════════════════════════════════════════════════════════════
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost

uint32_t prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V7.6 — GL10 FOC + S.BUS sbusUart + Gear (beeper removed) ===");
    Serial.println("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost");
  }
}

void debugPrint(uint32_t now) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  int rcT = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_THR])   : SVC;
  int rcS = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_STEER]) : SVC;
  int rc4 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_GEAR])  : SVC;
  int rc5 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR])   : SVMIN;

  char buf[120];
  snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           rcT, rcS, rc4, rc5,
           cachedJoy.rawY, cachedJoy.rawX,
           outL, outR, (int)currentGear,
           sbusData.failsafe, sbusData.lost_frame);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

void setup() {
  analogReadResolution(14);
  sbusRx.Begin();
  outputInit();
  debugInit();
}

void loop() {
  uint32_t now = micros();

  // 1. Read inputs
  if (sbusRx.Read()) {
    sbusData = sbusRx.data();
    sbusLastFrame = now;
    sbusValid = !sbusData.failsafe;
  }
  if ((now - sbusLastFrame) > SBUS_TIMEOUT) sbusValid = false;
  updateGear();
  updateJoystick(now);

  // 2. RC lockout — no RC signal means immediate stop
  ServoOutput mix;
  if (!sbusValid) {
    mix.left = SVC;  mix.right = SVC;
  } else {
    ServoOutput rc = rcDrive();
    mix = mixInputs(rc.left, rc.right, rcOverride(),
                    cachedJoyOut.left, cachedJoyOut.right);
  }

  // 3. Output — GL10 FOC handles command smoothing internally via its
  // Acceleration + Drag Force settings, so the Arduino sends the mixed
  // command straight through.
  outputWrite(mix.left, mix.right);

  // 4. Debug
  debugPrint(now);
}
