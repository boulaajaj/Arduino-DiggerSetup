// ═══════════════════════════════════════════════════════════════
// Digger Control V7.0 — GL10 FOC + Speed-Adaptive Steering
// ═══════════════════════════════════════════════════════════════
//
// Dual-input controller for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Hardware shift (2026-04-25): swapped from E10/E3665 to GL10 ESC
// + GL540L motor. The GL10's FOC handles motor acceleration
// compensation and smoothness internally — Arduino's job shrinks
// to: input mixing, override switch, soft limits, beeper alerts.
// Telemetry (X.BUS Read Register) is scaffolded in types.h but
// not polled — Serial2 is now used by S.BUS.
//
// Signal flow:
//   RC (S.BUS) ──► curvatureDrive ──┐
//                                   ├─► Mixer ─► Soft Limit
//   Joystick ──► curvatureDrive ───┘          ─► Inertia ─► PWM
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [DRIVE]      curvatureDrive — proven FRC algorithm (WPILib)
//   [RC]         S.BUS input — raw throttle + steering via Serial2
//   [JOYSTICK]   ADC input — deadband, expo curve
//   [MIXER]      Override switch — selects RC vs joystick
//   [DYNAMICS]   Soft power cap, inertia
//   [BEEPER]     Priority-driven audible alert (D8)
//   [OUTPUT]     ESC servo PWM
//   [DEBUG]      10 Hz serial CSV telemetry
//
// Pin map:
//   A0  ← Joystick Y (throttle)        [14-bit ADC]
//   A1  ← Joystick X (steering)        [14-bit ADC]
//   A4  (unused — Serial2 TX, S.BUS is RX-only)
//   A5  ← S.BUS RX (Serial2 via NPN inverter)
//   D8  → Beeper (active buzzer, direct GPIO)
//   D9  → Left ESC                      [Servo PWM]
//   D10 → Right ESC                     [Servo PWM]
//   D0/D1 → USB Serial (debug)
//
// S.BUS wiring (unchanged inverter circuit, now lands on A5):
//   R7FG S.BUS signal ──[1K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► A5 (Serial2 RX)

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
const uint8_t PIN_BEEPER = 8;   // Active buzzer (HIGH = on)
const uint8_t PIN_ESC_L  = 9;   // Left ESC PWM (50 Hz, 1000-2000 us)
const uint8_t PIN_ESC_R  = 10;  // Right ESC PWM

// S.BUS channel mapping (0-indexed, RC6GS V3 with no transmitter mix)
const uint8_t SBUS_CH_THR   = 1;  // CH2 = throttle (forward/back)
const uint8_t SBUS_CH_STEER = 0;  // CH1 = steering (left/right)
const uint8_t SBUS_CH_MODE  = 3;  // CH4 = gear (3-pos switch — reserved for V7.1)
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

// Expo curve blend weights — smooth low-end, no ESC deadband jump
const float EXPO_LINEAR = 0.4f;
const float EXPO_CUBIC  = 0.6f;

// Power range
const float SOFT_RANGE = 400.0f;  // Max servo offset from center (us)

// Inertia — asymmetric exponential filter
const float TAU_ACCEL = 0.3f;
const float TAU_DECEL = 0.5f;

// Curvature drive — pivot threshold
const float PIVOT_THRESHOLD = 0.10f;
const float PIVOT_SPEED_CAP = 0.45f;

// Reverse speed limit — percentage of forward max (straight-line only)
const float REVERSE_LIMIT = 0.35f;

// Reverse-beep trigger: how far below center the output must be
// before BEEP_REVERSE engages. 50 us = ~12.5% reverse — past deadband
// and clear of small drift.
const int   REVERSE_BEEP_US = 50;

// Debug
const uint32_t PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output


// ═══════════════════════════════════════════════════════════════
// [DRIVE] — curvatureDrive: proven FRC algorithm (WPILib)
// ═══════════════════════════════════════════════════════════════

WheelSpeeds curvatureDrive(float xSpeed, float zRotation) {
  xSpeed    = constrain(xSpeed, -1.0f, 1.0f);
  zRotation = constrain(zRotation, -1.0f, 1.0f);

  bool allowTurnInPlace = (fabsf(xSpeed) < PIVOT_THRESHOLD);

  float leftSpeed, rightSpeed;
  if (allowTurnInPlace) {
    float cappedRotation = constrain(zRotation, -PIVOT_SPEED_CAP, PIVOT_SPEED_CAP);
    leftSpeed  = xSpeed - cappedRotation;
    rightSpeed = xSpeed + cappedRotation;
  } else {
    leftSpeed  = xSpeed - fabsf(xSpeed) * zRotation;
    rightSpeed = xSpeed + fabsf(xSpeed) * zRotation;
  }

  float maxMag = max(fabsf(leftSpeed), fabsf(rightSpeed));
  if (maxMag > 1.0f) {
    leftSpeed  /= maxMag;
    rightSpeed /= maxMag;
  }
  return {leftSpeed, rightSpeed};
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
// [RC] — S.BUS input on Serial2 (A5 RX, NPN inverter)
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
  if (fabsf(zRotation) < 0.05f && xSpeed < -REVERSE_LIMIT) {
    xSpeed = -REVERSE_LIMIT;
  }
  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
  return wheelSpeedsToServo(ws);
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve
// ═══════════════════════════════════════════════════════════════

float expoCurve(float x) {
  float a = fabsf(x);
  return EXPO_LINEAR * a + EXPO_CUBIC * a * a * a;
}

int joyDeadband(int adc) {
  return (abs(adc - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : adc;
}

JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, 0.0f, 0.0f};
ServoOutput   cachedJoyOut = {SVC, SVC};
uint32_t      lastAdcTime = 0;
const uint32_t ADC_INTERVAL = 10000UL;  // 10ms = 100Hz

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
  cachedJoy.xSpeed    = signY * expoCurve(normY);
  cachedJoy.zRotation = -(signX * expoCurve(normX));  // joystick left = turn left

  float xSpeed = cachedJoy.xSpeed;
  float zRotation = cachedJoy.zRotation;
  if (fabsf(zRotation) < 0.05f && xSpeed < -REVERSE_LIMIT) {
    xSpeed = -REVERSE_LIMIT;
  }
  WheelSpeeds ws = curvatureDrive(xSpeed, zRotation);
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
    if (rcActive) { out.left = rcL; out.right = rcR; }
    else          { out.left = joyL; out.right = joyR; }
  }
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [DYNAMICS] — Soft limit, inertia
// ═══════════════════════════════════════════════════════════════

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
// [BEEPER] — Priority-driven audible alert on D8
// ═══════════════════════════════════════════════════════════════
//
// Active buzzer driven directly from GPIO (HIGH = sounding).
// Patterns are non-blocking, evaluated each loop pass against millis().
// Pattern priority is encoded in the BeepPattern enum value (higher
// = louder/more critical). Caller sets target each loop; if multiple
// conditions fire, the highest pattern wins. BATTERY/OVERTEMP are
// scaffolded but never set yet — telemetry isn't polled this build.

struct BeepStep { uint16_t onMs; uint16_t offMs; };
struct BeepProgram { uint8_t steps; uint16_t restMs; const BeepStep* seq; };

// Step sequences. Each entry is (on_ms, off_ms). After all steps,
// the program rests for restMs before repeating.
const BeepStep PAT_REVERSE[]  = {{1000, 0}};
const BeepStep PAT_BAT30[]    = {{200,  0}};
const BeepStep PAT_BAT20[]    = {{200,  200}, {200, 0}};
const BeepStep PAT_OVERTEMP[] = {{100,  100}, {100, 0}};

const BeepProgram BEEP_PROGRAMS[] = {
  {0, 0,    nullptr},        // BEEP_SILENT
  {1, 1000, PAT_REVERSE},    // BEEP_REVERSE: 1s on, 1s off — backup-alarm style
  {1, 9800, PAT_BAT30},      // BEEP_BATTERY_30: chirp every ~10s
  {2, 600,  PAT_BAT20},      // BEEP_BATTERY_20: double chirp every ~1s
  {2, 300,  PAT_OVERTEMP},   // BEEP_OVERTEMP: rapid double chirp
};

BeepPattern currentBeep = BEEP_SILENT;
uint8_t     beepStepIdx = 0;
uint32_t    beepStateStart = 0;
bool        beepOn = false;

void beeperInit() {
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
}

void setBeepPattern(BeepPattern p, uint32_t nowMs) {
  if (p == currentBeep) return;
  currentBeep = p;
  beepStepIdx = 0;
  beepStateStart = nowMs;
  beepOn = false;
  digitalWrite(PIN_BEEPER, LOW);
}

void beeperUpdate(uint32_t nowMs) {
  const BeepProgram& prog = BEEP_PROGRAMS[currentBeep];
  if (prog.steps == 0) {
    if (beepOn) { digitalWrite(PIN_BEEPER, LOW); beepOn = false; }
    return;
  }

  uint32_t elapsed = nowMs - beepStateStart;
  const BeepStep& step = prog.seq[beepStepIdx];

  if (beepOn) {
    if (elapsed >= step.onMs) {
      digitalWrite(PIN_BEEPER, LOW);
      beepOn = false;
      beepStateStart = nowMs;
    }
  } else {
    bool isLastStep = (beepStepIdx == prog.steps - 1);
    uint32_t gap = isLastStep ? prog.restMs : step.offMs;
    if (elapsed >= gap) {
      if (isLastStep) beepStepIdx = 0;
      else            beepStepIdx++;
      const BeepStep& next = prog.seq[beepStepIdx];
      if (next.onMs > 0) {
        digitalWrite(PIN_BEEPER, HIGH);
        beepOn = true;
      }
      beepStateStart = nowMs;
    }
  }
}

// Decide which pattern to play based on current state. Returns the
// highest-priority pattern that applies. Battery/temp are placeholders
// until X.BUS telemetry is wired back in.
BeepPattern selectBeepPattern(int outL, int outR) {
  bool reversing = (outL < SVC - REVERSE_BEEP_US) || (outR < SVC - REVERSE_BEEP_US);
  if (reversing) return BEEP_REVERSE;
  return BEEP_SILENT;
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
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Beep,FS,Lost

uint32_t prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V7.0 — GL10 FOC + S.BUS Serial2 + Beeper D8 ===");
    Serial.println("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Beep,FS,Lost");
  }
}

void debugPrint(uint32_t now) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  int rcT = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_THR])   : SVC;
  int rcS = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_STEER]) : SVC;
  int rc4 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_MODE])  : SVC;
  int rc5 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR])   : SVMIN;

  char buf[110];
  snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           rcT, rcS, rc4, rc5,
           cachedJoy.rawY, cachedJoy.rawX,
           outL, outR, (int)currentBeep,
           sbusData.failsafe, sbusData.lost_frame);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

uint32_t prevUs = 0;

void setup() {
  analogReadResolution(14);
  sbusRx.Begin();
  outputInit();
  beeperInit();
  debugInit();
  prevUs = micros();
}

void loop() {
  uint32_t now = micros();
  uint32_t nowMs = now / 1000UL;
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
  ServoOutput mix;
  if (!sbusValid) {
    mix.left = SVC;  mix.right = SVC;
    posL = 0;  posR = 0;
  } else {
    ServoOutput rc = rcDrive();
    mix = mixInputs(rc.left, rc.right, rcOverride(),
                    cachedJoyOut.left, cachedJoyOut.right);
  }

  // 3. Dynamics — soft limit + inertia
  int scL = softLimit((float)(mix.left  - SVC));
  int scR = softLimit((float)(mix.right - SVC));
  applyInertia((float)(scL - SVC), (float)(scR - SVC), dts);

  // 4. Output
  outputWrite();

  // 5. Beeper — evaluate against current output, then advance pattern
  setBeepPattern(selectBeepPattern(outL, outR), nowMs);
  beeperUpdate(nowMs);

  // 6. Debug
  debugPrint(now);
}
