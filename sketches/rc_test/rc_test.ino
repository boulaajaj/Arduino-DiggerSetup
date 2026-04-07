// ═══════════════════════════════════════════════════════════════
// Digger Control V4.2 — S.BUS + ESC Telemetry
// ═══════════════════════════════════════════════════════════════
//
// Dual-input tank mixer for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Signal flow:
//   RC (S.BUS) ──┐
//                ├─► Mixer ─► Spin Limiter ─► Reverse Limiter
//   Joystick ───┘         ─► Soft Limit ─► Inertia ─► ESC
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [RC]         S.BUS input — all channels on one wire via Serial1
//   [JOYSTICK]   ADC input — deadband, expo curve, tank mix
//   [MIXER]      Override switch — selects RC vs joystick
//   [DYNAMICS]   Spin/reverse limiter, soft power cap, inertia
//   [OUTPUT]     ESC servo PWM
//   [DEBUG]      10 Hz serial CSV telemetry
//
// Pin map:
//   D0  ← S.BUS (all RC channels, Serial1 RX via NPN inverter)
//   D2  ← ESC telemetry RX (SoftwareSerial via NPN inverter)
//   D3  → ESC telemetry TX (SoftwareSerial via 1K to bus)
//   A0  ← Joystick Y (throttle)     [14-bit ADC]
//   A1  ← Joystick X (steering)     [14-bit ADC]
//   D9  → Left ESC                   [Servo PWM]
//   D10 → Right ESC                  [Servo PWM]
//
// S.BUS wiring:
//   R7FG CH7 signal ──[1K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► D0 (Serial1 RX)

#include <Servo.h>
#include <SoftwareSerial.h>
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

// S.BUS channel mapping (0-indexed)
const uint8_t SBUS_CH_LEFT  = 0;  // CH1 = left track (pre-mixed by transmitter)
const uint8_t SBUS_CH_RIGHT = 1;  // CH2 = right track (pre-mixed by transmitter)
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

// Expo curve: 30% linear + 70% cubic — smooth low-end, no ESC deadband jump

// Power range
const float SOFT_RANGE = 400.0f;  // Max servo offset from center (us)

// Inertia — asymmetric exponential filter
// Tuned for ~50 lb machine, 4-5 ft long. Smooth but nimble.
const float TAU_ACCEL = 0.3f;  // Accel (s) — 63% in 0.3s, 95% in ~0.9s
const float TAU_DECEL = 0.5f;  // Decel/coast (s) — 63% in 0.5s, 95% in ~1.5s

// Spin turn safety — power cap during counter-rotation
const float SPIN_LIMIT = 0.45f;  // 45% at full pivot (35% still too weak for joystick on ground)

// Reverse speed limit — percentage of forward max
const float REVERSE_LIMIT = 0.35f;  // 35% max reverse

// Steering decay — reduce steering sensitivity at higher speeds
const float STEER_DECAY = 0.5f;  // At full speed, steering is 50% of max

// Debug
const unsigned long PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output


// ═══════════════════════════════════════════════════════════════
// [RC] — S.BUS input: all channels on D0 via Serial1
// ═══════════════════════════════════════════════════════════════
//
// S.BUS delivers all 16 channels + failsafe + frame-lost flags
// over a single wire at 143Hz. The bolderflight/sbus library
// handles Serial1 configuration (100000 baud, 8E2, inverted via
// external NPN transistor on D0).
//
// Channel values are raw S.BUS (172-1811). We map them to
// PWM-equivalent (1000-2000us) so the rest of the pipeline
// (deadband, mixer, dynamics) works unchanged.

bfs::SbusRx sbusRx(&Serial1);
bfs::SbusData sbusData;
bool sbusValid = false;           // True when receiving non-failsafe frames
unsigned long sbusLastFrame = 0;  // Timestamp of last decoded frame
const unsigned long SBUS_TIMEOUT = 100000UL;  // 100ms — force invalid if no frames

// Map S.BUS value (172-1811) to servo PWM (1000-2000)
int sbusToServo(int raw) {
  return map(constrain(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
}

int rcDeadband(int pw) {
  return (abs(pw - SVC) <= RC_DEADBAND) ? SVC : pw;
}

// Safe accessors — return neutral (or RC-only mode) if S.BUS lost
int rcLeft()     { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_LEFT]))  : SVC; }
int rcRight()    { return sbusValid ? rcDeadband(sbusToServo(sbusData.ch[SBUS_CH_RIGHT])) : SVC; }
int rcOverride() { return sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR]) : SVMIN; }  // Default RC-only if lost


// ═══════════════════════════════════════════════════════════════
// [TELEMETRY] — ESC telemetry via Hobbywing V4 polling on D2/D3
// ═══════════════════════════════════════════════════════════════
//
// Polls both ESCs on a shared bus using SoftwareSerial at 19200 baud.
// Alternates between ESC 0 and ESC 1 addressed requests.
// Each ESC responds with 5-byte telemetry packets (sent 3x each).
// RX on D2 (via NPN inverter), TX on D3 (via 1K resistor to bus).

const uint8_t TELEM_RX = 2;
const uint8_t TELEM_TX = 3;

SoftwareSerial telemSerial(TELEM_RX, TELEM_TX);

// Hobbywing V4 poll commands (last byte = sum of first 5)
const uint8_t HW_POLL_ESC0[] = {0x9B, 0x03, 0x00, 0x00, 0x00, 0x9E};
const uint8_t HW_POLL_ESC1[] = {0x9B, 0x03, 0x00, 0x00, 0x01, 0x9F};

// Telemetry state
unsigned long lastTelemPoll = 0;
const unsigned long TELEM_INTERVAL = 50000UL;  // 50ms = 20Hz per ESC (alternating)
bool pollEsc1Next = false;

// Raw telemetry values (best-effort decode from 5-byte packets)
// We extract a 16-bit value from bytes [2:3] of each response as RPM-like data
int telemRawL = 0;  // ESC 0 (left) latest raw value
int telemRawR = 0;  // ESC 1 (right) latest raw value
int telemBytesL = 0, telemBytesR = 0;  // Bytes received per poll cycle

void telemInit() {
  telemSerial.begin(19200);
}

void telemPoll(unsigned long now) {
  if ((now - lastTelemPoll) < TELEM_INTERVAL) return;
  lastTelemPoll = now;

  // Read any pending response bytes from previous poll
  uint8_t buf[32];
  int count = 0;
  while (telemSerial.available() && count < 32) {
    buf[count++] = telemSerial.read();
  }

  // Try to extract a value from the response (5-byte packets, look for repeated patterns)
  if (count >= 5) {
    // Use bytes [1] and [2] as a 16-bit raw telemetry value
    // These appear to change with motor state
    int rawVal = ((int)buf[1] << 8) | buf[2];
    if (pollEsc1Next) {
      telemRawL = rawVal;  // Previous poll was ESC 0
      telemBytesL = count;
    } else {
      telemRawR = rawVal;  // Previous poll was ESC 1
      telemBytesR = count;
    }
  }

  // Send next poll (alternate between ESC 0 and ESC 1)
  if (pollEsc1Next) {
    telemSerial.write(HW_POLL_ESC1, 6);
  } else {
    telemSerial.write(HW_POLL_ESC0, 6);
  }
  pollEsc1Next = !pollEsc1Next;
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve, tank mix
// ═══════════════════════════════════════════════════════════════
//
// 14-bit ADC. Double-read clears mux crosstalk between channels.
// Expo gives fine control at low stick, full range at high stick.
// Tank mix: throttle +/- steering → left/right.

float expoCurve(float x) {
  float a = fabsf(x);
  return 0.3f * a + 0.7f * a * a * a;  // Blend: 10%→3.07%, 50%→23.75%, 100%→100%
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

  int throttle = joyToServo(joyDeadband(cachedJoy.rawY));
  int steer    = joyToServo(joyDeadband(cachedJoy.rawX));
  int offset   = -(steer - SVC);  // Invert: joystick left = turn left

  // Steering decay: reduce steering as speed increases
  float speed = fabsf((float)(throttle - SVC)) / SOFT_RANGE;  // 0-1
  float decay = 1.0f - speed * STEER_DECAY;
  offset = (int)((float)offset * decay);

  cachedJoy.left  = constrain(throttle + offset, SVMIN, SVMAX);
  cachedJoy.right = constrain(throttle - offset, SVMIN, SVMAX);
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
    // Mode 1: RC only
    out.left = rcL;  out.right = rcR;
  } else if (ovr > OVR_HI) {
    // Mode 3: 50/50 blend — both contribute equally
    out.left  = SVC + ((rcL - SVC) + (joyL - SVC)) / 2;
    out.right = SVC + ((rcR - SVC) + (joyR - SVC)) / 2;
  } else {
    // Mode 2: RC priority — joystick active only when RC sticks are centered
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
//   1. Spin limiter    — reduce power during counter-rotation
//   2. Reverse limiter — cap backward speed to REVERSE_LIMIT
//   3. Pivot balance   — force equal magnitude during counter-rotation
//   4. Soft limit      — tanh saturation caps max deviation
//   5. Inertia         — asymmetric exponential (heavy feel)

float posL = 0, posR = 0;  // Smoothed servo offset from center

float fastTanh(float x) {
  float a = fabsf(x);
  return constrain(x / (1.0f + a + 0.28f * a * a), -1.0f, 1.0f);
}

int softLimit(float deviation) {
  if (fabsf(deviation) < 0.5f) return SVC;
  float val = SOFT_RANGE * fastTanh(deviation / SOFT_RANGE);
  return SVC + (int)(val >= 0.0f ? val + 0.5f : val - 0.5f);
}

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

// Reverse limiter: caps backward speed during STRAIGHT-LINE reverse only.
// During counter-rotation (turning), this is intentionally skipped because:
// 1. The spin limiter already caps pivot speed (SPIN_LIMIT)
// 2. The pivot balance forces equal magnitude for clean 360 turns
// 3. Applying reverse limit during turns would break track symmetry
// Note: reverse deviation during pivots may exceed REVERSE_LIMIT — this is by design.
void applyReverseLimit(int &left, int &right) {
  float dL = (float)(left - SVC);
  float dR = (float)(right - SVC);
  if (dL * dR < 0) return;  // Counter-rotating (turning) — skip, spin limiter handles it
  float maxReverse = SOFT_RANGE * REVERSE_LIMIT;
  if (left < SVC)  left  = max(left,  SVC - (int)maxReverse);
  if (right < SVC) right = max(right, SVC - (int)maxReverse);
}

// Pivot balance: when counter-rotating, force both tracks to equal magnitude.
// The faster track's magnitude is applied to both. Spin limiter already
// capped the max speed — this just ensures symmetry for a clean 360.
void applyPivotBalance(int &left, int &right) {
  float dL = (float)(left - SVC);
  float dR = (float)(right - SVC);
  if (dL * dR >= 0) return;  // Not counter-rotating — skip
  float magL = fabsf(dL);
  float magR = fabsf(dR);
  float mag = max(magL, magR);
  left  = SVC + (int)((dL >= 0 ? 1.0f : -1.0f) * mag);
  right = SVC + (int)((dR >= 0 ? 1.0f : -1.0f) * mag);
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
// Columns: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,FS,Lost,EscL,EscR

unsigned long prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V4.2 — S.BUS + Telemetry ===");
    Serial.println("# CSV: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,FS,Lost,EscL,EscR");
  }
}

void debugPrint(unsigned long now, const JoystickState &js) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  int rc1 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_LEFT])  : SVC;
  int rc2 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_RIGHT]) : SVC;
  int rc4 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_MODE])  : SVC;
  int rc5 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR])   : SVMIN;

  char buf[120];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
          rc1, rc2, rc4, rc5,
          js.rawY, js.rawX, outL, outR,
          sbusData.failsafe, sbusData.lost_frame,
          telemRawL, telemRawR);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

unsigned long prevUs = 0;

void setup() {
  analogReadResolution(14);
  sbusRx.Begin();  // Configures Serial1 at 100000, 8E2
  telemInit();     // SoftwareSerial at 19200 on D2/D3
  outputInit();
  debugInit();
  prevUs = micros();
}

void loop() {
  unsigned long now = micros();
  float dts = (float)(now - prevUs) * 1e-6f;
  if (dts <= 0) return;
  prevUs = now;

  // 1. Read inputs + poll ESC telemetry
  telemPoll(now);
  if (sbusRx.Read()) {
    sbusData = sbusRx.data();
    sbusLastFrame = now;
    sbusValid = !sbusData.failsafe;
  }
  // Force invalid if no frames for 100ms (wire unplugged / Serial1 stalled)
  if ((now - sbusLastFrame) > SBUS_TIMEOUT) sbusValid = false;
  updateJoystick(now);

  // 2. RC lockout — no RC signal means immediate stop
  MixerOutput mix;
  if (!sbusValid) {
    mix.left = SVC;  mix.right = SVC;
    posL = 0;  posR = 0;  // Reset inertia — stop immediately, no coasting
  } else {
    mix = mixInputs(rcLeft(), rcRight(), rcOverride(),
                    cachedJoy.left, cachedJoy.right);
  }

  // 3. Dynamics pipeline
  applySpinLimit(mix.left, mix.right);
  applyReverseLimit(mix.left, mix.right);
  applyPivotBalance(mix.left, mix.right);
  int scL = softLimit((float)(mix.left  - SVC));
  int scR = softLimit((float)(mix.right - SVC));
  applyInertia((float)(scL - SVC), (float)(scR - SVC), dts);

  // 4. Output
  outputWrite();

  // 5. Debug
  debugPrint(now, cachedJoy);
}
