// ═══════════════════════════════════════════════════════════════
// Digger Control V7 — GL10 FOC + Speed-Adaptive Steering
// ═══════════════════════════════════════════════════════════════
//
// Dual-input controller for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Hardware (2026-04-25): GL10 FOC ESC + GL540L motor (replaced the
// earlier sensored-ESC setup). The GL10's FOC handles motor
// acceleration compensation and smoothness internally — Arduino's job
// shrinks to: input mixing, override switch, gear caps.
// V7.2 removed the Arduino-side inertia filter; the ESC's own
// Acceleration + Drag Force settings own command smoothing now.
// V7.6 removed the reverse-direction beeper entirely — audible
// alerts will return as a battery-aware system (see GitHub issue tracker).
// Telemetry: X.BUS Read Register (func 0x10) is polled non-blocking on
// Serial1 (D0/D1) — READ-ONLY. 0x10 is service control; it never puts the
// ESC into BUS_MODE, so PWM control authority is fully preserved. Both
// ESCs share the one half-duplex X.BUS. See [TELEMETRY].
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
//   [TELEMETRY]  X.BUS Read Register (0x10) on Serial1 — V/I/RPM/temp
//   [BEEPER]     Active piezo on D8 — horn (RC SWD/CH7) + beep patterns
//   [DEBUG]      10 Hz serial CSV (control + telemetry)
//
// Pin map:
//   A0  ← Joystick Y (throttle)        [14-bit ADC]
//   A1  ← Joystick X (steering)        [14-bit ADC]
//   D11 (unused — sbusUart TX on SCI0, S.BUS is RX-only)
//   D12 ← S.BUS RX (sbusUart on SCI0 via NPN inverter)
//   D8     (reserved — future battery-aware beeper, V8/UNO R4)
//   D9  → Left ESC                      [Servo PWM]
//   D10 → Right ESC                     [Servo PWM]
//   D0/D1 → Serial1 (X.BUS half-duplex telemetry bus to both ESCs)
//   USB-C → USB CDC Serial (debug + firmware upload)
//
// S.BUS wiring (unchanged inverter circuit, now lands on D12):
//   R7FG S.BUS signal ──[10K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► D12 (sbusUart RX)

#include <Arduino.h>
#include <Servo.h>
#include <WiFiS3.h>
#include "sbus.h"
#include "types.h"
#include "web_page.h"   // const char INDEX_HTML[] — the dashboard, served at "/"


// Second hardware UART on D11=TX(pin 11) / D12=RX(pin 12) via SCI0.
// S.BUS only needs RX; TX is unused. Named `sbusUart` (not Serial2)
// to avoid the macro collision with the core's pre-declared _UART2_.
// UNO R4 WiFi: SCI0 is on D11/D12 (Nano R4 used A4/A5 for SCI0).
// These pin constants live here (not in [CONFIG] below) because sbusUart is a
// global constructed at static-init time — it must see them before [CONFIG].
const uint8_t PIN_SBUS_TX = 11;  // SCI0 TX (D11) — unused, S.BUS is RX-only
const uint8_t PIN_SBUS_RX = 12;  // SCI0 RX (D12) — inverted S.BUS in
UART sbusUart(PIN_SBUS_TX, PIN_SBUS_RX);


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — All tunable constants
// ═══════════════════════════════════════════════════════════════

// Pins
const uint8_t PIN_JOY_Y  = A0;  // Throttle
const uint8_t PIN_JOY_X  = A1;  // Steering
const uint8_t PIN_ESC_L  = 9;   // Left ESC PWM (50 Hz, 1000-2000 us)
const uint8_t PIN_ESC_R  = 10;  // Right ESC PWM
const uint8_t PIN_BEEPER = 8;   // D8 — active piezo (digital HIGH = beep)

// S.BUS channel mapping (0-indexed). Confirmed by live capture while
// the operator moved each control independently on the RC6GS V3:
// trigger → ch 1, wheel → ch 0.
const uint8_t SBUS_CH_THR   = 1;  // trigger → throttle (forward/back)
const uint8_t SBUS_CH_STEER = 0;  // wheel   → steering (left/right)
const uint8_t SBUS_CH_GEAR  = 3;  // CH4 = gear selector (3-pos switch)
const uint8_t SBUS_CH_OVR   = 4;  // CH5 = override switch (3-pos switch)
const uint8_t SBUS_CH_HORN  = 6;  // CH7 = SWD button → horn (beep at +100%)

// S.BUS value range (raw 172-1811, center ~992)
const int SBUS_MIN = 172;
const int SBUS_MAX = 1811;
const int HORN_ON_RAW = 1400;  // SWD raw above this = horn ON (toward +100% ~1811)

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

// Joystick steering polarity: -1.0f to flip left/right (set after the
// operator-side cable was rewired and right-stick produced a left turn).
const float JOY_STEER_DIR = -1.0f;

// Power range — full PWM authority (1000-2000 us = ±500 us from SVC)
const float SOFT_RANGE = 500.0f;  // Max servo offset from center (us)

// Gear scaling — RC CH4 selects speed cap. 3-position switch:
//   LOW  → 60% wheel speed cap   (training / tight spaces)
//   MID  → 70% wheel speed cap   (normal driving)
//   HIGH → 100% wheel speed cap  (full throttle authority)
// Failsafe: when S.BUS is invalid, gearScale stays at LOW for safety.
const float GEAR_LOW_SCALE  = 0.55f;  // Eco   (was 0.40 — bumped 2026-06-14 for low-end torque)
const float GEAR_MID_SCALE  = 0.70f;  // Normal (was 0.65)
const float GEAR_HIGH_SCALE = 1.00f;  // Boost

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
// cap was for the older sensored-ESC hardware; GL10 + GL540L can take
// full reverse without trouble.
const float REVERSE_LIMIT = 0.50f;  // reverse capped at 50% of forward max

// Debug
const uint32_t PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output

// Wi-Fi telemetry-dashboard tuning (monitoring only — never affects control).
// Centralized here per the [CONFIG] "all tunable constants" rule; the Wi-Fi
// identity (SSID/pass) and runtime objects stay in [WIFI].
const uint8_t  WIFI_AP_CHANNEL       = 11;   // 2.4 GHz AP channel — off the crowded 1/6 (issue #54)
const uint32_t SSE_INTERVAL_MS       = 200;  // dashboard push period = 5 Hz (X.BUS poll rate is independent)
const uint32_t WIFI_MODEM_TIMEOUT_MS = 50;   // per-call Wi-Fi/modem timeout so one stalled write can't freeze loop()
const size_t   SSE_FRAME_CAP         = 384;  // SSE frame buffer: ": hb\ndata: " + JSON + "\n\n"


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
// [RC] — S.BUS input on sbusUart / SCI0 (D12 RX, NPN inverter)
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
  cachedJoy.zRotation = JOY_STEER_DIR * signX * expoCurve(normX, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC);

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
// [TELEMETRY] — X.BUS Read Register (0x10) on Serial1 (D0/D1)
// ═══════════════════════════════════════════════════════════════
// Non-blocking, READ-ONLY. Uses func 0x10 (point-to-point service read),
// NOT 0x50 — so the ESC never enters BUS_MODE and PWM control is fully
// preserved. One request per ESC returns all registers; we alternate
// ESC0/ESC1, EMA-smooth the slow signals, keep RPM instantaneous, and run
// an independent per-ESC freshness watchdog. Both ESCs share the one
// half-duplex bus on D0/D1 (idle HIGH, standard UART polarity — no inverter).

const uint8_t  XBUS_HDR_MASTER = 0x0F;
const uint8_t  XBUS_HDR_SLAVE  = 0xF0;
const uint8_t  XBUS_FUNC_READ  = 0x10;
const uint8_t  NUM_ESCS        = 2;

// Registers read every poll (one request returns all of them).
const uint8_t  REG_VBAT   = 0x0C;  // ×0.1 V
const uint8_t  REG_IBUS   = 0x0D;  // ×0.1 A (signed)
const uint8_t  REG_MSPEED = 0x02;  // electrical Hz (signed)
const uint8_t  REG_TESC   = 0x20;  // raw − 40 = °C
const uint8_t  REG_TMOT   = 0x22;  // raw − 40 = °C
const uint8_t  TELEM_REGS[] = {REG_VBAT, REG_IBUS, REG_MSPEED, REG_TESC, REG_TMOT};
const uint8_t  TELEM_NREG    = sizeof(TELEM_REGS);

const uint32_t TELEM_POLL_MS    = 10;     // short gap; alternating → ~30-40 Hz/ESC
const uint32_t TELEM_TIMEOUT_US = 6000;   // GL10 replies in ~2-3ms; 6ms = margin + fast give-up
                                          // on a silent ESC (so a flaky one barely stalls the good one)
const uint32_t TELEM_STALE_MS   = 5000;   // mark invalid after this long with no good read

// EMA new-sample weights. Slow signals smoothed; RPM is instantaneous.
const float TELEM_A_VOLT = 0.30f;
const float TELEM_A_CURR = 0.50f;
const float TELEM_A_TEMP = 0.10f;

EscTelem telem[NUM_ESCS] = {};

uint8_t  telemEsc        = 0;       // ESC currently being polled
bool     telemWaiting    = false;   // true = request sent, awaiting response
uint32_t telemLastPollMs = 0;
uint32_t telemReqStartUs = 0;
uint8_t  telRx[64];
int      telRxLen = 0;

// ---- X.BUS RX byte-level diagnostics (temporary, post-solder bring-up) ----
uint32_t telDbgRxTotal    = 0;  // total bytes ever seen on D0 (Serial1 RX)
uint32_t telDbgEchoCount  = 0;  // count of 0x0F bytes (our own TX seen on bus)
uint32_t telDbgSlaveCount = 0;  // count of 0xF0 bytes (ESC response header)
uint8_t  telDbgSnap[16]   = {0};
int      telDbgSnapLen    = 0;
uint32_t telDbgPrintPrevMs = 0;

// Checksum = low 8 bits of the sum of bytes [1 .. len-2] (header & checksum
// excluded). Matches the framing confirmed on the bench during X.BUS bring-up.
uint8_t xbusChecksum(const uint8_t *f, int len) {
  uint8_t s = 0;
  for (int i = 1; i < len - 1; i++) s += f[i];
  return s;
}

void telemSendRequest(uint8_t esc) {
  uint8_t tx[6 + TELEM_NREG];
  int n = 0;
  tx[n++] = XBUS_HDR_MASTER;
  tx[n++] = esc;             // point-to-point slave address
  tx[n++] = 0x00;            // extra byte — ignored in service control
  tx[n++] = XBUS_FUNC_READ;
  tx[n++] = TELEM_NREG;      // data length = number of register addresses
  for (uint8_t i = 0; i < TELEM_NREG; i++) tx[n++] = TELEM_REGS[i];
  tx[n] = xbusChecksum(tx, n + 1);
  n++;
  while (Serial1.available()) Serial1.read();  // drop stale RX / prior echo
  Serial1.write(tx, n);                        // non-blocking (no flush())
  telRxLen = 0;
}

// Fold one register value into the telem struct for ESC index e.
void telemApplyReg(uint8_t e, uint8_t reg, uint16_t raw) {
  EscTelem *t = &telem[e];
  bool first = !t->valid;
  switch (reg) {
    case REG_VBAT: {
      float v = raw * 0.1f;
      t->voltage = first ? v : t->voltage + TELEM_A_VOLT * (v - t->voltage);
      break;
    }
    case REG_IBUS: {
      float a = (int16_t)raw * 0.1f;
      t->busCurrentA = first ? a : t->busCurrentA + TELEM_A_CURR * (a - t->busCurrentA);
      break;
    }
    case REG_MSPEED:
      t->rpmHz = (int16_t)raw;                 // instantaneous
      break;
    case REG_TESC: {
      float c = (float)(raw & 0xFF) - 40.0f;   // temp is the low byte, raw − 40
      t->escTempC = first ? c : t->escTempC + TELEM_A_TEMP * (c - t->escTempC);
      break;
    }
    case REG_TMOT: {
      float c = (float)(raw & 0xFF) - 40.0f;
      t->motorTempC = first ? c : t->motorTempC + TELEM_A_TEMP * (c - t->motorTempC);
      break;
    }
  }
}

// Scan the RX buffer for a valid 0x10 response from `esc`; parse if complete.
// Skips our own half-duplex TX echo (which starts with 0x0F, not 0xF0).
bool telemTryParse(uint8_t esc, uint32_t nowMs) {
  for (int i = 0; i + 6 <= telRxLen; i++) {
    if (telRx[i]     != XBUS_HDR_SLAVE) continue;
    if (telRx[i + 1] != esc)            continue;
    if (telRx[i + 3] != XBUS_FUNC_READ) continue;
    uint8_t dlen = telRx[i + 4];
    if (dlen == 0 || dlen % 3 != 0) continue;       // each reg = addr + 2 bytes
    if (i + 5 + dlen > telRxLen)    continue;        // full data not in yet
    const uint8_t *d = &telRx[i + 5];
    for (uint8_t g = 0; g + 3 <= dlen; g += 3) {
      uint16_t val = d[g + 1] | (d[g + 2] << 8);
      telemApplyReg(esc, d[g], val);
    }
    telem[esc].lastGoodMs = nowMs;
    telem[esc].valid = true;
    return true;
  }
  return false;
}

// Call every loop iteration. Sends one request at a time, alternating ESCs,
// and never blocks: it drains RX across iterations and times out on silence.
void telemUpdate() {
  uint32_t nowMs = millis();

  // Drain available bytes every iteration (cheap).
  while (Serial1.available() && telRxLen < (int)sizeof(telRx)) {
    uint8_t b = Serial1.read();
    telRx[telRxLen++] = b;
    telDbgRxTotal++;                                   // any byte on D0
    if (b == XBUS_HDR_MASTER) telDbgEchoCount++;       // 0x0F = our own TX echo
    if (b == XBUS_HDR_SLAVE)  telDbgSlaveCount++;      // 0xF0 = ESC reply
    if (telDbgSnapLen < 16) telDbgSnap[telDbgSnapLen++] = b;
  }

  if (telemWaiting) {
    if (telemTryParse(telemEsc, nowMs)) {
      telemWaiting = false;
      telemEsc = (telemEsc + 1) % NUM_ESCS;
    } else if ((micros() - telemReqStartUs) > TELEM_TIMEOUT_US) {
      telemWaiting = false;                          // no/late response — move on
      telemEsc = (telemEsc + 1) % NUM_ESCS;
    }
  } else if ((nowMs - telemLastPollMs) >= TELEM_POLL_MS) {
    telemLastPollMs = nowMs;
    telemSendRequest(telemEsc);
    telemReqStartUs = micros();
    telemWaiting = true;
  }

  // Per-ESC freshness watchdog.
  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    if (telem[i].valid && (nowMs - telem[i].lastGoodMs) > TELEM_STALE_MS) {
      telem[i].valid = false;
    }
  }
}


// ═══════════════════════════════════════════════════════════════
// [BEEPER] — active piezo on D8: horn (RC SWD, held) + queued patterns
// ═══════════════════════════════════════════════════════════════
// Non-blocking. Horn = continuous tone while the RC SWD button (CH7) is held.
// Patterns = a short on/off sequence (e.g. Wi-Fi-ready beep-beep). Both share
// D8; the horn ORs over any pattern. UX/alert only — no control-path impact.

bool hornActive = false;                             // set each loop from the RC horn channel
const uint16_t BEEP_WIFI_READY[] = {180, 140, 180};  // on, off, on (ms) → "beep beep"
const uint16_t* beepSeq = nullptr;
int      beepLen = 0, beepIdx = -1;
uint32_t beepPhaseMs = 0;

void beeperInit() { pinMode(PIN_BEEPER, OUTPUT); digitalWrite(PIN_BEEPER, LOW); }

// Queue a non-blocking on/off pattern (durations in ms, starting with ON).
void beepStart(const uint16_t *seq, int len) {
  beepSeq = seq; beepLen = len; beepIdx = 0; beepPhaseMs = millis();
}

// Call every loop. Drives D8 from the horn (held) OR the active pattern.
void beeperUpdate() {
  bool patternOn = false;
  if (beepIdx >= 0 && beepIdx < beepLen) {
    if (millis() - beepPhaseMs >= beepSeq[beepIdx]) { beepIdx++; beepPhaseMs = millis(); }
    patternOn = (beepIdx >= 0 && beepIdx < beepLen) && (beepIdx % 2 == 0);
  }
  digitalWrite(PIN_BEEPER, (hornActive || patternOn) ? HIGH : LOW);
}


// ═══════════════════════════════════════════════════════════════
// [WIFI] — AP + HTTP telemetry server (WiFiS3, stock UNO R4 WiFi)
// ═══════════════════════════════════════════════════════════════
// Hosts a Wi-Fi access point and serves telemetry as JSON at /data for
// the dashboard (dashboard/index.html). MONITORING ONLY — no control
// inputs are ever accepted over Wi-Fi (safety). Engineered non-blocking:
// one short client transaction per loop pass, a bounded request-read
// timeout, and a tiny payload, so it doesn't disturb the control loop.
// The Servo PWM is hardware-timed, so even a brief loop stall (e.g. a
// client connect) cannot glitch the ESC output.

const char WIFI_SSID[] = "Digger-Telemetry";
const char WIFI_PASS[] = "digger12345";   // WPA2 needs >= 8 chars
// Wi-Fi tuning constants (AP channel, SSE rate, modem timeout, frame cap) live
// in [CONFIG] per the project's tunable-constants rule.
WiFiServer wifiServer(80);
bool     wifiUp  = false;
uint32_t wifiSeq = 0;
// ETag for the static dashboard. Filled in wifiInit() from the page length +
// firmware tag, so browsers can cache the HTML and a refresh returns 304 instead
// of re-downloading ~33 KB through the blocking Wi-Fi modem (issue #54).
char pageEtag[24] = "\"d0\"";

// Server-Sent Events: one persistent connection streams telemetry, instead of
// the browser opening a fresh (slow) connection every poll. This is the big
// update-rate win on WiFiS3, and EventSource auto-reconnects after a dropout.
WiFiClient     sseClient;
uint32_t       sseLastMs = 0;

// Override switch → dashboard mode (0=RC, 1=joy/auto-middle, 2=blend).
int wifiMode() {
  if (!sbusValid) return 0;
  int ovr = rcOverride();
  if (ovr < OVR_LO) return 0;
  if (ovr > OVR_HI) return 2;
  return 1;
}

void wifiInit() {
  if (WiFi.status() == WL_NO_MODULE) {
    if (Serial) Serial.println("# WiFi: NO MODULE — ESP32-S3 radio not responding");
    return;
  }
  if (Serial) {
    Serial.print("# WiFi fw version: ");
    Serial.println(WiFi.firmwareVersion());
  }
  uint8_t st = WiFi.beginAP(WIFI_SSID, WIFI_PASS, WIFI_AP_CHANNEL);
  if (Serial) {
    Serial.print("# beginAP returned status=");
    Serial.println(st);
  }
  if (st == WL_AP_LISTENING) {
    wifiUp = true;
    wifiServer.begin();
    // Build the dashboard ETag once. Length + firmware tag, so it changes
    // whenever the embedded page changes and stale caches auto-invalidate.
    snprintf(pageEtag, sizeof(pageEtag), "\"d%uv711\"", (unsigned)strlen(INDEX_HTML));
    beepStart(BEEP_WIFI_READY, 3);   // "beep beep" — Wi-Fi AP is up/ready
    if (Serial) {
      Serial.print("# WiFi AP '");
      Serial.print(WIFI_SSID);
      Serial.print("' UP — http://");
      Serial.println(WiFi.localIP());
    }
  } else if (Serial) {
    Serial.print("# WiFi AP '"); Serial.print(WIFI_SSID);
    Serial.println("' FAILED to start");
  }
}

// Periodic Wi-Fi status line (every ~3 s) for bench diagnostics.
uint32_t wifiDbgPrev = 0;
void wifiDebug(uint32_t nowUs) {
  if (!Serial || (nowUs - wifiDbgPrev) < 3000000UL) return;
  wifiDbgPrev = nowUs;
  Serial.print("# WIFI up="); Serial.print(wifiUp);
  Serial.print(" status="); Serial.print(WiFi.status());
  Serial.print(" clients_seq="); Serial.println(wifiSeq);

  // X.BUS RX byte-level diagnostics — tells us whether D0 sees anything at all.
  Serial.print("# XBUS rx_total="); Serial.print(telDbgRxTotal);
  Serial.print(" echo(0x0F)=");     Serial.print(telDbgEchoCount);
  Serial.print(" slave(0xF0)=");    Serial.print(telDbgSlaveCount);
  Serial.print(" snap=[");
  for (int i = 0; i < telDbgSnapLen; i++) {
    char hex[4]; snprintf(hex, sizeof(hex), "%02X ", telDbgSnap[i]); Serial.print(hex);
  }
  Serial.println("]");
  telDbgSnapLen = 0;   // reset for next window's snapshot
}

// Build the telemetry JSON into body; returns its length. Shared by the
// one-shot /data endpoint and the SSE stream.
int buildTelemJson(char *body, size_t cap) {
  wifiSeq++;
  uint32_t nowMs = millis();
  // Per-ESC age (ms since last good frame). 0 if never received.
  uint32_t age0 = telem[0].lastGoodMs ? (nowMs - telem[0].lastGoodMs) : 999999UL;
  uint32_t age1 = telem[1].lastGoodMs ? (nowMs - telem[1].lastGoodMs) : 999999UL;
  int n = snprintf(body, cap,
    "{\"t\":%lu,\"seq\":%lu,\"gear\":%d,\"mode\":%d,\"fs\":%d,\"lost\":%d,"
    "\"outL\":%d,\"outR\":%d,"
    "\"e0\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d},"
    "\"e1\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d}}",
    (unsigned long)nowMs, (unsigned long)wifiSeq, (int)currentGear, wifiMode(),
    sbusData.failsafe ? 1 : 0, sbusData.lost_frame ? 1 : 0,
    outL, outR,
    telem[0].valid ? 1 : 0, (unsigned long)age0, (long)telem[0].rpmHz * 30,
    (int)lroundf(telem[0].busCurrentA * 10.0f), (int)lroundf(telem[0].voltage * 10.0f),
    (int)lroundf(telem[0].escTempC), (int)lroundf(telem[0].motorTempC),
    telem[1].valid ? 1 : 0, (unsigned long)age1, (long)telem[1].rpmHz * 30,
    (int)lroundf(telem[1].busCurrentA * 10.0f), (int)lroundf(telem[1].voltage * 10.0f),
    (int)lroundf(telem[1].escTempC), (int)lroundf(telem[1].motorTempC));
  // snprintf returns the length it WOULD have written; clamp to the buffer so
  // callers never read past `body` (Content-Length and write length stay valid
  // even if a frame were ever to overflow `cap`).
  if (n < 0) return 0;
  if ((size_t)n >= cap) n = (int)cap - 1;
  return n;
}

void wifiSendData(WiFiClient &client) {
  char body[360];
  int n = buildTelemJson(body, sizeof(body));
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                 "Access-Control-Allow-Origin: *\r\nConnection: close\r\nContent-Length: "));
  client.print(n);
  client.print(F("\r\n\r\n"));
  client.write(reinterpret_cast<const uint8_t *>(body), n);
}

// Serve the embedded dashboard. The page is static, so it carries an ETag and
// Cache-Control: no-cache (store-but-revalidate): the first visit downloads it
// once, then every refresh sends If-None-Match and gets a tiny 304 here instead
// of re-streaming ~33 KB through the blocking Wi-Fi modem (issue #54). Live
// values arrive separately over SSE, so the HTML itself never reloads in normal
// use. The 200 body is chunked so one big write doesn't hog the loop.
void wifiSendPage(WiFiClient &client, bool notModified) {
  if (notModified) {
    client.print(F("HTTP/1.1 304 Not Modified\r\nETag: "));
    client.print(pageEtag);
    client.print(F("\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n"));
    return;
  }
  size_t len = strlen(INDEX_HTML);
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                 "Cache-Control: no-cache\r\nETag: "));
  client.print(pageEtag);
  client.print(F("\r\nConnection: close\r\nContent-Length: "));
  client.print(len);
  client.print(F("\r\n\r\n"));
  const char *p = INDEX_HTML;
  size_t remaining = len;
  while (remaining) {
    size_t chunk = remaining > 1024 ? 1024 : remaining;
    size_t w = client.write(reinterpret_cast<const uint8_t *>(p), chunk);
    if (w == 0) break;          // client went away
    p += w; remaining -= w;
  }
}

// Non-blocking. Accepts at most one new client per call (bounded request read),
// and pushes a telemetry frame to the live SSE stream on its interval.
void wifiUpdate() {
  if (!wifiUp) return;

  // Cap EACH Wi-Fi/modem call below at WIFI_MODEM_TIMEOUT_MS. The WiFiS3 default
  // is MODEM_TIMEOUT = 10 000 ms — a stalled iPhone TCP window (Safari
  // backgrounded, weak signal) can otherwise freeze loop() for seconds, starving
  // servo + S.BUS updates. NOTE: this bounds each INDIVIDUAL modem operation, not
  // the total time in wifiUpdate() — serving the full page makes several
  // client.write() calls, so the aggregate can exceed it (page delivery is the
  // main cost, and caching keeps it rare). The cost is an occasionally dropped
  // telemetry frame — the right trade for a vehicle controller (issue #54).
  // Restored to the default before returning so AP bring-up keeps the full timeout.
  modem.timeout(WIFI_MODEM_TIMEOUT_MS);

  WiFiClient client = wifiServer.available();
  if (client) {
    // Read the request line + headers into one bounded buffer (cap + 25 ms) so
    // we can route on the first line AND honor a conditional-GET If-None-Match
    // for the cached dashboard. 512 B comfortably holds an iOS Safari header set.
    char req[512];
    int  ri = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < 25 && ri < (int)sizeof(req) - 1) {
      while (client.available() && ri < (int)sizeof(req) - 1) req[ri++] = client.read();
      if (ri >= 4 && req[ri - 4] == '\r' && req[ri - 3] == '\n' &&
                     req[ri - 2] == '\r' && req[ri - 1] == '\n') break;  // end of headers
    }
    req[ri] = '\0';

    if (strstr(req, "/events")) {
      // Upgrade to a persistent Server-Sent Events stream.
      if (sseClient.connected()) sseClient.stop();   // replace any stale stream
      sseClient = client;
      sseClient.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                        "Cache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n\r\n"));
      sseLastMs = 0;                                   // push first frame immediately
    } else if (strstr(req, "/data")) {
      wifiSendData(client); client.flush(); client.stop();
    } else {
      // Static dashboard. If the browser already holds our ETag, reply 304 and
      // skip the ~33 KB transfer entirely (issue #54).
      bool cached = strstr(req, "If-None-Match") && strstr(req, pageEtag);
      wifiSendPage(client, cached); client.flush(); client.stop();
    }
  }

  // Stream live telemetry to the dashboard over the open SSE connection.
  // Each push starts with an SSE comment line ": hb\n" — this is a no-op for
  // the EventSource client but exercises the TCP socket and helps prevent
  // Safari/iOS from parking the connection in a stalled state.
  if (sseClient.connected()) {
    uint32_t now = millis();
    if (now - sseLastMs >= SSE_INTERVAL_MS) {
      sseLastMs = now;
      // Build the whole SSE frame — heartbeat comment, data line, terminator —
      // into one buffer and ship it in a SINGLE write(). That is one AT
      // round-trip to the ESP32 bridge per push instead of three, cutting the
      // blocking modem calls 3:1 (issue #54). SSE_FRAME_CAP comfortably holds the
      // 11-byte prefix + ~250-byte JSON + 2-byte terminator.
      char frame[SSE_FRAME_CAP];
      int len = snprintf(frame, sizeof(frame), ": hb\ndata: ");
      // Reserve the last 2 bytes for the "\n\n" terminator so the JSON body can
      // never crowd it out; buildTelemJson clamps its own length to the cap we
      // pass, keeping `len` provably within `frame` for any payload size.
      len += buildTelemJson(frame + len, sizeof(frame) - len - 2);
      frame[len++] = '\n';
      frame[len++] = '\n';
      sseClient.write(reinterpret_cast<const uint8_t *>(frame), len);
    }
  }

  modem.timeout(MODEM_TIMEOUT);   // restore default 10 s timeout on the way out
}


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV (control + telemetry)
// ═══════════════════════════════════════════════════════════════
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,
//          V0dV,I0dA,RPM0,TE0,TM0,OK0, V1dV,I1dA,RPM1,TE1,TM1,OK1
// Telemetry columns are integer-scaled: voltage in 0.1 V (dV), current in
// 0.1 A (dA), RPM in electrical Hz, temps in °C, OK = fresh-telemetry flag.

uint32_t prevPrint = 0;

void debugInit() {
  Serial.begin(115200);
  delay(50);
  if (Serial) {
    Serial.println("# === Digger V7.11 — GL10 FOC + S.BUS + Gear + X.BUS telem + Wi-Fi AP + beeper/horn ===");
    Serial.println("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,V0dV,I0dA,RPM0,TE0,TM0,OK0,V1dV,I1dA,RPM1,TE1,TM1,OK1");
  }
}

void debugPrint(uint32_t now) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  int rcT = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_THR])   : SVC;
  int rcS = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_STEER]) : SVC;
  int rc4 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_GEAR])  : SVC;
  int rc5 = sbusValid ? sbusToServo(sbusData.ch[SBUS_CH_OVR])   : SVMIN;

  // Telemetry, integer-scaled so we avoid float printf on this core.
  int v0 = (int)lroundf(telem[0].voltage    * 10.0f);
  int i0 = (int)lroundf(telem[0].busCurrentA * 10.0f);
  int e0 = (int)lroundf(telem[0].escTempC);
  int m0 = (int)lroundf(telem[0].motorTempC);
  int v1 = (int)lroundf(telem[1].voltage    * 10.0f);
  int i1 = (int)lroundf(telem[1].busCurrentA * 10.0f);
  int e1 = (int)lroundf(telem[1].escTempC);
  int m1 = (int)lroundf(telem[1].motorTempC);

  char buf[200];
  snprintf(buf, sizeof(buf),
           "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           rcT, rcS, rc4, rc5,
           cachedJoy.rawY, cachedJoy.rawX,
           outL, outR, (int)currentGear,
           sbusData.failsafe, sbusData.lost_frame,
           v0, i0, telem[0].rpmHz, e0, m0, telem[0].valid ? 1 : 0,
           v1, i1, telem[1].rpmHz, e1, m1, telem[1].valid ? 1 : 0);
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

void setup() {
  analogReadResolution(14);
  sbusRx.Begin();
  outputInit();
  beeperInit();
  debugInit();
  Serial1.begin(115200);  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
  wifiInit();             // Wi-Fi AP + telemetry server (monitoring only)
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
  hornActive = sbusValid && (sbusData.ch[SBUS_CH_HORN] > HORN_ON_RAW);
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

  // 3.5 Telemetry — non-blocking X.BUS Read Register (0x10), read-only.
  // Never enters BUS_MODE, so it cannot affect the control output above.
  telemUpdate();

  // 3.6 Wi-Fi — serve telemetry JSON to the dashboard (monitoring only).
  wifiUpdate();
  wifiDebug(now);
  beeperUpdate();   // horn (RC SWD) + any queued beep pattern (D8)

  // 4. Debug
  debugPrint(now);
}
