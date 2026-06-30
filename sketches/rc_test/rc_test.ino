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
//   [GEAR]       RC CH4 → Eco 65% / Normal 80% / Boost 100% average-speed cap
//   [MIXER]      Override switch — selects RC vs joystick
//   [OUTPUT]     ESC servo PWM
//   [TELEMETRY]  X.BUS Read Register (0x10) on Serial1 — V/I/RPM/temp
//   [BEEPER]     Active piezo on D8 — horn (RC SWD/CH7) + beep patterns
//   [ALERT]      Battery (< 10.5 V, latched) + inactivity (RC off) alarms on D8
//   [DEBUG]      10 Hz serial CSV (control + telemetry)
//
// Pin map:
//   A0  ← Joystick Y (throttle)        [14-bit ADC]
//   A1  ← Joystick X (steering)        [14-bit ADC]
//   D11 (unused — sbusUart TX on SCI0, S.BUS is RX-only)
//   D12 ← S.BUS RX (sbusUart on SCI0 via NPN inverter)
//   D8  → Active piezo — horn + Wi-Fi/battery/inactivity alarms
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
#include <WDT.h>        // RA4M1 hardware watchdog — control-loop runaway backstop (#69)
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
const uint16_t BEEP_WIFI_READY[]   = {180, 140, 180};  // Wi-Fi-ready "beep beep": on, off, on (ms)
const int      BEEP_WIFI_READY_LEN = 3;                // phases in BEEP_WIFI_READY[]

// [ALERT] repeating alarm patterns (ms, starting with ON; played as a loop).
// Distinct on purpose so each is identifiable by ear — see OPERATOR-GUIDE.md.
const uint16_t ALERT_INACT[]   = {500, 1500};                   // one long beep / 2 s  (RC off)
const int      ALERT_INACT_LEN = 2;
const uint16_t ALERT_LOWV[]    = {120, 120, 120, 120, 120, 600};  // three fast chirps / ~1.2 s (low batt)
const int      ALERT_LOWV_LEN  = 6;

// [ALERT] tunables
const uint32_t INACT_RC_OFF_MS  = 60000UL;  // RC off this long → inactivity beep ("unplug me")
const float    LOWV_THRESH_V    = 10.6f;   // worst-of-two pack EMA below this → low-batt alarm (heads-up beep before the 10.0 V cutoff)
const float    LOWV_PLAUS_MIN_V = 6.0f;    // pack reading below this = not present / bad → ignore
const float    LOWV_PLAUS_MAX_V = 13.0f;   // pack reading above this = bad read → ignore
const uint32_t LOWV_DEBOUNCE_MS = 3000UL;  // must stay below thresh this long before latching
const uint32_t ALERT_STARTUP_MS = 60000UL;  // suppress low-V alarm until telemetry/EMA settles

// [SAFETY] battery motor-cutoff (#65) — a HARD stop below the audible low-V alarm.
// Worst-of-two pack EMA < CUTOFF_THRESH_V, validity-gated + debounced + latched →
// motors cut AND the D8 alarm chirps immediately (no startup grace; the validity
// gate alone guards the ~1 s telemetry warm-up). 10.0 V on a 3S pack = 3.33 V/cell
// average — conservative, above the ~3.0 V/cell damage line even with some cell
// imbalance. Latches until power-cycle. All three timings are tunable here.
const float    CUTOFF_THRESH_V    = 10.0f;   // worst pack EMA below this → cut motors
const uint32_t CUTOFF_DEBOUNCE_MS = 1500UL;  // sustained below thresh before cutting (ignore load sag; do NOT set 0)
const uint32_t CUTOFF_HOLD_MS     = 500UL;   // command neutral this long before cutting PWM (let GL10 wind down)
// Boot gate (#65): the cutoff latch lives in RAM, so a watchdog/brownout reset
// would clear it. To stop a low pack from driving in that window, the output is
// held OFF after boot until a valid battery reading confirms it's ABOVE the
// cutoff. If telemetry never reports (dead X.BUS), fail OPEN after this timeout
// so a telemetry fault can't permanently disable driving.
const uint32_t BATTERY_CONFIRM_MS = 3000UL;  // no valid battery within this → allow drive anyway (telemetry-optional)

// [SAFETY] low-battery Eco lockout (#65) — a STAGE BEFORE the hard cutoff. When
// the worst pack sags low for an extended period, force Eco gear regardless of
// the RC gear switch so a nearly-drained pack isn't hit with Boost/Normal load.
// Latches until power-cycle. (Staging: ~11.0 V → Eco lock; 10.0 V → hard cutoff.)
const float    ECO_LOCK_THRESH_V    = 11.0f;    // worst pack EMA below this → force Eco
const uint32_t ECO_LOCK_DEBOUNCE_MS = 15000UL;  // sustained below thresh before locking Eco

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

// Joystick throttle gain (#90) — the Genie stick under-ranges: full physical
// deflection only reaches ~0.75 xSpeed, so the rider couldn't hit the per-gear
// caps below. Lift it so full travel can reach the cap. Joystick-only; RC
// unaffected. Tunable.
const float JOY_THROTTLE_GAIN = 1.40f;

// ── Throttle output caps ──────────────────────────────────────────────────
// All caps below are MAX TRACK-OUTPUT fractions (0..1). A straight-line track
// output = xSpeed * gearScale, so each cap is converted to the xSpeed domain
// via outCapToX() at point of use — one helper, no scattered conversions.

// Per-gear joystick FORWARD cap (#90) — the joystick rider's max forward track
// output per gear (RC keeps the gear's full authority). Throttle axis only;
// steering / pivot unaffected.
const float JOY_CAP_ECO    = 0.65f;  // Eco
const float JOY_CAP_NORMAL = 0.75f;  // Normal
const float JOY_CAP_BOOST  = 0.90f;  // Boost

// REVERSE cap (#87) — flat across ALL gears and BOTH inputs (RC + joystick).
// Replaces the old per-gear REVERSE_LIMIT / REVERSE_LIMIT_LOW. Reverse output is
// a constant 65% regardless of gear. Within the GL10 Max Reverse Force setting.
const float REVERSE_CAP    = 0.65f;

// Power range — full PWM authority (1000-2000 us = ±500 us from SVC)
const float SOFT_RANGE = 500.0f;  // Max servo offset from center (us)

// Gear scaling — RC CH4 selects the AVERAGE-speed cap. 3-position switch:
//   LOW  → 65% average-speed cap  (training / tight spaces)
//   MID  → 80% average-speed cap  (normal driving — the everyday gear)
//   HIGH → 100% (the rail)        (full throttle authority)
// The cap limits the AVERAGE track speed, not each wheel: in a turn the outer
// track may use the headroom up to the ESC limit so the turn holds its speed
// (see curvatureDrive). Boost has no headroom (already at the rail).
// Failsafe: when S.BUS is invalid, gearScale stays at LOW for safety.
const float GEAR_LOW_SCALE  = 0.65f;  // Eco   (+10pp 2026-06-21 for usefulness; keeps ~35% turn headroom)
const float GEAR_MID_SCALE  = 0.80f;  // Normal (+10pp 2026-06-21 — the everyday gear; keeps ~20% turn headroom)
const float GEAR_HIGH_SCALE = 1.00f;  // Boost  (no turn headroom — at the rail)

// Eco gets extra PIVOT authority so the operator can still maneuver in tight
// spaces (pivot input cap; forward stays at GEAR_LOW_SCALE 65%). Effective pivot
// wheel speed = pivot cap × gear scale: 0.725 × 0.65 = 0.47 (vs 0.60 × 0.65).
// (Reverse is no longer Eco-special — it's a flat REVERSE_CAP for all gears.)
const float PIVOT_SPEED_CAP_LOW = 0.725f;

// Gear state — declared here so curvatureDrive() and rcCommand() can read
// it for the Eco-only conditional caps above. updateGear() in [GEAR]
// owns the writes.
float gearScale  = GEAR_LOW_SCALE;
Gear  currentGear = GEAR_LOW;

// [SAFETY] latches — set by the [SAFETY] module (search [SAFETY]), read here by
// updateGear() and by the output gate. Declared early so both can see them.
bool batteryCutoffLatched = false;  // worst pack <= CUTOFF_THRESH_V → cut motors
bool ecoLockLatched       = false;  // worst pack <= ECO_LOCK_THRESH_V → force Eco gear
bool batteryOkConfirmed   = false;  // a valid reading has confirmed pack ABOVE cutoff (boot gate)

// Convert a MAX TRACK-OUTPUT cap (0..1) to the xSpeed domain for the current
// gear (output = xSpeed * gearScale). Single point of truth for every cap.
inline float outCapToX(float outCap) { return outCap / gearScale; }

// Curvature drive — pivot/curvature blend band.
// |xSpeed| <= START: pure pivot (counter-rotate at PIVOT_SPEED_CAP)
// |xSpeed| >= END:   pure curvature (outer holds the average, inner slows)
// Between: smoothstep blend so the operator doesn't feel a mode jump. The band
// is WIDE (0.05–0.55) so the pivot↔forward/reverse hand-off is gradual — a
// narrow band made the transition snap near 15–20% throttle (#72).
const float PIVOT_BLEND_START = 0.05f;
const float PIVOT_BLEND_END   = 0.55f;
const float PIVOT_SPEED_CAP   = 0.60f;  // pivot rotation cap (~60% wheel power)

// Outer-track turn cap (#96). The #72 outer-track headroom lets the outer wheel
// borrow up to the ESC rail to hold speed through a turn — but when the INNER
// track is stopped (full steer) that headroom is wasted (the outer doesn't need
// ~99% to swing the nose). curvatureDrive fades the outer-track ceiling from the
// rail (straight, both tracks moving) down to TURN_TRACK_CAP (full steer, inner
// stopped), driven by |zRotation| — open-loop, smooth, no hard switch.
const float TURN_TRACK_CAP    = 0.70f;  // outer-track cap at full steer (field-tune)

// RC input gains — neutral baseline (1.0 = no scaling). Stick travel
// maps directly to curvatureDrive, which already handles inner-track
// slowdown and pivot/curvature blend. Earlier non-unity values were
// band-aids compensating for the flipped-ESC steering bug; with the
// root cause fixed, the gains return to neutral.
const float RC_THROTTLE_GAIN = 1.00f;
const float RC_STEERING_GAIN = 1.00f;

// Debug
const uint32_t PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output

// Wi-Fi telemetry-dashboard tuning (monitoring only — never affects control).
// Centralized here per the [CONFIG] "all tunable constants" rule; the Wi-Fi
// identity (SSID/pass) and runtime objects stay in [WIFI].
const uint8_t  WIFI_AP_CHANNEL       = 11;   // 2.4 GHz AP channel — off the crowded 1/6 (issue #54)
const uint32_t SSE_INTERVAL_MS       = 200;  // dashboard push period = 5 Hz (X.BUS poll rate is independent)
const uint32_t WIFI_MODEM_TIMEOUT_MS = 50;   // per-call Wi-Fi/modem timeout so one stalled write can't freeze loop()
const size_t   SSE_FRAME_CAP         = 384;  // SSE frame buffer: ": hb\ndata: " + JSON + "\n\n"

// Safety watchdog (#69). The MCU resets if loop() fails to service the control
// path (read inputs + write outputs) within WDT_TIMEOUT_MS — a reset stops PWM,
// so the ESCs go to neutral/failsafe instead of holding the last throttle. This
// bounds ANY loop stall (Wi-Fi serving or otherwise) to at most this long.
// Starting value; tune on the bench. Must stay above the worst-case single loop
// pass (with incremental page serving, a pass is far under this).
const uint32_t WDT_TIMEOUT_MS        = 250;
const size_t   WIFI_PAGE_CHUNK       = 1024;  // dashboard HTML bytes sent per loop pass (incremental)


// ═══════════════════════════════════════════════════════════════
// [DRIVE] — curvatureDrive: proven FRC algorithm (WPILib)
// ═══════════════════════════════════════════════════════════════

// Gear-aware tank mix. `gearScale` (Eco/Normal/Boost) caps the AVERAGE forward
// speed — NOT each wheel — so in a turn the outer track may use the headroom up
// to the ESC limit (±1.0) and the turn keeps its speed instead of slowing (#72).
// At Boost (gearScale = 1.0) there is no headroom, so the outer simply
// desaturates at the rail (unchanged behavior). Pivot is gear-scaled so low
// gears spin gently.
WheelSpeeds curvatureDrive(float xSpeed, float zRotation, float gearScale) {
  xSpeed    = constrain(xSpeed, -1.0f, 1.0f);
  zRotation = constrain(zRotation, -1.0f, 1.0f);

  // Pivot output: counter-rotate the tracks (capped), then gear-scale so low
  // gears pivot gently. Eco uses a looser cap to keep usable pivot authority.
  float pivotCap = (currentGear == GEAR_LOW) ? PIVOT_SPEED_CAP_LOW : PIVOT_SPEED_CAP;
  float cappedRotation = constrain(zRotation, -pivotCap, pivotCap);
  float pivotL = (xSpeed - cappedRotation) * gearScale;
  float pivotR = (xSpeed + cappedRotation) * gearScale;

  // Curvature output: the gear caps the AVERAGE (avg); an ADDITIVE steering term
  // (delta) forms the inner/outer differential. delta's sign is set by the
  // STEERING STICK alone — never by the throttle direction — so steering stays
  // consistent in reverse: stick-left = nose-left going forward AND backward
  // (#86 Part 1). The old multiply, avg*(1±|z|), scaled the differential by avg,
  // so when avg went negative (reverse) the inner/outer swapped — that was the
  // mid-range "steering reverses when backing up" bug. Forward is UNCHANGED:
  // for avg ≥ 0, avg ∓ delta == avg*(1∓|z|). Peak magnitude is still
  // |avg|+delta = |avg|·(1+|z|), so the #96 ceiling and #72 headroom are intact.
  float avg   = xSpeed * gearScale;
  float delta = fabsf(avg) * fabsf(zRotation);   // symmetric steering term, >= 0
  float curvL, curvR;
  if (zRotation > 0) {        // turn LEFT
    curvL = avg - delta;     // left = inner  (less forward / harder reverse)
    curvR = avg + delta;     // right = outer
  } else {                    // turn RIGHT
    curvL = avg + delta;
    curvR = avg - delta;
  }
  // Outer-track ceiling fades from the ESC rail (gentle turn — both tracks moving)
  // down to TURN_TRACK_CAP (full steer — inner track stopped). |zRotation| is the
  // turn-sharpness signal (inner = avg*(1-|z|) → 0 at full steer), so the borrowed
  // headroom shrinks smoothly as the inner stops — no knee, no RPM feedback (#96).
  // Straight (|z| = 0) keeps the full rail, so straight-line throttle is unchanged.
  // Forward AND reverse are treated symmetrically: REVERSE_CAP bounds the reverse
  // *average* speed upstream, and the outer track then borrows the same turn
  // headroom forward does — so a reverse turn swings precisely instead of slowing
  // (operator decision, 2026-06-28). No reverse-specific ceiling special-case.
  float ceiling = 1.0f - (1.0f - TURN_TRACK_CAP) * fabsf(zRotation);
  float peak = fmaxf(fabsf(curvL), fabsf(curvR));
  if (peak > ceiling) {    // desaturate against the faded ceiling, preserving the turn ratio
    float k = ceiling / peak;
    curvL *= k;
    curvR *= k;
  }

  // Smoothstep blend pivot → curvature as |xSpeed| grows, across a WIDE band so
  // the pivot↔drive hand-off is gradual (no snap near low throttle, #72). Zero
  // slope at both endpoints means the operator never feels a mode boundary.
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

// RC axis command (xSpeed/zRotation), post-gain and reverse-limit — pre-mix,
// pre-curvatureDrive. The mix combines RC + joystick at this axis level (#90),
// then curvatureDrive runs once on the combined command.
DriveCommand rcCommand() {
  float xSpeed    = (float)(rcThrottle() - SVC) / SOFT_RANGE;
  float zRotation = (float)(rcSteering() - SVC) / SOFT_RANGE;
  // Apply tunable input gains, then clamp: RC keeps the gear's full forward
  // authority (1.0); reverse is the flat REVERSE_CAP for every gear (#87).
  xSpeed    = constrain(xSpeed * RC_THROTTLE_GAIN, -outCapToX(REVERSE_CAP), 1.0f);
  zRotation = constrain(zRotation * RC_STEERING_GAIN, -1.0f, 1.0f);
  return {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [GEAR] — RC CH4 → average-speed cap (Eco 65% / Normal 80% / Boost 100%)
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
  // Low-battery Eco lockout (#65): once the pack has sagged low for a while,
  // force Eco regardless of the RC gear switch to ease load on a draining pack.
  // Latched until power-cycle.
  if (ecoLockLatched) {
    gearScale = GEAR_LOW_SCALE;
    currentGear = GEAR_LOW;
  }
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
DriveCommand  cachedJoyCmd = {0.0f, 0.0f};  // joystick axis command, post-gain/cap/rev-limit (#90)
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

  float xSpeed = cachedJoy.xSpeed * JOY_THROTTLE_GAIN;
  float zRotation = cachedJoy.zRotation;
  // Clamp throttle: per-gear joystick FORWARD cap, flat REVERSE_CAP backward —
  // both as track-output fractions converted to the xSpeed domain (#90/#87).
  // Throttle axis only; steering/pivot untouched. RC unaffected.
  float fwdCap = (currentGear == GEAR_LOW)  ? JOY_CAP_ECO
               : (currentGear == GEAR_HIGH) ? JOY_CAP_BOOST
                                            : JOY_CAP_NORMAL;
  xSpeed = constrain(xSpeed, -outCapToX(REVERSE_CAP), outCapToX(fwdCap));
  cachedJoyCmd = {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════

// Max/oppose combine for one axis (#90): the stronger same-direction input wins
// (NOT summed), opposing inputs subtract. result = max(a,b,0) + min(a,b,0).
//   same dir  60% & 30% → 60%   (larger wins, no halving)
//   opposite +100% & −30% → 70% (dominant minus opposing)
//   single    x & 0 → x         (full range — fixes the old 50/50 halving)
float maxOppose(float a, float b) {
  return fmaxf(fmaxf(a, b), 0.0f) + fminf(fminf(a, b), 0.0f);
}

// Combine RC + joystick at the AXIS level (xSpeed/zRotation), per override mode.
// curvatureDrive runs ONCE on the result (in loop), so a single operator always
// gets full range — the old code averaged the final wheel PWMs and halved it.
DriveCommand mixCommands(DriveCommand rc, int ovr, DriveCommand joy) {
  if (ovr < OVR_LO) {            // Mode 1 — RC only
    return rc;
  } else if (ovr > OVR_HI) {     // Mode 3 — dual: max/oppose per axis
    return { maxOppose(rc.xSpeed, joy.xSpeed),
             maxOppose(rc.zRotation, joy.zRotation) };
  } else {                        // Mode 2 — RC overrides joystick when RC active
    bool rcActive = (rc.xSpeed != 0.0f) || (rc.zRotation != 0.0f);
    return rcActive ? rc : joy;
  }
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

// Fail-safe output gate (#88 / #65). The Arduino drives PWM ONLY when it has a
// valid reason to (driveAllowed = RC valid AND battery OK). Otherwise:
//   1. EASE OUT — ramp both tracks smoothly from wherever they are down to
//      neutral over CUTOFF_HOLD_MS (gentle controlled stop, no jerk), then
//   2. STOP pulsing entirely (detach) so the ESCs see no signal and beep.
// Easing from the live command (never holding it) keeps it safe whatever the
// GL10 does on lost signal — by the time pulses stop, output is already neutral.
// RC-loss recovers automatically when the signal returns; the battery cutoff
// latch keeps driveAllowed false until a power-cycle. "Get a command → pass it.
// Get nothing → ease to a stop, then pass nothing." (CUTOFF_HOLD_MS is in [CONFIG].)
enum OutState { OUT_ACTIVE, OUT_HOLD, OUT_CUT };
OutState outState   = OUT_ACTIVE;
uint32_t outHoldMs  = 0;
int      rampFromL  = SVC;   // output captured when the gate closed (ease-out start)
int      rampFromR  = SVC;

void outputUpdate(bool driveAllowed, int mixL, int mixR) {
  uint32_t nowMs = millis();
  if (driveAllowed) {
    if (outState == OUT_CUT) {        // resume after an RC-loss cut
      escL.attach(PIN_ESC_L);
      escR.attach(PIN_ESC_R);
    }
    outState = OUT_ACTIVE;
    outputWrite(mixL, mixR);
    return;
  }
  // Disallowed: ease out to neutral from the live command, then cut the PWM.
  if (outState == OUT_ACTIVE) {
    outState = OUT_HOLD;
    outHoldMs = nowMs;
    rampFromL = outL;                 // ease out FROM wherever the tracks are
    rampFromR = outR;
  }
  if (outState == OUT_HOLD) {
    float t = (CUTOFF_HOLD_MS > 0) ? (float)(nowMs - outHoldMs) / (float)CUTOFF_HOLD_MS : 1.0f;
    if (t > 1.0f) t = 1.0f;
    outputWrite(rampFromL + (int)((SVC - rampFromL) * t),
                rampFromR + (int)((SVC - rampFromR) * t));
    if (t >= 1.0f) {
      escL.detach();                  // at neutral → stop pulsing → ESCs beep
      escR.detach();
      outState = OUT_CUT;
    }
  }
  // OUT_CUT: emit nothing (no writes while detached).
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
bool alarmOutputOn = false;                          // set each loop by [ALERT]; ORs onto D8
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
    if (millis() - beepPhaseMs >= beepSeq[beepIdx]) {
      beepIdx++;
      beepPhaseMs = millis();
    }
    patternOn = (beepIdx >= 0 && beepIdx < beepLen) && (beepIdx % 2 == 0);
  }
  // Horn (manual) ORs over one-shot patterns ORs over [ALERT] alarms.
  digitalWrite(PIN_BEEPER, (hornActive || patternOn || alarmOutputOn) ? HIGH : LOW);
}


// ═══════════════════════════════════════════════════════════════
// [ALERT] — battery + inactivity alarms layered onto the D8 piezo
// ═══════════════════════════════════════════════════════════════
// Audio only — NO motor-path impact (the low-voltage motor cutoff is PR #2 /
// issue #65). Picks the highest-priority active alarm and plays its repeating
// pattern via alarmOutputOn (OR'd onto D8 in beeperUpdate). The horn still
// sounds over any alarm.
//
//   Priority:  low-voltage (latched) > inactivity > (none)
//   Inactivity: RC transmitter off (sbusValid==false) > INACT_RC_OFF_MS.
//               Non-latching — clears when the RC comes back on.
//   Low-volt:   worst of the two packs' EMA voltage < LOWV_THRESH_V, but only
//               when BOTH ESCs report a plausible reading (power-sequencing:
//               a not-yet-powered pack reads ~0 V and must not false-alarm).
//               Debounced so an acceleration sag can't trip it; once latched it
//               beeps until power cycle even if voltage recovers.

uint32_t rcOffSinceMs   = 0;      // millis() when RC went off (0 = RC on)
uint32_t lowVStartMs    = 0;      // millis() when worst pack first dipped low (0 = above)
bool     lowVoltLatched = false;  // once true, stays until power cycle
uint32_t alertBootMs    = 0;      // set in setup() — startup-grace reference

// active repeating-alarm playback state
const uint16_t* alarmSeq = nullptr;
int      alarmLen = 0, alarmIdx = 0;
uint32_t alarmPhaseMs = 0;

void alertInit() { alertBootMs = millis(); }

// NOTE: [ALERT] below is AUDIO-ONLY — it drives the D8 piezo and never touches the
// motors. The motor-affecting battery cutoff lives in its own [SAFETY] section
// (search [SAFETY]); it only *borrows* this module's lowVoltLatched to start the
// chirp when it cuts.

// Call every loop. rcOn = sbusValid. Sets alarmOutputOn for the piezo.
void alertUpdate(bool rcOn) {
  uint32_t nowMs = millis();

  // --- inactivity: how long has the RC been off? ---
  if (rcOn)                    rcOffSinceMs = 0;
  else if (rcOffSinceMs == 0)  rcOffSinceMs = nowMs;
  bool inactiveAlarm = (rcOffSinceMs != 0) && (nowMs - rcOffSinceMs >= INACT_RC_OFF_MS);

  // --- low voltage: worst-of-two, validity-gated, debounced, latching ---
  if (!lowVoltLatched && (nowMs - alertBootMs >= ALERT_STARTUP_MS)) {
    float v0 = telem[0].voltage, v1 = telem[1].voltage;
    bool bothValid = telem[0].valid && telem[1].valid;
    bool plausible = (v0 >= LOWV_PLAUS_MIN_V && v0 <= LOWV_PLAUS_MAX_V &&
                      v1 >= LOWV_PLAUS_MIN_V && v1 <= LOWV_PLAUS_MAX_V);
    if (bothValid && plausible) {
      float worst = (v0 < v1) ? v0 : v1;
      if (worst < LOWV_THRESH_V) {
        if (lowVStartMs == 0) lowVStartMs = nowMs;
        if (nowMs - lowVStartMs >= LOWV_DEBOUNCE_MS) lowVoltLatched = true;
      } else {
        lowVStartMs = 0;   // recovered before debounce elapsed
      }
    } else {
      lowVStartMs = 0;     // can't measure both packs → reset debounce, stay silent
    }
  }

  // --- pick highest-priority alarm ---
  const uint16_t* seq = nullptr;
  int len = 0;
  if (lowVoltLatched) {
    seq = ALERT_LOWV;
    len = ALERT_LOWV_LEN;
  } else if (inactiveAlarm) {
    seq = ALERT_INACT;
    len = ALERT_INACT_LEN;
  }

  if (seq == nullptr) {
    alarmSeq = nullptr;
    alarmOutputOn = false;
    return;
  }

  // (re)start playback when the active alarm changes
  if (seq != alarmSeq) {
    alarmSeq = seq;
    alarmLen = len;
    alarmIdx = 0;
    alarmPhaseMs = nowMs;
  }

  // advance the repeating pattern (loops, unlike the one-shot beepStart)
  if (nowMs - alarmPhaseMs >= alarmSeq[alarmIdx]) {
    alarmIdx = (alarmIdx + 1) % alarmLen;
    alarmPhaseMs = nowMs;
  }
  alarmOutputOn = (alarmIdx % 2 == 0);   // even index = ON phase
}


// ═══════════════════════════════════════════════════════════════
// [SAFETY] — staged low-battery protection (#65)
// ═══════════════════════════════════════════════════════════════
// MOTOR-AFFECTING (unlike [ALERT], which is audio-only). Two latched stages off
// the worst-of-two pack EMA, validity-gated + debounced, NO startup grace (the
// validity gate is the only warm-up guard). Both latch until power-cycle:
//   Stage 1 — Eco lock   (≤ ECO_LOCK_THRESH_V ~11.0 V): updateGear() forces Eco
//             regardless of the RC gear switch, to ease load on a draining pack.
//   Stage 2 — Hard cutoff (≤ CUTOFF_THRESH_V 10.0 V): the output gate stops the
//             motors, and we assert lowVoltLatched so the D8 alarm chirps WITH
//             the cut — never a silent cutoff.

uint32_t cutoffStartMs  = 0;
uint32_t ecoLockStartMs = 0;

// Worst-of-two pack voltage if BOTH packs read a plausible value; else false
// (a not-yet-powered pack reads ~0 V and must not trip anything).
bool worstPackVoltage(float* worst) {
  float v0 = telem[0].voltage, v1 = telem[1].voltage;
  if (!(telem[0].valid && telem[1].valid)) return false;
  if (v0 < LOWV_PLAUS_MIN_V || v0 > LOWV_PLAUS_MAX_V ||
      v1 < LOWV_PLAUS_MIN_V || v1 > LOWV_PLAUS_MAX_V) return false;
  *worst = (v0 < v1) ? v0 : v1;
  return true;
}

void batteryEcoLockUpdate() {        // Stage 1 — force Eco
  if (ecoLockLatched) return;
  float worst;
  if (!worstPackVoltage(&worst)) {
    ecoLockStartMs = 0;
    return;
  }
  if (worst < ECO_LOCK_THRESH_V) {
    uint32_t nowMs = millis();
    if (ecoLockStartMs == 0) ecoLockStartMs = nowMs;
    if (nowMs - ecoLockStartMs >= ECO_LOCK_DEBOUNCE_MS) ecoLockLatched = true;
  } else {
    ecoLockStartMs = 0;              // recovered before debounce elapsed
  }
}

void batteryCutoffUpdate() {         // Stage 2 — hard cutoff
  if (batteryCutoffLatched) return;
  float worst;
  if (!worstPackVoltage(&worst)) {
    cutoffStartMs = 0;
    return;
  }
  if (worst >= CUTOFF_THRESH_V) batteryOkConfirmed = true;  // boot gate: pack confirmed above cutoff
  if (worst < CUTOFF_THRESH_V) {
    uint32_t nowMs = millis();
    if (cutoffStartMs == 0) cutoffStartMs = nowMs;
    if (nowMs - cutoffStartMs >= CUTOFF_DEBOUNCE_MS) {
      batteryCutoffLatched = true;
      lowVoltLatched = true;         // start the D8 alarm WITH the cut (no silent cutoff)
    }
  } else {
    cutoffStartMs = 0;               // recovered before debounce elapsed
  }
}


// ═══════════════════════════════════════════════════════════════
// [WIFI] — AP + HTTP telemetry server (WiFiS3, stock UNO R4 WiFi)
// ═══════════════════════════════════════════════════════════════
// Hosts a Wi-Fi access point and streams telemetry to the dashboard
// (dashboard/index.html). MONITORING ONLY — no control inputs are ever
// accepted over Wi-Fi (safety).
//
// SAFETY (#69): serving must NEVER starve the control loop. Originally the
// ~33 KB page was sent as one blocking burst (~1-2 s), during which loop()
// froze while the Servo PWM hardware kept emitting the last throttle → a
// runaway. Two defenses now:
//   1. wifiUpdate() does AT MOST ONE modem write per loop pass (one page
//      chunk OR one request OR one SSE frame), so control + failsafe run
//      between every chunk and the loop is never blocked for long.
//   2. The hardware watchdog (WDT_TIMEOUT_MS, armed in setup) resets the MCU
//      if the loop is ever stalled past the timeout regardless of cause —
//      PWM stops and the ESCs go to neutral. Backstop, not the primary fix.

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
bool           sseActive = false;   // we are holding a live SSE socket (#77 reaping)

// Incremental dashboard transfer (#69): the ~33 KB page is sent ONE
// WIFI_PAGE_CHUNK per loop pass, not in a single blocking burst, so the control
// path + failsafe + watchdog refresh run between chunks and the loop is never
// starved. pageRemaining > 0 means a transfer is in flight.
WiFiClient     pageClient;
const char    *pagePtr = nullptr;
size_t         pageRemaining = 0;

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
    beepStart(BEEP_WIFI_READY, BEEP_WIFI_READY_LEN);   // "beep beep" — Wi-Fi AP is up/ready
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
    "\"eco\":%d,\"cut\":%d,\"outL\":%d,\"outR\":%d,"
    "\"e0\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d},"
    "\"e1\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d}}",
    (unsigned long)nowMs, (unsigned long)wifiSeq, (int)currentGear, wifiMode(),
    sbusData.failsafe ? 1 : 0, sbusData.lost_frame ? 1 : 0,
    ecoLockLatched ? 1 : 0, batteryCutoffLatched ? 1 : 0,
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

// One-shot /data JSON. Header + body coalesced into a single write() so the
// whole response is one modem round-trip (one bounded op this loop pass).
void wifiSendData(WiFiClient &client) {
  char body[360];
  int n = buildTelemJson(body, sizeof(body));
  char buf[512];
  int h = snprintf(buf, sizeof(buf),
    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
    "Access-Control-Allow-Origin: *\r\nConnection: close\r\nContent-Length: %d\r\n\r\n", n);
  if (h > 0 && (size_t)(h + n) < sizeof(buf)) {
    memcpy(buf + h, body, n);
    client.write(reinterpret_cast<const uint8_t *>(buf), h + n);
  }
}

// Serve the embedded dashboard. The page is static, so it carries an ETag and
// Cache-Control: no-cache (store-but-revalidate): the first visit downloads it
// once, then every refresh sends If-None-Match and gets a tiny 304 here instead
// of re-streaming ~33 KB through the blocking Wi-Fi modem (issue #54). Live
// values arrive separately over SSE, so the HTML itself never reloads in normal
// use.
//
// The 200 body is NOT sent here in one burst — that single blocking burst was
// the #69 runaway root cause (it froze loop() for ~1-2 s while the ESC held the
// last throttle). Instead wifiBeginPage() sends only the headers (one coalesced
// write) and hands the body to the incremental sender in wifiUpdate(), which
// ships ONE WIFI_PAGE_CHUNK per loop pass so control + failsafe run between
// chunks.
void wifiBeginPage(WiFiClient &client) {
  size_t len = strlen(INDEX_HTML);
  char hdr[160];
  int h = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    "Cache-Control: no-cache\r\nETag: %s\r\nConnection: close\r\nContent-Length: %u\r\n\r\n",
    pageEtag, (unsigned)len);
  // snprintf returns <0 on error or >= size if truncated; only proceed with a
  // valid, fully-formed header. Otherwise abort (don't start a body transfer
  // behind a partial/garbage header).
  if (h <= 0 || h >= (int)sizeof(hdr)) {
    client.stop();
    return;
  }
  client.write(reinterpret_cast<const uint8_t *>(hdr), h);
  pageClient = client;          // refcounted handle survives the local going out of scope
  pagePtr = INDEX_HTML;
  pageRemaining = len;          // body streams over the next passes (branch A of wifiUpdate)
}

// 304 Not Modified for a cached dashboard — tiny, single coalesced write.
void wifiSend304(WiFiClient &client) {
  char hdr[120];
  int h = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 304 Not Modified\r\nETag: %s\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
    pageEtag);
  // Guard the snprintf return (could be <0 or truncated) before using it as a length.
  if (h > 0 && h < (int)sizeof(hdr)) {
    client.write(reinterpret_cast<const uint8_t *>(hdr), h);
  }
}

// Non-blocking, and bounded to AT MOST ONE modem write per loop pass (#69):
// either one page chunk, OR one request/response, OR one SSE frame. The control
// path + WDT.refresh() run every pass between these, so Wi-Fi can never starve
// the loop. Each modem call is also individually capped at WIFI_MODEM_TIMEOUT_MS
// (vs the WiFiS3 default 10 000 ms) so a single stalled TCP window can't freeze
// loop() for seconds (issue #54).
void wifiUpdate() {
  if (!wifiUp) return;

  modem.timeout(WIFI_MODEM_TIMEOUT_MS);

  // (A) A page transfer is in flight → send exactly ONE chunk and yield. Control
  // + failsafe + watchdog refresh run before we get back here next pass.
  if (pageRemaining > 0) {
    if (!pageClient.connected()) {
      // client went away mid-transfer
      pageClient.stop();
      pagePtr = nullptr;
      pageRemaining = 0;
    } else {
      size_t chunk = pageRemaining > WIFI_PAGE_CHUNK ? WIFI_PAGE_CHUNK : pageRemaining;
      size_t w = pageClient.write(reinterpret_cast<const uint8_t *>(pagePtr), chunk);
      if (w == 0) {
        // write stalled (modem timeout) — abort the transfer instead of spinning
        // forever: a 0-byte write makes no progress, so without this the page
        // never completes and SSE telemetry is starved one pass at a time.
        pageClient.stop();
        pagePtr = nullptr;
        pageRemaining = 0;
      } else {
        pagePtr += w;
        pageRemaining -= w;
        if (pageRemaining == 0) {
          pageClient.flush();
          pageClient.stop();
          pagePtr = nullptr;
        }
      }
    }
    modem.timeout(MODEM_TIMEOUT);
    return;
  }

  // (B) Otherwise accept at most one new client this pass.
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
      // Upgrade to a persistent Server-Sent Events stream. ALWAYS free any
      // previous SSE socket first — even a dead/half-open one — so its ESP32
      // link id is released (AT+CIPCLOSE) instead of leaking. The old code only
      // closed it when still connected(), so a client that dropped Wi-Fi without
      // a clean close leaked a link id every reconnect until the ~5-socket pool
      // was exhausted and the server could accept nothing (#77).
      sseClient.stop();
      sseClient = client;
      sseActive = true;
      sseClient.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                        "Cache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n\r\n"));
      sseLastMs = 0;                                   // push first frame immediately
    } else if (strstr(req, "/data")) {
      wifiSendData(client); client.flush(); client.stop();
    } else {
      // Static dashboard. If the browser already holds our ETag, reply 304 and
      // skip the ~33 KB transfer entirely (issue #54); otherwise send the headers
      // now and stream the body incrementally over the next passes (branch A).
      bool cached = strstr(req, "If-None-Match") && strstr(req, pageEtag);
      if (cached) {
        wifiSend304(client);
        client.flush();
        client.stop();
      } else {
        wifiBeginPage(client);     // body streams over the next passes (branch A)
      }
    }
    modem.timeout(MODEM_TIMEOUT);
    return;                                            // one modem op done this pass
  }

  // (C) Idle → push at most one SSE telemetry frame, AND proactively reap the
  // socket the instant it dies so a dropped client can't leak its ESP32 link id
  // and exhaust the ~5-socket pool (#77). The push starts with a ": hb\n" comment
  // (a no-op for EventSource) that exercises the TCP socket so Safari/iOS doesn't
  // park the connection in a stalled state.
  if (sseActive) {
    if (!sseClient.connected()) {
      sseClient.stop();          // peer gone (e.g. Wi-Fi dropped) → free the link id NOW
      sseActive = false;
    } else {
      uint32_t now = millis();
      if (now - sseLastMs >= SSE_INTERVAL_MS) {
        sseLastMs = now;
        // Build the whole SSE frame — heartbeat comment, data line, terminator —
        // into one buffer and ship it in a SINGLE write() (one AT round-trip vs
        // three, issue #54). SSE_FRAME_CAP holds the 11-byte prefix + JSON + "\n\n".
        char frame[SSE_FRAME_CAP];
        int len = snprintf(frame, sizeof(frame), ": hb\ndata: ");
        // Guard the snprintf return before using it as an offset (consistent with
        // wifiBeginPage/wifiSend304). The fixed 11-byte prefix can't really
        // truncate, but this keeps the frame+len / cap math provably in-bounds.
        if (len > 0 && len < (int)sizeof(frame) - 2) {
          // Reserve the last 2 bytes for the "\n\n" terminator so the JSON body
          // can never crowd it out; buildTelemJson clamps to the cap we pass.
          len += buildTelemJson(frame + len, sizeof(frame) - len - 2);
          frame[len++] = '\n';
          frame[len++] = '\n';
          size_t w = sseClient.write(reinterpret_cast<const uint8_t *>(frame), len);
          if (w == 0) {          // 0-byte write = dead socket → reap immediately (#77)
            sseClient.stop();
            sseActive = false;
          }
        }
      }
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
    Serial.println("# === Digger V7.14 — GL10 FOC + S.BUS + Gear + X.BUS telem + Wi-Fi AP + beeper/alarms + loop watchdog + smooth pivot/headroom ===");
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
  alertInit();
  debugInit();
  Serial1.begin(115200);  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
  wifiInit();             // Wi-Fi AP + telemetry server (monitoring only)

  // Arm the hardware watchdog LAST — after the slow Wi-Fi AP bring-up — so init
  // can't trip it. From here, loop() must call WDT.refresh() within
  // WDT_TIMEOUT_MS or the MCU resets: PWM stops, the ESCs see no signal and go
  // to neutral/failsafe, and the machine stops instead of holding the last
  // throttle. This is the runaway backstop for ANY loop stall (#69).
  if (WDT.begin(WDT_TIMEOUT_MS)) {
    if (Serial) {
      Serial.print("# WDT armed @ ");
      Serial.print(WDT.getTimeout());
      Serial.println(" ms");
    }
  } else if (Serial) {
    Serial.println("# WDT FAILED to arm — no loop-stall backstop!");
  }
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

  // 1.5 Staged low-battery protection (#65) — evaluate BEFORE gear select so the
  // Eco lock can override it. Stage 1 (~11 V) forces Eco; Stage 2 (10 V) cuts.
  batteryEcoLockUpdate();
  batteryCutoffUpdate();
  updateGear();              // honors ecoLockLatched (forces Eco when set)
  updateJoystick(now);

  // 2. Compute the drive mix (meaningful only when RC is valid).
  ServoOutput mix;
  if (sbusValid) {
    // Combine RC + joystick at the axis level (#90), then run curvatureDrive once
    // on the combined command so a single operator keeps full range.
    DriveCommand cmd = mixCommands(rcCommand(), rcOverride(), cachedJoyCmd);
    mix = wheelSpeedsToServo(curvatureDrive(cmd.xSpeed, cmd.zRotation, gearScale));
  } else {
    mix.left = SVC;  mix.right = SVC;
  }

  // 3. Fail-safe output gate (#88 / #65) — drive ONLY when RC is valid AND the
  // battery is above the cutoff (latched in step 1.5). Otherwise command neutral
  // (GL10 decelerates smoothly) then cut PWM so the ESCs lose signal and beep.
  // RC-loss recovers; the battery cutoff latches until power-cycle. GL10's
  // internal accel/drag is the only command smoothing (no Arduino-side ramp).
  // Boot gate (#65): don't drive until a valid reading confirms the pack is above
  // the cutoff — so a low pack can't drive in the brief window after a watchdog
  // reset wipes the RAM latch. Fail OPEN if telemetry never reports
  // (BATTERY_CONFIRM_MS) so a dead X.BUS can't permanently disable driving.
  bool batteryReady = batteryOkConfirmed || (millis() - alertBootMs > BATTERY_CONFIRM_MS);
  bool driveAllowed = sbusValid && batteryReady && !batteryCutoffLatched;
  outputUpdate(driveAllowed, mix.left, mix.right);

  // Control path serviced this pass (inputs read + output gate run) — and ONLY
  // now do we kick the watchdog. This is the sole refresh point: if anything
  // below (telemetry/Wi-Fi) ever stalls the loop beyond WDT_TIMEOUT_MS, the
  // refresh is missed, the MCU resets, and the machine stops instead of holding
  // the last throttle command (#69).
  WDT.refresh();

  // 3.5 Telemetry — non-blocking X.BUS Read Register (0x10), read-only.
  // Never enters BUS_MODE, so it cannot affect the control output above.
  telemUpdate();

  // 3.6 Wi-Fi — serve telemetry JSON to the dashboard (monitoring only).
  wifiUpdate();
  wifiDebug(now);
  alertUpdate(sbusValid);  // battery + inactivity alarms → alarmOutputOn (audio only)
  beeperUpdate();          // horn (RC SWD) + queued pattern + [ALERT] alarm (D8)

  // 4. Debug
  debugPrint(now);
}
