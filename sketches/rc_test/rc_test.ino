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
#include <WiFiS3.h>
#include <WDT.h>        // RA4M1 hardware watchdog — control-loop runaway backstop (#69)
#include "types.h"
#include "web_page.h"   // const char INDEX_HTML[] — the dashboard, served at "/"

// Phase D extraction (#117): pure extracted logic lives under src/ (domain/
// and observer layers) and is included here directly until the FirmwareApp
// composition root lands (migration window, #150).
#include "src/domain/battery/VoltagePlausibility.h"
#include "src/domain/battery/BatteryLadder.h"
#include "src/domain/thermal/ThermalHysteresis.h"
#include "src/domain/thermal/ThermalDerating.h"
#include "src/domain/operator_input/ExpoCurve.h"
#include "src/domain/operator_input/DeadbandPolicy.h"
#include "src/domain/drive/CurvatureDrive.h"
#include "src/domain/drive/GearPolicy.h"
#include "src/domain/drive/CommandMixer.h"
#include "src/domain/safety/SafetySupervisor.h"
#include "src/domain/safety/OutputGate.h"
#include "src/application/SystemSnapshot.h"
#include "src/telemetry/JsonEncoder.h"
#include "src/telemetry/CsvEncoder.h"
#include "src/ports/EscOutputPort.h"
#include "src/ports/JoystickPort.h"
#include "src/ports/AlertOutputPort.h"
#include "src/ports/RcInputPort.h"
#include "src/ports/TelemetrySource.h"


// The second hardware UART (SCI0, D11/D12), the S.BUS parser and their pin
// constants moved to infrastructure/radiolink/SbusReceiverAdapter.cpp
// behind ports/RcInputPort.h (#117 step 10 slice 3, #176).


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — All tunable constants
// ═══════════════════════════════════════════════════════════════

// Firmware version — SINGLE SOURCE OF TRUTH (#124). The debug banner prints
// it; docs and FIRMWARE-UPLOAD-LOG reference it. Bump here, nowhere else.
const char FIRMWARE_VERSION[] = "V7.35";

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
// Motor/ESC over-temp patterns (#111) — chosen to be unmistakable by ear vs the
// four above (2 short once / 1 long / 3 short fast / continuous horn):
const uint16_t ALERT_THERM_WARN[]   = {100, 100};                      // fast nonstop TRILL (>= 80 C)
const int      ALERT_THERM_WARN_LEN = 2;
const uint16_t ALERT_THERM_CUT[]    = {500, 150, 500, 150, 500, 1000};  // three LONG beeps (>= 95 C cut)
const int      ALERT_THERM_CUT_LEN  = 6;
const uint16_t BEEP_THERM_RESTORED[]   = {120, 100, 120, 100, 120, 100, 120, 1500};  // 4 quick flourish, one-shot (< 75 C)
const int      BEEP_THERM_RESTORED_LEN = 8;

// [ALERT] tunables
const uint32_t INACT_RC_OFF_MS  = 60000UL;  // RC off this long → inactivity beep ("unplug me")
const float    LOWV_THRESH_V    = 10.5f;   // worst-of-two pack EMA below this → low-batt alarm (heads-up beep before the 10.0 V cutoff)
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
// Latches until power-cycle. 10.8 V ≈ 30% on a 3S pack — the LiPo "knee", so the
// top ~70% keeps full Boost/Normal and Eco only eases the final steep stretch.
// (Ladder: 10.8 V → Eco lock (15 s) · 10.5 V → beep · 10.0 V → hard cutoff.)
const float    ECO_LOCK_THRESH_V    = 10.8f;    // worst pack EMA below this → force Eco
const uint32_t ECO_LOCK_DEBOUNCE_MS = 15000UL;  // sustained below thresh before locking Eco

// [SAFETY] motor / ESC over-temperature protection (#111) — staged, NON-LATCHING
// with hysteresis. Heat is the OPPOSITE of a drained LiPo: once it cools, driving
// must resume automatically, so each stage releases on its own (lower) threshold
// rather than latching to power-cycle like the battery ladder. Source = HOTTEST
// of all 4 sensors (ESC + motor temp on BOTH ESCs), already EMA-smoothed
// (TELEMETRY_EMA_ALPHA_TEMPERATURE) + a short trip debounce so a single spike / dropped frame can't
// flip a stage. A telemetry dropout (no fresh valid reading) HOLDS the last state
// — it never triggers a cut.
//   Warning beep  >= 80 C  (release < 78 C) : trill, motors keep running
//   Eco lock      >= 90 C  (release < 80 C) : force Eco gear
//   Hard cutoff   >= 95 C  (release < 75 C) : ease PWM to neutral, keep beeping
// NOTE: 95 C is PROVISIONAL — confirm on the bench that it sits just BELOW the
// GL10's own internal thermal limit (param 17 = temp-controlled fan; the GL10's
// internal throttle/cutoff number is unpublished) so our warned graceful cut
// fires before the ESC's silent one.
const float    TEMP_WARN_ON_C    = 80.0f;
const float    TEMP_WARN_OFF_C   = 78.0f;
const float    TEMP_ECO_ON_C     = 90.0f;
const float    TEMP_ECO_OFF_C    = 80.0f;
const float    TEMP_CUT_ON_C     = 95.0f;
const float    TEMP_CUT_OFF_C    = 75.0f;
const uint32_t TEMP_DEBOUNCE_MS  = 1000UL;   // sustained past a TRIP threshold before the stage flips on
const float    TEMP_PLAUS_MIN_C  = -20.0f;   // reading below this = bad / unplugged sensor → ignore
const float    TEMP_PLAUS_MAX_C  = 200.0f;   // reading above this = bad read → ignore

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

// REVERSE caps (#87/#113) — per gear, flat across BOTH inputs (RC + joystick).
// Reverse is held below forward authority; Boost is allowed a little more reverse
// than Eco/Normal. Applied through reverseCap() (defined with the other gear-cap
// helpers below) so there is ONE source of truth, not a scattered conditional.
// These are TRUE percentages only because the GL10s were recalibrated to the full
// 1000/1500/2000 us range on 2026-07-03. Previously they were calibrated while the
// firmware capped reverse at 65%, so they mislearned 65% as 100% reverse — which
// made a 65%-commanded reverse drive ~100%. Do NOT lower the firmware reverse cap
// and recalibrate at the same time, or that mislearning returns.
const float REVERSE_CAP_STD   = 0.55f;  // Eco + Normal
const float REVERSE_CAP_BOOST = 0.65f;  // Boost

// Power range — full PWM authority (1000-2000 us = ±500 us from SVC)
const float SOFT_RANGE = 500.0f;  // Max servo offset from center (us)

// ── ESC THROTTLE-CALIBRATION MODE (#113) ───────────────────────────────────
// TEMPORARY. When true, the throttle stick passes STRAIGHT THROUGH to the full
// ±100% PWM range (1000 / 1500 / 2000 us) on BOTH tracks — ALL caps, gear scaling
// and steering are bypassed — so the GL10s can learn the Arduino's TRUE endpoints
// (Option A, docs/GL10-OPERATION.md §5). Needed because the ESCs were previously
// calibrated while the firmware capped reverse at 65%, so they mislearned 65% as
// their 100% reverse endpoint (that is why 65%-commanded reverse drove ~100%).
// After BOTH ESCs are recalibrated, set this back to false and reflash the normal
// firmware (where REVERSE_CAP etc. become TRUE percentages again).
// SAFETY: motors reach full power in this mode — tracks clear / wheels up.
const bool CALIBRATION_MODE = false;

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
// (Reverse is capped per gear via reverseCap(): 55% Eco/Normal, 65% Boost.)
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

// [SAFETY] motor over-temp stage flags (#111) — NON-LATCHING (hysteresis). Set by
// thermalUpdate() off the hottest of all 4 sensors; read by updateGear() (Eco),
// the output gate (cut), alertUpdate() (beeps), and buildTelemJson() (dashboard).
bool tempWarnActive = false;  // hottest sensor >= TEMP_WARN_ON_C → warning trill
bool tempEcoActive  = false;  // hottest sensor >= TEMP_ECO_ON_C  → force Eco gear
bool tempCutActive  = false;  // hottest sensor >= TEMP_CUT_ON_C  → cut motors (auto-recovers)

// The cap helpers and gear decision tree are extracted to
// src/domain/drive/GearPolicy.* (#117 step 7, #166); these delegates keep
// every call site unchanged and own the global reads (gearScale,
// currentGear) visibly. The domain GearLevel enum mirrors types.h's Gear
// values — proven at compile time:
static_assert((int)GEAR_LOW == (int)GEAR_ECO &&
              (int)GEAR_MID == (int)GEAR_NORMAL &&
              (int)GEAR_HIGH == (int)GEAR_BOOST,
              "types.h Gear and domain GearLevel values must correspond");

// Convert a MAX TRACK-OUTPUT cap (0..1) to the xSpeed domain for the current
// gear (output = xSpeed * gearScale). Single point of truth for every cap.
inline float outCapToX(float outCap) {
  return trackCapToAxisDomain(outCap, gearScale);
}

// Per-gear reverse cap (#113): Eco/Normal hold 55%, Boost allows 65%. Single
// source of truth — both rcCommand() and the joystick clamp read this, so the
// reverse limit lives in exactly one place.
inline float reverseCap() {
  return reverseCapForGear((GearLevel)currentGear, REVERSE_CAP_STD,
                           REVERSE_CAP_BOOST);
}

// Curvature drive — pivot/curvature blend band.
// |xSpeed| <= START: pure pivot (counter-rotate at PIVOT_SPEED_CAP)
// |xSpeed| >= END:   pure curvature (outer holds the average, inner slows)
// Between: smoothstep blend so the operator doesn't feel a mode jump. The band
// is WIDE (0.05–0.55) so the pivot↔forward/reverse hand-off is gradual — a
// narrow band made the transition snap near 15–20% throttle (#72).
const float PIVOT_BLEND_START = 0.05f;
const float PIVOT_BLEND_END   = 0.55f;
const float PIVOT_SPEED_CAP   = 0.60f;  // pivot rotation cap (~60% wheel power)

// Pivot-branch throttle taper (#114). Throttle used to enter BOTH tracks of the
// pivot branch at full gain, so 10–20% throttle while steering shifted both
// tracks forward together — an instant surge (mirrored in reverse). Taper the
// pivot branch's throttle term by steering: while turning, the outer track
// keeps most of its pivot speed (it still gains (1−taper)·throttle) and the
// inner keeps a slight counter-rotation that eases through zero as throttle
// rises; forward speed then builds as the blend hands over to the curvature
// branch. The taper deliberately follows the RAW steering stick, not
// cappedRotation: past the pivot cap extra deflection adds no rotation, but it
// still reads as "hold the pivot", so full lock holds hardest (2026-07-03,
// field-validated — see docs/DECISION-LOG.md). 0.0 = old behavior, 1.0 =
// throttle fully suppressed at full steer. Straight-line (z = 0), pure pivot
// (x = 0), and at-speed turns (|xSpeed| >= PIVOT_BLEND_END → pure curvature)
// are mathematically unchanged. Field-tune WITHIN [0, 1] — enforced below
// because above 1.0 the term flips sign (forward stick would command reverse).
constexpr float PIVOT_THROTTLE_TAPER = 0.70f;
static_assert(PIVOT_THROTTLE_TAPER >= 0.0f && PIVOT_THROTTLE_TAPER <= 1.0f,
              "PIVOT_THROTTLE_TAPER outside [0,1] inverts pivot-branch throttle");

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
const size_t   SSE_FRAME_CAP         = 448;  // SSE frame buffer: ": hb\ndata: " + JSON + "\n\n"

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

// The curvature mix is extracted to src/domain/drive/CurvatureDrive.* (#117
// step 6, #164) — the algorithm and its commentary (#72/#86/#96/#114) moved
// with it. This delegate keeps the call sites and every test reference
// unchanged, and owns the one dependency the domain must not read: the
// gear-selected pivot cap (state-ownership rule — currentGear is read HERE,
// visibly, instead of hidden behind the parameter list).
WheelSpeeds curvatureDrive(float xSpeed, float zRotation, float gearScale) {
  float pivotCap = (currentGear == GEAR_LOW) ? PIVOT_SPEED_CAP_LOW
                                             : PIVOT_SPEED_CAP;
  TrackCommand command = curvatureDriveStep(
      xSpeed, zRotation, gearScale,
      CurvatureParameters{pivotCap, PIVOT_THROTTLE_TAPER, PIVOT_BLEND_START,
                          PIVOT_BLEND_END, TURN_TRACK_CAP});
  return {command.left, command.right};
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

RcFrame rcFrame = {};                    // latest received frame (domain type)
bool sbusValid = false;
uint32_t sbusLastFrame = 0;
const uint32_t SBUS_TIMEOUT = 100000UL;  // 100ms

int sbusToServo(int raw) {
  return map(constrain(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
}

// Deadband math extracted to src/domain/operator_input/DeadbandPolicy.*
// (#117 step 5, #162); this delegate keeps every call site unchanged.
int rcDeadband(int pw) {
  return centerSnapDeadband(pw, SVC, RC_DEADBAND);
}

int rcThrottle() { return sbusValid ? rcDeadband(sbusToServo(rcFrame.channels[SBUS_CH_THR]))   : SVC; }
int rcSteering() { return sbusValid ? rcDeadband(sbusToServo(rcFrame.channels[SBUS_CH_STEER])) : SVC; }
int rcOverride() { return sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_OVR]) : SVMIN; }

// RC axis command (xSpeed/zRotation), post-gain and reverse-limit — pre-mix,
// pre-curvatureDrive. The mix combines RC + joystick at this axis level (#90),
// then curvatureDrive runs once on the combined command.
DriveCommand rcCommand() {
  float xSpeed    = (float)(rcThrottle() - SVC) / SOFT_RANGE;
  float zRotation = (float)(rcSteering() - SVC) / SOFT_RANGE;
  // Apply tunable input gains, then clamp: RC keeps the gear's full forward
  // authority (1.0); reverse is the per-gear reverseCap() (55% Eco/Normal, 65%
  // Boost) (#87/#113).
  xSpeed    = constrain(xSpeed * RC_THROTTLE_GAIN, -outCapToX(reverseCap()), 1.0f);
  zRotation = constrain(zRotation * RC_STEERING_GAIN, -1.0f, 1.0f);
  return {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [GEAR] — RC CH4 → average-speed cap (Eco 65% / Normal 80% / Boost 100%)
// ═══════════════════════════════════════════════════════════════
//
// updateGear() is defined here (after [RC]) so it can read sbusValid /
// rcFrame directly. The gearScale and currentGear globals it writes
// are declared up in [CONFIG] so the drive functions and curvatureDrive
// can read them for the Eco-only conditional caps.

// The decision tree is extracted to src/domain/drive/GearPolicy.* (#117
// step 7, #166); this delegate owns the boundary reads — sbus CH4 and the
// Eco forces (battery Eco lock #65 latched OR thermal Eco #111
// non-latching, which clears once the motor cools below TEMP_ECO_OFF_C) —
// and mirrors the gearScale/currentGear globals every caller reads.
void updateGear() {
  // Built once — the values are [CONFIG] constants (loop() calls this every
  // pass, so no per-pass aggregate temporary).
  static const GearPolicyParameters GEAR_POLICY_PARAMETERS{
      GEAR_LOW_SCALE, GEAR_MID_SCALE, GEAR_HIGH_SCALE, OVR_LO, OVR_HI};
  GearSelection selection = gearPolicySelect(
      sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_GEAR]) : 0, sbusValid,
      ecoLockLatched || tempEcoActive, GEAR_POLICY_PARAMETERS);
  gearScale = selection.scale;
  currentGear = (Gear)selection.level;
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve
// ═══════════════════════════════════════════════════════════════

// expoCurve() now lives in src/domain/operator_input/ExpoCurve.* (#117
// step 5, #162) — same name and signature, so every call site (and the
// characterization tests) resolves to the domain implementation directly.

int joyDeadband(int adc) {
  return centerSnapDeadband(adc, ADC_CENTER, JOY_DEADBAND);
}

JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, 0.0f, 0.0f};
DriveCommand  cachedJoyCmd = {0.0f, 0.0f};  // joystick axis command, post-gain/cap/rev-limit (#90)
uint32_t      lastAdcTime = 0;
const uint32_t ADC_INTERVAL = 10000UL;  // 10 ms = 100 Hz

void updateJoystick(uint32_t now) {
  if ((now - lastAdcTime) < ADC_INTERVAL) return;
  lastAdcTime = now;

  // The ADC conditioning sequence (discard-read, 100 µs settle, real read)
  // lives in infrastructure/arduino/AdcJoystickAdapter.cpp behind
  // ports/JoystickPort.h (#117 step 10 slice 2, #174).
  joystickReadAxes(PIN_JOY_Y, PIN_JOY_X, &cachedJoy.rawY, &cachedJoy.rawX);

  float normY = constrain((float)(joyDeadband(cachedJoy.rawY) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float normX = constrain((float)(joyDeadband(cachedJoy.rawX) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float signY = (normY >= 0) ? 1.0f : -1.0f;
  float signX = (normX >= 0) ? 1.0f : -1.0f;
  cachedJoy.xSpeed    = signY * expoCurve(normY, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC);
  cachedJoy.zRotation = JOY_STEER_DIR * signX * expoCurve(normX, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC);

  float xSpeed = cachedJoy.xSpeed * JOY_THROTTLE_GAIN;
  float zRotation = cachedJoy.zRotation;
  // Clamp throttle: per-gear joystick FORWARD cap, per-gear reverseCap() backward —
  // both as track-output fractions converted to the xSpeed domain (#90/#87/#113).
  // Throttle axis only; steering/pivot untouched. RC unaffected.
  float fwdCap = (currentGear == GEAR_LOW)  ? JOY_CAP_ECO
               : (currentGear == GEAR_HIGH) ? JOY_CAP_BOOST
                                            : JOY_CAP_NORMAL;
  xSpeed = constrain(xSpeed, -outCapToX(reverseCap()), outCapToX(fwdCap));
  cachedJoyCmd = {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════

// maxOppose() (#90) and the mode selection are extracted to
// src/domain/drive/CommandMixer.* (#117 step 7, #166) — maxOppose keeps its
// name and signature, so call sites and tests resolve to the domain
// directly. This delegate converts DriveCommand ↔ AxisCommand
// (layout-identical) and passes the [CONFIG] thresholds. curvatureDrive
// runs ONCE on the result (in loop), so a single operator always gets full
// range — the old code averaged the final wheel PWMs and halved it.
DriveCommand mixCommands(DriveCommand rc, int ovr, DriveCommand joy) {
  AxisCommand mixed = mixAxisCommands(AxisCommand{rc.xSpeed, rc.zRotation},
                                      AxisCommand{joy.xSpeed, joy.zRotation},
                                      ovr, OVR_LO, OVR_HI);
  return {mixed.xSpeed, mixed.zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [OUTPUT] — ESC servo PWM (non-blocking)
// ═══════════════════════════════════════════════════════════════

// The Servo objects moved to infrastructure/arduino/PwmEscAdapter.cpp
// behind ports/EscOutputPort.h (#117 step 10 slice 1, #172). The clamp and
// the outL/outR globals stay HERE — application-visible output state (read
// by buildTelemJson and the OutputGate shim, reset by the test harness).
int outL = SVC, outR = SVC;

void outputInit() {
  escOutputInitialize(PIN_ESC_L, PIN_ESC_R);
  escOutputWritePulses(SVC, SVC);
}

void outputWrite(int left, int right) {
  outL = constrain(left,  SVMIN, SVMAX);
  outR = constrain(right, SVMIN, SVMAX);
  escOutputWritePulses(outL, outR);
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

// The state machine is extracted to src/domain/safety/OutputGate.* (#117
// step 9, #170); this delegate mirrors the gate globals, reads the clock,
// and executes the returned hardware actions in the original order:
// attach, then write, then detach (the final neutral write precedes the
// detach, so the ESCs see neutral before losing signal). The enum values
// correspond — proven at compile time:
static_assert((int)OUT_ACTIVE == (int)OUTPUT_GATE_ACTIVE &&
              (int)OUT_HOLD == (int)OUTPUT_GATE_HOLD &&
              (int)OUT_CUT == (int)OUTPUT_GATE_CUT,
              "OutState and domain OutputGateMode values must correspond");

void outputUpdate(bool driveAllowed, int mixL, int mixR) {
  OutputGateState gate{(OutputGateMode)outState, outHoldMs, rampFromL,
                       rampFromR};
  OutputGateAction action =
      outputGateStep(gate, driveAllowed, mixL, mixR, outL, outR, millis(),
                     SVC, CUTOFF_HOLD_MS);
  outState = (OutState)gate.mode;
  outHoldMs = gate.holdStartMs;
  rampFromL = gate.rampFromLeft;
  rampFromR = gate.rampFromRight;
  if (action.attach) escOutputAttach();
  if (action.write) outputWrite(action.leftPulse, action.rightPulse);
  if (action.detach) escOutputDetach();  // at neutral → stop pulsing → ESCs beep
}


// ═══════════════════════════════════════════════════════════════
// [TELEMETRY] — X.BUS telemetry via ports/TelemetrySource.h
// ═══════════════════════════════════════════════════════════════
// The 0x10 poller (framing, checksum, parse, EMA fold, poll state machine)
// lives in src/infrastructure/xc/XbusTelemetryAdapter.cpp (#117 step 10
// slice 4, #178) behind ports/TelemetrySource.h. READ-ONLY bus — PWM
// control authority is never affected. The sketch owns the telemetry
// array: [ALERT], [SAFETY], the dashboard JSON and debug all read telem[].

const uint8_t NUM_ESCS = 2;

EscTelem telem[NUM_ESCS] = {};

// X.BUS RX byte-level diagnostics owned by the adapter; printed (and the
// snapshot reset) by the [WIFI] wifiDebug() block below.
extern uint32_t telemetryDebugReceiveTotal;
extern uint32_t telemetryDebugEchoCount;
extern uint32_t telemetryDebugSlaveCount;
extern uint8_t  telemetryDebugSnapshot[];  // unsized: bound owned by the adapter
extern int      telemetryDebugSnapshotLength;

// Poll the bus; the adapter owns cadence, timeout and staleness internally.
void telemUpdate() { telemetrySourceUpdate(telem, NUM_ESCS); }


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

// The pin work lives in infrastructure/arduino/PiezoAdapter.cpp behind
// ports/AlertOutputPort.h (#117 step 10 slice 2, #174); the horn/pattern/
// alarm priority policy stays here.
void beeperInit() { alertOutputInitialize(PIN_BEEPER); }

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
  alertOutputSet(hornActive || patternOn || alarmOutputOn);
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
  // Priority (highest first): motor-overheat CUT (3 long) > battery low/cut
  // (3 short) > motor-overheat WARNING (trill) > inactivity (1 long). The two
  // hard stops sit above the warnings; among them the motor cut is the loudest,
  // longest pattern. tempCutActive supersedes tempWarnActive (a cut is also hot),
  // so the warning trill never plays while the cut alarm is sounding.
  const uint16_t* seq = nullptr;
  int len = 0;
  if (tempCutActive) {
    seq = ALERT_THERM_CUT;
    len = ALERT_THERM_CUT_LEN;
  } else if (lowVoltLatched) {
    seq = ALERT_LOWV;
    len = ALERT_LOWV_LEN;
  } else if (tempWarnActive) {
    seq = ALERT_THERM_WARN;
    len = ALERT_THERM_WARN_LEN;
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
//   Stage 1 — Eco lock   (≤ ECO_LOCK_THRESH_V ~10.8 V): updateGear() forces Eco
//             regardless of the RC gear switch, to ease load on a draining pack.
//   Stage 2 — Hard cutoff (≤ CUTOFF_THRESH_V 10.0 V): the output gate stops the
//             motors, and we assert lowVoltLatched so the D8 alarm chirps WITH
//             the cut — never a silent cutoff.

uint32_t cutoffStartMs  = 0;
uint32_t ecoLockStartMs = 0;

// Worst-of-two pack voltage if BOTH packs read a plausible value; else false
// (a not-yet-powered pack reads ~0 V and must not trip anything).
// Logic extracted to src/domain/battery/VoltagePlausibility.* (#117 step 1);
// this delegate keeps both ladder call sites and the test surface unchanged.
bool worstPackVoltage(float* worst) {
  return worstPlausiblePackVoltage(
      BatteryReading{telem[0].voltage, telem[0].valid},
      BatteryReading{telem[1].voltage, telem[1].valid},
      LOWV_PLAUS_MIN_V, LOWV_PLAUS_MAX_V, worst);
}

// Both stages extracted to src/domain/battery/BatteryLadder.* (#117 step 2,
// #154); these delegate shims keep every call site and test reference
// unchanged. The shims own the boundary work the domain must not do: read
// the clock (time is a parameter), mirror the ladder globals in/out, and —
// on the cutoff's just-latched signal — start [ALERT]'s alarm WITH the cut
// (domain code never writes another module's state). The early returns
// preserve the original property that a latched stage reads no telemetry.

void batteryEcoLockUpdate() {        // Stage 1 — force Eco
  if (ecoLockLatched) return;
  float worst = 0.0f;  // stays unwritten when implausible; the step ignores it then
  bool plausible = worstPackVoltage(&worst);
  BatteryLadderState ladder{ecoLockLatched, batteryCutoffLatched,
                            batteryOkConfirmed, ecoLockStartMs, cutoffStartMs};
  batteryEcoLockStep(ladder, plausible, worst, millis(),
                     ECO_LOCK_THRESH_V, ECO_LOCK_DEBOUNCE_MS);
  ecoLockLatched = ladder.ecoLockLatched;
  ecoLockStartMs = ladder.ecoLockStartMs;
}

void batteryCutoffUpdate() {         // Stage 2 — hard cutoff
  if (batteryCutoffLatched) return;
  float worst = 0.0f;  // stays unwritten when implausible; the step ignores it then
  bool plausible = worstPackVoltage(&worst);
  BatteryLadderState ladder{ecoLockLatched, batteryCutoffLatched,
                            batteryOkConfirmed, ecoLockStartMs, cutoffStartMs};
  bool cutoffJustLatched = batteryCutoffStep(ladder, plausible, worst, millis(),
                                             CUTOFF_THRESH_V, CUTOFF_DEBOUNCE_MS);
  batteryCutoffLatched = ladder.cutoffLatched;
  batteryOkConfirmed   = ladder.okConfirmed;
  cutoffStartMs        = ladder.cutoffStartMs;
  if (cutoffJustLatched) {
    lowVoltLatched = true;           // start the D8 alarm WITH the cut (no silent cutoff)
  }
}

// ── [SAFETY] motor / ESC over-temperature protection (#111) ──────────────────
// NON-LATCHING with hysteresis (heat ≠ a drained LiPo: once cool, drive resumes).
// Source = hottest of all 4 sensors among FRESH valid reads only; a telemetry
// dropout holds the last state and never cuts.

uint32_t tempWarnSinceMs = 0;   // trip-debounce timers (0 = not currently above ON)
uint32_t tempEcoSinceMs  = 0;
uint32_t tempCutSinceMs  = 0;

// The thermal ladder is extracted to src/domain/thermal/ (#117 step 3, #158);
// these delegate shims keep every call site and test reference unchanged.
// The shims own the boundary work the domain must not do: read the clock
// (time is a parameter), flatten telem[] into ThermalReadings, mirror the
// stage globals in/out, and — on the domain's cut-released signal — queue
// [BEEPER]'s one-shot "restored" flourish (4 quick beeps; the repeating cut
// alarm has already stopped because tempCutActive just went false). Domain
// code never calls another module.

// Hottest plausible temp across both ESCs (ESC + motor sensor each). Returns
// false if neither ESC has a fresh valid plausible reading → caller HOLDS state.
bool hottestMotorTemp(float* hot) {
  ThermalReading readings[NUM_ESCS * 2];
  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    readings[i * 2]     = ThermalReading{telem[i].escTempC, telem[i].valid};
    readings[i * 2 + 1] = ThermalReading{telem[i].motorTempC, telem[i].valid};
  }
  return hottestPlausibleTemperature(readings, NUM_ESCS * 2,
                                     TEMP_PLAUS_MIN_C, TEMP_PLAUS_MAX_C, hot);
}

void tempStageUpdate(bool* state, uint32_t* since, float temp,
                     float onC, float offC, uint32_t nowMs) {
  ThermalStageState stage{*state, *since};
  thermalStageStep(stage, temp, nowMs, onC, offC, TEMP_DEBOUNCE_MS);
  *state = stage.active;
  *since = stage.sinceMs;
}

void thermalUpdate() {
  float hot = 0.0f;  // defensive init: the out-param is left unchanged on reject
  bool haveReading = hottestMotorTemp(&hot);
  ThermalDeratingState derating{{tempWarnActive, tempWarnSinceMs},
                                {tempEcoActive,  tempEcoSinceMs},
                                {tempCutActive,  tempCutSinceMs}};
  bool cutReleased = thermalDeratingStep(
      derating, haveReading, hot, millis(),
      ThermalDeratingThresholds{TEMP_WARN_ON_C, TEMP_WARN_OFF_C,
                                TEMP_ECO_ON_C,  TEMP_ECO_OFF_C,
                                TEMP_CUT_ON_C,  TEMP_CUT_OFF_C,
                                TEMP_DEBOUNCE_MS});
  tempWarnActive = derating.warn.active;
  tempWarnSinceMs = derating.warn.sinceMs;
  tempEcoActive = derating.eco.active;
  tempEcoSinceMs = derating.eco.sinceMs;
  tempCutActive = derating.cut.active;
  tempCutSinceMs = derating.cut.sinceMs;
  if (cutReleased) beepStart(BEEP_THERM_RESTORED, BEEP_THERM_RESTORED_LEN);
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

// Credentials come from arduino_secrets.h (gitignored, #125) — copy
// arduino_secrets.h.example in this folder and fill in real values.
#include "arduino_secrets.h"
const char WIFI_SSID[] = SECRET_WIFI_SSID;
const char WIFI_PASS[] = SECRET_WIFI_PASS;   // WPA2 needs >= 8 chars
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
    // Build the dashboard ETag once — a CONTENT HASH (FNV-1a) of the embedded
    // page, so ANY edit invalidates stale browser caches, even one that doesn't
    // change the page length (the old "length + frozen v711 tag" ETag missed
    // same-length edits → 304 served a stale page). #109.
    uint32_t etagHash = 2166136261u;
    for (const char *p = INDEX_HTML; *p; ++p) {
      etagHash ^= (uint8_t)*p;
      etagHash *= 16777619u;
    }
    snprintf(pageEtag, sizeof(pageEtag), "\"d%08lx\"", (unsigned long)etagHash);
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
  Serial.print("# XBUS rx_total="); Serial.print(telemetryDebugReceiveTotal);
  Serial.print(" echo(0x0F)=");     Serial.print(telemetryDebugEchoCount);
  Serial.print(" slave(0xF0)=");    Serial.print(telemetryDebugSlaveCount);
  Serial.print(" snap=[");
  for (int i = 0; i < telemetryDebugSnapshotLength; i++) {
    char hex[4]; snprintf(hex, sizeof(hex), "%02X ", telemetryDebugSnapshot[i]); Serial.print(hex);
  }
  Serial.println("]");
  telemetryDebugSnapshotLength = 0;   // reset for next window's snapshot
}

// Observe the whole system into one immutable SystemSnapshot (#132). Called
// by the observer shims at their original observation points for exact
// timing parity; the once-per-cycle build moves into FirmwareApp (step 11).
SystemSnapshot buildSystemSnapshot(uint32_t nowMs) {
  SystemSnapshot snapshot = {};
  snapshot.nowMs = nowMs;
  snapshot.rcThrottleUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_THR])   : SVC;
  snapshot.rcSteeringUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_STEER]) : SVC;
  snapshot.rcGearUs     = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_GEAR])  : SVC;
  snapshot.rcOverrideUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_OVR])   : SVMIN;
  snapshot.rcFailsafe = rcFrame.failsafe;
  snapshot.rcLostFrame = rcFrame.lostFrame;
  snapshot.joystickRawY = cachedJoy.rawY;
  snapshot.joystickRawX = cachedJoy.rawX;
  snapshot.outLeftUs = outL;
  snapshot.outRightUs = outR;
  snapshot.gear = (int)currentGear;
  snapshot.overrideMode = wifiMode();
  snapshot.batteryCutoffLatched = batteryCutoffLatched;
  snapshot.ecoLockLatched = ecoLockLatched;
  snapshot.temperatureWarnActive = tempWarnActive;
  snapshot.temperatureEcoActive = tempEcoActive;
  snapshot.temperatureCutActive = tempCutActive;
  snapshot.esc[0] = telem[0];
  snapshot.esc[1] = telem[1];
  return snapshot;
}

// Build the telemetry JSON into body; returns its length. Shared by the
// one-shot /data endpoint and the SSE stream. Formatting lives in
// telemetry/JsonEncoder (#132); this shim owns the frame counter and the
// observation point.
int buildTelemJson(char *body, size_t cap) {
  wifiSeq++;
  return encodeTelemetryJson(body, cap, buildSystemSnapshot(millis()), wifiSeq);
}

// One-shot /data JSON. Header + body coalesced into a single write() so the
// whole response is one modem round-trip (one bounded op this loop pass).
void wifiSendData(WiFiClient &client) {
  char body[400];
  int n = buildTelemJson(body, sizeof(body));
  char buf[560];
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
    // Version + short tag only (#124) — the changelog lives in git history
    // and docs/FIRMWARE-UPLOAD-LOG.md, not in a string constant.
    Serial.print("# === Digger ");
    Serial.print(FIRMWARE_VERSION);
    Serial.println(" — dual-input track control, GL10 FOC ===");
    Serial.println("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,V0dV,I0dA,RPM0,TE0,TM0,OK0,V1dV,I1dA,RPM1,TE1,TM1,OK1");
  }
}

void debugPrint(uint32_t now) {
  if (!Serial || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  // Formatting (integer-scaled — no float printf on this core) lives in
  // telemetry/CsvEncoder (#132); this shim owns the cadence and the print.
  char buf[200];
  encodeDebugCsv(buf, sizeof(buf), buildSystemSnapshot(now));
  Serial.println(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

void setup() {
  analogReadResolution(14);
  rcInputInitialize();
  outputInit();
  beeperInit();
  alertInit();
  debugInit();
  telemetrySourceInitialize();  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
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
  if (rcInputReadFrame(&rcFrame)) {
    sbusLastFrame = now;
    sbusValid = !rcFrame.failsafe;
  }
  if ((now - sbusLastFrame) > SBUS_TIMEOUT) sbusValid = false;
  hornActive = sbusValid && (rcFrame.channels[SBUS_CH_HORN] > HORN_ON_RAW);

  // 1.5 Staged low-battery protection (#65) — evaluate BEFORE gear select so the
  // Eco lock can override it. Stage 1 (~11 V) forces Eco; Stage 2 (10 V) cuts.
  batteryEcoLockUpdate();
  batteryCutoffUpdate();
  thermalUpdate();           // motor/ESC over-temp stages (#111) — sets Eco/cut flags
  updateGear();              // honors ecoLockLatched + tempEcoActive (forces Eco when set)
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

  // 2.5 ESC CALIBRATION MODE (#113, temporary) — bypass ALL mixing/caps/gear and
  // send the raw throttle stick to the full ±100% range (1000/1500/2000 us) on
  // BOTH tracks, so the GL10s learn the Arduino's true endpoints. Steering is
  // ignored so both ESCs see identical full-range signals for a clean capture.
  if (CALIBRATION_MODE && sbusValid) {
    float thr = constrain((float)(rcThrottle() - SVC) / SOFT_RANGE, -1.0f, 1.0f);
    int pwm = SVC + (int)(thr * SOFT_RANGE);
    mix.left = pwm;
    mix.right = pwm;
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
  // The decision is extracted to src/domain/safety/SafetySupervisor.* (#117
  // step 8, #168); this call site owns the boundary reads (RC freshness,
  // battery flags, the millis()-based boot grace) and today consumes only
  // driveAllowed — the FailsafeReason feeds the OutputGate and
  // SystemSnapshot steps. The thermal cut (#111) is NOT latched: when the
  // motor cools below TEMP_CUT_OFF_C the gate re-opens and outputUpdate()
  // re-attaches the ESCs automatically.
  SafetyDecision safety = safetySupervisorDecide(SafetyInputs{
      sbusValid, batteryOkConfirmed,
      millis() - alertBootMs > BATTERY_CONFIRM_MS, batteryCutoffLatched,
      tempCutActive});
  outputUpdate(safety.driveAllowed, mix.left, mix.right);

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
