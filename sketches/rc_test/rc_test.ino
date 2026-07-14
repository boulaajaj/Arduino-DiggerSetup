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
#include "src/alerts/PatternPlayer.h"
#include "src/alerts/AlertPolicy.h"
#include "src/application/SystemSnapshot.h"
#include "src/telemetry/JsonEncoder.h"
#include "src/telemetry/CsvEncoder.h"
#include "src/ports/EscOutputPort.h"
#include "src/ports/JoystickPort.h"
#include "src/ports/AlertOutputPort.h"
#include "src/ports/RcInputPort.h"
#include "src/ports/TelemetrySource.h"
#include "src/ports/DashboardServicePort.h"
#include "src/ports/TelemetryFrameSource.h"
#include "src/ports/ClockPort.h"
#include "src/ports/WatchdogPort.h"
#include "src/ports/DebugConsolePort.h"
#include "src/application/RangeMath.h"


// The second hardware UART (SCI0, D11/D12), the S.BUS parser and their pin
// constants moved to infrastructure/radiolink/SbusReceiverAdapter.cpp
// behind ports/RcInputPort.h (#117 step 10 slice 3, #176).


// ═══════════════════════════════════════════════════════════════
// [CONFIG] — tunable constants, now grouped in src/config/ (#185)
// ═══════════════════════════════════════════════════════════════
// Values unchanged on move; one header per domain, per the target:
// BuildInfo (version SSOT #124) · Pins · InputConfig · DriveConfig ·
// BatteryConfig · ThermalConfig · AlertConfig · TelemetryConfig ·
// WifiConfig · SafetyConfig (watchdog). Pins.h uses the core's A0/A1
// macros and relies on <Arduino.h> being included first (it is, above).
#include "src/config/BuildInfo.h"
#include "src/config/Pins.h"
#include "src/config/InputConfig.h"
#include "src/config/DriveConfig.h"
#include "src/config/BatteryConfig.h"
#include "src/config/ThermalConfig.h"
#include "src/config/AlertConfig.h"
#include "src/config/TelemetryConfig.h"
#include "src/config/WifiConfig.h"
#include "src/config/SafetyConfig.h"

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
  out.left  = clampInt(out.left,  SVMIN, SVMAX);
  out.right = clampInt(out.right, SVMIN, SVMAX);
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
  return mapRange(clampInt(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
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
  xSpeed    = clampFloat(xSpeed * RC_THROTTLE_GAIN, -outCapToX(reverseCap()), 1.0f);
  zRotation = clampFloat(zRotation * RC_STEERING_GAIN, -1.0f, 1.0f);
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

  float normY = clampFloat((float)(joyDeadband(cachedJoy.rawY) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float normX = clampFloat((float)(joyDeadband(cachedJoy.rawX) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
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
  xSpeed = clampFloat(xSpeed, -outCapToX(reverseCap()), outCapToX(fwdCap));
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
  outL = clampInt(left,  SVMIN, SVMAX);
  outR = clampInt(right, SVMIN, SVMAX);
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
      outputGateStep(gate, driveAllowed, mixL, mixR, outL, outR, clockNowMs(),
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
// Sequencing lives in alerts/PatternPlayer (#183); the shim mirrors the
// globals so every call site and test reference stays put.
void beepStart(const uint16_t *seq, int len) {
  OneShotPatternState pattern{};
  oneShotPatternStart(pattern, seq, len, clockNowMs());
  beepSeq = pattern.sequence; beepLen = pattern.length;
  beepIdx = pattern.index;    beepPhaseMs = pattern.phaseMs;
}

// Call every loop. Drives D8 from the horn (held) OR the active pattern.
void beeperUpdate() {
  OneShotPatternState pattern{beepSeq, beepLen, beepIdx, beepPhaseMs};
  bool patternOn = oneShotPatternStep(pattern, clockNowMs());
  beepSeq = pattern.sequence; beepLen = pattern.length;
  beepIdx = pattern.index;    beepPhaseMs = pattern.phaseMs;
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

uint32_t rcOffSinceMs   = 0;      // clockNowMs() when RC went off (0 = RC on)
uint32_t lowVStartMs    = 0;      // clockNowMs() when worst pack first dipped low (0 = above)
bool     lowVoltLatched = false;  // once true, stays until power cycle
uint32_t alertBootMs    = 0;      // set in setup() — startup-grace reference

// active repeating-alarm playback state
const uint16_t* alarmSeq = nullptr;
int      alarmLen = 0, alarmIdx = 0;
uint32_t alarmPhaseMs = 0;

void alertInit() { alertBootMs = clockNowMs(); }

// NOTE: [ALERT] below is AUDIO-ONLY — it drives the D8 piezo and never touches the
// motors. The motor-affecting battery cutoff lives in its own [SAFETY] section
// (search [SAFETY]); it only *borrows* this module's lowVoltLatched to start the
// chirp when it cuts.

// Call every loop. rcOn = sbusValid. Sets alarmOutputOn for the piezo.
// The decision logic and playback live in alerts/AlertPolicy +
// alerts/PatternPlayer (#183); this shim mirrors the globals so every call
// site and test reference stays put.
void alertUpdate(bool rcOn) {
  uint32_t nowMs = clockNowMs();

  bool inactiveAlarm = inactivityAlarmStep(rcOffSinceMs, rcOn, nowMs, INACT_RC_OFF_MS);

  static const LowVoltageThresholds LOWV_THRESHOLDS{
      LOWV_THRESH_V, LOWV_PLAUS_MIN_V, LOWV_PLAUS_MAX_V,
      LOWV_DEBOUNCE_MS, ALERT_STARTUP_MS};
  LowVoltageLatchState lowVoltage{lowVStartMs, lowVoltLatched};
  lowVoltageLatchStep(lowVoltage, telem[0].voltage, telem[1].voltage,
                      telem[0].valid, telem[1].valid, nowMs, alertBootMs,
                      LOWV_THRESHOLDS);
  lowVStartMs = lowVoltage.belowSinceMs;
  lowVoltLatched = lowVoltage.latched;

  // Priority (highest first): motor-overheat CUT (3 long) > battery low/cut
  // (3 short) > motor-overheat WARNING (trill) > inactivity (1 long). The two
  // hard stops sit above the warnings; among them the motor cut is the loudest,
  // longest pattern. tempCutActive supersedes tempWarnActive (a cut is also hot),
  // so the warning trill never plays while the cut alarm is sounding.
  static const AlertPatternTable ALERT_PATTERNS{
      {ALERT_THERM_CUT, ALERT_THERM_CUT_LEN},
      {ALERT_LOWV, ALERT_LOWV_LEN},
      {ALERT_THERM_WARN, ALERT_THERM_WARN_LEN},
      {ALERT_INACT, ALERT_INACT_LEN}};
  AlertPattern selected = alertPatternSelect(
      tempCutActive, lowVoltLatched, tempWarnActive, inactiveAlarm,
      ALERT_PATTERNS);

  if (selected.sequence == nullptr) {
    alarmSeq = nullptr;
    alarmOutputOn = false;
    return;
  }

  RepeatingPatternState alarm{alarmSeq, alarmLen, alarmIdx, alarmPhaseMs};
  repeatingPatternSelect(alarm, selected.sequence, selected.length, nowMs);
  alarmOutputOn = repeatingPatternStep(alarm, nowMs);
  alarmSeq = alarm.sequence; alarmLen = alarm.length;
  alarmIdx = alarm.index;    alarmPhaseMs = alarm.phaseMs;
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
  batteryEcoLockStep(ladder, plausible, worst, clockNowMs(),
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
  bool cutoffJustLatched = batteryCutoffStep(ladder, plausible, worst, clockNowMs(),
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
      derating, haveReading, hot, clockNowMs(),
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
uint32_t wifiSeq = 0;

// The serving machine (AP bring-up + FNV-1a ETag, HTTP routing, incremental
// page transfer, SSE stream) lives in src/infrastructure/network/ (#181)
// behind ports/DashboardServicePort.h; the JSON body flows the other way
// through ports/TelemetryFrameSource.h. Serving state is adapter-owned.
extern bool wifiUp;  // read by wifiDebug() below

// Override switch → dashboard mode (0=RC, 1=joy/auto-middle, 2=blend).
int wifiMode() {
  if (!sbusValid) return 0;
  int ovr = rcOverride();
  if (ovr < OVR_LO) return 0;
  if (ovr > OVR_HI) return 2;
  return 1;
}

void wifiInit() {
  // AP bring-up (incl. banners + ETag hash) lives in the adapter; the
  // ready-beep stays application-side (alert policy, not networking).
  if (dashboardServiceInitialize(WIFI_SSID, WIFI_PASS, WIFI_AP_CHANNEL, INDEX_HTML)) {
    beepStart(BEEP_WIFI_READY, BEEP_WIFI_READY_LEN);   // "beep beep" — Wi-Fi AP is up/ready
  }
}

// Periodic Wi-Fi status line (every ~3 s) for bench diagnostics.
uint32_t wifiDbgPrev = 0;
void wifiDebug(uint32_t nowUs) {
  if (!debugConsoleReady() || (nowUs - wifiDbgPrev) < 3000000UL) return;
  wifiDbgPrev = nowUs;
  // Lines assembled with snprintf and printed whole (#187); the byte stream
  // matches the old Serial print-chains exactly (bool printed 1/0, decimal
  // status/counters, two-digit uppercase hex + trailing space per byte).
  char line[96];
  snprintf(line, sizeof(line), "# WIFI up=%d status=%d clients_seq=%lu",
           wifiUp ? 1 : 0, dashboardServiceRadioStatus(), (unsigned long)wifiSeq);
  debugConsolePrintLine(line);

  // X.BUS RX byte-level diagnostics — tells us whether D0 sees anything at all.
  char xbus[144];
  int n = snprintf(xbus, sizeof(xbus),
                   "# XBUS rx_total=%lu echo(0x0F)=%lu slave(0xF0)=%lu snap=[",
                   (unsigned long)telemetryDebugReceiveTotal,
                   (unsigned long)telemetryDebugEchoCount,
                   (unsigned long)telemetryDebugSlaveCount);
  for (int i = 0; i < telemetryDebugSnapshotLength && n < (int)sizeof(xbus) - 4; i++) {
    n += snprintf(xbus + n, sizeof(xbus) - n, "%02X ", telemetryDebugSnapshot[i]);
  }
  snprintf(xbus + n, sizeof(xbus) - n, "]");
  debugConsolePrintLine(xbus);
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
  return encodeTelemetryJson(body, cap, buildSystemSnapshot(clockNowMs()), wifiSeq);
}

// The network layer pulls frames through ports/TelemetryFrameSource.h —
// the application encodes, infrastructure ships bytes (#181).
int telemetryFrameBuild(char *body, size_t capacity) {
  return buildTelemJson(body, capacity);
}

// Non-blocking, and bounded to AT MOST ONE modem write per loop pass (#69).
// The serving machine (page chunks, request routing, SSE) lives in
// infrastructure/network/DashboardServer.cpp (#181).
void wifiUpdate() { dashboardServiceUpdate(); }


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV (control + telemetry)
// ═══════════════════════════════════════════════════════════════
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,
//          V0dV,I0dA,RPM0,TE0,TM0,OK0, V1dV,I1dA,RPM1,TE1,TM1,OK1
// Telemetry columns are integer-scaled: voltage in 0.1 V (dV), current in
// 0.1 A (dA), RPM in electrical Hz, temps in °C, OK = fresh-telemetry flag.

uint32_t prevPrint = 0;

void debugInit() {
  debugConsoleBegin(115200);
  if (debugConsoleReady()) {
    // Version + short tag only (#124) — the changelog lives in git history
    // and docs/FIRMWARE-UPLOAD-LOG.md, not in a string constant. Assembled
    // into one line; the byte stream matches the old print-chain exactly.
    char banner[96];
    snprintf(banner, sizeof(banner),
             "# === Digger %s — dual-input track control, GL10 FOC ===",
             FIRMWARE_VERSION);
    debugConsolePrintLine(banner);
    debugConsolePrintLine("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,V0dV,I0dA,RPM0,TE0,TM0,OK0,V1dV,I1dA,RPM1,TE1,TM1,OK1");
  }
}

void debugPrint(uint32_t now) {
  if (!debugConsoleReady() || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  // Formatting (integer-scaled — no float printf on this core) lives in
  // telemetry/CsvEncoder (#132); this shim owns the cadence and the print.
  // NOTE: `now` is µs (loop clock); the snapshot contract is ms, so read
  // clockNowMs() here — the CSV does not use nowMs, so output is unchanged.
  char buf[200];
  encodeDebugCsv(buf, sizeof(buf), buildSystemSnapshot(clockNowMs()));
  debugConsolePrintLine(buf);
}


// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

void setup() {
  joystickInitialize();    // 14-bit ADC resolution (adapter-owned, #187)
  rcInputInitialize();
  outputInit();
  beeperInit();
  alertInit();
  debugInit();
  telemetrySourceInitialize();  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
  wifiInit();             // Wi-Fi AP + telemetry server (monitoring only)

  // Arm the hardware watchdog LAST — after the slow Wi-Fi AP bring-up — so init
  // can't trip it. From here, loop() must call watchdogRefresh() within
  // WDT_TIMEOUT_MS or the MCU resets: PWM stops, the ESCs see no signal and go
  // to neutral/failsafe, and the machine stops instead of holding the last
  // throttle. This is the runaway backstop for ANY loop stall (#69).
  if (watchdogBegin(WDT_TIMEOUT_MS)) {
    if (debugConsoleReady()) {
      char line[48];
      snprintf(line, sizeof(line), "# WDT armed @ %lu ms",
               (unsigned long)watchdogTimeoutMs());
      debugConsolePrintLine(line);
    }
  } else if (debugConsoleReady()) {
    debugConsolePrintLine("# WDT FAILED to arm — no loop-stall backstop!");
  }
}

void loop() {
  uint32_t now = clockNowUs();

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
    float thr = clampFloat((float)(rcThrottle() - SVC) / SOFT_RANGE, -1.0f, 1.0f);
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
  // battery flags, the clockNowMs()-based boot grace) and today consumes only
  // driveAllowed — the FailsafeReason feeds the OutputGate and
  // SystemSnapshot steps. The thermal cut (#111) is NOT latched: when the
  // motor cools below TEMP_CUT_OFF_C the gate re-opens and outputUpdate()
  // re-attaches the ESCs automatically.
  SafetyDecision safety = safetySupervisorDecide(SafetyInputs{
      sbusValid, batteryOkConfirmed,
      clockNowMs() - alertBootMs > BATTERY_CONFIRM_MS, batteryCutoffLatched,
      tempCutActive});
  outputUpdate(safety.driveAllowed, mix.left, mix.right);

  // Control path serviced this pass (inputs read + output gate run) — and ONLY
  // now do we kick the watchdog. This is the sole refresh point: if anything
  // below (telemetry/Wi-Fi) ever stalls the loop beyond WDT_TIMEOUT_MS, the
  // refresh is missed, the MCU resets, and the machine stops instead of holding
  // the last throttle command (#69).
  watchdogRefresh();

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
