// application — telemetry/beeper/alert shim implementations (#189),
// verbatim from the .ino.
#include "AlertControl.h"

#include "FirmwareState.h"

#include "../config/AlertConfig.h"
#include "../config/Pins.h"
#include "../alerts/PatternPlayer.h"
#include "../alerts/AlertPolicy.h"
#include "../ports/AlertOutputPort.h"
#include "../ports/ClockPort.h"

// ═══════════════════════════════════════════════════════════════
// [TELEMETRY] — X.BUS telemetry via ports/TelemetrySource.h
// ═══════════════════════════════════════════════════════════════
// The 0x10 poller (framing, checksum, parse, EMA fold, poll state machine)
// lives in src/infrastructure/xc/XbusTelemetryAdapter.cpp (#117 step 10
// slice 4, #178) behind ports/TelemetrySource.h. READ-ONLY bus — PWM
// control authority is never affected. The sketch owns the telemetry
// array: [ALERT], [SAFETY], the dashboard JSON and debug all read telem[].

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


