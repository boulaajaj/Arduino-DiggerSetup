// application — battery/thermal shim implementations (#189), verbatim
// from the .ino.
#include "SafetyControl.h"

#include "AlertControl.h"
#include "FirmwareState.h"
#include "../config/AlertConfig.h"
#include "../config/BatteryConfig.h"
#include "../config/ThermalConfig.h"
#include "../domain/battery/VoltagePlausibility.h"
#include "../domain/battery/BatteryLadder.h"
#include "../domain/thermal/ThermalHysteresis.h"
#include "../domain/thermal/ThermalDerating.h"
#include "../ports/ClockPort.h"

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


