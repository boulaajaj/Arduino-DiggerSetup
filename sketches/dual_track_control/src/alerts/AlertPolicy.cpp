// alerts — alert policy implementations (#183), verbatim from the .ino.
#include "AlertPolicy.h"

bool inactivityAlarmStep(uint32_t& offSinceMs, bool rcOn, uint32_t nowMs,
                         uint32_t thresholdMs) {
  if (rcOn)                   offSinceMs = 0;
  else if (offSinceMs == 0)   offSinceMs = nowMs;
  return (offSinceMs != 0) && (nowMs - offSinceMs >= thresholdMs);
}

void lowVoltageLatchStep(LowVoltageLatchState& state, float voltage0,
                         float voltage1, bool valid0, bool valid1,
                         uint32_t nowMs, uint32_t bootMs,
                         const LowVoltageThresholds& thresholds) {
  if (!state.latched && (nowMs - bootMs >= thresholds.startupGraceMs)) {
    bool bothValid = valid0 && valid1;
    bool plausible =
        (voltage0 >= thresholds.plausibleMinV && voltage0 <= thresholds.plausibleMaxV &&
         voltage1 >= thresholds.plausibleMinV && voltage1 <= thresholds.plausibleMaxV);
    if (bothValid && plausible) {
      float worst = (voltage0 < voltage1) ? voltage0 : voltage1;
      if (worst < thresholds.alarmV) {
        if (state.belowSinceMs == 0) state.belowSinceMs = nowMs;
        if (nowMs - state.belowSinceMs >= thresholds.debounceMs) state.latched = true;
      } else {
        state.belowSinceMs = 0;   // recovered before debounce elapsed
      }
    } else {
      state.belowSinceMs = 0;     // can't measure both packs → reset debounce, stay silent
    }
  }
}

AlertPattern alertPatternSelect(bool thermalCutActive, bool lowVoltageLatched,
                                bool thermalWarnActive, bool inactivityActive,
                                const AlertPatternTable& table) {
  if (thermalCutActive)   return table.thermalCut;
  if (lowVoltageLatched)  return table.lowVoltage;
  if (thermalWarnActive)  return table.thermalWarn;
  if (inactivityActive)   return table.inactivity;
  return AlertPattern{nullptr, 0};
}
