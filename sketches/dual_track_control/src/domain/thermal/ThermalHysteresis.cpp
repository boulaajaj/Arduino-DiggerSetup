// domain/thermal — one hysteresis stage (#117 step 3, #158).
// Logic copied VERBATIM from dual_track_control.ino [SAFETY] tempStageUpdate();
// behavior is locked by tests/characterization/test_thermal_stages.cpp,
// tests/invariants/test_invariant_thermal_monotone.cpp and
// tests/thermal/test_thermal_domain.cpp.
#include "ThermalHysteresis.h"

void thermalStageStep(ThermalStageState& stage, float temperatureC,
                      uint32_t nowMs, float onC, float offC,
                      uint32_t debounceMs) {
  if (stage.active) {
    if (temperatureC < offC) {
      stage.active = false;
      stage.sinceMs = 0;
    }
  } else if (temperatureC >= onC) {
    if (stage.sinceMs == 0) stage.sinceMs = nowMs;
    if (nowMs - stage.sinceMs >= debounceMs) stage.active = true;
  } else {
    stage.sinceMs = 0;  // dropped back before debounce elapsed
  }
}
