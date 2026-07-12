// domain/thermal — warn/eco/cut staging (#117 step 3, #158).
// Logic copied VERBATIM from rc_test.ino [SAFETY]; behavior is locked by
// tests/characterization/test_thermal_stages.cpp,
// tests/invariants/test_invariant_thermal_monotone.cpp and
// tests/thermal/test_thermal_domain.cpp.
#include "ThermalDerating.h"

#include "ThermalHysteresis.h"

bool hottestPlausibleTemperature(const ThermalReading* readings,
                                 uint8_t count,
                                 float plausibleMinC, float plausibleMaxC,
                                 float* hottestOut) {
  // Running max starts at the lower plausibility bound: every accepted
  // reading is >= plausibleMinC, so the result is identical to the
  // firmware's original fixed -1000.0f sentinel — but stays correct for
  // any parameter range now that the bounds are parameters.
  float h = plausibleMinC;
  bool any = false;
  for (uint8_t i = 0; i < count; i++) {
    if (!readings[i].valid) continue;
    float t = readings[i].temperatureC;
    if (t >= plausibleMinC && t <= plausibleMaxC) {
      if (t > h) h = t;
      any = true;
    }
  }
  if (any) *hottestOut = h;
  return any;
}

bool thermalDeratingStep(ThermalDeratingState& derating, bool haveReading,
                         float hottestC, uint32_t nowMs,
                         const ThermalDeratingThresholds& thresholds) {
  if (!haveReading) return false;  // no fresh reading → hold every stage
  bool wasCut = derating.cut.active;
  thermalStageStep(derating.warn, hottestC, nowMs,
                   thresholds.warnOnC, thresholds.warnOffC,
                   thresholds.debounceMs);
  thermalStageStep(derating.eco, hottestC, nowMs,
                   thresholds.ecoOnC, thresholds.ecoOffC,
                   thresholds.debounceMs);
  thermalStageStep(derating.cut, hottestC, nowMs,
                   thresholds.cutOnC, thresholds.cutOffC,
                   thresholds.debounceMs);
  return wasCut && !derating.cut.active;  // cut released on THIS call
}
