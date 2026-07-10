// domain/battery — worst-of-two pack-voltage plausibility gate (#117 step 1).
// Logic copied VERBATIM from rc_test.ino [SAFETY] worstPackVoltage();
// behavior is locked by tests/invariants/test_invariant_plausibility.cpp and
// tests/characterization/test_battery_ladder.cpp.
#include "VoltagePlausibility.h"

bool worstPlausiblePackVoltage(const BatteryReading& pack0,
                               const BatteryReading& pack1,
                               float plausibleMinV, float plausibleMaxV,
                               float* worstOut) {
  float v0 = pack0.voltage, v1 = pack1.voltage;
  if (!(pack0.valid && pack1.valid)) return false;
  if (v0 < plausibleMinV || v0 > plausibleMaxV ||
      v1 < plausibleMinV || v1 > plausibleMaxV) return false;
  *worstOut = (v0 < v1) ? v0 : v1;
  return true;
}
