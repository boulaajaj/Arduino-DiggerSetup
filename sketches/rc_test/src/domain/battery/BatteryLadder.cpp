// domain/battery — staged low-battery protection ladder (#117 step 2, #154).
// Logic copied VERBATIM from rc_test.ino [SAFETY]; behavior is locked by
// tests/characterization/test_battery_ladder.cpp,
// tests/invariants/test_invariant_cutoff_dominates.cpp and
// tests/battery/test_battery_ladder_domain.cpp.
#include "BatteryLadder.h"

void batteryEcoLockStep(BatteryLadderState& ladder, bool plausible,
                        float worstV, uint32_t nowMs,
                        float ecoLockThresholdV, uint32_t ecoLockDebounceMs) {
  if (ladder.ecoLockLatched) return;
  if (!plausible) {
    ladder.ecoLockStartMs = 0;
    return;
  }
  if (worstV < ecoLockThresholdV) {
    if (ladder.ecoLockStartMs == 0) ladder.ecoLockStartMs = nowMs;
    if (nowMs - ladder.ecoLockStartMs >= ecoLockDebounceMs) {
      ladder.ecoLockLatched = true;
    }
  } else {
    ladder.ecoLockStartMs = 0;  // recovered before debounce elapsed
  }
}

bool batteryCutoffStep(BatteryLadderState& ladder, bool plausible,
                       float worstV, uint32_t nowMs,
                       float cutoffThresholdV, uint32_t cutoffDebounceMs) {
  if (ladder.cutoffLatched) return false;
  if (!plausible) {
    ladder.cutoffStartMs = 0;
    return false;
  }
  if (worstV >= cutoffThresholdV) ladder.okConfirmed = true;  // boot gate: pack confirmed above cutoff
  if (worstV < cutoffThresholdV) {
    if (ladder.cutoffStartMs == 0) ladder.cutoffStartMs = nowMs;
    if (nowMs - ladder.cutoffStartMs >= cutoffDebounceMs) {
      ladder.cutoffLatched = true;
      return true;  // latched on THIS call — caller starts the alarm WITH the cut
    }
  } else {
    ladder.cutoffStartMs = 0;  // recovered before debounce elapsed
  }
  return false;
}
