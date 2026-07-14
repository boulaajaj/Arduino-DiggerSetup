// Pure-domain suite (#117 step 2, #154): src/domain/battery/BatteryLadder
// tested directly on the host — no firmware, no stubs, time and thresholds
// passed explicitly. End-to-end behavior through the firmware's
// batteryEcoLockUpdate()/batteryCutoffUpdate() shims stays covered by
// tests/characterization/test_battery_ladder.cpp and
// tests/invariants/test_invariant_cutoff_dominates.cpp; this suite locks the
// extracted state machines at the unit level: debounce timing, latch
// permanence, timer resets, the boot gate, and the just-latched signal the
// application layer uses to start the alarm WITH the cut.
// (Basename differs from characterization/test_battery_ladder.cpp — the flat
// build/ directory forbids duplicate test basenames.)
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/domain/battery/BatteryLadder.h"

// Values mirror ECO_LOCK_THRESH_V / ECO_LOCK_DEBOUNCE_MS / CUTOFF_THRESH_V /
// CUTOFF_DEBOUNCE_MS — but they are parameters here: the domain functions
// have no config dependency.
const float    ECO_LOCK_THRESHOLD_V  = 10.8f;
const uint32_t ECO_LOCK_DEBOUNCE_MS  = 15000UL;
const float    CUTOFF_THRESHOLD_V    = 10.0f;
const uint32_t CUTOFF_DEBOUNCE_MS    = 1500UL;

inline BatteryLadderState freshLadder() {
  return BatteryLadderState{false, false, false, 0, 0};
}

inline void ecoStep(BatteryLadderState& ladder, bool plausible, float worstV,
                    uint32_t nowMs) {
  batteryEcoLockStep(ladder, plausible, worstV, nowMs,
                     ECO_LOCK_THRESHOLD_V, ECO_LOCK_DEBOUNCE_MS);
}

inline bool cutoffStep(BatteryLadderState& ladder, bool plausible,
                       float worstV, uint32_t nowMs) {
  return batteryCutoffStep(ladder, plausible, worstV, nowMs,
                           CUTOFF_THRESHOLD_V, CUTOFF_DEBOUNCE_MS);
}

TEST_CASE("eco lock latches only after the full debounce, then holds forever") {
  BatteryLadderState ladder = freshLadder();
  ecoStep(ladder, true, 10.5f, 1000);                    // below → timer starts
  CHECK(ladder.ecoLockLatched == false);
  CHECK(ladder.ecoLockStartMs == 1000);
  ecoStep(ladder, true, 10.5f, 1000 + ECO_LOCK_DEBOUNCE_MS - 1);
  CHECK(ladder.ecoLockLatched == false);                 // one ms short
  ecoStep(ladder, true, 10.5f, 1000 + ECO_LOCK_DEBOUNCE_MS);
  CHECK(ladder.ecoLockLatched == true);                  // exact boundary latches
  ecoStep(ladder, true, 12.6f, 1000 + ECO_LOCK_DEBOUNCE_MS + 1);
  CHECK(ladder.ecoLockLatched == true);                  // recovery never unlatches
  // Stage 1 never touches Stage 2's fields.
  CHECK(ladder.cutoffLatched == false);
  CHECK(ladder.okConfirmed == false);
  CHECK(ladder.cutoffStartMs == 0);
}

TEST_CASE("eco lock: recovery or an implausible reading resets the debounce") {
  BatteryLadderState ladder = freshLadder();
  ecoStep(ladder, true, 10.5f, 1000);
  ecoStep(ladder, true, 11.5f, 5000);                    // recovered → reset
  CHECK(ladder.ecoLockStartMs == 0);
  ecoStep(ladder, true, 10.5f, 6000);                    // fresh window
  ecoStep(ladder, false, 10.5f, 9000);                   // implausible → reset, NOT hold
  CHECK(ladder.ecoLockStartMs == 0);
  ecoStep(ladder, true, 10.5f, 10000);
  ecoStep(ladder, true, 10.5f, 10000 + ECO_LOCK_DEBOUNCE_MS);
  CHECK(ladder.ecoLockLatched == true);                  // full window from the restart
}

TEST_CASE("eco lock threshold is strict — exactly 10.8 V does not count as below") {
  BatteryLadderState ladder = freshLadder();
  ecoStep(ladder, true, ECO_LOCK_THRESHOLD_V, 1000);
  CHECK(ladder.ecoLockStartMs == 0);
  CHECK(ladder.ecoLockLatched == false);
}

TEST_CASE("boot gate: only a plausible reading at/above cutoff confirms the pack") {
  BatteryLadderState ladder = freshLadder();
  cutoffStep(ladder, false, 12.6f, 1000);                // implausible → no confirmation
  CHECK(ladder.okConfirmed == false);
  cutoffStep(ladder, true, 9.9f, 2000);                  // below cutoff → no confirmation
  CHECK(ladder.okConfirmed == false);
  cutoffStep(ladder, true, CUTOFF_THRESHOLD_V, 3000);    // exactly 10.0 V confirms
  CHECK(ladder.okConfirmed == true);
}

TEST_CASE("cutoff latches after the debounce and just-latched fires exactly once") {
  BatteryLadderState ladder = freshLadder();
  CHECK(cutoffStep(ladder, true, 9.5f, 1000) == false);  // timer starts
  CHECK(ladder.cutoffStartMs == 1000);
  CHECK(cutoffStep(ladder, true, 9.5f, 1000 + CUTOFF_DEBOUNCE_MS - 1) == false);
  CHECK(cutoffStep(ladder, true, 9.5f, 1000 + CUTOFF_DEBOUNCE_MS) == true);
  CHECK(ladder.cutoffLatched == true);
  CHECK(cutoffStep(ladder, true, 9.5f, 4000) == false);  // still latched, no re-fire
  CHECK(cutoffStep(ladder, true, 12.6f, 5000) == false);  // recovery never unlatches
  CHECK(ladder.cutoffLatched == true);
  // Once latched the step is inert: it no longer confirms the boot gate.
  CHECK(ladder.okConfirmed == false);
  // Stage 2 never touches Stage 1's fields.
  CHECK(ladder.ecoLockLatched == false);
  CHECK(ladder.ecoLockStartMs == 0);
}

TEST_CASE("stage isolation: each step leaves the other stage's fields untouched") {
  // Sibling fields carry non-default sentinels, so an accidental overwrite
  // with defaults (which the in-case checks above cannot catch) fails here.
  BatteryLadderState ladder{false, true, true, 0, 4242};
  ecoStep(ladder, true, 10.5f, 1000);                    // Stage 1 runs to latch
  ecoStep(ladder, true, 10.5f, 1000 + ECO_LOCK_DEBOUNCE_MS);
  CHECK(ladder.ecoLockLatched == true);
  CHECK(ladder.cutoffLatched == true);                   // sentinels survived
  CHECK(ladder.okConfirmed == true);
  CHECK(ladder.cutoffStartMs == 4242);

  ladder = BatteryLadderState{true, false, false, 4242, 0};
  CHECK(cutoffStep(ladder, true, 9.5f, 1000) == false);  // Stage 2 runs to latch
  CHECK(cutoffStep(ladder, true, 9.5f, 1000 + CUTOFF_DEBOUNCE_MS) == true);
  CHECK(ladder.cutoffLatched == true);
  CHECK(ladder.ecoLockLatched == true);                  // sentinels survived
  CHECK(ladder.ecoLockStartMs == 4242);
}

TEST_CASE("cutoff: recovery or an implausible reading resets the debounce") {
  BatteryLadderState ladder = freshLadder();
  cutoffStep(ladder, true, 9.5f, 1000);
  cutoffStep(ladder, true, 10.5f, 2000);                 // recovered → reset
  CHECK(ladder.cutoffStartMs == 0);
  cutoffStep(ladder, true, 9.5f, 3000);                  // fresh window
  cutoffStep(ladder, false, 9.5f, 4000);                 // implausible → reset, NOT hold
  CHECK(ladder.cutoffStartMs == 0);
  cutoffStep(ladder, true, 9.5f, 5000);
  CHECK(cutoffStep(ladder, true, 9.5f, 5000 + CUTOFF_DEBOUNCE_MS) == true);
}
