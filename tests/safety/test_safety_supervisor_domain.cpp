// Pure-domain suite (#117 step 8, #168): src/domain/safety/SafetySupervisor
// tested directly on the host — no firmware, no stubs. End-to-end behavior
// through the loop() gate stays covered by
// tests/characterization/test_output_gate.cpp and the cutoff/stale-RC
// invariants; this suite locks the decision truth table, the fail-open
// boot-grace law, and the reason priority at the unit level.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/domain/safety/SafetySupervisor.h"

// SafetyInputs positional order:
// {rcValid, batteryConfirmed, bootGraceElapsed, batteryCutoffLatched,
//  thermalCutActive}

TEST_CASE("all healthy: drive allowed with no failsafe reason") {
  SafetyDecision decision =
      safetySupervisorDecide(SafetyInputs{true, true, false, false, false});
  CHECK(decision.driveAllowed == true);
  CHECK(decision.reason == FAILSAFE_NONE);
}

TEST_CASE("each single failure denies with its own reason") {
  CHECK(safetySupervisorDecide(SafetyInputs{false, true, false, false, false})
            .reason == FAILSAFE_RC_STALE);
  CHECK(safetySupervisorDecide(SafetyInputs{true, false, false, false, false})
            .reason == FAILSAFE_BOOT_GATE);
  CHECK(safetySupervisorDecide(SafetyInputs{true, true, false, true, false})
            .reason == FAILSAFE_BATTERY_CUTOFF);
  CHECK(safetySupervisorDecide(SafetyInputs{true, true, false, false, true})
            .reason == FAILSAFE_THERMAL_CUT);
  CHECK(safetySupervisorDecide(SafetyInputs{false, true, false, false, false})
            .driveAllowed == false);
}

TEST_CASE("fail-open boot gate law (#65): grace elapsed allows an unconfirmed pack") {
  // A dead X.BUS never permanently disables driving.
  SafetyDecision decision =
      safetySupervisorDecide(SafetyInputs{true, false, true, false, false});
  CHECK(decision.driveAllowed == true);
  CHECK(decision.reason == FAILSAFE_NONE);
  // But a latched cutoff still dominates the grace window.
  CHECK(safetySupervisorDecide(SafetyInputs{true, false, true, true, false})
            .driveAllowed == false);
}

TEST_CASE("reason priority mirrors the original && chain order") {
  // RC stale wins over everything.
  CHECK(safetySupervisorDecide(SafetyInputs{false, false, false, true, true})
            .reason == FAILSAFE_RC_STALE);
  // Boot gate beats cutoff and thermal.
  CHECK(safetySupervisorDecide(SafetyInputs{true, false, false, true, true})
            .reason == FAILSAFE_BOOT_GATE);
  // Cutoff beats thermal.
  CHECK(safetySupervisorDecide(SafetyInputs{true, true, false, true, true})
            .reason == FAILSAFE_BATTERY_CUTOFF);
}

TEST_CASE("driveAllowed equals the original boolean chain for ALL 32 inputs") {
  for (int mask = 0; mask < 32; mask++) {
    SafetyInputs inputs{(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0,
                        (mask & 8) != 0, (mask & 16) != 0};
    bool batteryReady = inputs.batteryConfirmed || inputs.bootGraceElapsed;
    bool expected = inputs.rcValid && batteryReady &&
                    !inputs.batteryCutoffLatched && !inputs.thermalCutActive;
    CHECK(safetySupervisorDecide(inputs).driveAllowed == expected);
  }
}
