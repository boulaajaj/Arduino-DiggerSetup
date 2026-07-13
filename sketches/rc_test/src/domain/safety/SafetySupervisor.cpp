// domain/safety — the drive ALLOW/DENY supervisor (#117 step 8, #168).
// Behavior is locked by tests/characterization/test_output_gate.cpp,
// tests/invariants/test_invariant_cutoff_dominates.cpp,
// test_invariant_stale_rc_neutral.cpp and
// tests/safety/test_safety_supervisor_domain.cpp.
#include "SafetySupervisor.h"

SafetyDecision safetySupervisorDecide(const SafetyInputs& inputs) {
  bool batteryReady = inputs.batteryConfirmed || inputs.bootGraceElapsed;
  if (!inputs.rcValid) {
    return {false, FAILSAFE_RC_STALE};
  }
  if (!batteryReady) {
    return {false, FAILSAFE_BOOT_GATE};
  }
  if (inputs.batteryCutoffLatched) {
    return {false, FAILSAFE_BATTERY_CUTOFF};
  }
  if (inputs.thermalCutActive) {
    return {false, FAILSAFE_THERMAL_CUT};
  }
  return {true, FAILSAFE_NONE};
}
