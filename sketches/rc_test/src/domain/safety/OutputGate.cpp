// domain/safety — the fail-safe output gate (#117 step 9, #170).
// Logic copied VERBATIM from rc_test.ino [OUTPUT] outputUpdate(); behavior
// is locked by tests/characterization/test_output_gate.cpp, the
// output-bounds / cutoff-dominates / stale-RC invariants and
// tests/safety/test_output_gate_domain.cpp.
#include "OutputGate.h"

OutputGateAction outputGateStep(OutputGateState& gate, bool driveAllowed,
                                int commandedLeft, int commandedRight,
                                int currentLeft, int currentRight,
                                uint32_t nowMs, int neutralPulse,
                                uint32_t holdDurationMs) {
  OutputGateAction action{false, false, 0, 0, false};
  if (driveAllowed) {
    if (gate.mode == OUTPUT_GATE_CUT) {  // resume after an RC-loss cut
      action.attach = true;
    }
    gate.mode = OUTPUT_GATE_ACTIVE;
    action.write = true;
    action.leftPulse = commandedLeft;
    action.rightPulse = commandedRight;
    return action;
  }
  // Disallowed: ease out to neutral from the live command, then cut the PWM.
  if (gate.mode == OUTPUT_GATE_ACTIVE) {
    gate.mode = OUTPUT_GATE_HOLD;
    gate.holdStartMs = nowMs;
    gate.rampFromLeft = currentLeft;    // ease out FROM wherever the tracks are
    gate.rampFromRight = currentRight;
  }
  if (gate.mode == OUTPUT_GATE_HOLD) {
    float t = (holdDurationMs > 0)
                  ? (float)(nowMs - gate.holdStartMs) / (float)holdDurationMs
                  : 1.0f;
    if (t > 1.0f) t = 1.0f;
    action.write = true;
    action.leftPulse =
        gate.rampFromLeft + (int)((neutralPulse - gate.rampFromLeft) * t);
    action.rightPulse =
        gate.rampFromRight + (int)((neutralPulse - gate.rampFromRight) * t);
    if (t >= 1.0f) {
      action.detach = true;             // at neutral → stop pulsing → ESCs beep
      gate.mode = OUTPUT_GATE_CUT;
    }
  }
  // OUTPUT_GATE_CUT: emit nothing (no writes while detached).
  return action;
}
