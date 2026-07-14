// Pure-domain suite (#117 step 9, #170): src/domain/safety/OutputGate
// tested directly on the host — no firmware, no stubs, time/neutral/hold
// duration passed explicitly. End-to-end behavior through outputUpdate()
// (with the real Servo attach/detach) stays covered by
// tests/characterization/test_output_gate.cpp and the output-bounds /
// cutoff-dominates / stale-RC invariants; this suite locks the state
// machine's laws at the unit level.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/domain/safety/OutputGate.h"

// Values mirror SVC / CUTOFF_HOLD_MS — parameters here.
const int NEUTRAL = 1500;
const uint32_t HOLD_MS = 500;

inline OutputGateState activeGate() {
  return OutputGateState{OUTPUT_GATE_ACTIVE, 0, NEUTRAL, NEUTRAL};
}

TEST_CASE("allowed: commands pass through, no hardware edges while ACTIVE") {
  OutputGateState gate = activeGate();
  OutputGateAction action =
      outputGateStep(gate, true, 1700, 1300, 1700, 1300, 1000, NEUTRAL,
                     HOLD_MS);
  CHECK(action.write == true);
  CHECK(action.leftPulse == 1700);
  CHECK(action.rightPulse == 1300);
  CHECK(action.attach == false);
  CHECK(action.detach == false);
  CHECK(gate.mode == OUTPUT_GATE_ACTIVE);
}

TEST_CASE("deny from ACTIVE: HOLD snapshots the LIVE outputs and eases toward neutral") {
  OutputGateState gate = activeGate();
  // Live outputs 1800/1200 (passed as current), stale command irrelevant.
  OutputGateAction start =
      outputGateStep(gate, false, 1700, 1300, 1800, 1200, 1000, NEUTRAL,
                     HOLD_MS);
  CHECK(gate.mode == OUTPUT_GATE_HOLD);
  CHECK(gate.rampFromLeft == 1800);
  CHECK(start.write == true);
  CHECK(start.leftPulse == 1800);        // t = 0: still at the snapshot
  CHECK(start.detach == false);
  // Halfway through the hold window: midpoint of the ramp.
  OutputGateAction mid =
      outputGateStep(gate, false, 0, 0, 1800, 1200, 1250, NEUTRAL, HOLD_MS);
  CHECK(mid.leftPulse == 1650);          // 1800 + (1500-1800)*0.5
  CHECK(mid.rightPulse == 1350);
  CHECK(mid.detach == false);
  // Past the window: final neutral write AND the detach edge, then CUT.
  OutputGateAction done =
      outputGateStep(gate, false, 0, 0, 1800, 1200, 1600, NEUTRAL, HOLD_MS);
  CHECK(done.write == true);
  CHECK(done.leftPulse == NEUTRAL);
  CHECK(done.detach == true);
  CHECK(gate.mode == OUTPUT_GATE_CUT);
}

TEST_CASE("zero hold duration cuts immediately with one neutral write") {
  OutputGateState gate = activeGate();
  OutputGateAction action =
      outputGateStep(gate, false, 0, 0, 1900, 1100, 1000, NEUTRAL, 0);
  CHECK(action.write == true);
  CHECK(action.leftPulse == NEUTRAL);
  CHECK(action.detach == true);
  CHECK(gate.mode == OUTPUT_GATE_CUT);
}

TEST_CASE("CUT is inert: no writes, no edges, while denied") {
  OutputGateState gate{OUTPUT_GATE_CUT, 1000, 1800, 1200};
  OutputGateAction action =
      outputGateStep(gate, false, 1700, 1300, NEUTRAL, NEUTRAL, 5000,
                     NEUTRAL, HOLD_MS);
  CHECK(action.write == false);
  CHECK(action.attach == false);
  CHECK(action.detach == false);
  CHECK(gate.mode == OUTPUT_GATE_CUT);
}

TEST_CASE("resume from CUT re-attaches; recovery mid-HOLD does NOT (never detached)") {
  OutputGateState cut{OUTPUT_GATE_CUT, 1000, 1800, 1200};
  OutputGateAction resumed =
      outputGateStep(cut, true, 1600, 1400, NEUTRAL, NEUTRAL, 6000, NEUTRAL,
                     HOLD_MS);
  CHECK(resumed.attach == true);         // RC-loss recovery re-attaches
  CHECK(resumed.write == true);
  CHECK(cut.mode == OUTPUT_GATE_ACTIVE);
  OutputGateState hold{OUTPUT_GATE_HOLD, 1000, 1800, 1200};
  OutputGateAction recovered =
      outputGateStep(hold, true, 1600, 1400, 1650, 1350, 1200, NEUTRAL,
                     HOLD_MS);
  CHECK(recovered.attach == false);      // ESCs were never detached
  CHECK(recovered.write == true);
  CHECK(recovered.leftPulse == 1600);
  CHECK(hold.mode == OUTPUT_GATE_ACTIVE);
}
