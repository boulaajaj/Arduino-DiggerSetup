// application — output-stage implementations (#189), verbatim from the .ino.
#include "MotorOutput.h"

#include "RangeMath.h"
#include "../config/DriveConfig.h"
#include "../config/BatteryConfig.h"
#include "../config/Pins.h"
#include "../domain/safety/OutputGate.h"
#include "../ports/EscOutputPort.h"
#include "../ports/ClockPort.h"

// ═══════════════════════════════════════════════════════════════
// [OUTPUT] — ESC servo PWM (non-blocking)
// ═══════════════════════════════════════════════════════════════

// The Servo objects moved to infrastructure/arduino/PwmEscAdapter.cpp
// behind ports/EscOutputPort.h (#117 step 10 slice 1, #172). The clamp and
// the outL/outR globals stay HERE — application-visible output state (read
// by buildTelemJson and the OutputGate shim, reset by the test harness).
int outL = SVC, outR = SVC;

void outputInit() {
  escOutputInitialize(PIN_ESC_L, PIN_ESC_R);
  escOutputWritePulses(SVC, SVC);
}

void outputWrite(int left, int right) {
  outL = clampInt(left,  SVMIN, SVMAX);
  outR = clampInt(right, SVMIN, SVMAX);
  escOutputWritePulses(outL, outR);
}

// Fail-safe output gate (#88 / #65). The Arduino drives PWM ONLY when it has a
// valid reason to (driveAllowed = RC valid AND battery OK). Otherwise:
//   1. EASE OUT — ramp both tracks smoothly from wherever they are down to
//      neutral over CUTOFF_HOLD_MS (gentle controlled stop, no jerk), then
//   2. STOP pulsing entirely (detach) so the ESCs see no signal and beep.
// Easing from the live command (never holding it) keeps it safe whatever the
// GL10 does on lost signal — by the time pulses stop, output is already neutral.
// RC-loss recovers automatically when the signal returns; the battery cutoff
// latch keeps driveAllowed false until a power-cycle. "Get a command → pass it.
// Get nothing → ease to a stop, then pass nothing." (CUTOFF_HOLD_MS is in [CONFIG].)
OutState outState   = OUT_ACTIVE;
uint32_t outHoldMs  = 0;
int      rampFromL  = SVC;   // output captured when the gate closed (ease-out start)
int      rampFromR  = SVC;

// The state machine is extracted to src/domain/safety/OutputGate.* (#117
// step 9, #170); this delegate mirrors the gate globals, reads the clock,
// and executes the returned hardware actions in the original order:
// attach, then write, then detach (the final neutral write precedes the
// detach, so the ESCs see neutral before losing signal). The enum values
// correspond — proven at compile time:
static_assert((int)OUT_ACTIVE == (int)OUTPUT_GATE_ACTIVE &&
              (int)OUT_HOLD == (int)OUTPUT_GATE_HOLD &&
              (int)OUT_CUT == (int)OUTPUT_GATE_CUT,
              "OutState and domain OutputGateMode values must correspond");

void outputUpdate(bool driveAllowed, int mixL, int mixR) {
  OutputGateState gate{(OutputGateMode)outState, outHoldMs, rampFromL,
                       rampFromR};
  OutputGateAction action =
      outputGateStep(gate, driveAllowed, mixL, mixR, outL, outR, clockNowMs(),
                     SVC, CUTOFF_HOLD_MS);
  outState = (OutState)gate.mode;
  outHoldMs = gate.holdStartMs;
  rampFromL = gate.rampFromLeft;
  rampFromR = gate.rampFromRight;
  if (action.attach) escOutputAttach();
  if (action.write) outputWrite(action.leftPulse, action.rightPulse);
  if (action.detach) escOutputDetach();  // at neutral → stop pulsing → ESCs beep
}


