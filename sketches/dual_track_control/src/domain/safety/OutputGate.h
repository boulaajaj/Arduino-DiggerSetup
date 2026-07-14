// domain/safety — the fail-safe output gate (#117 step 9, #170).
// Extracted VERBATIM from dual_track_control.ino [OUTPUT] outputUpdate() (#88/#65).
// ACTIVE / HOLD (ease to neutral) / CUT (no pulses): "Get a command → pass
// it. Get nothing → ease to a stop, then pass nothing." Pure domain: time,
// the neutral pulse and the hold duration arrive as parameters; hardware
// effects come back as an action for the caller to execute — domain code
// never touches the Servo objects.
#pragma once

#include <stdint.h>

// Gate modes. Numeric values intentionally MATCH the sketch's OutState
// enum (OUT_ACTIVE/OUT_HOLD/OUT_CUT = 0/1/2) — the delegate shim casts
// between them and static_asserts the correspondence.
enum OutputGateMode : uint8_t {
  OUTPUT_GATE_ACTIVE = 0,
  OUTPUT_GATE_HOLD   = 1,
  OUTPUT_GATE_CUT    = 2,
};

// The gate's mutable state, held by the application layer (today: mirrored
// to/from the dual_track_control.ino globals by the delegate shim).
struct OutputGateState {
  OutputGateMode mode;
  uint32_t holdStartMs;   // when the ease-out began
  int rampFromLeft;       // outputs captured when the gate closed (ease-out start)
  int rampFromRight;
};

// What the caller must do this pass, in this order: attach, then write,
// then detach (the final neutral write precedes the detach, so the ESCs
// see neutral before losing signal).
struct OutputGateAction {
  bool attach;      // re-attach the ESCs (resume-from-CUT edge only)
  bool write;       // write leftPulse/rightPulse this pass
  int leftPulse;
  int rightPulse;
  bool detach;      // stop pulsing (ease-out completed)
};

// One gate pass. currentLeft/currentRight are the LIVE last-written outputs
// (the ease-out starts from wherever the tracks are, never from a stale
// command). Recovery mid-HOLD needs no re-attach — the ESCs were never
// detached (locked law).
OutputGateAction outputGateStep(OutputGateState& gate, bool driveAllowed,
                                int commandedLeft, int commandedRight,
                                int currentLeft, int currentRight,
                                uint32_t nowMs, int neutralPulse,
                                uint32_t holdDurationMs);
