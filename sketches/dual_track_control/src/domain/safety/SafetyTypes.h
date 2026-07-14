// domain/safety — shared value types (#117 step 8, #168).
// Pure domain: no Arduino includes (host-compilable by design).
#pragma once

#include <stdint.h>

// Why drive is denied (target glossary). Priority mirrors the original
// && chain: RC freshness, then the boot gate, then the battery cutoff,
// then the thermal cut.
enum FailsafeReason : uint8_t {
  FAILSAFE_NONE = 0,
  FAILSAFE_RC_STALE,
  FAILSAFE_BOOT_GATE,
  FAILSAFE_BATTERY_CUTOFF,
  FAILSAFE_THERMAL_CUT,
};

// Everything the supervisor needs, gathered by the application layer —
// including the boot-grace clock check (time is a parameter: the caller
// computes it, the domain never reads a clock).
struct SafetyInputs {
  bool rcValid;               // S.BUS fresh (per-channel failsafe upstream)
  bool batteryConfirmed;      // a plausible reading confirmed pack above cutoff
  bool bootGraceElapsed;      // fail-open window passed (dead X.BUS never
                              // permanently disables driving, #65)
  bool batteryCutoffLatched;  // hard cutoff latched until power-cycle (#65)
  bool thermalCutActive;      // over-temp cut, auto-recovers (#111)
};

// The supervisor's verdict. Today the application layer consumes only
// driveAllowed; `reason` feeds the OutputGate and SystemSnapshot steps.
struct SafetyDecision {
  bool driveAllowed;
  FailsafeReason reason;
};
