// domain/battery — staged low-battery protection ladder (#117 step 2, #154).
// Logic extracted VERBATIM from dual_track_control.ino [SAFETY] batteryEcoLockUpdate() /
// batteryCutoffUpdate() (#65). Pure domain: no Arduino includes; time and
// thresholds arrive as parameters (state-ownership rules: time is a
// parameter, dependencies are visible in the signature).
#pragma once

#include <stdint.h>

// The ladder's mutable state — owned by this module, held by the application
// layer (today: mirrored to/from dual_track_control.ino globals by the delegate shims,
// the interim strangler-fig arrangement until the composition root exists).
// Both latches hold until power-cycle; a debounce timer of 0 means "not
// currently below threshold".
struct BatteryLadderState {
  bool ecoLockLatched;      // Stage 1: worst pack sustained below Eco-lock threshold
  bool cutoffLatched;       // Stage 2: worst pack sustained below cutoff threshold
  bool okConfirmed;         // boot gate: a plausible reading confirmed pack ABOVE cutoff
  uint32_t ecoLockStartMs;  // Stage 1 debounce timer
  uint32_t cutoffStartMs;   // Stage 2 debounce timer
};

// Stage 1 — force Eco. Latches ecoLockLatched once worstV stays below
// ecoLockThresholdV for ecoLockDebounceMs. An implausible reading resets the
// debounce timer (a not-yet-powered pack must not trip anything).
// `plausible` / `worstV` come from the VoltagePlausibility gate.
void batteryEcoLockStep(BatteryLadderState& ladder, bool plausible,
                        float worstV, uint32_t nowMs,
                        float ecoLockThresholdV, uint32_t ecoLockDebounceMs);

// Stage 2 — hard cutoff + boot gate. Same debounce shape as Stage 1, plus:
// a plausible reading at or above cutoffThresholdV confirms the pack
// (okConfirmed, never revoked here). Returns true exactly when the cutoff
// latches on THIS call, so the application layer can start the low-voltage
// alarm WITH the cut (never a silent cutoff) — [ALERT] owns its own latch;
// domain code never writes another module's state.
bool batteryCutoffStep(BatteryLadderState& ladder, bool plausible,
                       float worstV, uint32_t nowMs,
                       float cutoffThresholdV, uint32_t cutoffDebounceMs);
