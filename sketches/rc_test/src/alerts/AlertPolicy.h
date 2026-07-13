// alerts — which alarm sounds, and when (#183). Moved VERBATIM from
// rc_test.ino [ALERT]: inactivity tracking, the low-voltage debounce/latch
// (with ITS OWN plausibility form — deliberately NOT consolidated with
// domain/battery/VoltagePlausibility: the two diverge on NaN, a documented
// SAFETY.md gap; consolidation is a behavior change needing its own issue),
// and the four-level priority ladder. Pure logic: time is a parameter.
#pragma once

#include <stdint.h>

#include "AlertTypes.h"

// Track how long the RC has been off; returns true when the inactivity
// alarm should sound. Non-latching — clears when the RC comes back on.
bool inactivityAlarmStep(uint32_t& offSinceMs, bool rcOn, uint32_t nowMs,
                         uint32_t thresholdMs);

// Low-voltage alarm latch: worst-of-two packs, validity-gated, plausibility-
// banded, debounced, startup-suppressed; once latched it stays latched.
void lowVoltageLatchStep(LowVoltageLatchState& state, float voltage0,
                         float voltage1, bool valid0, bool valid1,
                         uint32_t nowMs, uint32_t bootMs,
                         const LowVoltageThresholds& thresholds);

// Pick the highest-priority active alarm (thermal-cut > low-V latched >
// thermal-warn > inactivity); returns {nullptr, 0} when silent. A cut is
// also hot, so the warning trill never plays while the cut alarm sounds.
AlertPattern alertPatternSelect(bool thermalCutActive, bool lowVoltageLatched,
                                bool thermalWarnActive, bool inactivityActive,
                                const AlertPatternTable& table);
