// domain/battery — worst-of-two pack-voltage plausibility gate (#117 step 1).
// Extracted VERBATIM from dual_track_control.ino [SAFETY] worstPackVoltage() — the
// battery ladder's single trusted voltage source. Pure domain: no Arduino
// includes; bounds arrive as parameters (state-ownership rule: dependencies
// are visible in the signature).
#pragma once

#include "BatteryTypes.h"

// Worst-of-two pack voltage if BOTH packs read a plausible value; else false
// (a not-yet-powered pack reads ~0 V and must not trip anything).
//
// KNOWN GAP (#131 FINDING, locked as current behavior): a NaN voltage passes
// the [min,max] band (both comparisons are false) and the ternary min-select
// is slot-asymmetric under NaN. Do NOT "fix" here — that is a behavior
// change needing its own issue + operator sign-off.
bool worstPlausiblePackVoltage(const BatteryReading& pack0,
                               const BatteryReading& pack1,
                               float plausibleMinV, float plausibleMaxV,
                               float* worstOut);
