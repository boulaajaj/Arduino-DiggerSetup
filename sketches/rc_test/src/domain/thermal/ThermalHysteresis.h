// domain/thermal — one hysteresis stage with trip debounce (#117 step 3,
// #158). Logic extracted VERBATIM from rc_test.ino [SAFETY]
// tempStageUpdate(). Pure domain: time and thresholds arrive as parameters
// (state-ownership rules: time is a parameter, dependencies are visible in
// the signature).
#pragma once

#include <stdint.h>

#include "ThermalTypes.h"

// Flips ON when temperatureC stays >= onC for debounceMs; flips OFF
// immediately when temperatureC < offC (the on/off gap makes release
// chatter impossible, so release needs no debounce). Inside the gap
// (offC <= temperatureC < onC) an active stage HOLDS — that hold is
// test-locked behavior law.
void thermalStageStep(ThermalStageState& stage, float temperatureC,
                      uint32_t nowMs, float onC, float offC,
                      uint32_t debounceMs);
