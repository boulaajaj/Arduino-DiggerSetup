// domain/thermal — warn/eco/cut staging off the hottest sensor (#117 step 3,
// #158). Logic extracted VERBATIM from rc_test.ino [SAFETY]
// hottestMotorTemp() / thermalUpdate(). Pure domain: readings, time and
// thresholds arrive as parameters.
#pragma once

#include <stdint.h>

#include "ThermalTypes.h"

// Hottest plausible temperature across the given readings. Returns false
// when no reading is both fresh (valid) and inside [plausibleMinC,
// plausibleMaxC] — the out-param is left unwritten and the caller HOLDS
// state (a telemetry dropout never cuts and never releases).
bool hottestPlausibleTemperature(const ThermalReading* readings,
                                 uint8_t count,
                                 float plausibleMinC, float plausibleMaxC,
                                 float* hottestOut);

// One derating pass over all three stages. With haveReading false every
// stage HOLDS — the deliberate dropout asymmetry vs the battery ladder's
// timer reset (docs/SAFETY.md). Returns true exactly when the cut stage
// releases on THIS call, so the application layer can queue the "restored"
// flourish — [BEEPER] owns its own patterns; domain code never calls
// another module.
bool thermalDeratingStep(ThermalDeratingState& derating, bool haveReading,
                         float hottestC, uint32_t nowMs,
                         const ThermalDeratingThresholds& thresholds);
