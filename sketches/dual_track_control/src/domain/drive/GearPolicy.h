// domain/drive — gear selection policy (#117 step 7, #166).
// Extracted VERBATIM from dual_track_control.ino [GEAR] updateGear() and the
// reverseCap()/outCapToX() helpers. Pure domain: the RC pulse, validity,
// safety forces and every tunable arrive as parameters (state-ownership
// rule — the sbus/safety-flag reads live in the caller).
#pragma once

#include <stdint.h>

#include "DriveTypes.h"

// The gear decision tree: RC invalid → Eco (failsafe); CH4 pulse below/above
// the thresholds → Eco/Boost; between → Normal. forceEco (battery Eco lock
// OR thermal Eco stage) overrides last, regardless of the switch.
GearSelection gearPolicySelect(int gearPulse, bool rcValid, bool forceEco,
                               const GearPolicyParameters& parameters);

// Per-gear reverse cap (#113): Boost allows more reverse authority than
// Eco/Normal. Single source of truth for the reverse limit.
float reverseCapForGear(GearLevel level, float standardCap, float boostCap);

// Convert a MAX TRACK-OUTPUT cap (0..1) to the xSpeed domain for the given
// gear scale (output = xSpeed * gearScale). Single point of truth for every
// cap conversion.
float trackCapToAxisDomain(float outputCap, float gearScale);
