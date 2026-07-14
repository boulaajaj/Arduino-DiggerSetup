// application — the firmware's mutable cross-module state + the gear-cap
// delegates (#189, moved verbatim from the .ino [CONFIG] block). One module
// owns each write (see the per-field comments); everything else reads.
// types.h is the interim shared-types bridge until it dissolves (Phase E).
#pragma once

#include <stdint.h>

#include "../../types.h"
#include "../config/DriveConfig.h"
#include "../domain/drive/GearPolicy.h"

extern float gearScale;
extern Gear  currentGear;
extern bool batteryCutoffLatched;
extern bool ecoLockLatched;
extern bool batteryOkConfirmed;
extern bool tempWarnActive;
extern bool tempEcoActive;
extern bool tempCutActive;

// The cap helpers and gear decision tree are extracted to
// src/domain/drive/GearPolicy.* (#117 step 7, #166); these delegates keep
// every call site unchanged and own the global reads (gearScale,
// currentGear) visibly. The domain GearLevel enum mirrors types.h's Gear
// values — proven at compile time:
static_assert((int)GEAR_LOW == (int)GEAR_ECO &&
              (int)GEAR_MID == (int)GEAR_NORMAL &&
              (int)GEAR_HIGH == (int)GEAR_BOOST,
              "types.h Gear and domain GearLevel values must correspond");

// Convert a MAX TRACK-OUTPUT cap (0..1) to the xSpeed domain for the current
// gear (output = xSpeed * gearScale). Single point of truth for every cap.
inline float outCapToX(float outCap) {
  return trackCapToAxisDomain(outCap, gearScale);
}

// Per-gear reverse cap (#113): Eco/Normal hold 55%, Boost allows 65%. Single
// source of truth — both rcCommand() and the joystick clamp read this, so the
// reverse limit lives in exactly one place.
inline float reverseCap() {
  return reverseCapForGear((GearLevel)currentGear, REVERSE_CAP_STD,
                           REVERSE_CAP_BOOST);
}


