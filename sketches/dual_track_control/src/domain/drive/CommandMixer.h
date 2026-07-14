// domain/drive — RC/joystick command mixer (#117 step 7, #166).
// Extracted VERBATIM from dual_track_control.ino [MIXER]. Pure domain: the override
// pulse and its thresholds arrive as parameters.
#pragma once

#include "DriveTypes.h"

// Max/oppose combine for one axis (#90): the stronger same-direction input
// wins (NOT summed), opposing inputs subtract.
// result = max(a,b,0) + min(a,b,0).
//   same dir  60% & 30% → 60%   (larger wins, no halving)
//   opposite +100% & −30% → 70% (dominant minus opposing)
//   single    x & 0 → x         (full range — fixes the old 50/50 halving)
float maxOppose(float a, float b);

// Combine RC + joystick at the AXIS level per override mode: pulse below
// the low threshold → Mode 1 (RC only); above the high threshold → Mode 3
// (dual: max/oppose per axis); between → Mode 2 (RC overrides the joystick
// whenever RC is active — exact-zero compare is locked behavior).
AxisCommand mixAxisCommands(AxisCommand rc, AxisCommand joystick,
                            int overridePulse, int overrideLowThreshold,
                            int overrideHighThreshold);
