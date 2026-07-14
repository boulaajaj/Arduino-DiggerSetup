// domain/operator_input — exponential input curve (#117 step 5, #162).
// Extracted VERBATIM from dual_track_control.ino [JOYSTICK] expoCurve(). Pure domain:
// no Arduino includes; weights arrive as parameters ([CONFIG] owns the
// throttle/steering pairs).
#pragma once

// Magnitude-only expo shaping: linearWeight·|x| + cubicWeight·|x|³.
// The CALLER applies the sign (and any direction/gain factors) — this
// function never returns a negative value.
float expoCurve(float x, float linearWeight, float cubicWeight);
