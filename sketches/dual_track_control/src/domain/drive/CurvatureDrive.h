// domain/drive — curvature tank mix (#117 step 6, #164).
// Extracted VERBATIM from dual_track_control.ino [DRIVE] curvatureDrive() — the
// propulsion-path core (FRC/WPILib-proven algorithm: smoothstep pivot blend
// #72, reverse-steering consistency #86, faded turn headroom #96, pivot
// throttle taper #114). Pure domain: no Arduino includes; every tunable
// arrives as a parameter, and the gear-selected pivotCap is passed in
// (state-ownership rule: dependencies are visible in the signature — the
// currentGear read lives in the caller, fixing the documented sin).
#pragma once

#include "DriveTypes.h"

// Gear-aware tank mix. gearScale (Eco/Normal/Boost) caps the AVERAGE forward
// speed — NOT each track — so in a turn the outer track may use the headroom
// up to the ESC limit (±1.0) and the turn keeps its speed instead of slowing
// (#72). At Boost (gearScale = 1.0) there is no headroom, so the outer simply
// desaturates at the rail. Pivot is gear-scaled so low gears spin gently.
TrackCommand curvatureDriveStep(float xSpeed, float zRotation,
                                float gearScale,
                                const CurvatureParameters& parameters);
