// domain/drive — shared value types (#117 step 6, #164).
// Pure domain: no Arduino includes (host-compilable by design).
#pragma once

// Per-track normalized command, -1..+1 (post-gear, pre-PWM). Layout-identical
// to the sketch's WheelSpeeds; this is the target-glossary name the domain
// uses.
struct TrackCommand {
  float left;
  float right;
};

// The curvature-mix tunables, mirrored from [CONFIG] — parameters here: the
// domain has no config or global dependency. pivotCap arrives PRE-SELECTED
// for the current gear (the application layer owns the currentGear read).
struct CurvatureParameters {
  float pivotCap;            // pivot counter-rotation cap, gear-selected
  float pivotThrottleTaper;  // pivot-branch throttle taper by |steering| (#114)
  float pivotBlendStart;     // |xSpeed| where the pivot→curvature blend begins
  float pivotBlendEnd;       // |xSpeed| where the blend reaches pure curvature
  float turnTrackCap;        // outer-track ceiling at full steer (#96)
};
