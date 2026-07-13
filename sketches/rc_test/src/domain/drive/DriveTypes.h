// domain/drive — shared value types (#117 steps 6-7, #164/#166).
// Pure domain: no Arduino includes (host-compilable by design).
#pragma once

#include <stdint.h>

// Per-track normalized command, -1..+1 (post-gear, pre-PWM). Layout-identical
// to the sketch's WheelSpeeds; this is the target-glossary name the domain
// uses.
struct TrackCommand {
  float left;
  float right;
};

// Per-axis operator command (pre-mix, pre-curvatureDrive). Layout-identical
// to the sketch's DriveCommand.
struct AxisCommand {
  float xSpeed;
  float zRotation;
};

// Gear levels, glossary-named. Numeric values intentionally MATCH the
// sketch's Gear enum (GEAR_LOW/MID/HIGH = 0/1/2) — the delegate shims cast
// between them and static_assert the correspondence.
enum GearLevel : uint8_t {
  GEAR_ECO    = 0,
  GEAR_NORMAL = 1,
  GEAR_BOOST  = 2,
};

// The gear policy tunables, mirrored from [CONFIG] — parameters here.
struct GearPolicyParameters {
  float ecoScale;
  float normalScale;
  float boostScale;
  int selectLowThreshold;   // CH4 pulse below this → Eco
  int selectHighThreshold;  // CH4 pulse above this → Boost
};

// One gear decision: the level and its average-speed scale.
struct GearSelection {
  GearLevel level;
  float scale;
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
