// types.h — Shared data structures for Digger Control V7.6
#pragma once
#include <Arduino.h>

// EscTelem moved to the TelemetrySource port (#178) — the port speaks the
// type it delivers; included here so every existing consumer keeps
// resolving it through types.h.
#include "src/ports/TelemetrySource.h"

// Joystick ADC reading after deadband + expo curve, normalized to -1..+1
struct JoystickState {
  int   rawY, rawX;     // Raw 14-bit ADC values
  float xSpeed;         // -1..+1 throttle (after deadband + expo)
  float zRotation;      // -1..+1 steering (after deadband + expo)
};

// Normalized command pair (one per input source) before mixing
struct DriveCommand {
  float xSpeed;         // -1..+1 (post-constrain on both RC and joystick paths)
  float zRotation;      // -1..+1
};

// Mixer output after override switch + gear
struct WheelSpeeds {
  float left;           // -1..+1 (post-gear, pre-PWM)
  float right;
};

// PWM output to ESCs
struct ServoOutput {
  int left;             // microseconds
  int right;
};

// CH4 gear position. Caps the post-curvatureDrive wheel speeds; the
// actual scale factors live in the [CONFIG] section of dual_track_control.ino as
// GEAR_{LOW,MID,HIGH}_SCALE so they're tunable in one place.
enum Gear : uint8_t {
  GEAR_LOW  = 0,   // 65% cap — Eco (training / tight spaces)
  GEAR_MID  = 1,   // 80% cap — Normal driving
  GEAR_HIGH = 2,   // 100% cap — Boost (full throttle authority)
};
