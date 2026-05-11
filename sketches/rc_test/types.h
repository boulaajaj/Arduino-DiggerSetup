// types.h — Shared data structures for Digger Control V7.5
#pragma once
#include <Arduino.h>

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

// Telemetry sample from one ESC (via X.BUS read register)
struct EscTelem {
  float    voltage;     // V (battery / bus voltage, EMA smoothed)
  float    motorTempC;  // degC (EMA smoothed)
  float    escTempC;    // degC (EMA smoothed)
  uint32_t lastGoodMs;  // millis() of last successful read
  bool     valid;       // has at least one good reading
};

// Beeper state (priority-ordered, higher value = higher priority)
enum BeepPattern : uint8_t {
  BEEP_SILENT      = 0,
  BEEP_REVERSE     = 1,  // 1s on, 1s off — backup-alarm style
  BEEP_BATTERY_30  = 2,  // single chirp every 10s
  BEEP_BATTERY_20  = 3,  // double chirp every 1s
  BEEP_OVERTEMP    = 4,  // double chirp twice per second
};

// CH4 gear position. Caps the post-curvatureDrive wheel speeds; the
// actual scale factors live in the [CONFIG] section of rc_test.ino as
// GEAR_{LOW,MID,HIGH}_SCALE so they're tunable in one place.
enum Gear : uint8_t {
  GEAR_LOW  = 0,   // 40% cap — Eco (training / tight spaces)
  GEAR_MID  = 1,   // 65% cap — Normal driving
  GEAR_HIGH = 2,   // 100% cap — Turbo (full throttle authority)
};
