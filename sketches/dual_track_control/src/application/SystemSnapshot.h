// application — the immutable per-cycle observation of the whole system
// (#132). Built by the application at the observation point (today: the
// .ino observer shims, for exact timing parity; the once-per-cycle build
// moves into FirmwareApp at step 11) and passed const& to the observer
// encoders (telemetry/JsonEncoder, telemetry/CsvEncoder). Observers cannot
// write and never reach into module internals — the architecture checker
// allows exactly this header as the observers' one application/ include.
#pragma once

#include <stdint.h>

#include "../ports/TelemetrySource.h"

struct SystemSnapshot {
  uint32_t nowMs;      // observation time (drives the per-ESC age fields)

  // Operator inputs as observed: RC channels in servo-µs view (neutral when
  // the S.BUS link is invalid — matching what the debug CSV always showed)
  // plus the raw joystick ADC pair.
  int rcThrottleUs;
  int rcSteeringUs;
  int rcGearUs;
  int rcOverrideUs;
  bool rcFailsafe;
  bool rcLostFrame;
  int joystickRawY;
  int joystickRawX;

  // Command outputs + operating mode.
  int outLeftUs;
  int outRightUs;
  int gear;            // Gear enum value as an integer (0 Eco / 1 Normal / 2 Boost)
  int overrideMode;    // wifiMode() result as the dashboard shows it

  // Safety flags.
  bool batteryCutoffLatched;
  bool ecoLockLatched;
  bool temperatureWarnActive;
  bool temperatureEcoActive;
  bool temperatureCutActive;

  // Per-ESC telemetry copies (EscTelem from the TelemetrySource port).
  EscTelem esc[2];
};
