// domain/thermal — shared value types (#117 step 3, #158).
// Pure domain: no Arduino includes (host-compilable by design).
#pragma once

#include <stdint.h>

// One temperature sensor's report, as delivered by X.BUS telemetry.
// `valid` mirrors EscTelem.valid — the freshness watchdog's verdict for the
// ESC carrying the sensor; plausibility is ThermalDerating's job.
struct ThermalReading {
  float temperatureC;
  bool valid;
};

// One hysteresis stage: flips ON after the trip debounce, OFF immediately
// below the release threshold. sinceMs is the trip-debounce start timestamp,
// meaningful only while inactive (0 = debounce not running); it is not
// consulted while the stage is active.
struct ThermalStageState {
  bool active;
  uint32_t sinceMs;
};

// The three derating stages (#111). NON-LATCHING with hysteresis by design:
// heat is not a drained LiPo — once cool, drive resumes (deliberate
// asymmetry vs the battery ladder's latched cutoff; both test-locked).
struct ThermalDeratingState {
  ThermalStageState warn;
  ThermalStageState eco;
  ThermalStageState cut;
};

// On/off temperature pairs + trip debounce, mirrored from [CONFIG]
// (TEMP_*_ON_C / TEMP_*_OFF_C / TEMP_DEBOUNCE_MS) — parameters here: the
// domain has no config dependency.
struct ThermalDeratingThresholds {
  float warnOnC, warnOffC;
  float ecoOnC, ecoOffC;
  float cutOnC, cutOffC;
  uint32_t debounceMs;
};
