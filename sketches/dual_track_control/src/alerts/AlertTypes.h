// alerts — shared types for the alert policy and pattern players (#183).
// Domain-ish, no hardware: the piezo pin lives behind ports/AlertOutputPort.h
// and the .ino shim executes the final OR (horn || pattern || alarm).
#pragma once

#include <stdint.h>

// One beep pattern: ON/OFF phase durations in ms, starting with ON.
struct AlertPattern {
  const uint16_t* sequence;
  int length;
};

// One-shot pattern playback (e.g. the Wi-Fi-ready "beep beep"): plays the
// sequence once, then stays silent. index -1 = idle (boot state).
struct OneShotPatternState {
  const uint16_t* sequence;
  int length;
  int index;
  uint32_t phaseMs;
};

// Repeating alarm playback: loops the sequence until a different (or no)
// alarm is selected.
struct RepeatingPatternState {
  const uint16_t* sequence;
  int length;
  int index;
  uint32_t phaseMs;
};

// Low-voltage alarm latch: once latched it stays until power cycle.
struct LowVoltageLatchState {
  uint32_t belowSinceMs;  // when the worst pack first dipped low (0 = above)
  bool latched;
};

// Low-voltage alarm thresholds — the ALARM band, distinct on purpose from the
// motor-cutoff ladder in domain/battery/ (different thresholds, different
// plausibility form; see docs/SAFETY.md).
struct LowVoltageThresholds {
  float alarmV;
  float plausibleMinV;
  float plausibleMaxV;
  uint32_t debounceMs;
  uint32_t startupGraceMs;
};

// The four repeating alarms in priority order (highest first).
struct AlertPatternTable {
  AlertPattern thermalCut;
  AlertPattern lowVoltage;
  AlertPattern thermalWarn;
  AlertPattern inactivity;
};
