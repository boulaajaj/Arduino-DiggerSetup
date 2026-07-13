// telemetry/TelemetryScaling.h — pure wire-encode math (#117 step 4, #160).
// Extracted VERBATIM from rc_test.ino [WIFI] buildTelemJson(). Header-only
// observer-layer helpers: no Arduino includes, no state — the ONE
// implementation of the ×10 wire scaling (the dashboard JS divides by 10 on
// its side; that mapping is documented at the encoder). The X.BUS register
// DECODE half moved to infrastructure/xc/XbusTelemetryAdapter.h (#178):
// infrastructure/ may not include telemetry/, and the decode belongs with
// the X.BUS register semantics.
#pragma once

#include <stdint.h>
#include <math.h>

// ── wire encode: engineering units → compact SSE/JSON integers ───────────
// Compact keys and integer values are a deliberate bandwidth decision on the
// SSE frame budget (naming rules, exception 4).

inline int scaleToDeciInteger(float value) {         // volts/amps → tenths
  return (int)lroundf(value * 10.0f);
}

inline int roundToWholeInteger(float value) {        // temperatures → whole °C
  return (int)lroundf(value);
}

inline long electricalHzToDashboardRpm(int16_t electricalHz) {
  return (long)electricalHz * 30;
}
