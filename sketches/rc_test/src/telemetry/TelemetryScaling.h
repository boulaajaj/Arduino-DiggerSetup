// telemetry/TelemetryScaling.h — pure telemetry scaling math (#117 step 4,
// #160). Extracted VERBATIM from rc_test.ino: the X.BUS register decode in
// [TELEMETRY] telemApplyReg() and the compact-integer wire encode in [WIFI]
// buildTelemJson(). Header-only observer-layer helpers: no Arduino includes,
// no state — the ONE implementation of the ×10 wire scaling (the dashboard
// JS divides by 10 on its side; that mapping is documented at the encoder).
#pragma once

#include <stdint.h>
#include <math.h>

// ── X.BUS register decode: raw register value → engineering units ────────

inline float vbatRawToVolts(uint16_t raw) {          // REG_VBAT 0x0C, ×0.1 V
  return raw * 0.1f;
}

inline float busCurrentRawToAmps(uint16_t raw) {     // REG_IBUS 0x0D, ×0.1 A signed
  return (int16_t)raw * 0.1f;
}

inline int16_t motorSpeedRawToElectricalHz(uint16_t raw) {  // REG_MSPEED 0x02, signed
  return (int16_t)raw;
}

inline float temperatureRawToCelsius(uint16_t raw) {  // REG_TESC/TMOT: low byte, raw − 40
  return (float)(raw & 0xFF) - 40.0f;
}

// Exponential moving average fold; the first sample seeds the average.
inline float emaFold(float previous, float sample, float alpha, bool first) {
  return first ? sample : previous + alpha * (sample - previous);
}

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
