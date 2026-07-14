// config — motor/ESC over-temperature staging tunables (#185, #111).
#pragma once

#include <stdint.h>

// [SAFETY] motor / ESC over-temperature protection (#111) — staged, NON-LATCHING
// with hysteresis. Heat is the OPPOSITE of a drained LiPo: once it cools, driving
// must resume automatically, so each stage releases on its own (lower) threshold
// rather than latching to power-cycle like the battery ladder. Source = HOTTEST
// of all 4 sensors (ESC + motor temp on BOTH ESCs), already EMA-smoothed
// (TELEMETRY_EMA_ALPHA_TEMPERATURE) + a short trip debounce so a single spike / dropped frame can't
// flip a stage. A telemetry dropout (no fresh valid reading) HOLDS the last state
// — it never triggers a cut.
//   Warning beep  >= 80 C  (release < 78 C) : trill, motors keep running
//   Eco lock      >= 90 C  (release < 80 C) : force Eco gear
//   Hard cutoff   >= 95 C  (release < 75 C) : ease PWM to neutral, keep beeping
// NOTE: 95 C is PROVISIONAL — confirm on the bench that it sits just BELOW the
// GL10's own internal thermal limit (param 17 = temp-controlled fan; the GL10's
// internal throttle/cutoff number is unpublished) so our warned graceful cut
// fires before the ESC's silent one.
const float    TEMP_WARN_ON_C    = 80.0f;
const float    TEMP_WARN_OFF_C   = 78.0f;
const float    TEMP_ECO_ON_C     = 90.0f;
const float    TEMP_ECO_OFF_C    = 80.0f;
const float    TEMP_CUT_ON_C     = 95.0f;
const float    TEMP_CUT_OFF_C    = 75.0f;
const uint32_t TEMP_DEBOUNCE_MS  = 1000UL;   // sustained past a TRIP threshold before the stage flips on
const float    TEMP_PLAUS_MIN_C  = -20.0f;   // reading below this = bad / unplugged sensor → ignore
const float    TEMP_PLAUS_MAX_C  = 200.0f;   // reading above this = bad read → ignore
