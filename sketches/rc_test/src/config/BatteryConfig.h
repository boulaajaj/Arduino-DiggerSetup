// config — the low-battery motor-protection ladder tunables (#185, #65).
#pragma once

#include <stdint.h>

// [SAFETY] battery motor-cutoff (#65) — a HARD stop below the audible low-V alarm.
// Worst-of-two pack EMA < CUTOFF_THRESH_V, validity-gated + debounced + latched →
// motors cut AND the D8 alarm chirps immediately (no startup grace; the validity
// gate alone guards the ~1 s telemetry warm-up). 10.0 V on a 3S pack = 3.33 V/cell
// average — conservative, above the ~3.0 V/cell damage line even with some cell
// imbalance. Latches until power-cycle. All three timings are tunable here.
const float    CUTOFF_THRESH_V    = 10.0f;   // worst pack EMA below this → cut motors
const uint32_t CUTOFF_DEBOUNCE_MS = 1500UL;  // sustained below thresh before cutting (ignore load sag; do NOT set 0)
const uint32_t CUTOFF_HOLD_MS     = 500UL;   // command neutral this long before cutting PWM (let GL10 wind down)
// Boot gate (#65): the cutoff latch lives in RAM, so a watchdog/brownout reset
// would clear it. To stop a low pack from driving in that window, the output is
// held OFF after boot until a valid battery reading confirms it's ABOVE the
// cutoff. If telemetry never reports (dead X.BUS), fail OPEN after this timeout
// so a telemetry fault can't permanently disable driving.
const uint32_t BATTERY_CONFIRM_MS = 3000UL;  // no valid battery within this → allow drive anyway (telemetry-optional)

// [SAFETY] low-battery Eco lockout (#65) — a STAGE BEFORE the hard cutoff. When
// the worst pack sags low for an extended period, force Eco gear regardless of
// the RC gear switch so a nearly-drained pack isn't hit with Boost/Normal load.
// Latches until power-cycle. 10.8 V ≈ 30% on a 3S pack — the LiPo "knee", so the
// top ~70% keeps full Boost/Normal and Eco only eases the final steep stretch.
// (Ladder: 10.8 V → Eco lock (15 s) · 10.5 V → beep · 10.0 V → hard cutoff.)
const float    ECO_LOCK_THRESH_V    = 10.8f;    // worst pack EMA below this → force Eco
const uint32_t ECO_LOCK_DEBOUNCE_MS = 15000UL;  // sustained below thresh before locking Eco
