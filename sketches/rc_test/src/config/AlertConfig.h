// config — beep/alarm patterns + audible-alert tunables (#185, #51/#68).
#pragma once

#include <stdint.h>

const uint16_t BEEP_WIFI_READY[]   = {180, 140, 180};  // Wi-Fi-ready "beep beep": on, off, on (ms)
const int      BEEP_WIFI_READY_LEN = 3;                // phases in BEEP_WIFI_READY[]

// [ALERT] repeating alarm patterns (ms, starting with ON; played as a loop).
// Distinct on purpose so each is identifiable by ear — see OPERATOR-GUIDE.md.
const uint16_t ALERT_INACT[]   = {500, 1500};                   // one long beep / 2 s  (RC off)
const int      ALERT_INACT_LEN = 2;
const uint16_t ALERT_LOWV[]    = {120, 120, 120, 120, 120, 600};  // three fast chirps / ~1.2 s (low batt)
const int      ALERT_LOWV_LEN  = 6;
// Motor/ESC over-temp patterns (#111) — chosen to be unmistakable by ear vs the
// four above (2 short once / 1 long / 3 short fast / continuous horn):
const uint16_t ALERT_THERM_WARN[]   = {100, 100};                      // fast nonstop TRILL (>= 80 C)
const int      ALERT_THERM_WARN_LEN = 2;
const uint16_t ALERT_THERM_CUT[]    = {500, 150, 500, 150, 500, 1000};  // three LONG beeps (>= 95 C cut)
const int      ALERT_THERM_CUT_LEN  = 6;
const uint16_t BEEP_THERM_RESTORED[]   = {120, 100, 120, 100, 120, 100, 120, 1500};  // 4 quick flourish, one-shot (< 75 C)
const int      BEEP_THERM_RESTORED_LEN = 8;

// [ALERT] tunables
const uint32_t INACT_RC_OFF_MS  = 60000UL;  // RC off this long → inactivity beep ("unplug me")
const float    LOWV_THRESH_V    = 10.5f;   // worst-of-two pack EMA below this → low-batt alarm (heads-up beep before the 10.0 V cutoff)
const float    LOWV_PLAUS_MIN_V = 6.0f;    // pack reading below this = not present / bad → ignore
const float    LOWV_PLAUS_MAX_V = 13.0f;   // pack reading above this = bad read → ignore
const uint32_t LOWV_DEBOUNCE_MS = 3000UL;  // must stay below thresh this long before latching
const uint32_t ALERT_STARTUP_MS = 60000UL;  // suppress low-V alarm until telemetry/EMA settles
