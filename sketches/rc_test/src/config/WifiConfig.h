// config — Wi-Fi identity (#185). Monitoring only — never affects control.
// Credentials stay in arduino_secrets.h (gitignored, #125), sketch-side;
// the serving tunables live with infrastructure/network/WifiService.h.
#pragma once

#include <stdint.h>

const uint8_t  WIFI_AP_CHANNEL       = 11;   // 2.4 GHz AP channel — off the crowded 1/6 (issue #54)
