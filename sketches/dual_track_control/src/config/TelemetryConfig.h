// config — observer cadences (#185). The X.BUS poll constants live with
// infrastructure/xc/XbusTelemetryAdapter.h (#178) and the Wi-Fi serving
// tunables with infrastructure/network/WifiService.h (#181) — each single-
// homed with its machine.
#pragma once

#include <stdint.h>

// Debug
const uint32_t PRINT_INTERVAL = 100000UL;  // 10 Hz CSV output
