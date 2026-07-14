// config — the loop watchdog bound (#185, #69).
#pragma once

#include <stdint.h>

// Safety watchdog (#69). The MCU resets if loop() fails to service the control
// path (read inputs + write outputs) within WDT_TIMEOUT_MS — a reset stops PWM,
// so the ESCs go to neutral/failsafe instead of holding the last throttle. This
// bounds ANY loop stall (Wi-Fi serving or otherwise) to at most this long.
// Starting value; tune on the bench. Must stay above the worst-case single loop
// pass (with incremental page serving, a pass is far under this).
const uint32_t WDT_TIMEOUT_MS        = 250;
