// ports — the hardware-watchdog contract (#187, #69). A link-time seam
// (#130): ONE linked implementation
// (infrastructure/arduino/WatchdogAdapter.cpp). The refresh MUST be called
// exactly once per loop pass, after the control path — never add a second
// refresh point (real-time rules).
#pragma once

#include <stdint.h>

// Arm the watchdog; true on success. The MCU resets if refresh isn't
// called within the timeout — PWM stops and the ESCs fail safe (#69).
bool watchdogBegin(uint32_t timeoutMs);

// The actual armed timeout (the hardware may round the requested value).
uint32_t watchdogTimeoutMs();

void watchdogRefresh();
