// infrastructure/arduino — the ONE implementation of ports/WatchdogPort.h
// (#187, #69). Straight passthroughs to the RA4M1 hardware watchdog.
#include "../../ports/WatchdogPort.h"

#include <Arduino.h>
#include <WDT.h>

bool watchdogBegin(uint32_t timeoutMs) { return WDT.begin(timeoutMs); }

uint32_t watchdogTimeoutMs() { return WDT.getTimeout(); }

void watchdogRefresh() { WDT.refresh(); }
