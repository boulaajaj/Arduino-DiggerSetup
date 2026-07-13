// infrastructure/arduino — the ONE implementation of ports/ClockPort.h
// (#187). Straight passthroughs to the core clock.
#include "../../ports/ClockPort.h"

#include <Arduino.h>

uint32_t clockNowMs() { return millis(); }
uint32_t clockNowUs() { return micros(); }
