// ports — the clock contract (#187, target §2). A link-time seam (#130):
// ONE linked implementation (infrastructure/arduino/ArduinoClock.cpp).
// The application reads the clock through this port; domain code never
// reads it at all (time is a parameter there).
#pragma once

#include <stdint.h>

uint32_t clockNowMs();  // milliseconds since boot (millis semantics)
uint32_t clockNowUs();  // microseconds since boot (micros semantics)
