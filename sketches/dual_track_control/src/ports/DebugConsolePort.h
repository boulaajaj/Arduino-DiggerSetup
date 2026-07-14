// ports — the USB debug console contract (#187). A link-time seam (#130):
// ONE linked implementation (infrastructure/arduino/SerialConsoleAdapter.cpp).
// Callers assemble each line (snprintf) and print whole lines — matching
// the one-write-per-line style the debug stream already used.
#pragma once

#include <stdint.h>

// Open the console (includes the short boot settle the sketch always did).
void debugConsoleBegin(uint32_t baud);

// True when a host is attached and prints will be seen.
bool debugConsoleReady();

// Print one line (appends the newline exactly like println).
void debugConsolePrintLine(const char* line);
