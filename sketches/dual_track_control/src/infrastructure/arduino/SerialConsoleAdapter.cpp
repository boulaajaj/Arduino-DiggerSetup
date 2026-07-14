// infrastructure/arduino — the ONE implementation of
// ports/DebugConsolePort.h (#187): the USB CDC debug console.
#include "../../ports/DebugConsolePort.h"

#include <Arduino.h>

void debugConsoleBegin(uint32_t baud) {
  Serial.begin(baud);
  delay(50);  // boot settle, moved verbatim from debugInit() (boot-time only)
}

bool debugConsoleReady() { return (bool)Serial; }

void debugConsolePrintLine(const char* line) { Serial.println(line); }
