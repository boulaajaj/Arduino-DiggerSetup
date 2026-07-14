// infrastructure/arduino — active-piezo adapter (#117 step 10 slice 2,
// #174). The ONE implementation of ports/AlertOutputPort.h. Behavior is
// locked end-to-end by tests/characterization/test_alerts.cpp via the stub
// GPIO capture.
#include "../../ports/AlertOutputPort.h"

#include <Arduino.h>

static int alertPin = -1;

void alertOutputInitialize(int pin) {
  alertPin = pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void alertOutputSet(bool on) {
  digitalWrite(alertPin, on ? HIGH : LOW);
}
