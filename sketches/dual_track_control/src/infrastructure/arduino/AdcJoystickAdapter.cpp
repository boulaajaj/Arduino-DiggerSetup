// infrastructure/arduino — ADC joystick adapter (#117 step 10 slice 2,
// #174). The ONE implementation of ports/JoystickPort.h. The conditioning
// sequence moved VERBATIM from dual_track_control.ino updateJoystick(): discard-read →
// 100 µs mux settle → real read, per axis, Y then X. That sequence is
// hardware timing (14-bit ADC settling) and runs at the 100 Hz joystick
// cache cadence — not per loop pass — so the non-blocking budget is
// unchanged. Behavior is locked end-to-end by
// tests/characterization/test_input_shaping.cpp via the scripted stub
// analogRead.
#include "../../ports/JoystickPort.h"

#include <Arduino.h>

void joystickInitialize() {
  analogReadResolution(14);  // moved verbatim from setup() (#187) — the
                             // 14-bit range every axis constant assumes
}

void joystickReadAxes(int yPin, int xPin, int* rawY, int* rawX) {
  analogRead(yPin);
  delayMicroseconds(100);
  *rawY = analogRead(yPin);
  analogRead(xPin);
  delayMicroseconds(100);
  *rawX = analogRead(xPin);
}
