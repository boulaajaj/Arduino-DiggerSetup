// infrastructure/arduino — PWM ESC adapter (#117 step 10 slice 1, #172).
// The ONE implementation of ports/EscOutputPort.h. Owns the Servo objects
// (moved from rc_test.ino) — the only ESC-output code that includes a
// hardware library. Behavior is locked end-to-end by
// tests/characterization/test_output_gate.cpp via the stub Servo captures.
#include "../../ports/EscOutputPort.h"

#include <Arduino.h>
#include <Servo.h>

// External linkage: the test harness (FirmwareUnderTest.h) resets these
// between cases via an extern declaration.
Servo escL, escR;

static int escLeftPin = -1;
static int escRightPin = -1;

void escOutputInitialize(int leftPin, int rightPin) {
  escLeftPin = leftPin;
  escRightPin = rightPin;
  escL.attach(leftPin);
  escR.attach(rightPin);
}

void escOutputAttach() {
  escL.attach(escLeftPin);
  escR.attach(escRightPin);
}

void escOutputDetach() {
  escL.detach();
  escR.detach();
}

void escOutputWritePulses(int leftPulse, int rightPulse) {
  escL.writeMicroseconds(leftPulse);
  escR.writeMicroseconds(rightPulse);
}
