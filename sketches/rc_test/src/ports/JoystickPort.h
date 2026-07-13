// ports — the joystick input contract (#117 step 10 slice 2, #174).
// A link-time seam (#130): free functions, ONE linked implementation
// (infrastructure/arduino/AdcJoystickAdapter.cpp). The caller owns the
// sampling cadence, deadband, expo and gain pipeline — this port only
// delivers one conditioned raw ADC pair.
#pragma once

// Read both axes with the ADC conditioning the hall-effect joystick needs
// (discard-read, settle, real read — Y axis first, then X).
void joystickReadAxes(int yPin, int xPin, int* rawY, int* rawX);
