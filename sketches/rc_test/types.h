// types.h — Shared data structures for Digger Control V4.0
#pragma once
#include <Arduino.h>

// Joystick ADC reading after deadband + expo + tank mix
struct JoystickState {
  int rawY, rawX;    // Raw 14-bit ADC values
  int left, right;   // Tank-mixed servo commands (us)
};

// Mixer output after override switch selection
struct MixerOutput {
  int left, right;   // Servo commands (us) before dynamics
};

// RC tank-mixed output (throttle + steering → left/right)
struct RCMixed {
  int left, right;
};
