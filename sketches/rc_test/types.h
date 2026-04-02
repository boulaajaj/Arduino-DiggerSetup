// types.h — Shared data structures for Digger Control V3.4
#pragma once
#include <Arduino.h>

// Per-channel RC state with independent failsafe
struct RCChannel {
  int pw;                // Last valid pulse width (us)
  unsigned long lastOk;  // Timestamp of last valid reading (micros)
  bool valid;            // Signal received within failsafe window
};

// Non-blocking pulse reader for pins without interrupt support.
// Call poll() every loop iteration — it tracks edges via digitalRead
// and measures pulse width without blocking.
struct PulseReader {
  uint8_t pin;
  bool lastState;
  unsigned long riseTime;
  int pw;
  unsigned long lastOk;
  bool valid;

  void init(uint8_t p) {
    pin = p;
    pinMode(pin, INPUT);
    lastState = digitalRead(pin);
    riseTime = 0;
    pw = 1500;
    lastOk = 0;
    valid = false;
  }

  void poll() {
    bool state = digitalRead(pin);
    if (state != lastState) {
      unsigned long now = micros();
      if (state) {
        riseTime = now;  // Rising edge
      } else if (riseTime > 0) {
        unsigned long width = now - riseTime;
        if (width >= 800 && width <= 2200) {
          pw = width;
          lastOk = now;
        }
      }
      lastState = state;
    }
  }
};

// Joystick ADC reading after deadband + expo + tank mix
struct JoystickState {
  int rawY, rawX;    // Raw 14-bit ADC values
  int left, right;   // Tank-mixed servo commands (us)
};

// Mixer output after override switch selection
struct MixerOutput {
  int left, right;   // Servo commands (us) before dynamics
};
