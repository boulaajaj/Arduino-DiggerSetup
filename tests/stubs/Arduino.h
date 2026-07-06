// Arduino.h — HOST TEST STUB (tests/stubs). Shadows the real core via include
// order so sketches compile unchanged on a laptop. Fake clock, scripted ADC,
// captured GPIO. Test control surface lives in namespace stub.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ── Core constants ──────────────────────────────────────────────────────────
#define HIGH 1
#define LOW  0
#define INPUT  0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define PROGMEM
#define F(str) (str)

// UNO R4 analog pin ids (values arbitrary but stable; ADC is keyed by them).
const uint8_t A0 = 14;
const uint8_t A1 = 15;
const uint8_t A2 = 16;
const uint8_t A3 = 17;
const uint8_t A4 = 18;
const uint8_t A5 = 19;

// ── Test control surface: clock / ADC / GPIO ────────────────────────────────
namespace stub {

const int STUB_PIN_COUNT = 32;
const int ADC_DEFAULT_CENTER = 8192;  // 14-bit midpoint — matches firmware ADC_CENTER

// Fake monotonic clock, microsecond resolution. Tests advance it explicitly;
// delay()/delayMicroseconds() advance it too (mirrors real elapsed time).
// Reset starts at 1 s, not 0: on real hardware millis()==0 exists only for the
// first millisecond after power-on, and several firmware timers use 0 as a
// "not running" sentinel — starting at 0 would exercise a state no real boot
// sustains.
inline uint64_t clockMicros = 1000000ULL;

inline int  analogValues[STUB_PIN_COUNT] = {};
inline int  analogReadBits = 10;
inline int  pinModes[STUB_PIN_COUNT] = {};
inline int  digitalLevels[STUB_PIN_COUNT] = {};

inline void advanceMicros(uint64_t us) { clockMicros += us; }
inline void advanceMillis(uint64_t ms) { clockMicros += ms * 1000ULL; }

inline void setAnalogValue(uint8_t pin, int value) { analogValues[pin] = value; }

inline void resetArduinoCore() {
  clockMicros = 1000000ULL;
  analogReadBits = 10;
  for (int i = 0; i < STUB_PIN_COUNT; i++) {
    analogValues[i] = ADC_DEFAULT_CENTER;
    pinModes[i] = INPUT;
    digitalLevels[i] = LOW;
  }
}

}  // namespace stub

// ── Arduino API ──────────────────────────────────────────────────────────────
inline uint32_t millis() { return (uint32_t)(stub::clockMicros / 1000ULL); }
inline uint32_t micros() { return (uint32_t)stub::clockMicros; }
inline void delay(uint32_t ms) { stub::advanceMillis(ms); }
inline void delayMicroseconds(uint32_t us) { stub::advanceMicros(us); }

inline void analogReadResolution(int bits) { stub::analogReadBits = bits; }
inline int  analogRead(uint8_t pin) { return stub::analogValues[pin]; }
inline void pinMode(uint8_t pin, int mode) { stub::pinModes[pin] = mode; }
inline void digitalWrite(uint8_t pin, int level) { stub::digitalLevels[pin] = level; }
inline int  digitalRead(uint8_t pin) { return stub::digitalLevels[pin]; }

// Exact Arduino implementation (integer math, truncation toward zero).
inline long map(long x, long inMin, long inMax, long outMin, long outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Same semantics as the Arduino constrain() macro.
template <typename T>
inline T constrain(T value, T low, T high) {
  return (value < low) ? low : (value > high) ? high : value;
}

// Minimal IPAddress so WiFi.localIP() round-trips through Serial prints.
class IPAddress {
 public:
  IPAddress() : IPAddress(0, 0, 0, 0) {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    snprintf(text_, sizeof(text_), "%u.%u.%u.%u", a, b, c, d);
  }
  const char* toString() const { return text_; }

 private:
  char text_[16];
};

#include "SerialPortStub.h"
