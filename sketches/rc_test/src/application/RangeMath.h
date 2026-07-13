// application — pure range clamping and linear mapping (#187). These carry
// the EXACT Arduino constrain()/map() semantics, macro- and hardware-free,
// so application code compiles without <Arduino.h>:
// - clamp: ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// - mapRange: ArduinoCore-API map() verbatim — integer long math,
//   (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
#pragma once

inline int clampInt(int value, int low, int high) {
  return value < low ? low : (value > high ? high : value);
}

inline float clampFloat(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

inline long mapRange(long x, long inMin, long inMax, long outMin, long outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
