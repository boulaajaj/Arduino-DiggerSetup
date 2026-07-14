// application — pure range clamping and linear mapping (#187), macro- and
// hardware-free so application code compiles without <Arduino.h>. For the
// side-effect-free arguments every call site uses, results are identical
// to the Arduino equivalents (the functions evaluate each argument once
// and are int/float-typed, unlike the constrain() macro):
// - clamp: the constrain() ternary, ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
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
