// domain/operator_input — exponential input curve (#117 step 5, #162).
// Logic copied VERBATIM from dual_track_control.ino [JOYSTICK]; behavior is locked by
// tests/characterization/test_input_shaping.cpp and
// tests/operator_input/test_operator_input_domain.cpp.
#include "ExpoCurve.h"

#include <math.h>

float expoCurve(float x, float linearWeight, float cubicWeight) {
  float a = fabsf(x);
  return linearWeight * a + cubicWeight * a * a * a;
}
