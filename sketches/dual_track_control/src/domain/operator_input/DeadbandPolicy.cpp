// domain/operator_input — center-snap deadband (#117 step 5, #162).
// Logic from dual_track_control.ino [RC] rcDeadband() / [JOYSTICK] joyDeadband() —
// identical comparisons; Arduino's abs() macro is expanded manually here
// (equivalent for every reachable servo-µs/ADC value). Behavior is locked
// by tests/characterization/test_input_shaping.cpp and
// tests/operator_input/test_operator_input_domain.cpp.
#include "DeadbandPolicy.h"

int centerSnapDeadband(int value, int center, int width) {
  int offset = value - center;
  if (offset < 0) offset = -offset;
  return (offset <= width) ? center : value;
}
