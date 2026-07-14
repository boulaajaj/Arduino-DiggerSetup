// Pure-domain suite (#117 step 5, #162): src/domain/operator_input tested
// directly on the host — no firmware, no stubs, weights/centers/widths
// passed explicitly. End-to-end behavior through rcDeadband()/joyDeadband()
// and the joystick expo path stays covered by
// tests/characterization/test_input_shaping.cpp; this suite locks the
// extracted math at the unit level, including the exact [CONFIG] weight
// pairs and both deadband parameterizations.
// (Basename differs from characterization/test_input_shaping.cpp — the flat
// build/ directory forbids duplicate test basenames.)
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/domain/operator_input/DeadbandPolicy.h"
#include "../../sketches/dual_track_control/src/domain/operator_input/ExpoCurve.h"

// Values mirror the [CONFIG] pairs — parameters here, no config dependency.
const float EXPO_THROTTLE_LINEAR = 0.4f;
const float EXPO_THROTTLE_CUBIC  = 0.6f;
const float EXPO_STEER_LINEAR    = 0.7f;
const float EXPO_STEER_CUBIC     = 0.3f;

TEST_CASE("expo curve endpoints: zero stays zero, full stick reaches the weight sum") {
  CHECK(expoCurve(0.0f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC)
        == doctest::Approx(0.0f));
  // Both config pairs sum to 1.0, so full deflection keeps full authority.
  CHECK(expoCurve(1.0f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC)
        == doctest::Approx(1.0f));
  CHECK(expoCurve(1.0f, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC)
        == doctest::Approx(1.0f));
}

TEST_CASE("expo curve mid-stick softening matches the exact weight math") {
  // 0.4·0.5 + 0.6·0.125 = 0.275 — throttle feels softer around center.
  CHECK(expoCurve(0.5f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC)
        == doctest::Approx(0.275f));
  // 0.7·0.5 + 0.3·0.125 = 0.3875 — steering stays more direct.
  CHECK(expoCurve(0.5f, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC)
        == doctest::Approx(0.3875f));
}

TEST_CASE("expo curve is magnitude-only: the caller owns the sign") {
  CHECK(expoCurve(-0.5f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC)
        == doctest::Approx(expoCurve(0.5f, EXPO_THROTTLE_LINEAR,
                                     EXPO_THROTTLE_CUBIC)));
  CHECK(expoCurve(-1.0f, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC)
        == doctest::Approx(1.0f));  // never negative
}

TEST_CASE("deadband snaps to center inside AND exactly at the band edge") {
  // RC parameterization: servo µs around 1500 ± 50.
  CHECK(centerSnapDeadband(1500, 1500, 50) == 1500);
  CHECK(centerSnapDeadband(1550, 1500, 50) == 1500);  // edge inclusive
  CHECK(centerSnapDeadband(1450, 1500, 50) == 1500);  // edge inclusive, low side
  CHECK(centerSnapDeadband(1551, 1500, 50) == 1551);  // first count outside
  CHECK(centerSnapDeadband(1449, 1500, 50) == 1449);
}

TEST_CASE("deadband holds for joystick-scale parameters too") {
  // Joystick parameterization: 14-bit ADC counts around center ± width.
  CHECK(centerSnapDeadband(8192 + 480, 8192, 480) == 8192);
  CHECK(centerSnapDeadband(8192 - 480, 8192, 480) == 8192);
  CHECK(centerSnapDeadband(8192 + 481, 8192, 480) == 8192 + 481);
  CHECK(centerSnapDeadband(0, 8192, 480) == 0);        // full deflection passes
}
