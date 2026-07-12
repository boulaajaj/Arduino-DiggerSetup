// Pure-domain suite (#117 step 6, #164): src/domain/drive/CurvatureDrive
// tested directly on the host — no firmware, no stubs, tunables passed
// explicitly. End-to-end behavior through the firmware's curvatureDrive()
// delegate (including the gear-selected pivot cap) stays covered by
// tests/characterization/test_curvature_drive.cpp and test_mixer.cpp; this
// suite locks the algorithm's laws at the unit level: straight-line
// identity, pivot caps as parameters, the pivot throttle taper, reverse
// steering consistency (#86), faded-ceiling desaturation with ratio
// preservation (#96), and the smoothstep blend endpoints (#72).
// (Basename differs from characterization/test_curvature_drive.cpp — the
// flat build/ directory forbids duplicate test basenames.)
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>

#include "../../sketches/rc_test/src/domain/drive/CurvatureDrive.h"

// Values mirror [CONFIG] — parameters here, no config dependency.
const CurvatureParameters NORMAL_PARAMETERS{0.60f, 0.70f, 0.05f, 0.55f, 0.70f};
const CurvatureParameters ECO_PARAMETERS{0.725f, 0.70f, 0.05f, 0.55f, 0.70f};
const float GEAR_ECO   = 0.65f;
const float GEAR_BOOST = 1.00f;

TEST_CASE("straight line is the identity: both tracks exactly xSpeed*gearScale") {
  // With zRotation = 0 the pivot and curvature branches agree, the taper is
  // inert, and the ceiling stays at the full rail — at EVERY throttle.
  for (float xSpeed : {0.03f, 0.30f, 1.00f, -0.40f}) {
    TrackCommand command =
        curvatureDriveStep(xSpeed, 0.0f, GEAR_ECO, NORMAL_PARAMETERS);
    CHECK(command.left == doctest::Approx(xSpeed * GEAR_ECO));
    CHECK(command.right == doctest::Approx(xSpeed * GEAR_ECO));
  }
}

TEST_CASE("pure pivot at zero throttle counter-rotates at the gear-scaled cap") {
  TrackCommand command =
      curvatureDriveStep(0.0f, 1.0f, GEAR_BOOST, NORMAL_PARAMETERS);
  CHECK(command.left == doctest::Approx(-0.60f));   // cap, full left steer
  CHECK(command.right == doctest::Approx(0.60f));
  command = curvatureDriveStep(0.0f, -1.0f, GEAR_ECO, NORMAL_PARAMETERS);
  CHECK(command.left == doctest::Approx(0.60f * GEAR_ECO));
  CHECK(command.right == doctest::Approx(-0.60f * GEAR_ECO));
}

TEST_CASE("the pivot cap is a parameter: Eco's looser cap pivots harder") {
  TrackCommand normal =
      curvatureDriveStep(0.0f, 1.0f, GEAR_ECO, NORMAL_PARAMETERS);
  TrackCommand eco = curvatureDriveStep(0.0f, 1.0f, GEAR_ECO, ECO_PARAMETERS);
  CHECK(eco.right == doctest::Approx(0.725f * GEAR_ECO));
  CHECK(eco.right > normal.right);
}

TEST_CASE("pivot throttle taper: steering bleeds throttle out of the pivot branch") {
  // Inside the pure-pivot band (|x| <= blend start): throttle term is
  // x*(1 - taper*|z|); at full steer that is x*0.3.
  TrackCommand command =
      curvatureDriveStep(0.05f, 1.0f, GEAR_BOOST, NORMAL_PARAMETERS);
  float pivotThrottle = 0.05f * (1.0f - 0.70f);
  CHECK(command.left == doctest::Approx(pivotThrottle - 0.60f));
  CHECK(command.right == doctest::Approx(pivotThrottle + 0.60f));
}

TEST_CASE("#86 law: steering keeps the same nose direction in reverse") {
  // Pure-curvature region (|x| >= blend end), stick left (z > 0): the left
  // track must be the slower/harder-reverse one BOTH ways.
  TrackCommand forward =
      curvatureDriveStep(0.6f, 0.5f, GEAR_BOOST, NORMAL_PARAMETERS);
  TrackCommand reverse =
      curvatureDriveStep(-0.6f, 0.5f, GEAR_BOOST, NORMAL_PARAMETERS);
  CHECK(forward.left < forward.right);
  CHECK(reverse.left < reverse.right);   // same relation, not flipped
}

TEST_CASE("#96 law: desaturation against the faded ceiling preserves the turn ratio") {
  // x=1, z=0.5, Boost: curv pair (0.5, 1.5); ceiling 1-(0.3)(0.5)=0.85.
  TrackCommand command =
      curvatureDriveStep(1.0f, 0.5f, GEAR_BOOST, NORMAL_PARAMETERS);
  CHECK(command.right == doctest::Approx(0.85f));
  CHECK(command.right / command.left == doctest::Approx(3.0f));  // 1.5/0.5 kept
}

TEST_CASE("#72 law: smoothstep endpoints — pure pivot below start, pure curvature past end") {
  // At |x| = blend start the blend factor is exactly 0 (pivot only).
  TrackCommand atStart =
      curvatureDriveStep(0.05f, 0.2f, GEAR_BOOST, NORMAL_PARAMETERS);
  float pivotThrottle = 0.05f * (1.0f - 0.70f * 0.2f);
  CHECK(atStart.left == doctest::Approx(pivotThrottle - 0.2f));
  // At |x| >= blend end the factor is exactly 1 (curvature only).
  TrackCommand pastEnd =
      curvatureDriveStep(0.55f, 0.2f, GEAR_BOOST, NORMAL_PARAMETERS);
  float average = 0.55f;
  float delta = average * 0.2f;
  CHECK(pastEnd.left == doctest::Approx(average - delta));
  CHECK(pastEnd.right == doctest::Approx(average + delta));
}
