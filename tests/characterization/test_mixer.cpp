// Characterization: RC + joystick mixing (V7.34 baseline, #90 max/oppose).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("maxOppose: stronger same-direction input wins, opposing subtract") {
  resetHarness();
  CHECK(maxOppose(0.6f, 0.3f) == doctest::Approx(0.6f));    // larger wins, no halving
  CHECK(maxOppose(1.0f, -0.3f) == doctest::Approx(0.7f));   // dominant minus opposing
  CHECK(maxOppose(0.5f, 0.0f) == doctest::Approx(0.5f));    // single input at full value
  CHECK(maxOppose(0.0f, -0.4f) == doctest::Approx(-0.4f));
  CHECK(maxOppose(-0.6f, -0.2f) == doctest::Approx(-0.6f));  // both reverse: larger wins
  CHECK(maxOppose(0.5f, -0.5f) == doctest::Approx(0.0f));   // exact opposition cancels
}

TEST_CASE("Mode 1 (override < 1400): RC only, joystick ignored") {
  resetHarness();
  DriveCommand rc{0.3f, -0.2f};
  DriveCommand joy{1.0f, 1.0f};
  DriveCommand out = mixCommands(rc, 1000, joy);
  CHECK(out.xSpeed == doctest::Approx(0.3f));
  CHECK(out.zRotation == doctest::Approx(-0.2f));
}

TEST_CASE("Mode 2 (override 1400-1600): RC overrides joystick only while RC is active") {
  resetHarness();
  DriveCommand joy{0.5f, 0.1f};

  DriveCommand rcActive{0.2f, 0.0f};
  DriveCommand out = mixCommands(rcActive, 1500, joy);
  CHECK(out.xSpeed == doctest::Approx(0.2f));    // RC wins while touched
  CHECK(out.zRotation == doctest::Approx(0.0f));

  DriveCommand rcIdle{0.0f, 0.0f};
  out = mixCommands(rcIdle, 1500, joy);
  CHECK(out.xSpeed == doctest::Approx(0.5f));    // rider drives when RC idle
  CHECK(out.zRotation == doctest::Approx(0.1f));
}

TEST_CASE("Mode 3 (override > 1600): max/oppose blend per axis") {
  resetHarness();
  DriveCommand rc{0.6f, -0.5f};
  DriveCommand joy{0.3f, 0.5f};
  DriveCommand out = mixCommands(rc, 2000, joy);
  CHECK(out.xSpeed == doctest::Approx(0.6f));    // same direction: larger wins
  CHECK(out.zRotation == doctest::Approx(0.0f));  // opposition cancels
}

TEST_CASE("Mode boundaries: exactly 1400 and 1600 fall into Mode 2") {
  resetHarness();
  DriveCommand rc{0.0f, 0.0f};
  DriveCommand joy{0.9f, 0.0f};
  CHECK(mixCommands(rc, 1400, joy).xSpeed == doctest::Approx(0.9f));  // not Mode 1
  CHECK(mixCommands(rc, 1600, joy).xSpeed == doctest::Approx(0.9f));  // not Mode 3
}
