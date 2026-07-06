// Characterization: fail-safe output gate (#88/#65, V7.34 baseline).
// ACTIVE passthrough → HOLD (500 ms ease-out from the live command) → CUT
// (detach, no pulses), automatic re-attach on recovery.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("outputInit attaches both ESCs at neutral") {
  resetHarness();
  outputInit();
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(leftEscMicroseconds() == 1500);
  CHECK(rightEscMicroseconds() == 1500);
}

TEST_CASE("ACTIVE: commands pass through with the 1000-2000 us rail clamp") {
  resetHarness();
  outputInit();
  outputUpdate(true, 1600, 1400);
  CHECK(leftEscMicroseconds() == 1600);
  CHECK(rightEscMicroseconds() == 1400);

  outputUpdate(true, 2500, 500);                 // beyond the rail
  CHECK(leftEscMicroseconds() == 2000);
  CHECK(rightEscMicroseconds() == 1000);
}

TEST_CASE("gate close: 500 ms linear ease-out from the live command, then detach") {
  resetHarness();
  outputInit();
  outputUpdate(true, 1600, 1400);

  outputUpdate(false, 1600, 1400);               // gate closes NOW (t=0)
  CHECK(leftEscMicroseconds() == 1600);          // ease starts from live output
  CHECK(rightEscMicroseconds() == 1400);

  stub::advanceMillis(100);                      // t = 0.2
  outputUpdate(false, 1600, 1400);
  CHECK(leftEscMicroseconds() == 1580);
  CHECK(rightEscMicroseconds() == 1420);

  stub::advanceMillis(150);                      // t = 0.5
  outputUpdate(false, 1600, 1400);
  CHECK(leftEscMicroseconds() == 1550);
  CHECK(rightEscMicroseconds() == 1450);

  stub::advanceMillis(250);                      // t = 1.0 → neutral, then cut
  outputUpdate(false, 1600, 1400);
  CHECK(leftEscMicroseconds() == 1500);
  CHECK(rightEscMicroseconds() == 1500);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);

  // OUT_CUT emits nothing: no further writes, none while detached.
  size_t writesBefore = stub::servoOnPin(PIN_ESC_L).writes.size();
  stub::advanceMillis(100);
  outputUpdate(false, 1600, 1400);
  CHECK(stub::servoOnPin(PIN_ESC_L).writes.size() == writesBefore);
  CHECK(stub::servoOnPin(PIN_ESC_L).writesWhileDetached == 0);
}

TEST_CASE("late gate evaluation past the window clamps t to 1 (no overshoot)") {
  resetHarness();
  outputInit();
  outputUpdate(true, 1900, 1100);
  outputUpdate(false, 1900, 1100);
  stub::advanceMillis(750);                      // t would be 1.5
  outputUpdate(false, 1900, 1100);
  CHECK(leftEscMicroseconds() == 1500);
  CHECK(rightEscMicroseconds() == 1500);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
}

TEST_CASE("recovery: drive allowed again re-attaches and resumes immediately") {
  resetHarness();
  outputInit();
  outputUpdate(false, 1500, 1500);
  stub::advanceMillis(600);
  outputUpdate(false, 1500, 1500);               // fully cut
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_L).attached);

  outputUpdate(true, 1700, 1300);                // RC came back
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 2);
  CHECK(leftEscMicroseconds() == 1700);
  CHECK(rightEscMicroseconds() == 1300);
}
