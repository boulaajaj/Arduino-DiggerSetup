// Characterization: gear selection + output caps (V7.34 baseline).
// CH4 thresholds, S.BUS-invalid Eco failsafe, battery/thermal Eco locks,
// per-gear reverse caps (#113), outCapToX conversion.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("S.BUS invalid → failsafe to Eco") {
  resetHarness();
  gearScale = GEAR_HIGH_SCALE;                   // pretend we were in Boost
  currentGear = GEAR_HIGH;
  updateGear();
  CHECK(currentGear == GEAR_LOW);
  CHECK(gearScale == doctest::Approx(GEAR_LOW_SCALE));
}

TEST_CASE("CH4 three-position switch maps to Eco / Normal / Boost") {
  resetHarness();
  bfs::SbusData frame = stub::centeredSbusFrame();

  frame.ch[SBUS_CH_GEAR] = stub::SBUS_RAW_MIN;   // 1000 us < 1400
  feedSbusFrame(frame);
  updateGear();
  CHECK(currentGear == GEAR_LOW);
  CHECK(gearScale == doctest::Approx(0.65f));

  frame.ch[SBUS_CH_GEAR] = stub::SBUS_RAW_CENTER;  // 1500 us in the middle band
  feedSbusFrame(frame);
  updateGear();
  CHECK(currentGear == GEAR_MID);
  CHECK(gearScale == doctest::Approx(0.80f));

  frame.ch[SBUS_CH_GEAR] = stub::SBUS_RAW_MAX;   // 2000 us > 1600
  feedSbusFrame(frame);
  updateGear();
  CHECK(currentGear == GEAR_HIGH);
  CHECK(gearScale == doctest::Approx(1.00f));

  // Exactly 1400 us (raw 828) is NOT below the low threshold → Normal.
  frame.ch[SBUS_CH_GEAR] = 828;
  feedSbusFrame(frame);
  REQUIRE(sbusToServo(828) == 1400);
  updateGear();
  CHECK(currentGear == GEAR_MID);
}

TEST_CASE("battery Eco lock and thermal Eco lock override the gear switch") {
  resetHarness();
  bfs::SbusData frame = stub::centeredSbusFrame();
  frame.ch[SBUS_CH_GEAR] = stub::SBUS_RAW_MAX;   // operator asks for Boost

  feedSbusFrame(frame);
  ecoLockLatched = true;                         // draining pack (#65)
  updateGear();
  CHECK(currentGear == GEAR_LOW);
  CHECK(gearScale == doctest::Approx(GEAR_LOW_SCALE));

  ecoLockLatched = false;
  tempEcoActive = true;                          // hot motor (#111)
  feedSbusFrame(frame);
  updateGear();
  CHECK(currentGear == GEAR_LOW);

  tempEcoActive = false;                         // thermal lock releases on its own
  feedSbusFrame(frame);
  updateGear();
  CHECK(currentGear == GEAR_HIGH);               // Boost restored
}

TEST_CASE("reverseCap (#113): 55% Eco/Normal, 65% Boost") {
  resetHarness();
  currentGear = GEAR_LOW;
  CHECK(reverseCap() == doctest::Approx(0.55f));
  currentGear = GEAR_MID;
  CHECK(reverseCap() == doctest::Approx(0.55f));
  currentGear = GEAR_HIGH;
  CHECK(reverseCap() == doctest::Approx(0.65f));
}

TEST_CASE("outCapToX converts a track-output cap into the xSpeed domain") {
  resetHarness();
  gearScale = 0.65f;
  CHECK(outCapToX(0.65f) == doctest::Approx(1.0f));   // Eco fwd cap = full stick
  CHECK(outCapToX(0.55f) == doctest::Approx(0.55f / 0.65f));
  gearScale = 1.0f;
  CHECK(outCapToX(0.65f) == doctest::Approx(0.65f));  // Boost reverse cap
}
