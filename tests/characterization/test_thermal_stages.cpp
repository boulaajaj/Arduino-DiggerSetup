// Characterization: motor/ESC over-temperature protection (#111, V7.34).
// Non-latching staged hysteresis off the hottest of 4 sensors, 1 s trip
// debounce, telemetry-dropout hold, cool-down "restored" flourish.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("tempStageUpdate: 1 s trip debounce, immediate hysteresis release") {
  resetHarness();
  bool stage = false;
  uint32_t since = 0;

  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  CHECK_FALSE(stage);                            // debounce started, not tripped

  stub::advanceMillis(999);
  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  CHECK_FALSE(stage);

  stub::advanceMillis(1);
  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  CHECK(stage);                                  // 1 s sustained → ON

  tempStageUpdate(&stage, &since, 77.9f, 80.0f, 78.0f, millis());
  CHECK_FALSE(stage);                            // below release → OFF immediately

  // A spike that dips before 1 s never trips the stage.
  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  stub::advanceMillis(500);
  tempStageUpdate(&stage, &since, 70.0f, 80.0f, 78.0f, millis());
  stub::advanceMillis(1);
  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  stub::advanceMillis(999);
  tempStageUpdate(&stage, &since, 85.0f, 80.0f, 78.0f, millis());
  CHECK_FALSE(stage);
}

TEST_CASE("hottestMotorTemp: hottest plausible sensor across both ESCs") {
  resetHarness();
  float hot = 0.0f;

  setAllTemperatures(50.0f);
  telem[1].motorTempC = 72.0f;
  REQUIRE(hottestMotorTemp(&hot));
  CHECK(hot == doctest::Approx(72.0f));

  telem[1].motorTempC = 250.0f;                  // implausible → skipped
  REQUIRE(hottestMotorTemp(&hot));
  CHECK(hot == doctest::Approx(50.0f));

  telem[0].valid = false;                        // whole ESC stale → skipped
  telem[1].escTempC = 250.0f;
  telem[1].motorTempC = -30.0f;                  // also implausible
  CHECK_FALSE(hottestMotorTemp(&hot));
}

TEST_CASE("staged escalation: warn 80, Eco 90, cut 95 — each with its own debounce") {
  resetHarness();
  setAllTemperatures(85.0f);
  thermalUpdate();
  stub::advanceMillis(1000);
  thermalUpdate();
  CHECK(tempWarnActive);
  CHECK_FALSE(tempEcoActive);
  CHECK_FALSE(tempCutActive);

  setAllTemperatures(92.0f);
  thermalUpdate();                               // Eco debounce starts now
  stub::advanceMillis(1000);
  thermalUpdate();
  CHECK(tempEcoActive);
  CHECK_FALSE(tempCutActive);

  setAllTemperatures(96.0f);
  thermalUpdate();
  stub::advanceMillis(1000);
  thermalUpdate();
  CHECK(tempCutActive);
}

TEST_CASE("hysteresis releases: cut needs < 75, Eco < 80, warn < 78") {
  resetHarness();
  setAllTemperatures(96.0f);
  thermalUpdate();
  stub::advanceMillis(1000);
  thermalUpdate();
  REQUIRE((tempWarnActive && tempEcoActive && tempCutActive));

  setAllTemperatures(76.0f);                     // cooling, but not enough for cut
  thermalUpdate();
  CHECK_FALSE(tempWarnActive);                   // 76 < 78
  CHECK_FALSE(tempEcoActive);                    // 76 < 80
  CHECK(tempCutActive);                          // 76 >= 75 → still cut

  setAllTemperatures(74.9f);
  thermalUpdate();
  CHECK_FALSE(tempCutActive);                    // cooled → drive auto-restores
  CHECK(beepSeq == BEEP_THERM_RESTORED);         // one-shot flourish queued
}

TEST_CASE("telemetry dropout HOLDS the last thermal state — never cuts, never clears") {
  resetHarness();
  setAllTemperatures(85.0f);
  thermalUpdate();
  stub::advanceMillis(1000);
  thermalUpdate();
  REQUIRE(tempWarnActive);

  telem[0].valid = false;
  telem[1].valid = false;
  stub::advanceMillis(60000);
  thermalUpdate();
  CHECK(tempWarnActive);                         // held, not cleared
  CHECK_FALSE(tempCutActive);                    // and never escalated
}
