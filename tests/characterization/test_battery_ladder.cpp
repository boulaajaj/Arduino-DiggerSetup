// Characterization: staged low-battery protection (#65, V7.34 baseline).
// Worst-of-two plausibility gate, Eco lock 10.8 V / 15 s, hard cutoff
// 10.0 V / 1.5 s (latched, alarm asserted with the cut), boot-gate confirm.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("worstPackVoltage: both packs must be valid AND plausible") {
  resetHarness();
  float worst = 0.0f;

  setPackVoltages(11.2f, 10.9f);
  REQUIRE(worstPackVoltage(&worst));
  CHECK(worst == doctest::Approx(10.9f));

  telem[1].valid = false;                        // one stale pack → no reading
  CHECK_FALSE(worstPackVoltage(&worst));

  setPackVoltages(11.2f, 0.4f);                  // not-yet-powered pack reads ~0 V
  CHECK_FALSE(worstPackVoltage(&worst));

  setPackVoltages(11.2f, 14.2f);                 // implausibly high read
  CHECK_FALSE(worstPackVoltage(&worst));
}

TEST_CASE("Eco lock: latches after 15 s sustained below 10.8 V") {
  resetHarness();
  setPackVoltages(10.7f, 11.5f);

  batteryEcoLockUpdate();                        // debounce starts
  CHECK_FALSE(ecoLockLatched);

  stub::advanceMillis(14000);
  batteryEcoLockUpdate();
  CHECK_FALSE(ecoLockLatched);                   // 14 s — not yet

  stub::advanceMillis(1001);
  batteryEcoLockUpdate();
  CHECK(ecoLockLatched);                         // 15 s sustained → Eco lock

  setPackVoltages(12.4f, 12.5f);                 // recovery does NOT unlatch
  batteryEcoLockUpdate();
  CHECK(ecoLockLatched);
}

TEST_CASE("Eco lock debounce resets when voltage recovers before 15 s") {
  resetHarness();
  setPackVoltages(10.7f, 11.5f);
  batteryEcoLockUpdate();
  stub::advanceMillis(10000);

  setPackVoltages(11.0f, 11.5f);                 // sag ended at 10 s
  batteryEcoLockUpdate();

  setPackVoltages(10.7f, 11.5f);                 // dips again — fresh 15 s needed
  batteryEcoLockUpdate();
  stub::advanceMillis(14000);
  batteryEcoLockUpdate();
  CHECK_FALSE(ecoLockLatched);
}

TEST_CASE("hard cutoff: latches after 1.5 s below 10.0 V and asserts the alarm") {
  resetHarness();
  setPackVoltages(9.9f, 11.0f);

  batteryCutoffUpdate();                         // debounce starts
  CHECK_FALSE(batteryCutoffLatched);
  CHECK_FALSE(batteryOkConfirmed);               // 9.9 V never confirms the pack

  stub::advanceMillis(1499);
  batteryCutoffUpdate();
  CHECK_FALSE(batteryCutoffLatched);             // load sag window — not yet

  stub::advanceMillis(2);
  batteryCutoffUpdate();
  CHECK(batteryCutoffLatched);                   // sustained → cut
  CHECK(lowVoltLatched);                         // never a silent cutoff (#65)

  setPackVoltages(12.6f, 12.6f);                 // latch survives recovery
  batteryCutoffUpdate();
  CHECK(batteryCutoffLatched);
}

TEST_CASE("cutoff debounce resets on a validity dropout mid-sag") {
  resetHarness();
  setPackVoltages(9.9f, 11.0f);
  batteryCutoffUpdate();
  stub::advanceMillis(1000);

  telem[0].valid = false;                        // telemetry blip at 1.0 s
  batteryCutoffUpdate();

  setPackVoltages(9.9f, 11.0f);                  // reading returns
  batteryCutoffUpdate();
  stub::advanceMillis(1499);
  batteryCutoffUpdate();
  CHECK_FALSE(batteryCutoffLatched);             // needs a fresh 1.5 s
}

TEST_CASE("boot gate: a plausible reading at/above 10.0 V confirms the battery") {
  resetHarness();
  REQUIRE_FALSE(batteryOkConfirmed);
  setPackVoltages(10.5f, 11.0f);
  batteryCutoffUpdate();
  CHECK(batteryOkConfirmed);
}
