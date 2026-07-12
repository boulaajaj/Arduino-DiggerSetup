// Pure-domain suite (#117 step 3, #158): src/domain/thermal tested directly
// on the host — no firmware, no stubs, time and thresholds passed
// explicitly. End-to-end behavior through the firmware's tempStageUpdate()/
// hottestMotorTemp()/thermalUpdate() shims stays covered by
// tests/characterization/test_thermal_stages.cpp and
// tests/invariants/test_invariant_thermal_monotone.cpp; this suite locks the
// extracted logic at the unit level: trip-debounce boundaries, the
// hysteresis HOLD inside the on/off gap (behavior law), immediate release,
// plausible-hottest selection, dropout-holds-state, and the cut-released
// signal firing exactly once per release edge.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/rc_test/src/domain/thermal/ThermalDerating.h"
#include "../../sketches/rc_test/src/domain/thermal/ThermalHysteresis.h"

// Values mirror TEMP_*_ON_C / TEMP_*_OFF_C / TEMP_DEBOUNCE_MS /
// TEMP_PLAUS_MIN_C / TEMP_PLAUS_MAX_C — but they are parameters here: the
// domain functions have no config dependency.
const float    WARN_ON_C  = 80.0f;
const float    WARN_OFF_C = 78.0f;
const float    ECO_ON_C   = 90.0f;
const float    ECO_OFF_C  = 80.0f;
const float    CUT_ON_C   = 95.0f;
const float    CUT_OFF_C  = 75.0f;
const uint32_t DEBOUNCE_MS = 1000UL;
const float    PLAUSIBLE_MIN_C = -20.0f;
const float    PLAUSIBLE_MAX_C = 200.0f;

const ThermalDeratingThresholds THRESHOLDS{
    WARN_ON_C, WARN_OFF_C, ECO_ON_C, ECO_OFF_C, CUT_ON_C, CUT_OFF_C,
    DEBOUNCE_MS};

TEST_CASE("stage trips only after the full debounce at/above ON") {
  ThermalStageState stage{false, 0};
  thermalStageStep(stage, ECO_ON_C, 1000, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == false);              // timer starts, no trip yet
  CHECK(stage.sinceMs == 1000);
  thermalStageStep(stage, 95.0f, 1000 + DEBOUNCE_MS - 1,
                   ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == false);              // one ms short
  thermalStageStep(stage, 95.0f, 1000 + DEBOUNCE_MS,
                   ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == true);               // exact boundary trips
}

TEST_CASE("dropping below ON before the debounce elapses resets the timer") {
  ThermalStageState stage{false, 0};
  thermalStageStep(stage, 92.0f, 1000, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  thermalStageStep(stage, 85.0f, 1500, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.sinceMs == 0);                 // dropped back → reset
  thermalStageStep(stage, 92.0f, 2000, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  thermalStageStep(stage, 92.0f, 2000 + DEBOUNCE_MS - 1,
                   ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == false);              // fresh full window required
}

TEST_CASE("behavior law: an active stage HOLDS inside the on/off gap") {
  ThermalStageState stage{true, 0};
  // Eco band is 90 ON / 80 OFF: 85 C sits inside the gap and must HOLD.
  thermalStageStep(stage, 85.0f, 5000, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == true);
  // Exactly the OFF threshold is NOT below it — still holds.
  thermalStageStep(stage, ECO_OFF_C, 6000, ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == true);
  // Below OFF releases immediately, no debounce.
  thermalStageStep(stage, ECO_OFF_C - 0.5f, 6001,
                   ECO_ON_C, ECO_OFF_C, DEBOUNCE_MS);
  CHECK(stage.active == false);
  CHECK(stage.sinceMs == 0);
}

TEST_CASE("hottest selection skips stale and implausible readings") {
  const ThermalReading readings[4] = {
      {150.0f, false},   // hottest but stale → skipped
      {250.0f, true},    // above plausibility band → skipped
      {-40.0f, true},    // below plausibility band (unplugged) → skipped
      {72.0f, true},     // the only usable reading
  };
  float hottest = -1.0f;
  CHECK(hottestPlausibleTemperature(readings, 4, PLAUSIBLE_MIN_C,
                                    PLAUSIBLE_MAX_C, &hottest) == true);
  CHECK(hottest == doctest::Approx(72.0f));
}

TEST_CASE("hottest selection: band edges inclusive; none usable leaves out-param") {
  const ThermalReading edges[2] = {{PLAUSIBLE_MIN_C, true},
                                   {PLAUSIBLE_MAX_C, true}};
  float hottest = -1.0f;
  CHECK(hottestPlausibleTemperature(edges, 2, PLAUSIBLE_MIN_C,
                                    PLAUSIBLE_MAX_C, &hottest) == true);
  CHECK(hottest == doctest::Approx(PLAUSIBLE_MAX_C));
  const ThermalReading unusable[2] = {{90.0f, false}, {999.0f, true}};
  hottest = -1.0f;
  CHECK(hottestPlausibleTemperature(unusable, 2, PLAUSIBLE_MIN_C,
                                    PLAUSIBLE_MAX_C, &hottest) == false);
  CHECK(hottest == -1.0f);                   // rejected calls never write it
}

TEST_CASE("hottest selection stays correct for bounds below the old sentinel") {
  // The bounds are parameters: with a band reaching below -1000 the result
  // must still be the actual hottest reading, not a sentinel artifact.
  const ThermalReading deepCold[1] = {{-1500.0f, true}};
  float hottest = 0.0f;
  CHECK(hottestPlausibleTemperature(deepCold, 1, -2000.0f, 200.0f,
                                    &hottest) == true);
  CHECK(hottest == doctest::Approx(-1500.0f));
}

TEST_CASE("telemetry dropout HOLDS every stage — never cuts, never releases") {
  ThermalDeratingState derating{{true, 0}, {true, 0}, {true, 0}};
  CHECK(thermalDeratingStep(derating, false, 0.0f, 7000, THRESHOLDS) == false);
  CHECK(derating.warn.active == true);       // all stages held
  CHECK(derating.eco.active == true);
  CHECK(derating.cut.active == true);
  ThermalDeratingState cold{{false, 500}, {false, 500}, {false, 500}};
  CHECK(thermalDeratingStep(cold, false, 150.0f, 7000, THRESHOLDS) == false);
  CHECK(cold.warn.sinceMs == 500);           // timers held too, not reset
}

TEST_CASE("cut-released fires exactly once per release edge; stages stay independent") {
  ThermalDeratingState derating{{false, 0}, {false, 0}, {false, 0}};
  // Sustained 96 C trips all three stages after the shared debounce.
  CHECK(thermalDeratingStep(derating, true, 96.0f, 1000, THRESHOLDS) == false);
  CHECK(thermalDeratingStep(derating, true, 96.0f, 1000 + DEBOUNCE_MS,
                            THRESHOLDS) == false);  // trip is not a release
  CHECK(derating.warn.active == true);
  CHECK(derating.eco.active == true);
  CHECK(derating.cut.active == true);
  // 85 C sits inside every stage's on/off gap → everything holds, no edge.
  CHECK(thermalDeratingStep(derating, true, 85.0f, 3000, THRESHOLDS) == false);
  CHECK(derating.cut.active == true);        // 85 C inside the 95/75 gap holds
  CHECK(derating.warn.active == true);       // 85 C >= 78 holds warn too
  // 74 C is below every OFF threshold → all release; the cut edge fires ONCE.
  CHECK(thermalDeratingStep(derating, true, 74.0f, 4000, THRESHOLDS) == true);
  CHECK(derating.cut.active == false);
  CHECK(thermalDeratingStep(derating, true, 74.0f, 5000, THRESHOLDS) == false);
}

TEST_CASE("86 C sustained trips warn only — eco and cut never engage") {
  ThermalDeratingState derating{{false, 0}, {false, 0}, {false, 0}};
  thermalDeratingStep(derating, true, 86.0f, 1000, THRESHOLDS);
  thermalDeratingStep(derating, true, 86.0f, 1000 + DEBOUNCE_MS, THRESHOLDS);
  CHECK(derating.warn.active == true);
  CHECK(derating.eco.active == false);
  CHECK(derating.cut.active == false);
  CHECK(derating.eco.sinceMs == 0);          // never started debouncing
}
