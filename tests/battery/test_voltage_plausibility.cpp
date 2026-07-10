// Pure-domain suite (#117 step 1): src/domain/battery/VoltagePlausibility
// tested directly on the host — no firmware, no stubs, bounds passed
// explicitly. End-to-end behavior through the firmware's worstPackVoltage()
// delegate stays covered by tests/invariants/test_invariant_plausibility.cpp
// and tests/characterization/test_battery_ladder.cpp; this suite proves the
// extracted function is host-compilable and locks its contract at the unit
// level, including the #131 FINDING (NaN passes the band).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>

#include "../../sketches/rc_test/src/domain/battery/VoltagePlausibility.h"

// Bounds mirror LOWV_PLAUS_MIN_V / LOWV_PLAUS_MAX_V — but they are
// parameters here: the domain function has no config dependency.
const float PLAUSIBLE_MIN_V = 6.0f;
const float PLAUSIBLE_MAX_V = 13.0f;

inline bool worstOf(float voltage0, bool valid0, float voltage1, bool valid1,
                    float* worstOut) {
  return worstPlausiblePackVoltage(BatteryReading{voltage0, valid0},
                                   BatteryReading{voltage1, valid1},
                                   PLAUSIBLE_MIN_V, PLAUSIBLE_MAX_V, worstOut);
}

TEST_CASE("both packs must be valid — one stale reading rejects the pair") {
  float worst = -1.0f;
  CHECK(worstOf(12.6f, true, 12.6f, false, &worst) == false);
  CHECK(worstOf(12.6f, false, 12.6f, true, &worst) == false);
  CHECK(worstOf(12.6f, false, 12.6f, false, &worst) == false);
  CHECK(worst == -1.0f);  // rejected calls never write the out-param
}

TEST_CASE("plausibility band rejects out-of-band packs on either slot") {
  float worst = -1.0f;
  // Unpowered pack (~0 V) below the band.
  CHECK(worstOf(0.4f, true, 12.6f, true, &worst) == false);
  CHECK(worstOf(12.6f, true, 0.4f, true, &worst) == false);
  // Impossibly high read above the band.
  CHECK(worstOf(14.2f, true, 12.6f, true, &worst) == false);
  CHECK(worstOf(12.6f, true, 14.2f, true, &worst) == false);
  CHECK(worst == -1.0f);
}

TEST_CASE("band boundaries are inclusive; worst = min of the two packs") {
  float worst = -1.0f;
  CHECK(worstOf(PLAUSIBLE_MIN_V, true, PLAUSIBLE_MAX_V, true, &worst) == true);
  CHECK(worst == doctest::Approx(PLAUSIBLE_MIN_V));
  CHECK(worstOf(12.6f, true, 9.9f, true, &worst) == true);
  CHECK(worst == doctest::Approx(9.9f));
  CHECK(worstOf(9.9f, true, 12.6f, true, &worst) == true);
  CHECK(worst == doctest::Approx(9.9f));
}

// FINDING (#131, locked current behavior — do NOT "fix" without its own
// issue + operator sign-off): NaN passes the [min,max] band because both
// comparisons are false, and the ternary min-select `(v0 < v1) ? v0 : v1`
// is slot-asymmetric under NaN — NaN in slot 0 selects slot 1's voltage;
// NaN in slot 1 selects itself (NaN propagates to the out-param).
TEST_CASE("FINDING #131: NaN voltage is NOT rejected by the plausibility gate") {
  const float nan = std::nanf("");
  float worst = -1.0f;
  CHECK(worstOf(nan, true, 12.6f, true, &worst) == true);
  CHECK(worst == doctest::Approx(12.6f));  // slot-0 NaN: slot 1 selected
  worst = -1.0f;
  CHECK(worstOf(12.6f, true, nan, true, &worst) == true);
  CHECK(std::isnan(worst));                // slot-1 NaN: NaN propagates
}
