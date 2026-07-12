// Pure-domain suite (#117 step 7, #166): src/domain/drive GearPolicy +
// CommandMixer tested directly on the host — no firmware, no stubs,
// thresholds/scales passed explicitly. End-to-end behavior through
// updateGear()/reverseCap()/outCapToX()/mixCommands() stays covered by
// tests/characterization/test_gear_and_caps.cpp, test_mixer.cpp and
// tests/invariants/test_invariant_gear_reverse_caps.cpp; this suite locks
// the decision tree, the caps, the maxOppose truth table and all three
// override modes at the unit level.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/rc_test/src/domain/drive/CommandMixer.h"
#include "../../sketches/rc_test/src/domain/drive/GearPolicy.h"

// Values mirror [CONFIG] — parameters here, no config dependency.
const GearPolicyParameters GEAR_PARAMETERS{0.65f, 0.80f, 1.00f, 1400, 1600};
const float REVERSE_STANDARD = 0.55f;
const float REVERSE_BOOST    = 0.65f;

TEST_CASE("gear decision tree: thresholds are strict, between is Normal") {
  CHECK(gearPolicySelect(1200, true, false, GEAR_PARAMETERS).level == GEAR_ECO);
  CHECK(gearPolicySelect(1400, true, false, GEAR_PARAMETERS).level
        == GEAR_NORMAL);  // exactly the low threshold is NOT below it
  CHECK(gearPolicySelect(1500, true, false, GEAR_PARAMETERS).level
        == GEAR_NORMAL);
  CHECK(gearPolicySelect(1600, true, false, GEAR_PARAMETERS).level
        == GEAR_NORMAL);  // exactly the high threshold is NOT above it
  CHECK(gearPolicySelect(1800, true, false, GEAR_PARAMETERS).level
        == GEAR_BOOST);
  CHECK(gearPolicySelect(1800, true, false, GEAR_PARAMETERS).scale
        == doctest::Approx(1.00f));
}

TEST_CASE("gear failsafe and Eco forces dominate the switch") {
  // RC invalid → Eco no matter the pulse.
  CHECK(gearPolicySelect(1800, false, false, GEAR_PARAMETERS).level
        == GEAR_ECO);
  // forceEco (battery Eco lock OR thermal Eco) overrides even Boost.
  GearSelection forced = gearPolicySelect(1800, true, true, GEAR_PARAMETERS);
  CHECK(forced.level == GEAR_ECO);
  CHECK(forced.scale == doctest::Approx(0.65f));
}

TEST_CASE("reverse cap per gear and the cap-domain conversion") {
  CHECK(reverseCapForGear(GEAR_ECO, REVERSE_STANDARD, REVERSE_BOOST)
        == doctest::Approx(0.55f));
  CHECK(reverseCapForGear(GEAR_NORMAL, REVERSE_STANDARD, REVERSE_BOOST)
        == doctest::Approx(0.55f));
  CHECK(reverseCapForGear(GEAR_BOOST, REVERSE_STANDARD, REVERSE_BOOST)
        == doctest::Approx(0.65f));
  // outCapToX: a 55% track cap in Eco (scale 0.65) is ~84.6% of xSpeed.
  CHECK(trackCapToAxisDomain(0.55f, 0.65f) == doctest::Approx(0.55f / 0.65f));
  CHECK(trackCapToAxisDomain(0.65f, 1.00f) == doctest::Approx(0.65f));
}

TEST_CASE("maxOppose truth table (#90)") {
  CHECK(maxOppose(0.6f, 0.3f) == doctest::Approx(0.6f));    // larger wins
  CHECK(maxOppose(1.0f, -0.3f) == doctest::Approx(0.7f));   // dominant minus opposing
  CHECK(maxOppose(0.4f, 0.0f) == doctest::Approx(0.4f));    // single passes through
  CHECK(maxOppose(-0.4f, -0.7f) == doctest::Approx(-0.7f));  // reverse: larger magnitude wins
  CHECK(maxOppose(0.0f, 0.0f) == doctest::Approx(0.0f));
}

TEST_CASE("mixer modes: RC-only, RC-overrides, dual max/oppose") {
  const AxisCommand rc{0.5f, -0.2f};
  const AxisCommand joystick{0.8f, 0.1f};
  // Mode 1 (pulse below low threshold): RC only.
  AxisCommand mode1 = mixAxisCommands(rc, joystick, 1200, 1400, 1600);
  CHECK(mode1.xSpeed == doctest::Approx(0.5f));
  CHECK(mode1.zRotation == doctest::Approx(-0.2f));
  // Mode 3 (pulse above high threshold): max/oppose per axis.
  AxisCommand mode3 = mixAxisCommands(rc, joystick, 1800, 1400, 1600);
  CHECK(mode3.xSpeed == doctest::Approx(0.8f));
  CHECK(mode3.zRotation == doctest::Approx(-0.1f));
}

TEST_CASE("Mode 2: RC wins whenever ANY RC axis is nonzero (exact-zero law)") {
  const AxisCommand joystick{0.8f, 0.1f};
  // Both RC axes exactly zero → the joystick drives.
  AxisCommand idle = mixAxisCommands(AxisCommand{0.0f, 0.0f}, joystick,
                                     1500, 1400, 1600);
  CHECK(idle.xSpeed == doctest::Approx(0.8f));
  // The slightest RC input takes full authority — the compare is exact-zero
  // (locked behavior; RC deadband upstream is what makes this safe).
  AxisCommand touched = mixAxisCommands(AxisCommand{0.001f, 0.0f}, joystick,
                                        1500, 1400, 1600);
  CHECK(touched.xSpeed == doctest::Approx(0.001f));
  CHECK(touched.zRotation == doctest::Approx(0.0f));
}
