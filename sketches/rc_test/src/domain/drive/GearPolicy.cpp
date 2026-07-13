// domain/drive — gear selection policy (#117 step 7, #166).
// Logic copied VERBATIM from rc_test.ino [GEAR]; behavior is locked by
// tests/characterization/test_gear_and_caps.cpp,
// tests/invariants/test_invariant_gear_reverse_caps.cpp and
// tests/drive/test_gear_mixer_domain.cpp.
#include "GearPolicy.h"

GearSelection gearPolicySelect(int gearPulse, bool rcValid, bool forceEco,
                               const GearPolicyParameters& parameters) {
  if (!rcValid) {
    return {GEAR_ECO, parameters.ecoScale};
  }
  GearSelection selection;
  if (gearPulse < parameters.selectLowThreshold) {
    selection = {GEAR_ECO, parameters.ecoScale};
  } else if (gearPulse > parameters.selectHighThreshold) {
    selection = {GEAR_BOOST, parameters.boostScale};
  } else {
    selection = {GEAR_NORMAL, parameters.normalScale};
  }
  // Low-battery Eco lockout (#65, latched) OR motor over-temp Eco lock
  // (#111, non-latching): force Eco regardless of the RC gear switch to
  // ease load on a draining pack or a hot motor.
  if (forceEco) {
    selection = {GEAR_ECO, parameters.ecoScale};
  }
  return selection;
}

float reverseCapForGear(GearLevel level, float standardCap, float boostCap) {
  return (level == GEAR_BOOST) ? boostCap : standardCap;
}

float trackCapToAxisDomain(float outputCap, float gearScale) {
  return outputCap / gearScale;
}
