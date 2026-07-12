// domain/drive — RC/joystick command mixer (#117 step 7, #166).
// Logic copied VERBATIM from rc_test.ino [MIXER] maxOppose()/mixCommands();
// behavior is locked by tests/characterization/test_mixer.cpp and
// tests/drive/test_gear_mixer_domain.cpp.
#include "CommandMixer.h"

#include <math.h>

float maxOppose(float a, float b) {
  return fmaxf(fmaxf(a, b), 0.0f) + fminf(fminf(a, b), 0.0f);
}

AxisCommand mixAxisCommands(AxisCommand rc, AxisCommand joystick,
                            int overridePulse, int overrideLowThreshold,
                            int overrideHighThreshold) {
  if (overridePulse < overrideLowThreshold) {          // Mode 1 — RC only
    return rc;
  } else if (overridePulse > overrideHighThreshold) {  // Mode 3 — dual: max/oppose per axis
    return { maxOppose(rc.xSpeed, joystick.xSpeed),
             maxOppose(rc.zRotation, joystick.zRotation) };
  } else {                       // Mode 2 — RC overrides joystick when RC active
    bool rcActive = (rc.xSpeed != 0.0f) || (rc.zRotation != 0.0f);
    return rcActive ? rc : joystick;
  }
}
