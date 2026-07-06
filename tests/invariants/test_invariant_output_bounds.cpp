// Invariant #131-1: track outputs are ALWAYS within [SVMIN, SVMAX] µs — for
// any input combination, including NaN / Inf / out-of-range values that no
// real input path can produce today. The guarantee comes from the two-layer
// constrain (wheelSpeedsToServo float→µs, then outputWrite at the hardware
// boundary); this suite proves no path around it exists.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

#include <cmath>

TEST_CASE("mixer grid (#131): outputs bounded for every gear x xSpeed x zRotation, incl. out-of-range") {
  resetHarness();
  const struct { Gear gear; float scale; } gears[] = {
    {GEAR_LOW, GEAR_LOW_SCALE}, {GEAR_MID, GEAR_MID_SCALE}, {GEAR_HIGH, GEAR_HIGH_SCALE}};
  // Grid straddles the pivot blend band (0.05 / 0.55), the pivot caps
  // (0.60 / 0.725), full deflection, and out-of-range values.
  const float grid[] = {-2.5f, -1.0f, -0.725f, -0.55f, -0.3f, -0.05f, 0.0f,
                        0.05f, 0.3f, 0.55f, 0.725f, 1.0f, 2.5f};
  int checked = 0;
  int outOfBounds = 0;
  for (const auto& g : gears) {
    currentGear = g.gear;  // curvatureDrive reads this global for the pivot cap
    gearScale = g.scale;
    for (float xSpeed : grid) {
      for (float zRotation : grid) {
        ServoOutput out = wheelSpeedsToServo(curvatureDrive(xSpeed, zRotation, g.scale));
        if (out.left < SVMIN || out.left > SVMAX ||
            out.right < SVMIN || out.right > SVMAX) {
          outOfBounds++;
        }
        checked++;
      }
    }
  }
  CHECK(outOfBounds == 0);
  CHECK(checked == 3 * 13 * 13);  // proves the whole grid actually ran
}

TEST_CASE("non-finite mixer inputs (#131): NaN / Inf cannot escape the us clamp") {
  resetHarness();
  // No real input path produces non-finite floats (S.BUS and ADC are integer-
  // derived), but the float constrain() macro passes NaN through untouched —
  // only the int-domain clamp in wheelSpeedsToServo stops it. The float→int
  // conversion of NaN/Inf is toolchain-specific (x86 saturates differently
  // from the RA4M1's FPU), so assert ONLY boundedness, never the exact value.
  const float poison[] = {NAN, INFINITY, -INFINITY};
  const float normals[] = {-1.0f, -0.3f, 0.0f, 0.3f, 1.0f};
  int checked = 0;
  int outOfBounds = 0;
  auto probe = [&](float xSpeed, float zRotation) {
    ServoOutput out = wheelSpeedsToServo(curvatureDrive(xSpeed, zRotation, gearScale));
    if (out.left < SVMIN || out.left > SVMAX ||
        out.right < SVMIN || out.right > SVMAX) {
      outOfBounds++;
    }
    checked++;
  };
  for (float bad : poison) {
    for (float other : normals) {
      probe(bad, other);
      probe(other, bad);
    }
    for (float alsoBad : poison) probe(bad, alsoBad);
  }
  CHECK(outOfBounds == 0);
  CHECK(checked == 3 * 5 * 2 + 3 * 3);
}

TEST_CASE("outputWrite (#131): hardware boundary clamps arbitrary integers") {
  resetHarness();
  outputInit();
  const int extremes[] = {-2147483647 - 1, -1, 0, 999, 1000, 1500, 2000, 2001,
                          65535, 2147483647};
  for (int left : extremes) {
    for (int right : extremes) {
      outputWrite(left, right);
      CHECK(outL >= SVMIN);
      CHECK(outL <= SVMAX);
      CHECK(outR >= SVMIN);
      CHECK(outR <= SVMAX);
    }
  }
  checkInvariants();
}

TEST_CASE("end-to-end (#131): corrupt S.BUS raws + railed ADC never escape bounds") {
  resetHarness();
  setup();
  // Corrupt/degenerate raw channel values (S.BUS channels are 11-bit 0..2047;
  // feed beyond that too), Mode 3 blend so the railed joystick contributes,
  // Boost gear for zero cap headroom.
  const int16_t corruptRaws[] = {-1000, 0, 172, 992, 1811, 2047, 32767};
  const int16_t adcRails[] = {0, 16383};
  for (int16_t throttleRaw : corruptRaws) {
    for (int16_t steeringRaw : corruptRaws) {
      for (int16_t adc : adcRails) {
        stub::setAnalogValue(PIN_JOY_Y, adc);
        stub::setAnalogValue(PIN_JOY_X, 16383 - adc);
        bfs::SbusData frame =
            driveFrame(throttleRaw, steeringRaw, RAW_SWITCH_HIGH, RAW_SWITCH_HIGH);
        // 12 passes x 1 ms spans one full joystick ADC cache window (10 ms),
        // so the railed sticks actually reach the mixer.
        runControlPasses(12, &frame, 1, true);
      }
    }
  }
  checkInvariants();
}
