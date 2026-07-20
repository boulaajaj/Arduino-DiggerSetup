// Characterization: curvatureDrive + wheelSpeedsToServo (V7.34 baseline).
// Locks in: straight-line identity, per-gear pivot caps, smoothstep blend
// band (#72), pivot throttle taper (#114), reverse steering sign (#86),
// outer-track ceiling + desaturation (#96), gear turn headroom (#72).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("straight line: output = xSpeed * gearScale on both tracks") {
  resetHarness();
  WheelSpeeds ws = curvatureDrive(1.0f, 0.0f, 1.0f, PIVOT_SPEED_CAP_LOW);
  CHECK(ws.left == doctest::Approx(1.0f));
  CHECK(ws.right == doctest::Approx(1.0f));

  // Below the blend band the pivot branch contributes — but with zero
  // steering both branches equal xSpeed*gearScale, so straight-line throttle
  // is exact at ANY speed (the #114 taper leaves z=0 unchanged).
  ws = curvatureDrive(0.10f, 0.0f, 0.80f, PIVOT_SPEED_CAP_LOW);
  CHECK(ws.left == doctest::Approx(0.08f));
  CHECK(ws.right == doctest::Approx(0.08f));

  ws = curvatureDrive(0.0f, 0.0f, 1.0f, PIVOT_SPEED_CAP_LOW);
  CHECK(ws.left == doctest::Approx(0.0f));
  CHECK(ws.right == doctest::Approx(0.0f));
}

TEST_CASE("inputs beyond ±1 are clamped before mixing") {
  resetHarness();
  WheelSpeeds clamped = curvatureDrive(2.0f, -3.0f, 1.0f, PIVOT_SPEED_CAP_LOW);
  WheelSpeeds unit = curvatureDrive(1.0f, -1.0f, 1.0f, PIVOT_SPEED_CAP_LOW);
  CHECK(clamped.left == doctest::Approx(unit.left));
  CHECK(clamped.right == doctest::Approx(unit.right));
}

TEST_CASE("pure pivot counter-rotates at the per-gear pivot cap") {
  resetHarness();
  // Eco (default failsafe gear) uses the looser 0.725 cap for maneuvering
  // authority: wheel speed = cap * gearScale = 0.725 * 0.65.
  REQUIRE(currentGear == GEAR_LOW);
  WheelSpeeds ws = curvatureDrive(0.0f, 1.0f, GEAR_LOW_SCALE, PIVOT_SPEED_CAP_LOW);
  CHECK(ws.left == doctest::Approx(-0.725f * 0.65f));
  CHECK(ws.right == doctest::Approx(0.725f * 0.65f));

  // Normal/Boost use the standard 0.60 cap.
  ws = curvatureDrive(0.0f, 1.0f, GEAR_MID_SCALE, PIVOT_SPEED_CAP);
  CHECK(ws.left == doctest::Approx(-0.60f * 0.80f));
  CHECK(ws.right == doctest::Approx(0.60f * 0.80f));

  // Below the cap, steering passes through gear-scaled.
  ws = curvatureDrive(0.0f, 0.30f, GEAR_LOW_SCALE, PIVOT_SPEED_CAP_LOW);
  CHECK(ws.left == doctest::Approx(-0.30f * 0.65f));
  CHECK(ws.right == doctest::Approx(0.30f * 0.65f));
}

TEST_CASE("pivot throttle taper (#114): low throttle mid-steer arcs, no surge") {
  resetHarness();
  // 10% throttle at full left steer, Normal gear. Pivot branch throttle is
  // tapered to 0.10*(1-0.70) = 0.03, so the inner track keeps counter-rotating
  // (-0.456 before blend) instead of surging forward with the outer.
  WheelSpeeds ws = curvatureDrive(0.10f, 1.0f, GEAR_MID_SCALE, PIVOT_SPEED_CAP);
  // Blend t at |x|=0.10: smoothstep((0.10-0.05)/0.50) = 0.028.
  // pivot (L,R) = (0.03-0.6, 0.03+0.6)*0.8 ; curvature (L,R) = (0, 0.16).
  CHECK(ws.left == doctest::Approx(-0.443232f).epsilon(0.0005));
  CHECK(ws.right == doctest::Approx(0.494368f).epsilon(0.0005));
  CHECK(ws.left < 0.0f);   // inner still counter-rotating — the #114 fix
}

TEST_CASE("reverse steering (#86): stick-left turns the nose left in both directions") {
  resetHarness();
  WheelSpeeds forward = curvatureDrive(0.5f, 0.5f, GEAR_HIGH_SCALE, PIVOT_SPEED_CAP);
  WheelSpeeds reverse = curvatureDrive(-0.5f, 0.5f, GEAR_HIGH_SCALE, PIVOT_SPEED_CAP);

  // Forward left turn: left track slower.
  CHECK(forward.left < forward.right);
  CHECK(forward.left == doctest::Approx(0.2381f).epsilon(0.001));
  CHECK(forward.right == doctest::Approx(0.7521f).epsilon(0.001));

  // Reverse with the same stick: left track HARDER reverse — nose still
  // swings left. The delta term follows the steering stick, never the
  // throttle sign (the old avg*(1±|z|) mid-range reversal bug).
  CHECK(reverse.left < reverse.right);
  CHECK(reverse.left == doctest::Approx(-0.7521f).epsilon(0.001));
  CHECK(reverse.right == doctest::Approx(-0.2381f).epsilon(0.001));
}

TEST_CASE("outer-track ceiling (#96): full steer at full speed caps the outer at 0.70") {
  resetHarness();
  WheelSpeeds ws = curvatureDrive(1.0f, 1.0f, GEAR_HIGH_SCALE, PIVOT_SPEED_CAP);
  // Pure curvature (t=1): inner stops, outer would hit 2.0 → desaturated to
  // the faded ceiling 1 - (1-0.70)*|z| = 0.70.
  CHECK(ws.left == doctest::Approx(0.0f));
  CHECK(ws.right == doctest::Approx(0.70f));
}

TEST_CASE("gear turn headroom (#72): outer track borrows above the gear cap in Normal") {
  resetHarness();
  WheelSpeeds ws = curvatureDrive(1.0f, 0.5f, GEAR_MID_SCALE, PIVOT_SPEED_CAP);
  // avg=0.8, delta=0.4 → raw (0.4, 1.2); ceiling at |z|=0.5 is 0.85 →
  // k=0.85/1.2, preserving the turn ratio.
  CHECK(ws.right == doctest::Approx(0.85f));
  CHECK(ws.left == doctest::Approx(0.4f * 0.85f / 1.2f));
  // The outer exceeds the 0.80 gear cap — that is the #72 headroom.
  CHECK(ws.right > GEAR_MID_SCALE);
}

TEST_CASE("blend band (#72): smoothstep hand-off between pivot and curvature") {
  resetHarness();
  // At the band edges the branches are pure; midway they mix smoothly.
  // |x| = 0.30 (band middle): t = smoothstep(0.5) = 0.5 exactly.
  WheelSpeeds mid = curvatureDrive(0.30f, 0.40f, GEAR_MID_SCALE, PIVOT_SPEED_CAP);
  // pivot: thr = 0.3*(1-0.7*0.4) = 0.216 → (0.216∓0.4)*0.8 = (-0.1472, 0.4928)
  // curvature: avg = 0.24, delta = 0.096 → (0.144, 0.336), ceiling 0.88, no desat
  // blend 50/50: (-0.0016, 0.4144)
  CHECK(mid.left == doctest::Approx(-0.0016f).epsilon(0.005));
  CHECK(mid.right == doctest::Approx(0.4144f).epsilon(0.005));

  // At/above the band end the pivot branch is fully out.
  WheelSpeeds atEnd = curvatureDrive(0.55f, 0.40f, GEAR_MID_SCALE, PIVOT_SPEED_CAP);
  float avg = 0.55f * 0.80f;
  float delta = avg * 0.40f;
  CHECK(atEnd.left == doctest::Approx(avg - delta));
  CHECK(atEnd.right == doctest::Approx(avg + delta));
}

TEST_CASE("wheelSpeedsToServo: center 1500, rail clamp, truncation toward zero") {
  resetHarness();
  ServoOutput out = wheelSpeedsToServo({0.0f, 0.0f});
  CHECK(out.left == 1500);
  CHECK(out.right == 1500);

  out = wheelSpeedsToServo({1.0f, -1.0f});
  CHECK(out.left == 2000);
  CHECK(out.right == 1000);

  out = wheelSpeedsToServo({1.5f, -1.5f});   // beyond the rail clamps
  CHECK(out.left == 2000);
  CHECK(out.right == 1000);

  // (int) cast truncates toward zero: ±235.625 µs → ±235, symmetric.
  out = wheelSpeedsToServo({0.47125f, -0.47125f});
  CHECK(out.left == 1735);
  CHECK(out.right == 1265);
}
