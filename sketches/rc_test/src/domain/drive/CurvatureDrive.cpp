// domain/drive — curvature tank mix (#117 step 6, #164).
// Body copied VERBATIM from rc_test.ino [DRIVE] curvatureDrive() with two
// mechanical substitutions only: Arduino's constrain() macro expanded as
// clampFloat() (identical ternary, identical float semantics), and the
// tunables read from CurvatureParameters instead of [CONFIG] globals.
// Behavior is locked by tests/characterization/test_curvature_drive.cpp,
// test_mixer.cpp, the invariants suites, and
// tests/drive/test_curvature_drive_domain.cpp.
#include "CurvatureDrive.h"

#include <math.h>

// Arduino's constrain() macro, expanded with identical float semantics.
static inline float clampFloat(float value, float low, float high) {
  return (value < low) ? low : ((value > high) ? high : value);
}

TrackCommand curvatureDriveStep(float xSpeed, float zRotation,
                                float gearScale,
                                const CurvatureParameters& parameters) {
  xSpeed    = clampFloat(xSpeed, -1.0f, 1.0f);
  zRotation = clampFloat(zRotation, -1.0f, 1.0f);

  // Pivot output: counter-rotate the tracks (capped), then gear-scale so low
  // gears pivot gently. Eco uses a looser cap (the caller selects pivotCap)
  // to keep usable pivot authority.
  float cappedRotation = clampFloat(zRotation, -parameters.pivotCap,
                                    parameters.pivotCap);
  // Throttle is tapered by steering in THIS branch only (#114): while
  // turning, low throttle no longer shoves both tracks the same way — the
  // counter-rotation persists and decays smoothly instead of surging.
  float pivotThr =
      xSpeed * (1.0f - parameters.pivotThrottleTaper * fabsf(zRotation));
  float pivotL = (pivotThr - cappedRotation) * gearScale;
  float pivotR = (pivotThr + cappedRotation) * gearScale;

  // Curvature output: the gear caps the AVERAGE (avg); an ADDITIVE steering
  // term (delta) forms the inner/outer differential. delta's sign is set by
  // the STEERING STICK alone — never by the throttle direction — so steering
  // stays consistent in reverse: stick-left = nose-left going forward AND
  // backward (#86 Part 1). The old multiply, avg*(1±|z|), scaled the
  // differential by avg, so when avg went negative (reverse) the inner/outer
  // swapped — that was the mid-range "steering reverses when backing up"
  // bug. Forward is UNCHANGED: for avg ≥ 0, avg ∓ delta == avg*(1∓|z|).
  // Peak magnitude is still |avg|+delta = |avg|·(1+|z|), so the #96 ceiling
  // and #72 headroom are intact.
  float avg   = xSpeed * gearScale;
  float delta = fabsf(avg) * fabsf(zRotation);  // symmetric steering term, >= 0
  float curvL, curvR;
  // delta is subtracted from the track on the side we steer toward and added
  // to the other, regardless of travel direction. Forward that means the
  // steer-side track goes slower (the "inner" track); in reverse the same
  // maths makes it go harder-reverse — which is what keeps the nose turning
  // the SAME way backward. (So avoid the forward-only "inner/outer" framing
  // when reasoning about reverse.)
  if (zRotation > 0) {        // turn LEFT  (nose-left forward AND reverse)
    curvL = avg - delta;      // left  track: less forward / harder reverse
    curvR = avg + delta;      // right track: more forward / less reverse
  } else {                    // turn RIGHT
    curvL = avg + delta;
    curvR = avg - delta;
  }
  // Outer-track ceiling fades from the ESC rail (gentle turn — both tracks
  // moving) down to turnTrackCap (full steer — inner track stopped).
  // |zRotation| is the turn-sharpness signal (inner = avg*(1-|z|) → 0 at
  // full steer), so the borrowed headroom shrinks smoothly as the inner
  // stops — no knee, no RPM feedback (#96). Straight (|z| = 0) keeps the
  // full rail, so straight-line throttle is unchanged. Forward AND reverse
  // are treated symmetrically: reverseCap() bounds the reverse *average*
  // speed upstream, and the outer track then borrows the same turn headroom
  // forward does — so a reverse turn swings precisely instead of slowing
  // (operator decision, 2026-06-28). No reverse-specific ceiling
  // special-case.
  float ceiling =
      1.0f - (1.0f - parameters.turnTrackCap) * fabsf(zRotation);
  float peak = fmaxf(fabsf(curvL), fabsf(curvR));
  if (peak > ceiling) {  // desaturate against the faded ceiling, preserving the turn ratio
    float k = ceiling / peak;
    curvL *= k;
    curvR *= k;
  }

  // Smoothstep blend pivot → curvature as |xSpeed| grows, across a WIDE band
  // so the pivot↔drive hand-off is gradual (no snap near low throttle, #72).
  // Zero slope at both endpoints means the operator never feels a mode
  // boundary.
  float t = (fabsf(xSpeed) - parameters.pivotBlendStart) /
            (parameters.pivotBlendEnd - parameters.pivotBlendStart);
  t = clampFloat(t, 0.0f, 1.0f);
  t = t * t * (3.0f - 2.0f * t);

  return {
    pivotL * (1.0f - t) + curvL * t,
    pivotR * (1.0f - t) + curvR * t
  };
}
