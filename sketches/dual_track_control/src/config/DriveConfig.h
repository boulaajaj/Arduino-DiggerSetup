// config — drive/output tunables: servo authority, per-gear caps, pivot
// blend (#185).
#pragma once

// Servo PWM range (matches GL10's standard 50 Hz, 1-2 ms input spec)
const int SVC   = 1500;  // Center (neutral)
const int SVMIN = 1000;  // Full reverse
const int SVMAX = 2000;  // Full forward

// ── Throttle output caps ──────────────────────────────────────────────────
// All caps below are MAX TRACK-OUTPUT fractions (0..1). A straight-line track
// output = xSpeed * gearScale, so each cap is converted to the xSpeed domain
// via outCapToX() at point of use — one helper, no scattered conversions.

// Per-gear joystick FORWARD cap (#90) — the joystick rider's max forward track
// output per gear (RC keeps the gear's full authority). Throttle axis only;
// steering / pivot unaffected.
const float JOY_CAP_ECO    = 0.65f;  // Eco
const float JOY_CAP_NORMAL = 0.75f;  // Normal
const float JOY_CAP_BOOST  = 0.90f;  // Boost

// REVERSE caps (#87/#113) — per gear, flat across BOTH inputs (RC + joystick).
// Reverse is held below forward authority; Boost is allowed a little more reverse
// than Eco/Normal. Applied through reverseCap() (defined with the other gear-cap
// helpers in the sketch) so there is ONE source of truth, not a scattered
// conditional. These are TRUE percentages only because the GL10s were
// recalibrated to the full 1000/1500/2000 us range on 2026-07-03. Previously
// they were calibrated while the firmware capped reverse at 65%, so they
// mislearned 65% as 100% reverse — which made a 65%-commanded reverse drive
// ~100%. Do NOT lower the firmware reverse cap and recalibrate at the same
// time, or that mislearning returns.
const float REVERSE_CAP_STD   = 0.55f;  // Eco + Normal
const float REVERSE_CAP_BOOST = 0.65f;  // Boost

// Power range — full PWM authority (1000-2000 us = ±500 us from SVC)
const float SOFT_RANGE = 500.0f;  // Max servo offset from center (us)

// Gear scaling — RC CH4 selects the AVERAGE-speed cap. 3-position switch:
//   LOW  → 65% average-speed cap  (training / tight spaces)
//   MID  → 80% average-speed cap  (normal driving — the everyday gear)
//   HIGH → 100% (the rail)        (full throttle authority)
// The cap limits the AVERAGE track speed, not each wheel: in a turn the outer
// track may use the headroom up to the ESC limit so the turn holds its speed
// (see curvatureDrive). Boost has no headroom (already at the rail).
// Failsafe: when S.BUS is invalid, gearScale stays at LOW for safety.
const float GEAR_LOW_SCALE  = 0.65f;  // Eco   (+10pp 2026-06-21 for usefulness; keeps ~35% turn headroom)
const float GEAR_MID_SCALE  = 0.80f;  // Normal (+10pp 2026-06-21 — the everyday gear; keeps ~20% turn headroom)
const float GEAR_HIGH_SCALE = 1.00f;  // Boost  (no turn headroom — at the rail)

// Eco gets extra PIVOT authority so the operator can still maneuver in tight
// spaces (pivot input cap; forward stays at GEAR_LOW_SCALE 65%). Effective pivot
// wheel speed = pivot cap × gear scale: 0.725 × 0.65 = 0.47 (vs 0.60 × 0.65).
// (Reverse is capped per gear via reverseCap(): 55% Eco/Normal, 65% Boost.)
const float PIVOT_SPEED_CAP_LOW = 0.725f;

// Curvature drive — pivot/curvature blend band.
// |xSpeed| <= START: pure pivot (counter-rotate at PIVOT_SPEED_CAP)
// |xSpeed| >= END:   pure curvature (outer holds the average, inner slows)
// Between: smoothstep blend so the operator doesn't feel a mode jump. The band
// is WIDE (0.05–0.55) so the pivot↔forward/reverse hand-off is gradual — a
// narrow band made the transition snap near 15–20% throttle (#72).
const float PIVOT_BLEND_START = 0.05f;
const float PIVOT_BLEND_END   = 0.55f;
const float PIVOT_SPEED_CAP   = 0.60f;  // pivot rotation cap (~60% wheel power)

// Pivot-branch throttle taper (#114). Throttle used to enter BOTH tracks of the
// pivot branch at full gain, so 10–20% throttle while steering shifted both
// tracks forward together — an instant surge (mirrored in reverse). Taper the
// pivot branch's throttle term by steering: while turning, the outer track
// keeps most of its pivot speed (it still gains (1−taper)·throttle) and the
// inner keeps a slight counter-rotation that eases through zero as throttle
// rises; forward speed then builds as the blend hands over to the curvature
// branch. The taper deliberately follows the RAW steering stick, not
// cappedRotation: past the pivot cap extra deflection adds no rotation, but it
// still reads as "hold the pivot", so full lock holds hardest (2026-07-03,
// field-validated — see docs/DECISION-LOG.md). 0.0 = old behavior, 1.0 =
// throttle fully suppressed at full steer. Straight-line (z = 0), pure pivot
// (x = 0), and at-speed turns (|xSpeed| >= PIVOT_BLEND_END → pure curvature)
// are mathematically unchanged. Field-tune WITHIN [0, 1] — enforced below
// because above 1.0 the term flips sign (forward stick would command reverse).
constexpr float PIVOT_THROTTLE_TAPER = 0.70f;
static_assert(PIVOT_THROTTLE_TAPER >= 0.0f && PIVOT_THROTTLE_TAPER <= 1.0f,
              "PIVOT_THROTTLE_TAPER outside [0,1] inverts pivot-branch throttle");

// Outer-track turn cap (#96). The #72 outer-track headroom lets the outer wheel
// borrow up to the ESC rail to hold speed through a turn — but when the INNER
// track is stopped (full steer) that headroom is wasted (the outer doesn't need
// ~99% to swing the nose). curvatureDrive fades the outer-track ceiling from the
// rail (straight, both tracks moving) down to TURN_TRACK_CAP (full steer, inner
// stopped), driven by |zRotation| — open-loop, smooth, no hard switch.
const float TURN_TRACK_CAP    = 0.70f;  // outer-track cap at full steer (field-tune)

// RC input gains — neutral baseline (1.0 = no scaling). Stick travel
// maps directly to curvatureDrive, which already handles inner-track
// slowdown and pivot/curvature blend. Earlier non-unity values were
// band-aids compensating for the flipped-ESC steering bug; with the
// root cause fixed, the gains return to neutral.
const float RC_THROTTLE_GAIN = 1.00f;
const float RC_STEERING_GAIN = 1.00f;
