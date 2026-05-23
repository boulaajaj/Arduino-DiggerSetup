# Plant Characterization — XC E10 ESC + XC E3665 Motor

Measured responses of the ESC + motor system under known open-loop throttle
commands. This document is a reference for any future controller design —
gains, feed-forward tables, rate limits, and minimum throttle thresholds
should all start from the numbers recorded here, not from guesses.

## Test conditions (important — read before trusting any number)

- **Date:** 2026-04-20.
- **Load:** Rubber tracks mounted on sprockets, vehicle inverted (upside-down)
  so tracks spin freely. **No vehicle weight, no operator, no driving load.**
  Only friction from the tracks and drivetrain.
- **Tracks:** mounted, adding some rotational inertia and friction.
- **Battery:** 3S LiPo, bus voltage 11.7–11.8 V throughout all tests (no sag).
- **Temperature:** room temp; ESC internal reported ~18 °C.
- **Control rate:** 100 Hz (10 ms period), measured effective ~99 Hz.
- **Communication:** X.BUS half-duplex over Arduino Nano R4 Serial1 (D0/D1).
- **Telemetry:** alternated between ESC0 and ESC1 each cycle, so each ESC
  reports RPM at 50 Hz.
- **Throttle resolution:** internal scale 0–1000 = 0–100% (x0.1%).
- **Data files:** `docs/measurements/deadzone.csv`,
  `docs/measurements/staircase_5runs.csv`, `docs/measurements/step_test.csv`.

**Results will differ once the vehicle is loaded.** Track friction on the
ground, vehicle mass, and drivetrain bind will all raise the minimum throttle
needed for motion and change the steady-state curve. Re-run all tests after
the vehicle is assembled and on the ground to build the real FF table.

## Test 1 — Staircase 5% steps, 1 s hold, 5 runs

**Protocol:** 5 s baseline at 0%, then 12 × 1-second steps from 5% to 60%,
2 s rest, repeat 5 times. 60 step measurements per motor.

**Steady-state RPM (mean ± standard deviation across 5 runs, left motor):**

| Throttle % | Mean RPM | StDev | Notes |
| --- | --- | --- | --- |
| 5 | 0 | 0 | ZERO in all 5 runs |
| 10 | 1220 | 494 | **HIGHLY variable** — 379 to 1647 RPM |
| 15 | 2774 | 214 | settling down |
| 20 | 4154 | 367 | |
| 25 | 5714 | 189 | |
| 30 | 6827 | 224 | |
| 35 | 8120 | 203 | |
| 40 | 9003 | 128 | very repeatable |
| 45 | 9554 | 163 | |
| 50 | 10377 | 125 | |
| 55 | 10873 | 169 | |
| 60 | 11243 | 126 | |

**Response time (ms to reach 90% of new level from previous level):**

| Throttle % | Mean rise | Range |
| --- | --- | --- |
| 10 | 573 ms | 413–837 |
| 15 | 799 ms | 637–898 |
| 20 | 608 ms | 304–828 |
| 25 | 611 ms | 294–898 |
| 30 | 416 ms | 360–558 |
| 35 | 635 ms | 493–848 |
| 40 | 611 ms | 293–897 |
| 45 | 326 ms | 220–504 |
| 50 | 358 ms | 290–454 |
| 55 | 395 ms | 210–624 |
| 60 | 259 ms | 20–443 |

**Pattern:** response gets faster as throttle increases — each 5 % buys less
RPM, and what it does buy arrives quicker.

## Test 2 — Dead-zone verification, 3% to 8%, 3 s hold, 2 runs

**Protocol:** 5 s baseline at 0%, then each of 3%, 4%, 5%, 6%, 7%, 8% held
for 3 seconds with 2 s rests between, repeated twice.

| Throttle % | Motor L (RPM range) | Motor R (RPM range) | Motor ever moved? |
| ---------- | ------------------- | ------------------- | ----------------- |
| 3 | 0 — 0 | 0 — 0 | **NO** |
| 4 | 0 — 0 | 0 — 0 | **NO** |
| 5 | 0 — 0 | 0 — 0 | **NO** |
| 6 | 0 — 0 | 0 — 0 | **NO** |
| 7 | 0 — 0 | 0 — 356 | Only ESC1, inconsistent |
| 8 | 30 — 52 | 60 — 359 | Yes, but barely; asymmetric |

**Conclusion:** with tracks-only load, **the hard dead zone extends to 6%.**
The ESC accepts and reports our throttle commands (`Thr_in` in telemetry
matches what we sent), but the motor does not overcome its own friction
until the commanded value reaches roughly 7–8%, and that transition is
unreliable and asymmetric between motors.

## Shape of the curve (three regimes)

1. **Hard dead zone (0–6% throttle):** motor does not spin, period. Any
   controller that commands a non-zero throttle in this range is wasting
   energy and producing nothing.
2. **Transition region (7–10%):** one motor starts before the other, run-to-run
   variance is large (up to 41% at 10%). Closed-loop control that tries to
   live here will chatter — avoid.
3. **Reliable region (15%–60%):** variance drops under 5% of mean. Linear
   interpolation between measured points is a good enough model here.

## Asymmetry between the two motors / ESCs

At the same commanded throttle, ESC1 consistently reports higher RPM than
ESC0 by roughly 5–15%. At 8% throttle, ESC1 starts spinning before ESC0
every time. Likely causes:

- Slightly different track friction on each side (tracks are not perfectly
  matched when mounted).
- Minor motor or ESC calibration differences.

Any controller should either (a) store separate FF tables per ESC or
(b) apply a per-side trim that the integrator learns at runtime.

## Timing — what the data tells us about control loop bandwidth

- **Control / communication rate:** 100 Hz (10 ms period). Measured bus
  transaction time 4.7 ms average, 7.3 ms peak — we could push to ~125 Hz
  but there is no reason to. The plant does not respond that fast.
- **ESC processing delay:** ~300 ms from command to first motor motion.
- **Per-5% step response (rise to 90%):** 260–800 ms depending on where on
  the curve the step happens. Slowest in the low-mid range (5–25%).
- **Full 0→40% step:** ~2 seconds to steady state (from an earlier test).

**Implication:** outer control loop (the part that reads RPM and adjusts
throttle) cannot correct faster than about 3 Hz (~300 ms per update) without
oscillating. The safe design target is 1–2 Hz for the closed-loop correction,
with a slew-limited output smoothing everything in between.

## Startup / safety behavior: THR_NOTZERO lockout

Both ESCs latch the `THR_NOTZERO` flag (status bit 5) if they detect
non-zero throttle during the first second or so after power-up or after a
`Restart (0xA0)` broadcast. While latched:

- Telemetry shows our commanded `Thr_in` correctly, but `Thr_out = 0.0%`.
- Motor does not move.
- `BRAKING` flag is also set — the ESC is actively holding the motor still.

**Required startup sequence to avoid the lockout:**

1. Power on.
2. Wait ≥ 1 s for boot.
3. `Restart (0xA0)` broadcast.
4. Wait ≥ 1 s.
5. `Clear Flags (0xA1)` broadcast (twice, 100 ms apart — the first can race
   the reboot).
6. Hold throttle at 0 for ≥ 5 s before any commanded motion.
7. Then ramp.

Skipping any step risks latching the flag. Recovery requires another
Restart + Clear Flags cycle; the motor refuses to start until it is done.

## Summary of numbers for controller design

- `DEAD_ZONE_MAX` = 60 (below 6% commanded throttle, output zero always).
- `MIN_USABLE_THROTTLE` ≈ 100 (10%) for reliable motion.
- `MAX_OBSERVED_RPM` ≈ 11,250 at 60% throttle (no-load).
- Outer-loop correction update rate: 1–2 Hz.
- Output slew limit: 1% per 10 ms cycle (10 units in internal scale).
- Per-ESC asymmetry: ±15% RPM at the same throttle — handle explicitly.
- Startup: 5-second zero-throttle hold after Restart + Clear Flags.

## Files in this test series

All under `docs/measurements/`:

- `deadzone.csv` — 3%–8% verification (2 runs, 3 s hold each)
- `staircase_5runs.csv` — 5%–60% × 5 runs with 1 s holds (main characterization)
- `step_test.csv` — earlier 0→40% and 0→60% long-hold step response
