# Architecture — how the code is shaped, and why

Written for a human engineer landing on the repo. The machine-checked
contracts live elsewhere (spec: [ARCHITECTURE-TARGET](architecture/ARCHITECTURE-TARGET.md),
inventory: [FILE-MAP](architecture/FILE-MAP.md), authority hierarchy:
[authority-matrix](wiki/authority-matrix.md)); this page is the narrative.

## The shape

The production sketch's `.ino` is a small composition root: it includes
the application layer, delegates `setup()`/`loop()`, and does nothing
else. Everything real lives under
`sketches/dual_track_control/src/` in one-way layers:

```text
.ino ─▶ application ─▶ domain      (pure logic: battery, thermal,
              │        operator_input, drive, safety — no hardware
              │        includes, time passed as a parameter)
              ├──────▶ ports       (hardware contracts, link-time seams)
              ├──────▶ telemetry   (read-only observers of one snapshot)
              ├──────▶ alerts      (beep policy + pattern players)
              └──────▶ config      (every tunable, per domain)
        infrastructure ─▶ ports    (the ONLY layer with hardware includes:
                                    Arduino core, S.BUS, X.BUS, Wi-Fi)
```

One sanctioned read-back: the telemetry and alert observers read a single
application-owned struct (the SystemSnapshot) — a data edge by design,
not a dependency inversion.

Domain logic is host-testable by construction: the test suites compile the
REAL firmware against a stub Arduino environment, and the pure-domain
suites compile without stubs at all — an accidental hardware include fails
the build loudly ([TESTING](TESTING.md)).

## The signal pipeline

One loop pass, in order (full walk with ordering invariants:
[application-loop](wiki/application-loop.md)):

1. **Inputs** — S.BUS frame (all channels in one frame, one freshness
   timeout) and the 14-bit joystick ADC on its own cadence.
2. **Protection ladders** — battery Eco-lock and cutoff, thermal stages —
   run BEFORE gear selection so a lock can force Eco.
3. **Shaping and mixing** — deadband, per-axis expo, then one
   `curvatureDrive` pass over the combined RC+joystick command: the gear
   cap bounds the AVERAGE track speed (the outer track keeps turn
   headroom), pivot mode blends in smoothly at standstill.
4. **Safety decision + output gate** — drive-allowed is decided from RC
   freshness, battery confirmation, the cutoff latch, and the thermal cut;
   the gate executes ACTIVE → ease-to-neutral → CUT (ESCs lose signal and
   failsafe), re-attaching on recovery.
5. **Watchdog refresh — the only refresh point, directly after the control
   path.** Any stall below misses it, the MCU resets, PWM stops, the
   machine stops. This is the runaway backstop.
6. **Observers** — X.BUS telemetry poll and the Wi-Fi dashboard, both
   strictly read-only with respect to control.

Current values for every threshold and cap live in
`sketches/dual_track_control/src/config/` — no document carries a live
copy, by rule.

## The safety model

- **Battery ladder** (three escalating stages: Eco lock, audible alarm,
  motor cutoff that LATCHES until power-cycle) with a plausibility filter
  so garbage telemetry can neither trip nor release it. A boot gate holds
  drive until a valid reading confirms the pack after a reset wipes the
  latch — and deliberately fails OPEN if telemetry never reports, so a
  dead sensor bus cannot permanently disable driving.
- **Thermal stages** with hysteresis — a stage releases at a lower
  temperature than it trips, so the machine never oscillates at a
  boundary; unlike the battery cutoff, the thermal cut auto-recovers when
  the motor cools and the gate re-attaches the ESCs (RC-loss recovery
  behaves the same way).
- **Output gate state machine** — neutral is commanded and held before PWM
  is cut, so the ESC decelerates smoothly instead of freewheeling.
- **Three permanent invariants**: control is open-loop (telemetry never
  feeds throttle — a bad sensor cannot command motion); there is no
  control path over Wi-Fi (the dashboard can only watch); and the
  hardware watchdog is refreshed exactly once per pass, directly after
  the control path — never anywhere else. All are locked by executable
  invariant tests ([SAFETY](SAFETY.md)).

Every invariant maps to a host test that drives the real `loop()` under
injected faults and re-checks the property after every pass.

## Key decisions, with receipts

Dated rationale lives in [DECISION-LOG](DECISION-LOG.md); the big ones:

- **FOC in the ESC, not PID on the Arduino** — the GL10's internal
  field-oriented control owns motor smoothness, so the Arduino-side
  dual-loop PID plan was retired and the firmware's job shrank to
  shaping, mixing, capping, and gating (2026-04-25).
- **Telemetry via X.BUS Read Register, never bulk throttle** — the
  read-only service function preserves full PWM control authority; the
  bulk function would seize it.
- **Hardware UARTs only for telemetry** — bit-banged serial input is
  banned on the control board.
- **Bounded Wi-Fi serving** — capped modem writes, ETag/304 caching, and
  incremental page transfer keep the dashboard from ever stalling the
  control loop (the watchdog would catch it; the design avoids it).

## What runs where

- **RA4M1 (the Arduino's Cortex-M4)** — everything in this repo's `src/`:
  the pipeline, ladders, gate, watchdog, telemetry poll, HTTP/SSE serving.
- **ESP32-S3 (the UNO R4 WiFi's radio coprocessor)** — the Wi-Fi modem,
  driven through the core's AT-command bridge; no application code runs
  there today.
- **GL10 ESCs** — field-oriented commutation, acceleration and drag-brake
  shaping, and their own low-voltage/thermal protections, all internal;
  the Arduino speaks plain 50 Hz servo PWM to them.
