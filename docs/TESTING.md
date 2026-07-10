# Host Testing — Characterization + Safety Invariants (#47, #131 — Epic #116 Phase B)

The firmware's control logic runs on a laptop, byte-for-byte unchanged, inside
a fake Arduino environment. Two suites share the harness: **characterization**
(#47) locks in example behavior, and **invariants** (#131) asserts safety
properties that must always hold, under injected faults. Together they are the
**primary safety net for the Phase D refactor**: every extraction PR must keep
them green, proving behavior parity while no hardware is available.

## How it works — zero firmware changes

The production sketch is not modified for testing. Each test binary compiles
`sketches/rc_test/rc_test.ino` directly, plus the extracted
`sketches/rc_test/src/**/*.cpp` domain sources the sketch delegates to as the
Phase D migration (#117) proceeds. Pure-domain suites (one dir per extracted
domain, e.g. `battery/`) test the extracted code directly — no firmware, no
stubs:

```text
tests/
├── Makefile                    one host binary per test file; `make run`
├── vendor/doctest/doctest.h    doctest 2.4.12 (vendored, third-party — lint-exempt)
├── stubs/                      fake Arduino environment (test control surface)
│   ├── Arduino.h               fake clock (millis/micros), scripted analogRead,
│   │                           GPIO capture, UART/Serial stubs, map/constrain
│   ├── Servo.h                 captures attach/detach + every writeMicroseconds
│   ├── WiFiS3.h                inert AP/server/client + modem stub
│   ├── WDT.h                   counts refresh() calls
│   ├── sbus.h                  scripted S.BUS frames (bfs::SbusRx/SbusData)
│   └── SerialPortStub.h        Serial/Serial1/UART fakes — scripted RX, captured TX
├── characterization/
│   ├── FirmwareUnderTest.h     includes the REAL rc_test.ino + firmware state reset
│   └── test_*.cpp              one focused suite per behavior area
├── invariants/
│   ├── InvariantChecks.h       reusable safety-invariant checkers + loop driver (#131)
│   └── test_invariant_*.cpp    one property suite per safety invariant
└── battery/                    pure-domain suite for src/domain/battery/ (#117 step 1)
    └── test_voltage_plausibility.cpp
```

The stub headers shadow the real Arduino/library headers via include order
(`-I tests/stubs` first), so `#include <Arduino.h>` inside the sketch resolves
to the fake. Each stub owns its scripting state and reset (all under
`namespace stub`); `resetHarness()` in `FirmwareUnderTest.h` resets every stub
plus the firmware globals. Tests then script inputs (S.BUS frames, ADC values,
telemetry voltages, the clock) and assert on captured outputs (servo
microseconds, pin states, alarm patterns).

Because tests live in the same translation unit as the firmware, they can read
every internal (including `const` globals) — that is deliberate: this is a
characterization harness, not an API. `resetFirmwareState()` in
`FirmwareUnderTest.h` restores all mutable firmware globals between test
cases; it must be updated when a global is added (the harness fails loudly in
review otherwise — see the PR checklist in the workflow rules).

## What is characterized (V7.34 baseline, captured 2026-07-05)

| Suite | Behavior locked in |
| --- | --- |
| `test_curvature_drive` | straight-line, pure pivot (per-gear caps), smoothstep blend band, pivot throttle taper (#114), reverse steering sign (#86), outer-track ceiling + desaturation (#96), gear average cap (#72) |
| `test_input_shaping` | `sbusToServo` endpoints/center, RC + joystick deadbands, expo curves, `rcCommand` gains + reverse clamp, `updateJoystick` ADC→command pipeline (#90) |
| `test_gear_and_caps` | CH4 gear thresholds, S.BUS-invalid → Eco failsafe, Eco/thermal gear locks, `reverseCap()` per gear (#113), `outCapToX`, per-gear joystick forward caps |
| `test_mixer` | `maxOppose` algebra, override Mode 1/2/3 selection (#90) |
| `test_battery_ladder` | plausibility gating, worst-of-two, Eco-lock 10.8 V/15 s, cutoff 10.0 V/1.5 s latch + alarm assertion, boot gate + fail-open (#65) |
| `test_thermal_stages` | 80/90/95 °C stages, hysteresis releases, 1 s trip debounce, telemetry-dropout hold, restored flourish (#111) |
| `test_output_gate` | ACTIVE→HOLD ease-out ramp→CUT detach, RC-loss auto-recovery, cutoff latch (#88) |
| `test_alerts` | horn OR-ing, one-shot patterns, alarm priority ladder, inactivity 60 s, low-V debounce/latch + startup grace (#51/#68) |
| `test_telemetry_parse` | X.BUS checksum, 0x10 response parsing, register scaling, EMA weights, per-ESC staleness watchdog |
| `test_control_loop` | end-to-end `setup()`+`loop()`: scripted S.BUS → servo µs, failsafe to neutral, exactly one `WDT.refresh()` per pass |

Characterization tests capture **current behavior** — including behavior that
might look odd. If a test fails after a refactor, the refactor changed
behavior; fix the code, not the test. Changing an expected value requires an
issue + operator sign-off (it is a behavior change, not a refactor).

## Safety invariants + fault injection (#131)

`tests/invariants/` asserts **properties**, not examples: things that must hold
for any input combination, under injected faults. The invariant list — verbatim
from issue #131, with constants and per-file links — lives in
[`docs/SAFETY.md`](SAFETY.md); the motivating incident is #69 (blocking Wi-Fi
send froze `loop()` while the ESCs held the last PWM).

| Suite | Property held |
| --- | --- |
| `test_invariant_output_bounds` | outputs within [1000, 2000] µs for any input, incl. NaN/Inf/out-of-range |
| `test_invariant_stale_rc_neutral` | stale/failsafe RC ⇒ neutral in every override mode, joystick live or not |
| `test_invariant_cutoff_dominates` | battery-cutoff latch ⇒ neutral within 500 ms then silence, forever; boot-with-low-pack never drives |
| `test_invariant_thermal_monotone` | rising thermal stage never increases propulsion authority; dropout holds state |
| `test_invariant_plausibility` | implausible/stale/spiking telemetry never trips or releases the ladders |
| `test_invariant_gear_reverse_caps` | average track speed never exceeds the active gear/reverse cap |

Mechanics: `InvariantChecks.h` provides `runControlPasses()`, which drives the
real `loop()` at flight cadence and re-checks the universal invariants
(`checkInvariantsNow()`) after **every** pass — each scenario is a continuous
property check, not an endpoint assertion. Same rule as characterization:
weakening an invariant expectation is a behavior change → own issue + operator
sign-off.

## Running

```sh
make -C tests run          # Linux/WSL/CI: build + run every suite
make -C tests               # build only
```

On this project's Windows dev machine the suite runs through WSL:
`wsl -e make -C tests run`. CI runs it on every push/PR via
`.github/workflows/unit-tests.yml` (required status check).

## Commit gate (hard block)

Three layers, per issue #47:

1. **`.githooks/pre-commit`** — native git hook, the primary gate for EVERY
   commit (agent or human): runs the suite whenever firmware/tests/workflow
   files are staged, plus the firmware-without-tests guard below. Activate
   once per clone: `git config core.hooksPath .githooks`.
   Emergency human bypass: `git commit --no-verify` (document why in the PR).
2. **`.claude/hooks/test-gate.sh`** — Claude Code `PreToolUse[Bash]` hook
   that makes layer 1 agent-proof: it refuses `--no-verify` outright and
   blocks commits in a clone where `core.hooksPath` was never activated.
   Wire it in `.claude/settings.json`:

   ```json
   "PreToolUse": [
     {
       "matcher": "Bash",
       "hooks": [
         { "type": "command", "command": "bash .claude/hooks/test-gate.sh", "timeout": 30000 }
       ]
     }
   ]
   ```

3. **CI `unit-tests` job** — the layer that cannot be bypassed; required for
   merge.

The firmware-without-tests guard blocks a commit that stages changes under
`sketches/rc_test/` without staging any change under `tests/`. If the
firmware change genuinely needs no test update (comment-only, debug text),
re-run as `DIGGER_NO_TEST_CHANGE=1 git commit ...` and say why in the PR.
