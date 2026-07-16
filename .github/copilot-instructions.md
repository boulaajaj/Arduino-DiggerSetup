# Copilot Code Review Instructions — Arduino Digger Controller

## ⚠️ Current program: epic #116 — behavior-preservation covenant (TEMPORARY until epic completes)

The repo is mid architecture-remediation: organize code, split large files,
domain-driven design, doc-code sync, readable and meaningful tests. During
this program, organization must change NOTHING observable — no behavior, no
timing, no parameter/constant values, no input/output handling. Firmware
flashed after a refactor must make the machine perform exactly as before.

- Do NOT suggest behavior changes, constant retunes, or simplifications that
  alter what the firmware does — even where current behavior looks wrong.
  Suspected real defects: describe as an observation + recommend a follow-up
  issue instead of a code suggestion. If it looks SAFETY-CRITICAL (rider harm,
  runaway, ignored cutoff), flag it prominently — it needs its own issue and
  an immediate dedicated fix PR, never a review-correction commit.
- Test expectations are behavior law — changing one requires an issue +
  operator sign-off first. Never suggest editing an expectation to "fix" it;
  verify numeric claims against the code (incl. float32 rounding — forced-Eco
  output is 1825 µs because `0.65f*500.0f` rounds UP to exactly `325.0f`).
- Intentional behaviors — do not flag: thermal-stage hysteresis holds (e.g.
  eco holds at 85 °C in the 90/80 band); telemetry dropout freezes ladders;
  battery boot gate fails OPEN at 3 s; gear/reverse caps bound the AVERAGE
  track speed (per-track turn headroom is by design); battery cutoff latches
  until power-cycle while thermal cut auto-recovers; alarms are sound-only.
- DO flag: code-move errors (dropped/duplicated/reordered logic), dependency
  and naming rule violations (`.claude/rules/`), files past the 150/250-line
  policy, and documentation drifted from code.

## Working Agreement (four principles — mirrors CLAUDE.md, #53)

1. **Think before coding**: safety-relevant ambiguity (pins, ESC parameters,
   caps, timing) is a question, never a guess.
2. **Simplicity first**: no speculative features or one-caller abstractions.
3. **Surgical changes**: one intent per diff — no reformatting/renaming
   drive-bys on lines the task didn't require.
4. **Goal-driven**: a change is done when the host suite
   (characterization + invariants), local `arduino-cli` compile, and all CI
   workflows are green with zero unauthorized behavior change.

## Project Context
Arduino UNO R4 WiFi (Renesas RA4M1, 32-bit ARM Cortex-M4, 48MHz; ESP32-S3 Wi-Fi coprocessor) controlling a ride-on excavator.
Safety-critical: code errors can cause a 50lb machine with a child riding it to behave unexpectedly.

## Critical Checks (Flag as errors)
- **Blocking calls in loop()**: Flag any use of `delay()`, `pulseIn()`, or blocking `while` loops in production sketches. The control loop must be non-blocking at ~20kHz.
- **Missing constrain() on servo output**: Every value written to `writeMicroseconds()` MUST be constrained to 1000-2000.
- **Integer overflow**: Watch for arithmetic that could overflow 32-bit `int` (e.g. multiplying two large values like `micros()` differences, sensor readings). Flag multiplication without explicit casts. Extra care if code is ever ported to 8/16-bit AVR platforms.
- **Float without f suffix**: Flag `1.0` (promotes to `double`). The Cortex-M4 FPU handles single-precision `float` in hardware, but `double` has no hardware support on the RA4M1 — double-precision operations fall back to software emulation and are significantly slower. Always use `1.0f` to keep arithmetic in the hardware FPU.
- **Magic numbers**: In the production sketch (`sketches/dual_track_control/`), flag raw numeric constants outside `src/config/` per-domain headers (or the owning adapter for single-homed infrastructure tunables — X.BUS poll constants in `src/infrastructure/xc/`, Wi-Fi serving tunables in `src/infrastructure/network/`). All tunables must be named constants; single-file bench/test sketches keep theirs local.
- **Missing failsafe**: Any new RC input path must have a timeout/failsafe that returns to neutral (SVC = 1500) when signal is lost.
- **Global state mutation**: Flag functions that modify global state without clear documentation. Prefer passing state explicitly.

## Code Quality Checks (Flag as suggestions)
- **Cyclomatic complexity**: Flag functions with more than 3 levels of nesting or more than 10 branches.
- **Single Responsibility**: One file = one concept (150-line soft / 250-line hard limit, `.claude/rules/architecture.md`). In the production sketch (`sketches/dual_track_control/` — bench/test sketches are single-file tools), layers under `src/` respect the dependency direction (`.ino` → `application/` only; `application/` → `domain/` + `ports/` + `telemetry/` + `alerts/` + `config/`; `infrastructure/` implements `ports/` — full table in `.claude/rules/architecture.md`). Flag functions that mix input reading with output writing.
- **Inefficient patterns**: Flag nested loops, repeated calculations that could be cached, or string operations in the hot loop.
- **Consistent naming**: Constants = UPPER_SNAKE_CASE, structs = PascalCase, functions = camelCase, pins = PIN_NAME.

## Arduino-Specific
- Servo PWM range: 1000-2000us, center 1500us
- ADC: 14-bit (0-16383), center 8192
- SBUS range: 172-1811, center ~992
- Board FQBN: arduino:renesas_uno:unor4wifi
