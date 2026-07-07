# Safety Invariants (#131)

Properties of the propulsion path that must **always** hold — for any input
combination, in any override mode, under any injected fault. Each invariant is
executable: it has a host test in `tests/invariants/` that drives the real
firmware (see `docs/TESTING.md`), runs in CI with the #47 characterization
suite, and hard-blocks commits via `.githooks/pre-commit`.

This registry lists the invariants from issue
[#131](https://github.com/boulaajaj/Arduino-DiggerSetup/issues/131) verbatim,
with scope clarifications added in parentheses where the issue wording alone
could over-promise. The
human-facing safety-model narrative (battery ladder, thermal stages, output
gate, watchdog) will live in `docs/ARCHITECTURE.md` when #126 lands; this file
stays the single list the tests are named after.

## Motivating incident (#69)

On 2026-06-20 a blocking ~33 KB Wi-Fi page send froze `loop()` for 1–2 s while
the hardware-timed servo PWM **held the last throttle** — the machine kept
driving, uncommanded, under load. The fix was V7.13 (250 ms watchdog +
incremental serving; see `docs/DECISION-LOG.md` entries 2026-06-20 and
2026-06-21, and ADR-0001). This suite exists so that class of failure is a
**failing test, not a field discovery**.

## The invariants

| # | Invariant (verbatim from #131) | Key constants | Test |
| --- | --- | --- | --- |
| 1 | Track outputs always within [1000, 2000] µs for ANY input combination (including NaN / Inf / out-of-range) | `SVMIN`/`SVMAX`, double `constrain()` (float and µs domains) | `tests/invariants/test_invariant_output_bounds.cpp` |
| 2 | Stale or invalid RC input never produces non-neutral propulsion | `SBUS_TIMEOUT` 100 ms, receiver failsafe bit, forced-neutral mix + closed gate | `tests/invariants/test_invariant_stale_rc_neutral.cpp` |
| 3 | Battery cutoff latched ⇒ output gate closed ⇒ both tracks neutral, regardless of drive request | `CUTOFF_THRESH_V` 10.0 V, `CUTOFF_DEBOUNCE_MS` 1.5 s, permanent latch | `tests/invariants/test_invariant_cutoff_dominates.cpp` |
| 4 | Rising thermal stage never increases allowed propulsion (monotone derating) | 80/90/95 °C stages, 78/80/75 °C releases, 1 s trip debounce | `tests/invariants/test_invariant_thermal_monotone.cpp` |
| 5 | Implausible battery/temperature readings are never trusted (plausibility gates hold under fault injection) | voltage band [6, 13] V both packs, temperature band [−20, 200] °C per sensor, `TELEM_STALE_MS` 5 s | `tests/invariants/test_invariant_plausibility.cpp` |
| 6 | Output gate closed ⇒ commands ease to neutral within `CUTOFF_HOLD_MS`, then no pulses | `CUTOFF_HOLD_MS` 500 ms ramp, then `detach()` — signal loss, not neutral pulses | `tests/invariants/test_invariant_cutoff_dominates.cpp` (timing + monotone ramp), `test_invariant_stale_rc_neutral.cpp` |
| 7 | No joystick + RC combination exceeds the active gear / reverse caps (on the **average** track command) | gear 0.65/0.80/1.00, reverse 0.55/0.55/0.65 — single-track excursions in turns are design headroom (#72/#113), so per-track values may legitimately exceed the caps | `tests/invariants/test_invariant_gear_reverse_caps.cpp` |

`tests/invariants/InvariantChecks.h` holds the reusable checker the issue asked
for: `checkInvariants()` (full pulse-history bounds) plus a per-pass
`checkInvariantsNow()` that `runControlPasses()` re-evaluates after **every**
simulated loop pass, so every scenario continuously enforces the universal
invariants while exercising its specific one.

## Injected faults covered

Telemetry dropout mid-drive · one ESC silent · voltage spike/step shorter than
debounce · implausible voltage (unpowered 0.4 V, over-range 14.2 V) ·
temperature sensor spike (implausible 250 °C / −30 °C, sub-debounce 96 °C) ·
RC loss during pivot · receiver-failsafe bit with live joystick · boot with
low pack (boot gate) · NaN voltage (see known gaps).

## Host-testable vs bench-only

The watchdog (#69, `WDT_TIMEOUT_MS` 250 ms) cannot fire in the host stub; the
host suite asserts its *contract* (exactly one `WDT.refresh()` per loop pass,
after the control path — `test_control_loop.cpp`). That the WDT reset actually
stops PWM is physical behavior — tracked in
`docs/architecture/BENCH-VERIFICATION-DEFERRED.md`.

## Known gaps (documented, not silently fixed)

Both gaps below are **locked as current behavior** by `// #131 FINDING:` test
cases; fixing either is a behavior change requiring its own issue + operator
sign-off (behavior-preserving constraint during epic #116). Follow-up
hardening issues are pending operator filing; until then the full drafts live
in the appendix of the PR that introduced this registry
([PR #138](https://github.com/boulaajaj/Arduino-DiggerSetup/pull/138) — the
implementation PR for issue #131). Replace this pointer with the real issue
links once filed.

- **NaN pack voltage passes the plausibility band, asymmetrically by slot**:
  every comparison against NaN is false, so the [6, 13] V band never rejects
  it, and `worstPackVoltage()`'s ternary min-select makes the outcome
  slot-dependent — NaN in `telem[1]` propagates as "worst" and silently
  freezes the ladder (no trip, no confirm, no crash), while NaN in `telem[0]`
  loses the comparison and is masked by the healthy pack
  (`batteryOkConfirmed` can set while pack 0 reads garbage). Unreachable
  today: X.BUS voltages are built from uint16 registers and cannot be NaN.
  The temperature path is immune (its band check rejects NaN). Locked in
  `test_invariant_plausibility.cpp`.
- **Reverse-cap transient on gear upshift (≤ 10 ms)**: with full reverse
  joystick held (Mode 2/3), flipping CH4 Eco/Normal → Boost mixes the stale
  Eco-domain joystick clamp (−0.846 xSpeed) with the new Boost `gearScale`
  for up to one 100 Hz joystick-cache window — ~1077 µs average versus the
  1175 µs Boost reverse floor for up to 10 ms, inside the servo rails (the
  pass count depends on loop rate: a handful at the host test cadence, ~200
  at the real ~20 kHz loop).
  `rcCommand()` re-clamps every pass; only the cached joystick path has the
  gap. Locked in `test_invariant_gear_reverse_caps.cpp`.

## Rules for changing this file

An invariant here is **behavior law**: weakening a test expectation is a
behavior change requiring its own issue and operator sign-off
(`docs/TESTING.md`). New safety features add a row here in the same PR that
adds their invariant test.
