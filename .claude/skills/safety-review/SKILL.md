---
name: safety-review
description: Domain safety review for control-path changes — run BEFORE recommending any ESC/XC-Link parameter value, changing anything in src/domain/drive, src/domain/safety, src/application/MotorOutput or SafetyControl, or reviewing a PR that touches how throttle commands are produced. Walks the command-path checklist, the mechanical critical checks, and the permanent safety invariants.
---

# Safety review — control path & ESC parameters

This machine carries a child rider. A wrong cap, clip, or scaling silently
breaks a driving behavior. This skill is the procedure form of Working
Agreement principle 1 (think before coding).

## 1. Command-path checklist (mandatory before ANY ESC parameter suggestion)

1. List every command path that drives each motor: RC throttle, RC
   pivot/curvature, joystick equivalents, gear scaling (average-speed cap),
   reverse cap, override mixer (CH5 modes 1/2/3).
2. For each path, identify the per-direction min/max PWM it can produce
   (values: `src/config/DriveConfig.h` and `src/domain/drive/`).
3. Verify the proposed setting cannot clip, scale, or distort any of those
   commands in a way that breaks an intended behavior. Worked example: the
   GL10 default `Max Reverse Force = 50%` collapses the pivot differential —
   see `docs/GL10-PARAMETERS.md` for the per-parameter code-context analysis.
4. Report cross-impacts explicitly, naming the specific feature affected —
   never hand over a bare value.

## 2. Mechanical critical checks (on the diff)

- No blocking calls in the loop path: `delay()`, `pulseIn()`, unbounded `while`.
- Every value reaching `writeMicroseconds()` passes `constrain()` (1000–2000 µs).
- Non-finite handling: NaN/Inf/out-of-range inputs must still produce
  constrained outputs — the invariant suite locks this (`docs/TESTING.md`);
  check any new arithmetic on the output path against it.
- Any NEW input path has its own timeout returning to neutral (SVC = 1500).
- Float literals carry `f`; no raw tunables outside `src/config/` (production
  sketch) or the owning adapter.
- New mutable firmware global ⇒ added to `resetFirmwareState()` in
  `tests/characterization/FirmwareUnderTest.h`.

## 3. Permanent invariants (verify, never relax)

- Telemetry NEVER feeds throttle (X.BUS 0x10 is read-only monitoring).
- No control input over Wi-Fi — no path from network code to motor output.
- Watchdog refresh exactly once per loop pass, after the control path.
- Registry with test links: `docs/SAFETY.md`. If the diff moves any invariant
  boundary, the invariant suite must show it green before and after.

## 4. Verdict

Output a short written verdict: paths checked, per-direction extremes,
cross-impacts found (or "none"), invariants confirmed. A suspected genuine
defect is NOT fixed inline — severity-triage it per
`.claude/rules/behavior-preservation.md` (own issue BEFORE any fix).
