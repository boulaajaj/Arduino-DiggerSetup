---
name: field-tune
description: Field-tuning procedure for drive-feel constants (expo, deadband, caps, blend, alarm thresholds) — use when the operator asks to adjust how the machine drives or alarms, or after a field test produces tuning feedback. Tuning IS a behavior change and follows the covenant.
---

# Field tune — changing how the machine feels

## 1. Tuning is a behavior change

Every constant change alters observable behavior, so the covenant applies in
full (`.claude/rules/behavior-preservation.md`): it needs its OWN GitHub
issue BEFORE the change, describing WHAT changes, WHY, and HOW — never mixed
into a refactor or applied as a review correction. Characterization
expectations that encode the old value change in the same PR, explicitly
listed, with operator sign-off.

## 2. Where the tunables live

All operator-tunable constants are in `src/config/` per-domain headers —
Input (expo/deadband), Drive (caps, pivot, blend, taper), Battery/Thermal
(protection ladder), Alert (beeper thresholds), Safety. Smoothness is the
top project priority (`docs/MISSION.md`): never skip any throttle range, and
remember the GL10's own Acceleration/Drag settings own command smoothing —
do not reintroduce Arduino-side filtering.

## 3. Procedure per iteration

1. State the hypothesis: symptom → constant(s) → expected feel change.
2. Run the `safety-review` skill if the change touches caps, reverse,
   pivot, or any per-direction limit.
3. Host suite: update the affected expectations (they are law — the diff
   must show exactly which numbers moved), everything else stays green. The
   PR body cites the authorizing issue and the operator's sign-off — the
   evidence the tests-reviewer requires for any expectation change.
4. Flash via the `flash-and-log` skill (each iteration gets its log row).
5. Field-validate with the operator (Jason on RC, Malaki on joystick as
   relevant) — record what was felt, not just what was measured.
6. Log the outcome in `docs/DECISION-LOG.md`: date, constant old→new, why,
   result (kept / reverted / needs another pass).

## 4. Escalation

If a desired feel cannot be reached with existing constants, do NOT invent a
new control-path feature inline — file an issue for the mechanism change and
stop.
