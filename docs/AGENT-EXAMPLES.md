# Agent Examples — good/bad pairs for embedded work (#53)

Concrete good/bad task executions for AI agents in this repo. Every pair is
either a real event from this project's history or the project's canonical
checklist applied. The Working Agreement (CLAUDE.md) states the principles;
this file shows what following — and violating — them looks like.

## 1. ESC parameter change — think before coding (real: Max Reverse Force)

**Task:** "Pivot turns feel weak — is there an ESC setting for that?"

**Bad:** Agent looks up the GL10 manual, sees `Max Reverse Force` defaults to
50%, and answers "set it to 100% for stronger reverse". No analysis of what
the firmware actually commands. (Also bad in the other direction: recommending
the 50% default "for safety" — which would silently deliver −27.5% on the
reverse track during a ±55% pivot and collapse the pivot differential.)

**Good:** Agent runs the command-path checklist (CLAUDE.md "ESC / motor
configuration changes"): lists every path that drives each motor (RC throttle,
pivot/curvature, joystick, gear scaling, override mixer), computes the
per-direction min/max each path can produce (pivot at high gear commands one
track to −55%), verifies the proposed value clips none of them, and hands over
the value WITH the cross-impact analysis: "`Max Reverse Force = 100%` — the
factory 50% would halve the reverse track in pivot mode and break the
differential."

## 2. Reviewer claims a test expectation is wrong — verify, don't apply (real: PR #138)

**Task:** CodeRabbit posts a committable suggestion: "forced-Eco full-forward
truncates to 1824 µs, not 1825 — fix the assertion."

**Bad:** Agent applies the suggestion because the reviewer's float-truncation
reasoning sounds plausible. The green suite turns red — the "fix" changed a
behavior-law expectation to a wrong value.

**Good (what actually happened, 2026-07-06):** Agent treats the numeric claim
as unverified, compiles a probe against the real firmware, and finds
`0.65f * 500.0f` rounds UP to exactly `325.0f` → 1825 µs. It declines the
suggestion, replies on the thread with the probe evidence, and resolves the
thread. The suite stays green and the wrong claim is now documented for the
next reviewer.

## 3. A real defect found mid-PR — lock + issue, never an inline fix (real: PR #138)

**Task:** While writing invariant tests, the agent discovers NaN pack voltage
passes the [6, 13] V plausibility band, with slot-asymmetric consequences.

**Bad:** Agent "fixes" `worstPackVoltage()` in the same PR — it's two lines,
the tests are right there, and it's clearly a defect. Now a test-suite PR
silently changed safety-ladder behavior nobody discussed, reviewed as such,
or bench-verified.

**Good (what actually happened, 2026-07-06):** Agent locks the CURRENT
behavior in `// #131 FINDING:` test cases (so any accidental change trips the
suite), documents the gap in `docs/SAFETY.md` known-gaps with reachability
analysis (unreachable today — X.BUS voltages parse from uint16), and drafts a
dedicated follow-up issue describing what/why/how. The fix happens in its own
PR when the operator prioritizes it. Same pattern for the ≤10 ms gear-upshift
reverse transient found in the same session.

## 4. Vague symptom report — surgical diff, failing test first

**Task:** "Steering feels inverted when pivoting — fix the steering."

**Bad:** Agent rewrites `curvatureDrive()` "more clearly" while in there —
renames variables, reorders the blend logic, retunes `PIVOT_SPEED_CAP` because
60% "seemed low". The diff mixes a possible polarity fix with three unrelated
behavior changes, none test-locked, none discussed.

**Good:** Agent first reproduces the claim as a failing characterization
assertion (host suite drives the real mixer with a pivot-mode input and
asserts the expected track signs). If the assertion fails → genuine defect →
its own issue (what/why/how), then a minimal fix touching only the sign
error, with the new test proving it. If the assertion passes → the firmware
is correct; investigate wiring/transmitter config instead and say so. Either
way the diff carries exactly one intent.

## 5. Refactor task framing — hand the goal + the gate, not steps

**Task (operator to agent):** extract the `[TELEMETRY]` module during Phase D.

**Bad framing:** "First create `telemetry/XbusPoller.h`, then move lines
812–1040, then update the includes, then…" — a step list the agent follows
blindly; when step 3 doesn't compile it improvises, and nobody defined what
"done" means.

**Good framing:** "Goal: `[TELEMETRY]` lives in `src/telemetry/` per
ARCHITECTURE-TARGET.md §6, files ≤150 lines, dependency rules hold. Gate:
`wsl -e make -C tests run` green (unchanged expectations), local
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/dual_track_control`
clean, all CI workflows green, zero behavior change." The agent self-corrects
against the gate in a loop and stops when it passes — or reports precisely
which gate it cannot satisfy and why (per `.claude/rules/architecture.md`,
proposing an alternative instead of quietly violating a boundary).
