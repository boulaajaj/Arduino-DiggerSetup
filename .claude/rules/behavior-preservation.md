# Behavior-Preservation Covenant (highest-priority rule)

**Operator's words (2026-07-06):** "organizing, refactoring and improving the
code should never ever, ever cause change of controls or change of behavior
unless we explicitly discuss it. … We're not messing with the timing, the
behavior, the parameters. Nothing should be touched. Everything that was
handling inputs and outputs should remain exactly the same. If we upload the
firmware again, I would like the machine to perform exactly like it used to
perform before."

The firmware is field-tuned on a real machine with a child rider. The entire
value of any reorganization is a better-organized firmware that drives
IDENTICALLY. (Born as epic #116's law; permanent policy since the epic closed,
2026-07-20, #203.)

## The rule

1. **Organization PRs change ZERO observable behavior.** No timing changes, no
   parameter/constant value changes, no input or output handling changes, no
   control-flow reordering that alters outputs, no "while I'm here"
   improvements. Moving code, renaming, splitting files — nothing else.
2. **Flash-equivalence is the acceptance bar.** Firmware flashed after a
   refactor must make the machine perform exactly as before. Enforced by:
   characterization + invariant suites green (they compile the REAL firmware),
   local `arduino-cli` compile before every push, and one bench
   re-verification pass when hardware returns
   (`docs/architecture/BENCH-VERIFICATION-DEFERRED.md`).
3. **Test expectations are behavior law.** Changing an expected value —
   including float32 rounding points (forced-Eco = 1825 µs because
   `0.65f*500.0f` rounds UP to exactly `325.0f`), exact-zero compares, and
   hysteresis holds (Eco correctly HOLDS at 85 °C inside the 90/80 band) —
   requires its own issue + operator sign-off BEFORE any code.

## When something looks wrong: severity triage, not dogma

The covenant forbids SILENT changes, not fixes (operator: "be pragmatic — not
fixing something blatantly crazy is NOT the goal").

- **Blatantly dangerous** (rider harm, runaway, ignored cutoff): raise to the
  operator IMMEDIATELY and fix promptly in its OWN dedicated fix PR — safety
  outranks the covenant. Never park a genuine hazard behind
  "behavior-preserving".
- **Real but bounded/latent** (e.g. the NaN plausibility gap, the ≤10 ms
  gear-upshift reverse transient — see `docs/SAFETY.md` known gaps): lock
  current behavior in a `// FINDING` test, file the issue, fix when the
  operator prioritizes.
- **Odd-looking but intended** (hysteresis holds, fail-open boot gate,
  average-based caps): leave alone — it is law. `docs/SAFETY.md` and
  `docs/DECISION-LOG.md` record which is which.

**The constant in all three:** a behavior fix must be visible, discussed,
deliberate, and have its OWN GitHub issue — created BEFORE the fix —
describing WHAT changes, WHY, and HOW. A behavior fix can NEVER be included
inside any other PR, and NEVER applied as a PR-review correction — no
exceptions. Urgency changes how FAST the issue + dedicated PR happen (a
dangerous find = same day), never the path.

## Review-comment handling

Never apply a reviewer's (human or bot) committable suggestion that changes
behavior, constants, defaults, feature flags, or test expectations. Verify
numeric claims empirically against the real code before agreeing (precedent:
a bot's "1824 µs" claim was refuted by an on-firmware probe — the suggestion
would have turned the green suite red). If a review comment surfaces a
genuine behavior defect: reply "acknowledged — needs its own issue", get the
issue created, resolve the thread pointing at it, and fix in a dedicated PR.
