# Workflow Rules (agent-enforced)

- **Issue first, always.** No code changes, commits, or uploads without an
  open GitHub issue the work attaches to. No issue → conversation only.
- **Starting work = draft PR.** Branch `agent/<role-tag>/<description>`, one
  PR ↔ one issue, body contains `Closes #N` + summary + role tag + test plan.
  Active work must show In Progress on Project #1.
- **Behavior-preserving refactors only** during epic #116 Phase D: no logic
  changes mixed into structural PRs; characterization tests green before and
  after; local `arduino-cli` compile before every push.
- **Decisions get logged** in `docs/DECISION-LOG.md` as they are made (terse:
  date, what, why). Physical checks we cannot run (no hardware) go to
  `docs/architecture/BENCH-VERIFICATION-DEFERRED.md`.
- **Every firmware flash** (when hardware returns) gets a row in
  `docs/FIRMWARE-UPLOAD-LOG.md` immediately — an unlogged flash is treated as
  not done.
- **Review threads**: push the fix first, reply with the commit SHA and
  reasoning, then resolve. Never resolve before pushing. Engage the operator
  before applying review-bot suggestions to safety-relevant code.
