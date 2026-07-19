# Workflow Rules (agent-enforced)

- **Issue first, always.** No code changes, commits, or uploads without an
  open GitHub issue the work attaches to. No issue → conversation only.
- **Starting work = draft PR.** Branch `agent/<role-tag>/<description>`, one
  PR ↔ one issue, body contains `Closes #N` + summary + role tag + test plan.
  Active work must show In Progress on Project #1.
- **One open PR at a time** (operator rule, 2026-07-05). Do not start the next
  work item until the current PR is merged or closed — no parallel PRs, no
  merge-order dependencies to keep track of.
- **Behavior-preserving refactors only** during epic #116 Phase D: no logic
  changes mixed into structural PRs; characterization tests green before and
  after; local `arduino-cli` compile before every push.
- **Commit test gate (#47)**: `.githooks/pre-commit` hard-blocks commits when
  the host suite (`make -C tests run`) is red or when `sketches/dual_track_control/`
  changes without a `tests/` change. Agents never use `--no-verify` — fix the
  tests instead. Details: `docs/TESTING.md`.
- **Decisions get logged** in `docs/DECISION-LOG.md` as they are made (terse:
  date, what, why). Physical checks we cannot run (no hardware) go to
  `docs/architecture/BENCH-VERIFICATION-DEFERRED.md`.
- **Docs travel with code, same PR.** When production code changes, verify
  docs/wiki notes, architecture docs (incl. docs/architecture/FILE-MAP.md),
  docs/TESTING.md, docs/SAFETY.md and OPERATOR-GUIDE.md still match — the
  documentation-sync hook (`scripts/documentation_sync_hook.py`, #156)
  prompts this once per session; the prompt is a floor, not the ceiling.
- **Every firmware flash** (when hardware returns) gets a row in
  `docs/FIRMWARE-UPLOAD-LOG.md` immediately — an unlogged flash is treated as
  not done.
- **Review threads**: push the fix first, reply with the commit SHA and
  reasoning, then resolve. Never resolve before pushing. Engage the operator
  before applying review-bot suggestions to safety-relevant code.
