# Testing

Host-side suites that compile the **real firmware** (`dual_track_control.ino`) against
stub Arduino headers — no hardware needed. Canonical guide, commands, and the
commit gate: [TESTING](../TESTING.md).

- **Characterization suite** (`tests/characterization/`) — locks current
  behavior; the parity gate for every firmware-touching PR
- **Invariant suite** (`tests/invariants/`) — property tests + fault
  injection for the [safety system](safety-system.md); each maps to a row in
  [SAFETY](../SAFETY.md)
- **Commit gate** — `.githooks/pre-commit` blocks red suites and firmware
  changes without test changes
- **CI** — the same suites run in the `unit-tests` workflow alongside
  compile, lint, static-analysis, and structure checks; the
  `architecture-fitness` workflow additionally enforces the
  [target architecture's](architecture-remediation.md) dependency, file-size,
  and naming rules with a self-testing checker

**Test expectations are behavior law**: changing an expected value is a
behavior change requiring its own issue and operator sign-off — the core of
the [governance covenant](agent-governance.md). What cannot be tested on the
host (physical behavior) is deferred to a bench pass:
[BENCH-VERIFICATION-DEFERRED](../architecture/BENCH-VERIFICATION-DEFERRED.md).
Dashboard UI has its own local test path (Playwright screenshots) per
[.claude/rules/dashboard](../../.claude/rules/dashboard.md).
