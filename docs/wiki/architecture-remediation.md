# Architecture remediation (epic #116)

Active program: reorganize the single-sketch firmware into a domain-oriented
structure — **without changing one observable behavior**
([governance](agent-governance.md)). Firmware flashed after any refactor must
drive exactly as before.

- **Target architecture** —
  [ARCHITECTURE-TARGET](../architecture/ARCHITECTURE-TARGET.md): layers
  (`application` / `domain` / `ports` / `infrastructure`), dependency rules,
  migration order
- **Founding ADR** —
  [0001-domain-oriented-firmware-architecture](../architecture/adr/0001-domain-oriented-firmware-architecture.md)
- **Phases** — A Define → B Protect ([testing](testing.md) suites, done) →
  C Govern (CI fitness functions) → D Refactor (extract domains) → E Polish
- **Protection first** — the [characterization and invariant
  suites](testing.md) were built BEFORE any code moves, so every refactor
  step is verified against locked behavior
- **Physical checks we cannot run** (no hardware during the program) are
  queued in
  [BENCH-VERIFICATION-DEFERRED](../architecture/BENCH-VERIFICATION-DEFERRED.md)
- Decisions land in the [DECISION-LOG](../DECISION-LOG.md) as they are made

Work is tracked on the GitHub project board — one issue per PR, one PR open
at a time ([.claude/rules/workflow](../../.claude/rules/workflow.md)).
