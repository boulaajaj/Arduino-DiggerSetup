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
  C Govern (CI fitness functions, done) → D Refactor (extract domains,
  **active**) → E Polish
- **Extracted domains so far** — the [battery ladder](battery-ladder.md)'s
  voltage-plausibility gate and its two staged state machines (Eco lock,
  cutoff + boot gate) live in `src/domain/battery/`; the
  [thermal derating](thermal-derating.md) hysteresis stage, hottest-sensor
  selection, and warn/eco/cut staging live in `src/domain/thermal/`; the
  [X.BUS telemetry](xbus-telemetry.md) register-decode and ×10 wire-encode
  scaling lives in `src/telemetry/` (observer layer); the operator-input
  expo curve and center-snap deadband live in `src/domain/operator_input/`;
  the curvature tank mix — the propulsion-path core — plus the gear policy
  and the RC/joystick command mixer live in `src/domain/drive/`; the drive
  allow/deny supervisor (SafetyDecision + FailsafeReason) and the
  ACTIVE/HOLD/CUT [output gate](output-gate.md) state machine live in
  `src/domain/safety/`. All pure logic — no Arduino includes, time passed
  as a parameter, hardware effects returned as actions for the sketch to
  execute; the sketch keeps thin delegates until the composition root lands
- **Protection first** — the [characterization and invariant
  suites](testing.md) were built BEFORE any code moves, so every refactor
  step is verified against locked behavior
- **Physical checks we cannot run** (no hardware during the program) are
  queued in
  [BENCH-VERIFICATION-DEFERRED](../architecture/BENCH-VERIFICATION-DEFERRED.md)
- Decisions land in the [DECISION-LOG](../DECISION-LOG.md) as they are made

Work is tracked on the GitHub project board — one issue per PR, one PR open
at a time ([.claude/rules/workflow](../../.claude/rules/workflow.md)).
