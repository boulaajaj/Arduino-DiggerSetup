# Battery ladder

Staged response to falling pack voltage, always keyed to the **worse** of the
two packs: first a forced-Eco lock (machine keeps driving, slower), then an
audible low-voltage alarm ([alerts and alarms](alerts-and-alarms.md)), and at
the bottom a **latched cutoff** — the [output gate](output-gate.md) closes
and stays closed until power-cycle, even if voltage recovers. The alarm keeps
chirping through the cut, so a cutoff is never silent.

- Current thresholds, debounce times, and the boot-gate behavior:
  [CLAUDE.md](../../CLAUDE.md) and [SAFETY](../SAFETY.md) (canonical)
- Invariants and fault-injection tests (cutoff dominates every drive request;
  boot with a low pack never emits one non-neutral pulse):
  [SAFETY](../SAFETY.md) · [testing](testing.md)
- Voltage arrives via [X.BUS telemetry](xbus-telemetry.md); implausible
  readings are rejected by plausibility gates, and telemetry dropout
  deliberately freezes the ladder ([SAFETY](../SAFETY.md) "Known gaps"
  documents the edge cases). The worst-of-two plausibility gate and the
  two-stage ladder state machines are the first extracted domain of the
  [architecture remediation](architecture-remediation.md)
  (`src/domain/battery/`); the sketch keeps thin delegates
- Operator-facing behavior and what the beeps mean:
  [OPERATOR-GUIDE](../../OPERATOR-GUIDE.md)

Deliberate asymmetry: battery cutoff **latches** until power-cycle, while
[thermal cut](thermal-derating.md) auto-recovers — both are intended and
test-locked.
