# Output gate

The final authority between the mixer and the servo pins. When any
closing condition fires — [battery cutoff](battery-ladder.md),
[thermal cut](thermal-derating.md) — commands ease to neutral over a bounded
ramp, then the servo signal **detaches entirely** (no pulses at all, which an
ESC treats as signal loss, not as "hold neutral").

- Current ramp timing and gate conditions: [CLAUDE.md](../../CLAUDE.md) and
  [SAFETY](../SAFETY.md) (canonical)
- Invariants: gate closed ⇒ neutral within the ramp window, then no pulses;
  cutoff dominates any drive request in any [override
  mode](override-modes.md) — tests in [testing](testing.md)
- Every value written to the pins is double-bounded (float domain and
  microsecond domain) — the "outputs always in range" invariant in
  [SAFETY](../SAFETY.md)
- Position in the chain: [control pipeline](control-pipeline.md)

The boot-side counterpart: on power-up the gate waits for battery
confirmation but **fails open** after a bounded window, so the machine
remains drivable with dead telemetry — a deliberate, documented trade-off
([SAFETY](../SAFETY.md)).
