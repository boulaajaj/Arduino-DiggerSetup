---
sources:
  - sketches/dual_track_control/src/domain/thermal/
  - docs/SAFETY.md
---

# Thermal derating

Staged response to ESC and motor temperature — **monotone by design**: a
rising thermal stage never increases allowed propulsion. Stages run from a
warning, through a forced-Eco cap ([gear and reverse
caps](gear-and-reverse-caps.md)), to a full thermal cut via the
[output gate](output-gate.md). Each stage trips after a debounce and releases
with **hysteresis** — the machine holding a reduced stage anywhere inside its
hysteresis band is correct behavior, not a stuck state.

- Current stage temperatures, debounce, and release bands:
  [CLAUDE.md](../../CLAUDE.md) and [SAFETY](../SAFETY.md) (canonical)
- The monotonicity invariant and its tests: [SAFETY](../SAFETY.md) ·
  [testing](testing.md)
- Temperatures arrive via [X.BUS telemetry](xbus-telemetry.md); implausible
  spikes are rejected, and dropout freezes the active stage rather than
  releasing it. The hysteresis stage, hottest-sensor selection, and staging
  logic were extracted into domain code (`src/domain/thermal/`) as part of
  the [architecture remediation](architecture-remediation.md); the sketch
  keeps thin delegates

Deliberate asymmetry with the [battery ladder](battery-ladder.md): thermal
cut **auto-recovers** when temperature falls; battery cutoff latches until
power-cycle. Both intended, both test-locked.
