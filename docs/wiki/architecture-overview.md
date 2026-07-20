---
sources:
  - docs/architecture/ARCHITECTURE-TARGET.md
  - docs/architecture/FILE-MAP.md
---

# Architecture overview

The durable map of the firmware as it IS — independent of the migration
program that produced it ([architecture-remediation](architecture-remediation.md)
is the dated history of how we got here).

## Shape

The production sketch's `.ino` is a **composition root only**: it includes
the application layer and delegates `setup()`/`loop()`. Everything else
lives under `src/` in layers with a one-way dependency direction:

- **`domain/`** — pure logic (battery, thermal, operator_input, drive,
  safety). Host-testable by construction: no hardware includes, time and
  state passed as parameters.
- **`application/`** — orchestration and the firmware's mutable state; the
  former module sections live here. What one pass does:
  [application-loop](application-loop.md).
- **`ports/`** — hardware contracts as link-time seams;
  **`infrastructure/`** — the ONLY layer with hardware includes,
  implementing them: [ports-and-adapters](ports-and-adapters.md).
- **`telemetry/` + `alerts/`** — read-only observers of the
  SystemSnapshot, and the alert policy/pattern logic.
- **`config/`** — every tunable, per domain. Who owns which value:
  [configuration-authority](configuration-authority.md).

## Where the contracts live

- Canonical spec + dependency table:
  [ARCHITECTURE-TARGET](../architecture/ARCHITECTURE-TARGET.md) and
  [ADR-0001](../architecture/adr/0001-domain-oriented-firmware-architecture.md)
- Annotated file inventory: [FILE-MAP](../architecture/FILE-MAP.md)
- Mechanical enforcement: the architecture-fitness CI workflow
  (layer/size/naming rules) — see [testing](testing.md) for the gate list
- Human-facing narrative of all of the above:
  [ARCHITECTURE](../ARCHITECTURE.md) (#126)
- Which artifact wins for what: [authority-matrix](authority-matrix.md)
