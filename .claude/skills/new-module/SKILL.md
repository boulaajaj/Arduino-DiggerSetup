---
name: new-module
description: Scaffolding conventions for adding a new firmware concept (domain logic, port, adapter, or application module) to the production sketch — use when creating any new .h/.cpp pair under sketches/dual_track_control/src/ so the file lands in the right layer with tests and docs from the start.
---

# New module — where code goes and what must come with it

## 1. Pick the layer (dependency rules: `.claude/rules/architecture.md`)

| The concept is… | It goes in… | May include |
| --- | --- | --- |
| Pure logic (math, policy, state machine) | `src/domain/<domain>/` | other `domain/` + `config/` ONLY — never Arduino.h, ports, infrastructure |
| A hardware contract (what, not how) | `src/ports/` (header-only) | plain types |
| Hardware/library implementation | `src/infrastructure/<vendor>/` | its port + the hardware libs (the ONLY layer with hardware includes) |
| Orchestration/wiring of the above | `src/application/` | domain + ports + telemetry + alerts + config |

If the concept doesn't fit without breaking a dependency rule, STOP and say
so — propose a new port or moved responsibility instead of quietly violating
the boundary.

## 2. File conventions

- One file = one concept; 150-line soft / 250-line hard limit; never
  `Part2`-style fragments.
- Full-word names in the project's ubiquitous language
  (`.claude/rules/naming.md`) — no `Utils`/`Manager`/`Helper` files.
- Tunables go in the matching `src/config/<Domain>Config.h`; adapter-owned
  tunables stay single-homed with their machine in `src/infrastructure/`.
- State ownership: one module owns each write
  (`.claude/rules/state-ownership.md`); any new mutable firmware global is
  added to `resetFirmwareState()` in
  `tests/characterization/FirmwareUnderTest.h`.

## 3. What ships in the same PR

- Host tests for the new concept under `tests/` (domain logic must be
  host-testable by construction — no hardware includes).
- Wiki + doc sync: `docs/architecture/FILE-MAP.md`, affected `docs/wiki/` notes,
  `docs/architecture/ARCHITECTURE-TARGET.md` if the layer map changed.
- Green gate: host suite + local `arduino-cli` compile + CI checks
  (`architecture-fitness` will mechanically verify the layer rules).
