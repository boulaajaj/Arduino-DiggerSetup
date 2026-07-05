# Architecture Rules (canonical spec: docs/architecture/ARCHITECTURE-TARGET.md)

These rules govern all production-firmware code. The architecture document is
canonical; if this file and the doc disagree, the doc wins — fix this file.

## Layers and dependency direction (one-way, no exceptions)

- `.ino` → `application/` only. `application/` → `domain/` + `ports/` +
  `telemetry/` + `alerts/` + `config/`.
- `domain/` may include other `domain/` headers and `config/` ONLY. Never
  `Arduino.h`, `WiFiS3.h`, `Servo.h`, SBUS, X.BUS/GL10 code, `ports/`,
  `infrastructure/`, or `application/`.
- `infrastructure/` implements `ports/`; it never includes `domain/` internals
  or `application/`.
- `src/generated/` is machine-written. Never edit it by hand — edit the source
  (e.g. `dashboard/index.html`) and regenerate.

## File policy

- One file = one concept. Soft limit 150 lines, hard limit 250 for
  human-written source. Never split a file into `Part2`-style fragments to
  duck the limit — split by responsibility or ask.
- Banned file names: `Utils`, `Helpers`, `Helper`, `Manager`, `Common`,
  `Misc`, `Stuff`, `Data`. A `helpers/` folder inside a domain is acceptable
  only when every file in it names one precise responsibility.
- Names use the project's ubiquitous language (OutputGate, SafetyDecision,
  FailsafeReason, BatteryLadder, ThermalStage — see the glossary in the
  architecture doc), not mechanism words (PwmManager, ErrorCode).

## When you cannot comply

If a change appears to require breaking a dependency rule, stop and say so —
propose the alternative (new port, moved responsibility) instead of quietly
violating the boundary.
