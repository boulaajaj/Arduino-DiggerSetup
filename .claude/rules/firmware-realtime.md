---
paths:
  - "sketches/**"
---

# Real-Time Firmware Rules

- **No blocking calls in the control loop**: no `delay()`, no `pulseIn()`, no
  unbounded `while`. Non-blocking state machines only.
- **`constrain()` at every servo output boundary** — nothing reaches
  `writeMicroseconds()` outside 1000–2000 µs.
- **Float literals carry `f`** (`1.0f`, not `1.0`) — `double` is
  software-emulated on the RA4M1.
- **All tunables live in config** (today `[CONFIG]`, target `src/config/`).
  No magic numbers at point of use.
- **Per-channel failsafe**: any new input path gets an independent timeout
  that returns to neutral (SVC = 1500).
- **Watchdog refresh stays exactly once per loop pass**, after the control
  path is serviced — never add a second refresh point.
- **Permanent safety invariants** (never relax, never "temporarily" bypass):
  open-loop control (telemetry never feeds throttle), no control input over
  Wi-Fi, no `SoftwareSerial` for telemetry.

## Build & verification discipline (while hardware is unavailable)

- Compile locally before every push:
  `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/<sketch>`
- **No flashing, no bench claims.** Physical checks a change would need go
  into `docs/architecture/BENCH-VERIFICATION-DEFERRED.md`.
- Refactor PRs are behavior-preserving: no control-path behavior change,
  characterization tests green before and after.
