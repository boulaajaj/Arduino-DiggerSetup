---
paths:
  - "sketches/**"
---

# State Ownership Rules

- **One domain owns each piece of state; no other domain writes it.**
  Cross-domain communication happens through returned decision/status types
  (`SafetyDecision`, `BatteryStatus`), never by reaching into another
  module's variables. (Historical example of the sin: `[SAFETY]` writing
  `[ALERT]`'s `lowVoltLatched`.)
- **No namespace-scope mutable state in `domain/` code.** Module state lives
  in structs owned by the module and held by the application layer.
- **Dependencies are visible in the signature.** A function uses only what it
  receives — no hidden reads of globals behind a parameter list (historical
  example: `curvatureDrive()` taking `gearScale` but secretly reading the
  `currentGear` global).
- **Time is a parameter.** Domain code never calls `millis()`/`micros()`;
  the control cycle reads the clock once per pass and passes `now` down.
  This is what makes debounce/hysteresis/latch logic host-testable.
- **Observers are read-only.** Telemetry, dashboard, and debug output consume
  the immutable per-cycle `SystemSnapshot` — they never read domain internals
  and never write anything.
