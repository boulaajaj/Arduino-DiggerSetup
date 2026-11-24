---
parent: output-gate.md
sources:
  - docs/SAFETY.md
  - sketches/dual_track_control/src/application/FirmwareApp.cpp
  - sketches/dual_track_control/src/domain/safety/SafetySupervisor.cpp
  - tests/invariants/
---

# Drive safeguards

Read this topic when tracing a refused drive command. The receiver's
failsafe flag and the local stale-frame timer determine whether RC is valid.
Invalid RC blocks propulsion in every authority mode. The receiver's lost
frame flag is reported separately for diagnostics.

The [output gate](output-gate.md) applies the supervisor decision after
input shaping and gear selection. The watchdog is serviced after the
control path and before telemetry, Wi-Fi, and alerts. Its job is to reset
the controller if a loop stalls while the ESCs are still receiving PWM.

- [Safety invariants](../SAFETY.md#the-invariants) links each guarantee to
  an executable test, including bounds, dropout, cutoff, and gear limits.
- [Known gaps](../SAFETY.md#known-gaps-documented-not-silently-fixed) describes the pack plausibility case
  and the transient associated with cached joystick input during upshift.
- [Bench verification](../architecture/BENCH-VERIFICATION-DEFERRED.md)
  tracks what the host suite cannot prove about the physical machine.
- Related topics: [S.BUS input](sbus-input.md),
  [gear and reverse caps](gear-and-reverse-caps.md), [testing](testing.md).
