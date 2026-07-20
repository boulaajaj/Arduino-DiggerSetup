---
sources: none
---

# Home — Excavator Track Controller

Tank-style dual-track controller for a ride-on excavator: Arduino UNO R4
WiFi, two FOC ESCs, dual input (RC transmitter + rider joystick), Wi-Fi
telemetry dashboard. Project memory: [CLAUDE.md](../../CLAUDE.md) · full
spec: [PROJECT-PLAN.md](../../PROJECT-PLAN.md) · design philosophy:
[MISSION](../MISSION.md) (smoothness above all).

Which artifact wins for what: [authority-matrix](authority-matrix.md).

## Domains

- [Hardware](hardware.md) — board, ESCs, motors, batteries, wiring
- [Control pipeline](control-pipeline.md) — stick to track PWM, end to end
- [Safety system](safety-system.md) — invariants, ladders, gates, failsafes
- [Telemetry and dashboard](telemetry-and-dashboard.md) — X.BUS in, Wi-Fi out
- [Testing](testing.md) — host suites that compile the real firmware
- [Agent governance](agent-governance.md) — Karpathy method, covenant, bots
- [Architecture](architecture-overview.md) — layers, seams, and the loop
  ([application-loop](application-loop.md) ·
  [ports-and-adapters](ports-and-adapters.md) ·
  [configuration-authority](configuration-authority.md))
- [Architecture remediation](architecture-remediation.md) — epic #116 program
  (dated migration history)

## People and operations

- [OPERATOR-GUIDE](../../OPERATOR-GUIDE.md) — for the RC supervisor and rider
- [FIRMWARE-UPLOAD-LOG](../FIRMWARE-UPLOAD-LOG.md) — the device's flight log
- [DECISION-LOG](../DECISION-LOG.md) — every technical decision, dated
