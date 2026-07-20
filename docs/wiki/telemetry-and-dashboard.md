---
sources:
  - sketches/dual_track_control/src/infrastructure/xc/
  - dashboard/index.html
---

# Telemetry and dashboard

Read-only observation path: the ESCs report voltage, current, RPM, and
temperatures over [X.BUS telemetry](xbus-telemetry.md); the firmware smooths
and streams them to the [Wi-Fi dashboard](wifi-dashboard.md).

**Monitoring only — permanent decision.** There is no path from Wi-Fi (or any
telemetry) back into motor output; the [safety system](safety-system.md)
consumes telemetry for derating/cutoff decisions, but nothing network-side
can ever command a track. Canonical detail and current cadence/keys:
[CLAUDE.md](../../CLAUDE.md).

- [XBUS-PROTOCOL](../XBUS-PROTOCOL.md) — wire protocol reference
- Dashboard source lives at `dashboard/index.html`, embedded into firmware as
  `sketches/dual_track_control/web_page.h` (generated — edit the source, regenerate)
- Dashboard rules (frame budget, UI testing before flashing):
  [.claude/rules/dashboard](../../.claude/rules/dashboard.md)
- Telemetry feeds the [battery ladder](battery-ladder.md) and
  [thermal derating](thermal-derating.md); when telemetry drops out, those
  ladders deliberately freeze ([SAFETY](../SAFETY.md))
- Open latency work is hardware-gated:
  [HANDOFF-V8-V9-WIFI-LATENCY-AND-OFFLOAD](../HANDOFF-V8-V9-WIFI-LATENCY-AND-OFFLOAD.md)
