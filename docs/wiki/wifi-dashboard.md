# Wi-Fi dashboard

The UNO R4 WiFi hosts its own access point and serves a single-page
dashboard streaming live telemetry over Server-Sent Events. **Monitoring
only, permanently** — there is no control input over Wi-Fi, and no code path
from the network to motor output
([telemetry and dashboard](telemetry-and-dashboard.md)).

- Current AP name, addresses, stream cadence, and endpoints:
  [CLAUDE.md](../../CLAUDE.md) (canonical)
- Source of truth for the page is `dashboard/index.html`;
  `sketches/rc_test/web_page.h` is generated from it — never hand-edited
- Code home: the serving machine (AP bring-up, routing, incremental page
  transfer, SSE) is `src/infrastructure/network/` behind
  `src/ports/DashboardServicePort.h`; the JSON flows the other way through
  `src/ports/TelemetryFrameSource.h` — the application encodes, the network
  layer ships bytes ([architecture
  remediation](architecture-remediation.md), #181)
- Rules (SSE frame budget, wire-format keys, local Playwright screenshot
  testing BEFORE flashing):
  [.claude/rules/dashboard](../../.claude/rules/dashboard.md)
- History: a blocking page send once froze the control loop and became the
  motivating incident for the loop watchdog
  ([safety system](safety-system.md), [SAFETY](../SAFETY.md)); serving is
  incremental and bounded since — details in the
  [DECISION-LOG](../DECISION-LOG.md)
- Open latency investigation (hardware-gated):
  [HANDOFF-V8-V9-WIFI-LATENCY-AND-OFFLOAD](../HANDOFF-V8-V9-WIFI-LATENCY-AND-OFFLOAD.md)
