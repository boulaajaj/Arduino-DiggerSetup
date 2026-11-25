---
parent: wifi-dashboard.md
sources:
  - dashboard/index.html
  - scripts/generate_web_page.py
  - sketches/dual_track_control/src/telemetry/JsonEncoder.cpp
---

# Dashboard display and status

The two track panels show per-ESC speed, temperatures, estimated power,
and pack voltage and charge. The center view estimates vehicle speed and
shows direction from PWM commands. The lower row shows throttle commands,
current, power and elapsed viewing time. Gear and authority badges reflect
the firmware snapshot.

The browser keeps stale per-ESC readings visible at reduced opacity. It
eases gauges toward incoming values, shows a prioritized safety banner, and
reports quiet or failed connections through a separate status indicator.
The page can reconnect on its own or through the reconnect button.

- [Telemetry interfaces](telemetry-interfaces.md) explains the wire fields.
- [Notification channels](notification-channels.md) explains how visual
  warnings relate to piezo and ESC sounds.
- [Dashboard rules](../../.claude/rules/dashboard.md) cover frame budget and
  replay checks.
