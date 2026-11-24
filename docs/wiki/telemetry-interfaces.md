---
parent: telemetry-and-dashboard.md
sources:
  - sketches/dual_track_control/src/application/SystemSnapshot.h
  - sketches/dual_track_control/src/telemetry/JsonEncoder.cpp
  - sketches/dual_track_control/src/telemetry/CsvEncoder.cpp
  - sketches/dual_track_control/src/infrastructure/network/
---

# Telemetry interfaces

Follow the data from [X.BUS acquisition](xbus-telemetry.md) through the
firmware snapshot to its consumers. The snapshot captures operator inputs,
drive commands, mode, protection flags, and both ESC readings.

- **Dashboard JSON** is shared by the one-shot snapshot and the event
  stream. It exposes separate ESC validity and age, raw receiver flags,
  safety stages, commanded PWM, and compact measurements.
- **Serial CSV** is the local debug view. It includes the input channels
  and raw joystick readings, plus the output and ESC observations.
  Motor speed in CSV is electrical frequency; dashboard JSON converts it
  to the displayed RPM.
- [Wi-Fi dashboard](wifi-dashboard.md) covers delivery and buffering.
  [Dashboard display](dashboard-display.md) covers scale and freshness in
  the browser. [Tools](../../tools/README.md) indexes serial and replay tools.
