---
sources:
  - sketches/dual_track_control/src/ports/
  - sketches/dual_track_control/src/infrastructure/
---

# Ports and adapters — the hardware boundary

How the firmware talks to hardware without the logic ever including a
hardware header. Inventory with per-file notes:
[FILE-MAP](../architecture/FILE-MAP.md).

## The seam

A **port** (`src/ports/`) is a small header of free functions and plain
types — a contract stating WHAT the application needs (write these servo
microseconds, read an RC frame, what time is it). An **adapter**
(`src/infrastructure/`) implements it against the real hardware and is the
ONLY layer allowed to include Arduino, Wi-Fi, servo, S.BUS, or X.BUS
headers. Binding happens at link time — no virtual dispatch on the control
path.

Adapters are grouped by vendor/technology: the Arduino core adapters
(PWM/servo, ADC joystick, piezo, clock, watchdog, serial console), the
Radiolink S.BUS receiver (which owns the second UART and translates the
vendor frame into the port's own `RcFrame` type at the boundary), the XC
X.BUS telemetry poller, and the network stack (Wi-Fi service, dashboard
server, SSE stream).

## Two details worth knowing

- **Vendor types never cross the boundary.** Ports speak their own types;
  the adapter translates at the edge (the S.BUS adapter is the precedent).
- **One seam runs in reverse**: the network layer never encodes telemetry —
  the application builds the JSON frame and the network adapter only ships
  bytes, so infrastructure cannot reach into application state
  ([telemetry-and-dashboard](telemetry-and-dashboard.md)).

Host tests exploit the same seam: the stub environment implements the
hardware side, and the suites compile the real firmware against it
([testing](testing.md)).
