---
parent: hardware.md
sources:
  - docs/WIRING-GUIDE-V8.md
  - docs/INTERFACE-BOARD-NETLIST.md
  - sketches/dual_track_control/src/config/Pins.h
  - sketches/dual_track_control/src/infrastructure/radiolink/SbusReceiverAdapter.cpp
  - sketches/dual_track_control/src/application/OperatorInput.h
---

# Wiring and Arduino ports

This topic connects the receiver, joystick, ESCs, interface board, and
beeper to the Uno R4 WiFi interfaces they use. Each link records the
signal direction, the documented route, and its source. The receiver
reaches the Arduino through the interface board inverter. The ESC throttle
signals and their shared telemetry bus are separate paths.

Select **Wiring** in the explorer to trace these connections in either
direction. The [wiring guide](../WIRING-GUIDE-V8.md) records the pin and
UART assignments. The [netlist](../INTERFACE-BOARD-NETLIST.md) describes
the interface board connectors and shared nets.
