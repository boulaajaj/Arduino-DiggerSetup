# Hardware

Arduino **UNO R4 WiFi** (RA4M1 + ESP32-S3 Wi-Fi coprocessor) drives two
XC GL10 FOC ESCs paired with GL540L sensored brushless motors, powered by two
3S LiPo packs. Inputs: Radiolink RC system over [S.BUS](sbus-input.md) and a
hall-effect dual-axis joystick on the ADC. Canonical component table and pin
assignments: [CLAUDE.md](../../CLAUDE.md).

- [WIRING-GUIDE-V8](../WIRING-GUIDE-V8.md) — canonical wiring reference
- [INTERFACE-BOARD-PERFBOARD](../INTERFACE-BOARD-PERFBOARD.md) — soldered
  interface board (X.BUS merge + S.BUS inverter); design history in
  [INTERFACE-BOARD](../INTERFACE-BOARD.md) and netlist in
  [INTERFACE-BOARD-NETLIST](../INTERFACE-BOARD-NETLIST.md)
- [GL10-PARAMETERS](../GL10-PARAMETERS.md) — every ESC parameter analyzed
  against what the firmware actually commands (read before ANY ESC change)
- [GL10-OPERATION](../GL10-OPERATION.md) — calibration, factory reset, LED
  and beep codes; quick card: [GL10-QUICK-REFERENCE](../GL10-QUICK-REFERENCE.md)

The GL10's internal FOC owns motor smoothing — the Arduino sends open-loop
servo PWM commands only (see [control pipeline](control-pipeline.md)). The
hardware history (older sensored ESCs, the board migration) is recorded in
[CLAUDE.md](../../CLAUDE.md) and the [DECISION-LOG](../DECISION-LOG.md).
