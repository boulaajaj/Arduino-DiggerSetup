# Sketches

One production firmware, plus throwaway bench/bring-up tools kept for
reference. Only the production sketch follows the architecture rules
(`.claude/rules/architecture.md`); bench sketches are exempt by design.

## Production

| Sketch | What it is |
| --- | --- |
| `dual_track_control/` | **The shipped firmware** (V7.x lineage, renamed in #118): S.BUS + joystick input, curvature tank mix, gear caps, safety ladders, X.BUS telemetry, Wi-Fi dashboard. Everything under its `src/` tree. |

## Bench / bring-up tools

| Sketch | Purpose |
| --- | --- |
| `beeper_test/` | D8 piezo + SWD-horn bench test (#63; horn logic later moved into the firmware) |
| `hardware_diagnostic/` | RC pulse + ADC wiring diagnostic (pulseIn-based — bench only) |
| `pin_test/` | `attachInterrupt()` support test on D2/D3/D4/D7 |
| `sbus_test/` | LEGACY (Nano R4 era, S.BUS on D0) — superseded by `sbus_d12_test` |
| `sbus_d12_test/` | S.BUS-on-D12 (SCI0) bring-up for the UNO R4 WiFi |
| `telemetry_check/` | Read-only X.BUS telemetry bench tool (#36, 0x50 framing) |
| `telemetry_decode/` | Early X.BUS decode experiment (ADUM1201 @ 19200 + joystick direct drive) |
| `usb_test/` | USB CDC serial output sanity check |
