# Decision Log

Technical decisions, test results, and confirmed hardware facts.
Updated by session hooks — only technical content, no personal info.

---

## 2026-04-14 — X.BUS protocol confirmed

- X.BUS is single-wire half-duplex, 115200 8N1, non-inverted (idle HIGH).
- Protocol is master-polled: slaves never transmit unsolicited.
- Manufacturer: Shenzhen XC-ESC Technology Co., Ltd (not Spektrum, not JR).
- Full protocol translated and documented in `docs/XBUS-PROTOCOL.md`.

## 2026-05-23 — X.BUS telemetry confirmed working on breadboard

- Both GL10 ESCs respond to polling with telemetry via X.BUS.
- Breadboard uses Schottky diode merge + pull-up resistor on shared bus.
- Arduino polls on D1 (TX), receives on D0 (RX), same physical bus node.
- Exact diode orientation and pull-up value on breadboard: **not yet recorded — needs bench verification**.
- Test sketch: `sketches/xbus_master/xbus_master.ino`.

## 2026-05-23 — Interface board design

- Purpose: replace breadboard X.BUS merge + S.BUS inverter with a soldered plug-and-play board.
- Build plan: `docs/INTERFACE-BOARD.md`.
- X.BUS merge: no diodes needed (master-polled, no bus contention). Both ESC yellows connect directly to bus node.
- Components: 4.7kΩ pull-up, 1kΩ TX series resistor (on board between J3 and bus), 2N3904 S.BUS inverter with 2× 10kΩ.
- Motor control via PWM (D9/D10 → ESCs direct), X.BUS for telemetry only (RPM, current, voltage, temp).
- X.BUS telemetry confirmed working on breadboard with both GL10 ESCs.
- Six servo connectors: 2× ESC telemetry IN, 1× TX to Arduino, 1× RX to Arduino, 1× S.BUS IN from receiver, 1× inverted S.BUS OUT to Arduino.
- D0 conflict: X.BUS and S.BUS both want Serial1 RX. Only one can be connected at a time until a dual-UART board is used.
