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

## 2026-05-23 — Daughter board design started

- Purpose: replace breadboard X.BUS merge + S.BUS inverter with soldered stripboard.
- Build plan: `docs/DAUGHTER-BOARD.md`.
- Design uses: 2× 1N5819 Schottky (cathode→ESC, anode→bus), 4.7kΩ pull-up, 1kΩ TX series resistor (on main board), 2N3904 S.BUS inverter with 2× 10kΩ.
- Diode polarity in `docs/WIRING-GUIDE.md` §1 is documented incorrectly (says anode→ESC, should be anode→bus). Daughter board doc corrects this.
- D0 conflict: X.BUS and S.BUS both want Serial1 RX. Only one can be active at a time until a dual-UART board is used.
