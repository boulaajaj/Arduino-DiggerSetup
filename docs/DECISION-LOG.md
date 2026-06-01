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
- Breadboard used Schottky diode merge + 4.7kΩ pull-up. Superseded by the interface board design (no diodes needed for master-polled X.BUS).
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

## 2026-05-24 — UNO R4 WiFi migration: pin map, UART, power, 9-pin cable

- Migrating from Nano R4 to UNO R4 WiFi with Sensor Shield V5.0.
- **UART discovery:** On UNO R4 WiFi, SCI0 is on D11(TX)/D12(RX), NOT A4/A5 (those are I2C-only). Serial2 is reserved for WiFi ESP32-S3. Confirmed via ArduinoCore-renesas variant.cpp: D11=P411 (SCI0 TX), D12=P410 (SCI0 RX).
- **UART plan:** Serial1 (SCI2, D0/D1) for X.BUS 115200. sbusUart (SCI0, D12 RX) for S.BUS 100000 8E2. Both hardware UARTs, fully reliable.
- **Power:** GL10 BEC set to 7.4V → VIN. ISL854102FRZ buck converter → 5V on all rails. USB+VIN coexist (Schottky diodes). SEL jumper IN.
- **9-pin cable:** 5 signal (PWM L/R, XBUS TX/RX, SBUS) + 3 power (VIN, GND, 5V) + 1 spare. Joystick direct to shield A0/A1.
- **Full wiring spec:** `docs/WIRING-GUIDE-V8.md`.

## 2026-05-24 — GL10 throttle range calibration completed

- Both GL10 ESCs calibrated to match the same transmitter stick range.
- GL10 calibration sequence: full REVERSE on power-up → full FORWARD → NEUTRAL (not forward-first like generic ESCs).
- Confirmed success: red+green LEDs flash 4× with "so-mi-do" melody.
- Official procedure from GL10 User Manual Section 5 (`docs/GL10-Manual.pdf`).
- App name is TXC-Link (not XC-Link). Default Bluetooth password: 1234.
- Quick reference saved: `docs/GL10-QUICK-REFERENCE.md`.

## 2026-05-31 — S.BUS confirmed working on UNO R4 WiFi hardware

- `rc_test` V7.6 compiles for `arduino:renesas_uno:unor4wifi` (22% flash, 25% RAM) and runs on the board (COM7).
- S.BUS verified live on `sbusUart` SCI0 / D12 RX through the 2N3904 inverter. Frames valid, FS=0 / Lost=0.
- Inverter wiring fix: D12 (output) taps the **collector**; S.BUS-in from the receiver goes through 10 kΩ to the **base**; **emitter** to GND. Initial build had collector/base roles swapped (no signal); corrected.
- Verified end-to-end: throttle full range, steering, curvatureDrive differential on D9/D10, and Eco reverse cap producing 1375 µs (matches 1500 − (0.625 × 0.40 × 500) = 1375 µs, neutral offset included).
- X.BUS telemetry NOT active — `rc_test` does not poll it. Telemetry restore tracked in #36; Wi-Fi dashboard in #45.
