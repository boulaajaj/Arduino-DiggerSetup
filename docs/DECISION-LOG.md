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

## 2026-06-01 — Telemetry logging architecture + board-upgrade analysis (V8)

Research for the WiFi telemetry dashboard + black-box logging. Findings:

- **SD card cannot coexist with S.BUS on UNO R4 WiFi.** Hardware SPI is on
  D11 (MOSI) / D12 (MISO) / D13 (SCK); `sbusUart` (SCI0) is on D11 (TX) /
  D12 (RX). SD-over-SPI lands on the exact S.BUS pins. No third hardware UART
  to relocate S.BUS to (Serial1/D0-D1 reserved for X.BUS). So onboard SD is
  not viable on this board without evicting S.BUS. (Clock/RTC is unaffected —
  I2C on SDA/SCL, or NTP over WiFi; WiFi is unaffected — ESP32-S3 coprocessor.)
- **Stock UNO R4 WiFi does NOT offload the WebSocket/server.** The sketch
  runs on the RA4M1; the ESP32-S3 is a `WiFiS3` network coprocessor only.
  WebSocket/server libs (e.g. UnoR4WiFi_WebServer / mWebSockets) execute on
  the RA4M1 — same core as the control loop — and CAN block it. The ESP32-S3's
  8 MB flash is NOT sketch-accessible in stock mode. (Corrects the assumption
  in issue #45 that serving a page can't affect control-loop timing.)
- **Agreed logging data model:** telemetry frames tagged with `millis()`
  (NOT `micros()` — micros wraps at ~71 min; millis at ~49.7 days) + a
  monotonic integer sequence number for dedup/backfill. Client takes one
  (arduino-ts, wall-clock) anchor per connection. X.BUS polling must be a
  non-blocking state machine, not a blocking request/wait.
- **Path A — custom ESP32-S3 firmware (split-brain):** runs WebSocket +
  LittleFS ring buffer on the 8 MB flash + dedup/backfill on the ESP32;
  RA4M1 fire-and-forget streams frames over the internal link. Hardware-safe
  and fully recoverable (ESP32-S3 mask-ROM bootloader can't be erased; force
  download mode via GND+Download pins on the 6-pin header, reflash with
  espflash; Arduino publishes the original bridge firmware). RA4M1 untouched
  by ESP32 flashing. CATCH: the ESP32-S3 is also the USB-serial bridge for
  programming the RA4M1 — overwriting it breaks sketch upload + Serial Monitor
  until the bridge firmware is restored. Recurring dev-loop friction.
- **Path B — Giga R1 WiFi:** STM32H747 dual-core (M7 480 / M4 240), 1 MB RAM
  + 8 MB SDRAM, 2 MB MCU flash + 16 MB QSPI NOR (sketch-accessible), USB-A
  host for thumb-drive mass storage, 4 UARTs, onboard WiFi/BT. Programmed
  directly over USB-C (no bridge conflict). True offload of control vs I/O.
  COST: 3.3 V logic only — 5 V joystick needs level-shifting/divider, S.BUS
  inverter output swing must be re-verified; bigger enclosure + re-mount.
- **Capacity:** compact binary frame ~24-32 B → 8 MB ≈ ~250-300k frames
  ≈ ~15 h at 5 Hz.
- **Lean:** Giga for the full black-box + analytics goal (clean dual-core
  offload, more + natively-accessible storage, removable media, no flashing
  friction). Path A is the no-new-hardware fallback. Decision pending.

## 2026-06-01 — DECISION: stay on UNO R4 WiFi, stock config (Option C)

- Operator chose **no new hardware / no mechanical change**. Giga (Option B)
  declined. Onboard black-box logging deemed non-essential for now → Option A
  (custom ESP32-S3 firmware) also declined.
- **Plan = Option C:** stock firmware, sketch on RA4M1, WiFi via `WiFiS3`,
  WebSocket telemetry streamed to a client; **client-side logging** + per-
  connection wall-clock anchor; `millis()`+sequence frame tagging retained.
- **Caveat:** WebSocket/WiFi serving runs on the RA4M1 control core — must be
  engineered non-blocking (1-5 Hz, fire-and-forget, time-sliced) so control
  timing is unaffected. If too jittery under load, fall back to A or Giga.
- **Optional:** small RA4M1 RAM ring buffer (~tens of seconds) for reconnect
  gap-tolerance across brief WiFi drops — not a full-session black box.
- **Accepted tradeoff:** data recorded only while a client is connected; no
  always-on onboard recording. Acceptable for tuning/monitoring sessions.
- **Clarification:** "bridge friction" = dev-time only (flashing the ESP32-S3
  removes the USB upload/Serial bridge until stock firmware is restored);
  unrelated to telemetry data loss, and NOT incurred under Option C.

## 2026-06-01 — REVISED DECISION: Option A (custom ESP32-S3 firmware)

Black-box recording (buffer + backfill, survives WiFi loss) is required, so
Option C is insufficient. Chosen path:

- **Option A:** RA4M1 runs control only; ESP32-S3 runs WiFi + WebSocket +
  backfill + ring buffer on its 8 MB flash. True dual-CPU offload — telemetry
  feels real-time, no blank spots. No new board; keeps 5 V + sensor shield.
  (No 5 V alternative exists with this feature set — Giga/ESP32/Teensy are all
  3.3 V; UNO R4 WiFi is the only 5 V board with a second onboard CPU.)
- **ESP32-S3 does WiFi AND our app simultaneously** (dual-core 240 MHz) — the
  WiFi job is unaffected.
- **Dev-update friction (control sketch / RA4M1):** jumper → restore stock
  bridge → upload sketch over USB → jumper → re-flash our ESP firmware.
  Normal RA4M1 sketch upload is automatic over USB and needs NO jumper, but
  only while the stock bridge is present; flashing custom ESP firmware
  REQUIRES the GND+Download jumper (physical).
- **Jumper requires physical access** to the board → mitigated by wiring the
  GND + Download pins to a **panel-mounted switch/header** on the enclosure.
  Then the cycle = flip switch + USB + run script (`arduino-cli`/`espflash`),
  no disassembly. Switch is the only manual step; flashing is scriptable.
