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

- **An SPI SD-card module cannot coexist with S.BUS on UNO R4 WiFi.** (The
  board has no onboard SD slot — this is about adding an SPI SD module.)
  Hardware SPI is on D11 (MOSI) / D12 (MISO) / D13 (SCK); `sbusUart` (SCI0) is
  on D11 (TX) / D12 (RX). SD-over-SPI lands on the exact S.BUS pins. No third
  hardware UART to relocate S.BUS to (Serial1/D0-D1 reserved for X.BUS). So an
  SPI SD card is not viable on this board without evicting S.BUS. (Clock/RTC is
  unaffected — I2C on SDA/SCL, or NTP over WiFi; WiFi is unaffected — ESP32-S3
  coprocessor.)
- **Stock UNO R4 WiFi offloads WiFi/TCP but NOT WebSocket serving from the
  control core.** The split: the ESP32-S3 runs the WiFi radio + TCP/IP stack
  (reached via the `WiFiS3` bridge). The RA4M1 runs the Arduino sketch AND the
  WebSocket layer — HTTP-upgrade handshake, frame (de)masking, ping/pong,
  server state machine — in libraries like UnoR4WiFi_WebServer / mWebSockets.
  That WebSocket state machine is serviced inside `loop()` on the RA4M1, the
  same core as the control loop, so a busy/blocked loop stalls it and vice
  versa. Net: TCP/WiFi is offloaded; WebSocket serving is not — it shares the
  control core and CAN perturb control timing. (The WiFiS3 bridge exposes only
  socket I/O, so the ESP32-S3's 8 MB flash is not addressable as RA4M1 sketch
  storage without custom ESP firmware. Corrects the assumption in #45 that
  serving a page can't affect control-loop timing.)
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
- **Path B — Giga R1 WiFi:** STM32H747 dual-core (M7 480 / M4 240). Memory:
  1 MB RAM, 8 MB SDRAM, 2 MB MCU flash, 16 MB QSPI NOR (sketch-accessible).
  USB-A host for thumb-drive mass storage, 4 UARTs, onboard WiFi/BT. Programmed
  directly over USB-C (no bridge conflict). True offload of control vs I/O.
  COST: 3.3 V logic only — 5 V joystick needs level-shifting/divider, S.BUS
  inverter output swing must be re-verified; bigger enclosure + re-mount.
- **Capacity:** compact binary frame ~24-32 B → 8 MB ≈ ~250-300k frames
  ≈ ~15 h at 5 Hz.
- **Lean (at time of writing):** Giga for the full black-box + analytics goal.
  Path A is the no-new-hardware fallback. **Superseded — final decision is
  Option A (custom ESP32-S3 firmware); see the REVISED DECISION below.**

## 2026-06-01 — DECISION: stay on UNO R4 WiFi, stock config (Option C) — SUPERSEDED

> **Superseded** by the REVISED DECISION (Option A) below: black-box recording
> that survives WiFi drops is required, which stock Option C cannot provide.

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

## 2026-06-11 — Telemetry bring-up bench check (issue #36) — pre-flash baseline

Goal this session: verify X.BUS telemetry comes through from BOTH GL10 ESCs
on the new (post-migration) circuit, before enabling it over Wi-Fi (#45).
Scope intentionally limited to docs/memory/issue — no sketch or control changes.

- **Board confirmed:** Arduino UNO R4 WiFi on **COM7** (`arduino-cli board list`).
  Nano R4 → UNO R4 WiFi migration is complete; CLAUDE.md still lists the old
  Nano R4 / COM8 (stale, not corrected this session).
- **Flashed sketch = `rc_test` V7.6 (PWM-only), UNCHANGED.** Working tree clean
  (only untracked `dashboard/index.html`, unrelated #45 work). No edits to
  `rc_test.ino` or any control code. Control logic intact.
- **Live CSV columns are `RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost`.**
  The trailing `0,0,0` are Gear/Failsafe-count/Lost-frames — **NOT telemetry.**
  `rc_test` does not poll X.BUS, so nothing reads the telemetry line yet.
- **Control side healthy:** S.BUS FS=0 / Lost=0 over a multi-second capture;
  RC channels live, joysticks centered (~8192 on 14-bit ADC), OutL/OutR mixing
  on D9/D10.
- **Telemetry NOT yet verified.** The circuit is wired on **D0/D1 (Serial1)**,
  unchanged through the migration, but cannot be confirmed until a poll sketch
  is flashed. Verification is the next step (deferred — no flash this session).
- **#36 "Options to evaluate" (bridge MCU / relocate S.BUS / switch board) are
  OBSOLETE.** They assumed Nano R4 UART scarcity. On UNO R4 WiFi both hardware
  UARTs are free: **Serial1 (D0/D1) → X.BUS @ 115200**, **sbusUart (SCI0/D12)
  → S.BUS**. Go straight to the telemetry scope.
- **Confirmed telemetry approach (unchanged from plan):** Read Register (func
  **0x10**), NOT Throttle (0x50 forces BUS_MODE and fights PWM). Registers
  0x0C VbatBus, 0x20 mos-Tem, 0x22 mot-Tem, 0x02 motSpeed (RPM). 1 Hz
  alternating per ESC, EMA + freshness watchdog. Note: 0x10 is documented-safe
  but **not yet confirmed on GL10 hardware** (only 0x50 has been, on breadboard
  2026-05-23) — flashing a 0x10 poller doubles as that confirmation.
- **Issue alignment:** work = **#36** (restore telemetry) → **#45** (Wi-Fi
  dashboard). **#48** (ESP32-S3 / Giga black-box logging architecture) is
  explicitly set aside — not part of this work.

### Telemetry TEST RESULT (flashed `sketches/telem_check`, read-only)

Flashed a read-only telemetry tool (proven 0x50 framing, **throttle hard-wired
to 0** — never drives motors) to confirm the new circuit. Motors did not move
(Thr_out=0%, RPM=0 throughout).

- **ESC0: telemetry confirmed working.** V=**11.9 V** (plausible 3S at rest),
  status flags **BUS_MODE + CAP_CHARGED**, frames steady. The X.BUS link on
  D0/D1 (Serial1) on the UNO R4 WiFi works.
- **ESC1: NO RESPONSE** — "no data yet" for the entire run. Good/Bad frame
  ratio held ~50/50 (every poll targeting ESC1 timed out), confirming exactly
  one of two ESCs answers.
- **Likely causes for ESC1 (to chase next):** (a) ESC1's X.BUS yellow not
  landed on the bus node / cold joint on the new board, or (b) both ESCs share
  X.BUS address 0 (register 0x05) so ESC1 isn't individually addressable — each
  ESC needs a distinct X.BUS ID (0 and 1) set in TXC-Link. Breadboard test
  2026-05-23 had both responding, so a new-circuit wiring fault is the leading
  suspect.
- **Decode bug (cosmetic, not hardware):** ESC-temp prints garbage (~15898 °C)
  because it's decoded as int16 `d[14]|d[15]<<8`. Raw bytes suggest ESC temp is
  a single byte (~62 → 22 °C ≈ room temp). Fix the telemetry temp decode to
  1-byte before the real `rc_test` integration; voltage decode (offset 12–13)
  is correct.
- **Flight firmware (`rc_test` V7.6) must be re-flashed** to restore the
  machine after this bench check — `telem_check` is a diagnostic only.

### V7.7 — telemetry integrated into flight firmware (telemetry WITH control)

Added non-blocking X.BUS **Read Register (0x10)** polling into `rc_test`
(`[TELEMETRY]` module on `Serial1`/D0-D1). Read-only — 0x10 is point-to-point
service control, never enters BUS_MODE, so PWM/S.BUS control authority is fully
preserved. Polls ESC0/ESC1 alternately ~6 Hz each (80 ms gap, 12 ms non-blocking
timeout), EMA-smooths V/I/temps, RPM instantaneous, 5 s per-ESC freshness watchdog.

- **0x10 CONFIRMED WORKING on GL10** (first time — previously only 0x50 was).
  ESC0 returns clean values: **V=11.9 V, ESC 28 °C, motor 22 °C, RPM tracking
  throttle (7→38 Hz electrical as stick went 1766→2000 µs).** Per-register 0x10
  decode also fixes the bogus temp seen via 0x50 — temps now read room-plausible.
- **Control fully intact alongside telemetry:** FS=0 / Lost=0, OutL/OutR follow
  the stick. Confirms telemetry-with-control is real (not telemetry-only).
- **ESC1 still silent** (OK1=0) — same as the read-only bench test. Blocks
  per-track capture of the 2nd track. Chase: ESC1 X.BUS yellow on the bus node,
  or distinct X.BUS address (reg 0x05) — both ESCs may be at default addr 0.
- **Bus current reads 0.0 A** at idle/unloaded — verify under load (reg 0x0D).
- **CSV format changed** (V7.7): appended `V0dV,I0dA,RPM0,TE0,TM0,OK0` +
  `…1` columns (integer-scaled). `live_plot.py` / `monitor.py` / dashboard
  parsers must be updated for the new columns.
- Firmware flashed to UNO R4 WiFi / COM7 (58.9 KB, 22% flash / 26% RAM).

## 2026-06-13 — BOTH ESCs telemetry confirmed under load (V7.7)

After a **full power cycle** of the board (USB/board power removed + reset,
not just the reset button), **ESC1 telemetry now responds** — both ESCs report.

- Full-throttle Turbo, both tracks driven: **ESC0** V=11.7 V, I=3.3 A,
  RPM=200 Hz, ESC 25 °C, motor 22 °C; **ESC1** V=11.7 V, I=3.2 A, RPM=197 Hz,
  ESC 23 °C, motor 21 °C. OK0=OK1=1.
- **Good left/right symmetry** at matched command (200 vs 197 Hz, 3.3 vs 3.2 A)
  — bears on the open track-asymmetry item; looks balanced here.
- **Bus current (reg 0x0D) works under load** (~3.2–3.3 A). The earlier 0.0 A
  was genuine no-load idle, not a decode bug.
- **RPM tracks throttle on both ESCs** (ESC0 76→200, ESC1 56→197 during ramp).
- ESC1's prior silence was a transient bad state (likely leftover BUS_MODE from
  the earlier 0x50 misstep), cleared by the power cycle — NOT confirmed as a
  permanent wiring/address fix. If ESC1 goes silent again, check its X.BUS
  yellow on the bus node and X.BUS address (reg 0x05).
- Op note: after USB unplug, the UNO R4 WiFi only re-enumerated as COM7 after a
  full power cycle of the ESP32-S3 USB bridge (RA4M1 reset alone didn't).

## 2026-06-13 — Wi-Fi telemetry dashboard live (V7.8, #45)

Added a Wi-Fi AP + HTTP telemetry server to `rc_test` ([WIFI] module, WiFiS3)
and rewired `dashboard/index.html` from simulation to live `/data` polling.

- **AP mode** SSID `Digger-Telemetry`, pass `digger12345` (WPA2), IP
  **192.168.4.1**. `WiFi.beginAP` returns status 7 (WL_AP_LISTENING) on boot —
  AP confirmed broadcasting; telemetry both ESCs OK while AP up.
- **Server:** `GET /data` → compact JSON (millis `t`, `seq`, gear, mode, fs,
  lost, outL/outR, per-ESC ok/rpm/cur/v/tE/tM, integer-scaled). CORS `*`.
  `GET /` → plain-text info line. **Monitoring only — no control input over
  Wi-Fi** (safety, matches #45 decision).
- **Non-blocking design:** one client transaction per loop pass, 20 ms bounded
  request-read window, tiny payload. Servo PWM is hardware-timed so a brief
  loop stall can't glitch ESC output. (Heeds the RA4M1-shares-control-core
  caveat from the 2026-06-01 analysis — kept light, ~5 Hz client poll.)
- **Dashboard:** auto-targets `/data` when served by the board, else
  `http://192.168.4.1/data` when opened as a local file; 5 Hz poll, NO-LINK
  staleness watchdog, dims a stale ESC panel. RPM sent as electrical Hz ×30.
- Flash 26% / RAM 28% (WiFiS3 included). `wifiDebug()` prints a 3 s status
  comment line over USB during bring-up (remove once stable).
- **Open:** page is opened as a local file for now (laptop). To browse straight
  to 192.168.4.1 from a phone we'd embed the page in firmware (PROGMEM) — next
  step if wanted.

## 2026-06-13 — Dashboard served from board + SSE streaming (perf, #45)

Field-tested on iPhone (browse to 192.168.4.1). Two fixes:

- **Embedded the dashboard in firmware** (`web_page.h`, PROGMEM/flash, served at
  `/`). Any device — phone included — browses to 192.168.4.1; no local file.
  Page fetches same-origin so no CORS. Flash 36%, RAM unaffected (page in flash).
- **Update rate was ~1–2 Hz, not 5 Hz** — root cause: HTTP polling opened a NEW
  TCP connection every poll, and WiFiS3 per-connection setup is slow. **Switched
  to Server-Sent Events (SSE):** one persistent connection, server pushes a frame
  every 100 ms (10 Hz). `EventSource` also **auto-reconnects** after a dropout
  (helps the breadboard). Endpoint `GET /events` (text/event-stream); `/data`
  one-shot kept for debugging.
- **Faster X.BUS polling:** poll gap 80→10 ms, response timeout 12→6 ms, so a
  silent/flaky ESC barely stalls the healthy one (~30–40 Hz/ESC underlying).
- **Removed Google Fonts @import** — on the offline AP it stalled first paint
  (browser waiting on a fetch that can't succeed). Fixed the "loads messed up,
  then settles after 1–2 s" behavior; system-font fallbacks render instantly.

**Hardware observation (operator):** on the breadboard, one ESC's X.BUS telemetry
intermittently drops out under vibration (telemetry stops, then returns). Suspected
loose/unstable breadboard connection. Plan: solder the interface board (#43) to
stabilize. Not a firmware fault — control + the other ESC keep working through it.

### Dashboard responsiveness + direction (operator feedback)

- **Value lag fixed:** numbers (RPM/speed/current) eased down slowly after stick
  release because the dashboard applied its OWN EMA on top of the firmware's. The
  firmware already smooths V/I/temps and sends RPM raw, so the client EMA was pure
  lag — removed; values now render directly from the 10 Hz stream.
- **no-store on the page** so the iPhone always loads the latest dashboard build.
- **Direction indicators (open):** operator reports L/R arrows show the same
  direction during a pivot (counter-rotation). Dashboard already derives each
  arrow per-track from its own commanded output (outL→L, outR→R), which should
  differ in a pivot — to be diagnosed against live OutL/OutR + RPM-sign data
  before changing logic (is RPM signed on the GL10 in reverse?).
