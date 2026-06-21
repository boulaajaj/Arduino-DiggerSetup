# Decision Log

Technical decisions, test results, and confirmed hardware facts.
Updated by session hooks — only technical content, no personal info.

---

## 2026-06-20 — SAFETY (P0): Wi-Fi serving stalled control loop → runaway under load

- Loaded test (~60 lb): digger made **uncommanded movement (runaway toward operator)**
  during Wi-Fi/dashboard activity. Injury hazard.
- Cause: dashboard serving (~36 × 1 KB chunked page writes + SSE) blocks `loop()` ~1–2 s
  on the shared RA4M1 core. Servo PWM is hardware-timed and holds the last pulse, so the
  ESC keeps executing the **last throttle command** while `loop()` — and the in-loop
  RC-lockout failsafe — is frozen.
- `modem.timeout(50)` caps each modem call but NOT the cumulative ~36-call page send.
- Failsafe required (tracked P0 in #69): hardware WDT (~250 ms) refreshed only in the
  control section → MCU reset to neutral if the loop is starved; bound Wi-Fi work per
  loop pass; fail-to-neutral latched. Permanent fix = #55 (ESP32-S3 offload).
- "Stop unless Wi-Fi client connected" is the WRONG mechanism — the hazard occurs WHEN a
  client is connected (that's what blocks the loop); no client = clean loop.
- Interim rule: do NOT load/refresh the dashboard while driving under load.

## 2026-06-20 — Dashboard page-load froze control loop; fixed by HTTP caching

- **Confirmed via USB-serial capture:** with no Wi-Fi client, loop runs clean
  ~10 Hz, both ESCs telemetry OK (12.4–12.5 V), X.BUS solid after the interface-
  board resolder (rx_total climbs steadily).
- **Confirmed problem:** loading/refreshing the dashboard froze the loop 1–2 s
  (24 stalls in 35 s, up to ~2 s), starving S.BUS reads, servo updates, and X.BUS
  polling (rx_total nearly flat during stalls).
- **Root cause:** 33 KB dashboard HTML served `Cache-Control: no-store` →
  re-downloaded every refresh as ~33 sequential 1 KB blocking modem writes. The
  SSE telemetry path was already numbers-only and fine.
- **V7.9 (SHA dabfbf1):** AP channel 11, SSE 10→5 Hz, coalesced SSE writes,
  `modem.timeout(50)` around `wifiUpdate()`. Caps each modem call at 50 ms but a
  full page is ~33 calls → still ~2 s per load.
- **V7.10 (SHA 4197350) — field-confirmed fix:** serve `/` with `ETag` +
  `Cache-Control: no-cache`; parse `If-None-Match`; reply `304` on match. Refreshes
  no longer re-download the page or freeze the loop. First load still ~2 s once
  (inherent to serving 33 KB on the RA4M1); eliminating that is the V9 ESP32-S3
  offload (#55).
- **Safety note:** during any Wi-Fi-induced loop freeze, servo PWM holds its last
  value and RC/joystick/failsafe are not serviced — do not load the dashboard
  while driving until #55 lands.

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

## 2026-06-14 — Interface board soldered + back together

- **Solder build complete.** Machine reassembled with the soldered interface
  board (replaces the breadboard). Standalone power confirmed: Left ESC BEC
  (7.4 V) → 9-pin cable yellow thick wire → Arduino barrel jack → buck →
  board powers up without USB.
- **S.BUS inverter circuit confirmed working** on the soldered board with
  R3 = **1 kΩ** (NOT the documented 10 kΩ; field-verified working). Math:
  drives Q1 base ~2.6 mA vs 0.26 mA at 10 k — well above saturation needs,
  storage-time slowdown at 100 kbaud S.BUS is negligible. 10 kΩ preferred
  on new builds (lighter receiver load); current 1 kΩ acceptable to keep.
- **X.BUS telemetry confirmed working** on the soldered board: both ESCs
  reporting (V0=11.6 V, V1=11.5 V; RPM tracking; temps 22–25 °C; OK0=OK1=1).
- **2N3904 orientation confirmed** for this build: flat side **facing AWAY**
  from operator → from operator's view, legs L→R are **C / B / E**.
- **Joystick cable rewiring:** Y axis cable flipped on board side to correct
  forward/backward polarity (was reading ~5100 instead of ~8192 at rest;
  re-orientation restored ~8192 center). After Y fix, X (steering) was
  inverted.
- **JOY_STEER_DIR = -1.0f** added to `rc_test.ino` `[CONFIG]` and applied
  to joystick X (steering only — RC steering unchanged). Right stick now
  turns right. Field-verified by operator.

## 2026-06-15 — Wi-Fi self-drive root cause + telemetry-dropout discriminator

- **Committed flight firmware (`rc_test` V7.8, PR #50) has NO Wi-Fi→motor
  path.** Code trace: the only writes to the ESCs (`outputWrite` →
  `escL/escR.writeMicroseconds`) are fed solely by S.BUS (RC) and the
  joystick ADC. `wifiUpdate()` serves `/`, `/data`, `/events` (SSE) only and
  never touches motor output; the dashboard pages contain no command/POST
  code. Failsafe holds neutral (SVC) when S.BUS is invalid.
- **Reported "digger moves by itself when the dashboard opens" came from an
  uncommitted local build**, not in git and not in FIRMWARE-UPLOAD-LOG (last
  logged flash = 0875a87, monitoring-only). Remediation: reflash the committed
  monitoring-only firmware from PR #50 (check out the branch first — the
  dangerous build may still be in the working copy).
- **Telemetry-dropout discriminator:** when control-derived fields
  (throttle/dir/gear/mode) keep updating in the dashboard while only
  RPM/temp/V/I go stale (panels dim to 0.35), the SSE/Wi-Fi link is healthy
  and the **X.BUS half-duplex bus (D0/D1) is dropping** — not Wi-Fi. A Wi-Fi
  loss would freeze the whole stream and trip the OFFLINE banner. Confirms the
  #43 breadboard-vibration X.BUS dropout; fix is the interface-board solder,
  not firmware.
- **DECISION: Wi-Fi telemetry stays monitoring-only, permanently.** No path
  from Wi-Fi to motor output. PR #50 to be hardened (single-source page, debug
  cruft removed) after it merges to main.

## 2026-06-18 — PR #50 lint/safety pass (zero-risk only; driving untouched)

Cleared the 3 failing CI lint checks + 2 review-bot findings WITHOUT changing
any tested driving behavior. Both sketches recompile clean (rc_test 39% flash /
29% RAM; telem_check 20%).

- **`rc_test.ino` (flight) — only 2 edits, both behavior-neutral:**
  (1) 3× `(const uint8_t *)` → `reinterpret_cast<const uint8_t *>` in the Wi-Fi
  `client.write`/`sseClient.write` calls (cppcheck cstyleCast; identical machine
  code). (2) `snprintf` return clamped in `buildTelemJson()` so callers never
  read past the 360 B buffer (CodeRabbit "critical"; a no-op unless a frame ever
  overflows — it currently does not). **No change to control loop, mixing, expo,
  gear caps, S.BUS, joystick, servo PWM, X.BUS polling, or telemetry values.**
- **`telem_check.ino` (bench tool, never flashed to the machine):** fixed
  garbage ESC-temp display (2-byte → 1-byte `d[14]`, per 2026-06-11 finding) +
  cpplint whitespace reformatting. Display/format only.
- **`INTERFACE-BOARD-PERFBOARD.md`:** blank lines for markdownlint.
- **DEFERRED (NOT changed), with reasons:** (a) X.BUS checksum validation —
  protocol checksum is officially ambiguous and the 0x10-response checksum was
  never hardware-verified; hard rejection could silently kill working telemetry,
  so verify during #54 hardware bring-up first. (b) Dashboard `s==0`-forward
  arrow — needs hardware diagnosis (2026-06-13). (c) `wifiUpdate()` blocking /
  SSE coalescing — that is issue #54, gated on hardware inspection.
- **Re-test:** flight edits are behavior-neutral, so driving cannot change; a
  quick re-flash smoke check is cheap insurance but not required for control.

## 2026-06-19 — Doc alignment to current hardware (issue #52)

Confirmed canonical board = **UNO R4 WiFi** (migration complete); X.BUS telemetry
and the Wi-Fi dashboard are LIVE (V7.8), not deferred — corrects #52's table.

- Rewrote `PROJECT-PLAN.md` and `CLAUDE.md` to GL10/GL540L + UNO R4 WiFi +
  **open-loop PWM control** (Arduino outputs servo PWM 1000–2000 µs on D9/D10;
  GL10 internal FOC owns smoothing; no Arduino-side PID/feedforward/inertia) +
  live X.BUS 0x10 telemetry (monitoring-only) + V7.8. FQBN `unor4wifi`, port COM7.
- Deleted superseded retired-hardware files: `docs/WIRING-GUIDE.md` (old
  E10/E3665), `docs/PLANT-CHARACTERIZATION.md`, `docs/XBUS-INVESTIGATION.md`, and
  sketches `xbus_master` / `xbus_probe` / `xbus_inverted_test` / `xbus_poll_test`
  (also removed from the arduino-ci matrix). `WIRING-GUIDE-V8.md` is the canonical
  wiring reference.
- Swept E10/E3665/CS7581/UNO-Q tokens out of all current files (acceptance grep
  clean); remaining "Nano R4" mentions are historical/comparative only.
- `XBUS-PROTOCOL.md` kept as a CURRENT reference (live telemetry uses func 0x10).
- All sketches compile on `unor4wifi`; markdownlint clean on changed docs.

## 2026-06-20 — Battery + inactivity beeper (V7.12, PR #71 / #51)

New `[ALERT]` module on the D8 piezo (audio only — no motor-path change; the
low-voltage motor cutoff is split to PR #2 / #65 by risk).

- **Inactivity alarm:** RC transmitter off (`sbusValid==false`) > 60 s → one long
  beep / 2 s (`500/1500`, non-latching). `sbusValid` already encodes RC-off
  (S.BUS failsafe or frame-loss timeout). Purpose: unplug LiPos (parasitic ~2 W
  draw deep-discharges packs over 1–2 weeks).
- **Low-voltage alarm:** worst-of-two pack < **10.5 V** → three fast chirps /
  ~1.2 s (`120/120 ×3, 600`), validity-gated (both ESCs plausible 6–13 V; a
  not-yet-powered ~0 V pack can't false-alarm), 3 s sag debounce, 60 s startup
  grace, **latched until power cycle**. Priority: low-V > inactivity.
- **10.5 V chosen** (≈3.5 V/cell, ~30%) deliberately conservative: large/expensive
  15 Ah packs, long runtime, and the operator's balance charger false-declares a
  pack "dead" if it arrives too low. Stop early, stay healthy.
- **Bench test PASS (2026-06-20):** flashed a TEST build at `LOWV_THRESH_V=12.0`
  (packs resting ~12.2/12.4). Confirmed the low-V alarm beeps when the worse pack
  reached ~12 V, and latches. Production value restored to **10.5 V** (same code
  path, constant only) and re-flashed (SHA 8a33a9b). Note: clearing the latch
  power-cycles the board, which drops USB/COM7 — replug to re-upload.
- **Open finding — voltage signal is fast, not a battery average:** the alarm
  reads `telem[].voltage`, whose EMA is `TELEM_A_VOLT=0.30` at ~30 Hz/ESC ≈ 0.3 s
  smoothing — it sags under load. The 3 s debounce kills short sag spikes but not
  a sustained hard pull, so 10.5 V could false-latch under load in the field.
  Planned refinement before trusting 10.5 V live: a dedicated ~30 s EMA (per #40)
  or much longer debounce for the alarm signal.
- Beep vocabulary kept distinct (count the beeps): 2-short=Wi-Fi ready,
  steady=horn, 1-long/2 s=inactivity, 3-fast/latched=low battery. Documented in
  OPERATOR-GUIDE.md "Beep meanings" table (#70).
- Follow-ups opened: #72 (smooth pivot→straight transition), #73 (dashboard
  visual alarm: red battery + error code, firmware-sourced).

## 2026-06-21 — P0 Wi-Fi runaway failsafe (V7.13, PR #74 / #69)

Root cause: serving the ~33 KB dashboard as one blocking burst froze loop()
~1–2 s while the hardware-timed Servo PWM held the last throttle → uncommanded
runaway under load. The in-loop S.BUS failsafe couldn't help (it was frozen too).

Fix (two coupled parts):
- **Hardware watchdog (RA4M1 WDT, 250 ms):** armed at end of setup() (after AP
  bring-up); WDT.refresh() called ONLY after a successful control update (inputs
  read + outputs written). Any loop stall > 250 ms resets the MCU → PWM stops →
  ESCs neutral. Accepted as last-resort emergency stop (full reboot; AP blips).
- **Incremental Wi-Fi serving:** wifiUpdate() does at most ONE modem write per
  loop pass (one 1 KB page chunk OR one request OR one SSE frame); body streams
  via a pageRemaining state machine; headers/304/data coalesced to single
  writes. Keeps each loop pass well under the WDT timeout so normal refresh never
  trips it.
- Robustness (Copilot review): snprintf returns guarded; 0-byte page write
  aborts the transfer instead of spinning forever (would starve SSE).

**Operator-confirmed (2026-06-21):** dashboard refresh no longer stalls/resets
the loop; the watchdog stands as the rare backstop only.

Permanent fix remains #55 (offload Wi-Fi/HTTP/SSE to the ESP32-S3 spare core),
which removes Wi-Fi from the control core entirely. WDT is the interim backstop.
