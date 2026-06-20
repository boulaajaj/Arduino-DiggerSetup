# Handoff — V8 Wi-Fi Telemetry Reliability + V9 Architectural Offload

> Drop this into a fresh Claude Code session to pick up exactly where the
> planning conversation left off. Self-contained — assume the next session
> has zero prior context.

---

## Repo

`boulaajaj/Arduino-DiggerSetup` — Arduino UNO R4 WiFi controller for a
ride-on excavator. Current branch: `claude/restore-xbus-telemetry-36`.
**PR #50 open** with telemetry + Wi-Fi dashboard work.

## Standing rules (memory, non-negotiable)

1. **Issue-first workflow** — no code changes / commits / uploads without
   a clear plan, task, goal, AND an open GitHub issue. No issue →
   conversation only.
2. **Use official tools** — prefer first-party MCPs / `gh` CLI over
   scraping. Shell as last resort.
3. **Wi-Fi is monitoring-only**, permanently. No path from Wi-Fi to motor
   output. (Decided 2026-06-15 RCA.)

## What to set up first

**Create a milestone** named **"V8 — Telemetry reliability"** and add to it:

- **#43** — Solder interface board (existing)
- **#51** — Battery/inactivity beeper (existing)
- **NEW-A** — Wi-Fi serving latency mitigation (draft below)

Separately, **draft NEW-B** as V9 future work, no milestone yet.

---

## NEW-A — Wi-Fi serving latency mitigation

**Title:** `V8: Wi-Fi serving latency — bound modem timeout, coalesce SSE writes, set AP channel 11`

**Body:**

### Problem

Operator reports 2–5 second control delays when iPhone is connected to
the dashboard over Wi-Fi. Latency is intermittent and unpredictable.

### Root cause (researched, not speculation)

WiFiS3 calls block on `Modem::passthrough()` with default
`MODEM_TIMEOUT = 10 000 ms` (`ArduinoCore-renesas/libraries/WiFiS3/src/Modem.h`).
When the iPhone's TCP receive window stalls (Safari backgrounded, weak
signal, congestion), the ESP32-S3 bridge's `client->write()` sits in
`select()` for up to ~10 seconds (`espressif/arduino-esp32` issue #8303:
`WIFI_CLIENT_SELECT_TIMEOUT_US = 1 s` × `WIFI_CLIENT_MAX_WRITE_RETRY = 10`).
The RA4M1 sketch's `loop()` is frozen the whole time, so no servo update,
no S.BUS read.

Our SSE push currently uses **3 separate writes per frame** (`": hb\n"`,
`data: <json>`, `\n\n`), multiplying the stall risk.

Community confirmation: forum thread "R4 WiFi slow at serving web pages
(~100 ms / println)"; `ArduinoCore-renesas` #512 (2–3 s UDP delay
regression); `Links2004/arduinoWebSockets` #909 (freeze on page reload).

### Gate — hardware first

Operator suspects X.BUS dropouts are mechanical (loose ground / cold
joint / vibration on the soldered interface board). **Open machine and
verify all X.BUS solder joints + ground continuity + R1 pull-up before
any firmware change.** Many symptoms attributed to Wi-Fi may actually be
X.BUS.

### Plan (only if hardware checks pass), in order

1. **Set Wi-Fi AP channel = 11** in `beginAP()`. Avoids common 2.4 GHz
   Wi-Fi neighbors. Doesn't help vs RC6GS (FHSS).
2. **Coalesce SSE writes** — build `": hb\ndata: <json>\n\n"` into one
   buffer, single `sseClient.write(buf, n)`. Cuts AT round-trips per
   frame 3 → 1.
3. **`modem.timeout(50)`** before every Wi-Fi call (`Modem.h` exposes
   `void timeout(size_t)`); restore after. Caps worst-case loop stall at
   50 ms. Drops the occasional frame; correct trade for a vehicle
   controller.
4. **Only if 1–3 are insufficient**: drop SSE push 10 Hz → 2 Hz.

### Anti-patterns

- Don't reduce X.BUS poll rate.
- Don't touch S.BUS path.
- Don't claim Wi-Fi is the cause until you've measured `wifiUpdate()`
  execution time with
  `uint32_t t0 = micros(); …; uint32_t dt = micros() - t0;` and seen
  multi-100 ms `dt`.

### Acceptance

- Drive with phone connected. Input-to-track latency < 300 ms
  consistently.
- No control regression: FS=0, Lost=0, RC + joystick drive cleanly.

---

## NEW-B — V9 split-brain ESP32-S3 firmware (future)

**Title:** `V9: Split-brain ESP32-S3 firmware — telemetry buffer, replay, offload HTTP/SSE to spare core`

**Body:**

### Goal

Move all HTTP/SSE serving off the RA4M1 entirely so Wi-Fi load cannot
perturb control timing.

### Architecture

- Fork [`arduino/uno-r4-wifi-usb-bridge`](https://github.com/arduino/uno-r4-wifi-usb-bridge)
  (the stock ESP32-S3 bridge firmware).
- Keep its USB-serial bridge code untouched → `arduino-cli upload` to
  RA4M1 keeps working normally.
- Add our HTTP/SSE server as a parallel FreeRTOS task on the spare second
  core (240 MHz Xtensa). The bridge runs on core 0; our code on core 1;
  they share PSRAM but don't interrupt each other.
- RA4M1 sketch streams compact binary telemetry frames (24–32 B) to ESP32
  via a new AT command (e.g. `AT+DIGGERTELEM=<binary>`),
  fire-and-forget. No `Modem::passthrough()` waits.

### Telemetry buffer + replay

- Ring buffer in PSRAM, indexed by sequence number.
- 10 Hz × 30 min × 32 B = ~576 KB → trivially fits 8 MB PSRAM. 8 MB flash
  holds ~7 hours.
- HTTP endpoint `/replay?from=<seq>` returns missed frames after Wi-Fi
  dropouts. Client (dashboard) detects gap by seq and requests the fill.

### Build tooling (minimal new burden)

- `arduino-cli core install esp32:esp32`
- `arduino-cli compile --fqbn esp32:esp32:esp32s3 <sketch>`
- No PlatformIO, no Arduino IDE, no Docker, no npm.

### Operational caveat

First flash needs the panel switch (GND + Download pins) per existing
`docs/WIRING-GUIDE-V8.md`. After that, RA4M1 sketch upload still works.
Re-flashing the ESP32 (rarely) needs the switch again. ESP32-S3 mask-ROM
bootloader is unerasable, so even a botched flash is recoverable via
`espflash`.

### Triggers (when to start V9)

- NEW-A mitigations don't get latency to acceptable, OR
- Dashboard needs richer graphics that don't fit RA4M1's 256 KB flash, OR
- We need session-replay / black-box logging that survives Wi-Fi drops.

### Anti-patterns (V9)

- Don't go architectural before NEW-A is exhausted.
- Don't touch the bridge code; only add to it via 2–3 well-defined hook
  points (`setup_app()` from bridge `setup()`, one new AT-command
  handler, optional stats command).

---

## Quick reference — source-code citations

(So the next session doesn't have to re-research.)

- `ArduinoCore-renesas/libraries/WiFiS3/src/Modem.h` — `MODEM_TIMEOUT 10000`,
  `void timeout(size_t)`
- `ArduinoCore-renesas/libraries/WiFiS3/src/Modem.cpp` — `buf_read()`
  timeout loop; `passthrough()` blocks until `OK`
- `ArduinoCore-renesas/libraries/WiFiS3/src/WiFiClient.cpp` — `write`,
  `available`, `connected`, `stop` all call `modem.write(...)` (blocking)
- `arduino/uno-r4-wifi-usb-bridge` — `UNOR4USBBridge/cmds_wifi_netif.h`
  `_CLIENTSEND` handler does synchronous `client->write()`
- `espressif/arduino-esp32` #8303 — `WIFI_CLIENT_SELECT_TIMEOUT_US = 1 s`
  × 10 retries
- `arduino/ArduinoCore-renesas` #512 — 2–3 s UDP delay regression
- `Links2004/arduinoWebSockets` #909 — freeze on page reload, R4 WiFi

---

## Misc context

- **Beeper (#51)** plan: active piezo on D8.
  `IDLE_THRESH_MS = 5 min`, `BEEP_PERIOD_MS = 5 s`, `BEEP_ON_MS = 200`.
  Trigger when `sbusValid == false` continuously for ≥ 5 min. Silences
  instantly on `sbusValid` returning true. Pin D8 is already reserved in
  `CLAUDE.md`. Recommended part: 5 V active piezo, ~85 dB, single GPIO.
- **PR #50** open, branch `claude/restore-xbus-telemetry-36`.
  Decision-log entry for 2026-06-15 says PR is monitoring-only, no
  Wi-Fi→motor path; to be hardened (single-source page, debug cruft
  removed) after main merge.

---

## Kickoff prompt for the next session

> We are following the **issue-first workflow** rule from memory. Start
> by reading this handoff in full, then:
>
> 1. Create the V8 milestone named "V8 — Telemetry reliability".
> 2. Add **#43** and **#51** to it.
> 3. Create **NEW-A** as described above (Wi-Fi serving latency
>    mitigation) and add it to the V8 milestone.
> 4. Create **NEW-B** as described above (V9 split-brain firmware),
>    no milestone for now.
> 5. **Stop** after the GitHub admin is done. No code changes. Wait for
>    operator to open the machine and inspect X.BUS hardware first.
