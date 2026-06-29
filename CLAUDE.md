# Excavator Track Controller — Project Memory

## What This Is

Arduino **UNO R4 WiFi**-based tank-style track controller for a ride-on
excavator. Two FOC brushless motors drive rubber tracks via FOC ESCs. Dual
input: RC transmitter (Jason) and joystick (Malaki/rider). 3-position override
switch selects who has authority. X.BUS telemetry is streamed to a Wi-Fi
dashboard (monitoring only).

## Project Board — Single Source of Truth

[Project #1](https://github.com/users/boulaajaj/projects/1) is the **single
source of truth** for what is pending, in progress, and done. Anything being
worked on **must** show as **In Progress** on the board.

**The rule: starting work = open a draft PR, and every PR gets its own issue.**
As soon as work begins, open a *draft* PR whose body links its issue (`Closes
#N`). One PR ↔ one issue — never a PR with no issue (it won't appear on the
board, by design). A draft PR is the earliest GitHub-visible signal, and the
board automation keys off it:

- `.github/workflows/board-pr-in-review.yml` → when a PR links an **In Progress**
  issue, the PR card is added and set to **In Review**. A PR with no linked
  In-Progress issue is left off the board entirely.
- `.github/workflows/board-merged-pr-done.yml` → when a PR that is already on the
  board is **merged**, its card moves to **Done** (it never adds a stray card).
- Built-in Project workflow (enabled in the UI) moves **closed issues** to
  **Done**.

> **Automation limits (so expectations are right):** the In-Review step only
> fires when the linked closing issue is already **In Progress**, and it is
> skipped on PRs that don't receive the `PROJECTS_TOKEN` secret (e.g. fork PRs).
> In those cases set the board status by hand. Setting the issue itself to
> In Progress is still manual today.

So: **never leave active work as a bare branch with no PR** — it is invisible to
the board. One commit + a draft PR is enough to make it show. Agents must follow
this without being asked.

## Firmware Flash — Always Log It

**Every** firmware upload to the digger (production *or* bench/test) gets a row
appended to `docs/FIRMWARE-UPLOAD-LOG.md`, immediately after the flash — no
exceptions. The table columns are **Date · Version · SHA · Branch · Board/Port ·
Notes**; record the **PR #** inside the **Branch** cell (e.g.
`agent/C-Builder/foo (PR #93)`), and put **what changed** + **what to test** in
**Notes**. **Version** is the sketch version string carried in the build (e.g.
`rc_test V7.18`), not a git tag. This is the device's flight log; an unlogged
flash is treated as not done.

## People

- **Jason** — RC transmitter operator (safety supervisor)
- **Malaki** — Rider/operator using the joystick
- **boulaajaj** — GitHub owner / builder

## Hardware Stack

| Component | Model | Key Detail |
| --- | --- | --- |
| Controller | Arduino **UNO R4 WiFi** | Renesas RA4M1 (48MHz, 14-bit ADC, 5V tolerant) + ESP32-S3 Wi-Fi coprocessor |
| ESC (x2) | XC GL10 80A FOC | Standard 50 Hz servo PWM input, internal FOC compensation, IP67 |
| Motor (x2) | XC GL540L | Sensored brushless, paired with GL10 |
| Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| RC System | Radiolink RC6GS V3 + R7FG | 6CH gun-style (trigger = throttle, wheel = steering); the Arduino does the tank mixing |
| Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect (direct to ADC — 5V tolerant) |

**Hardware history:** previously ran XC sensored ESCs/motors with an
Arduino-side dual-loop PID. Swapped to GL10 + GL540L on 2026-04-25 — the FOC ESC
handles motor acceleration compensation and smoothness internally, so the
Arduino's job shrank to input mixing, override switching, and gear caps. The
earlier dual-loop PID / hall-sensor-tap control plans are retired (see
"Control strategy"). Then migrated Nano R4 → UNO R4 WiFi for onboard Wi-Fi
telemetry. V7.6 removed the reverse-direction beeper; audible alerts returned in
V7.11–V7.12 as a battery + inactivity alarm on D8 (issue #51 / #68).

## Control strategy (PWM out, FOC inside the ESC)

**The Arduino generates the throttle/reverse command and sends it as servo PWM.
The GL10's internal FOC executes it.**

| Who | Job |
| --- | --- |
| Arduino (RA4M1) | Read RC + joystick → expo/deadband → curvatureDrive mix → override → gear cap → output servo PWM (1000–2000 µs) per track on D9/D10. |
| GL10 ESC (FOC) | Convert that PWM command into smooth motor drive (phase currents, accel ramp, drag) — internal to the ESC. |

- **Open-loop command** — the Arduino does NOT read RPM and correct the output.
  Stick → PWM → ESC executes. No Arduino-side PID / feedforward / inertia.
- **Telemetry is monitoring-only** — X.BUS 0x10 is read-only for the dashboard,
  never fed back into throttle. No path from Wi-Fi to motor output (permanent).

## Pin Assignments (UNO R4 WiFi)

```text
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-5V direct (5V tolerant)]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-5V direct (5V tolerant)]
D11    (sbusUart TX on SCI0 — unused; S.BUS is RX-only)
D12 <- S.BUS RX (sbusUart on SCI0 via NPN inverter) [All RC channels at 100k 8E2]
D8     (reserved — future battery-aware beeper)
D9  -> Left Track ESC                       [Servo PWM, 50 Hz, 1000-2000 us]
D10 -> Right Track ESC                      [Servo PWM]
D0/D1 -> Serial1 (X.BUS half-duplex telemetry bus to both ESCs, 115200 8N1)
USB-C -> USB CDC Serial for debug + firmware upload
5V  -> RC Receiver + Joystick VCC + S.BUS inverter
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

### UART Architecture (UNO R4 WiFi)

The UNO R4 WiFi exposes Serial1 (SCI2 on D0/D1) plus a second UART that can be
instantiated on D11 (TX) / D12 (RX) via the RA4M1's SCI0 channel. The sketch
declares it under a unique name (NOT `Serial2`, which collides with the core's
pre-declared `_UART2_`, and is also reserved for the ESP32-S3 Wi-Fi bridge):

```cpp
const uint8_t PIN_SBUS_TX = 11;  // SCI0 TX (D11) — unused, S.BUS is RX-only
const uint8_t PIN_SBUS_RX = 12;  // SCI0 RX (D12) — inverted S.BUS in
UART sbusUart(PIN_SBUS_TX, PIN_SBUS_RX);
bfs::SbusRx sbusRx(&sbusUart);
```

| Port | Hardware | Pin | Function |
| --- | --- | --- | --- |
| Serial | USB CDC | (USB-C) | Debug telemetry / firmware upload |
| Serial1 | UART (SCI2) | D0 (RX) / D1 (TX) | X.BUS telemetry bus to both ESCs (115200 8N1) |
| sbusUart | UART (SCI0) | D11 (TX) / D12 (RX) | S.BUS RX from R7FG via inverter (100000 8E2) |

> **Note:** on the UNO R4 WiFi, SCI0 is on D11/D12 (the Nano R4 used A4/A5).
> S.BUS is electrically inverted at idle; an NPN-based inverter flips it so
> `sbusUart` sees standard UART polarity (idle HIGH). Full wiring:
> `docs/WIRING-GUIDE-V8.md`.

### Telemetry (live)

`types.h` defines `EscTelem { voltage, busCurrentA, rpmHz, escTempC, motorTempC,
lastGoodMs, valid }`. The `[TELEMETRY]` module polls it on Serial1 (D0/D1):

- **X.BUS Read Register (func 0x10), NOT Throttle (0x50).** 0x10 is
  point-to-point service control — it never puts the ESC into BUS_MODE, so PWM
  control authority is fully preserved. 0x50 would fight our PWM.
- **Registers:** 0x0C VbatBus (×0.1 V), 0x0D IbusBus (×0.1 A), 0x02 motSpeed
  (electrical Hz), 0x20 mos-Tem, 0x22 mot-Tem (raw − 40 °C).
- **Cadence:** non-blocking, alternating ESCs (~30-40 Hz/ESC underlying), EMA on
  V/I/temps, instantaneous RPM, 5 s per-ESC freshness watchdog. Both ESCs
  confirmed reporting (2026-06-13).
- **Standing rule:** never `SoftwareSerial` for telemetry — hardware UARTs only.

## Architecture Summary

Sketch: `sketches/rc_test/rc_test.ino` (V7.14 — GL10 FOC + telemetry + Wi-Fi + alarms) + `types.h` + `web_page.h`

Signal pipeline:

1. RC inputs: S.BUS on `sbusUart` (D12 RX via NPN inverter), all 16 channels
2. Joystick via 14-bit ADC (A0, A1, cached at 100 Hz) → deadband → expo curve (separate throttle/steering coefficients)
3. Both inputs → curvatureDrive() (symmetric add + desaturate, smoothstep blend into pivot mode):
   - At speed: inner track slows, outer track speeds up by the same delta — average wheel speed = xSpeed
   - At standstill: pivot mode counter-rotates the tracks, capped at PIVOT_SPEED_CAP (60%)
4. Override mode select (RC CH5: Mode 1=RC only, Mode 2=RC overrides joy, Mode 3=50/50 blend)
5. Gear cap (RC CH4): Eco 65% / Normal 80% / Boost 100% — caps the AVERAGE
   track speed (folded into curvatureDrive), so in a turn the outer track uses
   the gear→rail headroom and Eco/Normal hold speed through corners; Boost is at
   the rail. In Eco the pivot and reverse caps get a boost so the operator keeps
   usable maneuvering authority. Joystick throttle carries a ×1.05 gain (clamped
   to the gear cap).
6. Servo PWM out to GL10 ESCs on D9/D10 (50 Hz, 1000-2000 us). The FOC ESC owns command smoothing internally.
7. X.BUS 0x10 telemetry polled read-only on Serial1; EMA-smoothed and streamed to the Wi-Fi dashboard (monitoring only).

Code organized in searchable [MODULE] sections: [CONFIG], [DRIVE], [RC],
[JOYSTICK], [GEAR], [MIXER], [OUTPUT], [TELEMETRY], [WIFI], [DEBUG]. Search `[NAME]` to jump.

Loop rate: ~20,000 Hz (non-blocking), micros()-based timing.

## Wi-Fi telemetry dashboard

- UNO R4 WiFi AP `Digger-Telemetry` (WPA2) at `192.168.4.1`. The dashboard is
  embedded in flash (`web_page.h`, served at `/`) and streams telemetry over
  Server-Sent Events (`/events`) at ~5 Hz (`SSE_INTERVAL_MS = 200`); `/data` one-shot JSON kept for debug.
- **Monitoring only — no control input over Wi-Fi, ever** (safety; permanent
  decision). The dashboard mirror lives in `dashboard/index.html`.

## Audible Alerts — Implemented (V7.11–V7.12, active piezo on D8)

The V7.6-removed reverse beeper returned as a battery + inactivity alarm system.
An active piezo on **D8** is driven non-blocking by `[BEEPER]` / `[ALERT]`; the
horn ORs over one-shot patterns, which OR over repeating alarms. The original
percentage-based plan was implemented as a simpler **voltage** threshold:

| Event | Pattern | Trigger |
| --- | --- | --- |
| Wi-Fi ready | beep-beep (once at boot) | AP up |
| Horn | continuous tone (held) | RC CH7 SWD button (`HORN_ON_RAW`) |
| Inactivity ("unplug me") | one long beep / 2 s | RC off > 60 s (`INACT_RC_OFF_MS`) |
| Low battery | three fast chirps / ~1.2 s, **latched** | worse pack EMA < 10.5 V (`LOWV_THRESH_V`), debounced 3 s, suppressed first 60 s |

Alarms are **sound-only** — they do not stop or slow the motors. A low-battery
motor cutoff is still pending (issue #65). See OPERATOR-GUIDE.md for the
operator-facing beep table.

## Key Design Decisions

### GL10 FOC Replaces Custom PID Loops (DECIDED 2026-04-25)

The previous control plan layered current-feedforward + RPM-feedback PID on top
of the older sensored ESC. With the GL10's internal FOC, that whole stack moves
into the ESC. Arduino code keeps:

- Input shaping (expo, deadband, curvatureDrive)
- Mixing (override switch)
- Gear caps (Eco / Normal / Boost via RC CH4 — average-speed cap with turn headroom)

V7.2 removed the Arduino-side inertia filter (`applyInertia` / `TAU_*`): the
GL10's own Acceleration + Drag Force settings own command smoothing end-to-end,
and the Arduino-side filter was double-smoothing the stream (operator felt
"vehicle keeps coasting after stick release").

## Implementation Status

- [x] V5.0 sketch: curvatureDrive, S.BUS, expo, soft limits
- [x] GL10 + GL540L hardware swap (2026-04-25)
- [x] V7.0–V7.6: S.BUS on `sbusUart`, GL10 PWM control, tank curvature, gear spread, beeper removed
- [x] Migrated Nano R4 → UNO R4 WiFi (S.BUS on D11/D12, Serial1 free for X.BUS)
- [x] Soldered interface board (X.BUS merge + S.BUS inverter)
- [x] V7.7: X.BUS 0x10 telemetry integrated — both ESCs (V / I / RPM / temps)
- [x] V7.8: Wi-Fi telemetry dashboard (SSE, monitoring-only)
- [x] V7.9–V7.10: bounded modem timeout, coalesced SSE, AP ch11, page caching (#54)
- [x] V7.11–V7.12: beeper/horn + battery & inactivity alarms on D8 (#51, #68)
- [x] V7.13: loop watchdog + incremental Wi-Fi serving (#69)
- [x] V7.14: smooth pivot↔drive blend + Eco/Normal turn headroom + gear step-up + joystick gain (#72)
- [x] Field test at reduced power
- [ ] Wi-Fi serving latency mitigation (issue #54 — hardware-gated)
- [ ] Track-speed asymmetry investigation — per-ESC throttle calibration
- [ ] Low-battery motor cutoff (issue #65 — alarm is sound-only today)
- [ ] Current-sensor wiring/calibration on A2/A3 (issue #5)

## File Map

```text
PROJECT-PLAN.md                          — Full technical specification
OPERATOR-GUIDE.md                        — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino             — Main Arduino sketch (V7.14 — GL10 FOC + telemetry + Wi-Fi + alarms)
sketches/rc_test/types.h                 — Shared structs (JoystickState, EscTelem, ...)
sketches/rc_test/web_page.h              — Embedded Wi-Fi dashboard (PROGMEM), served at "/"
sketches/telem_check/telem_check.ino     — Read-only X.BUS telemetry bench tool (0x50 framing)
sketches/sbus_d12_test/sbus_d12_test.ino — S.BUS-on-D12 bring-up test
dashboard/index.html                     — Wi-Fi dashboard source (mirror of web_page.h)
docs/WIRING-GUIDE-V8.md                  — Canonical hardware wiring reference (UNO R4 WiFi)
docs/INTERFACE-BOARD-PERFBOARD.md        — Soldered interface board build
docs/GL10-Manual.pdf                     — XC-ESC official user manual (image-based, 3 pages)
docs/GL10-PARAMETERS.md                  — GL10 parameter reference + code-context analysis
docs/GL10-OPERATION.md                   — Startup, throttle calibration, factory reset, LED/beep reference
docs/XBUS-PROTOCOL.md                    — XC X.BUS protocol reference (used by live telemetry)
docs/CONTROL-RESEARCH.md                 — Tank mix, RC input, loop patterns research
docs/MISSION.md                          — Project design philosophy (smoothness above all)
docs/DECISION-LOG.md                     — Technical decision log
live_plot.py                             — Real-time matplotlib monitor
monitor.py                               — Simple serial monitor
```

## Coding Rules

### Architecture

- All code in `rc_test.ino` is organized in `[MODULE]` sections — search `[NAME]` to jump
- Structs go in `types.h` (solves Arduino auto-prototype limitation)
- All tunable constants go in the `[CONFIG]` section — no magic numbers in code
- When the sketch exceeds ~500 lines, split into `.h/.cpp` pairs (not multiple `.ino` files)

### Real-Time Safety

- **No blocking calls in the main loop** — no `delay()`, no `pulseIn()`, no `while` loops
- Use `micros()` fresh at point of use — never capture before a blocking call and use after
- Per-channel failsafe — each RC channel has an independent timeout, never combined
- ISRs must be under 10us — read pin, store timestamp, exit

### Telemetry

- **Never** use `SoftwareSerial` for telemetry input. Hardware UARTs only.
  Bit-banged debug TX is OK if needed.

### Style

- Use `float` with `f` suffix for FPU (`1.0f` not `1.0`) — `double` is software-emulated
- Use `constrain()` at all servo output boundaries
- Comment each `[MODULE]` section header with a brief description
- Commit messages: `V{major}.{minor}: {imperative verb} {what changed}`

### ESC / motor configuration changes

Every ESC parameter (XC-Link Bluetooth app, X.BUS register, programming
card) must be evaluated against **what PWM commands the Arduino code
actually sends** before recommending a value. Per-direction caps,
asymmetric scalings, and brake limits can silently clip legitimate
commands and break features.

Concrete example: `curvatureDrive` in pivot mode commands one track
forward and the other in reverse (e.g. left = +55%, right = -55%) at
high gear. The GL10's factory default `Max Reverse Force = 50%` would
deliver only -27.5% on the reverse track, collapsing the pivot
differential. Setting it to 100% lets pivot work as designed.

Before any XC-Link parameter change:

1. List every command path that drives each motor (RC throttle, RC
   pivot/curvature, joystick equivalents, gear scaling, override mixer).
2. Identify the per-direction min/max each path can produce.
3. Verify the proposed setting does not clip, scale, or distort any of
   those commands in a way that breaks an intended behavior.
4. Call out cross-impacts explicitly with the specific feature affected
   — never just hand over a value.

See `docs/GL10-PARAMETERS.md` for the full GL10 parameter list with
per-parameter code-context analysis.

## Build & Upload

- Board: Arduino UNO R4 WiFi
- FQBN: `arduino:renesas_uno:unor4wifi`
- Board package: `arduino:renesas_uno`
- Port: COM7
- Serial: 115200 baud
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
- Upload: `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/rc_test && arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi -p COM7 sketches/rc_test`
