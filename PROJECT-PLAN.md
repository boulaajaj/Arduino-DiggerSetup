# Excavator Track Controller — Project Plan

## What This Does

An **Arduino UNO R4 WiFi** sits between an RC receiver and two FOC ESCs,
implementing a **tank-style mixer** with a secondary joystick input for the
rider. The RC transmitter (Jason) can always override via a 3-position switch.

The controller's job is deliberately small: **read inputs, shape them, mix them,
cap them by gear, and output a servo-PWM throttle/reverse command to each ESC.**
The GL10 ESC's internal FOC owns motor smoothness and compensation, so there is
**no Arduino-side closed-loop control**.

The controller provides:

- **Curvature/tank mixing** — symmetric add + desaturate, smoothstep blend into pivot
- **Exponential response curve** — separate throttle/steering expo for fine low-speed control
- **Gear caps** — Eco / Normal / Boost average-speed limits on RC CH4 (outer track keeps turn headroom)
- **Override switch** — RC-only / RC-priority / 50-50 blend on RC CH5
- **Per-channel failsafe** — neutral hold when S.BUS is invalid
- **X.BUS telemetry (monitoring only)** — V / I / RPM / temps streamed to a Wi-Fi dashboard

---

## Control strategy (read this first)

**The Arduino sends the throttle/reverse command as servo PWM. The GL10's FOC
executes it.** Two chips, two jobs:

| Who | Job |
| --- | --- |
| **Arduino (RA4M1)** | Decides *what we want*: read RC + joystick → expo/deadband → curvatureDrive mix → gear cap → output servo PWM (1000–2000 µs) per track on D9/D10. |
| **GL10 ESC (internal FOC)** | Decides *how to spin the motor* to hit that command smoothly: phase currents, acceleration ramp, drag — all inside the ESC. |

- **Open-loop command.** The Arduino does **not** read RPM and correct the
  output. Stick position → PWM → ESC executes. The earlier dual-loop current+RPM
  PID / feedforward / inertia filter (from the previous sensored-ESC setup) is
  **gone** — superseded by the GL10's internal FOC.
- **Telemetry is monitoring-only.** X.BUS Read Register (0x10) is polled
  read-only for the dashboard; it is never fed back into the throttle, and there
  is no path from Wi-Fi to motor output.

---

## Hardware

| # | Component | Model | Key Specs |
| --- | ----------- | ------- | ----------- |
| 1 | Controller | Arduino **UNO R4 WiFi** | Renesas RA4M1 (48 MHz, 14-bit ADC, 5 V tolerant) + ESP32-S3 (Wi-Fi) |
| 2 | Battery (x2) | OVONIC 3S LiPo 15000 mAh 130C | 11.1 V, EC5 |
| 3 | ESC (x2) | **XC GL10 80A FOC** | Standard 50 Hz servo PWM in, internal FOC, IP67 |
| 4 | Motor (x2) | **XC GL540L** | Sensored brushless, paired with GL10 |
| 5 | RC System | Radiolink RC6GS V3 + R7FG | 6CH, S.BUS; gun-style (trigger = throttle, wheel = steering) — the **Arduino** does the tank mixing |
| 6 | Joystick | Genie 101174GT dual-axis | 5 V, 0-5 V analog, hall-effect (direct to ADC) |

> **History:** the project previously ran XC sensored ESCs/motors with an
> Arduino-side dual-loop PID. Swapped to GL10 + GL540L on 2026-04-25, then
> migrated from Nano R4 to UNO R4 WiFi for onboard Wi-Fi telemetry.

---

## Pin Map (UNO R4 WiFi)

```text
A0  ← Joystick Y (throttle)            [14-bit ADC, 0-5V direct (5V tolerant)]
A1  ← Joystick X (steering)            [14-bit ADC, 0-5V direct (5V tolerant)]
D11    (sbusUart TX on SCI0 — unused; S.BUS is RX-only)
D12 ← S.BUS RX (sbusUart on SCI0 via NPN inverter)   [100k 8E2]
D8     (reserved — future battery-aware beeper)
D9  → Left Track ESC                   [Servo PWM, 50 Hz, 1000-2000 us]
D10 → Right Track ESC                  [Servo PWM]
D0/D1 → Serial1 (X.BUS half-duplex telemetry bus to both ESCs, 115200 8N1)
USB-C → USB CDC Serial (debug + firmware upload)
VIN ← BEC / battery (7-24V)
GND → All components (common ground)
```

Full wiring spec: `docs/WIRING-GUIDE-V8.md`. Interface board: `docs/INTERFACE-BOARD-PERFBOARD.md`.

---

## Signal Pipeline

```text
RC (S.BUS) ──► curvatureDrive ──┐
                                ├─► override mixer ─► gear cap ─► servo PWM (D9/D10) ─► GL10 ESCs
Joystick (ADC) ──► curvatureDrive ─┘

X.BUS telemetry (Serial1, read-only) ──► EMA ──► Wi-Fi dashboard (monitoring only)
```

### Override Switch (RC CH5)

| Position | Mode | Who Controls |
| --- | --- | --- |
| LOW | RC only | Jason has full authority, joystick disabled |
| MID | RC priority | RC overrides when non-neutral |
| HIGH | 50/50 blend | RC + joystick averaged |

### Gear Caps (RC CH4)

The gear caps the **average** track speed (not each wheel). In a turn the outer
track uses the headroom up to the ±1.0 ESC rail, so Eco/Normal hold their speed
through corners; Boost is already at the rail.

| Position | Avg-speed cap | Use |
| --- | --- | --- |
| Eco | 65% | training / tight spaces (~35% turn headroom; reverse & pivot get extra authority) |
| Normal | 80% | normal driving — the everyday gear (~20% turn headroom) |
| Boost | 100% | full authority (at the rail) |

Reverse is capped at 50% of forward stick travel (62.5% in Eco); pivot at 60%
(72.5% in Eco). The pivot↔drive hand-off smoothstep-blends across |throttle|
5–55% so there's no snap at low speed. The joystick throttle carries a ×1.05
gain (clamped to the gear cap at full deflection).

> Failsafe: when S.BUS is invalid, the controller holds neutral and gear stays at Eco.

---

## Telemetry (live, monitoring-only)

- **X.BUS Read Register (func 0x10)** on Serial1 (D0/D1) — NOT Throttle (0x50),
  which would put the ESC into BUS_MODE and fight our PWM.
- **Registers:** 0x0C VbatBus (×0.1 V), 0x0D IbusBus (×0.1 A), 0x02 motSpeed
  (electrical Hz), 0x20 mos-Tem, 0x22 mot-Tem (raw − 40 °C).
- **Cadence:** non-blocking, alternating ESCs (~30-40 Hz/ESC underlying), EMA on
  V/I/temps, instantaneous RPM, 5 s per-ESC freshness watchdog.
- **Wi-Fi dashboard:** UNO R4 WiFi AP `Digger-Telemetry` at `192.168.4.1`,
  Server-Sent Events at ~5 Hz (`SSE_INTERVAL_MS = 200`), page served from flash.
  **Monitoring only — no control input over Wi-Fi, ever.**

---

## Build & Upload

```bash
# Compile
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/rc_test

# Upload (COM7)
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi -p COM7 sketches/rc_test

# Monitor / plot
python monitor.py
python live_plot.py
```

---

## Status (V7.14)

### Done

- [x] GL10 + GL540L hardware swap (2026-04-25)
- [x] S.BUS input on `sbusUart` (SCI0 / D12) via NPN inverter
- [x] curvatureDrive tank mixing + per-axis expo + pivot authority
- [x] Removed Arduino-side inertia / PID (GL10 FOC owns smoothing)
- [x] Gear caps (Eco 65% / Normal 80% / Boost 100%) capping average speed, outer-track turn headroom
- [x] Eco reverse/pivot boost; ×1.05 joystick throttle gain
- [x] Smooth pivot↔drive blend (wide 5–55% band) — no snap at low throttle (#72)
- [x] Migrated Nano R4 → UNO R4 WiFi
- [x] Soldered interface board (X.BUS merge + S.BUS inverter)
- [x] X.BUS 0x10 telemetry live — both ESCs (V / I / RPM / temps)
- [x] Wi-Fi telemetry dashboard (SSE ~5 Hz, monitoring-only)
- [x] Battery + inactivity alarms on D8 (V7.12)
- [x] Loop watchdog — runaway/stall failsafe drops ESCs to neutral (V7.13)
- [x] Field test at reduced power

### Pending

- [ ] Wi-Fi serving latency mitigation (issue #54 — hardware-gated)
- [ ] Track-speed asymmetry investigation — per-ESC throttle calibration
- [ ] Low-battery motor cutoff (issue #65 — alarm is sound-only today)
- [ ] Current-sensor wiring/calibration on A2/A3 (issue #5)

---

## File Map

```text
sketches/rc_test/
  rc_test.ino       — Main controller sketch (V7.14 — GL10 FOC + telemetry + Wi-Fi + alarms)
  types.h           — Shared structs (JoystickState, EscTelem, ...)
  web_page.h        — Embedded Wi-Fi dashboard (PROGMEM), served at "/"
sketches/telem_check/ — Read-only X.BUS telemetry bench tool (0x50 framing)
sketches/sbus_d12_test/ — S.BUS-on-D12 bring-up test
docs/WIRING-GUIDE-V8.md        — Canonical hardware wiring reference (UNO R4 WiFi)
docs/INTERFACE-BOARD-PERFBOARD.md — Soldered interface board build
docs/XBUS-PROTOCOL.md          — XC X.BUS protocol reference (used by telemetry)
docs/GL10-PARAMETERS.md        — GL10 parameter reference + code-context analysis
docs/DECISION-LOG.md           — Technical decision log
dashboard/index.html           — Wi-Fi dashboard source (mirror of web_page.h)
live_plot.py / monitor.py      — Serial monitors
PROJECT-PLAN.md                — This file
```
