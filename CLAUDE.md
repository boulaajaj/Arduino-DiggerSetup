# Excavator Track Controller — Project Memory

## What This Is

Arduino Nano R4-based tank-style track controller for a ride-on excavator.
Two FOC brushless motors drive rubber tracks via FOC ESCs. Dual input:
RC transmitter (Jason) and joystick (Malaki/rider). 3-position override
switch selects who has authority.

## People

- **Jason** — RC transmitter operator (safety supervisor)
- **Malaki** — Rider/operator using the joystick
- **boulaajaj** — GitHub owner / builder

## Hardware Stack

| Component | Model | Key Detail |
| --- | --- | --- |
| Controller | Arduino Nano R4 | Renesas RA4M1, 48MHz, 5V tolerant GPIO, 14-bit ADC |
| ESC (x2) | XC GL10 80A FOC | Standard 50 Hz servo PWM input, internal FOC compensation, IP67 |
| Motor (x2) | XC GL540L | Sensored brushless, paired with GL10 |
| Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| RC System | Radiolink RC6GS V3 + R7FG | 6CH, transmitter does tank mixing |
| Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect (direct to ADC — 5V tolerant) |
| Beeper | Active buzzer | Driven directly from D8 (HIGH = on) |

**Hardware history:** Previously used XC E10 ESC + XC E3665 motor.
Swapped to GL10 + GL540L on 2026-04-25. The FOC ESC handles motor
acceleration compensation and smoothness internally, so the Arduino's
job shrinks to input mixing, override switching, soft limits, and
beeper alerts. The earlier dual-loop PID / hall-sensor-tap / X.BUS
control plans are no longer needed for control — see "Telemetry"
below for what's deferred.

## Pin Assignments (Nano R4)

```text
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-5V direct (5V tolerant)]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-5V direct (5V tolerant)]
A4     (sbusUart TX on SCI0, currently unused)
A5  <- S.BUS RX (sbusUart on SCI0 via NPN inverter) [All RC channels at 100k 8E2]
D8  -> Beeper (active buzzer, direct GPIO) [HIGH = sounding]
D9  -> Left Track ESC                       [Servo PWM, 50 Hz, 1000-2000 us]
D10 -> Right Track ESC                      [Servo PWM]
D0/D1 — Serial1 hardware UART (currently unused; S.BUS moved off it)
USB-C — USB CDC Serial for debug + firmware upload
5V  -> RC Receiver + Joystick VCC + S.BUS inverter
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

### UART Architecture (Nano R4)

The Nano R4 exposes one stock hardware UART (Serial1 on D0/D1) plus a
second UART that can be instantiated on A4 (TX, pin 18) / A5 (RX, pin 19)
via the RA4M1's SCI0 channel. The sketch declares it under a unique
name (NOT `Serial2`) to avoid the core's macro collision with the
pre-declared `_UART2_` symbol:

```cpp
UART sbusUart(18, 19);            // A4 TX (unused), A5 RX (S.BUS)
bfs::SbusRx sbusRx(&sbusUart);
```

| Port | Hardware | Pin | Function |
| --- | --- | --- | --- |
| Serial | USB CDC | (USB-C) | Debug telemetry / firmware upload |
| Serial1 | UART (SCI2) | D0 (RX) / D1 (TX) | Currently unused |
| sbusUart | UART (SCI0) | A4 (TX) / A5 (RX) | S.BUS RX from R7FG via inverter |

S.BUS is electrically inverted at idle. A small NPN-based inverter
circuit on the receiver wire flips the signal so `sbusUart` sees standard
UART polarity (idle HIGH). Same inverter circuit that was on D0 in
V5.0/V6.0 — just relocated to A5. Library config is unchanged
(100000 baud, 8E2, courtesy of `bfs::SbusRx`).

### Telemetry (deferred)

`types.h` defines `EscTelem { voltage, motorTempC, escTempC, lastGoodMs, valid }`
and `BeepPattern { BAT_30, BAT_20, OVERTEMP, ... }`, but no telemetry is
polled in V7. With S.BUS occupying `sbusUart` on SCI0, the only remaining
hardware UART (`Serial1` on D0/D1, SCI2) is available but unused, and the
standing rule is "never SoftwareSerial for telemetry".
The plan when telemetry comes back online:

- **Use X.BUS Read Register (func 0x10), NOT Throttle (0x50).** Throttle
  frames put the ESC into BUS_MODE and override our PWM — incompatible
  with the GL10's PWM input strategy.
- **Registers to poll:** 0x0C VbatBus (×0.1 V), 0x20 mos-Tem (raw − 40 °C),
  0x22 mot-Tem (raw − 40 °C).
- **Cadence:** 1 Hz, alternating between the two ESCs (each sampled every 2 s),
  EMA filter with 10 s time constant. After 5 s with no good frame the
  reading is marked stale → battery/temp beeps suppressed (no false
  alarms), reverse beep still works.
- **Hardware path TBD** — `Serial1` (D0/D1) is free, so X.BUS telemetry
  can land there if the wiring is feasible. Alternative is to relocate
  S.BUS off `sbusUart` and reuse SCI0.

## Architecture Summary

Sketch: `sketches/rc_test/rc_test.ino` (V7.5 — GL10 FOC) + `types.h`

Signal pipeline:

1. RC inputs: S.BUS on `sbusUart` (A5 RX via NPN inverter), all 16 channels
2. Joystick via 14-bit ADC (A0, A1, cached at 100 Hz) → deadband → expo curve (separate throttle/steering coefficients)
3. Both inputs → curvatureDrive() (symmetric add + desaturate, smoothstep blend into pivot mode):
   - At speed: inner track slows, outer track speeds up by the same delta — average wheel speed = xSpeed
   - At standstill: pivot mode counter-rotates the tracks, capped at PIVOT_SPEED_CAP (60%)
4. Override mode select (RC CH5: Mode 1=RC only, Mode 2=RC overrides joy, Mode 3=50/50 blend)
5. Gear cap (RC CH4): Eco 40% / Normal 65% / Turbo 100% wheel-speed cap.
   In Eco the pivot and reverse caps get a +5pp boost so the operator
   keeps usable maneuvering authority.
6. Servo PWM out to GL10 ESCs on D9/D10 (50 Hz, 1000-2000 us). The FOC ESC owns command smoothing internally.
7. Beeper update on D8 — reverse alarm whenever output < SVC − 50us

Code organized in searchable [MODULE] sections: [CONFIG], [DRIVE], [RC],
[JOYSTICK], [MIXER], [BEEPER], [OUTPUT], [DEBUG]. Search
`[NAME]` to jump.

Loop rate: ~20,000 Hz (non-blocking), micros()-based timing.

## Beeper Patterns

Priority is encoded in the `BeepPattern` enum value (higher = more critical).
Each loop, `selectBeepPattern()` returns the highest-priority condition that
applies; `beeperUpdate()` plays it non-blocking against `millis()`.

| Pattern | Trigger | Cadence |
| --- | --- | --- |
| BEEP_REVERSE | Either ESC output < (SVC − 50us) | 1s on, 1s off (backup-alarm) |
| BEEP_BATTERY_30 | (deferred — needs telemetry) | 200ms chirp every 10s |
| BEEP_BATTERY_20 | (deferred — needs telemetry) | Double chirp every 1s |
| BEEP_OVERTEMP | (deferred — needs telemetry) | Rapid double chirp |

Only `BEEP_REVERSE` is wired in V7. The others are scaffolded for when
telemetry comes back online.

## Key Design Decisions

### GL10 FOC Replaces Custom PID Loops (DECIDED 2026-04-25)

The previous control plan layered current-feedforward + RPM-feedback PID
on top of the E10 ESC. With the GL10's internal FOC, that whole stack
moves into the ESC. Arduino code keeps:

- Input shaping (expo, deadband, curvatureDrive)
- Mixing (override switch)
- Gear caps (Eco 40% / Normal 65% / Turbo 100% via RC CH4)
- Beeper alerts

V7.2 removed the Arduino-side inertia filter (`applyInertia` / `TAU_*`):
the GL10's own Acceleration + Drag Force settings now own command
smoothing end-to-end, and the Arduino-side filter was double-smoothing
the stream (operator felt "vehicle keeps coasting after stick release").

The earlier feedforward tables (V7.0 on `claude/ff-trim-controller`)
were built from plant characterization of the E10/E3665 combo. They
don't apply to GL10/GL540L and are not used.

## Implementation Status

- [x] V5.0 sketch: curvatureDrive, S.BUS, expo, inertia, soft limits
- [x] V6.0 (main): X.BUS closed-loop RPM with curvatureDrive (E10/E3665 era)
- [x] GL10 + GL540L hardware swap (2026-04-25)
- [x] V7.0 sketch: S.BUS on `sbusUart`/A5, beeper on D8, GL10 PWM control
- [x] V7.1 tuning: tank-style curvature, pivot authority, expo split
- [x] V7.2: removed Arduino-side inertia filter (ESC owns smoothing)
- [x] V7.3: symmetric-add + desaturate in curvatureDrive (hold speed through turns)
- [x] V7.4: steering polarity fix + reverse cap + Eco/Normal/Turbo gear spread
- [x] V7.5: Eco-gear +5pp boost on reverse and pivot caps
- [x] Field test at reduced power
- [ ] X.BUS telemetry on `Serial1` (D0/D1) — voltage / temp / RPM (see issue #36)
- [ ] Track-speed asymmetry investigation — per-ESC throttle calibration
- [ ] Migrate to UNO R4 WiFi for wireless telemetry (issue #6)

## File Map

```text
PROJECT-PLAN.md                          — Full technical specification
OPERATOR-GUIDE.md                        — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino             — Main Arduino sketch (V7.5 — GL10 FOC)
sketches/rc_test/types.h                 — Shared structs (JoystickState, BeepPattern, EscTelem, ...)
sketches/serial2_test/serial2_test.ino   — Confirms second UART on A4/A5 via SCI0 works
sketches/xbus_master/xbus_master.ino     — X.BUS master test sketch (deferred — kept for reference)
docs/GL10-Manual.pdf                     — XC-ESC official user manual (image-based, 3 pages)
docs/GL10-PARAMETERS.md                  — Configurable parameter reference + code-context analysis
docs/GL10-OPERATION.md                   — Startup, throttle calibration, factory reset, LED/beep reference
docs/XBUS-PROTOCOL.md                    — Official XC X.BUS protocol reference (translated)
docs/XBUS-INVESTIGATION.md               — X.BUS investigation timeline and lessons learned
docs/CONTROL-RESEARCH.md                 — Tank mix, RC input, loop patterns research
docs/WIRING-GUIDE.md                     — Hardware wiring reference
docs/MISSION.md                          — Project design philosophy (smoothness above all)
docs/PLANT-CHARACTERIZATION.md           — Measured plant response (E10/E3665 era, kept for reference)
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

- Board: Arduino Nano R4
- FQBN: `arduino:renesas_uno:nanor4`
- Board package: `arduino:renesas_uno`
- Port: COM8
- Serial: 115200 baud
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
- Upload: `arduino-cli compile --fqbn arduino:renesas_uno:nanor4 && arduino-cli upload --fqbn arduino:renesas_uno:nanor4 -p COM8`
