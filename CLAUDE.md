# Excavator Track Controller — Project Memory

## Mission — read this first

**Smoothness of the throttle command is the top priority, above
responsiveness, efficiency, and closed-loop accuracy.** The Arduino must
send a continuous, slew-limited throttle stream that passes through
every percentage value in sequence. Never skip any part of the range —
not even a "dead zone" where the current motor happens not to respond.
Plant characteristics belong in the feed-forward mapping, never in the
output layer.

Full rationale, priority order, and examples of past misalignment:
[docs/MISSION.md](docs/MISSION.md). Read that document before
designing or reviewing any throttle-output change.

## What This Is

Arduino Nano R4-based tank-style track controller for a ride-on excavator.
Two brushless motors drive rubber tracks via ESCs. Dual input: RC transmitter
(Jason) and joystick (Malaki/rider). 3-position override switch selects who
has authority.

## People

- **Jason** — RC transmitter operator (safety supervisor)
- **Malaki** — Rider/operator using the joystick
- **boulaajaj** — GitHub owner / builder

## Hardware Stack

| Component | Model | Key Detail |
| --- | --- | --- |
| Controller | Arduino Nano R4 | Renesas RA4M1, 48MHz, 5V tolerant GPIO, 14-bit ADC |
| ESC (x2) | XC E10 Sensored Brushless 140A | Servo PWM input, sensored mode |
| Motor (x2) | XC E3665 2500KV Sensored Brushless | 4-pole, 8mm shaft, hall sensor cable to ESC |
| Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| RC System | Radiolink RC6GS V3 + R7FG | 6CH, transmitter does tank mixing |
| Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect (direct to ADC — 5V tolerant) |
| Current Sensor (x2) | CS7581 Hall-effect | Per-motor current monitoring |

## Pin Assignments (Nano R4)

```text
D0  <- S.BUS input (Serial1 RX)           [All RC channels via NPN inverter]
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-5V direct (5V tolerant)]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-5V direct (5V tolerant)]
A2  <- CS7581 Current Sensor (Left)        [14-bit ADC, future]
A3  <- CS7581 Current Sensor (Right)       [14-bit ADC, future]
D5     (available)                          [reserved: hall sensor tap if needed]
D6     (available)                          [reserved: hall sensor tap if needed]
D9  -> Left Track ESC                      [Servo PWM output]
D10 -> Right Track ESC                     [Servo PWM output]
5V  -> RC Receiver + Joystick VCC
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

**Future X.BUS pin assignment (when integrated):**

```text
D0  <- X.BUS shared bus (Serial RX)        [Half-duplex, direct — no inverter]
D1  -> X.BUS shared bus (Serial TX)        [Through 1K series resistor]
```

Note: X.BUS on D0/D1 conflicts with both USB Serial and S.BUS (Serial1).
The final architecture will need to resolve this — either use S.BUS for RC
input and X.BUS on a separate UART, or migrate RC to X.BUS-only control.

### UART Architecture (Nano R4)

The Nano R4 has a single hardware UART on D0/D1, which is also the USB
Serial connection. Standard Arduino `Serial` works over USB — no Router
Bridge, no Monitor object, no special workarounds needed.

**X.BUS Protocol (CONFIRMED 2026-04-14):** XC E10 "X.BUS" is a proprietary
modbus-like master-slave protocol by Shenzhen XC-ESC Technology Co., Ltd.
NOT Spektrum X-Bus, NOT JR PROPO XBUS — completely unrelated protocols that
share the name. Half-duplex UART on a single wire, 115200 baud, 8N1,
little-endian, non-inverted (standard UART polarity, idle HIGH).
Both ESCs share one bus (addressable 0-15). The Arduino acts as master —
**ESCs never transmit unless polled**. Full protocol spec: `docs/XBUS-PROTOCOL.md`.

| Port | Hardware | Pin | Function |
| --- | --- | --- | --- |
| Serial | UART | D0 (RX) / D1 (TX) | USB Serial AND X.BUS (shared, cannot use both simultaneously) |

**D0 conflict:** X.BUS on D0 conflicts with USB Serial. During bench
testing, unplug X.BUS from D0 to use Serial Monitor for debugging. Plug
X.BUS back for field operation. No code change needed — just a wire swap.

## Architecture Summary

Sketch: `sketches/rc_test/rc_test.ino` (V5.0 — Curvature Drive) + `types.h`

Signal pipeline:

1. RC inputs: S.BUS on Serial1 (D0 via NPN inverter), all 16 channels
2. Joystick via 14-bit ADC (A0, A1, cached at 100Hz) → deadband → expo curve
3. Both inputs → curvatureDrive() (WPILib algorithm):
   - At speed: steering slows inner track only, outer holds speed
   - At standstill: arcade-style pivot turns (45% cap)
4. Override mode select (RC CH5: Mode 1=RC only, Mode 2=RC overrides joy, Mode 3=50/50 blend)
5. Reverse speed limiter (35% of forward max, straight-line only)
6. Soft power scaling (tanh saturation)
7. Inertia simulation (asymmetric exponential: 0.3s accel, 0.5s decel)
8. Servo output to ESCs (D9, D10)

Code organized in searchable [MODULE] sections: [CONFIG], [DRIVE], [RC],
[JOYSTICK], [MIXER], [DYNAMICS], [OUTPUT], [DEBUG]. Search `[NAME]` to jump.

Loop rate: ~20,000 Hz (non-blocking), micros()-based timing.

## Key Design Decisions

### PID Compensates by BOOSTING, Not Reducing

When a track hits resistance (mud, cold rubber, uphill), it draws more current
than expected. The PID **increases** the ESC command to push through and
maintain speed. It does NOT reduce power — that would make the machine slow
down or stall when it encounters any load.

### RPM Feedback via Motor Hall Sensor Tap (DECIDED 2026-03-20)

**Decision: Tap the motor's existing hall sensor cable for RPM feedback.**

Why RPM instead of current-only PID:

- Current is a proxy for load, not speed. The PID needs to know actual motor
  speed to maintain consistent track velocity under varying conditions.
- The XC E3665 motors are **sensored** — they already have hall-effect sensors
  inside. A 6-pin cable carries Hall A/B/C signals from motor to ESC for
  commutation.
- These signals are 5V logic (powered by the ESC's 5V rail).

Why tap the motor hall cable (not sprocket RPM):

- Measuring at the motor gives high resolution (many pulses/rev)
- No gearbox lag — instant feedback, no backlash delay
- The signal is already there — just wire in parallel

Why not Bluetooth telemetry from the ESC:

- 50-200ms latency — too slow for real-time PID at 20kHz loop
- Proprietary protocol, poorly documented
- Needs extra BT module on the Arduino side
- Way more code complexity for worse results

### Hall Sensor Tap — Technical Details

**Motor sensor cable (6-pin JST-ZH, standard Hobbywing pinout):**

| Pin | Signal |
| --- | --- |
| 1 | 5V (from ESC) |
| 2 | Hall A |
| 3 | Hall B |
| 4 | Hall C |
| 5 | Temperature / NC |
| 6 | GND |

**Wiring — parallel tap (non-invasive):**

```text
Motor Hall A ──────┬──── ESC Hall A
                   │
              Arduino D5 (direct — 5V tolerant)

Motor Hall B ──────┬──── ESC Hall B
                   │
              Arduino D6 (direct — 5V tolerant)

Motor GND ─────────┬──── ESC GND
                   │
              Arduino GND
```

**No level shifting needed!**
The Nano R4 has 5V tolerant GPIO. Hall sensor outputs are 5V and can
connect directly to D5/D6.

**Resolution (4-pole motor, 2 pole pairs):**

- 2 electrical revolutions per mechanical revolution
- Counting rising edges on 2 hall lines = 4 edges per motor revolution
- At 5,000 RPM (loaded): ~333 edges/sec — easy for Arduino interrupts
- At 100 RPM: ~7 edges/sec — still workable

**RPM calculation:**

```text
edgesPerRevolution = 4  // 2 hall lines × 2 pole pairs
RPM = (edgeCount / edgesPerRevolution) / elapsed_seconds * 60
```

Or interval-based (better at low RPM):

```text
RPM = 60,000,000 / (microsBetweenEdges * edgesPerRevolution)
```

### Dual-Loop PID Architecture (DECIDED 2026-03-21)

**Both current AND RPM feedback are used simultaneously.**

- **Inner loop (feedforward):** CS7581 current sensors detect load spikes instantly.
  Current spikes BEFORE RPM drops. Pre-compensates by boosting power before the
  operator feels anything.
- **Outer loop (feedback):** RPM measurement (X.BUS or hall sensor) confirms actual
  motor speed. Fine-tunes to hold target RPM.
- This is standard industrial motor control: inner current + outer speed loop.

### RPM Source: X.BUS Telemetry (UPDATED 2026-04-14)

X.BUS is the **primary RPM source**. The protocol provides RPM in Hz as part
of its telemetry response (function code 0x50). No hall sensor tap or
additional hardware needed — RPM comes over the same wire used for throttle.

- **Both ESCs:** X.BUS Yellow wires joined → shared bus → D0 (Serial RX) + D1 (Serial TX)
- **Bus wiring:** TX through 1K series resistor, RX direct, 3K-10K pull-up to 5V
- **No NPN inverter** — X.BUS is standard UART (non-inverted), unlike S.BUS
- **Fallback:** Hall sensor tap on D5/D6 remains available if X.BUS latency is insufficient
- **Test with:** `sketches/xbus_master/xbus_master.ino`
- **Full protocol spec:** `docs/XBUS-PROTOCOL.md`
- **Investigation history:** `docs/XBUS-INVESTIGATION.md`

### Control Mode Selector (RC CH4 on D3, 3-position switch)

| CH4 | Mode | Layers Active |
| --- | --- | --- |
| LOW | RAW | Direct stick→ESC, no processing (baseline) |
| MID | RPM_ONLY | Outer RPM PID only (speed regulation, no smoothing) |
| HIGH | FULL_STACK | Current FF + RPM + expo + inertia + soft limits |

Flip CH4 on the field to A/B/C compare control quality in real-time.
Mode transitions reset all PID/inertia state to prevent jumps.

## Implementation Status

- [x] V5.0 sketch: curvatureDrive, S.BUS, expo, inertia, soft limits
- [x] PID load compensation design (current-based, boosts under resistance)
- [x] Anti-runaway failsafe (S.BUS timeout + RC lockout)
- [x] 7-panel live plot (live_plot.py)
- [x] Operator guide for Jason and Malaki
- [x] X.BUS protocol investigation (passive, inverted, active polling)
- [x] X.BUS official protocol received from XC-ESC Technology
- [x] X.BUS protocol translated and documented (docs/XBUS-PROTOCOL.md)
- [x] X.BUS master test sketch written (sketches/xbus_master/)
- [ ] **X.BUS bench test with correct protocol** ← NEXT
- [ ] X.BUS integration into main controller (replace Servo PWM with X.BUS throttle)
- [ ] Dual-loop PID implementation (X.BUS telemetry provides RPM + current)
- [ ] CS7581 current sensor wiring and calibration
- [ ] Field test at reduced power

## Next Steps

1. **Build half-duplex bus circuit** (simplified for Nano R4):
   - ESC X.BUS Yellow → shared bus
   - Arduino D1 (TX) → 1K resistor → bus
   - Arduino D0 (RX) → bus (direct, no inverter)
   - 3K-10K pull-up to 5V on the bus
   - **No NPN inverter** — X.BUS is non-inverted standard UART
2. **Upload `sketches/xbus_master/xbus_master.ino`** to Nano R4 (COM8)
3. **Test with one ESC first** — verify:
   - Protocol handshake (0x0F/0xF0 headers)
   - Checksum interpretation (narrow vs full)
   - Telemetry response timing (<2ms)
   - Status bit 3 (BUS_MODE) confirms X.BUS control active
   - Status bit 11 (CAP_CHARGED) before sending throttle
4. **Test with both ESCs** — verify addressed bus with 2 slaves
5. **Integrate into main controller** — replace Servo PWM with X.BUS throttle,
   use telemetry RPM for PID outer loop
6. **Goal:** All control AND telemetry via X.BUS — one wire per ESC replaces
   Servo PWM + hall sensors + current sensors

## File Map

```text
PROJECT-PLAN.md                          — Full technical specification
OPERATOR-GUIDE.md                        — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino             — Main Arduino sketch (V5.0 — Curvature Drive)
sketches/rc_test/types.h                 — Shared structs (JoystickState, MixerOutput, etc.)
sketches/xbus_master/xbus_master.ino     — X.BUS master protocol test sketch
sketches/xbus_probe/xbus_probe.ino       — X.BUS passive probe (historical — used wrong protocol)
sketches/xbus_poll_test/xbus_poll_test.ino — X.BUS active polling test (historical)
docs/XBUS-PROTOCOL.md                   — Official XC X.BUS protocol reference (translated)
docs/XBUS-INVESTIGATION.md              — X.BUS investigation timeline and lessons learned
docs/CONTROL-RESEARCH.md                — Tank mix, RC input, loop patterns research
docs/WIRING-GUIDE.md                    — Hardware wiring reference
live_plot.py                             — Real-time matplotlib monitor
monitor.py                              — Simple serial monitor
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

### Style

- Use `float` with `f` suffix for FPU (`1.0f` not `1.0`) — `double` is software-emulated
- Use `constrain()` at all servo output boundaries
- Comment each `[MODULE]` section header with a brief description
- Commit messages: `V{major}.{minor}: {imperative verb} {what changed}`

## Build & Upload

- Board: Arduino Nano R4
- FQBN: `arduino:renesas_uno:nanor4`
- Board package: `arduino:renesas_uno`
- Port: COM8
- Serial: 115200 baud
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
- Upload: `arduino-cli compile --fqbn arduino:renesas_uno:nanor4 && arduino-cli upload --fqbn arduino:renesas_uno:nanor4 -p COM8`
