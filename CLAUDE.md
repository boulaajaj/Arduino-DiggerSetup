# Excavator Track Controller — Project Memory

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
|-----------|-------|------------|
| Controller | Arduino Nano R4 | Renesas RA4M1, 48MHz, 5V tolerant GPIO, 14-bit ADC |
| ESC (x2) | XC E10 Sensored Brushless 140A | Servo PWM input, sensored mode |
| Motor (x2) | XC E3665 2500KV Sensored Brushless | 4-pole, 8mm shaft, hall sensor cable to ESC |
| Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| RC System | Radiolink RC6GS V3 + R7FG | 6CH, transmitter does tank mixing |
| Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect (direct to ADC — 5V tolerant) |
| Current Sensor (x2) | CS7581 Hall-effect | Per-motor current monitoring |

## Pin Assignments (Nano R4)
```
D0  <- X.BUS shared bus (Serial RX)        [Both ESCs on one bus — conflicts with USB Serial]
D1  -> X.BUS TX (Serial TX)               [For half-duplex commands, if needed]
D2  <- RC CH1 (Left motor, pre-mixed)      [attachInterrupt CHANGE]
D3  <- RC CH4 (Control mode, 3-pos)        [attachInterrupt CHANGE]
D4  <- RC CH2 (Right motor, pre-mixed)     [pulseIn — no interrupt on D4]
D5     (available)                          [reserved for future use]
D6     (available)                          [reserved for future use]
D7  <- RC CH5 (Override switch, 3-pos)     [pulseIn — no interrupt on D7]
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-5V direct (5V tolerant)]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-5V direct (5V tolerant)]
A2  <- CS7581 Current Sensor (Left)        [14-bit ADC]
A3  <- CS7581 Current Sensor (Right)       [14-bit ADC]
D9  -> Left Track ESC                      [Servo PWM output]
D10 -> Right Track ESC                     [Servo PWM output]
5V  -> RC Receiver + Joystick VCC
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

**Interrupt limitation:** On the Nano R4, `attachInterrupt()` only works on
D2 and D3. For D4 and D7, use `pulseIn()` (blocking but acceptable at low
update rates for RC signals).

### UART Architecture (Nano R4)
The Nano R4 has a single hardware UART on D0/D1, which is also the USB
Serial connection. Standard Arduino `Serial` works over USB — no Router
Bridge, no Monitor object, no special workarounds needed.

**X.BUS Protocol:** XC E10 "X.BUS" is NOT Spektrum X-Bus (I2C). It is XC
Technology's proprietary protocol, almost certainly half-duplex UART. Single
data wire = cannot be I2C. Both ESCs share one bus (addressable 0-15).

| Port | Hardware | Pin | Function |
|------|----------|-----|----------|
| Serial | UART | D0 (RX) / D1 (TX) | USB Serial AND X.BUS (shared, cannot use both simultaneously) |

**D0 conflict:** X.BUS on D0 conflicts with USB Serial. During bench
testing, unplug X.BUS from D0 to use Serial Monitor for debugging. Plug
X.BUS back for field operation. No code change needed — just a wire swap.

## Architecture Summary
Sketch: `sketches/rc_test/rc_test.ino` (V3.4) + `types.h`

Signal pipeline:
1. RC inputs: D2/D3 via attachInterrupt (ISR), D4/D7 via non-blocking PulseReader (poll)
2. Joystick via 14-bit ADC (A0, A1, cached at 100Hz) → deadband → expo curve → tank mix
3. Override mode select (RC CH5: Mode 1=RC only, Mode 2=RC overrides joy, Mode 3=50/50 blend)
4. Spin turn limiter (graduated, 35% at full pivot)
5. Reverse speed limiter (35% of forward max)
6. Soft power scaling (tanh saturation)
7. Inertia simulation (asymmetric exponential: 0.3s accel, 0.5s decel)
8. Servo output to ESCs (D9, D10)

Code organized in searchable [MODULE] sections: [CONFIG], [RC], [JOYSTICK],
[MIXER], [DYNAMICS], [OUTPUT], [DEBUG]. Search `[NAME]` to jump.

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
|-----|--------|
| 1 | 5V (from ESC) |
| 2 | Hall A |
| 3 | Hall B |
| 4 | Hall C |
| 5 | Temperature / NC |
| 6 | GND |

**Wiring — parallel tap (non-invasive):**
```
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
```
edgesPerRevolution = 4  // 2 hall lines × 2 pole pairs
RPM = (edgeCount / edgesPerRevolution) / elapsed_seconds * 60
```

Or interval-based (better at low RPM):
```
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

### RPM Source: X.BUS on Shared Bus + Hall Sensor Fallback
- **Both ESCs:** X.BUS Yellow → D0 (Serial RX), shared bus (addressable 0-15)
- **Fallback:** Direct motor hall sensor tap on D5/D6 (if X.BUS too slow)
- **Debug:** Serial over USB (disconnect X.BUS from D0 for bench testing)
- Test with `sketches/xbus_probe/xbus_probe.ino` first

### Control Mode Selector (RC CH4 on D3, 3-position switch)
| CH4 | Mode | Layers Active |
|-----|------|--------------|
| LOW | RAW | Direct stick→ESC, no processing (baseline) |
| MID | RPM_ONLY | Outer RPM PID only (speed regulation, no smoothing) |
| HIGH | FULL_STACK | Current FF + RPM + expo + inertia + soft limits |

Flip CH4 on the field to A/B/C compare control quality in real-time.
Mode transitions reset all PID/inertia state to prevent jumps.

## Implementation Status
- [x] V2.0 sketch: expo curve, tank mix, inertia, soft limits
- [x] PID load compensation (current-based, boosts under resistance)
- [x] Anti-runaway failsafe
- [x] 7-panel live plot (live_plot.py)
- [x] Operator guide for Jason and Malaki
- [x] X.BUS probe sketch written
- [ ] **X.BUS telemetry validation** ← NOW (wire, upload, test)
- [ ] Dual-loop PID implementation (after X.BUS decision)
- [ ] Joystick harness wiring identification
- [ ] CS7581 current sensor wiring and calibration
- [ ] Bench test on Nano R4
- [ ] Field test at reduced power

## Next Steps
1. **Wire both ESC X.BUS Yellow wires together** to shared bus
2. **Add 10kΩ pull-up to 5V** on the shared bus (holds line HIGH when idle)
3. **Connect shared bus to D0** (direct — Nano R4 is 5V tolerant)
4. **Upload `sketches/xbus_probe/xbus_probe.ino`** to Nano R4 (COM8)
5. **Test with one ESC first** (determine baud rate and protocol behavior):
   - Which baud rate locks (auto-detected, prioritize 250000)
   - Raw hex dump for pattern analysis
   - Whether ESC auto-sends or requires polling
   - Packet rate in Hz (need >20 Hz for outer loop)
6. **Test with both ESCs** if single ESC works — verify addressed bus behavior
7. **Contact XC Technology** (sales7@xc-bldc.com) for "XC.BUS Control Protocol" doc
8. **Goal:** All telemetry AND throttle control via X.BUS — no hall sensors or current sensors

## File Map
```
PROJECT-PLAN.md                    — Full technical specification
OPERATOR-GUIDE.md                  — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino       — Main Arduino sketch (V3.4)
sketches/rc_test/types.h           — Shared structs (RCChannel, PulseReader, etc.)
sketches/xbus_probe/xbus_probe.ino — X.BUS telemetry probe/test sketch
docs/CONTROL-RESEARCH.md           — Tank mix, RC input, loop patterns research
live_plot.py                       — Real-time matplotlib monitor
monitor.py                         — Simple serial monitor
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
