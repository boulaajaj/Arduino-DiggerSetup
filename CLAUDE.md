# Excavator Track Controller — Project Memory

## What This Is
Arduino UNO Q-based tank-style track controller for a ride-on excavator.
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
| Controller | Arduino UNO Q [ABX00162] | STM32U585, 160MHz, 3.3V logic, 14-bit ADC |
| ESC (x2) | XC E10 Sensored Brushless 140A | Servo PWM input, sensored mode |
| Motor (x2) | XC E3665 2500KV Sensored Brushless | 4-pole, 8mm shaft, hall sensor cable to ESC |
| Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| RC System | Radiolink RC6GS V3 + R7FG | 6CH, transmitter does tank mixing |
| Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect, needs 5V→3.3V divider |
| Current Sensor (x2) | CS7581 Hall-effect | Per-motor current monitoring |

## Pin Assignments (UNO Q)
```
D0  <- X.BUS shared bus (USART1 RX)        [Both ESCs on one bus, via 5V→3.3V divider]
D1  -> X.BUS TX (USART1 TX)               [For half-duplex commands, if needed]
D2  <- RC CH1 (Left motor, pre-mixed)      [interrupt CHANGE]
D3  <- RC CH4 (Control mode, 3-pos)        [interrupt CHANGE]
D4  <- RC CH2 (Right motor, pre-mixed)     [interrupt CHANGE]
D5     (available)                          [reserved for future use]
D6     (available)                          [reserved for future use]
D7  <- RC CH5 (Override switch, 3-pos)     [interrupt CHANGE]
D8  -> Debug serial output                 [SoftwareSerial TX, 115200 baud]
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-3.3V via divider]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-3.3V via divider]
A2  <- CS7581 Current Sensor (Left)        [14-bit ADC]
A3  <- CS7581 Current Sensor (Right)       [14-bit ADC]
D9  -> Left Track ESC                      [Servo PWM output]
D10 -> Right Track ESC                     [Servo PWM output]
PG8    RESERVED by Arduino Router bridge    [Do NOT use — Serial1/LPUART1]
5V  -> RC Receiver + Joystick VCC
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

### UART Architecture (REVISED 2026-03-29)
The UNO Q Zephyr firmware uses LLEXT (pre-built binary). Only two hardware
UARTs are enabled — USART2/3/UART4/5 cannot be added without rebuilding
firmware from source.

**CRITICAL:** Serial1/LPUART1 (PG8) is RESERVED by the Arduino Router bridge
(MPU-MCU communication). Do NOT use. Do NOT solder PG8.

**X.BUS Protocol:** XC E10 "X.BUS" is NOT Spektrum X-Bus (I2C). It is XC
Technology's proprietary protocol, almost certainly half-duplex UART. Single
data wire = cannot be I2C. Both ESCs share one bus (addressable 0-15).

| Port | Hardware | Pin | Function |
|------|----------|-----|----------|
| Serial | USART1 | D0 (RX) / D1 (TX) | X.BUS shared bus (both ESCs) |
| Serial1 | LPUART1 | PG8 | **RESERVED — Router bridge** |
| SoftwareSerial | Bit-bang TX | D8 (TX only) | Debug output to USB-serial adapter |

**D0 dual-use:** During bench testing, unplug X.BUS from D0 to use Serial
Monitor for debugging. Plug X.BUS back for field operation. No code change
needed — just a wire swap.

## Architecture Summary
Sketch: `sketches/rc_test/rc_test.ino` (V2.0)

Signal pipeline:
1. RC inputs via hardware interrupts (D2, D4, D7)
2. Joystick via 14-bit ADC (A0, A1) → deadband → expo curve (2.5) → tank mix
3. Override mode select (RC CH5: Mode 1=RC only, Mode 2=RC overrides joy, Mode 3=50/50 blend)
4. PID load compensation (current sensors on A2/A3) — boosts power under resistance
5. Soft power scaling (tanh saturation, 50% max)
6. Inertia simulation (spring-damper model — heavy machine feel)
7. Anti-runaway failsafe (2s stuck at max boost → neutral, latches until stick centered)
8. Servo output to ESCs (D9, D10)

Loop rate: 20,000+ Hz, zero blocking, micros()-based timing.

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
              Arduino D5 (via 5V→3.3V divider)

Motor Hall B ──────┬──── ESC Hall B
                   │
              Arduino D6 (via 5V→3.3V divider)

Motor GND ─────────┬──── ESC GND
                   │
              Arduino GND
```

**IMPORTANT: 3.3V level shifting required!**
The UNO Q is 3.3V logic. Hall sensor outputs are 5V. Use voltage dividers
(same 10k/6.8k as the joystick) or a logic level shifter. Connecting 5V
directly to D5/D6 will damage the STM32U585.

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

### RPM Source: X.BUS on Both ESCs + Hall Sensor Fallback
- **Left ESC:** X.BUS Yellow → D0 (Serial/USART1 RX) via 5V→3.3V divider
- **Right ESC:** X.BUS Yellow → PG8 (Serial1/LPUART1 RX, JMISC solder) via 5V→3.3V divider
- **Fallback:** Direct motor hall sensor tap on D5/D6 (if X.BUS too slow)
- **Debug:** SoftwareSerial TX on D8 → USB-to-serial adapter → PC
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
- [ ] Voltage dividers for joystick (5V → 3.3V)
- [ ] CS7581 current sensor wiring and calibration
- [ ] Bench test on UNO Q
- [ ] Field test at reduced power

## Next Steps
1. **Wire both ESC X.BUS Yellow wires together** to shared bus
2. **Add 10kΩ pull-up to 3.3V** on the shared bus (holds line HIGH when idle)
3. **Connect shared bus to D0** via 5V→3.3V voltage divider (10kΩ/6.8kΩ)
4. **Connect USB-to-serial adapter** to D8 (debug TX) for monitoring
5. **Upload `sketches/xbus_probe/xbus_probe.ino`** to UNO Q
6. **Test with one ESC first** (determine baud rate and protocol behavior):
   - Which baud rate locks (auto-detected, prioritize 250000)
   - Raw hex dump for pattern analysis
   - Whether ESC auto-sends or requires polling
   - Packet rate in Hz (need >20 Hz for outer loop)
7. **Test with both ESCs** if single ESC works — verify addressed bus behavior
8. **Contact XC Technology** (sales7@xc-bldc.com) for "XC.BUS Control Protocol" doc
9. **Goal:** All telemetry AND throttle control via X.BUS — no hall sensors or current sensors

## File Map
```
PROJECT-PLAN.md                    — Full technical specification
OPERATOR-GUIDE.md                  — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino       — Main Arduino sketch (V2.0)
sketches/xbus_probe/xbus_probe.ino — X.BUS telemetry probe/test sketch
live_plot.py                       — Real-time 7-panel matplotlib monitor
monitor.py                         — Simple serial monitor
```

## Build & Upload
- Board: Arduino UNO Q
- FQBN: `arduino:zephyr:unoq`
- Board package: `arduino:zephyr`
- Serial: 115200 baud
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
