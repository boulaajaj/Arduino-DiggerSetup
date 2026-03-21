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
D2  <- RC CH1 (Left motor, pre-mixed)     [interrupt CHANGE]
D4  <- RC CH2 (Right motor, pre-mixed)    [interrupt CHANGE]
D5  <- Motor Hall Sensor LEFT              [interrupt RISING — RPM feedback]
D6  <- Motor Hall Sensor RIGHT             [interrupt RISING — RPM feedback]
D7  <- RC CH5 (Override switch, 3-pos)     [interrupt CHANGE]
A0  <- Joystick Y axis (Throttle)          [14-bit ADC, 0-3.3V via divider]
A1  <- Joystick X axis (Steering)          [14-bit ADC, 0-3.3V via divider]
A2  <- CS7581 Current Sensor (Left)        [14-bit ADC]
A3  <- CS7581 Current Sensor (Right)       [14-bit ADC]
D9  -> Left Track ESC                      [Servo PWM output]
D10 -> Right Track ESC                     [Servo PWM output]
5V  -> RC Receiver + Joystick VCC
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

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

### XC E10 ESC Internals (RESEARCHED 2026-03-20)
The E10 uses **trapezoidal (6-step) commutation, NOT FOC**. XC sells a
separate GL10 (80A) with FOC for crawlers — the E10 is their racing/bashing
line.

**Control chain timing:**
| Layer | Rate | Notes |
|-------|------|-------|
| Arduino loop | 20,000 Hz | Sensor filtering, PID, inertia sim |
| Servo PWM output (D9/D10) | **50 Hz** | Arduino `Servo` library default — **this is the bottleneck** |
| ESC motor PWM switching | 8-16 kHz | Internal commutation frequency |
| ESC current control | 8-16 kHz | Cycle-by-cycle (per switching period) |
| ESC processing latency | ~1-2 ms | Sensored = instant rotor position |

**Input signal:** Standard 50 Hz servo PWM only. No DShot/OneShot. Most RC
car ESCs tolerate up to ~333-490 Hz, but this is unconfirmed for the E10.
Test on bench before relying on faster input rates.

**Why 50 Hz is fine for tracks:** The gear reduction + rubber track inertia +
machine weight create a mechanical time constant of ~200-500 ms. At 50 Hz
(20 ms updates), you get 10-25 updates per mechanical time constant — more
than enough for smooth motion. The 6-step torque ripple at motor speed
(thousands of RPM) is at kHz+ frequencies — physically filtered out by the
drivetrain.

**Outer loop safety margin:** Control theory requires the outer loop to run
5-10x slower than the inner loop. At 50 Hz outer vs 8-16 kHz inner, the
ratio is 160-320x — extremely conservative. Even 400 Hz servo output would
be safe (20-40x ratio).

**Upgrade path if needed:** The STM32 on the UNO Q can output arbitrary
PWM frequencies via hardware timers (bypass Servo library). If 50 Hz isn't
smooth enough in testing, increase to 200-400 Hz — a ~10-line code change,
no hardware change. But test at 50 Hz first.

### XC E10 X.BUS Protocol (RESEARCHED 2026-03-21)
The E10 supports **X.BUS** — a bi-directional digital bus protocol for robot
control and automated systems. X.BUS provides:
- **Bi-directional:** Read voltage, current, temperature, RPM from ESC
- **Precision:** Digital commands, no analog PWM jitter
- **Multi-device:** Bus topology, multiple ESCs on one line

**This changes the plan.** If X.BUS update rate is ≥100 Hz, we may not need
the hall sensor tap at all — the ESC can report RPM directly. X.BUS latency
needs to be measured empirically before committing to a feedback approach.

**Step 1:** Connect X.BUS to UNO Q, write a timing test sketch and plot to
measure actual update frequency before any other development.

### Rubber Track Cyclic Load (UNDERSTOOD 2026-03-21)
**This is NOT about stiction (starting from 0 RPM).** The concern is:

Stiff rubber tracks create a **periodic load variation** as each link
engages and disengages the sprocket teeth:
- **Link falls onto sprocket tooth** → rubber must deform to wrap → **more
  friction** (motor fights harder to pull track off the sprocket)
- **Link releases from sprocket** → rubber springs back → **less friction**
  (momentary load drop)

This is a **continuous cyclic disturbance** at every RPM:
```
disturbance_freq_Hz = RPM × sprocket_teeth / 60
```

The PID must be fast enough to smooth these per-tooth load pulses so the
track maintains constant velocity. This is why the feedback update rate
matters — if X.BUS can deliver RPM at 100-200Hz, the controller can
counteract the per-tooth ripple in real time.

### PID Upgrade Path: Current-Based → RPM-Based
The current V2.0 PID uses current feedback (CS7581 sensors). The next version
will switch to RPM-based PID:
```
target RPM  = f(stick position)     // commanded speed
actual RPM  = from hall sensor tap  // measured speed
error       = target - actual
PID output  = boost or reduce ESC command to match target
```

Current sensors will remain for:
- Hard safety cutoff (100A limit — motor stall/short protection)
- Telemetry and diagnostics
- Possible dual-loop control (inner current loop, outer RPM loop)

## Implementation Status
- [x] V2.0 sketch: expo curve, tank mix, inertia, soft limits
- [x] PID load compensation (current-based, boosts under resistance)
- [x] Anti-runaway failsafe
- [x] 7-panel live plot (live_plot.py)
- [x] Operator guide for Jason and Malaki
- [x] ESC internals research (trapezoidal 6-step, 8-16kHz, 50Hz servo)
- [x] Closed-loop speed governance analysis report
- [ ] **X.BUS update rate test** ← NEXT (connect X.BUS, measure frequency)
- [ ] X.BUS timing plot (visualize update rate)
- [ ] Decide: X.BUS RPM vs. hall sensor tap (based on measured X.BUS rate)
- [ ] RPM-based PID implementation
- [ ] Joystick harness wiring identification
- [ ] Voltage dividers for joystick (5V → 3.3V)
- [ ] CS7581 current sensor wiring and calibration
- [ ] Bench test on UNO Q
- [ ] Field test at reduced power
- [ ] Battery voltage monitoring (future)

## Next Steps (CURRENT — 2026-03-21)
1. **X.BUS update rate test** — FIRST PRIORITY:
   - Connect X.BUS from XC E10 to Arduino UNO Q
   - Write a sketch that reads X.BUS and timestamps each update
   - Write a plot script to visualize update frequency
   - If ≥100-200 Hz: use X.BUS for RPM feedback (no hall tap needed)
   - If <100 Hz: fall back to hall sensor tap approach
2. **RPM hall sensor interrupt code** (if X.BUS too slow):
   - ISRs on D5 and D6 (RISING edge) counting pulses
   - RPM calculation using interval between edges
   - Expose `rpmLeft` and `rpmRight` as global state
3. **Upgrade PID from current-based to RPM-based:**
   - `pidCompensate()` takes target RPM and actual RPM instead of current
   - Stick position maps to target RPM (calibration needed)
   - Current sensors remain for hard safety cutoff only
4. **Add RPM to serial telemetry** so live_plot.py can display it
5. **Verify hall sensor pinout** on the actual XC E3665 sensor cable before wiring
6. **Build 5V→3.3V voltage dividers** for D5 and D6 (same as joystick dividers)

## File Map
```
PROJECT-PLAN.md         — Full technical specification
OPERATOR-GUIDE.md       — User guide for Jason (RC) and Malaki (joystick)
sketches/rc_test/rc_test.ino — Main Arduino sketch (V2.0)
live_plot.py            — Real-time 7-panel matplotlib monitor
monitor.py              — Simple serial monitor
docs/xc-e10-closed-loop-analysis.md — Closed-loop speed governance report
```

## Build & Upload
- Board: Arduino UNO Q
- FQBN: `arduino:zephyr:unoq`
- Board package: `arduino:zephyr`
- Serial: 115200 baud
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
