# Excavator Track Controller — Project Plan

## What This Does
An Arduino UNO Q sits between an RC receiver and two ESCs, implementing a
**tank-style mixer** with a secondary joystick input (for the rider/operator).
The RC transmitter always takes priority. A third RC channel acts as a
joystick enable/override switch.

The controller provides:
- **Exponential response curve** for fine low-speed control
- **Inertia simulation** so the tracks feel heavy (smooth ramp, gradual stop)
- **Closed-loop PID** with current sensors for load compensation
- **Soft power limits** (exponential saturation, no hard clamps)
- **Anti-runaway failsafe** to prevent compensation from diverging

---

## Inventory

| # | Component | Model / Part | Key Specs |
|---|-----------|-------------|-----------|
| 1 | Controller | Arduino UNO Q [ABX00162] | STM32U585, 160MHz, 14-bit ADC, 3.3V logic |
| 2 | Battery (x2) | OVONIC 3S LiPo 15000mAh 130C | 11.1V, EC5 connector |
| 3 | ESC (x2) | XC E10 Sensored Brushless 140A | XT60 power, servo PWM input |
| 4 | Motor (x2) | XC E3665 2500KV Sensored Brushless | 8mm shaft |
| 5 | RC System | Radiolink RC6GS V3 + R7FG receiver | 6CH, gyro receiver |
| 6 | Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect |
| 7 | Current Sensor (x2) | CS7581 Hall-effect | Per-motor current monitoring |

---

## Inputs (7 channels)

| Ch | Source | Signal Type | Pin | Description |
|----|--------|-------------|-----|-------------|
| 1 | RC Receiver | Servo PWM | D2 | Left motor (pre-mixed by transmitter) |
| 2 | RC Receiver | Servo PWM | D4 | Right motor (pre-mixed by transmitter) |
| 5 | RC Receiver | Servo PWM | D7 | Override switch (3-pos) |
| — | Joystick | Analog 0-3.3V | A0 | Throttle Y axis (via 5V->3.3V divider) |
| — | Joystick | Analog 0-3.3V | A1 | Steering X axis (via 5V->3.3V divider) |
| — | CS7581 | Analog | A2 | Current sensor — left motor |
| — | CS7581 | Analog | A3 | Current sensor — right motor |

> **3.3V WARNING:** The UNO Q uses 3.3V logic. The joystick outputs 0-5V
> and MUST go through a voltage divider before connecting to A0/A1.
> Connecting 5V directly to any analog pin will damage the STM32U585 ADC.

> **Note:** The RC transmitter has its own internal tank mixing. CH1 and CH2
> arrive as left/right motor signals, NOT as throttle/steering. The Arduino
> passes them through directly. The joystick outputs raw throttle/steering,
> so the Arduino applies tank mixing only to the joystick signals.

## Outputs (2 channels)

| Ch | Destination | Signal Type | Pin | Description |
|----|-------------|-------------|-----|-------------|
| A | Left Track ESC | Servo PWM | D9 | Left motor speed + direction |
| B | Right Track ESC | Servo PWM | D10 | Right motor speed + direction |

---

## Override Switch Logic (Receiver CH5 -> D7, 3-position switch)

| CH5 Position | PWM Value | Joystick | RC Receiver |
|-------------|-----------|---------|-------------|
| LOW (~1000) | Mode 1 | Disabled | Full control |
| MID (~1500) | Mode 2 | Active | Fully overrides joystick |
| HIGH (~2000)| Mode 3 | Active | 50% override (blended control) |

**Mode 1 — Remote only:** Joystick input is completely ignored. RC
transmitter has full authority over both tracks.

**Mode 2 — Remote + Joystick (full override):** Joystick is active and
controls the tracks when RC sticks are at neutral. When RC gives any
non-neutral input (outside +/-50us deadband), RC takes 100% priority and
joystick is ignored.

**Mode 3 — Remote + Joystick (blended):** Both inputs are active
simultaneously. Output is 50% RC + 50% joystick, so the joystick retains
partial authority even when RC is giving input. Useful for fine-tuning
position while someone assists via remote.

---

## Signal Processing Pipeline (V2.0)

```
RC (attachInterrupt ISR) ──────────────────────────────────────┐
                                                                ├─ Mode Select ─┐
Joystick (analogRead 14-bit) ─ Deadband ─ Expo Curve ─ Tank Mix┘               │
                                                                                v
                                                    PID Load Compensation ──────┤
                                                    (CS7581 current feedback)   │
                                                                                v
                                                    Soft Power Scale (tanh)     │
                                                    (50%, no hard clamps)       │
                                                                                v
                                                    Inertia Simulation          │
                                                    (spring-damper model)       │
                                                                                v
                                                    Anti-Runaway Check          │
                                                                                v
                                                    Servo Output (D9, D10)
```

### RC Reading — Hardware Interrupts
All 3 RC channels decoded via `attachInterrupt()` on CHANGE. The STM32U585
supports interrupts on all digital pins — no PCINT register manipulation
needed. Main loop runs at 20,000+ Hz with zero blocking.

### Exponential Response Curve
Joystick input is mapped through `pow(normalized, 2.5)`. This gives fine
control at low stick (for precise maneuvering) and full authority at max
stick (for travel speed). The 14-bit ADC provides 16384 steps of resolution.

### Tank Mix (Joystick Only)
```
steerOffset = steering - 1500
left  = throttle + steerOffset
right = throttle - steerOffset
```

RC signals are already tank-mixed by the transmitter and pass through
without a second mix.

### PID Load Compensation — "Cruise Control for Tracks"
Closed-loop PID monitors per-motor current via CS7581 sensors. When a track
encounters resistance (mud, cold/stiff rubber, obstacle, uphill), it draws
more current than expected. The PID **boosts** the output to push through
and maintain consistent track speed.

How it works:
1. Compute expected current from commanded output: `expected = delta * CURRENT_PER_UNIT`
2. Compare against measured current from CS7581
3. If measured > expected, resistance is detected
4. PID computes a **boost** (extra us added to the ESC command)
5. Boost is applied in the direction of travel

**Adaptive gains:** At higher speeds, gains are reduced for stability
(prevents oscillation). At low speed (precise maneuvering), gains are
higher for responsive compensation. A `tanh()` soft ceiling on the boost
ensures it approaches COMP_MAX_BOOST smoothly — never abruptly.

**Hard safety cutoff:** If current exceeds CURRENT_HARD_LIMIT (100A), the
motor is forced to neutral immediately. This is separate from the PID —
it's a last-resort protection against motor stall, short, or jam.

### Soft Power Limits (Exponential Saturation)
Instead of hard clamps at OUTPUT_MIN/OUTPUT_MAX, the output follows a
`tanh()` curve that asymptotically approaches the limit. Small inputs pass
through nearly linearly; large inputs compress smoothly. No abrupt cutoff.

```
output = limit * tanh(input / limit)
```

### Inertia Simulation (Spring-Damper Model)
Replaces V1.2's asymmetric EMA with a physics-based model that makes the
output feel heavy:

```
spring_force = RESPONSE_FORCE * (target - position)
friction     = -FRICTION_COEFF * velocity
acceleration = (spring_force + friction) / VIRTUAL_MASS
velocity    += acceleration * dt
position    += velocity * dt
```

- **Heavy ramp-up:** Mass resists acceleration
- **At low speed:** Friction dominates, stops cleanly
- **At high speed:** Takes longer to stop (momentum)
- **Near neutral:** Snaps to zero (prevents micro-drift)

### Anti-Runaway Failsafe
If the PID boost has been at or near COMP_MAX_BOOST (>85%) for more than
2 seconds continuously, the track is physically stuck — the PID is pushing
as hard as it can but the resistance isn't going away. Rather than burning
out the motor or causing erratic behavior, the system goes to neutral.

The failsafe **latches** until the operator returns the stick to neutral,
which forces a deliberate restart. This prevents the machine from lurching
when the failsafe clears.

### Deadbands
- **RC:** +/-50us around 1500 — values in 1450-1550 snap to neutral
- **Joystick:** +/-480 around 8192 (14-bit ADC) — ~3% dead zone at center

### RC Failsafe
- If no RC signal for 500ms, both ESCs go to neutral (1500us = stop)

---

## Power

| Rail | Powers | Notes |
|------|--------|-------|
| 5V | RC Receiver VCC | From Arduino 5V pin |
| 5V | Joystick VCC | Genie 101174GT, ~5mA draw |
| 3.3V | Current sensors | If CS7581 variant is 3.3V-powered |
| GND | All components | Common ground is mandatory |
| VIN | Arduino itself | 7-24V from battery/BEC |

> **Joystick confirmed:** Genie 101174GT dual-axis joystick.
> Runs on 5V, outputs 0-5V analog per axis, center ~2.5V. Negligible
> current draw. Arduino 5V pin can power both the receiver and joystick.
> **REQUIRES voltage divider to 3.3V for UNO Q analog inputs.**

---

## Joystick Details (Genie 101174GT)

- **Part number:** 101174 / 101174GT
- **Type:** Dual-axis with active electronics (Hall effect suspected)
- **Power:** 5V DC
- **Output:** 0-5V analog per axis (center position ~2.5V)
- **Harness:** 6-wire adapter harness (3 per side)
- **Source:** Amazon ASIN B0F99C27BW (~$73)

---

## Wiring Diagram (V2.0 — UNO Q)

```
                        ARDUINO UNO Q
                    ┌───────────────────┐
    RC CH1 ────────>│ D2  (left motor)  │
    RC CH2 ────────>│ D4  (right motor) │
    RC CH5 ────────>│ D7  (override)    │
                    │              D9  ~│──> Left Track ESC
                    │             D10  ~│──> Right Track ESC
                    │                   │
  Joy Y (5V->3.3V)>│ A0  (throttle)    │
  Joy X (5V->3.3V)>│ A1  (steering)    │
  CS7581 Left ─────>│ A2  (current L)   │
  CS7581 Right ────>│ A3  (current R)   │
                    │                   │
         5V ───────>│ 5V         GND   │<── Common GND
   Battery ────────>│ VIN               │
                    └───────────────────┘

  Voltage Dividers (5V joystick -> 3.3V ADC):
    Joy output ──[10kΩ]──┬──[6.8kΩ]── GND
                         └──> A0 or A1 (3.3V max)
```

---

## Pin Summary (Quick Reference)

```
D2  <- RC CH1 (Left motor, pre-mixed)    [attachInterrupt, CHANGE]
D4  <- RC CH2 (Right motor, pre-mixed)   [attachInterrupt, CHANGE]
D7  <- RC CH5 (Override switch, 3-pos)   [attachInterrupt, CHANGE]
A0  <- Joystick Y axis (Throttle)        [14-bit ADC, 0-3.3V via divider]
A1  <- Joystick X axis (Steering)        [14-bit ADC, 0-3.3V via divider]
A2  <- CS7581 Current Sensor (Left)      [14-bit ADC]
A3  <- CS7581 Current Sensor (Right)     [14-bit ADC]
D9  -> Left Track ESC                    [Servo PWM output]
D10 -> Right Track ESC                   [Servo PWM output]
5V  -> RC Receiver + Joystick VCC
VIN <- Battery / BEC (7-24V)
GND -> All components (common ground)
```

---

## Hardware

### Previous: Arduino Nano V3 Clone
- **MCU:** ATmega328P (AVR, 16MHz, 2KB RAM, 32KB flash)
- **USB:** CH340G — requires driver install
- **RC reading:** Pin Change Interrupts (PCINT2) on D2, D4, D7
- **Board package:** `arduino:avr` -> Board: Arduino Nano
- **ADC:** 10-bit (1024 steps), 5V reference
- **Logic:** 5V
- **Status:** Retired — insufficient for PID + current sensing at 20kHz

### Current: Arduino UNO Q [ABX00162]
- **MCU:** STM32U585 (Arm Cortex-M33, 160MHz, 786KB RAM, 2MB flash)
- **MPU:** Qualcomm Dragonwing QRB2210 (Linux side, not used for control)
- **USB:** Native USB-C — no driver needed
- **RC reading:** `attachInterrupt()` on all digital pins
- **Board package:** `arduino:zephyr`
- **FQBN:** `arduino:zephyr:unoq`
- **ADC:** 14-bit (16384 steps), 3.3V reference
- **Logic:** 3.3V — requires voltage dividers for 5V peripherals
- **Loop rate:** 20,000+ Hz target
- **Why upgrade:** 10x clock speed + FPU for float math. The PID + expo
  curve + inertia simulation + 4x ADC reads need ~30-50us per loop.
  ATmega328P at 16MHz without FPU cannot achieve this.

---

## Software

### Sketch: `sketches/rc_test/rc_test.ino`
- **Version:** V2.0 (UNO Q, PID, inertia, soft limits)
- **Loop rate:** 20,000+ Hz (micros()-based timing, zero blocking)
- **Serial output:** 20 Hz at 115200 baud (telemetry with current data)

### Live Plot: `live_plot.py`
- **7-panel real-time monitor** (matplotlib)
- Displays all inputs (D2, D4, D7, A0, A1) and both ESC outputs (L, R)
- TODO: Add current sensor panels (IL, IR) and PID reduction panels (RL, RR)
- Run: `python live_plot.py` or `Ctrl+Shift+B` in VS Code -> "Live Plot"

### Tuning Constants (V2.0)
| Constant | Value | Description |
|----------|-------|-------------|
| `POWER_LIMIT_PCT` | 50 | Max output as % of full range |
| `EXP_CURVE` | 2.5 | Expo exponent (1=linear, 3=very fine) |
| `VIRTUAL_MASS` | 3.0 | Inertia: higher = more sluggish |
| `FRICTION_COEFF` | 8.0 | Inertia: higher = stops sooner |
| `RESPONSE_FORCE` | 20.0 | Inertia: higher = tracks stick faster |
| `CURRENT_PER_UNIT` | 0.25 A/us | Expected current per us of output |
| `COMP_MAX_BOOST` | 50us | Max extra us the PID can add |
| `COMP_KP` | 0.3 | PID proportional gain |
| `COMP_KI` | 0.05 | PID integral gain |
| `COMP_KD` | 0.01 | PID derivative gain |
| `CURRENT_HARD_LIMIT` | 100A | Force neutral (safety cutoff) |
| `RUNAWAY_TIME` | 2s | Anti-runaway time window |
| `RC_DEADBAND` | +/-50us | RC neutral zone |
| `JOY_DEADBAND` | +/-480 | Joystick neutral zone (14-bit ADC) |
| `FAILSAFE_TIMEOUT` | 500ms | Neutral if RC lost |

---

## Status
- [x] Joystick voltage/signal confirmed (Genie 101174GT, 5V, analog)
- [x] RC CH1 (D2) verified — left motor, center ~1500us
- [x] RC CH2 (D4) verified — right motor, center ~1497us
- [x] RC CH5 (D7) verified — 3-pos override, LOW ~972us, HIGH ~2060us
- [x] Tank mixer V1.2 written and tested (Nano V3)
- [x] RC pass-through verified (transmitter pre-mixes)
- [x] Joystick tank mix verified
- [x] 7-panel live plot with ESC output monitoring
- [x] V2.0 sketch written for UNO Q (PID, inertia, soft limits)
- [ ] Joystick harness wiring identified (reverse engineering in progress)
- [ ] Voltage dividers for joystick (5V -> 3.3V)
- [ ] CS7581 current sensor wiring and calibration
- [ ] Verify ESC/motor/sensor frequency compatibility
- [ ] Update live_plot.py for current + PID telemetry
- [ ] Bench test on UNO Q with ESCs disconnected
- [ ] Field test with ESCs at reduced power
- [ ] Battery voltage monitoring (future phase)
- [ ] Beeper/buzzer warnings (future phase)
