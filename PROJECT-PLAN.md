# Excavator Track Controller — Project Plan

## What This Does
An Arduino UNO Q sits between an RC receiver and two ESCs, implementing a
**tank-style mixer** with a secondary joystick input (for the rider/operator).
The RC transmitter always takes priority. A third RC channel acts as a
joystick enable/override switch.

The controller provides:
- **Exponential response curve** for fine low-speed control
- **Inertia simulation** so the tracks feel heavy (smooth ramp, gradual stop)
- **Dual-loop PID** — inner current loop (feedforward) + outer RPM loop (feedback)
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
| 8 | RPM/Telemetry | ESC X.BUS (primary) | Serial telemetry: RPM, current, voltage, temp |
| 9 | RPM Feedback (x2) | Motor hall sensor tap (fallback) | Parallel tap on motor→ESC sensor cable |

---

## Inputs

| Ch | Source | Signal Type | Pin | Description |
|----|--------|-------------|-----|-------------|
| 1 | RC Receiver | Servo PWM | D2 | Left motor (pre-mixed by transmitter) |
| 2 | RC Receiver | Servo PWM | D4 | Right motor (pre-mixed by transmitter) |
| 4 | RC Receiver | Servo PWM | D3 | Control mode selector (3-pos: RAW/RPM/FULL) |
| 5 | RC Receiver | Servo PWM | D7 | Override switch (3-pos) |
| — | Joystick | Analog 0-3.3V | A0 | Throttle Y axis (via 5V->3.3V divider) |
| — | Joystick | Analog 0-3.3V | A1 | Steering X axis (via 5V->3.3V divider) |
| — | CS7581 | Analog | A2 | Current sensor — left motor |
| — | CS7581 | Analog | A3 | Current sensor — right motor |
| — | ESC X.BUS (L) | Serial UART | D0 (USART1 RX) | X.BUS telemetry — left ESC (RPM, current, voltage, temp) |
| — | ESC X.BUS (R) | Serial UART | PG8 (LPUART1 RX, JMISC solder) | X.BUS telemetry — right ESC |
| — | Motor Hall Tap (L) | Digital 5V→3.3V | D5 | RPM feedback — left motor (FALLBACK if X.BUS too slow) |
| — | Motor Hall Tap (R) | Digital 5V→3.3V | D6 | RPM feedback — right motor (FALLBACK if X.BUS too slow) |

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

## Control Mode Switch (Receiver CH4 -> D3, 3-position switch)

| CH4 Position | PWM Value | Mode | What Runs |
|-------------|-----------|------|-----------|
| LOW (~1000) | RAW | Direct stick-to-ESC, no processing layers |
| MID (~1500) | RPM_ONLY | Outer RPM PID loop only (speed regulation) |
| HIGH (~2000) | FULL_STACK | Inner current FF + outer RPM + expo + inertia |

**RAW — Direct pass-through:** Stick → deadband → linear map → hard power
clamp → ESC. No PID, no expo curve, no inertia. Feels like the old Nano V3
setup. Instant response, no smoothing. Good for baseline comparison.

**RPM_ONLY — Speed regulation only:** Stick → deadband → linear map → RPM
PID trim → hard power clamp → ESC. The outer RPM loop maintains consistent
track speed under varying load, but no anticipation of load spikes (no
current feedforward) and no inertia smoothing. Good for testing whether
RPM feedback alone is sufficient.

**FULL_STACK — Premium control:** Stick → deadband → expo curve → tank mix →
mode select → current feedforward → RPM trim → anti-runaway → soft power
limit → inertia simulation → ESC. Disturbances are absorbed before the
operator feels them. Tracks feel heavy and deliberate. The full experience.

**Mode transitions:** When CH4 is flipped, all PID integrals, inertia
velocity/position, and anti-runaway state are reset to zero. This prevents
jumps or lurches when switching modes on the field.

**Hard current safety cutoff (100A) is ALWAYS active in ALL modes.**

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

## Signal Processing Pipeline (V2.1 — Dual-Loop Architecture)

```
RC (attachInterrupt ISR) ──────────────────────────────────────┐
                                                                ├─ Mode Select ─┐
Joystick (analogRead 14-bit) ─ Deadband ─ Expo Curve ─ Tank Mix┘               │
                                                                                v
                                              ┌─ Inner Loop (FEEDFORWARD) ─────┤
                                              │  CS7581 current sensors         │
                                              │  Detects load spike INSTANTLY   │
                                              │  Pre-boosts before RPM drops    │
                                              └────────────────────────────────┤
                                              ┌─ Outer Loop (FEEDBACK) ────────┤
                                              │  X.BUS RPM (or hall sensor)    │
                                              │  Measures actual motor speed   │
                                              │  Fine-tunes to hold target RPM │
                                              └────────────────────────────────┤
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

### Dual-Loop Control Strategy (DECIDED 2026-03-21)

**Both current AND RPM feedback are used simultaneously — NOT one or the other.**

- **Inner loop (feedforward via current):** CS7581 current sensors detect load
  disturbances instantly. When a track hits mud or an obstacle, current spikes
  BEFORE RPM drops. The inner loop pre-compensates by boosting power before
  the operator feels anything.

- **Outer loop (feedback via RPM):** X.BUS telemetry (or hall sensor tap) measures
  actual motor speed. This closes the loop on whether the compensation actually
  worked. It fine-tunes the ESC command to hold the target RPM.

- **Result:** Disturbances are absorbed before they become perceptible to a human.
  Current predicts the impact; RPM confirms the recovery. This is a standard
  industrial motor control pattern (inner current loop + outer speed loop).

### RPM Source: X.BUS First, Hall Sensor Fallback

**Primary — ESC X.BUS telemetry (testing required):**
The XC E10 has a dedicated X.BUS wire (3-pin: Yellow=data, Brown=GND, Red=BEC+).
The manual says it supports "real-time control of ESC and reading of operation
data through Bus" for "robot control or other automated programming control."
The X.BUS provides RPM + current + voltage + temperature in one packet.

**If X.BUS update rate is sufficient (>20 Hz for outer loop):** Use X.BUS for
the outer RPM loop. The inner loop still uses CS7581 current sensors directly
(ADC speed, no serial latency). This is the cleanest wiring solution.

**Fallback — Direct motor hall sensor tap:**
If X.BUS is too slow or unreliable, tap the motor's existing hall sensor cable
in parallel. This gives microsecond-resolution RPM at interrupt speed but
requires Y-splitters, voltage dividers, and signal isolation to prevent
ESC/Arduino interference.

**Step 1 (NOW):** Run `sketches/xbus_probe/xbus_probe.ino` to determine:
  - What baud rate X.BUS uses
  - What data format the packets are in
  - How fast packets arrive (Hz)
  - Whether the data is clean and decodable

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

#### Current Implementation (V2.0) — Current-Based
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

#### Planned Upgrade (V2.1) — RPM-Based PID via Motor Hall Sensor Tap

**Decision (2026-03-20):** Upgrade PID feedback from current-based to
RPM-based using a passive tap on the motor's existing hall sensor cable.

**Why RPM instead of current:**
- Current is a proxy for load, not speed. True closed-loop speed control
  requires measuring actual motor RPM.
- Handles all disturbances (mud, cold rubber, uphill, weight changes)
  because it measures what actually matters: is the motor spinning at the
  commanded speed?

**Why tap at the motor (not at the sprocket):**
- High resolution: many pulses per motor revolution (4 edges/rev with 2 hall lines)
- Zero delay: measures motor shaft directly, no gearbox backlash lag
- The signal already exists: motor's internal hall sensors are wired to the ESC

**Why not Bluetooth telemetry from the ESC:**
- 50-200ms latency — too slow for 20kHz PID loop
- Proprietary Hobbywing protocol, poorly documented
- Requires extra BT hardware on both sides
- Far more code complexity for inferior results

**Motor sensor cable pinout (6-pin JST-ZH, standard Hobbywing):**

| Pin | Signal |
|-----|--------|
| 1 | 5V (from ESC) |
| 2 | Hall A |
| 3 | Hall B |
| 4 | Hall C |
| 5 | Temperature / NC |
| 6 | GND |

> **VERIFY THIS PINOUT** on the actual XC E3665 sensor cable before wiring.
> Hobbywing generally follows this standard, but confirm with a multimeter.

**Wiring — passive parallel tap (non-invasive):**
```
Motor Hall A ──────┬──── ESC Hall A
                   │
              [10kΩ]──┬──[6.8kΩ]── GND     (5V→3.3V divider)
                      └──> Arduino D5

Motor Hall B ──────┬──── ESC Hall B
                   │
              [10kΩ]──┬──[6.8kΩ]── GND     (5V→3.3V divider)
                      └──> Arduino D6

Motor GND ─────────┬──── ESC GND
                   │
              Arduino GND
```

> **3.3V LEVEL SHIFTING REQUIRED!** The UNO Q uses 3.3V logic. Hall sensor
> outputs are 5V. Use the same 10k/6.8k voltage dividers as the joystick.
> Connecting 5V directly to D5/D6 will damage the STM32U585.

**Resolution (4-pole motor, 2 pole pairs):**
- 2 electrical revolutions per mechanical revolution
- Counting rising edges on 2 hall lines = 4 edges per motor revolution
- At 5,000 RPM (loaded): ~333 edges/sec
- At 100 RPM: ~7 edges/sec
- Both are well within Arduino interrupt capability

**RPM calculation (interval method — better at low RPM):**
```
edgesPerRev = 4  // 2 hall lines × 2 pole pairs
RPM = 60,000,000 / (microsBetweenEdges × edgesPerRev)
```

**V2.1 PID loop:**
```
target RPM  = f(stick position)     // stick maps to desired speed
actual RPM  = from hall sensor tap  // measured motor speed
error       = target - actual
PID output  = adjust ESC command to close the gap
```

Current sensors (CS7581) will remain active for:
- Hard safety cutoff (100A — motor stall / short / jam protection)
- Telemetry and diagnostics
- Possible future dual-loop control (inner current, outer RPM)

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

## Wiring Diagram (V2.3 — UNO Q, Shared X.BUS)

**IMPORTANT CORRECTIONS (2026-03-29):**
- XC E10 "X.BUS" is NOT Spektrum X-Bus (I2C). It has ONE data wire → cannot be I2C.
- It is a proprietary XC Technology protocol, almost certainly **half-duplex UART**.
- Industry survey: 8-10 of 12 major ESC brands use UART serial for telemetry.
- **Serial1/LPUART1 (PG8) is RESERVED** by the Arduino Router bridge — do NOT use.
- Both ESCs share one bus on D0 — the X.BUS is designed for up to 16 addressed ESCs.

```
                        ARDUINO UNO Q
                    ┌───────────────────┐
  X.BUS (shared)──>│ D0  (USART1 RX)   │  ← Both ESCs on one bus (via divider)
    RC CH1 ────────>│ D2  (left motor)  │
    RC CH4 ────────>│ D3  (ctrl mode)   │
    RC CH2 ────────>│ D4  (right motor) │
    [reserved]─────>│ D5                │  ← Available for future use
    [reserved]─────>│ D6                │  ← Available for future use
    RC CH5 ────────>│ D7  (override)    │
  Debug TX ────────>│ D8  (SoftSerial)  │──> USB-to-serial adapter → PC
                    │              D9  ~│──> Left Track ESC (servo PWM)
                    │             D10  ~│──> Right Track ESC (servo PWM)
                    │                   │
  Joy Y (5V->3.3V)>│ A0  (throttle)    │
  Joy X (5V->3.3V)>│ A1  (steering)    │
  CS7581 Left ─────>│ A2  (current L)   │
  CS7581 Right ────>│ A3  (current R)   │
                    │                   │
         5V ───────>│ 5V         GND   │<── Common GND
   Battery ────────>│ VIN               │
                    └───────────────────┘

  ESC X.BUS Wiring (SHARED BUS — both ESCs on one wire):
    ESC Left  (addr 0) Yellow ──┬── ESC Right (addr 1) Yellow
                                │
                           [10kΩ pull-up to 3.3V]
                                │
                      [10kΩ]──┬──[6.8kΩ]── GND     (5V→3.3V divider)
                              └──> D0 (USART1 RX)
    Both ESC Brown (GND) ─────── Arduino GND
    Both ESC Red   (BEC+) ─────── NOT CONNECTED

  Debug Output Wiring:
    D8 (SoftwareSerial TX) ──> USB-to-serial adapter RX
    Arduino GND ─────────────> USB-to-serial adapter GND
    (USB-to-serial adapter connects to PC via USB for Serial Monitor)

  Voltage Dividers (5V -> 3.3V, used for joystick AND X.BUS):
    5V signal ──[10kΩ]──┬──[6.8kΩ]── GND
                        └──> target pin (3.3V max)

  NOTE: PG8/Serial1 is RESERVED by the Arduino Router bridge.
  Do NOT solder PG8. Do NOT use Serial1 in user code.
```

---

## Pin Summary (Quick Reference)

```
D0  <- X.BUS shared bus (USART1 RX)     [Hardware UART, both ESCs on one bus, via divider]
D1  -> X.BUS TX (USART1 TX)             [For half-duplex commands, if needed]
D2  <- RC CH1 (Left motor, pre-mixed)    [attachInterrupt, CHANGE]
D3  <- RC CH4 (Control mode, 3-pos)      [attachInterrupt, CHANGE]
D4  <- RC CH2 (Right motor, pre-mixed)   [attachInterrupt, CHANGE]
D5     (available)                        [reserved for future use]
D6     (available)                        [reserved for future use]
D7  <- RC CH5 (Override switch, 3-pos)   [attachInterrupt, CHANGE]
D8  -> Debug serial output               [SoftwareSerial TX, to USB-serial adapter]
A0  <- Joystick Y axis (Throttle)        [14-bit ADC, 0-3.3V via divider]
A1  <- Joystick X axis (Steering)        [14-bit ADC, 0-3.3V via divider]
A2  <- CS7581 Current Sensor (Left)      [14-bit ADC]
A3  <- CS7581 Current Sensor (Right)     [14-bit ADC]
D9  -> Left Track ESC                    [Servo PWM output]
D10 -> Right Track ESC                   [Servo PWM output]
PG8    RESERVED by Arduino Router bridge  [Do NOT use — Serial1/LPUART1]
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

### Sketch: `sketches/xbus_probe/xbus_probe.ino`
- **Purpose:** Test/probe X.BUS telemetry from XC E10 ESC
- **What it does:**
  1. Auto-scans baud rates (115200, 100000, 19200, 57600, 38400, 9600)
  2. Dumps raw hex bytes for manual pattern analysis
  3. Attempts Spektrum X-Bus ESC packet decoding (address 0x20)
  4. Measures packet arrival rate (Hz) and inter-packet gap (ms)
  5. Reports decoded RPM, current, voltage, temperature
- **Wiring:** ESC X.BUS Yellow → D0 (USART1 RX) via 5V→3.3V divider
  - Debug output via SoftwareSerial on D8 → USB-to-serial adapter
- **Decision gate:** If packets arrive >20 Hz with clean data → use X.BUS
  for dual-loop PID. If too slow → fall back to hall sensor tap.

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

### Completed
- [x] Joystick voltage/signal confirmed (Genie 101174GT, 5V, analog)
- [x] RC CH1 (D2) verified — left motor, center ~1500us
- [x] RC CH2 (D4) verified — right motor, center ~1497us
- [x] RC CH5 (D7) verified — 3-pos override, LOW ~972us, HIGH ~2060us
- [x] Tank mixer V1.2 written and tested (Nano V3)
- [x] RC pass-through verified (transmitter pre-mixes)
- [x] Joystick tank mix verified
- [x] 7-panel live plot with ESC output monitoring
- [x] V2.0 sketch written for UNO Q (PID, inertia, soft limits)
- [x] Decided: Dual-loop PID (inner current + outer RPM)
- [x] Decided: X.BUS telemetry first, hall sensor tap as fallback
- [x] X.BUS probe sketch written (`sketches/xbus_probe/xbus_probe.ino`)

### Step 1: X.BUS Telemetry Validation (NOW)
- [ ] **Solder wire to JMISC PG8 pad** on UNO Q board
- [ ] **Wire Left ESC X.BUS Yellow to D0** via 5V→3.3V voltage divider
- [ ] **Wire Right ESC X.BUS Yellow to PG8** via 5V→3.3V voltage divider
- [ ] **Connect USB-to-serial adapter** to D8 (debug TX) + GND
- [ ] **Upload and run xbus_probe sketch**
- [ ] **Test each ESC on D0** (unplug X.BUS, use Serial Monitor for initial validation)
- [ ] **Determine baud rate** (auto-scan will find it)
- [ ] **Measure packet rate** — need >20 Hz for outer RPM loop
- [ ] **Verify data decode** — RPM, current, voltage, temp values make sense
- [ ] **DECISION GATE:** X.BUS fast enough? → V2.2 with dual X.BUS. Too slow? → Hall sensor fallback.

### Step 2a: If X.BUS Works (V2.1 with X.BUS)
- [ ] Integrate X.BUS serial reading into main rc_test.ino sketch
- [ ] Implement dual-loop PID: inner current (CS7581) + outer RPM (X.BUS)
- [ ] Add X.BUS telemetry to serial output + update live_plot.py
- [ ] Integrate Right ESC X.BUS on Serial1 (LPUART1/PG8)
- [ ] Bench test dual-loop PID with ESC powered

### Step 2b: If X.BUS Too Slow (V2.1 with Hall Sensor Tap)
- [ ] Verify hall sensor cable pinout on actual XC E3665 before wiring
- [ ] Build 5V→3.3V voltage dividers for D5 and D6 (hall tap lines)
- [ ] Add RPM hall sensor interrupt code to sketch (D5, D6)
- [ ] Implement dual-loop PID: inner current (CS7581) + outer RPM (hall)
- [ ] Signal isolation to prevent ESC/Arduino interference/noise
- [ ] Add RPM fields to serial telemetry + update live_plot.py

### Remaining (Both Paths)
- [ ] Joystick harness wiring identified (reverse engineering in progress)
- [ ] Voltage dividers for joystick (5V → 3.3V)
- [ ] CS7581 current sensor wiring and calibration
- [ ] Bench test on UNO Q with ESCs disconnected
- [ ] Field test with ESCs at reduced power
- [ ] Battery voltage monitoring (future phase)
- [ ] Beeper/buzzer warnings (future phase)
