# Arduino Nano Tank Mixer — Project Plan

## What This Does
An Arduino Nano sits between an RC receiver and two ESCs, implementing a
**tank-style mixer** with a secondary joystick input (from a boom lift).
The RC transmitter always takes priority. A third RC channel acts as a
joystick enable/override switch.

---

## Inputs (5 channels)

| Ch | Source          | Signal Type      | Pin  | Description                            |
|----|-----------------|------------------|------|----------------------------------------|
| 1  | RC Receiver     | Servo PWM        | D2   | Left motor (pre-mixed by transmitter)  |
| 2  | RC Receiver     | Servo PWM        | D4   | Right motor (pre-mixed by transmitter) |
| 5  | RC Receiver     | Servo PWM        | D7   | Override switch (3-pos, see below)     |
| —  | Boom Joystick   | Analog 0–5V      | A0   | Joystick throttle (Y axis)             |
| —  | Boom Joystick   | Analog 0–5V      | A1   | Joystick steering (X axis)             |

> **Note:** The RC transmitter has its own internal tank mixing. CH1 and CH2
> arrive as left/right motor signals, NOT as throttle/steering. The Arduino
> passes them through directly. The joystick outputs raw throttle/steering,
> so the Arduino applies tank mixing only to the joystick signals.

## Outputs (2 channels)

| Ch | Destination     | Signal Type      | Pin  | Description                        |
|----|-----------------|------------------|------|------------------------------------|
| A  | Left Track ESC  | Servo PWM        | D9   | Left motor speed + direction       |
| B  | Right Track ESC | Servo PWM        | D10  | Right motor speed + direction      |

---

## Override Switch Logic (Receiver CH5 → D7, 3-position switch)

| CH5 Position | PWM Value | Joystick           | RC Receiver                    |
|-------------|-----------|--------------------|---------------------------------|
| LOW (~1000) | Mode 1    | Disabled           | Full control                    |
| MID (~1500) | Mode 2    | Active             | Fully overrides joystick        |
| HIGH (~2000)| Mode 3    | Active             | 50% override (blended control)  |

**Mode 1 — Remote only:** Joystick input is completely ignored. RC
transmitter has full authority over both tracks.

**Mode 2 — Remote + Joystick (full override):** Joystick is active and
controls the tracks when RC sticks are at neutral. When RC gives any
non-neutral input (outside ±50μs deadband), RC takes 100% priority and
joystick is ignored.

**Mode 3 — Remote + Joystick (blended):** Both inputs are active
simultaneously. Output is 50% RC + 50% joystick, so the joystick retains
partial authority even when RC is giving input. Useful for fine-tuning
position while someone assists via remote.

---

## Signal Processing Pipeline

```
RC (PCINT ISR) ─────────────────────────────────────┐
                                                     ├─ Mode Select ─┐
Joystick (analogRead) ─ Deadband ─ Map ─ Tank Mix ──┘               │
                                                                     v
                                            Proportional Power Scale (40%)
                                                                     │
                                                                     v
                                               Asymmetric EMA Smoothing
                                               (800ms up / 400ms down)
                                                                     │
                                                                     v
                                                  Servo Output (D9, D10)
```

### RC Reading — Pin Change Interrupts (PCINT)
All 3 RC channels are decoded simultaneously in a hardware ISR
(`PCINT2_vect` on Port D). This eliminates the blocking behavior of
`pulseIn()` — the main loop runs at 1000+ Hz with no missed pulses.

### Tank Mix (Joystick Only)
The joystick outputs raw throttle (A0) and steering (A1). The Arduino
converts these to left/right motor values:

```
steerOffset = steering - 1500
left  = throttle + steerOffset
right = throttle - steerOffset
```

RC signals are already tank-mixed by the transmitter and pass through
without a second mix.

### Power Scaling — 40% Proportional
Output is scaled proportionally so the full stick travel maps to 40%
of the ESC range. No wasted stick travel — every bit of movement
produces a proportional change in output.

```
output = 1500 + (input - 1500) × 40%
```

| Stick | Input (μs) | Output (μs) | Power |
|-------|-----------|-------------|-------|
| Center | 1500 | 1500 | 0% |
| 25% | 1625 | 1550 | 10% |
| 50% | 1750 | 1600 | 20% |
| 75% | 1875 | 1650 | 30% |
| 100% | 2000 | 1700 | 40% |

ESC signal range: **1300μs** (full reverse) to **1700μs** (full forward).

### Asymmetric EMA Smoothing
Exponential Moving Average with different time constants for acceleration
vs. deceleration:

- **Accelerating** (moving away from center): **800ms** — gentle ramp up
- **Decelerating** (returning to center): **400ms** — quick stop

This prevents jerky starts while ensuring the machine stops quickly to
avoid collisions.

### Deadbands
- **RC:** ±50μs around 1500 — values in 1450–1550 snap to neutral
- **Joystick:** ±30 around 512 (ADC) — prevents drift at center

### Failsafe
- If no RC signal for 500ms, both ESCs go to neutral (1500μs = stop)

---

## Power

| Rail  | Powers              | Notes                                      |
|-------|---------------------|--------------------------------------------|
| 5V    | RC Receiver VCC     | From Arduino 5V pin                        |
| 5V    | Boom Joystick VCC   | CONFIRMED 5V — Genie 101174GT, ~5mA draw  |
| GND   | All components      | Common ground is mandatory                 |
| VIN   | Arduino itself      | 7–12V from battery/BEC recommended        |

> **Joystick confirmed:** Genie 101174GT dual-axis potentiometer joystick.
> Runs on 5V, outputs 0–5V analog per axis, center ≈ 2.5V. Negligible
> current draw. Arduino 5V pin can power both the receiver and joystick.

---

## Joystick Details (Genie 101174GT)

- **Part number:** 101174 / 101174GT
- **Type:** Dual-axis with active electronics (Hall effect suspected)
- **Power:** 5V DC
- **Output:** 0–5V analog per axis (center position ≈ 2.5V)
- **Harness:** 6-wire adapter harness (3 per side)
- **Source:** Amazon ASIN B0F99C27BW (~$73)

---

## Wiring Diagram

```
                        ARDUINO NANO
                    ┌───────────────────┐
    RC CH1 ────────>│ D2 (left motor)   │
    RC CH2 ────────>│ D4 (right motor)  │
    RC CH5 ────────>│ D7 (override) D9 ~│──> Left Track ESC
                    │             D10  ~│──> Right Track ESC
  Joy Y (Thr) ─────>│ A0                │
  Joy X (Str) ─────>│ A1                │
                    │                   │
         5V ───────>│ 5V         GND   │<── Common GND
   Battery ────────>│ VIN               │
                    └───────────────────┘
```

---

## Pin Summary (Quick Reference)

```
D2  ← RC CH1 (Left motor, pre-mixed)   [PCINT18, interrupt-driven]
D4  ← RC CH2 (Right motor, pre-mixed)  [PCINT20, interrupt-driven]
D7  ← RC CH5 (Override switch, 3-pos)  [PCINT23, interrupt-driven]
A0  ← Joystick Y axis (Throttle)       [Analog input, 0–5V]
A1  ← Joystick X axis (Steering)       [Analog input, 0–5V]
D9  → Left Track ESC                   [Servo PWM output, 1300–1700μs]
D10 → Right Track ESC                  [Servo PWM output, 1300–1700μs]
5V  → RC Receiver + Joystick VCC
VIN ← Battery / BEC (7–12V)
GND → All components (common ground)
```

---

## Hardware

### Current: Arduino Nano V3 Clone
- **MCU:** ATmega328P (AVR, 16MHz, 2KB RAM, 32KB flash)
- **USB:** CH340G — requires driver install
- **RC reading:** Pin Change Interrupts (PCINT2) on D2, D4, D7
- **Board package:** `arduino:avr` → Board: Arduino Nano → ATmega328P (NEW bootloader, 115200 baud)
- **Upload:** `arduino-cli upload -p COM7 --fqbn arduino:avr:nano:cpu=atmega328`
- **Serial:** 115200 baud

### Target: Arduino Nano R4 [ABX00143]
- **MCU:** Renesas RA4M1 (Arm Cortex-M4, 48MHz, 32KB RAM, 256KB flash)
- **USB:** Native USB — no driver needed
- **RC reading:** `attachInterrupt()` on all digital pins
- **Board package:** `arduino:renesas_uno`

### Migration Plan (V3 → R4)
1. Install `arduino:renesas_uno` board package
2. Swap board — same form factor, same pin layout, no wiring changes
3. Update sketch: replace PCINT ISR with `attachInterrupt()` for RC channels
4. No CH340 driver needed — native USB

---

## Software

### Sketch: `sketches/rc_test/rc_test.ino`
- **Version:** V1.2 (interrupt-based, proportional scaling)
- **Flash:** 6.5 KB (21% of 30 KB)
- **RAM:** 317 bytes (15% of 2 KB)
- **Loop rate:** ~1000 Hz (interrupt-driven, no blocking reads)
- **Serial output:** 20 Hz at 115200 baud

### Live Plot: `live_plot.py`
- **7-panel real-time monitor** (matplotlib)
- Displays all inputs (D2, D4, D7, A0, A1) and both ESC outputs (L, R)
- ESC panels zoomed to 1200–1800μs with 40% limit markers
- Run: `python live_plot.py` or `Ctrl+Shift+B` in VS Code → "Live Plot"

### Tuning Constants (in sketch)
| Constant | Value | Description |
|----------|-------|-------------|
| `POWER_LIMIT_PCT` | 40 | Max output as % of full range |
| `SMOOTH_TAU_UP` | 800 ms | Acceleration smoothing |
| `SMOOTH_TAU_DOWN` | 400 ms | Deceleration smoothing |
| `RC_DEADBAND` | ±50 μs | RC neutral zone |
| `JOY_DEADBAND` | ±30 | Joystick neutral zone (ADC units) |
| `FAILSAFE_TIMEOUT` | 500 ms | Neutral if RC lost |

---

## Status
- [x] Joystick voltage/signal confirmed (Genie 101174GT, 5V, analog)
- [x] RC CH1 (D2) verified — left motor, center ~1500μs
- [x] RC CH2 (D4) verified — right motor, center ~1497μs
- [x] RC CH5 (D7) verified — 3-pos override, LOW ~972μs, HIGH ~2060μs
- [x] Pin Change Interrupts (PCINT) — all 3 RC channels read simultaneously
- [x] Tank mixer sketch written and tested (V1.2)
- [x] RC pass-through verified (transmitter pre-mixes)
- [x] Joystick tank mix verified
- [x] 40% proportional power scaling implemented
- [x] Asymmetric EMA smoothing (800ms up / 400ms down)
- [x] 7-panel live plot with ESC output monitoring
- [ ] Joystick harness wiring identified (reverse engineering in progress)
- [ ] Full wiring on final board
- [ ] Migrate to Nano R4 (interrupt-based reading)
- [ ] Field test
