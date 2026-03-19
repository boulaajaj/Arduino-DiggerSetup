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
RC (attachInterrupt ISR) ───────────────────────────┐
                                                     ├─ Mode Select ─┐
Joystick (analogRead 14-bit) ─ Deadband ─ Map ─ Mix ┘               │
                                                                     v
                                          Exponential Power Scale (50%)
                                                                     │
                                                                     v
                                            Deceleration-Only Smoothing
                                                  (instant up / 300ms down)
                                                                     │
                                                                     v
                                                  Servo Output (D9, D10)
```

### RC Reading — Hardware Interrupts (attachInterrupt)
All 3 RC channels use `attachInterrupt()` with CHANGE mode. The Nano R4
supports hardware interrupts on all digital pins — no PCINT register
manipulation needed. Main loop runs at 1000+ Hz with no missed pulses.

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

### Exponential Power Scaling — 50% with Curve
Output uses an exponential curve (exponent 2.5) so full stick travel
maps to 50% of the ESC range with fine control at low inputs.

```
normalized = (input - 1500) / 500        // -1.0 to +1.0
curved     = sign × |normalized|^2.5     // exponential curve
output     = 1500 + curved × 500 × 50%   // scaled to power limit
```

| Stick | Input (μs) | Output (μs) | Power |
|-------|-----------|-------------|-------|
| Center | 1500 | 1500 | 0% |
| 10% | 1550 | 1500.8 | ~0.3% |
| 25% | 1625 | 1507.8 | ~3.1% |
| 50% | 1750 | 1544.2 | ~17.7% |
| 75% | 1875 | 1618.5 | ~37.0% |
| 100% | 2000 | 1750 | 50% |

ESC signal range: **1250μs** (full reverse) to **1750μs** (full forward).

### Deceleration-Only Smoothing
- **Accelerating** (moving away from center): **INSTANT** — no delay
- **Decelerating** (returning to center): **300ms EMA** — smooth, safe stop

The exponential curve provides natural fine control at low inputs,
replacing the need for acceleration ramping. Deceleration ramp prevents
jerky stops and keeps operators safe.

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

### Current: Arduino Nano R4 [ABX00143]
- **MCU:** Renesas RA4M1 (Arm Cortex-M4, 48MHz, 32KB RAM, 256KB flash)
- **USB:** Native USB on COM8 — no driver needed
- **RC reading:** `attachInterrupt()` on D2, D4, D7 (hardware interrupts)
- **ADC:** 14-bit (0-16383) — 16x joystick resolution vs Nano V3
- **Board package:** `arduino:renesas_uno` (v1.5.3)
- **FQBN:** `arduino:renesas_uno:nanor4`
- **Upload:** `arduino-cli upload -p COM8 --fqbn arduino:renesas_uno:nanor4 sketches/rc_test`
- **Serial:** 115200 baud

### Previous: Arduino Nano V3 Clone (retired)
- **MCU:** ATmega328P (AVR, 16MHz, 2KB RAM, 32KB flash)
- **USB:** CH340G on COM7
- **RC reading:** Pin Change Interrupts (PCINT2)
- **ADC:** 10-bit (0-1023)

---

## Software

### Sketch: `sketches/rc_test/rc_test.ino`
- **Version:** V2.0 (Nano R4, exponential curve, instant response)
- **Flash:** 45.7 KB (17% of 256 KB)
- **RAM:** 4.4 KB (13% of 32 KB)
- **Loop rate:** ~1000+ Hz (interrupt-driven, no blocking reads)
- **Serial output:** 20 Hz at 115200 baud

### Live Plot: `live_plot.py`
- **7-panel real-time monitor** (matplotlib)
- Displays all inputs (D2, D4, D7, A0, A1) and both ESC outputs (L, R)
- ESC panels zoomed to 1200–1800μs with 40% limit markers
- Run: `python live_plot.py` or `Ctrl+Shift+B` in VS Code → "Live Plot"

### Tuning Constants (in sketch)
| Constant | Value | Description |
|----------|-------|-------------|
| `POWER_LIMIT_PCT` | 50 | Max output as % of full range |
| `EXP_CURVE` | 2.5 | Exponential exponent (higher = finer low-end) |
| `SMOOTH_TAU_DOWN` | 300 ms | Deceleration smoothing (accel is instant) |
| `RC_DEADBAND` | ±50 μs | RC neutral zone |
| `JOY_DEADBAND` | ±480 | Joystick neutral zone (~3% of 14-bit range) |
| `JOY_CENTER` | 8192 | 14-bit ADC midpoint |
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
- [x] 40% proportional power scaling implemented (V1.2)
- [x] Asymmetric EMA smoothing — V1.2 (replaced by exponential in V2.0)
- [x] 7-panel live plot with ESC output monitoring
- [x] Migrate to Nano R4 (attachInterrupt, 14-bit ADC)
- [x] Exponential control curve (exponent 2.5, instant response)
- [x] 50% power cap with deceleration-only ramp (300ms)
- [ ] CH4 emergency kill switch (safety — disable all ESC output)
- [ ] Joystick harness wiring identified (reverse engineering in progress)
- [ ] Full wiring on final board
- [ ] Field test
