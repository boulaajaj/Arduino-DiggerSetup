# Arduino Nano Tank Mixer — Project Plan

## What This Does
An Arduino Nano sits between an RC receiver and two ESCs, implementing a
**tank-style mixer** with a secondary joystick input (from a boom lift).
The RC transmitter always takes priority. A third RC channel acts as a
joystick enable/override switch.

---

## Inputs (5 channels)

| Ch | Source          | Signal Type      | Pin  | Description                        |
|----|-----------------|------------------|------|------------------------------------|
| 1  | RC Receiver     | Servo PWM        | D2   | Throttle — forward / backward      |
| 2  | RC Receiver     | Servo PWM        | D4   | Steering — left / right            |
| 5  | RC Receiver     | Servo PWM        | D7   | Override switch (3-pos, see below) |
| —  | Boom Joystick   | Analog 0–5V      | A0   | Joystick throttle (Y axis)         |
| —  | Boom Joystick   | Analog 0–5V      | A1   | Joystick steering (X axis)         |

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
non-neutral input, RC takes 100% priority and joystick is ignored.

**Mode 3 — Remote + Joystick (blended):** Both inputs are active
simultaneously. RC override is reduced to ~50%, so the joystick retains
partial authority even when RC is giving input. Useful for fine-tuning
position while someone assists via remote.

---

## Tank Mix Formula

The mixer converts throttle + steering into individual track speeds:

```
Left  ESC = Throttle + Steering   (clamped to valid ESC range)
Right ESC = Throttle - Steering   (clamped to valid ESC range)
```

Turning in place: full steering + zero throttle
Forward straight: full throttle + zero steering

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
- **Type:** Dual-axis potentiometer (contact-based sensor)
- **Power:** 5V DC
- **Output:** 0–5V analog per axis (center position ≈ 2.5V)
- **Current:** ~1–5mA (potentiometer voltage divider)
- **Harness:** Comes with adapter harness — identify wires with multimeter
- **Source:** Amazon ASIN B0F99C27BW (~$73)

---

## Wiring Diagram

```
                        ARDUINO NANO
                    ┌───────────────────┐
    RC CH1 ────────>│ D2                │
    RC CH2 ────────>│ D4                │
    RC CH5 ────────>│ D7           D9  ~│────────> Left Track ESC
                    │             D10  ~│────────> Right Track ESC
  Joy Y (Thr) ─────>│ A0                │
  Joy X (Str) ─────>│ A1                │
                    │                   │
         5V ───────>│ 5V         GND   │<──── Common GND (all devices)
   Battery ────────>│ VIN               │
                    └───────────────────┘

  RC Receiver                        ESCs
  ┌──────────┐                   ┌──────────┐
  │ CH1 ─────┼──> D2             │ Left ESC │<── D9
  │ CH2 ─────┼──> D4             └──────────┘
  │ CH5 ─────┼──> D7             ┌──────────┐
  │ VCC <────┼─── 5V             │Right ESC │<── D10
  │ GND <────┼─── GND            └──────────┘
  └──────────┘
  Boom Joystick
  ┌──────────┐
  │ Y out ───┼──> A0
  │ X out ───┼──> A1
  │ VCC <────┼─── 5V
  │ GND <────┼─── GND
  └──────────┘
```

---

## Pin Summary (Quick Reference)

```
D2  ← RC Receiver CH1 (Throttle)       [PWM input, INT0]
D4  ← RC Receiver CH2 (Steering)       [PWM input]
D7  ← RC Receiver CH5 (Override sw)    [PWM input, 3-position]
A0  ← Joystick Y axis (Throttle)       [Analog input, 0–5V]
A1  ← Joystick X axis (Steering)       [Analog input, 0–5V]
D9  → Left Track ESC                   [PWM output ~]
D10 → Right Track ESC                  [PWM output ~]
5V  → RC Receiver + Joystick VCC
VIN ← Battery / BEC (7–12V)
GND → All components (common ground)
```

---

## Hardware

### Current: Arduino Nano V3 Clone (testing/development)
- **MCU:** ATmega328P (AVR, 16MHz, 2KB RAM, 32KB flash)
- **USB:** CH340G — requires driver install
- **Interrupts:** Only D2 (INT0) and D3 (INT1)
- **Board package:** `arduino:avr` → Board: Arduino Nano → Processor: ATmega328P (NEW bootloader, 115200 baud)
- **Upload via CLI:** `arduino-cli upload -p COM7 --fqbn arduino:avr:nano:cpu=atmega328`
- **Port:** COM7
- **Limitation:** `pulseIn()` blocking reads — confirmed: reading order affects which channels get data. Only first-read channel is reliable. All 3 channels work individually.

### Target: Arduino Nano R4 [ABX00143] (arriving in mail)
- **MCU:** Renesas RA4M1 (Arm Cortex-M4, 48MHz, 32KB RAM, 256KB flash)
- **USB:** Native USB — no driver needed
- **Interrupts:** All digital pins support `attachInterrupt()`
- **Board package:** `arduino:renesas_uno` (install via Boards Manager when R4 arrives)
- **Advantage:** Interrupt-based RC reading on all pins — non-blocking, all 3 channels read simultaneously
- **RGB LED:** Built-in programmable LED (can use for status indicators)
- **Logic:** 5V — all existing peripherals (joystick, receiver, ESCs) stay compatible

### Migration Plan (V3 → R4)
1. Install `arduino:renesas_uno` board package in Arduino IDE
2. Swap board on breadboard — same form factor, same pin layout
3. No wiring changes needed — pin assignments (D2, D4, D7, A0, A1, D9, D10) are identical
4. Update sketch: replace `pulseIn()` with `attachInterrupt()` for RC channels
5. No CH340 driver needed — native USB auto-detected
6. Select board: Tools → Board → Arduino Renesas UNO R4 Boards → Arduino Nano R4

---

## Open Questions Before Build

1. ~~**Joystick voltage** — confirmed 5V, potentiometer, 0–5V analog output~~
2. **ESC type** — Standard RC ESCs expecting 1000–2000μs servo pulses?
3. **Power source** — What battery/BEC will power the Arduino and receiver?
4. **Failsafe** — What should the ESCs do if the RC signal is lost?

---

## Status
- [x] Joystick voltage/signal confirmed (Genie 101174GT, 5V, analog)
- [x] RC CH1 (D2) verified — throttle, center ~1500μs
- [x] RC CH2 (D4) verified — steering, center ~1497μs
- [x] RC CH5 (D7) verified — 3-pos override, LOW ~972μs, HIGH ~2060μs
- [x] pulseIn() blocking confirmed — Nano R4 interrupt migration required
- [ ] Wiring complete
- [ ] Sketch written (V1 — pulseIn, Nano V3)
- [ ] Tank mix tested on bench
- [ ] Migrate to Nano R4 (interrupt-based reading)
- [ ] Field test
