# Excavator Track Controller — Wiring Guide V3.0

**Date:** 2026-03-30
**Board:** Arduino Nano R4 (Renesas RA4M1) + OSOYOO Nano IO Shield
**ESCs:** 2x XC E10 140A Sensored Brushless

> **The Nano R4 has 5V tolerant GPIO. No voltage dividers or level shifters
> needed for 5V signals (RC receiver, joystick, hall sensors).**

---

## Overview

Primary telemetry (RPM, voltage, current, temperature) is intended to come
from the ESC X.BUS, but this is unconfirmed (proprietary protocol, zero data
received during probing). Hall sensor taps and CS7581 current sensors are
supported as fallback. Both ESCs share one UART bus on D0 (X.BUS supports
up to 16 addressed ESCs).

---

## Components List

| # | Component | Qty | Purpose |
|---|-----------|-----|---------|
| 1 | Arduino Nano R4 | 1 | Main controller |
| 2 | OSOYOO Nano IO Shield | 1 | Pin breakout, 3-pin servo headers |
| 3 | XC E10 140A ESC | 2 | Motor speed controllers (Left + Right) |
| 4 | XC E3665 2500KV Motor | 2 | Track drive motors |
| 5 | Radiolink R7FG Receiver | 1 | RC input (4 channels used) |
| 6 | Genie 101174GT Joystick | 1 | Rider throttle/steering input |
| 7 | OVONIC 3S 15000mAh LiPo | 2 | Power (one per ESC) |

### Additional Parts Needed

| # | Part | Qty | Purpose | Approx Cost |
|---|------|-----|---------|-------------|
| 1 | 1N5819 Schottky diode | 2 | Diode-OR for shared X.BUS | $0.10 |
| 2 | 4.7kΩ resistor | 1 | Pull-up for X.BUS shared bus | $0.05 |
| 3 | Dupont jumper wires (M-M, M-F) | ~20 | Connections | $2 |

**Total additional cost: ~$3**

---

## Power Architecture

```
Battery 1 ──> ESC Left  (BEC set to 6.0V) ──> Arduino VIN pin
Battery 2 ──> ESC Right (BEC red wire DISCONNECTED)

Arduino VIN (6.0V) ──> Onboard regulator ──> 5V pin
                                                │
                        OSOYOO Nano IO Shield VCC rail
                                                │
                            ┌───────────────────┼───────────────────┐
                            │                   │                   │
                    RC Receiver 5V      Joystick 5V          (spare headers)
```

### Power Rules
1. **ESC Left BEC** → set to **6.0V** via XC-Link app → feeds Arduino **VIN**
2. **ESC Right BEC** → **CUT or disconnect the RED wire** from its servo connector
   - Only signal (white/orange) and ground (black/brown) connect to Arduino
   - Never connect two BECs to the same power rail
3. **Ground** → all device grounds connect to Arduino GND (shared ground bus on shield)

### Power Budget
| Device | Current |
|--------|---------|
| Arduino Nano R4 | ~100mA |
| RC Receiver | ~100mA |
| Joystick (2 axes) | ~17mA |
| **Total** | **~220mA** |

ESC BEC capacity: 5A continuous. Load is <5% of capacity.

---

## Pin Assignments

```
Pin   Signal                  Direction   Level Shifting
────  ──────────────────────  ──────────  ──────────────────────────
D0    X.BUS shared bus        ESC→Arduino Diode-OR, direct (5V tolerant)
D1    X.BUS TX (future)       Arduino→ESC None needed
D2    RC CH1 (left motor)     RX→Arduino  Direct (5V tolerant) [attachInterrupt]
D3    RC CH4 (control mode)   RX→Arduino  Direct (5V tolerant) [attachInterrupt]
D4    RC CH2 (right motor)    RX→Arduino  Direct (5V tolerant) [non-blocking poll]
D7    RC CH5 (override)       RX→Arduino  Direct (5V tolerant) [non-blocking poll]
D9    Left ESC servo PWM      Arduino→ESC None needed
D10   Right ESC servo PWM     Arduino→ESC None needed
A0    Joystick Y (throttle)   Joy→Arduino Direct (5V tolerant)
A1    Joystick X (steering)   Joy→Arduino Direct (5V tolerant)
────  ──────────────────────  ──────────  ──────────────────────────
D5    (available)
D6    (available)
D8    (available)
D11   (available)
D12   (available)
D13   (available)
A2    (available)
A3    (available)
A4    (available — I2C SDA)
A5    (available — I2C SCL)
```

**Interrupt limitation:** `attachInterrupt()` only works on D2 and D3 on the
Nano R4. D4 and D7 use non-blocking `PulseReader` polling for RC signal reading.

---

## Detailed Wiring — Signal by Signal

### 1. X.BUS Telemetry Bus (D0) — MOST CRITICAL

Both ESCs share one bus. Diode-OR prevents bus contention.

```
                                    5V
                                     │
                                   [4.7kΩ]  pull-up
                                     │
ESC Left  Yellow ──[1N5819]──────────┤
                   (anode→cathode→)  │
                                     ├──── Arduino D0 (Serial RX)
ESC Right Yellow ──[1N5819]──────────┤
                   (anode→cathode→)  │
                                    GND (not connected here,
                                         pull-up holds bus HIGH)

ESC Left  Brown ───── Arduino GND
ESC Right Brown ───── Arduino GND
ESC Left  Red   ───── Arduino VIN (BEC power, 6.0V)
ESC Right Red   ───── NOT CONNECTED (cut this wire!)
```

**Diode orientation:** Anode (band-less end) toward ESC, Cathode (banded end) toward Arduino.

**Why this works:** Each ESC can pull the bus LOW through its diode, but cannot
drive it HIGH (the 4.7k pull-up does that). When neither ESC transmits, the
pull-up holds the bus at 5V (idle/HIGH for UART). Only one ESC transmits at
a time (addressed by the master).

**Note:** X.BUS on D0 shares the hardware UART with USB Serial. You cannot
use Serial Monitor while X.BUS is connected to D0. Disconnect X.BUS for
bench debugging.

### 2. RC Receiver (D2, D3, D4, D7) — Direct Connection

The Nano R4 is 5V tolerant. RC receiver servo pulses connect directly.

```
RC Receiver CH1 signal ──→ Arduino D2  (left motor)   [attachInterrupt]
RC Receiver CH4 signal ──→ Arduino D3  (control mode)  [attachInterrupt]
RC Receiver CH2 signal ──→ Arduino D4  (right motor)   [non-blocking poll]
RC Receiver CH5 signal ──→ Arduino D7  (override)      [non-blocking poll]
```

**RC Receiver power:** Connect receiver VCC to Arduino 5V, GND to Arduino GND.
Use a spare IO shield header or direct jumper wires.

### 3. Joystick (A0, A1) — Direct Connection

The Genie 101174GT outputs 0-5V analog. Connects directly to Nano R4 ADC
(5V tolerant, 5V reference, 14-bit resolution).

```
Joystick Y output ──→ Arduino A0
Joystick X output ──→ Arduino A1
Joystick 5V       ──→ Arduino 5V
Joystick GND      ──→ Arduino GND
```

Full range 0-5V maps directly to the 14-bit ADC (0-16383). Center position
(~2.5V) reads approximately 8192.

### 4. ESC Servo PWM Output (D9, D10) — Direct

Arduino outputs 5V PWM. Direct connection to ESC servo input.

```
IO Shield D9  header signal pin ──→ ESC Left  servo signal wire
IO Shield D10 header signal pin ──→ ESC Right servo signal wire
```

**ESC ground:** The ESC servo connector GND must connect to Arduino GND.
Plug into the IO Shield D9/D10 headers (GND pin provides this).

**ESC VCC on servo connector:** Do NOT connect. The servo connector red wire
on both ESCs should not feed into the shield. Only signal + GND.
Exception: ESC Left's BEC red wire goes to VIN (see Power section), NOT to
the servo header.

### 5. Debug Serial — USB

The Nano R4 uses standard `Serial` over USB. No external USB-to-serial
adapter needed. Just connect USB-C cable to PC and open Serial Monitor
at 115200 baud.

**Note:** When X.BUS is connected to D0, Serial Monitor is unavailable
(D0 is shared between USB Serial and the hardware UART). Disconnect X.BUS
from D0 for debugging.

---

## OSOYOO Nano IO Shield Header Usage Map

```
DIGITAL SIDE:
  D0  ← X.BUS bus (via diode-OR)
  D1    (reserved for X.BUS TX)
  D2  ← RC CH1 (left motor, direct)     [attachInterrupt]
  D3  ← RC CH4 (control mode, direct)   [attachInterrupt]
  D4  ← RC CH2 (right motor, direct)    [non-blocking poll]
  D5    (available)
  D6    (available)
  D7  ← RC CH5 (override, direct)       [non-blocking poll]
  D8    (available)
  D9  → Left ESC servo signal + GND (RED wire NOT connected)
  D10 → Right ESC servo signal + GND (RED wire NOT connected)
  D11   (available)
  D12   (available)
  D13   (available)

ANALOG SIDE:
  A0  ← Joystick Y (direct)
  A1  ← Joystick X (direct)
  A2    (available)
  A3    (available)
  A4    (available — I2C SDA)
  A5    (available — I2C SCL)
```

---

## Wire Labels (for label maker)

| Label | Wire | From → To |
|-------|------|-----------|
| **XBUS-L** | ESC Left Yellow | ESC Left → Diode-OR → D0 |
| **XBUS-R** | ESC Right Yellow | ESC Right → Diode-OR → D0 |
| **XBUS-GND-L** | ESC Left Brown | ESC Left → Arduino GND |
| **XBUS-GND-R** | ESC Right Brown | ESC Right → Arduino GND |
| **BEC-6V** | ESC Left Red | ESC Left BEC → Arduino VIN |
| **RC1-L** | RC CH1 signal | Receiver CH1 → D2 |
| **RC2-R** | RC CH2 signal | Receiver CH2 → D4 |
| **RC4-MODE** | RC CH4 signal | Receiver CH4 → D3 |
| **RC5-OVRD** | RC CH5 signal | Receiver CH5 → D7 |
| **JOY-Y** | Joystick Y out | Joystick → A0 |
| **JOY-X** | Joystick X out | Joystick → A1 |
| **ESC-L** | Left ESC servo | D9 header → ESC Left signal |
| **ESC-R** | Right ESC servo | D10 header → ESC Right signal |

---

## ESC Configuration (via XC-Link Bluetooth app)

| Setting | ESC Left | ESC Right |
|---------|----------|-----------|
| BEC Voltage (Item 8) | **6.0V** | Any (BEC disconnected) |
| X.BUS Address (Item 16) | **0** | **1** |
| Communication BUS (Item 15) | X.BUS Protocol | X.BUS Protocol |
| Running Mode (Item 1) | Forward/Reverse | Forward/Reverse |
| Motor Direction (Item 10) | Set per track | Set per track |

---

## Safety Checklist (Before First Power-On)

- [ ] ESC Right BEC red wire is DISCONNECTED (cut or insulated)
- [ ] ESC Left BEC red wire goes to VIN, NOT to a servo header VCC
- [ ] All grounds are connected (ESCs, receiver, joystick, Arduino)
- [ ] Diode-OR cathodes (banded ends) face toward Arduino D0
- [ ] Pull-up resistor connects to 5V
- [ ] X.BUS addresses set differently on each ESC (0 and 1)
- [ ] ESC Left BEC set to 6.0V (not higher)
