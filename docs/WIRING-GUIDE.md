# Excavator Track Controller — Wiring Guide V2.3

**Date:** 2026-03-29
**Board:** Arduino UNO Q (ABX00162) + Sensor Shield V5.0
**ESCs:** 2x XC E10 140A Sensored Brushless

> **ALL Arduino UNO Q GPIO pins are 3.3V. 5V on ANY pin WILL DAMAGE the board.**

---

## Overview

All telemetry (RPM, voltage, current, temperature) comes from the X.BUS.
No separate current sensors or hall sensors are used.
Both ESCs share one UART bus on D0 (X.BUS supports up to 16 addressed ESCs).

---

## Components List

| # | Component | Qty | Purpose |
|---|-----------|-----|---------|
| 1 | Arduino UNO Q (ABX00162) | 1 | Main controller |
| 2 | Sensor Shield V5.0 (ProtoSupplies) | 1 | Pin breakout, 3-pin servo headers |
| 3 | XC E10 140A ESC | 2 | Motor speed controllers (Left + Right) |
| 4 | XC E3665 2500KV Motor | 2 | Track drive motors |
| 5 | Radiolink R7FG Receiver | 1 | RC input (4 channels used) |
| 6 | Genie 101174GT Joystick | 1 | Rider throttle/steering input |
| 7 | OVONIC 3S 15000mAh LiPo | 2 | Power (one per ESC) |

### Additional Parts Needed

| # | Part | Qty | Purpose | Approx Cost |
|---|------|-----|---------|-------------|
| 1 | TXS0108E 8-ch level shifter module | 1 | 5V-to-3.3V for all digital inputs | $1-2 |
| 2 | 1N5819 Schottky diode | 2 | Diode-OR for shared X.BUS | $0.10 |
| 3 | 4.7kΩ resistor | 1 | Pull-up for X.BUS shared bus | $0.05 |
| 4 | 10kΩ resistor | 2 | Voltage dividers for joystick (top) | $0.05 |
| 5 | 6.8kΩ resistor | 2 | Voltage dividers for joystick (bottom) | $0.05 |
| 6 | USB-to-serial adapter (3.3V TTL) | 1 | Debug output from D8 | $3 |
| 7 | Dupont jumper wires (M-M, M-F) | ~20 | Connections | $2 |

**Total additional cost: ~$7**

---

## Power Architecture

```
Battery 1 ──> ESC Left  (BEC set to 6.0V) ──> Arduino VIN pin
Battery 2 ──> ESC Right (BEC red wire DISCONNECTED)

Arduino VIN (6.0V) ──> Onboard regulator ──> 5V pin
                                                │
                        Sensor Shield V5.0 VCC rail (SEL jumper INSTALLED)
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
3. **Sensor Shield SEL jumper** → **INSTALLED** (connects Arduino 5V to all header VCC pins)
4. **Ground** → all device grounds connect to Arduino GND (shared ground bus on shield)

### Power Budget
| Device | Current |
|--------|---------|
| Arduino UNO Q | ~200mA |
| RC Receiver | ~100mA |
| Joystick (2 axes) | ~17mA |
| **Total** | **~320mA** |

ESC BEC capacity: 5A continuous. Load is <7% of capacity.

---

## Pin Assignments

```
Pin   Signal                  Direction   Level Shifting
────  ──────────────────────  ──────────  ──────────────────────────
D0    X.BUS shared bus        ESC→Arduino Diode-OR + voltage divider
D1    X.BUS TX (future)       Arduino→ESC None needed (3.3V out)
D2    RC CH1 (left motor)     RX→Arduino  TXS0108E channel 1
D3    RC CH4 (control mode)   RX→Arduino  TXS0108E channel 2
D4    RC CH2 (right motor)    RX→Arduino  TXS0108E channel 3
D7    RC CH5 (override)       RX→Arduino  TXS0108E channel 4
D8    Debug serial TX         Arduino→out None needed (3.3V out)
D9    Left ESC servo PWM      Arduino→ESC None needed (3.3V out)
D10   Right ESC servo PWM     Arduino→ESC None needed (3.3V out)
A0    Joystick Y (throttle)   Joy→Arduino Resistor divider (10k/6.8k)
A1    Joystick X (steering)   Joy→Arduino Resistor divider (10k/6.8k)
────  ──────────────────────  ──────────  ──────────────────────────
D5    (available)
D6    (available)
D11   (available)
D12   (available)
D13   (available)
A2    (available)
A3    (available)
A4    (available — I2C SDA)
A5    (available — I2C SCL)
PG8   RESERVED by Router bridge — DO NOT USE
```

---

## Detailed Wiring — Signal by Signal

### 1. X.BUS Telemetry Bus (D0) — MOST CRITICAL

Both ESCs share one bus. Diode-OR prevents bus contention.

```
                                    3.3V
                                     │
                                   [4.7kΩ]  pull-up
                                     │
ESC Left  Yellow ──[1N5819]──────────┤
                   (anode→cathode→)  │
                                     ├──── Arduino D0 (USART1 RX)
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
pull-up holds the bus at 3.3V (idle/HIGH for UART). Only one ESC transmits at
a time (addressed by the master).

**Voltage note:** The Schottky diode drops ~0.2-0.3V. If ESC outputs 5V logic,
the bus sees ~4.7V HIGH. The 4.7k pull-up to 3.3V overrides this to 3.3V when
idle. During active LOW from ESC, bus goes to ~0.3V (valid LOW). Since the
pull-up is to 3.3V, the bus never exceeds 3.3V — no additional voltage divider
needed on D0 when using this diode-OR with 3.3V pull-up.

### 2. RC Receiver (D2, D3, D4, D7) — via TXS0108E

The RC receiver outputs 5V servo pulses. Route through the level shifter.

```
TXS0108E Module:
  VA  ── 3.3V (from Arduino 3.3V pin)
  VB  ── 5V (from Sensor Shield 5V)
  GND ── Arduino GND
  OE  ── 3.3V (always enabled)

  B1 ←── RC Receiver CH1 signal (5V)     A1 ──→ Arduino D2
  B2 ←── RC Receiver CH4 signal (5V)     A2 ──→ Arduino D3
  B3 ←── RC Receiver CH2 signal (5V)     A3 ──→ Arduino D4
  B4 ←── RC Receiver CH5 signal (5V)     A4 ──→ Arduino D7
  B5-B8  (spare — available for future)  A5-A8 (spare)
```

**RC Receiver power:** Plug one servo cable from receiver into a spare Sensor
Shield 3-pin header (any unused D-pin header). The VCC pin provides 5V, GND
provides ground. Only the power pins matter — the signal pin of that header is
unused.

**RC signal routing:** The receiver's signal wires go to the TXS0108E B-side.
The A-side outputs go to Arduino pins D2/D3/D4/D7. Use short jumper wires.

### 3. Joystick (A0, A1) — Resistor Dividers

The Genie 101174GT outputs 0-5V analog. Must be divided to 0-3.3V for the ADC.

```
Joystick Y output ──[10kΩ]──┬──[6.8kΩ]── GND
                             └──→ Arduino A0

Joystick X output ──[10kΩ]──┬──[6.8kΩ]── GND
                             └──→ Arduino A1

Joystick 5V  ── Sensor Shield A0 or A1 header VCC pin (5V)
Joystick GND ── Sensor Shield A0 or A1 header GND pin
```

**Math:** Vout = 5V × 6.8k / (10k + 6.8k) = 5V × 0.405 = **2.02V at full deflection**
(from a 5V input). Center position (~2.5V in) produces ~1.01V out. Full range
maps to 0-2.02V, which is within the 0-3.3V ADC range. The 14-bit ADC gives
plenty of resolution even with the reduced range.

**Alternative ratio for wider range:** Use 4.7kΩ + 10kΩ instead:
Vout = 5V × 10k / (4.7k + 10k) = **3.4V** — just barely above 3.3V limit.
Safer: Use **5.1kΩ + 10kΩ** → Vout = 5V × 10k / (5.1k + 10k) = **3.31V** ≈ 3.3V max.

### 4. ESC Servo PWM Output (D9, D10) — Direct

Arduino outputs 3.3V PWM. Most ESCs accept 3.3V as valid HIGH (threshold is
typically 0.6 × VCC = 3.0V for 5V CMOS). No level shifting needed.

```
Sensor Shield D9  header signal pin ──→ ESC Left  servo signal wire
Sensor Shield D10 header signal pin ──→ ESC Right servo signal wire
```

**ESC ground:** The ESC servo connector GND must connect to Arduino GND.
Plug into the Sensor Shield D9/D10 headers (GND pin provides this).

**ESC VCC on servo connector:** Do NOT connect. The servo connector red wire
on both ESCs should not feed into the shield. Only signal + GND.
Exception: ESC Left's BEC red wire goes to VIN (see Power section), NOT to
the servo header.

### 5. Debug Serial (D8) — Direct

```
Sensor Shield D8 header signal pin ──→ USB-to-serial adapter RX
Sensor Shield D8 header GND pin   ──→ USB-to-serial adapter GND
USB-to-serial adapter ──→ PC USB port (open terminal at 115200 baud)
```

---

## Sensor Shield V5.0 Header Usage Map

```
DIGITAL SIDE (left of board):
  D0  [S/V/G] ← X.BUS bus (via diode-OR, NOT through shield header)
  D1  [S/V/G]   (reserved for X.BUS TX)
  D2  [S/V/G] ← RC CH1 via TXS0108E A1
  D3  [S/V/G] ← RC CH4 via TXS0108E A2
  D4  [S/V/G] ← RC CH2 via TXS0108E A3
  D5  [S/V/G]   (available)
  D6  [S/V/G]   (available)
  D7  [S/V/G] ← RC CH5 via TXS0108E A4
  D8  [S/V/G] → Debug TX to USB-serial adapter
  D9  [S/V/G] → Left ESC servo signal + GND (RED wire NOT connected)
  D10 [S/V/G] → Right ESC servo signal + GND (RED wire NOT connected)
  D11 [S/V/G]   (available)
  D12 [S/V/G]   (available)
  D13 [S/V/G]   (available)

ANALOG SIDE (right of board):
  A0  [S/V/G] ← Joystick Y (signal via divider, VCC/GND for power)
  A1  [S/V/G] ← Joystick X (signal via divider, VCC/GND for power)
  A2  [S/V/G]   (available)
  A3  [S/V/G]   (available)
  A4  [S/V/G]   (available — I2C SDA)
  A5  [S/V/G]   (available — I2C SCL)

SPECIAL HEADERS:
  I2C [SDA/SCL/V/G]  (available)
  UART [RX/TX/V/G]   (shared with D0/D1 — used by X.BUS)

POWER:
  SEL jumper: INSTALLED (5V from Arduino powers all VCC pins)
  Screw terminal: NOT USED
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
| **RC1-L** | RC CH1 signal | Receiver CH1 → TXS0108E B1 |
| **RC2-R** | RC CH2 signal | Receiver CH2 → TXS0108E B3 |
| **RC4-MODE** | RC CH4 signal | Receiver CH4 → TXS0108E B2 |
| **RC5-OVRD** | RC CH5 signal | Receiver CH5 → TXS0108E B4 |
| **JOY-Y** | Joystick Y out | Joystick → Divider → A0 |
| **JOY-X** | Joystick X out | Joystick → Divider → A1 |
| **ESC-L** | Left ESC servo | D9 header → ESC Left signal |
| **ESC-R** | Right ESC servo | D10 header → ESC Right signal |
| **DBG-TX** | Debug serial | D8 header → USB-serial RX |

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
- [ ] Sensor Shield SEL jumper is INSTALLED
- [ ] No 5V signal wires connect directly to Arduino GPIO (all go through level shifter or divider)
- [ ] Diode-OR cathodes (banded ends) face toward Arduino D0
- [ ] Pull-up resistor connects to 3.3V (NOT 5V)
- [ ] Joystick divider outputs measured with multimeter (must be < 3.3V at full deflection)
- [ ] All grounds are connected (ESCs, receiver, joystick, level shifter, Arduino)
- [ ] X.BUS addresses set differently on each ESC (0 and 1)
- [ ] ESC Left BEC set to 6.0V (not higher)
