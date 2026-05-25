# Interface Board — Netlist for Schematic Tools

Import this into **EasyEDA** (free, browser-based) or **KiCad** (free, desktop).
Use `docs/interface-board.kicad_sch` for KiCad direct import.

---

## Components (BOM)

| Ref | Part | Value | Package | Qty |
|-----|------|-------|---------|-----|
| R1 | Resistor | 4.7kΩ ¼W | Axial | 1 |
| R2 | Resistor | 1kΩ ¼W | Axial | 1 |
| R3 | Resistor | 10kΩ ¼W | Axial | 1 |
| R4 | Resistor | 10kΩ ¼W | Axial | 1 |
| Q1 | NPN Transistor | 2N3904 | TO-92 (E-B-C) | 1 |
| J1 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| J2 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| J5 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| J3 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| J4 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| J6 | 3-pin header | Servo male | 2.54mm pitch | 1 |
| F6 | 3-pin header | Servo male | 2.54mm pitch | 1 |

---

## Nets (every electrical connection)

| Net Name | Connected Pins |
|----------|---------------|
| **BUS** | J1.SIG, J2.SIG, R1.pin2, R2.pin1, J4.SIG |
| **5V** | R1.pin1, R4.pin1, J5.VCC, F6.+5V |
| **GND** | J1.GND, J2.GND, J5.GND, J3.GND, J4.GND, J6.GND, F6.GND, Q1.E |
| **VIN** | J1.VCC, F6.VIN |
| **TX_OUT** | R2.pin2, J3.SIG |
| **SBUS_IN** | J5.SIG, R3.pin1 |
| **Q1_BASE** | R3.pin2, Q1.B |
| **SBUS_OUT** | Q1.C, R4.pin2, J6.SIG |
| **N/C** | J2.VCC (not connected — red wire cut on ESC Right) |

---

## Net Descriptions

```text
BUS ──────── The shared X.BUS node. Both ESC signals merge here.
             R1 pulls it to 5V (idle HIGH). R2 protects Arduino TX.
             J4 reads the bus (RX). J3 writes through R2 (TX).

5V ─────────  Regulated 5V from Arduino (arrives via F6, cable pin 8).
             Powers pull-ups R1, R4, and the receiver through J5.

GND ──────── Common ground. All connectors share this rail.

VIN ──────── 7.4V BEC from ESC Left. Passes through from J1 to F6.
             Goes to Arduino VIN via cable pin 6.

TX_OUT ───── Arduino D1 signal after R2. Connects to J3 servo out.

SBUS_IN ──── Raw S.BUS from receiver. Goes through R3 to Q1 base.

Q1_BASE ──── R3 output to Q1 base. Current-limited S.BUS drive.

SBUS_OUT ─── Inverted S.BUS. Q1 collector through R4 pull-up.
             Connects to J6 servo out → cable → Arduino D12.

N/C ──────── J2 VCC pin. Isolated. ESC Right red wire must be cut.
```

---

## How to Use in EasyEDA (free, browser-based)

1. Go to [easyeda.com](https://easyeda.com) → New Project → Schematic
2. Place components: 4 resistors, 1 NPN (2N3904), 7 connectors
3. Wire them using the net table above
4. Switch to PCB view → route on a single-layer board
5. Print 1:1 → use as perfboard soldering guide

## How to Use in KiCad (free, desktop)

1. Open `docs/interface-board.kicad_sch` in KiCad
2. Complete the wiring per the net table
3. Run DRC → assign footprints → generate PCB
