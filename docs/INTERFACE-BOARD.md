# Interface Board — Build Plan

**Date:** 2026-05-23
**Purpose:** Permanent soldered replacement for the breadboard X.BUS merge
and S.BUS inverter circuits. All connections are servo-plug, plug-and-play.
Swap an ESC or Arduino without soldering.

---

## 1. Architecture

**Motor control:** PWM servo signals (Arduino D9/D10 → ESCs). Direct,
no processing, fastest response.

**Telemetry:** X.BUS polling (Arduino polls each ESC in turn, gets back
RPM, current, voltage, temperature, status). Used for PID feedback and
safety monitoring, NOT for motor control.

**RC input:** S.BUS from Radiolink R7FG receiver. Inverted UART — needs
an NPN inverter before the Arduino can read it.

---

## 2. X.BUS Merge Circuit

Per `docs/XBUS-PROTOCOL.md` §7, the official simplified circuit for
a 5V-tolerant Arduino:

```text
                    +5V
                     │
                   [4.7kΩ R1]  pull-up (holds bus idle HIGH)
                     │
                     ├──── ESC L Yellow (direct, no diode)
                     │
                     ├──── ESC R Yellow (direct, no diode)
    BUS NODE ────────┤
                     │
                     ├──── Arduino D0 / RX (direct)
                     │
                     └──[1kΩ R2]──── Arduino D1 / TX
```

**Why no diodes:** X.BUS is master-polled. Only one ESC responds at a
time (the one the Arduino addresses). No bus contention between ESCs,
so no isolation diodes needed. Both ESC yellow wires connect directly
to the same bus node. The 1kΩ series resistor on Arduino TX limits
current during the brief turnaround when Arduino releases and the ESC
starts responding.

**All grounds tied together** (ESC browns, Arduino GND).

**ESC red wires (BEC +5V) disconnected** on at least one ESC — never
connect two BECs to the same rail.

---

## 3. S.BUS Inverter Circuit

S.BUS is inverted UART (idle LOW). Arduino expects standard UART
(idle HIGH). One NPN common-emitter stage fixes this:

```text
                    +5V
                     │
                   [10kΩ R4]  collector pull-up
                     │
    S.BUS OUT ───────┤──── Q1 collector
    (to Arduino)     │
                     │
                     Q1 (2N3904)
                     │
    S.BUS IN ──[10kΩ R3]──── Q1 base
    (from RX)        │
                     │
                     Q1 emitter ──── GND
```

- S.BUS idle (LOW) → NPN off → collector pulled HIGH → Arduino sees idle HIGH
- S.BUS data (HIGH) → NPN on → collector pulled LOW → Arduino sees data LOW
- Clean inversion, matches standard UART polarity

---

## 4. Bill of Materials

| Ref | Part | Value | Qty | Purpose |
| --- | --- | --- | --- | --- |
| R1 | Resistor ¼W | **4.7 kΩ** | 1 | X.BUS pull-up to +5V |
| R2 | Resistor ¼W | **1 kΩ** | 1 | Arduino TX series (contention protection) |
| R3 | Resistor ¼W | **10 kΩ** | 1 | S.BUS NPN base resistor |
| R4 | Resistor ¼W | **10 kΩ** | 1 | S.BUS NPN collector pull-up |
| Q1 | NPN transistor | **2N3904** | 1 | S.BUS inverter |
| J1–J6 | Pin header | 3-pin 0.1″ male | 6 | Servo connectors |
| — | Stripboard | ~25 × 45 mm | 1 | Veroboard, copper strips horizontal |
| — | Hookup wire | 22 AWG | a few cm | Strip-to-strip jumpers if needed |

**No diodes.** The old design had Schottky diodes — those are not needed
per the X.BUS spec for master-polled operation.

---

## 5. Servo Connectors

Six 3-pin servo headers (signal / +5V / GND, Futaba/JR convention).

| Header | Label | Direction | Signal | +5V | GND |
| --- | --- | --- | --- | --- | --- |
| J1 | ESC-L-TEL | IN | ESC L X.BUS yellow | **N/C** (red wire cut) | ESC L brown |
| J2 | ESC-R-TEL | IN | ESC R X.BUS yellow | **N/C** (red wire cut) | ESC R brown |
| J3 | XBUS-TX | IN from Arduino | Arduino D1 (TX) → 1kΩ → bus | +5V from shield | GND |
| J4 | XBUS-RX | OUT to Arduino | Bus → Arduino D0 (RX) direct | +5V (shared rail) | GND |
| J5 | SBUS-IN | IN from receiver | RC receiver S.BUS signal | +5V → powers receiver | GND |
| J6 | SBUS-OUT | OUT to Arduino | Inverted S.BUS signal | +5V (shared rail) | GND |

**Power flow:** +5V enters the board from the Arduino servo shield
(through J3, J4, or J6). It powers the pull-up (R1), the inverter
(R4), and feeds the RC receiver through J5.

**ESC +5V pins (J1, J2):** Isolated from the +5V rail by track cuts.
The ESC red wires are already cut on the cable side, but the board
isolates them too as a safety net.

---

## 6. Stripboard Layout

Copper strips run **horizontally**. Viewed from the **component side**
(top). 7 strips × 15 columns.

### 6.1 Strip allocation

| Strip | Net | What connects here |
| --- | --- | --- |
| 1 | +5V rail | R1 top, R4 top, J3.+5V, J4.+5V, J5.+5V, J6.+5V |
| 2 | X.BUS bus node | R1 bottom, R2 bottom, J1.SIG, J2.SIG, J4.SIG (all direct) |
| 3 | TX input | J3.SIG, R2 top (track cut isolates from bus — R2 bridges strip 3→2) |
| 4 | S.BUS IN | J5.SIG, R3 left end |
| 5 | S.BUS OUT / collector | Q1 collector, R4 bottom, J6.SIG |
| 6 | NPN base | R3 right end, Q1 base |
| 7 | GND rail | Q1 emitter, J1.GND, J2.GND, J3.GND, J4.GND, J5.GND, J6.GND |

### 6.2 Track cuts

| # | Strip | Between cols | Reason |
| --- | --- | --- | --- |
| C1 | 1 | isolate J1.+5V | ESC L red wire safety |
| C2 | 1 | isolate J2.+5V | ESC R red wire safety |
| C3 | 3 | after J3.SIG area | Keep TX signal separate from anything else on strip 3 |

Only 3 cuts — much simpler than the previous design.

### 6.3 Components

| Ref | From → To | Notes |
| --- | --- | --- |
| R1 (4.7k) | Strip 1 → Strip 2 | Vertical, bridges +5V to bus |
| R2 (1k) | Strip 3 → Strip 2 | Vertical, bridges TX input to bus |
| R3 (10k) | Strip 4 → Strip 6 | Horizontal or angled, bridges S.BUS IN to NPN base |
| R4 (10k) | Strip 1 → Strip 5 | Vertical, bridges +5V to collector/S.BUS OUT |
| Q1 (2N3904) | E=Strip 7, B=Strip 6, C=Strip 5 | Flat side facing you; E-B-C left to right |

### 6.4 2N3904 pinout reminder

```text
   ╔═══╗
   ║flat║
   ╚╤╤╤╝
    E B C
```

Mount with flat side facing you (toward the component side). Emitter on
strip 7 (GND), base on strip 6, collector on strip 5 (S.BUS OUT).

---

## 7. Soldering Order

1. **Track cuts first.** Make all 3 cuts. Verify with multimeter
   (continuity/beep mode) — no beep across each cut.

2. **Resistors (R1, R2, R3, R4).** Insert, solder, snip. Verify
   values with meter before snipping leads.

3. **Transistor (Q1).** Confirm E-B-C orientation before soldering.
   Leave leads slightly long until verified, then snip.

4. **Continuity check (before headers):**
   - Strip 2 ↔ strip 1: ~4.7 kΩ (R1)
   - Strip 3 ↔ strip 2: ~1 kΩ (R2)
   - Strip 4 ↔ strip 6: ~10 kΩ (R3)
   - Strip 5 ↔ strip 1: ~10 kΩ (R4)
   - Strip 6 ↔ strip 7: ~0.7V forward drop (Q1 B-E junction, diode mode)

5. **Headers (J1–J6).** Tack one pin, check it's square, then solder
   remaining pins.

6. **Final check with headers:**
   - All GND pins beep together (strip 7)
   - J3.+5V, J4.+5V, J5.+5V, J6.+5V beep together (strip 1)
   - J1.+5V and J2.+5V do NOT beep to +5V rail (isolated)
   - J1.SIG and J2.SIG beep to J4.SIG (all on strip 2 / bus)
   - J3.SIG does NOT beep to J4.SIG directly (separated by R2 = 1kΩ)
   - J5.SIG to J6.SIG: no direct connection (NPN in between)

---

## 8. Wiring to Arduino Servo Shield

Plug servo cables from board headers to the Arduino shield headers:

| Board header | Arduino shield pin | Cable notes |
| --- | --- | --- |
| J3 (XBUS-TX) | D1 | Standard servo cable |
| J4 (XBUS-RX) | D0 | Standard servo cable |
| J6 (SBUS-OUT) | See note below | Standard servo cable |
| ESC L PWM | D9 (direct, not through this board) | ESC servo cable direct to shield |
| ESC R PWM | D10 (direct, not through this board) | ESC servo cable direct to shield |

**PWM cables go direct** from the ESC servo connectors to the Arduino
shield D9/D10 headers. No components needed — just plug in.

> **UNO R4 WiFi — both UARTs free:** X.BUS telemetry lands on Serial1
> (D0/D1) and S.BUS on `sbusUart` (SCI0, D11/D12), so both run at once
> with no contention. (The earlier D0 conflict came from a single-UART
> wiring where S.BUS and X.BUS both shared Serial1, before the SCI0
> second UART was adopted — no longer the case.)

---

## 9. Bring-up Test

1. **Visual inspection.** No solder bridges, no backwards transistor.
2. **Power test.** Plug J3 or J4 to Arduino shield (for +5V/GND).
   Meter reads ~5V on strip 1, 0V on strip 7.
3. **Bus pull-up.** Meter on J4.SIG: reads ~5V (R1 pulling up). Touch
   probe to GND, release — snaps back to 5V.
4. **S.BUS inverter.** Plug J5 to receiver, power system. Probe J6.SIG:
   idle = HIGH (~5V). Move a stick on the transmitter: see activity.
5. **One ESC telemetry.** Plug ESC L into J1, run
   `sketches/xbus_master/xbus_master.ino`. Confirm telemetry
   (RPM, voltage, current, temp).
6. **Both ESCs.** Add ESC R on J2. Confirm both addresses respond.
7. **Hot glue connectors** once everything is confirmed working
   (vibration protection for field use).
