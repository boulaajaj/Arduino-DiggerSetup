# Interface Board — Perfboard Solder Guide (UNO R4 WiFi)

**Date:** 2026-06-13   **Issue:** #43
**Supersedes the stripboard layout in `INTERFACE-BOARD.md` for the actual build.**
Schematic + BOM are unchanged from that doc; this adds a breadboard-style
perfboard layout and the **UNO R4 WiFi correction** (X.BUS and S.BUS run
**simultaneously** — both UARTs are free now, no "plug one at a time").

---

## 0. What this board does

Two tiny circuits on one board, all connections via 3-pin servo plugs:

1. **X.BUS telemetry merge** — both ESC telemetry wires + the Arduino's
   TX/RX share one half-duplex bus node, with a pull-up. (Read-only polling,
   master-arbitrated, so **no diodes** needed.)
2. **S.BUS inverter** — one 2N3904 flips the receiver's inverted S.BUS to
   normal UART polarity for the Arduino.

On the UNO R4 WiFi these use **two separate hardware UARTs**, so both are
plugged in and active at the same time:

| Signal | Arduino pin | Port |
| --- | --- | --- |
| X.BUS TX | **D1** | Serial1 (SCI2) |
| X.BUS RX | **D0** | Serial1 (SCI2) |
| S.BUS in (inverted) | **D12** | sbusUart (SCI0) |
| ESC PWM L / R | **D9 / D10** | Servo (direct, NOT through this board) |

---

## 1. Which perfboard to use → **the breadboard-style one** ✅

You have three. For this circuit:

| Board | Verdict |
| --- | --- |
| **Breadboard-style** (power rails + 5-hole columns) | **Use this.** The +5V/GND rails handle the many power/ground connections automatically, and each 5-hole column makes a perfect little signal node. Fewer jumpers, hard to get wrong. Matches your instinct. |
| Single isolated pads (24×24) | Also fine, most foolproof against accidental shorts, but you hand-wire *every* 5V/GND connection — more jumpers. Use only if you don't have the breadboard-style one. |
| Two-sided connected rows | Workable but the long connected rows invite accidental shared nets here. Skip for this. |

**Decision: breadboard-style perfboard.**

---

## 2. Bill of Materials (exact parts)

| Ref | Part | Value | Qty | Purpose |
| --- | --- | --- | --- | --- |
| Q1 | NPN transistor **2N3904** (TO-92) | — | 1 | S.BUS inverter |
| R1 | Resistor ¼W | **4.7 kΩ** (yellow-violet-red-gold) | 1 | X.BUS pull-up to +5V |
| R2 | Resistor ¼W | **1 kΩ** (brown-black-red-gold) | 1 | Arduino TX series (contention guard) |
| R3 | Resistor ¼W | **10 kΩ** (brown-black-orange-gold) | 1 | S.BUS NPN base resistor |
| R4 | Resistor ¼W | **10 kΩ** (brown-black-orange-gold) | 1 | S.BUS NPN collector pull-up |
| J1–J6 | 3-pin 0.1″ (2.54 mm) male header | — | 6 | Servo connectors (S / +5V / GND) |

> **Only 2N3904** for Q1 — do not substitute BC547 (different pin order →
> wiring mistakes). No diodes anywhere (master-polled X.BUS needs none).

---

## 3. Schematic

### X.BUS merge

```text
              +5V
               │
             [R1 4.7k]  pull-up — holds bus idle HIGH
               │
   BUS NODE ───┼──── J1 sig  (ESC-L X.BUS yellow)
               ├──── J2 sig  (ESC-R X.BUS yellow)
               ├──── J4 sig  → Arduino D0 (RX), direct
               └──[R2 1k]──── J3 sig → Arduino D1 (TX)
   (all GNDs common; ESC red/BEC wires NOT connected)
```

### S.BUS inverter (2N3904)

```text
              +5V
               │
             [R4 10k]  collector pull-up
               │
   J6 sig  ────┼──── Q1 collector      → Arduino D12 (inverted S.BUS out)
               │
              Q1 (2N3904)
               │
   J5 sig ──[R3 10k]── Q1 base         ← receiver S.BUS in
               │
              Q1 emitter ──── GND
```

---

## 4. Netlist — the source of truth ✅

The board is correct **if and only if** these pins end up electrically common.
After soldering, beep each net out with a multimeter. This is board-independent —
however you route it, match this:

| Net | Member pins / legs |
| --- | --- |
| **+5V** | shield 5V · R1(a) · R4(a) · J3.+5V · J4.+5V · J5.+5V · J6.+5V |
| **GND** | shield GND · Q1 **Emitter** · J1.GND · J2.GND · J3.GND · J4.GND · J5.GND · J6.GND |
| **XBUS_BUS** | R1(b) · R2(b) · J1.sig · J2.sig · J4.sig |
| **XBUS_TX** | R2(a) · J3.sig |
| **SBUS_IN** | R3(a) · J5.sig |
| **NPN_BASE** | R3(b) · Q1 **Base** |
| **SBUS_OUT** | R4(b) · Q1 **Collector** · J6.sig |

> **Not connected:** J1.+5V and J2.+5V (the ESC red/BEC wires). Leave those
> header pins floating — never tie two ESC BECs to a rail.

---

## 5. Breadboard-style placement (concrete)

Rails: top = **+5V**, bottom = **GND**. Jump shield 5V → +rail, shield GND →
−rail once. Each labeled column below is one 5-hole group = one signal net.

```text
 +5V RAIL ═══●═══●═══════════●══════════●═════  (R1,R4, and J3/J4/J5/J6 +5V pins jump up to here)
            │   │            │          │
          [R1] [R4]        J*.+5V ...  (jumpers)
            │   │
 col:      C2   C15        C5        C9        C12
 net:   XBUS_BUS SBUS_OUT XBUS_TX  SBUS_IN  NPN_BASE
            │      │  │       │        │        │
   R1(b)──→ C2     │  └Q1.C   R2(a)    R3(a)    R3(b),Q1.B
   R2(b)──→ C2   R4(b)        J3.sig   J5.sig
   J1.sig─→ C2   J6.sig
   J2.sig─→ C2
   J4.sig─→ C2
            R2 bridges C2──C5 ;  R3 bridges C9──C12
 GND RAIL ═══●════════════════════════════════  (Q1 emitter + all J*.GND jump down to here)
```

Practical notes:

- **XBUS_BUS (C2)** has exactly 5 members (R1, R2, J1, J2, J4) → fits one
  5-hole column. Land R1/R2 legs there and run a short jumper from each of
  J1.sig, J2.sig, J4.sig into C2.
- **R2 (1k)** bridges C2 → C5 (bus → TX). **R3 (10k)** bridges C9 → C12
  (S.BUS-in → base).
- **Q1 (2N3904)** flat-side toward you, legs **E-B-C left→right**: E to the
  GND rail, B to C12, C to C15.
- Place the 6 headers along the board edge so the cables hang off; route each
  header's **+5V pin to the +rail**, **GND pin to the −rail**, **sig pin to its
  net column** (except J1/J2 +5V = leave unconnected).

```text
   2N3904 (TO-92), flat side facing you:
        ___
       /   \
      | flat |
       \___/
       │ │ │
       E B C
```

---

## 6. Connectors → Arduino (UNO R4 WiFi)

| Header | Label | To Arduino | Notes |
| --- | --- | --- | --- |
| J1 | ESC-L-TEL | (ESC L X.BUS plug) | yellow=sig, brown=GND, **red=cut/NC** |
| J2 | ESC-R-TEL | (ESC R X.BUS plug) | same |
| J3 | XBUS-TX | **D1** | |
| J4 | XBUS-RX | **D0** | |
| J5 | SBUS-IN | (R7FG S.BUS) | +5V here powers the receiver |
| J6 | SBUS-OUT | **D12** | inverted S.BUS to sbusUart |
| — | ESC PWM L/R | **D9 / D10** | direct ESC→shield, NOT through this board |

**All three of D0, D1, D12 are connected at once** — both UARTs run together on
the UNO R4 WiFi. (This is the change from the old Nano R4 one-at-a-time note.)

---

## 7. Solder order

1. **Rails:** jumper shield 5V → +rail, shield GND → −rail.
2. **Resistors** R1, R2, R3, R4 (verify each value with the meter before snipping).
3. **Q1** — confirm E-B-C orientation before soldering; snip after verifying.
4. **Headers** J1–J6 — tack one pin, check square, solder the rest.
5. **Jumpers** for the signal nets (J1/J2/J4 sig → C2, header +5V→rail, GND→rail).

## 8. Continuity checks (before plugging into the Arduino)

- +5V rail ↔ GND rail: **no beep** (no short).
- XBUS_BUS ↔ +5V: ~**4.7 kΩ** (R1).
- XBUS_TX ↔ XBUS_BUS: ~**1 kΩ** (R2).
- SBUS_IN ↔ NPN_BASE: ~**10 kΩ** (R3).
- SBUS_OUT ↔ +5V: ~**10 kΩ** (R4).
- NPN_BASE ↔ GND: ~**0.7 V** forward drop (Q1 B-E, meter diode mode).
- All J*.GND beep together. J3/J4/J5/J6 +5V beep together. **J1.+5V / J2.+5V do NOT** beep to +5V.

## 9. Bring-up (with current firmware, V7.8)

The flight firmware already polls X.BUS (0x10) and reads S.BUS — no special test
sketch needed.

1. Power + plug J3→D1, J4→D0, J6→D12, ESC PWM→D9/D10.
2. Open the dashboard (192.168.4.1) or USB serial CSV.
3. Confirm S.BUS healthy (FS=0/Lost=0) and **both** ESCs report (OK0=1, OK1=1).
4. Wiggle/tap the board — telemetry should **stay solid** (this is the whole point
   vs the breadboard, which dropped an ESC under vibration).
5. Hot-glue the connectors once verified (field vibration protection).
