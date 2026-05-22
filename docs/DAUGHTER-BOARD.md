# Telemetry / S.BUS Daughter Board — Build Plan

**Date:** 2026-05-22
**Board type:** stripboard (Veroboard) — copper strips run **horizontally**
**Purpose:** Replace the breadboard X.BUS merge + S.BUS inverter with a soldered
daughter board that plugs into the main perf board.

---

## 1. Topology — Confirmation

X.BUS is **single-wire half-duplex** (confirmed from
`docs/XBUS-PROTOCOL.md` §3.1):

- 115200 8N1, **non-inverted** UART (idle HIGH)
- Bus held HIGH by a single shared pull-up resistor
- Master-polled — slaves never transmit unsolicited
- All participants (Arduino TX, Arduino RX, both ESC yellow wires) sit on
  **the same physical node**

Consequence for the merge:

- A passive diode-OR is sufficient for **receive** (ESC → Arduino), provided
  the diodes are oriented **cathode toward each ESC, anode toward the merged
  bus**, with a 5 V pull-up on the merged bus.
- For the **transmit** path (Arduino → ESCs), the Arduino TX pin (D1) must
  reach the same bus node. A 1 kΩ series resistor between D1 and the bus is
  enough to prevent contention current during turnaround. No diode required
  on D1 because the master-slave protocol guarantees only one driver pulls
  LOW at a time.
- Arduino RX (D0) taps the bus **directly**, no resistor, no diode.

> **Note on the repo's old wiring guide.** `docs/WIRING-GUIDE.md` lines
> 126–144 spec the diodes "anode toward ESC, cathode toward Arduino." With
> a pull-up to 5 V on the Arduino side that orientation is reverse-biased
> when the ESC drives LOW and no UART data would reach the Arduino. Either
> the breadboard is wired the opposite of what the doc says, or the doc is
> wrong. The orientation used in this build (anode → bus, cathode → ESC)
> is the electrically correct one.

S.BUS from the Radiolink R7FG is inverted UART (idle LOW). One NPN
common-emitter stage inverts and level-shifts it cleanly.

---

## 2. Bill of Materials

| Ref | Part | Value / Type | Qty | Notes |
| --- | --- | --- | --- | --- |
| D1, D2 | Schottky diode | 1N5819 (axial) or BAT54 (SMD) | 2 | Bus-side merge, anode → bus |
| R1 | Resistor | **4.7 kΩ** ¼ W | 1 | X.BUS pull-up to +5 V |
| R2 | Resistor | **10 kΩ** ¼ W | 1 | S.BUS inverter base resistor |
| R3 | Resistor | **10 kΩ** ¼ W | 1 | S.BUS inverter collector pull-up |
| Q1 | NPN transistor | **2N3904** (or BC547) | 1 | S.BUS inverter |
| J1–J5 | Pin header | 3-pin 0.1″ male, straight | 5 | Servo connector pitch |
| — | Hookup wire | 22 AWG insulated | a few cm | Strip-to-strip jumpers |
| — | Stripboard | ~25 mm × 50 mm, ≥9 strips × ≥16 holes | 1 | Veroboard |

**Main-board addition (not on the daughter board):**

| Ref | Part | Value | Notes |
| --- | --- | --- | --- |
| R_TX | Resistor | **1 kΩ** ¼ W | In series with Arduino D1; the D1 end of this resistor is tied to D0 and to the wire going to J4 SIG |

---

## 3. Schematic (logical)

```text
                                +5V (from J4 pin 2 / J5 pin 2 via rails)
                                  │
                                  │
                              [R1 4.7k]   X.BUS pull-up
                                  │
                          ┌───────┴────────┬───────────┐
                          │   BUS NODE     │           │
                          │                │           │
              ESC L SIG ──┤<─[D1 1N5819]──┤            │
              (J1.1)      │ K        A    │            │
                          │                │           │
              ESC R SIG ──┤<─[D2 1N5819]──┤            │
              (J2.1)      │ K        A    │            │
                          │                │           │
                          └────────────────┴───────────┴── J4.1  (to Arduino)
                                                            └─→ D0 direct
                                                            └─→ D1 via [1k] R_TX
                                                                  (R_TX on main board)


              S.BUS IN ──[R2 10k]── B │ Q1 (2N3904)
              (J3.1)                  │ ┌──── C ──── [R3 10k] ──── +5V
                                      │ │
                                      │ E ──── GND
                                      │ │
                                      └─┴── S.BUS OUT (to J5.1 → Arduino)

   +5V RAIL: J3.2, J4.2, J5.2 (J1.2 and J2.2 isolated — ESC red wires are cut)
   GND RAIL: J1.3, J2.3, J3.3, J4.3, J5.3, Q1 emitter
```

Notes on diode polarity (`──┤<──`):

- `K` is the cathode (banded end of 1N5819)
- `A` is the anode
- The cathodes face the ESC side; the anodes face the merged bus node.

---

## 4. Stripboard Layout

Board orientation: **copper strips horizontal**, viewed from the
**component side** (top). 9 strips (rows) × 16 holes (columns). Strip 1
is the top, strip 9 is the bottom. Columns numbered 1 (left) to 16
(right).

### 4.1 Strip allocation

| Strip | Function | Notes |
| --- | --- | --- |
| 1 | Spare (top edge) | Mounting / mechanical |
| 2 | X.BUS BUS NODE | J1.1, J2.1, J4.1, R1 lower, D1 anode, D2 anode all land here |
| 3 | ESC L cathode tail | Short — only the D1 cathode and J1.1 → strip 2 jumper land here. *Cut at col 5.* |
| 4 | ESC R cathode tail | Same, for D2 and J2.1. *Cut at col 7.* |
| 5 | S.BUS IN signal | J3.1, R2 left. *Cut at col 13* to keep J5.1 separated. |
| 6 | NPN base / R2 right | R2 right end, Q1 base |
| 7 | NPN collector / S.BUS OUT | Q1 collector, R3 lower, J5.1 |
| 8 | +5V RAIL | R1 upper, R3 upper, J3.2, J4.2, J5.2. **Cut at col 3 and col 5** to isolate J1.2 and J2.2 from the rail. |
| 9 | GND RAIL | Q1 emitter, J1.3, J2.3, J3.3, J4.3, J5.3 |

### 4.2 Top-down picture

Each cell is one hole. `■` = header pin. `R` = resistor lead. `D` = diode
lead (`>` = cathode/banded end, `<` = anode). `T` = transistor lead.
`×` = track cut (cut the copper strip *between* this hole and the one to
its right). `─` = continuous copper strip.

```text
        Col:  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16
Strip 1 :    ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─     (spare)
Strip 2 :    ─   ─   ─   R   ─   <   ─   <   ─   ─   ─   ─   ─   ─   ─   ─     (BUS NODE)
Strip 3 :    ─   ─   ─   ─   >   ×                                              (ESC L cathode → strip 2 via jumper at col 4)
Strip 4 :    ─   ─   ─   ─   ─   ─   >   ×                                      (ESC R cathode → strip 2 via jumper at col 6)
Strip 5 :                                                R   ─   ─   ─   ×     (S.BUS IN ── R2 left)
Strip 6 :                                                ─   R   T                (R2 right, Q1 base)
Strip 7 :                                                ─   ─   T   ─   R       (Q1 collector, R3 lower, J5.1)
Strip 8 :    ×       ×       ─   R   ─   ─   ─   ─   ─   ─   ─   R   ─   ─     (+5V RAIL with cuts isolating J1.2/J2.2)
Strip 9 :    ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─   ─     (GND RAIL)

Header pins (each header spans 3 strips: SIG / +5V / GND):
                  ↓ J1                ↓ J2          ↓ J4              ↓ J3              ↓ J5
              col 2            col 4         col 6            col 10           col 14

       J1.1 (SIG) on strip 3,  J1.2 (+5V) on strip 8 (isolated), J1.3 (GND) on strip 9
       J2.1 (SIG) on strip 4,  J2.2 (+5V) on strip 8 (isolated), J2.3 (GND) on strip 9
       J4.1 (SIG) on strip 2,  J4.2 (+5V) on strip 8,            J4.3 (GND) on strip 9
       J3.1 (SIG) on strip 5,  J3.2 (+5V) on strip 8,            J3.3 (GND) on strip 9
       J5.1 (SIG) on strip 7,  J5.2 (+5V) on strip 8,            J5.3 (GND) on strip 9
```

> Don't take the `─`/symbol picture as pixel-perfect. The text below
> (track cuts + jumpers + components) is the authoritative build list —
> the picture is just to orient you.

### 4.3 Track cuts

Cut the copper strip *between* the two listed columns. Use a stripboard
spot-face cutter or a 3 mm drill bit twirled by hand.

| # | Strip | Between cols | Reason |
| --- | --- | --- | --- |
| C1 | 3 | 5 ↔ 6 | Isolate ESC L cathode tail from the rest of strip 3 |
| C2 | 4 | 7 ↔ 8 | Isolate ESC R cathode tail from the rest of strip 4 |
| C3 | 5 | 13 ↔ 14 | Keep S.BUS-in (J3 SIG) separate from J5 SIG region |
| C4 | 8 | 1 ↔ 2 | Isolate J1.2 (+5V pin) from rail (ESC red wire cut anyway — belt + braces) |
| C5 | 8 | 3 ↔ 4 | Same isolation between J1.2 and J2.2 |
| C6 | 8 | 5 ↔ 6 | Same isolation, J2.2 from the rest of the +5V rail |

After cutting, hold the board up to a light and confirm each cut is
clean — a sliver of copper bridging the cut is the #1 cause of
mystery problems.

### 4.4 Components (placement)

| Ref | Strip(s) | Col(s) | Notes |
| --- | --- | --- | --- |
| R1 (4.7 k) | 8 → 2 | col 4 | Vertical. Upper lead in strip 8 (+5V), lower in strip 2 (bus) |
| D1 (1N5819) | 3 → 2 | cols 5 → 4 | Horizontal. Cathode (band) in col 5 strip 3, anode in col 4 strip 2 |
| D2 (1N5819) | 4 → 2 | cols 7 → 6 | Horizontal. Cathode in col 7 strip 4, anode in col 6 strip 2 |
| R2 (10 k) | 5 → 6 | col 9 → col 10 | Horizontal. Left lead strip 5 col 9, right lead strip 6 col 10 |
| R3 (10 k) | 8 → 7 | col 13 | Vertical. Upper strip 8, lower strip 7 |
| Q1 (2N3904) | 6 / 7 / 9 | col 11 | Flat side facing right. E (strip 9) – B (strip 6) – C (strip 7) by bending leads outward |
| J1 | 3 / 8 / 9 | col 2 | SIG on strip 3, +5V (isolated) on strip 8, GND on strip 9 |
| J2 | 4 / 8 / 9 | col 4 | SIG on strip 4, +5V (isolated) on strip 8, GND on strip 9 |
| J4 | 2 / 8 / 9 | col 6 | SIG on strip 2, +5V on strip 8, GND on strip 9 |
| J3 | 5 / 8 / 9 | col 10 | SIG on strip 5, +5V on strip 8, GND on strip 9 |
| J5 | 7 / 8 / 9 | col 14 | SIG on strip 7, +5V on strip 8, GND on strip 9 |

The 2N3904 pinout (TO-92, flat side facing you, leads down):

```text
   ╔═══╗
   ║flat║
   ╚╤╤╤╝
    E B C        ← Emitter / Base / Collector
```

Mount it with the flat side facing RIGHT so the leads enter
strip 9 (E) / strip 6 (B) / strip 7 (C) by bending the centre lead
slightly forward and the outer leads to span the right strips.

### 4.5 Jumpers (insulated wire)

There are none required if you placed the components per §4.4 — the
diodes and resistors bridge between the strips themselves. If you swap
to SMD parts (BAT54 for the Schottkys), add two short wire jumpers
to bridge cathode-tail strips to the bus node.

---

## 5. Soldering Order

Build from low-profile to high-profile, easy-to-replace to hard-to-
replace. Test continuity at every checkpoint — much faster than
desoldering later.

1. **Inspect the bare stripboard.** Hold to a light. Confirm no
   pre-existing copper bridges between strips.

2. **Cuts first.** Make all 6 track cuts listed in §4.3.
   Continuity-check each one with a multimeter (beep mode) by probing
   either side of the cut — you should get *no* beep.

3. **Resistors (R1, R2, R3).** Bend leads, insert, solder, snip flush.
   Verify resistance across the leads with the meter before snipping
   (4.7 k, 10 k, 10 k).

4. **Diodes (D1, D2).** **Band toward the column with the higher
   number** (i.e., toward strips 3 / 4, which are the ESC sides).
   Double-check polarity *before* soldering — desoldering Schottkys
   from stripboard is unpleasant.

5. **Transistor (Q1).** Get the orientation right (E–B–C, flat side
   right). Leave the leads slightly long until you confirm placement,
   then solder and snip.

6. **Continuity sanity check (no headers yet).**
   - Strip 2 (bus) ↔ strip 8 (+5V): should read ~4.7 kΩ (through R1).
   - Strip 3 (ESC L tail, col 1–5) ↔ strip 2 (bus): should read open
     in one direction, ~0.3 V drop forward (D1 forward test on meter
     diode mode).
   - Strip 5 ↔ strip 6: ~10 kΩ (through R2).
   - Strip 7 ↔ strip 8: ~10 kΩ (through R3).
   - Strip 6 (NPN base) ↔ strip 9 (GND): ~0.7 V drop forward
     (NPN B–E junction in diode test).

7. **Headers (J1–J5).** Insert all five 3-pin male headers from the
   top, with the plastic spacer on the component side and the long
   pins on the copper side. **Tack one pin first**, check that the
   header is square and flush, then solder the other two.

8. **Final continuity check (with headers in).**
   - J1.3 ↔ J2.3 ↔ J3.3 ↔ J4.3 ↔ J5.3 ↔ Q1 emitter: all beep (GND rail).
   - J3.2 ↔ J4.2 ↔ J5.2: all beep (+5V rail).
   - J1.2 and J2.2 should **not** beep to the +5V rail (cuts isolated them).
   - J4.1 ↔ J1.1: read ~Vf of D1 in diode mode (anode at J4 side).
   - J4.1 ↔ J2.1: same for D2.
   - J3.1 (S.BUS in) → through R2 → Q1 base: ~10 kΩ.

---

## 6. Connector Pinouts

All five headers are 3-pin straight male, 0.1″ pitch. Wire colours
follow Futaba/JR servo convention.

```text
Pin layout (looking into the male header from above, pin 1 at top):

    Pin 1 — SIGNAL   (yellow / orange / white)
    Pin 2 — +5V      (red)
    Pin 3 — GND      (black / brown)
```

| Header | Direction | Pin 1 (SIG) | Pin 2 (+5V) | Pin 3 (GND) |
| --- | --- | --- | --- | --- |
| J1 | ESC L Yellow IN | ESC L X.BUS yellow | **N/C** (cut from rail) | ESC L brown → GND |
| J2 | ESC R Yellow IN | ESC R X.BUS yellow | **N/C** (cut from rail) | ESC R brown → GND |
| J3 | RC S.BUS IN | RX S.BUS signal | +5V → RX VCC | GND → RX ground |
| J4 | X.BUS to Arduino | Merged bus → main board | +5V from main board | GND from main board |
| J5 | S.BUS inverted to Arduino | Inverted S.BUS → main board | +5V from main board | GND from main board |

**Cable warnings:**

- ESC red wires (J1.2, J2.2 cable side) **must remain cut** — both ESCs'
  BECs would otherwise back-feed each other through the +5V rail. The
  board cuts the +5V rail at those positions as belt-and-braces, but
  please confirm the wires are cut at the ESC end.
- The RC receiver's +5V comes from the main board through the daughter
  board: J4 (or J5) brings +5V onto the daughter board's rail, which
  feeds J3.2.

---

## 7. Main-Board Wiring Changes

After dropping the daughter board into the perf board, three short
runs need to be made on the main board:

```text
                          (daughter board J4.1 = merged X.BUS bus)
                                  │
                                  │ (single wire from J4.1)
                                  │
   Arduino D0 ─────────────────── ├── (RX, direct)
                                  │
   Arduino D1 ──[1 kΩ R_TX]────── ┘  (TX, through 1 kΩ series)


                          (daughter board J5.1 = inverted S.BUS)
                                  │
   Arduino D0 ←────────── (only one of X.BUS / S.BUS can use D0 at a time)


   Arduino +5V ──── daughter board J4.2 (or J5.2)
   Arduino GND ──── daughter board J4.3 (or J5.3)
```

> **D0 conflict reminder (already noted in `CLAUDE.md`).** Both X.BUS
> and S.BUS want Serial1 RX on D0. You can only use one at a time.
> During X.BUS bring-up, unplug J5 from D0; for radio-only runs, unplug
> J4 from D0.

If you have a free hardware UART (e.g., once you migrate the project
to a board with two UARTs), route X.BUS to one UART and S.BUS to the
other and the two headers become live simultaneously without
intervention.

---

## 8. Bring-up / Test Procedure

Run through this in order — do **not** plug both ESC batteries in
until step 6.

1. **Visual.** Diode bands point toward J1 / J2 (away from J4).
   Transistor flat side toward J5. No solder bridges.
2. **Continuity to the meter.** Re-run §5 step 8.
3. **Power-only test.** Plug J4 to main board (+5V, GND, signal floating
   on D0 / D1 OK). Confirm +5V on strip 8 with meter, GND on strip 9.
4. **S.BUS inverter test (no ESCs).** Plug J3 to the RC receiver,
   power the system. With a scope or logic analyser on J5.1:
   - Receiver on, transmitter idle: J5.1 should be HIGH (idle).
   - Move a stick: J5.1 should show inverted UART activity.
   - Match against J3.1 directly: should be the bitwise inverse.
5. **X.BUS pull-up test (no ESCs).** Meter on J4.1: should read ≈5 V
   (held by R1). Drop probe to GND momentarily and release — voltage
   should snap back to 5 V (pull-up working).
6. **One ESC at a time on X.BUS.** Plug ESC L into J1 (battery,
   armed, X.BUS address 0). Run `sketches/xbus_master/xbus_master.ino`.
   Confirm telemetry. Repeat for ESC R on J2 (address 1).
7. **Both ESCs.** Plug both, run xbus_master with both addresses.
   Confirm telemetry from each in turn.
8. **End-to-end.** Bring up `sketches/rc_test/rc_test.ino` with both
   the inverted S.BUS and X.BUS lines wired (one of them at a time
   until the D0 conflict is resolved — see §7).

If any of these fail, the most likely cause is a track cut not fully
severed or a diode in backwards. Re-check both before assuming
component failure.

---

## 9. Hardware-Version Note

The connector pinout and component values above assume the GL10 / GL540L
combo behaves identically to the E10 / E3665 setup on the X.BUS bus
(115200 8N1, idle HIGH, push-pull or open-drain TX). The user has
confirmed telemetry on the breadboard with the GL10s, so this should
hold. If a future XC ESC model needs a different bus voltage or
edge rate, the only component changes likely to be required are:

- R1 (pull-up): 3 k–10 k acceptable per the spec
- Schottkys D1 / D2: any low-Vf small-signal Schottky (BAT54, BAT85,
  SS14) works in place of the 1N5819

---

## 10. Protection (deferred)

This build deliberately omits TVS / ferrite / 100 nF protection on the
X.BUS line. If a future field test shows ESD or RF noise problems on the
bus, the cleanest place to add protection is **on the main board** at
the entry point of the J4 wire, not on the daughter board, because the
TVS diode wants the shortest possible loop to the Arduino GND.

Suggested retrofit (if needed):

```text
   J4 SIG wire ──[ferrite bead]──┬── 100 nF to GND ──┬── Arduino D0 / D1
                                 │                   │
                              SMAJ5.0CA              │
                                 │                   │
                                GND                 GND
```

---

## File map updates

- This file: `docs/DAUGHTER-BOARD.md` (new)
- Builds on: `docs/XBUS-PROTOCOL.md` §3.1, §7
- Supersedes (for the merge circuit only): `docs/WIRING-GUIDE.md` §1
  (which has incorrect diode polarity)
