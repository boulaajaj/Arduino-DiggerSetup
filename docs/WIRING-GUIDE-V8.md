# Wiring Guide V8.0 — UNO R4 WiFi + GL10 FOC

**Date:** 2026-05-24
**Board:** Arduino UNO R4 WiFi (Renesas RA4M1 + ESP32-S3)
**Shield:** Sensor Shield V5.0
**ESCs:** 2× GL10 80A Sensored Brushless FOC
**Motors:** 2× GL540L Sensored Brushless (1× CW, 1× CCW)

---

## 1. System Overview

```text
┌─────────────────────────────────────────────────────┐
│                    UNDER THE SEAT                    │
│                                                     │
│  ┌─────────────┐    ┌──────────┐                    │
│  │  Joystick   │───→│  Shield  │                    │
│  │  (A0, A1)   │    │  V5.0    │                    │
│  └─────────────┘    │  on UNO  │                    │
│                     │  R4 WiFi │                    │
│                     └────┬─────┘                    │
│                          │                          │
│                    9-PIN CABLE                      │
│                     (modular)                       │
│                          │                          │
├──────────────────────────┼──────────────────────────┤
│                    ON THE FRAME                      │
│                          │                          │
│                  ┌───────┴────────┐                  │
│                  │ Interface Board │                 │
│                  │  (soldered)     │                 │
│                  └──┬───┬───┬──┬──┘                  │
│                     │   │   │  │                     │
│              ┌──────┘   │   │  └──────┐              │
│              │          │   │         │              │
│         ┌────┴───┐  ┌──┴───┴──┐  ┌───┴────┐        │
│         │ESC Left│  │Receiver │  │ESC Right│        │
│         │ GL10   │  │  R7FG   │  │  GL10   │        │
│         └────┬───┘  └─────────┘  └───┬────┘        │
│              │                       │              │
│         ┌────┴───┐              ┌────┴───┐          │
│         │Motor L │              │Motor R │          │
│         │GL540L  │              │GL540L  │          │
│         └────────┘              └────────┘          │
└─────────────────────────────────────────────────────┘
```

---

## 2. Pin Assignments — UNO R4 WiFi

| Pin | Signal | Direction | UART | Connection |
|-----|--------|-----------|------|------------|
| **D0** | X.BUS RX | Bus → Arduino | Serial1 (SCI2) | Interface board J4 |
| **D1** | X.BUS TX | Arduino → Bus | Serial1 (SCI2) | Interface board J3 |
| **D9** | PWM Left | Arduino → ESC | — | ESC Left throttle |
| **D10** | PWM Right | Arduino → ESC | — | ESC Right throttle |
| **D12** | S.BUS RX | Board → Arduino | sbusUart (SCI0) | Interface board J6 |
| **A0** | Joystick Y | Joystick → Arduino | — | Throttle axis (14-bit ADC) |
| **A1** | Joystick X | Joystick → Arduino | — | Steering axis (14-bit ADC) |
| **VIN** | Power in | ESC → Arduino | — | BEC 7.4V |
| **5V** | Power out | Arduino → Board | — | Interface board +5V rail |
| **GND** | Ground | Common | — | All devices |

### Available Pins

| Pin | Notes |
|-----|-------|
| D2, D3, D4, D5, D6, D7, D8 | Free digital I/O |
| D11 | SCI0 TX (reserved — paired with D12 S.BUS UART) |
| D13 | Free (LED_BUILTIN) |
| A2, A3 | Free analog inputs |
| A4, A5 | Free (I2C SDA/SCL — no SCI on UNO R4 WiFi) |

### UART Architecture

| Port | SCI Channel | Pins | Baud | Purpose |
|------|-------------|------|------|---------|
| Serial | — | USB-C | 115200 | Debug / uploads |
| Serial1 | SCI2 | D0 (RX), D1 (TX) | 115200 | X.BUS telemetry (both ESCs) |
| sbusUart | SCI0 | D12 (RX), D11 (TX) | 100000 8E2 | S.BUS RC input |
| Serial2 | SCI? | Internal | — | **Reserved for WiFi ESP32-S3** |

**Key difference from Nano R4:** SCI0 is on D11/D12 (not A4/A5). Same
hardware peripheral, different pins. A4/A5 are I2C-only on the UNO R4 WiFi.

---

## 3. Power

```text
ESC Left BEC (7.4V) ──→ 9-pin cable pin 6 ──→ Arduino VIN
                                                    │
                                          Buck converter (ISL854102FRZ)
                                                    │
                                                  5V rail
                                                    │
                              ┌─────────────────────┼─────────────────┐
                              │                     │                 │
                     Shield servo headers    Joystick (A0, A1)    9-pin cable pin 8
                       (SEL jumper IN)                                │
                                                              Interface board +5V
                                                                      │
                                                    ┌─────────────────┼──────────┐
                                                    │                 │          │
                                              RC Receiver     X.BUS pull-up   S.BUS
                                              (via J5)        R1 (4.7kΩ)     inverter
                                                                              R4 (10kΩ)
```

| Source | Voltage | Destination | Current |
|--------|---------|-------------|---------|
| ESC Left BEC | 7.4V | Arduino VIN | — |
| Arduino regulator | 5V | Shield rail, joystick, interface board | ~300mA total |
| ESC Right BEC | — | **Disconnected** (red wire cut) | — |

**BEC setting:** Set to **7.4V** in TXC-Link app (ESC Left only).
**USB coexist:** Safe. Schottky diodes prevent backfeed. Plug USB anytime.
**Shield SEL jumper:** Leave **IN** (default). Arduino 5V powers servo headers.

---

## 4. Nine-Pin Cable

Waterproof 9-pin connector between the interface board (on the frame)
and the Arduino (under the seat). One cable, one disconnect point.

### Cable Pin Map

| Cable Pin | Shield Pin | Wire Color | Direction | Purpose |
|-----------|-----------|------------|-----------|---------|
| 1 | **D9** | Blue (thick) | Arduino → ESC | PWM left track |
| 2 | **D10** | Yellow | Arduino → ESC | PWM right track |
| 3 | **D1** | Green (thin) | Arduino → bus | X.BUS telemetry TX |
| 4 | **D0** | Blue (thin) | Bus → Arduino | X.BUS telemetry RX |
| 5 | **D12** | Red (thin) | Board → Arduino | S.BUS inverted (SCI0 RX) |
| 6 | **VIN** | Yellow (thick) | ESC → Arduino | Power in 7.4V |
| 7 | **GND** | Green (thick) | Common | Ground |
| 8 | **5V** | White (thin) | Arduino → board | 5V back to interface board |
| 9 | — | — | — | Spare (not connected) |

### Interface Board Side — 6 Female Servo Connectors

All connectors are standard **3-pin servo** (Signal, VCC, GND) for
structural support and modularity. VCC pin is soldered to the board
pad for mechanical strength but not electrically connected on signal-only
connectors (J3, J4, J6, F1, F2, F5).

| Connector | Pin 1 (Signal) | Pin 2 (VCC) | Pin 3 (GND) | Receives Plug From |
|-----------|---------------|-------------|-------------|-------------------|
| F1 — PWM L | Signal | N/C (structural) | GND | ESC Left throttle servo |
| F2 — PWM R | Signal | N/C (structural) | GND | ESC Right throttle servo |
| F3 — XBUS TX | Signal | N/C (structural) | GND | Interface board J3 |
| F4 — XBUS RX | Signal | N/C (structural) | GND | Interface board J4 |
| F5 — SBUS | Signal | N/C (structural) | GND | Interface board J6 |
| F6 — Power | VIN 7.4V | GND | 5V return | ESC BEC + Arduino 5V |

### Arduino Shield Side — 6 Servo Connectors

All connectors are standard **3-pin servo** plugging into shield headers.
Shield headers provide VCC from the 5V rail on the middle pin — this is
harmless on signal-only connectors (not connected on the cable side).

| Connector | Shield Header | Pin 1 (Signal) | Pin 2 (VCC) | Pin 3 (GND) |
|-----------|--------------|---------------|-------------|-------------|
| S1 — PWM L | D9 | Signal | (from shield, unused) | GND |
| S2 — PWM R | D10 | Signal | (from shield, unused) | GND |
| S3 — XBUS TX | D1 | Signal | (from shield, unused) | GND |
| S4 — XBUS RX | D0 | Signal | (from shield, unused) | GND |
| S5 — SBUS | D12 | Signal | (from shield, unused) | GND |
| S6 — Power | VIN/5V/GND | VIN 7.4V | GND | 5V return |

### Direct to Shield (not through cable)

| Connector | Shield Pin | Wires |
|-----------|-----------|-------|
| Joystick Y | A0 header | Signal + 5V + GND |
| Joystick X | A1 header | Signal + 5V + GND |

---

## 5. Interface Board

Full build plan in `docs/INTERFACE-BOARD.md`. Summary:

| Component | Purpose |
|-----------|---------|
| R1 (4.7kΩ) | X.BUS bus pull-up to 5V |
| R2 (1kΩ) | Arduino TX series resistor (contention protection) |
| R3 (10kΩ) | S.BUS NPN base resistor |
| R4 (10kΩ) | S.BUS NPN collector pull-up |
| Q1 (2N3904) | S.BUS signal inverter |
| J1–J6 | 3-pin servo headers (plug-and-play) |

**Receiver plugs into J5** on the interface board. Gets 5V + GND from the
board's +5V rail (fed by cable pin 8). S.BUS signal goes through the NPN
inverter (Q1) and exits via J6 into the cable.

---

## 6. ESC Configuration (TXC-Link App)

| Setting | ESC Left | ESC Right |
|---------|----------|-----------|
| BEC Voltage | **7.4V** | Any (red wire cut) |
| X.BUS Address | **0** | **1** |
| Motor Rotation | Per track | Per track |
| Max Reverse Force | 100% | 100% |
| Simulation Inertia | 1 | 1 |
| Drag Force | 30% | 30% |
| Max Brake Force | 60% | 60% |
| Cutoff Voltage | 3.2V/cell | 3.2V/cell |

Full parameter reference: `docs/GL10-QUICK-REFERENCE.md`

---

## 7. Wire Labels

| Label | Cable Pin | Wire Color | From → To |
|-------|-----------|------------|-----------|
| PWM-L | 1 | Blue (thick) | D9 → ESC Left throttle |
| PWM-R | 2 | Yellow | D10 → ESC Right throttle |
| XBUS-TX | 3 | Green (thin) | D1 → Interface board bus |
| XBUS-RX | 4 | Blue (thin) | Interface board bus → D0 |
| SBUS | 5 | Red (thin) | Interface board Q1 → D12 |
| VIN-7V4 | 6 | Yellow (thick) | ESC BEC → Arduino VIN |
| GND | 7 | Green (thick) | Common ground |
| 5V-OUT | 8 | White (thin) | Arduino 5V → Interface board |
| SPARE | 9 | — | — |
| JOY-Y | — | — | Joystick → A0 (direct to shield) |
| JOY-X | — | — | Joystick → A1 (direct to shield) |

---

## 8. Safety Checklist

- [ ] ESC Right BEC red wire is cut/insulated
- [ ] ESC Left BEC set to 7.4V in TXC-Link
- [ ] BEC power goes to VIN, not 5V pin
- [ ] Shield SEL jumper is IN (default)
- [ ] X.BUS addresses: ESC Left = 0, ESC Right = 1
- [ ] Both ESCs calibrated (Reverse → Forward → Neutral)
- [ ] All grounds connected (common ground bus)
- [ ] Interface board continuity checks pass
- [ ] USB cable disconnected during field operation (or fine to leave)
