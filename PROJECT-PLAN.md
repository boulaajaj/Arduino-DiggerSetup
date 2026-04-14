# Excavator Track Controller — Project Plan

## What This Does

An Arduino Nano R4 sits between an RC receiver and two ESCs, implementing
a **tank-style mixer** with a secondary joystick input for the rider.
The RC transmitter (Jason) can always override. A 3-position switch
selects who has authority.

The controller provides:

- **Exponential response curve** — fine low-speed control, full range at high stick
- **Inertia simulation** — asymmetric accel/decel for heavy machine feel
- **Spin turn limiter** — graduated power reduction during counter-rotation
- **Reverse speed limit** — caps backward speed to 50% of forward
- **Soft power limits** — tanh saturation, no hard clamps
- **Per-channel failsafe** — each RC channel independently monitored

---

## Hardware

| # | Component | Model | Key Specs |
| --- | ----------- | ------- | ----------- |
| 1 | Controller | Arduino Nano R4 | Renesas RA4M1, 48MHz, 14-bit ADC, 5V tolerant |
| 2 | Battery (x2) | OVONIC 3S LiPo 15000mAh | 11.1V, EC5, 130C |
| 3 | ESC (x2) | XC E10 Sensored 140A | Servo PWM input, X.BUS telemetry |
| 4 | Motor (x2) | XC E3665 2500KV Sensored | 4-pole, hall sensor, 8mm shaft |
| 5 | RC System | Radiolink RC6GS V3 + R7FG | 6CH, gyro receiver |
| 6 | Joystick | Genie 101174GT dual-axis | 5V, 0-5V analog, hall-effect |
| 7 | Current Sensor (x2) | CS7581 Hall-effect | Per-motor current |

---

## Pin Map

```text
D2  ← RC CH1 (left track, pre-mixed)   [attachInterrupt CHANGE]
D3  ← RC CH4 (control mode, 3-pos)     [attachInterrupt CHANGE]
D4  ← RC CH2 (right track, pre-mixed)  [non-blocking poll — no IRQ on D4]
D5     (reserved for left hall sensor)
D6     (reserved for right hall sensor)
D7  ← RC CH5 (override switch, 3-pos)  [non-blocking poll — no IRQ on D7]
D9  → Left ESC                          [Servo PWM]
D10 → Right ESC                         [Servo PWM]
A0  ← Joystick Y (throttle)             [14-bit ADC, 0-5V direct]
A1  ← Joystick X (steering)             [14-bit ADC, 0-5V direct]
A2  ← CS7581 current sensor (left)      [14-bit ADC]
A3  ← CS7581 current sensor (right)     [14-bit ADC]
5V  → RC receiver + joystick VCC
GND → All components (common ground)
```

D0/D1 reserved for X.BUS (Serial1) — conflicts with USB Serial during debug.

---

## Signal Pipeline

```text
RC (ISR + poll) ──┐
                  ├─► Mixer ─► Spin Limiter ─► Reverse Limiter
Joystick (ADC) ──┘        ─► Soft Limit ─► Inertia ─► ESC Output
```

### Override Switch (CH5)

| Position | Mode | Who Controls |
| ---------- | ------ | ------------- |
| LOW | RC only | Jason has full authority, joystick disabled |
| MID | RC priority | Both active, RC overrides when non-neutral |
| HIGH | 50/50 blend | RC + joystick averaged, both always contribute |

### Dynamics Pipeline

1. **Spin limiter** — graduated power cap during counter-rotation (50% at full pivot)
2. **Reverse limiter** — backward speed capped at 50% of forward max
3. **Soft limit** — tanh saturation caps max servo deviation
4. **Inertia** — asymmetric exponential: accel ~1.5s, decel/coast ~3.0s

---

## Tuning Constants

| Constant | Value | Description |
| ---------- | ------- | ------------- |
| `EXPO_LINEAR` | 0.3 | Linear blend of expo curve |
| `EXPO_SQUARE` | 0.7 | Quadratic blend (sum to 1.0) |
| `SOFT_RANGE` | 400 us | Max servo offset from center |
| `TAU_ACCEL` | 0.3 s | Acceleration time constant |
| `TAU_DECEL` | 0.5 s | Deceleration/coast time constant |
| `SPIN_LIMIT` | 0.25 | Power cap at full pivot turn (25%) |
| `REVERSE_LIMIT` | 0.25 | Max reverse as fraction of forward (25%) |
| `RC_DEADBAND` | 50 us | RC pulse dead zone |
| `JOY_DEADBAND` | 480 | Joystick ADC dead zone (~5.9%) |
| `FAILSAFE_US` | 500 ms | Per-channel signal timeout |

---

## File Map

```text
sketches/rc_test/
  rc_test.ino       — Main controller sketch (V3.4)
  types.h           — Shared structs (RCChannel, PulseReader, etc.)

sketches/xbus_probe/
  xbus_probe.ino    — X.BUS telemetry probe (standalone)

sketches/hw_diagnostic/
  hw_diagnostic.ino — Hardware diagnostic (ADC, RC, signal check)

sketches/pin_test/
  pin_test.ino      — Interrupt pin capability test

docs/
  CONTROL-RESEARCH.md — Tank mix, RC input, loop patterns research

live_plot.py        — Real-time 7-panel matplotlib monitor
monitor.py          — Simple serial monitor
PROJECT-PLAN.md     — This file
OPERATOR-GUIDE.md   — User guide for Jason (RC) and Malaki (joystick)
```

---

## Build & Upload

```bash
# Compile
arduino-cli compile --fqbn arduino:renesas_uno:nanor4 sketches/rc_test/rc_test.ino

# Upload (COM8)
arduino-cli upload --fqbn arduino:renesas_uno:nanor4 -p COM8 sketches/rc_test/rc_test.ino

# Monitor serial
python monitor.py

# Live plot
python live_plot.py
```

---

## Status

### Working (V3.1 — deployed on Nano R4)

- [x] Non-blocking RC input on all 4 channels (ISR + PulseReader)
- [x] Per-channel failsafe (independent 0.5s timeout)
- [x] Expo response curve (20% linear + 80% quadratic)
- [x] Inertia simulation (asymmetric: 1.5s accel, 3.0s decel)
- [x] Spin turn limiter (50% at full pivot, graduated)
- [x] Reverse speed limiter (50% of forward max)
- [x] Soft power limits (tanh saturation)
- [x] Override switch (3-pos: RC only / RC priority / Joystick only)
- [x] Tank mix for joystick (RC pre-mixed by transmitter)
- [x] ADC crosstalk fix (double-read with settling delay)
- [x] 10 Hz CSV telemetry with signal health indicators
- [x] Modular code architecture with searchable [MODULE] sections

### Pending — Field Tuning

- [ ] Expo/inertia field tuning (TAU_ACCEL, TAU_DECEL, EXPO blend)
- [ ] Spin turn limit calibration (SPIN_LIMIT value)
- [ ] Reverse limit calibration (REVERSE_LIMIT value)

### Pending — Telemetry & PID

- [ ] **Waiting on XC Technology reply** (email sent 2026-03-31):
  - X.BUS protocol spec → implement polling → test if transceivers survived BEC damage
  - E3665 hall sensor pinout → wire RPM tap if X.BUS fails
- [ ] CS7581 current sensor wiring and calibration (A2, A3)
- [ ] RPM closed-loop PID (target RPM from stick, actual from sensor)
- [ ] Dual-loop PID: inner current (feedforward) + outer RPM (feedback)
- [ ] Add RPM + current panels to live_plot.py

### Pending — Integration

- [ ] Bench test with ESCs at reduced power
- [ ] Field test on excavator
- [ ] Battery voltage monitoring
- [ ] Emergency stop integration

---

## X.BUS Telemetry Status

**XC E10 "X.BUS" is NOT Spektrum X-Bus.** It is XC Technology's
proprietary protocol on a single data wire (not I2C).

### Probing Results (2026-03-31): Zero Data

- Passive listening: 7 baud rates, zero bytes
- Active probing: All known protocols, zero responses
- Raw D0 sampling: zero toggles (line electrically dead at 3.3V)
- While driving under load: still zero

### Possible BEC Damage

Both ESC BEC red wires were connected together at 8.5V for 15-30min.
20-30% chance X.BUS hardware is damaged. **Rule: NEVER connect both
ESC BEC red wires. Only one BEC powers the bus.**

### Strategy

1. Wait for XC Technology reply with protocol spec
2. If X.BUS dead → motor hall sensor tap for RPM (needs pinout from XC)
3. RPM feedback is for closed-loop PID, not just display

---

## Architecture Decisions Log

| Date | Decision | Rationale |
| ------ | ---------- | ----------- |
| 2026-03-20 | RPM via motor hall tap (not sprocket, not BT) | High resolution, zero latency, signal exists |
| 2026-03-21 | Dual-loop PID (current + RPM) | Inner predicts, outer confirms — standard industrial |
| 2026-03-29 | XC X.BUS is UART not I2C | Single data wire, can't be I2C |
| 2026-03-30 | Revert from UNO Q to Nano R4 | UNO Q had Router Bridge complexity |
| 2026-04-01 | Non-blocking poll replaces pulseIn | pulseIn caused stale timestamp bug |
| 2026-04-01 | Per-channel failsafe | Combined failsafe masked dead channels |
| 2026-04-01 | Reverse limit 50% | Safety: limited visibility, operator reaction time |
| 2026-04-01 | Spin limit 50% at full pivot | Safety: full-speed pivot throws operator |
