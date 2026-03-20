# Hall Sensor RPM Reading — Research & Wiring Plan

## Summary

This document covers research into tapping the 6-pin Julet hall sensor connector
from the ESC/motor connection to read RPM on the Arduino, using a Y-splitter
approach. It also covers recommended prototyping hardware.

---

## 1. The 6-Pin Julet Connector — What's Inside

### Standard 6-Wire Hall Sensor Pinout (Typical)

| Wire Color | Function | Notes |
|------------|----------|-------|
| **Red** | +5V Power | Powers the hall sensors inside the motor |
| **Black** | GND | Common ground |
| **Yellow** | Hall Sensor A | Digital signal, 0V or 5V |
| **Green** | Hall Sensor B | Digital signal, 0V or 5V |
| **Blue** | Hall Sensor C | Digital signal, 0V or 5V |
| **White** | Speed/Temp sensor | 6th wire — speed sensor or temperature sensor depending on motor |

### How Hall Sensors Work

- The ESC provides regulated **5V DC** and **GND** to each sensor
- The ESC also provides a **pull-up voltage (5V)** on each signal wire
- When a motor magnet passes a sensor, it pulls the signal wire to **GND (0V)**
- When the magnet moves away, the signal returns to **5V** (via pull-up)
- Result: clean **5V square wave** on each signal wire
- Current draw is very low — approximately **6mA per sensor**

### Critical Warning — Pinouts Are NOT Universal

> Wire color can be whatever cable they bought that day, but the pin positions
> don't move.

**You MUST verify your specific connector with a multimeter before wiring.**
The colors above are the most common convention, but manufacturers change them.
The pin _position_ in the connector housing is more reliable than wire color.

### How to Verify Your Connector

1. With the motor disconnected, measure continuity from each wire to each pin position
2. On the ESC side, with motor disconnected, measure voltage on each pin:
   - Two pins should show ~5V (power) and 0V (ground)
   - Three pins should show ~5V (pull-up on hall signal lines)
   - One pin may show 5V or nothing (speed/temp sensor)
3. Write down your findings before connecting anything

---

## 2. Will the Y-Splitter Approach Work?

### Answer: YES — With Conditions

The Y-splitter approach (one female from ESC, splitting to two males — one to
motor, one to Arduino) **will work** for reading the hall sensor signals. Here's why:

### Why It Works

1. **Hall signals are 5V logic-level** — Arduino Uno runs at 5V logic, so they
   are directly compatible. No level shifting needed.
2. **Arduino digital inputs are high-impedance** — When configured as `INPUT`
   (not `INPUT_PULLUP`), an Arduino pin draws essentially zero current. It just
   "listens" passively without loading the circuit.
3. **The ESC provides power** — The hall sensors get their 5V and GND from the
   ESC. The Arduino does NOT need to power the sensors. It only reads the signal
   lines passively.
4. **Low current signals** — Hall sensor signals are ~6mA. Splitting this to an
   additional high-impedance input adds negligible load.

### What You Need to Connect (Arduino Side of Y-Splitter)

From the Y-splitter's second male connector, you need **4 wires** going to the Arduino:

| Y-Splitter Wire | Connect To | Purpose |
|-----------------|------------|---------|
| **GND (Black)** | Arduino GND | **ESSENTIAL** — shared ground reference |
| **Hall A (Yellow)** | Arduino D2 or D3 | Interrupt-capable pin for RPM counting |
| **Hall B (Green)** | Arduino D3 (optional) | Second hall for higher resolution |
| **Hall C (Blue)** | Another digital pin (optional) | Third hall for even higher resolution |
| **+5V (Red)** | **DO NOT CONNECT** | ESC already powers the sensors — do not bridge power rails |
| **White (Speed)** | Optional digital pin | If present, could be simplest RPM source |

### Important Notes

- **MUST share GND** between the ESC/motor system and the Arduino
- **Do NOT connect the 5V line** from the Y-splitter to the Arduino's 5V pin
  (this could create a ground loop or back-feed voltage between the two systems)
- For basic RPM, you only need **1 hall signal + GND** (2 wires)
- For direction detection, you need **2 hall signals + GND** (3 wires)
- The **white wire** (6th pin) may be a dedicated speed sensor — if so, it could
  give a cleaner single-pulse-per-revolution signal that's ideal for RPM

### Does the Hall Sensor Need Separate Power?

**No.** The ESC already provides 5V power to the hall sensors through the
existing connection. The Y-splitter passes this power through to the motor side.
The Arduino side only needs to passively read the signal lines. Do NOT connect
the 5V line from the Y-splitter to the Arduino.

---

## 3. The Cables You Ordered

### Cable 1: Nitomile 6-Pin Julet Y-Splitter (1 Female to 2 Male)

- **What it does:** Takes the single 6-pin connector from the ESC and splits it
  into two 6-pin male connectors
- **One male** goes to the motor (normal operation continues)
- **Other male** goes to a breakout cable for Arduino connection
- **Rating:** 7.9", waterproof
- **Price:** $9.99

### Cable 2: 6-Pin Julet Female Breakout Cable (Black, bare wires)

- **What it does:** Plugs into the second male of the Y-splitter and exposes
  individual colored wires
- **The bare wire ends** can be stripped and connected to the Arduino's screw
  terminal shield or sensor shield
- **Price:** $8.59

### Signal Flow

```
Motor Hall Sensors
       |
  [6-pin Julet Male]
       |
  [Y-Splitter Female]
      / \
     /   \
    /     \
[Male 1] [Male 2]
   |        |
[Motor]  [Breakout Cable Female]
   |        |
  ESC    Bare Wires --> Arduino
```

### Verdict: Good Cable Choices

These cables will work for this application. The Y-splitter maintains the
ESC-to-motor connection while providing a tap point. The breakout cable gives
you bare wires to connect to the Arduino.

---

## 4. Arduino Code Approach for RPM

### Simple Single-Hall RPM Reading

```cpp
// RPM reading from hall sensor via Y-splitter
// Connect one hall signal wire to D2 (interrupt pin)
// Connect GND from Y-splitter to Arduino GND

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

const int HALL_PIN = 2;  // Must be interrupt-capable pin
const int POLES = 1;     // Adjust based on motor pole count

void hallISR() {
  unsigned long now = micros();
  pulseInterval = now - lastPulseTime;
  lastPulseTime = now;
  newPulse = true;
}

void setup() {
  pinMode(HALL_PIN, INPUT);  // NOT INPUT_PULLUP — ESC provides pull-up
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
}

float getRPM() {
  if (pulseInterval == 0) return 0;
  // Each hall sensor triggers once per electrical revolution
  // Mechanical RPM = electrical RPM / (pole pairs)
  // You'll need to calibrate POLES for your specific motor
  float rpm = 60000000.0 / pulseInterval / POLES;
  return rpm;
}
```

### Key Calibration Note

- A BLDC motor with **N pole pairs** will give **N pulses per mechanical revolution** per hall sensor
- With 3 hall sensors, you get **3x the pulses** (better resolution at low speed)
- You'll need to determine your motor's pole count to convert electrical RPM to mechanical RPM
- At ~300-400 RPM, measuring **time between pulses** (as shown above) gives much
  better resolution than counting pulses over a time window

---

## 5. Recommended Prototyping Board: Sensor Shield V5.0

### Why This Is the Best Option for Your Project

The **Arduino Sensor Shield V5.0** is the ideal prototyping board for this project:

- **Every Arduino pin broken out to 3-pin headers** (GND, VCC, Signal) — these
  are the same format as standard servo connectors
- **Plug servos directly in** — no breadboard needed
- **Plug sensors directly in** — same 3-pin headers work for hall sensors
- **External power screw terminal** — for powering servos separately from Arduino
- **SEL jumper** — switches between Arduino 5V and external power for digital pins
- **Stacking headers** — can still stack other shields on top
- **~$3-5 on Amazon**

### How It Solves Your Connector Problem

| Need | Solution on Sensor Shield V5 |
|------|------------------------------|
| Servo connectors (ESC output) | Plug directly into D9, D10 3-pin headers |
| Hall sensor input | Strip breakout cable wires, crimp 3-pin DuPont headers, plug into D2, D3 |
| RC receiver input | Plug into D2, D4, D7 3-pin headers |
| Joystick input | Plug into A0, A1 3-pin headers |
| External servo power | Use screw terminal with SEL jumper removed |

### Alternative Options

| Board | Pros | Cons | Price |
|-------|------|------|-------|
| **Sensor Shield V5.0** | 3-pin servo headers on every pin, external power terminal | No screw terminals for bare wires | ~$3-5 |
| **Screw Terminal Shield** (DIYables/AIHJCNELE) | Screw terminals for bare wires, prototyping area | No 3-pin servo headers | ~$8-12 |
| **SparkFun ProtoScrew Shield** | Both screw terminals and proto area | More expensive, less servo-friendly | ~$15 |

### Recommendation

**Get the Sensor Shield V5.0.** It's the cheapest, most practical option for
your project since you're already using servo-style PWM signals for the ESCs
and will want to plug in hall sensor wires easily. Buy a pack of DuPont 3-pin
female crimp connectors (~$5 for a kit) to terminate the bare wires from
your breakout cable.

---

## 6. Wiring Checklist (Both Motors)

For each motor (left and right), you'll need:

- [ ] 1x Julet 6-pin Y-splitter
- [ ] 1x Julet 6-pin breakout cable (bare wire end)
- [ ] Verify pinout with multimeter before connecting
- [ ] Connect GND wire to Arduino GND (via sensor shield)
- [ ] Connect 1 hall signal wire to interrupt pin (D2 for left, D3 for right)
- [ ] Do NOT connect the 5V wire to Arduino
- [ ] Insulate/cap unused wires with heat shrink or electrical tape
- [ ] Test with serial monitor before integrating into main code

### Pin Assignment (Updated)

| Pin | Current Use | New Additional Use |
|-----|------------|-------------------|
| D2 | RC Left Motor Input (PCINT) | — (already in use, need to reassign) |
| D3 | Available (interrupt-capable) | Left Hall Sensor RPM |
| D2 | RC Left Motor | May need to move to free up interrupt |

> **Note:** D2 and D3 are the only hardware interrupt pins on the Uno. Currently
> D2 is used for RC input via Pin Change Interrupt. You may need to use Pin
> Change Interrupts (PCINT) for the hall sensors instead, since the RC inputs
> already use PCINT. Alternatively, rearrange pin assignments.

---

## 7. Risk Assessment

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Wrong pinout fries Arduino | Medium | **Verify with multimeter first** |
| Hall signal too noisy | Low | Add 100nF capacitor on signal line |
| Ground loop between systems | Low | Single ground point connection |
| Y-splitter signal degradation | Very Low | Logic-level signals, short cable runs |
| Interrupt conflict with RC pins | Medium | Use PCINT for hall sensors (same as current RC approach) |
| Motor RPM calculation wrong | Medium | Need to determine motor pole count for calibration |

---

## 8. XC E10 ESC + E3665 Motor — Bluetooth Settings & Interference Analysis

### Your Specific Hardware

- **ESC:** XC E10 — 140A sensored brushless ESC, 2-4S LiPo, built-in Bluetooth
- **Motor:** XC E3665 — sensored brushless, 4-pole, 2500KV, IP67, with hall sensors
- **App:** XC-Link (built-in Bluetooth, no external programmer needed)

### Will There Be Interference?

**The chances of success are HIGH.** Here's the detailed analysis:

#### Why Interference Risk Is LOW

1. **Hall signals are digital (0V / 5V square waves)** — not analog. Digital
   signals are inherently noise-resistant because you only care about HIGH vs LOW,
   not an exact voltage level.
2. **The ESC already reads these same signals successfully** — if the signals were
   too noisy, the motor wouldn't run smoothly in sensored mode. The fact that it
   works means the signals are clean enough.
3. **The Arduino only passively reads** — it doesn't inject any current or voltage
   into the signal lines, so it cannot create interference for the ESC.
4. **Short cable runs** — the Y-splitter is only 7.9" (20cm). EMI problems
   typically manifest over longer wire runs.

#### Potential Interference Sources (and Why They're Manageable)

| Source | Risk | Explanation |
|--------|------|-------------|
| ESC PWM switching noise | Low | The ESC switches motor phases at high frequency (8-20kHz). This could theoretically couple into the hall sensor wires, but since the ESC already deals with this successfully, the Arduino will too. |
| Motor back-EMF | Very Low | The hall sensor wires are physically separate from the motor phase wires inside the connector. The Y-splitter maintains this separation. |
| Ground loop | Low-Medium | If the Arduino is powered from a different source than the ESC (e.g., USB vs battery), there could be a ground potential difference. **Solution:** Power Arduino from the ESC's BEC output (6V/7.4V to VIN pin). |
| Bluetooth radio | Negligible | 2.4GHz Bluetooth has zero effect on DC-level hall sensor signals. |

#### If You DO Get Noise (Mitigation)

Based on Arduino Forum community experience with ESC + hall sensor setups:

1. **Add a 100nF ceramic capacitor** between each hall signal wire and GND, as
   close to the Arduino pin as possible. This filters high-frequency switching noise.
2. **Use shielded cable** for the Arduino-side breakout if unshielded wire picks
   up noise. Connect shield to GND at the Arduino end only.
3. **Add a ferrite clamp** on the breakout cable near the Arduino end.
4. **Software debounce** — ignore pulses shorter than a minimum interval (e.g.,
   discard any pulse interval < 100us, which would imply >600,000 RPM — impossible).

### XC E10 Bluetooth Settings — What to Adjust

Open the **XC-Link** app and connect to your E10 ESC. Here are the settings that
matter for clean hall sensor signal passthrough and smooth motor control:

#### Settings to CHANGE from Default

| Setting | Recommended Value | Why |
|---------|-------------------|-----|
| **Turbo Timing** | **0°** (disabled) | Turbo timing advances commutation beyond sensor position, adding electrical noise. Set to 0° to keep commutation perfectly aligned with hall sensor readings. You can increase later once RPM reading is proven working. |
| **Drag Brake** | **0%** (disabled) | Drag brake actively brakes the motor when throttle is neutral, which causes regenerative current flow and electrical noise. Disable it — your Arduino code already handles deceleration smoothing. |
| **Acceleration Level** | **Level 1** (slowest) | The E10 has 12 acceleration levels. Start with the slowest to get the smoothest ramp-up, which generates the least electrical transient noise. Your Arduino code already does its own acceleration smoothing (800ms tau). |
| **Running Mode** | **Forward/Brake/Reverse** | You need reverse for the digger tracks. |
| **BEC Voltage** | **6.0V** | Use 6.0V for standard servos AND to power the Arduino via VIN pin. This keeps the Arduino and ESC on the same ground, eliminating ground loop risk. |

#### Settings to LEAVE at Default

| Setting | Default | Why Leave It |
|---------|---------|-------------|
| **Sensored Mode** | Enabled (auto-detected) | The E10 auto-detects if a sensored motor is connected. Leave this on — it gives the cleanest hall sensor signals. |
| **LiPo Cell Count** | Auto-detect | Let it auto-detect your battery. |
| **Low Voltage Cutoff** | Enabled | Important safety feature — leave on. |
| **Smart Fan** | Auto (55°C threshold) | No reason to change this. |

#### Settings That Do NOT Affect Hall Sensor Signals

These ESC settings control the motor phase outputs (the big 3 power wires), NOT
the hall sensor signal wires. They won't interfere with your Arduino reading:

- Motor rotation direction (CW/CCW)
- Current limiting
- Over-temperature protection
- Battery cell count

### XC E3665 Motor — Hall Sensor Details

- **Pole count:** 4 poles = **2 pole pairs**
- **Hall sensors per revolution:** Each hall sensor triggers **2 times per
  mechanical revolution** (once per pole pair)
- **With 3 hall sensors:** You get **6 transitions per revolution**
- **RPM formula:** `RPM = 60,000,000 / pulseInterval / 2` (for single hall sensor)
  or `RPM = 60,000,000 / pulseInterval / 6 * 3` (using all 3 halls)
- **6th wire (white):** On the XC E3665, this is likely a **temperature sensor**
  (10kΩ NTC thermistor referenced to GND), NOT a speed sensor. The E10 ESC
  monitors motor temperature through this wire. You can ignore it for RPM purposes.

### Overall Confidence Assessment

| Factor | Confidence |
|--------|------------|
| Y-splitter will pass signals correctly | **95%** — it's just parallel wiring |
| Arduino can read hall signals | **95%** — 5V logic, well-documented |
| No interference from ESC | **85%** — digital signals are robust; add 100nF caps as insurance |
| RPM calculation will be accurate | **80%** — need to verify pole count in practice (spin motor by hand, count pulses) |
| Overall approach | **HIGH** — this is a well-proven technique used in ebike and RC communities |

The 15-20% uncertainty is not about whether it will work, but about calibration
(exact pole count) and potential need for a simple capacitor filter. The
fundamental approach is sound and widely used.

---

## Sources

- [Comprehensive Guide to Julet Connectors (electricvahicals.com)](https://electricvahicals.com/comprehensive-guide-to-julet-connectors-for-ebikes-6-pin-jolet-ebike-connector/)
- [Ebike Connector Pinout Explained (letrigo.com)](https://letrigo.com/blogs/knowledge/ebike-connector-pinout-explained)
- [Grin Technologies Connector Reference (ebikes.ca)](https://ebikes.ca/learn/connectors.html)
- [eRowBike Connector Reference](https://erowbike.com/connectors)
- [Arduino Forum — RPM Reading Using Hall Effect Sensor](https://forum.arduino.cc/t/rpm-reading-using-hall-effect-sensor/408805)
- [Arduino Forum — Listening to 5V Hall Sensor Signal](https://forum.arduino.cc/t/how-to-listen-in-on-5v-hall-sensor-signal-in-machine-with-arduino-due/398448)
- [Endless Sphere — 6 Wire Hall Diagram](https://endless-sphere.com/sphere/threads/6-wire-hall-diagram.119698/)
- [Endless Sphere — Hooking Up Arduino to E-bike](https://endless-sphere.com/sphere/threads/hooking-up-an-arduino-to-an-e-bike.75057/)
- [Sensor Shield V5.0 (ProtoSupplies)](https://protosupplies.com/product/sensor-shield-v5-0/)
- [Sensor Shield V5 Guide (PCBSync)](https://pcbsync.com/arduino-sensor-shield-v5/)
- [DIYables Proto Screw Shield](https://diyables.io/products/proto-screw-shield-assembled-terminal-block-prototype-expansion-board-for-arduino-uno)
- [Arduino Proto Shield Rev3 (Official)](https://store.arduino.cc/products/proto-shield-rev3-uno-size)
- [XC E10 ESC Product Page (xc-esc.com)](https://www.xc-esc.com/product/e10-electric-speed-controller/)
- [XC E3665 Motor Product Page (xc-esc.com)](https://www.xc-esc.com/product/e3665-sensored-brushless-motor-for-rc-cars/)
- [XC-Link App (xc-esc.com)](https://www.xc-esc.com/xc-link/)
- [XC-ESC E Series Guide (fatboysrc.co.uk)](https://fatboysrc.co.uk/index.php/2025/05/20/xc-esc-e-series-brushless-motor-and-esc/)
- [XC E10 ESC Programming Guide (xc-esc.com)](https://www.xc-esc.com/programming-escs-for-race-guide/)
- [Arduino Forum — Filtering Noise from Hall Sensor near ESC](https://forum.arduino.cc/t/how-to-filter-noise-out-of-hall-speed-sensor-esc-and-ignition-spark-closeby/264130)
- [Arduino Forum — Hall Sensor Picking Up PWM Frequency](https://forum.arduino.cc/t/hall-effect-sensor-picking-up-input-pwm-frequency-to-the-motor/1082674)
