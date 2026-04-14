# Tracked Vehicle Control Research

Reference document for the Arduino DiggerSetup project. Covers RC input
architecture, tank mixing best practices, and efficient loop patterns for
real-time motor control on the Arduino Nano R4 (Renesas RA4M1).

Last updated: 2026-04-01

---

## 1. RC Input on Arduino Nano R4

### Interrupt Pin Support (Confirmed)

The Nano R4 (Renesas RA4M1) has limited `attachInterrupt()` support.
Not all digital pins have IRQ lines routed to the Interrupt Control Unit.

| Pin | attachInterrupt | Notes |
| --- | --- | --- |
| D0 | Yes | Conflicts with Serial RX |
| D1 | Yes | Conflicts with Serial TX |
| D2 | Yes | Primary — used for RC CH1 |
| D3 | Yes | Primary — used for RC CH4 |
| D4 | **No** | digitalPinToInterrupt() returns -1 |
| D5 | **No** | No IRQ line |
| D6 | **No** | No IRQ line |
| D7 | **No** | digitalPinToInterrupt() returns -1 |
| D8 | Yes | Available |
| D12 | Yes | Available |
| D13 | Yes | Available (also LED_BUILTIN) |

Source: Arduino forum testing + RA4M1 ICU event multiplexing constraints.

### Why pulseIn() Is Problematic

`pulseIn()` blocks the CPU waiting for a complete HIGH pulse:

- **Blocking duration**: Up to 25ms per call (PULSE_TIMEOUT)
- **Alternating pattern**: CH2 and CH5 share polling, each gets ~20-25Hz
- **Timestamp drift**: Any `micros()` captured before pulseIn is stale by
  the time pulseIn returns, causing unsigned arithmetic wraparound in
  failsafe checks
- **Race condition**: ISR-updated timestamps from interrupt channels can
  appear to be "in the future" relative to the stale timestamp, causing
  failsafe to falsely trigger on ALL channels

### Recommended Pattern: Non-Blocking Pulse Measurement

Instead of blocking with pulseIn(), sample pin state every loop iteration
and measure pulse width by tracking edges over time:

```cpp
// Non-blocking pulse measurement for non-interrupt pins
struct PulseReader {
  uint8_t pin;
  bool lastState;
  unsigned long riseTime;
  int pw;               // Last measured pulse width (us)
  unsigned long lastOk;  // Timestamp of last valid measurement

  void init(uint8_t p) {
    pin = p;
    pinMode(pin, INPUT);
    lastState = digitalRead(pin);
    riseTime = 0;
    pw = 1500;
    lastOk = 0;
  }

  void poll() {
    bool state = digitalRead(pin);
    unsigned long now = micros();
    if (state && !lastState) {
      // Rising edge
      riseTime = now;
    } else if (!state && lastState && riseTime > 0) {
      // Falling edge — measure pulse
      unsigned long width = now - riseTime;
      if (width >= 800 && width <= 2200) {
        pw = width;
        lastOk = now;
      }
    }
    lastState = state;
  }
};
```

**Advantages over pulseIn():**

- Zero blocking — returns immediately every call
- Works at any loop rate (faster loop = more precise edge detection)
- Consistent timestamps — no stale `now` problem
- Same accuracy as pulseIn for RC signals (1-2ms pulses at 50Hz)

**Accuracy**: At 48MHz (Nano R4), the loop runs fast enough that edge
detection jitter is <5us — well within RC signal tolerance.

### Timestamp Management Rules

1. **Never capture `micros()` before a blocking call and use it after**
2. **Use fresh `micros()` at point of use** — especially for failsafe
3. **On ARM Cortex-M4, 32-bit reads are atomic** — but use `noInterrupts()`
   for reading multiple related volatile variables together
4. **ISR timestamps may be newer than main loop timestamps** — always
   account for this when comparing

---

## 2. Tank Mixing for Heavy Tracked Vehicles

### Industry Practices (1-5 Ton Machines)

Specific control parameters are proprietary to each OEM (Caterpillar,
Bobcat, Kubota, Komatsu). The following are engineering consensus values
derived from equipment specifications, operator feedback, and safety
standards.

### Reverse Speed Limiting

| Parameter | Value | Rationale |
| ----------- | ------- | ----------- |
| Max reverse | 50% of forward | Visibility is limited in reverse; operator reaction time is longer |
| Reverse ramp | Same as forward decel | Gradual engagement prevents track shock |

Real machines (Bobcat, CAT) limit reverse to 30-60% of forward max
depending on model. 50% is a safe starting point for a ride-on excavator.

### Pivot Turn (Counter-Rotation) Limiting

| Parameter | Value | Rationale |
| ----------- | ------- | ----------- |
| Full pivot power | 40-50% of forward max | Full-speed pivot on a 3t machine throws the operator |
| Graduated blend | Linear from 100% (straight) to 50% (full pivot) | Smooth transition, no abrupt cutoff |

A "full pivot" is when both tracks run at equal speed in opposite
directions. The spin ratio = min(|L|,|R|) / max(|L|,|R|) when signs
differ. Scale = 1.0 - spinRatio * (1.0 - SPIN_LIMIT).

### Turn Rate During Forward Travel

| Parameter | Value | Rationale |
| ----------- | ------- | ----------- |
| Inside track minimum | 0% (can stop, not reverse) during forward travel | Prevents unexpected reversal at speed |
| Differential limit | Inside track >= 20% of outside track at high speed | Prevents tip-over on hard surfaces |

Best practice: At high forward speed, limit the maximum speed differential
between tracks. A full-lock turn (one track stopped) at high speed causes
extreme lateral forces.

### Acceleration and Deceleration

| Parameter | Value | Rationale |
| ----------- | ------- | ----------- |
| Accel time constant | 1.0-2.0s (63% of target) | Prevents track slip on loose ground |
| Decel time constant | 2.0-4.0s (63% coast-down) | Simulates momentum of heavy machine |
| Asymmetric ratio | Decel ~2x slower than accel | Machine should coast, not stop abruptly |

Real hydraulic machines have inherent inertia from fluid dynamics in the
hydrostatic drive. Electric drive must simulate this or the machine feels
unnaturally responsive ("go-kart feel").

The asymmetric exponential filter is the recommended pattern:

```text
alpha = dt / (dt + tau)
position += (target - position) * alpha
```

Where `tau` = TAU_ACCEL when speeding up, TAU_DECEL when slowing down.

### Exponential Response Curve

| Parameter | Value | Rationale |
| ----------- | ------- | ----------- |
| Expo blend | 20% linear + 80% quadratic | Fine control at low stick, full range at high stick |
| Equivalent expo | ~1.8 power curve | Matches industrial joystick controller feel |
| Dead zone | 3-8% of travel | Prevents unintended creep; our 480/8192 = 5.9% is good |

Industrial joystick controllers (Danfoss, Parker, Rexroth) use
configurable expo curves. Quadratic (power 2.0) is the most common
default. A 20/80 linear/quadratic blend approximates power 1.8, which
gives slightly more low-speed authority than pure quadratic.

### Safety Practices

| Practice | Implementation | Standard |
| ---------- | --------------- | ---------- |
| Dead zone | 3-8% center dead zone | Common practice |
| Failsafe | Neutral on signal loss (per-channel, 0.5s) | ISO 15817 |
| Emergency stop | Bypass all ramps, immediate neutral | ISO 13850 |
| Max speed ramp | Never allow instantaneous full power | ISO 20474 |
| Neutral lockout | Must center controls before engaging | Common practice |
| Current limit | Hard cutoff at motor stall current | Safety critical |

### Relevant Standards

- **ISO 20474** — Earth-moving machinery safety (loaders, excavators)
- **ISO 15817:2012** — Remote operator control systems for earth-moving machinery
- **ISO 13850** — Emergency stop function requirements
- **SAE J2944** — Joystick control layout for surface vehicles
- **ISO 7752-1:2010** — Cranes control layout and characteristics

---

## 3. Efficient Loop Patterns for Real-Time Control

### Loop Architecture: Continuous + Time-Skip (Recommended)

```cpp
void loop() {
  unsigned long now = micros();
  float dt = (now - prevTime) * 1e-6f;
  prevTime = now;

  pollInputs();           // Non-blocking edge detection
  if (controlTimer(now))  // Fixed-rate control (e.g., 50Hz)
    runControl(dt);
  outputWrite();          // Non-blocking servo update
  debugPrint(now);        // Rate-limited serial output
}
```

**Key principles:**

- Loop runs as fast as possible (input polling benefits from speed)
- Control math runs at a fixed rate (decoupled from loop speed)
- No blocking calls anywhere in the main loop
- Servo.writeMicroseconds() is non-blocking (~2us)

### ISR Best Practices (ARM Cortex-M4)

- Keep ISRs under 10us (read pin, store timestamp, exit)
- Use `volatile` for all ISR-shared variables
- 32-bit reads are atomic on ARM — no `noInterrupts()` needed for single
  variable reads
- Use `noInterrupts()` only when reading multiple related variables that
  must be consistent with each other
- Never call Serial, delay, tone, or allocate memory in an ISR

### Float vs Fixed-Point

The Nano R4 has a hardware FPU (VFPv4). Float operations are 1-2 cycles:

- Use `float` for control math, filters, expo curves
- Use `int` for sensor values, PWM outputs, counters
- Never use `double` (software emulated, 10x slower)
- The `f` suffix matters: `1.0f` uses FPU, `1.0` may use software double

### Memory Budget (Nano R4: 32KB RAM)

Current sketch uses ~300 bytes of globals + ~3KB Arduino runtime.
~28KB available for future expansion (PID state, telemetry buffers, etc.).

### Timing Budget (per loop iteration at 48MHz)

| Operation | Time |
| ----------- | ------ |
| micros() | 2 us |
| digitalRead() x4 (polling) | 4 us |
| analogRead() x2 (double-read) | 400 us |
| Expo + tank mix | 15 us |
| Spin limit + soft limit | 15 us |
| Inertia filter | 10 us |
| Servo.writeMicroseconds() x2 | 4 us |
| Serial.println() (10Hz) | 50 us (amortized) |
| **Total** | **~500 us** |

At 500us per loop, the effective loop rate is ~2000Hz — far above the
50Hz control rate needed. This leaves massive headroom for PID, telemetry,
and future features.

---

## 4. Code Organization

### Module Pattern (Single .ino + types.h)

```text
sketches/rc_test/
  rc_test.ino    — Main sketch with [MODULE] sections
  types.h        — Shared struct definitions
```

Each module is marked with `[MODULE_NAME]` comments for searchability:

- `[CONFIG]` — All tunable constants
- `[RC]` — RC input (ISR + polling + failsafe)
- `[JOYSTICK]` — ADC reading, expo, tank mix
- `[MIXER]` — Override switch, input source selection
- `[DYNAMICS]` — Spin limiter, reverse limiter, soft limit, inertia
- `[OUTPUT]` — ESC servo PWM
- `[DEBUG]` — Serial CSV telemetry

### Why Single File

- Arduino IDE concatenates multiple .ino files alphabetically (fragile)
- Single file with clear sections is easier to review and prompt
- types.h solves the Arduino auto-prototype limitation for structs
- When the sketch grows past ~500 lines, consider splitting into .h/.cpp
  pairs (not multiple .ino files)
