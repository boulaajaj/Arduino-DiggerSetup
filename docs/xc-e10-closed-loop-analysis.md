# Technical Analysis of Closed-Loop Speed Governance for Tracked Robotics

*A Study of the Arduino UNO Q and XC E10 Hardware Ecosystem*

> **Note (2026-03-21):** The "Mechanical Considerations" section about stiction
> (0→1 RPM startup) is **incorrect** for our use case. The actual concern is
> **cyclic load variation from stiff rubber track engagement/disengagement on
> sprocket teeth** — a periodic disturbance at every RPM, not a one-time startup
> issue. See CLAUDE.md for the corrected understanding.

---

## Overview

The engineering of autonomous and semi-autonomous tracked vehicles necessitates
a rigorous approach to motion control, particularly when navigating unpredictable
terrain. The challenge is characterized by the requirement for constant rotational
velocity under fluctuating load conditions, such as the inherent resistance of
high-durometer rubber tracks, incline gradients, and physical obstructions.
Achieving a "smooth and controlled" feel in a skid-steer platform requires more
than simple open-loop throttle commands; it demands a deterministic, closed-loop
speed governor capable of sub-millisecond responsiveness. The specific hardware
ensemble—consisting of the Arduino UNO Q 2GB, the XC E10 sensored brushless
Electronic Speed Controller (ESC), and the E3665 sensored motor—presents a
sophisticated dual-architecture solution.

## Hardware Architecture: The UNO Q's Dual-Brain Approach

The Arduino UNO Q integrates the Qualcomm Dragonwing QRB2210 Microprocessor (MPU)
with the STMicroelectronics STM32U585 Microcontroller (MCU). In the context of a
speed governor, this allows for a logical separation of concerns:

| Component | Architecture | Primary Role in Speed Control |
|-----------|-------------|-------------------------------|
| Qualcomm QRB2210 | Quad-core Arm Cortex-A53 | High-level setpoint calculation and communication |
| STM32U585 | Arm Cortex-M33 (160 MHz) | Real-time PID execution and sensor interrupts |
| 2GB LPDDR4 RAM | LPDDR4 | Data buffering and Linux OS overhead |
| 16GB eMMC | eMMC | Storage for telemetry logs and OS |
| WCBN3536A | Wi-Fi 5 / BT 5.1 | Remote telemetry and parameter adjustment |

The MPU, running a full Debian Linux environment, can handle high-level inputs
like joystick processing, SLAM, or complex mission logic. The MCU, running on the
Zephyr Real-Time Operating System (RTOS), focuses exclusively on the time-critical
feedback loop.

The STM32U585 achieves a 12.5x throughput improvement over legacy Arduino platforms
like the UNO R3, ensuring that the control loop never misses an interrupt even at
high motor speeds.

## Sensored Motor Feedback: Hall Effect Physics

The E3665 motor utilizes three internal Hall effect sensors to detect the position
of the rotor's magnetic poles. These sensors provide a digital signal that toggles
between high and low states as the magnets pass by. The XC E10 uses these signals
internally to time its phase switching precisely.

### E3665 Motor Parameters

| Motor Parameter | Value | Implications for Feedback |
|----------------|-------|--------------------------|
| Pole Pairs | 2 | 2 electrical cycles per mechanical rev |
| Transitions per Rev | 12 | High resolution for low-speed governance |
| No-load Current (2500KV) | ≤ 5.0A | Baseline current for friction compensation |
| Operating Voltage | 2-4S LiPo | 7.4V to 14.8V nominal range |

For the E3665, where pole_pairs = 2, each mechanical revolution consists of two
electrical cycles. Since each electrical cycle produces six state changes across
the three Hall sensors (A, B, and C), the Arduino can observe 12 distinct
transitions per mechanical revolution.

## Signal Interfacing: Tapping Hall Sensors

### Interfacing Methods Comparison

| Method | Complexity | Safety Level | Impact on Signal Integrity |
|--------|-----------|-------------|---------------------------|
| Direct Connection | Low | Risky | Potential noise injection into ESC |
| Series Resistor (1k-10k) | Low | Moderate | Acceptable for low-speed testing |
| Logic Level Shifter | Moderate | High | Best for reliability and noise immunity |
| Opto-isolation | High | Extreme | Ideal for high-voltage industrial systems |

**Key concern:** The STM32U585 operates at 3.3V logic. Hall sensor outputs are 5V.
A high-impedance logic-level shifter or non-inverting buffer IC (such as the
74LVC244) is recommended to avoid loading the signal (which could cause the ESC to
desync — catastrophic motor failure under high load).

## Control Loop Update Rates

### XC E10 Effective Update Rates

The XC E10 supports "wave-by-wave current limiting" and "active freewheeling,"
suggesting an internal PWM frequency of 8-32 kHz. For the input signal (Arduino to
ESC), most high-end sensored ESCs can handle 200-400 Hz reliably.

| Update Frequency | Transport Delay | Control Loop Stability | Best Use Case |
|-----------------|----------------|----------------------|---------------|
| 50 Hz | 20 ms | Moderate | Slow-moving utility vehicles |
| 100 Hz | 10 ms | High | Standard off-road navigation |
| 200 Hz | 5 ms | Very High | High-speed agility and obstacles |
| 400 Hz | 2.5 ms | Maximum | Precision robotics and skid-steer |

### Zephyr RTOS Interrupt Considerations

The UNO Q runs the Arduino core on top of Zephyr OS. Simple interrupts can
occasionally misbehave if the system is under heavy load from the MPU side. RPM
calculation interrupts should be configured with "zero latency" priorities where
possible.

## Communication Protocols: PWM vs. X.BUS

The XC E10 supports X.BUS protocol for bi-directional communication:

| Protocol | Type | Direction | Advantage |
|----------|------|-----------|-----------|
| PWM (Standard) | Analog/Pulse | One-way | Simple, universal compatibility |
| X.BUS | Digital Bus | Bi-directional | Telemetry, precision, multi-device support |
| Bluetooth | Wireless | Bi-directional | Parameter setting via mobile app |

X.BUS allows reading voltage, current, temperature, and RPM directly from the ESC,
and is designed for "robot control or other automated programming control."

## Rubber Track Cyclic Load (CORRECTED)

> **Original report incorrectly described this as stiction (0→1 RPM startup).**

The actual phenomenon: stiff rubber tracks create a **periodic load variation** as
each link engages and disengages sprocket teeth:

- **Link falls onto sprocket tooth** → stiff rubber must deform to wrap around →
  **increased resistance** (motor works harder)
- **Link releases from sprocket** → rubber springs back → **decreased resistance**
  (momentary load drop)

This produces a cyclic disturbance at frequency = `RPM × sprocket_teeth / 60` Hz.
The closed-loop controller must be fast enough to compensate for these per-tooth
load pulses to maintain constant track velocity.

## ESC Configuration Recommendations

For external closed-loop control, the ESC's internal smoothing should be minimized:
- **Drag Brake Force**: Minimize (Arduino should be the master of deceleration)
- **Punch Level**: Minimize (Arduino controls acceleration curve)
- **Active Freewheeling**: Configure for most direct, linear response

The Arduino should be the sole "master" of the acceleration/deceleration curve.

## Thermal Management

The XC E10 features "Intelligent Heat Dissipation" with fan activation above 55°C.
The Arduino should monitor ESC temperature via X.BUS telemetry to prevent thermal
shutdown during prolonged heavy-load operation. The IP67 waterproof rating of both
the E10 and E3665 ensures resilience to mud and moisture.

---

## Sources

1. https://docs.arduino.cc/hardware/uno-q (UNO Q | Arduino Documentation)
2. https://www.arduino.cc/product-uno-q (Discover the New Arduino UNO Q)
3. https://docs.arduino.cc/tutorials/uno-q/user-manual/ (UNO Q User Manual)
4. https://www.dfrobot.com/product-2996.html (Arduino UNO Q - DFRobot)
5. https://dronebotworkshop.com/arduino-uno-q-1/ (Arduino Uno Q - DroneBot Workshop)
