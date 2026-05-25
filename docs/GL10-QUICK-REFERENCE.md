# GL10 ESC — Quick Reference

**Source:** Official GL10 User Manual (`docs/GL10-Manual.pdf`, downloaded
from [xc-bldc.com](https://en.xc-bldc.com/uploadfile/upload/2025071214241982.pdf)).

**App:** TXC-Link (download from XC-ESC website or Apple App Store).
**Bluetooth name:** `XC_G10-{hex ID}`, default password `1234`.

---

## Throttle Range Calibration (Manual Section 5)

Sets the ESC's throttle endpoints to match your transmitter's stick range.
**Do this for each ESC individually, with the same transmitter channel.**

| Step | Action | Wait for |
|------|--------|----------|
| 1 | ESC power **OFF**. Move stick to **full REVERSE**. Power ON the ESC. | Red/green LED stops blinking |
| 2 | Move stick to **full FORWARD** quickly. | Green LED stops blinking |
| 3 | Move stick to **NEUTRAL** quickly. | Red LED stops blinking |

**Success:** Red + green LEDs flash 4 times, motor beeps "so-mi-do" x4.
ESC then starts normally.

**Failure:** No lights/sounds — ESC just powers on. Retry from step 1.

**Sequence is: REVERSE → FORWARD → NEUTRAL** (not forward-first like
generic ESCs).

---

## Normal Startup (Manual Section 4)

1. Check circuit for open/short/poor contact
2. Check motor is not stuck
3. Plug in battery
4. Turn on power button
5. Listen for battery cell-count prompt (quinary tones: do=5 cells,
   mi=1 cell, so=prefix). Green light on after tones = normal.
6. Throttle must be at neutral — if not, red LED flashes + short beeps

---

## LED & Beep Codes (Manual Section 9)

| Condition | LED | Sound | Meaning |
|-----------|-----|-------|---------|
| Throttle not zeroed | Red flashes quickly | Short beeps | Stick not at neutral on startup |
| Signal lost | Red flashes slowly | Long beep (2s cycle) | No valid PWM from receiver |
| Low voltage | Red×1 Green×2 | Long×1 Short×2 | Battery below cutoff threshold |
| Over voltage | Red×1 Green×3 | None | Voltage exceeds ESC rating |
| MOS overtemp (>125°C) | Red×1 Green×4 | Long×1 Short×4 | ESC too hot, resumes <100°C |
| Cap overtemp (>105°C) | Red×1 Green×5 | Long×1 Short×5 | Capacitor too hot, resumes <100°C |
| Abnormal throttle params | Red×1 Green×7 | Long×1 Short×7 | Throttle still abnormal at neutral — recalibrate |
| Hall sensor error | Red×1 Green×8 | Long×1 Short×8 | Re-plug hall cable; check motor |
| Calibrate low range | Red + Green | None | Step 1 acknowledged |
| Calibrate high range | Green | None | Step 2 acknowledged |
| Calibrate neutral | Red | None | Step 3 acknowledged |
| Calibration success | Red+Green ×4 | "so-mi-do" ×4 | Done |
| Normal idle | Green steady | None | Armed, no throttle |
| Throttle active | Green flashes faster | None | Faster = more throttle |
| Braking | Red on | None | Red off when brake released |
| Turbo active | Green steady on | None | Turbo timing engaged |
| ESC self-check fail | Red×2 / Red×2 Green×1 / Red×2 Green×2 | None | Disconnect motor, return for service |

---

## Configurable Parameters (Manual Section 7)

| # | Item | Range | Default | Notes |
|---|------|-------|---------|-------|
| 1 | Running Mode | Fwd/Rev with Brake / Fwd/Rev | Fwd/Rev | Brake mode needed for drag brake |
| 2 | LiPo Cells | Auto / 2-4S | Auto | |
| 3 | BEC Voltage | 6.0V / 7.4V | 6V | Do not exceed servo max |
| 4 | Cutoff Voltage | Disable / 2.9-3.6V per cell | 3.2V | |
| 5 | Motor Rotation | CW / CCW | CW | |
| 6 | Torque Compensation | 1-8 level | 6 | Higher = more torque on obstacles |
| 7 | Simulation Inertia | 1-8 times | 6 | Higher = more coast. Low for crawling. |
| 8 | Drag Force | 0-100% | 20% | Active brake below 900 RPM at neutral |
| 9 | Max Brake Force | 20-100% | 20% | Proportional brake authority |
| 10 | Max Throttle | 20-100% | 100% | Forward power cap |
| 11 | Max Reverse Force | 20-100% | 50% | Reverse power cap |
| 12 | Acceleration | 0-12 | 6 | Torque comp interaction |
| 13 | Turbo Timing | 0-24 | 0 | Electrical degree advance |
| 14 | Turbo Delay | 0-1s | 1s | Time to engage turbo |
| 15 | Neutral Range | 2-15% | 7% | Deadband around 1500μs |
| 16 | X.BUS-ID | 0-15 | 0 | Slave address on shared bus |
| 17 | Cooling Fan | Temp Control / On | Temp Control | Fan on >55°C, off <50°C |
| 18 | Motor Param ID | Close / On | Close | Motor auto-identification |

**Updated defaults from GL10-PARAMETERS.md:** Inertia=1, Drag=30%,
Max Brake=60%, Max Reverse=100%. See `docs/GL10-PARAMETERS.md` on main
branch for the full code-context analysis of why.

---

## Factory Reset (Manual Section 8)

**Reset Bluetooth only** (lost password):
1. Connect ESC white wire to BEC red wire (+5V)
2. Power on ESC
3. Disconnect when green LED off, red LED on
4. Remove short circuit — Bluetooth resets to password `1234`

**Reset all parameters:** Use TXC-Link app → parameter page → Default button.

---

## Key Notes

- **App name is TXC-Link**, not XC-Link
- **Calibration order is Reverse → Forward → Neutral** (unique to GL10)
- **Drag brake activates below 900 RPM** when throttle returns to zero
- **Low voltage protection** reduces power to 20% after 10 seconds below cutoff
- **Inertia default 6 is high** for crawlers — manual says "small value
  recommended for crawling, higher for straight-line racing"
- **All beeps timeout after 5 minutes** to save power; fault re-checks in 5 min
