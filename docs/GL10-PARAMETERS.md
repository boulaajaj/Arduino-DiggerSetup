# GL10 ESC — Configurable Parameters Reference

**Source:** XC-ESC GL10 official user manual (`docs/GL10-Manual.pdf`,
3-page PDF from <https://www.xc-esc.com/product/rc-brushless-sensored-foc-gl10-esc-80a2-4s/>).

**Configuration interface:** XC-Link Bluetooth phone app (the GL10 has no
programming card or DIP switches — Bluetooth only).

**Default password:** `1234`.

## Configurable items table

| Seq | Item | Parameter Range | Default | Notes |
| --- | --- | --- | --- | --- |
| 1 | Running Mode | "Forward/Reverse with Brake" / "Forward/Reverse" | Forward/Reverse with Brake | Brake mode required for drag brake / max brake force to apply. |
| 2 | LiPo Cells | Auto Identify, 2-4S | Auto Identify | Sets low-voltage cutoff threshold. |
| 3 | BEC Voltage | 6.0V / 7.4V | 6.0V | Output to receiver/Arduino. **Do not exceed servo Vmax.** |
| 4 | Cutoff Voltage | Disable / 2.9-3.6V per cell | 3.2V | Per-cell cutoff. |
| 5 | Motor Rotation | CW / CCW | CW | Use to flip motor direction without rewiring. |
| 6 | Torque Compensation | 1-6 | 6 (max) | Output max torque when encountering obstacles. |
| 7 | **Simulation Inertia** | 1-6 times | **3** | ESC-side inertia simulation. Compounds with any Arduino-side smoothing. |
| 8 | **Drag Force** | 0-100% | **20%** | Active brake applied when stick returns to neutral — the "engine braking" feel. |
| 9 | **Max Brake Force** | 20-100% | **20%** | Cap on brake authority when transitioning forward→reverse range. |
| 10 | Max Throttle | 20-100% | 100% | Forward power cap. |
| 11 | **Max Reverse Force** | 20-100% | **50%** | Cap on reverse power. **Critical for tank-style pivot — see below.** |
| 12 | Acceleration | 0-12 | 6 | Throttle response rate / smoothing. |
| 13 | Turbo Timing | 0-24 | 0 | Electrical degree timing advance for turbo (race feature, irrelevant here). |
| 14 | Turbo Delay | 0-1s | 1s | Time to engage turbo. |
| 15 | Neutral Range of Throttle | 2-15% | 7% | Deadband around 1500 µs. |
| 16 | X-BUS-ID | 0-15 | 0 | Slave address on the shared X.BUS. |
| 17 | Cooling Fan | Temp. Control / On | Temp. Control | Fan control mode. |

## Running mode details (item 1)

- **"Forward/Reverse with Brake"** — When the trigger crosses from forward
  range into reverse range, braking is activated first; sustained reverse
  input then engages reverse. This is the mode that enables drag brake and
  max brake force settings.
- **"Forward/Reverse"** — Passing through neutral immediately reverses,
  no brake stage. "Generally used in special vehicles."

## Code-context analysis

This is the **"ESC / motor configuration changes" rule from `CLAUDE.md`** applied
to each setting: list every command path the Arduino code can drive a
motor through, and check whether the ESC parameter clips, scales, or
distorts a legitimate command.

### Command-stream summary (V7.1)

The Arduino sends a per-track PWM in 1000-2000 µs around 1500 µs neutral.
The maximum signed range before the ESC sees it:

| Path | Max forward (per track) | Max reverse (per track) |
| --- | --- | --- |
| RC straight throttle (high gear) | +500 µs (100%) | -500 µs (100%) |
| RC pivot full lock (high gear) | +275 µs ( 55%) | -275 µs ( 55%) |
| RC pivot full lock (low gear, 0.6× scale) | +165 µs ( 33%) | -165 µs ( 33%) |
| Joystick equivalents | same as RC after expo | same |

`PIVOT_SPEED_CAP = 0.55` is what sets the 55% pivot ceiling. Gear scales
that down to 33% in low gear and 38% in mid gear.

### Per-parameter impact

| Parameter | Impact on the V7.1 code | Recommendation |
| --- | --- | --- |
| **Max Reverse Force = 50% (default)** | **BREAKS PIVOT.** When pivot commands -55% on the slower track at high gear, the ESC delivers only -27.5%. Differential between tracks collapses, vehicle won't spin. Setting too low silently caps differential authority. | **Set to 100%.** Reverse is the symmetric half of throttle on a tank-style controller — capping it at <100% is wrong for differential drive. |
| **Simulation Inertia = 3 (default)** | ESC adds its own command smoothing on top of any Arduino-side inertia. With the V7.1 exponential decay filter still active, the two compound — the operator feels two layers of "coasting." Also slows command acceptance. | **Set to 1 (lowest).** Either the ESC or the Arduino owns inertia, not both. The simpler architecture is to let the FOC handle motor-side smoothness via Acceleration + Drag Force, and remove the Arduino-side inertia filter. |
| **Drag Force = 20% (default)** | Active brake when stick returns to neutral. This is the "predictable stop on release" the operator wants. 20% may be too soft for a 120 lb vehicle; vehicle may still feel coasty. | **Tune empirically: try 30-40% first.** Higher = sharper stop, lower = more coast. Easy to A/B. |
| **Max Brake Force = 20% (default)** | Cap on brake authority during commanded brake (forward→neutral or forward→reverse transition). | **Try 50-65%** for stronger emergency-stop authority while leaving headroom. |
| **Max Throttle = 100%** | Forward power ceiling. | Keep at 100%. Forward cap is already enforced in software via gear scaling. |
| Running Mode = Fwd/Rev with Brake | Required for drag brake and max brake force to apply. | Keep. |
| Neutral Range of Throttle = 7% | Deadband at center. Our V7.1 `RC_DEADBAND` already filters input around 1500 µs; ESC deadband stacks on top of that. | Keep at 5-7%, lower if commands feel "ignored" near center. |
| Acceleration = 6 | Throttle response rate at the ESC. Affects launch feel. | Keep at 6 unless launch feels jerky (lower) or sluggish (higher). |
| Torque Compensation = 6 | Output max torque on obstacles. Crawler-tuned default. | Keep. |
| BEC Voltage = 6.0V | Powers Arduino VIN per `WIRING-GUIDE.md`. | Keep. Do not change. |
| Motor Rotation | Set per-track via XC-Link so the two tracks rotate in opposite physical directions (one CW, one CCW) for forward motion. The V7.1 steering inversion in `curvatureDrive` (commit `6bfe536`) was retired in V7.4 (commit `431bf95`) after the motor/ESC replacement made the in-software flip over-correct. | Keep per-track; no software compensation needed. |
| X-BUS ID | ESC Left = 0, ESC Right = 1 per `WIRING-GUIDE.md`. | Keep. |

### Recommended XC-Link configuration (apply to BOTH ESCs)

| Item | Value | Why |
| --- | --- | --- |
| Running Mode | Forward/Reverse with Brake | Default; required for brake/drag brake to apply. |
| **Max Reverse Force** | **100%** | **Pivot requires symmetric forward/reverse authority.** |
| **Simulation Inertia** | **1** (lowest) | Avoid double-smoothing with Arduino-side inertia. |
| **Drag Force** | **30%** (start; tune up to 40% if still coasty) | Active stop when stick released. Address the safety concern with the 120 lb vehicle. |
| **Max Brake Force** | **60%** | Stronger emergency stop authority. |
| Max Throttle | 100% | Default. |
| Acceleration | 6 | Default. |
| Torque Compensation | 6 | Default (crawler-tuned). |
| Neutral Range of Throttle | 7% | Default. |
| BEC Voltage | 6.0V | Per WIRING-GUIDE. |
| LiPo Cells | Auto Identify (or 3S explicit) | Per current battery (3S). |
| Cutoff Voltage | 3.2V | Default. Protects 3S LiPos. |

The Arduino-side inertia filter that this doc originally recommended
removing was retired in V7.2 (commit `431e267`). The ESC's
Acceleration + MaxDragForce now own command smoothing end-to-end —
configuring those XC-Link values is the only smoothing knob left.

## Reconciliation with `XBUS-PROTOCOL.md`

The X.BUS protocol doc lists default values per register (e.g.
`StopBrakPow` default = 0 at register 0x15). Those are the **generic
register defaults across the XC ESC product family**, not what ships on
the GL10. The GL10's factory configuration overrides several of those:

| X.BUS Register | Generic Default | GL10 Factory Default |
| --- | --- | --- |
| `0x12 brakMaxPow` (Max Brake Force) | 0 | 20% |
| `0x15 StopBrakPow` (Drag Force) | 0 | 20% |
| `0x13 backMaxPow` (Max Reverse Force) | 50% | 50% |

Trust the GL10 manual's defaults over the X.BUS register doc when they
disagree.

## Notes for future reference

- **Reverse direction is implicit on a tank-style controller.** Anything
  that caps "reverse" power asymmetrically will distort differential
  steering. This applies to Max Reverse Force and to any throttle-curve
  setting that's biased forward.
- **The ESC does not expose drag brake direction.** Drag Force always
  acts toward neutral regardless of which way the motor was last spinning
  — good for our use case (we want the stop to feel symmetric on
  forward-coasting and reverse-coasting).
- **Simulation Inertia is software-only inside the ESC.** It does not
  add mechanical resistance; it just smooths the command. Stacking it
  with our Arduino-side inertia is double-smoothing that the operator
  feels as exaggerated coasting.
