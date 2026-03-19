# Handoff — Tank Mixer Controller

Last updated: 2026-03-18

## Current State (V2.0)

### What's Done
- **Arduino Nano R4 migration** — code now targets Renesas RA4M1 (Arm Cortex-M4, 48MHz)
- **Hardware interrupts** — `attachInterrupt()` on D2, D4, D7 (replaces AVR PCINT)
- **14-bit ADC** — joystick reads 0-16383 (was 0-1023), enabling smooth exponential curves
- **Exponential control curve** (exponent 2.5) — instant response, excavator-like feel
  - Fine control at low stick: 10% stick = ~0.3% output
  - Progressive: 50% stick = ~17.7% output
  - Full allowed power at 100% stick = 50% output
- **No acceleration delay** — throttle response is immediate (was 800ms EMA ramp)
- **Deceleration-only ramp** (300ms) — smooth, safe stop when user releases stick
- **50% power cap** — raised from 40% (output range: 1250-1750us)
- **Live plotter updated** — COM8, 14-bit ADC range, 50% limit markers
- **Branch protection** — main requires PR with 1 approving review
- **PR template** — Arduino-specific review checklist in `.github/pull_request_template.md`

### Hardware Setup
| Component | Detail |
|-----------|--------|
| Board | Arduino Nano R4 (ABX00143) |
| USB | Native USB on COM8 — no driver needed |
| Board package | `arduino:renesas_uno` (v1.5.3) |
| FQBN | `arduino:renesas_uno:nanor4` |
| Compile | `arduino-cli compile --fqbn arduino:renesas_uno:nanor4 sketches/rc_test` |
| Upload | `arduino-cli upload -p COM8 --fqbn arduino:renesas_uno:nanor4 sketches/rc_test` |

### Wiring (unchanged from V1.2)
Same pin layout, same form factor — no rewiring needed.

### Key Tuning Constants
| Constant | V1.2 Value | V2.0 Value | Notes |
|----------|-----------|-----------|-------|
| `POWER_LIMIT_PCT` | 40% | 50% | Increased for more authority |
| `EXP_CURVE` | N/A | 2.5 | Exponential exponent (higher = more fine control) |
| `SMOOTH_TAU_UP` | 800ms | removed | Instant response now |
| `SMOOTH_TAU_DOWN` | 400ms | 300ms | Slightly faster decel |
| `JOY_CENTER` | 512 | 8192 | 14-bit midpoint |
| `JOY_DEADBAND` | 30 | 480 | ~3% of 14-bit range |
| `JOY_MAX` | 1023 | 16383 | 14-bit max |

### Resource Usage
| | V1.2 (Nano V3) | V2.0 (Nano R4) |
|---|---|---|
| Flash | 6.5 KB / 30 KB (21%) | 45.7 KB / 256 KB (17%) |
| RAM | 317 B / 2 KB (15%) | 4.4 KB / 32 KB (13%) |

---

## What's Pending

### Priority 1 — Safety Kill Switch (Channel 4)
- Add CH4 RC input as emergency stop
- When CH4 is in DOWN position: all ESC outputs forced to neutral (1500us)
- Overrides everything — RC, joystick, all modes
- **Why:** During testing, a short circuit caused uncontrolled left track acceleration for ~10 seconds with no way to stop it

### Priority 2 — Field Testing
- Bench test with serial monitor (verify exponential curve feels right)
- Field test with ESCs at 50% power cap
- Tune `EXP_CURVE` exponent if needed (2.0 = quadratic, 3.0 = cubic)
- Tune `SMOOTH_TAU_DOWN` if deceleration feels too abrupt or too slow

### Priority 3 — Future Enhancements
- Joystick harness wiring identification (reverse engineering in progress)
- Full wiring on final board
- Consider independent exponential curves for RC vs joystick inputs
