# Digger Control System — Operator Guide

## For Jason (RC remote) and Malaki (joystick)

This machine has **two ways to drive it**: an RC remote controller and a
joystick. An Arduino Nano sits in the middle and
decides which input controls the tracks based on a 3-position switch on
the remote.

---

## How It Works

The machine has **two tracks** (left and right), each driven by its own
motor and ESC. The Arduino reads signals from both the RC remote and the
joystick, picks the right one based on the override switch, and sends
smoothed commands to the motors.

```
  Jason's RC Remote ──────┐
                          ├──> Arduino ──> Left Track Motor
  Malaki's Joystick ──────┘           ──> Right Track Motor
```

---

## The 3 Modes (Override Switch on RC Remote)

The 3-position switch on Jason's remote controls who drives:

### Mode 1 — Remote Only (switch DOWN)
- **Jason** has full control of both tracks
- **Malaki's** joystick is completely ignored
- Use this when Jason needs precise, solo control

### Mode 2 — Joystick with RC Override (switch MIDDLE)
- **Malaki** drives using the joystick
- **Jason** can take over instantly by moving his sticks
- As soon as Jason releases his sticks back to center, Malaki has
  control again
- This is the normal operating mode — Malaki drives, Jason supervises

### Mode 3 — Blended 50/50 (switch UP)
- **Both** inputs are active at the same time
- The output is a 50/50 mix of Jason's remote and Malaki's joystick
- Useful when both operators need to coordinate together
- Jason's input won't fully override Malaki's — they share control

---

## Controls

### Jason's RC Remote
- **Left stick / Right stick:** Controls left and right tracks directly
  (the transmitter handles the mixing internally)
- **3-position switch (CH5):** Selects the mode (see above)
- Moving any stick takes priority over the joystick in Mode 2

### Malaki's Joystick
- **Push forward:** Both tracks forward (machine drives straight ahead)
- **Pull back:** Both tracks backward
- **Push right:** Left track forward + right track backward (turns right)
- **Push left:** Right track forward + left track backward (turns left)
- **Diagonal:** Combines forward/backward with turning

---

## Safety Features

### 50% Power Limit with Exponential Curve
The motors are very powerful. The system limits output to **50% of full
power** at all times. This applies to BOTH the remote and the joystick.

The response follows an **exponential curve** — small stick movements
give very fine control, while larger movements ramp up progressively.
This feels like a real excavator: precise positioning at low speed,
strong power when you push further.

| Stick Position | Power Output | Feel |
|---------------|-------------|------|
| 10% | ~0.3% | Barely creeping — fine positioning |
| 25% | ~3% | Slow, controlled movement |
| 50% | ~18% | Moderate working speed |
| 75% | ~37% | Strong movement |
| 100% (full) | 50% (max) | Full allowed power |

### Instant Response, Smooth Stop
- **Speeding up:** **INSTANT** — the machine responds immediately to
  stick input. The exponential curve prevents jerky starts naturally.
- **Slowing down:** Takes about **0.3 seconds** to coast to a stop.
  This prevents sudden jolts when the stick is released.

This means: push the stick and it moves right away (no lag), but
release the stick and it comes to a smooth, safe stop.

### RC Signal Loss Protection
If the Arduino loses the RC signal for more than half a second (remote
turned off, out of range, etc.), both motors automatically stop.

### Jason Always Has Override
In Mode 2, Jason can always take control instantly by moving his sticks.
This is a safety net — if Malaki needs help or the machine is heading
somewhere it shouldn't, Jason can intervene immediately.

---

## Quick Reference

| What | How |
|------|-----|
| Malaki drives alone | Switch to Mode 2 (middle), Jason hands off |
| Jason drives alone | Switch to Mode 1 (down) |
| Both drive together | Switch to Mode 3 (up) |
| Emergency stop | Jason: sticks to center in any mode |
| Max speed | 50% of motor capacity (safety limit) |
| Response | Instant (exponential curve) |
| Stop time | ~0.3 seconds (smooth ramp) |

---

## Adjusting Settings

These values can be changed in the Arduino sketch
(`sketches/rc_test/rc_test.ino`) by editing the tuning constants at the
top of the file:

| Setting | Current Value | What It Does |
|---------|--------------|--------------|
| `POWER_LIMIT_PCT` | 50 | Max power output (%) — raise or lower as needed |
| `EXP_CURVE` | 2.5 | Exponential feel (higher = more fine control at low stick) |
| `SMOOTH_TAU_DOWN` | 300 | Deceleration smoothing in ms (higher = slower stop) |
| `RC_DEADBAND` | 50 | How far RC sticks must move to register (μs) |
| `JOY_DEADBAND` | 480 | How far joystick must move to register (14-bit ADC units) |

After changing a value, re-upload the sketch to the Arduino using:
```
arduino-cli upload -p COM8 --fqbn arduino:renesas_uno:nanor4 sketches/rc_test
```
