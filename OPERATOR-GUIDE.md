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

```text
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

### 40% Power Limit

The motors are very powerful. The system limits output to **40% of full
power** at all times. This applies to BOTH the remote and the joystick.

Full stick travel is still usable — pushing the stick halfway gives 20%
power, all the way gives 40%. No stick movement is wasted.

| Stick Position | Power Output |
| --------------- | ------------- |
| 25% | 10% |
| 50% | 20% |
| 75% | 30% |
| 100% (full) | 40% (max) |

### Smooth Acceleration

The system prevents jerky starts by gradually ramping up motor speed:

- **Speeding up:** Takes about **0.8 seconds** to reach full requested speed
- **Slowing down:** Takes about **0.4 seconds** to stop

This means the machine won't lurch forward when the stick is pushed
quickly, but it WILL stop quickly when the stick is released — important
for avoiding obstacles.

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
| ------ | ----- |
| Malaki drives alone | Switch to Mode 2 (middle), Jason hands off |
| Jason drives alone | Switch to Mode 1 (down) |
| Both drive together | Switch to Mode 3 (up) |
| Emergency stop | Jason: sticks to center in any mode |
| Max speed | 40% of motor capacity (safety limit) |
| Ramp up time | ~0.8 seconds |
| Stop time | ~0.4 seconds |

---

## Beep Meanings (the D8 buzzer)

The digger has one buzzer that makes different sounds. **Count the beeps** —
each pattern means one thing, and nothing else.

| What you hear | Meaning | What to do |
| --- | --- | --- |
| **Two short beeps** (once, right after power-on) | Wi-Fi telemetry is up | Connect your phone to `Digger-Telemetry` to see the dashboard (optional) |
| **Steady tone** (only while held) | Someone is pressing the horn button on the RC remote | Nothing — that's the horn |
| **One long beep, every 2 seconds** | The **RC remote has been turned off for over a minute** | **Unplug the LiPo batteries.** Left plugged in, they slowly drain and can be ruined within a week or two |
| **Three fast chirps, over and over, won't stop** | A **battery is low** (below 10.5 V) | **Stop and charge the batteries.** This is a *warning only* — it does **not** stop the motors. The packs are protected by you stopping and charging them, not by the firmware. The alarm will NOT turn off until you unplug and re-power the digger. (An automatic motor cutoff is planned — issue #65.) |

**Notes:**

- The low-battery alarm is *latched* on purpose — once it starts, it keeps going
  even if the voltage bounces back up, so you can't miss it. Power-cycle to reset
  it after charging.
- The low-battery alarm checks **both** batteries and sounds for the weaker one.
- It only triggers when both batteries are reporting — if one isn't plugged in
  yet, it stays quiet (so it won't false-alarm while you're powering up).
- These alarms are **sound only.** They do not stop or slow the motors. (A
  separate low-battery motor cutoff is planned — see issue #65.)

---

## Adjusting Settings

These values can be changed in the Arduino sketch
(`sketches/rc_test/rc_test.ino`) by editing the tuning constants at the
top of the file:

| Setting | Current Value | What It Does |
| --------- | -------------- | -------------- |
| `POWER_LIMIT_PCT` | 40 | Max power output (%) — raise or lower as needed |
| `SMOOTH_TAU_UP` | 800 | Acceleration smoothing in ms (higher = slower start) |
| `SMOOTH_TAU_DOWN` | 400 | Deceleration smoothing in ms (higher = slower stop) |
| `RC_DEADBAND` | 50 | How far RC sticks must move to register (μs) |
| `JOY_DEADBAND` | 30 | How far joystick must move to register (ADC units) |

After changing a value, re-upload the sketch to the Arduino using:

```bash
arduino-cli upload -p COM7 --fqbn arduino:avr:nano:cpu=atmega328 sketches/rc_test/rc_test.ino
```
