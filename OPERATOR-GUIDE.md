# Digger Control System — Operator Guide

## For Jason (RC remote) and Malaki (joystick)

This machine has **two ways to drive it**: an RC remote controller and a
joystick. An Arduino UNO R4 WiFi sits in the middle and
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

### Gear Speed Caps (the safety throttle)

The motors are very powerful, so top speed is capped by the **gear switch
(CH4)** on the remote. The cap applies to BOTH the remote and the joystick:

| Gear | Top speed | Reverse | Pivot (spin in place) |
| ---- | --------- | ------- | --------------------- |
| **Eco** | 55% | 62.5% | 72.5% |
| **Normal** | 70% | 50% | 60% |
| **Turbo** | 100% | 50% | 60% |

Full stick travel is always usable — the cap just sets the maximum the stick
maps to, so no stick movement is wasted. **Start in Eco.** Eco deliberately
gives reverse and pivoting a little extra authority so the machine can still
maneuver in tight spaces at low speed.

### Smooth Acceleration

The machine won't lurch when a stick is pushed quickly — but the smoothing is
now handled **inside the GL10 ESCs** (their internal FOC ramps the motor),
not by a delay in the Arduino. The stick command is sent straight through, so
releasing the stick stops the machine promptly — important for avoiding
obstacles. (The old Arduino-side ramp was removed when the GL10 ESCs took over
that job.)

### RC Signal Loss Protection

If the Arduino loses the RC signal for about a tenth of a second (remote turned
off, out of range, etc.), both tracks are held at neutral. A hardware watchdog
also resets the controller — and drops the ESCs to neutral — if the firmware
ever stalls.

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
| Max speed | Set by gear: Eco 55% / Normal 70% / Turbo 100% |
| Smoothing | Handled inside the GL10 ESCs (FOC) |
| Signal-loss stop | Tracks neutral after ~0.1 s of lost RC |

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
| `GEAR_LOW_SCALE` | 0.55 | Eco top-speed cap (fraction of full) |
| `GEAR_MID_SCALE` | 0.70 | Normal top-speed cap |
| `GEAR_HIGH_SCALE` | 1.00 | Turbo top-speed cap |
| `REVERSE_LIMIT` | 0.50 | Reverse cap (Normal/Turbo) as a fraction of forward |
| `PIVOT_SPEED_CAP` | 0.60 | Spin-in-place (pivot) cap |
| `RC_DEADBAND` | 50 | How far RC sticks must move to register (μs) |
| `JOY_DEADBAND` | 480 | How far the joystick must move to register (ADC units, ~6% of travel) |

Smoothing is no longer an Arduino setting — it lives in the GL10 ESC's
Acceleration / Drag parameters (see `docs/GL10-PARAMETERS.md`).

After changing a value, re-upload the sketch to the Arduino using:

```bash
arduino-cli upload -p COM7 --fqbn arduino:renesas_uno:unor4wifi sketches/rc_test
```
