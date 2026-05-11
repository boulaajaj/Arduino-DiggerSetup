# Mission — Smoothness Above All

This document is the constitution of the project. Everything else in the
codebase should be consistent with it. If a code change conflicts with any
point below, the change is wrong — not the mission.

## The goal

A ride-on excavator that a child can operate safely. Two brushless motors
drive rubber tracks through ESCs. Operators are a child (joystick) and an
adult supervisor (originally RC, now removed).

## The one rule

**Smoothness of the throttle command stream is the top priority. Above
responsiveness. Above efficiency. Above closed-loop accuracy.**

A kid can get thrown off the machine by a sudden jerk. A slightly
lethargic throttle is annoying. A sudden throttle jump is dangerous.
Always pick lethargic over jumpy.

## What "smooth" means precisely

The throttle command that the Arduino sends to the ESC must be a
continuous, slew-limited signal. Every value from 0% through 100% must
be reachable in sequence — the Arduino sends through 1%, 2%, 3%, and so
on, in order, limited to a small maximum change per control cycle.

Currently the slew limit is **1% per 10 ms cycle** (100% per second
across the full range). This value can be tuned, but the principle
cannot be traded away.

## What the Arduino is NOT allowed to do

1. **Skip any part of the throttle range.** The ESC or motor may have a
   dead zone where small throttle produces no motion. That is the ESC's
   business. The Arduino still sends those values. If we ever swap to an
   ESC without a dead zone, the controller must still work without a
   rewrite.
2. **Jump past a range in one step.** Even if the Arduino "knows" a range
   is uninteresting, it walks through it at the normal slew rate.
3. **Bake plant characteristics into the output layer.** Measured data
   about dead zones, minimum-viable throttle, S-curves, etc. belongs in
   the feed-forward *mapping* (target RPM → expected throttle). It does
   not belong anywhere downstream of that mapping.
4. **Let closed-loop corrections override smoothness.** If a trim
   correction would produce a jump larger than the slew limit, clip the
   trim, not the smoothness.
5. **Assume a specific ESC or motor.** Plant measurements inform the
   feed-forward table, but the controller architecture must work with
   any ESC that accepts X.BUS commands.

## What the Arduino IS responsible for

1. Read the joystick.
2. Apply stick shaping (deadband, expo curve).
3. Compute a target RPM per track.
4. Look up the feed-forward throttle for that target RPM.
5. Apply a small slow-loop trim based on actual-vs-target RPM error.
6. Slew-limit the final output so it changes no more than 1% per cycle.
7. Send the resulting throttle over X.BUS at 100 Hz.

That's it. Nothing else.

## Priorities in order

1. Smoothness of the throttle command.
2. Correct direction (forward when stick is forward).
3. Safety behavior at startup (respect THR_NOTZERO, send 0 before any
   motion, handle the 5-second arming window).
4. Responsiveness (stick change → motion happens in a reasonable time).
5. Closed-loop accuracy (target RPM matches actual RPM under load).
6. Efficiency (not wasting throttle in the dead zone).

Number 1 wins against any conflict with 2–6. Number 2 wins against 3–6.
And so on.

## How mistakes have happened

**2026-04-20, V7.0 first draft:** I (the assistant) added "dead-zone
skip" logic that snapped throttle values 1–60 to 0, thinking it was
avoiding a range the motor ignores. That introduced exactly the kind of
sudden jump this mission forbids — when the user moved the stick, the
throttle went from 0 to 60+ in one cycle. Amine called this out
immediately. The lesson: **do not optimize the output layer. Keep it
stupid and smooth.**

## How to use this document

Before designing or reviewing any throttle-output change:

1. Does this produce a throttle stream that is continuous and
   slew-limited, for any input?
2. Does it respect the priority order above?
3. Would this still work with a different ESC or motor?

If any answer is no, redesign before writing the code.

The feedback memory at
`~/.claude/projects/C--Users-ameen-OneDrive-Arduino-DiggerSetup/memory/feedback_smoothness_mission.md`
points back here. `CLAUDE.md` also references this file.
