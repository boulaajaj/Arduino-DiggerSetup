---
sources:
  - docs/SAFETY.md
  - sketches/dual_track_control/src/domain/safety/
---

# Safety system

Everything that can slow, stop, or refuse a drive command. The canonical,
test-linked registry of what must ALWAYS hold is [SAFETY](../SAFETY.md) —
each invariant there has an executable host test ([testing](testing.md)).

Layers, from input to output:

- Per-channel RC failsafe and stale-input neutralization
  ([sbus-input](sbus-input.md))
- [Battery ladder](battery-ladder.md) — staged response to falling pack
  voltage, from forced-Eco to latched cutoff
- [Thermal derating](thermal-derating.md) — staged response to ESC/motor
  temperature, monotone by design
- Plausibility gates — implausible voltage/temperature readings are never
  trusted ([SAFETY](../SAFETY.md))
- [Output gate](output-gate.md) — the final authority between the mixer and
  the servo pins
- Loop watchdog — born from a real incident (Wi-Fi stall while PWM held the
  last throttle); the story is in [SAFETY](../SAFETY.md) and the fix in the
  [DECISION-LOG](../DECISION-LOG.md)
- [Alerts and alarms](alerts-and-alarms.md) — sound-only today, by design

Operator-facing behavior (what the machine does when a layer trips):
[OPERATOR-GUIDE](../../OPERATOR-GUIDE.md). Known, deliberately-unfixed gaps
are documented in [SAFETY](../SAFETY.md) "Known gaps" — fixing one requires
its own issue per the [governance rules](agent-governance.md).
