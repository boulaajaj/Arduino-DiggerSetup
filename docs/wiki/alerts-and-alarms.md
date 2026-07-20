---
sources:
  - sketches/dual_track_control/src/alerts/
  - OPERATOR-GUIDE.md
---

# Alerts and alarms

Audible signals from a piezo beeper, driven non-blocking with a strict
priority: the horn overrides one-shot patterns, which override repeating
alarms. Today's set: a boot chirp when Wi-Fi is ready, a horn button on the
RC transmitter, an inactivity reminder ("unplug me"), and a latched
low-battery alarm fed by the [battery ladder](battery-ladder.md).

- Current pin, patterns, thresholds, and debounce:
  [CLAUDE.md](../../CLAUDE.md) (canonical); the beep table for operators:
  [OPERATOR-GUIDE](../../OPERATOR-GUIDE.md)
- **Sound-only by design**: alarms never slow or stop the motors — motion
  authority belongs exclusively to the [safety system](safety-system.md)
  ladders and the [output gate](output-gate.md). A low-battery motor cutoff
  exists separately in the [battery ladder](battery-ladder.md); the alarm is
  the operator-warning layer above it.
- History (a reverse beeper was removed, alerts returned as alarms):
  [DECISION-LOG](../DECISION-LOG.md)
