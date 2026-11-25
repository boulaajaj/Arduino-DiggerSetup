---
parent: safety-system.md
sources:
  - sketches/dual_track_control/src/alerts/
  - OPERATOR-GUIDE.md
---

# Alerts and alarms

The piezo combines the manual horn, one-shot patterns, and the selected
repeating alarm. Signals include Wi-Fi readiness, thermal warning, cutoff
and recovery, RC inactivity, and a low-battery alarm fed by the
[battery ladder](battery-ladder.md). See [notification channels](notification-channels.md)
for priority, ESC beeps, and the dashboard’s independent warning display.

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
