---
parent: alerts-and-alarms.md
sources:
  - sketches/dual_track_control/src/application/AlertControl.cpp
  - sketches/dual_track_control/src/alerts/AlertPolicy.cpp
  - sketches/dual_track_control/src/alerts/PatternPlayer.cpp
  - sketches/dual_track_control/src/config/AlertConfig.h
  - dashboard/index.html
---

# Notification channels

The same incident may produce several signals. The Arduino piezo plays
manual horn, one-shot and repeating patterns. The ESCs may beep when the
[output gate](output-gate.md) detaches PWM. The
[dashboard](dashboard-display.md) displays safety banners and reports
connection problems independently.

The repeating alarm order is thermal cutoff, low battery, thermal warning,
then RC off. A battery cutoff requests the low-battery pattern immediately.
Thermal recovery and Wi-Fi readiness each have a one-shot pattern.
The horn, one-shot pattern and active repeating alarm share a piezo output.

- [Operator beep meanings](../../OPERATOR-GUIDE.md#beep-meanings-the-d8-buzzer)
- [Battery ladder](battery-ladder.md) and [thermal derating](thermal-derating.md)
