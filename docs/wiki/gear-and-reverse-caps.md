# Gear and reverse caps

The speed-limiting layer between [curvature drive](curvature-drive.md) and
the [output gate](output-gate.md). A three-position RC switch selects a gear
(Eco / Normal / Boost); each gear caps the **average** track speed, leaving
the outer track headroom to hold speed through corners. Reverse gets its own,
lower cap per gear. In Eco, the pivot cap is boosted so the rider keeps
usable maneuvering authority.

- Current cap values, the pivot boost, and the joystick gain:
  [CLAUDE.md](../../CLAUDE.md) (canonical)
- The invariant and its test — no input combination exceeds the active caps
  on the average track command: [SAFETY](../SAFETY.md)
- Why reverse caps are true percentages (ESC recalibration history) and how
  the gear spread was tuned: [DECISION-LOG](../DECISION-LOG.md)
- The [battery ladder](battery-ladder.md) and
  [thermal derating](thermal-derating.md) can force the Eco cap regardless of
  the switch position

**Average, not per-track**: a single track legitimately exceeds the gear cap
mid-turn — that headroom is the design, not a bug. Reviewers and agents keep
tripping on this; it is locked by test.
