---
sources:
  - sketches/dual_track_control/src/domain/drive/CommandMixer.cpp
  - OPERATOR-GUIDE.md
---

# Override modes

Who is driving. A three-position switch on the RC transmitter selects the
authority model: RC only, RC-overrides-joystick, or a blended mode where both
inputs mix. This is how the supervisor (RC) can take over from the rider
(joystick) at any moment — the core supervision mechanism for a child-ridden
machine.

- Current mode mapping and blend behavior: [CLAUDE.md](../../CLAUDE.md)
  (canonical); operator-facing description:
  [OPERATOR-GUIDE](../../OPERATOR-GUIDE.md)
- RC loss in any mode neutralizes RC's contribution via per-channel failsafe
  ([sbus-input](sbus-input.md)); the stale-RC invariant is tested across all
  modes ([SAFETY](../SAFETY.md) · [testing](testing.md))
- Mode selection feeds the [control pipeline](control-pipeline.md) after
  mixing, before [gear caps](gear-and-reverse-caps.md)

Safety-relevant detail: the [battery cutoff](battery-ladder.md) and
[output gate](output-gate.md) dominate **every** mode — no authority setting
can drive through a closed gate.
