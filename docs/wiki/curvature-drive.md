# Curvature drive

The tank-mixing algorithm: throttle plus steering in, one speed per track
out. At speed it behaves like a car (inner track slows, outer speeds up by
the same delta — average speed stays what the stick asked); near standstill
it blends smoothly into **pivot mode**, counter-rotating the tracks in place.
Feeding throttle mid-pivot arcs the machine out of the pivot instead of
surging it forward.

- Current constants, blend thresholds, and taper values:
  [CLAUDE.md](../../CLAUDE.md) "Architecture Summary" (canonical)
- Why this mix was chosen and the research behind it:
  [CONTROL-RESEARCH](../CONTROL-RESEARCH.md)
- Field-tuning history (blend band, pivot throttle taper, reverse-steer
  consistency): [DECISION-LOG](../DECISION-LOG.md)
- Its outputs are bounded by [gear and reverse caps](gear-and-reverse-caps.md)
  and gated by the [safety system](safety-system.md)
- Position in the chain: [control pipeline](control-pipeline.md)

The average-speed property matters for safety review: caps bound the AVERAGE
track command, and single-track excursions in turns are design headroom, not
violations ([SAFETY](../SAFETY.md)).
