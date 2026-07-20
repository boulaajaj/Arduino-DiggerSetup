# Configuration authority — which values live where

Every number has exactly one home. The full hierarchy of who wins what:
[authority-matrix](authority-matrix.md); this page maps the value KINDS.

- **Domain and application tunables** — per-domain headers in the
  production sketch's `src/config/` (input shaping, drive caps, battery and
  thermal thresholds, alert timings, safety windows, Wi-Fi identity, pins,
  and the firmware version single source of truth). Values are law: docs
  describe behavior and never carry a live copy.
- **Adapter-owned tunables** — deliberately NOT in `src/config/`: a value
  that only tunes one adapter's machinery stays single-homed with that
  adapter (the X.BUS poll cadence constants live with the X.BUS poller, the
  Wi-Fi serving knobs with the network stack). Moving them out would create
  a second home without a second reader.
- **Dashboard branding** — display name, slogan, badge, accent colors are a
  SKIN, not platform config; they belong in one configuration block in the
  dashboard source (white-label rule; tracked as its own issue).
- **Operator-facing numbers** — the operator guide and project plan may
  quote values for humans, but they must MATCH `src/config/`; drift is a
  doc bug on the doc's side.
- **Historical numbers** — dated records (decision log, upload log) keep
  the values that were true at the time, forever; history is never
  retro-edited.

Changing any tunable is a behavior change: own issue, operator sign-off,
expectations updated in the same PR ([agent-governance](agent-governance.md),
[testing](testing.md)).
