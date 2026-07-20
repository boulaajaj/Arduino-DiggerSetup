---
sources: none
---

# Authority matrix — which artifact wins for what

"Canonical" gets used loosely across this repo. This page states the
hierarchy once; every other document points here instead of restating it.
When two sources disagree about a row's question, the artifact named in
that row wins — and the disagreement itself is a doc bug to file.

| Question | The authority | Where |
| --- | --- | --- |
| What does the firmware DO? | The code, proven by the host suites — test expectations are behavior law | [dual_track_control](../../sketches/dual_track_control/dual_track_control.ino) + [testing](testing.md) |
| What are the current tunable values? | The config layer — no doc carries a live copy | `sketches/dual_track_control/src/config/` (inventory: [FILE-MAP](../architecture/FILE-MAP.md)) |
| What structure must code follow? | The architecture spec + accepted ADRs (rules files summarize; the spec wins) | [ARCHITECTURE-TARGET](../architecture/ARCHITECTURE-TARGET.md) + [ADR-0001](../architecture/adr/0001-domain-oriented-firmware-architecture.md) |
| What must NEVER change without sign-off? | The safety-invariant registry, each invariant tied to an executable test | [SAFETY](../SAFETY.md) + [safety-system](safety-system.md) |
| How is the machine operated? | The operator guide (numbers there must match the config layer — drift is a doc bug) | [OPERATOR-GUIDE](../../OPERATOR-GUIDE.md) |
| Why is it this way? | The dated decision log — history is immutable, entries are never retro-edited | [DECISION-LOG](../DECISION-LOG.md) |
| Where do I start reading? | This wiki — navigation and synthesis only, never a second copy | [home](home.md) |
| How do agents work here? | Project memory + path-scoped rules + skills, in that order of generality | [CLAUDE.md](../../CLAUDE.md) + [agent-governance](agent-governance.md) |

Two standing corollaries:

- **A changed test expectation is a behavior change**, not a refactor — it
  needs its own issue and operator sign-off before any code is written
  ([agent-governance](agent-governance.md)).
- **Docs describe behavior, never live numbers.** Anything quoting a
  tunable outside the config layer is either a dated historical record
  (allowed) or drift (a bug).
