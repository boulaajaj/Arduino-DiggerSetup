# ADR-0001: Domain-oriented firmware architecture with enforced boundaries

**Status:** Accepted · 2026-07-05 · epic #116, issue #128

## Context

The production firmware is a single 1606-line `.ino` organized by `[MODULE]`
comment sections. It works and is well-documented, but:

- Comment sections are not boundaries — any function can read or write any of
  ~40 file-scope globals (e.g. the `[SAFETY]` module writes the `[ALERT]`
  module's `lowVoltLatched`; `curvatureDrive()` reads the `currentGear` global
  behind its parameter list).
- None of the control math can compile off-target, so there are no host tests
  on a safety-relevant propulsion path (issue #47 is blocked on this).
- The repo's own rule ("split at ~500 lines") and review guidance ("flag
  global state mutation") cannot be followed inside one `.ino`.
- The repository doubles as a portfolio demonstrating enterprise engineering
  practice; the architecture itself is a deliverable.

A real incident motivates the boundary discipline: a blocking ~33 KB Wi-Fi
page send froze `loop()` while the ESCs held the last throttle (#69). The fix
(incremental serving + watchdog) worked, but nothing structural prevents the
next accidental coupling between serving and control.

## Decision

Adopt the layered, domain-oriented architecture specified in
[`ARCHITECTURE-TARGET.md`](../ARCHITECTURE-TARGET.md):

1. **Layers with one-way dependencies:** `.ino` → `application/` →
   `domain/` + `ports/`; `infrastructure/` implements ports; observers read an
   immutable `SystemSnapshot`.
2. **Pure domain:** no Arduino/hardware includes, no namespace-scope mutable
   state, time passed as a parameter — every domain rule host-testable.
3. **Ports & adapters** with link-time substitution preferred over virtual
   interfaces (zero runtime cost on the 48 MHz RA4M1).
4. **One owner per piece of state;** cross-domain communication through
   returned decision/status types, never sideways writes.
5. **File policy:** ≤150 lines soft / 250 hard, allowlisted exceptions;
   generic names (`Utils`, `Helpers`, `Manager`, …) banned.
6. **CI-enforced:** architecture fitness functions (#129) fail the build on
   forbidden includes, layer violations, oversized files, banned names,
   mutable domain globals, and generated-file drift.

Migration is behavior-preserving, smallest-pure-logic-first (#117), protected
by characterization tests (#47) and safety invariants (#131), with a local
`arduino-cli` compile before every push. While no hardware is available,
physical checks accumulate in `BENCH-VERIFICATION-DEFERRED.md` for one bench
pass later.

## Consequences

**Positive:** propulsion math testable on a laptop with deterministic time;
state ownership explicit; the #69 class of coupling becomes a CI failure, not
a field discovery; new domains (e.g. future attachments) get an obvious home;
the repo demonstrates architecture governance, not just architecture diagrams.

**Negative / accepted costs:** more files and folders than a hobby sketch
needs (deliberate — showcase is a stated goal); some indirection when tracing
a value from stick to ESC (mitigated by the control-flow diagram and naming);
migration effort spread over ~11 PRs; contributors must learn the dependency
rules (enforced by CI, documented in CONTRIBUTING.md, #126).

**Rejected alternatives:**

- *Flat `.h/.cpp` pairs per `[MODULE]`* (original #117): reproduces the same
  coupling with more files; no host-testability guarantee.
- *Virtual-interface hexagonal everywhere:* unnecessary runtime indirection;
  link-time seams give the same inversion for free.
- *Full RTOS / event-driven rewrite:* out of scope; the superloop is simple,
  proven, and appropriate at ~20 kHz.
