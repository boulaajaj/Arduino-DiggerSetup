# Contributing

The human mirror of the agent-facing rules (`CLAUDE.md`, `.claude/rules/`,
`.claude/skills/`). Same law, different audience. When this page disagrees with the law
layer, the law wins — the path-scoped rules, and above them the canonical
spec (`docs/architecture/ARCHITECTURE-TARGET.md`); the full hierarchy is
[docs/wiki/authority-matrix.md](docs/wiki/authority-matrix.md). File the
discrepancy either way.

This firmware drives a real machine with a child rider. The bar for every
change is: **the machine behaves exactly as reviewed, and every behavior
change is deliberate, discussed, and test-locked.**

## Real-time rules (the control loop)

- No blocking calls in `loop()`: no `delay()`, no `pulseIn()`, no unbounded
  `while`. Non-blocking state machines only.
- `constrain()` every value before `writeMicroseconds()`.
- Every independent input path gets its own timeout that returns to
  neutral on loss.
- `micros()` is read fresh at the point of use.
- The watchdog is refreshed exactly once per pass, after the control path —
  never add a second refresh point.
- Float literals carry `f` (`1.0f`) — `double` is software-emulated on the
  RA4M1.

## Where things go

- Architecture layers and the dependency direction:
  [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) (narrative) and
  `.claude/rules/architecture.md` (law). One file = one concept, 150-line
  soft / 250-line hard limit, full-word names.
- Every tunable lives in `sketches/dual_track_control/src/config/`
  (adapter-local knobs stay with their adapter). Docs describe behavior,
  never live numbers.
- Tests mirror the source tree under `tests/` and compile the REAL
  firmware — see [docs/TESTING.md](docs/TESTING.md).

## Workflow

1. **Issue first.** Every change attaches to an open GitHub issue; one PR
   per issue.
2. **Draft PR immediately** on branch `agent/<role>/<description>` (humans:
   any clear branch name) with `Closes #N`, a summary, and a test plan.
   Active work must be visible on the project board.
3. **The gate**, before every push:
   - host suites green: `wsl -e make -C tests run` (Linux/macOS: plain
     `make -C tests run`)
   - firmware compiles:
     `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/dual_track_control`
   - the commit hook is active: `git config core.hooksPath .githooks`
     (it hard-blocks red tests and firmware changes without test changes)
4. **A changed test expectation is a behavior change** — it needs its own
   issue and operator sign-off BEFORE code. Characterization expectations
   are law, including float32 rounding points and hysteresis holds.
5. **Every firmware flash gets a row** in
   [docs/FIRMWARE-UPLOAD-LOG.md](docs/FIRMWARE-UPLOAD-LOG.md) immediately —
   an unlogged flash is treated as not done.
6. Commit messages: `V{major}.{minor}: {imperative verb} {what changed}`
   for firmware; plain imperative for tooling/docs.

## Documentation travels with code

A PR touching a mapped source area must update the pages that describe it
or carry an explicit documentation receipt — CI enforces this via
`docs/architecture/change-impact.json`. The wiki (`docs/wiki/`) is a
navigation layer: notes declare their sources and never restate tunable
values.

## Running the CI checks locally

```sh
python scripts/check_architecture.py     # layer/size/naming fitness
python scripts/check_wiki.py             # wiki graph + sources health
python scripts/check_change_impact.py    # doc-impact (vs origin/main)
python scripts/check_hook_registration.py
python scripts/generate_web_page.py --check
make -C tests run
```

The dashboard is generated: edit `dashboard/index.html`, never
`web_page.h`.
