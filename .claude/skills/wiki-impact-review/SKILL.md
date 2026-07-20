---
name: wiki-impact-review
description: Documentation-impact walk for a diff — use before finishing any PR that changes production code, when the documentation-sync hook fires, or when asked whether docs still match the code. Produces the documentation receipt for the PR body.
---

# Wiki-impact review — which knowledge pages must be reconsidered

## 1. Map the diff to knowledge pages

The machine-readable mapping is `docs/architecture/change-impact.json`
(#199) — CI enforces it (change-impact workflow), so start there. The table
below is the human-oriented summary; when the two disagree, the manifest
wins and the disagreement is a bug. `src/...` entries below are relative to
`sketches/dual_track_control/`:

| Changed area | Reconsider |
| --- | --- |
| `src/domain/drive/`, OperatorInput, MotorOutput | wiki control-pipeline notes, `OPERATOR-GUIDE.md` |
| `src/domain/battery/`, `src/domain/thermal/`, `src/domain/safety/`, SafetyControl | wiki safety notes, `docs/SAFETY.md` |
| `src/infrastructure/network/`, `dashboard/` | wiki telemetry/dashboard notes |
| `src/infrastructure/xc/`, `src/infrastructure/radiolink/`, `src/telemetry/` | wiki telemetry notes, `docs/XBUS-PROTOCOL.md`, `docs/WIRING-GUIDE-V8.md` |
| `src/ports/`, layer moves, new files | wiki architecture notes, `docs/architecture/ARCHITECTURE-TARGET.md`, `docs/architecture/FILE-MAP.md` |
| `.claude/`, `.github/`, `scripts/`, hooks | wiki agent-governance note, `docs/TESTING.md` |
| `src/config/` value changes | `OPERATOR-GUIDE.md` + `PROJECT-PLAN.md` (numbers there must match) |

## 2. Verify, then update or attest

For every page mapped: read the claims that overlap the diff and either
update the page IN THE SAME PR (wiki same-PR rule, `docs/wiki/README.md`) or
record that it is still accurate. Wiki notes stay links-only — never copy a
tunable value into one (`wiki-lint` enforces this).

## 3. Emit the receipt (goes in the PR body or a PR comment)

```
Documentation receipt
- Areas changed: <list>
- Pages examined: <list>
- Updated: <pages, or "none">
- No-change reasons: <page: why it still holds>
- Unverifiable without hardware: <items, or "none">
```

A firmware-touching PR without an update or a receipt is incomplete — the
run of `scripts/check_wiki.py` must also be green before hand-off.
