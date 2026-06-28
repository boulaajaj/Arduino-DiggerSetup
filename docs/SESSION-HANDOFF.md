# Session Handoff — Digger issue tracking + firmware backlog

Run from a local Claude Code session (VS Code) on branch
`claude/digger-reverse-turning-tv6b1n`, where `gh` is available.

## 1. Kanban board (GitHub Project)
- Use the existing project **#1 "Arduino Digger Issue Tracker"** (owner
  `boulaajaj`). It already has the default Workflows enabled with auto-add on
  `label:firmware`.
- Ensure it's a **Board** view with **To Do / In Progress / Done** columns
  (Status field). Add the columns if missing.
- Add all tracked issues to the board (firmware ones auto-add; the rest need an
  explicit add):
```bash
REPO=boulaajaj/Arduino-DiggerSetup
for N in 53 72 86 87 88 89 61 66 67 73 81 82 83 84 9 37; do
  gh project item-add 1 --owner boulaajaj --url https://github.com/$REPO/issues/$N
done
```
- **Verify the auto-workflows** move items correctly: new `firmware` issue → To Do;
  issue closed → Done; (optional) item assigned / linked PR opened → In Progress.
  Confirm with: `gh project item-list 1 --owner boulaajaj`.

## 2. Milestones (keep it to 3; close the finished one)

Naming convention: short theme name (not version numbers), so milestones group
*initiatives* while labels handle *area*. Three total:

| Milestone | Description |
|---|---|
| **Firmware: drive & safety** | Control-loop work on the GL10 FOC controller: drive feel (smooth pivot↔drive, consistent reverse steering, turn-rate taper), throttle/reverse caps, and RC-loss failsafe + audible alarms. Open-loop, safety-first; no Wi-Fi→motor path. |
| **Dashboard & tooling** | Wi-Fi telemetry dashboard (SSE, PWA, fit/shrink, alarm mirror), the replay/mock harness, and host-side scripts (live_plot, monitor, capture). Monitoring only. |
| **Karpathy method adoption** | Turn the repo over to AI-agent guardrails/workflows (agent-skills, planning/task-breakdown, ADR discipline) so future work is consistent and reviewable. |

```bash
REPO=boulaajaj/Arduino-DiggerSetup
# Create the 3 milestones (title + description)
gh api repos/$REPO/milestones -f title="Firmware: drive & safety" -f state=open \
  -f description="GL10 FOC control-loop work: drive feel (smooth pivot<->drive, consistent reverse steering, turn-rate taper), throttle/reverse caps, RC-loss failsafe + audible alarms. Open-loop, safety-first."
gh api repos/$REPO/milestones -f title="Dashboard & tooling" -f state=open \
  -f description="Wi-Fi telemetry dashboard (SSE/PWA/fit/alarm mirror), replay/mock harness, and host-side scripts (live_plot, monitor, capture). Monitoring only."
gh api repos/$REPO/milestones -f title="Karpathy method adoption" -f state=open \
  -f description="Adopt AI-agent guardrails/workflows (agent-skills, planning/task-breakdown, ADR discipline) so future work stays consistent and reviewable."
# Close the finished migration milestone
gh api -X PATCH repos/$REPO/milestones/1 -f state=closed   # V7 — GL10 FOC migration (done)

# Assign issues -> milestones
gh issue edit 72 86 87 88 89 --repo $REPO --milestone "Firmware: drive & safety"
gh issue edit 61 66 67 73 81 82 83 84 9 37 --repo $REPO --milestone "Dashboard & tooling"
gh issue edit 53 --repo $REPO --milestone "Karpathy method adoption"
```

## 3. Labels (trim 16 → ~12)
```bash
REPO=boulaajaj/Arduino-DiggerSetup
gh issue edit 7 87 --repo $REPO --add-label firmware   # tuning is always firmware
gh label delete tuning --repo $REPO --yes
gh label delete process --repo $REPO --yes
gh label delete cleanup --repo $REPO --yes
gh label delete waiting --repo $REPO --yes             # status belongs to the board
```

## 4. Firmware backlog (this branch, one issue at a time)
1. **#87 — Restore throttle caps.** Eco `0.55→0.65`, Normal `0.70→0.80`, Boost
   `1.00`; reverse `REVERSE_LIMIT 0.50→0.65`. Fix stale labels in
   `rc_test.ino:32`, the `types.h` Gear enum, and the `CLAUDE.md` gear table.
   (Confirm "reverse 65%" interpretation + whether to keep `REVERSE_LIMIT_LOW`.)
2. **#88 — Cut ESC PWM on RC loss.** On `!sbusValid`, detach the ESC servos
   (stop pulses) instead of writing neutral 1500 µs, so the GL10s detect loss and
   beep; debounce re-attach on stable signal return. Preserve immediate
   control-disable.
3. **#72 / #86 — Steering.** Make curvature mix **additive** so stick-left pivots
   left in forward AND reverse at any speed (no car-like reverse flip), and add a
   **continuous turn-rate taper** that reduces steering authority as throttle
   rises (floor > 0, no dead spot).
4. **#89 — Idle-while-connected alarm.** RC valid but no stick/joystick activity:
   first beep at 3 min, stronger/repeating at 5 min; any movement resets.

Each: show inner/outer wheel-speed (and PWM) math traces before flashing; all
tunables in `[CONFIG]`; failsafe must reach the safe state immediately.

## 5. Optional — agent-skills (start of #53 Karpathy adoption)
Vendor into `.claude/skills/` from github.com/addyosmani/agent-skills:
`planning-and-task-breakdown`, `documentation-and-adrs`, `git-workflow-and-versioning`.

## Issue index
- #72 smooth pivot↔drive transition · #86 reverse consistency + turn taper ·
  #87 throttle caps · #88 cut PWM on RC loss · #89 idle alarm.
