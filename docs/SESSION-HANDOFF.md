# Session Handoff — GitHub admin + firmware backlog (2026-06-28)

Carries the web session's state to a **local VS Code Claude Code session**, where
`gh` works (the web session's egress proxy blocks GitHub GraphQL/Projects and
gates REST, so `gh` can't run there).

## State so far
- **Issues created:** #86 (reverse-steering consistency + turn taper),
  #87 (restore throttle caps 80/65/65), #88 (cut PWM on RC loss),
  #89 (idle-while-connected alarm). #72 pre-existing (smooth pivot↔drive).
- **Branch:** `claude/digger-reverse-turning-tv6b1n` (firmware work goes here).
- **Project:** #1 "Arduino Digger Issue Tracker" (owner boulaajaj), default
  Workflows enabled, auto-add on `label:firmware`.

## GitHub admin to run locally (`gh`)
```bash
REPO=boulaajaj/Arduino-DiggerSetup
# Milestones: create 3, close V7 (#1)
gh api repos/$REPO/milestones -f title="Firmware: drive & safety" -f state=open
gh api repos/$REPO/milestones -f title="Dashboard & tooling" -f state=open
gh api repos/$REPO/milestones -f title="Karpathy method adoption" -f state=open
gh api -X PATCH repos/$REPO/milestones/1 -f state=closed
# Assign issues -> milestones
gh issue edit 72 86 87 88 89 --repo $REPO --milestone "Firmware: drive & safety"
gh issue edit 61 66 67 73 81 82 83 84 9 37 --repo $REPO --milestone "Dashboard & tooling"
gh issue edit 53 --repo $REPO --milestone "Karpathy method adoption"
# Add issues to Project #1
for N in 53 72 86 87 88 89 61 66 67 73 81 82 83 84 9 37; do
  gh project item-add 1 --owner boulaajaj --url https://github.com/$REPO/issues/$N; done
# Labels: merge tuning->firmware, drop low-value
gh issue edit 7 87 --repo $REPO --add-label firmware
gh label delete tuning --repo $REPO --yes
gh label delete process --repo $REPO --yes
gh label delete cleanup --repo $REPO --yes
gh label delete waiting --repo $REPO --yes
```
Auth if needed: `gh auth login` (scopes `repo project`) or `gh auth refresh -s project`.

## Firmware backlog (recommended order, one at a time)
1. **#87** throttle caps (quick regression fix): Eco 0.55→0.65, Normal 0.70→0.80,
   reverse 0.50→0.65; fix stale comments in `rc_test.ino:32`, `types.h` enum, `CLAUDE.md`.
2. **#88** cut ESC PWM on RC loss (detach servos vs neutral) — safety.
3. **#72 / #86** steering: additive curvature (reverse consistency) + continuous turn taper.
4. **#89** idle-while-connected alarm (3 min / 5 min).

## Optional: agent-skills to vendor into `.claude/skills/`
`planning-and-task-breakdown`, `documentation-and-adrs`, `git-workflow-and-versioning`
(from github.com/addyosmani/agent-skills) — start of the #53 Karpathy-method adoption.
