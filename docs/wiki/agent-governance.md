---
sources:
  - CLAUDE.md
  - .claude/rules/
  - docs/AGENT-EXAMPLES.md
---

# Agent governance

How AI agents (Claude Code, Copilot, CodeRabbit) are steered in this repo —
the Karpathy method (#53) plus the behavior-preservation covenant
(permanent policy — born in epic #116, retained after its 2026-07-20 close).

- **Working Agreement** — four principles (think before coding, simplicity
  first, surgical changes, goal-driven execution with a verifiable gate):
  [CLAUDE.md](../../CLAUDE.md)
- **The covenant** — organization changes NOTHING observable; behavior fixes
  need their own issue BEFORE the fix, never ride another PR:
  [.claude/rules/behavior-preservation](../../.claude/rules/behavior-preservation.md)
- **Worked examples** — real good/bad task-execution pairs:
  [AGENT-EXAMPLES](../AGENT-EXAMPLES.md)
- **Path-scoped rules** —
  [architecture](../../.claude/rules/architecture.md) ·
  [naming](../../.claude/rules/naming.md) ·
  [state-ownership](../../.claude/rules/state-ownership.md) ·
  [firmware-realtime](../../.claude/rules/firmware-realtime.md) ·
  [dashboard](../../.claude/rules/dashboard.md) ·
  [workflow](../../.claude/rules/workflow.md)
- **Project skills** (#127) — repeatable procedures the agent runs on
  demand, one folder per skill under `.claude/skills/`: safety-review,
  flash-and-log, new-module, field-tune, wiki-impact-review,
  prepare-pull-request. Each skill owns its procedure; canonical docs and
  this wiki only point to it.
- **Reviewer subagents** (#127) — read-only specialist reviewers under
  `.claude/agents/` (architecture, safety, tests, documentation) that
  challenge a diff before it is marked ready — they report, never edit.
- **Review bots** — `.coderabbit.yaml` (permanent covenant steering) and
  [copilot-instructions](../../.github/copilot-instructions.md) mirror the
  same rules so bot reviews align with the program
- **This wiki** is itself governed here: agents maintain it, PRs that change
  docs update affected notes ([README](README.md)); a session hook
  (`scripts/documentation_sync_hook.py`, #156) prompts the agent to re-check
  wiki + architecture docs whenever production code changes, and CI
  enforces the deterministic half (#199): the change-impact manifest
  (`docs/architecture/change-impact.json`) maps source areas to knowledge
  pages, and a PR touching a mapped area must update an affected page or
  carry a documentation receipt. The judgment half (#200) is the
  semantic-doc-lint skill — a monthly deep pass over every note's declared
  sources (a scheduled workflow opens the reminder issue), with per-PR
  coverage from the documentation-reviewer agent in the pre-request
  self-review.

The [testing](testing.md) suites are the enforcement arm: expectations are
law, and the gate defines "done" for any agent task.
