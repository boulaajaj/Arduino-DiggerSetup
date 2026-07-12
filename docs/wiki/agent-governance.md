# Agent governance

How AI agents (Claude Code, Copilot, CodeRabbit) are steered in this repo —
the Karpathy method (#53) plus the behavior-preservation covenant
(epic #116).

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
- **Review bots** — `.coderabbit.yaml` (epic-scoped covenant steering) and
  [copilot-instructions](../../.github/copilot-instructions.md) mirror the
  same rules so bot reviews align with the program
- **This wiki** is itself governed here: agents maintain it, PRs that change
  docs update affected notes ([README](README.md)); a session hook
  (`scripts/documentation_sync_hook.py`, #156) prompts the agent to re-check
  wiki + architecture docs whenever production code changes

The [testing](testing.md) suites are the enforcement arm: expectations are
law, and the gate defines "done" for any agent task.
