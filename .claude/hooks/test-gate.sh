#!/bin/sh
# Claude Code PreToolUse[Bash] gate (#47). Receives the tool call as JSON on
# stdin; exit 2 blocks the call and shows stderr to the agent.
#
# The real test gate is .githooks/pre-commit (runs on EVERY commit, agent or
# human). This hook makes it agent-proof:
#   1. an agent cannot commit with hooks bypassed (--no-verify / -n), and
#   2. an agent cannot commit in a clone where the gate was never activated
#      (core.hooksPath unset) — it is told to activate it instead.

input=$(cat)

case "$input" in
  *"git commit"*) ;;
  *) exit 0 ;;                       # not a commit — stay out of the way
esac

case "$input" in
  *--no-verify*|*" -n "*)
    echo "Blocked (#47 gate): --no-verify is not available to agents." >&2
    echo "Fix the failing tests (or the missing test update) instead." >&2
    exit 2
    ;;
esac

hooks_path=$(git config core.hooksPath 2>/dev/null)
if [ "$hooks_path" != ".githooks" ]; then
  echo "Blocked (#47 gate): the commit-time test gate is not active in this clone." >&2
  echo "Run once:  git config core.hooksPath .githooks   — then retry the commit."  >&2
  exit 2
fi

exit 0
