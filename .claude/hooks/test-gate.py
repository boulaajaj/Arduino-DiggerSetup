#!/usr/bin/env python3
"""Claude Code PreToolUse[Bash] gate (#47, parsing rewrite #206).

Reads the tool call as JSON on stdin; exit 2 blocks the call and shows
stderr to the agent. The real test gate is .githooks/pre-commit (runs on
EVERY commit, agent or human). This hook makes it agent-proof:
  1. an agent cannot commit with hooks bypassed (--no-verify / -n), and
  2. an agent cannot commit in a clone where the gate was never activated
     (core.hooksPath unset) — it is told to activate it instead.

Unlike the original shell-glob version (#206 false positives), this parses
the actual command: only arguments belonging to a real `git commit`
invocation are inspected, so `grep -n` in a compound command or prose
inside a quoted issue/PR body never triggers the block. Residual
conservatism: on unparseable input, plain substring checks apply.
"""

import json
import shlex
import subprocess
import sys

SEPARATORS = {"&&", "||", ";", "|", "&"}
GIT_VALUE_OPTIONS = {"-C", "-c", "--git-dir", "--work-tree", "--namespace",
                     "--exec-path"}


def block(*lines):
    for line in lines:
        print(line, file=sys.stderr)
    sys.exit(2)


def split_on_separators(tokens):
    commands, current = [], []
    for token in tokens:
        if token in SEPARATORS:
            commands.append(current)
            current = []
        else:
            current.append(token)
    commands.append(current)
    return [command for command in commands if command]


def commit_arguments_from(tokens, start_index):
    """Arguments of a `git ...` invocation at start_index, if it is a commit.

    Walks git's global options to find the subcommand; returns the argument
    list (up to the next separator) when the subcommand is `commit`, else
    None.
    """
    index = start_index + 1
    while index < len(tokens) and tokens[index] not in SEPARATORS:
        token = tokens[index]
        if token in GIT_VALUE_OPTIONS:
            index += 2
            continue
        if token.startswith("-"):
            index += 1
            continue
        if token == "commit":
            arguments = []
            for argument in tokens[index + 1:]:
                if argument in SEPARATORS:
                    break
                arguments.append(argument)
            return arguments
        return None  # some other git subcommand
    return None


def commit_argument_lists(command_text):
    """Argument lists of every real `git commit` invocation, or None.

    Attempt A parses line by line (newlines separate commands, so a
    multi-line `git commit -n` is caught). When a quoted string legally
    spans lines — e.g. a heredoc commit message inside "$(...)" — attempt B
    parses the whole text at once, where such strings collapse to single
    tokens and prose inside them can never look like arguments.
    """
    lines_parse = []
    try:
        for line in command_text.splitlines():
            lines_parse.extend(split_on_separators(shlex.split(line)))
    except ValueError:
        try:
            tokens = shlex.split(command_text)
        except ValueError:
            return None  # malformed even as a whole — caller falls back
        argument_lists = []
        for index, token in enumerate(tokens):
            if token == "git":
                arguments = commit_arguments_from(tokens, index)
                if arguments is not None:
                    argument_lists.append(arguments)
        return argument_lists

    argument_lists = []
    for tokens in lines_parse:
        if tokens[0] == "git":
            arguments = commit_arguments_from(tokens, 0)
            if arguments is not None:
                argument_lists.append(arguments)
    return argument_lists


def main():
    try:
        payload = json.load(sys.stdin)
    except (ValueError, UnicodeDecodeError):
        return 0
    command_text = (payload.get("tool_input") or {}).get("command", "")
    # Cheap pre-filter only — the parser below decides what a commit IS
    # (catches `git -C path commit` that the old "git commit" glob missed).
    if "git" not in command_text or "commit" not in command_text:
        return 0

    commit_arguments = commit_argument_lists(command_text)
    if commit_arguments is None:
        # Malformed input even as whole-text: substring check on the FIRST
        # line only (real invocations start there; heredoc prose does not),
        # then the hooksPath check below still applies.
        first_line = command_text.splitlines()[0] if command_text else ""
        commit_arguments = [first_line.split()]

    if not commit_arguments:
        return 0  # "git ... commit" appeared only as prose/quoted text

    for arguments in commit_arguments:
        if "--no-verify" in arguments or "-n" in arguments:
            block("Blocked (#47 gate): --no-verify is not available to agents.",
                  "Fix the failing tests (or the missing test update) instead.")

    hooks_path = subprocess.run(
        ["git", "config", "core.hooksPath"],
        capture_output=True, text=True).stdout.strip()
    if hooks_path != ".githooks":
        block("Blocked (#47 gate): the commit-time test gate is not active "
              "in this clone.",
              "Run once:  git config core.hooksPath .githooks   — then retry "
              "the commit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
