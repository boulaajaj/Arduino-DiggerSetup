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
# git commit short options that consume a value: in a short-option bundle the
# first of these swallows the rest of the cluster (so letters after it are a
# VALUE, not options — e.g. -mnope is -m "nope").
COMMIT_VALUE_SHORTS = "mcCFtS"
# Shortest unambiguous parse-options abbreviation of --no-verify among
# git-commit's --no-* long options (--no-edit, --no-gpg-sign, ...).
NO_VERIFY_PREFIX = "--no-v"


def bypasses_verification(arguments):
    """True when a commit argument list carries --no-verify in ANY spelling.

    Handles exact flags, parse-options long-option abbreviation
    (--no-v[erify]), and short-option bundling (-anm carries -n). Scanning
    stops at "--": everything after it is pathspecs.
    """
    for argument in arguments:
        if argument == "--":
            return False
        if argument.startswith("--"):
            if argument == "--no-verify" or (
                    argument.startswith(NO_VERIFY_PREFIX) and
                    "--no-verify".startswith(argument)):
                return True
            continue
        if argument.startswith("-") and len(argument) > 1:
            for letter in argument[1:]:
                if letter == "n":
                    return True
                if letter in COMMIT_VALUE_SHORTS:
                    break  # rest of the cluster is this option's value
    return False


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


REPO_TARGET_OPTIONS = {"-C", "--git-dir", "--work-tree"}


def commit_invocation_from(tokens, start_index):
    """The `git commit` invocation at start_index, if that is what it is.

    Walks git's global options to find the subcommand; returns
    {"arguments": [...], "repo_options": [...]} when the subcommand is
    `commit` (repo_options carries the -C/--git-dir/--work-tree pairs so the
    hooksPath check can target the same repository), else None.
    """
    index = start_index + 1
    repo_options = []
    while index < len(tokens) and tokens[index] not in SEPARATORS:
        token = tokens[index]
        if token in GIT_VALUE_OPTIONS:
            if token in REPO_TARGET_OPTIONS and index + 1 < len(tokens):
                repo_options.extend(tokens[index:index + 2])
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
            return {"arguments": arguments, "repo_options": repo_options}
        return None  # some other git subcommand
    return None


def commit_invocations(command_text):
    """Every real `git commit` invocation in the command, or None.

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
        invocations = []
        for index, token in enumerate(tokens):
            if token == "git":
                invocation = commit_invocation_from(tokens, index)
                if invocation is not None:
                    invocations.append(invocation)
        return invocations

    invocations = []
    for tokens in lines_parse:
        if tokens[0] == "git":
            invocation = commit_invocation_from(tokens, 0)
            if invocation is not None:
                invocations.append(invocation)
    return invocations


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

    invocations = commit_invocations(command_text)
    if invocations is None:
        # Malformed input even as whole-text: substring check on the FIRST
        # line only (real invocations start there; heredoc prose does not),
        # then the hooksPath check below still applies.
        first_line = command_text.splitlines()[0] if command_text else ""
        invocations = [{"arguments": first_line.split(), "repo_options": []}]

    if not invocations:
        return 0  # "git ... commit" appeared only as prose/quoted text

    for invocation in invocations:
        if bypasses_verification(invocation["arguments"]):
            block("Blocked (#47 gate): --no-verify is not available to agents.",
                  "Fix the failing tests (or the missing test update) instead.")

    for invocation in invocations:
        # Query the SAME repository the commit targets (git -C / --git-dir).
        hooks_path = subprocess.run(
            ["git"] + invocation["repo_options"] + ["config", "core.hooksPath"],
            capture_output=True, text=True).stdout.strip()
        if hooks_path != ".githooks":
            block("Blocked (#47 gate): the commit-time test gate is not "
                  "active in this clone.",
                  "Run once:  git config core.hooksPath .githooks   — then "
                  "retry the commit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
