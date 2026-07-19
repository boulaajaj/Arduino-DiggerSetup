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
conservatism: when the COMMAND TEXT cannot be tokenized even as a whole
(unclosed quotes), a fail-close token check applies over every line naming
git+commit (all tokens together when no single line does); a payload that
is not valid JSON at all is passed through (nothing to inspect).
"""

import json
import re
import shlex
import subprocess
import sys

# shlex punctuation characters — any token made solely of these is a command
# separator (covers &&, ||, ;, |, &, and subshell parens in any run length)
# EXCEPT redirection operators (>, <, >>, 2>&1 …), which stay part of the
# same simple command.
PUNCTUATION_CHARACTERS = set("();<>|&")


def is_redirection(token):
    return (bool(token) and set(token) <= set("<>&")
            and any(character in "<>" for character in token))


def is_separator(token):
    return (bool(token) and set(token) <= PUNCTUATION_CHARACTERS
            and not is_redirection(token))


def tokenize(text):
    """shlex tokens with operators split out even when glued to words.

    punctuation_chars makes `true;git commit -n` tokenize as
    ['true', ';', 'git', ...] instead of hiding the invocation inside a
    'true;git' word. Raises ValueError on unclosed quotes (caller decides).
    """
    lexer = shlex.shlex(text, posix=True, punctuation_chars=True)
    lexer.whitespace_split = True
    return list(lexer)


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
        if is_separator(token):
            commands.append(current)
            current = []
        else:
            current.append(token)
    commands.append(current)
    return [command for command in commands if command]


REPO_TARGET_OPTIONS = {"-C", "--git-dir", "--work-tree"}

# Commands that EXECUTE what follows them — `git` after one of these is a
# real invocation; `git` after anything else (echo, grep, ...) is data.
# The per-wrapper sets are the SHORT options that consume a value (so
# `sudo -u user git` skips `user`, while valueless `env -i` does NOT eat
# the wrapped command). Long options are self-contained or =-joined.
WRAPPER_COMMANDS = {
    "env": {"-u", "-C", "-S"},
    "sudo": {"-u", "-g", "-h", "-p", "-U", "-R", "-T", "-C", "-D"},
    "command": set(),
    "nice": {"-n"},
    "nohup": set(),
    "time": set(),
    "stdbuf": {"-i", "-o", "-e"},
    "timeout": {"-k", "-s"},
    "xargs": {"-a", "-d", "-E", "-e", "-I", "-i", "-L", "-l", "-n", "-P",
              "-s"},
}


def base_name(token):
    """Command basename: path prefixes and .exe stripped (/usr/bin/git)."""
    base = token.replace("\\", "/").rsplit("/", 1)[-1]
    return base[:-4] if base.endswith(".exe") else base
# Shells whose -c argument is a NESTED command string to parse recursively.
SHELL_COMMANDS = {"sh", "bash", "zsh", "dash", "ksh"}
MAXIMUM_NESTING = 5
ASSIGNMENT_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=")


DURATION_PATTERN = re.compile(r"[0-9.]+[smhd]?$")


def command_position_index(tokens, target_names):
    """Index of a target command when it sits in command position, else None.

    Leading environment assignments (`FOO=1`) and executing wrappers
    (`env`, `sudo`, ...) keep the position open, including a wrapper's
    option/value pairs (`sudo -u user git`) and duration arguments
    (`timeout 30 git`). The first other bare token is the wrapped command
    itself — so `env echo git commit -n` is echo's data, not an invocation.
    """
    current_wrapper = None
    previous_was_option = False
    for index, token in enumerate(tokens):
        name = base_name(token)
        if name in target_names:
            if (previous_was_option
                    and any(base_name(later) in target_names
                            for later in tokens[index + 1:])):
                # Ambiguous: this occurrence is a wrapper option's VALUE
                # (env -u git git commit …) — a later occurrence is the
                # command. Without a later one, this IS the command.
                previous_was_option = False
                continue
            return index
        if ASSIGNMENT_PATTERN.match(token):
            previous_was_option = False
            continue
        if name in WRAPPER_COMMANDS:
            current_wrapper = name
            previous_was_option = False
            continue
        if current_wrapper is not None and token.startswith("-"):
            # Only options KNOWN to take a value consume the next token —
            # valueless flags (env -i) must not eat the wrapped command.
            previous_was_option = token in WRAPPER_COMMANDS[current_wrapper]
            continue
        if current_wrapper is not None and previous_was_option:
            previous_was_option = False
            continue  # the option's value, e.g. sudo -u USER
        if current_wrapper is not None and DURATION_PATTERN.match(token):
            continue  # positional duration, e.g. timeout 30 git ...
        return None
    return None


def command_position_git_index(tokens):
    return command_position_index(tokens, {"git"})


def commit_invocation_from(tokens, start_index):
    """The `git commit` invocation at start_index, if that is what it is.

    Walks git's global options to find the subcommand; returns
    {"arguments": [...], "repo_options": [...]} when the subcommand is
    `commit` (repo_options carries the -C/--git-dir/--work-tree pairs so the
    hooksPath check can target the same repository), else None.
    """
    index = start_index + 1
    repo_options = []
    while index < len(tokens) and not is_separator(tokens[index]):
        token = tokens[index]
        if is_redirection(token):
            index += 2  # the operator and its target
            continue
        if (token.isdigit() and index + 1 < len(tokens)
                and is_redirection(tokens[index + 1])):
            index += 1  # file-descriptor prefix split off by the lexer (2>…)
            continue
        if token in GIT_VALUE_OPTIONS:
            if token in REPO_TARGET_OPTIONS and index + 1 < len(tokens):
                repo_options.extend(tokens[index:index + 2])
            index += 2
            continue
        if token.startswith(("--git-dir=", "--work-tree=")):
            repo_options.append(token)  # equals form targets the repo too
            index += 1
            continue
        if token.startswith("-"):
            index += 1
            continue
        if token == "commit":
            arguments = []
            for argument in tokens[index + 1:]:
                if is_separator(argument):
                    break
                arguments.append(argument)
            return {"arguments": arguments, "repo_options": repo_options}
        return None  # some other git subcommand
    return None


def fallback_invocations(text):
    """Whitespace-token check for text that cannot be tokenized (fail-close).

    Every line naming both `git` and `commit` is treated as a potential
    invocation, so an unclosed quote cannot smuggle a bypass onto a later
    line. If no single line names both, all tokens are checked together.
    """
    lines = [line.split() for line in text.splitlines()]
    suspicious = [tokens for tokens in lines
                  if "git" in tokens and "commit" in tokens]
    if not suspicious:
        suspicious = [text.split()]
    return [{"arguments": tokens, "repo_options": []}
            for tokens in suspicious]


def nested_shell_invocations(tokens, depth):
    """Invocations inside a shell -c string (`sh -c 'git commit -n'`).

    The shell itself must sit in command position — `echo sh -c '...'` is
    echo's data, not an executed shell.
    """
    index = command_position_index(tokens, SHELL_COMMANDS)
    if index is None:
        return []
    for argument_index in range(index + 1, len(tokens) - 1):
        argument = tokens[argument_index]
        # -c may be bundled with other shell shorts (-lc, -ec).
        if (argument.startswith("-") and not argument.startswith("--")
                and "c" in argument):
            nested_text = tokens[argument_index + 1]
            if depth + 1 >= MAXIMUM_NESTING:
                # Fail CLOSE at the depth cap: adversarially deep nesting
                # gets the conservative token check, never a free pass.
                return fallback_invocations(nested_text)
            nested = commit_invocations(nested_text, depth + 1)
            if nested is None:
                if "git" in nested_text and "commit" in nested_text:
                    return fallback_invocations(nested_text)
                return []
            return nested
    return []


def commit_invocations(command_text, depth=0):
    """Every real `git commit` invocation in the command, or None.

    Attempt A parses line by line (newlines separate commands, so a
    multi-line `git commit -n` is caught). When a quoted string legally
    spans lines — e.g. a heredoc commit message inside "$(...)" — attempt B
    parses the whole text at once, where such strings collapse to single
    tokens and prose inside them can never look like arguments.
    """
    # Shell line continuations join lines: `git commit \` + newline + `-n`
    # is ONE invocation. Applied before line splitting so attempt A sees it.
    command_text = command_text.replace("\\\n", " ")
    try:
        commands = []
        for line in command_text.splitlines():
            commands.extend(split_on_separators(tokenize(line)))
    except ValueError:
        try:
            commands = split_on_separators(tokenize(command_text))
        except ValueError:
            return None  # malformed even as a whole — caller falls back

    invocations = []
    for tokens in commands:
        index = command_position_git_index(tokens)
        if index is not None:
            invocation = commit_invocation_from(tokens, index)
            if invocation is not None:
                invocations.append(invocation)
        invocations.extend(nested_shell_invocations(tokens, depth))
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
        # Malformed input even as whole-text: fail-close token check over
        # every line naming git+commit; the hooksPath check still applies.
        invocations = fallback_invocations(command_text)

    if not invocations:
        return 0  # "git ... commit" appeared only as prose/quoted text

    for invocation in invocations:
        if bypasses_verification(invocation["arguments"]):
            block("Blocked (#47 gate): --no-verify / -n is not available "
                  "to agents.",
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
