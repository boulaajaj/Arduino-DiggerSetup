#!/usr/bin/env python3
"""Hook-registration check (#193) — proves every script in .claude/hooks/ is
actually wired into .claude/settings.json (a present-but-unregistered hook
script is dead code), then functionally exercises test-gate.py (#206): it
must block agent commits using --no-verify/-n in every shell spelling,
block commits where the #47 pre-commit gate is not active in the TARGET
repo, stay out of the way otherwise, and never false-positive on benign
flags, quoted prose, comments, or heredoc commit messages.
Run in CI (hooks-selftest.yml) and locally:
    python scripts/check_hook_registration.py
"""

import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS = os.path.join(REPO_ROOT, ".claude", "settings.json")
HOOKS_DIR = os.path.join(REPO_ROOT, ".claude", "hooks")
TEST_GATE = os.path.join(HOOKS_DIR, "test-gate.py")

DEEP_NESTING = "git commit -n"
for _ in range(6):
    DEEP_NESTING = "sh -c " + shlex.quote(DEEP_NESTING)

# (name, command, expected exit) — run BEFORE core.hooksPath is activated,
# so every real invocation blocks (2) and every non-invocation passes (0).
UNGATED_CASES = [
    ("non-commit command passes through", "ls -la", 0),
    ("git commit --no-verify is blocked", "git commit --no-verify -m x", 2),
    ("git commit -n is blocked", "git commit -n", 2),
    ("git -C <path> commit -n is blocked", "git -C /tmp commit -n", 2),
    ("bundled short options carrying -n are blocked (-anm)",
     "git commit -anm x", 2),
    ("abbreviated long option is blocked (--no-verif)",
     "git commit --no-verif -m x", 2),
    ("glued separator cannot hide the invocation (true;git)",
     "true;git commit -n", 2),
    ("glued hash is not a comment (true#;git)", "true#;git commit -n", 2),
    ("quoted hash literal is not a comment", 'echo "#"; git commit -n', 2),
    ("bypass flag before a trailing comment is blocked",
     "git commit -n # note", 2),
    ("environment-assignment prefix cannot hide the invocation",
     "FOO=1 git commit -n", 2),
    ("env wrapper cannot hide the invocation",
     "env git commit --no-verify", 2),
    ("wrapper with its own option-value cannot hide it",
     "sudo -u operator git commit -n", 2),
    ("wrapper option-value named git cannot hide it",
     "env -u git git commit -n", 2),
    ("git as a wrapper option's value is not an invocation",
     "env -u git commit -n", 0),
    ("long-form wrapper option with separate value is caught",
     "sudo --user operator git commit -n", 2),
    ("equals-joined long wrapper option is self-contained",
     "sudo --user=operator git commit -n", 2),
    ("valueless wrapper option before git is still caught",
     "env -i git commit -n", 2),
    ("valueless wrapper flag does not eat the wrapped command",
     "env -i echo git commit -n", 0),
    ("absolute-path git is still a commit invocation",
     "/usr/bin/git commit -n", 2),
    ("relative-path git is still a commit invocation", "./git commit -n", 2),
    ("git as data of another command is not an invocation",
     "echo git commit -n", 0),
    ("git as data stays data behind a wrapper (env echo git)",
     "env echo git commit -n", 0),
    ("git as data after a line-spanning quote stays data",
     'echo "first\nsecond" && echo git commit -n', 0),
    ("real bypass after a line-spanning quote is blocked",
     'echo "first\nsecond" && git commit -n', 2),
    ("duration-argument wrapper cannot hide it (timeout 30 git)",
     "timeout 30 git commit -n", 2),
    ("numeric token after other wrappers is a command, not a duration",
     "sudo 30 echo git commit -n", 0),
    ("redirection cannot hide the invocation (git 2>/dev/null)",
     "git 2>/dev/null commit -n", 2),
    ("leading redirection cannot hide the invocation",
     ">/dev/null git commit -n", 2),
    ("leading fd redirection cannot hide the invocation",
     "2>/dev/null git commit -n", 2),
    ("glued separator after the flag is still blocked",
     "git commit --no-verify&&git push", 2),
    ("commit is blocked while core.hooksPath is not .githooks",
     "git commit -m x", 2),
    ("prose in a quoted body is not a commit",
     'gh issue create --body "never use git commit --no-verify or -n here"',
     0),
    ("git log --grep commit -n 5 is not a commit",
     "git log --grep commit -n 5", 0),
]

# Run AFTER core.hooksPath is activated in the temp repo.
GATED_CASES = [
    ("commit passes once the #47 gate is active", "git commit -m x", 0),
    ("compound command with grep -n passes",
     'git commit -m fix && git push && gh pr create '
     '--body "checked with grep -n"', 0),
    ("short-option value containing n passes (-mnope)",
     "git commit -mnope", 0),
    ("a message literally named -n is a value, not a flag",
     "git commit -m -n", 0),
    ("long-option separate value named -n is a value",
     "git commit --message -n", 0),
    ("pathspec after -- is not an option (file named -n)",
     "git commit -am x -- -n", 0),
    ("trailing redirection on a clean commit passes",
     "git commit -m x > /dev/null 2>&1", 0),
    ("prose flags in a trailing comment do not block",
     "git commit -m x # never use -n here", 0),
    ("comment strips before the malformed part (prose -n inert)",
     'git commit -m x # prose -n and an "unclosed quote', 0),
    ("bypass flag still blocked inside a compound command",
     "git add . && git commit -n -m x && git push", 2),
    ("heredoc commit message mentioning flags passes",
     'git commit -m "$(cat <<\'EOF\'\n'
     "fixes the grep -n and --no-verify prose case\n"
     'EOF\n)"', 0),
    ("multi-line real bypass is still blocked", "cd .\ngit commit -n", 2),
    ("line-continuation bypass is blocked (backslash-newline)",
     "git commit \\\n-n", 2),
    ("unclosed quote cannot hide a later-line bypass",
     'echo "unclosed\ngit commit -n', 2),
    ("adversarially deep sh -c nesting fails CLOSED", DEEP_NESTING, 2),
    ("nested shell -c bypass is blocked", "sh -c 'git commit -n'", 2),
    ("bundled shell shorts (-lc) bypass is blocked",
     "bash -lc 'git commit --no-verify'", 2),
    ("nested shell with git as data passes",
     "sh -c 'echo git commit -n'", 0),
    ("shell -c as data of another command passes",
     "echo sh -c 'git commit -n'", 0),
]

# hooksPath is checked in the repo the commit TARGETS ({other} = a repo
# where the gate was never activated), even from an active clone.
TARGETED_REPO_CASES = [
    ("git -C into a gate-less repo is blocked even from an active clone",
     'git -C "{other}" commit -m x', 2),
    ("equals-form --git-dir targeting is honored",
     'git --git-dir="{other}/.git" commit -m x', 2),
    ("fallback keeps -C targeting on unparseable input",
     'git -C {other} commit -m x "unclosed', 2),
]


def isolated_environment(**overrides):
    environment = dict(os.environ)
    # A developer's global/system core.hooksPath must not leak into the
    # temp-repo assertions (the gate reads effective config).
    environment["GIT_CONFIG_GLOBAL"] = os.devnull
    environment["GIT_CONFIG_SYSTEM"] = os.devnull
    environment.update(overrides)
    return environment


def gate_exit_code_raw(raw_text, working_dir, environment=None):
    """Run test-gate.py with arbitrary raw stdin."""
    try:
        result = subprocess.run(
            [sys.executable, TEST_GATE], input=raw_text,
            capture_output=True, text=True, cwd=working_dir,
            env=environment or isolated_environment(), timeout=30)
    except subprocess.TimeoutExpired:
        return -1  # deterministic failure: matches neither 0 nor 2
    return result.returncode


def gate_exit_code(command, working_dir, environment=None):
    """Run test-gate.py with a PreToolUse-style JSON payload on stdin."""
    payload = json.dumps({"tool_input": {"command": command}})
    return gate_exit_code_raw(payload, working_dir, environment)


def main():
    failures = []
    checks = 0

    def check(name, condition):
        nonlocal checks
        checks += 1
        if not condition:
            failures.append(name)

    # 1. Registration — script present in .claude/hooks/ implies wired in
    #    .claude/settings.json (this is the regression #193 fixed).
    with open(SETTINGS, encoding="utf-8") as settings_file:
        settings = json.load(settings_file)
    commands = [hook.get("command", "")
                for event_entries in settings.get("hooks", {}).values()
                for entry in event_entries
                for hook in entry.get("hooks", [])
                if hook.get("type") == "command"]
    hook_scripts = [name for name in sorted(os.listdir(HOOKS_DIR))
                    if os.path.isfile(os.path.join(HOOKS_DIR, name))]
    check("at least one hook script exists", bool(hook_scripts))
    for script in hook_scripts:
        check(f"{script} is registered in .claude/settings.json",
              any(f".claude/hooks/{script}" in command
                  for command in commands))

    # 2. Behavior — exercise the gate in a temp repo where core.hooksPath is
    #    controlled (exit 2 = blocked, exit 0 = allowed through).
    if shutil.which("git") is None:
        print("check-hook-registration: git unavailable — "
              "functional checks skipped (registration checks still ran)")
    else:
        with tempfile.TemporaryDirectory() as temp_dir:
            subprocess.run(["git", "init", "-q", temp_dir], check=True)
            for name, command, expected in UNGATED_CASES:
                check(name, gate_exit_code(command, temp_dir) == expected)
            check("malformed payload with a bypass fails closed",
                  gate_exit_code_raw("git commit -n", temp_dir) == 2)
            check("malformed payload without a commit passes",
                  gate_exit_code_raw("just some text", temp_dir) == 0)
            check("non-string command value never crashes the gate",
                  gate_exit_code_raw('{"tool_input":{"command":null}}',
                                     temp_dir) == 0)
            check("missing git fails closed (empty PATH)",
                  gate_exit_code("git commit -m x", temp_dir,
                                 isolated_environment(PATH="")) == 2)
            subprocess.run(["git", "-C", temp_dir, "config",
                            "core.hooksPath", ".githooks"], check=True)
            for name, command, expected in GATED_CASES:
                check(name, gate_exit_code(command, temp_dir) == expected)
            with tempfile.TemporaryDirectory() as other_repo:
                subprocess.run(["git", "init", "-q", other_repo], check=True)
                for name, template, expected in TARGETED_REPO_CASES:
                    check(name,
                          gate_exit_code(template.format(other=other_repo),
                                         temp_dir) == expected)

    for name in failures:
        print(f"FAIL: {name}")
    print(f"check-hook-registration: {checks - len(failures)}/{checks} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
