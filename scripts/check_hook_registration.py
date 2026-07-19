#!/usr/bin/env python3
"""Hook-registration check (#193) — proves every script in .claude/hooks/ is
actually wired into .claude/settings.json (a present-but-unregistered hook
script is dead code), then functionally exercises test-gate.py (#206): it
must block agent commits using --no-verify/-n, block commits in a clone
where the #47 pre-commit gate was never activated, stay out of the way
otherwise, and NOT false-positive on benign flags in compound commands or
on prose inside quoted arguments.
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


def registered_commands():
    """Every command string registered under any hook event."""
    with open(SETTINGS, encoding="utf-8") as settings_file:
        settings = json.load(settings_file)
    commands = []
    for event_entries in settings.get("hooks", {}).values():
        for entry in event_entries:
            for hook in entry.get("hooks", []):
                if hook.get("type") == "command":
                    commands.append(hook.get("command", ""))
    return commands


def gate_exit_code(command, working_dir):
    """Run test-gate.py with a PreToolUse-style JSON payload on stdin."""
    payload = json.dumps({"tool_input": {"command": command}})
    environment = dict(os.environ)
    # Isolate git config: a developer's global/system core.hooksPath must not
    # leak into the temp-repo assertions (the gate reads effective config).
    environment["GIT_CONFIG_GLOBAL"] = os.devnull
    environment["GIT_CONFIG_SYSTEM"] = os.devnull
    result = subprocess.run([sys.executable, TEST_GATE], input=payload,
                            capture_output=True, text=True,
                            cwd=working_dir, env=environment)
    return result.returncode


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
    commands = registered_commands()
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
            check("non-commit command passes through",
                  gate_exit_code("ls -la", temp_dir) == 0)
            check("git commit --no-verify is blocked",
                  gate_exit_code("git commit --no-verify -m x", temp_dir) == 2)
            check("git commit -n is blocked",
                  gate_exit_code("git commit -n", temp_dir) == 2)
            check("git -C <path> commit -n is blocked",
                  gate_exit_code("git -C /tmp commit -n", temp_dir) == 2)
            check("bundled short options carrying -n are blocked (-anm)",
                  gate_exit_code("git commit -anm x", temp_dir) == 2)
            check("abbreviated long option is blocked (--no-verif)",
                  gate_exit_code("git commit --no-verif -m x", temp_dir) == 2)
            check("glued separator cannot hide the invocation (true;git)",
                  gate_exit_code("true;git commit -n", temp_dir) == 2)
            check("environment-assignment prefix cannot hide the invocation",
                  gate_exit_code("FOO=1 git commit -n", temp_dir) == 2)
            check("env wrapper cannot hide the invocation",
                  gate_exit_code("env git commit --no-verify", temp_dir) == 2)
            check("wrapper with its own option-value cannot hide it",
                  gate_exit_code("sudo -u operator git commit -n",
                                 temp_dir) == 2)
            check("wrapper option-value named git cannot hide it",
                  gate_exit_code("env -u git git commit -n", temp_dir) == 2)
            check("valueless wrapper option before git is still caught",
                  gate_exit_code("env -i git commit -n", temp_dir) == 2)
            check("git as data of another command is not an invocation",
                  gate_exit_code("echo git commit -n", temp_dir) == 0)
            check("git as data stays data behind a wrapper (env echo git)",
                  gate_exit_code("env echo git commit -n", temp_dir) == 0)
            check("git as data after a line-spanning quote stays data",
                  gate_exit_code('echo "first\nsecond" && echo git commit -n',
                                 temp_dir) == 0)
            check("real bypass after a line-spanning quote is blocked",
                  gate_exit_code('echo "first\nsecond" && git commit -n',
                                 temp_dir) == 2)
            check("duration-argument wrapper cannot hide it (timeout 30 git)",
                  gate_exit_code("timeout 30 git commit -n", temp_dir) == 2)
            check("redirection cannot hide the invocation (git 2>/dev/null)",
                  gate_exit_code("git 2>/dev/null commit -n", temp_dir) == 2)
            check("glued separator after the flag is still blocked",
                  gate_exit_code("git commit --no-verify&&git push",
                                 temp_dir) == 2)
            check("commit is blocked while core.hooksPath is not .githooks",
                  gate_exit_code("git commit -m x", temp_dir) == 2)
            # #206 false-positive regressions: benign flags and quoted prose
            # must not trip the bypass block.
            check("prose in a quoted body is not a commit",
                  gate_exit_code(
                      'gh issue create --body "never use git commit '
                      '--no-verify or -n here"', temp_dir) == 0)
            check("git log --grep commit -n 5 is not a commit",
                  gate_exit_code("git log --grep commit -n 5", temp_dir) == 0)
            subprocess.run(["git", "-C", temp_dir, "config",
                            "core.hooksPath", ".githooks"], check=True)
            check("commit passes once the #47 gate is active",
                  gate_exit_code("git commit -m x", temp_dir) == 0)
            check("compound command with grep -n passes",
                  gate_exit_code(
                      'git commit -m fix && git push && gh pr create '
                      '--body "checked with grep -n"', temp_dir) == 0)
            check("short-option value containing n passes (-mnope)",
                  gate_exit_code("git commit -mnope", temp_dir) == 0)
            check("pathspec after -- is not an option (file named -n)",
                  gate_exit_code("git commit -am x -- -n", temp_dir) == 0)
            check("trailing redirection on a clean commit passes",
                  gate_exit_code("git commit -m x > /dev/null 2>&1",
                                 temp_dir) == 0)
            check("bypass flag still blocked inside a compound command",
                  gate_exit_code(
                      "git add . && git commit -n -m x && git push",
                      temp_dir) == 2)
            check("heredoc commit message mentioning flags passes",
                  gate_exit_code(
                      'git commit -m "$(cat <<\'EOF\'\n'
                      "fixes the grep -n and --no-verify prose case\n"
                      'EOF\n)"', temp_dir) == 0)
            check("multi-line real bypass is still blocked",
                  gate_exit_code("cd .\ngit commit -n", temp_dir) == 2)
            check("line-continuation bypass is blocked (backslash-newline)",
                  gate_exit_code("git commit \\\n-n", temp_dir) == 2)
            check("unclosed quote cannot hide a later-line bypass",
                  gate_exit_code('echo "unclosed\ngit commit -n',
                                 temp_dir) == 2)
            deep = "git commit -n"
            for _ in range(6):
                deep = "sh -c " + shlex.quote(deep)
            check("adversarially deep sh -c nesting fails CLOSED",
                  gate_exit_code(deep, temp_dir) == 2)
            check("nested shell -c bypass is blocked",
                  gate_exit_code("sh -c 'git commit -n'", temp_dir) == 2)
            check("bundled shell shorts (-lc) bypass is blocked",
                  gate_exit_code("bash -lc 'git commit --no-verify'",
                                 temp_dir) == 2)
            check("nested shell with git as data passes",
                  gate_exit_code("sh -c 'echo git commit -n'", temp_dir) == 0)
            check("shell -c as data of another command passes",
                  gate_exit_code("echo sh -c 'git commit -n'", temp_dir) == 0)
            # -C targeting: hooksPath is checked in the repo the commit
            # TARGETS, not the hook's working directory.
            with tempfile.TemporaryDirectory() as other_repo:
                subprocess.run(["git", "init", "-q", other_repo], check=True)
                check("git -C into a gate-less repo is blocked even from an "
                      "active clone",
                      gate_exit_code(f'git -C "{other_repo}" commit -m x',
                                     temp_dir) == 2)
                check("equals-form --git-dir targeting is honored",
                      gate_exit_code(
                          f'git --git-dir="{other_repo}/.git" commit -m x',
                          temp_dir) == 2)

    for name in failures:
        print(f"FAIL: {name}")
    print(f"check-hook-registration: {checks - len(failures)}/{checks} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
