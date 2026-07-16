#!/usr/bin/env python3
"""Hook-registration check (#193) — proves every script in .claude/hooks/ is
actually wired into .claude/settings.json (a present-but-unregistered hook
script is dead code), then functionally exercises test-gate.sh: it must block
agent commits using --no-verify/-n, block commits in a clone where the #47
pre-commit gate was never activated, and stay out of the way otherwise.
Run in CI (hooks-selftest.yml) and locally:
    python scripts/check_hook_registration.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS = os.path.join(REPO_ROOT, ".claude", "settings.json")
HOOKS_DIR = os.path.join(REPO_ROOT, ".claude", "hooks")
TEST_GATE = os.path.join(HOOKS_DIR, "test-gate.sh")


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
    """Run test-gate.sh with a PreToolUse-style JSON payload on stdin."""
    payload = json.dumps({"tool_input": {"command": command}})
    result = subprocess.run(["sh", TEST_GATE], input=payload,
                            capture_output=True, text=True, cwd=working_dir)
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
              any(script in command for command in commands))

    # 2. Behavior — exercise the gate in a temp repo where core.hooksPath is
    #    controlled (exit 2 = blocked, exit 0 = allowed through).
    if shutil.which("sh") is None or shutil.which("git") is None:
        print("check-hook-registration: sh/git unavailable — "
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
            check("commit is blocked while core.hooksPath is not .githooks",
                  gate_exit_code("git commit -m x", temp_dir) == 2)
            subprocess.run(["git", "-C", temp_dir, "config",
                            "core.hooksPath", ".githooks"], check=True)
            check("commit passes once the #47 gate is active",
                  gate_exit_code("git commit -m x", temp_dir) == 0)

    for name in failures:
        print(f"FAIL: {name}")
    print(f"check-hook-registration: {checks - len(failures)}/{checks} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
