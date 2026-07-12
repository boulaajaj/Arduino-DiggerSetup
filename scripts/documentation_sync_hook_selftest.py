#!/usr/bin/env python3
"""Selftest for documentation_sync_hook.py (#156) — proves the hook fires on
watched paths, stays silent on excluded/unwatched paths and malformed input,
and dedupes per session. Run in CI (hooks-selftest.yml) and locally:
    python scripts/documentation_sync_hook_selftest.py
"""

import json
import os
import subprocess
import sys
import tempfile

HOOK = os.path.join(os.path.dirname(__file__), "documentation_sync_hook.py")


def run_hook(stdin_text, temp_dir):
    env = dict(os.environ)
    for var in ("TMPDIR", "TEMP", "TMP"):  # isolate the session marker
        env[var] = temp_dir
    result = subprocess.run([sys.executable, HOOK], input=stdin_text,
                            capture_output=True, text=True, env=env)
    return result.returncode, result.stdout.strip()


def payload(file_path, session_id="session-a"):
    return json.dumps({"session_id": session_id,
                       "tool_input": {"file_path": file_path}})


def main():
    failures = []
    checks = 0

    def check(name, condition):
        nonlocal checks
        checks += 1
        if not condition:
            failures.append(name)

    with tempfile.TemporaryDirectory() as temp_dir:
        # 1. Watched firmware file fires with valid additionalContext JSON.
        code, out = run_hook(payload("sketches/rc_test/rc_test.ino"), temp_dir)
        fired = json.loads(out) if out else {}
        context = fired.get("hookSpecificOutput", {})
        check("fires on watched .ino", code == 0 and
              context.get("hookEventName") == "PostToolUse" and
              "DOCUMENTATION-SYNC" in context.get("additionalContext", ""))

        # 2. Same session again: deduped, silent.
        code, out = run_hook(payload("sketches/rc_test/types.h"), temp_dir)
        check("dedupes within a session", code == 0 and out == "")

        # 3. A different session fires again.
        code, out = run_hook(
            payload("tests/battery/test_battery_ladder_domain.cpp",
                    session_id="session-b"), temp_dir)
        check("fires again for a new session", code == 0 and out != "")

    silent_paths = (
        "sketches/rc_test/web_page.h",            # generated
        "sketches/rc_test/arduino_secrets.h",     # secret
        "sketches/rc_test/src/generated/page.h",  # generated tree
        "src/generated/page.h",                   # generated tree, relative root
        "tests/vendor/doctest/doctest.h",         # third-party
        "tests/build/test_alerts",                # build artifact, relative root
        "docs/wiki/battery-ladder.md",            # docs are the TARGET, not a trigger
        "",                                        # missing path
    )
    for path in silent_paths:
        with tempfile.TemporaryDirectory() as temp_dir:
            code, out = run_hook(payload(path, session_id="fresh"), temp_dir)
            check("silent on %r" % path, code == 0 and out == "")

    # Malformed stdin must exit 0 silently — a hook never breaks the session.
    with tempfile.TemporaryDirectory() as temp_dir:
        code, out = run_hook("not json at all", temp_dir)
        check("silent on malformed payload", code == 0 and out == "")

    if failures:
        print("documentation-sync-hook selftest: %d/%d FAILED: %s"
              % (len(failures), checks, ", ".join(failures)))
        return 1
    print("documentation-sync-hook selftest: %d checks OK" % checks)
    return 0


if __name__ == "__main__":
    sys.exit(main())
