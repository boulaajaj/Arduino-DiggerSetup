#!/usr/bin/env python3
"""PostToolUse hook (#156): documentation-sync prompt on production-code edits.

Claude Code pipes the tool payload to stdin after every Edit/Write. When the
edited file is production code (not generated, not a secret), this emits the
JSON `additionalContext` form — the only PostToolUse output that reaches the
model (plain stdout is transcript-only) — telling the agent to verify the
architecture docs and docs/wiki against the change in the same PR.

Fires once per session (marker file keyed by the payload's session_id): the
prompt is a nudge to audit at the end of the task, not per-edit noise.
"""

import json
import os
import re
import sys
import tempfile

WATCHED = (
    re.compile(r"(^|[\\/])sketches[\\/].*\.(ino|h|cpp)$"),
    re.compile(r"(^|[\\/])dashboard[\\/]index\.html$"),
    re.compile(r"(^|[\\/])scripts[\\/][^\\/]+\.py$"),
    re.compile(r"(^|[\\/])\.github[\\/]workflows[\\/][^\\/]+\.ya?ml$"),
    re.compile(r"(^|[\\/])tests[\\/]"),
)

EXCLUDED = (
    re.compile(r"web_page\.h$"),          # generated from dashboard/index.html
    re.compile(r"arduino_secrets\.h$"),   # gitignored secret, never documented
    re.compile(r"(^|[\\/])src[\\/]generated[\\/]"),
    re.compile(r"(^|[\\/])tests[\\/]build[\\/]"),
    re.compile(r"(^|[\\/])vendor[\\/]"),
)

PROMPT = (
    "[DOCUMENTATION-SYNC HOOK #156] Production code changed this session "
    "(first hit: {path}). Before this task ends, verify the documentation "
    "still matches the code and update it in the SAME PR wherever it does "
    "not: docs/wiki/ notes covering the changed area (same-PR rule, "
    "docs/wiki/README.md), docs/architecture/ARCHITECTURE-TARGET.md, "
    "docs/architecture/FILE-MAP.md, docs/TESTING.md, docs/SAFETY.md, and "
    "OPERATOR-GUIDE.md for operator-facing behavior. Tunable values stay "
    "canonical in src/config/ — docs describe behavior, never live numbers "
    "(#124). If everything already matches, say so briefly and move on."
)


def should_fire(file_path):
    if not file_path:
        return False
    if any(pattern.search(file_path) for pattern in EXCLUDED):
        return False
    return any(pattern.search(file_path) for pattern in WATCHED)


def marker_path(session_id):
    safe = re.sub(r"[^A-Za-z0-9_-]", "_", session_id or "unknown")
    return os.path.join(tempfile.gettempdir(),
                        "documentation-sync-hook-" + safe)


def main():
    try:
        payload = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        return 0  # malformed payload: a hook must never break the session
    file_path = (payload.get("tool_input") or {}).get("file_path", "")
    if not should_fire(file_path):
        return 0
    marker = marker_path(payload.get("session_id", ""))
    if os.path.exists(marker):
        return 0
    try:
        open(marker, "w").close()
    except OSError:
        pass  # fail open: reminding twice beats never reminding
    print(json.dumps({
        "hookSpecificOutput": {
            "hookEventName": "PostToolUse",
            "additionalContext": PROMPT.format(path=file_path),
        }
    }))
    return 0


if __name__ == "__main__":
    sys.exit(main())
