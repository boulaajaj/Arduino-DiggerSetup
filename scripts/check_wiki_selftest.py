"""Self-test for the wiki lint (#145, v2 #202).

Builds a throwaway repo skeleton with one deliberate violation per check,
asserts each fires, then asserts a clean skeleton passes. CI runs this
before the real lint so a silently-broken linter cannot go green.
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from check_wiki import run  # noqa: E402

FRONTMATTER = "---\nsources: none\n---\n"

EXPECTED_VIOLATIONS = {
    "broken link -> missing-note.md": "note links to a file that does not exist",
    "broken link -> ../../../outside.md": "link escaping the repo root",
    "broken link -> ../": "link resolving to a directory, not a file",
    "'10.5 V'": "note restates a voltage",
    "'60%'": "note restates a percentage (regression: trailing \\b never matched %)",
    "orphan": "note with no inbound wiki links",
    "island-one.md: not reachable": "linked pair disconnected from home",
    "readme-only.md: not reachable": "note linked only from README, not from home.md",
    "no sources frontmatter": "note without a frontmatter sources block",
    "unknown source path in frontmatter -> docs/GONE.md":
        "declared source that does not exist",
    "broken anchor -> home.md#no-such-heading": "anchor with no matching heading",
    "broken anchor -> stale.md#in-fence":
        "a # line inside a code fence is not a heading",
    "duplicate title": "two notes sharing one H1",
    "deprecated phrase ('test-gate.sh')": "retired name reappearing",
    "nested/sub-orphan.md: orphan": "recursive discovery finds subdirectory notes",
}


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def build_violating_repo(root: Path) -> Path:
    repo = root / "repo"
    wiki = repo / "docs" / "wiki"
    write(root / "outside.md", "exists on disk, but outside the repo root\n")
    write(wiki / "README.md", "[home](home.md)\n[readme only](readme-only.md)\n")
    write(wiki / "home.md",
          FRONTMATTER +
          "# Home\n"
          "[gone](missing-note.md)\n"
          "[escape](../../../outside.md)\n"
          "[directory](../)\n"
          "[safety](safety.md)\n"
          "[bad anchor](home.md#no-such-heading)\n"
          "[dup a](dup-a.md)\n[dup b](dup-b.md)\n[stale](stale.md)\n"
          "[ghost](ghost-sources.md)\n")
    write(wiki / "safety.md",
          "Cutoff at 10.5 V and caps at 60% live here.\n")
    write(wiki / "dup-a.md", FRONTMATTER + "# Same Title\n[home](home.md)\n")
    write(wiki / "dup-b.md", FRONTMATTER + "# Same Title\n[home](home.md)\n")
    write(wiki / "stale.md",
          FRONTMATTER + "# Stale\nStill mentions test-gate.sh here.\n"
          "```\n# in fence\n```\n"
          "[fence anchor](stale.md#in-fence)\n")
    write(wiki / "ghost-sources.md",
          "---\nsources:\n  - docs/GONE.md\n---\n# Ghost\n[home](home.md)\n")
    write(wiki / "readme-only.md", FRONTMATTER + "# Readme only\n[home](home.md)\n")
    write(wiki / "orphan-note.md", FRONTMATTER + "# Orphan\n[home](home.md)\n")
    write(wiki / "island-one.md", FRONTMATTER + "# One\n[two](island-two.md)\n")
    write(wiki / "island-two.md", FRONTMATTER + "# Two\n[one](island-one.md)\n")
    write(wiki / "nested" / "sub-orphan.md",
          FRONTMATTER + "# Nested orphan\n[home](../home.md)\n")
    return repo


def build_clean_repo(root: Path) -> Path:
    repo = root / "repo"
    wiki = repo / "docs" / "wiki"
    write(repo / "docs" / "CANONICAL.md",
          "# Canonical\n\n## Real Constants\n\nThe real constants live here.\n"
          "\n```\n# not a heading (inside a code fence)\n```\n"
          "\n## Real Constants\n\nDuplicate heading gets a -1 slug.\n")
    write(wiki / "README.md", "[home](home.md)\n")
    write(wiki / "home.md",
          FRONTMATTER + "# Home\n[safety](safety.md)\n[drive](drive.md)\n"
          "[nested](nested/deep.md)\n")
    write(wiki / "safety.md",
          "---\nsources:\n  - docs/CANONICAL.md\n---\n"
          "# Safety\nThresholds: see "
          "[canonical](../CANONICAL.md#real-constants). [drive](drive.md)\n")
    write(wiki / "drive.md",
          "---\nsources:\n  - docs/CANONICAL.md\n---\n"
          "# Drive\nLoops back to [safety](safety.md). Duplicate-heading "
          "anchor: [second](../CANONICAL.md#real-constants-1).\n")
    write(wiki / "nested" / "deep.md",
          FRONTMATTER + "# Deep\n[home](../home.md)\n")
    return repo


def main() -> int:
    with tempfile.TemporaryDirectory() as raw:
        repo = build_violating_repo(Path(raw))
        report = "\n".join(run(repo))
        missing = [f"self-test: expected a '{marker}' failure ({why})"
                   for marker, why in EXPECTED_VIOLATIONS.items()
                   if marker not in report]
        for problem in missing:
            print(f"FAIL: {problem}")
        if missing:
            print("\n--- violating-fixture output for debugging ---")
            print(report)
            return 1

    with tempfile.TemporaryDirectory() as raw:
        repo = build_clean_repo(Path(raw))
        failures = run(repo)
        if failures:
            print("FAIL: self-test: clean fixture should pass, got:")
            print("\n".join(failures))
            return 1

    print(f"wiki-lint self-test: OK "
          f"({len(EXPECTED_VIOLATIONS)} failure modes demonstrated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
