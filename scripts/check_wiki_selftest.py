"""Self-test for the wiki lint (#145).

Builds a throwaway docs/wiki fixture with one deliberate violation per
check, asserts each fires, then asserts a clean fixture passes. CI runs
this before the real lint so a silently-broken linter cannot go green.
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from check_wiki import run  # noqa: E402

EXPECTED_VIOLATIONS = {
    "broken link": "note links to a file that does not exist",
    "tunable-looking value": "note restates '10.5 V'",
    "orphan": "note with no inbound wiki links",
    "not reachable from home.md": "linked pair disconnected from home",
}


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def build_violating_wiki(root: Path) -> None:
    wiki = root / "docs" / "wiki"
    write(wiki / "README.md", "[home](home.md)\n")
    write(wiki / "home.md", "[gone](missing-note.md)\n[safety](safety.md)\n")
    write(wiki / "safety.md", "Cutoff at 10.5 V is documented here.\n")
    write(wiki / "orphan-note.md", "[home](home.md)\n")
    write(wiki / "island-one.md", "[two](island-two.md)\n")
    write(wiki / "island-two.md", "[one](island-one.md)\n")


def build_clean_wiki(root: Path) -> None:
    wiki = root / "docs" / "wiki"
    write(root / "docs" / "CANONICAL.md", "The real constants live here.\n")
    write(wiki / "README.md", "[home](home.md)\n")
    write(wiki / "home.md", "[safety](safety.md)\n[drive](drive.md)\n")
    write(wiki / "safety.md",
          "Thresholds: see [canonical](../CANONICAL.md). [drive](drive.md)\n")
    write(wiki / "drive.md", "Loops back to [safety](safety.md).\n")


def main() -> int:
    with tempfile.TemporaryDirectory() as raw:
        root = Path(raw)
        build_violating_wiki(root)
        report = "\n".join(run(root))
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
        root = Path(raw)
        build_clean_wiki(root)
        failures = run(root)
        if failures:
            print("FAIL: self-test: clean fixture should pass, got:")
            print("\n".join(failures))
            return 1

    print(f"wiki-lint self-test: OK "
          f"({len(EXPECTED_VIOLATIONS)} failure modes demonstrated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
