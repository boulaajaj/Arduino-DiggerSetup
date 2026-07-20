#!/usr/bin/env python3
"""Selftest for check_change_impact.py (#199) — exercises the matching
logic with injected inputs and validates the REAL manifest against the
repo. Run in CI (change-impact workflow) and locally:
    python scripts/check_change_impact_selftest.py
"""

import importlib.util
import os
import sys

CHECKER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "check_change_impact.py")
specification = importlib.util.spec_from_file_location("checker", CHECKER)
checker = importlib.util.module_from_spec(specification)
specification.loader.exec_module(checker)

SYNTHETIC = {"mappings": [
    {"areas": ["src/domain/drive/"], "pages": ["docs/a.md", "docs/b.md"]},
    {"areas": ["scripts/"], "pages": ["docs/c.md"]},
]}


def main():
    failures = []
    checks = 0

    def check(name, condition):
        nonlocal checks
        checks += 1
        if not condition:
            failures.append(name)

    check("untouched areas affect nothing",
          checker.affected_pages(["README.md"], SYNTHETIC) == set())
    check("a touched area maps to its pages",
          checker.affected_pages(["src/domain/drive/CurvatureDrive.cpp"],
                                 SYNTHETIC) == {"docs/a.md", "docs/b.md"})
    check("two touched areas union their pages",
          checker.affected_pages(
              ["src/domain/drive/x.h", "scripts/tool.py"],
              SYNTHETIC) == {"docs/a.md", "docs/b.md", "docs/c.md"})
    check("prefix matching does not swallow siblings",
          checker.affected_pages(["src/domain/driver_station.md"],
                                 SYNTHETIC) == set())
    check("receipt pattern accepts the skill's block",
          bool(checker.RECEIPT_PATTERN.search("...\nDocumentation receipt\n")))
    check("receipt pattern accepts the explicit no-effect line",
          bool(checker.RECEIPT_PATTERN.search(
              "No documentation effect: comment-only")))
    check("receipt pattern rejects unrelated prose",
          not checker.RECEIPT_PATTERN.search("updated the docs folder"))
    check("the REAL manifest validates against the repo",
          checker.validate_manifest(checker.load_manifest()) == [])

    for name in failures:
        print(f"FAIL: {name}")
    print(f"check-change-impact selftest: "
          f"{checks - len(failures)}/{checks} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
