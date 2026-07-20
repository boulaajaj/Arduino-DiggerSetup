#!/usr/bin/env python3
"""Change-impact check (#199) — the deterministic half of doc-code sync.

Reads docs/architecture/change-impact.json (source areas → knowledge
pages), computes which pages a diff potentially affects, and passes when:
  1. no mapped area was touched, or
  2. at least one affected page is updated in the same diff, or
  3. the PR body carries a documentation receipt (the wiki-impact-review
     skill's receipt block, or an explicit no-documentation-effect line).
Otherwise it fails, listing the affected pages and both ways to satisfy it.

The manifest itself is validated on every run: every page must be an
existing file, every area prefix must match something in the repo.

CI (change-impact workflow) provides BASE_SHA and PR_BODY; locally it
compares against origin/main:
    python scripts/check_change_impact.py
"""

import glob
import json
import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANIFEST_PATH = os.path.join(REPO_ROOT, "docs", "architecture",
                             "change-impact.json")
# Anchored: the receipt block's heading line, or an explicit no-effect line
# WITH a non-empty reason — mid-prose mentions never count.
RECEIPT_PATTERN = re.compile(
    r"^\s*documentation receipt\b|^\s*no documentation effect:\s*\S",
    re.IGNORECASE | re.MULTILINE)


def load_manifest():
    with open(MANIFEST_PATH, encoding="utf-8") as manifest_file:
        return json.load(manifest_file)


def validate_manifest(manifest):
    """Every page exists; every area prefix matches something real."""
    problems = []
    for mapping in manifest["mappings"]:
        for page in mapping["pages"]:
            if not os.path.isfile(os.path.join(REPO_ROOT, page)):
                problems.append(f"manifest page does not exist: {page}")
        for area in mapping["areas"]:
            absolute = os.path.join(REPO_ROOT, area)
            if not (os.path.exists(absolute) or glob.glob(absolute + "*")):
                problems.append(f"manifest area matches nothing: {area}")
    return problems


def changed_files(base_reference):
    output = subprocess.run(
        ["git", "diff", "--name-only", f"{base_reference}...HEAD"],
        capture_output=True, text=True, cwd=REPO_ROOT, timeout=60)
    if output.returncode != 0:
        raise RuntimeError(output.stderr.strip())
    return [line for line in output.stdout.splitlines() if line]


def affected_pages(files, manifest):
    pages = set()
    for mapping in manifest["mappings"]:
        if any(name.startswith(area)
               for name in files for area in mapping["areas"]):
            pages.update(mapping["pages"])
    return pages


def main():
    manifest = load_manifest()
    problems = validate_manifest(manifest)
    if problems:
        for problem in problems:
            print(f"FAIL: {problem}")
        return 1

    base_reference = os.environ.get("BASE_SHA", "origin/main")
    pull_request_body = os.environ.get("PR_BODY", "")
    files = changed_files(base_reference)
    pages = affected_pages(files, manifest)
    updated = pages.intersection(files)

    if not pages:
        print("change-impact: no mapped area touched — nothing to reconsider")
        return 0
    if updated:
        print("change-impact: affected pages updated in this diff: "
              + ", ".join(sorted(updated)))
        return 0
    if RECEIPT_PATTERN.search(pull_request_body):
        print("change-impact: documentation receipt present for: "
              + ", ".join(sorted(pages)))
        return 0

    print("FAIL: this change touches mapped source areas but neither "
          "updates an affected page nor carries a documentation receipt.")
    print("Affected pages: " + ", ".join(sorted(pages)))
    print("Satisfy by updating one of them in this PR, or add the "
          "wiki-impact-review skill's receipt (or an explicit "
          "'No documentation effect: <reason>' line) to the PR body.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
