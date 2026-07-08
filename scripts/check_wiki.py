"""Wiki lint (#145) — health checks for the docs/wiki/ knowledge graph.

The third operation of the Karpathy LLM-wiki pattern (#141): ingest and
query exist; this is lint. Stdlib-only, identical locally and in CI.

Checks:
  1. Link integrity — every relative Markdown link in a wiki note resolves.
  2. Orphans — every note (except README.md/home.md) has an inbound link
     from another wiki note.
  3. Reachability — every note is reachable from home.md via wiki links.
  4. Tunable drift — wiki notes must not carry tunable-looking values
     (numbers with units); those live only in canonical docs
     (docs/wiki/README.md anti-drift rule).

Exit code 1 on any failure.
"""
from __future__ import annotations

import re
import sys
from collections import deque
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LINK_PATTERN = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
EXTERNAL_SCHEMES = ("http://", "https://", "mailto:")
HUB_NOTES = {"README.md", "home.md"}  # allowed to have no inbound links
TUNABLE_PATTERN = re.compile(
    r"\b\d+(?:\.\d+)?\s?(?:V|A|Hz|kHz|MHz|s|ms|us|µs|°C|%)(?!\w)")


def wiki_links(note: Path) -> list[str]:
    return [target.split("#")[0]
            for target in LINK_PATTERN.findall(note.read_text(encoding="utf-8"))
            if target and not target.startswith(EXTERNAL_SCHEMES)
            and not target.startswith("#")]


def check_wiki(wiki_root: Path,
               repo_root: Path) -> tuple[list[str], dict[str, set[str]]]:
    """Return (failures, wiki-internal link graph by note name)."""
    failures: list[str] = []
    graph: dict[str, set[str]] = {}
    notes = sorted(wiki_root.glob("*.md"))
    if not notes:
        return [f"{wiki_root}: no wiki notes found"], graph

    for note in notes:
        graph.setdefault(note.name, set())
        for target in wiki_links(note):
            resolved = (note.parent / target).resolve()
            try:
                resolved.relative_to(repo_root.resolve())
                inside_repo = True
            except ValueError:
                inside_repo = False  # escapes the repo — never a valid target
            if not inside_repo or not resolved.is_file():
                failures.append(f"{note.name}: broken link -> {target}")
            elif resolved.parent == wiki_root.resolve():
                graph[note.name].add(resolved.name)
        for line_number, line in enumerate(
                note.read_text(encoding="utf-8").splitlines(), 1):
            for match in TUNABLE_PATTERN.findall(line):
                failures.append(
                    f"{note.name}:{line_number}: tunable-looking value "
                    f"({match!r}) — constants live in canonical docs only "
                    f"(docs/wiki/README.md anti-drift rule)")
    return failures, graph


def check_graph_shape(graph: dict[str, set[str]]) -> list[str]:
    """Orphan and reachability checks over the wiki-internal link graph."""
    failures: list[str] = []
    inbound: dict[str, int] = {name: 0 for name in graph}
    for source, targets in graph.items():
        for target in targets:
            if target != source and target in inbound:
                inbound[target] += 1
    for name, count in sorted(inbound.items()):
        if count == 0 and name not in HUB_NOTES:
            failures.append(f"{name}: orphan — no other wiki note links to it")

    reachable: set[str] = set()
    queue = deque(["home.md"]) if "home.md" in graph else deque()
    reachable.update(queue)
    while queue:
        for target in graph.get(queue.popleft(), ()):
            if target not in reachable:
                reachable.add(target)
                queue.append(target)
    for name in sorted(graph):
        if name not in reachable and name not in HUB_NOTES:
            failures.append(f"{name}: not reachable from home.md")
    return failures


def run(repo_root: Path) -> list[str]:
    wiki_root = repo_root / "docs" / "wiki"
    failures, graph = check_wiki(wiki_root, repo_root)
    failures += check_graph_shape(graph)
    return failures


def main() -> int:
    failures = run(Path(__file__).resolve().parent.parent)
    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        print(f"\nwiki-lint: {len(failures)} violation(s)")
        return 1
    print("wiki-lint: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
