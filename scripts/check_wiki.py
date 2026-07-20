"""Wiki lint (#145, v2 #202) — health checks for the docs/wiki/ graph.

The third operation of the Karpathy LLM-wiki pattern (#141): ingest and
query exist; this is lint. Stdlib-only, identical locally and in CI.

Checks:
  1. Link integrity — every relative Markdown link in a wiki note resolves
     (recursively: notes may live in subdirectories).
  2. Orphans — every note (except README.md/home.md) has an inbound link
     from another wiki note.
  3. Reachability — every note is reachable from home.md via wiki links.
  4. Tunable drift — wiki notes must not carry tunable-looking values
     (numbers with units); those live only in their authority
     (docs/wiki/README.md anti-drift rule).
  5. Heading anchors — a link's #fragment must match a heading in the
     target Markdown file.
  6. Sources frontmatter — every note (except README.md) declares the
     canonical sources it navigates to, and each declared path exists.
  7. Duplicate titles — two notes may not share the same H1.
  8. Deprecated phrases — retired names/framings may not reappear.

Exit code 1 on any failure.
"""
from __future__ import annotations

import re
import subprocess
import sys
from collections import deque
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LINK_PATTERN = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
EXTERNAL_SCHEMES = ("http://", "https://", "mailto:")
HUB_NOTES = {"README.md", "home.md"}  # allowed to have no inbound links
FRONTMATTER_EXEMPT = {"README.md"}
TUNABLE_PATTERN = re.compile(
    r"\b\d+(?:\.\d+)?\s?(?:V|A|Hz|kHz|MHz|s|ms|us|µs|°C|%)(?!\w)")
DEPRECATED_PHRASES = {
    "sketches/rc_test": "renamed to dual_track_control (#118)",
    "test-gate.sh": "the gate is test-gate.py (#206)",
    "Phase D target": "Phase D completed (#189) — describe the present",
}


def note_links(note: Path) -> list[str]:
    """Raw relative link targets, fragments preserved."""
    return [target
            for target in LINK_PATTERN.findall(note.read_text(encoding="utf-8"))
            if target and not target.startswith(EXTERNAL_SCHEMES)
            and not target.startswith("#")]


def heading_slugs(markdown_path: Path) -> set[str]:
    """GitHub-style anchor slugs for every heading in a Markdown file."""
    slugs: set[str] = set()
    for line in markdown_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            heading = line.lstrip("#").strip().lower()
            heading = re.sub(r"[^\w\- ]", "", heading, flags=re.UNICODE)
            slugs.add(heading.replace(" ", "-"))
    return slugs


def parse_sources(note: Path) -> tuple[list[str] | None, str | None]:
    """The frontmatter sources list, or (None, why-it-is-malformed)."""
    lines = note.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "---":
        return None, "no sources frontmatter (--- block with a sources list)"
    block: list[str] = []
    for line in lines[1:]:
        if line.strip() == "---":
            break
        block.append(line)
    else:
        return None, "unterminated frontmatter block"
    in_sources = False
    sources: list[str] = []
    for line in block:
        stripped = line.strip()
        if stripped == "sources: none":
            return [], None
        if stripped == "sources:":
            in_sources = True
            continue
        if in_sources and stripped.startswith("- "):
            sources.append(stripped[2:].strip())
        elif stripped and not line.startswith(" "):
            in_sources = False
    if not sources:
        return None, "frontmatter has no sources list (or it is empty)"
    return sources, None


def check_note(note: Path, name: str, wiki_root: Path, repo_root: Path,
               graph: dict[str, set[str]]) -> list[str]:
    failures: list[str] = []
    for target in note_links(note):
        path_part, _, fragment = target.partition("#")
        resolved = (note.parent / path_part).resolve()
        try:
            resolved.relative_to(repo_root.resolve())
            inside_repo = True
        except ValueError:
            inside_repo = False  # escapes the repo — never a valid target
        if not inside_repo or not resolved.is_file():
            failures.append(f"{name}: broken link -> {target}")
            continue
        if fragment and resolved.suffix == ".md":
            if fragment.lower() not in heading_slugs(resolved):
                failures.append(
                    f"{name}: broken anchor -> {target} "
                    f"(no such heading in the target)")
        try:
            graph[name].add(
                resolved.relative_to(wiki_root.resolve()).as_posix())
        except ValueError:
            pass  # links outside the wiki are canon pointers, not graph edges

    text = note.read_text(encoding="utf-8")
    for line_number, line in enumerate(text.splitlines(), 1):
        for match in TUNABLE_PATTERN.findall(line):
            failures.append(
                f"{name}:{line_number}: tunable-looking value "
                f"({match!r}) — constants live in their authority only "
                f"(docs/wiki/README.md anti-drift rule)")
        for phrase, why in DEPRECATED_PHRASES.items():
            if phrase in line:
                failures.append(
                    f"{name}:{line_number}: deprecated phrase "
                    f"({phrase!r}) — {why}")

    if note.name not in FRONTMATTER_EXEMPT:
        sources, problem = parse_sources(note)
        if problem:
            failures.append(f"{name}: {problem}")
        else:
            for source in sources or []:
                absolute = repo_root / source
                if not (absolute.exists() or list(
                        absolute.parent.glob(absolute.name + "*"))):
                    failures.append(
                        f"{name}: unknown source path in frontmatter "
                        f"-> {source}")
    return failures


def git_ignored(note: Path, repo_root: Path) -> bool:
    """True when git ignores the file (personal scratch notes stay local)."""
    try:
        return subprocess.run(
            ["git", "check-ignore", "-q", str(note)],
            cwd=repo_root, capture_output=True, timeout=10).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False  # no git — lint everything


def check_wiki(wiki_root: Path,
               repo_root: Path) -> tuple[list[str], dict[str, set[str]]]:
    """Return (failures, wiki-internal link graph by relative path)."""
    failures: list[str] = []
    graph: dict[str, set[str]] = {}
    notes = sorted(note for note in wiki_root.rglob("*.md")
                   if not git_ignored(note, repo_root))
    if not notes:
        return [f"{wiki_root}: no wiki notes found"], graph

    titles: dict[str, str] = {}
    for note in notes:
        name = note.relative_to(wiki_root).as_posix()
        graph.setdefault(name, set())
        failures += check_note(note, name, wiki_root, repo_root, graph)
        for line in note.read_text(encoding="utf-8").splitlines():
            if line.startswith("# "):
                title = line[2:].strip()
                if title in titles:
                    failures.append(
                        f"{name}: duplicate title ({title!r}) — already "
                        f"used by {titles[title]}")
                else:
                    titles[title] = name
                break
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
