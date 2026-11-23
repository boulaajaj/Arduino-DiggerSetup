#!/usr/bin/env python3
"""Build or check the standalone knowledge map from the tracked project wiki."""
from __future__ import annotations

import argparse
from functools import lru_cache
import hashlib
import json
import posixpath
import re
from pathlib import Path, PurePosixPath
import subprocess
import sys
from urllib.parse import unquote, urlsplit

import check_wiki

ROOT = Path(__file__).resolve().parents[1]
WIKI = "docs/wiki/"
HOME = WIKI + "home.md"
OUTPUT = "docs/knowledge-graph.html"
TEMPLATE = "scripts/knowledge_graph.html"
REPOSITORY = "https://github.com/boulaajaj/Arduino-DiggerSetup"
LINK = re.compile(
    r"!?(?<!\\)\[(?P<label>[^\]]+)\]\(\s*"
    r"(?P<target><[^>]+>|(?:[^\s()]|\([^()]*\))+)\s*(?:\"[^\"]*\"\s*)?\)"
)
FENCE = re.compile(r"^ {0,3}(`{3,}|~{3,})")
FRONTMATTER = re.compile(r"\A---\n(.*?)\n---(?:\n|$)", re.S)
MARKDOWN = (".md", ".markdown", ".mdx")
LINK_OR_CODE = re.compile(
    r"(?P<code_delimiter>`+)(?:(?!`).)*?(?P=code_delimiter)|" + LINK.pattern
)


@lru_cache(maxsize=None)
def document(text: str) -> tuple[dict[str, str], str, list[dict]]:
    """Return graph metadata, original body, and headings outside fenced code."""
    front = FRONTMATTER.match(text)
    metadata = {}
    if front:
        for key, value in re.findall(r"^(parent|status):[ \t]*(.*)$", front[1], re.M):
            if key in metadata:
                raise ValueError(f"duplicate {key} declaration")
            metadata[key] = value.strip()
        text = text[front.end():]
    headings = []
    used = set()
    offset = front[0].count("\n") if front else 0
    for number, line in enumerate(prose_lines(text), 1):
        match = check_wiki.HEADING_PATTERN.match(line)
        if not match:
            continue
        title = match[2].strip().rstrip("#").rstrip()
        plain = LINK.sub(lambda m: m["label"], title).replace("`", "")
        base = re.sub(r"[^\w\- ]", "", plain.lower()).replace(" ", "-")
        slug = base
        suffix = 1
        while slug in used:
            slug = f"{base}-{suffix}"
            suffix += 1
        used.add(slug)
        headings.append({"title": plain, "anchor": slug, "line": number + offset,
                         "level": len(match[1])})
    return metadata, text, headings


def prose_lines(text: str):
    fence = ""
    for line in text.splitlines():
        match = FENCE.match(line)
        if match:
            marker = match[1]
            if not fence:
                fence = marker
                yield ""
                continue
            if marker.startswith(fence) and not line[match.end():].strip():
                fence = ""
        yield "" if fence or match else line


def links(body: str):
    # Preserve backticks in link labels, omit link-shaped samples in code spans.
    for match in LINK_OR_CODE.finditer("\n".join(prose_lines(body))):
        if match["target"]:
            yield match["label"], match["target"].strip("<>")


def list_paths(root: Path) -> tuple[set[str], set[str]]:
    raw = subprocess.check_output(["git", "ls-files", "-z"], cwd=root)
    paths = {name.decode() for name in raw.split(b"\0") if name}
    files = {path for path in paths if (root / path).is_file()
             and (root / path).resolve().is_relative_to(root.resolve())}
    directories = {parent.as_posix() for path in files
                   for parent in PurePosixPath(path).parents if parent.as_posix() != "."}
    return files, directories


def resolve(path: str, target: str, *, from_root: bool = False):
    url = urlsplit(target)
    if url.scheme or url.netloc:
        return None
    source_path = unquote(url.path)
    if not source_path:
        return path, unquote(url.fragment)
    folder = "" if from_root or source_path.startswith("/") else posixpath.dirname(path)
    return posixpath.normpath(posixpath.join(folder, source_path.lstrip("/"))), unquote(url.fragment)


def summary(text: str) -> str:
    paragraphs = re.split(r"\n\s*\n", "\n".join(prose_lines(text)).strip())
    for paragraph in paragraphs:
        if paragraph.startswith(("#", "|", "-", "<!--", ">", "---")):
            continue
        value = LINK.sub(lambda m: m["label"], paragraph)
        value = re.sub(r"[\s]+", " ", value.replace("**", "").replace("`", "")).strip()
        if value:
            return value if len(value) <= 420 else value[:417].rsplit(" ", 1)[0] + "…"
    return ""


def build(root: Path) -> tuple[dict, list[str]]:
    files, directories = list_paths(root)
    notes = sorted(path for path in files if path.startswith(WIKI) and path.endswith(".md"))
    if HOME not in notes:
        raise ValueError(f"Missing tracked wiki root: {HOME}")
    nodes, edges, warnings, errors = {}, {}, set(), set()

    @lru_cache(maxsize=None)
    def read(path):
        try:
            return document((root / path).read_text(encoding="utf-8"))
        except ValueError as error:
            raise ValueError(f"{path}: {error}") from error

    def add_node(path, wiki=False):
        kind = "wiki" if wiki else "directory" if path in directories else "file"
        markdown = path.lower().endswith(MARKDOWN)
        metadata, body, headings = read(path) if markdown else ({}, "", [])
        title = next((item["title"] for item in headings if item["level"] == 1), "")
        if wiki and not title:
            errors.add(f"{path}: wiki note must have a level-one title")
        title = title or PurePosixPath(path).name
        node = {"id": path, "title": title, "kind": kind, "summary": summary(body)}
        if wiki:
            node["body"] = body.strip()
            node["status"] = metadata.get("status", "unspecified")
            if "status" in metadata and metadata["status"] not in {"current", "historical"}:
                errors.add(f"{path}: status must be current or historical")
            parent = metadata.get("parent")
            if path == HOME:
                if parent is not None:
                    errors.add(f"{path}: Home cannot declare a parent")
            elif not parent:
                errors.add(f"{path}: declare one parent relative to this note")
            else:
                target = resolve(path, parent)
                if target is None or target[1] or target[0] not in notes:
                    errors.add(f"{path}: unknown wiki parent {parent}")
                else:
                    node["parent"] = target[0]
        nodes[path] = node

    def scan(path, references, *, wiki):
        for label, raw, relation in references:
            resolved = resolve(path, raw, from_root=relation == "source")
            if resolved is None:
                if relation == "source":
                    errors.add(f"{path}: declared source must be a repository path: {raw}")
                continue
            target, anchor = resolved
            if target not in files and target not in directories:
                (errors if wiki else warnings).add(f"{path}: unresolved reference {raw}")
                continue
            heading = None
            if anchor and target.lower().endswith(MARKDOWN):
                heading = next((item for item in read(target)[2] if item["anchor"] == anchor), None)
                if not heading:
                    (errors if wiki else warnings).add(f"{path}: unknown heading {raw}")
                    if wiki:
                        continue
            if target == path:
                continue
            if target not in nodes:
                if not wiki:
                    continue
                add_node(target)
            if nodes[target].get("parent") == path and relation == "link":
                relation = "child"
            key = path, target, relation, anchor
            edge = {"from": path, "to": target, "type": relation,
                    "label": " ".join(label.split()).replace("`", "")}
            if anchor:
                edge["anchor"] = anchor
            if heading:
                edge["line"] = heading["line"]
            edges.setdefault(key, edge)

    for path in notes:
        add_node(path, wiki=True)
    for path in notes:
        body = read(path)[1]
        references = [(label, raw, "link") for label, raw in links(body)]
        sources, _ = check_wiki.parse_sources(root / path)
        references.extend((item.rstrip("/").split("/")[-1], item, "source")
                          for item in (sources or []))
        scan(path, references, wiki=True)

    # Canonical documents remain one click from the wiki. Cross-references
    # between indexed resources are useful without crawling the whole repo.
    for path in sorted(nodes.keys() - set(notes)):
        if path.lower().endswith(MARKDOWN):
            scan(path, ((label, raw, "link") for label, raw in links(read(path)[1])), wiki=False)

    for path in notes:
        parent = nodes[path].get("parent")
        if parent and not any(edge["from"] == parent and edge["to"] == path and
                              edge["type"] == "child" for edge in edges.values()):
            errors.add(f"{path}: parent must link to this note from {parent}")

        trail = [path]
        while ancestor := nodes[trail[-1]].get("parent"):
            if ancestor in trail:
                errors.add(f"{path}: parent cycle ({' → '.join([*trail, ancestor])})")
                break
            trail.append(ancestor)
        if trail[-1] != HOME:
            errors.add(f"{path}: hierarchy must lead to {HOME}")
        nodes[path]["depth"] = len(trail) - 1
        if len(trail) > 1:
            nodes[path]["domain"] = trail[-2]

    if errors:
        raise ValueError("\n".join(sorted(errors)))
    graph = {"home": HOME, "repository": REPOSITORY, "ref": "main",
             "nodes": [nodes[path] for path in sorted(nodes)],
             "edges": [edges[key] for key in sorted(edges)]}
    payload = json.dumps(graph, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    graph["revision"] = hashlib.sha256(payload.encode()).hexdigest()[:12]
    return graph, sorted(warnings)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the committed HTML")
    options = parser.parse_args()
    try:
        failures = check_wiki.run(ROOT)
        if failures:
            raise ValueError("\n".join(failures))
        graph, warnings = build(ROOT)
        template = (ROOT / TEMPLATE).read_text(encoding="utf-8")
        if template.count("<!--GRAPH_DATA-->") != 1:
            raise ValueError("HTML template must contain one graph data marker")
        payload = json.dumps(graph, ensure_ascii=False, separators=(",", ":"))
        rendered = template.replace("<!--GRAPH_DATA-->", payload.replace("<", "\\u003c"))
        output = ROOT / OUTPUT
        if options.check:
            if not output.exists() or output.read_bytes() != rendered.encode("utf-8"):
                raise ValueError(f"{OUTPUT} is stale. Run python scripts/build_knowledge_graph.py and commit the result.")
        else:
            output.write_bytes(rendered.encode("utf-8"))
        for warning in warnings:
            print(f"Reference outside wiki: {warning}", file=sys.stderr)
        print(f"Knowledge graph {graph['revision']} {'verified' if options.check else 'generated'}: {OUTPUT}")
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Knowledge graph: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
