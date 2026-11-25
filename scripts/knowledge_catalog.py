"""Inventory the checkout for the explorer, using paths and source evidence."""
from collections import Counter
from pathlib import PurePosixPath
import csv
import re

GENERATED = {
    "docs/knowledge-graph.html", "docs/knowledge-explorer-kit.html",
    "sketches/dual_track_control/web_page.h",
}
EXPLORER_FILES = {
    "docs/knowledge-explorer-kit.html", "scripts/knowledge_explorer_kit.html",
    "scripts/knowledge_inspector.py", "scripts/knowledge_catalog.py",
    "scripts/knowledge_subsystems.py",
}
CPP = {".h", ".hpp", ".cpp", ".ino"}
CODE = CPP | {".py", ".mjs", ".js"}
SYMBOL = {
    "Type": re.compile(r"^\s*(?:class|struct|enum(?: class)?)\s+(\w+)\b"),
    "Function": re.compile(
        r"^(?:(?:inline|static|constexpr|extern)\s+)*(?:[\w:<>&*]+\s+)+"
        r"([A-Za-z_]\w*)\s*\((?!.*\b(?:if|while)\b)"),
    "Definition": re.compile(r"^\s*(?:#define\s+|(?:inline\s+|static\s+)*(?:const|constexpr)\s+[\w:]+\s+)([A-Z][A-Z\d_]+)\b"),
    "Test case": re.compile(r'^\s*(?:TEST_CASE|SUBCASE)\("([^"]+)"'),
}
INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"')
IMPORT = re.compile(r"""^\s*import\b.*?["'](\.[^"']+)["']""")


def line_ref(match, text):
    return text.count("\n", 0, match.start()) + 1


def unique(entries):
    return list({(e["label"], e["line"], e.get("kind")): e for e in entries}.values())


def code_entries(lines, suffix, path):
    entries = []
    patterns = SYMBOL if suffix in CPP else {
        "Function": re.compile(r"^\s*(?:async )?def\s+(\w+)|^\s*(?:export )?(?:async )?function\s+(\w+)"),
        "Class": re.compile(r"^\s*(?:export )?class\s+(\w+)"),
    }
    if path.startswith("tests/vendor/"):
        return entries
    for number, line in enumerate(lines, 1):
        for kind, pattern in patterns.items():
            match = pattern.match(line)
            if match:
                name = next(value for value in match.groups() if value is not None)
                entries.append({"label": name, "line": number, "kind": kind})
    return unique(entries)


def inventory(graph, root, tracked, document, summary, links, resolve, markdown):
    """Keep the wiki hierarchy and add an independently browsable file tree."""
    files = tracked | {path for path in EXPLORER_FILES
                       if path in GENERATED or (root / path).is_file()}
    directories = {folder.as_posix() for name in files for folder in PurePosixPath(name).parents
                   if folder.as_posix() != "."}
    nodes = {node["id"]: node for node in graph["nodes"]}
    children = Counter(PurePosixPath(path).parent.as_posix() for path in files | directories)
    edges = {(e["from"], e["to"], e["type"], e.get("anchor", "")): e for e in graph["edges"]}
    documents = {}
    unresolved = set()

    def connect(owner, target, relation, label, **kwargs):
        key = owner, target, relation, kwargs.get("anchor", "")
        edges.setdefault(key, {"from": owner, "to": target, "type": relation, "label": label, **kwargs})

    for path in sorted(directories):
        folder = PurePosixPath(path)
        node = nodes.setdefault(path, {"id": path, "kind": "directory", "title": folder.name})
        node["summary"] = f"Repository directory. Browse its {children[path]} immediate entries."
        node["fileType"] = "Directory"

    for path in sorted(files):
        file = PurePosixPath(path)
        suffix = file.suffix.lower()
        label = {
            **dict.fromkeys(CPP, "Firmware / C++"), ".md": "Documentation", ".html": "HTML",
            ".py": "Python", ".mjs": "JavaScript", ".js": "JavaScript", ".yml": "Workflow / YAML",
            ".yaml": "YAML", ".json": "JSON", ".csv": "Measurement data", ".pdf": "PDF",
            ".png": "Image", ".kicad_sch": "Circuit schematic",
        }.get(suffix, "Repository file")
        node = nodes.setdefault(path, {"id": path, "kind": "file", "title": file.name})
        node["fileType"] = "Generated artifact" if path in GENERATED else label
        if path in GENERATED:
            node["summary"] = node.get("summary") or f"Generated {label} artifact. Open its repository source for the complete output."
            continue
        if suffix not in CODE and suffix not in markdown and suffix != ".csv" and not path.startswith(".github/workflows/"):
            node["summary"] = node.get("summary") or f"{label} in {file.parent.as_posix()}."
            continue
        text = (root / path).read_text(encoding="utf-8-sig")
        lines = text.splitlines()
        if suffix in markdown:
            _, body, headings = document(text)
            documents[path] = body, {heading["anchor"]: heading["line"] for heading in headings}
            first_title = next((h["title"] for h in headings if h["level"] == 1), file.name)
            node["title"] = first_title
            node["summary"] = summary(body) or f"Documentation for {first_title}."
            outline = [{"label": item["title"], "anchor": item["anchor"], "line": item["line"],
                        "kind": f"Level {item['level']}"}
                       for item in headings]
            if outline:
                node["outlineEntries"] = outline
        elif suffix in CODE:
            description = []
            for line in lines[:10]:
                text_line = re.sub(r"^\s*(?://+|#)\s*", "", line).strip()
                if line.lstrip().startswith(("//", "# ")) and text_line:
                    description.append(text_line)
                elif description and line.strip():
                    break
            node["summary"] = node.get("summary") or " ".join(description)[:400] or f"{label} source at {path}."
            entries = code_entries(lines, suffix, path)
            if entries:
                node["codeEntries"] = entries
            for number, line in enumerate(lines, 1):
                match = INCLUDE.match(line) if suffix in CPP else IMPORT.match(line)
                if not match:
                    continue
                target, _ = resolve(path, match[1])
                if target in files:
                    connect(path, target, "includes", match[1], sourceLine=number)
        elif suffix == ".csv":
            reader = csv.reader(lines)
            headers = next(reader, [])
            node["summary"] = f"Recorded measurements with {len(headers)} columns and {sum(1 for _ in reader)} rows."
            node["outlineEntries"] = [{"label": name, "line": 1, "kind": "CSV column"} for name in headers]
        else:
            name = re.search(r"^name:\s*(.+)$", text, re.M)
            node["summary"] = f"GitHub Actions workflow: {name[1] if name else file.stem}."
            node["outlineEntries"] = [
                {"label": entry[1].strip(), "line": line_ref(entry, text), "kind": "Step"}
                for entry in re.finditer(r"^\s*-\s+name:\s*(.+)$", text, re.M)
            ]

    for path, (body, _) in documents.items():
        for label, raw in links(body):
            target = resolve(path, raw)
            if not target:
                continue
            name, anchor = target
            if name not in nodes:
                unresolved.add((path, raw))
                continue
            target_headings = documents.get(name, ("", {}))[1]
            line = target_headings.get(anchor)
            if anchor and name in documents and not line:
                unresolved.add((path, raw))
                continue
            if name == path:
                continue
            relation = "child" if nodes[name].get("parent") == path else "link"
            connect(path, name, relation, " ".join(label.replace(chr(96), "").split()),
                    **({"anchor": anchor} if anchor else {}), **({"line": line} if line else {}))

    for path, reference in sorted(unresolved):
        nodes[path].setdefault("unresolved", []).append(reference)
    graph["nodes"] = sorted(nodes.values(), key=lambda node: node["id"])
    graph["edges"] = [edges[key] for key in sorted(edges)]
    graph["inventory"] = {"files": len(files), "directories": len(directories),
                          "unresolved": len(unresolved)}
    return graph
