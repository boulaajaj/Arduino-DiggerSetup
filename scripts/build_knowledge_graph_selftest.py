"""Regression checks for hierarchy, source resolution, and generated-file drift."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import subprocess

import build_knowledge_graph as graph
from knowledge_catalog import inventory


class KnowledgeGraphTests(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.root = Path(scratch.name)
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        self.write(graph.HOME, "---\nsources: none\n---\n# Home\n\n[Domain](domain.md)\n")
        self.write("docs/wiki/domain.md",
                   "---\nparent: home.md\nsources: none\n---\n# Domain\n\n"
                   "[Topic](topic.md)\n")
        self.write("docs/wiki/topic.md",
                   "---\nparent: domain.md\nstatus: historical\nsources:\n"
                   "  - firmware/\n---\n# Topic\n\n[Output](../reference.md#output)\n")
        self.write("docs/reference.md", "# Reference\n\n## Output\n\nDescription.\n")
        self.write("firmware/example.cpp", "// Example source\n")
        self.write(graph.TEMPLATE,
                   '<script type="application/json"><!--GRAPH_DATA--></script>\n')
        self.write("unrelated.txt", "One\n")
        subprocess.run(["git", "add", "."], cwd=self.root, check=True)

    def write(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        return path

    def test_hierarchy_and_canonical_references(self):
        data, warnings = graph.build(self.root)
        self.assertFalse(warnings)
        nodes = {node["id"]: node for node in data["nodes"]}
        topic = nodes["docs/wiki/topic.md"]
        self.assertEqual(topic["parent"], "docs/wiki/domain.md")
        self.assertEqual(topic["depth"], 2)
        self.assertEqual(topic["status"], "historical")
        self.assertEqual(nodes["firmware"]["kind"], "directory")
        self.assertIn({"from": topic["id"], "to": "docs/reference.md", "type": "link",
                       "label": "Output", "anchor": "output", "line": 3}, data["edges"])

    def test_parent_cycle_rejected(self):
        domain = self.root / "docs/wiki/domain.md"
        domain.write_text(domain.read_text().replace("parent: home.md", "parent: topic.md"))
        with self.assertRaisesRegex(ValueError, "parent cycle"):
            graph.build(self.root)

    def test_parent_requires_a_real_link(self):
        home = self.root / graph.HOME
        home.write_text(home.read_text().replace("[Domain](domain.md)", "Domain"))
        with self.assertRaisesRegex(ValueError, "parent must link"):
            graph.build(self.root)

    def test_unknown_heading_rejected(self):
        self.write("docs/reference.md", "# Reference\n\n## Renamed\n")
        with self.assertRaisesRegex(ValueError, "unknown heading"):
            graph.build(self.root)

    def test_code_examples_are_not_links(self):
        markdown = "[`title`](real.md) and `[sample](code.md)`\n~~~md\n[example](fenced.md)\n~~~"
        self.assertEqual(list(graph.links(markdown)), [("`title`", "real.md")])

    def test_inventory_covers_unlinked_tracked_paths_and_evidenced_edges(self):
        self.write("docs/notes.md", "# Unlinked note\n\n[Output](reference.md#output)\n")
        self.write("firmware/helper.h", "struct Helper {};\n")
        self.write("firmware/example.cpp", '#include "helper.h"\n\nint adjust(int x) { return x; }\n')
        self.write("firmware/local.h", "// Local scratch file\n")
        subprocess.run(["git", "add", "docs/notes.md", "firmware/helper.h", "firmware/example.cpp"],
                       cwd=self.root, check=True)
        files, _ = graph.list_paths(self.root)

        def build_inventory():
            base, _ = graph.build(self.root)
            return inventory(base, self.root, files, graph.document, graph.summary,
                             graph.links, graph.resolve, graph.MARKDOWN)

        result = build_inventory()
        nodes = {node["id"]: node for node in result["nodes"]}
        self.assertTrue(files <= nodes.keys())
        self.assertNotIn("firmware/local.h", nodes)
        self.assertIn("firmware", nodes)
        self.assertEqual(nodes["docs/notes.md"]["title"], "Unlinked note")
        self.assertIn({"from": "docs/notes.md", "to": "docs/reference.md", "type": "link",
                       "label": "Output", "anchor": "output", "line": 3}, result["edges"])
        self.assertIn({"from": "firmware/example.cpp", "to": "firmware/helper.h", "type": "includes",
                       "label": "helper.h", "sourceLine": 1}, result["edges"])
        self.assertEqual(nodes["firmware/example.cpp"]["codeEntries"][0]["label"], "adjust")
        self.assertEqual(result, build_inventory())

    def test_single_command_and_stale_detection(self):
        def run(*args):
            with patch.object(graph, "ROOT", self.root), \
                 patch("sys.argv", [graph.__file__, "--wiki-only", *args]):
                return graph.main()

        self.assertEqual(run(), 0)
        initial = (self.root / graph.OUTPUT).read_bytes()
        self.assertEqual(run(), 0)
        self.assertEqual((self.root / graph.OUTPUT).read_bytes(), initial)
        self.write("unrelated.txt", "Two\n")
        self.assertEqual(run("--check"), 0)
        topic = self.root / "docs/wiki/topic.md"
        topic.write_text(topic.read_text() + "\n[Home](home.md)\n")
        self.assertEqual(run("--check"), 1)
        self.assertEqual(run(), 0)
        self.assertEqual(run("--check"), 0)


if __name__ == "__main__":
    unittest.main()
