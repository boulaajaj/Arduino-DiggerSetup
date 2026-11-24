---
parent: agent-governance.md
sources: none
---

# Project Wiki — AI-maintained knowledge graph

A navigation layer over this repo's documentation, built for graph
visualization (Obsidian) and fast orientation. It follows the Karpathy
LLM-wiki pattern adopted in #53: **agents create and maintain these
interconnected notes; humans read and visualize them** (issue #141).

## Knowledge structure

- [Home](home.md) is the main index. Domain indexes group related clusters
  and topics. Topic notes can link across clusters while keeping one parent.
- Firmware, tests, and canonical docs are the source layer. Notes summarize
  behavior and link to the evidence. The generated explorer also offers a
  separate file tree for browsing the entire checkout.
- For maintenance, follow this guide and the [project rules](../../CLAUDE.md).
  Read the index before starting a query, update affected notes and links when
  ingesting evidence, and record substantive changes in the
  [wiki maintenance log](log.md). Run lint after updating the notes and resolve
  any semantic discrepancies against the canonical source.

This adapts [Andrej Karpathy’s LLM Wiki pattern](https://gist.github.com/karpathy/442a6bf555914893e9891c11519de94f)
to a source-controlled firmware project.

## How to visualize

From the repository root, run:

```sh
python scripts/build_knowledge_graph.py
```

Open `docs/knowledge-explorer-kit.html` in a browser. The standalone file includes
both views and works offline. GitHub and VS Code links open their respective
destinations. Set the local checkout path in the viewer to enable VS Code links.
The revision displayed in the viewer identifies the graph inputs, so unrelated
commits do not invalidate the HTML.

### Obsidian

1. Open **the repository root** as an Obsidian vault (File → Open folder as
   vault). The graph then includes both these hub notes and the canonical
   docs they link to.
2. `.obsidian/` (your local vault settings) is gitignored — theme and graph
   tweaks stay on your machine.
3. Notes use standard relative Markdown links, so everything here also reads
   normally on GitHub. Obsidian graphs Markdown links natively.

## Rules for this folder (anti-drift)

- **Navigation layer, not a second copy.** Notes describe stable concepts and
  link to the canonical source — which artifact is canonical for what is
  stated once in [authority-matrix](authority-matrix.md). Tunable values —
  constants, thresholds, pin numbers, versions — live ONLY in their
  authority (the config layer for tunables); a wiki note that restates one
  is a bug. Stable
  identity is fine: component model names and ordinary descriptive
  quantities ("three-position switch") make notes readable and only change
  when the thing itself changes.
- **One parent per topic.** Every Markdown note except `home.md` declares a
  relative `parent` path in frontmatter. The parent must link to the note using
  a standard Markdown link. All parent chains reach Home. Additional links
  connect related topics across the hierarchy. Mark dated context with
  `status: historical`; `status: current` is optional.
- **Agents own this folder.** A PR that adds, removes, or renames a doc — or
  changes what a subsystem IS — updates the affected wiki notes in the same
  PR. Link fixes are cheap; that is the point of keeping content out.
- **Every note declares its sources** (#202): a `---` frontmatter block with
  a `sources:` list of the repo paths the note navigates to (`sources: none`
  for pure-navigation pages). Declared paths must exist — a vanished source
  fails the lint, which is the point: the note must be reconsidered.
- **CI enforces this** (`wiki-lint` workflow, `scripts/check_wiki.py`,
  #145/#202): broken links and anchors, orphan notes, notes unreachable
  from [home](home.md), tunable-looking values, missing or invalid sources
  frontmatter, duplicate titles, and deprecated phrases all fail the build.
  The same job checks parent chains and whether the HTML matches its inputs.
  Stage new files before building so the generator includes them. Keep new
  hierarchy links in the existing relative Markdown syntax.
- Start at [home](home.md).
