# Project Wiki — AI-maintained knowledge graph

A navigation layer over this repo's documentation, built for graph
visualization (Obsidian) and fast orientation. It follows the Karpathy
LLM-wiki pattern adopted in #53: **agents create and maintain these
interconnected notes; humans read and visualize them** (issue #141).

## How to visualize

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
- Start at [home](home.md).
