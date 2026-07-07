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
  link to the canonical source. Constants, thresholds, pin numbers, and
  versions live ONLY in the canonical docs (`CLAUDE.md`, `docs/SAFETY.md`,
  …) — a wiki note that restates a number is a bug.
- **Agents own this folder.** A PR that adds, removes, or renames a doc — or
  changes what a subsystem IS — updates the affected wiki notes in the same
  PR. Link fixes are cheap; that is the point of keeping content out.
- Start at [home](home.md).
