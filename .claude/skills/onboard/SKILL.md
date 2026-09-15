---
name: onboard
description: Set up or refresh the person's private Amibou brain on Day 1, when they say "set me up", "onboard me" or "fill in my brain": the seven-question intake into aios-intake.md and the Day-1 file set, run against the brain from this repository. The Day-1 context pages land as proposals the person approves. Idempotent.
context: [*]
writes: proposal
artifacts: [decisions]
---
<!-- amibou:adapted from kit/.claude/skills/onboard/SKILL.md (MIT, (c) 2026 Nate Herk); the changes below are Amibou's -->

# Onboard, through Amibou

Nate Herk's `onboard` skill run from a project repository against the person's private brain. Read `<brain>/.claude/skills/onboard/SKILL.md` and follow it with these substitutions, which are Amibou's:

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok` with `access: write`; otherwise say what it printed and stop. Every file the skill reads or writes (`aios-intake.md`, `context/`, `connections.md`, `decisions/log.md`, the manuals) lives under `brain`. This repository is not the AIOS and gets nothing.
2. **Before the interview**, call `brain_query` with each of the seven questions' topics, so answers already in the brain are reused and not asked again.
3. **The intake and the registry** (`aios-intake.md`, `connections.md`, the `decisions/log.md` entry) are written under the brain as the skill says. **The Day-1 context pages are not written by hand.** For each page the scaffold would create, write an event under `<brain>/sources/onboard/<date>-<slug>.json` with `proposeNode: { kind, title, body, path: "context/<name>.md" }`, the body being the person's own answers, then run `amibou observe <brain> <that file>`. Tell the person to review with `amibou proposals <brain>` and apply with `amibou approve`; a page that already exists is never overwritten.
4. **After approval**, run `amibou build <brain>` so the index reflects the new pages, and tell the person to `amibou project <brain> --serve` to see them.

