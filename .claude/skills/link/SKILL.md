---
name: link
description: Make this repository, a file, a folder or a URL findable from the person's private Amibou brain with the smallest useful routing edit, when they say "link this into my brain" or "add this to my routing". Writes one route in the brain's operating manual, never in this repository's.
context: [*]
writes: proposal
---
<!-- amibou:adapted from kit/.claude/skills/link/SKILL.md (MIT, (c) 2026 Nate Herk); the changes below are Amibou's -->

# Link, through Amibou

Nate Herk's `link` skill, run from a project repository with the brain as the target. Read `<brain>/.claude/skills/link/SKILL.md` and follow it with these substitutions, which are Amibou's:

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok` with `access: write`; otherwise say what it printed and stop. The "intended project root" whose manual gets the route is `brain`, never this repository.
2. **The target** is this repository's root as an absolute path, or the file, folder or URL the person named. Its purpose comes from the argument, else from this repository's `README.md` first line. Verify it exists; invent nothing.
3. **The route** lands in the brain's `AGENTS.md` and `CLAUDE.md` under "Other worlds" (created if missing), in both manuals where they differ. This repository's own manuals are not the target.
4. **Read back** manual, index, target, as the skill says, and report in two lines.

