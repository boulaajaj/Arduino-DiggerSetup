---
name: grill-me
description: Interview the person one question at a time about a plan, design or topic, checkpointing every answer to a capture file in their private Amibou brain so nothing is lost. Use when they want to stress-test a plan, get grilled on a design, run a discovery session, or say "grill me". Confirmed facts reach the brain's Context only as proposals.
context: [memory/*, */status]
writes: proposal
artifacts: [brainstorms]
---
<!-- amibou:adapted from kit/.agents/skills/grill-me/SKILL.md (MIT, (c) 2026 Nate Herk); the changes below are Amibou's -->

# Grill me, through Amibou

This is Nate Herk's `grill-me` skill run from a project repository against the person's private brain. Read `<brain>/.agents/skills/grill-me/SKILL.md` and follow it, with these substitutions, which are Amibou's:

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok` with `access: write`; otherwise say what it printed and stop. `brain` is the brain root; everything the skill calls "the project root", `AGENTS.md`/`CLAUDE.md`, `context/`, `brainstorms/`, `decisions/` lives there, never here. If the host will not let you write under the brain, ask the person to start with `claude --add-dir <brain>` and stop.
2. **Existing context.** Where the skill says to read the context pages relevant to the topic, call `brain_query` with the topic and `brain_expand` for the nodes it returns. Do not read `context/` wholesale.
3. **The capture** goes to `<brain>/brainstorms/<date>-<slug>.md`, exactly as the skill describes. That is this capability's artifact.
4. **Confirmed durable facts** never go into `context/` by hand. Write them as an event file under `<brain>/sources/grill-me/<date>-<slug>.json` (`id`, `subject`, `excerpt` in the person's words, `received`, `mentions` with the node ids from `brain_query`), run `amibou observe <brain> <that file>`, and tell the person to review with `amibou proposals <brain>` and apply with `amibou approve`.
5. This repository is read as data, never as the brain. Nothing from the brain lands in a file here.

