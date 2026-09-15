---
name: level-up
description: Walk the person through Nate Herk's Three Ms to turn one gap in their private Amibou brain into one shipped artifact or one verified repair, when they ask to level up, close an audit gap, or find what to automate next. One run, one artifact. Decisions reach the brain's Context only as proposals.
context: [memory/*, */status]
connections: []
writes: proposal
artifacts: [decisions]
---
<!-- amibou:adapted from kit/.agents/skills/level-up/SKILL.md (MIT, (c) 2026 Nate Herk); the changes below are Amibou's -->

> *Adapted from The Three Ms of AI. (c) 2026 Nate Herk. All rights reserved. The Three Ms of AI is a trademark of Nate Herk.*

# Level up, through Amibou

Nate Herk's `level-up` skill run from a project repository against the person's private brain. Read `<brain>/.agents/skills/level-up/SKILL.md` and its `references/3ms-framework.md`, and follow them with these substitutions, which are Amibou's:

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok` with `access: write`; otherwise say what it printed and stop. `brain` is the root for every path the skill names.
2. **What matters, what is reachable, what was decided.** Where the skill reads `context/priorities.md`, `context/about-me.md`, `connections.md`, `decisions/log.md` and the recent audits, call `brain_query` for each question ("what are my priorities", "what is connected", "what was decided recently") and `brain_expand` where the answer is not at Level 2. Read `<brain>/connections.md` and `<brain>/.amibou/connections.json` for what is reachable.
3. **Capabilities that exist** are `amibou capabilities <this repository>` plus `<brain>/.agents/skills/*/SKILL.md`, not this repository's.
4. **The scoped automation spec** goes to `<brain>/decisions/log.md` as the skill says: that is this capability's artifact. A new skill the person chose to build goes into the brain unless they say otherwise, and carries the capability frontmatter from `docs/CAPABILITIES.md` (`context`, `connections`, `writes`).
5. **A durable decision** for Context goes through a proposal: write it as an event under `<brain>/sources/level-up/<date>-<slug>.json` and run `amibou observe`; never edit `context/` by hand.

