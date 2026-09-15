---
name: audit
description: Audit the person's private Amibou brain and score its Four Cs out of 25 each, find stale nodes, unrouted projects, missing provenance, mismatched manuals and the startup cost of its capabilities, when they ask to audit their brain or check migration readiness. Saves a dated report in the brain. Changes nothing it inspects.
context: [*]
writes: none
artifacts: [audits]
---
<!-- amibou:adapted from kit/.claude/skills/audit/SKILL.md (MIT, (c) 2026 Nate Herk); the changes below are Amibou's -->

# Audit, through Amibou

Nate Herk's `audit` skill run from a project repository against the person's private brain. Read `<brain>/.claude/skills/audit/SKILL.md`, its `rubric.md`, `history.md` and `compatibility.md`, and follow them with these substitutions, which are Amibou's:

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok`; a read grant is enough. `brain` is the audited project; this repository is not.
2. **Breadth is the point of an audit**, so this is the one capability permitted to walk the index rather than route through it: call `brain_query` once for the landscape (Levels 0 and 1 come back with every answer) and `brain_expand <id>@2` for the nodes you need to inspect; read `<brain>/.amibou/index.json` directly if the host allows it. Do not read every page.
3. **Evidence Amibou already computes**, to use rather than redo: `amibou brain validate <brain>` (orphans, unsourced inferences, duplicate ids), `amibou build <brain> --dry-run` (Level 1 cost), `amibou capabilities <this repository>` and the same for the brain (startup cost of the installed set, capabilities missing their contract), `amibou connection list <brain> --verify` (connection states), `amibou proposals <brain>` (unreviewed observations), and `<AMIBOU_HOME>/query.log` for retrieval that reached Level 4 every time.
4. **Score with the rubric**, and add these findings under Context (stale by `updated`, inference never confirmed, no sidecar), Connections (state), Capabilities (startup cost past 200 tokens each, `writes` missing) and Cadence (no manual run recorded for a scheduled capability). No fifth score.
5. **The report** goes to `<brain>/audits/audit-<date>.md`, read back, as the skill says. That is the only write. The inspected system remains unchanged; say "No operating manuals were changed."

