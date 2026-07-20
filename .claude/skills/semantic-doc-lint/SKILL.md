---
name: semantic-doc-lint
description: The periodic semantic health review of the knowledge base — the judgment half that structural lint cannot do. Use when the monthly reminder issue appears, when the operator asks whether the docs still tell the truth, or after any large refactor lands. Read-only findings; fixes go through issues.
---

# Semantic documentation lint — does the prose still describe the code?

Structural lint (`scripts/check_wiki.py`) proves the graph is healthy;
the change-impact check (#199) proves changed areas were considered.
Neither can judge whether a sentence is still TRUE. This procedure is that
judgment pass. It reports; it never fixes inline.

## 1. Walk the declared sources

Every wiki note declares its `sources:` frontmatter (#202). For each note:
read the note, read its sources, and ask of every claim that overlaps the
source: is this still what the code/doc says? The two prior precedents to
look for by class:
- **Superseded claims** — code moved on, prose did not (the pre-#185
  "tunables live in CLAUDE.md" class).
- **Described-but-removed** — the note explains something that no longer
  exists (the retired module-sections class).

## 2. Cross-page contradiction sweep

Compare notes that share a topic (the [authority-matrix](../../../docs/wiki/authority-matrix.md)
rows are the topic list). Two pages answering the same question
differently is a finding even when each cites a real source.

## 3. Coverage gaps

Grep the last month of `docs/DECISION-LOG.md` entries and merged PR titles
for concepts mentioned repeatedly that have NO wiki page — recurring
nouns without a home are the strongest signal a page is missing.

## 4. Report — never fix inline

For each finding: file ONE issue (label `documentation`) stating the page,
the claim, what the source actually says, and the suggested correction.
Then record a one-line receipt in `docs/DECISION-LOG.md`: date, notes
walked, findings count, issue numbers (zero findings is also a record).
Behavior-relevant discoveries follow the covenant's severity triage —
a doc claiming something SAFER than the code does is operator-escalation,
not just an issue.

## Per-PR advisory layer (already standing)

Between periodic runs, the same judgment runs in miniature on every PR:
the `documentation-reviewer` agent in the pre-request self-review
(prepare-pull-request skill, step 1) plus the #199 receipt requirement.
This skill is the deep pass those quick passes cannot afford.
