---
name: prepare-pull-request
description: The full PR lifecycle procedure for this repo — use when starting any new work item, opening a PR, handling review rounds, or merging. Encodes issue-first, the board rules, the review-thread protocol, and the autonomous merge protocol.
---

# Prepare pull request — issue to merge

## 1. Before any code

- An open GitHub issue must exist for the work (issue-first, no exceptions;
  no issue → conversation only). One PR ↔ one issue.
- Set the issue to **In Progress** on Project #1 (manual today).
- Branch from fresh `origin/main`: `agent/<role-tag>/<description>`.
- Never push to a merged/closed PR's branch — check `gh pr view --json state`
  before the first push of a session.

## 2. Opening

- First commit + **draft PR immediately** (earliest board-visible signal);
  body carries `Closes #N`, summary, role tag, test plan.
- Gates before every push: host suite (`wsl -e make -C tests run`) and local
  `arduino-cli` compile when firmware is touched; run the
  `wiki-impact-review` skill and include its receipt.

## 3. Self-review FIRST, then review rounds

1. **Before the first bot request**: run the applicable reviewer agents
   (`.claude/agents/` — architecture, safety, tests, documentation) on the
   diff, plus a **stale-reference sweep**: grep for every name, count,
   path, and claim the diff makes stale elsewhere (docstrings, docs,
   cross-references) and fix what surfaces. Self-review must match or
   exceed the bots. Never write derived values (counts, line numbers) in
   prose — the artifact is the inventory.
2. Mark ready; request Copilot —
   `gh api -X POST repos/<owner>/<repo>/pulls/<PR>/requested_reviewers -f 'reviewers[]=copilot-pull-request-reviewer[bot]'`
   — and comment `@coderabbitai review`. After every fix commit, repeat
   from step 1 (self-review the fix, then one request); never re-request
   on unchanged code.
3. For each finding: VERIFY the claim against the code first (probe and
   refute with evidence when wrong; never apply a behavior-changing
   suggestion, see `.claude/rules/behavior-preservation.md`).
   **Scope guard (operator 2026-07-19):** a suggestion that EXPANDS the
   change's scope beyond its issue is declined into a follow-up issue by
   default — one intent per PR extends to review rounds.
4. Push the fix, reply to the thread with the commit SHA + reasoning, THEN
   resolve it (GraphQL `resolveReviewThread`). Declined suggestions get the
   rationale in the reply; the thread still resolves.

## 4. Merge (autonomous protocol, operator-amended 2026-07-19)

- Conditions: **ONE clean bot cycle on the final commit** — the single
  request from step 2 returns no new findings (a silently consumed request
  counts) — plus all CI green and 0 unresolved threads. Do not re-invite
  reviews of unchanged code.
- `gh pr merge --squash --admin --delete-branch`, then update local `main`
  and proceed to the next board item.
- No hard round cap, but a repeating finding pattern gets root-caused
  (retrospective + operator decision), never re-fixed round after round.
- A behavior-affecting or safety-relevant change is NEVER auto-merged —
  operator sign-off first.
