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

## 3. Review rounds (repeat until quiet)

1. Mark ready; request Copilot (`gh api -X POST .../requested_reviewers`) and
   comment `@coderabbitai review`.
2. For each finding: VERIFY the claim against the code first (bots have been
   wrong — and right — here; never apply a behavior-changing suggestion, see
   `.claude/rules/behavior-preservation.md`).
3. Push the fix, reply to the thread with the commit SHA + reasoning, THEN
   resolve it (GraphQL `resolveReviewThread`). Declined suggestions get the
   rationale in the reply; the thread still resolves.
4. Re-request both bots after every round — a round only closes when they
   have re-reviewed the fix commit.

## 4. Merge (autonomous protocol, operator-authorized 2026-07-12)

- Conditions: 3 consecutive clean bot cycles ~5 min apart on the head commit
  (a consumed re-request with no new findings counts as quiet), all CI green,
  0 unresolved threads.
- `gh pr merge --squash --admin --delete-branch`, then update local `main`
  and proceed to the next board item.
- A behavior-affecting or safety-relevant change is NEVER auto-merged —
  operator sign-off first.
