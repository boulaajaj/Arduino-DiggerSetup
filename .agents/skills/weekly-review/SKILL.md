---
name: weekly-review
description: Answer "what should I be on top of this week?" from the person's private Amibou brain: route to open work across every domain, read each node's status and next action, read recent decisions from memory, check a connection for what moved, rank by what is blocked, stale or dated, and write a dated review in the brain with a source on every claim.
context: [*/status, memory/decisions]
connections: [github]
writes: none
artifacts: [reviews]
---

# Weekly review

This capability is Amibou's own. It exists to prove the Capabilities C: one short request, a multi-step workflow, a durable artifact with sources. It does not restate the brain; ranking and omission are the work.

1. **The brain.** Run `amibou bridge where --json`. Continue only on `status: ok`; a read grant is enough for the reading, and writing the review needs the host to allow writes under `brain` (`claude --add-dir <brain>`).
2. **Open work, every area.** Run `amibou cards <brain> --json`: every project's status, next action, open questions and `updated`, whatever the size of the brain (Level 1 of a `brain_query` is bounded to the nodes a question touches, so it is not the list of everything). From it, pick every project whose status is not done, closed or decided; `--attention` alone lists the ones flagged blocked, stale, open or unconfirmed.
3. **Each open project** at Level 2: `brain_expand <id>@2`. Note the status, the next action, any open question, and the `updated` date. Go to `@3` only where a node names a dependency you cannot place.
4. **Decisions not yet reflected.** Call `brain_query` with "what was decided recently" and read the memory nodes it returns; note any commitment no status carries.
5. **What moved outside.** If `amibou connection list <brain>` shows GitHub `connected` or `available`, run `amibou connection query <brain> github "what is open"` and note anything newer than the brain's status for the same thing. Live state is cited as `github:<repo>#<n>`, never copied into Context.
6. **Rank.** Blocked first, then a date within the week, then stale (an `updated` older than 60 days on an open node), then the rest. Omit what needs nothing.
7. **Write the review** to `<brain>/reviews/<date>-weekly-review.md`: a heading, at most eight items, each with what to do, why now, and its source as a path in the brain or a connection record id. End with a "Trace" section: the `brain_query` and `brain_expand` calls made and the deepest level reached. Read it back and give the person the path.

An inference stays an inference in the review; say "unconfirmed". A claim without a source is a failure, not a rough edge. Nothing from the brain lands in this repository.

