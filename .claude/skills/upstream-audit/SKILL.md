---
name: upstream-audit
description: Audit what changed in WSJT-X (WSJTX/wsjtx release/3.x) since the ported version and classify each change A/B/C. Use at the release-cycle midpoint (~day 7 after a tag), when a new upstream v3.x tag appears, or when the user asks what upstream changed.
---

# Upstream audit

Procedure of record: `docs/notes/RELEASE_PROCEDURE.md`, section "Following WSJT-X upstream". Previous
audits: #435 (v3.2.0-rc1), #642 (rc1 -> v3.3.0-beta1).

## 1. Run the script

```sh
scripts/upstream-diff.sh                     # DEFAULT_BASE..newest v* tag on `upstream`
scripts/upstream-diff.sh <base> <target>     # explicit pair
```

`WSJTX_DIR` overrides the clone location (default `../WSJT-X`). It fetches the `upstream` remote
(`WSJTX/wsjtx`), not `origin` (stale since 2024-03), and compares **trees**: a later tag need not contain an
earlier one, so commit counts across tags mean nothing.

## 2. Classify every changed, cited file

For each file the script lists, run the `git diff` it prints and read the hunks:

| class | meaning | action |
|---|---|---|
| A | alters decode/synth results | port |
| B | real bug fix | port |
| C | refactor, plumbing, GUI | record, don't port |

Most of a version's diff is C. A changed file the script lists as **uncited** is either deliberately not
ported (SuperFox, FT2, GUI; see CLAUDE.md "Scope") or a missing `Ported from` citation; add the citation if
the code is ported.

Triage is judgement over many small diffs: run it on an Opus main loop and read the hunks yourself
(`docs/notes/AGENT_WORKFLOW.md` §2, §2a). Don't hand the classification to a subagent; a survey that only
returns `file:line` pointers can go to `Explore`.

## 3. Record

Open or update an Issue with the A/B/C table, the base/target tags, and what was **not** read. Port A and B in
separate PRs, each a `Mode: port` plan naming the upstream `file:line` and the tier A/B tests that judge it
(each reads the Fortran first and keeps the citation valid; a deliberate divergence gets a comment, and then
needs `reviewer`, `AGENT_WORKFLOW.md` §4). After the ports merge, bump `DEFAULT_BASE` in `scripts/upstream-diff.sh` and say so in the Issue,
and update the reference line in CLAUDE.md ("Following WSJT-X").
