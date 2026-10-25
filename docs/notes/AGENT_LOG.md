# Agent log

One row per finished task, written by `scripts/agent-log.sh` (see `docs/notes/AGENT_WORKFLOW.md` §7). Tokens are measured
from the session transcript and its subagents: input + cache creation + output per model, cache reads in parentheses.
`?` means not recorded, never guessed. The whole transcript of a session is counted, so a session that did several
tasks charges each row with everything up to that point: compare rows from separate sessions, or read the difference
between rows of one. Reviewed at each release (`scripts/agent-log.sh --summary <last tag>`): at most one process change,
then compare.

| date | task | mode | commits | tokens by model | session | gate first time | review found | fixes after push | user corrections | note |
|---|---|---|---|---|---|---|---|---|---|---|
| 2026-10-25 | #692 #693 PeriodBudget #696 #697-item1 (one session, 2026-10-25) | port+design | 16 | claude-opus-5-5: 1835k (+36.5M cached); claude-sonnet-5-5: 2251k (+261.4M cached) | 53889a44 | ? | doc contradictions + weak test legs (#697) | ? | ? | seed row: whole session, several tasks, not separable |
