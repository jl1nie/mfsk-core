# Agent workflow: which model runs the loop, and when to delegate

Status: standard for assistant sessions in this repo, adopted 2026-10-25. Replaces the "every change goes to
`runner`, every diff to `reviewer`" rule of d4d0164. Source of the criteria: Anthropic, *Building agents: when to
fold subagents into your main agent loop*
(<https://claude.com/ja/resources/articles/building-agents-when-to-fold-subagents-into-your-main-agent-loop>).

## 1. Why the old rule cost more than it saved

The article's finding, from enterprise deployments: a **single agent loop with progressively disclosed context**
(short pointers, full text loaded only when needed) usually beats a split into subagents on quality, cost and
latency. Each handoff is a nested model loop, and the summary that comes back loses detail; an agent that saw
every step can connect them, a stack of summaries cannot.

Mapped onto this repo, the d4d0164 rule paid for every change three times:

1. the Opus main session read the code to write the spec (Opus tokens),
2. `runner` re-read the same files to carry it out (Sonnet tokens),
3. `reviewer` re-read the code and the diff (Opus tokens again), and the main session then read both reports.

For a small change, (1) + (3) alone exceed what Sonnet doing the whole job in one loop would cost, and the
Opus main session stayed Opus for the whole conversation. The cost lever is **the model running the main loop**,
not the number of agents.

## 2. Pick the main-loop model by task

Default: start sessions with `/model opusplan` (Claude Code alias: Opus while in plan mode, Sonnet when
executing). Design happens in plan mode with Opus; once the plan is approved the same loop, with the same
context, implements on Sonnet. No spec handoff, no re-reading.

| task | main loop | why |
|---|---|---|
| implement an approved plan / spec, refactor, test or CI fix, docs, CHANGELOG, `.ja.md` twin, release metadata | **Sonnet** | the decisions are made; this is reading and editing volume |
| port an upstream change classified A/B whose Fortran diff is local (one routine) | **Sonnet**, Opus plan first if it touches `engine/**` | fidelity is checked by tier A+B, not by the model |
| design: new protocol, public API shape, `engine/**` change shared by several protocols, fixed-point numerics | **Opus** (plan mode) | wrong design is the expensive failure |
| diagnosis: recall / precision / SNR moved and the cause is not yet measured | **Opus** | ranking hypotheses (`CLAUDE.md` §2.3) is the hard part |
| upstream triage (`scripts/upstream-diff.sh`, A/B/C classification) | **Opus** | judgement over many small diffs |
| release step 0, status questions, "where is X" | **Sonnet** (or Haiku via `Explore`) | lookup |

Switch inside a session with `/model` when the task changes kind; the context stays. Use a lower effort level
for mechanical work.

## 3. When to spawn a subagent

Spawn only when one of the article's three conditions holds. Each maps to a concrete case here:

| condition (article) | here | agent |
|---|---|---|
| **context bloat** that would degrade the main loop | tier C sweeps, the full merge-gate log, benchmark runs, `scripts/pre-push-check.sh`, embedded flash logs: thousands of lines where only the numbers matter | `runner` (Sonnet), returns numbers and failures only |
| **independent parallel work** with a context that can be thrown away | the same mechanical change across several protocols; measurements on several modes; sweeps per mode | several `runner`s in one message |
| **a read that must not be anchored on the author's** | review of a decoder-internal / FEC / LLR / SNR / `engine/**` change before it lands | `reviewer` (Opus), given the diff and spec, not the author's conclusion |

Do **not** spawn when:

- the change is a handful of edits you can make in the main loop (the brief costs more than the work);
- your next step depends on details the summary would drop (diagnosis, design);
- the work is a lookup whose target you already know;
- you want a different model for an implementation: switch `/model` instead. Never implement through a `fork`
  (it inherits the main model).

## 4. Review: when `reviewer` is required

`reviewer` (Opus) is required before landing a change to:

- decoder internals of any protocol, `engine/**`, FEC / LLR / OSD, SNR formulas, sync, TX waveform / envelope;
- a new decode strategy (SIC rounds, `sic_early`, …) — its precision guard is the first thing checked;
- `fixed-point` numerics on the shipped embedded path.

Not required (the hooks and tier A+B are the gate): docs, CHANGELOG, tests-only changes that do not lower a
floor or raise a phantom budget, CI / scripts, bindings glue, skimmer UI, release metadata. Anything that lowers a
recall floor or raises `max_extra` is a decoder change for this purpose.

## 5. Token hygiene (any model)

- Load on demand: `CLAUDE.md` is the index; read the linked note when the task needs it, not up front.
- Large files (`CHANGELOG.md`, `docs/notes/BENCHMARKS.md`, `ROADMAP.md`, `docs/historical/**`): `grep -n` then
  read a range; never read them whole.
- Long commands: pipe through `tail -n 40` or `grep -E 'FAILED|error|panicked|test result'`; run one test binary
  (`--test <binary>`) while iterating, the full merge gate once before landing.
- Batch independent tool calls in one turn.
- Check `/cost` at the end of a task when trying a new way of working; compare like with like (§6).

## 6. What the harness enforces, and what is advice

`CLAUDE.md` is context the model reads, not a gate: under a long context or a pressing task it can be skipped. A
rule that matters is moved to a layer the harness executes:

| rule | layer | enforced? |
|---|---|---|
| default main model `opusplan` | `.claude/settings.json` `model` | yes at session start; a model picked explicitly (`/model`, `--model`, the session's model picker in the app) overrides it |
| `runner` on Sonnet, `reviewer` on Opus | `model:` in `.claude/agents/*.md` | yes |
| no `fork`, no `model` override on `runner` | PreToolUse hook on `Agent`: `.claude/hooks/agent-guard.sh` (denies the call) | yes |
| fmt / clippy / rustdoc / feature matrix | `.githooks` pre-commit / pre-push | yes, once `scripts/install-hooks.sh` ran |
| when to spawn (§3), when to switch model (§2), review required (§4) | this note | advice — judgement calls a hook cannot make |

Before adding a rule to `CLAUDE.md`, ask whether it is mechanical (a path, a flag, a tool argument). If it is,
it belongs in a hook or a script, and `CLAUDE.md` only points at it.

## 7. Changing this standard

Measure, as for code (`CLAUDE.md` §2.3): pick a handful of representative tasks (a doc fix, a CI fix, a one-routine
upstream port, a decoder change with review), record `/cost` and the outcome (did the merge gate pass first time,
did review find a real defect), change one thing, compare. Run a task more than once before concluding: the
article's example regression showed up in one run of two.
