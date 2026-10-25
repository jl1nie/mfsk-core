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
| **a read that must not be anchored on the author's** | a change the tests cannot judge (§4) | `reviewer` (Opus), given the diff and spec, not the author's conclusion |

Do **not** spawn when:

- the change is a handful of edits you can make in the main loop (the brief costs more than the work);
- your next step depends on details the summary would drop (diagnosis, design);
- the work is a lookup whose target you already know;
- you want a different model for an implementation: switch `/model` instead. Never implement through a `fork`
  (it inherits the main model).

## 4. Review: when `reviewer` is required

The criterion is **whether the gate can judge the change**, not which directory it touches. Tier A (bit-exact
parity, roundtrip) and tier B (`assert_golden`: recall floor + phantom ceiling + SNR) already catch a wrong
upstream port; a second Opus read of a change the tests fully cover buys little.

**Not required** — the hooks and tier A+B are the review:

- an upstream port that mirrors the Fortran / C change line for line (parameter, threshold, table, loop bound),
  with the `Ported from…` pointer updated to the new upstream `file:line`, and tier A+B green with no floor or
  budget touched. The author's check: diff the Rust against the upstream diff once, by hand;
- a bug fix with a test that failed before and passes after;
- docs, CHANGELOG, CI / scripts, bindings glue, skimmer UI, release metadata, tests-only changes.

**Required** — the change is one the tests cannot judge:

- the gate itself moves: a recall floor lowered, `max_extra` raised, a golden fixture regenerated, an assertion
  loosened or deleted;
- a deliberate divergence from WSJT-X, or code with no upstream counterpart;
- a new decode strategy or a new path into an existing one (SIC rounds, `sic_early`, …): its precision guard is
  what gets reviewed;
- `fixed-point` / embedded runtime path (host CI does not compile `embedded-poc/**`);
- a structural change in `engine/**` shared by several protocols (not a constant inside it);
- a public API change (`Decoder<P>`, `DecodeParams`, the C ABI).

When unsure, ask the one question: "if this were wrong, which test would fail?" If you can name it and it ran,
skip the review.

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
| four minimal feature sets build under `-D warnings` when `mfsk-core/src` or its manifest is staged | `.githooks/pre-commit` (skip with `PRECOMMIT_MINCHECK=0`) | yes |
| "is it green": one verdict line, honest exit status, a clean-checkout run of the gate | `scripts/verify.sh` | only when run: the author must read its last line (§8.1) |
| "did CI pass", with the jobs that matter for the change named | `scripts/ci-wait.sh [sha] [--need REGEX]` | only when run (§8.1) |
| when to spawn (§3), when to switch model (§2), review required (§4) | this note | advice — judgement calls a hook cannot make |

Before adding a rule to `CLAUDE.md`, ask whether it is mechanical (a path, a flag, a tool argument). If it is,
it belongs in a hook or a script, and `CLAUDE.md` only points at it.

## 7. Changing this standard

Measure, as for code (`CLAUDE.md` §2.3): pick a handful of representative tasks (a doc fix, a CI fix, a one-routine
upstream port, a decoder change with review), record `/cost` and the outcome (did the merge gate pass first time,
did review find a real defect), change one thing, compare. Run a task more than once before concluding: the
article's example regression showed up in one run of two.

## 8. Verification discipline

What went wrong in the 2026-10-25 sessions (#693, #692, `PeriodBudget`, #696) was rarely the design: it was a
check that was run and misread, run on the wrong thing, or not run. Each rule below names the mechanism.

**8.1 The verdict is one line.** Run `scripts/verify.sh` directly (`run_in_background` with nothing around it, no
`cmd > log; echo $?`): its **last line** is `VERIFY PASS` or `VERIFY FAIL: <stage>`, and its exit status is the
verdict's, so a completion notification's exit code means what it says. A wrapper's exit code is the wrapper's: a
background `bash pre-push-check.sh > log; echo "exit $?" >> log` once reported "completed (exit code 0)" for a script
that had exited 101. After a push, `scripts/ci-wait.sh [sha] --need 'Kotlin|Swift'` does the same for CI; `--need`
pins the jobs the change depends on (an absent job is a failure), because the Kotlin and Swift sources are compiled
nowhere else.

**8.2 Passing on this disk is not passing.** Tier B tests use vendored data only (`CLAUDE.md` §4). A test that reads
a git-ignored or generated path (`embedded-poc/assets/*_sweep/*.wav`) passes here and fails in CI, where
`MFSK_REQUIRE_CORPUS` refuses to skip. That turned `main` red from `dfde0a05`, with the author's gate, the
reviewer's run and pre-push all green. `scripts/verify.sh`'s `clean` stage runs the merge gate on a throwaway
`git worktree` of HEAD: **commit first**, it tests HEAD. A test that needs a generated corpus is tier C and says so
(`#[ignore]`); to cover the same property in tier A, synthesise the signal in the test with deterministic noise.

**8.3 A subagent's report is a lead, not a fact.** Before a number or a behaviour from a survey goes into a doc, an
issue comment or the CHANGELOG, re-run the one command that shows it. The #696 survey said an unbudgeted call
leaves `BudgetReport` at its default; FT8's default strategy fills `stages_run` and `rows_subtracted` with no budget
(`stages_run: 2184`), and the wrong sentence was published on the issue.

**8.4 A claim in a doc has a test or a measurement behind it; a new public field has three lines in its plan.** The
three: its definition (per call, or per period?), who sets it, and the one test that fails if the definition is
wrong. `SlotResult::candidates` was documented as "the same whether the audio came whole or as the prefixes' last
call"; it is not (FT8's `SicEarly`: 480 whole, 338 at the end), and nothing asserted it until review.

**8.5 A scripted bulk edit is followed by a build, and an insertion beside an attribute is read.** A Python
substitution that inserted `pub use budget::PeriodBudget;` between a `#[cfg(...)]` and the `pub use` it gated
broke every build without wspr / jt9 / jt65 / q65. Put an insertion next to an attribute in with `Edit`, after
reading the line above; the pre-commit minimal feature checks (§6) catch the rest in seconds.

**8.6 Waiting is not a stopping point.** Start `verify.sh` or `ci-wait.sh` in the background and do the work that
does not depend on it; list that work before the push, not after. Stop only for a decision that is the user's.

**8.7 The reviewer brief.** State the intent in two sentences; list what to check (the labels set at each site,
the cfg matrix, the docs' claims against the code); **give the verification log path and say not to rerun the gate**
(rerun only a command whose result is doubted); and say what was not verified (Kotlin, Swift, embedded flash), so
the reviewer reads those as a compiler would. `.claude/agents/reviewer.md` carries the standing parts.

