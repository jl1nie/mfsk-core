# Releases, cadence and following WSJT-X

Moved out of the repo-root `CLAUDE.md` (2026-10-12): the release sequence, the tier-C sweeps before a tag, the biweekly cadence, and `scripts/upstream-diff.sh`. Run `scripts/release-status.sh` first, always.

## Releases — go through CD, never `cargo publish` locally

`mfsk-core` ships to crates.io via `.github/workflows/release.yml`,
triggered by a `vX.Y.Z` tag push. The workflow gates the publish on
the CI for the same commit going green (`wait-for-ci` job), then
runs `cargo publish -p mfsk-core --features full` + builds the
`mfsk-ffi` artifact + creates the GitHub release with the attached
tarball.

`wait-for-ci` checks two things, not one. The workflow-run poll
tolerates `skipped` (a docs-only push legitimately skips the build
matrix), which is too wide for the job carrying the golden tier-B
assertions — so a second step requires `Test (tier A+B …)` to have
concluded `success` at *job* granularity. Together with
`MFSK_REQUIRE_CORPUS=1` in `ci.yml`, a green `Test (tier A+B …)` is a
positive statement that the golden tests ran against real recordings
rather than skipping. Before the recordings were vendored they did
skip, silently, for five protocols.

**Do not `cargo publish` from a local clone.** crates.io publishes
are irreversible — once a version is up, you cannot unpublish
(yank exists but blocks new dependents while existing dependents
keep using the broken version). A local publish bypasses the CI
gate that this whole workflow exists to enforce; even if your
local SHA happens to be CI-green, future releases get sloppier
when "just publish locally" is in the playbook. Push the tag and
let CD do it.

**Step 0, before anything else: `scripts/release-status.sh`.**

It prints the release state computed from the repository — whether the
workspace version is tagged, whether the CHANGELOG's top section agrees
with it, how the cadence stands, and which protocols' own source has
changed since `sweep-baseline.json` was last refreshed, with the commit
subjects so "clippy drift" and "decoder change" are distinguishable at a
glance.

This exists because release preparation kept failing the same way: the
state was reconstructed by hand, from memory and inference, and every
fact involved is mechanically derivable. Three errors in one session on
2026-08-23 — claiming sweeps were outstanding when a note recorded them
as done with an explicit re-run condition that was never evaluated;
naming three protocols as needing re-sweeping on the strength of commits
that predated the sweep; and not noticing that `0.10.0` had its
CHANGELOG written and its version bumped but no tag, through an entire
conversation about preparing a release. The procedure below was already
written down. What was missing was anything that computed the state, so
each attempt re-derived it, and re-derivation is where the errors came
from.

For the same reason, don't take a release-state claim from this file
either: as of 2026-08-24 the script reports `0.10.0` bumped, its
CHANGELOG section written and current, and **no tag** — but run it rather
than quoting that sentence.

Sequence:
1. Merge release PR into `main`.
2. `git checkout main && git pull`.
3. **Run the tier-C sensitivity sweeps** (see below).
4. `git tag vX.Y.Z <merge-sha>` then `git push origin vX.Y.Z`.
5. Watch the Actions tab for the `Release` workflow.


### Step 3: tier-C sensitivity sweeps, before every tag

**Which ones, from `scripts/release-status.sh` — not from recall.** It
compares each protocol's own source tree against the date
`sweep-baseline.json` was last refreshed and prints the commit subjects,
so a clippy sweep and a decoder change are told apart without going to
look them up.

```sh
scripts/release-status.sh                      # says which, and why
scripts/run-sensitivity-sweeps.sh fst4         # then just those
```

CI never runs these. They need the generated corpora under
`embedded-poc/assets/*_sweep/` (~17 GB, gitignored, built by
`scripts/gen_*_sweep_wavs.sh` on top of WSJT-X's Fortran simulators),
so on CI every one of them silently skips. A release is the interval
where that gap actually matters — shipping a sensitivity regression to
crates.io is the outcome the sweeps exist to prevent, and it is
irreversible once published.

They **assert nothing** by design: sensitivity is a curve, and a
threshold that moved 0.3 dB is a judgement call rather than a boolean.
The script now diffs itself: every recall-vs-SNR sweep it runs dumps a
per-trial CSV to `target/sweep-csv/`, and at the end it runs
`scripts/sweep-regression-check.py` against those CSVs, which
interpolates each channel's 50%-crossing SNR and prints the delta from
`docs/notes/sweep-baseline.json`, flagging `!!` on any move
`>= 0.5 dB`. It also prints a **precision** table for ft8/ft4/fst4: the
unexpected decodes (any message other than the one injected) per group,
compared over the SNR cells the run shares with the baseline and flagged
`!!` when they rise by `>= 3` and `>= 1.5x`. This replaced manually eyeballing the printed tables (or
spawning several agents to do it) against `docs/notes/*BENCHMARK.md` —
see `~/.claude/projects/.../memory/project_sensitivity_sweep_pre_release_20260814.md`
for what that used to cost. Still sanity-check the prose in
`docs/notes/*BENCHMARK.md` too — the JSON baseline tracks 50%-crossing SNR
and, for ft8/ft4/fst4, unexpected decodes (`_meta.precision`); WSPR's
phantom-decode count is still only in its printed table. FT8, FT4 and
FST4 sweep as their upstream task T1 (`scripts/upstream_tasks.json`),
so one sweep serves both baselines. FT8's T1 runs `.sic_early()`, where
the phantom-prone code lives, which replaced the separate
`ft8_sic_early` pass. `ft8_itu` and `ft8_busy` keep the default
`decode()` path under a baseline. The full procedure, setup included,
is `docs/notes/TIER_C_MANUAL.md`. FT8 and FT4 are
**not** narrowed to a window around the crossing any more: the sweeps take
seconds, and the phantoms sit outside that window. Once you
understand why a number moved, refresh the baseline with
`python3 scripts/sweep-regression-check.py --update-baseline
target/sweep-csv/*.csv`, and update `docs/notes/BENCHMARKS.md` too if
the reason is one worth recording there (new hardware counts — the
table records the machine).

`--update-baseline` also stamps the JSON's `_meta.protocols` block —
date, commit, machine, trial count — **for the protocols in that run
only**, which is what `release-status.sh` reads to decide what is
actually outstanding. So a partial refresh
(`run-sensitivity-sweeps.sh ft4 fst4`) silences ft4 and fst4 and
leaves the other seven still flagged, instead of the whole file
looking current because one date moved. Don't hand-edit those dates:
the point is that the number and its provenance are written by the
same run.

A nightly workflow was considered and rejected: on a solo, bursty repo
most nights would re-measure unchanged code, and this project already
deleted one scheduled tier for precisely that reason (see `ci.yml`'s
note that it "only ever cost wall-clock for output nobody was
routinely reading").


### Public API check before a minor tag

`scripts/semver-check.sh` (cargo-semver-checks against the latest tag, `--features full`) lists every public item that was removed, renamed
or made `#[non_exhaustive]`, and every enum discriminant that moved. Each line must be in the CHANGELOG's "Breaking changes at a glance"; a line
that is not is either a missed entry or an accident. First run, 0.13.1 against `main` (2026-10-17): 7 lints; the removed `ModulationParams` consts,
the OSD functions, `jt9::baseband` and the `#[non_exhaustive]` types were already listed, the `Mode` discriminants were not (added).

### Release cadence — biweekly, decoupled from merging

PRs land on `main` immediately as they're ready — CHANGELOG.md's
top (unreleased/latest-numbered) section accumulates entries between
tags, so `main`'s history and the CHANGELOG are always current for
anyone reading the repo directly. **Tagging is separate and
throttled**: established 2026-07-19 after v0.7.0-v0.7.3 shipped in a
4-day burst (see `~/.claude/projects/.../memory/` for the session
that measured this) — that burst wasn't itself a problem (each tag
was a genuinely complete, coherently-scoped unit of work, not an
arbitrary slice; per-tag diff size was comparable to or larger than
historically slower-cadence releases), but four crates.io publishes /
GitHub Releases in four days is more update-notification noise than
downstream consumers want, even when every individual change was
sound.

**Default cadence: every 2 weeks** (13-14 days is the normal target,
not an overrun to apologize for) from the last regularly-scheduled tag,
bundling everything merged to `main` since then into one release PR +
tag. Don't tag opportunistically just because a feature or fix
finished — let it sit in the unreleased CHANGELOG section until the
next scheduled cut, *unless* the escape hatch below applies.

The regular cuts so far ran 8 to 13 days apart (v0.7.4 → v0.10.1); 13-14 days is the target, not an overrun.

**Escape hatch**: an out-of-cadence tag is fine for a security fix, a
data-loss/correctness bug serious enough to want off the broken
version quickly, or whenever the user explicitly asks for an
immediate release regardless of reason. Cutting early is the user's
call, not something to infer on your own from "this seems important."

**Versioning within this cadence**: a new protocol/mode addition is
patch-level by this crate's own established convention (MSK144
shipped as `0.7.4`, not `0.8.0` — grep `CHANGELOG.md` **and**
`docs/historical/CHANGELOG-*.md` for prior protocol additions before
assuming otherwise; older minor series move out of the top-level file
by the rule under "House conventions", and `CHANGELOG.md`'s own footer
lists the archives).
Minor bumps
(`0.6→0.7`) have historically marked a more structural change — a
breaking or structural API move, e.g. `0.7.0`'s generic
`decode_frame_for::<P>` API landing alongside FST4's remaining
sub-modes, or `0.10.0` collecting three public search-parameter type
changes — not simply "a release with new capability in it". When
genuinely unsure which a given accumulated batch warrants, ask rather
than default to whichever bump feels more exciting.


## Following WSJT-X upstream — run `scripts/upstream-diff.sh` on a schedule

WSJT-X is the reference, and it keeps moving, so we follow it on a
cadence instead of when someone happens to remember. **Run
`scripts/upstream-diff.sh` at the midpoint of the biweekly release cycle
(about day 7 after the last tag), and whenever a new upstream tag
appears.** Not at the cut: a change found then cannot be ported, measured
(tier C) and merged before the tag, so it slips a whole cycle. At the
midpoint the second week is the porting window and the cut ships what was
found. At the cut itself, only check that the Issue from the midpoint run
has no open A/B items left. It fetches, prints
the tags and branches by date, and lists the changed upstream files that
`mfsk-core` cites, with the Rust files that cite them. Read-only apart
from the fetch.

Facts that cost an hour each time they were re-derived (2026-10-10, #642):

- **The live upstream is `WSJTX/wsjtx`, the `upstream` remote** of the
  `../WSJT-X` clone. `origin` (saitohirga) stopped in 2024-03 and
  `upstream/master` is the 3.0.x line; neither shows new algorithms. The
  development line is `release/3.x` and its `v3.x.0-*` tags. We follow
  that, not `master`.
- **A tag is not necessarily an ancestor of a later tag.** v3.2.0-rc1 was
  cut from the development line on 2026-09-18 and v3.3.0-beta1 descends from
  the same line without containing it (merge-base `0b32aba69`). Commit
  counts and `git log base..target` are meaningless across such a fork; the
  **tree diff** is the comparison. The script says when this applies.
- Most of a version's diff is refactor (3.3's PCM-engine move was the large
  majority). Classify each change A (alters results), B (real bug fix) or C
  (refactor/plumbing); only A and B are ported. The script finds the files,
  it cannot classify them — read the diff it points at.

Record the audit in an Issue (#435 for rc1, #642 for rc1→beta1) with the
A/B/C table and what was **not** read. When the port PRs for a version have
merged, bump `DEFAULT_BASE` in the script and say so in the Issue. A changed
upstream file the script lists as uncited is either not ported or a missing
citation; add the citation if it is ported (`q65_callers.F90` was).


