# Running tests: which tier, which command

Moved out of the repo-root `CLAUDE.md` (2026-10-12). The tiers themselves are defined in `CONTRIBUTING.md` (Testing philosophy); this is the operational half.

## Running tests — pick the tier, never blanket `--ignored`

`CONTRIBUTING.md` "Testing philosophy" defines the four tiers (A
invariants / B golden fidelity / C sensitivity / D deleted). This
section is the operational half: which command to actually type.

**The merge gate — use this by default.** Every non-ignored test
across every protocol, i.e. tiers A + B. ~90 s on a Ryzen 9 9900X.
This is exactly what CI's `Test (tier A+B — invariants + golden)` job
runs:

```sh
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
```

`MFSK_REQUIRE_CORPUS=1` turns a missing golden recording into a
failure instead of a silent skip. Set it locally too — without it a
mis-resolved asset path reports green (that is exactly how five
protocols' golden tests skipped unnoticed before the recordings were
vendored).

Tier B goes through `tests/common/golden.rs::assert_golden`, which takes
recall **and** precision together. There is deliberately no recall-only
entry point: adding a golden test forces a decision about the protocol's
phantom budget. Target `max_extra: 0`; anything else is documented debt.

**Iterating on one protocol**: scope with `--test`, don't reach for
`--ignored`.

```sh
cargo test -p mfsk-core --features full,internal-testing --release --test ft8_qso3_full_parity_recall -- --nocapture
```

**The trap: a blanket `-- --ignored` locally escalates into tier C.**

```sh
# DON'T — this is a sensitivity measurement campaign, not a check.
cargo test -p mfsk-core --features full,internal-testing --release -- --ignored
```

`--ignored` with no `--test` scope runs *every* `#[ignore]`d function
in the crate, which includes the tier-C sensitivity sweeps
(`fst4_sweep`, `ft8_sweep`, `jt9_sweep`, `jt65_sweep`, `wspr_sweep`,
`q65_*_sweep`, …). Those are the ones that consume the ~17 GB
generated corpora under `embedded-poc/assets/*_sweep/`. **On CI they
silently skip** — the corpora aren't there, and most of those
binaries aren't in any CI glob anyway — so the command looks harmless
in `ci.yml` and is not harmless here. `fst4_sweep` **alone** had not
finished after 35 minutes when it was killed (2026-08-12), and it is
one of a dozen such binaries.

CI does run some `--ignored` suites, but always scoped per binary
(`--test q65_ap_sweep --test q65_snr_sweep … -- --ignored`) and
push-only (or on a PR carrying the `run-full-sweep` label). If you want a
specific ignored test, name its binary the same way.

**Tier C** belongs to two moments only: before a release tag, and
after a change that plausibly moves sensitivity. It has its own
runner, which checks the corpora are present up front instead of
producing a wall of silent skips:

```sh
scripts/run-sensitivity-sweeps.sh              # everything present
scripts/run-sensitivity-sweeps.sh ft4 fst4     # or just what moved
```

It also refuses a corpus without a `.corpus-stamp` or with a seed
other than 1 (`scripts/lib/corpus-stamp.sh`, 2026-09-30). If it stops
there, regenerate that corpus into an **empty** directory. Don't
override the check to get numbers. The last time a stale corpus got
through, it showed as a JT65 "+0.68 dB regression" in unchanged
code. The generators also refuse an unseeded simulator binary. On this machine `target/ft8sim/ft8sim` and
`target/ft4sim/ft4sim` are such builds, so pass `target/ft8sim-2b9/` /
`ft4sim-2b9/`, or `ft8sim-3.2/` for the ITU and busy-band corpora.

The runner also compares against upstream: every task in
`scripts/upstream_tasks.json` for a swept protocol is paired per trial
with WSJT-X's committed outcome, and both sides are timed
(`scripts/upstream-baseline.py`). The record is
`docs/notes/UPSTREAM_EVALUATION.md`, an evaluation sheet; its verdicts
and speeds feed the headline table at the top of `BENCHMARKS.md`, so a
re-measurement updates both. The sheet's p-values and crossings stay out
of the headline.
A `jt9` comparison is only meaningful on a task defined on both sides.
Define it there, never as "both at their defaults". Q65's first `jt9`
comparisons differed in band, Rx frequency and AP sequencing, and were
not measuring anything.

**"Plausibly moves sensitivity" means the code path, not the
protocol name.** Before running a protocol's sweep, check it actually
shares the code you touched. FT8's `decode_block::coarse_sync` /
`SYNC_LAG_S` is FT8-only — FST4 reaches sync through
`engine::sync::coarse_sync` + `engine::sync2d::fst4_sync_search`, so
an FT8 coarse-sync change cannot move an FST4 curve and running that
sweep buys nothing (issue #280, where this was learned the expensive
way). `release-status.sh` prints this warning itself whenever a commit
touches `src/engine` — heed it rather than sweeping everything.

**Other feature sets.** `fixed-point` is the numeric path embedded
ships and is worth a second run whenever you touch the decode
pipeline — it reorders candidates through `SpecCell = u16`
quantisation, so it can diverge from host f32 on the same fixture:

```sh
cargo test -p mfsk-core --features full,fixed-point --release --test ft8_qso3_apoff_recall
```

`wspr-ddc` and `wspr-ddc-cascade` are the WSPR embedded channelizer
swaps (`wspr::ddc`'s single-stage and two-stage-cascade streaming
down-converters, replacing the host-default whole-slot FFT
`decimate_to_baseband`) — mutually exclusive (`compile_error!` in
`decode_scan_inner` if both are on), each with its own real-signal
golden test against the WSJT-X recording, worth a run whenever
`wspr::ddc` changes:

```sh
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing,wspr-ddc-cascade --release --test wspr_wsjtx_samples wspr_cascade_ddc_golden_recall_and_precision
```

**`internal-testing` is not optional for whole-crate commands** — see the
feature-flag section above for why the `E0603`s are not your fault.

## Test fixture paths

Never hardcode absolute paths like `/home/ubuntu/...` or `/Users/...`
for test inputs. AI assistants tend to "fix" path failures by
translating to whichever local environment they happen to run in
(commit `119657a` flipped `/home/minoru/` → `/home/ubuntu/`), which
just relocates the bug.

- **In-repo assets**: use the `asset_path!` macro from
  `mfsk-core/tests/common/mod.rs` (integration tests) or
  `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>")`
  (unit tests under `src/`). Vendor the file under
  `embedded-poc/assets/` if it's not already there — the FT8 / JT9
  reference recordings already live there, and the tier-B goldens are
  under `embedded-poc/assets/golden/` (see its README for the upstream
  mapping), resolved through `tests/common/corpus.rs`.
- **Out-of-tree user-machine assets** (e.g. the full WSJT-X tarball):
  `option_env!("WSJTX_SAMPLES_DIR")` and skip cleanly when unset.
- **Diagnostic output paths** (test writes a WAV for human inspection):
  `/tmp/...` literals are fine — the human-in-the-loop step assumes a
  known location. Don't replace these with `tempfile`.


