# Codebase cleanup plan — 2026-05

Single-target focus: production users are on m5stack-s3-app; Core2
hardware bring-up resumes weekend 2026-05-17/18 under issue #61
(Core2→S3 unification). Until that lands, this document tracks the
intermediate cleanup so the integration starts on a tidy base.

## Stages and ordering

The Greek-letter labels match the original proposal in conversation
(α dead-code / β feature-cfg / γ scaffold / δ docs / ε restructure).
α was folded into β.5 (compiler-visible dead code), and the
execution order is γ-first because scaffolding cleanup unblocks
the others — hence the non-alphabetical sequence below.

```
γ ──→ β ──→ δ ──→ ε
(scaffolding) (feature/cfg) (docs sync) (decode_block.rs restructure)
```

Each later stage assumes the earlier stages have landed:

- γ retires dead code first so β/δ/ε see a smaller surface.
- β narrows the feature matrix so δ docs and ε refactor know which
  cfg combos are actually shipped.
- δ ratifies the post-β documentation snapshot so ε's larger
  refactor lands on stable narrative ground.
- ε is the only week-scale item; γ/β/δ are 1–3 days each.

## γ — scaffolding cleanup

**Done 2026-05-13 (PR #66).**

- Removed `embedded-poc/m5stack-core2/src/bin/rx_skeleton.rs` and
  `embedded-poc/m5stack-s3/src/bin/rx_skeleton.rs` — both 257-line
  UNVERIFIED stubs that would be replaced wholesale by #61. The
  live-RX production path lives in `embedded-poc/m5stack-s3-app`.
- Removed corresponding `[[bin]] name = "rx-skeleton"` entries from
  both Cargo.toml files.
- Updated `m5stack-core2/README.md` and `m5stack-s3/README.md` to
  point at the post-#61 consolidation plan instead of the per-board
  stubs.

Deferred (touched only if hardware bring-up reveals churn):

- `m5stack-core2/logs/` (43 files, 1.1 MB, last touched 2026-05-04)
  — kept as historical record; referenced by memory entries.
- `#[ignore]` test audit — most are diagnostic sweeps (jt9/rx.rs,
  ft4_diag_low_snr, ft8_qso3_coarse_sync_probe, etc.) the team
  reaches for during debugging. Hand-evaluate per file in δ.
- `embedded-poc/scripts/{flash-monitor.sh,udp-log-listen.sh}` —
  both actively used per CLAUDE.md, keep.

## β — feature flag / cfg cleanup

**Done 2026-05-13. β.5 in PR #67; β.1–4 audited, no findings.**

### β.5 compiler-visible dead code (code change)

Landed in PR #67. Cleared all four warnings under
`cargo check --features fft-rustfft,fixed-point`:

- `fn recompute_nsync`, `fn recompute_snr_xsnr2` in
  `mfsk-core/src/ft8/decode_block.rs` — cfg tightened from
  `feature = "fft-rustfft"` to
  `all(feature = "fft-rustfft", not(feature = "fixed-point"))`,
  matching the only caller (`retain_mut` block that needs the
  xsnr2/xbase post-process, f32-only).
- `const ALLSUM_WIN` in
  `embedded-poc/embedded-shared/src/stage1_inc.rs` — removed.
  Leftover from a Phase E2 per-half allsum draft; siblings
  (`ALLSUM_FREQ_*`) are still in use.
- `basis_re` / `basis_im` parameters of
  `process_candidates_into_with_cs_scratch_tuned` — referenced
  via `let _ = &basis_re;` under `#[cfg(feature = "fft-rustfft")]`
  to mark intent (signature stays for embedded callers).

### β.1–4 cfg audit results (no code change required)

Surveyed every `#[cfg(feature = …)]` site against the four-cell
build matrix `{fixed-point, !fixed-point} × {fft-rustfft, fft-extern}`.
All four cells produce zero warnings with the relevant
`cargo check --features …` invocation:

| cell | features | status |
|---|---|---|
| host f32 (default) | `fft-rustfft` | clean |
| host fixed-point bench | `fft-rustfft, fixed-point` | clean |
| embedded f32 (unused in production) | `fft-extern` | clean |
| embedded ship | `fft-extern, fixed-point` | clean |

cfg gate counts by feature (2026-05-13 sweep):

- `fft-rustfft` / `fft-extern`: ~67 sites across mfsk-core. Every
  branch reachable from at least one live cell above.
- `fixed-point`: ~32 sites in `decode.rs` + `decode_block.rs`. All
  branches live.
- `nstep-half`: 4 sites in `mfsk-core/src/ft8/params.rs` and
  `decode_block.rs` (symmetric `#[cfg(not(feature = "nstep-half"))]`
  / `#[cfg(feature = "nstep-half")]` pairs). Embedded preset
  enables it; default host stays on WSJT-X-faithful NSPS/4.
- `parallel`: 5 sites in `mfsk-core/src/core/pipeline.rs`. Default
  features enable it; both branches reachable.
- `profile-coarse`: read via `cfg!(feature = …)` macro at runtime
  in `decode_block.rs` (search the file for
  `cfg!(feature = "profile-coarse")`), not via `#[cfg]` gating —
  intentional, the feature wires `MFSK_PROFILE_COARSE` env-var
  fallback for embedded.

No dead cfg branches found. Acceptance criteria met.

## δ — documentation sync

**Status: PR #70 open 2026-05-14, awaiting CI / merge.**

All five sub-tasks landed in a single commit on branch `cleanup-delta`:

- δ.1 `docs/ROADMAP.md` compressed 508 → 243 lines (0.6.2 marked
  shipped, Open follow-ups rebuilt against the live issue set
  `#23/#24/#25/#58/#61/#63/#64/#65`, Cleanup 2026-05 pointer
  section added, legacy A0/A0' detail dropped, Phase B rewritten
  against current `m5stack-s3-app` phase log).
- δ.2 root `README.md` `embedded-poc/` pointer updated —
  `m5stack-s3-app/` called out as production crate, the
  `m5stack-{s3,core2}/` benches tracked under `#61`.
- δ.3 five remaining "AP-biased" sites (`mfsk-core q65/rx.rs`,
  `mfsk-ffi src/lib.rs ×2`, `mfsk-ffi include/mfsk.h`, `README.md`)
  updated to "AP-hint"; workspace grep returns zero hits.
- δ.4 new shared `embedded-poc/CLAUDE.md` (110 lines) +
  per-crate trims: `m5stack-core2/CLAUDE.md` 78 → 36, `m5stack-s3/CLAUDE.md`
  86 → 39; new `m5stack-s3-app/CLAUDE.md` (47) and
  `embedded-shared/CLAUDE.md` (28); all per-crate < 50 lines.
- δ.5 broken cross-refs to retired `qsb_partial_gain` /
  `subtract_signal_weighted` fixed in `core/pipeline.rs`,
  `ft8/decode.rs`, `embedded-poc/m5stack-{core2,s3}/Cargo.toml`,
  `embedded-poc/CLAUDE.md`, `m5stack-core2/CLAUDE.md`. Remaining
  grep hits are intentional date-stamped historical notes plus
  the FT4 crate's own `subtract_signal_weighted` (different
  module, still in use).

Acceptance verified: `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
--features full -p mfsk-core` + `-p mfsk-ffi` clean, no broken
intra-doc cross-refs; pre-commit hook (`fmt` + `clippy --workspace
--all-targets --features full -- -D warnings` + `rustdoc -D warnings`)
green.

Original sub-task descriptions are preserved below for the audit
trail; the **acceptance summary above is the authoritative status**.

Five sub-tasks, can be done in any order (no internal dependencies):

### δ.1 ROADMAP refresh

`docs/ROADMAP.md` currently mixes 0.6.2 ("in flight") status,
0.6.0 status, and a legacy post-0.5.12 plan. Refresh after γ/β
landed:

1. Move 0.6.2 from "in flight" to "shipped" (released
   2026-05-08 per memory, plus the 0.6.x post-merge sweep in
   #53/#60/#62/#66/#67 cleared 2026-05-13).
2. Compress the legacy post-0.5.12 section — only items still
   relevant carry forward. **A0 is closed** (#40, since 0.6.0);
   **A0'** for embedded AP loop shipped as Step 4 of
   `process_one_candidate_inner` in 0.6.1; the remaining
   `decode_block_with_ap` symmetric port is a new issue (none
   filed yet — file under δ.1 if pursued).
3. Update "Open follow-ups" section to the current issue set:
   - **#23** FST4-60A golden lockdown (Phase A1, still open)
   - **#24** JT65 golden + erasure-metadata path (Phase A2)
   - **#25** MSK144 (community invitation, out of 3-month plan)
   - **#58** D-1 redundant compute_llr (low-priority profile)
   - **#61** Core2→S3 unification (weekend hardware work)
   - **#63** WSJT-X-faithful OSD precoding (host-only,
     correctness, deprioritised)
   - **#64** decode_block_multipass fft_cache hoist (host perf
     follow-up to #60)
   - **#65** SyncOnly+DataOnly cd0 share (host perf nice-to-have)
4. Add a "Cleanup 2026-05" section pointing at this doc so the
   ROADMAP captures γ/β shipped + δ/ε pending.

### δ.2 Root README badge / build commands

`README.md` (root) — verify:

- Crate version badge reads 0.6.2.
- Build commands in the quick-start use the post-β feature
  presets that actually exist (`default`, `embedded-rx`,
  `full`). No stale references to retired features.
- `embedded-poc/` example commands point at `m5stack-s3-app`
  (current production), not `m5stack-core2/m5stack-s3` (the
  compute-bench crates).

### δ.3 LIBRARY.{md,ja.md} terminology cross-check

PR #53 standardised "AP-hint BP" in the LIBRARY.md / LIBRARY.ja.md
strategy tables, but a 2026-05-13 sweep shows two known carry-over
sites that this stage needs to finish:

- `mfsk-core/src/q65/rx.rs` — the `decode_scan_with_ap_for`
  docstring (currently at line ~516; grep for the function name)
  opens with "AP-biased version of [`decode_scan_for`]" — update
  the wording to "AP-hint variant of [`decode_scan_for`]" or
  similar to match the rest of the doc set.
- `README.md:228` (root) — quick-start references
  `decode_scan_with_ap*` as "AP-biased".

Plus broader verification across the doc set:

- §3 Q65 "four decoder strategies" table reads
  `AWGN / AP-hint / fast-fading / AP-list` everywhere (done in
  #53; verify nothing slipped back).
- §3 narrative paragraphs after the table use **AP-hint BP**
  (not bare "AP", not "AP-biased BP" — verify).
- Q65 FFI block in LIBRARY.ja.md (search for
  `mfsk_q65_decode_with_ap`) has the matching "AP-hint BP"
  inline annotation.
- Re-grep after touching the two known sites:
  ```sh
  grep -rn 'AP-biased\|AP biased\|AP-bias' \
    mfsk-core/ mfsk-ffi/ mfsk-ffi-ft8/ embedded-poc/ docs/ README.md \
    --exclude=CLEANUP_2026_05.md
  ```
  Expected output: zero hits in source/docs, optional historical
  notes in commit-message-like context only.

### δ.4 Embedded CLAUDE.md consolidation

Two crate-level CLAUDE.md files exist:
`embedded-poc/m5stack-core2/CLAUDE.md` and
`embedded-poc/m5stack-s3/CLAUDE.md`. Both duplicate the
"one-time setup" section (espup install, export-esp.sh,
~/.espressif, ~/.cargo/bin/espflash) and the
"trouble we've already debugged" list.

Refactor:

1. Create `embedded-poc/CLAUDE.md` containing the shared
   one-time setup + the cross-board debug list (LX6 vs LX7
   differences moved here as a comparison table).
2. Trim each crate's CLAUDE.md to crate-specific notes only:
   board name, target triple, `cargo build` invocation, any
   board-specific gotchas (PSRAM mode, opt-level rationale).
3. Add a header pointer in each crate CLAUDE.md to the shared
   `embedded-poc/CLAUDE.md`.
4. `embedded-poc/m5stack-s3-app/` currently has no CLAUDE.md — add
   one (production crate; weekend Core2 work will likely need it).
   `embedded-poc/embedded-shared/` similarly — add a stub pointing
   at the shared doc.

### δ.5 Source-file docstring grep for retired paths

Per the 0.6.2 ROADMAP, several APIs were removed:
`subtract_signal_weighted`, `qsb_partial_gain`,
`ft8::llr::symbol_spectra`, `ft8::sync::coarse_sync` (the host
wrapper). Grep across the workspace for any remaining mention
and either delete the reference or note "removed in 0.6.2":

```sh
grep -rn 'subtract_signal_weighted\|qsb_partial_gain\|ft8::llr::symbol_spectra\|ft8::sync::coarse_sync' mfsk-core/ mfsk-ffi/ mfsk-ffi-ft8/ embedded-poc/ docs/ README.md
```

Hand-evaluate each hit. Real removals are dead references that
should be deleted; intentional "historical note" mentions (e.g.
ROADMAP entries) stay but get a date stamp.

### Acceptance for δ

- `cargo doc --no-deps --all-features` builds with no broken
  cross-references.
- ROADMAP "Open follow-ups" lists exactly the issues currently
  open with `state:open` filter.
- No instance of "AP" without "-hint" or "-list" qualifier
  outside intentional historical context.
- Each embedded crate CLAUDE.md is < 50 lines (down from current
  150+) with the bulk in `embedded-poc/CLAUDE.md`.

## ε — decode_block.rs restructure (week-scale)

**Status: planned, sequenced after `#61` weekend bring-up
(2026-05-17/18). ~6 work days, split into 7 phases each shipped
as its own PR so any phase can be reverted in isolation if a
recall regression sneaks in.**

`mfsk-core/src/ft8/decode_block.rs` is 3 517 lines (as of
2026-05-14, post-γ/β/δ). Logical blocks, with line ranges current
on `main` plus the merged-δ-PR-#70 view:

| range | lines | content |
|---|---|---|
| 1–244 | ~245 | module doc / `use` / constants (`NFFT_SPEC`, `NSTEP`, `DEFAULT_Q_THRESH`, …) |
| 246–552 | ~310 | `Spectrogram` struct + `compute_spectrogram` (`fft-rustfft` and `fft-extern` variants) |
| 553–1079 | ~525 | `coarse_sync` family (`_with_allsum`, `precompute_coarse_allsum*`, `fill_coarse_allsum`, `coarse_sync_inner`) |
| 1080–1645 | ~565 | `fill_symbol_spectra` family + `SymMask` enum + `symbol_spectra_direct*` + `BASIS_SCRATCH_LEN` body |
| 1647–2287 | ~640 | `decode_block` / `_tuned` / `_with_ap*` entry points + two `decode_block_multipass` variants + two `fine_refine_pass1` variants + `xsnr2_db_simple` + `decode_block_into*` |
| 2288–2524 | ~235 | `refine_candidates*` + `sync_quality_block0` + `bp_step_select` |
| 2525–2838 | ~314 | `process_candidates*` family (`_tuned`, `_with_ap`, `_into*`, `_with_cs_scratch*`) |
| 2839–3170 | ~332 | `process_one_candidate_inner` (starts 2841) plus the `WSJTX_NHARDERRORS_MAX` constant at 2839 the moved body depends on — the host + embedded shared per-cand LLR → BP → OSD → AP staircase + the OSD dispatch (#63's seam target) |
| 3171–3517 | ~350 | `#[cfg(test)]` tests |

External callers (out of `mfsk-core`) reach into
`ft8::decode_block::*` from two crates:

- `mfsk-ffi-ft8/src/lib.rs` — `decode_block`, `decode_block_into`,
  `BASIS_SCRATCH_LEN`.
- `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs` +
  `m5stack-{s3,core2}/src/bin/rx_wavsim.rs` — `DEFAULT_Q_THRESH`,
  `NFFT_SPEC`, `BASIS_SCRATCH_LEN`, `xsnr2_db_simple`.

Intra-crate reaches (must be preserved by re-export through the
parent module file): `crate::ft8::decode_block::{coarse_sync,
compute_spectrogram, fill_symbol_spectra, SymMask, Spectrogram,
process_one_candidate_inner}` from `ft8/decode.rs`,
`ft8/baseline.rs`, `core/sync.rs`, `core/llr.rs`.

### Goals (restated from the original ε proposal)

1. Split per pipeline stage using the Edition 2018+ module
   layout (no `mod.rs`):
   - `mfsk-core/src/ft8/decode_block.rs` stays as the parent
     module file and keeps the public `decode_block` entry
     function as the facade — external callers see no API change.
   - Per-stage submodules live in a new sibling directory:
     `mfsk-core/src/ft8/decode_block/spectrogram.rs`,
     `decode_block/coarse_sync.rs`,
     `decode_block/fill_symbol_spectra.rs`,
     `decode_block/process_candidates.rs`,
     plus `decode_block/per_candidate.rs` (split out from the
     literal "process_candidates" naming so the LLR/BP/OSD/AP
     staircase has its own file) and
     `decode_block/osd_strategy.rs` (the `#63` seam, see ε.5).
2. Collapse the public-but-`#[doc(hidden)]` API surface — 16
   `#[doc(hidden)]` sites today. Many are "pub for cross-module
   reach" that the submodule split itself eliminates.
3. Lift the OSD dispatch (the WSJT-X-faithfulness deviation
   documented in PR #62) into a dedicated `osd_strategy` module
   that #63 (precoding) can hook into without touching the rest
   of the decoder.

### Phase plan

Each phase is a single PR. Pre-commit hook runs `fmt` + `clippy
--workspace --all-targets --features full -- -D warnings` +
`rustdoc -D warnings`; per-phase acceptance also runs the qso3
golden test (`ft8_qso3_apon_recall` + `decode_block_qso3` at
`--release --features full`) so recall regressions are caught at
PR time, not on the final phase.

#### ε.1 — module skeleton (~0.5 day)

- Create empty `mfsk-core/src/ft8/decode_block/` directory and
  empty submodule files for `spectrogram`, `coarse_sync`,
  `fill_symbol_spectra`, `process_candidates`, `per_candidate`,
  `osd_strategy`.
- Parent `decode_block.rs` declares them as `mod spectrogram;`
  etc. with no `pub use` yet (nothing in the children, so
  nothing to re-export).
- **Acceptance**: all tests pass, no API surface change, build
  size diff < 100 B.

#### ε.2 — `spectrogram.rs` (~0.5 day)

- Move lines 246–552 (`Spectrogram` struct + both cfg-gated
  `compute_spectrogram` variants) to `decode_block/spectrogram.rs`.
- Parent re-exports `pub use spectrogram::{Spectrogram,
  compute_spectrogram};` (both names are intra-crate-reached and
  the latter is on the public surface graduated in v0.6.0).
- Keep both `#[cfg(feature = "fft-rustfft")]` and
  `#[cfg(feature = "fft-extern")]` variants in the same submodule
  so the cfg gates do not duplicate across module boundaries.
- **Acceptance**: qso3 golden green, embedded build size ±100 B.

#### ε.3 — `coarse_sync.rs` (~1 day)

- Move lines 553–1079 (`coarse_sync`, `_with_allsum`,
  `coarse_allsum_len`, `precompute_coarse_allsum`,
  `precompute_coarse_allsum_into`, `fill_coarse_allsum`,
  `coarse_sync_inner`) to `decode_block/coarse_sync.rs`.
- Parent re-exports `pub use coarse_sync::*;` (entire family is
  reached from `ft8/decode.rs` + `core/sync.rs` doc-link).
- Verify the intra-doc-links in `fec/ldpc/bp.rs:577`,
  `core/sync.rs:224`, `ft8/decode.rs:626`, `ft8/sync.rs:8` still
  resolve via the parent's `pub use` re-export — they should keep
  pointing at the public facade
  (`crate::ft8::decode_block::coarse_sync`), not at the internal
  submodule path, so future re-layouts don't break the links.
- **Acceptance**: `cargo test --release --test
  ft8_qso3_apon_recall` green with all-AP-off goldens (7/8
  WSJT-X, 16/18 JTDX) and all-AP-on goldens (5/6 JTDX extras)
  unchanged.

#### ε.4 — `fill_symbol_spectra.rs` (~1.5 day, largest split)

- Move lines 1080–1645 (`symbol_spectra_direct`, `SymMask`,
  `sym_in_mask`, the 7+ `fill_symbol_spectra*` variants,
  `BASIS_SCRATCH_LEN`, `symbol_spectra_direct_into`) to
  `decode_block/fill_symbol_spectra.rs`.
- Preserve the cfg-gate matrix: `fft-rustfft`/`fft-extern` ×
  `Spectrogram-based`/`cd0-based` × `generic Sc` produces 7
  function bodies under different cfg combinations. Move them as
  a single block to avoid splitting one combination across files.
- Verify the PR #60 `fft_cache` hoist scope still works after
  the module split — the hoist relies on a `pub(super)` visibility
  shared with the call site; after the move, `pub(super)` now
  means "the new submodule", which is the same scope.
- **Risk**: the 4-cell build matrix (`{fixed-point, !fixed-point}
  × {fft-rustfft, fft-extern}`) must all stay clean. Run all four
  `cargo check` combinations locally before pushing:
    - `cargo check --features fft-rustfft`
    - `cargo check --features fft-rustfft,fixed-point`
    - `cargo check --features fft-extern`
    - `cargo check --features fft-extern,fixed-point`
- **Acceptance**: every β build-matrix cell builds clean; qso3
  recall unchanged.

#### ε.5 — `per_candidate.rs` + `osd_strategy.rs` (~1.5 day, design crux)

This is the `#63` seam. The split here is what makes the WSJT-X-
faithful precoding work tractable.

- Move lines 2839–3170 (`process_one_candidate_inner` body, the
  shared host + embedded per-cand LLR → BP → OSD → AP staircase)
  to `decode_block/per_candidate.rs`.
- Lift the OSD dispatch (lines ~2953–3010 in the current file)
  out of the staircase and into `decode_block/osd_strategy.rs`.
  Strawman API:

  ```rust
  // decode_block/osd_strategy.rs (proposed)
  pub enum OsdStrategy {
      /// Current 0.6.x baseline: ndeep=2, escalate to ndeep=3
      /// for q >= 18. Stand-in for WSJT-X precoding.
      McMfskNdeep,
      /// #63: WSJT-X-faithful nord=1 + npre1=1 precoding.
      /// Stub for now; #63 fills in the body.
      WsjtxNpre1,
  }

  pub(super) fn run_osd(
      strategy: OsdStrategy,
      llr: &[f32],
      q: u32,
  ) -> Option<OsdResult> { /* dispatch */ }
  ```

- `process_one_candidate_inner` upgrades from `pub(super)` to
  `pub(crate)` — host `ft8/decode.rs:563` calls it, and after the
  move "super" becomes `decode_block::per_candidate` which the
  host module cannot reach directly.
- Existing 4-LLR-variant loop (`llra/b/c/d` × pass-IDs 14/15/16/17)
  stays in `per_candidate.rs`; the strategy is dispatched once
  per variant. `#63` lands later by filling in `WsjtxNpre1` —
  no other ε.5 file needs to change.
- **Risk**: this is the recall-sensitive phase. Each commit before
  push must pass `cargo test --release --features full --test
  ft8_qso3_apon_recall` and `decode_block_qso3` golden tests
  with **zero entry drift** from current 7/8 + 16/18 + 5/6.
- **Acceptance**: golden recall unchanged; `cargo doc --no-deps`
  for both `mfsk-core` and `mfsk-ffi` clean.

#### ε.6 — `process_candidates.rs` (~0.5 day)

- Move lines 2288–2838 (`refine_candidates*`, `sync_quality_block0`,
  `bp_step_select` (two cfg variants), `process_candidates*`
  family) to `decode_block/process_candidates.rs`.
- The caller side of `per_candidate::process_one_candidate_inner`.
- **Acceptance**: full test suite green.

#### ε.7 — pub map tighten (~0.5 day)

- Audit all 16 `#[doc(hidden)]` items. For each:
  - External consumer in `mfsk-ffi-ft8` or `embedded-poc`?
    → keep `pub`, keep `#[doc(hidden)]`.
  - Intra-crate (`ft8/decode.rs` only)? → demote to `pub(crate)`,
    drop `#[doc(hidden)]`.
  - Submodule-internal only? → demote to `pub(super)`.
- Run `cargo public-api --diff` against the pre-ε baseline; the
  only legitimate diffs are re-export path changes (same name,
  new submodule prefix), not new or removed surface items.

### Estimated post-ε file sizes

- `decode_block/spectrogram.rs` ~315 lines
- `decode_block/coarse_sync.rs` ~525 lines
- `decode_block/fill_symbol_spectra.rs` ~565 lines
- `decode_block/process_candidates.rs` ~480 lines
- `decode_block/per_candidate.rs` ~280 lines
- `decode_block/osd_strategy.rs` ~80 lines
- Parent `decode_block.rs` ~1 300 lines (constants ~245 + module
  declarations + re-exports + `decode_block*` entry points ~640
  + tests ~350)

Total: 3 517 → ~3 545 lines across 7 files. The line count is
intentionally close to neutral; the cleanup buys readability, not
density.

### Risk register

| | risk | mitigation |
|---|---|---|
| R1 | `pub(super)` scope shifts when a function moves submodules | ε.1 skeleton + 4-cell cfg matrix builds catch this early |
| R2 | PR #60 `fft_cache` hoist breaks when `fill_symbol_spectra` moves | ε.4 verifies the hoist still spans the new caller / callee modules; `pub(super)` survives |
| R3 | `OsdStrategy` enum can't express `npre1` precoding | Read `WSJT-X/lib/osd174_91.f90:230-289` before ε.5; if needed, promote `OsdStrategy` to a trait-object dispatch instead of an enum |
| R4 | recall drift in ε.5 (OSD code is the most numerically sensitive) | per-commit golden test runs; abort + rebase if any entry drifts |
| R5 | embedded build size swings > 2 KB | `cargo size` snapshot on each phase commit; monomorphization across module boundaries can balloon |

### Sequencing

```
weekend 2026-05-17/18
  └─ #61 Core2 fold-in (separate branch, hardware bring-up)
        ↓ merge
2026-05-19 Mon  ε.1 skeleton + ε.2 spectrogram     (~1 day)
2026-05-20 Tue  ε.3 coarse_sync                     (~1 day)
2026-05-21 Wed  ε.4 fill_symbol_spectra             (~1.5 day)
2026-05-22 Thu (pm)  ε.5 per_candidate + osd_strategy (~1.5 day)
2026-05-23 Fri  ε.6 process_candidates + ε.7 pub-map (~1 day)
                       total ~6 work days
```

`#63` (WSJT-X-faithful OSD precoding) is the natural follow-on
PR after ε.5 — fill in `osd_strategy::WsjtxNpre1`, no other
file touches. `#64` and `#65` (host perf nice-to-haves) can also
land on top of ε without conflict.

### Acceptance for ε

- No behavioural change. WSJT-X golden recall on `qso3_busy.wav`
  holds (7/8 WSJT-X golden, 16/18 JTDX golden, 5/6 JTDX AP-on
  extras, embedded 7 total post-SlotEnd).
- File-level git churn is contained to `decode_block/` directory;
  external API (`#[doc(hidden)]` entries we still need to expose)
  unchanged — `cargo public-api --diff` shows only path renames.
- Build size unchanged within ±2 KB on the embedded preset
  (`cargo size --release --features
  embedded-fixed-point,embedded-runtime --target
  xtensa-esp32s3-espidf` from `mfsk-ffi-ft8`).
- All seven phase PRs ship; if any later phase forces an earlier
  one to revert, the earlier phase is re-landed before the later
  phase is re-attempted.

## Out of scope for this cleanup

- Implementing #61 (Core2→S3 unification) — wait for weekend
  hardware bring-up.
- Implementing #63 (OSD WSJT-X faithfulness / npre1 precoding) —
  ε creates the seam, then #63 implements against it.
- New protocol work (#23 FST4, #24 JT65) — independent track,
  schedule after ε lands.
