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

**Status: planned for weekend session.**

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

`mfsk-core/src/ft8/decode_block.rs` is ~3500 lines and contains the
host fft-rustfft path, the embedded fixed-point path, the host
fixed-point bench path, coarse_sync, compute_spectrogram,
fill_symbol_spectra family, process_candidates*, and OSD dispatch.

Restructure goals (not yet committed to as design):

1. Split per pipeline stage using the Edition 2018+ module
   layout (no `mod.rs`):
   - `mfsk-core/src/ft8/decode_block.rs` stays as the parent
     module file and keeps the public `decode_block` entry
     function as the facade — external callers see no API change.
   - Per-stage submodules live in a new sibling directory:
     `mfsk-core/src/ft8/decode_block/spectrogram.rs`,
     `mfsk-core/src/ft8/decode_block/coarse_sync.rs`,
     `mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs`,
     `mfsk-core/src/ft8/decode_block/process_candidates.rs`.
2. Collapse the public-but-`#[doc(hidden)]` API surface — many
   "pub for benchmarking only" items leaked because they were
   reached by tests across module boundaries. After γ/β the set
   of legitimate cross-module exports is smaller; tighten the
   pub map.
3. Lift the OSD dispatch (the WSJT-X-faithfulness deviation
   documented in PR #62) into a dedicated `osd_strategy` module
   that #63 (precoding) can hook into without touching the rest of
   the decoder.

Acceptance for ε:

- No behavioural change. WSJT-X golden recall on qso3_busy holds.
- File-level git churn is contained to `decode_block/` directory;
  external API (`#[doc(hidden)]` entries we still need to expose)
  unchanged.
- Build size unchanged within ±2 KB on the embedded preset.

### ε.0 — Pre-flight survey (2026-05-14)

Investigation snapshot used to derive the split below. Re-verify
before code lands if anything in `decode_block.rs` shifts between
now and ε start.

**Current section map of `mfsk-core/src/ft8/decode_block.rs`**
(3517 lines on `docs/roadmap-add-72-74` @ commit `27425cf`):

| Lines        | Block                                                                  | Notes                                                                                                  |
|--------------|------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|
| 21-50        | crate imports                                                          | Forward-deps on `super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness}`                 |
| 52-112       | `AudioSample` trait + `i16`/`i8` impls                                 | Public utility, no stage affinity                                                                      |
| 114-214      | Constants + runtime tunables (`NFFT_SPEC`, `NSTEP`, `RATIO_EPS`, `SYNC_LAG_S`, `NMS_ALPHA`, `DEFAULT_Q_THRESH`, `LlrT` alias) | Shared across all stages; `nstep-half` cfg lives here                                                  |
| 217-322      | `Spectrogram` struct + impls (host f32 / embedded i16 variants)        | Public; crosses every stage boundary                                                                   |
| 324-548      | `compute_spectrogram` (host + embedded cfg-split)                      | **Stage A**, pub                                                                                       |
| 553-1077     | `coarse_sync` family + `fill_coarse_allsum` + `coarse_sync_inner`      | **Stage B**, pub for {`coarse_sync`, `coarse_sync_with_allsum`, `coarse_allsum_len`, `precompute_coarse_allsum`, `precompute_coarse_allsum_into`} |
| 1079-1645    | symbol-spectra family (`symbol_spectra_direct{,_into}`, `SymMask`, all `fill_symbol_spectra{,_generic,_into,_into_generic}` cfg-split, `BASIS_SCRATCH_LEN`) | **Stage C**, `#[doc(hidden)] pub` — used by `decode.rs` + `embedded-shared`                            |
| 1647-1893    | Public entry points (`decode_block{,_tuned,_with_ap,_with_ap_tuned}`) + `decode_block_multipass` (host f32 path) | Façade — kept at parent module                                                                         |
| 1895-2154    | `recompute_nsync`, `xsnr2_db_simple`, `recompute_snr_xsnr2`            | SNR helpers — wedged into the `decode_block_multipass` body in source order                            |
| 2131-2210    | Embedded `decode_block_multipass` + `fine_refine_pass1` (cfg-split)    | Embedded path                                                                                          |
| 2213-2286    | `decode_block_into{,_tuned}` (fixed-point)                             | Façade — kept at parent module                                                                         |
| 2288-2434    | `pass1_limit`, `RefinedCandidate`, `refine_candidates{,_into,_with}`   | **Stage D**, pub for `RefinedCandidate` + `refine_candidates_into`                                     |
| 2443-2477    | `sync_quality_block0` + `LlrT` cfg alias                               | Shared helper                                                                                          |
| 2495-2532    | `bp_step_select` (host / embedded cfg-split)                           | Shared helper                                                                                          |
| 2534-2837    | `process_candidates*` family (8 entry points)                          | **Stage E**, `#[doc(hidden)] pub` — `process_candidates_into_with_cs_scratch_tuned` is the embedded API |
| 2862-3171    | `process_one_candidate_inner` (Step 1 BP llra → Step 2 BP llrd/b/c → **Step 3 OSD** → Step 4 AP iaptype loop) | Per-candidate inner. **OSD seam target.**                                                              |
| 3173-end     | `#[cfg(test)] mod tests`                                               | Inline unit tests                                                                                      |

**External callers of `decode_block::*` (must stay reachable):**

- `mfsk-core/src/ft8/decode.rs` — `compute_spectrogram`,
  `coarse_sync`, `fill_symbol_spectra`, `SymMask`.
- `mfsk-core/src/ft8/baseline.rs` — `Spectrogram`.
- `mfsk-core/src/core/sync.rs`, `mfsk-core/src/ft8/{sync,llr}.rs`
  — doc-link references only (no `use`).
- `mfsk-core/tests/*` — `decode_block`, `DEFAULT_Q_THRESH`,
  `precompute_coarse_allsum`, `coarse_sync_with_allsum`,
  `compute_spectrogram`, `BASIS_SCRATCH_LEN`.
- `embedded-poc/embedded-shared/src/{dual_core,pipeline,stage1_inc,apps/*}.rs`
  — `coarse_sync`, `coarse_sync_with_allsum`,
  `process_candidates_into_with_cs_scratch_tuned`,
  `refine_candidates_into`, `RefinedCandidate`, `Spectrogram`,
  `BASIS_SCRATCH_LEN`, `NFFT_SPEC`, `compute_spectrogram`,
  `DEFAULT_Q_THRESH`.
- `embedded-poc/m5stack-{core2,s3,s3-app}/*` — `BASIS_SCRATCH_LEN`,
  `DEFAULT_Q_THRESH`, `NFFT_SPEC`, `xsnr2_db_simple`.

Implication: nothing on the external-caller list can change its
fully-qualified path. The parent `decode_block.rs` must
`pub use` every item above so call sites stay byte-identical.

### ε.1 — File layout (committed)

Edition 2018+ module layout (no `mod.rs`); parent stays as
`mfsk-core/src/ft8/decode_block.rs`, submodules live under
`mfsk-core/src/ft8/decode_block/`:

| File                                       | Approx. lines | Contains                                                                                                    |
|--------------------------------------------|--------------:|-------------------------------------------------------------------------------------------------------------|
| `decode_block.rs` (parent / façade)        |       ~700    | `use` re-exports, public entry points (`decode_block{,_tuned,_with_ap,_with_ap_tuned,_into,_into_tuned}`), `decode_block_multipass` (host f32 + embedded cfg-split), `#[cfg(test)] mod tests` |
| `decode_block/params.rs`                   |       ~100    | All constants + `nstep-half` cfg + `LlrT` cfg alias + runtime tunables (`ratio_eps`, `sync_lag_s`, `pass1_limit`) |
| `decode_block/audio_sample.rs`             |        ~60    | `AudioSample` trait + `i16` / `i8` impls                                                                    |
| `decode_block/spectrogram.rs`              |       ~270    | `Spectrogram` struct + impls + `compute_spectrogram` (cfg-split host/embedded)                              |
| `decode_block/coarse_sync.rs`              |       ~530    | Stage B family (`coarse_sync`, `coarse_sync_with_allsum`, `coarse_allsum_len`, `precompute_coarse_allsum{,_into}`, `fill_coarse_allsum`, `coarse_sync_inner`) |
| `decode_block/fill_symbol_spectra.rs`      |       ~570    | Stage C family — every `symbol_spectra_*` and `fill_symbol_spectra*` cfg variant + `SymMask` + `BASIS_SCRATCH_LEN` + `fill_symbol_spectra_via_cd0` |
| `decode_block/refine_candidates.rs`        |       ~200    | Stage D family + `RefinedCandidate` + `sync_quality_block0` + `fine_refine_pass1` (cfg-split)               |
| `decode_block/snr.rs`                      |       ~190    | `xsnr2_db_simple`, `recompute_nsync`, `recompute_snr_xsnr2` (host f32 only)                                 |
| `decode_block/process_candidates.rs`       |       ~310    | Stage E family (8 entry points) + `bp_step_select` + `WSJTX_NHARDERRORS_MAX` + the internal `process_candidates_tuned_with_ap` / `process_candidates_with_ap` drivers |
| `decode_block/process_one.rs`              |       ~250    | `process_one_candidate_inner` with **Step 3 swapped for `osd_strategy::decode`** (see ε.2)                  |
| `decode_block/osd_strategy.rs`             |        ~80    | OSD seam — see ε.2                                                                                          |

Estimated total: ~3260 lines vs current 3517. The ~250-line drop
comes from import dedup + dropping the `#[cfg(feature = "...")]`
duplication where the cfg-split body collapses to one cfg-gated
file. **It is fine if this number ends up larger** — the
acceptance bar is parity, not LOC reduction.

Inside the parent file, the **only** items are:

```rust
mod audio_sample;
mod coarse_sync;
mod fill_symbol_spectra;
mod osd_strategy;
mod params;
mod process_candidates;
mod process_one;
mod refine_candidates;
mod snr;
mod spectrogram;

pub use audio_sample::AudioSample;
pub use coarse_sync::{
    coarse_allsum_len, coarse_sync, coarse_sync_with_allsum,
    precompute_coarse_allsum, precompute_coarse_allsum_into,
};
pub use fill_symbol_spectra::{
    fill_symbol_spectra, fill_symbol_spectra_generic, fill_symbol_spectra_into,
    fill_symbol_spectra_into_generic, symbol_spectra_direct, symbol_spectra_direct_into,
    BASIS_SCRATCH_LEN, SymMask,
};
pub use params::{DEFAULT_Q_THRESH, NFFT_SPEC};
pub use process_candidates::{
    process_candidates, process_candidates_into, process_candidates_into_tuned,
    process_candidates_into_with_cs_scratch, process_candidates_into_with_cs_scratch_tuned,
    process_candidates_tuned,
};
pub use refine_candidates::{refine_candidates_into, sync_quality_block0, RefinedCandidate};
pub use snr::xsnr2_db_simple;
pub use spectrogram::{compute_spectrogram, Spectrogram};

// Façade entry points (definitions, not re-exports — see ε.3
// for why these stay at the parent file).
pub fn decode_block<S: AudioSample>(...) -> Vec<DecodeResult> { ... }
// ... decode_block_tuned, decode_block_with_ap, decode_block_with_ap_tuned,
//     decode_block_into, decode_block_into_tuned, decode_block_multipass (private),
//     #[cfg(test)] mod tests
```

### ε.2 — OSD strategy seam (the seam #63 hooks)

Current shape — `process_one_candidate_inner` Step 3
(`decode_block.rs:2958-3016`):

```rust
if accepted.is_none() && matches!(depth, DecodeDepth::BpAllOsd) && q >= 12 {
    let llr_full_f32: LlrSet<f32> = compute_llr(cs_scratch);
    for (llr, pid) in [(&llra, 14u8), (&llrb, 15), (&llrc, 16), (&llrd, 17)] {
        let osd = if q >= 18 {
            osd_decode_deep(llr, 3, Some(check_crc14))    // ndeep=3
        } else {
            osd_decode(llr)                                // ndeep=2
        };
        if let Some(osd) = osd { /* accept with pid */ }
    }
}
```

Why this is the right seam: PR #62 documented that WSJT-X
`osd174_91.f90:230-289` always calls `nord=1 + npre1=1` —
order-1 MRB **plus** precoding patterns derived from `G`-matrix
columns — and mfsk-core has no precoding implementation. The
ndeep-2/3 dispatch above is the local stand-in. #63 is the
issue to swap this for the strict path; ε's contribution is
to widen the door so #63 doesn't have to touch
`process_one_candidate_inner` again.

Proposed `decode_block/osd_strategy.rs` interface:

```rust
//! OSD fallback strategy seam (Step 3 of process_one_candidate_inner).
//!
//! Two implementations:
//! - [`HostDeepDispatch`] (current behaviour): ndeep=2 when q<18,
//!   ndeep=3 when q≥18, applied to all four llra/b/c/d.
//! - [`WsjtxFaithful`] (#63 placeholder): `nord=1 + npre1=1` —
//!   not yet implemented; ε ships only the trait.

use crate::fec::ldpc::bp::BpResult;
use crate::fec::ldpc::osd::{osd_decode, osd_decode_deep};
use crate::ft8::llr::LlrSet;
use crate::ft8::params::LDPC_N;

/// Result of an OSD attempt — same shape `process_one_candidate_inner`
/// stuffs into its `accepted` slot, including the pass-ID byte so the
/// caller doesn't have to remember the 14/15/16/17 mapping.
pub struct OsdHit {
    pub bp: BpResult,
    pub pass_id: u8,
}

/// Step 3 trait. `try_decode` runs over the four LLR variants
/// in `llr_set` and returns the first acceptance, or `None`.
///
/// `q` is the symbol-level sync quality (`sync_quality` value
/// after stage 3 refill, range 0..=21) — implementations may
/// use it to dispatch between cheap and deep search.
pub trait OsdStrategy {
    fn try_decode(&self, llr_set: &LlrSet<f32>, q: u32) -> Option<OsdHit>;
}

/// Current behaviour (matches v0.6.2). ndeep=2/3 split on q,
/// applied to all four llra/b/c/d. Pass-ID 14/15/16/17.
pub struct HostDeepDispatch;

impl OsdStrategy for HostDeepDispatch { /* extracted from Step 3 */ }

/// #63 target. **Stub only** in ε — returns `None`. The actual
/// `nord=1 + npre1=1` implementation lands when #63 is picked up;
/// keeping the type here means the trait surface stops moving
/// before that work begins.
#[cfg(feature = "fft-rustfft")]
pub struct WsjtxFaithful;

#[cfg(feature = "fft-rustfft")]
impl OsdStrategy for WsjtxFaithful {
    fn try_decode(&self, _llr_set: &LlrSet<f32>, _q: u32) -> Option<OsdHit> {
        None  // #63 fills this in
    }
}

/// Default for the current `process_one_candidate_inner` call site.
/// `WsjtxFaithful` is opt-in via a runtime knob in #63.
pub fn default_strategy() -> &'static dyn OsdStrategy {
    &HostDeepDispatch
}
```

`process_one_candidate_inner` Step 3 becomes:

```rust
if accepted.is_none() && matches!(depth, DecodeDepth::BpAllOsd) && q >= 12 {
    let llr_full_f32 = compute_llr(cs_scratch);
    if let Some(hit) = osd_strategy::default_strategy().try_decode(&llr_full_f32, q) {
        accepted = Some((hit.bp, hit.pass_id));
    }
}
```

Design constraints honoured:

- **No behavioural change in ε.** `default_strategy()` returns
  `HostDeepDispatch`, which is bit-identical to the inline
  Step 3 code (same LLR set, same dispatch, same pass-IDs).
  Golden recall holds by construction.
- **Static dispatch is fine.** The trait stays object-safe and
  uses `&dyn` here because there is exactly one call site and
  swapping at runtime (per #63) is the future flexibility we
  want. If profiling shows the vtable matters we can switch to
  a generic later — but Step 3 only fires when q≥12 + BP
  failed, so it's a cold path.
- **No new `pub` surface.** `osd_strategy` is `pub(crate)` from
  the parent `decode_block`; #63 will decide whether to
  re-export the trait when it ships its runtime knob.

### ε.3 — Why the façade entry points stay at the parent file

`decode_block`, `decode_block_tuned`, `decode_block_with_ap{,_tuned}`,
`decode_block_into{,_tuned}`, and the private
`decode_block_multipass` form one coherent unit that wires every
stage together. Moving them into a submodule (e.g.
`decode_block/entry.rs`) would force a `pub use` chain through the
parent for every public entry, which is exactly the kind of
re-export ceremony Rust 2018+ inline modules let us avoid.
Keeping them at the parent also keeps `#[cfg(test)] mod tests`
co-located with the only function the tests actually call into.

### ε.4 — Submodule visibility map

The split rule: **submodules expose to siblings via `pub(super)`
where re-exports aren't needed**, and the parent uses `pub use`
only for items the external caller survey above flagged.

| Submodule item                                                                                                                                                                              | Visibility            | Reason                                                  |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------|---------------------------------------------------------|
| `Spectrogram`, `compute_spectrogram`, `coarse_sync*`, `precompute_coarse_allsum*`, `coarse_allsum_len`, `fill_symbol_spectra*`, `symbol_spectra_direct*`, `SymMask`, `BASIS_SCRATCH_LEN`, `RefinedCandidate`, `refine_candidates_into`, `sync_quality_block0`, `process_candidates*` family, `xsnr2_db_simple`, `AudioSample`, `NFFT_SPEC`, `DEFAULT_Q_THRESH` | `pub` (re-exported)   | On external-caller survey list                          |
| `fill_coarse_allsum`, `coarse_sync_inner`, `fill_symbol_spectra_via_cd0`, `refine_candidates_with`, `pass1_limit`, `bp_step_select`, `WSJTX_NHARDERRORS_MAX`                                 | `pub(super)`          | Only called from sibling modules; never pub             |
| `recompute_nsync`, `recompute_snr_xsnr2`                                                                                                                                                    | `pub(super)`          | Only called from `decode_block_multipass`               |
| `process_candidates_tuned_with_ap`, `process_candidates_with_ap`                                                                                                                            | `pub(super)`          | Only called from sibling `process_candidates::*`        |
| `osd_strategy::OsdStrategy`, `osd_strategy::HostDeepDispatch`, `osd_strategy::default_strategy`                                                                                              | `pub(super)`          | Used by `process_one`; not re-exported until #63        |
| `process_one_candidate_inner`                                                                                                                                                               | `pub(super)`          | Already `pub(super)` today; stays the same              |
| `params::LlrT`, `params::ratio_eps`, `params::sync_lag_s`                                                                                                                                   | `pub(super)`          | Shared scalar / runtime tunables                        |

**`pub`-but-`#[doc(hidden)]` items kept verbatim:** every
`#[doc(hidden)]` entry in the survey continues to be
`#[doc(hidden)] pub` after ε. We are not relitigating which
items deserve to be in the documented surface — that is a
separate decision (#72 / #74 follow-ups). ε is mechanical.

**Items that could become `pub(crate)` later but stay `pub`
for ε:** `symbol_spectra_direct`, `symbol_spectra_direct_into`,
`process_candidates`, `process_candidates_tuned` — the external
caller survey did not surface a non-`decode_block.rs` user, but
they are part of the historically-stable surface that
`#[doc(hidden)] pub` exists to keep cheap. Re-classify under #72.

### ε.5 — Sequenced execution plan

Designed so each step is a self-contained PR that compiles + tests
green on its own. Numbered steps map to commit boundaries; the
whole sequence is one Issue + one PR thread.

0. **ε.0 (this section)** — design doc ratified. **Already
   shipped in the δ docs sync sweep.** No code touched.
1. **ε.1** — Move `params`, `audio_sample`, `spectrogram` into
   submodules. Smallest, most independent piece; touches no
   logic. Updates `pub use` chain at parent. CI: `cargo check`
   on every feature row of the cfg matrix, `cargo test`,
   `cargo test --features fixed-point`, Core2 +
   S3 `cargo build --release` (no flash needed).
2. **ε.2** — Extract `coarse_sync`. Largest single move
   (~530 lines) but logically isolated. Same CI gates.
3. **ε.3** — Extract `fill_symbol_spectra`. Big cfg matrix
   here (fft-rustfft × fixed-point × generic-Sc); careful that
   every `#[cfg(...)]` arm still has a path through.
4. **ε.4** — Extract `refine_candidates` + `snr`.
5. **ε.5** — Extract `process_candidates` (the 8-entry-point
   family) and the `bp_step_select` helper.
6. **ε.6** — Extract `process_one` (split out
   `process_one_candidate_inner`).
7. **ε.7** — Land `osd_strategy.rs` with `HostDeepDispatch` as
   the only inhabitant; refactor Step 3 to dispatch through it.
   **Golden recall must be byte-identical** before and after
   (qso3_busy AP-off 7/8, AP-on 5/6 extras; full reference
   suite). This is the step that creates the seam #63 will
   later fill with `WsjtxFaithful`.
8. **ε.8** — Pub-map tighten (the items currently `pub` only
   because they were reached cross-module — re-classify per
   §ε.4). Coordinate with #72 if that lands first.

`ε.2`-`ε.7` are each one-day moves; the full sequence is
estimated at 5-6 working days plus regression runs. **Each
step must hold:**

- `cargo test` green on host (default feature set).
- `cargo test --features fixed-point` green.
- `cargo check --no-default-features --features <every-row>`
  green across the cfg matrix (`fft-rustfft` × `fixed-point` ×
  `nstep-half`).
- Core2 + S3 `cargo build --release` green.
- WSJT-X reference recall: qso3_busy AP-off ≥ 7/8 host, no
  regression in JTDX-extras count, embedded recall unchanged.
- Build-size delta < ±2 KB on the embedded preset
  (`embedded-poc/m5stack-s3` release).

### ε.6 — Pre-flight checklist for the implementer

Before starting ε.1 in code:

- [ ] Confirm `#61` (Core2 → S3 fold-in) is either merged or on
  a branch that won't conflict with `decode_block/` directory
  creation.
- [ ] Branch from `main` not the active δ branch.
- [ ] Snapshot `cargo test` baseline output to `logs/epsilon-baseline.txt`
  so each step can diff against it.
- [ ] Snapshot Core2 + S3 build-size baselines
  (`ls -l target/.../release/*.elf`) for the ±2 KB acceptance check.
- [ ] Open one tracking Issue ("ε decode_block.rs restructure"); each
  step in §ε.5 is one commit on a single PR off that issue.

## Out of scope for this cleanup

- Implementing #61 (Core2→S3 unification) — wait for weekend
  hardware bring-up.
- Implementing #63 (OSD WSJT-X faithfulness / npre1 precoding) —
  ε creates the seam, then #63 implements against it.
- New protocol work (#23 FST4, #24 JT65) — independent track,
  schedule after ε lands.
