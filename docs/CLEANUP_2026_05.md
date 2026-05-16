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

**Done 2026-05-14 (PR #70).** All five sub-tasks below landed in
the single PR: ROADMAP refresh (δ.1), root README quick-start
refresh (δ.2), LIBRARY.{md,ja.md} AP-hint terminology cross-check
(δ.3), embedded CLAUDE.md consolidation (δ.4), source-file docstring
grep for retired paths (δ.5).

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

**Done 2026-05-17 across 6 stacked PRs.** Shipped in 0.6.3.
Final shape (from `mfsk-core/src/ft8/decode_block.rs` original
3,517 lines → 416 line parent + 6 stage submodules):

  | file | lines | role | PR |
  |---|---:|---|---|
  | `decode_block.rs` | 416 | parent / facade | — |
  | `decode_block/types.rs` | 184 | audio sample + tunables | #77 |
  | `decode_block/spectrogram.rs` | 357 | `Spectrogram` + `compute_spectrogram` | #83 (re-opened #78) |
  | `decode_block/coarse_sync.rs` | 537 | Costas search + allsum | #79 |
  | `decode_block/fill_symbol_spectra.rs` | 601 | per-symbol DFT family | #80 |
  | `decode_block/process_candidates.rs` | 1,596 | engine + facade impls | #81 |
  | `decode_block/osd_strategy.rs` | 117 | OSD dispatch (`#63` hook) | #82 |

ε.6's OSD-strategy seam is now load-bearing for issue #63's
WSJT-X-faithful OSD work (also 0.6.3).

---

### ε — original design notes (kept for historical context)

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

## Out of scope for this cleanup

- Implementing #61 (Core2→S3 unification) — wait for weekend
  hardware bring-up.
- Implementing #63 (OSD WSJT-X faithfulness / npre1 precoding) —
  ε creates the seam, then #63 implements against it.
- New protocol work (#23 FST4, #24 JT65) — independent track,
  schedule after ε lands.
