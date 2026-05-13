# Codebase cleanup plan — 2026-05

Single-target focus: production users are on m5stack-s3-app; Core2
hardware bring-up resumes weekend 2026-05-17/18 under issue #61
(Core2→S3 unification). Until that lands, this document tracks the
intermediate cleanup so the integration starts on a tidy base.

## Stages and ordering

```
γ (scaffolding cleanup) ─┐
                         ├─→ β (feature/cfg cleanup) ─→ δ (docs sync) ─→ ε (decode_block.rs restructure)
γ ───────────────────────┘
```

Each later stage assumes the earlier stages have landed:

- γ retires dead code first so β/δ/ε see a smaller surface.
- β narrows the feature matrix so δ docs and ε refactor know which
  cfg combos are actually shipped.
- δ ratifies the post-β documentation snapshot so ε's larger
  refactor lands on stable narrative ground.
- ε is the only week-scale item; γ/β/δ are 1–3 days each.

## γ — scaffolding cleanup (this PR)

**Done 2026-05-13.**

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

## β — feature flag / cfg cleanup (weekend)

Survey targets (all `mfsk-core/src/`):

1. **`fft-extern` vs `fft-rustfft`** — scattered across decode_block.
   List every `#[cfg(feature = "fft-rustfft")]` /
   `#[cfg(feature = "fft-extern")]` site and confirm both branches
   are reachable from a live build target. Anything that's only
   reachable from a retired path (e.g. m5stack-core2 dual_core
   diag) can collapse.
2. **`fixed-point` × `fft-rustfft` combinations** — four-way matrix
   (`{fixed-point, !fixed-point} × {fft-rustfft, !fft-rustfft}`).
   Verify each cell has at least one production caller. The host
   `(fixed-point, fft-rustfft)` cell is the one with the
   `fill_symbol_spectra_into` wrapper that warns
   `_basis_re`/`_basis_im` unused — likely dead.
3. **`profile-coarse` / `nstep-half`** — confirm both are still on
   the embedded-rx preset; if one has been folded into core
   behaviour, drop the cfg gate.
4. **`parallel`** — host-only rayon path. Verify the workspace
   still builds it; if not, the inactive-code diagnostic at
   decode.rs:655/1170 will keep firing.
5. **Compiler-visible dead code** carrying over from γ:
   - `recompute_nsync` (decode_block.rs:1891) — `#[warn(dead_code)]`
   - `recompute_snr_xsnr2` (decode_block.rs:2072) — same
   - `ALLSUM_WIN` const (embedded-shared/stage1_inc.rs:53)
   - Unused `basis_re`/`basis_im` parameters (decode_block.rs:2698–2699)

Acceptance for β:

- `cargo build` with each documented preset (`embedded-rx`,
  default host, host + fft-rustfft + uvpacket) finishes with zero
  warnings on the touched files.
- Surviving cfg gates are guarded by `#[cfg(...)]` only when both
  branches actively ship.

## δ — documentation sync (after β)

1. `docs/ROADMAP.md` — currently mixes 0.6.0/0.6.1/0.6.2 status
   with the original post-0.5.12 plan. After β stabilises the
   feature surface, condense the legacy section and update
   "Open follow-ups" to reference the current issue set
   (#23, #24, #58, #61, #63, #64, #65).
2. `README.md` (root) — verify the badges / build commands match
   the post-β feature set.
3. `docs/LIBRARY.{md,ja.md}` — cross-check the §3 "four decoder
   strategies" table against the actual rx.rs / decode.rs entry
   points (post-PR #53 we standardised "AP-hint BP" terminology,
   so the table examples and the FFI table at LIBRARY.ja.md:857
   should both use that wording).
4. Embedded `CLAUDE.md` files (`m5stack-core2`, `m5stack-s3`,
   `m5stack-s3-app`, `embedded-shared` if it grows one) —
   factor out the duplicated "espup / export-esp.sh / espflash"
   one-time-setup section to a single shared CLAUDE.md at
   `embedded-poc/CLAUDE.md`.
5. Per-source-file docstrings citing retired paths
   (`subtract_signal_weighted`, `qsb_partial_gain`, etc. — already
   removed in 0.6.2 per ROADMAP) — grep and update.

## ε — decode_block.rs restructure (week-scale)

`mfsk-core/src/ft8/decode_block.rs` is ~3500 lines and contains the
host fft-rustfft path, the embedded fixed-point path, the host
fixed-point bench path, coarse_sync, compute_spectrogram,
fill_symbol_spectra family, process_candidates*, and OSD dispatch.

Restructure goals (not yet committed to as design):

1. Split per pipeline stage —
   `decode_block/{spectrogram, coarse_sync, fill_symbol_spectra,
   process_candidates}.rs`. Keep `decode_block::run` as the public
   facade.
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
  external API (#[doc(hidden)] entries we still need to expose)
  unchanged.
- Build size unchanged within ±2 KB on the embedded preset.

## Out of scope for this cleanup

- Implementing #61 (Core2→S3 unification) — wait for weekend
  hardware bring-up.
- Implementing #63 (OSD WSJT-X faithfulness / npre1 precoding) —
  ε creates the seam, then #63 implements against it.
- New protocol work (#23 FST4, #24 JT65) — independent track,
  schedule after ε lands.
