# Changelog

## 0.6.7 — fix coarse_sync wasm32 runtime panic

0.6.6 left `std::time::Instant::now()` calls in `coarse_sync` gated
only by `feature = "std"` (3 timestamps + a final diagnostic block).
Any consumer building for `wasm32-unknown-unknown` with the default
`std` feature panicked the first time stage-2 ran — `Instant::now` is
unimplemented on that target. `webft8` and any future WASM PWA
consumer were affected; embedded (esp-idf) was not because esp-idf
provides a working clock.

### Fixed

- **`coarse_sync.rs`**: the 4 timestamp reads + the `eprintln!`
  diagnostic block are now gated on `#[cfg(feature = "profile-coarse")]`
  rather than `#[cfg(feature = "std")]`. WASM builds without
  `profile-coarse` (the normal case — feature is off by default)
  compile the timing code out entirely and no longer touch
  `Instant::now`.

### Removed

- `MFSK_PROFILE_COARSE` env-var dispatch in `coarse_sync` (added in
  `#130` as a host convenience to flip profiling without rebuild, but
  never exercised by any in-tree script, test, or CI job — the same
  diagnostic is reachable via `--features profile-coarse`). Embedded
  apps already enabled the feature flag directly, so their serial-log
  output is unchanged.

### Notes

- `mfsk-ffi-ft8` bumped to 0.6.7 in lock-step (no behavioral change in
  the FFI itself — it tracks `mfsk-core` so users see matching version
  numbers in release artifacts).

## 0.6.6 — cold-start slot bootstrap from coarse_sync top-5 DT median

Embedded `m5stack-s3-app` auto-sync previously consumed only
confirmed-decode DT median, so a cold start (or post-reset) with
zero confirmed decodes left the slot mis-aligned indefinitely
("BtnA required" in the log) until the operator intervened —
defeating the mountain-top "leave the laptop at home" use case
when the band is quiet for the first slot. WebFT8 faces the same
constraint with no manual sync available at all.

### Added

- **`mfsk_core::core::sync::bootstrap_dt_median(cands, top_k)`**
  (public): DT median over the top-`K` highest-score
  `SyncCandidate`s. Empirically — on `qso3_busy` + `191111`
  reference captures — `K=5` tracks confirmed-decode DT median
  within ±70 ms, while `K=10`/`20` wash out into 200–275 ms
  misses under false-candidate noise. Gated by new integration
  test `tests/ft8_coarse_sync_bootstrap.rs` (100 ms `|Δ|`
  budget). Uses `select_nth_unstable_by` for O(N) top-K
  partition + O(K log K) winner sort.

### Changed

- `embedded-shared::dual_core::SpeculativeOut` gains
  `bootstrap_dt_med: Option<f32>`, computed in
  `run_speculative_slot` right after pass1 returns and before
  the audio-window partition. `n_pass1 == 0` slots get `None`.
- `m5stack-s3-app/decode_pipeline.rs` auto-sync gains a fourth
  branch: when `n_dec == 0 && best_n == 0 && bootstrap_dt_med
  .is_some()`, the helper's value drives the bootstrap shift
  but `best_n` stays at 0, so the first confirmed-decode slot
  reclaims the HWM via the existing path. Soft anchor only.
- `m5stack-core2-app` / `m5stack-cores3-app` / `embedded-shared
  ::apps::rx_wavsim` consumers thread `bootstrap_dt_med`
  through as `_` (no slot-drift correction needed in wav_sim
  mode).

### Fixed

- Stale comment in `m5stack-s3-app/src/audio.rs` claimed the
  consumed shift hint was "pass1 candidate DT median"; in fact
  the pre-0.6.6 producer was confirmed-decode median. Corrected.

### Notes for WebFT8

`bootstrap_dt_median` is the same helper WebFT8 calls from its
cold-start path — no platform-specific wrapping required. Pass
the raw `Vec<SyncCandidate>` from either
`decode_block::coarse_sync` or `core::sync::coarse_sync::<Ft8>`.

## 0.6.5 — crates.io surface refresh (M5StickS3 FT8 controller PoC)

Non-functional patch. Refreshes the project's discoverable surface
on crates.io and docs.rs to reflect that the embedded port is no
longer aspirational:

- `mfsk-core/Cargo.toml` `description` rewritten to lead with the
  working `embedded-poc/m5stack-s3-app` M5StickS3 FT8 controller
  (LCD UI, BLE CI-V to IC-705, acoustic mic, QSO FSM, ~1.2 s
  post-SlotEnd decode on Xtensa LX7).
- `mfsk-ffi-ft8/Cargo.toml` `description` notes M5StickS3 end-to-end
  verification.
- `mfsk-core/src/lib.rs` "Why this exists" gains a fourth bullet
  for the handheld-controller use case, with cross-links to the
  M5StickS3 source crate and `docs/MANUAL_M5STICKS3.md`.
- `README.md` opens with a hero photo of the device decoding five
  real on-air FT8 callsigns from a single 15 s slot
  (`docs/assets/m5sticks3-ft8-decode.jpg`).

Zero behaviour change vs 0.6.4 — no source code modifications, no
new features, no bug fixes. Pure metadata + docs.

## 0.6.4 — Goertzel per-symbol DFT (drop BASIS 120 KB internal-DRAM scratch)

Non-breaking minor — the embedded path's per-symbol DFT moves from
the BASIS sin/cos-table dot product to a generalised Goertzel
recursion. Same **magnitude** as the DFT bin
`|Σ x[n] exp(-jωn)|` for each FT8 tone — the complex output
differs by a fixed phase rotation per the standard generalised-
Goertzel `s[N-1] - exp(-jω)·s[N-2]` extraction, but FT8 downstream
consumes `|cs|²` so the rotation is irrelevant. **Zero caller-
provided scratch**, +0.16..+0.63 dB SNR improvement (f32 Goertzel
has more precision than the pre-existing Q15 BASIS dot product).

The 120 KB internal-DRAM win unblocks downstream embedded work that
couldn't fit alongside BASIS — most immediately, M5StickS3 Qso-mode
I2S bidirectional DMA descriptor allocation (`i2s_alloc_dma_desc:
allocate DMA buffer failed` on pre-0.6.4 firmwares).

### Added

- **`fill_symbol_spectra_goertzel`** (public): generalised
  Goertzel recursion for per-symbol DFT. Output-compatible with
  `fill_symbol_spectra_into` at the `cs: &mut [[Cmplx<f32>; 8]; 79]`
  level — modulo the phase rotation noted above — so downstream
  `sync_quality` / LLR / BP / OSD consume it without changes. **Call
  signature is shorter, not identical**: the BASIS `basis_re` /
  `basis_im` scratch slice arguments go away (5 args vs 7),
  reflecting the zero-scratch property. Sample-outer / tone-inner
  loop ordering for FPU pipeline parallelism on Xtensa LX6/LX7 —
  measured S3 stage3 1.47 s (matches BASIS asm dot-product speed)
  with zero internal-DRAM scratch.

### Changed

- **`process_candidates_into_with_cs_scratch_tuned`** and
  **`refine_candidates_into`**: the `not(fft-rustfft)` (embedded)
  branch now calls `fill_symbol_spectra_goertzel` instead of
  `fill_symbol_spectra_into`. Per-cell output is mathematically
  equivalent. Function signatures unchanged — BASIS scratch args
  still accepted for API back-compat with embedded callers
  (`embedded-shared::dual_core::stage3_split` /
  `pass2_split` thread them through). The args are ignored on the
  new path; will be removed in 0.7.0.

### Fixed

- **`fixed-point` now implies `nstep-half`**. The two features were
  independent in `[features]` but always co-enabled on every
  embedded target. Decoupling silently gave host `fixed-point`
  builds NSTEP=NSPS/4 while embedded used NSPS/2 — a completely
  different time-grid that made `host decode_block_into` produce a
  different decoded set than the embedded path on busy bands (e.g.
  4 decodes vs 7 on qso3_busy.wav). Coupling fixes the host-as-
  embedded-simulator contract.
- **`Plan3840Sc16`** (`core::dsp::fft_mixed_3840_sc16`): import
  `num_traits::Float` under `#[cfg(not(feature = "std"))]` so
  `f32::round()` at the i16 clamp resolves on `no_std` builds.
  Phase 1.7.7a follow-up — fixed the `Build (ft8)` / `Build (wspr)`
  / `Build (alloc ft8 fft-extern fixed-point)` CI matrix entries
  that had regressed.

### Deprecated (will remove in 0.7.0)

- `BASIS_SCRATCH_LEN`, `fill_symbol_spectra_into`,
  `symbol_spectra_direct_into`, `fill_symbol_spectra_into_generic`,
  `mfsk_ft8_basis_scratch_len` (FFI export). Superseded by
  `fill_symbol_spectra_goertzel` + zero-scratch call sites. Kept
  through 0.6.x for downstream `mfsk-ffi-ft8` consumers; the
  `dual_core::{init, pass2_split, stage3_split}` BASIS scratch args
  drop at the same time.

The `#[deprecated]` attribute itself is not added in this release —
applying it would trip the workspace-wide `-D warnings` CI gate via
the still-extant internal callers (FFI shim, embedded apps,
regression tests, plus the BASIS-internal `fill_symbol_spectra_into
→ ..._into_generic → symbol_spectra_direct_into` chain). The
attribute lands in the same 0.7.0 commit that drops the symbols,
with the call-site removals as the gating prep work. Until then
this CHANGELOG entry and the public-API doc comments are the
authoritative deprecation notice.

### Other notes

- Bundled `mfsk-ffi-ft8` from 0.6.3 to 0.6.4 (workspace-internal
  version tracking; the crate is `publish = false` — distributed
  via GitHub Releases, not crates.io).
- Phase 1.7.7 host validation tests (`tests/ft8_goertzel_vs_basis`,
  `tests/ft8_decode_block_fixed_point_baseline`) ship in this
  release as regression guards.

## 0.6.3 — `decode_block.rs` per-stage split + WSJT-X-faithful OSD

Non-breaking patch. Two interleaved storylines:

1. **Structural** — `mfsk-core/src/ft8/decode_block.rs` (3,517 lines)
   carved into a 7-sibling submodule tree (`docs/historical/CLEANUP_2026_05.md`
   ε plan, shipped as PRs #77 / #83 / #79 / #80 / #81 / #82). Pure
   refactor, zero behaviour change.
2. **OSD WSJT-X faithfulness** — `osd_strategy::try_fallback` (the
   host BpAllOsd Step-3 hook ε.6 carved out) rewired from mfsk-core's
   pre-existing brute-force `osd_decode` (ndeep=2, 4,186 patterns) /
   `osd_decode_deep(_, 3, _)` (ndeep=3, ~121k patterns) dispatch to
   the WSJT-X-faithful `nord=1 + npre1=1` (ndeep=2) /
   `+ npre2=1, ntau=14` (ndeep=3) entries, plus the 4 missing
   `ft8b.f90:422-459` post-decode validity gates and one
   `nsync<=10 && xsnr<-24` clamp-ordering bug fix. Issue #63.

### Improved

- **OSD pattern coverage reduced by ~25-700×** without WSJT-X /
  JTDX golden regression. New `osd_decode_npre1` (ndeep=2, ~165
  post-gate patterns) replaces the brute-force `osd_decode` on the
  FT8 OSD path; new `osd_decode_npre1_npre2` (ndeep=3, npre1 + a
  16,384-bucket `ntau=14` G-column XOR hash table) replaces
  `osd_decode_deep(_, 3, _)`. Both are WSJT-X line-for-line ports
  of `osd174_91.f90` with their `ntheta=10` / `ntheta=12`
  early-reject gates intact.
- **Four missing post-OSD validity gates** ported from
  `ft8b.f90:422-459`: all-zero codeword reject, message-type
  validity (`i3 > 5 || (i3==0 && n3 > 6)`), quirky free-text reject
  (`i3==0 && n3==2`), `unpack77` success guard. Applied in the
  `decode_block_multipass` retain_mut on host f32 (fixed-point
  doesn't surface OSD-pass results).
- **`nsync<=10 && xsnr<-24` gate clamp-ordering fix**.
  `recompute_snr_xsnr2` used to pre-clamp its return to `>= -24 dB`
  before the gate could test it — the source comment "on qso3_busy
  it had no effect in any case" already flagged the dead-letter.
  Function now returns raw xsnr; caller gates first, then clamps for
  display. Matches WSJT-X bit-for-bit on the order of operations.
- **Three `qso3_busy.wav` OSD CRC-luck phantoms eliminated**
  (`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25, `CQ EA2BFM IN83`
  e=31). The npre1 / npre2 / WSJT-X post-OSD gates all failed to
  filter these because their `nsync >= 13` puts them above
  `ft8b.f90:456`'s bail-out — they would also surface from WSJT-X
  proper at the same `max_cand=60` / `q_thresh=0.8` settings.
  mfsk-core adds an OSD-pass-specific `hard_errors > 22` ceiling
  (`OSD_HARDERRORS_MAX` in `decode_block/osd_strategy.rs`) that
  matches the empirical phantom boundary — `OSD_HARDERRORS_MAX` is
  the only mfsk-core-specific deviation from WSJT-X in this
  release (WSJT-X uses 36 universally; we restrict to 22 only on
  the OSD path because BP can legitimately produce e=20 cases like
  `N1PJT HB9CQK -10`).
- **WSJT-X 8-entry recall on `qso3_busy.wav`**: 7/8 maintained.
- **JTDX 18-entry recall** on `qso3_busy.wav`: **13/18** (was 16/18
  pre-0.6.3 — the dropped 3 are exactly the phantoms above; the
  JTDX golden inadvertently included them).
- **AP-on multipass extras** on `qso3_busy.wav`: **4/6** (was 5/6
  pre-0.6.3 — `CQ EA2BFM IN83` no longer surfaces because the OSD
  gate rejects it pre-AP rescore).

### Changed (internal — non-breaking)

- **`decode_block.rs` per-stage split.** Single 3,517-line file
  carved into 7 sibling submodules. Zero behaviour change.
  External callers see the same public paths via re-exports.

  | file | lines | role |
  |---|---:|---|
  | `decode_block.rs` | 423 | parent / facade |
  | `decode_block/types.rs` | 184 | audio sample + tunables |
  | `decode_block/spectrogram.rs` | 357 | `Spectrogram` + `compute_spectrogram` |
  | `decode_block/coarse_sync.rs` | 537 | Costas search + allsum |
  | `decode_block/fill_symbol_spectra.rs` | 601 | per-symbol DFT family |
  | `decode_block/process_candidates.rs` | 1,589 | engine + facade impls |
  | `decode_block/osd_strategy.rs` | 139 | OSD dispatch (`#63` hook) |
- **OSD scaffolding shared across ndeep=2 and ndeep=3.** New
  private `osd_setup_ldpc174_91` / `osd_npre1_pass` /
  `osd_result_from_best` helpers factor the WSJT-X-faithful setup /
  enumeration / result assembly so the ndeep=3 entry reuses them
  zero-cost and ndeep≥4 (nord-2/3/4 outer loop) can land later on
  the same scaffold.

### Added (public API — additive, FT8)

- `mfsk_core::fec::ldpc::osd::osd_decode_npre1(llr) -> Option<OsdResult>`
  — WSJT-X ndeep=2 entry. Replaces internal use of `osd_decode` on
  the FT8 OSD path; `osd_decode` stays for callers that prefer
  brute-force ndeep=2.
- `mfsk_core::fec::ldpc::osd::osd_decode_npre1_npre2(llr) -> Option<OsdResult>`
  — WSJT-X ndeep=3 entry. Replaces internal use of
  `osd_decode_deep(_, 3, _)` on the FT8 OSD path.

### Hardware (embedded)

- **Issue #61 closed.** `embedded-poc/m5stack-core2/` bench crate
  retired; new `embedded-poc/m5stack-core2-app/` consumes the new
  `embedded-poc/mfsk-app-shared/` carve-out (shared with
  `m5stack-s3-app`). Three-phase sequence in PR #76: Phase 1
  carve-out, Phase 2 Core2 app crate, Phase 3 bench retirement.
  Verified on M5StickS3 (S3 LX7, 7/8 golden) and M5Stack Core2
  (LX6, 7/8 per-slot from `qso3_busy.wav` via `wav_sim`, 400 alive
  ticks, stable heap).

### Infra

- **CI `Test (features = full)` job split into 5 parallel matrix
  entries** (#85). Old single ~22 min job → `Test (default)` +
  `Test (ft8 sweeps)` + `Test (q65 sweeps)` + `Test (uvpacket
  sweeps)` + `Test (lib + ft4 / fst4 / FST4 gated)`. Wall-clock
  ~10 min on a touch-everything PR.
- **`dorny/paths-filter@v3` gate** on the sweep matrix entries
  (#85). PRs that don't touch decoder source (`src/{ft8,fec,msg,
  core}/`, `Cargo.toml`, `.github/workflows/`) skip the slow sweep
  jobs entirely — ~3 min total for refactor / doc / embedded-poc-only
  PRs. Push to main always runs every sweep (post-merge regression
  safety net).

## 0.6.2 — host pipeline matches embedded subtract + cs-source

Non-breaking patch. After v0.6.1 unified the per-candidate inner,
the host pipeline still diverged from embedded `decode_block` on
two upstream axes: the cs spectra source (host computed cs from
cd0, embedded fills from 12 kHz audio) and the SIC subtract
function (host used a constant-amplitude weighted subtract,
embedded uses WSJT-X-style channel-aware LPF subtract). Both
divergences are eliminated in 0.6.2; host now matches embedded
bit-for-bit on the cs values it feeds the unified inner.

### Improved

- **Host `decode_frame_subtract_with_ap` recall on busy bands.** On
  `qso3_busy.wav` AP-on with operator context (`mycall=K1JT`,
  `hiscall=HA0DU`):
  - **14 → 18 decodes** total.
  - **JTDX-extras coverage 1/6 → 5/6.** Surfaces CQ EA2BFM,
    KD2UGC F6GCP, K1BZM EA3CJ on top of the F5RXL CQ already
    caught in 0.6.1. The single remaining miss
    (K1BZM DK8NE -19) requires a wider AP-list / callsign hash
    table — out of scope for this patch.
- **Host single-pass `decode_frame_with_ap`** unchanged at 14/18
  (single-pass doesn't benefit from the improved subtract).
- **Embedded `decode_block`** unchanged at 16/18 (the changes touch
  only the host driver path).

### Changed (internal — non-breaking)

- `ft8::decode::process_candidate` — cs-source rewired from
  `symbol_spectra(cd0, i_start)` to `fill_symbol_spectra(audio,
  refined.freq_hz, refined.dt_sec, SyncOnly+DataOnly)`, matching
  embedded's per-symbol-region DFT directly on 12 kHz audio. cd0
  still used for `fine_refine_3stage` and `sync_cv` computation.
- `decode_frame_subtract_with_ap`,
  `decode_frame_subtract_with_known_and_ap_inner`, and
  `decode_sniper_sic` — sequential subtract switched from
  `subtract_signal_weighted` (constant amplitude × QSB-aware
  partial gain) to `subtract_signal_lpf` (WSJT-X-style channel-
  aware LPF subtract, matching `decode_block_multipass`).

### Removed (technically breaking, in practice no consumers)

- `mfsk_core::ft8::subtract::subtract_signal` — was a thin wrapper
  around `subtract_signal_weighted(audio, result, 1.0)`. No
  in-tree, embedded-poc, or mfsk-ffi-ft8 callers (verified via
  `grep`).
- `mfsk_core::ft8::subtract::subtract_signal_weighted` — replaced
  everywhere by `subtract_signal_lpf` (which doesn't accept a
  partial gain — the QSB-aware attenuation it provided is no
  longer needed since LPF subtract handles channel variation
  natively).
- `mfsk_core::ft8::llr::symbol_spectra` — removed; was only used
  by `process_candidate` which now calls
  `decode_block::fill_symbol_spectra` directly. The protocol-
  generic `core::llr::symbol_spectra::<P>` (used by FT4 / JT9 /
  uvpacket / WSPR pipelines) is untouched.
- `qsb_partial_gain` (private) — only consumer was the now-removed
  weighted subtract.

The removed `subtract_signal*` and `symbol_spectra` items had no
out-of-tree callers in the workspace; the FT8 subtract public
surface is now `subtract_signal_lpf` + `refine_signal_freq`.

### Deferred / scoped out

- **Embedded recall++** beyond 16/18 on `qso3_busy.wav`. The
  remaining 2 misses (K1BZM DK8NE -19, WA2FZW DL5AXX -15) are
  intrinsically below `decode_block::coarse_sync`'s sync-min
  threshold even at sync_min=0.5 / max_cand=200; a WSJT-X-faithful
  dual-window NMS prototype was tried during the v0.6.2 plan but
  produced identical 16/18 hit count, so it wasn't worth the code
  volume. Surfacing those entries needs either a different sync
  algorithm or a wider AP-list — both deferred.
- **Embedded fixed-point AP** support (apmag in Q3i8) — still
  tracked for 0.7.x.

### Test floor changes

- `tests/ft8_qso3_apon_recall.rs::JTDX_EXTRAS_HARD_FLOOR_MULTIPASS`:
  1 → **5** (locks in the host multipass recall jump).

## 0.6.1 — host/embedded per-candidate pipeline unification

Non-breaking patch. After v0.6.0 unified the FT8 coarse-sync path,
the *per-candidate* decode body still diverged: host's
`process_candidate` (decode.rs) ran a hand-written eager-LLR + BP +
OSD + AP staircase; embedded's `process_candidates_with`
(decode_block.rs) ran a lazy-LLR staircase with tighter sync gates.
On busy bands (e.g. `qso3_busy.wav`), embedded `decode_block` caught
16/18 of the JTDX golden but host `decode_frame_with_ap` only 14/18.

This patch extracts a shared `process_one_candidate_inner` from
`process_candidates_with` and routes the host pipeline through it.
Both paths now run the *exact same* LLR / BP / OSD / AP staircase
on the per-candidate cs spectra. The remaining 14-vs-16 gap on this
WAV is upstream of the inner — the cs-source itself differs (host
`core::llr::symbol_spectra(cd0, i_start)` vs embedded
`fill_symbol_spectra(audio, freq, dt, mask)`) and is tracked as a
separate refactor.

### Added (host f32 only — `#[cfg(feature = "fft-rustfft")]`)

- `decode_block_with_ap(audio, freq_min, freq_max, sync_min, depth,
  max_cand, ap_hint) -> Vec<DecodeResult>`. AP-aware variant of
  `decode_block`; mirrors its behaviour exactly when
  `ap_hint = None` and runs the full WSJT-X iaptype loop (5..12)
  per candidate when `Some(&ap)`. Preferred entry point for
  mountain-top apps that want full AP rescue from a single call.
- `decode_block_with_ap_tuned(...with bp_max_iter + strictness)` —
  same with runtime-tunable BP iterations and strictness.

Embedded fixed-point builds (`#[cfg(feature = "fixed-point")]`)
keep their existing iaptype-1-only hardcoded path; full AP for that
build is deferred — `apmag = max(|llra|)*1.01` is f32-domain and
`ApHint::build_ap` heap pressure exceeds Core2 stage-3 budget.

### Changed (internal — non-breaking)

- Embedded OSD pass IDs shifted: 4/5/6/7 → 14/15/16/17, freeing the
  WSJT-X-canonical 5..12 range for the AP iaptype loop. Pre-flight
  grep confirmed no in-tree test gates on `pass == N` for `N ≥ 4`;
  out-of-tree consumers reading `DecodeResult::pass` for OSD-decoded
  results need to read 14/15/16/17 instead of 4/5/6/7.
- Host `process_candidate` body shrinks ~250 lines → delegate to the
  unified inner. Outer prelude (downsample, fine_refine_3stage,
  symbol_spectra, nsync gate, EqMode cs choice) preserved.
- The unified inner emits one decode per candidate after running
  `unpack77` + plausibility gate (the embedded path's behaviour).
  Old host path was lenient — it returned any CRC-converged
  77-bit codeword without unpacking. The change is what tightens
  busy-band recall vs host's old behaviour, but means
  *synthesised-with-arbitrary-bits* round-trip tests need a real
  FT8 message; updated 5 affected tests to use
  `pack77("CQ", "JA1ABC", "PM95")`.

### Improved

- Host `decode_frame_with_ap` and `decode_frame_subtract_with_ap`
  now share the WSJT-X-faithful staircase with `decode_block`. The
  qso3_busy.wav 14-vs-16 gap on the JTDX golden is structurally
  closed for everything *post* coarse-sync; only the cs-source
  divergence remains.

### Internal

- New private `process_one_candidate_inner` in `decode_block.rs`
  (`pub(super)`-visible to the FT8 module). Generic over `LlrT` so
  it serves both f32 (host + embedded f32-rustfft) and Q3i8
  (embedded fixed-point) builds with bit-identical behaviour when
  `ap_hint = None`.
- `process_candidates_with` collapsed into
  `process_candidates_with_ap` (the same body now takes
  `Option<&ApHint>`); call sites passing `None` are bit-identical to
  the prior shape.

## 0.6.0 — FT8 sync consolidation, refactor audit, AP iaptype 2

**Breaking** — bundled refactor that closes #40, #46, #48, and most of
the #49 cat A / B / C audit. After v0.6.0 the FT8 sync path has a single
WSJT-X-faithful implementation (`decode_block::coarse_sync`) and the
"`ft8::sync` namespace" wrappers that drifted in #40 are gone. The user-
visible behaviour change for callers already on 0.5.12 is small —
`freq_hint: Option<f32>` becomes a no-op on FT8 wide-band paths, and the
`ft8::sync` thin-wrapper functions are removed (out-of-tree callers must
turbofish the generic).

### Removed

- `mfsk_core::ft8::sync::coarse_sync` (#46) — was already a one-line
  trampoline. Use `mfsk_core::ft8::decode_block::{compute_spectrogram,
  coarse_sync}` directly:
  ```rust
  // before (0.5.12)
  let cands = mfsk_core::ft8::sync::coarse_sync(&audio, 200.0, 2800.0, 1.3, None, 50);
  // after (0.6.0)
  let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 2800.0);
  let cands = mfsk_core::ft8::decode_block::coarse_sync(&spec, 200.0, 2800.0, 1.3, 50);
  ```
- `mfsk_core::ft8::sync::{compute_spectra, fine_sync_power,
  fine_sync_power_split, refine_candidate, refine_candidate_double}`
  (#49 cat B). All five were one-line wrappers around
  `mfsk_core::core::sync::*::<Ft8>` — call the generic directly.

### Changed (behaviour, signatures unchanged)

- **`freq_hint: Option<f32>`** on every `decode_frame*` and
  `decode_sniper*` is silently ignored. `decode_block::coarse_sync` does
  not honour candidate-score promotion; sniper paths
  (`decode_sniper_ap`) constrain `freq_min`/`freq_max` ±250 Hz around
  `target_freq` so the loss is contained.
- **`symbol_spectra` / `score_costas_block` / `fine_sync_power*` /
  `refine_candidate*` take `i32` for sample-index parameters** (#40 / #45).
  The host wide-band path was casting `i_start = ((dt+0.5)*200) as
  usize`, saturating any negative-dt candidate to 0 and silently
  misaligning the symbol grid by ~1.75 symbols. Threading `i32` through
  with WSJT-X all-or-nothing per-symbol boundary checks (= the same
  behaviour `decode_block::fill_symbol_spectra_via_cd0` already had) lifts
  host AP-off recall on `qso3_busy.wav` from 5/8 → **7/8** (matches the
  embedded `decode_block`). Out-of-tree callers of these
  `core::sync` / `core::llr` / `ft4::refine_fine` functions need to update
  their callsites to pass `i32`.

### Added

- **AP iaptype 2 — mycall-only lock** (`ft8::decode::process_candidate`).
  Mirrors WSJT-X `lib/ft8/ft8b.f90` "MyCall ??? ???" pass: locks bits
  0–28 + i3=001 (~32 bits) without forcing call2 / grid. Triggered as
  a new pass (id 5) whenever `ApHint::call1` is set; surfaces follow-up
  replies whose sender isn't the call2 in operator context. JTDX uses
  this iaptype to land entries that pass 8 (mycall + dxcall) cannot
  reach. `check_result` already rejects decodes whose unpacked text
  doesn't contain the AP-locked call1, so iaptype 2 cannot leak phantoms.

### Improved

- **FT8 host wide-band recall on busy bands**. `qso3_busy.wav`:
  WSJT-X 8-entry golden 5/8 → 7/8 (CQ F5RXL IN94 -3 dB now decodes; 3
  phantoms above 2 kHz eliminated). JTDX 18-entry golden 13/18 → 16/18.
  AP-on JTDX-extras 0/6 → 1/6.

### Public API graduation (non-breaking)

- `decode_block::coarse_sync` and `compute_spectrogram` graduate from
  `#[doc(hidden)]` to officially public — they are the canonical FT8
  surface, not benchmarking aids (#48 step B).
- `decode_block::Spectrogram`, `coarse_sync_with_allsum`,
  `coarse_allsum_len`, `precompute_coarse_allsum` /
  `precompute_coarse_allsum_into`, `refine_candidates_into`,
  `process_candidates_into_with_cs_scratch` /
  `_with_cs_scratch_tuned` graduate as well — embedded consumers
  (`embedded-shared::dual_core` / M5Stack apps) and `mfsk-ffi-ft8` were
  already depending on them; the disclaimer wasn't accurate (#49 cat C).

### Internal

- **WAV-loader test consolidation** (#49 cat A). 14 near-duplicate
  RIFF/WAVE parsers across `tests/ft8_*`, `tests/q65_*`, `tests/fst4_*`,
  `tests/wspr_*`, `tests/ft4_*`, `tests/jt9_*` collapse into four
  helpers (`load_wav_i16` / `load_wav_f32` strict + `_opt` soft) in
  `tests/common/mod.rs`. Net 342 lines deleted.

### Fixed (additional)

- **Host `decode_frame_subtract_with_ap` is now WSJT-X-faithful**.
  Mirrors the structure of `decode_block::decode_block_multipass` (which
  is a faithful port of `lib/ft8/ft8b.f90:432-437`). Two changes from
  pre-v0.6.0:
  1. **Fixed `sync_min` across all 3 passes** (was sync_min × {1.0, 0.75, 0.5}).
     Progressive relaxation lets phantoms slip through later passes when
     SIC artefacts dominate the residual.
  2. **Sequential subtract within each pass** (was batch-after-pass).
     Each accepted decode immediately subtracts so the next candidate
     in the same pass sees a cleaner residual.

  Pass termination matches WSJT-X (skip pass 2 when pass 1 returned 0;
  skip pass 3 when pass 2 returned no NEW). No recall change on the
  qso3_busy.wav reference (the host's remaining 14/18 vs embedded's
  16/18 gap on that WAV is structurally downstream — see "Known
  limitation" below) but materially more faithful in shape and will
  matter on WAVs where SIC artefact phantoms differ.

### Deferred

- `decode_block_with_ap` (ROADMAP A0' / "AP-on extras parity"). The
  remaining 2-entry gap between embedded `decode_block` (16/18 of the
  JTDX 18-entry golden on `qso3_busy.wav`) and host `decode_frame*`
  (14/18) is **inside the per-candidate processing** —
  `process_candidates_tuned` (decode_block's path) and
  `process_candidate` (decode.rs's path) are separate decoder pipelines
  with different LLR / BP staircase / sync_quality gates, and the 3
  extra decodes embedded catches (KD2UGC F6GCP, CQ EA2BFM, K1BZM EA3CJ)
  pass `process_candidates_tuned`'s gates but not `process_candidate`'s.

  Unifying the two pipelines is a non-trivial refactor (the embedded
  path is fixed-point-aware, the host path uses f32 throughout) — it's
  a 0.7.0-class change. Tracked for that release. v0.6.0 ships with
  the host pipeline structurally aligned to the WSJT-X-faithful
  multipass shape (above) but with the per-candidate processing still
  on the host-specific code path.
- `Protocol::Sync` associated-type (#48 option A) — full type-system
  enforcement against future protocol-sync drift. Scoped out: the blast
  radius is every `impl Protocol` site + the embedded feature matrix,
  for benefit that's speculative for non-FT8 protocols (none have shown
  drift in 18 months post-fork). Tracked in #48.

## 0.5.12 — FT8 SIC correctness, FT4 WSJT-X parity, JT9 7/7, AP iaptype-1

Drop-in upgrade for everyone on 0.5.11 — non-breaking, no API
removals, no behaviour changes for callers that don't opt into the
new AP entrypoint. Bundles three correctness fixes that ship in
0.5.11 as known bugs, plus the AP-list close-out for issue
[#31](https://github.com/jl1nie/mfsk-core/issues/31).

### Fixed (high-severity)

- **FT8 phase-2 SIC subtracts caller-supplied `known` signals**
  ([#26](https://github.com/jl1nie/mfsk-core/pull/26)). The
  `decode_frame_subtract_with_known` /
  `decode_frame_subtract_with_known_and_ap` paths added the caller's
  Phase-1 results to the dedup list but never subtracted them from
  the residual — so every Pass-1 / Pass-2 SIC iteration ran on the
  original audio with the strong known signals still in place,
  defeating the entire purpose of the pipelined Phase-1 → Phase-2
  architecture. Weak signals masked by known strong ones stayed
  masked. Introduced in `48b1f37` and shipped in 0.5.10 / 0.5.11.
  Fix: after Pass 0 (which still uses the precomputed FFT against
  the original audio so Phase-1-missed signals can surface), all
  caller-supplied `known` results are subtracted from the residual
  before Pass 1 begins, with a `residual_dirty` flag that keeps
  the FFT-cache reuse correct.

### Fixed

- **FT4 GFSK `BT = 1.0` and subtract `NFILT = 1400`**
  ([#27](https://github.com/jl1nie/mfsk-core/pull/27)) — WSJT-X
  parity. `FT4_GFSK.bt`, `FT4_SUBTRACT.gfsk.bt`, and `Ft4::GFSK_BT`
  were uniformly `2.0` (the FT8 value); WSJT-X
  `lib/ft4/gen_ft4wave.f90` calls `gfsk_pulse(1.0, tt)` and
  `lib/ft4/subtractft4.f90` declares `bt=1.0`. Same file declares
  `nfilt=1400` (≈58 ms half-window at 12 kHz); we had `lpf_half=2000`
  (= FT8's NFILT=4000 misapplied to FT4). Self-cancellation residual
  remains tight because the encoder/subtractor sides stay aligned —
  just at the correct value now. Test power-ratio accumulators
  promoted to `f64` to remove a `f32` precision drift that made the
  ratio non-deterministic at large slot lengths.
- **FT4 / FST4 `_with_options` accept `freq_hint: Option<f32>`**
  ([#28](https://github.com/jl1nie/mfsk-core/pull/28)). PR #21's
  `_with_options` variants silently dropped the `freq_hint`
  parameter, breaking the "`_with_options` is a strict superset of
  its non-options sibling + depth + strictness" invariant the rest
  of the API follows. Restored on the three drift sites
  (`ft4::decode_frame_with_cache_and_options`,
  `ft4::decode_frame_subtract_with_options`,
  `fst4::decode_frame_with_cache_and_options`) and forwarded into
  the pipeline.

### Added

- **FT8 AP iaptype-1 (blind CQ) decode pass** + `ft8_qso3_apon_recall`
  test — closes [#31](https://github.com/jl1nie/mfsk-core/issues/31)
  ([#39](https://github.com/jl1nie/mfsk-core/pull/39)). The AP loop
  in `ft8::decode::process_candidate` now always tries an
  `ApHint::new().with_call1("CQ")` configuration (locks bits 0–28
  + i3=001, the WSJT-X `lib/ft8/ft8b.f90` ipass=5 / iaptype=1
  pattern) whenever the caller passes `Some(&ApHint)`, in addition
  to the existing operator-context passes (iaptypes 2..6 / passes
  6/7/8/9/10/11). `decode_frame` (legacy non-AP) keeps calling
  through with `ap_hint = None` and stays bit-for-bit identical.
  The new integration test under `tests/ft8_qso3_apon_recall.rs`
  drives `decode_frame_with_ap` twice on the vendored
  `qso3_busy.wav` (once `None`, once with `mycall=K1JT,
  hiscall=HA0DU`) and gates on the strict-superset invariant from
  the issue's acceptance criteria, plus six JTDX-captured AP-on
  extras tracked behind a `JTDX_EXTRAS_HARD_FLOOR` seam that grows
  as the host-vs-embedded coarse-sync parity gap (filed as
  [#40](https://github.com/jl1nie/mfsk-core/issues/40)) closes.

### Improved

- **JT9 recall locked at 7/7** (was 5/5 in 0.5.10 / 0.5.11)
  ([#38](https://github.com/jl1nie/mfsk-core/pull/38)). JTDX
  cross-check on the `samples/JT9/130418_1742.wav` golden surfaced
  two real signals beyond the original 5-entry table —
  `G7CNF N4HFA EL89` @ 1461 Hz and `JA1KAU PD0JAC -23` @ 1505 Hz —
  both already produced by `decode_scan` once `freq_max_hz` was
  bumped from 1500 to 1550 Hz. The 0.5.11 release-window CHANGELOG
  said "5/5 restored" and the test was `#[ignore]`'d behind a
  stale "JT9 host path 2/5" message; both are corrected. The test
  now runs on CI (vendored asset, `#[ignore]` removed) so any
  recall regression below 7/7 is caught at PR time.
- **CI release pipeline** ([#30](https://github.com/jl1nie/mfsk-core/pull/30))
  — gates the crates.io publish on the full CI matrix and builds
  the FFI artifacts before publishing so embedded-build breakages
  can't sneak past again.
- **Test-fixture path hygiene** ([#29](https://github.com/jl1nie/mfsk-core/pull/29),
  [#36](https://github.com/jl1nie/mfsk-core/pull/36),
  [#37](https://github.com/jl1nie/mfsk-core/pull/37)). Replaced
  every hardcoded `/home/ubuntu/...` test fixture path with
  `concat!(env!("CARGO_MANIFEST_DIR"), …)` resolution. Vendored the
  WSJT-X `samples/FT8/210703_133430.wav`,
  `samples/JT9/130418_1742.wav`, and the `jl1nie/RustFT8`
  on-air recordings (`191111_110130.wav`, `191111_110200.wav`)
  under `embedded-poc/assets/` so CI runners and contributor
  checkouts actually exercise the recall tests instead of silently
  `return`ing from `if !path.exists()` guards. CLAUDE.md documents
  the convention going forward.

## 0.5.11 — embedded-build hotfix on top of 0.5.10

`mfsk-core` 0.5.10 published cleanly to crates.io (the `--features full`
build used by the publish step has `std`), but the release-artifact
jobs for `mfsk-ffi-ft8` on Xtensa ESP32 / ESP32-S3 failed: the new
`build_jt9_mettab` / `jt9_branch_metrics` helpers in `fec/conv/mod.rs`
called `f32::round`, which lives in `std`, not `core`. Under the
embedded `no_std + alloc` configuration that compiled against the
stale `(crates.io: 0.5.10)` mfsk-core lib, that resolves to an
unresolved-method error.

Fix: import `num_traits::Float` under `#[cfg(not(feature = "std"))]`
in `fec/conv/mod.rs`, matching the pattern already used in
`fec/conv/fano.rs`. Also gates `wspr::decode` / `wspr::demod` and
`ft8::subtract` behind the FFT-backend features so the
single-protocol matrix entries (`--features wspr`, `--features ft8`,
etc.) actually build — these were three pre-existing breakages
that ci.yml had been catching since 0.5.5 but were silently
tolerated under the `--features full` umbrella that local testing
covered. 13/13 matrix entries now green.

No semantic changes vs 0.5.10. The crates.io 0.5.10 publish stays
as-is (libcore consumers were unaffected); 0.5.11 supersedes it
for everyone, and the embedded `mfsk-ffi-ft8` 0.5.10 binaries that
0.5.10's release CD never produced are first published at 0.5.11.

## 0.5.10 — JT9 WSJT-X recall 5/5, FT4 SIC, depth/strictness + AP IF wiring

### JT9 — issue [#19](https://github.com/jl1nie/mfsk-core/issues/19) closed (1/5 → 5/5)

Six load-bearing source-faithful fixes lift `decode_scan` recall on
`samples/JT9/130418_1742.wav` from 1/5 to a full 5/5 — every
WSJT-X golden frame (`CQ GM7GAX IO75`, `TF3G N7MQ CN84`,
`K1JT KF4RWA 73`, `CQ M0WAY IO82`, `K1JT N5KDV EM41`) now decodes
through the public pipeline:

1. **`packgrid` formula** (`msg/jt72.rs::pack_grid4_plain`) —
   replaced a self-consistent but non-WSJT-X-compatible long-offset
   formula with the integer-arithmetic equivalent of `lib/packjt.f90`
   `packgrid` + `grid2deg`: `ng = (179 − 10·fl − sl)·180 + (10·fla + sla)`.
   Pinned canonical `ng` values for six representative grids
   (`grid_wsjtx_canonical_ng`).
2. **`afc9` 3-parameter chi-square AFC** — replaced the 1-D
   `afc_simple` frequency scan with the WSJT-X parabolic line
   search over (frequency, drift, integer-sample time shift) plus
   the `shft` cshift+zero-fill helper, mirroring `lib/afc9.f90` +
   `lib/fchisq.f90` exactly.
3. **`chkss2` schk metric + two-stage gate** — added the
   `lib/chkss2.f90` sync-quality measure and applied the WSJT-X
   `sync ≥ 1.0 && schk ≥ 1.5` gate from `jt9_decode.f90:139` before
   spending Fano cycles.
4. **`jt9fano` xx0 mettab** — ported the calibrated 256-entry
   asymmetric LUT (≈+25 max reward, −525 min penalty, slope-2
   linear extension beyond `ib=160`) to replace the linear
   `m = 0.5·l − bias` form for `ConvFano232`. Without this LUT,
   Fano was latching onto plausible-looking neighbour codewords at
   marginal SNR (`CN84` → `EH03`, `IO82` → garbage). Fano delta
   moves to 170 = `nint(3.4·50)` to match the new mettab scale.
5. **Pre-decode NMS removed** — the 5 Hz freq-domain non-maximum
   suppression had no WSJT-X equivalent and was dropping real-but-
   weak goldens that lost coarse-score competitions to phantoms in
   the same NMS bin. WSJT-X uses post-decode `done(iaa:ibb)=.true.`
   suppression which we already mirror via the `seen` dedup loop.
6. **`coarse_search` per-freq collapse** — was emitting one candidate
   per `(freq, time)` cell, letting strong signals (EM41) crowd the
   top-N with their many time variants. Now keeps only the best-
   scoring time alignment per freq bin, mirroring WSJT-X
   `lib/sync9.f90` `ccfred(i) = max over lags of sum`. Total
   candidates collapse 3393 → 261 on the golden, weak signals
   (1119 Hz `IO75` at score 0.96) survive truncation.

`FecOpts::max_cycles_per_bit` added so callers can override the
Fano per-bit cycle budget — wires the WSJT-X depth-retry knob
without forcing a global config change.

### FT4 / FST4 / FT8 — public IF expansion (PRs from @chemicalsno)

Three contributor PRs landed, each shipping a focused IF extension
with WSJT-X provenance and unit + integration test coverage:

- **PR [#20](https://github.com/jl1nie/mfsk-core/pull/20):**
  `ft4::subtract` module — successive-interference cancellation
  primitives mirroring `lib/ft4_subtract.f90`. Public surface:
  `subtract_signal`, `subtract_signal_weighted`, `subtract_signal_lpf`,
  `refine_signal_freq`. Uses the existing `FT4_SUBTRACT` config
  (BT=2.0, hmod=1.0, nsps=576) so there's a single source of truth
  for the GFSK-shaping constants.
- **PR [#21](https://github.com/jl1nie/mfsk-core/pull/21):**
  `ft4::decode::decode_frame_with_options` and
  `fst4::decode::decode_frame_with_options` — surfaces the
  `DecodeDepth` × `DecodeStrictness` axes that the internal generic
  pipeline already accepted. The legacy `decode_frame` is unchanged
  (shim using `BpAllOsd` + `Normal`), giving a clean mapping to the
  WSJT-X Fast / Normal / Deep menu.
- **PR [#22](https://github.com/jl1nie/mfsk-core/pull/22):**
  `ft8::decode::decode_frame_with_ap` — wide-band counterpart to the
  prior `decode_sniper_ap` (which scanned only ±250 Hz). Threading
  AP hints through `process_candidate` was already supported
  internally; this PR exposes it on the public scan.

Three `tests/`-level integration tests
(`tests/ft4_decode_with_options.rs`, `tests/ft4_subtract_pipeline.rs`,
`tests/ft8_decode_frame_with_ap.rs`) validate the production-shape
contract for each new API beyond the inline unit tests the PRs
already shipped.

### Other

- `xsnr2_db_simple` SNR calibration narrative cleared from "embedded
  known limitations" — `DecodeResult.snr_db` now lands within ±3 dB
  of JTDX absolute on real silicon (was 4–12 dB low; resolved in
  0.5.7 + 0.5.8). EMBEDDED.md / .ja.md refreshed accordingly.
- Stale `/home/minoru/` paths in test + diag files replaced with
  `/home/ubuntu/`, restoring
  `ft8_qso3_apoff_recall::qso3_apoff_meets_wsjtx_golden_floor` on
  `cargo test --features full`.
- Outstanding scope (`FST4-15` / `FST4W` golden, JT65 erasure-aware
  golden harness, MSK144 implementation) tracked in
  [#23](https://github.com/jl1nie/mfsk-core/issues/23) / [#24](https://github.com/jl1nie/mfsk-core/issues/24) / [#25](https://github.com/jl1nie/mfsk-core/issues/25).

## 0.5.9 — WSJT-X golden recall: WSPR 8/8, FT4 6/6, JT9 1/5 (encoder bug #19)

End-to-end recall harness for the WSJT-X-distributed reference
recordings (`samples/{WSPR,FT4,JT9}/*.wav`). Two protocols moved up,
one is honestly documented as broken pending an encoder bugfix.

**WSPR — `samples/WSPR/150426_0918.wav`:** 3 / 8 → **8 / 8** in
~0.88 s. Five orthogonal upgrades stack to clear the bottom of the
list:

- Sub-bin demod: `wspr/decode.rs` runs mode-0 lag refine (5×) + mode-1
  freq refine (5×) per candidate before Fano. No Fano evaluation per
  cell — sync score gates cell selection. Lifts 3/8 → 4/8.
- Negative-dt support via front-side audio padding. WSJT-X's
  `wsprd::readwavfile` lets dt drift up to ~−2 s; we now do the same
  before the big 1.47 M-pt FFT (4/8 → 5/8).
- Fano metric bias correction toward `wsprd` parity (constant
  `1.0` instead of `0.0`) so weak-signal threshold stepping doesn't
  underflow before convergence.
- 2-pass subtract+re-coarse: after a successful Fano decode the
  resolved frame is subtracted from the spectrogram so a co-channel
  weak signal can be re-coarsed.
- OSD-2 fallback for the BP/Fano hard-error tail and a Type-3 phantom
  filter so we don't ship `<0...0>` decodes that pass the FEC but
  carry no callsign.

`wspr::decode_scan_default` no longer publishes a fixed candidate
budget — see the [Status](#status) note in `README.md` for the
recall vs. cost trade-off knobs (`SearchParams::max_candidates`,
`subtract` flag).

**FT4 — `samples/FT4/000000_000002.wav`:** 0 / 6 → **6 / 6**.
Multi-slice port of the WSJT-X FT4 demod path:

- Nuttall window + `nsym = 4` LLR aggregation (per-protocol — FT8
  stays on its existing window).
- `sync4d` 2-pass (Δf, Δt) refinement around each candidate.
- LLR tail-patch for chunks not divisible by `nsym` (last few bits
  were silently dropped before).
- WSJT-X `rvec` scrambler + RTTY-format unpack ported verbatim from
  `lib/77bit/`.
- Per-bin polyfit baseline reverted in coarse_sync (was masking real
  signals) and polyfit shape clamp tightened to `1.0..2.0` to keep
  the LLR magnitudes sane near the noise floor.

**JT9 — `samples/JT9/130418_1742.wav`:** 0 / 5 → **1 / 5**, plus a
faithful WSJT-X port of the demod pipeline that's *ready* for a
5/5 result the moment the encoder side is fixed.

- New `src/jt9/softsym.rs` ports `lib/softsym.f90` end-to-end:
  `downsam9` (NFFT1 = 653 184 → NFFT2 = 1512 brick-wall band-select
  → IFFT to 27.78 Hz baseband), `peakdt9` (sliding-window sync
  score = sync_avg/data_avg − 1), simple AFC (sub-tone offset
  search), `twkfreq`, and `symspec2` (16-sample coherent-sum LLRs).
  Replaces the box-car `baseband.rs` / `demod_bb.rs` path that
  was carrying ~3–5 dB of unnecessary out-of-band noise into LLRs.
- Sync scores now sit at WSJT-X parity for all 5 golden signals
  (1346 Hz scores 589 vs ~30 reference scale).
- Currently only 1 / 5 (1224 Hz `K1JT KF4RWA 73`) recovers
  end-to-end. The other four either miss or land on a *plausible
  but wrong* message — same callsigns, wrong grid. Both Gray-code
  directions tested at encode; WSJT-X rejects both. Self-roundtrip
  passes for all 5 messages → bug is in the encoder path
  (almost certainly `pack_grid` in `msg/jt72.rs`) and is symmetric
  enough that our test suite roundtrips through it. Tracked in
  [#19](https://github.com/jl1nie/mfsk-core/issues/19).
- `softsym.rs` ships in 0.5.9 because it's the right pipeline; the
  recall jump waits for #19.

**Test harness** (`tests/{wspr,ft4,jt9}_wsjtx_samples.rs`,
`tests/q65_wsjtx_samples.rs` extended): one assertion per WAV that
locks the reference golden list with `±2 Hz` / `±0.2 s` tolerance,
runs only when the WSJT-X tree is present at the expected sibling
path so the harness stays portable across packaging environments.

API additions in 0.5.9 (all additive, no breaking changes):
- `mfsk_core::jt9::softsym::{AudioFft, downsam9, peakdt9, symspec2,
  llrs_from_c5}` — public for callers that want the reference
  pipeline directly.
- WSPR `SearchParams::subtract` and `max_candidates` knobs (already
  present, now fully wired through `decode_scan`).

## 0.5.8 — `xsnr2_db_simple` calibration fix (median noise floor + emp. constant)

0.5.7 introduced `xsnr2_db_simple` but kept WSJT-X's `/3e6` + `-27 dB`
calibration constants verbatim — those numbers are tied to WSJT-X's
f32 spectrogram amplitude scale and don't survive the embedded u16
spec's auto-gain. The result was every reading clamped to the
`-24 dB` floor on real silicon. Two changes land it on JTDX absolute
SNR within ±3 dB across weak / mid / strong signals:

1. Per-frequency baseline switches from **mean** to **median (P50)**
   over a ±50 freq bin × time-decimated window. The plain mean is
   dragged upward by the very signal we're measuring (W1FC at 0 dB
   gave xbase ≈ 4.8 M before, but only 0.14 M after — a 35× drop in
   bias). Sort cost ~100 µs at LX7 240 MHz, well inside the
   post-SlotEnd budget.

2. Calibration constant becomes a **single empirical `46 dB`** offset
   instead of the WSJT-X `/3e6 - 1` / `-27 dB` combo. Pair-matched
   on the qso3_busy reference (real M5StickS3 silicon, 2026-05-05):

   ```text
   signal      ours     JTDX    err
   W1FC        +2.8       0     +2.8
   A92EE       -8.2      -9     +0.8
   WM3PEN      +4.1       0     +4.1
   N1JFU      -15.4     -14     -1.4
   K1JT HA0DU -14.5     -13     -1.5
   F5RXL       -3.9      -3     -0.9
   ```

m5stack-s3-app's decode_pipeline now passes the xsnr2 number into
the UI and drops the `+3 dB` offset hack (snr_norm's
`DEFAULT_CALIBRATION_OFFSET_DB` returns to 0.0).

Embedded-only follow-up (audio): `audio.rs` adds a 5 ms envelope
ramp around the AUDIO_GATE flips so transitioning into / out of
the per-decode mute window doesn't click the speaker.

API surface unchanged from 0.5.7 — pure constant + algorithm tune
of `xsnr2_db_simple`. No breaking changes; safe patch bump.

## 0.5.7 — `xsnr2_db_simple`: WSJT-X-comparable SNR for any Spectrogram

Adds `mfsk_core::ft8::decode_block::xsnr2_db_simple(spec, result,
cell_scale)` — a `std`-free port of the WSJT-X `ft8b.f90:449-454`
xsnr2 SNR formula that runs on *any* `Spectrogram`, host f32 or
embedded u16. Closes the long-standing per-block-auto-gain bias on
the embedded `compute_snr_db` path: same callsign now decodes with
±1-2 dB of WSJT-X / JTDX absolute SNR for both weak and strong
signals, where the old per-Costas-block adjacent-tone ratio drifted
~0–15 dB depending on which block's gain factor it landed on.

Internally it computes a localised per-frequency baseline (= mean
over time, ±50 bins around the decode's carrier — `~150 Hz`) so the
embedded path doesn't depend on `baseline::fit_baseline`'s
polynomial smoother, which is `std`-only and therefore not on the
embedded build's feature set. Cell-scale param undoes the
`FP_SPEC_SHIFT` shift in the embedded u16 spec so xsig and xbase
land in the same WSJT-X calibration regime.

Embedded apps (`m5stack-s3-app`'s `decode_pipeline`) drop their
previous `+3 dB calibration offset` hack and call the new function
directly. Host `decode_block_multipass` (with `fft-rustfft`) keeps
its `recompute_snr_xsnr2` polynomial-baseline path, which stays
preferable when std is available.

API additions (host + embedded):
- `pub fn ft8::decode_block::xsnr2_db_simple`

No breaking changes; safe minor bump on the 0.5.x line.

## 0.5.6 — post-0.5.5 quick-fix: no_std math imports + allsum 7-tone alignment

Two follow-up fixes to issues uncovered after the 0.5.5 publish:

1. **no_std `f32::cos / sin / round` errors** in the Xtensa cross-build
   (`mfsk-ffi-ft8 — Xtensa ESP32 / S3` CD jobs). `refine_fine.rs` and
   `core/dsp/fft_mixed_3840.rs` were using inherent `f32` math methods
   (std-only); under `no_std` (Xtensa, embedded targets) those methods
   don't exist on bare `f32` and require the `num_traits::Float` trait
   via `libm`. Added the standard `#[cfg(not(feature = "std"))] use
   num_traits::Float;` import these two modules were missing. Host
   build unaffected.

2. **5 internal-consistency unit-test failures** inherited from a
   v0.5.4-to-v0.5.5 intermediate commit (`48b1f37`, "WSJT-X-faithful
   decode pipeline"). `coarse_sync_inner`'s inline `owned_allsum`
   summed 8 tones (`for k in 0..NTONES`) while the public helper
   `fill_coarse_allsum` summed 7 (matching WSJT-X `sync8.f90:66`,
   where tone 7 is data-only and never a Costas position). The score
   formula's divisor `(NTONES - 2) = 6` is calibrated to the 7-tone
   sum — the 8-tone version was the bug. Aligned `owned_allsum` to
   7 tones; updated the column-by-column reference helper in the
   test suite that mirrored the 8-tone version. All 314 tests pass.

Recall regression numbers (host f32 / qso3_busy) unchanged at floor —
WSJT-X golden 7/8, JTDX 16/18 — verified post-fix.

## 0.5.5 — embedded recall fix (Hann→Rect window) + runtime BP iter + nstep-half feature

A targeted patch addressing two latent embedded-side bugs uncovered while
benchmarking M5StickS3 (ESP32-S3 LX7) against the JTDX 18-entry decode
of the qso3 busy-band reference WAV.

Headline numbers — qso3_busy on LX7 ship config (pass1=30, max_cand=15):

| Path | Pre-0.5.5 | 0.5.5 |
|---|---|---|
| Decodes (vs JTDX 18-entry golden) | 3 | **6** |
| post-SlotEnd | 0.71 s (0.5.4) → 1.22 s (latent regression) | **1.28 s** |
| Time / decode | ~240 ms | **213 ms** |

### Library (mfsk-core)

API additions — runtime tunable BP iterations (the dominant time-scaling
knob inside stage 3, particularly on weak signals where every LLR variant
exhausts `BP_MAX_ITER` × NMS):

- `ft8::params::DEFAULT_BP_MAX_ITER` (= 30, WSJT-X reference) —
  named alias for the existing `BP_MAX_ITER`.
- `ft8::decode_block::decode_block_tuned(.., bp_max_iter)` — host-facing.
- `ft8::decode_block::decode_block_into_tuned(.., bp_max_iter, basis_re, basis_im)`
  — fixed-point + caller-basis variant.
- `ft8::decode_block::process_candidates_tuned`,
  `process_candidates_into_tuned`,
  `process_candidates_into_with_cs_scratch_tuned` — for callers that
  bypass `decode_block`'s spectrogram + coarse_sync (e.g. embedded
  dual-core dispatch).

The non-`_tuned` entry points keep their stable signatures and forward
to the `_tuned` variants with `DEFAULT_BP_MAX_ITER`.

New cargo feature:

- `nstep-half` — flips `ft8::params::NSTEP` from `NSPS/4` (= 480, the
  WSJT-X faithful default) to `NSPS/2` (= 960). Embedded targets enable
  this so the spec time-axis (= `NMAX/NSTEP - 3`) stays at 184 cells
  instead of doubling to 372 — at NSPS/4 the stage-1 FFT count and
  spectrogram memory both 2× and overrun the LX7 / LX6 post-SlotEnd
  budget. Host stays on the WSJT-X-faithful value by default; existing
  host tests are unaffected.

Doc + comment cleanup:

- `coarse_sync_inner`'s "tone_step ≈ 2.13 multi-bin Plan A" comment is
  rewritten to reflect the NFFT=3840 single-bin gather actually in
  effect, and to flag that Hann-window + multi-bin sum were always
  paired (do not re-add one without the other).

### Embedded (`embedded-poc/embedded-shared`)

The two bugs:

1. **`stage1_inc` was applying a Hann window** while the host
   `compute_spectrogram` had migrated to **rectangular** at the
   NFFT=3840 cutover (commit dec4016 era). Hann's coherent gain 0.5
   plus its 2-bin mainlobe spread were specifically what the
   integer-bin migration moved away from; keeping Hann nullified the
   integer-bin SNR advantage. Removing it (and the `+1` shift that
   compensated Hann's gain) is the recall fix above.

2. **A latent stage-3 OOB**: `coarse_sync_inner`'s `m_base` is derived
   from `params::NSTEP`. Task #24 moved the host to NSPS/4 (m_u up to
   ~350) but `stage1_inc` kept building the spec at NSPS/2 (n_time =
   184). With the ship config's `q_thresh=12` from 0.5.4, few candidates
   reached the deep-m blocks and the panic was masked; with
   `DEFAULT_Q_THRESH` lowered (later WSJT-X-faithful change) the
   embedded build started panicking at `power_acc`. Fixed structurally
   by the `nstep-half` feature.

API additions:

- `embedded_shared::apps::rx_wavsim::run(.., bp_max_iter)` — accepts the
  runtime BP iter cap. The old 4-arg form is gone (breaking change for
  callers of this internal app crate; both bundled targets are updated).
- `embedded_shared::apps::rx_wavsim::RxSweepCfg` + `run_sweep(wavs, cfgs)`
  — rotate through configs slot-by-slot for in-binary parameter sweeps.
  Used to find the (pass1, max_cand, q_thresh, bp_max_iter) Pareto in
  one flash instead of N flashes.
- `dual_core::stage3_split(.., bp_max_iter, ..)` — threads the BP cap
  through the work-stealing dispatch.

`stage1_inc::WorkerCtx::hann` field + table generation deleted.

### Per-target binaries

- `embedded-poc/m5stack-s3/src/bin/rx_wavsim.rs` and
  `embedded-poc/m5stack-core2/src/bin/rx_wavsim.rs` pass
  `mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER` for the new arg.
- `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs` likewise.

### Sweep insights captured for future tuning (qso3_busy / LX7)

| pass1 / max_cand / q / BP | post-SlotEnd | decodes |
|---|---|---|
| 30 / 15 / 6 / 30 (ship) | 1.30 s | 6 |
| 45 / 20 / 6 / 30 | 2.14 s | 6 |
| 60 / 30 / 6 / 30 | 2.68 s | 6 |
| 45 / 20 / 6 / **15** | 1.80 s | 6 |
| 45 / 20 / **12** / 30 | 1.43 s | 6 |

After the rect-window fix, all configs converge to 6 / 18 JTDX (5 hits +
1 WSJT-X-only). The 7th decode (N1PJT HB9CQK -10 dB, fractional bin
alignment) requires `fine_refine_pass1`'s 192k-FFT cd0 path — infeasible
on Xtensa. **Pareto-optimal ship config = `(30, 15, q=6, BP=30)`**.

## 0.5.4 — embedded streaming pipeline + S3 LX7 sub-1 s + bench against WSJT-X reference

API-additive patch on the host side; all changes are concentrated in
`embedded-poc/embedded-shared` (the M5Stack Core2 / M5StickS3 sample
crates) and the FT8 `decode_block` parameter list. The headlines:

- **M5StickS3 (ESP32-S3 LX7) gets a full-recall sub-1 s decode**:
  qso3 busy band (the WSJT-X formally-distributed reference WAV
  `samples/FT8/210703_133430.wav`) decodes 7/13 callsigns in
  **0.707 s post-SlotEnd**; same recall, 1.434 s on Core2 LX6.
  No relaxed-recall feature, no MAX_CAND reduction.
- **The streaming RX pipeline was redesigned around FreeRTOS Queues**.
  Old shared-state-with-resets coordination (`STATE: UnsafeCell`,
  `AUDIO_FILL` / `PAIR_DONE` atomics, `mark_slot_boundary`,
  `take_spec_and_allsum`, `peek_latest`) is retired in favour of
  single-ownership `Box<ChunkMsg>` / `Box<SpecBundle>` / `Box<Slot>`
  pointer-passing through depth-1/2 queues.
- **A reference-vs-host benchmark test** comparing host wide-band
  `decode_frame` (BpAllOsd, max_cand=200) against the embedded
  `decode_block` on the WSJT-X recording is added at
  `tests/ft8_reference_suite_recall.rs`. It shows the embedded
  budget caps at 7/13 on busy band — the missing 6 require iterative
  subtraction, which `decode_block` doesn't implement.

### Library (mfsk-core)

API additions:

- `pub const DEFAULT_Q_THRESH: u32 = 12` (in `ft8::decode_block`) —
  the recommended `process_candidates*` `q_thresh` knob.

API change (only affects `#[doc(hidden)]` callers — `decode_block` /
`decode_block_into` keep their signatures by passing the default
internally):

- `process_candidates(audio, cands, depth)`
  → `process_candidates(audio, cands, depth, q_thresh)`.
- `process_candidates_into(audio, cands, depth, basis_re, basis_im)`
  → `process_candidates_into(audio, cands, depth, q_thresh,
  basis_re, basis_im)`.
- `process_candidates_into_with_cs_scratch(...)` similarly gains
  `q_thresh: u32`.

The `MFSK_Q_THRESH` env-var override on `q_thresh()` and the
matching `Q_THRESH_DEFAULT` const are removed.

Feature surface trimmed (23 → 16; all removals are
embedded-targeted — host library consumers using the default feature
set are unaffected):

- Retired: `relaxed-recall` (now a runtime parameter — see
  `q_thresh` above), `fixed-point-llr` and `llr-i8` (folded into
  `fixed-point`; the integer pipeline used `Q3i8` LLR at this point,
  which Phase 1 of issue #15 confirmed was host-recall-equivalent to
  `Q11i16` with half the BP scratch — **superseded in 0.6.2 / 0.6.3**:
  the wider real-silicon LX7 sweep showed Q3i8's quantization step
  was actually the recall ceiling on Xtensa (host f32 16/18 vs
  Q3i8 9/18 on `qso3_busy.wav`), and the active scalar moved back
  to `Q11i16`; see 0.6.2 entry below and
  `mfsk-core/src/ft8/decode_block/process_candidates.rs` LlrT docs),
  `fixed-point-coarse-i32`
  (only useful on FPU-less targets we don't currently ship for —
  hurts on LX6/LX7), `fixed-point-bp` (alias for `fixed-point-llr`),
  `fixed-point-cs` (placeholder, never wired up), `osd-deep` and
  `eq-fallback` (internal toggles, never enabled by any caller),
  `embedded-tx` / `embedded-rx` (aggregate presets, no caller used
  them), `esp32s3` (alias of `embedded-rx`, only referenced by the
  retired `embedded-poc/esp32s3/` PoC crate).
- Kept: `std`, `alloc`, `ft8`/`ft4`/`fst4`/`wspr`/`jt9`/`jt65`/`q65`/
  `packet-bytes`/`uvpacket`, `parallel`, `fft-rustfft`, `fft-extern`,
  `fixed-point` (now the single embedded knob), `profile-coarse`,
  `full`.

The `Q11i16` scalar type itself stays in `core::scalar` for manual /
test use — only the built-in feature wiring is gone. (Note: the
Q11i16 wiring was restored in 0.6.2 once the real-silicon sweep
contradicted the host-only Phase 1 reading. `Q3i8` is the type that
now lives "only in `core::scalar` for manual / test use" — the two
swapped roles. See the rewritten note above.)

### Embedded (`embedded-poc/`)

- New `embedded-shared` library architecture: `pipeline` (queue
  helpers + `ChunkMsg` / `SpecBundle` / `Slot` types), `wav_sim`
  (WAV-fed producer task), `stage1_inc` (incremental FFT consumer +
  per-slot finaliser), `dual_core` (job/result queues + work-stealing
  stage 3 dispatch). All ownership transfers via `Box::into_raw`
  raw-pointer items on FreeRTOS Queues — `mpsc::sync_channel`-equivalent
  semantics, no shared mutable state.
- New `apps` module: `rx_wavsim::run(wavs, pass1_limit, max_cand,
  q_thresh)` and `compute_bench::run(target_name, qso_wavs)`. The
  per-target `m5stack-{core2,s3}/src/bin/rx_wavsim.rs` and `main.rs`
  now collapse to ~13-line shims that just supply WAV slices and
  config.
- `dual_core::stage3_split` is now **work-stealing**: both PRO_CPU
  and APP_CPU pull individual candidates from a shared
  `Vec<Option<RefinedCandidate>>` via `AtomicUsize::fetch_add(1)`,
  absorbing the per-cand BP wall-clock variance (failed cands run
  all 4 LLR variants; on qso3 ~7 of 15 fail). Saved **-239 ms on
  qso3 S3** at zero recall change.
- `stage1_inc` emits its `SpecBundle` on `spec_q` as soon as pair 92
  finalises (≈ 200 ms before SlotEnd). Main runs stage 2 in parallel
  with the tail of audio capture; only pass 2 + stage 3 are in the
  post-SlotEnd budget.
- Asset / header consolidation: `embedded-poc/{assets,bindings.h}`
  shared between m5stack-core2 / m5stack-s3. The older
  `embedded-poc/esp32s3/` PoC (synth + FFT round-trip) is retired
  as superseded by m5stack-s3.

### Performance benchmark (post-SlotEnd, q_thresh=12, full recall)

Both chips on the same WAV trio at the production setting
(PASS1=30, max_cand=15, BpAll, no OSD):

| WAV  | results | Core2 LX6 | S3 LX7 |
|------|---------|----------:|-------:|
| qso1 (mid-band, 3 stations)        | 3/3 ✓ | 1.303 s | 0.574 s |
| qso2 (mid-band, 5 stations)        | 5/5 ✓ | 0.632 s | 0.370 s |
| qso3 busy band (WSJT-X reference)  | 7/7 ✓ | 1.434 s | 0.707 s |

vs the 0.5.3 numbers (1.83 / 1.45 / 1.98 s slot-total on Core2): a
29 / 56 / 28 % reduction respectively, mostly stage 3 work-stealing
absorbing the per-cand variance.

The host wide-band reference on the same qso3 WAV finds 13/13 in
~140 ms (Ryzen, BpAllOsd, max_cand=200). The 6 callsigns the
embedded path misses on the busy band are below the coarse_sync
top-100 entirely — they need iterative subtraction (the WSJT-X
wide-band path's hallmark, not in `decode_block`) to surface. A/B
test on real S3 silicon (`logs/s3_pass100_max30_2026-05-04.log`)
confirmed widening PASS1 → 100 / max_cand → 30 / OSD on the embedded
path moves the qso3 needle by **zero** — see `docs/EMBEDDED.md` for
the rationale on why ship stays at PASS1=30 / max_cand=15.

## 0.5.3 — embedded perf: dual-core decode + Phase E (Core2 LX6 sub-2 s)

API-additive patch. The headline is on the embedded side: with full
phase-A-through-E pipeline parallelism, the M5Stack Core2 (ESP32-LX6,
240 MHz dual-core) decodes a busy-band 15 s FT8 slot in **1.98 s
wall-clock** with all 7 ground-truth callsigns recovered and zero
phantoms — the previous baseline (single-core, PASS1=100, max_cand=30,
no overlap) was 8.85 s for the same recording.

The library-side surface change is small (one constructor + one type
alias on a `#[doc(hidden)]` struct) and exists to make the embedded
PoC plumbing possible from outside the crate. No host-path behaviour
changes.

### Library (mfsk-core)

- `Spectrogram::from_parts(n_freq, n_time, data)` — public constructor
  for the FT8 spectrogram, layout-checked against the column-major
  `data[time * n_freq + freq]` invariant. Lets embedded callers
  hand-build a spectrogram (e.g. computed incrementally during slot
  capture) and feed it straight into `coarse_sync` etc.
- `pub type SpecCell` — re-export of the cell type (`u16` under
  `fixed-point`, `f32` otherwise). Used together with `from_parts` so
  the bin doesn't have to depend on the feature-gated alias.

`Spectrogram` itself is still `#[doc(hidden)]` — the public surface
here is "construct + read", not "depend on layout long-term".

### Embedded (`embedded-poc/m5stack-core2`)

End-to-end performance journey for one busy-band FT8 slot (qso3,
WSJT-X 210703 sample, 13 ground-truth signals -8 to -18 dB):

  | Stage              | qso3 wall-clock |
  | ------------------ | --------------- |
  | 0.5.2 baseline     | 8.85 s          |
  | + Phase 0/1        | 5.24 s          |
  | + Phase A/B/C      | 3.30 s          |
  | + Phase D          | 3.22 s          |
  | + Phase E (final)  | **1.98 s**      |

Recall preserved at every step (3+5+7 truth across qso1/2/3 = 15/15).

- **Phase A** — `esp_dsp_fft::prewarm()`. Force the f32+i16 twiddle
  tables to init at boot so concurrent first-call init from main and
  worker tasks can't race in `dsps_fft2r_init_*`.
- **Phase B** — `dual_core.rs` adds a persistent FreeRTOS worker
  pinned to APP_CPU and a second 60 KB Q15 BASIS scratch (`.bss`,
  internal DRAM). 120 KB total scratch out of ~300 KB internal DRAM.
- **Phase C** — Pass 2 (`refine_candidates_into`) and Stage 3
  (`process_candidates_into`) candidate lists halve across PRO_CPU /
  APP_CPU; results merged + globally re-sorted by `q_block0` (Pass 2)
  / concatenated (Stage 3).
- **Phase D** — `coarse_sync_split` partitions the carrier-bin range
  in two; each half scores independently, merge by
  `SyncCandidate.score` descending. Modest gain (-9 % stage 2) because
  `coarse_sync` is dominated by setup / sort / dedupe.
- **Phase E** — `stage1_inc.rs`. Per-chunk worker on APP_CPU
  (priority 3 — preempted by both the dual-core worker during decode
  and `wav_sim` during push, runs in the slack). Mirrors the audio
  buffer in PSRAM, locks auto-gain shift after the first 1 s, then
  advances the 92-pair FFT loop as each pair's 2 880-sample window
  becomes available. By the time the slot-end notify fires, all 92
  pairs are done and `take_spec()` returns a `Spectrogram` that the
  decode loop uses straight away — stage 1 disappears from the
  decode latency budget. Total stage-1 compute over a slot (~1.0 s)
  is hidden under capture (~6 % of the 15 s window).

Memory configuration (Core2-specific, in `sdkconfig.defaults`):

- `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` (was 16384) so the
  per-candidate 5 KB cs `Box` from `refine_candidates_with`'s
  `BinaryHeap` lands in PSRAM. With dual-core both halves hold
  ~75 KB simultaneously; at the previous threshold these stayed in
  internal DRAM and corrupted tlsf at the Pass 2 → Stage 3
  transition.

### New embedded binary: `rx-wavsim`

`embedded-poc/m5stack-core2/src/bin/rx_wavsim.rs` — streaming RX
bench that pumps the baked QSO WAVs into the `mfsk_ft8_stream_*` ring
at real-time pace (1 200-sample chunks every 100 ms, slot-boundary
notify on WAV completion) and runs the dual-core + Phase E decode
pipeline once per simulated slot. Validates the streaming + decode
path end-to-end without I2S PDM mic hardware.

Two new bin-side modules:

- `wav_sim.rs` — WAV-fed simulated capture task (substitute for I2S
  DMA capture). Pinned to PRO_CPU at priority 4; pushes 1 200-sample
  chunks through `mfsk_ft8_stream_push_i16`, signals decode via task
  notification on WAV completion, fans each chunk out to a per-chunk
  hook (used by `stage1_inc`).
- `stage1_inc.rs` — Phase E incremental stage-1 worker. Mirrors the
  audio buffer in PSRAM, computes per-pair FFTs as audio windows
  fill, exposes the prebuilt spectrogram via `take_spec()`.

End-to-end on Core2, 4-cycle run (qso1→2→3→1):

  qso1     1.83 s   3/3 ✓
  qso2     1.45 s   5/5 ✓ (incl. -17.9 dB OH3NIV, -18 dB LZ1JZ)
  qso3     1.98 s   7/7 ✓ (busy band, incl. -18.2 dB N1PJT)

Build: `cargo build --release --bin rx-wavsim`. Same `+esp` toolchain
+ `espflash` flow as the other binaries in the crate.

### Production firmware lessons surfaced by `rx-wavsim`

The simulated streaming bench reproduced five categories of bug that
real I2S DMA firmware would hit:

1. FIFO ring boundary — `STREAM_CAP == SLOT_LEN` evicts the leading
   samples, producing an 8.4 ms phase shift that drops weak-signal
   recall on busy bands (qso3 7→2). Cap `wav_sim`'s push at
   `SLOT_SAMPLES` to keep `slot[0..180_000]` aligned.
2. Capture-task priority inversion — wav_sim above main task means
   notify doesn't preempt, ring gets polluted with next WAV's prefix
   before decode peeks. Fixed by `vTaskPrioritySet(NULL, 6)` on main
   (wav_sim 4, dual_core worker 5).
3. Dual-core blocking → low-priority capture grabs idle PRO_CPU during
   decode's stage 2/3 worker waits.
4. `tlsf_malloc` heap corruption on dual-core transition — see
   `SPIRAM_MALLOC_ALWAYSINTERNAL` note above.
5. Worker stack underestimate — Stage 3 stacks ~5 KB of `LlrSet`
   intermediates, blew through 8 KB. Bumped to 16 KB.

These all manifest the same way on real I2S — sim catches them
deterministically, hours/days off the real-hardware bring-up budget.

## 0.5.2 — streaming capture API + metadata rebalance

API-additive minor over 0.5.1. Two threads:

1. **`mfsk_ft8_stream_*` C ABI** — bridge real-time receivers (I2S
   DMA, USB Audio, sound-card capture) to the slot-oriented decoder
   without each consumer rewriting the same ring buffer + resampler.
2. **README / Cargo.toml metadata** rebalanced so embedded targets
   (ESP32-S3, RP2350, Cortex-M, fixed-point hot path) appear as the
   primary positioning rather than a tail "PoC included" remark, and
   `uvpacket` is honestly framed as an experimental applied example.
   First crates.io release where the new wording surfaces.

### What's new

- `mfsk_core::core::dsp::resample::LinearResamplerI16To12k` — Q32
  fixed-point streaming linear resampler. Carries phase + last_in
  across calls so chunk boundaries don't introduce a discontinuity.
  Caller-buffer output (no per-call `Vec` allocation). Pure scalar
  i64 arithmetic — runs on FPU-less MCUs. Pairs with the new C ABI
  below.
- `mfsk-ffi-ft8::mfsk_ft8_stream_*` — opaque handle bundling the
  resampler with a fixed-cap 12 kHz ring buffer. Seven entry points
  (`_new` / `_free` / `_push_i16` / `_buffered_samples` /
  `_peek_latest` / `_drain` / `_clear`). Decoding itself is *not*
  bundled — capture and decode typically run on different cores or
  RTOS tasks; the caller takes a snapshot via `_peek_latest` into
  their own scratch and hands that to the existing
  `mfsk_ft8_decode_i16`. Available in both `host` and
  `embedded-fixed-point` builds — the streaming primitives are
  pure-arithmetic with no FFT / DSP backend dependency.
- `embedded-poc/m5stack-core2/src/bin/rx_skeleton.rs` — second
  binary in the existing m5stack-core2 crate that pairs the working
  FFT-planner glue with a placeholder PDM capture task and a 15 s
  decode-trigger loop. **Cross-build status: UNVERIFIED at the time
  of this release.** The file header lists four items the reader is
  expected to verify on real hardware (I2S PDM driver init shape,
  sample-rate clock, slot-boundary timer source, decode_one heap
  discipline). Built only when explicitly named:
  `cargo build --release --bin rx-skeleton`.
- `mfsk-ffi-ft8/examples/streaming_recipe.c` — single-file C example
  showing where the streaming wrapper fits in any I2S / USB Audio /
  ALSA capture loop. Platform-agnostic placeholder hooks; compiles
  via `gcc -c -I include`.
- `docs/EMBEDDED.md` — new "Streaming capture: I2S / USB Audio →
  12 kHz ring" section under §"Using from C / C++". Includes the
  typical RTOS wiring (capture task pushes, decode task peeks every
  15 s), notes on slot-boundary alignment (decode_block tolerates
  ±2 s drift via coarse-sync, so NTP / GPS / freerunning all work),
  and resampler-quality (~–55 dBc distortion in 200–3000 Hz, well
  below FT8 LDPC operating SNR).

### Tests

- 4 new resampler unit tests (`passthrough`, `48k→12k decimation`,
  `6k→12k upsample`, `chunked-input matches single-call`).
- 7 new `MfskFt8Stream` ABI integration tests
  (`new/free safety`, `passthrough`, `48k→12k`, `ring overwrite-oldest`,
  `drain advances tail`, `chunk-boundary seamlessness`, `clear`).

### Compatibility

- All 0.5.0 / 0.5.1 entry points unchanged. `mfsk_ft8_decode_i16` /
  `mfsk_ft8_decode_i16_alloc` / `mfsk_ft8_pack77` / `…_message_to_tones`
  / `…_tones_to_i16` / `…_tones_to_f32` keep their signatures.
- cbindgen-generated `mfsk_ft8.h` regenerated; new symbols append at
  the bottom.
- mfsk-ffi-ft8 stays `publish = false` (GitHub Releases binaries are
  the channel).

## 0.5.1 — `mfsk-ffi-ft8` adds the FT8 transmit chain

0.5.0 shipped the `mfsk-ffi-ft8` C ABI for the FT8 **decode** slice.
0.5.1 fills in the matching **transmit / synth chain** so embedded
C/C++ projects can both decode and generate FT8 from the same
library. API-additive — the v0.5.0 decode signature is unchanged.

### What's new

- `mfsk_ft8_pack77(call1, call2, report, out_message77[77])` — pack
  three message tokens (typical CQ, callsign, report-or-grid) into
  a 77-bit FT8 message via `mfsk_core::msg::wsjt77::pack77`.
- `mfsk_ft8_message_to_tones(message77[77], out_itone[79])` — LDPC
  encode + CRC-14 + Costas + Gray-mapping. Wraps
  `mfsk_core::ft8::wave_gen::message_to_tones`.
- `mfsk_ft8_tones_to_i16(itone[79], f0_hz, amp_i16, out, out_len)` —
  GFSK synth into a caller-provided i16 buffer (12 kHz mono,
  151 680 samples = 12.64 s of audio). No surprise heap-alloc.
- `mfsk_ft8_tones_to_f32(...)` — same as above, f32 amplitude /
  PCM.
- `mfsk_ft8_synth_output_len()` — returns 151 680 (the buffer-size
  constant, exposed for runtime sizing).

### Workflow fix (also lands in this release)

`release.yml` Xtensa CI builds (`release-ffi-esp32`,
`release-ffi-esp32s3`) needed `-Z build-std=core,alloc,panic_abort`
+ explicit `cargo +esp` to pick up rust-src — fixed for v0.5.1+.
The v0.5.0 release's Xtensa binaries were uploaded manually after
local cross-builds; v0.5.1's should attach automatically.

### Breaking changes

None. Existing `mfsk_ft8_decode_i16` / `mfsk_ft8_decode_i16_alloc` /
`mfsk_ft8_result_list_free` keep their v0.5.0 signatures.

## 0.5.0 — End-to-end embedded port (M5Stack Core2 reference)

The 0.4.x line built the embedded baseline (no_std + alloc, pluggable
FFT, fixed-point kernels). 0.5.0 ties it together: the first real-
audio FT8 decode pipeline running end-to-end on a hobbyist-class MCU.
Reference target M5Stack Core2 (Xtensa LX6 + esp-dsp ASM) decodes
real-QSO recordings at 2.9–4.0 s per 14 s slot, matching host f32
recall. The published library is unchanged in shape — what's new are
features, perf, and integration documentation.

### What's new

#### Embedded perf chain (Core2 LX6, fixed-point + fixed-point-llr)

vs the 0.4.4 first-flash baseline:

- **Stage 1 (spectrogram)**: −42 % via two-for-one real-FFT trick.
  Pack two consecutive real audio frames into one complex N=4096 FFT
  and recover per-frame spectra by post-butterfly. Halves FFT count
  on the standard FT8 slot (184 → 92). `compute_spectrogram` under
  `fixed-point`.
- **Stage 2 (coarse_sync)**: feature-gated i32 inner loop
  (`fixed-point-coarse-i32`) — **off by default**; helps FPU-less
  targets (RP2040, M0+), hurts ~25 % on LX6/LX7 where FPU+ALU
  parallelism is the win. The default f32 path stays.
- **Stage 3 (refine + BP)**: four wins in series —
  - `BpScratch` reusable pool eliminates 7 `vec![]` per BP call
    (~12 KB Q11i16 / 24 KB f32) → 1 alloc per slot instead of ~75.
    Old `bp_decode_*` signatures keep working (allocate scratch
    internally); new `bp_decode_*_with_scratch` for hot-path callers.
  - `SymMask::SyncBlocks12` skips re-filling Costas block 0 in the
    Stage-3 SyncOnly fill (Pass 2 already populated it). 56 DFT /
    surviving candidate saved.
  - Step-2 BP staircase **lazy LLR + skip-variant-a** — Step 1's
    `compute_llr_fast` already produces llra/llrd; Step 2 reuses
    llrd, lazy-computes llrb (nsym=2) and llrc (nsym=3) only as
    needed. Variant a is identical to Step 1's input, so it's
    skipped entirely. New `compute_llr_partial<P, S, T>(cs, nsym)`
    in `core::llr` and the FT8 wrapper.

  Combined Core2 stage-3 wall-clock: qso1 858 → 690 ms (−20 %),
  qso2 1098 → 920 ms (−16 %), qso3 (busy) 2971 → 1826 ms (−39 %).

  **Total slot time** (Core2): qso1 3.75 → **2.88 s**, qso2 3.99 →
  **3.10 s**, qso3 5.17 → **3.99 s** — all three within −22 to
  −24 %, recall preserved (3 / 5 / 7 results).

#### `is_plausible_callsign` — stricter CRC false-positive filter

New `mfsk_core::msg::wsjt77::is_plausible_callsign` adds an ITU
prefix-allowlist check on top of the structural `is_valid_callsign`.
Catches `Z74QTJ`, `Q1ABC`, etc. — letter+digit prefix gaps where
random codewords passing CRC-14 land disproportionately on busy
bands. ~85 hardcoded entries (1-char letter prefixes
`F G I K M N R W` + 2-char letter+digit ITU Appendix-42 series).
`is_plausible_message` (the existing block-result filter at
`ft8::decode_block::process_candidates_with`) now uses it
automatically. `is_valid_callsign` stays public for callers who
want the permissive structural check.

#### New Cargo features

| Feature | Default | Purpose |
|---|---|---|
| `fixed-point-cs` | off | `Cmplx<Q14i16>` cs storage (4 KB instead of 8 KB per Box). |
| `fixed-point-coarse-i32` | off | Stage-2 i32 path. **FPU-less only.** |
| `profile-coarse` | off | Always-on coarse_sync sub-stage timing. |

`std` no longer transitively pulls `rustfft` (now only `fft-rustfft`
does). Lets `std` be enabled on Xtensa for `std::time::Instant`
without dragging the host-only FFT crate.

#### New public API surface (additive)

- `core::llr::compute_llr_partial<P, S, T>(cs, nsym)` — single-
  variant LLR for the staircase.
- `ft8::llr::compute_llr_partial<T>(cs, nsym)` — FT8 wrapper.
- `fec::ldpc::bp::BpScratch<P, T>` + `bp_decode_generic_nms_with_scratch` +
  `bp_decode_nms_with_scratch` — pool-aware BP entry points.
- `ft8::decode_block::SymMask::SyncBlocks12` — Costas blocks 1+2 mask.
- `msg::wsjt77::is_plausible_callsign`.

### Documentation

- New [`docs/EMBEDDED.md`](docs/EMBEDDED.md): targets we test,
  feature-flag map, the FFT / dot-product extern contracts, BASIS
  scratch placement, Q-format reference, Core2 perf ballpark, and
  what we deliberately don't ship (audio I/O, RTOS glue, display,
  networking).
- `embedded-poc/m5stack-core2/` is the worked example for one
  target; raw measurement logs in `embedded-poc/m5stack-core2/logs/`.

### Known limitations

- **SNR estimate on the embedded path** reads ~4–12 dB low on strong
  signals vs the host wide-band `decode_frame`. Cause is structural
  (the block path skips the Wiener equalisation that boosts strong-
  signal estimates on the wide-band path), not quantisation —
  reproducible identically on host f32 and fixed-point. Documented
  in `docs/EMBEDDED.md`; proper fix deferred to a future minor.
- I2S live-capture wiring on the m5stack-core2 example is not
  shipped; the example decodes baked WAV assets.

### Breaking changes

None. The release is API-additive: new pub fns / types added; old
ones keep working unchanged.

## 0.4.4 — LDPC min-sum kernels (NMS + OMS) for embedded targets

Adds `NormalizedMinSum` and `OffsetMinSum` check-node update kernels
to the shared LDPC belief-propagation decoder
(`fec/ldpc/bp.rs::bp_decode_generic_kind`). Default behaviour is
unchanged — `BpKind::SumProduct` (WSJT-X-equivalent log-domain
sum-product) stays the implicit pick on every existing code path.
Embedded callers can now opt into a min-sum approximation that
skips the per-iteration `tanh` / `atanh` cache, trading ~0.05–0.15 dB
threshold for substantially faster decode on FPU-poor targets and
materially less work on host targets too.

### What's new

- **`mfsk_core::core::BpKind`** enum:
  - `SumProduct` — default, WSJT-X parity.
  - `NormalizedMinSum { alpha: f32 }` — `L_c→v ≈ α · sign(∏) · min|L|`,
    typical α ≈ 0.7..0.9.
  - `OffsetMinSum { beta: f32 }` — `L_c→v ≈ sign · max(min|L| − β, 0)`,
    typical β ≈ 0.5.
- **`FecOpts.bp_kind`** field threading the choice into the LDPC
  codec impls (`Ldpc174_91` for FT8/FT4, `Ldpc240_101` for FST4 +
  uvpacket). Existing `FecOpts {…}` literals migrate via
  `..FecOpts::default()` rest syntax.
- **min1 / min2 + XOR-sign trick** in the min-sum path: per check
  node, the two smallest `|L|` and the parity of incoming negatives
  are computed once per iteration; the per-edge output then picks
  `min2` if the edge's own `|L|` owns `min1`, else `min1`. The
  inner loop is therefore O(check_degree) instead of the
  sum-product's O(check_degree²) `tanh`-cache lookups, before any
  floating-point savings are counted.

### Sign convention fix

The WSJT-X sum-product path computes `tmn = ∏ tanh(−toc/2)` then
`tov = 2 · atanh(−tmn)` — algebra gives the output sign as
`(−1)^nrw · sign(∏ toc[s≠j])`. The textbook NMS formula
`α · sign(∏) · min` lacks that `(−1)^nrw` factor, so on
odd-row-weight checks (nrw=7 in LDPC174_91, mixed in LDPC240_101)
the unflipped NMS output disagreed with SP and BP diverged in
noise. The implementation XORs `nrw_ichk & 1` into the extrinsic
sign so the new kernels produce sign-compatible messages.

### Verified

- `tests/ldpc_min_sum.rs::ldpc174_clean_round_trip_every_kind` and
  `ldpc240_clean_round_trip_every_kind` — all three kernels recover
  the original info from clean LLRs in ≤ 5 BP iterations.
- `tests/ldpc_min_sum.rs::nms_oms_threshold_within_0p5_db_of_sum_product`
  (`#[ignore]`d so the default `cargo test` stays fast) sweeps Eb/N0
  over 30 trials per dB and asserts NMS / OMS thresholds land within
  0.5 dB of `SumProduct`. Observed at `target=0.5`: SP / NMS / OMS
  all hit 50 % decode rate at 1.5 dB Eb/N0; per-dB rates differ by
  ~3-7 percentage points (well under the published 0.1-0.2 dB NMS
  calibration loss for short LDPC codes).
- `cargo test --features full --release -- --include-ignored` all
  green; clippy + fmt clean; `cargo publish --dry-run` clean.

### Embedded-target notes

ESP32 family FPU coverage:

- ESP32 (LX6) / ESP32-S2 / ESP32-S3 (LX7) all ship single-precision
  hardware FPU — the f32 NMS path runs at native float speed.
- ESP32-C3 / C6 / H2 (RISC-V) have **no** FPU; f32 is software
  emulated. NMS still helps (the per-iteration `tanh`/`atanh` cost
  is dominant), but a fixed-point i16 LLR follow-up will give
  another 2-4× on these targets. Tracked as next-PR scope.

The `embedded-rx` Cargo preset still defaults to SumProduct so an
existing build sees no behaviour change. Embedded consumers
explicitly construct `FecOpts { bp_kind: BpKind::NormalizedMinSum
{ alpha: 0.75 }, ..Default::default() }` to opt in.

## 0.4.3 — Q65 multi-period averaging (ionoscatter port)

Adds the WSJT-X `iavg=1` / `iavg=2` averaged-decode path to the
Q65 receive chain. The 0.4.2 honest-test pass left the WSJT-X
ionoscatter reference set (`30A_Ionoscatter_6m/*.wav`) at 0/4
decoded with an explicit `multi-period averaging not yet ported`
docstring; this release ports it and recovers `K1JT K9AN R-16` —
the actual exchange in the recording, decoded by averaging the 4
slots before BP / fast-fading. Single-period EME paths
(`60A_EME_6m`, `60D_EME_10GHz`) are unchanged.

### What's new

- **`mfsk_core::q65::decode_multi_period_for<P>`** + `decode_multi_period`
  Q65-30A wrapper. Stateless API: pass `&[&[f32]]` of audio slots
  in chronological order and an optional AP-list candidate set;
  returns deduplicated `Vec<Q65Decode>`. Internally maintains an
  exponential-moving-average spectrogram with time constant
  `min(navg, 4)` (matches WSJT-X's `lib/qra/q65/q65.f90:300-304`
  accumulator) and runs coarse sync search on the running average
  before each candidate goes through a 3-stage decode ladder:
  - **Stage B (AP-list)** when `ap_codewords` is supplied — averaged
    Bessel-metric intrinsics → `Q65Codec::decode_with_codeword_list`.
    Mirrors WSJT-X's `iavg=1` q3 path.
  - **Stage C-fading** — averaged wide energies →
    `intrinsics_fast_fading` BP, swept across
    `b90·Ts ∈ {3, 8, 15} × {Gaussian, Lorentzian}`.
  - **Stage C-plain** — averaged narrow energies → Bessel-metric BP
    fallback.
- Per-slot at most one decode is appended (the first stage that
  succeeds for any candidate wins). Repeated identical messages
  across slots are deduplicated by `(message, ±4 Hz freq)` because
  the running EMA collapses copies of the same QSO into one
  signal.

### Verified

- `tests/q65_wsjtx_samples.rs::ionoscatter_6m_full_stack_decodes_via_averaging`
  asserts ≥1 decode across the 4-slot WSJT-X ionoscatter stack.
  Both paths (no AP-list, K1JT/K9AN AP-list) recover
  `K1JT K9AN R-16` at 1010 Hz / dt=0.90 s.
- `tests/q65_wsjtx_samples.rs::ionoscatter_6m_receive_chain_runs`
  smoke test still passes (single-period chain unchanged).
- 6 m EME (`eme_6m_sample_yields_decode_with_ap` — 3 W7GJ exchanges
  via plain + AP-CQ) and 10 GHz EME
  (`q65_fast_fading::eme_10ghz_reference_decodes_with_fast_fading`
  — 3 VK7MO/K6QPV decodes via fast-fading) untouched.
- `cargo test --features full --release -- --include-ignored` all
  green; clippy + fmt clean.

### Implementation notes

- Reuses the existing `extract_data_energies` /
  `extract_data_energies_wide` energy extractors (private to
  `q65::rx`), `mfsk_bessel_metric` and `intrinsics_fast_fading`
  intrinsics builders, `Q65Codec::{decode, decode_with_codeword_list}`
  decoders, and `super::search::coarse_search_on_spec_for<P>`. No
  changes to `Spectrogram`, `SearchParams`, or any existing public
  surface — the multi-period entry is an additive layer.
- Stateless by design: real-time consumers manage the slot buffer
  themselves. A `Q65Averager` struct + WSJT-X-style even/odd
  parity (`iseq=0/1`) split are reasonable follow-ups but were not
  needed to clear the WSJT-X reference set.
- AP-list path uses the existing `standard_qso_codewords(my, his,
  grid)` builder. When the call+grid pair is unknown, pass `None`
  and the fading + plain BP ladder handles it.

## 0.4.2 — documentation consistency pass

Patch release. No public-API changes; host builds (`--features full`)
are byte-identical to 0.4.1. Brings the README, crate docs, mfsk-ffi
README and `docs/LIBRARY.{md,ja.md}` back in line with the 0.4.x
reality (embedded port, Q65 family, registry semantics).

### Documentation

- README, lib.rs feature table and CHANGELOG updated to reflect the
  full feature surface: `fft-rustfft` / `fft-extern` /
  `embedded-tx` / `embedded-rx` / `esp32s3` are now listed, the
  example `Cargo.toml` snippet uses `version = "0.4"`, and the
  `Status` section references the embedded port instead of the
  retired 0.3.x baseline.
- `docs/LIBRARY.md` §4 (and the Japanese mirror) gain a single
  receive-pipeline data-flow diagram covering the
  `samples → coarse_sync → refine → symbol_spectra → equalize_local
  → compute_llr → P::Fec::decode_soft → P::Msg::unpack` chain, plus
  a paragraph spelling out *why* there is no `Demodulator` /
  `Receiver` trait (the path is realised as free functions generic
  over `P: Protocol` so monomorphisation produces per-protocol code
  identical to a hand-written decoder).
- `FecCodec` trait docstring (`mfsk-core/src/core/protocol.rs`) now
  has a "Symbol granularity" section: the trait surface is bit-level
  by contract, non-binary codes (Q65 QRA over GF(2⁶), JT65 RS over
  GF(2⁶)) pack/unpack symbols inside their own `encode`, and
  `Q65Fec::decode_soft` returns `None` by design — the real Q65
  decode runs symbol-level via `crate::fec::qra::Q65Codec` from
  `crate::q65::rx::decode_at_for`.
- README adds a "Static set of protocols" callout: `PROTOCOLS` is
  fixed at compile time by Cargo features; there is no runtime
  `register_protocol()` API by design (every wired ZST is verified
  by `tests/protocol_invariants.rs` and that guarantee can't be
  extended to types unknown at compile time).
- `mfsk-ffi/README.md` protocol table gains the missing Q65 row
  plus the dedicated `mfsk_q65_decode{,_with_ap,_fading,_with_ap_list}`
  + `mfsk_encode_q65` ABI entries that 0.2.0 already shipped.

### Tests

- `tests/protocol_invariants.rs` cross-protocol asserts tightened:
  - `every_wired_protocol_has_a_unique_protocol_id` now derives the
    expected distinct-id count imperatively from the active feature
    flags, so the `unique.len() == expected` assertion is meaningful
    under any feature combination — not just `--features full`
    where it was previously gated.
  - `registry_entries_match_zst_trait_constants` now asserts that
    the count of verified entries equals `PROTOCOLS.len()`. Adding a
    new ZST + registry entry but forgetting the matching `check!`
    line trips this count cross-check instead of silently passing,
    and uvpacket sub-modes are now covered by their own `check!`
    lines.
  - Module doc updated to call out that the per-protocol invariant
    tests are feature-gated, so `cargo test --features full` is
    required for full eleven-ZST coverage.
- `mfsk-core/src/lib.rs` "Trait surface verification" section now
  reports the actual ~25 invariants split across modulation /
  frame-layout / codec layers (was "17") and notes the default
  `cargo test` only exercises FT8 + FT4.

### crates.io metadata

- `mfsk-core/Cargo.toml` `description` rewritten to mention the
  embedded-port story (`no_std + alloc`, pluggable FFT, ESP32-S3
  PoC) so the crates.io listing card matches the README.
- `categories` extended with `embedded` and `no-std` (now 5 of 5
  slots used).

### CI

- `feature-matrix` job adds `q65`, `uvpacket`, `embedded-tx` and
  `embedded-rx` rows. The embedded entries build the library
  no_std + alloc on the Linux host; the standalone `embedded-poc/`
  Xtensa binaries remain excluded from CI (PoC scope).

## 0.4.1 — embedded port (no_std + alloc, FFT trait, ESP32-S3 PoC)

Adds an embedded-target port without breaking the existing host
API. Host builds (`--features full`) are byte-identical to 0.4.0.

### What's new

- **`no_std + alloc` builds work end-to-end.** Default features
  still pull `std` so existing users see no behaviour change; new
  presets `embedded-tx` (TX synthesis only) and `embedded-rx`
  (full decode pipeline, requires caller-supplied FFT) build with
  `--no-default-features` against `xtensa-esp32s3-espidf`,
  `thumbv8m.main-none-eabihf`, etc.
- **Pluggable FFT backend via `mfsk_core::core::fft`.** New
  `Fft` / `FftPlanner` trait pair; the rustfft path stays the host
  default, embedded callers plug in their own impl through the
  `fft-extern` feature + an `extern "Rust"` factory function.
- **Caller-buffer TX APIs.** `*_into(out, …)` variants for FT8 /
  FT4 / WSPR / uvpacket synthesisers + `*_OUTPUT_LEN` constants
  let embedded callers drive I2S DMA buffers without per-burst
  `Vec` allocations. Vec-returning convenience wrappers preserved.
- **ESP32-S3 PoC binary** at `embedded-poc/esp32s3/` (excluded
  from the host workspace; uses `+esp` toolchain). Wires
  `mfsk-core --features fft-extern` to esp-dsp's hand-written
  Xtensa FFT (`dsps_fft2r_fc32_ae32_`) via `esp-idf-sys`'s
  managed-component pipeline. Validates the embedded port
  builds-and-links on real hardware.

### Workarounds bundled

- **Xtensa LLVM 19.1.2 codegen bug**: `if cond { 0.5_f32 }
  else { 1.0_f32 }` triggers `XtensaISD::PCREL_WRAPPER`
  instruction-selection SIGSEGV. `mfsk_core::ft8::decode` and
  `mfsk_core::core::pipeline` rewrite the gain calculation as
  `1.0 - 0.5 * (cond as u32 as f32)` (functionally identical;
  PER-sweep tests unchanged).

### New / changed features

| Feature | Default | Notes |
|---|:---:|---|
| `std` | ✓ | Already-on for host builds; bundles `alloc`. |
| `alloc` |   | Bare `no_std + alloc`. |
| `embedded-tx` |   | `alloc + ft8 + ft4 + wspr` (synth-only). |
| `embedded-rx` |   | `embedded-tx + fft-extern` (decode-capable). |
| `esp32s3` |   | Alias for `embedded-rx`. |
| `fft-rustfft` | ✓ | Host default; pulls `rustfft`. |
| `fft-extern` |   | Caller supplies `mfsk_core_make_default_fft_planner`. |
| `parallel` | ✓ | Now requires `std` (rayon is std-only). |

### Implementation notes

- `num-complex` and `crc` switched to `default-features = false`;
  `num-traits = "0.2", features = ["libm"]` added so call sites
  can `use num_traits::Float` under no_std.
- `std::*` references in the decode-side modules replaced with
  `core::*` / `alloc::*` equivalents. `std::collections::HashMap`
  in `msg::hash_table` swapped for `alloc::collections::BTreeMap`
  (small LRU bounded at 1000 entries; O(log n) lookups dwarfed
  by surrounding LDPC cost).
- `core::dsp::{downsample, subtract}`, `core::{sync, llr,
  pipeline}`, `wspr::{rx, spectrogram}`, etc. moved to the FFT
  trait via `core::fft::default_planner()`.

### Verified

- `cargo test --features full --release -- --include-ignored`
  passes (262 tests + the PER sweep cells unchanged).
- `cargo +esp build --target xtensa-esp32s3-espidf
   --no-default-features --features esp32s3 -Zbuild-std=core,alloc` ✓
- `cargo build --target thumbv8m.main-none-eabihf
   --no-default-features --features esp32s3` ✓
- ESP32-S3 PoC links the esp-dsp ASM FFT through the trait,
  ELF ~1.5 MB total / ~440 KB code.

## 0.4.0 — Q65 + abstraction unification

First release on the 0.4 line; cumulative since the 0.2.1 crates.io
publish. The headline is **the WSJT-family API surface**: a new
protocol (Q65-30A), trait-level cleanups that close abstraction
leaks the multi-protocol port surfaced, and a registry that gives
every protocol a uniform metadata view. The in-tree `uvpacket`
applied-example module is also rebuilt end-to-end (separate
section below) but it is gated behind `--features uvpacket` and
not part of the default-features API.

### WSJT-family additions (BREAKING vs 0.2.1)

- **Q65-30A** — full decode / encode / synthesis port from WSJT-X,
  including fast-fading log-likelihoods and AP-list handling. New
  `Q65a30` re-export, `--features q65`. (Cumulative across 0.3.x.)
- **`MessageCodec::verify_info`** — CRC verification lifted out of
  the LDPC layer into the message-codec trait, so the FEC code no
  longer has hard-coded knowledge of CRC-24 vs CRC-14 dispatch.
  Required because the same `Ldpc240_101` mother code is now
  shared across FST4 (CRC-24, 77-bit msg) and Q65 (CRC-14, 91-bit
  msg) and `uvpacket` (CRC-16, 96-bit raw bytes).
- **`Ldpc240_101` family unified** — single LDPC implementation
  used by FST4, Q65, and uvpacket (previously each had its own
  copy with subtle constant divergence).
- **`ProtocolMeta` registry** — every `Protocol` impl exposes a
  uniform metadata block (band rate, Costas pattern length,
  symbol count, …). Cross-protocol invariant tests assert the
  registry stays internally consistent (`tests/protocol_invariants.rs`).
- **`PacketBytesMessage`** — variable-length-bytes message codec,
  exposed as `--features packet-bytes`. Used as the byte-pipe
  building block for callers that want LDPC + interleaver + sync
  but do not need WSJT-77's structured-message dispatch.
- **`mfsk_core::VERSION`** — crate version constant, useful for
  FFI / WASM consumers verifying which build they linked against.

The trait reshuffle is the breaking part: `MessageCodec` impls
that were closed against `mfsk-core ≤ 0.2.1` need to add the new
`verify_info` method. Default implementations cover the common
"length-then-CRC" cases.

### `uvpacket` applied example (gated, redesigned)

`uvpacket` is an in-tree example of how the abstractions handle a
non-WSJT mode (3 kHz NFM / SSB voice-channel packet protocol).
**Breaking changes within `--features uvpacket` are expected
within the 0.4.x line** — pin the exact patch version if you depend
on it. ABI consolidation will follow in a future release.

The 0.4 redesign replaced the 0.3.x coherent-QPSK pipeline (which
failed over-the-air despite passing AWGN bench) with a
single-carrier **π/4-shifted DQPSK** modem at 1200 / 600 baud:

- 127-chip BPSK m-sequence preamble, **four primitive-polynomial
  variants** (one per `Mode`). Sync identifies the time offset
  and the payload mode in one matched-filter pass per centre.
- **9-tap T-spaced LMS equaliser** trained closed-form on the
  preamble. Differential demod is invariant to constant phase
  rotation and tolerates LO walk / clarifier offset to the AFC
  search-range limit; no pilots needed.
- **Dedicated header LDPC block** (Robust, unpunctured) carries
  `(block_count, app_type, sequence)` + CRC-16. Receiver decodes
  the header first (1 LDPC), reads `n_blocks`, then decodes the
  payload (`n_blocks` LDPCs). Total `1 + n_blocks` LDPC decodes
  per frame, vs ≤ 128 brute-force before.
- **AFC** at sync time, ±200 Hz default; callers widen for
  harsher channels.
- **UltraRobust** mode (header_code 0): half-baud (600 Hz)
  variant of Robust for marathon QSL on weak SSB / V-UHF mountain
  paths. ~4 dB tougher than Robust on every fading channel
  measured (Rayleigh, SSB realistic, FM realistic), see the
  positioning matrix in `docs/UVPACKET.md` §3.1.
- **WSJT-X-compatible SNR reporting** on every decoded frame
  (`DecodedFrame.snr_db`, dB / 2.5 kHz reference, −30 dB floor).
  Per-mode calibrated to ±0.3 dB residual against AWGN truth.
- **Shared-pair preamble correlator** — auto-detect path shares
  the differential pair products `aᵢ = mf[k]·conj(mf[k-1])`
  across the 3 NSPS_BASE preambles, ~36 % per-offset reduction at
  K=3. Bit-identical PER vs the per-preamble form (verified via
  `tests/uvpacket_per_modes_sweep`).

Removed from the 0.3 uvpacket:
- All coherent-QPSK encode / decode entry points; pilot symbols
  and the LMS phase tracker.
- 31-chip preamble + spread-header indirection (replaced by the
  4-variant 127-chip preamble + dedicated header block).
- Brute-force `(mode × n_blocks)` layout sweep.
- `framing::pack` / `framing::unpack` (replaced by `pack_header` /
  `unpack_header`; mode field removed from the header word).
- `UvFast` mode (header_code 2 ≤ 0.3.5); replaced by `UvUltraRobust`.

### Performance characterisation

PER thresholds (90 %, Eb/N0_info / SNR_2.5kHz dB) for the four
uvpacket modes on the channel models in
`mfsk-core/tests/common/air_channel.rs`:

| Mode (net bps) | AWGN | Rayleigh fd=5 | SSB realistic | FM realistic | Multipath 3-tap |
|---|---:|---:|---:|---:|---:|
| **UltraRobust** (504) | +4 / −3.7 | +8 / +0.3 | +4 / −3.7 | +6 / −1.7 | +6 / −1.7 |
| Robust (1008) | +6 / +1.3 | +12 / +7.3 | +8 / +3.3 | +10 / +5.3 | +8 / +3.3 |
| Standard (1200) | +8 / +4.0 | +12 / +8.0 | +8 / +4.0 | +10 / +6.0 | +10 / +6.0 |
| Express (1800) | +10 / +7.8 | +20 / +17.8 | >+15 / >+12.8 | +20 / +17.8 | fail |

Reproduce via `cargo test --release --features uvpacket --test
uvpacket_per_modes_sweep -- --ignored --nocapture`.

## 0.3.5 (continued) — 2026-04-29

uvpacket sync detector rewrite — replaces `|⟨preamble, mf_out⟩|²` as
the per-offset score with the **normalised coherence ratio**
`|⟨preamble, mf_out⟩|² / Σ|sᵢ|²`, fixing a structural false-sync
class that the 0.3.4 / 0.3.5 sync gate band-aids couldn't reach.

(In-place 0.3.5 update — no version bump to keep the published-crate
history clean. The earlier 0.3.5 entry below describes the
non-zero-median fix that this commit completes.)

### Background

The old detector summed `±sᵢ` for the 31 BPSK preamble bits and used
`|sum|²` as the match score. By Cauchy-Schwarz that magnitude is
bounded by `N·Σ|sᵢ|²`, but the bound is reached **only** when `sᵢ ∝
b̄ᵢ` for all i (the actual coherent-preamble signature). For a single
dominant sample (microphone click, USB plug-event, fan tick, …) the
sum is nearly as large as if the whole preamble had aligned, yet
*only one* sample contributed coherently. The old detector saw
"large magnitude" and accepted; the LDPC sweep then ran on noise.

uvpacket-web field reports showed `max/median = 139` from a single
field-amplitude impulse, vs `≤ 17` for proper noise. New direct
measurement: an isolated single-sample spike of 0.5 amplitude in
30 k samples of noise gives `max/median = 2209` under the old
detector — false sync every snapshot in environments with any
impulsive interference.

### Fixed

- New `preamble_coherence_score(mf_out, offset) -> f32` returns the
  normalised ratio. Bounded above by `PREAMBLE_LEN = 31`; saturates
  at 31 for a coherent BPSK preamble; collapses to ~1 for any
  single-sample dominance or random uncorrelated content.
- `rx::decode` and `rx::diag_sync_stats` now generate scores via
  `preamble_coherence_score` instead of `preamble_correlation(...).
  norm_sqr()`. The downstream `SYNC_PEAK_REL_TO_MEDIAN = 20×` gate
  and the threshold-relative-NMS peak picking are unchanged — only
  the *scoring metric* changed.

### Empirical (release, 30 000-sample buffers)

| scenario               | old detector | new detector |
|------------------------|--------------|--------------|
| pure white noise       | 13.5         | 12.3         |
| 1500 Hz tone           | 6.3          | 6.2          |
| 1200 Hz tone           | 2.5          | 2.8          |
| **noise + 0.5 click**  | **2 209**    | **10.6**     |
| AM(1500 Hz, 1200 Hz)   | 8.4          | 8.9          |
| strong tone @ 1500 Hz  | 6.2          | 7.1          |
| **real preamble +10 dB**| (varies)    | **46.5**     |

The impulse case dropped from 2 209 to 10.6 (well below the 20×
gate); the real-preamble case climbed to 46.5 (well above). Clean
separation, while every other point on the table is roughly
unchanged. All 271 existing uvpacket tests pass byte-identically —
the metric is mathematically equivalent for actual preambles.

### Roadmap note

A longer preamble (127 or 255 bits) would push the real-signal
saturation ratio higher (linear in `N`) without affecting the
noise floor, giving more headroom. That's a wire-format break and
deferred for now; the 31-bit + coherence-score combination already
restores the gate's intended noise rejection.

## 0.3.5 — 2026-04-29

uvpacket sync-gate hardening: 0.3.4's `max/median ≥ 20` rejection
collapsed to a no-op when the input buffer was partially zero (e.g.
the first few seconds of a fresh ring-buffer capture in uvpacket-web,
where the unfilled portion of the worklet's ring buffer was being
returned as zeros). With > 50 % of correlation scores at exactly 0,
`median(scores) = 0` and the defensive `if median <= 0 { return true }`
branch let noise through to the LDPC sweep — the very runaway 0.3.4
was supposed to fix.

### Fixed

- `global_max_is_sync_outlier` and `diag_sync_stats` now compute the
  median over **non-zero scores only**. An all-zero buffer (no audio
  at all) trivially rejects; a partially-zero buffer (e.g. ring-buffer
  pre-fill) produces a meaningful median from the real-audio portion.
- Adds `tests/uvpacket_noise_floor.rs::noise_floor_half_zero_buffer`
  as a regression test (7 s buffer, first half zeros, second half
  σ=0.003 noise). Confirmed: 0 frames, 2.3 ms decode.

No behaviour change for buffers without zero-padding artefacts (all
271 existing uvpacket tests still pass byte-identically).

## 0.3.4 — 2026-04-29

uvpacket RX: hard sync-rejection on the auto-detect path, fixing a
runaway-CPU bug discovered by uvpacket-web (https://jl1nie.github.io/webft8/uvpacket/)
under steady-state listening on noise-only audio.

### Fixed

- `uvpacket::rx::decode` and `uvpacket::rx::decode_multichannel` now
  short-circuit when the global preamble-correlation peak is not a
  clear outlier from the score-distribution median (≥ `20×` median).

  On pure χ²(2)-distributed noise the natural `max/median` ratio
  saturates around `ln(N)/ln(2) ≈ 17` (extreme-value statistics over
  `N ≈ 80 k` correlation offsets in a 7 s buffer); on real signal at
  +1 dB Eb/N0_info — Robust mode's 50 %-PER threshold — the ratio
  is `≈ 56`. The 20× gate cleanly separates them with a 4.5 dB
  signal-side margin (rejection at `−3.5 dB SNR`, well below any
  rate's actual decoding threshold).

  Without the gate, the 50 % relative-peak threshold left ~290 false
  NMS-survived peaks per 7 s noise buffer, each running a
  `4 modes × 32 n_blocks` LDPC BP+OSD-2 sweep — empirically 30–180 s
  of release-mode work per call. With the gate, a noise buffer
  short-circuits in `~330 µs` (≈ 7 000× speedup; new test
  `tests/uvpacket_noise_floor.rs`).

  No behaviour change for real signals — all 271 existing uvpacket
  tests still pass byte-identically.

## 0.3.3 — 2026-04-29

Multi-channel SSB receive + slotted-ALOHA TX primitives for
uvpacket. The 0.3.2 single-station SSB experience generalises
to a private group sharing one RF channel (e.g., 430.090 MHz
USB) where each TX picks a random free audio slot via LBT.

WSJT-family modes and the existing single-channel uvpacket API
are unchanged.

### Added

- `mfsk-core::uvpacket::rx::decode_multichannel(audio,
  &mc_opts, &fec_opts) -> Vec<(f32, DecodedFrame)>` — coarse-
  grid frequency sweep across the configured SSB passband,
  per-grid-point matched filter + preamble peak detection,
  frequency-axis NMS to drop adjacent-grid duplicates, and
  per-peak `(mode × n_blocks)` decode. Returns the detected
  audio centre alongside each decoded frame.
- `MultiChannelOpts { band_lo_hz, band_hi_hz, coarse_step_hz,
  nms_radius_hz, peak_rel_threshold }` with sensible defaults
  (300–2700 Hz / 25 Hz / 600 Hz / 0.5).
- `mfsk-core::uvpacket::rx::measure_slot_energies(audio,
  &mc_opts, slot_spacing_hz) -> Vec<SlotEnergy>` — per-slot
  mean matched-filter magnitude survey for the LBT step before
  a slotted-ALOHA TX. Policy-free: the helper just reports
  energies, the caller picks free-vs-busy by their own rule.
- `SlotEnergy { audio_centre_hz, mean_mf_magnitude }`.

### Operating concept

A private group shares one RF SSB channel. Inside the audio
passband the modem recognises a 1200 Hz slot grid (typically
800 Hz and 2000 Hz centres in 2.4 kHz SSB). Each TX:

1. Listens — captures a short audio buffer, runs
   `measure_slot_energies` to survey occupancy.
2. Picks a random free slot — uniform-random from the slots
   below an application-chosen energy threshold.
3. Transmits — `tx::encode(&header, &payload, picked_centre)`.

This is **slotted ALOHA on the audio-frequency axis**, plus
LBT. CSMA/CD proper isn't applicable to half-duplex SSB radio;
slotted ALOHA + LBT + ARQ at the application layer behaves
equivalently with much less mechanism, and lines up with the
natural amateur-radio "watch the frequency, find a clear spot,
transmit" practice.

mfsk-core supplies the primitives only; the application layer
owns the RNG, the ARQ ACK + retry state machine, and any
voice-mode coexistence policy.

### Cost

`decode_multichannel`: ~1 matched-filter pass per coarse-grid
step. With default settings (300–2700 Hz, 25 Hz step) ≈ 96
passes ≈ 70 ms in release per second of audio.
`measure_slot_energies`: 1 MF pass per slot, ~1 ms each at 1
sec audio. Effectively free.

### Empirical

- 2 simultaneous frames at 800 Hz / 2000 Hz centres in clean
  audio: both decoded with detected centres within ±50 Hz of
  truth.
- Same setup at +8 dB Eb/N0_info AWGN: both decoded.
- Slot survey with one busy slot at 800 Hz: busy slot's mean
  MF magnitude is > 5× the free slot's.

## 0.3.2 — 2026-04-29

Focused single-feature release on top of 0.3.1: **AFC (automatic
frequency control) for uvpacket** so the modem operates correctly
on SSB carriers without requiring TX/RX VFO-dial alignment.

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) and the
0.3.1-shipped uvpacket NFM path are unchanged. No breaking API
changes — AFC is opt-in via a new entry-point function.

### Added

- `mfsk-core::uvpacket::rx::decode_known_layout_with_afc(audio,
  sample_offset, audio_centre_hz, mode, n_blocks, &fec_opts,
  &afc_opts) -> Result<DecodedFrame, DecodeError>`. Runs the AFC
  search, then re-invokes the standard decoder at the corrected
  centre frequency.
- `mfsk-core::uvpacket::rx::AfcOpts { search_hz: f32 }` with
  `Default` returning `AfcOpts { search_hz: 200.0 }`. The total
  search window is `±search_hz`; 200 Hz covers typical SSB VFO
  mismatch worst-case.
- `pub fn diag_estimate_freq_offset` — test/characterisation hook
  that returns the AFC's Δf estimate without running the full
  decode roundtrip.
- `tests/uvpacket_afc.rs` — round-trip clean recover at ±150 Hz,
  baseline-fails-at-offset control, ±100 Hz at +6 dB AWGN
  (10/10), optional accuracy-print diagnostic.

### Algorithm

Frequency-grid preamble-correlation search at 25 Hz steps across
`[−search_hz, +search_hz]` (default 17 candidates). At each
candidate `audio_centre_hz + Δf_test`, run the matched filter and
take the best preamble-correlation magnitude over the ±NSPS
jitter window. Pick the coarse-grid winner, then parabolic-fit
the three adjacent magnitudes for sub-grid resolution. Re-run
the standard decoder at the corrected centre frequency.

The first attempt was an FFT-over-chip-rate-samples (cheap but
wrong): at non-trivial Δf the integer-sample preamble correlator
that picks `best_off` itself rolls off as `sinc(δ · 31 / 1200)`,
landing on noise samples for `|δ| ≳ 20 Hz` — the FFT then
operates on garbage. The frequency-grid search sidesteps this
because the preamble correlator magnitude itself peaks at the
correct Δf.

### Cost

~17× single-decode cost (full down-convert + matched-filter at
each grid point), ~50–100 ms total per attempted decode in
release mode. Tolerable for opportunistic SSB decode; can be
tightened by lazy-evaluating only enough grid points to
distinguish the winner from its neighbours, if profiling demands.

### Empirical accuracy

Clean-channel AFC estimate vs injected truth (search ±200 Hz):

```
Δf_true (Hz)  AFC_est (Hz)  decode
−150          −150.00       ✓
−100          −100.00       ✓
 −50           −49.99       ✓
 −20           −22.34       ✓ (mid-grid; LMS absorbs residual)
   0             0.00       ✓
 +20           +22.34       ✓
 +50           +50.00       ✓
+100          +100.00       ✓
+150          +150.00       ✓
+200          +200.00       ✓
```

≤ 0.01 Hz error at multi-of-25 Hz Δf; ≤ 2.5 Hz error at mid-grid
Δf. The LMS quadratic phase fit downstream absorbs the residual
without trouble (residual frequency offset over a 0.5 s burst is
within the LMS linear-term capacity).

### Operating envelope

With AFC, uvpacket decodes correctly across the full SSB VFO-
mismatch range (±200 Hz default; configurable). Combined with
the existing modem characterisation, this opens the modem up to
HF SSB weak-signal data and microwave SSB applications.

NFM users can keep using `decode_known_layout` — AFC is an extra
~50–100 ms per decode that's pure overhead on a static-VFO
channel.

### Known limitations

- AFC is per-frame static. Doppler-induced carrier drift across
  the burst is still absorbed by the LMS phase fit (constant +
  linear + quadratic), which works for ≤ ~10 Hz/s drift —
  typical for HF / VHF / UHF SSB.
- The auto-detect `decode()` path doesn't yet take an `AfcOpts`.
  Multi-frame SSB scans go through `decode_known_layout_with_afc`
  with caller-managed framing for now.

## 0.3.1 — 2026-04-29

Additive release on top of 0.3.0. Headline: the new `uvpacket`
applied-example module — a coherent QPSK + LDPC packet protocol
that fits inside an NFM voice passband (or SSB) and reuses the
WSJT FST4 LDPC mother code. Built end-to-end through a
modulation-pivot mid-cycle (initial 4-GFSK design failed
orthogonality at h=0.5, replaced by single-carrier QPSK + RRC +
m-sequence preamble + pilot phase tracking).

WSJT-family modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65) are
**unchanged** in this release. No breaking API changes.

### Added

- `mfsk-core::uvpacket` module (feature-gated `uvpacket`, off by
  default). Four-mode rate ladder (Robust/Standard/Fast/Express,
  1008–1800 net bps) with kSR-greedy puncture-set selection
  (Ha–McLaughlin) on the `Ldpc240_101` mother code's parity bits.
  Byte-pipe API (`app_type` 4-bit dispatch tag); bypasses the
  generic `MessageCodec` to fit non-WSJT use cases.
- TX (`uvpacket::tx::encode`): 31-bit BPSK m-sequence preamble
  → QPSK Gray-mapped data + pilots every 32 sym → RRC pulse
  shaping (α=0.5, span 6) → upconvert to 1500 Hz audio centre at
  12 kHz sample rate.
- RX (`uvpacket::rx::decode_known_layout` /
  `decode_known_layout_with_opts` / `decode`): 2× downconvert →
  matched filter → preamble correlation with parabolic sub-sample
  timing recovery → weighted LMS quadratic phase fit over all
  anchors (preamble centre + pilots) → magnitude-based σ²_n
  estimator from data symbols → σ-aware QPSK soft demap →
  per-LDPC-block decision-directed phase correction → BP+OSD-2
  decode (override via `&FecOpts` for OSD-3 etc.).
- AWGN + Rayleigh-flat-fading harness in
  `tests/common/channel.rs`. `awgn_sigma_for_eb_n0_info` now
  takes per-burst measured `signal_power` for cross-modulation-
  comparable Eb/N0_info numbers.
- Diagnostic test suites: `tests/uvpacket_ldpc_direct.rs`
  (modem-bypassed LDPC threshold sweep) and
  `tests/uvpacket_modem_diag.rs` (TX power audit, rx estimator
  audit, demod-only BER vs theory sweep).
- `mfsk-core/examples/uvpacket_samples.rs` — generates
  representative WAV files at `audio_samples/uvpacket/`
  (clean / +8 / +4 / +2 dB AWGN / 5 Hz Rayleigh / Express clean).
- `docs/UVPACKET.md` + `docs/UVPACKET.ja.md` — design narrative,
  characterisation tables (LDPC ceiling vs end-to-end), SNR
  calibration history, FM-threshold-margin analysis, SSB
  compatibility note.
- `docs/RELEASE_NOTES_0.3.1.md`.

### Characterisation

All numbers are Eb/N0 per **information bit** (WSJT cross-mode-
fair convention).

- **AWGN, 50 % PER**: Robust +1 dB, Standard / Fast +2 dB,
  Express +3 dB. 100 % PER at +4 dB across all modes.
- **AWGN, LDPC-only ceiling** (modem-bypassed): Robust 50 % PER
  at +0.5 dB, Express at +1.5 dB. End-to-end gap (modem
  implementation loss): 0.5–2 dB depending on mode.
- **Rayleigh, ≥ 90 % PER** (4-block, 20-byte payload): Robust at
  +10 dB / 5–10 Hz Doppler, +12 dB at 1 Hz; the higher-rate modes
  mostly +10 dB across.
- **Operating envelope**: Robust at −3.7 dB SNR_3kHz vs the NFM
  FM-threshold floor at ~+20 dB SNR_3kHz → ~24 dB margin. The
  channel CNR floor binds before the modem on NFM. On SSB the
  modem operates to its true threshold.

### Known limitations

- No automatic frequency control yet. SSB use requires both ends
  to agree on `audio_centre_hz` to within ~10 Hz. AFC is planned
  for a follow-up cycle.
- `Protocol::ID = ProtocolId::UvPacket` and several
  `ModulationParams` trait constants are decorative for uvpacket
  (the module bypasses the generic mfsk-core TX/RX pipeline).
  Documented as a scope-boundary trade-off in
  `mfsk-core/src/uvpacket/protocol.rs` and `docs/LIBRARY.md`
  §10.1 rather than spinning uvpacket out as a sibling crate.

## 0.3.0 — 2026-04-29

Internal cleanup release that closes long-standing abstraction
leaks in the FEC / message / pipeline layers, opens the door for
non-Wsjt77 message codecs, and lifts `coarse_sync` to handle
multi-frame chained signals at the same audio centre.

The 0.3.0-cycle uvpacket protocol prototype was developed and
abandoned within this cycle (an honest airtime comparison vs.
AFSK 1200 / AX.25 invalidated the original "drop-in" pitch); the
redesign for 0.3.1 is captured in `docs/0.3.1_PLAN.md`.

### Added

- `fec::ldpc::params::LdpcParams` sealed trait + generic
  `bp_decode_generic<P>` / `osd_decode_generic<P>` /
  `ldpc_encode_generic<P>`. Both `Ldpc174_91` and `Ldpc240_101`
  collapse onto the same algorithm code; ~600 lines of duplicate
  BP / OSD in `fec/ldpc240_101/{bp,osd}.rs` deleted.
- `MessageCodec::verify_info(&[u8]) -> bool` — message-level
  integrity verification hook. `Wsjt77Message` overrides to
  delegate to `check_crc14` / `check_crc24` (length-dispatched
  between K=91 and K=101). Future codecs with bespoke or no
  integrity field can opt out by overriding the default
  unconditional accept.
- `WsjtApCompatible` sealed marker trait on the AP module —
  `process_candidate_ap` / `decode_sniper_ap` / `ap_bits_for` only
  accept message codecs whose 77-bit field matches the Wsjt77
  layout. Codecs with different layouts (e.g. byte-oriented
  packet codecs) fail to compile against the AP path, surfacing
  the constraint at the type level instead of as a runtime panic.
- `PacketBytesMessage` byte-payload `MessageCodec` worked example
  (4-bit length + 80-bit payload + 7-bit CRC-7 in 91 info bits —
  the K of `Ldpc174_91`). Demonstrates that the trait
  accommodates byte-oriented protocols alongside the WSJT-77
  callsign-packing flavour. Gated on the new `packet-bytes`
  Cargo feature; not used by any wired protocol in 0.3.0.

### Changed

- `core::sync::coarse_sync` now emits multiple Costas peaks per
  frequency bin via greedy non-maximum suppression with ±MLAG
  spacing (cap 8 / bin). Strict superset of the previous
  one-or-two-peaks-per-bin behaviour: slot-based protocols
  (FT8/FT4/WSPR/JT9/JT65/Q65) keep byte-identical output because
  the second-best lag falls below `sync_min` after the
  noise-floor normalisation. Chained-frame protocols (multiple
  frames at the same audio centre, separated only in time) gain
  multi-frame discovery in a single pipeline pass.
- `pipeline::encode_tones_for_snr` drops its local `crc14` /
  `crc24` reconstruction and feeds `FecResult.info` straight back
  into `fec.encode`. The verifier-acceptance invariant guarantees
  this is bit-identical to the previous "extract msg77, recompute
  CRC, encode" path. Same simplification in
  `pipeline_ap::finalise_result`.
- `DecodeResult.message77: [u8; 77]` becomes `info: Box<[u8]>`
  carrying the FEC's full K information bits. The legacy 77-bit
  field survives as `DecodeResult::message77()` accessor for
  Wsjt77-family ergonomics that mfsk-ffi and the FT4 / FST4
  doctests rely on.
- `Q65Codec::decode` becomes CRC-agnostic: the trailing CRC-12
  check moves up to `Q65Message::verify_info`, mirroring the
  same shape as the LDPC families. `Q65DecodeError::CrcMismatch`
  is retained as a type but no longer produced internally.

### Removed

- The 0.3.0-cycle uvpacket protocol prototype (UvPacket150 / 300 /
  600 / 1200, the `mfsk-core/src/uvpacket/` module, the
  `docs/UVPACKET.{md,ja.md}` deep-dive, the
  `tests/uvpacket_roundtrip.rs` integration test). The
  redesign is in `docs/0.3.1_PLAN.md`.
- `ProtocolId::UvPacket` enum variant (will be reintroduced in
  0.3.1 with the new sub-mode tags).
- The `uvpacket` Cargo feature (its byte-codec content moved to
  the new `packet-bytes` feature).

### Internal

- `LIBRARY.{md,ja.md}` §11 motivating-example section reverted
  with the rest of the prototype.

## 0.2.1 — 2026-04-26

Patch release with no code changes — README hot-fix only.

The 0.2.0 README's `docs/LIBRARY.{md,ja.md}` links resolved to
`github.com/jl1nie/mfsk-core/blob/HEAD/mfsk-core/docs/LIBRARY.*`
when crates.io rendered the README, which is 404 because `docs/`
lives at the workspace root (the crate's
`readme = "../README.md"` pulls the workspace README in). Switched
both links to absolute `https://github.com/.../blob/main/docs/...`
URLs so they resolve from both crates.io and direct GitHub viewing.

## 0.2.0 — 2026-04-26

The Q65 wave: complete the WSJT-X Q65 family (terrestrial Q65-30A
plus EME Q65-60A‥E), expose all four Q65 decoder strategies through
the C ABI, and validate the trait surface end-to-end with a generic
checker plus a runtime registry. ~330 tests across the workspace
(up from ~230 at 0.1.0).

### Added — Q65 weak-signal decoder family complete

- `fec::qra::fast_fading` + `fading_tables` modules port
  `q65_intrinsics_fastfading`, `q65_esnodb_fastfading`,
  `fadengauss.c` and `fadenlorentz.c` from WSJT-X. Decodes the
  10 GHz EME reference recording (60D, VK7MO ↔ K6QPV) where the
  AWGN Bessel front end fails.
- `q65::ap_list::standard_qso_codewords` + `Q65Codec::decode_with_codeword_list`
  port `q65_decode_fullaplist` and `q65_set_list.f90` (the WSJT-X
  206-codeword "full AP list"). At SNR −25 dB (1 dB below the
  published Q65-30A threshold), AP-list decodes 6/6 frames where
  plain BP fails 0/6.
- New entry-point families in `q65::rx`, generic over the sub-mode
  ZST: `decode_at_fading_for<P>` / `decode_scan_fading_for<P>` and
  `decode_at_with_ap_list_for<P>` / `decode_scan_with_ap_list_for<P>`.

### Added — Q65 reaches C/C++/Kotlin via `mfsk-ffi`

- New `MfskProtocol::Q65a30 = 6` enum variant routes Q65-30A through
  the generic-handle path (`mfsk_decoder_new` + `mfsk_decode_f32`).
- Dedicated `mfsk_q65_decode{,_with_ap,_fading,_with_ap_list}`
  function family takes a `MfskQ65SubMode` parameter
  (`A30 / A60 / B60 / C60 / D60 / E60`) and reaches every sub-mode
  with every decoder strategy. New `MfskQ65FadingModel` enum
  (`Gaussian / Lorentzian`) for the fast-fading entry point.
- `mfsk_encode_q65` synthesises any sub-mode from
  `(call1, call2, grid_or_report)`.
- `mfsk-ffi` remains `publish = false` — consumers clone the
  workspace and `cargo build -p mfsk-ffi`.

### Added — Trait surface verified end-to-end

- New `mfsk_core::PROTOCOLS` static + `ProtocolMeta` struct +
  `by_id` / `by_name` / `for_protocol_id` lookup helpers
  (`mfsk-core/src/registry.rs`). Lets UI layers and FFI bridges
  enumerate the wired protocols at runtime; all six Q65 sub-modes
  appear as distinct entries (different NSPS / tone spacing) sharing
  `ProtocolId::Q65`.
- New `tests/protocol_invariants.rs` runs a single generic
  `assert_protocol_invariants::<P: Protocol>` against every wired
  ZST (FT8, FT4, FST4, WSPR, JT9, JT65, plus all six Q65 sub-modes
  — 11 in total) checking 17 trait-level invariants per ZST.
  Cross-checks every `PROTOCOLS` entry against its ZST through a
  separate code path so registry typos are caught.

### Changed

- `ModulationParams::GRAY_MAP` doc contract loosened from
  `len() == NTONES` to `len() ∈ [2^BITS_PER_SYMBOL, NTONES]` to
  match the actual range across protocols (JT9 trims its map to
  the 8 data tones; JT65 / Q65 extend with identity over the sync
  slots). Surfaced by the new invariants test.
- README + `docs/LIBRARY.{md,ja.md}` extended with new sections on
  Q65 decoder-strategy selection (when to use AWGN vs AP vs
  fast-fading vs AP-list) and on the runtime registry / invariants
  test.

### CI

- Heavy synthetic SNR / AP / fast-fading sweeps gated with
  `#[ignore = "slow: ..."]`; local `cargo test` skips them in
  debug mode (10+ min → seconds), CI runs them in release mode via
  `--include-ignored` (~10 s total).

## 0.1.0 — 2026-04-19

Initial release. Consolidates nine previously-separate workspace
crates from the `jl1nie/webft8` project into a single `mfsk-core`
crate with feature-gated protocol modules:

- `mfsk-core`, `mfsk-fec`, `mfsk-msg` → `core`, `fec`, `msg` modules
- `ft8-core`, `ft4-core`, `fst4-core`, `wspr-core`, `jt9-core`,
  `jt65-core` → per-protocol modules behind features of the same
  name

Features shipped at 0.1.0:

- FT8 (15 s, 8-GFSK, LDPC(174, 91))
- FT4 (7.5 s, 4-GFSK, LDPC(174, 91))
- FST4-60A (60 s, 4-GFSK, LDPC(240, 101))
- WSPR (120 s, 4-FSK, convolutional r=½ K=32 + Fano, incl. Type 1/2/3)
- JT9 (60 s, 9-FSK, convolutional r=½ K=32 + Fano)
- JT65 (60 s, 65-FSK, RS(63, 12) GF(2⁶), incl. erasure-aware decode)

Companion (not published): `mfsk-ffi` sibling crate exposing a
C ABI + `mfsk.h` header via cbindgen, with C++ driver and Kotlin
JNI example scaffolds.

Algorithms derived from WSJT-X (K1JT et al.); each source file
cites the corresponding upstream file.
