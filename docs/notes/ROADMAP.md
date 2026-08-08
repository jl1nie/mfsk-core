# Roadmap (post-0.9.0)

## Strategic state (2026-08-07)

Three tracks, at very different maturities:

1. **Host DSP / protocol — mature, effectively maintenance.** All eight
   protocol families sit at or near WSJT-X sensitivity parity. The one
   disclosed exception, JT65 vs. `ftrsdap`, was narrowed (not fully
   closed) 2026-08-08: the stochastic Chase decoder port
   (`jt65::chase::decode_at_with_chase`, #169) cut the gap from ~7-8 dB
   to ~2-3 dB at deep SNR — see `docs/notes/BENCHMARKS.md`'s JT65
   section for the measured before/after table. Golden lockdowns and
   the FST4 / FT4 / Q65 sensitivity closes all landed across the
   0.6.x–0.7.x line. What's still open here (#143 / #193 FST4 AP+SIC,
   #192 FT8 engine unification, #224 JT4, #148 research, and the
   residual ~2-3 dB JT65 gap if #169 is reopened) is **tail work** —
   calibration, behaviour-preserving refactor, or low-demand modes — not
   a frontier. Advances here should be demand-driven (e.g. VK3NV's
   FST4-15/30 use case behind #143), not pursued for their own sake.

2. **Host application / ergonomics — newly opened, consumer-driven.**
   0.9.0's theme ("make the streaming decode surface easy to build host
   UIs on") added `msg::decoded::Decoded`, per-protocol `to_decoded`, the
   `serde` feature, and completed streaming `.on_result` /
   `decode_scan_streaming` across all protocols. `session::SlotAssembler`
   (audio ingestion) is written and **parked pending a real consumer**.
   This track only advances when an actual host UI (WebFT8, a desktop
   app) needs the next piece — building further ahead of a consumer
   risks fixing an API shape nothing has validated.

3. **Embedded controller (Phase B-Core) — the real frontier, stalled.**
   The main production target (M5Stack CoreS3 UAC FT8 controller) has its
   crate skeleton and UAC host code shipped and compiling clean, but
   **#163 — live IC-705 hardware verification — has never been done**,
   and no cores3-app feature commit has landed in ~2 months. The whole
   downstream sequence (shared UAC hoist → BLE CI-V → ADIF → touch UI →
   TX keying) is blocked behind that single verification step. #163 is a
   human-at-the-bench task, which is why it hasn't moved on its own.

The open strategic question this doc deliberately does **not** decide
(it's the maintainer's call, not to be inferred from momentum): whether
the next cycle's centre of gravity is **finishing the embedded product**
(drive #163 through and unblock Phase B-Core) or **serving the library's
host-UI consumers** (extend track 2). The two aren't exclusive, but
attention is.

## Current line — 0.8.x shipped, 0.9.0 accumulating

- **0.8.0** — legacy BASIS `fill_symbol_spectra_into` path removed
  (#162; a breaking FFI change to `mfsk_ft8_decode_i16`'s signature),
  bundled with the accumulated 0.7.5 content rather than cutting 0.7.5
  separately: JT65 interleave TX/RX-convention fix (#24), the JT9 AWGN
  sweep, and the new Q65 sub-modes.
- **0.8.1** — decode-side `snr_db` added to `WsprResult`, `Jt65Result`,
  `Jt9Result`, `Q65Result` (#226; breaking — a new required field on
  four public types). Closes the gap where only the FT8-family shared
  `DecodeResult` and MSK144 carried an SNR estimate; `mfsk-ffi`'s
  `push_simple` had been hardcoding `0.0` for the other four.
- **0.9.0 (unreleased) — streaming ergonomics for host UIs.**
  `msg::decoded::Decoded` (a unified, owned, `Send` decode row for host
  UIs) + a `to_decoded(..)` conversion on every protocol's native result
  type; the `serde` feature (off by default, `no_std`-clean) deriving
  Serialize/Deserialize on `Decoded` + `ProtocolId`; streaming
  `.on_result` / `decode_scan_streaming` now complete across all
  protocols, documented in `docs/reference/STREAMING.md` (+ `.ja`).
  Minor bump per this crate's "new cross-cutting public API surface =
  minor" convention (cf. 0.7.0's generic-API landing); the actual
  `v0.9.0` tag is cut on the next biweekly cadence slot, not
  opportunistically.
  - **Parked, not in 0.9.0**: `session::SlotAssembler` (streaming audio
    ingestion — resample-to-12k + slot windowing + sample-counted slot
    timing). Code-complete and tested on branch
    `claude/streaming-interface-docs-vuet32`, held unmerged until a real
    consumer (a desktop UI, or a refactor of the embedded `audio.rs`
    slot statics onto it) validates the `push`/`poll`/`mark_slot_start`
    API shape. See track 2 above.

## Prior line (0.7.x) — shipped through 0.7.4 (2026-07-19)

Numeric recall/sensitivity figures for every protocol are **not**
duplicated here — `README.md`'s "What's solid" section is the
canonical, kept-current source (duplicating numbers in two docs is
exactly how this file's FT8 figures drifted out of sync with reality
for two months; see `CHANGELOG.md` for the full per-release detail).
This section is just a byline of what shipped since the last time
this doc was substantially updated (0.6.5, 2026-05-18):

- **0.7.0** — FST4 gained its remaining four sub-modes
  (FST4-15/30/120/300, via the `fst4_submode!` macro) and
  `coarse_sync` was parallelised under `--features parallel`.
- **0.7.1 / 0.7.2** — FST4 AWGN sensitivity gap vs. WSJT-X's
  published thresholds closed from ~2.4-3.1 dB down to a common
  ≈0.1-0.6 dB across all five sub-modes (issue #146 — coherent
  full-slot sync, an FST4-specific `LLR_NSYM_MAX` override, a
  zsum-OSD fallback). See `docs/notes/FST4_BENCHMARK.md` for how to
  reproduce the sweep.
- **0.7.3** — FT4 AWGN sensitivity gap narrowed ~1.8 dB → ~0.3 dB
  (coherent Costas-block scorer + an OSD-gate fix); FT8 CCIR
  moderate/poor fading recall gap closed by widening
  `OSD_HARDERRORS_MAX` back to WSJT-X's universal 36, which also
  resolved issue #150's JTDX-18 ground-truth question as a side
  effect.
- **0.7.4** — MSK144 meteor-scatter decode shipped (issue #25): full
  WSJT-X port (LDPC(128,90)+CRC-13, MSK/OQPSK DSP, burst-scan sync),
  3/3 golden-WAV recall. A systematic -1 dB SNR bias found
  post-ship was root-caused (missing fixed bandpass filter in the
  analytic-signal front end) and fixed, then cross-validated
  against a real WSJT-X `jt9` build (25/28 cells exact match — see
  `docs/notes/MSK144_BENCHMARK.md`). Also established this repo's
  release cadence policy: PRs land on `main` immediately, but
  crates.io tags are cut **biweekly** rather than per-change (escape
  hatch for security/correctness-critical fixes or explicit
  request) — see `CLAUDE.md`'s "Release cadence" section.

Embedded-line status (M5StickS3 / CoreS3 / Core2) has its own
detailed writeup under **Phase B** below — the short version: Phase
B-Core (CoreS3, the main production target) has its crate skeleton
and UAC host support shipped and compiling clean, but has never been
verified against live hardware (issue #163); Phase B-Stick
(M5StickS3, demo/fallback) and the Core2 sibling are both frozen,
unchanged since late May.

### What landed in 0.6.5 (2026-05-18)

Non-functional crates.io / docs.rs surface refresh — no source code
modifications, no new features, no bug fixes vs 0.6.4. The embedded
port is no longer aspirational, and the discoverable surface had to
catch up:

- **`mfsk-core/Cargo.toml` `description`** rewritten to lead with
  the working `embedded-poc/m5stack-s3-app` M5StickS3 FT8 controller
  (LCD UI, BLE CI-V to IC-705, acoustic mic, QSO FSM, ~1.2 s
  post-SlotEnd decode on Xtensa LX7).
- **`mfsk-ffi-ft8/Cargo.toml` `description`** notes M5StickS3
  end-to-end verification.
- **`mfsk-core/src/lib.rs` "Why this exists"** gains a fourth
  bullet for the handheld-controller use case, with cross-links to
  the M5StickS3 source crate and `docs/reference/MANUAL_M5STICKS3.md`.
- **`README.md`** opens with a hero photo of the device decoding
  five real on-air FT8 callsigns from a single 15 s slot
  (`docs/assets/m5sticks3-ft8-decode.jpg`).

### What landed in 0.6.4 (2026-05-18)

- **Phase 1.7.7-Stick Goertzel migration** (PR #104, commit `5501a2f`).
  The per-symbol DFT in `fill_symbol_spectra` switched from the
  precomputed Q15 `BASIS` table + esp-dsp ASM `dsps_dotprod_s16_ae32`
  dot product to a generalised Goertzel recursion
  (`fill_symbol_spectra_goertzel`) — 2-tap IIR per (sym, tone) with
  3 f32 state values on the stack, zero scratch, zero extern symbols.
  Sample-outer / tone-inner loop interleave lets LLVM unroll the
  constant `NTONES=8` inner loop so 8 independent dependent chains
  run through the FPU pipeline in parallel; result is stage-3
  wall-clock equivalent to BASIS (~1.4 s on S3 `qso3_busy.wav`) with
  **+0.16..+0.63 dB SNR improvement** (f32 Goertzel beats Q15 BASIS
  precision).
- **`fixed-point` ⇒ `nstep-half` coupling** (commit `dfc0cd5`).
  The two features were always co-enabled on every embedded target
  in practice; decoupling silently gave host fixed-point builds a
  different NSTEP grid than the embedded path, making host validation
  diverge from real-silicon behaviour. Now `fixed-point` implies
  `nstep-half`.
- **mag² saturation fix** in `compute_spectrogram` (same commit).
- **Internal-DRAM saving**: ~60 KB BASIS scratch × 2 (re/im) × 2
  cores = **120 KB freed** on M5StickS3, which is exactly what the
  Qso-mode bidirectional I2S DMA descriptor needs. Qso mode now
  boots cleanly on first try.

The legacy BASIS code path (`fill_symbol_spectra_into` and the
`mfsk_core_dot_q15_i32` extern symbol) and the `basis_re`/`basis_im`
scratch arguments on `mfsk_ft8_decode_i16` remain for back-compat
with 0.5.x callers. **Still present as of 0.7.4** — this was
originally slated for 0.7.0 removal, which never happened (0.7.0's
actual scope was FST4 sub-modes + `coarse_sync` parallelisation,
unrelated). Now tracked as [#162](https://github.com/jl1nie/mfsk-core/issues/162)
rather than re-promising a version number here.

### What landed in 0.6.3 (2026-05-17)

- **WSJT-X-faithful OSD precoding** (#63, PRs #86 + #87 + #88).
  `npre1` (ndeep=2) + `npre2` (ndeep=3) dispatch + 4 missing
  post-OSD validity gates from `ft8b.f90:422-459` + nsync/xsnr
  clamp-ordering fix, plus an empirically-tuned
  `OSD_HARDERRORS_MAX = 22` ceiling on the OSD path
  (mfsk-core-specific deviation from WSJT-X's universal 36) that
  eliminates the qso3_busy phantoms `npre1`/`npre2` alone did not
  filter. See `CHANGELOG.md` 0.6.3 entry for the
  WSJT-X-faithfulness / phantom-elimination trade-off rationale.
- **ε `decode_block.rs` restructure** — 6 stacked PRs (#77 types,
  #83 spectrogram, #79 coarse_sync, #80 fill_symbol_spectra, #81
  process_candidates, #82 osd_strategy). 3,517 lines → 423-line
  parent + 6 stage submodules. The OSD-strategy seam ε.6 carved
  out is load-bearing for #63's WSJT-X-faithful `npre1`/`npre2`
  dispatch.
- **#61 Core2 fold-in** (PR #76) — `mfsk-app-shared` carve-out +
  `m5stack-core2-app` sibling + retirement of the standalone Core2
  bench. Both production apps now share board-agnostic QSO FSM /
  UI / WiFi / NVS / log fanout.

### What landed across 0.6.0 → 0.6.2

- **0.6.0**: FT8 sync consolidation (`decode_block::coarse_sync` is
  canonical, `ft8::sync::coarse_sync` removed); `#40` host
  wide-band coarse-sync gap closed via `i_start as i32` + WSJT-X
  all-or-nothing boundary; `#46` / `#48 step B` / `#49 cat A-C`.
- **0.6.1**: per-candidate inner unification —
  `process_one_candidate_inner` is shared between the host
  `process_candidate` body and `decode_block::run`. New public
  `decode_block_with_ap[_tuned]` entries; AP iaptype loop reachable
  from embedded as Step 4 of the inner. Embedded OSD pass IDs
  migrated 4-7 → 14-17 to free 5..12 for AP.
- **0.6.2**: host cs-source unified onto `fill_symbol_spectra`
  (drops the cd0 + `ft8::llr::symbol_spectra` divergence); host SIC
  switched to `subtract_signal_lpf`; dead-code removal
  (`subtract_signal_weighted`, `subtract_signal`,
  `qsb_partial_gain`, `ft8::llr::symbol_spectra`); embedded
  `LlrT` Q3i8 → **Q11i16** (i8 → i16, ~16× LLR resolution; ship
  recall +1 entry, post-SlotEnd 1.341 s → 1.191 s); fixed-point
  build repair (broken since 0.6.1 by an internal call-site rename).

### Embedded fine_refine attempt postmortem (deferred)

Two paths were tried during 0.6.2 to lift embedded recall toward
the JTDX-extra band:

1. **Per-symbol DFT iteration** via `fill_symbol_spectra` per
   candidate — projected ~100 ms, actually >5 s blocked compute
   (200k+ × 1920-sample DFTs per slot), tripping the FreeRTOS Task
   Watchdog.
2. **`cd0` complex-baseband via cascaded esp-dsp FIR decimate**
   (3:1 → 4:1 → 5:1 = 60:1) — solved the compute side but hit a
   PSRAM bandwidth ceiling (~80 MB/s on S3 OCT mode means ~1 s
   just for memory traffic for 15 candidates; further alignment
   requirements, scratch fragmentation, lazy-alloc TLSF corruption
   were all incidental).

Honest read: embedded recall stays at **7/18** until either an
i16-throughout FIR refine path or a streaming chunk-processing
refactor lands. Both are out of scope for a 0.6.x patch — defer
to a 0.7.x design pass.

### Open follow-ups

Currently open GitHub issues (state:open as of 2026-08-07, verified
directly against the GitHub API — this is the live worklist; if you're
reading this file to decide what to work on next, trust this section
over any recall numbers or hardware status stated elsewhere in it).
Grouped by the three tracks in **Strategic state** above.

**Embedded (frontier):**

- **#163** — CoreS3 Phase 1-Verify: live IC-705 hardware RX
  confirmation. **The bottleneck for the entire Phase B-Core line** —
  the UAC host code compiles clean but has never run against real
  hardware, and there's been no cores3-app feature commit since
  2026-06-07. Everything downstream (Phase 1.5 / 2 / 5 / 6 / 7-Core) is
  sequenced behind it. A human-at-the-bench task (1500 Hz tone injection
  → PCM reaches the pipeline → FT8 candidate in the decoded list → live
  antenna). See **Phase B-Core** below.

**Host DSP / protocol (maturity — tail, not frontier):**

- **#143** — FST4 AP decode + SIC for FST4-15/30. Real user demand
  (VK3NV's real-time weak-signal messaging); the building blocks all
  exist (`msg/ap.rs`, `msg/pipeline_ap.rs`, `core/dsp/subtract.rs`),
  missing only the FST4 wiring + per-sub-mode `SubtractCfg` calibration.
  Low priority until crowded-band use materialises; active design
  discussion in-issue.
- **#193** — FST4 has no SIC path (no `SubtractCfg`, no
  `decode_frame_subtract`). The SIC half of #143; needs numerical
  calibration against WSJT-X's FST4 subtract path, after which
  `impl SupportsFlatSic for Fst4s60 {}` (+ siblings) is trivial.
- **#192** — FT8 decode engine never unified with `engine::pipeline`
  (`fine_refine_3stage` needs `<P>` generalisation). Behaviour-preserving
  refactor; not urgent, not blocked by anything. Requires
  numerical-diff-from-reference rigor so a careless port doesn't regress
  FT4/FST4's independently-calibrated OSD gates.
- **#169** — JT65 gap vs. WSJT-X's `ftrsdap` stochastic Chase decoder.
  Root-caused (the old `decode_at_with_erasures` tries a single
  deterministic erasure ordering; `ftrsdap` runs randomized soft-symbol
  trials using the 2nd-most-reliable symbol too). **Narrowed
  2026-08-08**: ported the algorithmic shape (not WSJT-X's literal
  magic numbers) as an additive opt-in, `jt65::chase::decode_at_with_chase`
  / `decode_scan_chase*`. Measured on the same AWGN corpus: 50%
  crossing moved −14 dB → ≈−19.5 dB, closing ~5 dB of the ~7-8 dB gap;
  ~2-3 dB remains at the deepest cells (−20/−22 dB), plausibly from
  WSJT-X's literal spectral-power candidate ranking (`pp`, deliberately
  not ported — see `chase`'s module doc) and/or its much larger trial
  budgets. See `docs/notes/BENCHMARKS.md`'s JT65 section for the full
  before/after table. Remaining ~2-3 dB left open, same
  demand-driven bar as before — Q65 still covers JT65's deep-SNR niche
  for most on-air use.
- **#224** — JT4 not implemented (WSJT-X ships JT4A/JT4F golden WAVs).
  "Doable but demand unclear" — every usage signal found was WSJT-X
  boilerplate, not dated on-air data, and Q65 has partly superseded its
  role. Track, don't commit.
- **#148** — Research idea (not a commitment, from VK3NV): blind-paired
  FST4-120 with soft combining, as a Doppler-robust FST4-300 alternative.
  Q65's multi-period averaging (`q65/rx.rs`) is the architectural
  precedent.

**Host application / ergonomics (emerging, consumer-driven):**

- *No open issue yet.* 0.9.0 shipped the `Decoded` row + streaming
  surface; `session::SlotAssembler` is parked on branch
  `claude/streaming-interface-docs-vuet32` pending a real consumer
  (desktop UI, or the embedded `audio.rs` slot-statics replacement) to
  validate its shape before landing. File an issue if/when a reference
  host UI is committed to.

**Non-code decision:**

- **#125** — License (GPLv3 vs. a permissive license for
  broader/proprietary adoption). Needs a decision, not code.

Recently closed since the 2026-07-19 snapshot (see the closed issue /
`git log` for fix commits): **#24** — JT65 interleave TX/RX-convention
bug, fixed, shipped in 0.8.0; **#162** — legacy BASIS
`fill_symbol_spectra_into` path removed, done, shipped in 0.8.0 (a
breaking `mfsk_ft8_decode_i16` FFI change).

Closed since the 2026-05-18 snapshot (see the closed issue / `git
log` for the fix commit, not re-derived here):

- ~~**#25**~~ — MSK144 decode path (0.7.4).
- ~~**#58**~~ — coalesce redundant `compute_llr` (OSD/AP steps).
- ~~**#64**~~ — hoist `fft_cache` through host `decode_block_multipass`.
- ~~**#65**~~ — share `cd0` between SyncOnly + DataOnly `fill_symbol_spectra`.
- ~~**#73**~~ — `EqMode::Adaptive` collapsed into `EqMode::Local`.
- ~~**#74**~~ — `DecodeDepth::Bp` cheapest-rung caller check.
- ~~**#110**~~ — qso(FSM) slot parity tracking (TX/peer-decode collision).
- ~~**#113**~~ — CI: skip Build/Test/C++ matrix on docs-only PRs.
- ~~**#116**~~ — FT8: classify JTDX 5/18 misses on `qso3_busy.wav`.
- ~~**#117**~~ — FT8: auto-AP iaptype-2 from same-slot decoded callsigns.
- ~~**#146**~~ — FST4 AWGN sensitivity gap vs. WSJT-X's published table
  (0.7.1/0.7.2 close-out, see *Current line* above).
- ~~**#147**~~ — docs: `CHANGELOG.md` size + docs/ reader-facing vs.
  internal-notes split.
- ~~**#150**~~ — FT8 JTDX-only extra decodes ground-truth question
  (resolved as a side effect of 0.7.3's `OSD_HARDERRORS_MAX` widening).
- ~~**#156**~~ — MSK144 SNR sensitivity verification against real
  WSJT-X (0.7.4, see *Current line* above).
- ~~**#72**~~ — `DecodeStrictness` duplicate definition + uncalibrated
  copy for FT4/FST4. Turned out almost entirely resolved already
  (2026-07-18, before this closure): FT4's copy was retuned against
  a new `ft4sim`-based sweep (`core/pipeline.rs`'s `DecodeStrictness`,
  see `docs/notes/FT4_BENCHMARK.md` section 6), and FST4 bypasses
  these gates entirely per #146's fix — so the two enums now hold
  independently-calibrated, not-actually-duplicate values. Closed
  the residual "should the type itself be unified" question as no
  longer worth it (would need a protocol discriminator, net
  complexity increase).
- ~~**#171**~~ — Q65 AWGN sensitivity gap, root-caused and closed
  2026-07-19 (same day it was found). Two contributing causes, both
  resolved: (1) `coarse_search_for`'s reported best alignment is off
  by up to ~1/5 of a symbol period at low SNR with nothing refining it
  before decode — fixed with a local fine-timing retry
  (`decode_at_with_fine_timing_for`, mirrors WSJT-X's `q65_loops` `idt`
  loop); (2) the remaining gap after that fix was a comparison
  artifact, not a bug — WSJT-X's default `jt9` decode always has
  access to a free "CQ ??? ???" AP hypothesis that `decode_scan_for`
  (this crate's genuinely blind baseline) doesn't, by design.
  `decode_scan_with_ap_for` + a `"CQ"` hint (now a second column in
  `tests/q65_sim_sweep.rs`) matches WSJT-X's numbers almost exactly
  across all six wired sub-modes. FEC/BP stack was verified
  byte-for-byte correct against WSJT-X before either finding (10 code
  tables + WHT + every `pdmath` primitive, programmatic diff, zero
  discrepancies). Usage note for future sessions: applications wanting
  WSJT-X-equivalent behavior for CQ traffic should default to the
  AP-hinted path with at least a `"CQ"` hint, not the plain one.

Closed during the 0.6.x line (compact context table — for full
prose, see the closed issue / linked PR / `git log`):

| ref | summary | closed in |
|---|---|---|
| #40 | host wide-band coarse-sync candidate gap (AP-off recall 5/8 → 7/8) | v0.6.0 |
| #46 | sync consolidation — `decode_block::coarse_sync` + `compute_spectrogram` public | v0.6.0 |
| A0' | `decode_block_with_ap` as Step 4 of `process_one_candidate_inner` | v0.6.1 |
| #61 | fold `m5stack-core2` into S3 dual-core; retire Core2 bench (PR #76) | 0.6.3 |
| #63 | WSJT-X-faithful OSD `npre1`/`npre2` precoding (see "What landed in 0.6.3") | 0.6.3 |
| #105 | EMBEDDED.md rewrite (kept deep tech ref, refreshed for 0.6.4) | 0.6.4 |
| #23 | FST4-60A golden lockdown — wrong `NSPS`/`NDOWN`/`GFSK_BT` + missing `rvec` message scramble (PR #136) | 0.6.8 |
| #23 | FST4-15/30/120/300 sub-modes wired (`fst4_submode!` macro, WSJT-X-cross-checked; no golden WAV available — synth-roundtrip + source-verification only) | 0.7.0 |

(Note: `#23` appears twice — GitHub issue numbers aren't reused; this
issue was reopened/re-closed across two separate pieces of work,
FST4-60A golden lockdown then the remaining sub-modes.)

The JTDX AP-on-multipass recall is now 6/6 (was 5/6; see
`BENCHMARKS.md`'s FT8 section for the current number and mechanism).
The former remaining miss, `K1BZM DK8NE`, was closed by issue
[#182](https://github.com/jl1nie/mfsk-core/issues/182) — an
`osd_decode_npre1` LLR-fidelity gap (fed raw channel LLR instead of
BP-refined `bp_llr_zsum`), not the AP-list breadth this note used to
suspect.

Architectural notes that did not graduate to issues:

- **#48 option A** — `Protocol::Sync` associated type for
  type-system enforcement against future protocol-sync drift.
  Scoped out of v0.6.x (8 protocols + 2 macros + embedded feature
  matrix; benefit is speculative for non-FT8 protocols).
- **#49 cat D** — `engine::sync::coarse_sync<P>` `NotFt8` marker
  bound. Subsumed structurally by Phase 4 dispatch in v0.6.0; the
  doc note ("FT8 should not use this") is the practical backstop
  until #48 option A lands.
- **Embedded fine_refine** — design i16-throughout streaming FIR
  cd0 path so `fine_refine_pass1` can run on S3 within slot budget
  without a PSRAM round-trip per candidate. No issue filed yet;
  prerequisite is the ε `decode_block` restructure (see
  `docs/historical/CLEANUP_2026_05.md`) so the seam exists to hook a
  streaming alternative onto.

## Cleanup 2026-05

γ / β / δ / ε all landed in 0.6.3 (last stage PRs #77 + #83 + #79 +
#80 + #81 + #82 carved `decode_block.rs` from 3,517 lines into a
423-line parent + 6 stage submodules). See
[`docs/historical/CLEANUP_2026_05.md`](../historical/CLEANUP_2026_05.md) (HISTORICAL) for the
original four-stage plan; the per-stage acceptance-criteria template
documented there is reused for future cleanup waves.

# Roadmap (legacy, written for post-0.5.12)

Most of the post-0.5.12 plan landed in the 0.6.x bundle (PR #50,
shipped 2026-05-10) or is now tracked under a GitHub issue. The
sections below survive as historical context plus quick file-path
hints; the live worklist is the **Open follow-ups** section above.

## Phase A — Host protocol golden lockdowns

(A0 / A0' both closed in v0.6.x — see the closed-issues table above.)

- **A1** FST4-60A (`#23`) — closed in 0.6.8 (PR #136). Root cause was
  `Fst4s60`'s `NSPS`/`NDOWN`/`GFSK_BT` being never-revisited
  placeholders (wrong vs. `WSJT-X/lib/fst4_decode.f90` /
  `fst4sim.f90` / `gen_fst4wave.f90`) plus a missing `rvec` pre-LDPC
  message scramble; `tests/fst4_wsjtx_samples.rs` is un-ignored and
  recovers the golden decode.
- **A2** JT65 (`#24`) — **scope has moved on from the description
  below** (kept as a historical note; see "Open follow-ups" above
  for the current, accurate scope). Original framing: current
  implementation is JT65A; WSJT-X ships JT65B samples, so add a
  `Jt65b` ZST mirroring the Q65 sub-mode generic pattern. That's no
  longer the plan — JT65B/C are explicitly out of scope per the live
  issue, which is now about porting WSJT-X's `b65a`/`kvasd`
  soft-symbol erasure-metadata pipeline (or a lighter synth-only
  golden as a fallback).
- **A3** FST4-15/30/120/300 (`#23` stretch) — landed via the
  `fst4_submode!` macro (mirrors `q65_submode!`); all constants
  cross-checked against WSJT-X `fst4_decode.f90`/`fst4sim.f90` and
  pinned by an automated `DownsampleCfg`/`GfskCfg`-vs-trait-constant
  test. No golden WAV exists for these four periods (WSJT-X's sample
  tree only ships FST4-60A / FST4W-1800 recordings) — validated by
  synth-roundtrip self-consistency + source cross-checks only;
  requested by VK3NV (issue #23 comments) for a multi-period
  weak-signal messaging project. FST4-900 / FST4-1800 and FST4W
  remain deferred indefinitely (no user demand). **Update**: even
  without a real-audio golden, all five sub-modes now have a
  rigorous AWGN-sensitivity characterization vs. WSJT-X's published
  thresholds (issue #146, closed 0.7.1/0.7.2 — see *Current line*
  above and `docs/notes/FST4_BENCHMARK.md`), so "synth-only" no
  longer means "unquantified."

## Phase B — embedded controller line

**2026-05-17 pivot**: Phase 1 UAC hardware verification confirmed
**M5StickS3 cannot do USB host** (board lacks VBUS source circuit, ID
pin wiring, host power switch IC — silicon supports it, board doesn't
wire for it). Phase B splits in two:

- **Phase B-Stick** — `m5stack-s3-app`, demoted to **demo / acoustic
  fallback** path. Frozen after Phase 1.7.9 (2026-05-27 — see the
  detailed phase list below); Phases 1 (UAC), 2 (BLE CI-V), 5 (ADIF),
  6 (buttons), TX keying all roll forward to Phase B-Core.
- **Phase B-Core** — `m5stack-cores3-app` (NEW crate), the **main
  production controller**. M5Stack CoreS3 has AXP2101 PMIC + AW9523B
  I/O expander (BUS_OUT_EN pin 1 controls VBUS boost for host mode),
  so UAC is viable.

Pattern: Phase B-Core reuses the `mfsk-app-shared` + `embedded-shared`
sibling-crate carve-out proven by `m5stack-core2-app` (Issue #61 Phase
2). See memory `project_m5stick_s3_no_usb_host` for the hardware
diagnosis, and `~/.claude/plans/happy-honking-globe.md` for the pivot
plan.

### Phase B-Stick — m5stack-s3-app (DEMO / FALLBACK)

Per `embedded-poc/m5stack-s3-app/CLAUDE.md` and the Phase 0..6 markers
in module doc-comments:

- **Phase 0 / 0.5 / 3** — Done. LCD bring-up, WAV-fed pipeline,
  4-region UI (status / waterfall / decoded list / TX strip).
- **Phase 4** — QSO FSM dry-run done (auto-CQ visible on LCD,
  `qso.rs` ~360 lines, 8 host-side unit tests). No TX audio
  synthesis on Stick (rolled forward to Phase B-Core).
- **Phase 0.6 / 0.7** — Done. WiFi UDP log streaming (UART /
  LCD / UDP fanout) + USB-CDC freeze fix (`println!` gated on
  `usb_serial_jtag_is_connected`) + runtime boot-mode selector
  (NVS + KEY2 long-press) for WiFi / decoder coexistence.
- **Phase 1 (UAC)** — ❌ **Not viable on Stick hardware**. The UAC
  code that shipped via PRs #29-#35 + #102 (`uac.rs` ~445 lines,
  managed-component bindings, board-agnostic) remains in-tree as
  canonical reference; lifted to `m5stack-cores3-app` in
  Phase 0-Core, then hoisted into `mfsk-app-shared` in
  Phase 1.5-Core after dual-board verification.
- **Phase 1.5 (Acoustic capture)** — ✅ Done (Phase 1.7.1-Stick).
  ES8311 ADC mic-mode → I2S RX → `LinearResamplerI16To12k` →
  `decode_pipeline::run_with_source`. `BootMode::Acoustic` added to
  the NVS cycle; live on `main`.
- **Phase 1.7 (QSO mode bidir I2S + demo toggle)** — ✅ Done. PR
  #121 adds a BtnA-long-press demo mode inside `BootMode::Qso`:
  audio source switches from mic to baked-in `qso3_busy.wav`,
  speaker plays back the WAV, TX scheduler is suppressed, LCD shows
  `DEMO MODE — TX OFF`. Deterministic demo safety net when ambient
  acoustic conditions kill the live WebFT8-to-mic path.
- **Phase 1.7.9 (cold-start auto-sync bootstrap)** — ✅ Done. PR
  #133 / v0.6.6. Auto-sync gains a fourth branch:
  `n_dec == 0 && best_n == 0 && bootstrap_dt_med.is_some()` →
  coarse_sync top-5 DT median drives a one-shot slot shift,
  `best_n` stays at 0 so the next confirmed-decode slot reclaims
  HWM. Removes the "BtnA required" cold-start dead-end for
  no-GPS / no-NTP operation. Helper
  (`mfsk_core::engine::sync::bootstrap_dt_median`) shared with
  WebFT8, gated by `tests/ft8_coarse_sync_bootstrap.rs`.
- **Phases 2 / 5 / 6 / TX keying** — Rolled forward to Phase B-Core.
  Stick frozen after Phase 1.7 demo toggle (1.7.9 is a
  controller-only auto-sync delta — no new HW surface).

### Phase B-Core — m5stack-cores3-app (MAIN TARGET, NEW)

Hardware: **M5Stack CoreS3** (full variant with 500 mAh battery +
GC0308 camera + LTR-553 proximity + BMI270 IMU). CoreS3 SE (cheaper
sensor-stripped variant) deferred until that hardware is procured;
when needed, gated behind `cfg(feature = "cores3_se")` in the same
crate.

**Status (corrected 2026-07-19 — this section previously said
"planned. Brings up after CoreS3 unit arrives", which was stale and
self-contradicted by this file's own Phase D section citing a
CoreS3 benchmark log)**: Phase 0-Core and Phase 1-Core both shipped
**2026-05-23** (PR #132, commit `1a93c92`) — CoreS3 hardware has
been on the bench and flashed since then. What's actually still
open is **Phase 1-Verify**, tracked as
[#163](https://github.com/jl1nie/mfsk-core/issues/163): the UAC code
compiles clean but has never been run against a live IC-705, and
there's been no cores3-app feature commit since 2026-06-07 (6+
weeks). The `(tasks #33/#34)` / `(task #48)` / `(task #49)` /
`(task #50)` / `(task #51)` annotations previously on the phases
below were leftover placeholder numbers that resolved to unrelated,
already-closed May items (CI hygiene, the v0.6.2 release, S3
boot-mode work) — removed below rather than left misleading.

- **Phase 0-Core** — DONE (2026-05-23). Crate skeleton: board.rs
  (ILI9342C SPI pins, I2C0 for AXP2101 + AW9523B + FT6336U, USB-OTG
  fixed GPIO 19/20, ES7210 audio pins), pmic.rs (AXP2101 + AW9523B
  init, distinct from S3-app's M5PM1 and Core2-app's AXP192),
  display.rs (ILI9342C via mipidsi — Core2-app pattern), main.rs
  orchestration, decode_pipeline.rs thin wrapper. First flash
  verified: I2C scan confirmed all peripherals, LCD up, 7/7 decodes
  on `qso3_busy.wav` via wav_sim, 137 ms post_slotend.
- **Phase 1-Core** — DONE (2026-05-23, same day). UAC: `uac.rs`
  cloned from s3-app verbatim; pmic.rs drives **AW9523B P1
  (BUS_OUT_EN) HIGH** before `usb_host_install()` (omission =
  floating VBUS = floating host capability). `BootMode::Uac`
  dispatch arm wired. `cargo check --release` clean on
  `xtensa-esp32s3-espidf` — not yet flashed/run against real
  hardware.
- **Phase 1-Verify** — **OPEN, not yet attempted**
  ([#163](https://github.com/jl1nie/mfsk-core/issues/163)). 1500 Hz
  tone injection from IC-705 → FT8 candidate appears in decoded
  list; then live antenna → end-to-end RX confirmed. This is the
  actual current bottleneck for the whole Phase B-Core line.
- **Phase 1.5-Core** — Hoist `uac.rs` into `mfsk-app-shared` (gated
  `cfg(feature = "uac")`); both s3-app and cores3-app consume via
  shared. Deferred until Phase 1-Verify passes (no premature
  abstraction) — unchanged blocking logic, just correctly blocked on
  a real open issue now instead of a placeholder task number.
- **Phase 2-Core** — BLE CI-V to IC-705. Same `civ.rs` work that
  was queued on s3-app; CoreS3 also has BLE.
- **Phase 5-Core** — ADIF (`flash_log.rs` + `adif.rs`). Crate-agnostic;
  could be hoisted to `mfsk-app-shared` from day 1.
- **Phase 6-Core** — FT6336U capacitive touch driver (CoreS3 base
  has no physical buttons beyond Power). Replaces the Stick
  `buttons.rs` paradigm with touch zones (menu / decoded-list
  tap-to-select / TX-strip tap-to-send).
- **Phase 7-Core** — TX keying (paired with Phase 2-Core
  `civ::set_ptt` + Phase 1-Core TX audio synth to UAC OUT endpoint).

`mfsk-ffi-ft8/src/stream.rs::mfsk_ft8_stream_*` and `embedded-shared`
resampler API are the seams shared between B-Stick and B-Core.

## Phase C — Quality / infra

- **C1** Embedded CI cross-build — still pending. `xtensa-esp32-espidf`
  / `xtensa-esp32s3-espidf` targets only build at release time today.
  The cleanup-2026-05 ε restructure (`decode_block` split) is a soft
  prerequisite so the embedded build does not have to re-validate a
  3500-line file on every PR.
- **C2** Reproducible release builds — `esp-rs/xtensa-toolchain@v1.5`
  is pinned, but the `+esp` Rust version inside the action still
  floats. Pin or capture in the artifact.
- **C3** m5stack-s3-app release artifact — `release.yml` builds
  `libmfsk_ft8.a` only; add a job emitting a flashable
  `espflash save-image` for the s3-app once Phase 1 UAC stabilises.

## Phase D — ESP32-S3 LX7 PIE SIMD acceleration

Pure decoder-side perf push for the S3 critical path (pass 2 +
stage1_inc). Detailed plan in [`docs/notes/PHASE_D_PIE_SIMD.md`](PHASE_D_PIE_SIMD.md).

| Sub-phase | Target | Status |
|---|---|---|
| **D1** | Re-bind esp-dsp FFI to `_aes3_` (LX7 PIE) | **Done** — commit `053bd67`; sc16 PIE 1.98→1.65s (−17%) on S3 sequential bench |
| **D2′** | f32 scalar Goertzel micro-optimisations (scratch hoist, alignment) | **Done** — commit `053bd67`; bounds-check hoisted out of inner loop |
| **D3** | PIE allsum + score for `coarse_sync` / `stage1_inc` | **Done** — commit `0b7978b`; sliding-window allsum |
| **D4** | PIE `\|x\|²` post-FFT in `stage1_inc` | **Done** — commit `a8235e7`; demux-mag² DC-hoist + 4× unroll |

CoreS3 dual-core wav_sim (qso3, 2026-06-05): **post_slotend 136〜138 ms, 7/7 decodes**.
Log: `embedded-poc/m5stack-cores3-app/logs/cores3_phaseD1_2026-06-05.log`.

## Quick file-path index

- Host FT8 reference (post-ε split, 0.6.3):
  `mfsk-core/src/ft8/decode_block.rs` (423-line parent / facade),
  `mfsk-core/src/ft8/decode_block/types.rs` (audio sample +
  tunables), `…/spectrogram.rs` (`Spectrogram` +
  `compute_spectrogram`), `…/coarse_sync.rs` (Costas search +
  allsum), `…/fill_symbol_spectra.rs` (per-symbol DFT family —
  **`fill_symbol_spectra_goertzel` is the current path; the
  BASIS `fill_symbol_spectra_into` is back-compat only, still
  present as of 0.7.4, tracked for removal under
  [#162](https://github.com/jl1nie/mfsk-core/issues/162)**),
  `…/process_candidates.rs` (engine +
  facade impls), `…/osd_strategy.rs` (OSD dispatch, #63 hook).
  Host `decode_frame*` family + `refine_fine` gate:
  `mfsk-core/src/ft8/decode.rs`.
- Probe templates: `mfsk-core/src/jt9/decode.rs::gate_diag::probe_missing_goldens`.
- Protocol-specific: `mfsk-core/src/fst4/decode.rs` ⇔
  `WSJT-X/lib/fst4_decode.f90`; `mfsk-core/src/jt65/{mod,rx,decode}.rs`
  ⇔ `WSJT-X/lib/jt65_decode.f90`.
- Embedded shared layers (board-agnostic):
  `embedded-poc/embedded-shared/src/{pipeline,dual_core,stage1_inc,esp_dsp_fft}.rs`
  (decoder layer), `embedded-poc/mfsk-app-shared/src/{qso,wifi,log_sink,boot_mode,ui/}`
  (controller layer).
- Embedded apps: `embedded-poc/m5stack-s3-app/src/{uac,civ,qso,audio,buttons,flash_log}.rs`
  (M5StickS3, demo / acoustic fallback per Phase B-Stick);
  `embedded-poc/m5stack-core2-app/src/` (Core2 sibling,
  wav_sim only).
- User manual: [`docs/reference/MANUAL_M5STICKS3.md`](../reference/MANUAL_M5STICKS3.md)
  ([JA](../reference/MANUAL_M5STICKS3.ja.md)).
- Infra: `.github/workflows/{ci,release}.yml`.
