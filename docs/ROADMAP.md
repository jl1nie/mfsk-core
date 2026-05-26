# Roadmap (post-0.6.5)

## Current line (0.6.x) — shipped through 0.6.5 (2026-05-18)

The 0.6 line shipped in four cuts: a bundled `v0.6.0` + `v0.6.1` + `v0.6.2`
via PR #50 (2026-05-10), then `v0.6.3` (2026-05-17, PR #89 — WSJT-X-faithful
OSD precoding + the ε `decode_block` split), then `v0.6.4` (2026-05-18,
PR #104 — Phase 1.7.7-Stick Goertzel migration), and finally `v0.6.5`
(2026-05-18, PR #108 — crates.io surface refresh; no behaviour change).
The cleanup-2026-05
γ/β/δ/ε plan landed end-to-end across these cuts; see
`docs/historical/CLEANUP_2026_05.md` (historical) for the original prescription.
Headline numbers (all on `qso3_busy.wav`):

- Host AP-off recall: **7/8** WSJT-X golden, **13/18** JTDX golden
  (`decode_block` path). The JTDX number peaked at 16/18 in 0.6.2
  but 0.6.3's WSJT-X-faithful OSD `npre1`/`npre2` precoding +
  `OSD_HARDERRORS_MAX = 22` ceiling identified 3 of those as
  CRC-luck phantoms and dropped them — the 13/18 is true positives
  only. See 0.6.3 CHANGELOG entry for the
  faithfulness-vs-recall trade-off rationale.
- Host AP-on multipass: **17 decodes total / 4 of 6 JTDX-extras**
  on the `decode_frame_subtract_with_ap` path (also lost one
  CRC-luck phantom in 0.6.3 — 5/6 → 4/6). The path had been
  stuck at 1/6 pre-0.6.2 because it used `subtract_signal_weighted`
  instead of the WSJT-X-faithful `subtract_signal_lpf`; 0.6.2
  unified onto the LPF path → 5/6, then 0.6.3 trimmed the
  CRC-luck phantom to 4/6 true positives.
- Embedded S3 (M5StickS3, fixed-point + `esp-dsp` + Goertzel):
  **6/18 + 1 bonus = 7 total** in **~1.19 s** post-SlotEnd, with
  **+0.16..+0.63 dB SNR improvement** over the BASIS-era 0.5.x
  baseline (Phase 1.7.7-Stick Goertzel migration). The 6/18+1=7
  embedded decode count was last formally measured in the 0.6.2 →
  0.6.3 Q11i16 ship sweep; 0.6.3's host-side OSD tightening was
  not re-measured on embedded (no embedded log captured post-0.6.3
  in `embedded-poc/m5stack-s3/logs/`). Treat as the last-confirmed
  number, not a re-measured 0.6.5 figure — `wav_sim` re-run is
  needed to confirm whether the host phantom drop applies to the
  embedded path as well. The `~1.19 s` post-SlotEnd is similarly
  the 0.6.3 measurement (CHANGELOG 0.6.3 "Verified on M5StickS3
  S3 LX7, 7/8 golden" only confirms the WSJT-X 8-set, not the
  JTDX 18-set or the wall-clock).
- Embedded internal-DRAM use: BASIS scratch dropped (120 KB freed
  on dual-core), unblocking M5StickS3 Qso-mode bidirectional I2S
  DMA allocation.

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
  the M5StickS3 source crate and `docs/MANUAL_M5STICKS3.md`.
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
with 0.5.x callers, scheduled for removal in 0.7.0.

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

Currently open GitHub issues (state:open as of 2026-05-18):

- **#23** — FST4-60A golden lockdown (host); FST4-15 / FST4W
  stretch. Carried forward from the post-0.5.12 "Phase A1".
- **#24** — JT65B golden lockdown + erasure-metadata path.
  Carried forward from the post-0.5.12 "Phase A2".
- **#25** — MSK144 decode path. Community-contribution invitation;
  not on the 3-month roadmap.
- **#58** — coalesce redundant `compute_llr` between Step 3 (OSD)
  and Step 4 (AP) in `decode_block`. Low-priority host perf.
- **#64** — hoist `fft_cache` through host `decode_block_multipass`
  (perf follow-up to #60, which landed the single-pass hoist).
- **#65** — share `cd0` between SyncOnly + DataOnly
  `fill_symbol_spectra` calls (host perf nice-to-have).
- **#72** — `DecodeStrictness` duplicate definition + uncalibrated
  copy for FT4 / FST4. API hygiene; pick one definition and remove
  the duplicate.
- **#73** — `EqMode::Adaptive` has collapsed into `EqMode::Local`
  in practice. Either restore the distinct fallback behaviour or
  drop the variant.
- **#74** — `DecodeDepth::Bp` is the cheapest staircase rung;
  confirm there is a real caller before keeping it.

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

The 5 remaining JTDX AP-on extras on `qso3_busy.wav` that surface
only with `subtract_signal_lpf` multipass are a host-side win
(coarse-sync upstream of AP); no embedded issue filed yet —
revisit when an embedded-only WAV exhibits a post-coarse-sync
gap.

Architectural notes that did not graduate to issues:

- **#48 option A** — `Protocol::Sync` associated type for
  type-system enforcement against future protocol-sync drift.
  Scoped out of v0.6.x (8 protocols + 2 macros + embedded feature
  matrix; benefit is speculative for non-FT8 protocols).
- **#49 cat D** — `core::sync::coarse_sync<P>` `NotFt8` marker
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
[`docs/historical/CLEANUP_2026_05.md`](historical/CLEANUP_2026_05.md) (HISTORICAL) for the
original four-stage plan; the per-stage acceptance-criteria template
documented there is reused for future cleanup waves.

# Roadmap (legacy, written for post-0.5.12)

Most of the post-0.5.12 plan landed in the 0.6.x bundle (PR #50,
shipped 2026-05-10) or is now tracked under a GitHub issue. The
sections below survive as historical context plus quick file-path
hints; the live worklist is the **Open follow-ups** section above.

## Phase A — Host protocol golden lockdowns

(A0 / A0' both closed in v0.6.x — see the closed-issues table above.)

- **A1** FST4-60A (`#23`) — `tests/fst4_wsjtx_samples.rs` is `#[ignore]`d
  with "decode_frame returns 0 messages"; root-cause line-walk of
  `WSJT-X/lib/fst4_decode.f90` against `mfsk-core/src/fst4/decode.rs`
  still pending. Probe template:
  `mfsk-core/src/jt9/decode.rs::gate_diag::probe_missing_goldens`.
- **A2** JT65 (`#24`) — current implementation is JT65A; WSJT-X
  ships JT65B samples. Add a `Jt65b` ZST mirroring the Q65 sub-mode
  generic pattern (`Q65a30`, `Q65a60`, ...) and lock recall against
  `samples/JT65/JT65B/*.wav` via a new
  `tests/jt65b_wsjtx_samples.rs` harness.
- **A3** FST4-15 / FST4W — `#23` "stretch", deferred indefinitely
  (no user demand; FST4-60A is the dominant terrestrial sub-mode).

## Phase B — embedded controller line

**2026-05-17 pivot**: Phase 1 UAC hardware verification confirmed
**M5StickS3 cannot do USB host** (board lacks VBUS source circuit, ID
pin wiring, host power switch IC — silicon supports it, board doesn't
wire for it). Phase B splits in two:

- **Phase B-Stick** — `m5stack-s3-app`, demoted to **demo / acoustic
  fallback** path. Frozen at Phase 1.5; Phases 1 (UAC), 2 (BLE CI-V),
  5 (ADIF), 6 (buttons), TX keying all roll forward to Phase B-Core.
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
  (`mfsk_core::core::sync::bootstrap_dt_median`) shared with
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

Status: planned. Brings up after CoreS3 unit arrives.

- **Phase 0-Core** (task #48) — Crate skeleton: board.rs (ILI9342C
  SPI pins, I2C0 for AXP2101 + AW9523B + FT6336U, USB-OTG fixed
  GPIO 19/20, ES7210 audio pins), pmic.rs (AXP2101 + AW9523B init,
  distinct from S3-app's M5PM1 and Core2-app's AXP192), display.rs
  (ILI9342C via mipidsi — Core2-app pattern), main.rs orchestration,
  decode_pipeline.rs thin wrapper. Workspace member added.
- **Phase 1-Core** (task #49) — UAC: clone `uac.rs` from s3-app
  verbatim; pmic.rs drives **AW9523B P1 (BUS_OUT_EN) HIGH** before
  `usb_host_install()` (omission = floating VBUS = floating host
  capability). BootMode::Uac dispatch arm.
- **Phase 1-Verify** (tasks #33 / #34) — 1500 Hz tone injection from
  IC-705 → FT8 candidate appears in decoded list; then live antenna
  → end-to-end RX confirmed.
- **Phase 1.5-Core** (task #50) — Hoist `uac.rs` into
  `mfsk-app-shared` (gated `cfg(feature = "uac")`); both s3-app and
  cores3-app consume via shared. Deferred until Phase 1-Verify
  passes (no premature abstraction).
- **Phase 2-Core** — BLE CI-V to IC-705. Same `civ.rs` work that
  was queued on s3-app; CoreS3 also has BLE.
- **Phase 5-Core** — ADIF (`flash_log.rs` + `adif.rs`). Crate-agnostic;
  could be hoisted to `mfsk-app-shared` from day 1.
- **Phase 6-Core** (task #51) — FT6336U capacitive touch driver
  (CoreS3 base has no physical buttons beyond Power). Replaces the
  Stick `buttons.rs` paradigm with touch zones (menu / decoded-list
  tap-to-select / TX-strip tap-to-send).
- **Phase 7-Core** — TX keying (paired with Phase 2-Core
  `civ::set_ptt` + Phase 1-Core TX audio synth to UAC OUT endpoint).

Sequencing: B-Stick Phase 1.5 (acoustic) starts immediately as a
parallel demo while CoreS3 hardware is procured. B-Core Phase 0 starts
the moment CoreS3 lands on the bench. `mfsk-ffi-ft8/src/stream.rs::mfsk_ft8_stream_*`
and `embedded-shared` resampler API are the seams shared across both.

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

## Quick file-path index

- Host FT8 reference (post-ε split, 0.6.3):
  `mfsk-core/src/ft8/decode_block.rs` (423-line parent / facade),
  `mfsk-core/src/ft8/decode_block/types.rs` (audio sample +
  tunables), `…/spectrogram.rs` (`Spectrogram` +
  `compute_spectrogram`), `…/coarse_sync.rs` (Costas search +
  allsum), `…/fill_symbol_spectra.rs` (per-symbol DFT family —
  **`fill_symbol_spectra_goertzel` is the current path; the
  BASIS `fill_symbol_spectra_into` is back-compat only,
  removed in 0.7.0**), `…/process_candidates.rs` (engine +
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
- User manual: [`docs/MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md)
  ([JA](MANUAL_M5STICKS3.ja.md)).
- Infra: `.github/workflows/{ci,release}.yml`.
