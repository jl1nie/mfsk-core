# Roadmap (post-0.6.2)

## v0.6.2 status (shipped, 2026-05-10)

Cumulative `v0.6.0` + `v0.6.1` + `v0.6.2` shipped as one release via PR #50
(commit `ba067fd`, crates.io `mfsk-core@0.6.2` / `mfsk-ffi-ft8@0.6.2`).
The 2026-05-13 post-merge sweep (`#53`, `#60`, `#62`, `#66`, `#67`, `#68`,
`#69`) closed the documentation / dead-code follow-ups against this bundle
without bumping the crate version. See `docs/CLEANUP_2026_05.md` for the
γ/β/δ/ε cleanup plan that drove that sweep.
Headline numbers (all on `qso3_busy.wav`):

- Host AP-off recall: **7/8** WSJT-X golden, **16/18** JTDX golden
  (`decode_block` path).
- Host AP-on multipass: **18 decodes total / 5 of 6 JTDX-extras**
  (was 1/6 pre-0.6.2 because `decode_frame_subtract_with_ap` had been
  using `subtract_signal_weighted` instead of the WSJT-X-faithful
  `subtract_signal_lpf`).
- Embedded S3 (M5StickC Plus2-S3, fixed-point + `esp-dsp`):
  **6/18 + 1 bonus = 7 total** in **~1.19 s** post-SlotEnd.

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

Currently open GitHub issues (state:open as of 2026-05-14):

- **#23** — FST4-60A golden lockdown (host); FST4-15 / FST4W
  stretch. Carried forward from the post-0.5.12 "Phase A1".
- **#24** — JT65B golden lockdown + erasure-metadata path.
  Carried forward from the post-0.5.12 "Phase A2".
- **#25** — MSK144 decode path. Community-contribution invitation;
  not on the 3-month roadmap.
- **#58** — coalesce redundant `compute_llr` between Step 3 (OSD)
  and Step 4 (AP) in `decode_block`. Low-priority host perf.
- **#61** — fold `m5stack-core2` into the S3 dual-core pipeline
  and retire the Core2 crate. Weekend hardware bring-up
  (2026-05-17/18) per `docs/CLEANUP_2026_05.md` (ε prerequisite).
- **#63** — WSJT-X-faithful OSD `npre1` precoding for host
  `BpAllOsd`; reduces false positives. Deprioritised — see PR #62
  for the design note documenting the current parity gap.
- **#64** — hoist `fft_cache` through host `decode_block_multipass`
  (perf follow-up to #60, which landed the single-pass hoist).
- **#65** — share `cd0` between SyncOnly + DataOnly
  `fill_symbol_spectra` calls (host perf nice-to-have).

Carry-overs from the post-0.5.12 plan that have since closed:

- **A0** (`#40` host wide-band coarse-sync gap) — closed by v0.6.0.
- **A0'** (`decode_block_with_ap` embedded reach) — landed in
  v0.6.1 as Step 4 of `process_one_candidate_inner`. The
  `decode_block_with_ap` symmetric public entry exists but the
  5 missing JTDX-extras on `qso3_busy.wav` sit upstream of AP
  (host coarse-sync surfaces them only with `subtract_signal_lpf`
  multipass — a host-side win, not embedded). No new issue filed
  yet; revisit when an embedded-only WAV surfaces where the gap is
  post-coarse-sync.

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
  `docs/CLEANUP_2026_05.md`) so the seam exists to hook a
  streaming alternative onto.

## v0.6.0 status (shipped, 2026-05-09)

Bundled refactor + AP iaptype 2 release. Closes:

- **#40** (host wide-band coarse-sync candidate gap) — host
  `decode_frame_with_ap` now routes through `decode_block::coarse_sync`
  + the i_start-as-i32 fix; AP-off recall on `qso3_busy.wav` 5/8 → 7/8
  (matches WSJT-X parity).
- **#46** (sync consolidation PR) — `ft8::sync::coarse_sync` removed;
  `decode_block::coarse_sync` + `compute_spectrogram` graduate to public
  API.
- **#48 step B** (FT8 sync routes through decode_block).
- **#49 cat A** (WAV-loader test consolidation, 14 dupes → 4 helpers).
- **#49 cat B** (`ft8::sync` thin wrappers deleted).
- **#49 cat C** (`#[doc(hidden)]` graduation for items embedded
  consumers + FFI already depend on).

A0 / A0' (host coarse-sync gap and `decode_block_with_ap`) from the
earlier roadmap were the carry-overs at the time of the v0.6.0 cut;
the master Open follow-ups list above (under v0.6.2 status) supersedes
this section.

## Cleanup 2026-05

`docs/CLEANUP_2026_05.md` tracks a four-stage post-0.6.2 tidy-up
(γ scaffolding → β feature/cfg → δ docs sync → ε `decode_block.rs`
restructure). Status as of 2026-05-14:

- **γ** Done 2026-05-13 (PR #66). Retired
  `embedded-poc/m5stack-{core2,s3}/src/bin/rx_skeleton.rs`.
- **β** Done 2026-05-13 (PR #67 + #69). Compiler-visible dead code
  cleared; `Cmplx` unified with `num_complex::Complex` via type
  alias (−5 `unsafe` cast wrappers); cfg-matrix audit confirms
  every fixed-point × fft-{rustfft,extern} cell builds clean.
- **δ** Documentation sync — current sweep.
- **ε** `decode_block.rs` restructure into a `decode_block/`
  submodule directory + OSD strategy seam for `#63` precoding.
  Week-scale; intentionally sequenced after the weekend hardware
  bring-up for `#61` (Core2 → S3 unification) so the two refactors
  do not collide.

# Roadmap (legacy, written for post-0.5.12)

Most of the post-0.5.12 plan landed in the 0.6.x bundle (PR #50,
shipped 2026-05-10) or is now tracked under a GitHub issue. The
sections below survive as historical context plus quick file-path
hints; the live worklist is the **Open follow-ups** section above.

## Phase A — Host protocol golden lockdowns

- **A0** (`#40` host coarse-sync gap) and **A0'** (`decode_block_with_ap`)
  closed in v0.6.0 / v0.6.1 respectively.
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

## Phase B — m5stack-s3-app

Per `embedded-poc/m5stack-s3-app/CLAUDE.md` (and the Phase 0..6
markers in module doc-comments), current state as of 2026-05-13:

- **Phase 0 / 0.5 / 3** — Done. LCD bring-up, WAV-fed pipeline,
  4-region UI (status / waterfall / decoded list / TX strip).
- **Phase 4** — QSO FSM dry-run done (auto-CQ visible on LCD,
  `qso.rs` ~360 lines, 8 host-side unit tests). No TX audio
  synthesis yet (parked behind Phase 1 UAC per user judgement —
  speaker output is a stopgap, real path is USB UAC OUT to the
  radio).
- **Phase 0.6 / 0.7** — Done. WiFi UDP log streaming (UART /
  LCD / UDP fanout) + USB-CDC freeze fix (`println!` gated on
  `usb_serial_jtag_is_connected`) + runtime boot-mode selector
  (NVS + KEY2 long-press) for WiFi / decoder coexistence.
- **Phase 1 UAC** — Pending. `uac.rs` is still a doc placeholder;
  port `espressif/esp_usb_audio` via `esp-idf-svc` bindings,
  drain a 12 kHz sample ring into `decode_pipeline.rs`. 48 → 12
  kHz resample reuses the Q32 linear resampler from
  `mfsk-ffi-ft8/src/stream.rs`.
- **Phase 2 BLE CI-V** — Pending. `civ.rs` comment-only stub;
  uncomment `esp32-nimble` in `Cargo.toml`, implement central
  pairing with the IC-705 BLE service + K7MDL2 framing.
- **Phase 5 ADIF / Phase 6 buttons** — Pending. `flash_log.rs`
  (littlefs mount / rotate / dump), `adif.rs` (append-only
  `/qso.adi`), `buttons.rs` (GPIO 11/12 IRQ + Monitor/Cursor/
  QSO-prep/Menu mode FSM).
- **TX keying** — Pending. Pair with Phase 2 (`civ::set_ptt`)
  and Phase 1 (TX audio synth to UAC OUT endpoint).

Sequencing not committed beyond "Phase 1 UAC next".
`mfsk-ffi-ft8/src/stream.rs::mfsk_ft8_stream_*` and
`embedded-shared` resampler API are the seams.

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

- Host FT8 reference: `mfsk-core/src/ft8/decode_block.rs` (canonical
  coarse-sync + per-candidate inner), `mfsk-core/src/ft8/decode.rs`
  (host `decode_frame*` family + `refine_fine` gate).
- Probe templates: `mfsk-core/src/jt9/decode.rs::gate_diag::probe_missing_goldens`.
- Protocol-specific: `mfsk-core/src/fst4/decode.rs` ⇔
  `WSJT-X/lib/fst4_decode.f90`; `mfsk-core/src/jt65/{mod,rx,decode}.rs`
  ⇔ `WSJT-X/lib/jt65_decode.f90`.
- Embedded app: `embedded-poc/m5stack-s3-app/src/{uac,civ,adif,qso,buttons,flash_log}.rs`
  (current state per the section above);
  `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs` (Phase 1
  UAC integration site).
- Infra: `.github/workflows/{ci,release}.yml`.
