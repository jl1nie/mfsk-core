# Roadmap (post-0.5.12)

## Context

0.5.12 closed FT8 AP-list issue #31 (iaptype-1 / blind-CQ pass added,
`ft8_qso3_apon_recall` regression test landed) and bundled three
0.5.11-shipping correctness fixes — most critically #26 (FT8 phase-2
SIC was running on un-subtracted residual; weak signals masked by
known strong ones stayed masked). Also re-locked JT9 recall at 7/7
after JTDX cross-check expanded the original 5-entry golden, and
restored FT4 GFSK BT=1.0 / NFILT=1400 to WSJT-X parity (#27, #28).

Three threads of work — confirmed with the user — for the next 3-month
horizon:

1. **Close the AP-on value loop** — issue #40 (host coarse-sync
   candidate gap) blocks the just-shipped AP iaptype-1 from surfacing
   any of the 6 JTDX-confirmed AP-on extras on `qso3_busy.wav`. Until
   #40 closes the AP work has no measurable real-WAV effect.
2. Lock the remaining host-side protocol goldens (FST4 #23, JT65 #24)
   using the same WSJT-X-source-faithful methodology that closed JT9.
3. Take `embedded-poc/m5stack-s3-app` from its current state (LCD-rendered
   WAV-fed FT8 demo; Phase 0/0.5/3 done) to a complete mountain-top FT8
   QSO transceiver controller (Phase 1/2/4/5/6 + TX keying).

User-confirmed scope decisions:
- **MSK144 (#25)** stays open as community-contribution invitation — NOT
  on this roadmap. The unique correlator + meteor-burst loop are too far
  from the FT/JT/Q-family pipeline to fit the 3-month plan honestly.
- **Embedded fixed-point pipeline** stays FT8-only. No FT4/JT9 embedded
  ports in scope; mountain-top app will be FT8-only.
- **m5stack-s3-app MVP target** = full QSO transceiver (RX + TX + CI-V +
  QSO FSM + ADIF), not just an RX spotter. Phased delivery: v0.6 = RX
  spotter useful in field, v0.7 = full QSO controller.

## Phase A — Host AP value loop + protocol golden lockdowns

### A0. Host coarse-sync candidate gap (#40) — small / medium, **highest priority**

The FT8 AP iaptype-1 pass landed in 0.5.12 (#31, PR #39) but currently
catches **0/6** of the JTDX-confirmed AP-on extras on `qso3_busy.wav`.
Root cause is upstream of AP: `decode_frame_with_ap` (host wide-band)
misses the underlying coarse-sync candidates at 1196 / 244 / 472 / 2039
Hz that `decode_block` (embedded path) and JTDX both pick up, so the
AP loop in `process_candidate` has nothing to rescue. Until A0 closes,
the 0.5.12 AP work has no measurable real-WAV effect — that's why this
sits ahead of A1/A2.

Approach: bisect what makes the two host pipelines diverge on the same
WAV at the same `sync_min`. Likely suspects (in rough priority):

- `coarse_sync` algorithm differences: `src/ft8/sync.rs` (host) vs
  `decode_block.rs` per-tone DFT path. Compare candidate count, score
  thresholding, NMS behaviour.
- `refine_fine::refine_fine_3stage` (host-only) — WSJT-X-faithful 3-stage
  filter intentionally rejects birdie phantoms above 2 kHz; the gate
  may be cutting real signals too. `decode.rs:464–486` is the call site.
- `nsync ≤ 6` early-return in `process_candidate` (`decode.rs:493`)
  vs whatever the embedded path uses at the same stage.

Tools to reuse:
- `tests/ft8_qso3_apon_recall.rs` — already has the AP-on / AP-off
  diff harness. Add a third pass that prints all coarse_sync candidates
  pre-`process_candidate` with their scores so the divergence is visible.
- `JTDX_EXTRAS_HARD_FLOOR` constant (currently `0`) is the seam: each
  candidate the host starts catching, raise the floor toward 6.

Estimate: 3-7 days of investigation. Outcome: floor at 6 (matching JTDX
recall on this WAV), or root-cause documented as deliberate divergence
with separate on-air WAV showing the host-path advantage.

### A0'. `decode_block_with_ap` for embedded — medium, follow-on

Host AP work doesn't help mountain-top runs until embedded gets AP
too. `decode_block.rs:2376` currently passes `None` to `bp_decode`.
Symmetric port of the host pass-5..12 multi-pass loop into the
embedded pow-of-2 FFT pipeline. Shape mirrors `decode_frame_with_ap` →
`process_candidate`, but without the rustfft dependency. Files:

- `mfsk-core/src/ft8/decode_block.rs` — add `decode_block_with_ap` /
  `_with_ap_options` paralleling `decode_frame_with_ap` /
  `decode_frame_with_ap_full`.
- `mfsk-core/tests/ft8_qso3_apon_recall.rs` — add a sibling
  `decode_block_with_ap` arm so the same WAV regression covers both
  paths.
- New issue to file (not yet open).

Estimate: 1 week after A0 (depends on understanding the coarse-sync
divergence first — A0 may inform the embedded port).

### A1. FST4-60A golden (#23) — small / medium

`tests/fst4_wsjtx_samples.rs` already exists but is `#[ignore]`d with
"decode_frame returns 0 messages". This is **not** "add a harness" — it's
"debug why our fst4 decode produces 0 results on the WSJT-X reference",
the same flavour as the JT9 work that just closed.

Approach: line-walk `WSJT-X/lib/fst4_decode.f90` + `lib/fst4sim.f90`
against `mfsk-core/src/fst4/decode.rs`. The LDPC(240, 101) + Costas-8
sync layout is shared with FT8 / WSPR and well-tested, so the divergence
is almost certainly in soft-symbol extraction or the sync-quality gate.

Tools to reuse:
- The probe pattern from `mfsk-core/src/jt9/decode.rs::gate_diag::
  probe_missing_goldens` (sweep frequencies in 0.5 Hz steps, print
  per-stage scores) — port to fst4.
- Sample at `/home/ubuntu/src/WSJT-X/samples/FST4+FST4W/210115_0058.wav`.

Estimate: 3-5 days. Outcome: `tests/fst4_wsjtx_samples.rs` passes with
recall locked, `#[ignore]` removed.

### A2. JT65 golden (#24) — medium

Sample mismatch discovered during exploration: WSJT-X ships **JT65B**
samples (`samples/JT65/JT65B/*.wav`, 8 files) but our implementation is
**JT65A**. Recommended path: add JT65B sub-mode and lock recall against
WSJT-X-distributed material. Reasons:

- The `mfsk-core::jt65::rx::demodulate_aligned` + `decode_at_with_erasures`
  + `Rs63_12::decode_jt65_erasures` chain is sub-mode-agnostic — JT65A vs
  JT65B differs only in NSPS / tone-spacing constants and `T_SLOT_S`.
- This mirrors the Q65 sub-mode generic pattern (`Q65a30`, `Q65a60`,
  `Q65b60`, …) that already works in the codebase.
- Locking against on-disk WSJT-X samples is genuine regression coverage,
  not a synth-only gate.

Files to add/touch:
- `mfsk-core/src/jt65/mod.rs` — new `Jt65b` ZST alongside the existing
  Jt65A module-as-protocol arrangement; reuse the `decode_scan_for<P>` /
  `decode_at_for<P>` generic shape that Q65 uses.
- `mfsk-core/src/jt65/rx.rs` — confirm tone-spacing parameter is generic
  over the protocol ZST; refactor if hard-coded.
- `mfsk-core/tests/jt65b_wsjtx_samples.rs` — new harness mirroring
  `tests/jt9_wsjtx_samples.rs` (which has the most recent / best
  template).
- `README.md` recall table — add JT65B line.
- Close #24.

Estimate: 1-2 weeks. Probe-debug iteration likely needed for the
low-SNR samples; the `decode_at_with_erasures` path is already complete
on JT65A so the soft-decision RS work doesn't repeat.

### A3. FST4-15 / FST4W (deferred)

FST4-15 and FST4W are tracked in #23 as "stretch". Deferred — no user
demand surfaced, FST4-60A is the dominant terrestrial sub-mode. Issue
remains open as a placeholder.

## Phase B — m5stack-s3-app: WAV-fed demo → full QSO transceiver

The app source already uses `Phase 0..6` markers in module doc-comments.
Phases below pick up those markers in an order chosen so each delivers
something field-deployable rather than "everything-or-nothing".

### B1. Phase 1 — Live UAC audio input (~2 weeks)

Replace the WAV-loop in `decode_pipeline.rs::wav_simulator_thread` with
USB Audio Class host capture from a transceiver (IC-705, FT-991A, …).

- `embedded-poc/m5stack-s3-app/src/uac.rs` — currently 16-line stub. Port
  the espressif `usb_host_uac` recipe to Rust via `esp-idf-svc` bindings.
- `decode_pipeline.rs` — drain a UAC sample ring at 12 kHz instead of
  looping WAV bytes.
- 48 kHz (typical radio rate) → 12 kHz resampler: re-use the Q32
  fixed-point linear resampler already shipped in
  `mfsk-ffi-ft8/src/stream.rs`. Either link via `mfsk-ffi-ft8` C API
  from the s3-app, or vendor the same algorithm directly into a small
  Rust module — choose at implementation time based on link footprint.
- `audio.rs::AUDIO_GATE` already supports muting playback during decode
  stress peaks — reuse.

Verifies: real S3 connected to IC-705 USB-OTG, tune 14.074 MHz, see live
FT8 decode lines in the LCD scroll panel.

### B2. Phases 5 + 6 — Persistent log + button input (~1 week)

Make the app useful as a "spotter that records what it heard" before any
TX automation lands.

- `flash_log.rs::LittleFsLog` — finish the three TODOs (mount VFS, write
  + rotate, dump). 45-line existing skeleton.
- `adif.rs` — append-only `/littlefs/qso.adi` with WSJT-X-compatible
  record format. Currently a 8-line placeholder.
- `buttons.rs` — wire GPIO 11/12 interrupts (pins are in `board.rs`) to
  an event queue. State-machine modes: Monitor → Cursor (select callsign
  in list) → QSO-prep (preview reply) → Menu (config). 13-line stub
  today.

After B2: v0.6 ship-ready as field RX spotter. Push tag, release.yml
publishes the m5stack-s3-app binary alongside the existing
`mfsk-ffi-ft8-*-esp32s3-xtensa.tar.gz` (see C3 below).

### B3. Phase 2 — BLE CI-V transport (~2 weeks)

- `civ.rs` — comment-only 26-line stub today. Uncomment the
  `esp32-nimble` dependency in `Cargo.toml`. Implement BLE central
  pairing with the IC-705 BLE service + the K7MDL2 protocol framing.
- UART fallback for non-BLE radios: USB-OTG pins are already defined
  (GPIO 19/20 in `board.rs`).
- API surface: `CivClient::{connect, read_freq, set_freq, set_mode,
  set_ptt}`.

### B4. Phase 4 + TX keying — QSO FSM + audio modulation (~3 weeks)

Hardest single block. Turns the spotter into a transceiver controller.

- `qso.rs::QsoManager` — currently 51-line type skeleton. Implement the
  IDLE → CALLING → REPORT → FINAL → DONE state machine with retry
  counters + timeout transitions, mirroring WSJT-X
  `lib/genft8.f90`'s auto-sequencer.
- TX audio modulation: feed `mfsk_core::ft8::wave_gen::tones_to_i16`
  output through I2S DAC → audio cable to radio mic input. Sample-rate
  match (12 kHz → 48 kHz I2S DAC clock) via simple repeat-N + linear
  interp.
- TX timing: hold PTT via `civ.rs::set_ptt(true)`, play 13 s of audio
  synced to slot boundary (`time_sync.rs` already publishes UTC), release
  PTT. Slot boundary detection: re-use `time_sync.rs`'s median-DT
  estimate when no GPS PPS / NTP is available.
- Callsign hash table: reuse `mfsk_core::msg::CallsignHashTable`.

Post-B4: v0.7 ships. The "leave the laptop at home" goal is realised.

## Phase C — Quality / infra

### C1. Embedded CI cross-build (~2 days)

The 0.5.10 release fail-and-hotfix-to-0.5.11 cycle this morning came
from `f32::round` missing under `no_std`. The Xtensa build runs only at
release-time today, so the regression was caught post-tag. Add a
`feature-matrix-embedded` job to `.github/workflows/ci.yml`:

- `--target xtensa-esp32-espidf --no-default-features --features
  embedded-fixed-point,embedded-runtime` (mfsk-ffi-ft8)
- `--target xtensa-esp32s3-espidf` ditto
- Runs on every PR, not just tag-push.

The release.yml binary-build steps can stay as the artifact-emitting
counterpart — same compile invocation, different post-build action.

### C2. Reproducible release builds (~1 day)

Pin the `+esp` toolchain version in `.github/workflows/release.yml`'s
`esp-rs/xtensa-toolchain@v1.5` step so artifacts are deterministic
across release runs (currently floats on whatever `+esp` version the
action vendors).

### C3. m5stack-s3-app artifact in release.yml (deferred to v0.6)

Add a release.yml job that builds the s3-app binary + packages a
flashable image (`espflash save-image` output) so v0.6 release
artifacts include a one-step install for the field-deployable
spotter.

## Sequencing (3-month horizon)

```
Now           +1m           +2m           +3m
 │             │             │             │
 A0 host coarse-sync gap ┐   │             │
 (3-7d, blocking 0.5.12  │   │             │
  AP work value)         │   │             │
                         │   │             │
 A0' decode_block_with_ap│   │             │
 (~1w, after A0)         │   │             │
         │               │   │             │
         A1 FST4─┐   A2 JT65──┐             │
         (3-5d)  │   (1-2w)   │             │
                 │            │             │
 B1 UAC live audio ───────────┼             │
 (~2w, in parallel with A)    │             │
                 │            │             │
                 B2 log+buttons┐            │
                 (~1w)         │            │
                              v0.6          │
                               │            │
                               B3 CI-V (~2w) ┐
                               │             │
                               │             B4 QSO+TX (~3w) ┐
                               │             │             v0.7
 C1 embedded-CI                │             │             │
 (in parallel, ~2d)            │             │             │
                               │             │             │
 C2 reproducible release (~1d, opportunistic)
 C3 s3-app artifact (with v0.6)
```

A0 sits at the front because the AP iaptype-1 pass shipped in 0.5.12
has no measurable effect on real WAVs until it closes. A1 + A2 in
Phase A are independent of Phase B and can run in parallel when
there's host-side context-switching headroom.

## Verification per phase

- **A0 host coarse-sync gap**: `cargo test --release -p mfsk-core
  --features full --test ft8_qso3_apon_recall -- --nocapture` ⇒
  `JTDX AP-on extras: 6/6 hit`, then raise `JTDX_EXTRAS_HARD_FLOOR`
  to 6. Bonus: the AP-off baseline of `decode_frame_with_ap` matches
  `decode_block`'s 7/8 against the WSJT-X canonical golden.
- **A0' embedded AP**: `cargo test --release -p mfsk-core --features
  full --test ft8_qso3_apon_recall` (with the new
  `decode_block_with_ap` arm enabled) ⇒ same JTDX extras coverage on
  the embedded path. Optionally re-flash M5Stack S3 with a slot
  containing operator-context QSO and visually confirm AP rescues.
- **A1 FST4-60A**: `cargo test --release -p mfsk-core --features full
  --test fst4_wsjtx_samples` (no `--ignored`, the test stops being
  ignored) ⇒ recall locked.
- **A2 JT65B**: `cargo test --release -p mfsk-core --features full
  --test jt65b_wsjtx_samples` ⇒ recall locked. README table updated.
- **B1 UAC**: real S3 connected to IC-705 via USB-OTG, tune 14.074 MHz
  during a live FT8 window, decode lines appear on LCD scroll within
  one slot of activation.
- **B2 v0.6**: power-cycle test — boot S3, decode for 10 minutes, power
  off, power on, ADIF file from before persists; button cycles modes.
- **B3 CI-V**: S3 reads IC-705 freq via BLE, writes a new freq, radio
  responds.
- **B4 v0.7**: two-station test — S3 calling CQ, WSJT-X on PC replies;
  S3 auto-sends report, QSO completes through 73; ADIF entry written
  with correct callsign / RPRT / time.
- **C1**: PR with intentional `no_std` regression (e.g. `f32::round`
  without `num_traits::Float` import) is caught by ci.yml before merge.

## Critical file paths

Host-side (Phase A):
- `mfsk-core/src/ft8/sync.rs` + `mfsk-core/src/ft8/decode_block.rs`
  (A0 — coarse-sync algorithm comparison)
- `mfsk-core/src/ft8/refine_fine.rs` (A0 — phantom-filter gate audit)
- `mfsk-core/tests/ft8_qso3_apon_recall.rs::JTDX_EXTRAS_HARD_FLOOR`
  (A0 progress seam — grow toward 6 as candidates start surviving)
- `mfsk-core/src/ft8/decode.rs::process_candidate` (A0' — port the
  multi-pass AP loop into the embedded `decode_block_*` family)
- `mfsk-core/tests/fst4_wsjtx_samples.rs` (existing, `#[ignore]`d)
- `mfsk-core/src/fst4/decode.rs` ⇔ `WSJT-X/lib/fst4_decode.f90`
- `mfsk-core/src/jt65/{mod,rx,decode}.rs` ⇔ `WSJT-X/lib/jt65_decode.f90`
- `mfsk-core/src/jt9/decode.rs::gate_diag::probe_missing_goldens`
  (template for FST4 / JT65 / FT8-coarse-sync probe patterns)

Embedded app (Phase B):
- `embedded-poc/m5stack-s3-app/src/{uac,civ,adif,qso,buttons,flash_log}.rs`
  (each is currently stub or skeleton)
- `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs` (replace WAV-loop
  source for B1)
- `embedded-poc/m5stack-s3-app/src/{audio,time_sync,display}.rs`
  (already-functional, will be reused / extended)
- `mfsk-ffi-ft8/src/stream.rs` (Q32 resampler reuse for B1)

Infra (Phase C):
- `.github/workflows/ci.yml` (add embedded matrix job)
- `.github/workflows/release.yml` (pin esp toolchain, add s3-app
  artifact)
