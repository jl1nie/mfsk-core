# S3 / CoreS3: what the hard-sync gate and OSD give without the FIR baseband (stage 0, host)

2026-11-25. Stage 0 of the S3 plan (`MIDRANGE_EMBEDDED_EVAL.md` is the earlier experiment). The question: the low-memory
profile (`decode_block_low_mem`) reaches `qso3_busy` 14 of 20 and 279 of `jt9`'s 402, the ESP32 driver 7 and 131. How much
of that needs the FIR baseband (`cd0_fir`), which the S3 tried in 0.6.2 and gave up (`ROADMAP.md`, "Embedded fine_refine
attempt postmortem": PSRAM bandwidth) and which costs more instructions than Goertzel? **Everything here is host data**
(Apple M-series, fixed-point build, 8 kHz input unless said); nothing was run on an Xtensa core.

## Variants (all with `LlrEffort::Minimal`, coarse search in one band, `sync_min` 1.3)

| | candidates | fine refine | symbol spectra | OSD |
|---|---|---|---|---|
| V0 | coarse score order (`decode_block`, the ESP32 driver) | none (the embedded `fine_refine_pass1` is a no-op) | Goertzel | no |
| V3 | score order | none | Goertzel | yes |
| V8 | score order | `fine_sync_12k` (the board's `fine_sync_split`) | Goertzel | no |
| V1 / V2 | hard-sync gate, then the pass-2 re-rank | none | Goertzel | no / yes |
| V6 / V7 | gate, then the pass-2 re-rank | `fine_sync_12k` | Goertzel | no / yes |
| V10 / **V9** | **the first `max_cand` by hard sync, no re-rank** | `fine_sync_12k` | Goertzel | no / **yes** |
| V5 / V4 | gate | FIR baseband (`fine_refine_3stage`) | FIR baseband | no / yes (the ICAP profile) |

## Results (`hosttest/icap-app/tests/s3_gate_osd.rs`, `icap-profile --variant vN`)

| | 22 recordings (of 402), 15 / 30 cand. | `qso3_busy` (of 20), 15 / 30 | instructions, `qso3_busy`, 15 cand. |
|---|---:|---:|---:|
| V0 ESP32 driver | 131 / 144 | 7 / 7 | 308 M |
| V3 score order + OSD | 135 / 148 | 7 / 7 | |
| V8 score order + `fine_sync_12k` | 136 / 145 | 7 / 7 | 248 M |
| V1 gate | 173 / 233 | 7 / 7 | |
| V2 gate + OSD | 184 / 249 | 8 / 9 | |
| V6 gate + `fine_sync_12k` | 174 / 248 | 9 / 10 | |
| V7 gate + `fine_sync_12k` + OSD (with re-rank) | 185 / 267 | 11 / 14 | |
| V10 gate order + `fine_sync_12k` | 211 / 248 | 9-10 / 10 | 259 M |
| **V9** gate order + `fine_sync_12k` + OSD | **221 / 267** | **12-13 / 14** | **283 M** |
| V5 FIR baseband, no OSD | 216 / 257 | 10 / 10 | 596 M |
| V4 FIR baseband + OSD | 227 / 279 | 13 / 14 | 614 M |

(`qso3_busy` 13 at 8 kHz input, 12 at 12 kHz for V9 and V4 at 15 candidates; the 22-recording figures are the same at both rates.)

**What each piece is worth** (22 recordings / `qso3_busy`):
- The gate alone: +42 / +89 (15 / 30 candidates) on the recordings, **nothing on `qso3_busy`** (7 stays 7).
- `fine_sync_12k` after the gate: `qso3_busy` 7 to 9-10; recordings +1 / +15. Without the gate it adds nothing (V8 = V0 + 5).
- OSD: small on its own (V3 = V0 + 4) and +10 / +19 on top of gate and fine sync, `qso3_busy` +3 / +4. **It only pays on a good candidate list.**
- Re-ranking the gate's list by block-0 quality (the pass-2 step of the boards) **costs** recall at 15 candidates: V7 185 against V9 221.
- **V9 has 94 % / 91 % of V4's recall gain over V0 on the recordings and all of it on `qso3_busy`**, with no FIR baseband.

**Cost** (instructions per slot, `qso3_busy`, 12 kHz input so the harness's resampler is not counted; host, not LX7): V9 283 M is
below V0's 308 M. The FIR variants are twice that (596-614 M). So the gate, `fine_sync_12k` and OSD do not raise the
instruction count of the ESP32 driver, and the FIR path doubles it; this agrees with the operation count (the FIR cascade
is about 3.0 M multiply-adds a candidate, Goertzel about 1.2 M iterations).

## Caveats (read before relying on this)

- **Host only.** LX7 times, PSRAM effects and the dual-core split are not in any of it. In particular `fine_sync_12k`
  is documented at **~292 ms on the board** (`dual_core.rs:366`, for the candidates it is given; split over two cores), and V9 depends on it:
  without it (V1) `qso3_busy` is 7. It is the piece to measure first.
- The harness has **no per-message dedupe** between candidates (the boards' `run_speculative_slot` has `dedup_by_message`). V9's
  extras are two copies of a strong signal at another frequency (`PA3EPP SP8NFO` 150-250 Hz from `jt9`'s 906 Hz line); they
  are attested in `profile::ATTESTED_EXTRAS` with that reason.
- The boards run `sync_min` 1.0 and the partial spectrogram of `SPEC_EMIT_PAIR` 87; the harness uses 1.3 and the whole slot.
  The gate on a partial spectrogram is untested (plan risk 6).
- V9 takes the first `max_cand` by hard sync *before* fine sync; the boards fine-sync all 30 of pass 1 when there is slack. Not the same flow.
- OSD's cost on LX7 is unmeasured for FT8 (FT4 on an S3: +2 345 ms over 31 candidates; `FT4_BENCHMARK_LOG.md` §22). Here OSD is +10 / +19 decodes
  on the recordings and +3 / +4 on the busy slot.

## What this changes in the plan

- **The S3 needs no FIR baseband for this recall.** Stage 1's arms become: B1 the ESP32 driver (V0), V8 (the board with time to spare), V10 (gate,
  no OSD), **V9 (gate + OSD)**; V4 stays as a reference only.
- The new library item is the gate on the boards' own spectrogram (`coarse_sync_gated_spec`), and the pass-2 re-rank is bypassed for the gate's list.
- OSD is decided on the board: G1 (V10) against G2 (V9) costs, with the number of candidates that reach OSD.
