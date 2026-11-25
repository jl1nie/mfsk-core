# Mid-range embedded FT8: keep the band as one baseband (evaluation)

2026-11-25. An experiment, not a design: nothing here is decided, and nothing is committed. It answers one question for
the targets between the ESP32 line and a PC: **what should a decoder keep between its phases when it cannot hold the
slot?** It follows `ICAP_EMBEDDED_ANALYSIS.md`, which found that the gap to ICAP's own `ft8dec` was what each phase
leaves in RAM, not the algorithms.

Every figure is from one Apple M-series Mac (one thread; this machine swings in speed with its power state, so a time
is a direction, not a ratio) and **nothing was run on a Cortex-A9, an A53 or an RP2350**. Each number says whether it
was measured, derived from the code, or guessed.

## 1. The class, and the one assumption about input

The target is defined by resources, not by a board: **working memory `W`** (the fast part: SRAM on an MCU, the L2
cache on a Cortex-A), **the input rate**, and **how the audio arrives**. Examples that fit, none of them measured
here: ICAP (A9, 240 KiB, 8 kHz, SD read forward), RP2350, STM32H7, a Raspberry Pi 3B (A53, 512 KB L2, 1 GB DRAM) or a
Pi Zero 2 W. The ESP32 line (PSRAM holds the slot, compute is dear) is not changed by this and not in the class.

The only thing asked of the input is **the samples in order, once** (`BandStream::feed`). A live stream, a slot in
RAM and a file read forward are the same to it: a test feeds one slice, single samples and uneven chunks and gets
the same bytes. Re-reading (`AudioSource`, `CountingSource`) is kept only to measure the method that needs it.

## 2. Three ways to spend the memory

| | what stays in RAM after the audio is read | audio reads |
|---|---|---|
| **A** (`decode_block_low_mem_src`, the ICAP profile of the last note) | a band of the spectrogram at a time; 4 candidates' 200 Hz basebands | 13 sweeps |
| **B** (ICAP's `ft8dec`) | a u8 log waterfall of the whole band, 166 KiB | 3 (one per pass) |
| **C** (`ft8::decode_block::band_bb`, new) | **the whole band as one complex baseband**, 3 kHz, `i16` pairs, 180 KB (any input rate) | **1** |

B was **not re-implemented**: ICAP's `ft8dec` is B, and was measured as it is (it is the reference in the tables). C
is the FST4 monitor's idea (`embedded-shared/src/fst4_monitor.rs`: one coarse baseband during capture, then a recentre
per candidate) done for FT8 and carried through the whole decode.

**What C does** (`mfsk-core/src/ft8/decode_block/band_bb.rs`, 4 unit tests and 3 integration tests):

1. `BandStream`: mix by 1600 Hz, 12 kHz -> 6 kHz (17 taps) -> 3 kHz (117 taps, pass 1400 / stop 1600 Hz), kept as
   `i16` pairs. Baseband sample `k` is centred on input sample `4k`, so `cd0_fir`'s grid is a multiple of it.
2. **The coarse search from the baseband**: each frame's 480 baseband samples, zero-padded to 960, transformed
   (complex), bin `i` of the audio spectrogram being baseband bin `i - 512`; the cells are rescaled to the audio
   spectrogram's (f32: x16/300^2; fixed-point: the 3840-point `sc16` FFT divides by 256). The gate and the scorer are
   the existing ones (`banded_peaks_with`).
3. **A candidate's `cd0`**: shift by `freq - 1600` Hz, then 3:1 (15 taps) and 5:1 (43 taps), about 0.9 M multiply-adds
   against 3.0 M from the audio (derived). The candidate loop is the existing one (`decode_from_basebands`, which A
   now shares).
4. **Subtraction on the baseband** (`BandBaseband::subtract`, `decode_band_sic`): `subtractft8.f90`'s algorithm. The
   GFSK reference comes from `GfskStream` every fourth sample, its complex amplitude is the `cos^2`-low-passed
   `bb * conj(r)` (1000 taps at 3 kHz = WSJT-X's 4000 at 12 kHz) with the truncated window renormalised, then
   `bb -= amp * r`. The window slides in O(1) per sample and holds 1002 samples, so a subtraction is about two
   references and a few complex multiplies per baseband sample.

## 3. Results

All at 8 kHz input (12 -> 8 -> 12 kHz), band 200-3000 Hz, `sync_min` 1.3, OSD on, 4 coarse bands, groups of 4.

### ft8_lib's 22 recordings, against `jt9` (402 decodes)

| | decodes of `jt9`'s 402 | extras | audio read (at 8 kHz) | peak heap above the audio |
|---|---:|---:|---:|---:|
| ft8_lib | 260 | 5 | | |
| `ft8dec`, 3 passes | 333 (82.8 %) | 14 | 720 KB | 234 KB arena |
| A, 15 / 30 candidates, f32 | 225 / 274 | 0 / 0 | 1 920 / 2 880 KB | 0.52 MB |
| A, 15 / 30 candidates, fixed-point | 227 / 279 | 1 / 2 | 2 150 / 3 110 KB | 0.22 MB |
| **C**, one pass, 30 cand., f32 / fixed | 272 / 279 | 1 / 2 | **240 KB** | 0.59 / 0.33 MB |
| **C**, 3 passes, 15 cand., f32 / fixed | **334 / 345** | 4 / 4 | 240 KB | 0.59 / 0.33 MB |
| **C**, 3 passes, 30 cand., f32 / fixed | **353 / 350** | 4 / 5 | 240 KB | 0.59 / 0.33 MB |

Measured (`icap-profile --band 4 --band-coarse`, `--band-sic 3`, `tests/band_baseband.rs`). The audio is read exactly
once for every file. Two checks separate the pieces: with the coarse search still on the audio and only the
candidates' `cd0` cut from the baseband, the decoded messages are **identical** to A's (30 candidates, f32 and
fixed-point, every file), so the cut loses nothing; with the coarse search on the baseband too, the sets differ by a
few (f32, 30 candidates: 3 messages only A finds, 2 only C finds, and a splatter copy kept in place of the real
signal, net 272 against 274; fixed-point 279 against 279).

### `qso3_busy.wav` (WSJT-X's and JTDX's 20)

| | of 20 | extras | time |
|---|---:|---:|---|
| ft8_lib | 8 | | |
| `ft8dec` | 16 | | 38 ms |
| A, one pass | 14 | 0 | |
| A + exact audio subtraction (3 passes) | 19 | 0 | about 14 s |
| **C, 3 passes** (15 and 30 cand., f32 and fixed-point, 8 and 12 kHz) | **19** | **0** | **59-71 ms**, of which subtraction 22 |
| stock host decoder, one thread: Fast / Normal / Deep | | | 60 / 205 / 399 ms |

The one missed, `WA2FZW DL5AXX RR73`, is the entry the golden marks as possibly a JTDX false positive; the exact
audio subtraction misses it too. (Times: separate runs on one Mac, not alternated.)

### Cost, `qso3_busy.wav` (measured, `/usr/bin/time -l`, 21 runs minus 1)

| instructions per slot | 15 cand. f32 / fixed | 30 cand. f32 / fixed |
|---|---:|---:|
| A, one pass | 910 / 871 M | 1 460 / 1 453 M |
| C, one pass | 885 / 866 M | 1 254 / 1 142 M |
| C, 3 passes | 2 042 / 1 764 M | 2 779 / 2 594 M |
| `ft8dec`, 3 passes | 929 M | |

On the 22 recordings the candidate stage is 2.7x faster (122 ms against 333 ms, 15 candidates) and the whole slot
2.3x (297 ms against 692 ms), but by instructions only 3-21 % of it is the arithmetic: **most of what C saves is the
audio reads, not the multiply-adds.** C with three passes costs about 1.9x `ft8dec`'s instructions for 3 more of the
20 and 12 more of the 402.

Subtraction on the 22 recordings: 0.39-0.41 s in all (18 ms a slot) against 70 s for the exact audio-path
subtraction at 15 candidates (a different, older configuration; read it as two orders of magnitude, not a ratio).
On a clean synthetic signal both remove 40.2-40.3 dB of the energy (the ramps at the ends of the frame and the
quarter-second amplitude filter set the floor), so the baseband version loses nothing there.

### Memory and code

- The baseband is 180 KB whatever the input rate. With it, the decode needs 0.33 MB (fixed-point) and 0.59 MB (f32,
  whose band spectrogram is twice the size). More bands and groups of 1 bring fixed-point to 0.26 MB (16 bands) with
  the same 345; it does not go below the baseband plus about 80 KB. **The ICAP data region (240 KiB) is not met by
  C**; A's 0.22 MB is.
- ARM Thumb-2, Cortex-A9, `opt-level=z`, LTO, `panic=abort`, a stub FFT, OSD on, linked from `decode` only: **A
  103.6 KB** (text 78.3 + rodata 25.4), **C with three passes and subtraction 107.5 KB** (+3.9 KB). The 74 KB quoted
  in `ICAP_EMBEDDED_ANALYSIS.md` does not reproduce with this build (a different configuration; not found again).
  Either is over ICAP's 80 KiB window.

## 4. What was verified, and how

Each new test was made to fail once by mutating the code it guards (`band_bb.rs`):

| test | mutations it caught |
|---|---|
| `cd0_from_the_band_is_the_direct_cd0` (error < 1 %, measured 0.04-0.2 %) | a delay of one sample in either stage, a 1 Hz centre error, a x1.1 scale, a 21-tap stage 2 |
| `the_band_spectrogram_is_the_audios` (main lobes 6 %, measured 3 %; noise power 10 %, measured 1.5 %) | the centre bin +1, scale x1.15, a frame of 3/4 length. **Not caught: a frame grid off by one baseband sample (0.33 ms), which moves the cells by under 0.5 %.** |
| `subtracting_a_clean_signal_takes_it_out` (within 1 dB of the audio path) | a reference 0.5 Hz off, 3 Hz off, a start one baseband sample late, no end correction, a window phasor of the wrong sign, an amplitude x0.9 |
| `the_sliding_window_is_the_direct_convolution` (2 LSB, measured 0) | the same last four |
| `the_baseband_does_not_depend_on_how_the_audio_arrives` | a phasor restarted per chunk |
| `tests/band_baseband.rs`, `icap_budget.rs` | floors on the 22 recordings and `qso3_busy`; one read, no restart, no skipped sample; the heap ceiling |

The merge gate (tier A+B) is 1422 passed, 0 failed (1417 before, plus these 5). The embedded configuration (no
`fft-rustfft`, f32 and fixed-point) passes its tests, the 7 `no_std` feature rows tried build, clippy
(`--workspace --all-targets`, CI's invocation) and `cargo doc -D warnings` are clean for this code.

## 5. Things the work turned up

- **The ESP32 boards do not build at `main`** (since `2ad660fe`, #761, 2026-11-23): `ft8/stage_timing.rs` uses
  `AtomicU64`, which Xtensa lacks. Host CI never compiles `embedded-poc`, so nothing noticed. With that one type
  swapped, temporarily and since restored, `cargo check --release` passes for `m5stack-cores3-app`, `-s3-app`,
  `-core2-app` and `m5stack-s3` **with this work's changes**. Not flashed.
- `band_bb` did not build `no_std` at first (`f32::round` and `f64::cos` need `num_traits::Float` there). The host
  tests do not see that; an ARM cross-build did.
- **The extras of the subtraction passes** (decodes `jt9` lacks; f32 has 4, fixed-point 5 at 30 candidates). Three are
  listed by `ft8dec` too, at the same frequency (`<...> DF1XG JO53`, `UA9SIX GM0LIR R-07`, `CQ UT9LB KN89`: now in
  `ATTESTED_EXTRAS`), two are a splatter copy of a strong signal (`K4VBM HA8EK -15` 150 Hz above `jt9`'s, and
  `RR73` of the same pair, already attested), one was attested before (`EA8BEV LU3DW RR73`), and **two are listed by
  no other decoder: `EA8PP JH0INP PM96` (websdr_test7, f32) and `EA8ESB JM5WSV OP12` (websdr_test5, fixed-point), both
  near 940 Hz, both found in the subtraction passes**. Possible chance CRC matches; not investigated.
  `MAX_UNEXPLAINED = 1` holds the tests to it. A new decode strategy needs its precision guard before it ships
  (`CLAUDE.md` §4); this is not that.
- Keeping "the first found" of a message, as A does, can keep a splatter copy over the real signal.

## 6. What is not known

- **Any figure on a real Cortex-A9, A53 or RP2350.** The instruction counts are the only portable number; the cache
  and DRAM effects that decide a Pi 3B are not in them. The one measurement worth asking for is
  `FT8_SLOT_PROFILE.md` §1's three commands on a Pi 3B, plus `cargo run --release -p icap-app-hosttest --bin
  icap-profile -- --max-cand 15 --low-mem 4 --band 4 --band-sic 3 --wav embedded-poc/assets/qso3_busy.wav --rate 8000`.
- Whether C's 0.33 MB (fixed-point) / 0.59 MB (f32) fits a Pi 3B's L2 (512 KB, shared): the baseband is read through
  twice per candidate and per pass, so probably the working set is smaller than the peak; **a guess**.
- The FT4, FST4 and WSPR forms. FT4's `NDOWN` is 18 at 32 samples a symbol, FST4's DDC configs exist unused
  (`fst4::ddc`), WSPR's decoder already works on a 375 Hz baseband and subtracts there
  (`wspr::subtract::subtract_signal_baseband`); the pieces that carry are the streaming input, the band baseband, the
  baseband spectrogram and the sliding-window amplitude. Q65, JT9, JT65 need a spectrogram reader on the baseband and
  per-symbol FFTs of it; not looked at beyond reading their entry points.
- The effect of the baseband's edges (the last ~100 Hz of each end): the signals there are not in the golden sets.
- Sniper (a narrow search around one frequency) and AP are not in this path.

## 7. If this is adopted: a sketch, not a decision

1. `BandStream` / `BandBaseband` and the `AudioSource` family move to `engine/` and take the protocol's constants
   (`NSPS`, `NDOWN`, tone spacing) instead of FT8's; the FIR design is derived, not copied.
2. `cd0_fir` and `engine/dsp/ddc.rs` share their filter stages (today three copies of the same cascade logic:
   `Stage`, `FirStage`, `StreamingComplexDdc`).
3. The reference for subtraction needs a complex-phasor accessor on `GfskStream` (`peek_complex`, `skip`: added here,
   public, small) and a `Protocol`-driven version of `BandBaseband::subtract`.
4. The precision guard (§5), a dedupe that keeps the stronger copy, and the Pi/RP2350 measurement come first.

## Files

- New: `mfsk-core/src/ft8/decode_block/band_bb.rs` (`#[doc(hidden)] pub`, experimental), `hosttest/icap-app/tests/band_baseband.rs`.
- Changed: `ft8/decode_block/{coarse_sync,process_candidates,spectrogram,cd0_fir}.rs` (the candidate loop and the band
  scorer are now shared by A and C; A's output is bit-identical), `engine/dsp/gfsk.rs` (`peek_complex`, `skip`),
  `hosttest/icap-app/{src/profile.rs,src/bin/icap-profile.rs,tests/icap_budget.rs}` (`--band`, `--band-coarse`,
  `--band-sic`; method C's budget). The plan said a new `hosttest/midrange-eval` crate; the heap meter, FFT shim,
  WAV loader and scoring are in `icap-app-hosttest`, so C went there instead of copying them.
