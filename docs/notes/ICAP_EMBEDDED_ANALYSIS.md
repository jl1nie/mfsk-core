# ICAP and the embedded line: why `ft8dec` and mfsk-core differ, and what to carry to the next mode

2026-11-25. Written after the first measured comparison of the ICAP app's own FT8 decoder (`ft8dec.c`,
icap-sdk `apps/ft8`) with mfsk-core's low-memory profile (`decode_block_low_mem` / `_src`, `hosttest/icap-app`).
**Follow-up (same day): `MIDRANGE_EMBEDDED_EVAL.md`** carries out §6's "cheap subtraction" and "fewer reads" on one complex baseband of the whole band: the audio is read once, subtraction costs 18 ms a slot, and 345 of `jt9`'s 402 are decoded (`ft8dec`: 333). **Its code-size figures replace §3.5's**: built the same way as that note's crate was, A comes to 103.6 KB, not 74 KB, and the 74 KB below could not be reproduced.

Everything below is **host** data (Apple silicon, one thread) unless it says otherwise: nothing here was run on a
Cortex-A9. What was measured, what was derived from the code and what is a guess is marked.

Related: `docs/reference/EMBEDDED.md` (the ESP32 line), `CORES3_FT8_SLOT_BUDGET.md`, `JTTY_EMBEDDED_BUDGET.md`,
`FT8_REAL_VS_JT9.md` (the recordings and the `jt9` reference used here).

## 1. The platform

| | ICAP (IC-705 / IC-9700) | ESP32 line (`embedded-poc`) |
|---|---|---|
| CPU | Cortex-A9, NEON, 4 KiB-class stack, no malloc | Xtensa LX6/LX7, esp-dsp FFT/dot-product asm |
| RAM for the app | 80 KiB window (code) + 240 KiB data region (heap) | 8 MB PSRAM on CoreS3 / StickS3 |
| audio | 8 kHz, on the SD card, read forward (`icap_read`), no seek: a read behind the position closes and reopens the file | 12 kHz, in RAM (the slot is 360 KB) |
| time | decode of the previous period while the next is captured; cut off if the next capture finishes first | decode in the 2 s after slot end, before key-up |
| source | `icap-sdk`: `README.md`, `apps/q65/README.md` (the memory map), `apps/digi-common/digiapp.c` (`src_read`) | `EMBEDDED.md` |

The ESP32 design assumes PSRAM: the spectrogram (360 KB) and the audio live there. ICAP has neither. That is the
root of most of what follows: **the ESP32 pipeline is a "slot in RAM" pipeline, ICAP needs a "slot on SD" one.**

## 2. The comparison (qso3_busy.wav and the 22 recordings)

`qso3_busy.wav` is WSJT-X's own sample `210703_133430.wav`, 20 signals known from WSJT-X and JTDX.

| | ft8_lib | ft8dec (3 passes) | mfsk-core, ESP32 driver | mfsk-core ICAP profile (30 cand., OSD) |
|---|---:|---:|---:|---:|
| qso3_busy, of 20 | 8 | **16** | 7 | 14 (19 with exact 3-pass subtraction) |
| 22 recordings, of `jt9`'s 402 | 260 | **333** | 131 | 279 |
| instructions, qso3_busy | | 929 M | | 1446 M (868 M at 15 cand.) |
| SD bytes moved, qso3_busy (8 kHz) | | **720 KB**, 564 calls, 2 restarts | | 3.1 MB, 2284 calls, 12 restarts |
| heap above the audio | | 232 KB (arena) | 1.29 MB | 220 KB |
| code (decode only, Thumb-2) | | not measured | | 74 KB (`.text` 59 + `.rodata` 15); **103.6 KB when re-measured 2026-11-25, see the follow-up** |
| wall, host | 12 ms | 38 ms | 35 ms (per file mean) | 47 ms |

Measured with: `scratchpad` harnesses (`ft8dec` host driver with a `CountingSource`-equivalent model; `icap-profile`;
`/usr/bin/time -l` instruction counts, difference of 21 and 1 repetitions). Reproduce the mfsk side with
`cargo run --release -p icap-app-hosttest --features fixed-point --bin icap-profile -- --low-mem 4 --max-cand 30 --stream 4`.

## 3. Why: the differences, in order of effect on the radio

### 3.1 SD traffic: 4x (720 KB against 3.1 MB). **The largest gap, and the one most likely to decide the radio.**

*Measured:* ft8dec reads the slot 3 times (360 201 samples, one pass per decode pass). mfsk-core's profile reads it 13
times: 1 (the fixed-point gain scan) + 4 (one per coarse band) + 8 (one per group of 4 candidates).

*Cause, from the code:* what each design **keeps in RAM between the phases**.

| | ft8dec | mfsk-core |
|---|---|---|
| after the first read | a **waterfall of 8-bit log magnitudes**: 456 bins x 93 blocks x 2 (time) x 2 (frequency) = 169 632 cells x 1 B = **166 KiB**, the whole band | the spectrogram would be 977 bins x 184 frames x u16 = **351 KiB**: it does not fit, so it is built a band at a time and dropped |
| what the decode reads | the waterfall: candidates are scored (`hard sync` of 21 Costas symbols), their LLRs made, BP/OSD run, all from it | the audio again: each candidate's complex baseband is built from the samples |
| per candidate | nothing kept | a 200 Hz complex baseband, 3 200 x f32 = 25.6 KB (12.8 KB as i16) |

So ft8dec's representation is **lossy but complete for the band** (magnitudes, 0.5 dB steps, 2x oversampled in time
and frequency), and small enough to hold. mfsk-core's decode needs **complex** symbol spectra (phase), which the
magnitude spectrogram does not hold, so it goes back to the audio. Two choices follow from that and each costs reads:
- a u16 linear cell (2 B) where a u8 log cell (1 B) would do for the coarse stage: this alone is the 4 bands;
- per-candidate baseband built from the audio: the groups. Holding 4 at a time (the 240 KiB region) gives 8 reads.

*Not a cost of the algorithm, a cost of the architecture.* Cutting it: fewer coarse passes (a u8 log spectrogram is one
pass; or 2 bands; or the gain from the capture, which the app already sees), i16 basebands so that 8 candidates
fit a group (4 reads), and subtraction applied on the fly while a pass reads (ft8dec's way: **zero extra reads**).

### 3.2 Compute: 0.9x to 1.6x the instructions, with fewer signals decoded

*Measured:* 868 M (15 candidates) and 1446 M (30) instructions against ft8dec's 929 M, which includes its 3 passes and
its subtraction. Per decoded signal ft8dec is cheaper (2.3 ms against 3.4 ms on the host).

*Where mfsk-core's go (measured, 30 candidates, host):* coarse + hard sync 12.3 ms (4 bands; 4.0 ms with 1, so 8.5 ms
is frames FFT'd again for each band), `fine_sync` 1.0 ms (before it moved onto the baseband), stage 3 about 34 ms of
which the FIR baseband about 20 ms and OSD 5 ms.

*Per candidate, derived from the filters:* the FIR cascade is about 3.0 M multiply-adds: 0.61 M the mixer (12 kHz x
complex), **1.52 M the first stage (15 taps at 12 kHz, 50 %)**, 0.63 M the second, 0.28 M the third.

*Causes:*
1. **Per-candidate DSP.** ft8dec has none: a candidate is a handful of table reads in the waterfall. mfsk-core pays
   about 3 M MAC for each of 15-30 candidates: this is the price of coherent processing (fine sync with a frequency
   search on the baseband, complex LLRs, OSD on good LLRs) and of staying close to WSJT-X.
2. **12 kHz against 8 kHz.** ft8dec is native 8 kHz (2560-point FFT, 1280-sample symbols). mfsk-core's `Protocol` is
   12 kHz throughout, so every 12 kHz-rate loop (the mixer, stage 1, the spectrogram frames, the FFT size 3840 against
   2560) does 1.5x the work, and the host harness adds a 12 -> 8 -> 12 kHz resample that a radio would replace by a
   cheap interpolator. *Derived, not measured:* going native would cut the 12 kHz-rate part of the work by a third.
3. **The bands.** 4 coarse bands cost 8.5 ms in repeated frame FFTs for memory that a u8 spectrogram would not need.
4. **No subtraction yet** in the profile: ft8dec's figures include it; ours add 7-14 s of exact subtraction (below).

*Not a cause:* IPC. The host runs mfsk-core at IPC 7.1 and ft8dec at 5.6; a Cortex-A9 is a 2-issue core and will not
show that. **Use the instruction ratio, not the cycle ratio, to predict the radio.**

### 3.3 Recall: ft8dec +2 on qso3_busy, +14 points on the 22; mfsk-core better per pass

*Measured:* ft8dec decodes 12 signals in pass 1 and 4 more in passes 2-3 (after subtracting what it had). mfsk-core's
profile decodes 14 in one pass, and 19 with three passes of exact subtraction. So **the whole gap is subtraction**:
mfsk-core is ahead per pass (OSD, a frequency search on the baseband, hard-sync candidate choice) and behind with no
affordable subtraction. The 4 signals ft8dec gets in later passes are the ones next to strong stations
(`KD2UGC` 6.8 Hz from `N1PJT` at -2 dB, `EA2BFM`, `EA3CJ`, `DL5AXX`).

*The subtraction that exists is the wrong shape for the radio:* `ft8::subtract::subtract_signal_lpf` correlates and
filters the 12 kHz samples (about 600 M MAC per signal: 0.4 s each on the host, 14 s for the 19 signals of
`qso3_busy`), holds the audio in RAM and rewrites it. ft8dec removes each signal while the next pass reads: per symbol
it correlates a GFSK reference with the signal, interpolates the complex amplitude between symbol centres and
subtracts, streaming, with a 1.5-symbol lag.

### 3.4 Memory: equal, by different routes

Both hold about 220-230 KB. ft8dec holds more information (the whole band's waterfall) in the same space because its
cell is 1 B; mfsk-core holds a band and re-reads. The ESP32 driver's 1.29 MB was never meant for this region
(`decode_block`: u16 spectrogram 360 KB + a 660 KB coarse accumulator + 226 KB `sync2d`, before the changes
below; the spectrogram is still the one array that cannot be shrunk by restructuring).

### 3.5 Code size: not yet comparable

mfsk-core's decode path alone is 74 KB in Thumb-2 (`opt-level=z`, LTO, `panic=abort`; ARM mode 97 KB) with a stub
FFT: the 80 KiB window is nearly gone before the FFT, the app shell, the transmit encoder and the C library. The
biggest function is `process_one_candidate_inner` (18 KB). `core::fmt` pieces remain (several KB). Q65.APP gets its
Rust decoder into the window with `build-std`, `panic=immediate-abort` and a split image (read-only data in the data
region). ft8dec's own size was not measured. **Open.**

## 4. What was changed in this round, and what each was worth

All measured on the host, `qso3_busy` and the 22 recordings (jt9 reference), fixed-point unless noted.

| change | effect |
|---|---|
| coarse sync: no `sync2d` grid, `allsum` per carrier | peak heap 1.29 -> 0.45 MB, **bit-identical** output |
| spectrogram a band at a time (`coarse_sync_banded`) | 0.45 -> 0.16-0.20 MB, bit-identical, +8.5 ms (4 bands) |
| no block-0 spectrum kept per candidate | heap no longer grows with `max_cand` (1.54 MB at 300 -> 0.20), bit-identical |
| candidates ordered by hard sync read from the spectrogram (`coarse_sync_gated`) | 22 recordings 32.6 -> 58.0 %, qso3 7 -> 10 (with refinement) |
| OSD in the embedded build (`osd` feature) | 62.2 -> 67.2 % (22 rec.), qso3 10 -> 14, no new phantoms |
| 200 Hz baseband by FIR cascade in place of Goertzel DFTs (`cd0_fir`) | 2.3x faster stage 3, recall equal or better (270 -> 282 of 402) |
| stage A-C refinement on the baseband (`fine_refine_3stage`), no audio access after the baseband | recall -3 (282 -> 279), the audio is read only by the coarse passes and the group builds |
| streaming through `AudioSource` (forward reads, counted) | SD 12.6 MB -> 4.7 MB per slot at group 4; recall unchanged |

*The first FIR design was wrong in a way a clean test did not see:* 9 and 17 taps left 990 Hz only 28 dB down (a hazard
frequency that folds onto the tones). Strong, clean test signals decoded through it. What caught it was a frequency-
response test at the aliasing frequencies (`cd0_fir.rs`), and a weak-signal count that moves with the filters.

## 5. Rules for the next embedded work (FT4, WSPR, Q65, JTTY)

1. **Decide what stays in RAM between the phases first, and count the reads that follows.** A representation that
   holds the whole band (ft8dec's u8 log waterfall) makes every later stage free of the SD; one that holds only
   candidates (mfsk-core's basebands) pays a read per group. Write the budget as `reads x slot bytes`, not as a heap
   size. The `CountingSource` model (read, skip to a read ahead, restart behind) is in
   `ft8/decode_block/source.rs`; `hosttest/icap-app/tests/icap_budget.rs` holds it as a test.
2. **Hold a ceiling for the heap and the reads in a test from the start**, per recording, on the build the target
   ships (`fixed-point`). A heap meter is the process's allocator: such a test needs a binary of its own.
3. **Subtract while reading**, not in a copy of the audio: ft8dec's pass 2 is the same read as its pass 1 with the
   found signals removed on the way. A subtraction that holds and rewrites the slot is a non-starter for a region
   that cannot hold the slot, and 0.4 s a signal on a host is minutes on the radio.
4. **Take the native rate of the target** (8 kHz here) where the protocol code allows it. A rate conversion in front
   of a 12 kHz decoder costs compute twice (the conversion, then 1.5x the samples).
5. **Rank candidates cheaply and widely, evaluate few.** The coarse score ranks the ghosts of a strong signal above
   weaker real ones (`qso3_busy`: first 30 by score hold 9 real signals, by hard sync 21; measured once by hand, no test pins it). A hard-sync count read from
   the spectrogram (no DFT, no audio) is the gate; both ft8dec and `jt9`'s `ft8b` have one.
6. **Test filters at the frequencies that alias, and test sensitivity near the threshold.** A clean, strong signal
   passes a wrong filter and a wrong symbol grid. Mutation-check the new tests (this round: 6 filter/grid mutations;
   5 caught by the response, Goertzel-agreement and weak-signal tests; the 6th, a 1.7 % scale error, does not change
   a decode).
7. **Mind the configuration matrix.** Two things broke the workspace-wide `clippy` that CI runs and were found only by
   running it: a function gated on `not(fft-rustfft)` that a host-only test crate used, and a `let` of a unit value
   in the f32 build. `cargo clippy --workspace --all-targets --features full,internal-testing` is the check.
8. **Do not trust an estimate of "how much faster" until it is timed.** The first estimate of the FIR path was 16x
   from a count that left out the mixer and the first stage (about 70 % of the work); the measured gain was 4.7x on the
   stage and 2.3x on the decode.

## 6. Open

- **Cortex-A9 numbers.** Nothing here ran on one. The only anchor is ft8dec's "a few seconds" on the IC-705 against
  38 ms on the host. Scaling by instructions gives 3-5 s for the profile without subtraction; scaling SD time by the
  4x in section 3.1 is the larger and the less certain term. **The SD read speed of the radio (`icap_read`, 512-sample
  calls) is the single most useful number to get.**
- **A cheap subtraction** (3.3): amplitude from the 200 Hz baseband, reference synthesised and removed while a pass
  reads. Target: the 4 signals of `qso3_busy`, a few tens of ms.
- **Fewer reads** (3.1): the gain from the capture, 2 coarse bands or a u8 spectrogram, i16 basebands, group 8.
- **Code size** (3.5): `build-std` with `panic=immediate-abort`, the split image, and what OSD (3-4 signals, about
  10 KB?) is worth in a window of 80 KiB. The ARM figures above are for a stub FFT.
- **8 kHz native.** `Protocol` constants are 12 kHz; the FIR cascade and the spectrogram would take an `NDOWN` and an
  `NSPS` from them (FT4: `NDOWN` 18, 32 samples a symbol, as FT8's 60 and 32).
- **FT4, WSPR, Q65 on the same footing.** The pieces that carry: `AudioSource`/`CountingSource`, the heap meter and
  `--diag`, `cd0_fir` (parameterised by the protocol's decimation), the hard-sync gate (by `SyncMode`), the OSD gate.
