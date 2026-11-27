# Changelog

## Unreleased

### OSD capped per slot: `CellsWorker::with_osd_deferred` + `osd_pass` (off by default; a divergence, Issue #766; host, 4 780 slots)

- The attempts that reach the OSD fallback are queued with an `OsdRank` score (a logistic model on the LLR statistics, the hard syncs and the search score; `CELLS_OSD_RANK`, fitted on the generated sets, the 60 real slots held out) and `osd_pass(cells, k, ..)` runs the best `k` after the slot's BP work. Unbounded it decodes what the OSD at once does (test), `k = 0` what the flow does without OSD.
- Host (7 397 true decodes with the OSD at once, 6 931 without; real slots 1 024 / 974): K = 4 / 6 / 8 / 10 runs the OSD on 15 / 23 / 31 / 38 % of the attempts and keeps 28 / 33 / 38 / 44 of the 50 decodes the OSD adds on the real slots (7 308 / 7 347 / 7 371 / 7 381 over all). The OSD pass is the only way found to fit the window: the OSD at once costs 1.2 s a slot on the CoreS3.
- The pre-existing `ft8dec_sic.rs` needless borrow under `fixed-point` clippy is fixed.

### OSD seeds `a` and `d` only (`OsdSeeds::AD`, off by default; a divergence, Issue #766; host, 143 400 candidates)

- `CellsWorker::with_osd_seeds(OsdSeeds::AD)`: the OSD fallback runs variants `a` and `d` at both snapshots (4 runs) in place of `ft8b.f90`'s ten. Variants `b`, `c`, `e` never returned on 4 780 slots (600 returns): **0 of 143 400 candidates differ** (7 442 decodes, 7 397 in the reference, both ways).
- Host instructions of the OSD fallback (qso3_busy / qso1 / qso2, OSD on): `try_fallback` 319 / 286 / 337 -> 119 / 104 / 122 M (-63 %); the f32 LLR is not recomputed (the BP stage's one-symbol `llra` / `llrd` are the full set's, test `the_ad_llrs_are_the_full_ones`).

### FT8 OSD for a 32-bit core: `osd174_91_into`, `osd174_91`'s answer bit for bit (host tests, board `ft8-osd-bench`)

- **New** `fec::ldpc::osd_emb::{osd174_91_into, Osd174Scratch, OsdHit}`: `ndeep = 2` over `u32` rows, the generator's columns from a compile-time table, the caller's memory (about 4.5 KB, kept in the `BpScratch`) and a distance that stops once it cannot win. `try_fallback` uses it, with the f32 BP scratch of its `zsave` inputs kept too: the fallback allocates nothing after its first run.
- **Equal to `osd174_91` on every input** (`tests/osd_embedded.rs`: the upstream fixture's inputs and 22 000 synthetic ones, with ties and a-priori masks; seen to fail with an early stop, a stable sort, a short window, `ntheta` 11). The host's instructions are 7 % *more* (x86-64 runs two `u32` ops for one `u64`); the board is 28 % faster.
- CoreS3 (44 real OSD inputs, `ft8-osd-bench`): **10.09 -> 7.26 ms a call, 44 of 44 equal, stack 9.65 KB -> under the baseline**. A failing candidate still runs 10 of them (5 LLR variants x 2 `zsave`): about 70 ms of OSD a candidate, so OSD does not fit the window as the staircase calls it (design: a single call, plan 2026-11-27).

### Cells flow on the CoreS3: a faster search and a faster BP, bit for bit (host and board, `ft8-stage-bench`)

- `band_best` scores a carrier at a time (`carrier_scores`): the neighbour terms of a Costas symbol are made once for every row of the carrier and each of the 60 positions adds up 21 entries; the scores are the old ones at every position (test over every position, 184 rows and nine shorter fills). Host 63.8 -> 42.4 M instructions, 14.0 -> 5.6 M conditional branches; **CoreS3 `band` 0.90-0.94 s -> 0.46 s** (one core; CPU-bound: the same per position over a narrow and the whole range).
- The NMS decoder reads edge tables made at compile time (`LdpcParams::NMS_EDGES`; every index range-checked at compile time) in place of searching the code's tables on every pass: **76 000 -> 37 600 host instructions a BP pass** (`scripts/bp-instr-per-pass.sh`), the results bit for bit the old decoder's (the old one is kept in the tests; f32 and Q11i16, 174,91 and 240,101). CoreS3: a BP pass 0.68-0.75 -> 0.41-0.44 ms (one fit with a large intercept gave 0.26), stage 3 at the `d` floor 0.93 / 0.71 / 0.74 -> 0.62 / 0.48 / 0.46 s; the compile-time tables are about 6 % slower per pass than a first version's heap tables and keep 4.6 KB of internal DRAM free (plan section 12a). `unsafe` reads with the lengths asserted once.
- With the `d` floor (#765) the cells flow's 30 candidates end at +0.27 / +0.22 / +0.20 s after the slot boundary on one core, 15 at +0.00 / +0.13 / +0.10 s (an estimate, plan section 13; the app's other load is not measured). Detail and the predictions: `S3_CELLS_IMPLEMENTATION_PLAN.md` sections 12-13.

### Cells flow: variant `d` only from score 16 up (Issue #765; host, measured over 4 780 slots and the SNR ladders)

- **New** `CellsWorker::with_d_min_score` and `CELLS_D_MIN_SCORE` (16): a candidate of lower search score gets the staircase's `llra` only (grid and retry); `CellsWorker::new` keeps the whole staircase. `decode_block_cells` and `decode_block_cells_sic` set 16: **-49 % of the BP passes over the set, -24.5 % on the 60 real slots, 4 of 6 935 decodes lost (1 of 975 real), no ladder slot lost**; `Decoder<Ft8>`, `DecodeDepth` and every WSJT-X-shaped path are untouched (`PassCtx::skip_d`, internal). A divergence from `ft8b` (it tries all variants), like the flow itself.
- Instructions at 30 candidates, OSD off, no AP (x86, `scripts/s3-stage-instr.sh`): `qso3_busy` / `qso1` / `qso2` 312 / 286 / 303 -> 276 / 239 / 249 M (stage 3's BP 139 / 115 / 131 -> 103 / 68 / 77); decodes 9 / 4 / 5 unchanged. Board time not measured (prediction: 0.2-0.4 s a 30-candidate pass).
- Test `a_worker_skips_variant_d_under_its_score_floor` (identical decode and BP stats from the floor up, half the calls on a failing candidate under it; seen to fail with the floor not applied and applied to every candidate).

### Cells flow: which BP work is wasted (host, board's build; design note sections 26-27)

- **New** `BpStats` (`calls`, `decoded`, `passes`; `#[non_exhaustive]`) on `BpScratch` (`stats`, `take_stats`) and `CellsWorker::{bp_stats, take_bp_stats}`: the message-update passes BP ran on that scratch, the unit its time scales with (the Mac and x86 counts of section 26 were by probe). Plain integer adds at the exits of the NMS and kind decoders; no result changes. Not counted: the `zsum` entry points, the host build's AP pass and OSD (the boards' build has neither pass). Tests: exact counts at the decode and loop-end exits, a pinned table at the stall exits, a rejected converged word; each seen to fail with its exit off by one. The 12 kHz resampler golden test that had failed on CI since the 8 kHz decimator commit is fixed (its constants were wrong, not the code).
- `icap-profile --no-osd`, `scripts/s3-stage-instr.sh` (instructions per stage, old S3 against the cells flow), `icap-cands` + `scripts/cands-yield.py` (what each candidate costs and decodes: 60 slots, 1 800 candidates, 975 decodes in the reference).
- Measured (design note section 27): on `qso3_busy` at 30 candidates (x86) the cells flow's excess over `ft8dec` is all BP (139 M against 24 M instructions; the spectrogram and search sides 168 M against 179 M); on the 60 real slots at the same knobs `ft8dec` decodes 822 against 975 for 16.9 passes a decode against 78.1; **95 % of the 76 155 passes go to candidates that do not decode**. Over 4 780 slots and the SNR ladders: a score floor loses decodes at the sensitivity edge, a hard-sync floor of 8 loses none but saves nothing on a busy real band, and the staircase's `d` from score 16 up (retry kept) saves 49 % of the passes for 4 of 6 935 decodes (Issue #765). Nothing applied; the real slots are the tuning set.

### Cells flow against ft8dec on Apple Silicon: where the BP calls go (measurement only; design note section 26)

- The cells flow is 3.3x `ft8dec`'s instructions (4.4x its wall time) at 30 candidates / OSD off on an M5, the x86 direction, not a reversal. BP is 60 % of the time: 87 candidates reach BP against `ft8dec`'s 35 (68 failed against 16), 2.7 calls against 2.0 on a failed one, 29.6 iterations a call against 24.1. A BP cap of 25 (not applied) keeps every decode on 60 slots and saves 2-3 % of instructions. The x86 record's 179 BP calls are these 63 plus 116 of the host build's blind-CQ AP pass, which the board does not have, so the 3.3x includes that pass (x86 with it off: 1.54x).

### CoreS3: the cells flow's first board runs, and the ft8dec comparison (`S3_CELLS_IMPLEMENTATION_PLAN.md` §9, steps 1-2)

- **New** (embedded, `fixed-point`): `esp_dsp_fft` plans a 2 560-point sc16 transform (`MixedRadix2560Sc16Fft`, 256 x 10 on the esp-dsp kernel); `fft_mixed_2560_sc16::build_twiddles` is `pub`. `ft8-cells-bench` (CoreS3): esp-dsp against the software plan differs by up to 83 LSB (mean 3.1; the shipped 3 840 plan: 123 / 3.7), a row pair is 7.0 ms of the 160 ms it covers, and `decode_block_cells` decodes the same sets as the host on 3 recordings (2.25 s at 15 candidates, 3.06 s at 30, one core).
- Against ft8dec (icap-sdk 1ca9f04) on the same 8 kHz audio: equal decode counts at equal knobs (30 candidates, OSD off: 19 / 19 on 3 recordings). On x86 the cells flow takes **1.54x the instructions at one pass and 1.31x at two** with the AP pass off (the board's build; the host build runs a blind-CQ AP pass the board has not, which made it 2.55x / 1.88x: the "BP is the gap" reading is withdrawn); on the CoreS3 it takes 1.96-2.37x the time at one pass, 1.54-1.70x at two, SIC 1.6-1.9x the plain flow. The design note's Apple Silicon figures point the other way and are not explained; `scripts/compare-ft8dec-cells.sh` repeats the measurement on a Mac. Details: the plan, section 10.
- **New** `CellsWorker::decode_round` (a worker over the caller's memory serves every round; `decode_block_cells_sic` makes its worker once for all passes); output unchanged (22 recordings, the three recordings, `decode_block` tests). Doc fix: `process_candidate_with_scratch` is not allocation-free (small LLR buffers per attempt; with `fft-rustfft` the AP pass makes a BP scratch).
- `decode_block_cells_sic` keeps **one `HalfDbCells` for the whole slot**: each later pass's residual cells are written into the first pass's store (`search_near_into`, `spectrogram_cells_into`; the demux writes into a caller's buffer), no 172 KB store made per pass. Cells and candidates equal the allocating path's, whatever the store held (a test fills it with a pattern); 22 recordings, 1-3 passes, unchanged.
- **New** `sic_passes` (the passes of `decode_block_cells_sic` over the caller's memory: the first pass's candidates and cells, the `CellsWorker`, the tables and a `SicScratch` are passed in), `SicStorage` (the ring, the chunk buffers, the symbol table and the signals' states, made once: `new(rate, sic_signal_capacity(max_cand, passes))`, lent by `scratch()`), `sic_signal_capacity`. `ResidualSource` no longer allocates (the signals are corrected in place, `finish`). `decode_block_cells_sic` is the allocating front end of it; output unchanged (22 recordings at 1-3 passes; a test runs four slots through one `SicStorage` and one worker and equals fresh memory to the bit).
- **New** `SigState` (opaque) and `SicScratch::new` (a scratch over the firmware's own slices: ring in internal DRAM, say). CoreS3 (`ft8-sic-bench`, one core, 30 candidates, OSD off): `sic_passes` over a `SicStorage` and the internal-DRAM `cs` staging takes the time of `decode_block_cells_sic` (qso3_busy 2 passes 5.99 s against 6.01 s, 3 passes 8.48 / 8.48 s) with the same decoded sets (11 / 5 / 5); the ring in internal DRAM instead of PSRAM is 0.8-1.7 % faster, so the ring's placement is not the board's extra 10 % over the host instruction ratio. `SicStorage` costs 2.6 KB of internal DRAM and 38.5 KB of PSRAM.

### FT8 at 8 kHz (fixed-point): what the CoreS3 firmware needs of the library (step 0 of `S3_CELLS_IMPLEMENTATION_PLAN.md`)

- **New** `CellsWorker` (+ `CellTables`, `Grid`, `Found`): `decode_block_cells`'s per-candidate step over the caller's memory (staging block, BP scratch, tables), the unit of the board's work-stealing loop; `decode_round` / `decode_block_cells` are built on it, decodes unchanged to the bit. A failing candidate allocates no `Box` / `Vec` of its own (the shared `engine::llr` scratch, <= 696 B, remains). `process_candidate_with_scratch` / `Ft8BpScratch` are `#[doc(hidden)]`.
- **New** `HalfDbCells::{new, row_mut, mark_filled, filled_rows}`, `search_cells` + `merge_cells_parts` (any split of the carriers gives `coarse_sync_gated_cells`'s list, bit for bit), `candidate_ready` (every row a decode reads is written): a store the board writes a row at a time, searched over the rows written. A complete store's search is unchanged to the instruction.
- **New** `LinearResamplerI16To8k` / `resample_to_8k` (Kaiser low-pass, pass 3 200 / stop 4 000 Hz, 40 dB, ~135 taps at 48 kHz); the 12 kHz outputs are bit for bit what they were (golden hashes). The 22 recordings through 12 -> 48 -> 8 kHz: 225 / 285 of `jt9`'s 402 at 15 / 30 candidates, the ICAP model 226 / 286.
- Not measured yet (step 0d): the cells flow without OSD, and the early emit at row pair 87. Nothing here has run on a board.

### FT8 at 8 kHz (fixed-point): `ft8dec`'s subtraction passes on the cells flow (`decode_block_cells_sic`)

- **New** `decode_block_cells_sic` (`fixed-point`, `Hz8000`): after the first pass, every decoded signal is subtracted **as the audio is read** (a port of `ft8dec.c`'s `sig_corr` / `sig_sub` / `build_wf`: the GFSK reference correlated per symbol, the complex amplitude interpolated between symbol centres, 1.5 symbols of lag, no copy of the slot, a ring of 8 192 samples, no FFT), the cells of the residual are made and only the carriers within +-18 cells of the new signals are searched and decoded; up to `passes` passes. Decodes report a refined place (`ft8dec`'s parabola on the 79 tones and `tcal`). `decode_block_cells` and the other flows are unchanged.
- 60 real slots, 30 candidates: `jt9` hits 1 001 -> **1 192** (2 passes) -> 1 206 (3); union of `jt9`'s and `ft8dec`'s lists 1 035 -> 1 247 -> 1 262; `ft8dec`: 1 189 / 1 204 hits at 2 / 3 passes. At 20 / 24 candidates one pass already gives 912 / 979 hits, two passes 1 099 / 1 162. Decodes in neither list 11 -> 20 -> 21 (of the 9 new at 2 passes: 2 read as real by a wrong-tone scan, 2 probably, 1 attested by `jt9` elsewhere, 2 read like noise (a subtraction ghost is the likely reading), 2 not checkable).
- **Cost** (host instructions a slot): 396 -> **881 M** at 2 passes (`ft8dec` 850 M), 1 443 M at 3 (1 278 M); at 15 candidates 180 -> 488 -> 859 M. A pass after the first decodes at most `max_cand / 2` candidates (8 at least; a divergence from `ft8dec`, which decodes 300 every pass): that took 314 M off two passes for 5 hits. Not ESP32 cycles; noise-only slots (tier C) not run. The number of passes is the caller's (`passes`).

### FT8 at 8 kHz (fixed-point): the S3's stage 3 from the half-dB cells (`decode_block_cells`)

- **New** `coarse_sync_gated_cells`, `HalfDbCells`, `decode_block_cells` (`fixed-point`, `Hz8000`): the search keeps its cells (a byte each, written by the demux, 172 KB) and the symbol spectra of a candidate are read from them, no per-candidate DFT, no `fine_sync_at`; a candidate that fails gets `ft8dec`'s second try at the place the Costas tones point to when that is >= 1/4 cell off. A deliberate divergence from WSJT-X; the existing flows are untouched. The u16 spectrogram is bit for bit what it was.
- 60 real slots (38 `20m_busy` + the 22 recordings), against the union of `jt9`'s and `ft8dec`'s lists (1 409): found 765 / 1 035 at 15 / 30 candidates (the `fine_sync_at` flow 769 / 1 036); on `jt9`'s list alone 754 / 1 001 (761 / 1 009). Decodes in neither list 3 / 11 (7 of the 11 read as real signals by a wrong-tone scan; 4 with a hashed call are not checkable).
- **Cost** (host instructions a slot): 165 / 382 M at 15 / 30 candidates against 295 / 549 M; peak heap 258 KB against 422 KB. Not ESP32 cycles. `snr_db` not validated (about 1.3 dB below the other flow's); noise-only slots (tier C) not run; no board producer of the cells yet.

### FT8 at 8 kHz (fixed-point): the gated candidate search scores on `ft8_lib`'s statistic

- **`Ft8Rate::coarse_ft8lib`** (`Hz8000`, `fixed-point` only): `coarse_sync_gated` scores every (time, frequency) position of the slot by `ft8_lib`'s **mean of the dB contrasts** of each Costas tone against the adjacent tones and symbols (integer, one division a position) on cells of **0.5 dB units of the demux's 32-bit power** (`spectrogram::half_db`; the shipped u16 cell's `>> 12` throws the noise floor away: the same search on it was *below* `sync8`), keeps the best `12 x pool` per band, suppresses neighbours (+-2 bins, +-2 rows), takes the best 300 and orders them by hard sync as before. DT -2.26..+2.46 s (`ft8dec`'s rows). Banded, and bit-identical for any number of bands. The demux is now generic over the cell conversion (the u16 cells are bit-for-bit what they were: a checksum test). **A deliberate divergence from WSJT-X** (`sync8.f90`), ported from `ft8_lib`'s `sync_score` / `find_candidates` (MIT). `coarse_sync_gated_spec`, 12 kHz and f32 keep `sync8`.
- 60 real slots (38 `20m_busy` + the 22 recordings, 1 338 `jt9` decodes; `ft8dec`'s 1 026 as the yardstick), S3 flow, 8 kHz, host, against the two windows with the `sync8` search: hits at 10 / 15 / 20 / 30 / 60 candidates **523 / 737 / 888 / 993 / 997 -> 534 / 761 / 919 / 1 009 / 1 011**; ft8dec's 1 026: 982 -> 1 001 at 60. Extras at 60 candidates 25 -> 37 (15 added, 3 gone): 12 of the 15 are listed by `ft8dec`, all 15 read under 40 of 58 data tones wrong (noise-only: 40 or more); noise-only slots (tier C) not run. 22 recordings (`jt9`'s 402): S3 flow 230 / 289 at 15 / 30 candidates (224 / 287), native low-memory profile 229 / 287 (221 / 284).
- **Cost** (instructions per slot, 12 slots): `coarse_sync_gated` alone is dearer, 107 -> 152 M whatever the candidate count (measured on the stage alone); a candidate is too (about 15 against 11 M, a fit) because more of them are real and reach the full DFT: 255 -> 295 M at 15 candidates, 499 -> 553 M at 30, 760 -> 1 012 M at 60. Peak heap of the S3 flow 425 216 -> 425 216..432 292 B. At 919 hits the new flow is at 373 M and the `sync8` flow at about 378 M by a linear interpolation (888 / 993 hits; an upper bound of its cost is not known); its ceiling is higher (1 011 against 997).

### FT8 at 8 kHz (fixed-point): the stage-3 symbol DFT weights its window with a Hann window over two symbols

- **`Ft8Rate::symbol_hann_2t`** (`Hz8000`, `fixed-point` only): `fill_symbol_spectra_goertzel_at` runs its Goertzel over `2 x nsps` samples centred on the symbol, weighted by `sin^2` (a rotation of `(cos, sin)` into a half-window table once per call, 5 KB), instead of `nsps` samples rectangular; the output is rotated back by `e^{-j w nsps/2}` to the rectangular kernel's phase (without it every odd tone's cell had the opposite sign, and OSD's coherent `llrb`/`llrc`/`llre` read them: found by the review); `goertzel_window_end_sample_at` is half a symbol later. **A deliberate divergence from WSJT-X** (`ft8b.f90:153-161`). `Hz12000` and the f32 build are bit-for-bit unchanged. The coherent metrics with overlapping windows are validated by the corpora's recall only; the SNR from these cells would be about 0.5 dB low (computed, not measured).
- 60 real slots (38 `20m_busy` + the 22 recordings, 1 338 `jt9` decodes), S3 flow, 8 kHz, host, against the coarse window alone (5.1a): at **15 candidates 700 -> 737 hits (ft8dec's 1 026: 695 -> 734)**, 30 candidates 921 -> 993 (905 -> 980), 60 candidates 922 -> 997 (905 -> 982); against neither window (the shipped state) 710 -> 737 at 15, 893 -> 993 at 30. Instructions per slot (12 slots): 198 -> 255 M at 15 candidates (+29 %), 392 -> 500 M at 30 (+28 %), 608 -> 760 M at 60 (+25 %); **30 candidates with both windows (993 hits, 500 M) against 60 without (896, 608 M): +97 hits for 18 % fewer instructions.** Extras at 60 candidates 19 -> 25 (15 added, 9 gone): 10 are listed by `ft8dec`, 10 read under 40 of 58 data tones wrong, 4 are hashed or special-event calls the count cannot judge (one, `<...> JH1RGG`, unconfirmed). Noise-only slots (tier C) and the boards not run.
- 22 recordings: `the_s3_flow_at_native_8khz_meets_its_floor` 224 / 287 of 402 at 15 / 30 candidates (216 / 272 with the coarse window alone); three decodes `jt9` lacks are now attested (`GJ0KYZ VK2LAW QF56`, `CQ DX IK0YVV JN62`, `SM2EKA UT7IS -06`: the last two are copies 100 Hz from the signal, where the audio holds the message). **Correction to the entry below**: `SM2EKA UT7IS -06` is not "an alias of the 200 Hz baseband" and is not debt.

### FT8 at 8 kHz (fixed-point): the coarse spectrogram weights its frame with a Hann window over two symbols

- **`Ft8Rate::Hz8000` in the `fixed-point` build only**: the coarse spectrogram's frame (`compute_spectrogram_bins_src`) is `nfft` samples (two symbols) weighted by `sin^2` (Q15), starting `nsps / 2` earlier, instead of `nsps` samples rectangular. Same centre (the rows mean the same time), same cell for a steady tone (`sum w = nsps`; a one-symbol tone 0.67, -1.7 dB), noise cells 0.75 of the rectangular frame's (-1.25 dB): a signal is about 0.5 dB weaker against the noise, so `xsnr2_db_simple` (not rate-aware, not compensated) would read about 0.5 dB low at 8 kHz, unmeasured. `Hz12000` and the f32 build are bit-for-bit what they were. **A deliberate divergence from WSJT-X** (`sync8.f90`: rectangular).
- Why: the hard-sync count at `ft8dec`'s own place for the signals the S3 flow loses is 10 of 21 rectangular, 17 Hann; the shipped frame's sidelobes bury the Costas array of a signal that is not on the bin grid (a 25 Hz neighbour leaks 260 of 30 000 rectangular, 0 Hann). 60 real slots (38 `20m_busy` + the 22 recordings, 1 338 `jt9` decodes; `ft8dec`'s 1 026 messages as the yardstick), S3 flow, 8 kHz, host: at 60 candidates 896 -> 922 hits, 879 -> 905 of `ft8dec`'s, extras 19 -> 15; at 30 candidates 893 -> 921 hits, 877 -> 905; **at 15 candidates 710 -> 700 hits (-10), at 10 519 -> 506**. Instructions per slot (12 slots, `time -l`): +0.4 M at 60 candidates (607.4 -> 607.8 M), +17 M at 15, +25 M at 30.
- So it pays from about 20 candidates; the CoreS3's `max_cand` 15 loses 10 hits to it until the back end window follows (`FT8_WINDOW_AND_COARSE_DESIGN.md` 5.1b: the symbol DFT's Hann 2T doubles stage 3 and takes 15 candidates to 738). On the 22 recordings alone (`the_s3_flow_at_native_8khz_meets_its_floor`): 15 candidates 216 / 216, 30 candidates 272 / 268 of 402, no unattested extra. It also moves the public `decode_block_low_mem` / `coarse_sync_gated` at `Hz8000` in the fixed-point build (22 recordings, 30 candidates: 280 -> 284 of 402; one more decode `jt9` lacks, `SM2EKA UT7IS -06` at 1615.6 Hz where `jt9` has it at 1716 Hz: a copy of the signal 100 Hz below, attested; `the_low_memory_profile_at_native_8khz_meets_its_floor` now runs in both builds). The boards' `stage1_inc` / `esp_dsp_fft.rs` do not produce this frame yet; noise-only slots (tier C) not run.

### FT8 / FT4 plausibility verdict: a non-standard call may have a digit run in its number part (`DM100ZM`)

- On real busy-band recordings the verdict was refusing stations `jt9` decodes: `ft8_lib/test/wav/20m_busy` (38 recordings, 936 reference decodes) and 27 more, five decoder profiles, verdict on against off: off gained 12 to 31 decodes a profile (27 of the 31 distinct pairs are the special-event stations `DM100ZM` and `LZ365BM`: 25 of their 28 reference decodes were refused; the SIC profile also lost 3 neighbouring standard messages through the cascade). The `i3=4` call grammar took the rightmost digit as the number part, so their prefix was four characters. **`is_c58_special_call` now takes the whole digit run, up to three** (two refuses both stations; a longer run is refused). On those recordings the verdict now refuses only a **phantom** (`CQ ZNEWVJUSXYW`, no digit), which the SIC profile accepts with it off.
- Cost, 2 M uniform payloads: survivors 652 496 -> 654 566 (+0.32 %; `i3=4` 4.5 -> 5.3 %); the ceiling is a test (`phantom_survival_is_bounded`, release only). Against JTDX's `ALLCALL7.TXT` the arm refuses 8 of the 225 callsigns no standard call carries (`tests/callsign_acceptance.rs`, `wsjt77::tests::the_nonstandard_arm_...`, `$JTDX_ALLCALL7`): 4 look like typos, 3 like real special-event calls with a four-character prefix (`8J4Y70A`, `8J9J2BM`, `DL0E35Y`; a four-character prefix would take them for +1.0 % survivors: stage S1b, **not taken**, waiting for tier C's noise slots, which are not on this machine). `i3=5` and the WSPR types refused no real decode, but the recordings hold no such traffic: unchanged, untested by this evidence. WSJT-X's own acceptance stays one call away: `.message_filter(|_| true)`.

### FT8 at 8 kHz for the S3's flow, host side (fixed-point; interim `_at` forms)

- **`engine::dsp::fft_mixed_2560_sc16::Plan2560Sc16`** (256 x 10: the 3840 plan's 256-point sc16 kernel and gain 1/256, an f32 outer step, new `fft_10` = 2 x 5) and the 2560 arm of `RustFftPlanner16` (the generic 1/N fallback would have quantised every cell to a tenth). The **fixed-point spectrogram takes an `Ft8Rate`** (the 8 kHz assert is gone): the input is scaled by 12000 / rate after the gain shift, so a cell is the 12 kHz one's for the same signal and noise density (`FP_SPEC_SHIFT`, `ratio_eps` carry over). New, non-breaking: `fine_sync_at`, `fill_symbol_spectra_goertzel_at`, `goertzel_window_end_sample_at`, `refine_candidates_into_at` (the released functions delegate at 12 kHz; **12 kHz bit-for-bit unchanged**: the V9 decodes of the 22 recordings before and after are identical, the low-memory dump too). If the boards follow, these become a `rate` argument on the released functions (a break) and the `_at` twins go.
- Measured on the host (fixed-point, V9, the S3's flow, 22 recordings and `qso3_busy`): 8 kHz native 216 / 268 / 269 of 402 at 15 / 30 / 45 candidates against 12 kHz's 221 / 267 / 267 (`qso3_busy` 12 and 14 either way); the decode 21 % shorter, `qso3_busy` 279.5 M -> 213.8 M instructions at 15 candidates, 549.9 M -> 448.6 M at 30. The 15-candidate loss (-5) is explained stage by stage in the note: all six lost signals were the 15th and last of the 12 kHz gate list and are pushed just outside it by +-1 flips of the sync stage's hard-sync count and coarse membership, which the 16-bit spectrogram's quantisation makes about four times as frequent as the rate alone does (Costas decisions differing from f32: 3.9-4.1 % at one rate, 1.0 % between the rates in f32); the same flow in f32 loses 1 at 15 candidates (mean -0.33 over 15 candidate counts, against -0.4 in fixed-point). On the 38 real busy-band recordings of `ft8_lib/test/wav/20m_busy` (936 reference decodes) 8 kHz native is 2 in 600 behind (483 against 481 at 15 candidates, 609 against 607 at 30; f32 486 against 486 and 608 against 606): the -5 above is the upper end of a 0.3-0.4 % loss, not its size. **No board code changed and nothing ran on an Xtensa core**; details and the caveats in `docs/notes/S3_GATE_OSD_ABLATION.md`.

### FT8 low-memory decode at 8 kHz as it is (`Ft8Rate`)

- **`ft8::decode_block::Ft8Rate { Hz12000, Hz8000 }`**, an argument of **`decode_block_low_mem(src, rate, ...)`** and **`coarse_sync_gated(src, rate, ...)`** (both unreleased, so no break): the ICAP's 8 kHz audio is read as it is, with no 12 -> 8 -> 12 kHz resample in front. The grid is the same (3.125 Hz bins, 0.04 s rows, a tone 2 bins), so `sync_grid`, the score and the gate did not change; what takes the rate is the spectrogram's frame / FFT / step (1280 / 2560 / 320) and stage 1 of the 200 Hz cascade (8 kHz -> 4 kHz, 11 taps; 9 is the least that passes the alias test). A deliberate divergence from WSJT-X (12 kHz only), marked in the code. **12 kHz is bit-for-bit what it was** (every candidate and decode of the 22 recordings, f32 and fixed-point, dumped before and after: identical).
- Measured, 22 recordings, `--low-mem 4`, host: 30 candidates 273 of 402 against the round trip's 274 (45 and 60 candidates: equal), 15 candidates 224 against 225, no extras; the decode takes 754 ms against 957 ms (-21 %, three runs each, alternating), 15 candidates 479 against 639 ms. The derived "a third less" of the 12 kHz-rate part (`ICAP_EMBEDDED_ANALYSIS.md` §3.2) is a different quantity from this whole-decode figure; the measurement is recorded beside it. `qso3_busy`, the decode alone (20 repetitions): 765 M -> 615 M instructions at 15 candidates, 1169 M -> 979 M at 30, decodes unchanged (11, 14). With `fft-extern` the binary's planner must give a 2560-point real FFT at `Hz8000`. Not done: `fixed-point` at 8 kHz (panics: no 2560-point 16-bit plan, `FP_SPEC_SHIFT` calibrated for 3840), `decode_block` / `Decoder<Ft8>` at 8 kHz, a board, the Cortex-A9.

### Q65: the coarse search from a source (`coarse_search_streamed`)

- **`q65::search::coarse_search_streamed::<P>(src, rate, nominal_start, &SearchParams, max_drift, bands)`**: the sync search without the slot's spectrogram (5.7 MB for 30A, 11.4 MB for 60A). A bin's curve reads its own column, so each of `bands` forward sweeps keeps one band of columns (about 170 KB at 16 bands for 30A); the candidates equal the whole-spectrogram search's to the bit (synthetic; two real recordings, 47 candidates). The price is `bands` times the FFTs and the reads. Underneath, the search's per-bin curve and candidate selection are shared by the whole and the banded paths. The scan's shift grid and b90 ladder are not streamed. Figures derived, not measured on a target.

### Q65: decode one candidate from a source read forward (`decode_candidate_streamed`)

- **`q65::decode_candidate_streamed::<P>(&mut impl PcmSource, rate, &Q65Candidate)`**: `decode_candidate` for a slot that cannot be held. One symbol's samples at a time (`i16 / 32768`); the fast-fading metric streams its wide rows through `FastFadingStream` and keeps 16 128 B in place of the 48-290 KB wide spectrogram, plus about 14 B a sample of per-symbol buffers (so no saving against the spectrogram at 300 s; the audio is never held); a failing candidate is one forward sweep, a decode reads once more for the SNR. Plain, hint and list equal the slice decode to the bit, fading to float rounding (a real 10 GHz EME recording: 8 point decodes compared, SNR bits equal). Underneath, `SymbolWindows` (a slice or a `PcmSource`) replaced the audio slice in the row loop and the SNR's composite spectrum, with no result moved; the fit checks use `checked_add`. Not streamed: the coarse search. Memory figures are derived, not measured on a target.

### Decoding one known candidate: `Alignment` and `decode_candidate` (`SniperRequest` deprecated)

- **`engine::alignment::Alignment`** (`start_sample`, `freq_hz`; each mode re-exports it) and, per mode, a config struct and one function: **`wspr::decode_candidate(WsprInput::{audio, baseband}, &WsprCandidate)`**, **`jt65::decode_candidate(audio, rate, &Jt65Candidate)`** (`Jt65Strategy::{Hard, Erasures, Chase}`), **`jt9::decode_candidate(audio, rate, &Jt9Candidate)`**, **`q65::decode_candidate::<P>(audio, rate, &Q65Candidate)`** (precedence `ap_list`, `fading`, `ap_hint`, plain). Table and migration: `docs/reference/LIBRARY.md` §2.5.
- **Deprecated since 0.17.0: `wspr::`, `jt9::`, `jt65::`, `q65::SniperRequest`**, removal in the next minor. `SniperRequest::decode` calls the function, so the decode is one. The four modes' tests and the boards' WSPR candidate loop call `decode_candidate`; the in-crate tests that build a `SniperRequest` through the internal `DecodeRequest::sniper` (Q65's pileup and hash table among them) remain and must move before removal. Q65's `on_result` has no counterpart.
- Each option is pinned where it decides (`docs/notes/DECODE_CANDIDATE.md`); those conditions sit near thresholds; they pass on Apple arm64 and on CI's x86_64 runner (72f6fea4), not run in fixed-point.

### Q65: the fast-fading metric one row at a time (`FastFadingStream`)

- **`fec::qra::FastFadingStream`** (new): `intrinsics_fast_fading` for a decoder that cannot hold the wide-energy spectrogram (48 KB sub-mode A, 290 KB E): `push_row` per symbol, `finish` writes the `63 x 64` metric (16 128 B kept, whatever the sub-mode), no mean energy needed in advance (the noise variance divides the logits at the end). Probabilities equal `intrinsics_fast_fading`'s to float rounding (tolerance test over all sub-modes, both models, spreads across the table; six mutations caught), `FastFadingState` to the bit. Nothing existing changed; no decode moved. The SNR estimate still reads the wide energies again after the decode.
- **`fec::qra::FadingRow`** (new, under the stream, which is now built on it): the stateless block for a decoder with its own layout (several spreads from one row, the metrics of several candidates interleaved, a store to spill them to). `metric(row, out)` is a row's logits before the division by the noise variance; `noise_var(mean_energy)`, `probabilities(metric, noise_var, out)` and `state(noise_var)` finish them once the mean is known; `weights()` / `es_no_metric()` expose the window; the raw tap tables stay private. The stream's output is unchanged (it is the blocks by construction; `intrinsics_fast_fading` agrees to float rounding over all sub-modes, both models and spreads, and for a bank of three spreads over the same rows); eleven mutations of the block tried (the clamp removed, widened to -300 and moved to -90 included), all caught after one test was added for the clamp. A downstream no_std decoder that carried a patch for a per-row metric builds against this tree except for its `lorentz_taps` call, which this does not provide; its numbers were not compared and nothing ran on a device.

### Breaking: one shape for the new `ft8::decode_block` API (source, config structs, one entry per function)

- **`ft8::decode_block::source::AudioSource` is `engine::source::PcmSource`** (any mode's, and it collided with `engine::sync::AudioSource`, the enum; `AudioSample` moved with it, `ft8::decode_block::AudioSample` still resolves). **No `_src` twins**: `coarse_sync_gated(src, &GatedSearch, bands)` and `decode_block_low_mem(src, &GatedSearch, depth, LowMem)` take a `&mut impl PcmSource` (`SliceSource(&slot)` for memory); the slice forms and `coarse_sync_gated_src` / `decode_block_low_mem_src` are gone. The host driver no longer carries a `low_mem` parameter (`decode_block_low_mem` always took the low-memory path).
- **Config structs, not argument lists**: `GatedSearch::new(freq_min, freq_max, sync_min, keep)` with `.lag_s()`, `.fill()` (what is looked for; how it is held is `LowMem`) serves `coarse_sync_gated` and `coarse_sync_gated_spec` (7 positional arguments, two `Option`s, before). `LowMem`, `SpecFill`, `ReadCost` and `GatedSearch` are `#[non_exhaustive]` (`LowMem::new`, `SpecFill::new().gate_block2()`).
- **`ft8::decode_block::osd_available()`** (`const fn`): without `fft-rustfft` and the `osd` feature `DecodeDepth::osd` is ignored with no error; the CoreS3 app's compile-time check now asks the library. `band_bb` and its gate are `__internal::ft8_band_bb` (experimental, no semver).
- No decode moved (every floor unchanged); the FIR stage / mixer cost of the earlier unification is gone (`perf:` commit): A 0.59 s -> 0.53 s, C 1.18 s -> 1.19 s on `qso3_busy` x 21 (host, fixed-point). The low-memory profile is not on `Decoder<Ft8>` / `Ft8Extras` (it is a different engine from `DecodeRequest`'s): a separate embedded profile type is the plan.

### CoreS3 / S3 apps: the hard-sync gate and OSD as switches (off)

- **`dual_core::DecodeConfig::hard_sync_gate`** (default `false`; `rx_wavsim`, `m5stack-s3-app`, `m5stack-core2-app` set it) picks pass 1 with `coarse_sync_gated_spec` on the SpecBundle's spectrogram and keeps the gate's order through pass 2. The CoreS3 app reads it from `MFSK_FT8_GATE=1`; `MFSK_FT8_OSD=1` (with the new `ft8-osd` feature, which builds `mfsk-core/osd`; without the feature it does not compile) sets `DecodeDepth::osd`. Both off by default: the **`dual_core.rs` path is the same calls with the same arguments** (by inspection; it has no host test). `MFSK_FT8_OSD=1` additionally needs `MFSK_FT8_OSD_STACKS_RAISED=1`: stage 3 with OSD needs ~25 KB of stack (the linked ELF) against 11 KB without it and the board's tasks have 16 KB.
- On the host's mirror (`mirror_gate_and_osd`, 81 phases of `qso3_busy`): gate + OSD 8.81 decodes of 20 against 6.56, gate + OSD + fine sync 11.35, no unexplained decode. **Not run on a board**; its time, stack (OSD ~25 KB against the tasks' 16 KB, static estimate) and internal-DRAM floor are what `ft8-gate-bench` is for. `docs/notes/S3_GATE_OSD_ABLATION.md`.

### FT8: the hard-sync gate from a spectrogram you already hold (`coarse_sync_gated_spec`)

- **`ft8::decode_block::coarse_sync_gated_spec(&spec, &GatedSearch)`** (`GatedSearch::new(freq_min, freq_max, sync_min, keep).lag_s(..).fill(SpecFill::new(valid_rows))`) (new, building-block tier): `coarse_sync_gated` read from a `Spectrogram`, which is what the boards' `stage1_inc` builds during capture; on a whole slot (`fill: None`) it returns `coarse_sync_gated`'s candidates bit for bit. `SpecFill { valid_rows, gate_block2 }` describes a streaming spectrogram: the hard-sync count skips unmeasured rows (they read as tone 0); `gate_block2 = true` is `PartialBlock2::Gate` and costs recall (116 / 168 against 216 / 269 of `jt9`'s 402 at 15 / 30 candidates), so the boards' default stays off. Existing callers are unchanged.
- Measured (host, fixed-point, `docs/notes/S3_GATE_OSD_ABLATION.md`): the gate, `fine_sync_12k` and OSD on the ESP32 driver's own Goertzel path reach 221 / 267 of 402 and `qso3_busy` 13 / 14, against the driver's 131 / 144 and 7, at 283 M instructions a slot (the driver 308 M). On the board's own flow (`ft8_embedded_pipeline_mirror`, 81 phases) the gate, `fine_sync_12k` and OSD take `qso3_busy` from 6.6 to 11.4 decodes of 20 with no unexplained decode. **Not run on a board; the only callers in `embedded-poc` are `dual_core.rs` (off by default) and `ft8-gate-bench`.**

### FT8 band baseband: one read of the audio, subtraction on the baseband (experimental)

- **`__internal::ft8_band_bb`** (`ft8::decode_block::band_bb`, experimental, `internal-testing`, no semver): the whole band as one complex 3 kHz baseband (`i16`, 180 KB), built in a single forward pass (`BandStream::feed`); the coarse search (`coarse_sync_gated_band`), every candidate's `cd0` and the subtraction between passes (`BandBaseband::subtract`, `subtractft8.f90`'s algorithm, a sliding window) all work on it. `GfskStream` gains `peek_complex` and `skip`.
- Measured (host, 22 recordings, 3 passes): 334-353 of `jt9`'s 402, fixed-point 342 at 15 candidates (ICAP's `ft8dec`: 333), `qso3_busy` 19 of 20 with no extras, from one read of the audio (240 KB at 8 kHz) and 18 ms of subtraction a slot; heap 0.33 MB fixed-point, 0.59 MB f32. **Two decodes no other decoder lists** (budgeted in the tests). `BandBaseband::subtract_f32` does the subtraction without `f64` (for Xtensa) with the same recall and the baseband to 1 LSB; the rest of `band_bb` still has a little `f64`. Not run on a radio, a Pi or a board. Detail: `docs/notes/MIDRANGE_EMBEDDED_EVAL.md`.

### FT8 low-memory decode: the slot is read forward from a source, never held

- **`decode_block_low_mem`** (`ft8::decode_block`, new, building-block tier): candidates picked by hard sync read from the spectrogram (`coarse_sync_gated`, a **divergence from `sync8.f90`'s score order**: the first 30 by score hold 9 real signals of `qso3_busy`, by hard sync 21), the spectrogram built a band at a time, each candidate's 200 Hz baseband by a FIR cascade (`cd0_fir`, a divergence from `ft8_downsample.f90`'s FFT) from `PcmSource` (`SliceSource`, `CountingSource`: what a forward-only SD card costs). New cargo feature `osd` (off): the OSD fallback in builds without `fft-rustfft`.
- Measured (host, `hosttest/icap-app`, ft8_lib's 22 recordings against `jt9`'s 402): 227 / 279 decoded at 15 / 30 candidates (fixed-point), against `decode_block`'s 131; peak heap 0.22 MB, independent of the candidate count. `decode_block`'s decodes are unchanged (bit-identical, tier A+B), **but `coarse_sync` no longer builds its 226 KB `sync2d` grid (peaks per carrier instead), so the ESP32 boards' heap use and `coarse_sync` time differ: not flashed**; the `profile-coarse` log line is kept (`allsum=0` on the boards, it is built during capture). Not run on any radio or board. Detail: `docs/notes/ICAP_EMBEDDED_ANALYSIS.md`.

### The ESP32 boards build again

- **`ft8::stage_timing` no longer stops the Xtensa boards compiling.** 0.16.2's `MFSK_TRACE_STAGE_FT8` counters are `AtomicU64`, which Xtensa lacks, so `m5stack-cores3-app`, `-s3-app`, `-core2-app` and `m5stack-s3` failed with E0432 from `2ad660fe` (2026-11-23) on; host CI does not build `embedded-poc`, so nothing noticed. The tracing is now compiled only where 64-bit atomics exist (`target_has_atomic = "64"`) and is the no-op elsewhere; host behaviour is unchanged. `cargo check --release` passes for the four boards again. Not flashed: no runtime path changed.

### Skimmer

- **A short hole in a SpyServer stream no longer drops the open slots.** A missed sequence number (the server dropped messages while the path stalled, seen on a tethered link on 2026-11-22) called `IqReceiver::gap`, which drops every slot open across the hole. A hole of up to `GAP_FILL_MAX_S` (0.5 s) is now filled with silence in place, so the slots go on and FT8's FEC decides; a longer one keeps the old behaviour. Silence is the byte 128 in offset-binary `Cu8`, 0 elsewhere (`silence`). Test `short_spyserver_hole_keeps_the_slot`: an FT8 frame with 0.1 s missing decodes in `Cs16` and `Cu8`, one with 0.6 s missing is dropped, and it fails with the old `gap` call; `silence_is_zero_in_every_format` guards the `Cu8` byte (a zero-byte fill still decoded in that test). Checked: `skimmer-core` tests (`--release`), clippy. Not checked: a recorded corpus of real holes, a live SpyServer session; rtl_tcp (no sequence numbers) is unchanged.

## 0.16.2 — FT8 decodes 35-41 % faster on x86 (one thread) and faster on Apple arm64: BP tables and a rational `tanh`, real-input FFTs, `subtract` (a reference recurrence, buffers kept per thread, an aarch64 FFT length), `refine_dt` in parallel; `mfsk-serve`; WSPR follows `wsprd` (#760, #761, #762)

### Subtract keeps its two largest buffers per thread, so a decode does not depend on glibc's `mmap` threshold (#761)

- `subtract` allocated `cfilt` (1.44 MB for a slot) and rustfft's scratch on every call. Above glibc's `mmap` threshold each call takes fresh pages and page-faults on all of them; the threshold had been raised past `cfilt` by the old complex downsample cache's temporaries until `405f6e08` made them smaller, and then x86's `Fast` decode went 52 -> 60 ms with `subtract` 18 -> 26 ms (17 calls). Bisected to that commit, to its `downsample.rs`, and by sweeping `MALLOC_MMAP_THRESHOLD_` (the step is between 1.0 and 1.5 MB). Both buffers are per-thread work buffers now; `cfilt` is zeroed each call and rustfft runs on `process_with_scratch`, so every sample is unchanged (a test runs a subtract after others of other lengths against a fresh thread; tier A+B: no fixture moved).
- Measured (x86, one thread): `Fast` 59.8 -> 51.0 ms, the same as with the threshold fixed at 1 GB; `Normal` 185.5 -> 174.3, `Deep` 385.3 -> 383.6. Not done: the reference waveform, `dphi` (0.6 MB each), `refine_dt`'s trial copies and the spectrogram are above the default 128 KB threshold too (`Fast` is 58.0 ms at a 256 KB threshold); they are for when they are found to matter. Linux/glibc only; not measured on Apple or a Pi. Detail: `docs/notes/FT8_SLOT_PROFILE.md`.

### Review of the #761 work: `tanh_approx` clamped to `[-1, 1]`, tests for the tables and the FFT length, comments corrected

- `tanh_approx` returned 1.0000001 near its input clamp; its output is clamped now. Over every `f32` in `[0, 9]` exactly one input gave more than 1 (7.9037843, and its negative); only that output changes (1.0000001 -> 1). A product that has it as a factor moves by one ulp; the sweeps for this release run on the clamped version. Its stated error was wrong: every `f32` in `[0, 9]` gives at most 4.95e-7 (at 5.7797), not 3.5e-7, and libm `f32` 4.87e-8; the test checks that point and a 1e-5 grid instead of a 1e-3 grid that missed it.
- New tests: `sum_product_tables_match_the_searches` (the BP index tables against the `mn` / `bit != j` searches for LDPC(174,91), (240,101), (240,74), (128,90)); `fast_fft_len_is_the_next_2a_135` (the aarch64 subtract length, now a function every target tests). Corrected comments: `platanh`'s doc was attached to `tanh_approx`; `fft_len` is not only rounding for a buffer no longer than the frame (unpadded, the kernel wraps there); `residual_band_power` scores the whole buffer where `sqf` scores the frame-aligned residual (pre-existing, now marked); the `BpScratch` and min-sum test notes; `u16` table indices have a compile-time bound. Found by the reviewer agent; none of it changes a decode except the clamp.

### Real-input FFTs where the input is real: the FT8 spectrogram, the downsample cache, the `refine_dt` score (#761)

- Three transforms took a real signal through a complex FFT with a zero imaginary part: the FT8 spectrogram (372 rows of 3840 points a call), `build_fft_cache` (the downsample spectrum, 192 000 points for FT8; shared by FT4, FST4 and the engine pipeline) and `refine_dt`'s residual score. They are real-input FFTs now (`realfft`, already a dependency), half the work, as the WSJT-X sources do it (`four2a(..., 0)` in `sync8.f90`, `ft8_downsample.f90`, `subtractft8.f90`). The cache keeps its full length: the negative half is filled with the conjugates. Values differ from before by rounding.
- Measured (Apple arm64, one thread, `qso3_busy.wav`, base and new alternated three times): `Fast` 63.2 -> 59.8 ms, `Normal` 227 -> 206 ms (last checkpoint 82.3 -> 75.7), `Deep` 423 -> 401 ms (last checkpoint 179 -> 175). FT8 and FT4 over the 26 vendored recordings at three depths: the same message set in all 156 runs. Tier A+B: 172 binaries, 1402 passed. **Tier C (FT8, FT4, FST4 sweeps) is not run here.**

### `[profile.dist]`: fat LTO and one codegen unit for installed binaries (#761)

- `cargo install --profile dist` builds with `lto = "fat"` and `codegen-units = 1` (the workspace had no release profile settings; `release` is unchanged, so the merge gate's test builds do not slow down). Measured on `mfsk-serve bench qso3_busy.wav` (Apple arm64, one thread, three alternations): `Fast` 63.8 -> 61.1 ms, `Normal` last checkpoint 83 -> 80 ms, `Deep` last checkpoint 181 -> 172 ms; output identical on 26 recordings x 3 depths. `MANUAL_SERVE.md` / `.ja.md` and the serve README install with it. Not measured on a Cortex-A72.

### `mfsk-serve --threads N` (#761)

- Every command takes `--threads N` or `MFSK_SERVE_THREADS=N`, which builds rayon's global pool with `N` threads (`0` or absent: rayon's default, every core). The decoder's threads (candidate-parallel work, the three `refine_dt` trials) are otherwise unbounded, which matters on a board shared with other processes. Output does not depend on it. Checked: `--threads 1`/`2`/`4`/`0` on `qso3_busy.wav` at `Deep`, same user CPU time (0.43-0.45 s), wall 0.74 s on one thread against 0.39-0.40 s on two or more; a non-number is an error. Manual: `MANUAL_SERVE.md` and `.ja.md`.

### `subtract_tones_lpf_refine_dt`: the three trials run in parallel (#761)

- The -90 / +90 / 0 sample trials (each: copy the audio, subtract at its offset, score the residual: a forward and an inverse FFT, then a forward FFT) share only the read-only audio and reference, so with `parallel` they run on separate threads (`rayon::join`); the last, best-offset, application needs their scores and follows. Each computation is the same as before, so the output is **bit-identical** (26 recordings x `Normal`/`Deep` with 4 threads: all 52 outputs equal to the last digit; tier A+B green). The 11 FFTs of one call become 5 in sequence.
- Measured (Apple arm64, `RAYON_NUM_THREADS=4`, base and new binaries alternated twice): `Normal` 227 -> 193 ms, `Deep` 425 -> 381 ms, the checkpoint that only subtracts (B) 66 -> 33 ms at `Normal`, 85 -> 43 ms at `Deep`. With one thread nothing moves. **The last checkpoint, the one after the period ends, does not move** (82 ms `Normal`, 180 ms `Deep`): it subtracts with the plain, sequential call. Not run on a Cortex-A72.

### SumProduct BP (FT8, FT4, FST4, FST4W, MSK144; the embedded FT4 path too): index tables and a vectorisable `tanh` approximation (#761)

- `bp_decode_generic_kind_with_scratch` and `bp_llr_zsum_ap_with_scratch` searched `mn` for the edge to subtract and tested `bit != j` on every term of each check's product. Both are now tables built once per `BpScratch` (padded with a `1.0` slot, so each product has a fixed length): the same arithmetic in the same order, **bit-identical**. The `tanh` of the half-message cache is a rational approximation with no libm call (`tanh_approx`, Eigen's coefficients, absolute error at most 4.95e-7 over every `f32` in `[0, 9]` against 4.87e-8 for libm `f32`, output clamped to `[-1, 1]`), one loop over all edges that vectorises. **A divergence from the WSJT-X BPs, which call `tanh`**, marked in the code; it is the same on every target.
- Measured (Apple arm64, one thread, `qso3_busy.wav`, base and new binaries alternated three times, repeatable to 1 ms): `Fast` 68 -> 63.5, `Normal` 240 -> 228, `Deep` 474 -> 426 ms. The BP split before: `tanh` 38 %, the product 31 %, the check-to-variable update 20 %. Over the 26 vendored recordings at three depths, the decoded message set is identical in all 78 runs (the tables alone: rows identical to the last digit). Tier A+B green on aarch64. Not run on x86 or a Cortex-A72; **the tier C sweeps are not run here**: this is every protocol's default BP (`FecOpts::default()`), so `ft8 ft4 fst4 fst4w msk144` before the release this is in. It also runs on the CoreS3 FT4 ship path (`embedded-shared` `ft4_rx` -> `engine::pipeline`): **not flashed**, no board timing.

### Subtract on aarch64: the FFT is zero-padded to a length rustfft's NEON path is fast at (#761)

- `subtract` and its residual score ran a 180 000-point FFT pair (`2^5 3^2 5^4`, `NFFT = NMAX` in `subtractft8.f90`). On an Apple M-series core rustfft's NEON path takes 1.2-1.5 ms for that length and 0.47 ms for 196 608 (`2^16 * 3`); the pair was 97 % of a warm `subtract` call there (20 % on x86). On aarch64 the buffer is now zero-padded to the next `2^a * {1, 3, 5}` (a pure power of 3 measured slower); other targets are unchanged and bit-identical. **A divergence from `subtractft8.f90`**, marked in the code: same taps, more zero tail, so the output differs by float rounding.
- Measured (Apple arm64, AC, one thread, `qso3_busy.wav`, base and new binaries alternated three times): one FFT pair 2.4 -> 1.0 ms, `refine_dt` 14.3 -> 6.4 ms; a whole call `Fast` 99 -> 68-97, `Normal` 398-780 -> 240-315, `Deep` 638-698 -> 476-544 ms, the same rows. This Mac swings by up to 2x between runs; read the direction, not the ratios. Tier A+B green on aarch64. Not measured on a Cortex-A72; x86 not re-run.

### FT8's subtract reference: a phasor recurrence, a deliberate divergence from `gen_ft8wave.f90` (#762)

- `synth_complex_f32_into` takes the reference's `cos`/`sin` from a phasor recurrence anchored every 256 samples by an exact `f64` phase (one rotation per sample, the carrier's and the modulation's multiplied into one first), where `gen_ft8wave.f90` and this crate called `cos` and `sin` on an `f32` phase for every sample. The reference is within 1.7e-5 of the exact waveform where it was within 7e-4; 0.607 -> 0.389 ms, one `subtract` 1.25 -> 1.04 ms (x86, one thread). A shape with modulation above 0.1 rad per sample (either sign) and a build without `std` keep the loop, bit for bit. The M5Stack apps enable `std` and so take the recurrence, but their FT8 decode runs no subtraction; an `f64` add per sample and an `f64` `sin_cos` per block would be software on an `f32`-only FPU, and are not measured on any board. Accepted on the condition that the tests show no problem; the divergence is marked in the code and recorded in #762.
- What moved: `full_slot_output_is_pinned` re-recorded (2 821 of 180 000 samples, each by 1) and `ft8_eme_delay.txt` (its faintest row, `K1BZM DK8NE -10`, 244.75 -> 244.25 Hz, 15 -> 17 hard errors; `jt9`: 18). Over the 26 vendored recordings at three depths the decoded messages are the same in all 78 runs, no frequency or DT moves, and the SNR against `jt9` on 389 rows is +0.196 -> +0.195 dB, SD 1.744 -> 1.746 (`scripts/compare-serve-decodes.py`; `MFSK_WRITE_SUBTRACT_PIN=1` re-records the pin). **Run the tier C FT8 sweep before the release this is in** (`scripts/run-sensitivity-sweeps.sh ft8`): it is not run here.

### Subtract: the end-correction table is built once, and `refine_dt` builds its reference once (#761)

- `subtract_tones_lpf` (FT8, FT4, and every other user of `engine::dsp::subtract`) rebuilt `end_correction(lpf_half)` on every call: `lpf_half + 1` sums of up to `lpf_half + 1` taps, about two million sequentially dependent f32 adds for FT8's 2000, for a table that depends on nothing but `lpf_half`. `subtractft8.f90` builds it once; so does this now, by the same function, so every value is bit-identical (a test pins the bits; tier A+B: 172 binaries, 1397 passed, no fixture moved).
- Measured against `MFSK_TRACE_STAGE_FT8`'s split (`qso3_busy.wav`, x86, one thread): one `subtract` 2.41 -> 1.35 ms; a whole call at `Fast` 79 -> 67 ms, `Normal` 293 -> 239 ms, `Deep` 594 -> 533 ms, the same rows. `subtract` had been 45 % of `Fast` and 57 % of `Normal`; the table was 29 % of one `subtract`. A Cortex-A72's add latency is longer, so the share there is probably larger; not measured.
- `subtract_tones_lpf_refine_dt` (the checkpoints and `sic_early`) applied the subtraction four times, offsets -90, +90, 0 and the best, and each application rebuilt the GFSK reference, which does not depend on the offset: 0.69 ms of each. It is built once now; every sample is unchanged (a test compares against the old four-build search on a full slot, a frame that starts before the buffer, a buffer shorter than the frame and no filter; tier A+B: 172 binaries, 1398 passed). One `refine_dt` call 8.9 -> 6.55 ms; the checkpoint that only subtracts (B) 65 -> 43 ms at `Normal`, 85 -> 57 ms at `Deep`; a whole call at `Normal` 243 -> 217 ms and at `Deep` 540 -> 507 ms; `Fast` does not use it.
- `synth_complex_f32_into` (the subtract reference) built its phase-rate array with a bound test on every element that was always true (a pulse ends at `(nsym + 2) * nsps` at most); the `zip` form, as `synth_f32_lead_tail_into` already had it, has none and vectorises, with the same additions in the same order, so every value is bit-identical (a test pins FT8's, FT4's and FST4's shapes and one- and two-symbol frames; tier A+B: 172 binaries, 1399 passed). The reference 0.695 -> 0.607 ms, one `subtract` 1.35 -> 1.25 ms (x86, one thread).

### FT8 — `MFSK_TRACE_STAGE_FT8` says where a call spends its time (#761)

- With the variable set, every `Decoder<Ft8>` call (`decode`, `decode_prefix`) prints one `TRACE_STAGE_FT8_ALL` line on stderr: wall time, the time put to stages, and each stage's time and count, largest first (spectrogram, coarse sync, FFT cache, downsample, fine refine, symbol spectra, the LLR metrics, BP, OSD, subtract, baseline, SNR). Times are exclusive and summed over threads, so use `RAYON_NUM_THREADS=1` for one core's split. Measured on `qso3_busy.wav` (x86, one thread) the stages account for 92-100 % of a call; unset, a stage costs one relaxed atomic load. No result changes; the stage list and the first split are in #761.

### `mfsk-serve` — a resident decode/encode process on JSON lines (`apps/serve`, #760)

- One process keeps a decoder per mode, so what a `Decoder` holds between calls (callsign hash table, the two latest periods, a period's checkpoint state) lives across requests. Requests are JSON lines on stdin: `decode` (`wav`/`.raw`, `depth`, `band`, `period`, `budget_ms`, `learn`, `prefix` + `samples` for `decode_prefix`), `encode` (`encoder::encode` to raw s16le; what the message sends as hashes is learned). Answers are `row` / `done` / `encoded` / `error` lines; a bad line never ends the process.
- `mfsk-serve bench FILE` times a recording per depth (cold, warm, and at the decoder's checkpoints): the measurement #761 needs on a Raspberry Pi. No speed claim is made for any board; on x86 a warm process saves 10-21 ms of an 82-620 ms call, so the point is the kept state.
- Sample app, `publish = false`, installed with `cargo install --git`; no library change, no tag. Tier A in `apps/serve/tests` (state across requests, checkpoints end with the whole period, encode→decode, error lines). Manual: `docs/reference/MANUAL_SERVE.md`.

### Docs — `DESIGN_RATIONALE.md` leads with the design, history goes to the changelogs

- Part I explains why the crate is built as it is (purpose, WSJT-X as reference, the `Protocol` trait, the `jt9`-shaped API, the
  three tiers, wide-band as the main path, measurement and precision, Rust); Part II keeps the embedded PASS1 decision and the not-ported list.
- The AP-coupling and 0.12 accounts now live only in `docs/historical/`; Q65's front-end notes moved to `LIBRARY.md` §2.5 (and `.ja.md`).
- Shared-code share re-measured at `f0e8dcc6`: `engine` + `fec` + `msg` are 47.2k of 108.0k lines (44 %). No code change.

### Changed — rustc 1.100.0; the 1.99.0 `black_box` workaround is gone (#757, #758)

- rust-lang/rust#163626 (1.99.0's aarch64 `loop-vectorize` hang on FT8's Costas phasor table) is fixed upstream and ships in 1.100.0. The host pin
  (`rust-toolchain.toml`, every `dtolnay/rust-toolchain@`) moves 1.98.1 → 1.100.0, and the `black_box` in `ft8::refine_fine` is removed; the table is bit-identical.
- Measured on 1.100.0 without the workaround: the standalone loop compiles in ~0.1 s (1.99.0: killed at 30 s); `--features full` release builds for aarch64
  apple-darwin / apple-ios / linux-gnu take 15-17 s; merge gate 1395 passed; CI green incl. Android, iOS, Swift, Kotlin.
- **A downstream build on rustc 1.99.x for aarch64 at opt-level 3 hangs again** (1.98.x and 1.100+ are fine). Use 1.100.0 or later.

### Added — `wsprd` v3.3.0-beta6 output through the public API

- `WsprExtras::wsprd()` and `Option` fields per `wsprd` flag (`passes`, `subtraction`, `jitter`, `more_candidates`, `osd: WsprOsd`,
  `freq_span_hz`, `dial_error_hz`, `fano_bias`, `replica_from_text`, `wsprd_gates`); new `WsprResult` fields (`sync`, `pass`, `blocksize`,
  `jitter`, `decode_type`, `nhardmin`, `cycles`, `metric`, `f1_hz`); `hashtable.txt` read / write; `Decoder::state()`; `wspr::wsprd_fmt`.
- Measured: 24 golden cases and 4 synthetic ones byte-equal to `wsprd` (`tests/wsprd_beta6_parity.rs`, `docs/notes/WSPRD_PARITY.md`).

### Fixed — fidelity to `wsprd.c`; **default-path results can change**

- `-s`, `-B` (pass 1 skipped only with `npasses > 2`), `-o n` (`osdwspr` ported, every rung of every pass), `fmin`/`fmax` with `-e` once per pass, `-z`.
- Also on the `Depth` path, plainly: (a) subtraction takes `f1` itself, not `1500 + f1` rounded to `f32`; (b) a decode `unpk_` will not print
  stops the ladder; (c) the callsign table is 32768 buckets, a colliding call evicts, the whole run's table is handed back; (d) `Depth::Fast`
  runs pass 1 after an empty pass 0. Merge gate and snapshots unchanged; tier C (`wspr`) on `main` at `4b4d5ac0` (that change included): -31.40 dB, +0.00 dB against the baseline.

### Fixed — WSPR scan matches `wsprd`'s candidates: coarse values kept at or below `minsync1`, no front padding; **default-path results can change**

- `refine_candidate` keeps the coarse frequency, shift, drift **and sync** of a candidate whose refined sync is at or below `minsync1`, and
  `minsync2` is tested on the coarse sync (`wsprd.c:1325-1343`); the crate dropped it. And `decode_scan` no longer prepends 3.072 s of zeros
  (`wsprd` has none; it reaches a negative shift directly, `wsprd.c:278,1250`): the pad changed the coarse sync of the same alignment.
- Measured (WSPR AWGN sweep, 260 files): the decoded files equal `wsprd` v3.3.0-beta6's on all 260 (0.16.1 missed one at -32); crossing
  -31.40 → -31.44 dB (`wsprd` -31.44). The reported SNR of all 9 golden decodes now equals `wsprd`'s to 4 decimals (was 0.01-0.12 dB off with the pad; why
  is not measured); DJ6OL still at dt -1.94 s, parity 32/32. The four `wspr_*` snapshots move in `snr_db` only. Detail: `docs/notes/WSPRD_PARITY.md`.
- **Breaking (source):** `BasebandCandidate` gains the pub field `lag_baseband` (signed `shift`; `start_sample` still clamps at 0), so a struct
  literal outside the crate stops compiling. `NEGATIVE_DT_PAD` stays, unused; `WSPRD_NPOINTS` (114 s) is new and `decode_scan` cuts its audio to it
  (`NPOINTS_MAX` is unchanged). Embedded: the pipelines pad their own buffer; the literal in `wspr_dual_core.rs` is updated, not built or flashed here.

### Skimmer

- **Skimmer 1.1.1 is the skimmer on `mfsk-core` 0.16.2** (tag `skimmer-v1.1.1`; the four version files and `package.json` agree; no skimmer code change). It takes the library by path, so its decode side gets 0.16.2's: FT8 decoding 35-41 % faster on x86 at one thread (SumProduct BP's index tables and rational `tanh`, real-input FFTs, `subtract`'s reference recurrence, its work buffers kept per thread, `refine_dt`'s parallel trials; on aarch64 `subtract`'s FFT length), WSPR following `wsprd`, and the three deliberate divergences from WSJT-X the release notes name (#761, #762).
  Checked: `skimmer-core` and `sdr-io` tests (release), `cargo check` of the workspace; not checked: a live SpyServer / rtl_tcp session, the GUI build, the installers (the tag run builds them), a Raspberry Pi.

## 0.16.1 — FT4 is sent where WSJT-X sends it: `tx_buffer_start` is `Modulator.cpp`'s `delay_ms` (3600 samples, 0.300 s), not the DT = 0 placement (#749)

### Changed — transmit

- **FT4 `tx_buffer_start` is WSJT-X's own start, 3600 samples (0.300 s), not 5424** (#749): `Modulator::start` plays FT4
  from `delay_ms = 300` (`Modulator.cpp:167`); 0.16.0 placed it at DT = 0 instead, a divergence chosen without asking.
  `FrameLayout::TX_DELAY_MS` (default `None`, FT4 `Some(300)`) carries it; every other mode is unchanged. Over a chain
  with no latency an FT4 frame now decodes at DT -0.152 s (this crate's decoder, `tests/encoder.rs`; `jt9` not run).
  A simulator wanting DT = 0 uses the new `engine::tx::dt0_buffer_start` (FT4 5424). C / Swift / Kotlin: same field.

### Skimmer

- **Skimmer 1.1.0 is the skimmer on `mfsk-core` 0.16.1** (tag `skimmer-v1.1.0`; the four version files and `package.json` agree). It takes the library by path, so its receive side gets 0.16.0's: a reported FT8 SNR fitted over the decode band as `jt9` does (SD against `jt9` 1.29 → 0.35 dB on ft8_lib's 22 recordings, #747), FT4 subtraction of `gen_ft4wave`'s frame (#739), JT9 candidates picked as `jt9` does and JT9/JT65 `dt` from 1.0 s (#742), JTTY alias frames refused (#728). Transmit changes do not reach it. Also: the SpyServer and rtl_tcp clients now live in the std-only `sdr-io` crate (no behaviour change intended).
  Checked: `skimmer-core` and `sdr-io` tests (release), `cargo check` of the workspace; not checked: a live SpyServer / rtl_tcp session, the installers (the tag run builds them).

## 0.16.0 — every transmit waveform is WSJT-X's own generator's, and one value, `tx_buffer_start`, says where to place it in every mode (breaking for FT4 hosts); JT9 picks its candidates as `jt9` does and JT9/JT65 report DT from WSJT-X's 1.0 s; FT4 subtracts `gen_ft4wave`'s frame; WSJT-X v3.3.0-beta4 → beta6 ported (#726)

WSJT-X v3.3.0-beta4 → beta6, ported (#726): every transmit waveform now follows WSJT-X's own generator, and JT9 decodes as `jt9`
does. Measurements behind each entry: [`docs/notes/WSJTX_BETA6_PORT.md`](docs/notes/WSJTX_BETA6_PORT.md).

### Changed — what a host must change (this is why 0.16 is a minor)

- **Place every transmission at `tx_buffer_start`** (#743): the slot sample the synthesised buffer's first sample goes to, so the
  frame lands at DT = 0 on a WSJT-X receiver. FT4 5424, FT8 6000, the 1 s modes 12 000 (`Encoded::tx_buffer_start(rate)`,
  `ModeMeta::tx_buffer_start_samples_12k`, appended to C `MfskModeInfo`, Swift and Kotlin `txBufferStartSamples12k`).
  `tx_start_offset_s` is the DT reference only. **An FT4 host that placed by it now sends 48 ms late**: FT4's frame is 105 symbols,
  60 480 samples, a ramp symbol before the tones (`gen_ft4wave`). WSJT-X itself starts FT4 at 0.300 s, ~150 ms earlier (marked).
- **JT9 and JT65 `tx_start_offset_s` is 1.0 s, not 0.0** (#742), where WSJT-X starts them (`Modulator.cpp:163`); `Decoder`'s DT was 1.0 s
  above `jt9`'s, and a host placing by it keyed up 1 s early.
- **Types:** `Waveform` is `#[non_exhaustive]` with `Ft4` and `Fst4`; `GfskParams` gains `frame_ramp: bool` (a struct literal needs it);
  `Jt9Extras::search` supplies only the band. `mfsk_synth_lead_len` and the other lead helpers were added and removed within this cycle.
  `FrameLayout::N_RAMP` is the one definition of the lead (`N_RAMP / 2` symbols, registry and engine alike): it must be 2 exactly
  for `Waveform::Ft4`, which `tests/protocol_invariants.rs` checks for every protocol.

### Fixed — transmit

- **FT4, FST4, FT8 match WSJT-X's generators** (#729, #736): FT4 is `gen_ft4wave`'s 105-symbol frame, FST4 `gen_fst4wave`'s (no
  extended first and last symbol, a half-symbol ramp), and the phase is a `u64` turn accumulator (no `f64` per sample, which the
  LX7 cannot afford). Against beta6's own generators on a fixed tone list: FT8 -68 → -95 dB, FT4 -21 → -95 dB, FST4-15 -16 → -88 dB
  (new tier A `tests/gen_wave_vs_wsjtx.rs`). The CoreS3's live `GfskStream` is unchanged.
- **WSPR, JT9, JT65, Q65 match `Modulator::modulate`** (#737): `sin` of a phase advanced first, the same accumulator, per-symbol
  frequency in `f64`. Was `cos` before the advance in `f32`: -35 dB on a 110 s WSPR frame. Within 2e-4 of an `f64` reference.

### Fixed — receive

- **FT8 SNR follows `jt9` on real recordings** (#747): the spectrum baseline `xsnr2` divides by is fitted over the decode band
  (`get_spectrum_baseline.f90` → `baseline.f90`, `nfa..nfb`, 0 outside), not 0-6000 Hz; white noise could not tell the two apart. On
  ft8_lib's 22 recordings against beta6's `jt9`: SD 1.29 → 0.35 dB, rows past 4 dB 7 → 0 (largest 6.4 → 3.4); `qso3_busy.wav` 7.3 →
  0.4 dB. No decode gained or lost (snapshots, tier C FT8 +0.00 dB). The fixed-point (embedded) path has its own SNR, unchanged.
- **JT9 picks its candidates as `jt9` does** (#742): `sync9`'s 0.732 Hz bins walked upwards, the first that passes `ccflim`/`red2lim`
  decoded (`jt9::sync9`), not this crate's own search in score order cut at 32. On `130418_1742.wav` the same rows as beta6's `jt9`
  at `-d 1`/`-d 3`, 1050-1550 and 200-4000 Hz, with and without an Rx window (DT within 0.1 s, SNR 0.5 dB); two DTs were 0.2 s off.
  The crate's own `peakdt9` gate is gone. Tier C now decodes as its `jt9 -d 3` side: -26.58 dB (baseline -26.64), `jt9` only 3 of 300.
- **FT4 subtracts `gen_ft4wave`'s frame** (#739, #734), as `subtractft4.f90` does: the residue of a clean signal goes from -20 dB to
  -29 dB. Three marginal messages leave `ft4_eq_local` (Rust-only equaliser), now guarded by a recall floor of 12 of `jt9`'s 14.
- **Q65 decodes a noiseless frame** (#740, #735; **marked divergence**): the `s3lim` clip base is floored at the range `int16` audio
  carries. On float input with no noise the 40th percentile was f32 rounding and the clip flattened the tones' sidelobes; WSJT-X's
  `int16` input cannot reach that. The floor never engages on the vendored recordings.
- **JTTY refuses a misaligned view of a decoded frame** (#728; upstream PR #702, `is_frame_alias`): upstream's own case went from 4
  false frames in 24 trials to 0, with 24/24 still decoded in full.
- **C: a JT9 decoder refuses `t_early_s`, `t_late_s`, `score_threshold` and `max_cand`** (#744, `MFSK_STATUS_UNSUPPORTED`), as
  every option a mode lacks is: its scan reads none of them since #742, and they were accepted and ignored.
- **WSPR refuses an out-of-range type-2 suffix and type 63** (#727; upstream `ffa884606`), as beta6's `wsprd` does: `unpackpfx` no
  longer wraps. 1 488 rows of the 26 046-row `unpk_` fixture, rebuilt from beta6, change.

### Performance

- **GFSK pulse and NCO `sin` are beta6's polynomials** (#738): no `exp`, division or libm `sin` per sample; within about 1 dB of the
  `erf`/`sinf` form against beta6's generators (FT8 -94 dB, FT4 -96 dB). FT8 `synthesize` on x86_64: 704 → 535 µs. Not measured on
  the LX7.

### Tests

- **All 22 of ft8_lib's real FT8 recordings are tier B**, against every line v3.3.0-beta6's `jt9` prints (402; rc1 401): 15 more
  vendored (`embedded-poc/assets/`, MIT), so the comparison `bench/ft8-compare` made by hand runs on every PR. 402 of 402 found;
  the six outputs `jt9` lacks are named, with their evidence, and any other fails as a phantom; every SNR within 4 dB of `jt9`'s.

### Docs

- **README shortened, 823 → ~230 lines, and its Japanese twin now mirrors it** (was a 58-line summary). It leads with what an operator
  can run and states each fact once. Corrected on the way: FST4W was missing from the protocol and feature tables; FST4 has seven
  periods; the WSJT-X speed column reads `UPSTREAM_EVALUATION.md`'s 2026-10 ratios; the Swift binding is built and tested by CI; the
  design diagram's `core` box is `engine`. Design text moved to `DESIGN_RATIONALE.md` §7.
- **The beta6 port is recorded in `docs/notes/WSJTX_BETA6_PORT.md`**, and the reference docs follow it: TX placement in `LIBRARY`,
  `BINDINGS` and `EMBEDDED` (FT4's citations at beta6's lines), JT9's scan and `SearchTuning` in `LIBRARY` and `CANDIDATES.md`.

## 0.15.1 — FT8 follows `jt9` on real recordings: 401 of `jt9`'s 401 decodes on ft8_lib's 22 recordings (was 383), reported and subtracted where WSJT-X does (`xdt`), `sync8.f90`'s lag grid and candidate dedupe, blind-CQ AP on every candidate, `i3=4` 11-character calls, `ft8b`'s last bit-metric group (#721)

A drop-in patch for 0.15.0: no breaking API change (it adds `engine::LlrTail`, defaulted on the trait). FT8 decodes more on crowded real bands, with no false decode added on the labelled corpora (tier C: no channel worse than the 0.15.0 baseline). Reported FT8 `dt_sec` moves by -5 ms and a few `sync_score` / SNR / hard-error values move; `sic_rounds` and `decode_block` stay the simplified strategies (#722).

- **FT8's multi-symbol bit metrics follow `ft8b.f90` at the end of each half** (new `engine::LlrTail`, defaulted `SyncFrontEnd::LLR_TAIL`; FT8 sets `ContinueIntoNext`, every other protocol keeps the overlap patch it had). `do k=1,29,nsym` runs the last group on into the next (Costas) symbol; the shared sweep re-anchored it on the last data symbols instead (made for FT4, #18). Bits 79-87 and 166-174 of `llrb`/`llrc`/`llre` differed from `jt9`'s in a few signs on identical audio (all-sign agreement with jt9's traced LLRs on 638 candidates: 45 % / 40 % / 66 % before, 94 % / 94 % / 95 % after), enough to lose OSD decodes near 29 hard errors. On ft8_lib's 22 recordings `mfsk-core` now decodes 386 of `jt9 -8 -d 3`'s 401 messages, was 383; FT4/FST4/FST4W unchanged. Also reaches the a7 list decoder (`ft8_a7.f90`/`ft8c.f90` use the same loop).
- **FT8 reports a decode, and subtracts it, where WSJT-X does** (`refine_fine::XDT_OFFSET_S`). `ft8b.f90` / `ft8_a7.f90` return `xdt = (ibest-1)*dt2`, one 200 Hz sample before the start the correlation found, and `subtractft8` starts there; this crate used `ibest`, 5 ms late. A clean +38 dB signal was subtracted to -15.6 dB here and -55.8 dB by `jt9`, now -55.9 dB. Reported `dt_sec` moves by -5 ms (bias against `jt9` +5.1 -> +0.3 ms). On ft8_lib's 22 recordings the default strategy goes 386 -> 390 of `jt9`'s 401 with fewer outputs `jt9` lacks (11 -> 8), 855 -> 861 of 902 crowded synthetic signals (`jt9`: 860). **`sic_rounds(n)` and `decode_block` lose a little in a crowded band** (`qso3_busy.wav`: `sic_rounds(3)` 20 -> 19 messages, the `decode_block` full-parity floor 20 -> 19; 902 synthetic: 838 -> 828): they have no re-subtraction with a refined DT between rounds, which `sic_early` does as `jt9` does, and the old 5 ms error had been hiding it. They are the simplified strategies; `sic_early` is the faithful one (21 of 21 on `qso3_busy.wav`).
- **FT8 closes the remaining gaps to `jt9` on real recordings, plus one in the message filter** (#721). (1) A re-decoded message is subtracted again before it is dropped as a duplicate, as `ft8b.f90:463-467` subtracts every valid decode ahead of `ft8_decode.f90`'s dupe check: a signal left by a shallower first subtraction, or a splattering signal's copies at other frequencies, no longer buries its neighbours. (2) `coarse_sync`'s lag grid is `sync8.f90`'s (`jstrt` truncated to 12, 1-based; `JZ` 62): it was two steps (80 ms) off on identical `sync2d` values, so coarse DT, the `±MLAG` window and `ft8b`'s ±10-sample start all differed. The fixed-point `nstep-half` grid (80 ms steps, no upstream counterpart) keeps its rounded origin: the same formula there costs the board's driver 5 of 7 floor decodes. (3) The blind-CQ AP pass runs on every ladder candidate, as iaptype 1 does upstream; the `nsync >= 12` cost gate is gone. (4) An `i3=4` call may be up to 11 characters (`c58`), so `<...> DA0FONTANE` is reported as `jt9` reports it. (5) `coarse_sync` dedupes on the integer-lag DT `(lag-0.5)*tstep`, as `sync8.f90:143` does, not on the parabolically interpolated one: a weak candidate 3 Hz from a +22 dB signal at exactly the 0.04 s window (`jt9` DT 0.94 vs 0.90; 0.931 here) was dropped, and with it the second and third decode and subtraction of that signal, so `CQ UB3AQS KO85` stayed buried. Only for candidate lists of 60 or more (`FAITHFUL_DEDUPE_MIN_CAND`): at the embedded cap of 30 the extra near-duplicates pushed `OH3NIV ZS6S -03` out of `qso1.wav` (4 -> 3 of 5 in `ft8_decode_block_real_qso`), so a lean list keeps the interpolated-DT dedupe. (6) `decode_block_multipass` (host only) also subtracts a re-found message before dropping it; the `decode_block` full-parity floor is back at 20.
  Measured: ft8_lib's 22 recordings 390 -> **401 of `jt9`'s 401** (default `sic_early`; 7 outputs `jt9` lacks, 6 attested elsewhere in the session, `SP7NHS YC2FCE -15` undecided), `sic_rounds(3)` 381 -> 387; `qso3_busy.wav` `sic_rounds(3)` back to 20 messages; the labelled busy (2900 signals) and dense (4102) corpora: no false decode in any set, none in the 200 noise-only files, recall within +3 / -2 signals of before (dense `real`, 902 signals: identical). Reported SNR on the dense set, 3 644 decodes both make: mean difference to `jt9` +0.02 dB, 90 % identical, all but 4 within 1 dB (both about -0.4 dB against the injected SNR). Decode time about +17 % (1.29 s per file against `jt9`'s 3.57 s, single thread); random-payload survival through the codec verdict +1.3 %. The embedded (non-`fft-rustfft`) `decode_block` never subtracts; the cold-start DT bootstrap moved with the grid (22 recordings: median error 0.140 -> 0.080 s; one fixture's budget 130 -> 180 ms). Detail: `docs/notes/FT8_REAL_VS_JT9.md`.
- **Tests:** `ft8_real_recordings_vs_jt9` also holds five more vendored ft8_lib recordings (`191111_110615/110645`, `websdr_test2/5/11`), one per #721 fix; each fails without its fix. `ft8_noise_slots_decode_nothing` (tier B): 24 noise slots per condition, no hint and a CQ hint, `max_extra: 0`, for the blind-CQ pass that now runs on every candidate. `ft8::llr` checks all five LLR sets against a direct port of `ft8b.f90`; `sic_early_eq_mode_reaches_sic_engine` no longer pins one noise seed (its "reliably" was 1 draw of 12). 18 FT8 snapshot fixtures moved again, no message added or lost (`decode_snapshot.rs` header lists the rows). Measurement and what is still different: `docs/notes/FT8_REAL_VS_JT9.md`.

### Skimmer

- **Skimmer 1.0.2 is the skimmer on `mfsk-core` 0.15.1** (tag `skimmer-v1.0.2`; no skimmer code change, the four version files and `package.json` agree). It takes the library by path, so its FT8 channels get the #721 changes above: the default `sic_early` strategy decodes 401 of `jt9`'s 401 messages on ft8_lib's 22 recordings (383 under 0.15.0), and a reported FT8 `dt` moves by -5 ms. The skimmer uses neither `sic_rounds` nor `decode_block`, so the loss the entry above names for those does not reach it. Checked: `skimmer-core` tests (`--release`), `cargo check` of the workspace, the tag run's installers. Not checked: a live SpyServer or `rtl_tcp` session on this build.

## 0.15.0 — the public surface narrowed to what applications use: `engine::pipeline` crate-private and no public path behind `internal-testing` (breaking), `#[non_exhaustive]` on the growing option/result/error types (breaking), registry capability bits that describe the API (breaking), candidates and decode method readable from a result, `Row::new` / `SlotResult::new`, a parameter block WSJT-X's GUI cannot send no longer crashes a decoder, `RR73` and `/R` `/P` AP follow WSJT-X 3.3 beta

### Breaking changes

- **`engine::pipeline` is crate-private and no public path depends on `internal-testing` any more** (#693, closes #420). The types it
  produced moved to `engine::frame`: `DecodeResult`, `DecodeStrictness`, `DecodeDepth`, `LlrEffort`, `FftCache`, `BudgetCheck`,
  `BudgetReport` (the `ft8::decode` / `ft4::decode` / `fst4::decode` / `decoder` re-exports are unchanged). What the crate's tests and
  the embedded builds reached through `internal-testing` is now `mfsk_core::__internal::{pipeline, protocol, msg, fst4, wspr, jt9,
  jt65, q65, fec, sync2d, ft8}` (hidden from the docs, no semver promise): `engine::pipeline::process_candidate_basic` is
  `__internal::pipeline::process_candidate_basic`, `msg::decode_request` is `__internal::msg::decode_request`, and so on.
  `Ft4CoarsePhasors::buffer_addrs()` is `__internal::sync2d::buffer_addrs(&phasors)`. `mfsk-ffi` no longer enables the feature.
- **`#[non_exhaustive]`** on the option, result and error types that will grow: `jtty::rx::Params`, `jt65::{ChaseParams, HintSpec}`,
  `uvpacket` `AfcOpts` / `MultiChannelOpts` / `DecodedFrame` / `SlotEnergy` / `SyncStats` and its three error enums, `decoder::{Depth, ApMode,
  QsoProgress}`, `Jt9Depth`, `GridDepth`, `jtty::rx::{SideChannels, Subtracted}`, `jtty::pack::{ExchangeProfile, PackError}`, the msk144 and jtty result structs, `iq::RateTrackConfig`, `q65::{Caller, DxFromHistory}`. Build an
  option from `Default` and assign; add a wildcard arm to a `match`. `WsprMessage` and `Jt72Message` stay exhaustive (the variants are
  the message formats). Tier A+B unchanged. Migration: `LIBRARY.md` §1.4 (and `.ja.md`).
- **Registry capability bits describe the public API** (#698). Bit 0 is `caps::WSJT77_FAMILY` (the FT8 / FT4 / FST4 decoders: `ap_hint`,
  `eq`, `filter`, the QSO-context AP ladder); `DECODE_HANDLE` is a deprecated alias and never meant "a decoder opens", which is the new
  `caps::DECODER` (bit 18: every `Mode`; C `MFSK_CAP_DECODER`, what `mfsk_decoder_open` accepts). Bits 10-12 (`KNOWN_FILTER`,
  `KNOWN_SUBTRACT`, `FFT_CACHE`) described the 0.12 builders, which no public API has had since 0.13: removed from Rust, kept in C /
  Kotlin / Swift as deprecated names that no mode sets, numbers not reused. `tests/registry_caps.rs` checks bits 0 and 18 against
  `AnyDecoder`, `mfsk-ffi/tests/mode_introspection.rs` bit 18 against `mfsk_decoder_open`. C migration: `DECODE_HANDLE` → `WSJT77_FAMILY` or `DECODER`, drop tests of 10-12.
- **The registry carries no default search parameters** (#698). `registry::DecodeDefaults`, `DecodeProfile::defaults` and
  `DecodeProfile::sniper_max_cand_cap` are gone: nothing read them but `default_params`' fallback band, and FT8 / FT4 / FST4's values
  described no decoder (e.g. FT8's 100-3000 Hz and `sync_min` 0.8, FT4's sniper cap 15). `decoder::default_params(mode)` is the one source and
  states each mode's band (WSPR / JT9 / JT65 / Q65 from their `search::default_search_params()`, FST4W `jt9 -W`'s 1400-1600).
  `DecodeProfile` keeps `caps` and `sync_scale`. No mode's defaults moved: `tests/default_params.rs` pins all 26 at their 0.14 values.
  C ABI unchanged. Detail: `LIBRARY.md` §8.1.
- **`encoder::Encoded`'s fields are private** (#700): read `mode()`, `tones()`, `text()`, `hashed_calls()`. The tones are always the ones
  `encode` made for the mode, so `synthesize` cannot be handed a wrong count or tone; its doc claimed a panic on an unsupported sample
  rate, and none was found (one sub-mode per family, 0 Hz to 192 kHz: too low a rate gives degenerate audio, 0 Hz none). The GFSK ramp
  at a rate other than 12 kHz is computed in `u64` (FST4-1800 at 24 kHz overflowed a 32-bit `usize`). Tones from elsewhere
  go to `engine::tx::synthesize::<P>`, which checks them. `Decoded::protocol` stays the family on purpose: the exact sub-mode is the
  decoder's (`AnyDecoder::mode()`), documented. Nothing in the workspace wrote the fields; C / Kotlin / Swift unchanged. `LIBRARY.md` §1.4.

### Added

- **`Row::new(decoded, native)` and `SlotResult::new(rows)`** (#712): an application builds the rows and results it consumes, for a test of a
  display, a logger or a replay, without decoding audio (both are `#[non_exhaustive]`, so a struct expression is closed outside the crate).
  Default `RowDetail` / `BudgetReport`, `candidates` `None`; assign a field when it matters, as `CompletedSlot::new` (#592). `tests/decoder_result_new.rs`
  builds them from outside the crate. Additive.
- **`decoder::params_read(mode)`: which `DecodeParams` fields a mode's decoder can read** (#698), as `ParamFields` / `ParamField`; C
  `mfsk_mode_fields_read` + `MFSK_FIELD_*`, Kotlin `Mfsk.fieldsRead` / `FIELD_*`, Swift `Mode.paramsRead`. A field outside the set changes
  nothing for that mode, so a host can say which settings are effective; conditions per field are in the doc (`LIBRARY.md` §2.2). Found
  while building it: WSPR reads no band (wsprd takes none), JT65 no AP setting (#703), no mode `station.grid`.
  `tests/params_read.rs` sets every unread field to a value that would matter on a synthetic signal of each decoder and requires the
  same decode. Additive; no decode changed.
- **`RowDetail::method` and `SlotResult::candidates`: what an application can read about a decode** (#696). `method: Option<DecodeMethod>` says how a row was decoded (`Bp`, `Osd`, `Ap { iaptype }`,
  `ListA7`, `ListA8`; `None` for WSPR, JT9, JT65, Q65, FST4W) where `pass` is a number that means something else per mode and that FT4 / FST4 give twice (5 is both OSD depth 3 and an a-priori decode);
  `pass` stays, its table is in `docs/notes/PASS_IDS.md`. `candidates` is what the first coarse search of the call kept (FT8, FT4, FST4; per call, so FT8's `SicEarly` reports checkpoint A's in a whole-period call and C's in a prefix sequence's last; `None` for a call that made none, as FT8's checkpoint B); the engine's `DecodeOutcome` gains the same field. **#697: WSPR, JT9, JT65 and Q65 report it too** (every mode but FST4W; per-mode definitions in `docs/notes/CANDIDATES.md`: WSPR's is pass 0's list and is not narrowed to the band as `wsprd`'s `npk` is, JT9's the truncated list, JT65's pass 1's, Q65's the first scan's). `method`'s `iaptype` is WSJT-X's number only for the QSO context's hypotheses: rows of a free-form `ap_hint` are `Ap { iaptype: None }`.
  C ABI (appended, ABI 3): `MfskDecode::{method, iaptype}` (`MFSK_METHOD_*`), `MfskBudgetReport::candidates`; Kotlin `MfskMethod`, `iaptype`, `candidates`; Swift `DecodeMethod`. Additive.
- **`decoder::PeriodBudget`: one compute allowance for the calls of a period** (#626, #627): `share(period, mode, fraction)`, `left()`, `finish(took, &budget_report)` (true once per
  period, on the first call that ran out) and, with `std`, `within(start)`. Moved from the skimmer's `OpenPeriod`; the accounting holds no clock.
- **A channel takes its mode and prefix points from its decoder** (#692): `iq::ChannelSpec` (`From<Mode>`, `From<&AnyDecoder>`, `From<&Decoder<P>>`),
  `IqReceiver::add_channel(dial, impl Into<ChannelSpec>)` (a bare `Mode` value still compiles; an inferred `m.into()` argument now needs its type) and `update_channel` (applies from the next slot; another mode is
  `IqError::ModeChange`). `Decoder::decode_slot` / `decode_slot_with` and the `AnyDecoder` twins take the `CompletedSlot` and pick `decode_prefix` or `decode`
  themselves; a slot of another mode is `Err(ModeMismatch)`. Measured: points on the channel for a decoder that wants none return the same rows
  (`qso3_busy`, 14 / 14, SNR within 0.1 dB; `docs/notes/IQ_PREFIX_DESIGN.md` §10). Skimmer: no second `mode`, no throwaway decoder to ask for points, and the points
  follow the options only once the decoder has them. Additive.

### Documentation

- **Changing only the band keeps the mode's defaults** (#711, part 1): `DecodeParams::for_band`'s rustdoc called its block "the GUI's defaults",
  but it is the bare block (AP `Full`; FT8's and JT65's GUI start with AP off). The protocol modules' rustdoc examples, the prelude's and
  `STREAMING.md`'s (and `.ja.md`) now build from `default_params(mode).band(lo, hi)` or `with_defaults()` (WSPR, which reads no band);
  `tests/default_params.rs` checks that `band()` touches nothing else and that `for_band` is not a mode's defaults. No decoder changed.
- **The quick starts take one path** (#708): README and `LIBRARY.md` §1.1 encode with `encode` + `TxAudio` as rustdoc and §1 already did (README's
  hand-written pack77 → tones → waveform became a pointer to §5), all decode from the mode's defaults (`with_defaults()`, not `for_band`'s AP `Full`),
  and rustdoc / §1 say when to hold an `AnyDecoder` (mode picked at run time, README's path) instead of `Decoder::<Ft8>`.
  The rustdoc `Cargo.toml` snippets said `0.12`; `tests/doc_dependency_versions.rs` reads the `mfsk-core = "X"` / `{ version = "X" … }` lines of
  rustdoc, README and `docs/reference/*.md` against the crate's `major.minor`. §2.4's `AnyDecoder::decode_with` callback is `Row<()>`; §1.4 says the reference describes `main`.

### Deprecated

- **`IqReceiver::set_prefix_points`** (since 0.15, still works): `update_channel` with a `ChannelSpec` from the decoder replaces it (#692). No removal date is set.

### Results

- `RR73` is packed as the acknowledgment `MAXGRID4+3` (32403), not the grid 32373, for every non-CQ type 1/2 message in all 77-bit
  modes (WSJT-X v3.3.0-beta3 `58c7a522f`, #694 A1). The AP lock on `RR73` follows (`2c1b22b6c`, A2): in FT8, FT4 and FST4 it now
  matches beta3+ senders, not <= 3.2 senders. Q65 (fix): `genq65` has always sent 32403 while rc1's `q65_ap.f90` locked the
  grid form, which no Q65 sender transmits; the lock now matches every Q65 sender. Unpack is unchanged (both values read `RR73`);
  `q65_rr73_fixup` stays (`genq65.f90:43`). Tier A+B: no decode or golden change.
- **FT8 a8 and Q65's full-AP list give a standard call with a `/R` or `/P` suffix the full candidate set** (#694, WSJT-X v3.3.0-beta4
  `015b48945`, `6b6124f9d`, `b2204822b`): `stdcall_suffix` keeps such a call standard, and 207 `CQ call`, 208-211 `<MyCall> call` +
  nothing/RRR/RR73/73 and 212-411 `MyCall <call>` + the report ladder (FT8 cuts it at +-30 dB, Q65 does not) are added, 411 in all
  with a standard MyCall. a7 is unchanged, as upstream. Other calls still get 206. **`q65::MAX_AP_CODEWORDS` 206 -> 411**; the list
  grows only for such DX calls.

- **A block WSJT-X's GUI cannot send no longer crashes or hangs a decoder** (#717). `Decoder` hands its mode the block normalised: band
  edges clamped into 0-6000 Hz (a NaN edge to that end), a non-finite Rx / Tx frequency or a non-finite or negative tolerance unset;
  `params()` still shows what was set. Before, on one synthetic signal per mode, an infinite band top panicked FT8 / FST4 (debug) and Q65
  (release) and did not return from JT9 / JT65; a NaN Rx frequency or tolerance lost the FST4W decode. Every value the GUI can send passes
  through bit-identically: tier A+B unchanged. `tests/decoder_params_normalised.rs`.

### Skimmer

- An rtl_tcp connection says what it waits for: `Event::Preparing { stage, up_to_s }` at the start of "waiting for the server to
  apply the settings" (up to 20 s) and "measuring the sample rate" (35 s), shown in the GUI's server line and printed by the CLI;
  the ~56 s between Connected and Decoding were silent.

Older releases are archived out of this file to keep it skimmable. The rule:
this file holds the unreleased section and the two latest minor series, and
each new minor's release PR moves the oldest series out.
0.14.0 in
[`docs/historical/CHANGELOG-0.14.md`](docs/historical/CHANGELOG-0.14.md),
0.13.0 – 0.13.1 in
[`docs/historical/CHANGELOG-0.13.md`](docs/historical/CHANGELOG-0.13.md),
0.12.0 in
[`docs/historical/CHANGELOG-0.12.md`](docs/historical/CHANGELOG-0.12.md),
0.11.0 in
[`docs/historical/CHANGELOG-0.11.md`](docs/historical/CHANGELOG-0.11.md),
0.8.0 – 0.10.1 in
[`docs/historical/CHANGELOG-0.8-0.10.md`](docs/historical/CHANGELOG-0.8-0.10.md),
0.6.0 – 0.7.4 in
[`docs/historical/CHANGELOG-0.6-0.7.md`](docs/historical/CHANGELOG-0.6-0.7.md),
0.1.0 – 0.5.12 in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
