# Changelog

## Unreleased

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
