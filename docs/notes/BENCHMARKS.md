# Benchmarks vs. WSJT-X

Current-state results only — no release history here. For "how did we
get to these numbers" narrative, see `CHANGELOG.md` (per-release
detail) or `docs/notes/ROADMAP.md` (open follow-ups). For "how do I
reproduce this sweep myself", see the protocol-specific
`*_BENCHMARK.md` docs linked per section below.

Two kinds of numbers appear per protocol:

- **Golden-WAV recall** — decode a real WSJT-X-distributed recording,
  compare messages/frequency/timing (and for MSK144, SNR) against a
  known-correct reference decode.
- **AWGN sensitivity sweep** — a `*sim`-generated corpus swept across
  SNR points, giving a 50%-recall crossing to compare against WSJT-X's
  published or measured threshold.

## Summary

| Protocol | Golden-WAV recall | AWGN gap vs. WSJT-X | Status |
|----------|-------------------|----------------------|--------|
| FT8      | 7/8 (WSJT-X), 17/18 (JTDX) | AWGN ≈ −20.8 dB (WSJT-X: −20 to −21 dB) | at parity |
| FT4      | 6/6 | AWGN ≈ −17.2 dB (WSJT-X: −17.5 dB, ~0.3 dB gap) | at parity |
| FST4     | 1/1 (FST4-60A only) | 0.10-0.60 dB across 5 sub-modes | at parity |
| WSPR     | 8/8 | AWGN 50% ≈ −29.8 dB, matches published sensitivity floor | at parity |
| JT9      | 5/5 | AWGN 50% ≈ −26.3 dB, no measurable gap vs. `jt9 -9` | at parity |
| JT65     | none available | **~7-8 dB** at deep SNR | known gap, deprioritized |
| Q65      | 2 real EME recordings | 0.2-1.4 dB vs. analytical target across 10 sub-modes; matches/beats WSJT-X's own decode with CQ-AP hint | at/above parity |
| MSK144   | 3/3 (incl. exact SNR match) | AWGN 50% ≈ −5.2 to −5.8 dB, 25/28 cells exact match vs. a real `jt9` build | at parity |

All AWGN 50%-crossing figures below are linear-interpolated between the
nearest swept SNR points, in each `*sim` generator's 2500 Hz
reference-bandwidth convention (matches WSJT-X's own published
numbers directly, no unit conversion needed).

## Decode speed (single golden-WAV, host)

Wall-clock time for one full decode call against each protocol's real
WSJT-X-recorded golden WAV (see the recall sections below for which
file and which decode function). Measured 2026-07-20, single run per
row — treat as an order-of-magnitude reference, not an averaged
benchmark.

**Compute environment**: AMD Ryzen 9 9900X (12C/24T), 32 GB RAM,
Ubuntu 24.04.2 LTS under WSL2 (kernel 6.6.87.2-microsoft-standard-WSL2),
rustc 1.97.1, `cargo test --release --features full` (includes the
`parallel` feature — some decode paths use `rayon` internally, so this
is wall time on a many-core host, not a single-thread figure).

| Protocol | Golden WAV | Slot length | Decode time |
|---|---|---:|---:|
| Q65-120D | 210117_0920.wav (rainscatter, fading metric) | 120 s | 0.15 s |
| Q65-120E | 6 m ionoscatter (fading metric) | 120 s | 0.32 s |
| JT9 | 130418_1742.wav | 60 s | 0.33 s |
| Q65-60D | 201212_1838.wav (10 GHz EME, fading metric) | 60 s | 0.39 s |
| FT8 | qso3_busy.wav (16-signal busy band) | 15 s | 0.45 s |
| MSK144 | 181211_120800.wav | 30 s | 0.84 s |
| MSK144 | 181211_120500.wav | 15 s | 0.88 s |
| WSPR | 150426_0918.wav | 120 s | 0.93 s |
| Q65-300A | 201210_0505.wav (optical scatter, fading metric) | 293.8 s | 1.05 s |
| FT4 | 000000_000002.wav | 7.5 s | 1.20 s |
| Q65-60A | 6 m EME (plain BP + AP) | 60 s | 1.57 s |
| Q65-60B | 1296 MHz troposcatter ×1 slot (multi-period averaging) | 60 s | 1.89 s |
| Q65-30A | 6 m ionoscatter ×4 slots (multi-period averaging) | 4×30 s | 2.55 s |
| FST4-60A | 210115_0058.wav | 60 s | 2.60 s |

Notes:

- These are real-recording decode times, not synthetic sweeps — they
  reflect actual candidate density and search cost on real audio, not
  a clean-signal best case.
- FT8's `qso3_busy.wav` was the outlier at **4.73 s** (busy band, 16
  simultaneous signals) until a profiling pass (2026-07-20) found the
  cost wasn't BP/OSD at all but `subtract_tones_lpf`'s successive-
  interference-cancellation step: a naive O(candidates × NFRAME ×
  lpf_half) direct time-domain convolution, ~310 ms per accepted
  decode. Replaced with WSJT-X's own algorithm
  (`lib/ft8/subtractft8.f90`/`lib/ft4/subtractft4.f90`) — a cached
  filter-response FFT + one forward/inverse FFT pair per call,
  O(N log N) — dropping this row to 0.45 s (~10.5×) with byte-identical
  recall (7/8 golden, 7 phantom, 14 total). FT4's golden test was
  already on a different (non-LPF) subtract path and unaffected.
- Q65-300A (293.8 s slot, ~20× FT8's audio length) still only takes
  1.05 s — the fast-fading metric's per-candidate cost dominates, not
  a full-buffer rescan. An earlier profiling pass found this same
  golden test took 8.95 s before an unasserted diagnostic pre-check
  was removed (see CHANGELOG); 1.05 s reflects the load-bearing decode
  path only.
- Not comparable to the embedded (Xtensa) numbers quoted elsewhere in
  this doc (e.g. FT8's ~0.7-1.2 s post-SlotEnd) — those run a
  different no_std/fixed-point pipeline on a much slower MCU core;
  this table is host x86_64 only.

## FT8

- **WSJT-X 8-entry golden: 7/8** (`decode_frame_with_ap` host,
  `decode_block` embedded — same result both paths).
- **JTDX 18-entry golden: 17/18** (`decode_block`). The one miss,
  `WA2FZW DL5AXX`, is classified a likely JTDX false positive —
  `coarse_sync` candidates exist at its claimed frequency but no AP
  context recovers a message.
- **Host AP-on multipass JTDX-extras: 5/6** via
  `decode_frame_subtract_with_ap`. The remaining miss (`K1BZM DK8NE`,
  fixed AP context `mycall=K1JT`/`hiscall=HA0DU`) needs a wider
  AP-list / callsign hash table.
- **Embedded (M5StickS3, Xtensa LX7, fixed-point)**: 6/18 + 1 bonus =
  7 total on the same WAV, in ~1.19 s post-SlotEnd via the streaming
  pipeline. M5Stack Core2 (LX6): ~2.8 s.
- **SNR calibration**: `xsnr2_db_simple` lands reported SNR within
  ±3 dB of JTDX absolute on real silicon.
- CCIR moderate/poor fading recall gap closed in 0.7.3 by widening
  `OSD_HARDERRORS_MAX` back to WSJT-X's universal 36.

**AWGN/CCIR sensitivity sweep** (`tests/ft8_sweep.rs`, `ft8sim`-driven,
`-5` to `-26` dB grid, 4 channel conditions):

| Channel | mfsk-core 50% crossing | WSJT-X published | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −20.8 dB | −20 to −21 dB | within range |
| CCIR good | ≈ −20.6 dB | — (no separate WSJT-X figure) | — |
| CCIR moderate | ≈ −18.6 dB | — | — |
| CCIR poor | ≈ −18.5 dB | — | — |

Reproduce: `docs/notes/FT8_BENCHMARK.md`.

## FT4

- **6/6 WSJT-X golden** (`samples/FT4/000000_000002.wav`).
- **AWGN sensitivity gap: ~0.3 dB** (was ~1.8 dB pre-0.7.3) — 50%
  recall crossing moved from −15.5 dB to −17.2 dB after a coherent
  Costas-block scorer fix and an OSD-attempt gate that had been
  checking a non-coherent score.
- Successive-interference-cancellation primitives
  (`subtract_signal*`, `refine_signal_freq`) ported from
  `lib/ft4_subtract.f90`; WSJT-X's Fast/Normal/Deep decode-depth menu
  exposed via `decode_frame_with_options`.

**AWGN/CCIR sensitivity sweep** (`tests/ft4_sweep.rs`, `ft4sim`-driven,
4 channel conditions):

| Channel | mfsk-core 50% crossing | WSJT-X published | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −17.2 dB | −17.5 dB | 0.3 dB |
| CCIR good | ≈ −17.3 dB | — (no separate WSJT-X figure) | — |
| CCIR moderate | ≈ −15.75 dB | — | — |
| CCIR poor | ≈ −16.1 dB | — | — |

Reproduce: `docs/notes/FT4_BENCHMARK.md`.

## FST4

Five T/R-period sub-modes wired (FST4-15/30/60A/120/300), every
constant verified directly against WSJT-X `fst4_decode.f90` /
`fst4sim.f90` source.

| Sub-mode | mfsk-core 50% crossing | WSJT-X published | Gap |
|----------|------------------------:|------------------:|----:|
| FST4-15  | ≈ −20.60 dB | −20.7 dB | 0.10 dB |
| FST4-30  | ≈ −23.90 dB | −24.2 dB | 0.30 dB |
| FST4-60  | ≈ −27.62 dB | −28.1 dB | 0.48 dB |
| FST4-120 | ≈ −30.70 dB | −31.3 dB | 0.60 dB |
| FST4-300 | ≈ −34.78 dB | −35.3 dB | 0.52 dB |

Closed via a coherent full-slot local sync search, an FST4-specific
`LLR_NSYM_MAX` override, an nsym=4 LLR rung, and a zsum-OSD fallback.

Only FST4-60A has a real-recording golden-WAV lock (1/1,
`samples/FST4/210115_0058.wav`) — WSJT-X's sample tree ships no
FST4-15/30/120/300 recordings, so those four are validated by
synth-roundtrip self-consistency plus `fst4sim` sweep only, not a real
on-air recording. FST4-900/1800 and FST4W (the WSPR-style one-way
beacon variant) remain out of scope — no user demand as of writing.

Reproduce: `docs/notes/FST4_BENCHMARK.md`.

## WSPR

- **8/8 WSJT-X golden** (`samples/WSPR/150426_0918.wav`), ~0.88 s
  end-to-end on a desktop build — sub-bin demod + 2-pass
  subtract+re-coarse + OSD-2 fallback + Type-3 phantom filter.

**AWGN sensitivity sweep** (`tests/wspr_sweep.rs`, `wsprsim`-driven,
13 SNR points × 20 trials each):

| SNR | Recall |
|---:|---:|
| −34 dB | 0.0% |
| −32 dB | 0.0% |
| −31 dB | 0.0% |
| −30 dB | 40.0% |
| −29 dB | 95.0% |
| −28 dB | 95.0% |
| −27 dB | 100.0% |
| −26 dB | 100.0% |
| −24 dB | 100.0% |
| −20 dB | 100.0% |
| −15 dB | 100.0% |
| −10 dB | 100.0% |
| 0 dB | 100.0% |

50% crossing ≈ −29.8 dB — consistent with WSJT-X's published WSPR
sensitivity floor (commonly cited around −28 to −30 dB, 2500 Hz
reference bandwidth).

## JT9

- **5/5 WSJT-X golden** (`samples/JT9/130418_1742.wav`) via the full
  WSJT-X-faithful softsym pipeline (`afc9` + `chkss2` + `xx0` mettab +
  `sync9` per-freq collapse).

**AWGN sensitivity sweep** (`tests/jt9_sweep.rs`, `jt9sim`-driven,
20 trials/SNR):

| SNR | Recall |
|---:|---:|
| −30 dB | 0.0% |
| −28 dB | 0.0% |
| −27 dB | 10.0% |
| −26 dB | 65.0% |
| −25 dB | 90.0% |
| −24 dB | 100.0% |
| −22 dB | 100.0% |
| −20 dB | 100.0% |
| −18 dB | 100.0% |
| −15 dB | 100.0% |
| −10 dB | 100.0% |
| −5 dB | 100.0% |
| 0 dB | 100.0% |
| +5 dB | 100.0% |
| +10 dB | 100.0% |

50% crossing ≈ −26.3 dB — no measurable gap vs. a real WSJT-X `jt9 -9`
build on the identical 300-file corpus (100% to −25 dB, 80% at
−26 dB there; per-cell differences are within 20-trial sampling noise
at the steep part of the curve).

## JT65 — known gap, deliberately not closed

- No real-recording golden WAV available: WSJT-X's own v3 reference
  samples need soft-symbol erasure metadata that lives in private
  WSJT-X branches.
- This crate's hard-decision `decode_at`/`decode_at_with_erasures` hits
  50% recall around **−14 dB** and near-zero below −19 dB, while
  WSJT-X's own no-`kvasd` path (`jt9 -6`) holds ~100% down to −22 dB on
  the identical corpus — a real **~7-8 dB gap**.
- **Root cause**: `jt9 -6` isn't plain hard-decision RS either —
  `lib/extract.f90` calls `ftrsdap` (`lib/ftrsd/ftrsdap.c`), a
  stochastic Chase decoder that runs many randomized soft-symbol
  erasure-pattern trials around Berlekamp-Massey RS, using both the
  most- and second-most-reliable symbol per position. This crate only
  tries a single deterministic increasing-erasure-count ordering — a
  materially weaker algorithm.
- **Deliberately deprioritized** ([#169](https://github.com/jl1nie/mfsk-core/issues/169)):
  Q65 already covers JT65's deep-SNR narrowband use case across a
  wider sensitivity range (10 wired sub-modes), and JT65 on-air
  traffic has largely migrated to it. Porting `ftrsdap` is a
  substantial, JT65-specific algorithmic port with no other payoff —
  not planned unless there's an actual request for deeper JT65 recall.

**AWGN sensitivity sweep** (`tests/jt65_sweep.rs`, `jt65sim`-driven,
20 trials/SNR):

| SNR | Recall |
|---:|---:|
| −25 dB | 0.0% |
| −22 dB | 0.0% |
| −20 dB | 0.0% |
| −19 dB | 0.0% |
| −18 dB | 5.0% |
| −17 dB | 15.0% |
| −16 dB | 30.0% |
| −15 dB | 25.0% |
| −14 dB | 50.0% |
| −12 dB | 45.0% |
| −10 dB | 60.0% |
| −5 dB | 80.0% |
| 0 dB | 100.0% |
| +5 dB | 100.0% |
| +10 dB | 100.0% |

(The −15 dB / −12 dB dips relative to their neighbors are 20-trial
sampling noise, not a non-monotonic decoder — visible on the steep
part of every sweep in this doc at this trial count.)

## Q65

- Real recordings: WSJT-X's 6 m EME (W7GJ exchanges) and 10 GHz EME
  reference both decode.
- Fast-fading metric (Gaussian/Lorentzian channel models) recovers
  5-8 dB on Doppler-spread channels, required for microwave EME.
- AP-list template matching decodes 6/6 frames at SNR −25 dB where
  plain BP fails 0/6.

**AWGN sensitivity sweep** (`tests/q65_sim_sweep.rs`, `q65sim`-driven,
15 trials/SNR for the 15/30/60 s sub-modes, 5 trials/SNR for the
120/300 s ones — proportionally longer audio + the fine-timing retry
below multiply decode cost). All 10 wired sub-modes, plain
(`decode_scan_for`, no assumptions) 50% crossing vs. `q65params.f90`'s
analytical AWGN threshold (`-27 + 10*log10(7200/nsps)`, WSJT-X's own
formula — depends only on T/R period, not sub-mode letter):

| Sub-mode | mfsk-core 50% crossing (plain) | `q65params.f90` target | Gap |
|----------|--------------------------------:|------------------------:|----:|
| Q65-15A  | ≈ −21.9 dB | −21 dB | 0.9 dB |
| Q65-30A  | ≈ −24.7 dB | −24 dB | 0.7 dB |
| Q65-60A  | ≈ −27.9 dB | −27 dB | 0.9 dB |
| Q65-60B  | ≈ −27.9 dB | −27 dB | 0.9 dB |
| Q65-60C  | ≈ −28.2 dB | −27 dB | 1.2 dB |
| Q65-60D  | ≈ −28.4 dB | −27 dB | 1.4 dB |
| Q65-60E  | ≈ −27.9 dB | −27 dB | 0.9 dB |
| Q65-120D | ≈ −30.8 dB | −30 dB | 0.8 dB |
| Q65-120E | ≈ −31.2 dB | −30 dB | 1.2 dB |
| Q65-300A | ≈ −35.2 dB | −35 dB | 0.2 dB |

- **CQ-AP-hinted path matches WSJT-X almost exactly.** WSJT-X's
  default `jt9` decode always has free access to the "CQ ??? ???" AP
  hypothesis, so every real `jt9` decode on CQ traffic implicitly gets
  AP-list benefit — a fair comparison needs this crate's
  `decode_scan_with_ap_for` + a `"CQ"` hint, not the blind baseline
  above. That comparison closes the gap almost exactly (e.g. Q65-30A
  −26 dB: 0%→40%, matching WSJT-X's own reported 40%; Q65-60A −28 dB:
  47%→93%).
- **Direct cross-check against a real `jt9 -3` build on
  Q65-120D/120E/300A found no regression, and two sub-modes exceed
  WSJT-X's own plain decode** by 2.5-3.4 dB (Q65-120D ≈−28.2 dB,
  Q65-120E ≈−27.6 dB for `jt9 -3` vs. this crate's ≈−30.8/−31.2 dB
  above) — because `decode_scan_for`'s fine-timing retry runs
  unconditionally, while `jt9 -3` without `-c`/`-x` doesn't get it.
  Q65-300A's curve is statistically identical to `jt9`'s own at every
  tested SNR point (both cross 50% at ≈−35 dB).

## MSK144

- **3/3 WSJT-X golden** across both `samples/MSK144/*.wav` recordings
  — message, frequency, timing, **and SNR** all gated (a systematic
  −1 dB SNR bias found post-ship was root-caused to a missing fixed
  1500 Hz-centered bandpass filter in the analytic-signal front end
  and fixed, closing the gap to an exact match).
- **AWGN sensitivity cross-validated directly against a real WSJT-X
  `jt9 -k` build** on `msk144sim`-generated signals: 25 of 28
  (ping-length × SNR) cells matched exactly, the other 3 differed by
  exactly 1 file out of 20 — no measurable recall gap at any tested
  SNR.
- MSK40 (the legacy shorthand mode) and the *adaptive* RX-equalizer
  training loop (as opposed to the fixed bandpass filter above, which
  is ported) remain out of scope.

**AWGN sensitivity sweep** (`tests/msk144_snr_sweep.rs`, self-contained
`msk144sim`-recipe synthesis, 20 seeds/SNR):

| SNR | Short ping (TRp=15s, width=0.12) | Long ping (TRp=30s, width=2.5) |
|---:|---:|---:|
| −9 dB | 0.0% | 0.0% |
| −8 dB | 0.0% | 0.0% |
| −7 dB | 0.0% | 0.0% |
| −6 dB | 40.0% | 0.0% |
| −5 dB | 90.0% | 60.0% |
| −4 dB | 100.0% | 100.0% |
| −3 dB | 100.0% | 100.0% |

50% crossing ≈ −5.8 dB (short ping) / −5.2 dB (long ping), both in
WSJT-X's 2500 Hz reference-bandwidth convention.

Reproduce: `docs/notes/MSK144_BENCHMARK.md`.
