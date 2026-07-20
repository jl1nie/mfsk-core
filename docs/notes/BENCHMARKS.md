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
| FT8      | 7/8 (WSJT-X), 17/18 (JTDX) | CCIR fading gap closed (0.7.3) | at parity |
| FT4      | 6/6 | ~0.3 dB (was ~1.8 dB) | at parity |
| FST4     | 1/1 (FST4-60A only) | 0.10-0.60 dB across 5 sub-modes | at parity |
| WSPR     | 8/8 | matches published sensitivity floor | at parity |
| JT9      | 5/5 | no measurable gap | at parity |
| JT65     | none available | **~7-8 dB** at deep SNR | known gap, deprioritized |
| Q65      | 2 real EME recordings | matches WSJT-X almost exactly with AP hint; 2 sub-modes measurably beat WSJT-X's own plain decode | at/above parity |
| MSK144   | 3/3 (incl. exact SNR match) | 25/28 cells exact match vs. a real `jt9` build | at parity |

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
- Reproduce: `docs/notes/FT8_BENCHMARK.md`.

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
- Reproduce: `docs/notes/FT4_BENCHMARK.md`.

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
- **AWGN sensitivity sweep** (`tests/wspr_sweep.rs`, `wsprsim`-driven,
  13 SNR points × 20 trials): 100% recall 0 to −27 dB, 95% at
  −28/−29 dB, 40% at −30 dB, 0% at −31 dB and below — consistent with
  WSJT-X's published WSPR sensitivity floor.

## JT9

- **5/5 WSJT-X golden** (`samples/JT9/130418_1742.wav`) via the full
  WSJT-X-faithful softsym pipeline (`afc9` + `chkss2` + `xx0` mettab +
  `sync9` per-freq collapse).
- **AWGN sweep** (`jt9sim`-driven, 300 files): no measurable gap vs. a
  real WSJT-X `jt9 -9` build — agreement within margin at every tested
  SNR (~100% at −24 dB, ~50% near −26 dB).

## JT65 — known gap, deliberately not closed

- No real-recording golden WAV available: WSJT-X's own v3 reference
  samples need soft-symbol erasure metadata that lives in private
  WSJT-X branches.
- **AWGN sweep** (`tests/jt65_sweep.rs`, `jt65sim`-driven, 15 SNR
  points): this crate's hard-decision `decode_at`/
  `decode_at_with_erasures` hits 50% recall around −14 dB and
  near-zero below −19 dB, while WSJT-X's own no-`kvasd` path (`jt9
  -6`) holds ~100% down to −22 dB on the identical corpus — a real
  **~7-8 dB gap**.
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

## Q65

- Real recordings: WSJT-X's 6 m EME (W7GJ exchanges) and 10 GHz EME
  reference both decode.
- 10 wired sub-modes (Q65a15/a30/a60/b60/c60/d60/e60/d120/e120/a300).
  `q65sim` AWGN sweep matches `q65params.f90`'s analytical threshold
  formula almost exactly across all of them.
- Fast-fading metric (Gaussian/Lorentzian channel models) recovers
  5-8 dB on Doppler-spread channels, required for microwave EME.
- AP-list template matching decodes 6/6 frames at SNR −25 dB where
  plain BP fails 0/6.
- A residual gap that looked like a decode weakness was root-caused to
  comparison methodology, not a bug: WSJT-X's default `jt9` decode
  always has free access to the "CQ ??? ???" AP hypothesis, so every
  real `jt9` decode on CQ traffic implicitly gets AP-list benefit.
  Using `decode_scan_with_ap_for` with a `"CQ"` hint closes the gap
  almost exactly (e.g. Q65-30A −26 dB: 0%→40%, matching WSJT-X's own
  reported 40%).
- Direct cross-check against a real `jt9 -3` build on Q65-120D/120E/
  300A found **no regression, and two sub-modes exceed WSJT-X's own
  plain decode** by 2.5-3.4 dB (Q65-120D/120E) — because
  `decode_scan_for`'s fine-timing retry runs unconditionally, while
  `jt9 -3` without `-c`/`-x` doesn't get it.

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
  SNR. 50% crossing ≈ −5.5 to −6 dB (WSJT-X's 2500 Hz reference-
  bandwidth convention) for both short (~0.4 s) and long (~2.5 s) ping
  profiles.
- MSK40 (the legacy shorthand mode) and the *adaptive* RX-equalizer
  training loop (as opposed to the fixed bandpass filter above, which
  is ported) remain out of scope.
- Reproduce: `docs/notes/MSK144_BENCHMARK.md`.
