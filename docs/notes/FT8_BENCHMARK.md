# FT8 sensitivity benchmark — environment setup

How to reproduce the FT8 AWGN/fading SNR sweep (`tests/ft8_sweep.rs`) from
a clean checkout. Mirrors [`FT4_BENCHMARK.md`](FT4_BENCHMARK.md) and
[`FST4_BENCHMARK.md`](FST4_BENCHMARK.md) — same `WSJT-X Fortran source →
simN sim binary → WAV corpus → cargo test --ignored` pipeline, this time
for `ft8sim`.

This is a different thing from the existing CI "ft8 characterization"
suite (`ft8_decode_block_snr_sweep` and friends, see `.github/workflows/ci.yml`)
— those sweeps synthesize signals with a homegrown LCG noise generator,
not validated against any WSJT-X-native ground truth. `ft8sim` is
WSJT-X's own Fortran simulator (same family as `fst4sim`/`ft4sim`),
so a corpus generated from it is a true Watterson-fading reference,
not a self-consistency check.

## 1. Prerequisites / 2. Build `ft8sim`

Same packages and pattern as [`FT4_BENCHMARK.md`](FT4_BENCHMARK.md)
sections 1-2 — `ft8sim` reuses the identical shared lib subtree (FT8 and
FT4 share LDPC(174,91), CRC-14, `sgran` RNG seeding):

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
scripts/build_ft8sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X, out-dir = target/ft8sim/
```

Sanity-check:

```sh
target/ft8sim/ft8sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav
```

(Same 7-arg CLI as `ft4sim`: `message f0 DT fdop del nfiles snr` — no
T/R-period argument, FT8 has no sub-modes either.)

## 3. Generate the WAV corpus

```sh
scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
# defaults: ft8sim-path = target/ft8sim/ft8sim
#           out-dir     = embedded-poc/assets/ft8_sweep/
```

Same 4 channels × grid × `TRIALS` structure. Default `SNRS` grid (`-5`
down to `-26` dB) straddles FT8's published WSJT-X AWGN threshold of
about **-20 to -21 dB** (2500 Hz ref BW).

## 4. Run the sweep

```sh
MFSK_FT8_SWEEP_DIR=embedded-poc/assets/ft8_sweep \
  cargo test --test ft8_sweep --release \
  --features ft8,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

### Measured 2026-07-18 (this corpus/seed)

50% crossing, linear-interpolated:

| Channel | ~50% crossing |
|---|---:|
| AWGN | ≈ -20.4 dB |
| CCIR good | ≈ -20.0 dB |
| CCIR moderate | ≈ -18.3 dB |
| CCIR poor | ≈ -18.2 dB |

AWGN and CCIR-good land within ~1 dB of the -20/-21 dB published figure
— a much tighter match than FT4 showed against its -17.5 dB reference
(~2 dB gap, see `FT4_BENCHMARK.md`). This is consistent with FT8's
production `decode_frame` running through `decode_block`'s WSJT-X-
faithful pipeline (post-#48 consolidation) rather than the generic
`engine::pipeline` path FT4/FST4 use. CCIR moderate/poor show a larger
~2-2.5 dB gap — not yet investigated; a candidate follow-up for the
diagnose-before-fixing approach in `FST4_BENCHMARK.md` section 6 if
fading-specific sensitivity becomes a priority.

## 5. No `DecodeStrictness` probe here

Unlike the FT4/FST4 sweeps, this one doesn't include a strictness
calibration probe. FT8's production path calls `process_candidate`
(not `pub`) with its own already-calibrated `ft8::decode::DecodeStrictness`
(see the "Calibrated from real WAV bench 2026-04-07" doc comment in
`src/ft8/decode.rs`) — a different, already-tuned struct from the
uncalibrated copy in `engine::pipeline` that FT4/FST4 share (issue #72).
There's no public hook to vary it from an external test. What this
sweep gives FT8 for the first time is a systematic Watterson-fading
corpus to validate that existing calibration against, rather than a new
calibration target.

## The experiment log

The dated sections (§6 onward: what was tried, measured, ruled out) are in [`docs/historical/FT8_BENCHMARK_LOG.md`](../historical/FT8_BENCHMARK_LOG.md), with their numbers unchanged. Current results: [`BENCHMARKS.md`](BENCHMARKS.md), [`UPSTREAM_EVALUATION.md`](UPSTREAM_EVALUATION.md), `sweep-baseline.json`.

- 6. The old CI "ft8 characterization" suite — removed 2026-07-18
- 7. CCIR moderate/poor fading gap — diagnosed and closed (issue #72 follow-up, 2026-07-18)
- 8. SIC LPF window bug — JTDX 17/18 → 18/18 closed (issue #180, 2026-07-25)
- 9. AWGN/CCIR sweep re-measured, `DecodeDepth` redesign confirmed to have zero sweep impact (issue #182 follow-up, 2026-07-26)
- 10. CCIR moderate/poor "no separate WSJT-X figure" replaced with real `jt9 -8 -d3` ground truth (2026-07-26)
- 11. Issue #190 root cause: `jt9` CLI's implicit CQ-AP, not a numerical gap (2026-07-26)
- 12. Pre-0.11.0 tier-C re-measurement — the section-9 figures are stale by up to 1.1 dB (2026-09-20)
- 13. Real `jt9` from WSJT-X 3.2.0-rc1 is up to 0.84 dB more sensitive on FT8; FT4 did not move (2026-09-24)
- 14. After the WSJT-X 3.x port: what moved, and what did not close (#439, PR #451, 2026-09-25)
- 15. The a-priori passes against `ft8b.f90`: which LLR variants, and what bounds the hard errors (#456, 2026-09-26)
- 16. The a7 and a8 list decoders against `jt9` 3.2.0-rc1 (#464, 2026-09-26)
