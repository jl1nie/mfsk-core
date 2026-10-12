# FT4 sensitivity benchmark — environment setup

How to reproduce the FT4 AWGN/fading SNR sweep (`tests/ft4_sweep.rs`) from
a clean checkout on any machine. This mirrors the FST4 methodology in
[`FST4_BENCHMARK.md`](FST4_BENCHMARK.md) (built for
[#146](https://github.com/jl1nie/mfsk-core/issues/146)) and feeds the
`DecodeStrictness` recalibration tracked in
[#72](https://github.com/jl1nie/mfsk-core/issues/72) — FT4 currently
hardcodes `DecodeStrictness::Normal` with threshold numbers copy-pasted
from the FT8 calibration, never re-tuned for FT4's own SNR/OSD behaviour.

The whole pipeline is: **WSJT-X Fortran source → `ft4sim` binary → WAV
corpus on disk → `cargo test --ignored` reads the WAVs and prints a
recall table.** Nothing here needs network access at test time — only
the one-time build step touches WSJT-X source.

## 1. Prerequisites

| Requirement | Ubuntu/Debian package | Purpose |
|---|---|---|
| `gfortran` | `gfortran` | compiles `ft4sim.f90` + supporting lib |
| `gcc` / `g++` | `build-essential` | compiles `gran.c`, `sgran.c`, `init_random_seed.c`, `crc14.cpp` |
| `libboost-dev` | `libboost-dev` | `crc14.cpp` uses `boost::augmented_crc` |
| FFTW single-precision | `libfftw3-dev` | linked as `-lfftw3f` |
| WSJT-X source tree | — (see below) | provides `lib/ft4/*.f90` |

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
```

**WSJT-X source.** `scripts/build_ft4sim.sh` only needs the `lib/`
subtree (no Qt build, no `cmake` step) — any WSJT-X checkout with the
`ft4/ft4sim.f90` simulator works, including the upstream tree:

```sh
git clone https://github.com/jl1nie/WSJT-X.git ../WSJT-X
```

Per the repo-root `CLAUDE.md` test-fixture-path rule, never hardcode an
absolute path to this checkout — `build_ft4sim.sh` takes it as an
explicit argument (falling back to the `../WSJT-X` sibling-of-repo-root
convention only as a default).

## 2. Build `ft4sim`

```sh
scripts/build_ft4sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X (sibling of this repo), out-dir = target/ft4sim/
```

Produces `target/ft4sim/ft4sim`. FT4 reuses FT8's `encode174_91`
LDPC(174,91) codec (`genft4.f90` calls it directly), so this build adds
only two FT4-specific compile units (`genft4.f90`, `gen_ft4wave.f90`) on
top of the same shared lib subtree `build_fst4sim.sh` already compiles
(`wavhdr`, `packjt77`, `watterson`, `fftw3mod`/`four2a`). It additionally
needs `crc14.cpp` (Boost-based CRC-14, C++) + `sgran.c` /
`init_random_seed.c` (RNG seeding) — `fst4sim` doesn't need these because
FST4 uses CRC-24 and never calls `sgran`.

Sanity-check the build:

```sh
target/ft4sim/ft4sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav   # one 7.5 s AWGN trial at -15 dB
```

(`ft4sim`'s CLI has no T/R-period argument — unlike `fst4sim`, FT4 has
no sub-modes: `message f0 DT fdop del nfiles snr`, 7 positional args.)

## 3. Generate the WAV corpus

```sh
scripts/gen_ft4_sweep_wavs.sh [ft4sim-path] [out-dir]
# defaults: ft4sim-path = target/ft4sim/ft4sim
#           out-dir     = embedded-poc/assets/ft4_sweep/
```

Covers 4 channel conditions (`awgn`, `ccir_good`, `ccir_moderate`,
`ccir_poor`) × the `SNRS` grid × `TRIALS` repeats (default 20). Same
idempotent/incremental behaviour as `gen_fst4_sweep_wavs.sh` — widening
the grid and re-running only tops up missing cells.

The default `SNRS` grid (`-5` down to `-23` dB) straddles FT4's
published WSJT-X AWGN threshold of about **-17.5 dB** (2500 Hz ref BW;
compare FT8's -21 dB — FT4 trades sensitivity for the shorter 7.5 s slot
and less FEC interleaving). Extend the grid deeper if the measured 50%
crossing turns out weaker than expected, per the FST4 #146
grid-censoring lesson.

## 4. Run the sweep

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release \
  --features ft4,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

`uvpacket` is only required because `tests/common/channel.rs` (pulled in
via `mod common`) unconditionally imports `mfsk_core::uvpacket` —
unrelated to FT4 itself.

Output is a plain recall table (decodes / trials per channel × SNR
cell) — no pass/fail assertions, this is a measurement tool.

## The experiment log

The dated sections (§5 onward: what was tried, measured, ruled out) are in [`docs/historical/FT4_BENCHMARK_LOG.md`](../historical/FT4_BENCHMARK_LOG.md), with their numbers unchanged. Current results: [`BENCHMARKS.md`](BENCHMARKS.md), [`UPSTREAM_EVALUATION.md`](UPSTREAM_EVALUATION.md), `sweep-baseline.json`.

- 5. `DecodeStrictness` probe (issue #72)
- 6. Numeric retune (issue #72, 2026-07-18)
- 7. Coherent full-slot Δt search (`ft4_sync_search`, 2026-07-18)
- 8. `nsync>=18` OSD-depth gate — real bug, confirmed not the AWGN gap (issue #72, 2026-07-18)
- 9. `ft4_sync_search` scorer was non-coherent within each Costas block (issue #72, 2026-07-18) — ~1 dB AWGN gain
- 10. BP/OSD decode strength — mostly ruled out, one faithfulness fix, null on AWGN (issue #72, 2026-07-18)
- 11. 3-segment Δt-search retry — implemented, measured, reverted (issue #72, 2026-07-18)
- 12. `osd_score_min` gate on the wrong (stale) score — real fix, ~0.5 dB further AWGN gain (issue #72, 2026-07-18)
- 13. Coarse-candidate stage was the wrong algorithm — `getcandidates4.f90`
- 14. Dead-code follow-up: `sync2d_refine`/`Sync2dConfig` removed (2026-07-20)
- 15. Phase 4 `smax` early-reject — implemented, measured, reverted (2026-07-20)
- 16. `refine_freq`'s search radius was 5x too wide (issue #182 follow-up, 2026-07-26)
- 17. First hardware measurement — M5Stack CoreS3 (2026-08-29)
- 18. Δt-window narrowing — what it costs in recall (2026-08-29)
- 19. Two optimisations against §17's device number (2026-08-29)
- 20. The DDC front end — `downsample_cached` replaced, and it costs nothing (2026-08-30)
- 21. The candidate budget — the search parameter was never WSJT-X's (2026-08-30)
- 22. Which rungs of the ladder earn their cost (2026-08-30)
- 23. Band occupancy — the design point, and where the budget actually sits (2026-08-30)
- 24. Does the host simulation stand in for the board? For FT4, yes (2026-08-30)
- 25. The whole slot on hardware — three projections, all wrong (2026-08-30)
- 26. The combine stage: two wrong theories and a 41 % win (2026-08-30)
- 27. The DDC: the premise was wrong, and the fix was 10 % (2026-08-30)
- 28. FT8 was paying the same tax (2026-08-30)
- 29. The coherent scorer was 78 % off the fast path (2026-08-30)
- 30. Dual-core is worth 1.33x, not 2x (2026-08-30)
- 31. Block-mode FIR: 172 ms, not the 648 predicted (2026-08-30)
- 32. Streaming the coarse stage, and a 100 % that was luck (2026-08-30)
- 33. Does the streamed coarse stage keep up with capture? (2026-08-30)
- 34. A slot budget the candidate loop actually respects (2026-08-30)
- 35. FT8's stage split, and what the waterfall was costing (2026-08-30)
- 36. What `fixed-point` actually buys FT8 (2026-08-30)
- 37. Budget is decodes, not headroom (2026-08-31)
- 38. Inside the LLR/BP tail — it is BP (2026-08-31)
- 40. FT8's LLR/BP in f32 — the projection was 6 %, the slot moved 0.7 % (2026-09-01)
- 41. The streaming spectrogram, measured — u16 is 0.63× f32 (2026-09-01)
- 42. The shared decimation, built and on the board (2026-09-01)
- 43. The budget was anchored to the wrong end of the slot (2026-09-01)
- 44. Dual-core, re-measured — and a pipeline that lost (2026-09-01)
- 45. The stacks were the memory problem, and WiFi exposed it
- 46. The Δt window goes back to WSJT-X's (2026-09-02)
- 47. Inside the Δt search, and a cache that made everything slower (2026-09-02)
- 48. The residual AWGN gap was a missing AP pass (2026-09-13)
- 49. Two changes that could have moved FT4 and did not (2026-09-14)
- 50. The on-device bring-up log, moved here from EMBEDDED.md (2026-09-16)
- 51. Real `jt9 -5` did not change between WSJT-X 2b9d654 and 3.2.0-rc1 (2026-09-24)
