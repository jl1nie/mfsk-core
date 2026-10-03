# WSPR against `wsprd` v3.2.0-rc1 (2026-10-03)

How the crate's WSPR output was brought to `wsprd`'s on the WSJT-X golden
(`150426_0918.wav`, dial 10.1387 MHz), and how to repeat it.

## Method

`target/upstream/build-v3.2.0-rc1/wsprd` (from `scripts/build_jt9_upstream.sh`)
is relinked from a copy of `lib/wsprd/wsprd.c` with `fprintf(stderr, …)` added at
the points below, so the reference's numbers are read rather than inferred. The
copy lives outside the export; the baseline binary is untouched.

| probe | what it shows |
|---|---|
| after the coarse search | candidate `freq`, `shift`, `drift`, `sync`, `snr` |
| at each Fano call | `ib`, jitter `ii`, `shift`, the 162 soft symbols, `cycles/81` |
| at each accepted decode | pass, candidate index, refined `f1`, `shift1`, `drift1` |
| start of each pass | `idat`/`qdat` (46 080 floats each) |
| around the first `subtract_signal2` | `idat`/`qdat` before and after |

The crate's side prints the same quantities, offset by the 1152-sample pad.

## Findings, in the order they were found

| difference | effect | fix |
|---|---|---|
| noise floor `sorted[123]`, `wsprd.c:1118` is `tmpsort[122]` | SNR | index 122 |
| sync window `sin(π i/512)`, `wsprd.c:1048` is `sin(0.006147931 i)` | < 0.001 dB | the constant |
| recording cut at 114 s *after* a 3 s pad, so the last 3 s were lost | SNR 0.05-0.26 dB; a nominal frame lost 0.6 s | read pad + 114 s |
| pad 3.0 s = 1125 baseband samples, `wsprd`'s time lattice is `128 (k0+1)` | every lag (128, 64, 16, ±8 jitter) 27 samples off | pad = 9 × 4096 samples |
| candidates ordered by sync, `wsprd.c:1186` sorts by SNR | NM7J (the -1 dB signal, first in `wsprd`) decoded third | stable sort by SNR |
| coarse drift bin: `ifd = ifr + (int)offset`; the C is `(int)(ifr + offset)` | `idrift` -1, 0, +1 tied, -1 won: every drift was -1 | truncate the sum |
| a pass's candidates decoded in parallel, subtracted at the end | later candidates saw unsubtracted neighbours: W5BIT's soft symbols off by 2.6 of 256 | refine in parallel, decode and subtract one at a time |
| reference phase by exact rotation | residual 3e-4 off `wsprd`'s, 1.2 % in the next pass's data | `float phi += dphi`, `cos(double)` as in the C |

## Why it takes this much

Fano is sensitive to its input. G8VDQ converges at 8080 cycles/bit and metric
-187 on `wsprd`'s 162 soft symbols; on a vector that differs from them by 0.2 of
256 levels on average it needs 15 266 and ends at metric -184; at 1.3 it needs
13 318. `fano_decode` itself, fed `wsprd`'s symbols, reproduces 8080 and -187
exactly, so the decoder and the metric table were never the problem: the data
reaching them was.

## Result

SNR within 0.02 dB of `wsprd`'s float for every signal (default and `-d`), DT
equal to 3 decimals, frequency within 0.1 Hz, nine of nine decoded at the default
`-C 10000` (the GUI's `-C 500` decodes eight and G8VDQ needs OSD, as upstream).
`tests/wspr_wsjtx_samples.rs` checks SNR against `wsprd`'s printed integers (±1 dB).

AWGN sweep (`wspr_awgn_snr_sweep`, 260 trials): 50 % crossing -31.40 dB
(baseline -31.33), phantoms 1 in 260 (as before, now in the -31 dB cell),
`wsprd` -31.44. Speed on the 9-signal golden: `wsprd` 1.32-1.39 s, this crate
1.31 s; per file at low SNR 20 ms (`wsprd` 80 ms).

## Not matched

- `dt` prints from the refined alignment, as `wsprd` does; `start_sample`
  remains the jittered one the decode succeeded at.
- `wsprd`'s Type-3 `noprint` decodes end the ladder; here `unpack` rejects them
  and the ladder continues.
