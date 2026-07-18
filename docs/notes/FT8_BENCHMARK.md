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
`core::pipeline` path FT4/FST4 use. CCIR moderate/poor show a larger
~2-2.5 dB gap — not yet investigated; a candidate follow-up for the
diagnose-before-fixing approach in `FST4_BENCHMARK.md` section 6 if
fading-specific sensitivity becomes a priority.

## 5. No `DecodeStrictness` probe here

Unlike the FT4/FST4 sweeps, this one doesn't include a strictness
calibration probe. FT8's production path calls `process_candidate`
(not `pub`) with its own already-calibrated `ft8::decode::DecodeStrictness`
(see the "Calibrated from real WAV bench 2026-04-07" doc comment in
`src/ft8/decode.rs`) — a different, already-tuned struct from the
uncalibrated copy in `core::pipeline` that FT4/FST4 share (issue #72).
There's no public hook to vary it from an external test. What this
sweep gives FT8 for the first time is a systematic Watterson-fading
corpus to validate that existing calibration against, rather than a new
calibration target.

## 6. The old CI "ft8 characterization" suite — removed 2026-07-18

`.github/workflows/ci.yml` used to have a push-only matrix entry named
`ft8 characterization` covering `ft8_coarse_sync_concurrent`,
`ft8_decode_block_coarse_diag`, `ft8_decode_block_depth_sweep`,
`ft8_decode_block_pass1_sweep`, `ft8_decode_block_snr_sweep`, and
`ft8_qso3_coarse_sync_probe` (~10 min on every `ft8/**` push). All six
were audited and removed along with the matrix entry:

- All six were `println!`-only diagnostics with **zero assertions** —
  the tier could never fail regardless of what the numbers showed, so
  it bought no regression protection for its wall-clock cost.
- `ft8_coarse_sync_concurrent` required the `fixed-point` feature,
  which the CI `full` feature set doesn't enable — it was already
  running zero tests before this cleanup, just silently.
- `ft8_decode_block_snr_sweep` was the one actually synthesizing its
  own AWGN with a homegrown LCG generator (no fading model at all) and
  comparing two of our own decoders against each other with no
  external ground truth — the most direct match for "verifying against
  a non-golden simulator."
- The other four used real on-air recordings (`REAL_QSO_WAVS`,
  checked into `embedded-poc/assets/`), so they weren't synthetic, but
  still had no assertions.

Actual FT8 recall regression protection was already covered elsewhere
and needed no replacement: `ft8_qso3_apoff_recall` /
`ft8_qso3_apon_recall` are hard-assertion, always-on (`default` suite,
not `#[ignore]`'d) — these are the tests that would actually catch a
recall regression.

The separate `ft8 recall` matrix tier (`ft8_decode_block_real_qso`,
`ft8_reference_suite_recall`) turned out to have the same problem —
audited and fixed the same day: `ft8_reference_suite_recall` (a
PASS1_LIMIT/max_cand embedded-tuning config sweep, informational only)
was deleted; `ft8_decode_block_real_qso` was converted to a
hard-assertion floor test (embedded ship config vs host `decode_frame`
truth on `qso1`/`qso2`/`qso3_busy` — `qso1`/`qso2` have no WSJT-X
golden and were otherwise completely untested) and is no longer
`#[ignore]`'d, so the matrix tier itself was removed — it now runs in
`default`.

## 7. CCIR moderate/poor fading gap — diagnosed and closed (issue #72 follow-up, 2026-07-18)

Followed section 4's flagged open item using the same diagnose-before-
fixing discipline `FT4_BENCHMARK.md` sections 8-12 used to close FT4's
AWGN gap, including that investigation's two hard lessons: verify a
hypothesis with a real narrow re-sweep before crediting it, and make
sure any diagnostic applies the exact gates the real decode path does
before trusting a "rescue."

**Ruled out first (wrong angle, not a valid comparison):** hypothesised
that `process_candidate`'s unused `EqMode::Local` (every production
call site hardcodes `EqMode::Off`) might help under fading, and used
`decode_sniper_eq` (`target_freq ± 250 Hz`) as a stand-in to vary it
externally, since `decode_frame`/`process_candidate` have no public
`eq_mode` hook. **Corrected after review**: sniper mode is a hardware-
roofing-filter accommodation and `EqMode::Local` is specifically tuned
for BPF edge distortion, not a general channel condition — neither is
a valid stand-in for comparing against WSJT-X or for testing whether
equalization-in-general helps fading. (For the record, the invalid
experiment did show `EqMode::Local` making CCIR recall worse in every
cell tested — consistent with a BPF-tuned equalizer misapplied to a
different distortion pattern, but not treated as evidence either way.)

**Real diagnosis**: built `ft8_diag_weak_trials` (`tests/ft8_sweep.rs`,
mirrors `ft4_diag_weak_trials`) replicating `process_candidate`'s
prefix (`coarse_sync` → `fine_refine_3stage` → `nsync` gate) directly
against the public building blocks it itself calls — `process_candidate`
and `process_one_candidate_inner` aren't `pub` outside `crate::ft8`.
Across CCIR moderate/poor losing trials near the section-4 crossings:
the correct near-golden candidate was found essentially every time,
with `fine_refine_3stage` landing on the true position and `nsync`
clearing its gate comfortably (10-21 of 21) — coarse-sync and fine-
refine are healthy. The bottleneck is downstream, in LLR/BP/OSD.

Went internal (`ft8::decode::tests::ft8_diag_internal_osd_trace`,
`src/ft8/decode.rs` — `process_candidate` is a private `fn`, reachable
from that module's own `#[cfg(test)] mod tests` via `use super::*`)
with temporary `MFSK_FT8_OSD_TRACE`-gated tracing in
`process_one_candidate_inner`/`osd_strategy::try_fallback` (since
removed) to see per-stage `hard_errors`. Found: `OSD_HARDERRORS_MAX =
22` (`decode_block/osd_strategy.rs`) — a deliberate mfsk-core deviation
from WSJT-X's universal 36, tightened to filter 3 candidates on
`qso3_busy.wav` judged CRC-luck phantoms — was discarding *golden*
decodes under CCIR fading. Multiple independent LLR variants converged
on the exact transmitted text (`"CQ JL1NIE PM95"`, known ground truth
from `ft8sim`) at `hard_errors` in the high 20s/low 30s, all rejected
by the 22 ceiling.

**Controlled experiment**: widened `OSD_HARDERRORS_MAX` to WSJT-X's own
36 and re-measured (`ft8_snr_sweep`, real `decode_frame`):

| Channel | -21 | -20 | -19 | -18 | -17 |
|---|---:|---:|---:|---:|---:|
| CCIR moderate | 0%→0% | 10%→10% | 5%→25% | 55%→85% | 65%→85% |
| CCIR poor | 0%→5% | 10%→10% | 15%→35% | 45%→65% | 65%→90% |

No regressions anywhere in the swept range. Cross-checked against the
hard-assertion real-recording tests (`ft8_qso3_apoff_recall`,
`ft8_decode_block_real_qso`): **byte-identical** — same 7/8 golden +
7 phantom = 14 total, same 4/4, 5/5, 12/14+2. The widening didn't touch
those specific candidates at all.

It touched a different set, and the result reframes the section's own
history: `ft8_qso3_jtdx_recall.rs` (JTDX's 18-entry aggressive list)
went from 13/18 to **17/18** — recovering exactly the 3 candidates
(`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25, `CQ EA2BFM IN83` e=31)
that `OSD_HARDERRORS_MAX = 22` was introduced in 0.6.3 specifically to
exclude as assumed phantoms (recall had been 16/18 before that). Our
decoder now arrives at those 3 candidates' *exact* JTDX-claimed text
independently — two separate decoders converging on the same CRC-14-
protected message is strong evidence against coincidence (~1/16384²
if genuinely random), reclassifying them as real rather than phantom.
This also materially informs issue #150 (JTDX-18 ground-truth
question): 3 of the 4-5 previously-unverified entries now have
independent corroboration. `ft8_qso3_apon_recall.rs`'s multipass extras
floor similarly went from 4/6 back to its pre-0.6.3 5/6 (same
`CQ EA2BFM` recovery, different test).

**Adopted**: `OSD_HARDERRORS_MAX` set permanently to WSJT-X's 36 (was
22), doc comments in `osd_strategy.rs` rewritten to record this history
rather than the retracted phantom rationale, `ft8_qso3_jtdx_recall.rs`'s
`MIN_JTDX_HITS` raised 13→17, `ft8_qso3_apon_recall.rs`'s
`JTDX_EXTRAS_HARD_FLOOR_MULTIPASS` raised 4→5. Full non-ignored suite
and `-D clippy::perf` green throughout.

**Net effect on the section-4 crossings**: CCIR moderate ≈-18.3 dB →
comfortably better than -18 dB (85% recall there now, vs 55% before);
CCIR poor ≈-18.2 dB → similarly improved (65%→90% across -18/-17 dB).
Both channels' gap toward WSJT-X's own reported fading behaviour is
substantially narrowed, from a single root cause shared with (and
independently corroborated by) the real-recording JTDX comparison —
not two separate fixes.
