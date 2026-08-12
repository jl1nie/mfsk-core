# WSPR on ESP32-S3 — candidate-loop measurement plan

Written 2026-08-13, for issue
[#260](https://github.com/jl1nie/mfsk-core/issues/260) (VK3NV's
streaming-DDC proposal). **A plan, not a result** — nothing here has
been run on hardware yet.

The issue's own closing position is that RAM and the front-end FFT
look tractable and that "a real S3 measurement of the WSPR candidate
loop is now the decisive experiment". Three rounds of desk analysis
later that is still true: the CPU question is the only one nobody has
moved. This file exists so the measurement happens once, deliberately,
with its decision criteria written down first — the discipline
[#282](https://github.com/jl1nie/mfsk-core/issues/282) arrived at the
hard way, where three of five source-diff proposals were withdrawn on
contact with a real reference decoder.

## The enabling fact: no refactor is needed to measure

This is the part that makes the measurement cheap, and it wasn't
obvious from the issue thread.

`decode_scan`'s stages are already `pub` building blocks —
`wspr::baseband::decimate_to_baseband`,
`wspr::coarse_baseband::coarse_baseband`,
`wspr::decode::decode_at_baseband_nblocks`,
`wspr::subtract::subtract_signal_baseband` — and every `wspr`
submodule is `pub mod`. `tests/wspr_wsjtx_samples.rs`'s
`wspr_diag_candidate_cost_split` already drives exactly this sequence
on the host to produce `WSPR_BENCHMARK.md`'s cost split.

**The seam #260 proposes adding is, for measurement purposes, already
there.** The device-side bench is that same diagnostic minus
`decimate_to_baseband`, fed a baseband produced on the host. The
streaming DDC does not need to exist, and no public API has to change,
before the decisive number can be obtained.

## Build feasibility — verified, with one correction

Checked by running, 2026-08-13:

```sh
cargo check -p mfsk-core --no-default-features --features "alloc,wspr,fft-extern"
```

**Fails with 5 errors**, despite `Cargo.toml` filing `wspr` under
"Lightweight modes — embedded-port-clean". That comment is currently
false and should be fixed or qualified whichever way this goes:

| location | error | in-tree precedent for the fix |
|---|---|---|
| `wspr/osd.rs:38` | `std::sync::OnceLock` | `ft8/refine_fine.rs:68-76` — `#[cfg(feature = "std")]` `OnceLock` + rebuild-per-call fallback |
| `wspr/decode.rs:113` (×2) | `String` not in scope | `use alloc::string::String;` (cf. `msg/decoded.rs:56`) |
| `wspr/decode.rs:825` | `f32::round` | `use num_traits::Float;` (already a dep, `libm` feature) |
| `wspr/baseband.rs:63` | `f32::round` | same |

**However, this probably does not block the bench.** The ESP-IDF
targets are `std` targets: `embedded-poc/m5stack-s3/Cargo.toml:55`
already passes `features = ["alloc", "std", "ft8", "fft-extern", ...]`
to `mfsk-core`. Adding `"wspr"` to that list should compile as-is,
`OnceLock` included.

So there are two paths, and **Phase 0 is to find out which one applies
by trying the one-word change first**:

- **A (expected)**: add `"wspr"` to the existing feature list, it
  builds, proceed. No source changes at all.
- **B (fallback)**: if the ESP-IDF `std` shim is missing something,
  apply the four fixes above (~30 min, all mechanical, all with
  precedent) and re-try. Worth doing eventually regardless, since a
  genuine `no_std` WSPR is a precondition for any FFI/embedded shipping
  path.

`baseband.rs`'s `NFFT1 = 1_474_560` planner call is the one thing the
ESP-DSP backend cannot serve — but it only executes if
`decimate_to_baseband` is *called*, and the bench never calls it. It
must compile, not run. `coarse_baseband`'s FFT is 512-point, which the
ESP-DSP backend handles.

## Phase 1 — baseline: what the candidate loop actually costs

### Host side (one-off, produces a baked asset)

Add an `#[ignore]`d test alongside `wspr_diag_candidate_cost_split`
that runs `decimate_to_baseband` on the WSJT-X golden
(`150426_0918.wav`) and writes the result as a flat `f32` binary:

```
embedded-poc/assets/wspr_golden_baseband.bin
  = idat[46080] f32 LE, then qdat[46080] f32 LE
  = 368 640 bytes (360 KiB)
```

360 KiB of flash is unremarkable next to the ~1.3 MB binaries the
Core2 app already writes. Vendor it — the golden WAV it derives from is
already in-tree, so this is a derived artifact, not new corpus.

### Device side

New bin in `embedded-poc/m5stack-s3/src/bin/` (the compute-bench
crate, which exists for exactly this and has no UI/QSO-FSM baggage),
following `rx_wavsim.rs`'s `include_bytes!` pattern. Single-core,
sequential, no rayon — establish the honest per-core number before
reasoning about splitting it.

Replicate one `decode_scan` call, i.e. one *outer* iteration, with
`inner = 1+2`:

```
coarse_baseband (pass 1)
per-candidate: decode_at_baseband_nblocks(nblocks=[1])
subtract_signal_baseband × (decodes)
coarse_baseband (pass 2)
per-candidate: decode_at_baseband_nblocks(nblocks=[1,2,3])
```

`outer = 1` deliberately: `WSPR_BENCHMARK.md`'s
`wspr_diag_pass_ablation` found the outer pass contributes zero
marginal recall on this golden at ~2.5× the cost, and #260 proposes
making it optional on embedded. Measuring `outer = 2` first would
spend device time on the half we already expect to drop.

### What to record

Per stage, and **per candidate** — the per-candidate spread is the
number that matters, since `WSPR_BENCHMARK.md`'s totals hide a 4×
pass-1/pass-2 asymmetry:

| metric | why |
|---|---|
| wall-clock per stage | the headline |
| wall-clock per candidate, pass 1 and pass 2 separately | host ratio is 8.9 ms vs 35.3 ms; does it hold on LX7? |
| **`minsync1` pass rate** | decides whether candidates cost 9 or 17-19 `tone_amplitudes` evals — see below |
| `tone_amplitudes` call count | ground truth for the traffic model |
| Fano vs OSD split inside pass 2 | `WSPR_BENCHMARK.md` Option D was never split; do it here where it's cheap |

The `minsync1` rate is the highest-value single number. The refine
cascade in `decode_at_baseband_nblocks_gated_drift` runs 1 + 4 + 4 = 9
evaluations, then **8 more** (fine lag + fine freq) only when
`best_sync > MINSYNC1 = 0.10`, plus 0-2 for drift refine. Each
evaluation reads 162 × 256 × 2 × 4 B = **324 KiB** of baseband. So
whether a candidate costs ~2.9 MiB or ~6.0 MiB of reads is decided
entirely by that gate, and nobody has measured how often it fires on
real candidates.

## Phase 2 — the decisive bandwidth experiment: SRAM vs PSRAM A/B

Rather than trying to instrument PSRAM bytes directly, run the
identical code twice with `idat`/`qdat` in different memory and take
the delta. This is the same back-to-back A/B discipline the host bench
work settled on, and it answers the question that matters
("is this loop bandwidth-bound?") without needing a traffic model to
be correct.

- **Arm A**: `idat`/`qdat` in PSRAM (`heap_caps_malloc(MALLOC_CAP_SPIRAM)`).
- **Arm B**: `idat`/`qdat` in internal SRAM (`MALLOC_CAP_INTERNAL`).

360 KiB against the S3's 512 KB internal SRAM is tight but plausible
**with WiFi/BT disabled** — the bench crate has no radio. If it doesn't
fit, fall back to a half-length baseband (23040 samples, 180 KiB) run
in both arms; the ratio is still informative even though the absolute
time isn't comparable to Arm A's full run.

Alternatively/additionally, ESP-IDF ships `xtensa_perfmon`, which can
count data-cache misses directly. Worth trying if the A/B is
inconclusive, but the A/B is the cheaper first move and its result is
harder to misread.

**Interpretation:**

| A − B | reading | consequence |
|---|---|---|
| small (< ~15 %) | compute-bound | loop fusion (#260's proposal) buys little; don't build it. Optimise arithmetic instead |
| large (> ~40 %) | bandwidth-bound | fusion is the right lever; Phase 3 is justified |
| in between | mixed | measure the oscillator-table cost (below) before choosing |

### The uncounted term: oscillator tables

`demod.rs`'s `tone_amplitudes` rebuilds eight `[f32; 257]` oscillator
tables **per symbol** — the code says so, and says the cost is
"trivial at NSPS=256", which is a host judgement:

```
8 arrays × 257 f32 × 4 B  ≈  8.2 KB written per symbol
× 162 symbols             ≈  1.33 MB written per evaluation
                          + the same volume read back in the mixing loop
```

That is roughly **8× the volume of the 324 KiB PSRAM read**, at the
faster internal-SRAM tier. On LX7 it may well be comparable in
wall-clock, and it is invisible to any PSRAM-traffic instrumentation.
Phase 2 should time `tone_amplitudes` with the tables hoisted out of
the symbol loop (wsprd's own optimisation — it reuses them when the
drift-corrected `fp` is unchanged) as a third arm.

This matters for Phase 3: naively fusing N hypotheses multiplies the
table cost by N. Carrying the recurrence inline instead — bit-identical,
since the tables are generated by exactly the recurrence the mixing loop
then consumes in order — drops the term entirely and reduces
per-hypothesis live state to 8 f32. **Fusion and dropping the tables
want to be the same patch.**

## Phase 3 (conditional on Phase 2) — within-stage hypothesis fusion

Only if Phase 2 says bandwidth-bound.

The five refine stages are a coordinate descent — each stage's search
points are defined by the previous stage's winner (stage 2 evaluates at
stage 1's `best_lag`, etc.), a literal port of wsprd's mode-0/mode-1
alternation. **Cross-stage fusion is not available.** Within a stage the
hypotheses are independent and fuse cleanly:

| stage | hypotheses | fusable |
|---|---:|---|
| 1 coarse lag | 5 | yes — same freq/drift, shifted read window |
| 2 coarse freq | 4 | yes — identical samples, different oscillators |
| 3 drift | ≤2 | yes |
| 4 fine lag | 4 | yes |
| 5 fine freq | 4 | yes |

So 17-19 traversals collapse to **~5**, not the ~2 #260 hoped for — but
against a larger base, so ~3.8× rather than ~4.5×. The existing inner
loop already fuses the four *tones* against one sample read, so this
generalises machinery that is present rather than introducing a new
pattern.

`subtract_signal_baseband` is the other candidate, and arguably the
better first target: ~0.96 MiB of temporaries (`refi`/`refq` 324 KiB +
`ci`/`cq` 330 KiB + `cfi`/`cfq` 330 KiB) and a 360-tap FIR that reads
each `ci` element ~360 times. Two restructurings preserve the algorithm
exactly — generate `refi`/`refq` inline from the phase recurrence
instead of materialising them, and run the FIR from a 360-sample ring
buffer (~2.8 KB, internal-SRAM resident).

**Hard gate on any of this**: `wspr_wsjtx_sample_recall_vs_golden` 8/8,
and byte-identical `WsprResult`s against the pre-change host build.
Fusion is an arithmetic reordering, so bit-identity is a reasonable bar;
if it can't be met, that's a finding worth stopping on.

## Decision table

What the Phase 1 total means for #260's question 2, per one
`decode_scan` (`inner = 1+2`, `outer = 1`), against WSPR's 120 s slot:

| device total | reading |
|---|---|
| < ~20 s | comfortable. Dual-core split not even needed; the outer pass could stay optional-but-affordable |
| ~20-60 s | works single-core, dual-core gives real headroom. Proceed with the DDC |
| ~60-110 s | needs the dual-core split *and* Phase 3 to be safe. Viable but tight |
| > ~110 s | doesn't fit a 120 s cadence with capture. Either an embedded-specific decode ladder is genuinely required, or the target is spotter-mode (decode arrives late) rather than real-time |

Note the last row is the one #260's own ~140 s extrapolation lands in —
and that extrapolation was disowned by its author as "not a prediction".
Which is precisely why this gets measured rather than argued.

## Predictions, recorded in advance

So they can be wrong on the record, per this repo's habit:

1. Phase 1 total lands well under the 140 s extrapolation — that figure
   scaled a sequential host number by an FT8-derived ratio, and WSPR's
   inner loop is a dense sequential MAC over contiguous f32, which is
   the LX7's better case.
2. Phase 2 shows the loop is **bandwidth-influenced but not
   bandwidth-dominated** — the access is strictly sequential
   (`k = lag + i·256 + j`, monotonic), which is the burst-friendly
   pattern, unlike the FT8 `fine_refine` case that hit ~80 MB/s.
3. The oscillator-table term turns out to be non-trivial, and hoisting
   it is a bigger single win than hypothesis fusion.
4. `minsync1` fires on a clear majority of the 13 golden candidates, so
   the real eval count is nearer 19 than 9.

## What this plan does *not* measure

**Sensitivity.** Every number here is time and memory. Whether a
DDC-fed decoder achieves the same *recall* is a separate question, and
the current corpus cannot answer it:

- `scripts/gen_wspr_sweep_wavs.sh` generates single-signal, DT = 0,
  f0 = 0 files — so it can never exercise SIC (one transmitter), and it
  pins all four tones to exact bin centres (`DF_BASEBAND` = 0.7324 Hz,
  tone spacing = 1.4648 Hz = exactly 2.0 bins), hiding any
  frequency-quantisation loss by construction.
- The only real recording is one 8-transmitter WAV.

That is the mirror image of JT65's situation in
[#169](https://github.com/jl1nie/mfsk-core/issues/169), where the
corpus sat at the *worst* case and hid a 7-8 dB loss uniformly. Before
the streaming DDC's equivalence can be claimed, the corpus needs an f0
axis (including half-bin, ±0.366 Hz), a DT axis, and a multi-signal
case — `wsprsim` takes `f0` and `DT` positionally, so this is the same
one-pass exercise #282 ran with `fst4sim`/`q65sim`.

**These are independent tracks.** The corpus work gates the *DDC*; it
does not gate this measurement, which is timing on existing code.
Either can go first.

## Priority note

This is bench time, and so is
[#163](https://github.com/jl1nie/mfsk-core/issues/163) (live IC-705
verification), which `ROADMAP.md` records as the single blocker for the
whole Phase B-Core sequence. #260 is a design discussion with no
commitment attached. Phase 0 + 1 here is small enough to be worth
doing on its own merits — it converts the thread's one genuine unknown
into a number, and it is the one contribution the hardware makes that
desk analysis cannot — but it should not displace #163.
