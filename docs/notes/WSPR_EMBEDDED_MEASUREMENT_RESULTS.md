# WSPR on ESP32-S3 — candidate-loop measurement, results

Measured 2026-08-13 against
[`WSPR_EMBEDDED_MEASUREMENT_PLAN.md`](WSPR_EMBEDDED_MEASUREMENT_PLAN.md),
for issue [#260](https://github.com/jl1nie/mfsk-core/issues/260), whose
own framing was that "a real S3 measurement of the WSPR candidate loop
is now the decisive experiment". This is that measurement.

Raw logs: `embedded-poc/m5stack-cores3-app/logs/wspr-bench_cores3_*.log`.

## Headline

1. **It now fits, with real margin instead of a corpus-specific
   coincidence.** One `decode_scan` started at **1 214 s** against
   WSPR's 120 s slot (10.1× over, 1 755× the host) and now finishes at
   **101.6 s** (0.85× the slot) with the shipped `DeadlineDriven`
   default. Eight independent, additive fixes: `opt-level=3` (1.19×),
   `minsync2` (4.1×), ranking pass-2 candidates by refined sync to
   deep-process only the strongest 2 (1.25× further — evidence-bounded,
   not provably lossless like the first two), a ping-pong rewrite of
   `refine_cascade`'s stack usage (112.1 KB peak → 61.5 KB, no wall-
   clock change by itself but the enabler for the next line), genuine
   dual-core dispatch on all three passes (1.17× further, real —
   152.3 s), a slot-deadline-aware time budget on pass 2's
   failing-candidate ladder (152.3 s → 120.3 s), a Fano cycle-budget
   cap on passes 0/1/2 (120.3 s → 106.0 s, −12.2 %, and — the point of
   doing it — closes the structural gap that let pass 0/1 burn an
   *unbounded, uncapped* Fano+OSD ladder on every `minsync2` survivor
   regardless of how busy the band is; see "Pass 0/1 get a Fano
   cycle-budget cap" below), and a re-implemented FFT-based LPF in
   `subtract` (106.0 s → 101.6 s, −4.1 %, now that the esp-dsp bug
   blocking it is fixed; see "The FFT-based LPF, re-implemented"
   below). An AWGN-sweep-backed alternative,
   `RecallPriority`, trades the deadline guarantee for a fixed
   recall-safety margin instead (105.6 s on the current build, not
   slot-bounded). 9/9 golden held at every step, on both budget modes.
2. **The loop is compute-bound, not bandwidth-bound.** Moving half the
   baseband traffic to internal SRAM buys 5 %. So **#260's Phase 3
   (within-stage hypothesis fusion) should not be built** — its entire
   value was amortising memory traffic, and traffic is not the cost.
3. **The cost is not where the plan looked.** `tone_amplitudes` — the
   routine the whole plan is about — is **8.5 %** of decode time (of
   the pre-`minsync2` total). Pass-2 Fano is **51 % of the entire
   scan** and OSD another 20 %, both spent almost entirely on
   candidates that never decode. Pass 2 alone was 76 % of the scan,
   attempting OSD **896 times for 0 successes**, and returning one
   decode.
4. **That failure population was separable — and wsprd already knew
   it.** Every successful decode converges inside 20 % of the Fano
   budget; essentially every failure burns 99 % of it. The candidate-
   list filter that removes most of that failure population before it
   ever reaches Fano turned out to already be part of the reference
   decoder (`wsprd.c:1294`'s `minsync2`) — documented in this crate's
   own source comment and never implemented. Implementing it is what
   produced the 4.1× above, with **0 phantoms** as a side effect (see
   below).
5. **Dual-core is real but smaller than hoped, and pass 2 isn't safe
   yet.** A work-steal port of the FT8 dual-core pattern measured
   1.35-1.47× on passes 0/1, then **crashed pass 2 with a stack
   overflow** — a genuine resource conflict (two ~100-125 KB task
   stacks don't fit the chip's ~236 KB largest contiguous free block),
   not a tuning slip. Reverted, not shipped; see "Dual-core" below for
   the full account and what it would take to make safe.

## Hardware

| | |
|---|---|
| Board | **M5Stack CoreS3** — ESP32-S3 rev v0.2 (LX7), 16 MB flash, 8 MB Quad PSRAM |
| Clocks | **240 MHz CPU, 80 MHz PSRAM** — see "The clock nobody set" below |
| Crate / bin | `embedded-poc/m5stack-cores3-app`, `--bin wspr-bench` |
| Bench | `embedded_shared::apps::wspr_bench` |
| Input | `embedded-poc/assets/wspr_golden_baseband.bin` — the 375 Hz baseband of the WSJT-X golden `150426_0918.wav`, baked on the host |
| `max_candidates` | 100, matching `wspr_wsjtx_sample_recall_vs_golden` and `WSPR_BENCHMARK.md` |
| Host reference | Ryzen 9 9900X, `--release --features full` |

**The plan assumed an M5StickS3** (`embedded-poc/m5stack-s3`, the
compute-bench crate). The board on the bench is a CoreS3, and the two
differ where Phase 2 cares: StickS3 has **Octal** PSRAM, CoreS3
**Quad**. Flashing the `m5stack-s3` build to a CoreS3 boot-loops on
`octal_psram: PSRAM chip is not connected, or wrong PSRAM line mode`,
so the bench got a second bin in the CoreS3 app crate; both call the
same shared body. Quad PSRAM makes the measured PSRAM penalty an
**upper bound** on a StickS3's, which only strengthens conclusion 2.

## Correctness first: the device reproduces the host exactly

Nine decodes, matching the host's `decode_scan` (condition B of
`wspr_diag_pass_ablation`, inner = 1+2, outer = 1) **decode for
decode**, and covering **all 9 goldens**:

```
ND6P DM04 30 | 1444.0 Hz | dt  1.12 s |  -8 dB      NM7J   DM26 30 | 1500.7 Hz | dt -0.80 s |  -1 dB
W3HH EL89 30 | 1584.6 Hz | dt  0.82 s | -10 dB      KI7CI  DM09 37 | 1515.2 Hz | dt  0.52 s | -20 dB
WD4LHT EL89 30 | 1486.8 Hz | dt 0.61 s |  -5 dB      DJ6OL  JO52 37 | 1527.7 Hz | dt -1.91 s | -18 dB
W5BIT EL09 17 | 1458.1 Hz | dt 0.10 s | -14 dB      W3BI   FN20 30 | 1592.1 Hz | dt  0.74 s | -25 dB
                                                    G8VDQ  IO91 37 | 1462.9 Hz | dt  2.23 s | -23 dB
```

Worth stating plainly because it is the load-bearing check on
everything else here, and because the FFT backend needed a fix (below)
to get this far. The numbers below are timings of a decoder that is
producing the right answers, not of a broken one.

## Phase 0 — build feasibility: path A, as the plan expected

```sh
# embedded-poc/m5stack-s3/Cargo.toml, mfsk-core features += "wspr"
cargo check --release --bin rx-wavsim     # clean
```

No source changes. The plan's four `no_std` breaks (`OnceLock`, two
`String`s, two `f32::round`s) are real but only bite a genuine `no_std`
build; ESP-IDF targets are `std` targets. `NFFT1 = 1_474_560` compiles
and is never called, exactly as predicted.

## Phase 1 — what the candidate loop costs

One `decode_scan`, single core, sequential, baseband in PSRAM.
`ta` = `tone_amplitudes`; its ms column is the call count × the 59.1 ms
per call measured directly in arm C.

| pass | coarse | decode | subtract | total | % of scan | s/cand | ta calls | ta /cand | ta ms | ta % of decode | minsync1 | fano ok/try | osd ok/try |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 0 | 13.1 s | 87.6 s | 18.3 s | 118.9 s | 9.8 % | 6.7 | 293 | 22 | 17.3 s | 19.8 % | 8/13 = 61 % | 7/108 | 0/0 |
| 1 | 14.0 s | 156.0 s | 2.7 s | 172.7 s | 14.2 % | 11.1 | 386 | 27 | 22.8 s | 14.6 % | 4/14 = 28 % | 1/192 | 0/0 |
| 2 | 1.8 s | **920.8 s** | — | 922.6 s | **76.0 %** | 65.8 | 994 | 71 | 58.8 s | **6.4 %** | 3/14 = 21 % | 1/897 | **0/896** |
| **total** | | | | **1 214.3 s** | | | 1 673 | | 98.9 s | **8.5 %** | | 9/1 197 | 0/896 |

Against the plan's decision table:

> **> ~110 s** — doesn't fit a 120 s cadence with capture. Either an
> embedded-specific decode ladder is genuinely required, or the target
> is spotter-mode (decode arrives late) rather than real-time.

1 214 s lands there with an order of magnitude to spare. The plan's
**prediction 1 was wrong**: it expected the total to land "well under"
#260's disowned ~140 s extrapolation, on the reasoning that WSPR's
inner loop is a dense sequential MAC over contiguous `f32` — the LX7's
better case. That reasoning was sound about the *inner loop* and
irrelevant to the total, because the inner loop is 8.5 % of it.

### What a fitting configuration would need

10.1× to close, from levers that exist:

| lever | factor | remaining |
|---|---:|---:|
| both cores (candidate loops are independent) | 2.0× | 607 s |
| drop pass 2 | 4.2× | 145 s |
| drop pass 1 as well | 1.5× | 97 s |

Pass 2 costs 76 % of the scan and returns **one** decode — G8VDQ at
−23 dB. The host ablation agrees exactly: `B \ A` on this file is
`["G8VDQ IO91 37"]` and nothing else. Pass 1 returns one more. So an
embedded ladder of "pass 0 only, both cores" fits a 120 s slot at
~60 s and keeps **7 of 9** decodes; adding pass 1 costs 75 s for the
eighth. That is the shape of the "embedded-specific decode ladder" the
plan's last row anticipated, now with the recall price attached.

Note `coarse_baseband` is cheap on the device (13-14 s, and 1.8 s in
pass 2) — the 7× spread is `max_drift`: passes 0/1 search ±4 Hz of
drift (9 values), pass 2 searches 0 (`wsprd.c:1005`), and
`refine_alignment_top_k`'s inner loop is proportional.

## Phase 2 — bandwidth: the loop is compute-bound

The plan's Phase 2 runs the *whole scan* twice, once per memory tier.
**That experiment is not available on this hardware**, for a reason the
plan could not have known: the refine cascade needs ~93 KB of stack
(below), and 93 KB plus two 180 KiB buffers does not fit the ~326 KB of
internal DRAM an application gets on an S3.

So the A/B moved one level down, onto `tone_amplitudes`. That is not a
weaker substitute — `tone_amplitudes` **is** the loop's traffic: every
call streams 162 × 256 × 2 × 4 B = 324 KiB of baseband. It runs in its
own 48 KB task that exits before the scan task is created, 48 KB being
the largest stack that still leaves a 180 KiB contiguous internal block
free (measured: 96 KB stack → 136 KB largest block; 48 KB → 184 KB).

| arm | µs / call |
|---|---:|
| `idat` + `qdat` both in PSRAM | 59 126 |
| `idat` internal SRAM, `qdat` PSRAM | 56 075 |
| both internal | not runnable — only one 180 KiB block fits |

**PSRAM is 5 % slower with half the traffic moved**, so ~10 %
extrapolated. The plan's own reading of that band:

> small (< ~15 %) — compute-bound; loop fusion (#260's proposal) buys
> little; don't build it. Optimise arithmetic instead.

Put the other way: 324 KiB in 59 ms is **5.5 MB/s** of effective
baseband throughput, an order of magnitude below what 80 MHz Quad PSRAM
delivers. The loop is nowhere near the memory system's limit.

This confirms the plan's **prediction 2** and then some — the influence
is at the weak end of the predicted band. The access pattern is
`k = lag + i·256 + j`, strictly monotonic, the burst-friendly case,
exactly as predicted.

**Consequence: do not build #260's Phase 3.**

## Oscillator tables — 30 % of `tone_amplitudes`, and free to remove

`tone_amplitudes` rebuilds eight `[f32; 257]` oscillator tables per
symbol; the source calls that "trivial at NSPS=256", a host judgement.
The bench times a copy that carries the same recurrence inline — the
stored `c0[j]`/`s0[j]` are produced by exactly the recurrence the
mixing loop then consumes in index order, so running scalars perform
the same operations on the same values.

| | µs / call |
|---|---:|
| `tone_amplitudes` as shipped | 59 131 |
| recurrence carried inline, no tables | **42 011** |
| | **71 % of shipped** |

**`bit-identical = true`** on device — all four `IsQs` arrays match
exactly, as the argument says they must. 8 224 B of stores and reloads
per symbol, 1.33 MB per call, collapsing to eight live `f32`.

So the plan's **prediction 3 is confirmed at the routine level**: the
table term is non-trivial (30 %) and hoisting it beats hypothesis
fusion. But see the next section for what it is worth at the *scan*
level — 1.42× on 8.5 % of the time is ~2.5 % overall. It is a good,
free change; it is not the lever that closes a 10× gap.

## Where the time actually goes: the FEC ladder

The counters and arm C together say something neither the plan nor
`WSPR_BENCHMARK.md` anticipated. Across the whole scan,
`tone_amplitudes` runs 1 673 times at 59.1 ms = **98.9 s of 1 164 s of
decode time, 8.5 %**. The remaining **1 065 s** is
`nblock*_bit_metrics`, Fano, and OSD.

Pass 2 is where this concentrates: 920.8 s of decode, of which ~58.8 s
is `tone_amplitudes`, leaving **862 s across 897 Fano attempts and 896
OSD attempts — ~960 ms per attempt**, for **one** decode.

`wspr::osd::osd_decode` (a port of `osdwspr.f90`) runs whenever Fano
fails *and* a callsign table exists, which in pass 2 is every failing
position. It succeeded zero times. The plan asked for this split to be
measured "where it is cheap"; it was not cheap, but it was measured.

### The split, measured

Running pass 2's loop a second time with `confirmed = None` is an exact
controlled experiment, not an estimate: `None` is the only thing that
changes, and it makes the `and_then` short-circuit before `osd_decode`
instead of running it and rejecting the result. Every other operation —
the refine cascade, the DT peak-up loop, `nblock*_bit_metrics`, Fano —
is identical. Sound here only because pass 2's OSD produced no decodes,
so removing it cannot change which candidates succeed. Neither loop
mutates the baseband, so their order does not matter.

| pass 2 decode | s | share |
|---|---:|---:|
| with OSD | 920.7 | 100 % |
| without OSD | 678.6 | 73.7 % |
| **OSD** | **242.2** | **26.3 %** |

*Provenance*: both figures are sums of the per-candidate timings the
bench logs, taken from two runs of the same build — the OSD-on loop
from `wspr-bench_cores3_240mhz_osdsplit_*.log` (its 920.7 s sum agrees
with the bench's own `decode_us` of 920.795 s) and the OSD-off loop
from `wspr-bench_cores3_osdsplit2_*.log`, whose 3 300 s capture window
closed before the summary line. Combining them is sound because the
work is deterministic and demonstrably so: the four OSD-on candidates
present in *both* logs agree to within 2 ms (69 717/69 715,
67 778/67 777, 69 230/69 229, 69 522/69 520), and every counter matched
exactly across the 160 MHz and 240 MHz runs.

Per-candidate the ratio is strikingly uniform — 51.1-51.7 s without OSD
against 67.8-71.3 s with it, for 13 of the 14 candidates, and
10.4 vs 14.1 s for the one early-exiting outlier. This is a decoder
doing the same futile work 14 times.

So the 862 s of non-`tone_amplitudes` time in pass 2 divides:

| term | s | per attempt | % of the 1 214 s scan |
|---|---:|---:|---:|
| OSD | 242.2 | 270 ms × 896 | 19.9 % |
| Fano + bit metrics | 619.8 | 691 ms × 897 | **51.0 %** |
| `tone_amplitudes` | 58.8 | 59 ms × 994 | 4.8 % |

**Fano is the dominant cost of embedded WSPR — half the entire scan**,
spent on 897 attempts that converge once. `ConvFano` is a sequential
decoder: a codeword it cannot decode costs its full node budget, and in
pass 2 essentially every candidate is noise. OSD is the second term at
20 %, and it is pure waste here — 896 attempts, zero successes,
structurally gated to only ever confirm a callsign Fano already found.

Neither is addressed by anything #260 proposes, and neither is
addressed by the oscillator-table hoist. The lever that matters is a
cheaper *reject* path: something that decides "this candidate is noise"
before paying 691 ms of Fano, 68 times per candidate.

## Fano convergence budget — the failure population is separable

Measured on VK3NV's prompt (issue #260, 2026-08-13): if successful
decodes finish early and failures run to exhaustion, a lower budget
rejects most of the workload without needing a classifier.

`FanoDecodeResult` already carries `cycles`, `max_np` and `converged`,
and the budget is already a `FecOpts` knob (`max_cycles_per_bit`,
default 10 000 → 810 000 cycles over WSPR's 81 bits), so this needed
counters, not a decoder change: `fec::conv::fano::instrument` buckets
every attempt into deciles of budget, keeping converged and failed
apart.

| pass | converged | budget used | failed | budget used |
|---|---:|---|---:|---|
| 0 | 7 | all in the first decile, **max < 1 %** | 101 | mean **99 %** |
| 1 | 1 | **7 %** | 191 | mean **99 %** |
| 2 | 1 (G8VDQ) | **~20 %** (third decile) | 896 | mean **99 %** |

The separation is about as clean as it could be:

- **every successful decode converges inside 20 % of the budget** —
  eight of nine inside 10 %, seven inside 1 %;
- **essentially no failure terminates early**; the mean failed attempt
  burns 99 % of its allowance.

G8VDQ, the −23 dB decode that pass 2 exists to find and that costs
920 s to get, lands at ~20 %.

### Reading the pass-2 raw counts

Pass 2's counters show 898 converged and 1 792 failed against 897
decode attempts. That is not a contradiction: `osd_decode` **re-runs
Fano internally** to extract info bits from the hard codeword
(`osd.rs:16` — "zero hard errors → trivial Fano convergence"), and this
run also replays the pass for the OSD split. So the total decomposes
exactly as 897 (OSD-off replay) + 897 (real loop) + 896 (inside OSD) =
2 690, and the 896 first-decile "convergences" are OSD's own round-trip,
not decode attempts. The two third-decile entries are G8VDQ, found once
in each loop. Passes 0 and 1 have neither OSD nor a replay, so their
counts map 1:1 onto the `wspr::instrument` figures (7/108, 1/192) and
need no such unpicking.

### What it implies, and what it does not

A ladder is not even needed in the form proposed. With successes at
≤ 20 % and failures at 99 %, **almost nothing would be promoted**, so a
single lower budget does the same work as a 5 % → 20 % → full ladder
without needing `ConvFano` to be checkpointable. `max_cycles_per_bit`
is already public; testing this costs a parameter, not a rewrite.

Two things this does **not** establish.

**Wall-clock.** Pass 2's 619.8 s covers Fano *and*
`nblock*_bit_metrics`, and only the former scales with the cycle
budget. 619.8 s is therefore an upper bound on what a quarter-budget
could save, not an estimate. Splitting those two is the next cheap
measurement.

**Recall.** This is one recording. That every decode on it fits in
20 % of budget says nothing about a weaker signal on a different one,
and the WSPR corpus — single-signal, DT = 0, f0 = 0, all four tones on
exact bin centres — cannot currently answer that. The corpus gap gates
this exactly as it gates everything else here.

## Where G8VDQ ranks after `minsync2` — near the top, not buried

VK3NV's follow-up: if the eventual weak decode stays near the top of
`minsync2`'s survivors, ranked-deep processing (process the top N by
sync, deepest ladder first) is a simpler lever than anything
budget-adaptive.

Needed the exact post-refine `best_sync` `minsync2` gates on — not the
coarse peak-search score `wspr_diag_minsync2_would_drop` used earlier,
which is a looser proxy for the same thing. Extracted the refine
cascade (`decode_at_baseband_nblocks_gated_drift`'s stages 1-5) into
its own function so a diagnostic could call the *exact* value being
gated, not a hand-reconstructed approximation of it, then replicated
`decode_scan_inner`'s real sequence (both early passes, each
subtracting its own decodes) to reach the actual pass-2 residual:

```
G8VDQ: rank 1 of 14 by refined sync (3 of 14 candidates survive minsync2)
```

Second-highest refined sync of any pass-2 candidate (0.2142, behind
one non-decoding candidate at 0.2294), and comfortably inside the 3
that clear `minsync2`. On this file the weak decode pass 2 exists to
find is not buried in the tail — a plain top-N-by-sync cutoff would
have reached it without needing the Fano-budget classifier either of
us had been assuming was necessary.

One recording, so this is directional rather than conclusive — same
corpus caveat as everywhere else in this document.

## opt-level — codegen mattered too, but on the search code, not the DSP

VK3NV's follow-up on #260 asked whether the `opt-level = 1` every
embedded crate here carries — attributed to an LX7 LLVM f32-select
regression, `embedded-poc/m5stack-s3/Cargo.toml`'s original comment —
is a ceiling on the *algorithm* or a *code-generation* one, since
1.51× on a 1.5× clock bump looks like execution-limited behaviour
rather than something pinned on PSRAM latency.

Ran the identical build at `opt-level = 3` (`m5stack-cores3-app` only;
see "The clock nobody set" for why sibling crates are untouched):

| | O1 | O3 | speedup |
|---|---:|---:|---:|
| `tone_amplitudes`, PSRAM (Phase 2) | 59 126 µs | 55 309 µs | 1.07× |
| table-free inline (arm C) | 42 011 µs | 42 582 µs | **0.99×** |
| pass 0 (successful candidates only, 7) | 7 529 ms | 7 041 ms | 1.07× |
| pass 0 (failed candidates only, 6) | 81 300 ms | 69 659 ms | 1.17× |
| pass 2 | 922 631 ms | 763 876 ms | **1.21×** |
| **one decode_scan** | **1 214 262 ms** | **1 024 550 ms** | **1.19×** |

Both builds: **9/9 golden decodes, decode-for-decode identical.** The
f32-select regression the comment attributed `opt-level = 1` to does
not fire on this path.

The pattern answers the question directly: dense `f32` arithmetic
(`tone_amplitudes`, the successful-candidate subset that is almost
entirely `tone_amplitudes`) gains **~1.07×**, essentially the noise
floor — the inline recurrence variant gained *nothing*, meaning O1 was
already extracting what there was to extract from that code. The
Fano-dominated failing-candidate subset gains **1.17×**, and pass 2
(51 % Fano, 20 % OSD) gains **1.21×**, the largest of any stage — OSD's
smaller functions are exactly the shape codegen quality helps most.

So: **a ceiling on the search code, not the DSP.** 1.19× is real and
worth taking, but it does not change the decision table — 1 024.6 s is
still an order of magnitude over the 120 s slot, and the compute-bound
finding above is unaffected (arm C's 0.99× if anything reinforces it:
the hand-tightened FP loop had nothing left for the compiler to find).

Changed for `m5stack-cores3-app` on this evidence. Not extended to the
sibling crates, whose f32-select-bearing FT8 code this measurement
does not exercise.

## `minsync2`: the reference decoder's own candidate filter, missing

`wsprd.c:1294` drops a candidate from its list — before Fano, before
OSD, before any of the DT-peak-up ladder — if its **refined** sync
(post the same 5-stage cascade this crate's `best_sync` already
computes) doesn't clear 0.12 (passes 0/1) or 0.10 (the final pass).
`decode_scan_inner`'s own source comment already documented these two
thresholds (`wsprd.c:1002-1009`); nothing applied them. Every coarse
candidate paid the full Fano + DT-peak-up ladder regardless of how
weak its refined sync was.

Applied inside `decode_at_baseband_nblocks_gated_drift`, right after
`best_sync` is finalised and before Fano starts, reusing the existing
`refine_drift` flag to pick the threshold — wsprd ties `minsync2` to
the identical `ipass < 2` conditional it ties `maxdrift` to, so no new
parameter was needed.

**Host** (Ryzen 9 9900X), same 9/9 golden recall throughout:

| condition | before | after | speedup |
|---|---:|---:|---:|
| inner=1 only, outer=1 | 496.6 ms | 100.6 ms | **4.9×** |
| `decode_scan` (outer=1) | 709.5 ms | 601.1 ms | 1.18× |
| `decode_scan_subtract` (outer=2) | 2 378.6 ms | 944.5 ms | **2.52×** |

`decode_scan_subtract` benefits most — its double SIC re-decodes the
same non-decoding candidates every round, so `minsync2` compounds.

**Device** (CoreS3, 240 MHz, `opt-level=3`), one `decode_scan`:

| | before | after | speedup |
|---|---:|---:|---:|
| pass 2 alone | 763.9 s | 133.9 s | **5.7×** |
| pass 2 Fano/OSD attempts | 897 | 149 | 6.0× |
| **total** | **1 024.6 s** | **249.2 s** | **4.1×** |

Still 9/9 golden, decode-for-decode identical.

**A real precision gap this exposed, and fixed.** `decode_scan_subtract`
was producing two decodes outside the curated golden list on this file
— `68K/YJ0WKZ 30` and a Type-3 hashed `<#054f9> GR42IR 21` — that no
existing test caught: the recall test used `decode_scan_subtract`, the
phantom test used `decode_scan`, so between the two tests neither
function's recall-and-precision pair was ever checked together. This
is exactly the failure mode `common::golden::assert_golden` was written
to prevent — its own doc comment names WSPR's 2026-08-11 phantom
incident as the motivating case — and WSPR's own tests had never been
migrated to it. Migrated: the two tests collapse into one
`assert_golden` call, 9/9 recall and 0 phantoms asserted together.
`minsync2` removed both extras with no recall cost.

**Also found, unrelated to `minsync2` but load-bearing for interpreting
it**: `decode_scan_subtract`'s doc comment claimed it "mirrors WSJT-X
`wsprd.c`'s `npasses=3` SIC loop". True when written, false since an
earlier change (#275) ported that same three-pass loop *inside*
`decode_scan` itself — citing the identical `wsprd.c` lines. Two nested
layers ended up claiming the same construct; wsprd has one, this ran it
twice. Nothing shipped was affected (`decode_scan_default` and the
`mfsk-ffi` C ABI both already routed to `decode_scan`), but the golden
test had been guarding the *wrapper*, so the shipped decoder's own
recall had never actually been asserted. Fixed alongside `minsync2`;
the wrapper is now `#[deprecated]`.

## The clock nobody set

`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ` defaults to **160** on esp32s3, and
Quad PSRAM to **40 MHz**. No crate in this tree ever overrode either:
all 101 device logs under `embedded-poc/*/logs/` report `cpu freq:
160000000 Hz`. The CoreS3's LX7 is rated for 240 MHz and its PSRAM part
for 80 MHz.

Fixed for `m5stack-cores3-app` in this branch. Effect on this bench,
same binary, same input:

| | 160 MHz / 40 MHz | 240 MHz / 80 MHz | |
|---|---:|---:|---:|
| `tone_amplitudes`, PSRAM | 91 369 µs | 59 126 µs | 1.55× |
| `coarse_baseband`, pass 0 | 20.8 s | 13.1 s | 1.59× |
| one `decode_scan` | 1 828.7 s | 1 214.3 s | 1.51× |

Counters were identical across both runs, as they must be — the work is
deterministic — which is a useful cross-check on the whole harness.

**The sibling crates have the same omission and were deliberately not
touched.** `m5stack-s3`, `m5stack-s3-app` and `m5stack-core2-app` all
carry the published FT8 timings in `docs/notes/BENCHMARKS.md` and
`ROADMAP.md` (S3 qso3 0.707 s post-SlotEnd, Core2 1.434 s, …). Raising
their clocks invalidates every one of those numbers, which is a
re-measurement job of its own rather than a drive-by.

## Stack: the candidate loop needs ~93 KB — up to ~124 KB once OSD runs

`uxTaskGetStackHighWaterMark`, cumulative minimum free since task
start, at three points in one sequential 128 KB-stack run:

| point reached | used | free |
|---|---:|---:|
| after pass 0 (Fano only) | 92.3 KB | 35.7 KB |
| after pass 1 (Fano only) | 102.8 KB | 25.2 KB |
| **after pass 2 (Fano + OSD)** | **123.3 KB** | **4.7 KB** |

The cascade holds several 10 368-byte `IsQs` live at once (`best_isqs`,
the current `eval` result, `isqs_owned` in the DT peak-up loop) on top
of `tone_amplitudes`'s own 8 224 B of tables. That alone explains
passes 0/1's ~93-103 KB. Passes 0 and 1 never call `osd_decode` —
`decode_pass1_candidate` passes no callsign table, so the `.and_then`
gating OSD short-circuits before it — so the **+20.5 KB jump at pass 2
has no structural explanation other than OSD**, confirmed by source
audit below.

Not bench overhead — any production embedded WSPR path pays it. It is a
hard constraint on where the baseband can live: even 93 KB of stack
plus 360 KiB of baseband does not fit internal DRAM, so an embedded
WSPR decoder has a PSRAM-resident baseband whether it wants one or not.

The first attempt died on this: the 32 KB ESP-IDF main task faulted
inside the *first* `log::info!` — `LoadProhibited` in
`multi_heap_internal_lock`, with SP already 42 KB below the stack.

### What OSD's own frame costs, audited

`wspr::osd::osd_decode`'s local arrays sum to **~12.2 KB**, two-thirds
of it one array: `g: [[u8; 162]; 50]` at **8 100 B** — a full
byte-per-bit copy of the GF(2) generator matrix, permuted by
reliability order and then mutated in place by Gaussian elimination.

The nested Fano call at the end of `osd_decode` (`codec.decode_soft`,
recovering info bits from the zero-error codeword) does **not** add
meaningfully to this: `fano_decode` constructs a fresh `FanoScratch`
whose `nodes: Vec<Node>` is heap-allocated, and
`build_branch_metrics_wsprd` / `wsprd_normalised_symbols` return a
`Vec` and a lazy iterator respectively — no large stack arrays on that
path. (An earlier pass at this audit assumed the nested call was a
major contributor; it is not.)

So ~12.2 KB of `osd_decode`'s own frame accounts for roughly 60 % of
the measured +20.5 KB pass-2 jump. The remaining ~8 KB was not
resolved by source inspection alone — compiler stack-slot reuse and
closure-capture layout are not reliably hand-computable — and would
need an instrumented on-device measurement to close.

**A real, partial lever exists**: bit-packing `g` from `[[u8;162];50]`
to word-packed GF(2) rows (`[[u32;6];50]`, 162 bits → 6 words) would
cut that array from 8 100 B to ~1 200 B, recovering ~6.9 KB — real, but
only about a third of the observed gap, and it requires rewriting the
Gaussian-elimination row-XOR to operate on packed words plus
re-verifying bit-exact OSD correctness against the golden. Not done —
audited and left for whoever picks up the dual-core work next.

## Dual-core: 1.35-1.47× on passes 0/1, then a stack overflow on pass 2

Attempted a direct port of the FT8 embedded pipeline's own dual-core
pattern (`dual_core.rs`'s `Stage3WorkSteal`): a shared
`Vec<Option<BasebandCandidate>>` and an `AtomicUsize` both the main
core and a persistent APP_CPU worker task `fetch_add` against, so a
pass's candidates drain dynamically across both cores instead of a
static half-and-half split. Dynamic dispatch matters here for the same
reason it matters for FT8 stage 3: per-candidate cost is sharply
bimodal (a `minsync2`-rejected or Fano-converged candidate finishes in
~1 s; one that reaches the full Fano + DT-peak-up ladder and fails
takes 10+ seconds), and a static split would strand whichever core
drew the slow candidates.

Simpler than FT8's version in one respect — WSPR's `tone_amplitudes`
builds its oscillator tables as call-local stack arrays, so there is no
per-candidate scratch buffer to share between cores (FT8 needs 5 KB
`cs Box` staging in each of `CS_SCRATCH_MAIN`/`_WORKER`) — but harder
in the one that matters: **stack**. FT8's worker task runs at 16 KB;
WSPR's refine cascade needs 93-124 KB (see above), so it could not
reuse FT8's worker or its tuned stack size, and needed an independent
queue + persistent task (`wspr_dual_core.rs`, not committed).

**Measured, on passes 0 and 1 (which don't reach OSD):**

| pass | sequential (O3+`minsync2`) | dual-core | speedup |
|---|---:|---:|---:|
| 0 | 22.0 s | 16.3 s | 1.35× |
| 1 | 43.5 s | 29.6 s | 1.47× |

Below the hoped-for ~2×, and for a specific, honest reason: after
`minsync2`, each pass has only 13-14 dispatchable candidates, and the
"slow" subset among them (the ones that reach the full Fano ladder) is
small — 1-2 per pass. Work-steal's worst case with a single dominant
straggler is close to `max(fast_total / 2, slow_item)`, not
`total / 2` — with so few slow items, one core finishing everything
else while the other is still on the straggler is close to the
observed outcome, not a bug.

**Pass 2 crashed**: `***ERROR*** A stack overflow in task wspr_scan has
been detected`, reproduced across 4 automatic reboots before the
capture window ran out. Root cause, not a sizing slip that a bigger
number would have fixed: `wspr_scan` (the task issuing work) itself
holds a stack for its entire life, and a worker task needs a
*second*, comparably-sized stack concurrently — two ~100-125 KB
allocations do not both fit inside the ~236 KB largest contiguous free
block this build measures (`heap_caps_get_largest_free_block`), no
matter how the two are individually sized. Restricting the worker to
passes 0/1 only (which peak at ~103 KB, no OSD) and tearing it down
before pass 2 starts is the structurally sound fix, but was not
implemented or re-verified before this write-up — the attempt was
reverted rather than shipped in a state that had only been proven to
crash.

**Net assessment**: dual-core is a real, additive lever — 1.4×-ish on
two of three passes, for free once built safely — but it is not close
to enough on its own, and pass 2 (76 % of the scan) needs either the
scoped pass-0/1-only redesign above, a reduction in OSD's own stack
footprint (see the audit above), or both, before it can run at all.

## Pass 2 candidate-ladder truncation: rank by refined sync, deep-process the top 2

Dual-core stalled on pass 2 specifically (stack, above); the
alternative lever is spending less per candidate rather than more
cores. Pass 2 already refines and `minsync2`-filters every coarse
candidate before Fano/OSD — the question is whether every survivor
needs the *full* ladder, or whether ranking by refined sync and
deep-processing only the strongest few is safe.

**Evidence, not a guess.** A 260-trial AWGN sweep across 13 SNR levels
(`wspr_rank_sweep`, `tests/wspr_sweep.rs`) seeded a
[`WsprCallsignTable`] with the known transmitted callsign+grid (opening
OSD's gate the same way a cross-slot "heard this station before" does
in production) and ranked every `minsync2` survivor by refined sync:

```
   SNR   trials       found     rank<=1     rank<=2  via fano/osd/none
  -34dB       20           0           0           0     0/   0/  20
  -32dB       20           8           8           8     1/   7/  12
  -31dB       20          15          15          15     2/  13/   5
  -30dB       20          20          20          20    12/   8/   0
  -29dB       20          20          20          20    18/   2/   0
  -28dB..+0dB (all)  20          20          20          20    20/   0/   0
```

The real signal, whenever it decoded at all, **never ranked worse than
1** — never rank 2 or beyond, in any of 260 trials. This sweep is
single-signal (no SIC residual, no competing stations), so it brackets
pass 2's isolated-weak-candidate case rather than reproducing pass 2
exactly, but it's the broadest evidence available and it never once
needed more than rank 1. `PASS2_DEEP_LADDER_TOP_N = 2` is therefore the
minimum this evidence supports, not a round number.

**The same sweep reverses an earlier over-generalization.** The
WSJT-X golden file alone showed OSD succeeding 0/0 times, which read
as "OSD is dead weight on pass 2." At -32/-31 dB — the marginal-
detection transition zone — OSD instead accounts for the *majority* of
hits (7/8 and 13/15 in the table above). This constant trims *which*
candidates reach the ladder; it does not touch the ladder itself, and
OSD stays in it.

**Not wsprd-faithful**, unlike `minsync2` above — wsprd runs the
ladder on every `minsync2` survivor with no further cut, and this
crate's own top-N cut costs real (if evidence-bounded) recall risk
rather than being provably safe. Gated behind a new feature,
`wspr-pass2-topn`, off by default: host stays on the full-ladder path
(not part of `full`), embedded turns it on via `embedded-shared`'s
`wspr-bench` feature. Same pattern `fixed-point` already established
for letting host simulate embedded's behavior byte-for-byte — add
`wspr-pass2-topn` to an explicit `--features` list to exercise it
without flashing.

**First flash showed nothing — because the bench bypasses
`decode_scan`.** `embedded_shared::apps::wspr_bench` predates this
function and runs its own hand-inlined pass 0/1/2 loop (the "D
pattern" noted in `embedded-poc/CLAUDE.md` — avoids a `tlsf_malloc`
corruption bug when `decode_block`-shaped helpers are called from a
long bench loop), not `decode_scan_inner`'s dispatch. The change was
correctly gated and host-verified, but the first on-device run showed
pass 2 unchanged at 107.5 s / 14 full-ladder candidates — the bench's
manual loop never called the new code at all. Fix: made
`decode_pass2_top_n` `pub` and switched the bench's own pass-2 loop to
call it directly instead of re-deriving the candidate loop a second
time.

**Measured, CoreS3, O3 + `minsync2` already applied:**

| | before (full ladder, 3 survivors) | after (top 2) |
|---|---:|---:|
| pass 2 decode | 107.5 s | **62.25 s** (−42 %) |
| pass 2 fano/osd attempts | 149/148 | 81/80 |
| one `decode_scan` TOTAL | 222.8 s | **177.6 s** (−20 %) |
| golden recall | 9/9 | **9/9**, unchanged |

Stack headroom stayed healthy post-decode (16.3 KB free, no crash).
177.6 s is 1.48× the 120 s slot, down from 2.08× before this change —
real progress toward sub-2-minute, not yet there. Host: 9/9 golden with
and without the feature; the wsprd-faithful default path is unaffected
(866-871 ms, matching the pre-change baseline exactly).

## Dual-core, take two: the `refine_cascade` stack fix unlocked it

The first dual-core attempt (above) crashed on pass 2 and was
reverted. Revisited the same day after the pass-2 top-N work, on the
hypothesis that fewer deep-ladder candidates might leave more stack
headroom — it doesn't, directly: per-candidate peak stack is set by
what the candidate's *own* code path needs, not by how many other
candidates exist. Confirmed by flashing the dual-core dispatcher
(rebuilt with the `wspr-pass2-topn` `pub` split below) unchanged: the
runtime safety guard (see next paragraph) correctly found no room and
fell back to sequential on every pass, zero crashes, zero speedup —
proof the guard works, not proof dual-core was viable yet.

**The runtime guard, added specifically because of the first crash**:
every worker spawn now checks `heap_caps_get_largest_free_block`
against the stack size it's about to request before attempting
`xTaskCreatePinnedToCore`; if the block isn't there, that phase falls
back to sequential instead of attempting an allocation that either
fails outright or (worse) succeeds too small. This is what made it
safe to iterate on stack sizing live on hardware instead of getting it
exactly right on paper first.

**What actually unlocked it**: auditing `refine_cascade` itself,
which the earlier stack audit had flagged but not fixed ("the
remaining ~8 KB was not resolved by source inspection alone").
`refine_cascade`'s 13-eval search called a closure (`eval`) from five
distinct source locations, each returning a fresh `IsQs` (10 368 B) by
value — exactly the shape that defeats naive stack-slot reuse, since
each call site is a separate `let` binding the optimizer has no
obligation to coalesce with the others. Rewrote `tone_amplitudes` into
an `_into` variant that writes through `&mut IsQs`, and
`refine_cascade` to hold exactly two such buffers
(`best_isqs`/`scratch`) ping-ponged via `core::mem::swap` — bit-
identical output (verified against golden before/after), but the
cumulative stack peak through a full pass-0/1/2 run dropped from
**112.1 KB to 61.5 KB**. Far bigger than OSD bit-packing's own ~7 KB —
the audit's "several `IsQs` live at once" language was right, just
underestimating how many "several" was.

That 50 KB is what actually mattered: `SCAN_STACK` (the task's own
fixed reservation) doesn't shrink on its own just because the code
inside it needs less — it was still reserving the old 128 KiB, none
of which returns to the heap for a worker to use. Dropped it to 90 KiB
(≈27 KB margin over the new measured peak), which is what freed real
room in the ~236 KB largest contiguous free block. With that, both
worker sizes (`PASS01_WORKER_STACK` 80 KiB, `PASS2_WORKER_STACK`
88 KiB — both generously margined now that there was margin to give)
cleared the guard on the next flash.

**Confirmed genuinely parallel, not a coincidentally-plausible
sequential sum**: added start/stop timestamps on both sides of pass
2's split. `pass2 main started` and `pass2 worker started` log at the
identical millisecond; the worker's candidate (one that converges)
finishes in 11.1 s while main's candidate (one that exhausts the full
Fano+OSD ladder without converging) takes 48.1 s. Wall-clock for the
pass is `max(48.1, 11.1)`, not their sum — exactly what real 2-core
overlap predicts, and readily distinguishable from a bug that
happened to produce a plausible-looking number.

**Measured, CoreS3, full stack (O3 + `minsync2` + top-N + ping-pong +
dual-core):**

| pass | before (sequential) | after (dual-core) | speedup |
|---|---:|---:|---:|
| 0 (coarse+subtract sequential, decode dual-core) | 53.1 s | 47.4 s | 1.12× |
| 1 (coarse+subtract sequential, decode dual-core) | 60.5 s | 46.7 s | 1.29× |
| 2 (coarse+rank sequential, ladder dual-core) | 64.1 s | 58.3 s | 1.10× |
| **one `decode_scan` TOTAL** | **177.6 s** | **152.3 s** | **1.17×** |

9/9 golden recall unchanged (same 9 callsigns; discovery order differs
run-to-run under concurrent dispatch, which is expected and harmless —
`decode_scan`'s dedup is order-independent).

**Where this leaves sub-2-minute**: 152.3 s is 1.27× the 120 s slot,
down from 2.08× at this investigation's start and 1.48× after the
top-N commit alone. Pass 2's dual-core speedup (1.10×) is the
weakest of the three specifically because its 1-1 static split has no
mechanism to rebalance when one candidate is much slower than the
other (here, 48.1 s vs 11.1 s) — work-steal doesn't help at N=2 either
(there's nothing to steal once both cores have their one item), so
closing that gap further would need either a smarter top-N candidate
selection (rank by *expected* cost, not just refined sync) or
accepting the bottleneck.

That bottleneck is exactly the candidate that never converges — see
the next section.

## Pass 2's ladder gets a time budget — and a 20 s demo lands under the slot

The remaining gap is concentrated in one place: pass 2's 48.1 s
non-converging candidate. wsprd's own DT peak-up × nblocks ladder has
no early-exit for "this clearly isn't going anywhere" — every position
runs before a candidate is given up on. Added an optional per-position
check to `decode_from_refined` (`budget: Option<&(dyn Fn() -> bool +
Sync)>`, `None` everywhere by default — every existing caller is
unaffected) and wired an absolute-deadline version through
`wspr_dual_core::pass2_split` down to each candidate's own worker.

**On-device demo, `PASS2_CANDIDATE_TIME_BUDGET_US = Some(20_000_000)`
(20 s), reverted before commit — not the shipped default:**

| | unbudgeted | 20 s budget |
|---|---:|---:|
| worker (converges) | 11.1 s | 11.1 s, unaffected |
| main (never converges) | 48.1 s | cut off at 12.1 s |
| pass 2 decode | 56.5 s | 20.4 s |
| **`decode_scan` TOTAL** | **152.3 s** | **116.3 s** |
| golden recall | 9/9 | 9/9, unchanged |

116.3 s is **under the 120 s slot** — on this file, with this budget.
That qualifier matters: the reason recall didn't move is that the
candidate the budget cut off was never going to decode regardless of
how long it ran (its refined sync ranks it *above* the real signal
purely on sync strength, not because it *is* a signal — see "Where
G8VDQ ranks after `minsync2`" above for the same distinction). A
20 s cutoff has zero cost on a file where the truncated candidate is
genuinely noise. It is **not** proven safe on a file where the
rank-0-by-sync candidate is a real, weak signal that happens to need
more than 20 s of DT-peak-up search to converge — the AWGN sweep
behind `PASS2_DEEP_LADDER_TOP_N` established that the real signal
never ranks worse than 1 by sync, but says nothing about how long a
*genuine* signal's own ladder search can take in the worst case, only
that this file's specific failing candidate happens to be slow.
`PASS2_CANDIDATE_TIME_BUDGET_US` ships `None` (off) pending that
separate measurement — sizing it correctly is AWGN-sweep work, not
something to infer from one file's single data point, however
suggestive.

## The AWGN sweep, and a deadline-vs-recall choice instead of one number

Ran that separate measurement. `wspr_pass2_ladder_position_sweep`
(`tests/wspr_sweep.rs`) counts how many DT peak-up × nblocks positions
a *converging* signal actually needs, using the same
`rank_pass2_candidates` + `deep_decode_pass2_candidate` pair
production calls, across the AWGN corpus — 20 trials/SNR first pass,
then 80 more each at -30/-31/-32 dB (the sensitivity-floor band where
it matters, 20 → 100 trials there) specifically to check whether the
first pass's max was a small-sample artifact:

```
   SNR   trials   found   max pos   mean pos   p90 pos
 -32dB      100      41        43       22.2        38
 -31dB      100      84        42       15.6        35
 -30dB      100      99        44        7.0        20
 -29dB and above: max 13, dropping to 1 by -28 dB
```

Max moved 43 → 44 with 5× the sample — essentially unchanged — and
p90 dropped clearly below max (35-38 vs the first pass's 42-43,
where p90 ≈ max was itself a symptom of too few "found" trials to
resolve the tail). This is a real, stable worst case, not a small-
sample fluke: **44 of 68 positions**, `docs/notes` cross-referencing
its own earlier caution (`feedback_sparse_snr_sampling_looks_like_bug`
memory) about exactly this failure mode.

44 positions × 0.7071 s/position (S3's measured per-position cost,
from the 68-position full-ladder run above) × 1.3 margin ≈ 40.5 s,
rounded down to **40 s** — verified on-device (`RecallPriority
(40_000_000)`): pass 2 decode 56.5 s → 40.6 s, TOTAL 152.3 s →
**136.5 s**, 9/9 golden unchanged. Materially smaller than the 20 s
demo's gain (116.3 s) — because the 20 s demo was, by this evidence,
actually **unsafe**: 20 s is below the un-margined 43-position figure
(30.4 s) the first sweep pass already implied, before the 44-position
re-check even happened.

**Then the user reframed the goal**: meeting the 120 s deadline is
the first priority; a recall-preserving fixed budget is a second,
explicitly-selectable option, not the default. A fixed budget
anchored to pass 2's own start (what both demos above used) is blind
to how much of the slot pass 0/1 already spent — on a run where they
happen to take longer, it doesn't adapt, and TOTAL can still miss the
deadline. Replaced the single constant with `Pass2Budget`, an enum
`wspr_bench` selects explicitly:

- **`DeadlineDriven` (now the default)** — `deadline_us = scan_start_us
  + SLOT_US` (120 s), captured once at the very top of `run_scan` so
  it reflects the whole scan's elapsed time, not just pass 2's own.
  Cuts pass 2's ladder off exactly when the slot boundary arrives,
  whatever that leaves — guarantees on-time completion; pass-2 recall
  risk varies run-to-run with what pass 0/1 consumed.
- **`RecallPriority(40_000_000)`** — the AWGN-sweep number above,
  fixed regardless of the slot. Choose this when losing a real weak
  decode is worse than running over.

**Both verified back to back on the same golden file:**

| | `DeadlineDriven` | `RecallPriority(40s)` |
|---|---:|---:|
| pass 2 main (non-converging) | cut at 16.1 s | cut at 32.2 s |
| pass 2 decode | 24.4 s | 40.6 s |
| **TOTAL** | **120.3 s** | 136.5 s |
| golden recall | 9/9 | 9/9 |

This run happened to have pass 0/1 consume 94.1 s of the 120 s budget
— more than `RecallPriority`'s 40 s assumes will be left — so
`DeadlineDriven` only had 16.1 s for pass 2's hard candidate, *less*
margin over the 44-position worst case than the fixed number gives.
That is exactly the tradeoff the two names describe, demonstrated on
one real run rather than asserted: `DeadlineDriven` hits the slot
almost exactly (120.3 s, one ladder-position's worth over) but its
recall margin shrinks or grows with pass 0/1; `RecallPriority` holds
its margin fixed but not the deadline. Shipped default is
`DeadlineDriven`, per the explicit priority order the user set —
`RecallPriority` stays in the source as the documented alternative,
one line to switch.

## Waste audit: subtract's reference generation, and an FFT rewrite that didn't survive contact with hardware

Asked, at this point, for a detailed audit of computational waste
across pass 0/1/2 rather than another specific lever. Two findings in
`subtract_signal_baseband`, the per-decode coherent-subtraction step
(called once per accepted decode, so 7-8 times a scan on this file):

1. **Reference-signal generation called `.cos()`/`.sin()` on every
   one of 41 472 samples** (82 944 transcendental calls) instead of
   the per-symbol rotation recurrence `demod::tone_amplitudes_into`
   already uses elsewhere in this crate (one `cos`/`sin` pair per
   symbol, 324 total, then cheap multiply-adds for the other 255
   samples in that symbol).
2. **The 360-tap sin-window LPF was a direct `O(nc2 × NFILT)`
   convolution** (~15 M mult-adds per channel, ~30 M for the complex
   pair, per call) — the exact anti-pattern `engine::dsp::subtract`
   already solved for FT8/FT4 with an FFT-based `O(N log N)` circular
   convolution.

**Finding 1 shipped.** Per-symbol rotation recurrence, renormalised
once per symbol (`(c, s) /= |c, s|`) since a pure-multiply recurrence
run continuously across 41 472 samples would otherwise drift off the
unit circle — bounds the drift to what one 256-sample run can
accumulate rather than letting the whole reference compound it.
Verified against the golden file (9/9, byte-identical decode list)
before and after. **Measured on CoreS3**: pass 0 subtract 17.7 s ->
**10.7 s** (−40 %), pass 1 subtract 2.6 s -> **1.5 s** (−42 %). TOTAL
stayed ~120 s (`DeadlineDriven` targets the slot deadline regardless
of how the budget is spent inside it) — the ~8 s saved bought pass 2's
ladder more time before the cutoff instead (main candidate 16.1 s ->
23.5 s), a better allocation of the same wall-clock, not a lower one.

**Finding 2 was implemented, verified correct on host, and then
reverted after breaking on the actual device — worth recording in
full, since it's a real trap for whoever revisits FFT-based rewrites
on this embedded target.** Ported the same technique
`engine::dsp::subtract::subtract_tones_lpf_fft` uses for FT8/FT4
(cached circular-delay window placement + one forward/inverse FFT
pair), but through `crate::engine::fft::FftPlanner` rather than
`rustfft` directly, since that trait is the portable abstraction
WSPR's own `coarse_baseband.rs` already uses and FT8/FT4's subtract
path is host-only. Packed `(ci, cq)` as one complex signal
(`ci + i·cq`) so both channels shared a single FFT pair — valid by
linearity regardless of whether the window's own FFT is real.

It passed every host differential test against the direct-convolution
reference (bit-exact to float tolerance), including at the realistic
`nc2 = 42 192` size that exercises overlap-save blocking. On the
actual S3 it produced a **silently corrupted residual** — pass 1's
`coarse_baseband` found 0 candidates instead of ~14 — through three
distinct embedded-only bugs, found and fixed one at a time across
several flash cycles:

1. **Off-by-one in the circular-delay kernel placement.** The target
   sum `y[i] = Σ_j window[j]·ci[i−half+j]` is `(ci ⊛ h)[i]` for
   `h[d] = window[half−d]`, not the simpler `window[d+half]` this
   function first shipped with. For an *odd*-length, exactly-centred
   kernel (FT8/FT4's cosine² window, centred on index `lpf_half`) the
   two placements coincide by symmetry — but `NFILT = 360` is even, so
   WSPR's sin window's symmetry point sits at index 179.5, not on an
   integer index, and the two placements diverge by one tap. Caught by
   the host differential test (a ~1 % discrepancy on a slowly-varying
   test signal, exactly consistent with a one-sample shift, not FFT
   rounding noise) — fixed before ever reaching hardware.
2. **The `esp-dsp` backend's hardware FFT ceiling.** `embedded_shared
   ::esp_dsp_fft`'s own doc comment states it plainly:
   `dsps_fft2r_fc32_ae32` "supports any power-of-2 length up to 4096
   in the default build, or 32768 if
   `CONFIG_DSP_TABLE_SIZE_4096_TO_32768` is enabled" — the largest
   Kconfig option `esp-dsp` ships, full stop. The first version used
   `nfft = 65 536` (comfortably ≥ `nc2` for a single-block transform,
   no blocking needed) because that's what the *math* wanted and
   `fft-rustfft` (host) has no such ceiling. A plan request past the
   compiled table size doesn't panic — it silently computes with
   truncated/wrapped twiddle data. Invisible on host by construction
   (different backend entirely); only visible as a corrupted residual
   on the real device. Fixed by capping at 32 768 and adding overlap-
   save blocking (two blocks cover `nc2`), re-verified against the
   direct-convolution reference on host at the now-block-exercising
   size.
3. **A cross-backend FFT normalisation mismatch.** `rustfft` (host,
   `fft-rustfft`) leaves both forward and inverse transforms
   unscaled — the caller divides by `N` once, which is what this
   code did. But `embedded_shared::esp_dsp_fft::EspDspFft::process`'s
   inverse path (`if !self.forward { ... let scale = 1.0/self.len;
   ... }`) already divides by `len` internally, as part of
   implementing "inverse via conjugate-flip" (`esp-dsp` has no native
   inverse-mode kernel). Scaling by `1/nfft` again on top of that —
   correct for `rustfft`, silently wrong for `esp-dsp` — made
   `cfi`/`cfq` land `nfft`× too small, so "subtraction" removed
   essentially nothing. `engine::fft::FftPlanner`'s own trait doc
   comment doesn't specify a normalisation convention either way, so
   nothing about the type signature would have caught this — found
   by reading `EspDspFft::process`'s source directly after ruling out
   the first two causes.

After fixing all three (confirmed via the exact `--cfg` flags rustc
was invoked with, to rule out a stale-build artifact), the residual
was **still** wrong — same symptom, same magnitude, unchanged. That
points at a fourth, undiagnosed embedded-only discrepancy between
`rustfft` and `esp-dsp` behind the same trait. Reverted at that point:
subtract is safety-critical in a way that's easy to underweight —
getting it wrong doesn't crash or slow anything down, it silently
changes *which signals decode*, and continuing to guess against real
hardware on that kind of bug, one flash cycle at a time, was no longer
a responsible use of the remaining time. `docs/notes/` is where this
belongs recorded rather than left as dead code with elaborate
comments in the shipped module — whoever revisits this should start
by instrumenting `EspDspFft` directly (a known-input round-trip dumped
over serial, compared sample-by-sample against the host `rustfft`
result) rather than iterating on `subtract.rs` itself again.

### Follow-up: the fourth bug, root-caused

Asked to actually root-cause the fourth discrepancy rather than leave
it as a documented mystery — `esp_dsp_fft` is a shared backend, not a
WSPR-only concern, and "revert and move on" doesn't serve whoever
reaches for it next.

Found by reading the actual vendored `esp-dsp` source (the pinned
managed component, `espressif/esp-dsp` 1.8.2 — confirmed against
`idf_component.yml`'s `^1.4` resolving to that exact version for this
project's builds), not by more on-device guessing:
`dsps_fft2r_init_fc32` guards its work with a **C file-scope static**,
`dsps_fft2r_initialized`, that is set once and never keyed by the
`table_size` argument:

```c
esp_err_t dsps_fft2r_init_fc32(float *fft_table_buff, int table_size) {
    if (dsps_fft2r_initialized != 0) { return result; }  // <- no-op regardless of table_size
    ...
    dsps_fft_w_table_size = table_size;
    result = dsps_gen_w_r2_fc32(dsps_fft_w_table_fc32, dsps_fft_w_table_size);
    dsps_fft2r_initialized = 1;
}
```

Whichever size is requested **first, from anywhere in the process**,
wins forever. `EspDspPlanner::ensure_table` tracked only its own
`initialised_max` and assumed a bigger request would "grow" the table
— reasonable-sounding, not what the C side does. Worse, `no_std`'s
`with_default_planner` constructs a *fresh* `EspDspPlanner` per call
(see `downsample.rs`), so even that per-instance tracking couldn't
have caught it: a fresh instance remembers nothing about what a
previous instance already asked for.

Sequence on device: `wspr_bench` calls `esp_dsp_fft::prewarm(512)`,
which really does build a 512-entry table. `coarse_baseband`'s own
512-pt plans keep matching it, so nothing looked wrong for the entire
life of this project until the LPF rewrite planned 32768. The Rust
wrapper dutifully bumped `initialised_max` to 32768 and believed the
table had grown; the C side silently no-opped, leaving the *real*
table at 512 entries. The kernel then read **~32 KiB past a
2 KiB allocation** for every LPF FFT/IFFT — not a crash (no MMU
protection on that heap range), just deterministic garbage, which is
exactly why all three of the real, independently-verified fixes above
left the symptom byte-for-byte unchanged: none of them touched this.

**Fix**: track the *actually-initialised* size at module (process)
scope — matching where the real C-side state lives — and force a
genuine regenerate, `dsps_fft2r_deinit_fc32()` then
`dsps_fft2r_init_fc32()`, whenever the requested size differs from it,
not just when it's bigger. Applied to both `EspDspPlanner` (fc32) and,
defensively, `EspDspPlanner16` (sc16, identical one-shot-gate
structure on the C side) — see `FC32_TABLE_LEN` / `ensure_fc32_table`
in `embedded_shared::esp_dsp_fft` for the implementation and full
reasoning, including a documented concurrency caveat (two cores
planning *different* fc32 sizes at once would race the same way the
original bug did — not currently possible on any call path in this
repo, flagged for whoever adds the next concurrent FFT caller).

**Verified on real CoreS3 hardware** with a self-test built into
`wspr-bench` (`fft_multisize_selftest`, runs before the WSPR pipeline
every boot): plans a 512-pt FFT, then a 4096-pt FFT, then 512 again —
the exact interleaving that used to corrupt — checking each against
its known analytic peak bin. First flash with the fix caught a
**second, previously-masked bug**: the self-test's original "big"
size, 32768, made `dsps_fft2r_init_fc32` return
`ESP_ERR_DSP_PARAM_OUTOFRANGE` (`table_size > CONFIG_DSP_MAX_FFT_SIZE`).
This project's `sdkconfig.defaults` sets `CONFIG_DSP_MAX_FFT_SIZE_8192`,
not `_32768` — the module's own doc comment claiming a 32768 ceiling
via `CONFIG_DSP_TABLE_SIZE_4096_TO_32768` was simply wrong (that
symbol doesn't exist in 1.8.2's Kconfig) and had gone unnoticed
because the one-shot-gate bug made `dsps_fft2r_init_fc32` short-circuit
before it ever reached its own bounds check, whenever a smaller size
(512) had already been requested first — so a genuine 32768 request
had never actually been *tried* on this hardware until this fix made
the C call honest again. Corrected the doc comment, re-pointed the
self-test at 4096 (inside the real ceiling), reflashed: self-test
**PASS**, full WSPR pipeline **9/9** golden decodes, TOTAL 120.2 s —
unchanged from the pre-fix baseline, as expected (WSPR's own call
pattern never mixed sizes, so this fix is a no-op for it; only a
*future* multi-size caller, like the reverted LPF rewrite, would ever
exercise the new deinit/reinit path in production).

**On the reverted LPF rewrite specifically**: this fix makes it
*structurally* viable to revisit — the mechanism that silently broke
it is gone — but two things would need to happen first, not just a
re-flash: raise `CONFIG_DSP_MAX_FFT_SIZE_32768` in `sdkconfig.defaults`
(today's 8192 ceiling is below the block size that rewrite used), and
re-verify the other two real fixes (kernel-placement, normalisation)
still hold given the twiddle table is now genuinely regenerated
per-size rather than silently reused. Not attempted here — the
instruction for this follow-up was root-cause the backend, not
re-ship the WSPR feature, and `mfsk-core/src/wspr/subtract.rs` is
staying in its shipped (direct-convolution) state.

**Left open, deliberately not touched**: reading `dsps_fft2r_init_sc16`
(the fc32 sibling used by every FT8 embedded decode via
`fixed-point`'s `MixedRadix3840Sc16Fft`, on by default in every app
crate) turned up a second, structurally *different* and much odder
detail — its `fft_table_buff == NULL` branch ignores its own
`table_size` argument for table *generation* and always builds
`CONFIG_DSP_MAX_FFT_SIZE` (8192) entries, not the requested size (256
for FT8's inner mixed-radix stage). Taken at face value that would
mean every sc16 FFT shorter than 8192 runs against the wrong twiddle
angles — which flatly contradicts this project's own repeatedly-
verified real-hardware FT8 recall under `fixed-point` (7/7, multiple
independent sessions). That contradiction means the reading above is
missing something, not that the recall numbers are wrong. Applied the
same *safe* one-shot-gate defensive fix to `EspDspPlanner16` (identical
structure to the confirmed fc32 bug, and a no-op for every current
sc16 caller — they all request exactly 256, forever), but did **not**
touch the `CONFIG_DSP_MAX_FFT_SIZE`-forcing behavior itself. Whoever
picks this up should start with a device round-trip dump of
`dsps_fft_w_table_sc16`'s actual contents rather than more source-
reading — see `EspDspPlanner16`'s doc comment in `esp_dsp_fft.rs` for
the full citation.

## Pass 0/1 get a Fano cycle-budget cap — and a real bug in `ConvFano` it exposed

Prompted by a direct question: the 120.3 s "fits the slot" result was
measured on one recording with 9 real signals. Pass 0 (47.4 s) + pass
1 (46.7 s) — 78 % of the scan — had *no* triage at all beyond
`minsync2`: every survivor paid the full, uncapped Fano+OSD ladder,
unlike pass 2 (`minsync2` + top-N + a deadline budget). Nothing bounds
that cost as a busier band produces more `minsync2` survivors.

The lever was already measured and sitting unused: "Fano convergence
budget" above found every real decode on the golden converges inside
20 % of the uncapped budget (worst case, G8VDQ: 162 075 cycles = 2 001
cycles/bit, exactly 20.01 % of 10 000/bit, confirmed by a direct
`fec::conv::fano::instrument::OK_CYCLES_MAX` read), while failing
candidates burn a mean 99 %. `FecOpts::max_cycles_per_bit` exists for
exactly this (`Jt9Depth`'s `Fast`/`Normal`/`Deep`/`Max` ladder already
uses it for JT9) and is `None` (uncapped) at every WSPR call site.

**The cap that went in**: `WSPR_FANO_CYCLE_BUDGET = 5_000` cycles/bit
— JT9's own `Fast` tier, reused rather than inventing a number, giving
2.5× margin over the measured worst real case. Applied at the one hot
-path call (`decode_from_refined`'s `decode_soft_pooled`, shared by
all three passes). OSD's own internal Fano re-run (`osd.rs`, trivial
convergence against an already-zero-hard-error codeword) and the
test-only `decode_from_deinterleaved_llrs` were deliberately left on
`FecOpts::default()` — capping an always-fast, always-converging call
buys nothing and only adds risk.

**First attempt did nothing — `ConvFano` was silently ignoring
`FecOpts` entirely.** `decode_soft`/`decode_soft_pooled`'s `opts`
parameter was named `_opts`: both hardcoded
`Self::DEFAULT_MAX_CYCLES`, never reading `max_cycles_per_bit` at all.
JT9's own `ConvFano232::decode_soft` does this correctly
(`opts.max_cycles_per_bit.unwrap_or(Self::DEFAULT_MAX_CYCLES)`) two
codecs down in the same file — `ConvFano` (WSPR's codec) never got the
same wiring. Caught because a before/after AWGN sweep and a before
/after device flash both came back **byte-identical**, which a genuine
cap should not do; traced to the unused parameter by inspection, not
guesswork. Fixed in `mfsk-core/src/fec/conv/mod.rs` for both
`decode_soft` and `decode_soft_pooled`, mirroring the JT9 pattern —
this also fixes the same latent gap for any other future `ConvFano`
caller, not just this one.

**AWGN sweep, before vs. after the (real) cap**, same 100-trial
corpus used earlier in this doc:

| SNR | uncapped | capped (5 000/bit) |
|---:|---:|---:|
| −32 dB | 22/100 (22.0 %) | 19/100 (19.0 %) |
| −31 dB | 70/100 (70.0 %) | 67/100 (67.0 %) |
| −30 dB | 96/100 (96.0 %) | 96/100 (96.0 %) |
| −29 dB and up | 100 % | 100 % (unchanged, all cells) |

A real, small cost, concentrated at the extreme floor. Tried widening
to 7 000/bit: −31 dB partially recovered (69/100), −32 dB did not move
at all (19/100 either way) — that cell's shortfall isn't proportional
to the cap, so it isn't worth trading away most of the speed gain to
chase. 5 000/bit ships. The 50 % crossing (≈ −31.5 dB) does not
visibly move. Golden recall unaffected (9/9, `wspr_golden_recall_and_
precision`) at every cap value tried.

**Device, CoreS3, same golden file, `opt-level=3` + `minsync2` + top-N
+ ping-pong + dual-core + the earlier subtract fix already baked in**
(before/after isolates only this change):

| | before | after | Δ |
|---|---:|---:|---:|
| pass 0 decode | 16 273 ms | 11 203 ms | −31 % |
| pass 1 decode | 29 690 ms | 17 587 ms | −41 % |
| pass 2 decode | 33 012 ms | 35 459 ms | +7 % (`DeadlineDriven` spends the freed time here, same reallocation behaviour as the earlier subtract fix) |
| **TOTAL** | **120 679 ms** | **105 969 ms** | **−12.2 % (−14.7 s)** |

9/9 golden held. Measured `RecallPriority(40 000 000)` too, back to
back on the same flash: **also 105 969 ms, bit-identical to
`DeadlineDriven`** — pass 0/1 finishing faster now hands pass 2 enough
natural slack that neither budget is the binding constraint for this
candidate mix, so the two policies converge here. They remain
conceptually different (one is deadline-anchored, one isn't) and would
still diverge on a mix where pass 2's ladder is the bottleneck.

Net effect on the question that prompted this: pass 0/1 now have a
real ceiling (previously none), and this measurement's TOTAL gained
~14.7 s of margin against the 120 s slot as a side effect — not the
primary goal, but a genuine, disclosed win alongside the structural
one.

### The cap's -3 % *is* cap-proportional — and 10 000/bit was never a ceiling

The section above shipped `WSPR_FANO_CYCLE_BUDGET = 5_000` while
disclosing a cost at the sensitivity floor (-32 dB 22 %→19 %, -31 dB
70 %→67 %) and dismissed it with: *"Tried 7 000/bit: -31 dB partially
recovered, -32 dB did not move at all, so that cell's shortfall isn't
cap-proportional."* That is an observation of non-monotonicity, not an
explanation, and it was **wrong** — it rested on a single alternative
sample that happened to land on a flat stretch of the curve. Sweeping
the cap properly (same 100-trial corpus, host):

| cap (cycles/bit) | -32 dB | -31 dB | -30 dB | golden | host wall |
|---|---:|---:|---:|:--:|---:|
| **5 000 (shipped)** | 19 % | 67 % | 96 % | 9/9 | 1.7 s |
| 6 000 | 19 % | 69 % | 96 % | — | — |
| 7 000 | 19 % | 69 % | 96 % | — | — |
| 8 000 | **21 %** | 69 % | 96 % | — | — |
| 10 000 (`DEFAULT_MAX_CYCLES`) | 22 % | 70 % | 96 % | — | — |
| 20 000 | 24 % | 73 % | 96 % | — | — |
| 50 000 | 27 % | 76 % | 98 % | — | — |
| 100 000 | 31 % | 75 % | 98 % | 9/9 | 28.8 s |
| 200 000 | **32 %** | 73 % | **99 %** | — | — |
| 500 000 | 23 % | 61 % | 91 % | **FAILS** | 54.7 s |

Monotonic from 5 000 to ~200 000. The -32 dB column is simply flat
between 5 000 and 7 000 and starts moving at 8 000 — one more sample
point would have shown it.

**Why capping costs recall at all** is structural, and visible in
`wspr/decode.rs`'s ladder: when Fano fails, the only fallback is
`osd_decode_packed`, and it is gated twice — on `confirmed` being
`Some` (a callsign table built by an *earlier* Fano decode) and then
on `table.accepts(&msg)`. An AWGN sweep trial contains one signal, so
there is no earlier decode, no table, and no rescue path. A capped
Fano failure at the floor is therefore an outright loss, not a
downgrade to a slower decoder.

**The larger finding: the pre-existing default was leaving sensitivity
on the table.** `ConvFano::DEFAULT_MAX_CYCLES = 10_000` was never
measured as a ceiling — it was just the default nobody had swept past.
At 100-200 k the same corpus yields **-32 dB 19 %→32 %, -31 dB
67 %→73-75 %, -30 dB 96 %→99 %**. The whole cap discussion was framed
as "how much sensitivity does 5 000 cost against 10 000", when 10 000
was itself costing ~10 points at the floor.

**And there is a real ceiling, now located.** At 500 000 everything
reverses: -30 dB drops to 91 % and the golden file fails three tests,
emitting four CRC-passing garbage decodes —
`"<#077ff> DRH1U  7"`, `"<#06dcf> UA94MJ 60"`, `"461/Y11DRI 0"`,
`"<#05b7b> ITM6XF 22"`. Given long enough, Fano finds a codeword that
satisfies the CRC but is not the transmitted message; those then
poison both the carried callsign table and the SIC residual, which is
why recall collapses rather than merely plateauing. "More cycles is
always safer" is false, and the failure mode is a precision failure,
not a timeout.

### The recall curve was the wrong instrument. Phantoms end the usable range an order of magnitude earlier.

Everything above measures recall — whether the transmitted message was
found. It cannot see a setting that buys hits by accepting more false
codewords, which is precisely how this knob fails. Every corpus file
holds exactly one transmitted message, so any other decode is a
phantom by construction; `wspr_awgn_snr_sweep` now counts them
alongside recall (a permanent column, not a one-off diagnostic):

| cycles/bit | -32 dB | -31 dB | -30 dB | **phantoms / 500 trials** |
|---:|---:|---:|---:|---:|
| 5 000 | 19 % | 67 % | 96 % | **0** |
| **10 000** | 22 % | 70 % | 96 % | **0** |
| 20 000 | 24 % | 73 % | 96 % | **0** |
| 50 000 | 27 % | 76 % | 98 % | **2** |
| 100 000 | 31 % | 75 % | 98 % | **7** |
| 500 000 | 23 % | 61 % | 91 % | golden fails (4) |

Read on recall alone, "higher is better up to ~200 000". Read with the
phantom column, **the usable range ends between 20 000 and 50 000** —
and in a multi-pass SIC decoder a false decode is not a cosmetic
error: it enters the carried callsign table and gets subtracted from
the residual, which is why recall itself eventually collapses.

**Shipped: host = 10 000, which is `wsprd`'s own number.**
`lib/wsprd/wsprd.c:799` reads
`unsigned int maxcycles=10000; //Decoder timeout limit`, overridable
there by a CLI flag exactly as it is here by a feature. So the host
default is neither slower nor less faithful than the reference
decoder, and it is also what this path effectively ran at before the
cap was wired up (`ConvFano::DEFAULT_MAX_CYCLES` is the same 10 000).
20 000 measured clean as well, but there is no principled reason to
sit between the reference decoder's timeout and the onset of false
decodes.

| | cycles/bit | -32 dB | -31 dB | -30 dB | phantoms | golden |
|---|---:|---:|---:|---:|---:|:--:|
| host (default) | 10 000 | 22 % | 70 % | 96 % | 0/500 | 9/9, 3.5 s |
| embedded (`wspr-fano-cap-fast`) | 5 000 | 19 % | 67 % | 96 % | 0/500 | 9/9, 1.7 s |

Embedded's 5 000 stays: it is what keeps a CoreS3 `decode_scan` inside
the 120 s slot (-12.2 % TOTAL), it is phantom-free, and what it costs
is floor recall — it hears less, it does not invent. Verified both
directions by `compile_error!` probe (the feature reaches `mfsk-core`
in the CoreS3 build, and is off on host), and re-measured on device:
**TOTAL 103 955 ms, coarse 13 312 / 14 291 / 1 806 ms and subtract
8 985 / 1 288 ms all identical to the pre-split build, 9/9 golden.**

The first attempt at this section shipped 50 000 for host, picked off
the recall curve because -31 dB peaks there. It produces two phantoms
in 500 trials and is 4.5× slower than the reference decoder's own
budget. Both facts were available; neither was measured before
choosing. That is the reusable lesson here, and it is why the phantom
column is now part of the sweep rather than something to remember to
check.

## The FFT-based LPF, re-implemented — now that the fourth bug is fixed

Directed to do both, in order: apply the cycle-budget cap above, then
retry the FFT-based LPF rewrite ("Waste audit" above) now that its
blocking fourth bug (`esp-dsp`'s process-global one-shot twiddle
table) is root-caused and fixed. `mfsk-core/src/wspr/subtract.rs`'s
module doc comment carries the full technical account; this section
is the result.

**Design choice**: `LPF_NFFT = 8192`, not the first attempt's 32768.
Raising `CONFIG_DSP_MAX_FFT_SIZE_32768` in `sdkconfig.defaults` would
let `nc2 ≈ 42192` fit in a single block, but that's a shared,
project-wide config knob every other embedded FFT caller
(`coarse_baseband`'s 512-point spectrogram included) also lives under
— changing it needs its own memory/impact investigation, and six
overlap-save blocks at 8192 already work. Kept the first attempt's two
independently-correct fixes verbatim: kernel placement
(`h[d] = window[half-d]`, the even-length-window off-by-one) and the
cross-backend inverse-FFT normalisation `#[cfg]`.

**Host**: a new differential test (`lpf_fft_matches_direct`,
`mfsk-core/src/wspr/subtract.rs`) at the realistic `nc2 ≈ 42192` size
— the first attempt's own test size, which exercises all six blocks,
not a single-block toy — passes at max relative error `< 1e-4`
against the direct-convolution reference on a structured (slow +
fast + pseudo-noise) synthetic signal. Full merge gate, `fixed-point`
variant, clippy, and the WSPR AWGN sweep all green; the sweep's
numbers are identical to the cycle-budget-only baseline cell for cell,
confirming the FFT path doesn't move sensitivity at all on host.

**Device (CoreS3)**: flashed clean, no crash, **9/9 golden held** —
the failure mode that ended the first attempt (silently corrupted
residual, 0 pass-1 candidates) does not reproduce.

| | cap-only | cap + FFT LPF | Δ |
|---|---:|---:|---:|
| pass 0 subtract | 10 660 ms | 10 293 ms | −3.4 % |
| pass 1 subtract | 1 522 ms | 1 475 ms | −3.1 % |
| **TOTAL** (`DeadlineDriven`) | **105 969 ms** | **101 599 ms** | **−4.1 % (−4.4 s)** |

**The subtract speedup itself is much smaller than the flop count
predicted** — `O(nc2 × NFILT)` (~30 M mult-adds) vs `O(nc2 log
LPF_NFFT)` (~7 M mult-add-equivalents across 13 FFT calls per
subtract) suggested close to an order of magnitude, not 3-4 %. The
likely reason: `Vec<Complex32>` buffers aren't 16-byte aligned, and
`esp_dsp_fft.rs`'s own doc comment (`Align16Quad`/`AlignedStaging`)
documents that the accelerated LX7 PIE (`aes3`) kernel this project's
`sdkconfig.defaults` enables needs exactly that alignment — an
unaligned buffer likely falls back to a slower copy-through-staging
path instead of the fast kernel. Not chased further here (out of this
follow-up's scope); flagging for whoever revisits this that an aligned
buffer type (this crate already has one, `AlignedStaging`) is the
next thing to try before concluding FFT genuinely isn't worth it here.

**Where the rest of the −4.4 s TOTAL actually came from, found by
comparing two back-to-back flashes**: measuring `RecallPriority(40s)`
for completeness (same pattern as the cycle-budget section) turned up
pass 1's Fano-attempt count differing between the `DeadlineDriven` and
`RecallPriority` builds of the *identical* FFT-LPF source — 35 vs 52,
despite `PASS2_BUDGET` only touching pass 2's dispatch. Re-flashing
the exact same `DeadlineDriven` binary a second time reproduced 35
attempts bit-for-bit both times, ruling out a race condition —
the two *different* compiled binaries simply don't agree on
floating-point results down to the last bit (ordinary IEEE754
non-associativity interacting with whatever the optimizer chose to
inline/vectorise differently elsewhere in the binary), and that
difference happened to land close enough to `minsync2`'s 0.12
threshold to flip a few marginal candidates' fate. Recall (9/9) was
unaffected either way — this is a pre-existing sensitivity of
threshold-based filtering to float-level noise, not something the FFT
rewrite introduced, but it means the measured TOTAL delta is not
purely "FFT is N% faster at the subtract step" — part of it is this
same noise, in the same direction this run. `RecallPriority(40s)`
itself: 105 569 ms (also −4.1 s vs its own cap-only baseline).

**Net**: safe (no crash, no recall regression, host-verified before
ever touching hardware), a real if modest win, and the alignment gap
above is the concrete next lever if more is wanted from this same
approach. `mfsk-core/src/wspr/subtract.rs` now ships the FFT-based
LPF in production — direct convolution remains as
[`lpf_apply_direct`], the differential test's reference, not on the
call path.

## The alignment lever, measured: −12.7 % on subtract, and the 512-point half is a *regression*

The section above closed by flagging an unaligned `Vec<Complex32>` as
the likely reason the FFT LPF returned 3-4 % where its flop count
suggested an order of magnitude, and named [`AlignedComplexBuf`]-style
staging as "the next thing to try before concluding FFT genuinely
isn't worth it here". That was a hypothesis about the *allocator* —
whether a 64 KB `Vec<Complex32>` actually lands off a 16-byte
boundary is not something host code can answer — so it was measured
before anything was rewritten to chase it.

**The probe**: `esp_dsp_fft.rs` gained a counter pair
(`pie_alignment_report`, two relaxed atomics per transform, `aes3`
only) splitting `EspDspFft::process` calls into in-place vs
staged-round-trip, each with an OR-mask of the lengths involved. Every
length the backend plans is a power of two, so the mask reads back as
an exact set.

```
PIE alignment: in-place 2 calls (len mask 0x200) | staged 1182 (len mask 0x3200)
```

**Confirmed, and worse than assumed**: 1 182 of 1 184 transforms took
the staging path. The allocator returns a 16-byte-aligned block only
by luck — twice, both 512-point. The staged set is exactly
`{512, 4096, 8192}`, and the arithmetic lines up: 104 of them are the
LPF's (13 transforms × 8 subtractions across passes 0/1), 1 077 are
`coarse_baseband`'s 512-point spectrogram (`n_time` ≈ 359 × 3 passes).

`AlignedComplexBuf` (new, `mfsk-core/src/engine/fft.rs` — a
`Vec<Align16Quad>` reinterpreted as `[Complex32]`, with unit tests
asserting the alignment, length and zeroing) replaces the two
`vec![Complex32; 8192]` buffers in `lpf_apply_fft`:

| | staged LPF | aligned LPF | Δ |
|---|---:|---:|---:|
| pass 0 subtract | 10 293 ms | 8 985 ms | **−12.7 %** |
| pass 1 subtract | 1 475 ms | 1 288 ms | **−12.7 %** |
| coarse (all passes) | 29 409 ms | 29 409 ms | 0 |
| **TOTAL** | **105 459 ms** | **103 959 ms** | **−1 500 ms (−1.4 %)** |
| PIE in-place / staged | 2 / 1 182 | 106 / 1 078 | −104 staged |

9/9 golden held. This A/B is unusually clean: every non-subtract stage
came back bit-identical, and the −1 495 ms summed subtract delta
accounts for essentially the whole −1 500 ms TOTAL — none of the
build-to-build float noise the previous section had to reason around.

### The 512-point half looked like a regression. It is a measurement floor.

The same change on `coarse_baseband`'s 512-point buffer drove staging
to near-zero (1 183 in-place, 1 staged) and appeared to cost
**+219 ms**, consistently across all three passes — TOTAL 104 179 ms,
with a revert restoring 13 312 / 14 291 / 1 806 ms and 103 959 ms
*exactly*. Reproducible, so not run-to-run noise. It was still the
wrong conclusion, and four controls were needed to say why. Each
measures pass-0 `coarse` between the same two log markers:

| build | pass 0 coarse | what it changes |
|---|---:|---|
| pristine | 13 370 ms | — |
| code-shape control | 13 370 ms | slice bound outside the loop, `process(buf)`; allocation-neutral |
| **null edit** | **13 430 ms** | `black_box(&spec.n_time)` — semantically nothing |
| `Vec::with_capacity` for `cells` | 13 470 ms | removes ~6 600 reallocations |
| aligned 512-pt buffer | 13 414 ms | the "regression" |

**A provably semantics-free edit reproduces the effect.** The
`black_box` line allocates nothing, touches no buffer, and changes no
value; its only effect is to shift every instruction after it. That
is enough to move this stage ~60 ms. The code-shape control, which
compiles to the same machine code as pristine, reproduces pristine
*bit-for-bit* — so the split is not "edited vs unedited" but "same
instruction layout vs different one".

Each competing explanation was tested and killed on its own:

- **Allocation placement** — a device probe printed the addresses.
  Plain and aligned land in the *same heap* (512-pt: `0x3fceb038` vs
  `0x3fcec050`, both internal DRAM; 8 192-pt: both PSRAM). Asking for
  16-byte alignment does not re-route the allocation.
- **Extra allocation count** — an allocate-but-never-use control
  reproduced the swing (+282 ms) with the FFT still running on the
  unaligned buffer, and a *plain* extra allocation of the same size
  reproduced it equally (+242 ms). So not the aligned path either.
- **`ps` moving** — the 735 KB array `refine_alignment_top_k` reads
  ~10⁸ times was the best remaining suspect, since its rows are
  2 048 B apart and all inherit the base's 32-byte cache-line phase.
  Probed directly: base `0x3c0f49b4` **unchanged** with or without an
  extra 4 KB allocation. Different heaps — the 4 KB blocks are
  internal DRAM, `ps` is PSRAM. Killed.
- **Reallocation churn** in `cells` (~6 600 per scan) — real, and
  worth fixing on its own merits, but `with_capacity` did not recover
  the baseline either. Not the mechanism.

**Root cause: instruction-cache / code-layout sensitivity of
`refine_alignment_top_k`.** Its loop nest (`dfreq` × `k0` × `idrift` ×
162 symbols, four strided PSRAM reads per innermost step) runs from
flash through the S3's instruction cache, and this stage's wall-clock
has a ~60-130 ms (~0.5-1 %) floor set purely by where those
instructions land. **The 512-point alignment change sits inside that
floor.** It is not a regression; it is not measurable either way with
this instrument.

Two consequences worth carrying forward. First, the shipped LPF
result is unaffected — its −1 500 ms is an order of magnitude outside
the floor, and it was measured across two builds whose `coarse`
numbers were *identical* (13 312 / 14 291 / 1 806 both times), so no
layout shift is folded into it. Second, **any future `coarse`-stage
change under ~150 ms cannot be attributed from TOTAL or from the
stage timer alone** — it needs a control that isolates layout, the way
the null edit does here. That is the reusable finding; the
512-point buffer itself is left unaligned, not because alignment
hurts, but because there is no evidence it does anything.

The PIE counter stays in the tree. It is what turned the first half
of this from argument into measurement, and it is how the next caller
gets checked rather than assumed.

**A correction to the previous section's headline number.** Rebuilding
that exact source today reproduces its subtract stages bit-for-bit
(10 293 / 1 475 ms) but totals **105 459 ms, not the 101 599 ms
recorded**. The FFT-LPF's own gain is solid; the TOTAL it was
attributed to carried ~4 s of the build-to-build float sensitivity
that section documents, in the favourable direction. Treat the
per-stage numbers as the reproducible ones and TOTAL as ±4 s. The
current shipped figure is **103 959 ms**.

## A latent bug in the shipped FFT backend, found on the way

Not a WSPR finding, and arguably the most consequential half of the day.

`coarse_baseband` plans a 512-point FFT over a plain
`vec![Complex::new(0.0, 0.0); 512]`. On the LX7 PIE path (`aes3`) that
corrupted the heap: `StoreProhibited` inside `tlsf_free`, with the
block header holding what were recognisably float samples.

The `_aes3_` kernels move float **pairs** with `ee.ldf.64.ip` /
`ee.stf.64.ip`, which require an 8-byte-aligned address. `Complex32` is
`repr(C)` over two `f32`, so its alignment is 4 — a `Vec<Complex32>`
may land on a 4-mod-8 boundary, and when it does the kernel's first
`ee.stf.64.ip` writes four bytes *below* the allocation, into the
preceding TLSF block header.

**FT8 has always been one allocator decision away from this.** Its only
PIE FFT is the 256-point inner stage of `MixedRadix3840Fft`, working on
rows *inside* a 3840-element buffer — a 4-byte underrun there lands in
the same allocation and is invisible. Both call sites now stage through
a 16-byte-aligned buffer when, and only when, the caller's is not
already aligned, so the aligned fast path stays zero-copy.

## Corrections to the plan

Reading the source to write the bench turned up three places where the
plan, and the host cost-split diagnostic it was drawn from, understate
production cost:

1. **`decode_scan` runs three inner passes, not two.**
   `decode_scan_inner` has an `early_pass` loop over 0 and 1
   (`nblocks = [1]`, drift refine on, `maxdrift = 4`) before pass 2.
   Pass 1 is skipped only when pass 0 decoded nothing (`wsprd.c:999`).
   The plan's sequence, and `wspr_diag_candidate_cost_split`, run
   pass 0 + pass 2 only. The bench runs all three and logs each, so the
   plan's subtotal stays derivable: **1 041.5 s** for pass 0 + pass 2.
2. **Pass 2's ladder is `[1, 2, 3, 0]` — four rungs, not three.** The
   fourth is wsprd's `ib == 4` bit-by-bit rung.
3. **The traffic model omits the DT peak-up loop.** The plan counts the
   refine cascade's 9 or 17-19 `tone_amplitudes` evaluations and stops.
   But mode 2 wraps its decode in a peak-up loop of up to 17 positions
   (`iifac = 8`, `idt ≤ 128/8`) *per* `nblocks` rung, each position
   calling `tone_amplitudes` again. Measured: **22, 27 and 71** calls
   per candidate in passes 0, 1 and 2 — pass 2 near the 4 × 17 + 19
   ceiling, as a pass whose candidates almost all fail must be.

On **prediction 4** ("`minsync1` fires on a clear majority of the 13
golden candidates, so the real eval count is nearer 19 than 9"): half
right. Pass 0 is 61 %, a clear majority. Passes 1 and 2 are 28 % and
21 % — the residual after subtraction is mostly noise, and the gate
does its job. The eval count is nearer 19 than 9 regardless, but
because of the peak-up loop, not the gate.

## What this does not measure

Everything the plan said it would not: **sensitivity**. Every number
here is time and memory. The corpus gap the plan documents —
`gen_wspr_sweep_wavs.sh` generating single-signal, DT = 0, f0 = 0 files
that pin all four tones to exact bin centres — is untouched and still
gates any claim that a DDC-fed decoder decodes as well.

Also unmeasured: **dual-core**, deliberately, per the plan ("establish
the honest per-core number before reasoning about splitting it"). The
2× in the ladder table above is an assumption, not a measurement.
