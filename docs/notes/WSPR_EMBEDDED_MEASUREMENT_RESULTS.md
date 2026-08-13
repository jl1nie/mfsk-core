# WSPR on ESP32-S3 — candidate-loop measurement, results

Measured 2026-08-13 against
[`WSPR_EMBEDDED_MEASUREMENT_PLAN.md`](WSPR_EMBEDDED_MEASUREMENT_PLAN.md),
for issue [#260](https://github.com/jl1nie/mfsk-core/issues/260), whose
own framing was that "a real S3 measurement of the WSPR candidate loop
is now the decisive experiment". This is that measurement.

Raw logs: `embedded-poc/m5stack-cores3-app/logs/wspr-bench_cores3_*.log`.

## Headline

1. **It now fits — on the slot deadline, exactly.** One `decode_scan`
   started at **1 214 s** against WSPR's 120 s slot (10.1× over,
   1 755× the host) and now finishes at **120.3 s** (1.002× the slot)
   with the shipped `DeadlineDriven` default — one ladder-position's
   granularity over 120 s, not under, because that default's whole
   point is racing the deadline rather than promising a hard number.
   Six independent, additive fixes: `opt-level=3` (1.19×), `minsync2`
   (4.1×), ranking pass-2 candidates by refined sync to deep-process
   only the strongest 2 (1.25× further — evidence-bounded, not
   provably lossless like the first two), a ping-pong rewrite of
   `refine_cascade`'s stack usage (112.1 KB peak → 61.5 KB, no wall-
   clock change by itself but the enabler for the next line), genuine
   dual-core dispatch on all three passes (1.17× further, real —
   152.3 s), and a slot-deadline-aware time budget on pass 2's
   failing-candidate ladder (152.3 s → 120.3 s). An AWGN-sweep-backed
   alternative, `RecallPriority`, trades the deadline guarantee for a
   fixed recall-safety margin instead (136.5 s, not slot-bounded) —
   see "The AWGN sweep, and a deadline-vs-recall choice instead of one
   number" below. 9/9 golden held at every step, on both budget modes.
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
