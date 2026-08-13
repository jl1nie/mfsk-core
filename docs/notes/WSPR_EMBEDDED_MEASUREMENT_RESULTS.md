# WSPR on ESP32-S3 — candidate-loop measurement, results

Measured 2026-08-13 against
[`WSPR_EMBEDDED_MEASUREMENT_PLAN.md`](WSPR_EMBEDDED_MEASUREMENT_PLAN.md),
for issue [#260](https://github.com/jl1nie/mfsk-core/issues/260), whose
own framing was that "a real S3 measurement of the WSPR candidate loop
is now the decisive experiment". This is that measurement.

Raw logs: `embedded-poc/m5stack-cores3-app/logs/wspr-bench_cores3_*.log`.

## Headline

1. **It does not fit.** One `decode_scan` costs **1 214 s** on the
   device against WSPR's 120 s slot — 10.1× over budget, 1 755× the
   host. Bottom row of the plan's decision table.
2. **The loop is compute-bound, not bandwidth-bound.** Moving half the
   baseband traffic to internal SRAM buys 5 %. So **#260's Phase 3
   (within-stage hypothesis fusion) should not be built** — its entire
   value was amortising memory traffic, and traffic is not the cost.
3. **The cost is not where the plan looked.** `tone_amplitudes` — the
   routine the whole plan is about — is **8.5 %** of decode time.
   Pass-2 Fano is **51 % of the entire scan** and OSD another 20 %,
   both spent almost entirely on candidates that never decode. Pass 2
   alone is 76 % of the scan, attempts OSD **896 times, succeeds 0
   times**, and returns one decode.
4. **That failure population is separable.** Every successful decode
   converges inside 20 % of the Fano budget; essentially every failure
   burns 99 % of it. A lower `max_cycles_per_bit` — an existing knob —
   is the cheapest thing to test next.

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

## Stack: the candidate loop needs ~93 KB

`uxTaskGetStackHighWaterMark` after pass 0, on a 128 KB task: 35 716 B
free — 92.3 KB used. The cascade holds several 10 368-byte `IsQs` live
at once (`best_isqs`, the current `eval` result, `isqs_owned` in the DT
peak-up loop) on top of `tone_amplitudes`'s own 8 224 B of tables, at
`opt-level = 1`.

Not bench overhead — any production embedded WSPR path pays it. It is a
hard constraint on where the baseband can live: 93 KB of stack plus
360 KiB of baseband does not fit internal DRAM, so an embedded WSPR
decoder has a PSRAM-resident baseband whether it wants one or not.

The first attempt died on this: the 32 KB ESP-IDF main task faulted
inside the *first* `log::info!` — `LoadProhibited` in
`multi_heap_internal_lock`, with SP already 42 KB below the stack.

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
