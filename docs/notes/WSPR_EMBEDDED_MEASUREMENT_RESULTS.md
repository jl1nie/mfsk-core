# WSPR on ESP32-S3 — candidate-loop measurement, results

Measured 2026-08-13 against
[`WSPR_EMBEDDED_MEASUREMENT_PLAN.md`](WSPR_EMBEDDED_MEASUREMENT_PLAN.md),
for issue [#260](https://github.com/jl1nie/mfsk-core/issues/260). The
plan's own framing was that "a real S3 measurement of the WSPR
candidate loop is now the decisive experiment". This is that
measurement.

## Hardware, and one caveat that matters

| | |
|---|---|
| Board | **M5Stack CoreS3** — ESP32-S3 rev v0.2, 240 MHz¹, 16 MB flash, 8 MB **Quad** PSRAM |
| Crate / bin | `embedded-poc/m5stack-cores3-app`, `--bin wspr-bench` |
| Bench | `embedded_shared::apps::wspr_bench` |
| Input | `embedded-poc/assets/wspr_golden_baseband.bin` — the 375 Hz baseband of the WSJT-X golden `150426_0918.wav`, baked on the host |
| `max_candidates` | 100, matching `wspr_wsjtx_sample_recall_vs_golden` and `WSPR_BENCHMARK.md` |
| Host reference | Ryzen 9 9900X, `--release --features full` |

¹ The boot log reports `cpu freq: 160000000 Hz`. The CoreS3 sdkconfig
does not raise it to 240 MHz, so **every device number below is at
160 MHz** and would improve by up to 1.5× at 240 MHz. That does not
change any conclusion — the gaps here are one to two orders of
magnitude — but it is the first thing to fix before re-measuring.

**The plan assumed an M5StickS3** (`embedded-poc/m5stack-s3`, the
compute-bench crate). The board on the bench is a CoreS3, and the two
differ where Phase 2 cares: StickS3 has **Octal** PSRAM (~80 MB/s),
CoreS3 has **Quad**. Flashing the `m5stack-s3` build to a CoreS3
boot-loops on `octal_psram: PSRAM chip is not connected, or wrong PSRAM
line mode`, so the bench got a second bin in the CoreS3 app crate. Both
bins call the same shared bench body. Quad PSRAM makes the measured
PSRAM penalty an **upper bound** on what a StickS3 would show — which
only strengthens the Phase 2 conclusion below, since that penalty came
out small.

## Phase 0 — build feasibility: path A, as the plan expected

```sh
# embedded-poc/m5stack-s3/Cargo.toml, mfsk-core features += "wspr"
cargo check --release --bin rx-wavsim     # clean
```

No source changes. The plan's four `no_std` breaks
(`OnceLock`, two `String`s, two `f32::round`s) are real, but they only
bite a genuine `no_std` build; ESP-IDF targets are `std` targets. The
`Cargo.toml` comment filing `wspr` under "embedded-port-clean" is still
wrong for a true `no_std` consumer and still worth fixing, but it did
not block anything here.

`NFFT1 = 1_474_560` compiles and is never called, exactly as predicted.

## Phase 1 — what the candidate loop costs

<!-- PHASE1 -->

## Phase 2 — bandwidth: the loop is compute-bound

The plan's Phase 2 ran the whole scan twice, once with the baseband in
PSRAM and once in internal SRAM. **That experiment is not available on
this hardware**, for a reason the plan could not have known: the refine
cascade needs ~93 KB of stack (see "Stack" below), and 93 KB plus two
180 KiB buffers does not fit the ~326 KB of internal DRAM an
application gets on an S3.

So the A/B moved down one level, onto `tone_amplitudes` itself. That is
not a weaker substitute — `tone_amplitudes` **is** the loop's traffic.
Every call streams 162 × 256 × 2 × 4 B = 324 KiB of baseband, and a
scan is a few thousand calls. The arm runs in its own 48 KB task that
exits before the scan task is created, because 48 KB is the largest
stack that still leaves a 180 KiB contiguous internal block free
(measured: 96 KB stack → 136 KB largest block; 48 KB → 184 KB).

| arm | µs / call |
|---|---:|
| `idat` + `qdat` both in PSRAM | 91 369 |
| `idat` internal SRAM, `qdat` PSRAM | 84 495 |
| both internal | not runnable — only one 180 KiB block fits |

**PSRAM is 8 % slower with half the traffic moved**, so ~16 %
extrapolated to all of it. The plan's own reading of that band:

> small (< ~15 %) — compute-bound; loop fusion (#260's proposal) buys
> little; don't build it. Optimise arithmetic instead.

Put the other way: 324 KiB in 91 ms is **3.6 MB/s** of effective
baseband throughput. Quad PSRAM delivers an order of magnitude more
than that. The loop is nowhere near the memory system's limit; it is
arithmetic-bound.

This confirms the plan's prediction 2 ("bandwidth-influenced but not
bandwidth-dominated") and then some — the influence is at the weak end
of the predicted band. The access pattern is `k = lag + i·256 + j`,
strictly monotonic, which is the burst-friendly case, exactly as
predicted.

**Consequence for #260: do not build the within-stage hypothesis
fusion (Phase 3).** Its entire value proposition was amortising memory
traffic across hypotheses, and the traffic is not what costs.

## The uncounted term: oscillator tables — 30 %, and free to remove

`tone_amplitudes` rebuilds eight `[f32; 257]` oscillator tables per
symbol. The source calls that cost "trivial at NSPS=256", which is a
host judgement, and the plan flagged it as the term invisible to any
PSRAM-traffic instrumentation.

The bench times a table-free copy that carries the same recurrence
inline — the stored `c0[j]`/`s0[j]` are produced by exactly the
recurrence the mixing loop then consumes in index order, so carrying
them as running scalars performs the same operations on the same
values.

| | µs / call |
|---|---:|
| `tone_amplitudes` as shipped | 91 379 |
| recurrence carried inline, no tables | **64 427** |
| | **70 % of shipped** |

**`bit-identical = true`** — the bench compares all four `IsQs` arrays
and they match exactly, as the argument above says they must.

So: **1.42× on the single hottest routine in WSPR decoding, with
byte-identical output.** 8 × 257 × 4 B = 8 224 B of stores and reloads
per symbol, 1.33 MB per call, collapsing to eight live `f32`.

This confirms the plan's prediction 3 — the table term is non-trivial,
and hoisting it is a bigger win than hypothesis fusion. In a
compute-bound regime it is now the *only* one of the two worth having.

## Stack: the candidate loop needs ~93 KB

`uxTaskGetStackHighWaterMark` after pass 0, on a 96 KB task: **2 876 B
free**. The cascade holds several 10 368-byte `IsQs` live at once
(`best_isqs`, the current `eval` result, `isqs_owned` in the DT peak-up
loop) on top of `tone_amplitudes`'s own 8 224 B of tables, at
`opt-level = 1`.

This is not bench overhead — any production embedded WSPR path pays the
same stack. It is worth recording because it is a hard constraint on
where the baseband can live: 93 KB of stack plus 360 KiB of baseband
does not fit internal DRAM, so an embedded WSPR decoder has a
PSRAM-resident baseband whether it wants one or not.

The first three attempts died on this. The 32 KB ESP-IDF main task
faulted inside the *first* `log::info!` — `LoadProhibited` in
`multi_heap_internal_lock`, with SP already 42 KB below the task stack.

## A latent bug in the shipped FFT backend, found on the way

Not a WSPR finding, and the more consequential half of the day.

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
PIE FFT is the 256-point inner stage of `MixedRadix3840Fft`, which
works on rows *inside* a 3840-element buffer — a 4-byte underrun there
lands in the same allocation and is invisible. Both call sites now
stage through a 16-byte-aligned buffer when, and only when, the
caller's is not already aligned, so the aligned fast path stays
zero-copy.

## Corrections to the plan

Reading the source to write the bench turned up two places where the
plan, and the host cost-split diagnostic it was drawn from, understate
production cost:

1. **`decode_scan` runs three inner passes, not two.** `decode_scan_inner`
   has an `early_pass` loop over 0 and 1 (`nblocks = [1]`, drift refine
   on, `maxdrift = 4`) before pass 2. Pass 1 is skipped only when pass 0
   decoded nothing (`wsprd.c:999`). The plan's sequence, and
   `wspr_diag_candidate_cost_split`, both run pass 0 + pass 2 only.
2. **Pass 2's ladder is `[1, 2, 3, 0]` — four rungs, not three.** The
   fourth is wsprd's `ib == 4` bit-by-bit rung.

The bench runs the full three-pass sequence and logs each pass
separately, so the plan's pass-0 + pass-2 subtotal stays derivable.

A third, larger omission is in the plan's traffic model. It counts the
refine cascade's 9 or 17-19 `tone_amplitudes` evaluations and stops
there. But mode 2 — the Fano step — wraps its decode in a **DT peak-up
loop** of up to 17 positions (`iifac = 8`, `idt ≤ 128/8`), *per*
`nblocks` rung, each position calling `tone_amplitudes` again. A
candidate that fails every rung in pass 2 pays 4 × 17 = 68 extra
evaluations on top of the cascade's ~19. The measured per-candidate
counts below are the ground truth the model should have been built on.

<!-- COUNTS -->

## What this does not measure

Everything the plan said it would not: **sensitivity**. Every number
here is time and memory. The corpus gap the plan documents —
`gen_wspr_sweep_wavs.sh` generating single-signal, DT = 0, f0 = 0 files
that pin all four tones to exact bin centres — is untouched by this
work and still gates any claim that a DDC-fed decoder decodes as well.
