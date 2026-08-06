# WSPR decode-speed investigation

New investigation (2026-08-06), same "diagnose against real WSJT-X
audio, profile, propose, verify against recall before implementing"
methodology as `FT4_BENCHMARK.md`/`FST4_BENCHMARK.md`/`Q65_BENCHMARK.md`'s
speed sections — first such document for WSPR. Prompted by a
follow-up question after the FT8/FST4 → FT4/Q65 → WSPR efficiency
campaign (PR #233/#235/`perf/wspr-jt9-hotpath-pass`): that pass landed
4 WSPR items (FFT-planner caching, a dead clone, `fano_decode` scratch
pooling, an LPF-loop iterator conversion), all verified byte-identical
against `wspr_wsjtx_sample_recall_vs_golden`, but a true apples-to-apples
same-session measurement (`git worktree`, before any WSPR work vs after
all 4 items) found only **~1-2% overall wall-clock improvement**
(~1615ms → ~1602ms on the WSJT-X `150426_0918.wav` golden, via
`wspr_speed_diag`) — small, and almost entirely attributable to item 1
(the FFT-planner cache) alone. This document investigates *why* the
other 3 items barely moved the needle: none of them touched the
dominant cost.

**This is a proposal document, not a completed fix** — the options
below are analyzed and ranked but none are implemented yet. Treat this
the way `Q65_BENCHMARK.md`'s "Q65-60A: deferred" section treats its own
unimplemented follow-up: real diagnostic data backing each option, but
a decision on which to pursue (and in what order) is still open.

## Methodology: a real cost-split diagnostic

No per-stage breakdown existed for WSPR before this — `wspr_speed_diag`
(added alongside the perf-review's Phase 0) only times the whole
`decode_scan_subtract` call end-to-end. Added
`wspr_diag_candidate_cost_split` (`tests/wspr_wsjtx_samples.rs`,
`#[ignore]`d) to fix that: it replicates `decode_scan`'s exact
call sequence using its own `pub` building blocks
(`decimate_to_baseband`, `coarse_baseband`, `decode_at_baseband`,
`decode_at_baseband_nblocks`, `subtract_signal_baseband`), with
`Instant` timers around each stage, on the real WSJT-X golden WAV
(`150426_0918.wav`).

## Finding 1: `decode_scan_subtract` is a nested 2-pass-inside-2-pass structure

This wasn't obvious from reading `wspr_speed_diag`'s single end-to-end
number, but became clear once `wspr_diag_candidate_cost_split` was
built: `decode_scan_subtract` (`decode.rs:501`) is an **outer**
`NPASSES = 2` SIC loop (12 kHz audio, `subtract_tones`), and each outer
iteration calls `decode_scan` (`decode.rs:278`), which is *itself* an
**inner** 2-pass structure (375 Hz baseband, `subtract_signal_baseband`):
coarse search → pass-1 candidates (single-`nblock` Fano) → subtract →
re-coarse on the residual → pass-2 candidates (3-`nblock` Fano/OSD
sweep). So a full `decode_scan_subtract` call pays the coarse+refine+
Fano pipeline **up to 4 times** in the worst case (2 outer × 2 inner),
not 2.

`wspr_diag_candidate_cost_split`'s output for one `decode_scan` call
(i.e. one outer-pass iteration) on the golden WAV:

```
decimate_to_baseband      =     35.1 ms
coarse_baseband (pass 1)  =     16.0 ms  (13 candidates)
pass-1 refine+Fano total  =    115.4 ms  (8878.0 µs/cand, 8 decoded, nblocks=[1])
subtract_signal_baseband  =    102.0 ms  (8 decodes subtracted)
coarse_baseband (pass 2)  =     15.8 ms  (13 candidates)
pass-2 refine+Fano total  =    459.2 ms  (35326.2 µs/cand, 9 decoded, nblocks=[1,2,3])
------------------------------------------------
sum of stages above       =    743.6 ms
```

Doubling for the outer `NPASSES = 2` loop (`decode_scan_subtract` runs
this whole sequence twice, second time on the 12 kHz-level residual)
lands very close to `wspr_speed_diag`'s independently-measured
~1600ms end-to-end total — confirming the nesting model, not just
theorizing about it.

## Finding 2: pass-2's 3-`nblock` sweep is the dominant single cost

Within one `decode_scan` call, pass-2's refine+Fano total (459.2ms) is
**~62% of the summed stages**, and per-candidate it costs **~4x**
pass-1's (35.3 ms/cand vs 8.9 ms/cand; both passes
share the same candidate count, 13, so the delta is entirely the
`nblocks=[1,2,3]` sweep vs `nblocks=[1]`). Mode-0/mode-1 (lag/freq
refine — 9 `tone_amplitudes` evals, shared once per candidate
regardless of `nblocks`) is paid identically by both passes; the ~4x
delta is 2 *extra* `(nblock_bit_metrics + Fano + OSD-fallback-on-failure)`
cycles, at roughly 13ms each.

**Combined**: pass-2's sweep, run inside *each* of the outer SIC's 2
passes, is roughly `459ms × 2 ≈ 918ms` of the ~1600ms total — the
single biggest lever in this whole decode path, well ahead of
anything item 1-4 touched.

## Options considered

Ranked by confidence × expected impact × implementation risk, not
implementation order — see "Recommended sequencing" below.

### Option A — parallelize the per-candidate loops (rayon)

**What**: WSPR's pass-1 and pass-2 candidate loops
(`for c in &cands { decode_at_baseband(...) }` /
`decode_at_baseband_nblocks(...)`) are fully sequential today. Each
candidate's decode is a pure function of `(idat, qdat, sample_rate,
candidate)` — no shared mutable state during the loop body, results
only get pushed to a local `Vec` afterward. FT8 already established
the exact idiom for this crate
(`src/ft8/decode.rs:361-368`): `#[cfg(feature = "parallel")]`
`candidates.par_iter().filter_map(...).collect()`, falling back to
plain `.iter()` when the feature is off. `par_iter()` + `.collect()`
is order-preserving, so downstream dedup logic (which relies on
first-occurrence-wins) stays correct unchanged.

**Confidence**: highest of the four options. This doesn't change the
algorithm or its output at all — same computation, same results, just
concurrent — so it doesn't carry the recall-risk the other options do.
Real speedup depends on core count and `max_candidates` (13 real
candidates on the golden WAV; more on a busy band), but pass-2's
459ms/call component alone has enough independent per-candidate work
(13 candidates × ~35ms each) to be worth parallelizing even on a
modest core count.

**Risk**: low. Main things to verify: `WsprResult` and the closures'
captured types need to be `Send`/`Sync` (should be — no interior
mutability, no `Rc`/`RefCell` in the call chain); confirm the
`parallel` feature's existing cfg-gate pattern compiles clean on the
no_std/embedded combos the way FT8's own gate already does (WSPR is
`std`-only-realistic today per the existing FFT-planner-cache work,
but should still respect the feature boundary for consistency).

### Option B — adaptive/early-exit `nblock` sweep

**What**: pass-2 unconditionally tries all of `nblocks=[1,2,3]` per
candidate, even when `nblock=1` already converges (its Fano succeeds
— `best_type1`/`best_other` get set, but the loop keeps trying `2`/`3`
anyway looking for a *lower-hard-error* result, not just any
converging one). A cheaper gate — stop the sweep once a candidate hits
some `hard_errors` threshold low enough that further `nblock` values
are very unlikely to improve it — could skip a meaningful fraction of
the 2 extra cycles per candidate.

**Confidence**: medium. The 13ms/extra-cycle cost is real and this
would cut some of it, but *how much* depends on how often `nblock=1`
already converges cleanly vs how often the coherent-block gain from
`nblock=2/3` is actually needed to convert a near-miss into a decode
(this is explicitly the scenario the code comment at `decode.rs:171-175`
describes W3BI needing) — an aggressive early-exit could cost real
recall on exactly the weak signals pass 2 exists for. **Needs a
calibration diagnostic** (mirroring Q65's
`q65_candidate_score_calibration_diag` — profile real hard-error-count
distributions across `nblock` values on the golden WAV, and ideally a
larger off-air corpus) before choosing a threshold, not a guessed
cutoff.

**Risk**: medium-high — this is an algorithmic change to the decode
ladder, not a pure refactor; needs the full recall-invariance bar
(8/8 golden) *and* probably a wider sensitivity check than the single
golden WAV gives, since the failure mode (losing weak-signal recall)
wouldn't necessarily show up on one 8-transmitter slot.

### Option C — question the outer/inner 2-pass nesting itself

**What**: `decode_scan_subtract`'s outer `NPASSES=2` SIC (12 kHz,
`subtract_tones`) and `decode_scan`'s own inner 2-pass SIC (375 Hz
baseband, `subtract_signal_baseband`) are two *different*
implementations of essentially the same idea (remove decoded signals,
re-search the residual for weaker ones) operating at two different
sample rates. It's not obvious both layers are pulling their weight —
`decode_scan_subtract`'s own doc comment already notes it caps at 2
outer passes specifically because "the bulk of the SIC benefit lands
on pass 2 once the strong ... signals have been removed," which is a
claim about the *inner* pass 2 doing most of the work, not evidence
that the *outer* pass 2 (a second full `decode_scan` call, i.e. up to
2 more full inner passes) earns its ~2x cost.

**Confidence**: low-to-medium on the *direction* (there's likely real
redundancy here), very low on knowing the *right* fix without
measuring — this needs its own dedicated investigation (which recall
gains, if any, does dropping/merging a layer cost — the W3BI-class
signal specifically motivated the *inner* pass 2 per the code
comments, but the *outer* pass 2's marginal contribution isn't
documented anywhere and should be measured directly: run
`decode_scan_subtract` with `NPASSES=1` against the golden WAV and see
which of the 8 golden messages, if any, only the outer pass 2 finds).

**Risk**: highest of the four — this is an architectural question
about the decode strategy, not a mechanical optimization, and getting
it wrong costs real recall on weak/crowded-band signals, which is
WSPR's whole reason for existing as a mode. Do not attempt without a
dedicated calibration pass first (same spirit as Option B, but at a
coarser granularity), and treat a "no measurable recall loss" result
on one golden WAV as necessary, not sufficient, given only 8
transmitters are in it.

### Option D — OSD fallback cost

**What**: the per-audit finding that `osd::osd_decode`'s order-2
search (`osd.rs:180-192`) is `C(50,2) = 1225` trials × O(N) work,
run as a fallback whenever Fano fails to converge — which, given pass
2's `nblocks=[1,2,3]` sweep, can happen up to 3x per candidate. Not
separately broken out by `wspr_diag_candidate_cost_split` (it's
folded into the pass-1/pass-2 refine+Fano totals above), so its actual
share of the ~35ms/candidate pass-2 cost isn't known yet — would need
a further diagnostic split (time `codec.decode_soft_pooled` vs
`osd::osd_decode` separately inside the sweep) before this is
actionable.

**Confidence**: unknown pending the above split — this is explicitly
the least-investigated of the four options. Likely a smaller
contributor than A-C (OSD is a fallback path, not the common case),
but worth a follow-up measurement rather than dismissing it outright.

**Risk**: low if pursued as pure allocation/loop-count reduction
inside OSD (same shape as this session's other neutral items); medium
if pursued as reducing the *trial count* (that would be a sensitivity
trade-off like Option B, needing the same calibration discipline).

## Recommended sequencing

1. **Option A first** — parallelize the candidate loops. Highest
   confidence, lowest risk, no recall-invariance question at all (it's
   not an algorithm change), and the plumbing already exists in this
   crate (FT8's exact pattern to copy). This alone, on a multi-core
   host, could meaningfully cut into the ~918ms pass-2-across-both-outer-passes
   component without touching anything that needs new measurement
   discipline beyond "still 8/8 golden, still same messages."
2. **Then split Option D's OSD-vs-Fano cost** inside pass 2's sweep —
   cheap to add (a few more `Instant` timers in
   `wspr_diag_candidate_cost_split` or a dedicated variant), and tells
   us whether Option D is worth a dedicated pass or can be folded into
   whatever Option B/C work follows.
3. **Option B and C are architectural/algorithmic and need their own
   calibration diagnostics before any code changes** — do not
   implement either from the reasoning in this document alone. Build
   the calibration diagnostics described in each section first (hard-
   error-count distribution across `nblock` values for B; an
   `NPASSES=1` vs `NPASSES=2` golden-recall diff for C), review what
   they show, *then* decide whether either is worth pursuing — matching
   this campaign's own established discipline (measure before
   implementing, not after).

None of this is scheduled — this document exists so the next session
picking up WSPR speed work starts from real profiled data instead of
re-deriving it.
