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

**Update (2026-08-06, same day)**: Option A (candidate-loop
parallelization) is now implemented and merged into
`perf/wspr-jt9-hotpath-pass` — measured ~2.4x speedup, by far the
largest single win of the whole FT8/FST4/FT4/Q65/WSPR campaign.
Option C's calibration diagnostic is also done, with a clean result
recommending implementation (see that section below). The rest of this
document's "proposal, not completed fix" framing still applies to
Options B and D and to Option C's *implementation* (only its
calibration is done) — see "Recommended sequencing" at the bottom for
current status of each.

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

**Calibration run** (`wspr_diag_pass_ablation`,
`tests/wspr_wsjtx_samples.rs`, added 2026-08-06): 3 conditions on the
golden WAV, all via `pub` building blocks (`decode_scan` itself already
*is* "inner=1+2, outer=1" — no new helper needed for that half):

```
A inner=1only outer=1            83.7 ms  7/8 golden  [...all but W3BI...]
B inner=1+2   outer=1           130.2 ms  8/8 golden  [...+ W3BI...]
D inner=1+2   outer=2 (prod)    330.9 ms  8/8 golden  [same set as B]

inner pass-2 contributes (B \ A): ["W3BI FN20 30"]
outer pass-2 contributes (D \ B): []
```

Reproduced 3x, deterministic (no run-to-run variation in which golden
messages hit, only timing). **The inner pass 2 earns its cost exactly
as the code comments claim — it's the only thing that recovers W3BI
(-27 dB SNR).** The outer pass 2, on this golden WAV, recovers
**nothing** beyond what a single `decode_scan` call already finds,
while costing an extra ~201ms — roughly **2.5x** condition B's own
cost, for zero measured benefit here.

**Caveat, unchanged from before the calibration run**: this is one
golden WAV with 8 transmitters (the only real WSPR sample in this
repo's `WSJT-X/samples/WSPR/` — no second sample exists to
cross-check against). A "0 marginal recall" result on 8 signals is
suggestive, not proof, that the outer pass never helps on a busier or
differently-conditioned band — WSPR slots can carry many more
concurrent transmitters than this one does, and the outer pass's
whole justification is recovering signals masked by *other* signals'
subtraction residue, which a sparser slot exercises less. Treat this
as strong evidence to act on with an appropriate fallback (see below),
not as license to delete the outer pass outright without further
safety margin.

**Recommendation**: this now has enough evidence to implement, but
conservatively — not by deleting `NPASSES=2` outright. A reasonable
shape: keep the outer loop but make its second iteration
*conditional* (e.g. skip it when pass-1's own inner-pass-2 already
found signals covering a configurable fraction of the coarse
candidates, or simply expose `NPASSES` as tunable and default it to 1
while keeping 2 available for callers who want maximum recall on busy
bands at 2.5x the cost) rather than a blanket removal — this keeps the
"more real evidence might change this" caveat above from becoming a
silent regression for someone else's use case. Still needs the full
recall suite (`wspr_wsjtx_sample_recall_vs_golden`, 8/8) as a hard
gate regardless of which shape is chosen.

**Risk**: was rated highest of the four before this calibration run;
now downgraded to medium given the clean, reproduced 0-contribution
result — but the single-sample caveat above keeps it above Option A's
risk floor. Do not skip the recall-suite gate even though the
diagnostic already ran it once.

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

1. ~~**Option A first**~~ — **done** (`perf/wspr-jt9-hotpath-pass`,
   commit `71fc382`). Parallelized the candidate loops via
   `par_iter()`, mirroring FT8's existing pattern. Measured ~2.4x
   speedup (git worktree A/B, alternating, same session), no algorithm
   change, 8/8 golden held.
2. ~~**Option C's calibration**~~ — **done** (`wspr_diag_pass_ablation`,
   see the updated Option C section above). Clean, reproduced result:
   the outer `NPASSES=2` pass contributes 0 marginal golden recall on
   the one real WSPR sample available, at ~2.5x the cost of stopping
   after one `decode_scan` call. **Now the top implementation
   candidate** given both its measured impact (bigger than Option A's,
   on paper — cutting straight to ~130ms vs today's ~330ms on this
   golden) and now-downgraded risk, but still needs the conditional/
   tunable-rather-than-deleted shape discussed in that section, plus
   the full recall gate, before landing.
3. **Then split Option D's OSD-vs-Fano cost** inside pass 2's sweep —
   cheap to add (a few more `Instant` timers in
   `wspr_diag_candidate_cost_split` or a dedicated variant), and tells
   us whether Option D is worth a dedicated pass.
4. **Option B still needs its own calibration diagnostic before any
   code changes** — hard-error-count distribution across `nblock`
   values, not yet built. Do not implement from the reasoning in that
   section alone.

Only Option A is actually landed — the rest (including Option C, whose
calibration is now done but whose implementation is not) exists here
so the next session picking up WSPR speed work starts from real
profiled data instead of
re-deriving it.
