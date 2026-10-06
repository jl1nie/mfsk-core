# Early decode for live use — design (#572)

Status: **draft for review, 2026-10-07.** Nothing here is implemented; nothing in this file has a committed probe behind
it — every number cited was measured with a throwaway harness (native and `wasm32-unknown-unknown`, since WebFT8 is the
reference consumer and runs on the latter) and then discarded. §8 records what each number is sourced from. Target: 0.14;
breaking changes are allowed, but #573 (`#[non_exhaustive]`) lands first.

## 0. Goal and use cases

FT8 rows should reach a live caller at about 11.8 s into the period, as WSJT-X delivers them, and the caller should decide
how much of the machinery it wants. The design has to serve every caller below without a special path for any of them:

| | caller | how audio arrives | threads | what it wants |
|---|---|---|---|---|
| U1 | offline decode of a recording | the whole period at once | any | today's `decode`, unchanged |
| U2 | desktop GUI, native | pushed in blocks | one per channel | rows at 11.8 s, the rest at the end |
| U3 | browser PWA (WebFT8) | AudioWorklet blocks into one worker | one, cannot block | same as U2, never blocking |
| U4 | skimmer over `IqReceiver` | wideband IQ, N channels | a pool | early rows on some channels only |
| U5 | C / Kotlin / Swift app | `mfsk_stream_push_*` | the app's | the same through the C ABI |
| U6 | research / validation | a recording | any | `jt9`'s exact 41 / 47 / 50 calls |
| U7 | a caller with its own clock and buffers | its own | its own | to choose *when* to call, with no cutter |
| U8 | a host short on CPU, or on a busy band | any | any | to skip a stage, cap redecode cost, or go deep, period by period |

## 1. What upstream does (v3.2.0-rc1, `567ad29ce`)

FT8 is decoded three times per period, on growing prefixes of the same audio. One `hsym` is 3456 samples, 0.288 s.

| call | `nzhsym` | samples | time | `ft8_decode.f90` |
|---|---|---|---|---|
| A | 41 | 0..141 696 | 11.8 s | Zero the tail and run three sync+decode passes. Keep the decodes (`ndec_early`, `itone_save`/`f1_save`/`xdt_save`, SAVE). |
| B | 47 (46 in ft8md "very early") | 0..162 432 | 13.5 s | **No search.** Subtract the A decodes whose frame fits (`xdt-0.5 < 0.396`), then `go to 900`. |
| C | 50 | 0..172 800 | 14.4 s | Head is B's cleaned buffer, tail is fresh audio. Subtract the A decodes not yet subtracted, then run three passes. |

- A starts the period: `nzhsym==41` or a new `nutc` resets the a7 table. With `ndepth==1`, nothing runs before 50.
- In 3.2.0-rc1, `syncmin=2.0` at 41 is commented out. mfsk-core already follows this (#452).
- The live GUI bails out on wall-clock time (`tseq >= 13.4 s` in A, `14.3 s` in B's subtraction). `jt9` replays 41/47/50 on
  a file (`jt9.f90:340-354`).
- ft8md (#463) is opt-in: `m_multithreadFT8` defaults to `false` (`mainwindow.cpp:619`). So the A/B/C path above is what a
  default user runs.
- WSJT-X's own extra suppression from multiple SIC rounds (~17.65 dB against ~6.6 dB for one subtraction) comes from "the
  *outer* `do ipass=1,npass` loop re-detecting the same residual signal as a fresh candidate in a later pass" — a real
  station is meant to be redecoded, not just subtracted once. mfsk-core's own code comments cite this directly
  (`ft8/decode.rs`, issues #177/#179).

## 2. What mfsk-core has

- `Ft8Strategy::SicEarly` already runs A, B and C over a whole-slot buffer (`ft8/decode.rs`,
  `decode_frame_subtract_staged_with_ap_inner`, #180). Its rows stream as A finds them, but the call cannot start until
  the period is complete.
- What upstream keeps between the calls (`early_results`, `buf_b`, `deferred`) is local to that one function.
- The function is already length-generic: B's fit limit is computed from `b_len`, not hard-coded at 47, so stages at other
  sample counts need no new algorithm.
- Checkpoint A itself runs `CHECKPOINT_SIC_ROUNDS = 3` internal SIC rounds over its prefix, and checkpoint C runs 3 more —
  up to 6 rounds total, each up to `max_cand` candidates, each candidate re-triaged and (if it survives) fully redecoded.
- The subtraction loops between checkpoints (`for r in &early_results { subtract_signal_lpf_refine_dt(&mut buf_b, r); }`
  at checkpoint B, and the equivalent for `deferred` at checkpoint C) run with **no budget check inside them at all** —
  confirmed by reading both loops directly. This matters for §6 trap 6.
- `SlotInput` is one whole period and is already `#[non_exhaustive]`. `IqReceiver` and the C stream cut whole slots with
  `SlotCutter`. The boards run their own prefix path (`embedded-shared`).

## 3. Principles

1. **One primitive, many drivers.** The decoder gets one new entry point. Everything that decides *when* to call it is a
   separate, optional layer, so a caller can take the layer that fits or none at all (U7).
2. **The audio length is the checkpoint.** The caller says *which kind* of stage to run and passes the period so far. How
   many samples that is, is up to the caller. Upstream's counts are a published default, not a rule (the ft8md 46, U7).
   This does **not** mean a shorter prefix is cheaper to process — see trap 6: `compute_spectrogram`/`coarse_sync`/
   `build_fft_cache` cost is fixed regardless of prefix length. What a caller actually controls by choosing a shorter
   prefix is *when* it can call (how much real audio exists yet), not the cost of any one call.
3. **Every sequence is valid.** Calls may be skipped, repeated or out of order. Each case has a defined result, and the
   worst case is today's whole-period decode. Nothing errors except a mode that has no stages (U8).
4. **The state lives in the `Decoder`**, where `ft8_decode.f90` keeps its SAVE variables, as 0.13 already does for
   everything kept between periods.
5. **No clock and no blocking.** Time is the sample count, and every call returns as soon as its work is done (U3).

## 4. Layer 1: the decoder entry point

```rust
pub enum Stage {
    /// Search the period so far; stream what is found. Starts the period's stage state.
    Early,
    /// Subtract what `Early` found from a longer prefix. No search, no rows.
    Prepare,
    /// The whole period: search what is left; stream only what `Early` did not.
    Final,
}

impl Decoder<Ft8> {
    pub fn decode_stage(&mut self, slot: &SlotInput<'_>, stage: Stage, on_row: Option<OnRow<'_, Ft8Row>>)
        -> SlotResult<Ft8Row>;
}
impl AnyDecoder {
    pub fn decode_stage(&mut self, slot: &SlotInput<'_>, stage: Stage, ...) -> Result<AnySlotResult, Unsupported>;
}
```

`decode(slot)` remains, and is exactly `decode_stage(slot, Final)` with no stage before it. U1 sees no change.

**`Early`'s prefix does not have to be upstream's.** Principle 2 means a caller may pass *any* prefix to `Early`,
including the whole period. When it does, `Final` has no fresh tail to add (`b_len == c_len == audio.len()`), so it
degenerates to "subtract `Early`'s rows from this same buffer, then search the residual" — one search, not two. This is
not a special case to code separately; it falls out of `Prepare`/`Final` being defined in terms of *whatever* `Early`
covered. **Verified, not assumed** (§8): composing exactly this sequence — `Early(whole period, SinglePass)`, subtract its
rows, `Final`'s own dedup against them, then `SicRounds(3)` on the residual — reproduces `SicEarly`'s full result exactly
on all three #587 recordings, row for row, no misses, no extras. This is also the WebFT8 use case: its phase 1
(`SinglePass`) / phase 2 (`SicEarly`, over the same complete audio) becomes `Early` then `Final`, each running one search
instead of `SicEarly` redundantly re-running an internal checkpoint-A search over data `Early` already covered.

**Rows.**
- `on_row` sees each row of the period **once**, when it is first found.
- `Final` returns the period's **complete** set (`Early`'s rows and its own), in the order of discovery. That is the same
  list a whole-period `SicEarly` returns, which is what makes equivalence testable.
- A row is never retracted: the dedup runs before delivery (#243).
- `RowDetail` gains `stage: Stage`, so a caller replying to a CQ can tell an early row from a final one. This field is why
  #573 comes first.
- `Final` must pass `Early`'s rows as `known` to its own internal dedup, not only subtract them from the buffer. An
  imperfectly-subtracted residual can still re-decode the same message weakly (the #243-class hazard); a harness or
  implementation that only subtracts and then compares output texts will double-count it as new.

**Every sequence** (`p` is `slot.period`, `None` means "the current period"):

| call | state before | result |
|---|---|---|
| `Early` | any | Discard any earlier state and start period `p`. Search the prefix and keep its rows. A second `Early` in the same period starts over: its rows are deduplicated against those already delivered. |
| `Prepare` | `Early` of `p` | Subtract the early rows whose frame fits this prefix. Rows: none. |
| `Prepare` | none, or another period | No-op. |
| `Final` | `Early` of `p` (± `Prepare`) | Cleaned head (through `Prepare`, if called) plus a fresh tail from `Early`'s prefix to the period's end — upstream's C when that prefix is 141 696; empty when `Early` already covered the period. Subtract whatever is left, search. Clear the state. |
| `Final` | none, or another period | Today's whole-period decode (the flat fallback `SicEarly` already uses when A found nothing). Clear the state. |

**The options a caller has.**
- **Strategy.** Stages exist for `Ft8Strategy::SicEarly`, the `Normal`/`Deep` default. With `SinglePass` or `SicRounds(n)`,
  `Early` searches the prefix with that strategy, `Prepare` is a no-op, and `Final` runs the strategy on the residual.
  This is the real lever for a tight budget or a busy band (trap 6): `SinglePass`/`SicRounds(1)` bound how many times a
  real signal gets redecoded (1-2 rounds) where `SicEarly`/`SicRounds(3)` allows up to 6 (checkpoints A and C, 3 rounds
  each). A caller on a hard deadline should pick a cheaper strategy, not a shorter prefix (principle 2).
- **Depth.** With `Depth::Fast` (`ndepth==1`), `Early` and `Prepare` return nothing, as upstream does. A caller that wants
  early rows anyway sets a strategy in `Ft8Extras` explicitly. That is a library extension, written down as one.
- **Budget.** Each call takes `slot.budget`. The per-candidate check (already in the engine) works correctly at
  candidate granularity. It does **not** cover the subtraction loops between stages (trap 6) — `Prepare` and `Final`'s
  own subtract-before-search step must add a check before each row's subtraction, the same shape as the existing
  per-candidate check, or they inherit `SicEarly`'s current gap.
- **Abandoning a period.** `Decoder::abandon_period()` is for a gap or a retune. Nothing needs it, because the next
  `Early` resets anyway, but it frees the buffers.

**Modes without stages.** `AnyDecoder::decode_stage` returns `Unsupported` for `Early` and `Prepare`, and `Final` works
for every mode. `registry::caps` gains `CAP_EARLY`, so a driver asks instead of guessing. FT4 has no early decode
upstream. JT9, JT65, Q65, WSPR and FST4 decode at the period's end.

**What the state holds.** Per decoder, so per channel: the early rows; B's cleaned head (one `i16` buffer of period
length, 360 KB); the deferred rows; the pinned gain (trap 1); the period index. Allocated at the first `Early`, never for
a caller that does not use stages.

## 5. Layer 2: drivers (optional; take one, or none)

**5.1 What the core publishes.**
- `ProtocolMeta::early_points: &'static [(u32, Stage)]` holds upstream's default. FT8 is `[(141_696, Early), (162_432,
  Prepare)]`, and every other mode has an empty slice. `Final` is the slot length, which the registry already gives.
- `SlotCutter::open_prefix() -> Option<(period, &[T])>` lets a caller look at the slot being cut. The cutter holds that
  buffer already, so nothing new is stored.

**5.2 `IqReceiver` (U4).**
- Early decode is opt-in per channel: `set_early(channel, true)`.
- An opted-in channel yields `StagePoint { channel, period, stage, audio }` events beside its `CompletedSlot`s, through
  the same `push_*` → `out` pull. A channel without the option costs nothing.
- `audio` is an owned `Send` copy, so the decode can run on any thread. The f32 prefixes at A and B are 567 KB and 650 KB,
  about 1.2 MB per channel-slot.
- The receiver scales an opted-in channel's slot by the gain of its first stage point, not by the RMS of the whole slot
  (trap 1).

**5.3 The C ABI stream (U5, and Kotlin/Swift through it).**
- `mfsk_stream_set_early(s, on)` enables it.
- `mfsk_stream_stage_ready(s)` and `mfsk_stream_take_stage_i16(s, out, cap, &stage, &period, &utc)` take a stage point.
- `mfsk_decoder_decode_stage_i16(dec, samples, n, stage, period)` decodes it. Rows go through the existing
  `set_on_decode` callback, and `MfskRow` gains the stage.
- Queue rule: like a slot, one stage point waits at a time. A newer one replaces it, so a slow consumer skips a stage
  rather than piling them up, and the table in §4 says what a skipped stage means.

**5.4 Your own loop (U2, U3, U7).** A caller can skip all of the above:
- U3 (WebFT8): push AudioWorklet blocks into its own buffer. When the buffer passes `early_points`, call
  `decode_stage`. Each call returns in hundreds of milliseconds to a few seconds depending on strategy (§8), and
  nothing blocks.
- U2: the same, on its own thread.
- U7: any counts it likes.

**5.5 `jt9` replay (U6).** Call `decode_stage` on `&audio[..141_696]`, `&audio[..162_432]` and the whole slot.
Alternatively, feed the recording through the stream with no clock set: the grid free-runs, and the stage points land on
exactly those counts.

**Latency.** Through a driver, a stage point is seen by the `push` that crosses its count, so latency equals the push
block size.

## 6. Traps, and where each is handled

1. **f32 gain.** `frame.rs::f32_gain` scales by the audio's own RMS. On a prefix, that RMS changes from one stage to the
   next, so the same signal would reach the level-sensitive 16-bit engine at three levels. The fix: the gain is pinned at
   the period's first stage, kept in the state, and reused. `i16` audio has no gain (it is `jt9`'s `id2`).
2. **The zero tail is part of the algorithm.** Buffers stay full length with the content zeroed past the prefix, so
   `subtract_signal_lpf` has room for a frame near the edge. A short `SlotInput` never becomes a short buffer. The
   current code already works this way; keep it.
3. **Hash table timing.** `Early`'s calls are learned before `Final` searches, as upstream learns at each decode.
   `on_row` resolves against the table as it stood when the stage began. The returned rows also see what was learned
   earlier in the period.
4. **Period identity.** Covered by the rule table in §4. A stale state can never reach the wrong period.
5. **Wall clock.** Upstream's `tseq` bail-outs are not ported, because this crate has no clock. `Budget` serves that
   purpose.
6. **A prefix does not shrink a round's cost; the real, fixable budget gap is the unbudgeted subtraction loop between
   checkpoints, not the per-candidate search.** Measured in three stages (§8 has the raw numbers and sourcing):
   - `compute_spectrogram`/`coarse_sync`/`build_fft_cache` cost is **flat regardless of prefix length** —
     `compute_spectrogram`'s time dimension is the hard-coded `NMAX = 15*12000` (`params.rs`), not `audio.len()`, and
     `build_fft_cache` zero-pads to a hard-coded 192 000 (`engine/dsp/downsample.rs`) the same way. A shorter `Early`
     prefix does not make any one call cheaper.
   - The per-candidate search loop's own budget check works correctly, at candidate granularity (gaps of a few tens of
     ms between checks while candidates are processed). A busy band legitimately redecodes real signals up to 6 times
     under `SicEarly` (checkpoints A and C, 3 SIC rounds each) — deliberate, upstream-faithful (§1), not a defect.
   - The actual, large (hundreds of ms to over a second) unbudgeted gap is the subtraction loop between checkpoints
     (§2): `subtract_signal_lpf_refine_dt` costs roughly 60-100 ms per row, runs once per row found by the previous
     stage, with **no check at all** inside the loop. This is real and fixable: add a budget check before each row's
     subtraction, the same shape the per-candidate loop already has. `Final` and `Prepare`'s own subtract-before-search
     step must have this from the start, or they carry `SicEarly`'s current gap forward unchanged.

## 7. Validation

1. **Equivalence.** `Early` on the A prefix, `Prepare` on the B prefix, then `Final` on the whole slot gives the same
   rows, in the same order, as `SicEarly` on the whole period. Covered by the FT8 goldens
   (`ft8_qso3_staged_sic_check`, `ft8_qso3_full_parity_recall`) and a test of its own. Run it under `fixed-point` too.
2. **Against upstream.** The rows at each stage equal what an instrumented `jt9` reports at `nzhsym` 41, 47 and 50 on the
   same recording.
3. **Precision.** This is the path where phantom decodes have come from (#243, #253). Required: zero extra decodes
   against the 20-entry `qso3_busy` union, and an unchanged FT8 unexpected-decode count in `sweep-baseline.json`.
4. **Level.** `f32` input gives the same rows as the same audio as `i16` at the pinned gain.
5. **Every row of the §4 table**, as a test. Also: a prefix shorter than A, `Early` called twice, and a period change
   between stages.
6. **Drivers.** A recording through `IqReceiver` and through the C stream yields stage points at the published counts
   and the same rows as test 1.
7. **The `Early`(full period)/`Final` sequence vs `SicEarly`**, on the #587 recordings: same rows, no misses, no extras
   (§8 §4's verified claim). Separately, `Final`'s own dedup must not report a re-find of one of `Early`'s own rows as
   new (the pitfall in §4's "Rows").
8. **The subtraction budget check (trap 6).** A `BudgetCheck` that returns `false` partway through `Final`'s or
   `Prepare`'s row-subtraction loop stops it before the remaining rows are subtracted, the same way the per-candidate
   loop already stops. A regression here is silent: nothing currently tests that this loop is interruptible at all.
9. **Prefix-independence of the per-round fixed cost (trap 6).** A round's `compute_spectrogram`/`coarse_sync` cost
   should not vary (beyond noise) across prefixes from a small fraction of the period to the whole period. A
   regression here would show as cost growing with prefix length, which would also silently invalidate principle 2's
   "shorter prefix ≠ cheaper call" statement.

## 8. What was measured, and where the numbers came from

Every number below is from a throwaway `wasm-bindgen` probe (never committed) built against this crate with
`internal-testing`, run under `wasm-pack --target nodejs --release` + Node, and in two cases also natively, against the
real `embedded-poc/assets/` recordings `qso3_busy.wav`, `191111_110130.wav`, `191111_110200.wav`. Same settings
throughout: band (100, 3000), sync_min 1.0, max_cand 200, Deep strictness, OSD on — WebFT8's own `Ft8Run::wide(1.0,
SicEarly, Deep)`.

- **`Early`+`Final` matches `SicEarly` exactly.** Composed `Early(SinglePass)` → subtract its rows → dedup against them
  → `SicRounds(3)` on the residual, and diffed the row set against `SicEarly` on the same audio: 21/21 on `qso3_busy`
  (including the one row, `CQ DX DL8YHR JO41`, that a flat `SicRounds(3)` *without* the subtraction step misses — the
  subtraction is what recovers it), 5/5 with 0 new on each sparse recording, matching `SicEarly`'s own 0 new there.
- **`Early`+`Final`'s cost vs `SicEarly`, timing the subtraction step too** (an earlier version of this measurement
  omitted that step between two separate timers and overstated the saving): 69-85 % of `SicEarly`'s wall time across
  the three recordings, under `wasm32-unknown-unknown`. The saving is avoiding the duplicate checkpoint-A search; the
  subtraction cost itself is paid by both sequences and is not something this design avoids.
- **Per-round fixed cost (`compute_spectrogram` + `coarse_sync` + `build_fft_cache`) is flat across prefixes** from
  1.5 s to the full 15 s on `qso3_busy.wav` — every point measured within a few ms of the others (~110 ms under wasm).
- **Reproducing #587's own budget experiment** (`SlotInput::budget`, a closure crossing the wasm/JS boundary via
  `js_sys::Date::now()` exactly as WebFT8's `run_slot` does): a 300 ms budget on `qso3_busy.wav` costs ~1.1-1.2 s wall,
  matching the issue's own reported number within 7 %. `BudgetReport::stages_run` at no budget is exactly `6 × 200` —
  the full checkpoint A (3 rounds) + checkpoint C (3 rounds) structure, confirming the redecode count directly rather
  than inferring it.
- **The unbudgeted subtraction loop, and its cost**: found by the #587 reporter's own timestamped-gap measurement
  (closure call times logged across the whole decode, native and wasm, independently), which located large unchecked
  stretches that line up closely with an isolated timing of `subtract_signal_lpf_refine_dt` on the rows a prior pass
  found (roughly 60-100 ms/row on both native and wasm). Confirmed by reading the two subtraction loops in
  `decode_frame_subtract_staged_with_ap_inner` directly: neither contains a `budget.allows()` call.

None of this is in the crate's test suite. A real implementation needs its own measurements against the committed
goldens, not a rerun of these throwaway numbers.

## 9. Order of work

1. #573: `#[non_exhaustive]` on `RowDetail`, `SlotResult` and the rest.
2. Move `early_results`, `buf_b` and `deferred` out of the function into a state type, so `SicEarly` is three stage calls
   in a row, **plus** a budget check in both subtraction loops (trap 6) — not deferred to a later step, since it is a
   real, currently-shipping gap in `SicEarly` itself, not only a property of the new API. No behaviour change beyond
   that budget check; validation 1 and 8 prove it.
3. `decode_stage`, `Stage`, `RowDetail::stage`, `CAP_EARLY`, `abandon_period`. Validations 2-5, 7.
4. `early_points` and `open_prefix`. Then `IqReceiver::set_early` and stage points. Then the C entries, then Kotlin and
   Swift. Validation 6.
5. Moving the boards' prefix path onto the same stages is a separate decision. The boards have no `Decoder` today.

## 10. Open questions

- **Should `Prepare` be public?** It produces no rows. In its favour: it matches `jt9`, and it lets a caller use the
  1.7 s between A and C. Against it: one more case in the table. Recommended: keep it, so drivers can call it and a
  caller can skip it.
- **`Early` with `Depth::Fast`.** Should it follow upstream and do nothing, as proposed, or should it run a single
  pass? The first keeps parity; the second is friendlier to U8.
- **Should stages be offered for FT4?** Upstream has none, so it would be a library extension. Not in this design.
- **ft8md (#463)** has its own stages. If it is ported, it should reuse `Stage` rather than add a second vocabulary.
- **The subtraction budget check's own budget semantics.** Checking before each row (trap 6, §4's "Budget") is
  coarser than the per-candidate check (a row's own subtraction cost, not a candidate's). Worth deciding whether a
  partially-subtracted set of rows (stopped mid-loop) should be reported any differently from a candidate loop stopped
  mid-search, or whether `BudgetReport`'s existing fields already cover it without a new one.
- **Relationship to #587.** WebFT8's phase 1 (`SinglePass`, full period) / phase 2 (`SicEarly`, same full period) is
  exactly the `Early`/`Final` sequence this design proposes, verified in §8. The separate, larger finding from that
  issue — a 300 ms budget still costing over a second — is the subtraction-loop gap in §6 trap 6 and §9 step 2's fix,
  found by the issue's reporter, not by this design; it is a bug in `SicEarly` as shipped today, and the fix belongs in
  step 2 regardless of whether the rest of this design lands.

## 11. Considered and dropped

- **A `Checkpoint::{A, B, C}` enum tied to upstream's counts.** It names a moment rather than what the stage does, so it
  cannot express ft8md's 46 or a caller's own counts. `Stage` plus "the audio length is the checkpoint" can.
- **A per-period session object holding `&mut Decoder`.** A borrow that lasts 15 s fits no threading model here.
  Keeping the state in the `Decoder` gives the same result with no borrow.
- **One blocking call over an audio source.** It holds the decoder for about 14 s and cannot work in a single-threaded
  worker (U3) without SharedArrayBuffer, Atomics and COOP/COEP. A native caller can write it in ten lines over §5.4, so
  it is not needed in the library.
- **A resumable decode job polled in steps.** The decode is CPU-bound and already takes a `Budget`; splitting the
  decode at stage and subtraction-row boundaries covers this.
- **A stage field on `SlotInput`.** It would make `decode` mode-dependent in a way it cannot report. The convention in
  `AnyDecoder` is that an option a mode lacks is `Unsupported`.
- **Reviving 0.12's `FftCache`/`.known()`.** Considered for #587 before the actual cause was found. `FftCache` only ever
  skipped `build_fft_cache`'s forward FFT, not `compute_spectrogram`/`coarse_sync` (both prefix-independent and cheap
  regardless, per §6 trap 6) — it would not have touched either the duplicate-search cost (`Early`/`Final` fixes that)
  or the subtraction-loop gap (§9 step 2 fixes that). Not needed for either problem.
