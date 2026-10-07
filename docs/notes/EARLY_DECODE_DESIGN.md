# Early decode for live use — design (#572)

Status: **implemented 2026-10-07** (§7 steps 2 and 3, plus the C ABI half of §10). The design below is kept as
written; where the implementation departs from it, §12 says how and why. Target: 0.14.

Third revision. The first folded in #587, which **#589** then closed on its own (the budget is polled after checkpoint A
and before each row's subtraction in the B and C loops) with no new API, so that material is gone. The second answered
five open questions but grew the public surface to a `Stage` input enum, a `StageKind` for the registry table,
`prefix_points()`, `open_prefix()`, `CAP_EARLY` and `abandon_period`. Writing the caller samples killed most of that:
the stage is something the decoder can infer from what it already holds, and the published-positions table turned out
to be bookkeeping the caller was keeping *for* the library. What is left is one method pair.

| | |
|---|---|
| **added** | `Decoder::decode_prefix` / `decode_prefix_with`, the same pair on `AnyDecoder`, `RowDetail::stage` |
| **new types** | `Stage` — **output only**, matched on a row, never passed in |
| **unchanged** | `decode` / `decode_with`, `SlotInput`, every existing contract |

## 0. Goal and use cases

FT8 rows should reach a live caller at about 11.8 s into the period, as WSJT-X delivers them.

| | caller | how audio arrives | in scope |
|---|---|---|---|
| U1 | offline decode of a recording | the whole period at once | yes — `decode`, untouched |
| U2 | desktop GUI, native | pushed in blocks | **yes** |
| U3 | browser PWA, one worker, cannot block | AudioWorklet blocks | **yes** |
| U6 | research / validation | a recording, replayed at `jt9`'s counts | **yes** |
| U8 | a host short on CPU, or a busy band | any | **yes** — `Budget` and the strategy |
| U4 | skimmer over `IqReceiver` | wideband IQ, N channels | §10; the receiver half is `IQ_PREFIX_DESIGN.md` (#600) |
| U5 | C / Kotlin / Swift app | `mfsk_stream_push_*` | §10: a follow-up |

## 1. What upstream does (v3.2.0-rc1, `567ad29ce`)

FT8 is decoded three times per period, on growing prefixes of the same audio. The unit is 3456 samples — **`jt9`'s own
streaming block, which is JT9's half-symbol** (`NSPS` 6912/2, `jt9/baseband.rs:19`), not FT8's symbol and not this
crate's spectrogram step. None of the three divides the others: 141 696/1920 = 73.8 symbols, and /480 = 295.2 steps
(`NSTEP`, or 960 under `nstep-half`). A caller cannot derive these counts, which is why §4 publishes them in prose and
the decoder acts on them itself.

| call | `nzhsym` | samples | time | `ft8_decode.f90` |
|---|---|---|---|---|
| A | 41 | 0..141 696 | 11.8 s | Zero the tail and run three sync+decode passes. Keep the decodes (SAVE). |
| B | 47 | 0..162 432 | 13.5 s | **No search.** Subtract the A decodes whose frame fits, then `go to 900`. |
| C | 50 | 0..172 800 | 14.4 s | Head is B's cleaned buffer, tail is fresh audio. Subtract what is left, run three passes. |

- A starts the period: `nzhsym==41` or a new `nutc` resets the a7 table. With `ndepth==1`, nothing runs before 50.
- **No AP pass runs while `nzhsym < 50`** (`npasses=5`); already followed, cited at `ft8/decode.rs:1332-1333`.
- **a7 and a8 run only at `nzhsym == 50`**, with AP on. Sourced from this crate's port header
  (`ft8/list_decode.rs:3-5`, citing `ft8_decode.f90:250-305`) rather than the Fortran, which is not in this container —
  validation 2 is where that gets checked against it.
- In 3.2.0-rc1, `syncmin=2.0` at 41 is commented out; mfsk-core follows (#452). The live GUI bails out on wall clock
  (`tseq`); `jt9` replays 41/47/50 on a file (`jt9.f90:340-354`). ft8md (#463) is opt-in and off by default.

## 2. What mfsk-core has

- `Ft8Strategy::SicEarly` already runs A, B and C over a whole-slot buffer (`decode_frame_subtract_staged_with_ap_inner`,
  #180). Its rows stream as A finds them, but the call cannot start until the period is complete.
- **A short buffer already works.** `audio.len() < A_SAMPLES` → a flat pass (`decode.rs:1275-1297`); above it, the staged
  path runs with `b_len`/`c_len` = `*_SAMPLES.min(audio.len())`. What is missing is only that nothing is *kept* between
  calls — `early_results`, `buf_b` and `deferred` are locals.
- A second flat fallback: checkpoint A found nothing (`early_results.is_empty()`), because upstream gates B's search and
  C's splice on `ndec_early >= 1` (`decode.rs:1342-1374`). Both fallbacks are **internal branches** of the staged path.
- The budget is polled after A and before each row's subtraction in B and C (#589), guarded by
  `staged_sic_skips_b_and_c_when_a_spends_the_budget` (`decode.rs:2045`).
- a7's cross-period state is written by `remember(state, slot.period, results)` (`decoder/frame.rs:616`), once per
  `decode`, only with `extras.a7`, and it returns immediately when `period` is `None` (`frame.rs:517`).
- `SlotInput::period` is `Option<i64>`, documented as "`None` (a lone recording) leaves it untouched" — **unknown**, not
  "the current one" (`decoder/mod.rs:121-123`).
- `Decoder` offers `decode` and `decode_with` (`decoder/mod.rs:279`, `:287`); `AnyDecoder` mirrors them. The new pair
  follows that convention.

## 3. Principles

1. **The decoder infers the stage.** It holds the period's state and is handed the audio; which checkpoint a call is, is
   a function of those two. The caller names a period and hands audio, nothing else. (The samples are what settled this:
   every shape that made the caller name the stage also made it keep a copy of the decoder's own progress.)
2. **The audio length is the checkpoint.** Upstream's counts are what the decoder acts on; a prefix between them does
   nothing but return. A shorter prefix is **not** cheaper (trap 5) — what a caller buys by choosing one is *when* it
   can call, not the cost of the call.
3. **Every sequence is defined, and nothing errors.** Calls may be skipped or repeated. The worst case is today's
   whole-period decode.
4. **The state lives in the `Decoder`**, where `ft8_decode.f90` keeps its SAVE variables, as 0.13 already does for
   everything kept between periods.
5. **No clock, and no waiting.** Time is the sample count. `decode_prefix` never waits on a lock, clock, channel or I/O —
   but it is **synchronous and CPU-bound** and holds the calling thread until its work is done. "Non-blocking" would be
   wrong: a single browser worker cannot service messages during the call. What the library owes is a *bounded*
   occupancy, and `Budget` is that bound; what the caller owes itself is that capture does not share the thread (§4).

## 4. The entry point

```rust
impl Decoder<P> {
    /// Decode the period so far, keeping what this period has already found.
    ///
    /// Call it as audio arrives, with everything of the period received up to
    /// now and `slot.period` set. FT8 acts at 141 696 and 162 432 samples and
    /// at the period's full length; a call between those returns at once.
    /// The call whose audio is the whole period returns the period's complete
    /// row set and ends the sequence. Finish a period with this method:
    /// mixing in `decode` midway is unspecified (§4, "Mixing the two").
    pub fn decode_prefix(&mut self, slot: &SlotInput<'_>) -> SlotResult<P::Row>;

    /// [`Decoder::decode_prefix`], handing each row to `on_row` as it is found.
    pub fn decode_prefix_with(&mut self, slot: &SlotInput<'_>, on_row: OnRow<'_, P::Row>)
        -> SlotResult<P::Row>;
}
```

The same pair on `AnyDecoder`, returning `AnySlotResult`. A mode with no early decode acts only at the full length, so
the call is correct there and simply does nothing earlier — a mode-generic caller needs no capability check.

**The caller.** This is the whole of U2/U3:

```rust
buf.extend_from_slice(block);
let out = decoder.decode_prefix_with(&SlotInput::i16(&buf).period(period), &|row| {
    if row.detail.stage == Stage::Early { reply_this_period(row) } else { log(row) }
});
if buf.len() >= SLOT_SAMPLES { publish(out.rows); buf.clear(); period += 1; }
```

No progress tracking: the decoder knows what this period has had. U6 is the same loop over
`[141_696, 162_432, 172_800]`; U8 adds `.budget(…)` to the `SlotInput`, as `decode` already takes.

**Rows.** `on_row` sees each row once, when first found. The final call returns the period's **complete** set in
discovery order — the same list a whole-period `SicEarly` returns, which is what makes equivalence testable. A row is
never retracted (the dedup runs before delivery, #243), and the final search must take the earlier rows as `known` for
its own dedup, not only subtract them: an imperfectly-subtracted residual can re-decode the same message weakly, and a
harness that only compares texts will count it as new (#243-class).

`RowDetail::stage` is the one new public type, `Stage::{Early, Prepare, Final}`, **read on a row and never passed in**.
A caller answering a CQ needs to know a row arrived early enough to act on this period; that is #572's entire point.

**The splice boundary is the end of the cleaned region.** An earlier revision said the final call's fresh tail runs
"from `Early`'s prefix to the period's end", which would overwrite the samples the 162 432 call had just cleaned with
raw audio. Upstream splices at **B's** length, and so does this crate:

```rust
buf_c[..b_len].copy_from_slice(&buf_b[..b_len]);                  // cleaned head
buf_c[b_len..c_len].copy_from_slice(&audio_clean[b_len..c_len]);  // raw tail
```

(`decode.rs:1432-1433`.) So the state carries `cleaned_through`, a sample count:

- The 162 432-sample call subtracts the rows whose whole message fits inside that prefix — the `dt_fit_limit` gate,
  already computed from `b_len` rather than hard-coded (`decode.rs:1400-1401`) — and sets `cleaned_through`. Rows that
  did not fit stay unsubtracted, as upstream's `deferred` list does (`decode.rs:1406-1422`): subtracting them against a
  zeroed tail would fit the reference waveform to silence.
- A later prefix call with a **longer** prefix advances `cleaned_through` and subtracts what now fits. One at or below it
  is a no-op — splicing raw audio back over a cleaned region is exactly the bug above.
- The final call splices at `cleaned_through`, then subtracts every row still unsubtracted against the complete buffer.
  **A skipped middle call is therefore not a special case**: `cleaned_through` is 0, the whole buffer is raw, and every
  early row is subtracted at the end against complete audio, which is what checkpoint C already does for its deferred
  rows.

Two things this makes explicit: the first call cleans nothing (its residual is dropped, `decode.rs:1337-1339`, because
`ft8_decode.f90` reloads `dd=iwave` fresh at B), and "already subtracted" is tracked **per row**, not per region.

**The state**, per decoder and so per channel: the early rows, each flagged subtracted or not; the cleaned buffer (one
`i16` buffer of period length, 360 KB) and `cleaned_through`; the pinned gain (trap 1); the pinned strategy (trap 6);
the period index. Allocated at the first prefix call, never for a caller that does not use them.

**Division of labour.** `decode_prefix` holds its thread (principle 5), so capture and decode cannot be the same thread
unless capture is buffered ahead. Native (U2): capture on one thread, decode on another; `Decoder` is `Send`. One
browser worker (U3): either the worklet writes a `SharedArrayBuffer` ring the worker drains (needs COOP/COEP), or blocks
queue as worker messages and the worker is unresponsive for the call — viable, and the condition to size for is that
**the input buffer covers the longest call the caller allows**, which is what it set as `Budget`. This is a property of
a synchronous decoder, not of the stages: today's `decode` has it too. The stages change the arithmetic in the caller's
favour, because a bounded early call returns rows at 11.8 s instead of holding one longer call until the period is over.

**Mixing the two.** `decode` keeps its contract — one period, one shot, nothing kept. Calling it midway through a
prefix sequence is **unspecified**: the rows are whatever falls out, most likely the earlier ones again. Unspecified,
not undefined — no `unsafe` is involved and there is no soundness question; the worst case is duplicates the caller
must drop. Not designed for and not tested. The one invariant that holds regardless is trap 4: state for an abandoned
period never reaches a different one, because the next prefix call for a new period discards it.

### What this revision settles

**1. The last call is the whole period, and with no earlier call it is `decode`.** An earlier revision said "today's
whole-period decode (the flat fallback)", naming two different things as one: the flat path is reached only when
checkpoint A finds nothing or the buffer is shorter than A (§2), and taking it as the contract would lose the rows only
checkpoint C finds — on `qso3_busy`, `CQ DX DL8YHR JO41`.

**2. `period: None` keeps no state.** Not an error and not a new type: the existing convention already covers it.
`SlotInput::period`'s own doc calls `None` "a lone recording", and `remember()` returns immediately on it for a7
(`frame.rs:517`). A prefix call without a period is therefore a one-shot search of that prefix — the same thing a7
already does in the same situation. An earlier revision made it a runtime rejection, which the typed `SlotResult` could
not report, and then a `Stage::Early { period }` variant to make it a compile error, which forced a second `StageKind`
for the registry table. Inferring the stage removes the question entirely.

**3. a7 and a8 run on the final call only**, and `remember()` with them, because upstream runs both at `nzhsym == 50`
(§1). A period that ends without a final call remembers nothing, which is the `period: None` case again. Remembering at
an early call would store a partial row set under that period's index and quietly weaken a7 two periods later.

**4. No AP pass before the final call**, keeping checkpoint A's rule, already implemented and cited
(`decode.rs:1332-1333`).

**5. The strategy is pinned at the first prefix call.** It is read per call today (`frame.rs:604-608`), and letting it
change mid-period leaves the later calls subtracting rows a different search found — trap 6, and it would make
validation 1's equivalence claim ill-formed.

## 5. Traps

1. **f32 gain.** `frame.rs::f32_gain` scales by the audio's own RMS, which changes from one prefix to the next, so the
   same signal would reach the level-sensitive 16-bit engine at three levels. Pinned at the period's first prefix call
   and reused. `i16` audio has no gain (it is `jt9`'s `id2`).
2. **The zero tail is part of the algorithm.** Buffers stay full length with the content zeroed past the prefix, so
   `subtract_signal_lpf` has room for a frame near the edge. A short `SlotInput` never becomes a short buffer. The
   current code already works this way.
3. **Hash table timing.** Calls learned early are visible to the final search, as upstream learns at each decode.
   `on_row` resolves against the table as it stood when the call began; the returned rows also see what was learned
   earlier in the period.
4. **Period identity.** The decoder compares `slot.period` against its state; a mismatch discards. No stale state can
   reach the wrong period, whatever order the calls come in.
5. **A shorter prefix does not make a call cheaper.** `compute_spectrogram`'s time dimension is the hard-coded
   `NMAX = 15*12000` (`ft8/params.rs:16`), not `audio.len()`, and FT8's `build_fft_cache` (`ft8/downsample.rs:44`)
   transforms `FT8_CFG.fft1_size`, a hard-coded 192 000 (`ft8/downsample.rs:17`). So principle 2's freedom is about
   *when* a caller can call. A caller on a deadline picks a cheaper strategy, or a `Budget`.
6. **Strategy drift across calls.** Settled point 5: pinned at the first prefix call, like the gain.
7. **The `A_SAMPLES` floor is not a floor on prefix calls.** `audio.len() < A_SAMPLES` falls back to a flat pass today
   (§2) to stop `SicEarly` recursing into its own checkpoint structure. A prefix call below that length is still the
   period's first stage: it runs the strategy over that prefix and keeps the rows.

## 6. Validation

1. **Equivalence.** Prefix calls at 141 696 and 162 432, then the whole slot, give the same rows in the same order as
   `SicEarly` on the whole period. Covered by the FT8 goldens (`ft8_qso3_staged_sic_check`,
   `ft8_qso3_full_parity_recall`) and a test of its own; run under `fixed-point` too. **For `i16` input** — with `f32`
   the pinned gain makes the prefixes see a different level from a whole-period call, which validation 4 covers.
2. **Against upstream.** The rows at each count equal what an instrumented `jt9` reports at `nzhsym` 41, 47 and 50 on
   the same recording. Also where §1's second-hand a7/a8 citation gets checked against the Fortran.
3. **Precision.** This is where phantom decodes have come from (#243, #253): zero extra decodes against the 20-entry
   `qso3_busy` union, and an unchanged FT8 unexpected-decode count in `sweep-baseline.json`.
4. **Level.** `f32` input gives the same rows as the same audio as `i16` at the pinned gain.
5. **The splice boundary** (§4) — where the first revision was wrong, so the item to write first. No sample in
   `0..cleaned_through` differs from what the middle call left there: a **raw-audio** comparison, not a row comparison,
   because the row set can come out right while the buffer has been quietly re-dirtied and the bug then shows only as a
   sensitivity loss. Plus: middle call skipped; two middle calls with a growing prefix (no row subtracted twice); a
   middle call at or below `cleaned_through` leaving the buffer byte-identical.
6. **A final call with no earlier one is byte-for-byte `decode`** (settled point 1), on the goldens, including the rows
   only checkpoint C finds.
7. **a7 across the boundary** (settled point 3). With `a7` on: a period decoded as two prefix calls leaves the same a7
   state as one `decode`, and a period that ends without a final call leaves no entry for that index.
8. **Strategy pinning** (trap 6). A first call under one strategy and a final call under another gives the same rows as
   the final call under the first one's.
9. **`period: None`** (settled point 2) keeps no state: two such calls in a row are independent one-shots.
10. **Prefix-independence of the per-round fixed cost** (trap 5) — cost should not vary beyond noise across prefixes
    from a small fraction of the period to the whole of it.

## 7. Order of work

1. #573: `#[non_exhaustive]` on `RowDetail`, `SlotResult` and the rest.
2. Move `early_results`, `buf_b` and `deferred` out of the function into a state type — `cleaned_through` and the
   per-row subtracted flag included — so the whole-period `SicEarly` is three internal stages over that state. No
   behaviour change; validations 1, 5 and 6 prove it. #589's budget polls move with the loops.
3. `decode_prefix` / `decode_prefix_with` on `Decoder` and `AnyDecoder`, `RowDetail::stage`, and the pins (gain,
   strategy). Validations 2-4, 7-10.
4. **The boards are out of scope.** They do not go through `Decoder`, they run `embedded-shared`'s own prefix path,
   and that path has already diverged from the host's. It is maintained on its own terms; nothing here is meant to
   reach it, and this design does not count it as a consumer.

## 8. Open questions

- **`Depth::Fast`.** Upstream runs nothing before 50 at `ndepth==1`, so a prefix call would return nothing. Follow it,
  or run a single pass for a caller that asked for prefixes anyway?
- **FT4 looks like the tighter case and is not.** Half the period suggests less time to decide a reply; the deadline
  is what matters, and the two are the same within 0.1 s:

  | | signal | period | TX starts | slack to key-up | buffer searched |
  |---|---|---|---|---|---|
  | FT8 | 79 × 1920 = 12.64 s | 15.0 s | 0.5 s | **1.86 s** | 180 000 |
  | FT4 | 105 × 576 = 5.04 s | 7.5 s | 0.5 s | **1.96 s** | 90 000 |

  (`ft4/mod.rs:77,116,119,120`, `ft8/params.rs`; 103 active symbols plus 2 ramp.) FT4 has *more* slack than FT8 and
  half the buffer to search in it, which is the likeliest reason upstream never needed checkpoints there — the decode
  fits between the period's end and key-up.

  The mechanical obstacle is larger than the motivation anyway: `Ft4Strategy` is `SinglePass | SicRounds(n)` with **no
  `SicEarly`** (`decoder/frame.rs:106-110`). This design is a public entry onto machinery FT8 already has
  (`early_results`, `buf_b`, `deferred`); for FT4 it would mean writing the A/B/C algorithm and recording a deliberate
  divergence from WSJT-X, not adding an entry point.

  Not in this design. (The boards' own FT4 receiver does get cut by its budget — `BudgetReport::cut_at_score`'s doc is
  named after its `SlotOutcome::cut_at_score` — but the boards are out of scope per §7 step 4, so that is not an
  argument for this API.)
- **ft8md (#463)** has its own stages. If ported, it should reuse `Stage` rather than add a second vocabulary.

## 9. Considered and dropped

- **`Stage` as an input.** The caller naming the stage made it keep an index into the published counts, i.e. a copy of
  the decoder's own progress; the decoder can infer the stage from (state, audio length). `Stage` survives as an output
  on the row.
- **`prefix_points()` / `ProtocolMeta::early_points`.** Published so a caller would know where to call. The samples
  showed the mode-generic caller — its last justification — does not need it either, once a call with no work to do
  simply returns. The counts are in `decode_prefix`'s doc, and §1 says what grid they are on.
- **`StageKind`.** Existed only to type a `&'static` table that no longer exists.
- **`SlotCutter::open_prefix()`.** A caller already feeding a cutter knows the period boundary; this only saved it a
  buffer copy. Convenience, not contract.
- **`CAP_EARLY`** (a mode with no early decode is handled by the call doing nothing) and **`abandon_period`** (the next
  prefix call for a new period already discards).
- **Defining what happens when `decode` and `decode_prefix` are mixed.** Unspecified instead (§4).
- **A `Checkpoint::{A,B,C}` enum tied to upstream's counts.** Names a moment rather than what the stage does, so it
  cannot express ft8md's 46 or a caller's own counts.
- **A per-period session object holding `&mut Decoder`.** A borrow that lasts 15 s fits no threading model here.
- **One blocking call over an audio source.** Holds the decoder ~14 s and cannot work in a single-threaded worker
  without SharedArrayBuffer, Atomics and COOP/COEP. A native caller writes it in ten lines over §4.
- **Letting `decode` continue a prefix sequence.** It is the same silent contract change as folding prefixes into
  `decode` outright, narrowed; a separate method with its own doc is what makes the feature discoverable at all.

## 10. Follow-ups, deliberately not here

#572 asks how the C ABI would reach this, as a question. It is answerable once the method pair exists, and each half is
its own issue: driver support in `IqReceiver` (opt-in per channel, owned `Send` prefix copies of ~1.2 MB per
channel-slot), and the C ABI entries with `MfskRow`'s stage field (and Kotlin/Swift over them).

#572 also asks whether the boards could move onto the same API. The answer is no, and it is settled rather than
deferred: see §7 step 4.

## 12. As implemented

Step 2 is `ft8::decode::{Staged, StagedStep, staged_steps}`: the whole-period `SicEarly` is `staged_steps` over a fresh
`Staged` to `Final`, and every FT8 golden and staged test passed unchanged before step 3 was written. Step 3 is
`Decoder::decode_prefix` / `decode_prefix_with`, the same pair on `AnyDecoder`, `RowDetail::stage` and
`decoder::Stage`. Validations 1, 5, 6, 7, 8 and 9 are `tests/decoder_prefix.rs` (also run under `fixed-point`); 4 is
weakened to "the same messages" (below); 2 (against an instrumented `jt9`) and 10 (cost per prefix) are not done.

Where it departs from §4 and §5, each a simplification that keeps "the final call is `decode`":

1. **The final call is the whole period, 180 000 samples, not 172 800.** `decode` reads past C's count where checkpoint
   A finds nothing (the flat fallback runs over the whole buffer), so a final call at 172 800 would not equal it.
   Replaying `jt9`'s counts is therefore `[141_696, 162_432, 180_000]`.
2. **B runs once, at 162 432; a skipped B is done by the final call exactly as `decode` does it**, rather than every A
   row being subtracted against the complete audio. `cleaned_through` is B's count or nothing, and a later middle call
   does nothing, so "no row subtracted twice" and "a middle call at or below it leaves the buffer as it was" hold by
   construction. A prefix call whose budget runs out inside B keeps nothing of B, so the final call does it whole.
3. **A call below 141 696 samples returns nothing** (principle 2), not a first stage over that prefix (trap 7). The
   two were in conflict; the principle won because a caller calling on every block would otherwise trigger a search
   on every block below A.
4. **`Stage` has `Early` and `Final`.** `Prepare` would mark rows B finds, and B searches nothing. The enum is
   `#[non_exhaustive]`, so ft8md's stages (§8) can add variants.
5. **Equivalence is for audio exactly one period long.** A prefix is padded to 180 000 samples (trap 2), and a longer
   whole-period buffer (`qso3_busy.wav` is 180 101) reaches `build_fft_cache`'s 192 000-point window with samples no
   prefix had. The tests cut the recording to 180 000.
6. **`f32` (validation 4).** The pinned gain is measured on the first prefix; a whole-period decode measures it on the
   whole period, so the two are not byte-equal. The test asks for the same messages.
7. **`Depth::Fast` (§8's open question) follows upstream**: FT8's single pass, its SIC rounds and its sniper have no
   checkpoints, so they return nothing before the whole period, as `ndepth == 1` runs nothing before 50.
8. **The C ABI half of §10 is done** (`mfsk_decoder_decode_prefix_i16` / `_f32`, `MfskDecode::stage`, `MFSK_STAGE_*`;
   Kotlin and Swift `decodePrefix`). A prefix call refused for a short output buffer has run its stage, so the
   retry the ABI asks for is answered from the held rows. `IqReceiver` driver support followed in #600
   ([`IQ_PREFIX_DESIGN.md`](IQ_PREFIX_DESIGN.md)), which brings back a `prefix_points()` §9 had dropped, on the
   decoder, for the consumer that needs it.

The pinned settings are the period's search (strategy, sync, candidates, OSD, strictness) and the `f32` gain. The
callsign table is not taught before the final call: an early row is resolved against a copy of it, and the final
call resolves the whole set in decode order, which is what makes its text equal `decode`'s.

