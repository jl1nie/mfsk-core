# Early decode for live use — design (#572)

Status: **draft for review, revised 2026-10-06.** Nothing here is implemented. Target: 0.14; breaking changes are
allowed, but #573 (`#[non_exhaustive]`) lands first.

This is the second revision. The first folded #587 (WebFT8's two-phase decode, and a budget overshooting its deadline)
into this design. #587 was then closed by **#589**, which polls the budget after checkpoint A and before each row's
subtraction in the B and C loops, with no new API — and WebFT8 measured the fix under `wasm32` and kept its own
two-phase arrangement. So this design is back to #572 alone: an early-decode entry point, judged on whether it delivers
checkpoint A's rows at 11.8 s, not on anything #587 needed.

## 0. Goal and use cases

FT8 rows should reach a live caller at about 11.8 s into the period, as WSJT-X delivers them, and the caller should decide
how much of the machinery it wants. The design has to serve every caller below without a special path for any of them:

| | caller | how audio arrives | threads | what it wants |
|---|---|---|---|---|
| U1 | offline decode of a recording | the whole period at once | any | today's `decode`, unchanged |
| U2 | desktop GUI, native | pushed in blocks | one per channel | rows at 11.8 s, the rest at the end |
| U3 | browser PWA | AudioWorklet blocks into one worker | one, cannot block | same as U2, never blocking |
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
- **No AP pass runs while `nzhsym < 50`** (`npasses=5`). mfsk-core already follows this, with the citation at
  `ft8/decode.rs:1332-1333`.
- **a7 and a8 run only at `nzhsym == 50`**, after the candidate loop, with AP on. Sourced here from this crate's own
  port header (`ft8/list_decode.rs:3-5`, which cites `ft8_decode.f90:250-305` / `ft8_a7.f90` / `ft8_a8d.f90`) rather
  than from the Fortran directly — the WSJT-X tree is not in this container, so the line range is quoted at second hand
  and should be re-read against it before the implementation relies on it.
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
- **Two internal flat fallbacks already exist, and §4 depends on both being understood as internal:**
  - `audio.len() < A_SAMPLES` → a flat `SicRounds(3)` pass over the whole (short) buffer. It must be the flat inner and
    not `.sic_early()`, which dispatches back into this function and would recurse (`decode.rs:1275-1297`, guarded by
    `sic_early_with_ap_silence_shape`).
  - checkpoint A found nothing (`early_results.is_empty()`) → the same flat pass over the full audio, because
    upstream's B search and C's "cleaned head + raw tail" step are both gated on `ndec_early >= 1`
    (`decode.rs:1342-1374`).
- The budget is polled after checkpoint A and before each row's subtraction in the B and C loops (**#589**). The
  subtraction itself is unchanged, following `subtractft8.f90`'s `lrefinedt` path, and
  `staged_sic_skips_b_and_c_when_a_spends_the_budget` (`ft8/decode.rs:2045`) guards the fix.
- a7's cross-period state is written by `remember(state, slot.period, results)` (`decoder/frame.rs:616`), once per
  `decode` call, only when `extras.a7` is set, and it returns immediately when `slot.period` is `None`
  (`frame.rs:516-520`). It keeps the two latest periods, replacing any entry for the same index.
- `SlotInput` is one whole period and is already `#[non_exhaustive]`. Its `period` is `Option<i64>`, documented as
  "`None` (a lone recording) leaves it untouched" — **unknown**, not "the current one" (`decoder/mod.rs:118-128`).
- `IqReceiver` and the C stream cut whole slots with `SlotCutter`. The boards run their own prefix path
  (`embedded-shared`).

## 3. Principles

1. **One primitive, many drivers.** The decoder gets one new entry point. Everything that decides *when* to call it is a
   separate, optional layer, so a caller can take the layer that fits or none at all (U7).
2. **The audio length is the checkpoint.** The caller says *which kind* of stage to run and passes the period so far. How
   many samples that is, is up to the caller. Upstream's counts are a published default, not a rule (the ft8md 46, U7).
   This does **not** mean a shorter prefix is cheaper to process — see trap 6:
   `compute_spectrogram`/`coarse_sync`/`build_fft_cache` cost is fixed regardless of prefix length. What a caller
   actually controls by choosing a shorter prefix is *when* it can call (how much real audio exists yet), not the cost of
   any one call.
3. **Every sequence is valid.** Calls may be skipped, repeated or out of order. Each case has a defined result, and the
   worst case is today's whole-period decode. Nothing errors except a mode that has no stages (U8), and `Early`/`Prepare`
   without a period index (§4).
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

**`Early`'s prefix does not have to be upstream's.** Principle 2 means a caller may pass any prefix to `Early`,
including one shorter than checkpoint A, and including the whole period. Whatever it covers is what `Prepare` and
`Final` are defined against, so there is no separate case to code: `Final`'s fresh tail is "from `Early`'s prefix to the
period's end", which is empty when `Early` already reached the end.

Note the `A_SAMPLES` floor in §2 does **not** apply to `Early`. That floor exists only to stop `SicEarly` recursing into
its own internal checkpoint structure on a buffer too short to have one. `Early` *is* the checkpoint, so it has nothing
to recurse into: on a short prefix it runs its strategy over that prefix and keeps the rows as the period's early rows.

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

**Every sequence.** `p` is `slot.period`.

| call | state before | result |
|---|---|---|
| `Early` | any | Discard any earlier state and start period `p`. Search the prefix and keep its rows. A second `Early` in the same period starts over: its rows are deduplicated against those already delivered. |
| `Prepare` | `Early` of `p` | Subtract the early rows whose frame fits this prefix. Rows: none. |
| `Prepare` | none, or another period | No-op. |
| `Final` | `Early` of `p` (± `Prepare`) | Cleaned head (through `Prepare`, if called) plus a fresh tail from `Early`'s prefix to the period's end — upstream's C when that prefix is 141 696; empty when `Early` already covered the period. Subtract whatever is left, search, then run a7/a8. Clear the state. |
| `Final` | none, or another period | **Exactly `decode(slot)` as it behaves today**: the full staged path, which internally falls back to a flat pass only on its own two conditions (§2). Clear the state. |
| `Early` or `Prepare` | — | `slot.period` is `None` → rejected, nothing decoded, no state kept. |

### What this revision settles

Five things the previous revision left open. Each is resolved against the source rather than by preference.

**1. `Final` with no prior stage is `decode(slot)`, not the flat fallback.** The previous revision's table said "today's
whole-period decode (the flat fallback `SicEarly` already uses when A found nothing)", which named two different things
as one. The flat fallback is reached only when checkpoint A finds nothing (`decode.rs:1342`) or the buffer is shorter
than A (`:1275`); it is an *internal branch* of the staged path, not a synonym for it. Taking the flat path as the
contract would lose rows that only checkpoint C finds — on `qso3_busy` a flat `SicRounds(3)` without the subtraction
step misses `CQ DX DL8YHR JO41`. So `Final` with no state runs the whole staged decode, the flat branches included
where they already apply, and `decode(slot)` needs no change at all.

**2. `Early` and `Prepare` require `slot.period`.** `SlotInput::period` is `Option<i64>` and `None` already means
*unknown* — "a lone recording" in its own doc comment (`decoder/mod.rs:121-123`) — not "the current period". Stage state
spans calls, so principle 3's "a stale state can never reach the wrong period" is only enforceable with an identity to
compare. `Early`/`Prepare` with `period: None` are therefore rejected (one more `Unsupported`-shaped outcome; nothing is
decoded and no state is kept). `Final` keeps accepting `None`, so U1 and U6 are untouched.
*Considered instead:* fingerprinting the state from the prefix. Rejected — it costs a hash over 141 696 samples on every
call and still cannot separate two periods whose audio is identical, where requiring the index the caller already has
costs one line.

**3. a7 and a8 run in `Final`, and only there.** Upstream runs both at `nzhsym == 50` with AP on
(`ft8/list_decode.rs:3-5`, citing `ft8_decode.f90:250-305`; second-hand, see §1). `Early` is 41 and `Prepare` is 47, so
neither runs them, and `Prepare` searches nothing in any case. `remember()` likewise runs only at `Final`: a7 needs the
period's *complete* row set, and only `Final` has it. A period that ends without `Final` therefore remembers nothing —
the same state a7 is in for a lone recording today, since `remember` already returns early on `period: None`
(`frame.rs:517`). Calling it at `Early` would store a partial set under that period's index and quietly weaken a7 two
periods later.

**4. `Early` runs no AP pass.** It keeps checkpoint A's rule, which this crate already implements and cites:
`base_pass.without_ap()`, with the comment "`ft8_decode.f90` runs no AP pass while `nzhsym < 50` (npasses=5)"
(`decode.rs:1332-1333`). `Final` runs with AP, as checkpoint C does. `Prepare` does not search, so it does not arise.

**5. The strategy is pinned at `Early`.** Today the strategy is read on every `decode` call, from that call's depth and
`Ft8Extras::tuning` (`frame.rs:604-608`). For stages that is not safe: `Early` under `SinglePass` leaves a different
residual than under `SicRounds(3)`, and `Prepare`/`Final` are defined relative to *what `Early` actually removed*. So
the strategy is captured into the stage state at `Early`, beside the pinned gain, and a later `Prepare`/`Final` in the
same period uses the pinned one; a caller that passes a different strategy mid-period is ignored rather than obeyed, and
the pinning is a trap in its own right (trap 7). Without this, validation 1's equivalence claim is not even well-formed.

**The options a caller has.**
- **Strategy.** Stages exist for `Ft8Strategy::SicEarly`, the `Normal`/`Deep` default. With `SinglePass` or `SicRounds(n)`,
  `Early` searches the prefix with that strategy, `Prepare` is a no-op, and `Final` runs the strategy on the residual.
  This is the real lever for a tight budget or a busy band (trap 6): `SinglePass`/`SicRounds(1)` bound how many times a
  real signal gets redecoded (1-2 rounds) where `SicEarly`/`SicRounds(3)` allows up to 6 (checkpoints A and C, 3 rounds
  each). A caller on a hard deadline should pick a cheaper strategy, not a shorter prefix (principle 2). Whichever it
  picks is pinned for the period (settled point 5).
- **Depth.** With `Depth::Fast` (`ndepth==1`), `Early` and `Prepare` return nothing, as upstream does. A caller that wants
  early rows anyway sets a strategy in `Ft8Extras` explicitly. That is a library extension, written down as one.
- **Budget.** Each call takes `slot.budget`, and gets the poll points #589 added: after `Early`'s search, and before each
  row's subtraction in `Prepare` and `Final`. Those are the same loops, so the stages inherit the fix rather than needing
  their own.
- **Abandoning a period.** `Decoder::abandon_period()` is for a gap or a retune. Nothing needs it, because the next
  `Early` resets anyway, but it frees the buffers.

**Modes without stages.** `AnyDecoder::decode_stage` returns `Unsupported` for `Early` and `Prepare`, and `Final` works
for every mode. `registry::caps` gains `CAP_EARLY`, so a driver asks instead of guessing. FT4 has no early decode
upstream. JT9, JT65, Q65, WSPR and FST4 decode at the period's end.

**What the state holds.** Per decoder, so per channel: the early rows; B's cleaned head (one `i16` buffer of period
length, 360 KB); the deferred rows; the pinned gain (trap 1); the pinned strategy (trap 7); the period index. Allocated
at the first `Early`, never for a caller that does not use stages.

## 5. Layer 2: drivers (optional; take one, or none)

**5.1 What the core publishes.**
- `ProtocolMeta::early_points: &'static [(u32, Stage)]` holds upstream's default. FT8 is `[(141_696, Early), (162_432,
  Prepare)]`, and every other mode has an empty slice. `Final` is the slot length, which the registry already gives.
  `ProtocolMeta` is not `#[non_exhaustive]` today, so this field is one of the reasons it is on #573's list A.
- `SlotCutter::open_prefix() -> Option<(period, &[T])>` lets a caller look at the slot being cut. The cutter holds that
  buffer already, so nothing new is stored. It yields the period index too, which `Early` now requires (settled point 2).

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

**5.4 Your own loop (U2, U3, U7).** A caller can skip all of the above: push blocks into its own buffer, and when the
buffer passes `early_points`, call `decode_stage` with the period index it is already tracking. Each call returns when
its work is done and nothing blocks, which is what U3's single worker needs.

**5.5 `jt9` replay (U6).** Call `decode_stage` on `&audio[..141_696]`, `&audio[..162_432]` and the whole slot, numbering
the period. Alternatively, feed the recording through the stream with no clock set: the grid free-runs, and the stage
points land on exactly those counts.

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
4. **Period identity.** Covered by the rule table in §4 and settled point 2: the index is required for `Early`/`Prepare`,
   so a stale state can never reach the wrong period.
5. **Wall clock.** Upstream's `tseq` bail-outs are not ported, because this crate has no clock. `Budget` serves that
   purpose.
6. **A shorter prefix does not make a call cheaper.** `compute_spectrogram`/`coarse_sync`/`build_fft_cache` cost is flat
   regardless of prefix length — `compute_spectrogram`'s time dimension is the hard-coded `NMAX = 15*12000`
   (`ft8/params.rs:16`), not `audio.len()`, and FT8's `build_fft_cache` (`ft8/downsample.rs:44`) transforms
   `FT8_CFG.fft1_size`, a hard-coded 192 000 (`ft8/downsample.rs:17`), the same way. This matters to #572 because it means principle 2's freedom is about *when*
   a caller can call, not about buying a cheaper call; a caller on a deadline picks a cheaper strategy instead (§4,
   "Strategy"). The budget gap that used to be described here — the unpolled subtraction loops between checkpoints — was
   fixed in **#589**; `Prepare` and `Final` reuse those same loops and so must keep the same per-row poll.
7. **Strategy drift across stages.** Settled point 5: the strategy is pinned at `Early` and reused, like the gain. A
   period whose stages ran under different strategies has no defined residual, so the equivalence in validation 1 would
   not be testable and the subtraction in `Prepare`/`Final` would be removing rows that a different search found.

## 7. Validation

1. **Equivalence.** `Early` on the A prefix, `Prepare` on the B prefix, then `Final` on the whole slot gives the same
   rows, in the same order, as `SicEarly` on the whole period. Covered by the FT8 goldens
   (`ft8_qso3_staged_sic_check`, `ft8_qso3_full_parity_recall`) and a test of its own. Run it under `fixed-point` too.
   **For `i16` input**: with `f32` the pinned gain (trap 1) makes the stages see a different level from a whole-period
   call, which is what validation 4 covers instead.
2. **Against upstream.** The rows at each stage equal what an instrumented `jt9` reports at `nzhsym` 41, 47 and 50 on the
   same recording. This is also where §1's second-hand a7/a8 citation gets checked against the Fortran.
3. **Precision.** This is the path where phantom decodes have come from (#243, #253). Required: zero extra decodes
   against the 20-entry `qso3_busy` union, and an unchanged FT8 unexpected-decode count in `sweep-baseline.json`.
4. **Level.** `f32` input gives the same rows as the same audio as `i16` at the pinned gain.
5. **Every row of the §4 table**, as a test. Also: a prefix shorter than A (which must stage, not hit the `A_SAMPLES`
   flat floor — §4), `Early` called twice, a period change between stages, and `Early`/`Prepare` with `period: None`
   being rejected without touching the state.
6. **`Final` with no prior stage is byte-for-byte `decode(slot)`** (settled point 1) — the same rows in the same order
   on the goldens, including the rows only checkpoint C finds. A regression here would silently downgrade every U1
   caller to the flat path.
7. **a7 across a stage boundary** (settled point 3). With `a7` on: a period decoded as `Early` + `Final` leaves the same
   a7 state as the same period decoded in one `decode` call, and a period that ends after `Early` with no `Final`
   leaves no entry for that index at all.
8. **Strategy pinning** (trap 7). `Early` under one strategy followed by `Final` under another gives the same rows as
   `Final` under `Early`'s strategy — the second one is ignored, not obeyed.
9. **Drivers.** A recording through `IqReceiver` and through the C stream yields stage points at the published counts
   and the same rows as test 1.
10. **Prefix-independence of the per-round fixed cost** (trap 6). A round's `compute_spectrogram`/`coarse_sync` cost
    should not vary (beyond noise) across prefixes from a small fraction of the period to the whole period. A
    regression would show as cost growing with prefix length, which would also silently invalidate principle 2.

## 8. Order of work

1. #573: `#[non_exhaustive]` on `RowDetail`, `ProtocolMeta`, `SlotResult` and the rest.
2. Move `early_results`, `buf_b` and `deferred` out of the function into a state type, so `SicEarly` is three stage calls
   in a row. No behaviour change; validation 1 and 6 prove it. #589's budget polls move with the loops.
3. `decode_stage`, `Stage`, `RowDetail::stage`, `CAP_EARLY`, `abandon_period`, and the pins (gain, strategy).
   Validations 2-8.
4. `early_points` and `open_prefix`. Then `IqReceiver::set_early` and stage points. Then the C entries, then Kotlin and
   Swift. Validation 9.
5. Moving the boards' prefix path onto the same stages is a separate decision. The boards have no `Decoder` today.

## 9. Open questions

- **Should `Prepare` be public?** It produces no rows. In its favour: it matches `jt9`, and it lets a caller use the
  1.7 s between A and C. Against it: one more case in the table. Recommended: keep it, so drivers can call it and a
  caller can skip it.
- **`Early` with `Depth::Fast`.** Should it follow upstream and do nothing, as proposed, or should it run a single
  pass? The first keeps parity; the second is friendlier to U8.
- **How `Early`/`Prepare` report a missing period index.** Settled point 2 rejects the call; whether that is
  `Unsupported`, a distinct error, or an empty `SlotResult` with a flag depends on what #573 settles for these types.
- **Should stages be offered for FT4?** Upstream has none, so it would be a library extension. Not in this design.
- **ft8md (#463)** has its own stages. If it is ported, it should reuse `Stage` rather than add a second vocabulary.

## 10. Considered and dropped

- **A `Checkpoint::{A, B, C}` enum tied to upstream's counts.** It names a moment rather than what the stage does, so it
  cannot express ft8md's 46 or a caller's own counts. `Stage` plus "the audio length is the checkpoint" can.
- **A per-period session object holding `&mut Decoder`.** A borrow that lasts 15 s fits no threading model here.
  Keeping the state in the `Decoder` gives the same result with no borrow.
- **One blocking call over an audio source.** It holds the decoder for about 14 s and cannot work in a single-threaded
  worker (U3) without SharedArrayBuffer, Atomics and COOP/COEP. A native caller can write it in ten lines over §5.4, so
  it is not needed in the library.
- **A resumable decode job polled in steps.** The decode is CPU-bound and already takes a `Budget`, which #589 now polls
  at the row granularity the subtraction loops needed.
- **A stage field on `SlotInput`.** It would make `decode` mode-dependent in a way it cannot report. The convention in
  `AnyDecoder` is that an option a mode lacks is `Unsupported`.
- **Fingerprinting the stage state instead of requiring `period`.** Settled point 2.
