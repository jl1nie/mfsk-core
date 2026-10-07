# `IqReceiver` prefix slots: early decode over wideband IQ

Status: **implemented 2026-10-08** (#600; the skimmer's adoption, §7, is a separate PR). Follow-up U4 of [`EARLY_DECODE_DESIGN.md`](EARLY_DECODE_DESIGN.md) §10: `Decoder::decode_prefix`
(#572, PR #599) exists, and `IqReceiver` still hands over whole slots only, so a wideband consumer (the skimmer) cannot
reach it.

## 1. What is missing

`decode_prefix` takes "the period so far" and infers the stage from its length: FT8 under `SicEarly` acts at 141 696
samples (checkpoint A, ~11.8 s, rows returned), at 162 432 (B, subtraction only) and at the whole period. A caller
feeding a WAV or a sound card cuts those prefixes itself. A caller on `IqReceiver` cannot: a channel's audio exists only
inside the receiver until the slot completes, and `tap_audio` gives the unscaled continuous stream, not the slot as the
receiver would hand it over (see §3, the gain).

## 2. Who decides when: the decoder, per channel

The points are not a property of the mode. They follow the decoder's settings: FT8 has them under `SicEarly` and not
under `SinglePass`, `SicRounds(n)`, `Depth::Fast` or the sniper; FT4, FST4, WSPR, JT9, JT65 and Q65 have none. Two FT8
channels on one stream can differ, and one channel's points change when its options do. So the receiver does not look
them up; each channel carries the points its own decoder asked for:

```rust
let points = decoders[&id].prefix_points();   // &'static [usize]: [141_696, 162_432] or []
rx.set_prefix_points(id, points);             // false: unknown channel
```

- **`Decoder::prefix_points()` / `AnyDecoder::prefix_points()`** (new): the prefix lengths, in 12 kHz samples, at which
  `decode_prefix` does work before the whole period, from the mode, depth and extras, like `delivery_is_exact`. Empty
  for a decoder that does nothing before the end. The whole period is not listed: it is always delivered.
- **`IqReceiver::set_prefix_points(id, &[usize]) -> bool`** (new): off (empty) by default, so a consumer that never
  calls it sees exactly today's slots. Takes effect from the next slot that opens; a slot already open keeps the points
  it opened with, so one period's prefixes are all cut under one setting.

`EARLY_DECODE_DESIGN.md` §9 dropped a `prefix_points()` because the mode-generic caller it was for did not need one: a
call with no work returns at once. The receiver is the consumer that does. Without points it would have to copy the
open slot (up to 720 kB of `f32`) on every block for calls that mostly return nothing, or pick its own cadence and
deliver A late by up to that cadence. The function lives on the decoder rather than on `ProtocolMeta` for the reason
above: it depends on settings, not on the mode.

## 3. The gain is pinned at the slot's first delivery

The receiver scales each slot to a fixed RMS (`TARGET_RMS`, 2 000 in `i16` units) measured over the slot it hands
over. With prefixes, the slot is handed over two or three times, and each prefix would be scaled by its own RMS: the
same signal at three levels, and checkpoint B's subtraction and the final search would run on audio at a different
level from the one A's rows were found at. `decode_prefix` already met this for `f32` input (its trap 1) and pins its
gain at the first prefix call; the receiver has to do the same, one level up.

So a channel with points measures the gain on the slot's first prefix and uses it for every later prefix and the final
slot. Since `TARGET_RMS == F32_TO_I16_RMS`, the decoder's own pinned gain on that first prefix is 1.0 (its 1e-3 snap),
and every later delivery reaches the 16-bit engine unscaled again: one level for the period, set by the receiver.

The cost is that a period whose level rises after 11.8 s is scaled by its first 11.8 s, as a WSJT-X operator's fixed
audio gain would scale it. A slot with no prefix delivered (points empty, a silent prefix, or a slot that opened after
its first point) is scaled over the whole slot as now: **no change for any existing consumer.**

## 4. Delivery: one ordered stream, `CompletedSlot` as "the slot so far"

A prefix goes out through the same `push_*(.., &mut Vec<CompletedSlot>)` as a whole slot, in the order the samples
arrive. Considered:

| | for | against |
|---|---|---|
| **Same `Vec<CompletedSlot>`; a prefix is a slot whose `audio` is shorter than the period** | no signature changes; order across a channel's deliveries is the arrival order; `slot.input()` is already what `decode_prefix` takes; `decode_prefix` infers the stage from the length too | the name says "completed" |
| A separate drain, `take_prefixes()`, like `take_audio` | names say what they hold | the caller has to interleave two queues; one large push can hold slot `j`'s prefix and its whole, and draining in the wrong order costs the early rows |
| A `SlotEvent { Prefix, Whole }` output enum | explicit | breaks all three `push_*` and every consumer, for a distinction the audio length already makes |
| A `mpsc::Sender` per channel | a slow decoder blocks nobody | the receiver takes no threads or queues today (`IqReceiver` is a pull model, by decision); the skimmer's lanes already are this, one level up |

Chosen: the first. `CompletedSlot` (already `#[non_exhaustive]`) gains `CompletedSlot::is_whole()`, comparing
`audio.len()` against the mode's period; its doc says a channel with prefix points also delivers prefixes. A consumer
that set points calls `decode_prefix` for **every** delivery of that channel, the whole slot included; that call is
`decode` when no prefix came before it (`decode_prefix`'s settled point 1).

A prefix is cut at **exactly** its point: `audio.len() == point`, not "the open slot's length when a block happened to
cross it". The rows then do not depend on the push size, and the tests can compare against a WAV cut at the same count.

## 5. What a prefix sequence promises across the receiver's own events

- **Retune, gap, clock step, pause.** The open slot is dropped today, and its whole is never delivered. Prefixes
  already delivered stay delivered: their rows were real decodes of the old audio. The decoder's prefix state for that
  period is discarded by the next period's first call (`decode_prefix`: "a call for another period discards"). No a7
  state is kept for a period without a final call. So a dropped slot costs exactly what it costs today, minus nothing.
- **A period index seen twice.** After `restart`, the next slot is found from the clock again. A clock stepped
  backwards can reopen a period already partly delivered, with other audio; the decoder would continue that period's
  sequence over it: B and the final would subtract A's rows from audio they were not found in. So on a channel with
  points, a slot whose index is at or below the last index that channel delivered anything for is **not delivered at
  all**, prefix or whole. A clock stepped backwards then costs that one slot, which is rarer and cheaper than a period
  spliced from two recordings. A channel without points delivers it as today. The test is a clock stepped back across
  an open slot with prefixes.
- **Channel added mid-slot.** The cutter does not open a slot in the past beyond its overlap, so the first slot is the
  next whole one, with its prefixes.

## 6. Cost

A prefix is an owned copy (`Send`, like the whole slot): at most two extra copies per FT8 slot per channel, 567 kB and
650 kB of `f32` on top of the 720 kB whole. Allocation and copy are ~0.1 ms; the decode they feed is seconds.
`Arc`-shared growing buffers would save the copies and add `unsafe` or a lock to the cutter's hot path; not worth it
here.

## 7. Consumer side (the skimmer), for the record

The skimmer spreads a channel's slots over lanes, each with its own decoder. A period's prefixes and its whole must go
to **one** lane, since the prefix state lives in that lane's decoder: the lane is picked at the period's first
delivery and remembered by `(channel, period)` until its whole goes out. A lane busy with the previous period's final
delays A's rows; the least-pending pick already avoids that when another lane is free. `prefix_points()` is asked again
whenever the options change and handed to `set_prefix_points`. A row found at A shows at ~12 s with `Stage::Early`; the
final call's matching row (`RowDetail::delivery`) updates it in place, as the skimmer's resolved rows already do. Its
per-slot time budget applies per call. A separate PR.

## 8. Not here

- **The C ABI's `mfsk_iq_*`** decodes inside the handle with one decoder per channel, so it could cut prefixes from
  its own decoder's points without a new entry. Its own issue (U5 with `mfsk_stream_push_*`).
- **`SlotCutter`** stays whole-slot in its public API. The receiver reads the open slot through a crate-private
  accessor; the C audio stream does not need one until U5.

## 9. Validation

1. **Nothing changes without points.** Every existing `iq_receiver.rs` test passes unchanged; slots and levels are
   byte-equal.
2. **Prefixes at the points.** An FT8 channel with `[141_696, 162_432]` delivers, per slot, two prefixes of exactly
   those lengths and then the whole, in that order, for push sizes of 1, 8 192 and a whole slot at once.
3. **Pinned level.** A prefix's samples are the first samples of that slot's whole, bit for bit.
4. **Same rows.** Over the two-channel scene, `decode_prefix` on every delivery gives the same final messages as
   `decode` on the whole slots, and checkpoint A's rows arrive with the first prefix.
5. **Points follow the setting.** `prefix_points()` is `[A, B]` for FT8 `SicEarly` (its strategy at `Depth::Normal` and `Deep`), `[]`
   at `Depth::Fast`, under `SinglePass` and the sniper, and `[]` for every other mode.
6. **A period is not reopened** on a channel with points after a backwards clock step (§5).
