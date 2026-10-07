# mfsk-core — Streaming Decode Interface

> **日本語版:** [STREAMING.ja.md](STREAMING.ja.md)

This document covers the **streaming delivery** surface of mfsk-core:
`Decoder::decode_with` and its row callback, what its delivery contract
guarantees, **why it is a plain synchronous callback rather than an
`async fn` / `Future` / channel-based API**, and a complete worked
example of bridging it into a Tokio async client.

For the wider library surface (the `Decoder` model, trait hierarchy, DSP
primitives) see [LIBRARY.md](LIBRARY.md), and [BINDINGS.md](BINDINGS.md) for
the C ABI — this doc drills into one section of LIBRARY.md (§2.4, "Streaming
delivery") and adds the async-bridge example.

---

## 1. What "streaming" means here

A decode operates on **one complete audio slot already in hand** — a
15 s FT8 buffer, a 110.6 s WSPR buffer, and so on. It is not an
open-ended socket read. "Streaming" therefore does not mean *feeding
samples in incrementally*; the whole slot is passed by reference. It
means the opposite direction: **results flow out incrementally**, one
callback per accepted message, *as the decoder finds them*, instead of
being handed back only as a single `SlotResult` when the entire slot has
finished decoding.

The batch API and the streaming API are the **same decoder**. Streaming
is purely additive:

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, Row, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;

let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((100.0, 3000.0)));
let slot = SlotInput::i16(&audio);

// Batch: get everything at the end.
let result = decoder.decode(&slot);
for row in &result.rows { /* ... */ }

// Streaming: same decode, plus a callback fired per row along the
// way. `result.rows` still holds the full batch afterwards.
let on_row = |row: &Row<DecodeResult>| {
    // fired as each candidate is accepted; row.decoded is the cross-mode row
};
let result = decoder.decode_with(&slot, &on_row);
```

A caller that never uses `decode_with` sees zero behavioural difference
from `decode`.

### Why stream at all?

A single slot's decode is not instantaneous — on a busy FT8 band a
full-depth wide-band search with OSD escalation can take a meaningful
fraction of a second on a host, and much longer on an embedded target.
Streaming lets a UI paint the first (typically strongest) decode the
moment it lands rather than staring at a spinner until the last, most
expensive candidate has been chased through deep OSD. It also lets a
downstream stage (logging, QSO-state machine, spotting upload) start
work on early results while later ones are still being computed.

---

## 2. Per-protocol entry points

The row type is `Row<R>`: the cross-mode `decoded: Decoded` (text, frequency,
dt, SNR, protocol), a `detail: RowDetail`, and the mode's own result `native: R`
— `DecodeResult` (FT8/FT4/FST4), `Q65Result`, `WsprResult`, `Jt65Result`,
`Jt9Result`, which are structurally distinct. So the callback type is one
generic shape, `&(dyn Fn(&Row<R>) + Sync)` (`decoder::OnRow`), with the mode's
`R` filled in.

| Protocol(s)          | Entry point                                       | Callback type                          |
|----------------------|---------------------------------------------------|----------------------------------------|
| FT8 / FT4 / FST4     | `Decoder<P>::decode_with(&slot, on_row)`          | `&(dyn Fn(&Row<DecodeResult>) + Sync)` |
| Q65                  | `Decoder<Q65…>::decode_with`                      | `&(dyn Fn(&Row<Q65Result>) + Sync)`    |
| WSPR                 | `Decoder<Wspr>::decode_with`                      | `&(dyn Fn(&Row<WsprResult>) + Sync)`   |
| JT65                 | `Decoder<Jt65>::decode_with`                      | `&(dyn Fn(&Row<Jt65Result>) + Sync)`   |
| JT9                  | `Decoder<Jt9>::decode_with`                       | `&(dyn Fn(&Row<Jt9Result>) + Sync)`    |
| any, chosen at run time | `AnyDecoder::decode_with`                      | `&(dyn Fn(&Decoded, &RowDetail) + Sync)` |
| FT8 (`ft8::decode_block`) | `ft8::decode_block::decode_block_streaming`  | `&mut dyn FnMut(&DecodeResult)`        |
| JTTY                 | `jtty::rx::Stream::push(samples, &mut cb)`        | `&mut dyn FnMut(MessageUpdate)`        |

Notes:

- **Every `Decoder`** has `decode_with` beside `decode`; the returned
  `SlotResult` still holds the full batch.
- **A row is resolved when it is delivered.** The callback sees the row's text
  resolved against the decoder's callsign hash table as it stood when the
  period began; the rows `decode_with` returns also see calls learned earlier
  in the same period (hashes are learned after the candidate loop, in decode
  order). A `<...>` in a streamed row can therefore read resolved in the
  returned one. To pair a streamed row with a returned one, use
  `RowDetail::delivery` (#592): a streamed row carries its position among the
  period's deliveries, and a returned row the position of the delivery it was
  (the first with the same bits, frequency and time, compared exactly), so the
  pairing needs no key and no rounding. `None` on a returned row the callback
  never saw, and in a build without `std`. To compare rows across decoders or
  periods instead, compare the message bits, not the text: `row.native.message77()`
  on a typed row, and `RowDetail::info` on any row (every mode fills it). One
  message at two frequencies has one key, so add the frequency.
  A candidate whose payload does not unpack is delivered by neither.
- **Averaged Q65** (`averaging` on, with `SlotInput::period`) yields at most one
  result a period, so its callback fires once per period.
- **WSPR/JT65/JT9 used to have no builder**, and each grew a
  `decode_scan_streaming` *sibling* free function instead — the
  pattern that, one axis at a time, left those three modes with 32
  public `decode_*` functions. Issue #403 replaced them with per-mode
  request types and 0.13 with `Decoder<P>`, where streaming is one method
  like everywhere else.
- **`ft8::decode_block::decode_block_streaming`** takes `&mut dyn
  FnMut` rather than `&dyn Fn + Sync` in *both* its feature-gated
  variants: the embedded (`not(fft-rustfft)`) single-pass pipeline is
  strictly sequential (no rayon on no_std), and the host (`fft-rustfft`)
  multipass/subtract driver (`decode_block_multipass`) is also always
  sequential — it has no rayon inside it, unlike the single-pass and sniper
  strategies — so a `FnMut` closure that mutates
  captured state is safe on both, and the `Sync` bound is unnecessary
  either way. Before issue #243 the host variant couldn't safely expose
  a callback here: its `xsnr2` SNR validity gate (`ft8b.f90:483`) ran
  as a post-hoc batch *after* subtracting had finished, which could
  drop or mutate a result *after* it would already have streamed, with
  no revise/retract event this callback provides. The gate now runs
  inline, immediately per candidate — right after that candidate's
  signal is subtracted and before it is accepted — so both feature
  variants share the same §3a exact-match contract below. See
  `tests/ft8_decode_block_streaming_host.rs` for the host-side
  exact-match verification (mirrors
  `tests/ft8_decode_block_streaming.rs`'s embedded one).

---

## 3. Delivery contract

There are exactly **two** contracts, decided by whether the strategy
runs sequentially or under `rayon` (`feature = "parallel"`). The
authoritative statement is the doc comment on the engine's row callback
(`DecodeRequest::on_result`, now crate-private; `Decoder::decode_with` wraps
it); summarised here. Which strategy a decoder runs is `Depth`'s call
([LIBRARY.md](LIBRARY.md) §2.2), so the contract follows the depth:

### 3a. Sequential — exact match

`cb` fires **exactly once per row that ends up in the returned
`SlotResult`, in the same order**. Zero divergence between what you stream and
what the batch return holds (apart from the hash resolution in §2: compare
message bits, not text).

Covers: FT8's `SicRounds(n)` and `SicEarly` and FT4's `SicRounds(n)` — which
is what `Depth::Normal` and `Depth::Deep` run for FT8 and FT4, and the
default block's depth is `Deep`;
`ft8::decode_block::decode_block_streaming` (both the embedded and host
`fft-rustfft` variants, since issue #243); the JT65 and JT9 decoders; Q65's
scan. Q65 with `averaging` is a variant of the sequential shape: it fires
once **per period** that yields an accepted decode (its natural streaming unit
for multi-period EME / ionoscatter averaging), not once per candidate.

### 3b. Parallel — completion order, possible transient duplicate

`cb` fires from **whichever thread decoded that candidate, in
completion order** (not candidate-exploration order), and **before**
the final cross-candidate dedup pass. On the rare occasion two sync
candidates converge on the same message, `cb` may fire for both even
though only one survives into the returned rows. Callers wanting exact
parity should dedup by the message bits on their side — `.message77()` on a
typed row, `RowDetail::info` through `AnyDecoder` or the C ABI's row — the
same key the crate's own dedup uses.

`AnyDecoder::delivery_is_exact()` (and `Decoder::delivery_is_exact()`, C's
`mfsk_decoder_delivery_is_exact`, Kotlin's and Swift's `deliveryIsExact`) says
which contract the current mode, depth and extras run: `true` is §3a, `false`
is "§3b, not promised" — a caller that gets `true` can skip its guard. Ask again
after changing the depth or the extras.

Covers: the single-pass strategies (`SinglePass`, which is FT4's `Fast`
depth, and FST4's only strategy) and FT8's `sniper` mode; WSPR (its pass-1 and
pass-2 candidate loops run under `rayon::par_iter()`).

**This is why the callback must be `Sync`** — it may be called concurrently
from multiple rayon worker threads.

### Does delivery order favour strong signals?

Tends to, on both families: `coarse_sync` returns candidates sorted by
Costas sync score descending, and both the sequential loop and the
parallel sweep process that list in-order, so higher-scored (typically
stronger, since sync score correlates with SNR) candidates tend to
surface first. But it is a **correlation, not a guarantee** — sync
score is a pre-demod correlation-power measurement, not a predictor of
post-demod BP/OSD cost, so a highly-scored candidate can still need
full OSD escalation while a lower-scored one converges in one BP pass.
On the sequential strategies specifically, a candidate ahead in the
list that needs deep OSD blocks every candidate behind it (single
thread); the parallel strategies have no such head-of-line blocking.

### Audited: no "revoke-less retract" gap exists anywhere else (2026-08-09)

A *revoke-less retract* is a specific failure mode distinct from
either contract above: `cb` fires for a candidate that later, via some
**separate** post-processing step downstream of the firing point,
never makes it into the returned `Vec` — not because of the
documented §3b duplicate-completion-order behaviour, but because a
gate or filter ran *after* the callback had already committed to
delivering that result, with no revise/retract event to tell the
caller so. This is worse than either documented contract: it silently
breaks even §3b's weaker guarantee (every batch result fires *at
least* once — a revoke-less retract can make the delivered set and
the batch `Vec` disagree in the other direction too, once a caller
starts relying on "streamed implies real").

This shape bit real code twice, both on FT8/FT4/FST4 (the only
protocols that had a `.known(...)` cross-phase-dedup builder method —
WSPR/Q65/JT65/JT9 don't have the concept at all, so they were never
exposed to it). 0.13 removed `.known()` from the public API (its job is
decoder state now), so that combination no longer exists for a `Decoder`
caller; the engine's request keeps it behind `internal-testing`, and the
fixes are what the table below audits:

1. **FT8 host multipass driver** (issue #243) — the `xsnr2` SNR
   validity gate used to run as a batch *after* an entire pass (or,
   in an early cut of the fix, after the whole 3-pass loop) had
   already fired `on_result` for its candidates. Fixed by moving the
   gate inline, immediately per candidate, before it's ever accepted.
2. **FT8's `SicEarly`, FT4's `SicRounds`/single-pass, FST4's
   single-pass** — `.known(...)` was subtracted from the input audio
   up front (correct) but never threaded into the actual per-candidate
   dedup; a caller-level post-filter silently dropped anything that
   matched `known`, *after* `on_result` had already fired for it.
   Fixed by gating on `known` atomically, before the callback point
   (FT8: threaded `known` into the existing per-candidate dedup; FT4/
   FST4: a `pipeline::known_filtered_on_result` wrapper, since the
   shared generic engine underneath has no `known` parameter to thread
   it into).

Both were found by direct reproduction against real signals, not
inspection alone — see `tests/ft8_streaming_sic_early_with_known_matches_batch_exactly`
and `tests/ft4_streaming_sic_rounds_with_known_matches_batch_exactly`
for standing regression coverage.

Following both fixes, **every** `on_result`/`cb` call site in the
crate was re-audited directly (not by inference from the fix above).
Each one fires `cb` on exactly the value/set that is then committed to
the returned collection, with no filtering step running in between —
usually `if let Some(cb) = on_result { cb(&r); } vec.push(r);` in one
block, occasionally (FT4/FST4's `decode_frame_subtract`) a `for r in
&deduped { cb(r); }` loop immediately followed by
`all_results.extend(deduped)` with nothing else between the two. Both
shapes give the same guarantee: nothing can remove a value from the
committed set after its callback has already fired.

The 0.13 `Decoder` adds one layer above those sites and keeps the guarantee:
its wrapper turns the engine's result into a `Row` (unpacking the payload
against the hash table) and drops a result whose payload does not unpack, from
the callback and from the returned rows alike (`decoder/frame.rs`,
`frame_decode`).

| Site (engine; line numbers as of commit `1a4cbda`, re-check if this file has since diverged) | Location |
|---|---|
| FT8 `decode_block_multipass`/`decode_block_streaming` | `ft8/decode_block/process_candidates.rs:486` |
| FT8 `sic_inner_passes_with_cache` (covers `SicRounds`/`SicEarly`) | `ft8/decode.rs:630` |
| FT8 `decode_frame_inner` (parallel/sequential single-pass) | `ft8/decode.rs:374,387` |
| FT8 `decode_sniper_inner` (parallel/sequential sniper) | `ft8/decode.rs:996,1016` |
| FT4/FST4 `decode_frame` (parallel/sequential single-pass, generic engine) | `engine/pipeline.rs:994,1014` |
| FT4/FST4 `decode_frame_subtract` (`SicRounds`, generic engine) | `engine/pipeline.rs:1239` |
| WSPR scan (`decode_scan_inner`, pass 1 / pass 2) | `wspr/decode.rs` |
| Q65 scan | `q65/decode_request.rs:374` |
| Q65 averaged path (`decode_multi_period_for`) | `q65/rx.rs:1345` |
| Q65 internal scan helpers (`decode_scan_fading_for`, `decode_scan_with_ap_list_for`, `decode_scan_inner`) | `q65/rx.rs:511,610,700` |
| JT65 scan (`decode_scan_inner`, one loop for plain and Chase since #403) | `jt65/mod.rs` |
| JT9 scan (`decode_scan_inner`) | `jt9/mod.rs` |

If you're adding a new `_streaming` sibling or row-callback hook to a protocol
that also has (or gains) a cross-phase-dedup parameter, that combination is
exactly where this bug class lives — verify the callback fires from the same
branch that commits the value to the returned collection, the way every row
above does, rather than assuming a later post-filter is harmless just
because it's "only" filtering the return value.

---

## 4. Why a synchronous callback, and not `async` / Tokio / a channel

This is a deliberate design decision, not an unfinished one. The short
version: **mfsk-core stays runtime-agnostic so that every consumer can
choose its own concurrency model — including the ones that cannot have
a runtime at all — and bridging to Tokio at the edge is trivial (§5),
so nothing is lost by keeping async out of the core.**

The reasons, in order of weight:

### 4a. Portability — the core must stay `std`- and executor-free

mfsk-core's stated goal is a single crate "consumed identically from
several runtimes (native Rust, WebAssembly, Android JNI, C ABI)." The
`engine` and protocol layers are `no_std`-clean because the ESP32
embedded targets (`embedded-poc/m5stack-*-app`) are **first-class
consumers of this exact decode path** — the same `decode_block`
streaming callback runs on an Xtensa LX7 with no allocator-backed
executor and no `std`.

An `async fn` / `Future`-returning API — or anything that requires a
channel or executor by default — would pull `std` and a runtime into
the call graph *everywhere*, which breaks those `no_std` targets, the
`wasm32-unknown-unknown` build, the C ABI (`libmfsk.so`), and the JNI
scaffold. A synchronous `Fn` callback compiles unchanged on all of
them.

### 4b. There is nothing to `await`

Async is the right tool for **I/O-bound** work with many suspension
points — waiting on sockets, timers, disk. A decode is the opposite:
**CPU-bound compute over a fixed in-memory buffer**, start to finish,
with no external event to yield to. Making it `async` would add runtime
machinery and function-colour churn for **zero** latency or throughput
benefit — there is no await point where a reactor could do useful work
instead.

### 4c. No forced runtime choice (no function colouring)

An `async` decode API colours the whole call stack above it: every
caller must also become `async`, and must run *some* executor. A
synchronous callback forces none of that. The caller picks its model —
Tokio, `async-std`, a raw `std::thread`, a GUI event loop, a bare
embedded superloop, or nothing at all — and mfsk-core never needs to
know which. Baking in `tokio::sync::mpsc` (or any channel type) would
force a specific runtime on consumers that can't use one, and would
make the crate carry a heavy optional dependency for a job the caller
can do in one line.

### 4d. Precedent in the codebase

The plain-callback idiom already exists here:
`process_candidates_with_ap` takes a fill-closure
(`F: FnMut(&mut [[Cmplx<f32>;8];79], &SyncCandidate, SymMask)`).
`decode_with`'s row callback follows the same shape rather than introducing a
second, async-flavoured pattern next to it.

### The upshot

Because mfsk-core delivers results through a closure *you* supply, you
own how they cross a thread or into a runtime. Want them on a Tokio
channel? Put a `Sender` in the closure. Want them on a GUI thread?
Post to your event loop from the closure. Want cross-slot background
continuation (the WSJT-X "Fast"-mode shape — keep decoding after the
next slot's capture has started)? move the `Decoder` into a
`std::thread::spawn` and call `decode()` there (`Decoder<P>` is `Send`). None
of that requires core-library support, and all of it composes at the
application edge.

---

## 5. Worked example: calling from a Tokio async client

The goal: decode FT8 slots **without blocking the async runtime**, and
receive each message in an `async` loop **as it is decoded** rather than all
at once at the end of the slot.

Three facts drive the shape:

1. **The decode is blocking, CPU-bound work.** It must not run on a
   Tokio worker thread (it would stall the reactor). Run it on
   `tokio::task::spawn_blocking`.
2. **The decoder is stateful.** A `Decoder` keeps its callsign hash table (and
   FT8's a7 list) from slot to slot, so one blocking task owns one decoder for
   the life of the channel and is fed slots; a fresh decoder per slot would
   forget every hashed call.
3. **The callback is the bridge.** It captures a
   `tokio::sync::mpsc::Sender`, clones each borrowed row's owned `Decoded`,
   and pushes it. mfsk-core never sees the channel, the runtime, or `async`.

### `Cargo.toml`

```toml
[dependencies]
# add `features = ["serde"]` if you want to serialize decode rows to JSON.
mfsk-core = "0.13"
tokio = { version = "1", features = ["rt-multi-thread", "macros", "sync"] }
# Optional, only for the Stream adaptor in §5.3:
tokio-stream = "0.1"
```

### 5.1 The bridge

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, Row, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::msg::decoded::Decoded; // the crate's owned, Send UI row

use tokio::sync::mpsc;

/// One 15 s FT8 slot (12 kHz mono i16 PCM) and its index on the UTC grid.
pub struct Slot {
    pub period: i64,
    pub audio: Vec<i16>,
}

/// Start an FT8 decoder on a blocking worker. Send it slots; every accepted
/// message comes back on the returned channel as it lands.
///
/// The worker owns the `Decoder` (and so its hash table) until `slots` is
/// dropped; the returned channel closes when it finishes.
pub fn spawn_ft8_worker(
    params: DecodeParams,
) -> (std::sync::mpsc::Sender<Slot>, mpsc::Receiver<Decoded>) {
    let (slot_tx, slot_rx) = std::sync::mpsc::channel::<Slot>();
    // Bounded, so a slow consumer applies backpressure instead of letting
    // memory grow without limit. A single slot never yields anywhere near
    // 64 messages, so this practically never blocks the producer here.
    let (tx, rx) = mpsc::channel::<Decoded>(64);

    tokio::task::spawn_blocking(move || {
        let mut decoder = Decoder::<Ft8>::new(params);
        while let Ok(slot) = slot_rx.recv() {
            // The closure is the entire bridge between mfsk-core's synchronous
            // world and Tokio's async world. It is `Fn` (only `&self` methods:
            // `Sender::blocking_send`) and `Sync`, satisfying `decode_with`'s
            // `&(dyn Fn(&Row<_>) + Sync)` bound — required because a
            // single-pass strategy dispatches candidates across rayon worker
            // threads and may call this concurrently.
            let on_row = |row: &Row<DecodeResult>| {
                // `row.decoded` is already an owned, `Send` `Decoded` (text,
                // freq, dt, snr, protocol), resolved against this decoder's
                // callsign table — exactly what we want to move across the
                // channel. `blocking_send` is correct here: this runs on a
                // spawn_blocking thread (and, for the parallel strategies,
                // possibly a rayon worker) — never a Tokio runtime worker —
                // so it will not panic the way `blocking_send` does inside
                // async context. A send error means the receiver was dropped;
                // nothing more to do.
                let _ = tx.blocking_send(row.decoded.clone());
            };
            // `period` lets a7 and averaging see consecutive slots. The
            // returned `SlotResult` repeats what was streamed, so drop it.
            let _ = decoder.decode_with(&SlotInput::i16(&slot.audio).period(slot.period), &on_row);
        }
        // `tx` is dropped here as the task returns, closing the channel and
        // ending the consumer's loop.
    });

    (slot_tx, rx)
}
```

> `Decoded` (`mfsk_core::msg::decoded::Decoded`) is the crate's unified,
> owned decode row — `text` / `freq_hz` / `dt_sec` / `snr_db` /
> `protocol`, `Clone` + `Send`, and `Serialize`/`Deserialize` under
> `--features serde`. Every `Row` carries one in `.decoded`, whatever the
> mode, so the same bridge shape works for any `Decoder<P>`; a worker over an
> `AnyDecoder` uses `AnyDecoder::decode_with`, whose callback is handed the
> `Decoded` directly. See [LIBRARY.md](LIBRARY.md) and
> `docs/notes/DECODED_ROW.md`.

### 5.2 Consuming the stream

```rust
#[tokio::main]
async fn main() {
    let (slots, mut rx) = spawn_ft8_worker(DecodeParams::for_band((200.0, 3000.0)));

    // Your capture pipeline supplies this: one 15 s slot of 12 kHz mono
    // i16 PCM, aligned to the slot boundary (~180 000 samples).
    slots.send(Slot { period: 0, audio: load_one_ft8_slot() }).unwrap();
    drop(slots); // no more slots: the worker finishes and closes the channel

    // Each message arrives here the moment the decoder accepts it, not
    // batched at the end. The loop exits when the worker finishes and drops
    // its Sender.
    while let Some(msg) = rx.recv().await {
        println!(
            "{:+5.1} dB  {:7.1} Hz  dt={:+.2}s  {}",
            msg.snr_db, msg.freq_hz, msg.dt_sec, msg.text,
        );
        // ...or forward `msg` to a QSO state machine, a spot uploader,
        // a websocket, a database write — all `.await`-able from here.
    }

    println!("slot decode complete");
}

# fn load_one_ft8_slot() -> Vec<i16> { Vec::new() }
```

### 5.3 Optional: expose it as a `Stream`

If you prefer to hand the results to combinator-based consumers
(`while let Some(x) = stream.next().await`, `.map()`, `.filter()`),
wrap the receiver:

```rust
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::StreamExt;

let (slots, rx) = spawn_ft8_worker(params);
let stream = ReceiverStream::new(rx);
tokio::pin!(stream);
while let Some(msg) = stream.next().await {
    // same as §5.2, now composable with the StreamExt combinators
}
```

### 5.4 Common variations

- **Non-blocking producer.** If you would rather drop results than ever
  block the decode thread on a slow consumer, swap `blocking_send` for
  `try_send` and handle `Err(TrySendError::Full)` (e.g. count drops).
  With a bounded channel and `blocking_send`, a stalled consumer instead
  applies backpressure and slows the decode — usually what you want for
  a correctness-critical UI.
- **Sequential, exact-match delivery.** If you want the stronger
  contract from §3a (callback order == batch order, no transient
  duplicates), use a sequential strategy — `Depth::Normal` or `Deep` on FT8
  and FT4 (the default block is `Deep`), or `Ft8Extras::tuning.strategy =
  Some(Ft8Strategy::SicRounds(3))` — instead of a single-pass one. The
  bridge code is identical; only the decoder's settings change.
- **Other protocols.** Any `Decoder<P>` (WSPR, JT65, JT9, Q65, FT4, FST4) has the
  same `decode_with`; build `Decoder::<P>::new(..)` inside the same
  `spawn_blocking` shell, exactly as FT8 above. The closure captures
  the same `Sender`.
- **Several channels.** One worker, one `Decoder`, one hash table per
  channel — also what the IQ receiver's callers do with one `AnyDecoder` per
  `ChannelId` ([LIBRARY.md](LIBRARY.md) §2.7).
- **Cancellation.** Dropping the `Receiver` makes the next
  `blocking_send` in the closure return `Err`, which you can use to stop
  early — but note the decode itself has no interior cancellation point,
  so a `spawn_blocking` task runs to completion regardless. For hard
  cancellation, give `SlotInput::budget(..)` a predicate that reads a flag
  (every mode polls it once per candidate, [LIBRARY.md](LIBRARY.md)
  §2.3) and decode in shorter units.

---

## 6. See also

- [LIBRARY.md](LIBRARY.md) §2.4 — the streaming section in the wider API
  reference, and the `Decoder` / `DecodeParams` / extras surface it sits in.
- `Decoder::decode_with` doc comment (`mfsk-core/src/decoder/mod.rs`) and the
  engine's row-callback doc (`DecodeRequest::on_result`,
  `mfsk-core/src/msg/decode_request.rs`) — the authoritative, always-current
  delivery contract.
- `mfsk-core/tests/ft8_decode_block_streaming.rs` — the embedded
  `decode_block_streaming` exact-match test.
- `mfsk-core/tests/ft8_decode_block_streaming_host.rs` — the host
  `fft-rustfft` `decode_block_streaming` exact-match test (issue #243).
- `mfsk-core/tests/wspr_wsjtx_samples.rs` — WSPR's row callback against real
  signals.
- [BINDINGS.md](BINDINGS.md) — the same streaming idea across the C
  boundary: `mfsk_decoder_set_on_decode` and the `mfsk_stream_*` ring,
  callback-based for the same portability reasons.
