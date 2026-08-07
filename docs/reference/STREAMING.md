# mfsk-core — Streaming Decode Interface

> **日本語版:** [STREAMING.ja.md](STREAMING.ja.md)

This document covers the **streaming delivery** surface of mfsk-core:
the `.on_result(cb)` callback on the `DecodeRequest` / `SniperRequest`
family and its per-protocol siblings, what its delivery contract
guarantees, **why it is a plain synchronous callback rather than an
`async fn` / `Future` / channel-based API**, and a complete worked
example of bridging it into a Tokio async client.

For the wider library surface (trait hierarchy, DSP primitives, the C
ABI) see [LIBRARY.md](LIBRARY.md) — this doc drills into one section
of it (§4's "Streaming delivery") and adds the async-bridge example.

---

## 1. What "streaming" means here

A decode operates on **one complete audio slot already in hand** — a
15 s FT8 buffer, a 110.6 s WSPR buffer, and so on. It is not an
open-ended socket read. "Streaming" therefore does not mean *feeding
samples in incrementally*; the whole slot is passed by reference. It
means the opposite direction: **results flow out incrementally**, one
callback per accepted message, *as the decoder finds them*, instead of
being handed back only as a single `Vec` when the entire slot has
finished decoding.

The batch API and the streaming API are the **same call**. Streaming
is purely additive:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;

// Batch: get everything at the end.
let outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
    .decode();
for r in &outcome.results { /* ... */ }

// Streaming: same decode, plus a callback fired per result along the
// way. `outcome.results` still holds the full batch afterwards.
let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
    // fired as each candidate is accepted
};
let outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
    .on_result(&on_result)
    .decode();
```

A caller that never calls `.on_result()` sees zero behavioural
difference from before the feature existed.

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

There is **no single shared callback type** across protocols —
`DecodeResult` (FT8/FT4/FST4), `Q65Result`, `WsprResult`,
`Jt65Result`, `Jt9Result` are structurally distinct. So this is a
consistent *pattern* to code against (same name and semantics inside
each protocol's own API family), not one trait to abstract over.

| Protocol(s)          | Entry point                                                            | Callback type                          |
|----------------------|------------------------------------------------------------------------|----------------------------------------|
| FT8 / FT4 / FST4     | `DecodeRequest<P>` / `SniperRequest<P>` — `.on_result(cb)`             | `&(dyn Fn(&DecodeResult) + Sync)`      |
| Q65                  | `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` — `.on_result(cb)` | `&(dyn Fn(&Q65Result) + Sync)`         |
| WSPR                 | `decode_scan_streaming` / `decode_scan_subtract_streaming`             | `&(dyn Fn(&WsprResult) + Sync)`        |
| JT65                 | `decode_scan_streaming`                                                | `&(dyn Fn(&Jt65Result) + Sync)`        |
| JT9                  | `decode_scan_streaming`                                                | `&(dyn Fn(&Jt9Result) + Sync)`         |
| FT8 (embedded no_std)| `ft8::decode_block::decode_block_streaming`                            | `&mut dyn FnMut(&DecodeResult)`        |

Notes:

- **Builders (FT8/FT4/FST4/Q65)** carry `.on_result(cb)` as one more
  chainable method; the returned `DecodeOutcome` still holds the full
  batch.
- **WSPR/JT65/JT9 have no builder**, so each gained a
  `decode_scan_streaming` *sibling* alongside its existing
  `decode_scan` (WSPR also has `decode_scan_subtract_streaming` next to
  `decode_scan_subtract`). The sibling takes the same arguments plus a
  trailing `on_result` reference and returns the same `Vec` the
  non-streaming version does.
- **The embedded FT8 path** (`decode_block_streaming`, only compiled
  when `fft-rustfft` is *off*) takes `&mut dyn FnMut` rather than
  `&dyn Fn + Sync`: that path is strictly sequential (no rayon on
  no_std), so a `FnMut` closure that mutates captured state is safe and
  the `Sync` bound is unnecessary. Not available under `fft-rustfft` —
  the host multipass/subtract driver has a post-hoc SNR re-gate that
  can drop or mutate a result *after* it would already have streamed,
  which this callback has no revise/retract event for.

---

## 3. Delivery contract

There are exactly **two** contracts, decided by whether the strategy
runs sequentially or under `rayon` (`feature = "parallel"`). The
authoritative version is the doc comment on
`DecodeRequest::on_result`; summarised here:

### 3a. Sequential — exact match

`cb` fires **exactly once per result that ends up in the returned
`Vec`, in the same order**. Zero divergence between what you stream and
what the batch return holds.

Covers: `.sic_rounds(n)` and `.sic_early()` (FT8/FT4 SIC strategies);
the embedded `decode_block_streaming`; JT65 and JT9
`decode_scan_streaming`; every Q65 builder; WSPR's
`decode_scan_subtract_streaming` (fires only at its own outer SIC-pass
accept point). Q65's `MultiPeriodRequest` is a variant of the
sequential shape: it fires once **per slot** that yields an accepted
decode (its natural streaming unit for multi-period EME / ionoscatter
averaging), not once per candidate.

### 3b. Parallel — completion order, possible transient duplicate

`cb` fires from **whichever thread decoded that candidate, in
completion order** (not candidate-exploration order), and **before**
the final cross-candidate dedup pass. On the rare occasion two sync
candidates converge on the same message, `cb` may fire for both even
though only one survives into the returned `Vec`. Callers wanting exact
parity should dedup by `.message77()` on their side — the same key the
crate's own dedup uses.

Covers: the default single-pass strategy and `SniperRequest`
(FT8/FT4); WSPR's `decode_scan_streaming` (both coarse-search passes
run under `rayon::par_iter()`).

**This is why the parallel-path callback must be `Sync`** — it may be
called concurrently from multiple rayon worker threads.

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
`.on_result()` follows the same shape rather than introducing a second,
async-flavoured pattern next to it.

### The upshot

Because mfsk-core delivers results through a closure *you* supply, you
own how they cross a thread or into a runtime. Want them on a Tokio
channel? Put a `Sender` in the closure. Want them on a GUI thread?
Post to your event loop from the closure. Want cross-slot background
continuation (the WSJT-X "Fast"-mode shape — keep decoding after the
next slot's capture has started)? `std::thread::spawn` and call
`.decode()` there. None of that requires core-library support, and all
of it composes at the application edge.

---

## 5. Worked example: calling from a Tokio async client

The goal: decode one 15 s FT8 slot **without blocking the async
runtime**, and receive each message in an `async` loop **as it is
decoded** rather than all at once at the end.

Two facts drive the shape:

1. **The decode is blocking, CPU-bound work.** It must not run on a
   Tokio worker thread (it would stall the reactor). Run it on
   `tokio::task::spawn_blocking`.
2. **The callback is the bridge.** It captures a
   `tokio::sync::mpsc::Sender`, converts each borrowed `DecodeResult`
   into an owned `Send` value, and pushes it. mfsk-core never sees the
   channel, the runtime, or `async`.

### `Cargo.toml`

```toml
[dependencies]
mfsk-core = "0.8"
tokio = { version = "1", features = ["rt-multi-thread", "macros", "sync"] }
# Optional, only for the Stream adaptor in §5.3:
tokio-stream = "0.1"
```

### 5.1 The bridge

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

use tokio::sync::mpsc;

/// One decoded FT8 message, lifted out of the borrow-scoped, non-`Send`
/// `DecodeResult` into an owned value we can move across the channel.
#[derive(Debug, Clone)]
pub struct Decoded {
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
    pub text: String,
}

/// Decode one 15 s FT8 slot (12 kHz mono i16 PCM) on a blocking worker,
/// streaming each accepted message back to the async caller as it lands.
///
/// Returns immediately with the receiving half of a channel; the decode
/// runs on `spawn_blocking`. The channel closes when the decode finishes.
pub fn decode_slot_stream(audio: Vec<i16>) -> mpsc::Receiver<Decoded> {
    // Bounded, so a slow consumer applies backpressure instead of letting
    // memory grow without limit. A single slot never yields anywhere near
    // 64 messages, so this practically never blocks the producer here.
    let (tx, rx) = mpsc::channel::<Decoded>(64);

    tokio::task::spawn_blocking(move || {
        // The closure is the entire bridge between mfsk-core's synchronous
        // world and Tokio's async world. It is `Fn` (only `&self` methods:
        // `Sender::blocking_send`) and `Sync`, satisfying `.on_result`'s
        // `&(dyn Fn(&DecodeResult) + Sync)` bound — required because FT8's
        // default wide-band strategy dispatches candidates across rayon
        // worker threads and may call this concurrently.
        let on_result = move |r: &DecodeResult| {
            let text = unpack77(r.message77())
                .unwrap_or_else(|| "<unpack-fail>".into());

            // `blocking_send` is correct here: this runs on a spawn_blocking
            // thread (and, for the parallel strategy, possibly a rayon
            // worker) — never a Tokio runtime worker — so it will not panic
            // the way `blocking_send` does inside async context. A send
            // error means the receiver was dropped; nothing more to do.
            let _ = tx.blocking_send(Decoded {
                freq_hz: r.freq_hz,
                dt_sec: r.dt_sec,
                snr_db: r.snr_db,
                text,
            });
        };

        // Wide-band search 100–3000 Hz, sync_min 1.5, up to 100 candidates.
        // `.on_result` is additive — the returned DecodeOutcome still holds
        // the full batch, which we drop here since we streamed everything.
        let _outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
            .on_result(&on_result)
            .decode();
        // `on_result` (and its captured `tx`) is dropped here as the task
        // returns, closing the channel and ending the consumer's loop.
    });

    rx
}
```

### 5.2 Consuming the stream

```rust
#[tokio::main]
async fn main() {
    // Your capture pipeline supplies this: one 15 s slot of 12 kHz mono
    // i16 PCM, aligned to the slot boundary (~180 000 samples).
    let audio: Vec<i16> = load_one_ft8_slot();

    let mut rx = decode_slot_stream(audio);

    // Each message arrives here the moment the decoder accepts it, not
    // batched at the end. The loop exits when the decode task finishes
    // and drops its Sender.
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

let stream = ReceiverStream::new(decode_slot_stream(audio));
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
  duplicates), use a sequential strategy — e.g.
  `.sic_rounds(3)` or `.sic_early()` on FT8 — instead of the default
  wide-band pass. The bridge code is identical; only the builder method
  changes.
- **Other protocols.** For WSPR/JT65/JT9, call the free function
  `decode_scan_streaming(&audio, sample_rate, nominal_start_sample,
  &params, &on_result)` inside the same `spawn_blocking` shell; the
  closure captures the same `Sender`. For Q65, use its
  `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` builder exactly
  as FT8 above.
- **Cancellation.** Dropping the `Receiver` makes the next
  `blocking_send` in the closure return `Err`, which you can use to stop
  early — but note the decode itself has no interior cancellation point,
  so a `spawn_blocking` task runs to completion regardless. For hard
  cancellation, decode in shorter units (per-candidate `SniperRequest`
  calls) and check a flag between them.

---

## 6. See also

- [LIBRARY.md](LIBRARY.md) §4 — the streaming section in the wider API
  reference, and the `DecodeRequest` / `SniperRequest` builder surface
  it sits in.
- `DecodeRequest::on_result` doc comment
  (`mfsk-core/src/msg/decode_request.rs`) — the authoritative,
  always-current delivery contract.
- `mfsk-core/tests/ft8_decode_block_streaming.rs` — the embedded
  `decode_block_streaming` exact-match test.
- `mfsk-core/tests/wspr_wsjtx_samples.rs` — WSPR
  `decode_scan_streaming` / `decode_scan_subtract_streaming` against
  real signals.
- [EMBEDDED.md](EMBEDDED.md) — the C ABI streaming tutorial for the
  `mfsk-ffi-ft8` FFI artifact (the same streaming idea across the C
  boundary, callback-based for the same portability reasons).
