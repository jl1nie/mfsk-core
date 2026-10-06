# mfsk-core — Rust API reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

A pure-Rust reimplementation of the WSJT-X weak-signal decoders — FT8,
FT4, FST4, WSPR, JT9, JT65, Q65, MSK144 and JTTY — plus the
experimental, non-WSJT `uvpacket` mode, behind one generic core. The
core (`engine` / `fec` / `msg`) is protocol-agnostic; each protocol is
a zero-sized type that plugs a FEC codec, a message codec and a sync
mode into it. One receive flow runs for every wired protocol —
`coarse-sync → refine → LLR → FEC decode → message unpack` — with
per-protocol strategy variations layered on top, and one decode API on
WSJT-X's own model drives it: a persistent `Decoder<P>` per mode,
parameterised by WSJT-X's parameter block ([§2](#2-the-decode-api)). MSK144 and JTTY sit
beside that core rather than in it: neither has a slot, so neither has
a `Protocol` type ([§3.1](#31-generic-vs-bespoke-per-protocol)).

This document is the Rust host API. Other audiences:

| you are | read |
|---|---|
| calling from C, C++, Kotlin or Swift | [`BINDINGS.md`](BINDINGS.md) |
| targeting `no_std` / fixed-point / an MCU | [`EMBEDDED.md`](EMBEDDED.md) |
| delivering decodes as they are found | [`STREAMING.md`](STREAMING.md) |
| after the badges and a 20-line quickstart | [`README.md`](../../README.md) |
| asking *why* a design went the way it did | [`../notes/DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md) |

## Contents

- [1. Quick start](#1-quick-start)
  - [1.1 Use cases: a decoder is something you keep](#11-use-cases-a-decoder-is-something-you-keep)
  - [1.2 Coming from 0.12](#12-coming-from-012)
- [2. The decode API](#2-the-decode-api)
  - [2.1 `Decoder<P>`](#21-decoderp)
  - [2.2 `DecodeParams` and `Depth`](#22-decodeparams-and-depth)
  - [2.3 Compute budget](#23-compute-budget)
  - [2.4 Streaming delivery](#24-streaming-delivery)
  - [2.5 Extras, and the protocols outside `Decoder`](#25-extras-and-the-protocols-outside-decoder)
  - [2.6 Message acceptance](#26-message-acceptance)
  - [2.7 Wideband IQ input](#27-wideband-iq-input)
  - [2.8 Non-standard, compound and suffixed callsigns](#28-non-standard-compound-and-suffixed-callsigns)
- [3. Protocols](#3-protocols)
  - [3.1 Generic vs bespoke, per protocol](#31-generic-vs-bespoke-per-protocol)
  - [3.2 Geometry](#32-geometry)
  - [3.3 Per-protocol notes](#33-per-protocol-notes)
  - [3.4 Decoder strategies](#34-decoder-strategies)
- [4. Module and crate map](#4-module-and-crate-map)
- [5. The `Protocol` trait hierarchy](#5-the-protocol-trait-hierarchy)
- [6. Engine primitives](#6-engine-primitives)
- [7. Feature flags](#7-feature-flags)
- [8. Runtime registry and trait-surface verification](#8-runtime-registry-and-trait-surface-verification)
- [License](#license)

---

## 1. Quick start

```toml
[dependencies]
mfsk-core = { version = "0.13", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below enable
several for illustration.

**Decode an FT8 slot.** Synthesise a frame, then decode it back:

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::pack77;

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. Decode it back. One decoder per mode; the parameter block is
// WSJT-X's own (band, depth, station, ...). `Depth::Deep` is its default.
let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((100.0, 3_000.0)));
let result = decoder.decode(&SlotInput::i16(&audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in &result.rows {
    let d = &row.decoded;
    println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
             d.freq_hz, d.dt_sec, d.snr_db, d.text);
}
```

Real audio arrives as one period at 12 kHz from the nominal start.
`engine::dsp::resample` converts other sample rates; a `SlotInput` takes
`&[i16]` or `&[f32]`. Keep the `Decoder` between periods: it is where the
callsign hash table lives ([§2.1](#21-decoderp)). `Decoder::<Ft8>::with_defaults()`
starts from the block the WSJT-X GUI starts from instead of a band you name
([§2.2](#22-decodeparams-and-depth)).

### 1.1 Use cases: a decoder is something you keep

Before 0.13 a decode was a function call: audio in, rows out, and whatever
had to survive from one period to the next (the callsign hash table, the
previous cycle's decodes, Q65's averages) was the caller's to carry and hand
back. In 0.13 a decoder is an object you create once and keep, the way
WSJT-X keeps one decoder per mode running for the whole session. You change
its settings between periods, as the WSJT-X GUI does, and it remembers what
WSJT-X remembers. The examples below are the common jobs, each with the
reason the API is shaped that way. The rules they follow are listed at the
top of [§2](#2-the-decode-api).

**A live receiver: keep the decoder, number the periods.** This is the
use the API is built around. Create the decoder once, give it each
period as it completes, and number the periods on the UTC grid
(`t / T`). The number tells the decoder which periods are consecutive,
which FT8's a7 and Q65's averaging need. The callsign table lives in the
decoder, so a call heard in one period resolves a hashed `<...>` in the
next:

```rust
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::{pack77, pack77_type4};

/// One 15 s period holding one FT8 frame, as a receiver hands it over.
fn period(msg77: &[u8; 77]) -> Vec<i16> {
    let tones = message_to_tones::<Ft8>(msg77);
    let frame = synthesize_i16::<Ft8>(&tones, 12_000, 1_500.0, 20_000);
    let mut audio = vec![0i16; 180_000];
    audio[6_000..6_000 + frame.len()].copy_from_slice(&frame);
    audio
}

// Created once, kept for the whole session.
let mut rx = Decoder::<Ft8>::with_defaults();

// Period 0: JA1ABC calls CQ. The decoder learns the call.
let p0 = period(&pack77("CQ", "JA1ABC", "PM95").unwrap());
rx.decode(&SlotInput::i16(&p0).period(0));

// Period 1: a reply that names JA1ABC only by a 12-bit hash.
let p1 = period(&pack77_type4("JL1NIE/1", "JA1ABC", "RR73", false).unwrap());
let heard = rx.decode(&SlotInput::i16(&p1).period(1)).rows;
assert!(heard.iter().any(|r| r.decoded.text.contains("<JA1ABC>")));

// The same audio through a decoder that did not hear period 0.
let mut fresh = Decoder::<Ft8>::with_defaults();
let blind = fresh.decode(&SlotInput::i16(&p1).period(1)).rows;
assert!(blind.iter().all(|r| !r.decoded.text.contains("<JA1ABC>")));
```

*Why:* this is what a `jt9` process does, and keeping the state where
upstream keeps it means a caller cannot lose it or mix it up. In 0.12 every
caller carried the table, and the crate's own IQ receiver never resolved a
`<...>` (0.12.0 was yanked for it).

**Working a QSO: tell the decoder what the operator knows.** Your call, the
station you are working and how far the QSO has got go into the parameter
block, as the GUI fills them in. The decoder derives its a-priori
hypotheses from them, with upstream's tables: `MyCall DxCall ???` while you
wait for a report, `... RRR` / `73` / `RR73` once you have sent one. Update
them between periods with `params_mut()`; the decoder keeps everything else.

```rust
use mfsk_core::decoder::{
    ApMode, DecodeParams, Decoder, QsoContext, QsoProgress, SlotInput,
};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::new(
    DecodeParams::for_band((200.0, 4_000.0))
        .station("JL1NIE", "PM95") // MyCall, MyGrid
        .rx_freq(1_500.0)          // where the DX is
        .tx_freq(1_500.0)          // where you transmit
        .ap(ApMode::Full),         // "Enable AP"
);
let period = vec![0i16; 180_000]; // one period from the radio

// You answered JA1ABC's CQ.
rx.params_mut().qso = QsoContext {
    his_call: "JA1ABC".into(),
    his_grid: "PM95".into(),
    progress: QsoProgress::Replying,
};
rx.decode(&SlotInput::i16(&period).period(100));

// Next period: you have sent the report, so expect RRR / 73 / RR73.
rx.params_mut().qso.progress = QsoProgress::RogerReport;
rx.decode(&SlotInput::i16(&period).period(101));
```

*Why:* this is how WSJT-X finds the weak reply it is waiting for. With
the QSO context set, a weak FT8 reply decoded in 20 of 30 trials, against
0 of 30 without AP. Nothing runs until `station` is set, and FT8's default
has AP off, as the GUI's "Enable AP" box starts.

**Several bands or modes at once: one decoder each.** A skimmer, or a
receiver watching FT8 and FT4 together, holds one decoder per channel.
`AnyDecoder` picks the mode at run time, and a decoder is `Send`, so each
can run on its own thread:

```rust
use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;

let channels = [Mode::Ft8, Mode::Ft4];
let workers: Vec<_> = channels
    .into_iter()
    .map(|mode| {
        std::thread::spawn(move || {
            let mut rx = AnyDecoder::with_defaults(mode);
            let slot = vec![0i16; mode.meta().slot_samples_12k as usize];
            rx.decode_i16(&slot, Some(0)).rows.len()
        })
    })
    .collect();
for w in workers {
    w.join().unwrap();
}
```

*Why:* two WSJT-X instances keep two tables, and so do two decoders. A
call heard on 20 m does not resolve a hash on 40 m, and two threads never
contend for one table. To give a new channel a head start, call
`learn_callsign` on it.

**A deadline: decode what fits in the time you have.** The library reads no
clock. You pass a predicate, and it is asked between candidates whether to
go on. The report says whether the deadline cut the search short:

```rust
use std::time::{Duration, Instant};
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let period = vec![0i16; 180_000];
let deadline = Instant::now() + Duration::from_millis(500);
let keep_going = || Instant::now() < deadline;

let result = rx.decode(&SlotInput::i16(&period).budget(&keep_going));
if result.budget.exhausted {
    println!("cut short: {} candidates skipped", result.budget.candidates_skipped);
}
```

*Why:* the same code runs on a desktop, in wasm and on an MCU, which have
three different clocks, and a process can be suspended mid-slot. Honoured by
FT8, FT4 and FST4 ([§2.3](#23-compute-budget)).

**Rows on screen as they are found.** `decode_with` hands each row to a
callback as soon as it is decoded, and still returns them all at the end:

```rust
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let period = vec![0i16; 180_000];
let all = rx.decode_with(&SlotInput::i16(&period).period(7), &|row| {
    println!("{:+.1} s {:6.0} Hz  {}", row.decoded.dt_sec, row.decoded.freq_hz, row.decoded.text);
});
println!("{} rows in the period", all.rows.len());
```

*Why:* a deep FT8 search runs for a while, and a GUI should not wait for
all of it. The order and de-duplication contract is in
[`STREAMING.md`](STREAMING.md).

**A search other than the GUI's.** `Depth` gives what WSJT-X's Fast, Normal
and Deep give. For anything else (a microcontroller's budget, a sweep that
pins one knob, or 0.12's search), set the mode's `Tuning` extra. It
overrides only the fields you set:

```rust
use mfsk_core::decoder::{Decoder, Ft8Strategy};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let tuning = &mut rx.extras_mut().tuning;
tuning.sync_min = Some(0.8); // 0.12's FT8 search: a lower sync floor,
tuning.max_cand = Some(60); // fewer candidates,
tuning.strategy = Some(Ft8Strategy::SinglePass); // one pass, no subtraction
```

*Why:* what WSJT-X does lives in `DecodeParams`, and what this crate adds
lives in `Extras`, so a setting that is not WSJT-X's is visible as such in
your code.

**Audio at another rate.** The decoders take 12 kHz audio, as `jt9` does.
`engine::dsp::resample::resample_to_12k` (and its `f32` and streaming
forms) converts other rates. Above 12 kHz it first low-passes at the source
rate so nothing above 6 kHz folds into the band: at 48 kHz with WSJT-X's own
filter, the 49-tap `fil4` that `Detector.cpp` runs on the sound card's
audio, and at other rates with a low-pass built to the same specification.
Then it interpolates linearly. Up to 0.13.0 there was no filter: 48 kHz
went down by plain decimation, and full-band noise (a microphone, a noisy
sound card) cost FT8 about 6 dB (#576).

```rust
use mfsk_core::engine::dsp::resample::resample_to_12k;

let at_48k = vec![0i16; 48_000 * 15]; // 15 s from a 48 kHz sound card
let at_12k = resample_to_12k(&at_48k, 48_000);
assert_eq!(at_12k.len(), 180_000);
```

**One recording: one decoder, one call.** A file is the degenerate case of
a live receiver: `Decoder::new(params)` and one `decode`, as in the quick
start above. There is no separate one-shot API, because there is nothing a
one-shot call could do differently.

A period is the whole period from its nominal start. Shorter and longer
buffers are accepted (`tests/decoder_input_length.rs` drives every mode from
an empty buffer to one and a half periods); a shorter one yields only what it
holds. Up to 0.13.0 a buffer shorter than an FT8 or FT4 frame could panic
(#567).

### 1.2 Coming from 0.12

0.13 replaced the decode API rather than extending it: the per-family
request builders are gone, and one decoder per mode, driven by WSJT-X's own
parameter block, took their place ([§2](#2-the-decode-api)). 0.12.0 is
yanked and there is no 0.12.1. Why the API went this way, and what it gave
up to get there, is [`DESIGN_RATIONALE.md` §6](../notes/DESIGN_RATIONALE.md#6-the-decode-api-follows-wsjt-xs-decoder-013).

**What changes in your results, not only in your code.** The defaults are
now the WSJT-X GUI's ([§2.2](#22-decodeparams-and-depth)), so the same audio
with no options set does not come back the same:

| | 0.12 | 0.13 | measured |
|---|---|---|---|
| FT8 search | sync 0.8, 60 candidates | `Depth::Deep`: sync 1.3, 1000 candidates, `SicEarly` | on `qso3_busy.wav` the same 21 messages as `jt9 -8 -d 3`; 3.8–4.0× `jt9`'s speed in the upstream comparison ([`BENCHMARKS.md`](../notes/BENCHMARKS.md)) |
| band | 100–3000 Hz | FT8 / FT4 200–4000 Hz, FST4 600–1400 Hz | — |
| AP | on whenever a hint was given | FT8 and JT65 off, as their "Enable AP" boxes start; QSO-context AP once `station` is set | weak replies on FT8: 0 of 30 decoded with AP off, 20 of 30 on |
| JT9 / JT65 | the all-zero codeword (`000AAA 000AAA RA90`) came back from silence | dropped, as upstream does | — |
| WSPR | close to `wsprd` | `wsprd`'s own numbers | on the WSJT-X golden, SNR within 0.02 dB of an instrumented `wsprd` and DT equal to 3 decimals |

To keep a 0.12 search, set the frame family's `Tuning` extra
([§2.5](#25-extras-and-the-protocols-outside-decoder)); to get AP without a
QSO, set `station`, or the `ap_hint` extra.

**Code.** What a 0.12 caller has to change (the full list is `CHANGELOG.md`,
`## 0.13.0`):

| area | 0.12 | 0.13 |
|---|---|---|
| decode entry, FT8 / FT4 / FST4 | `msg::decode_request::DecodeRequest<P>` / `SniperRequest<P>` and their builders | `Decoder::<P>::new(DecodeParams)` + `decode(&SlotInput)`; options in `P::Extras`. The request types are `pub(crate)` (`internal-testing` reopens them) |
| decode entry, WSPR / JT9 / JT65 / Q65 | `wspr::`, `jt9::`, `jt65::`, `q65::DecodeRequest`; Q65 `SniperRequest`, `MultiPeriodRequest` | the same `Decoder<P>`; the wide-band requests are `pub(crate)`. `SniperRequest` (decode at a known alignment) stays public. Q65 averaging is `averaging` + `SlotInput::period` |
| options | builder methods per request (`.osd()`, `.strictness()`, `.eq_mode()`, `.ap_hint()`, `.sic_*()`, `.contest()`, `.tx_freq()`, …) | `DecodeParams` (WSJT-X's block: band, `rx_freq_hz`, `tx_freq_hz`, `depth`, `station`, `qso`, `ap`, `contest`, `eme_delay`, …) and per-mode `Extras` (`Tuning`, `ap_hint`, `eq`, `filter`, `a7`, `sniper`, `noise_blanker`, Q65's) |
| cross-period state | `.previous_cycle()`, `.hash_table(Arc)`, WSPR `.table(&mut)` / `.confirmed()`, `MultiPeriodRequest` | decoder state: `Decoder::clear()`, `learn_callsign`, `unpack77`; hash tables per decoder, never shared |
| `wsjtx_depth` | FT8 constructor with `WsjtxDepth::{D1, D2, D3}` | `DecodeParams::depth`, `Depth::{Fast, Normal, Deep}`: **decides every search setting**, per mode, as `ndepth` |
| defaults | FT8 sync 0.8 / 60 candidates; AP on when hinted; band 100-3000 | the depth's: `Deep` (FT8 sync 1.3, 1000 candidates); FT8 and JT65 AP off; FT8 / FT4 band 200–4000, FST4 600–1400 (`default_params`) |
| AP | a free-form `ApHint` only (Q65 alone had QSO codewords) | QSO-context AP for FT8, FT4, FST4 from `station` + `qso` + `ap` and upstream's `naptypes`; `ApHint` stays as the `ap_hint` extra |
| JT9 / JT65 | reported the all-zero codeword | do not |
| return | `DecodeOutcome { results, fft_cache, budget }` | `SlotResult { rows: Vec<Row { decoded, detail, native }>, budget }`; no `fft_cache` |
| streaming | `.on_result(cb)` on each request | `Decoder::decode_with(&slot, on_row)`; rows resolved against the decoder's table |
| budget | `.budget(check)` | `SlotInput::budget(check)` |
| audio | `&[i16]` (frame family), `&[f32]` (others) | `SlotInput::i16` / `SlotInput::f32` for every mode |
| runtime mode | `iq::IqMode` | `registry::Mode` and `AnyDecoder` |
| IQ | `IqReceiver` decoded inside `push_*` with frozen defaults, `on_decode`, `set_time_anchor`, `IqDecode` rows | pull: `push_*(.., &mut Vec<CompletedSlot>)`, `set_time(utc_ns, at_sample)` → `ClockChange`, `retune` → `RetuneReport`; decode with one `AnyDecoder` per channel |
| time | per-receiver slot arithmetic | `slotgrid::{SlotGrid, SampleClock, SlotCutter}` |

**Removed, and what to do instead.**

| 0.12 | instead |
|---|---|
| `.hash_table(Arc)` — one table shared by several requests | each decoder keeps its own, as each upstream process does. Two channels of one mode learn separately; seed one with `learn_callsign` |
| `.previous_cycle()`, `MultiPeriodRequest` — state passed in per call | the decoder keeps it. Number the periods with `SlotInput::period` so it knows which are consecutive |
| `.known(list)` — signals an earlier pass already decoded, subtracted before searching | no counterpart yet. The early-decode design ([#572](https://github.com/jl1nie/mfsk-core/issues/572)) covers it: a caller re-searching a slot already decoded once (WebFT8's two-phase decode, [#587](https://github.com/jl1nie/mfsk-core/issues/587)) is its `Early`/`Final` called over the same complete period, which subtracts `Early`'s rows before the second search |
| `.message_filter(closure)` / `.also_accept(closure)` | `MessageFilter::Only(f)` / `AlsoAccept(f)` with `f` a function pointer, so the extras stay `Clone + 'static`. A closure that captures nothing coerces; a predicate that needs data reads it from a `static` ([§2.6](#26-message-acceptance)) |
| `.fft_cache()` / the outcome's `fft_cache` — the slot FFT handed to a second pass | no counterpart, and not restored by #572: the cache only ever skipped `build_fft_cache`'s per-round forward FFT, not `compute_spectrogram`/`coarse_sync` (the expensive part), so it would not have fixed what #587 measured. #572's `Early`/`Final` fixes that by not re-searching, not by caching |
| `wsjtx_depth(WsjtxDepth::D1…D3)` | `Depth::Fast` / `Normal` / `Deep` in `DecodeParams`, now for every mode |

The synthesis API (`engine::tx`), `dt_sec` from the nominal start and
`engine::search` of 0.12 are unchanged.

---

## 2. The decode API

mfsk-core decodes the way WSJT-X does. WSJT-X runs one decoder per mode
(`jt9 -s`); the GUI fills one parameter block (`lib/jt9com.f90`) before
every period and the decoder reads it, keeping across periods only what
its SAVE and module variables hold. `mfsk_core::decoder` is that model:

| WSJT-X | mfsk-core |
|---|---|
| the decoder process of one mode | `Decoder<P>` |
| the `params` block (`nfa`, `nfb`, `nfqso`, `ndepth`, `mycall`, `hiscall`, `lft8apon`, …) | `DecodeParams`, changed between periods with `params_mut()` |
| the audio of one period (`id2`) | `SlotInput` |
| what SAVE variables keep (hash tables, a7, averaged spectra) | `P::State`, per decoder, allocated on first use |
| nothing: upstream has no such options | `P::Extras`, typed per mode, so an option a mode lacks does not compile |

There is no per-call options object and no one-shot API beside it: a
recording is `Decoder::<P>::new(params)` and one `decode`. The engine
functions underneath (`decode_frame`, `process_candidate_basic`, the
`GenericPipelineProtocol` trait, the per-family `DecodeRequest` types) are
`pub(crate)`, so downstream cannot bypass the decoder; the non-default
`internal-testing` feature reopens them for the crate's own integration
tests. `Decoder<P>` exists for the seven slot-decoded families (FT8, FT4,
FST4, WSPR, JT9, JT65, Q65). uvpacket, MSK144 and JTTY keep their own entry
points ([§2.5](#25-extras-and-the-protocols-outside-decoder)).

**The rules the API follows.** Each one is upstream's behaviour, kept on
purpose; knowing them predicts most of what follows.

1. **One decoder per mode, and it keeps what upstream keeps.** Hash tables,
   FT8's a7 rows, Q65 and JT65 averages live in the `Decoder`, are never
   shared between decoders, and survive until `clear()` ([§2.1](#21-decoderp)).
2. **The parameter block is WSJT-X's, and each mode reads only what its
   upstream decoder reads.** A field a mode does not read is ignored, as
   `jt9` ignores it, not rejected: check the "read by" column of
   [§2.2](#22-decodeparams-and-depth).
3. **`Depth` sets the search, as `ndepth` does.** The library's own knobs
   (`Tuning`) override a setting only when you set them
   ([§2.5](#25-extras-and-the-protocols-outside-decoder)).
4. **What upstream does not have is an `Extras` field, typed per mode.** An
   option a mode lacks does not compile; through `AnyDecoder` or the C ABI it
   is an `Unsupported` error, never a silent no-op.
5. **The defaults are the WSJT-X GUI's**, so an unconfigured decoder does
   what an unconfigured WSJT-X does.
6. **There is no way around the decoder.** The engine underneath is
   `pub(crate)`, so every caller gets the same state handling and the same
   defaults.

### 2.1 `Decoder<P>`

```text
pub struct Decoder<P: Decodable> { params, extras, state }
```

`Decodable` is implemented by every slot-decoded ZST. It binds the mode
(`const MODE: Mode`), `type State` (what upstream keeps across periods),
`type Extras` (what this library adds) and `type Row` (the mode's native
result).

| method | effect |
|---|---|
| `Decoder::new(params)` | a decoder with that block. Allocates nothing until the first decode |
| `Decoder::with_defaults()` | the block the mode starts with in the GUI ([§2.2](#22-decodeparams-and-depth)) |
| `params()` / `params_mut()` | the block, changed between periods as the GUI rewrites it; state is kept |
| `extras()` / `extras_mut()` / `with_extras(e)` | the mode's library options ([§2.5](#25-extras-and-the-protocols-outside-decoder)) |
| `decode(&SlotInput)` | decode one period → `SlotResult<P::Row>` |
| `decode_with(&SlotInput, on_row)` | the same, handing each row to `on_row` as it is found ([§2.4](#24-streaming-delivery)) |
| `unpack77(&[u8])` | a packed 77-bit message as text, `<...>` resolved against **this** decoder's table |
| `learn_callsign(&str)` | teach this decoder's table a callsign (`save_hash_call`); `false` for a mode with no hashed calls |
| `clear()` | forget everything carried across periods (WSJT-X's "Clear Avg" and `ndepth & 128`) |

`Decoder<P>` is `Send` (it is not required to be `Sync`): move it to a worker
thread, give each channel its own.

**A period in, rows out.** `SlotInput` is one whole period of audio from
its nominal start:

| field / constructor | meaning |
|---|---|
| `SlotInput::i16(&[i16])`, `SlotInput::f32(&[f32])` | the audio, `Audio::I16` (what `jt9` reads as `id2`) or `Audio::F32`. The frame family (FT8, FT4, FST4) takes 16-bit audio as WSJT-X does, so `F32` is scaled to a fixed RMS (`decoder::F32_TO_I16_RMS`) first; WSPR, JT9, JT65 and Q65 work in `f32`, so `I16` is divided by 32768. A caller never picks a level |
| `.period(n)` | the period's index on the UTC grid (`t / T`). State that needs consecutive periods (FT8 a7, Q65 averaging) is used only when it is known; without it a lone recording leaves that state untouched |
| `.budget(check)` | a deadline predicate, [§2.3](#23-compute-budget) |

There is **no staged or early-decode entry point**: a `SlotInput` is the
whole period. (WSJT-X's nzhsym 41/47/50 early decode is not part of this
API; the boards run their own prefix path on the low-level items.)

A `SlotResult<R>` is `rows: Vec<Row<R>>`, in the order they were found, and
`budget: BudgetReport`. A `Row<R>` carries three views of one decode:

| field | what |
|---|---|
| `decoded: Decoded` | the cross-mode row: `text` (resolved against the decoder's hash table), `freq_hz`, `dt_sec`, `snr_db`, `protocol` |
| `detail: RowDetail` | what the modes share beyond it: `sync_score`, `sync_cv`, `hard_errors`, `pass`, `info`, `hash_resolved` (the text needed the table for a `<...>`), `copied_last_tx` (Q65 Pileup). A mode without a field leaves it at its default; WSPR, JT9 and JT65 fill none of them |
| `native: R` | the mode's own result: `DecodeResult` (FT8, FT4, FST4), `WsprResult`, `Jt9Result`, `Jt65Result`, `Q65Result` |

**What a decoder keeps across periods** is what its upstream decoder keeps,
and nothing else. It is per decoder and never shared between decoders, as
upstream's tables are per process: two channels of one mode have two
tables.

| mode | `State` | upstream |
|---|---|---|
| FT8, FT4, FST4 | `FrameState`: the callsign hash table; for FT8 with the `a7` extra, the decodes of the last two periods | `packjt77`; `ft8_a7.f90` |
| Q65 | `Q65State`: the hash table, and the running average of the symbol spectra (`s1a`, `navg`) with the last period's index | `packjt77`; `q65.f90` SAVE |
| WSPR | `WsprState`: the callsign table that lets OSD confirm a station Fano already heard (not capped) | wsprd `hashtable.txt` |
| JT9 | `()` — its 72-bit messages carry no hashed calls | — |
| JT65 | `Averager`: the periods `avg65` sums (up to 64, each 63 × 64 symbol powers; 16 KB apiece, allocated as they come) | `jt65_decode.f90` `avg65` |

Hashes are resolved and learned **after** the candidate loop, single
threaded, in decode order (`unpack77_learn`): a message does not resolve its
own hash against a call it introduces, and a call heard in period *n*
resolves a `<...>` in period *n*+1 of the **same** decoder. Tables are
allocated lazily on first insert, as one block, so a `Decoder::new` costs
nothing on an embedded heap.

```rust
use mfsk_core::decoder::{DecodeParams, Decoder};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::pack77_type4;

// "<JA1ABC> JL1NIE/1 RR73": the standard call travels as a 12-bit hash.
let msg77 = pack77_type4("JL1NIE/1", "JA1ABC", "RR73", false).unwrap();

// A fresh decoder cannot say whose hash that is ...
let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)));
let blind = decoder.unpack77(&msg77).unwrap();
assert!(blind.contains("<...>"), "{blind}");

// ... one that has heard the call can; another decoder still cannot.
assert!(decoder.learn_callsign("JA1ABC"));
assert!(decoder.unpack77(&msg77).unwrap().contains("<JA1ABC>"));
let other = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)));
assert!(other.unpack77(&msg77).unwrap().contains("<...>"));
```

**Choosing the mode at run time: `AnyDecoder`.** `AnyDecoder::new(Mode,
DecodeParams)` is an enum with one variant per `registry::Mode` this build
has (`Mode::ALL`, name lookup `Mode::from_name`), dispatched by `match`:
no `Box<dyn>` and no allocation on the decode path. It exists for code that
holds the mode as data: the IQ receiver's callers ([§2.7](#27-wideband-iq-input)),
the C ABI, a GUI. Its methods mirror `Decoder`'s, with the mode-erased result
`AnySlotResult { rows: Vec<Decoded>, details: Vec<RowDetail>, budget }`
(code that wants a mode's native result holds the typed `Decoder<P>`).
`extras_mut()` returns an `AnyExtras` to match on; `set_ap_hint` sets the
free-form AP hint on the modes that take one and returns
`Err(Unsupported { mode, option })` on the others; `decode_i16(audio,
period)` is the short form. `AnyDecoder` exists only when at least one
protocol feature does.

```rust
# #[cfg(all(feature = "ft8", feature = "wspr"))] {
use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, AnyExtras, DecodeParams};
use mfsk_core::msg::ApHint;

let mut dec = AnyDecoder::new(Mode::Ft8, DecodeParams::for_band((200.0, 3_000.0)));
dec.set_ap_hint(Some(ApHint::new().with_call1("CQ").with_call2("JA1ABC"))).unwrap();
if let AnyExtras::Ft8(e) = dec.extras_mut() {
    e.a7 = true; // the mode's own options, typed
}
assert_eq!(dec.mode(), Mode::Ft8);

// WSPR has no AP hint: the mismatch is an error value, not a no-op.
let mut wspr = AnyDecoder::with_defaults(Mode::Wspr);
assert!(wspr.set_ap_hint(None).is_err());
# }
```

### 2.2 `DecodeParams` and `Depth`

`DecodeParams` is `lib/jt9com.f90`'s `params` block (`#[non_exhaustive]`,
built with `DecodeParams::for_band((lo, hi))` and chained setters). **Each
mode reads what its upstream decoder reads and ignores the rest, as `jt9`
does**; a field does not make an option exist for a mode.

| field | upstream | read by |
|---|---|---|
| `band_hz` | `nfa`, `nfb` | every mode |
| `rx_freq_hz` | `nfqso` | FT8 (candidates within 10 Hz of it are decoded first; hypotheses naming a partner only within 50 Hz of it; a8; the sniper's centre), FT4 and FST4 (the same AP rule), JT9 (the Rx-frequency pass), Q65 (the q3 list decode) |
| `tol_hz` | `ntol` | JT9 (default 50 Hz), Q65 (F Tol, default 10 Hz) |
| `tx_freq_hz` | `nftx` | FT8: both-callsign hypotheses within 50 Hz of it |
| `depth` | `ndepth & 7` | every mode, table below |
| `averaging` | `ndepth & 16` | Q65 and JT65 (both need `SlotInput::period`; JT65's is `jt65::averaging`, `avg65`) |
| `deep_search` | `ndepth & 32` | JT65's upstream flag; not yet read |
| `station` | `mycall`, `mygrid` | FT8, FT4, FST4 (AP), Q65 (AP list) |
| `qso` | `hiscall`, `hisgrid`, `nQSOProgress` | the same |
| `ap` | `lft8apon`, `lapcqonly` | the same: `ApMode::{Off, CqOnly, Full}` |
| `contest` | `ncontest` | FT8 (keeps `/R` and `TU; ` messages, [§2.6](#26-message-acceptance); the contest's `CQ` token; Fox), FT4 and FST4 (the `CQ` token), Q65 (the callers list) |
| `eme_delay` | `emedelay` | FT8 (reports `dt` 2 s later), Q65 (the late search edge moves to +5.5 s, +4.0 s on Q65-15) |

**Defaults follow the GUI.** `Decoder::with_defaults()` and
`decoder::default_params(mode)` give `Depth::Deep` (the GUI's `NDepth`
default), AP **off for FT8 and JT65** (their "Enable AP" boxes start
unchecked; the other modes have no box), and the band the GUI implies: the
GUI has none of its own (`nfa` is the waterfall's start, `nfb` its right
edge), so FT8 and FT4 take `jt9`'s command-line 200–4000 Hz, FST4 the GUI's
own F Low / F High 600–1400 Hz, and the other modes their registry band.
`DecodeParams::for_band` is the bare block: `Deep`, AP `Full`, no station or
QSO. With no station call that leaves only the blind `CQ` hypothesis, so it
is not the same as `default_params(Mode::Ft8)`, which is `ApMode::Off`.

**`Depth` decides every search setting, as `ndepth` does**, unless the
mode's `Tuning` extra sets one ([§2.5](#25-extras-and-the-protocols-outside-decoder)). `Fast`,
`Normal` and `Deep` are `ndepth` 1 / 2 / 3, and per mode they set what the
mode's upstream decoder sets for them, line by line (the citations are on
each `Decodable` impl; the source is the `v3.2.0-rc1` export, not the 2.7
tree):

| mode | Fast | Normal | Deep | upstream |
|---|---|---|---|---|
| FT8 | sync 2.1, 1000 candidates, no OSD, 2 flat SIC rounds, nsync floor 8 | sync 2.1, OSD, `SicEarly`, floor 8 | sync 1.3, OSD, `SicEarly`, floor 6 / 7 | `ft8_decode.f90:175-181`, `ft8b.f90:178-180,430-437` |
| FT4 | sync 1.18, 200 candidates, 1 pass, no OSD, no AP | 3 passes, no OSD | 3 passes, OSD | `ft4_decode.f90:31,192-203,323-324` |
| FST4 | minsync 1.20 (FST4-15: 1.15), 200 candidates, OSD, no `i0 ± 1` timing retry, no AP | + the retry (`jittermax`) | same | `fst4_decode.f90:53,234-248,308-309,421-423` |
| WSPR | `wsprd -qB`: 2 passes, no jitter | `-C 500 -o 4`: 3 passes, OSD | `+ -d`: more candidates | `wsprd.c:819-900`; GUI `mainwindow.cpp:2824-2826` |
| JT9 | Fano limit 5000 | 10000 | 30000 | `jt9_decode.f90:83-100` |
| JT65 | 2 passes, `nvec` 100 | 2 passes, 1000 | 4 passes, 1000 | `jt65_decode.f90:110-119` |
| Q65 | `maxiters` 40, `(idf, idt, maxdist)` (1, 1, 4) | 60, (3, 3, 5) | 100, (5, 5, 5) | `q65_decode.f90:183-188`, `q65_loops.f90:27-40` |

**QSO-context AP (FT8, FT4, FST4).** The AP hypotheses are derived from
`station`, `qso` and `ap` through upstream's `naptypes` tables
(`ft8b.f90:55-70`, `ft4_decode.f90:132-137`, `fst4_decode.f90:133-138`);
nothing runs unless `station` is set. The `iaptype`s tried in a period by
`nQSOProgress` (1 = `CQ ??? ???`, 2 = `MyCall ??? ???`, 3 = `MyCall DxCall ???`,
4 / 5 / 6 = type 3 ending `RRR` / `73` / `RR73`):

| `qso.progress` | FT8 | FT4, FST4 |
|---|---|---|
| `Calling` | 1, 2 | 1, 2 |
| `Replying`, `Report` | 2, 3 | 2, 3 |
| `RogerReport`, `Rogers` | 3, 4, 5, 6 | 3, 6 |
| `Signoff` | 3, 1, 2 | 3, 1, 2 |

A type that names `MyCall` needs a standard `station.call`, one that names
`DxCall` a standard `qso.his_call` (a non-standard call such as `PJ4/K1ABC`
rules those types out); types 3 and above run only within 50 Hz of the Rx or
Tx frequency. `ApMode::CqOnly` keeps type 1, `Off` none; FT4 and FST4 run no
AP at `Fast`; Fox (and FT4's Hound) run none. The blind `CQ` uses the
contest's token (`CQ TEST`, `CQ FD`, `CQ RU`). Hound's "below 950 Hz only"
is not applied. The free-form `ap_hint` extra ([§2.5](#25-extras-and-the-protocols-outside-decoder))
**replaces** this derivation when set. Q65 derives its codeword list from the
same fields instead (`standard_qso_codewords`, or the contest list), used as
the q3 list when `rx_freq_hz` is set. FT8's a8 runs when the hint holds
MyCall, DxCall and the partner's grid with an Rx frequency; a7 is the `a7`
extra.

**The strategy extensions are where phantom decodes come from.** Both
false-decode bugs this suite has shipped were in subtraction paths
(#243 in `__staged_sic`, #253 in `.sic_early()`), so a new strategy
ships with its precision guard in the same PR.

### 2.3 Compute budget

`SlotInput::budget(check)` takes a caller-supplied predicate
(`&(dyn Fn() -> bool + Sync)`) polled between candidates. **The library
reads no clock of its own** — the deadline is whatever your predicate
compares against, which is what keeps it usable from wasm and from a
process that was suspended mid-slot.

`SlotResult::budget` is a `BudgetReport` saying what the cut left
undone: candidates skipped, stages run, and how good the best skipped
candidate was — so a caller can tell "nothing was there" from "we ran
out of time with a promising candidate still queued". `rows_subtracted`
covers the one place the budget is spent on something other than a
candidate: FT8 `SicEarly`'s checkpoint-B and -C subtraction loops. Below
the row count, with `exhausted` set, the cut came while cleaning up
rather than while looking — the fields above cannot say so, because a row
being subtracted carries no candidate ranking.

Honoured by FT8, FT4 and every FST4 sub-mode
(`MFSK_CAP_BUDGET` is the same fact published to C). WSPR, JT9, JT65 and Q65
decode the whole period and return an empty report.

### 2.4 Streaming delivery

`Decoder::decode_with(&slot, &|row: &Row<_>| …)` delivers each row as it is
found, on top of the `SlotResult` the call returns — for a UI that wants
something on screen before a long slot finishes. `AnyDecoder::decode_with`
takes `&(dyn Fn(&Decoded, &RowDetail) + Sync)`.

The delivery-order and de-duplication contract is **not repeated
here**: [`STREAMING.md`](STREAMING.md) is the authoritative account.
In one line: a sequential strategy delivers exactly the rows the call
returns, in the same order; a parallel one delivers in completion order and
may show a transient duplicate that the returned rows have already deduped.
A row handed to the callback is resolved against the hash table as it stood
when the period began; the returned rows also see calls learned earlier in
the same period.

Every mode offers the same shape through the same method. WSPR's is the
parallel contract, not the exact one — see [`STREAMING.md`](STREAMING.md)
§3b. JTTY has no `Decoder` and delivers by callback from inside the audio
call: `jtty::rx::Stream::push(samples, &mut |update| …)` (and `finish`) call
it on the caller's thread — [§2.5](#25-extras-and-the-protocols-outside-decoder).

### 2.5 Extras, and the protocols outside `Decoder`

**Extras** are what the library adds beyond upstream. Each mode's
`Decodable::Extras` holds only the options that mode supports, so an option
a mode lacks is a compile error, not a runtime refusal (the C ABI and
`AnyExtras` return `Unsupported` for the same mismatch at run time). Extras
are `Clone + Default`, set with `extras_mut()` or `with_extras(..)`, and
may change between periods.

*The frame family* (`Ft8Extras`, `Ft4Extras`, `Fst4Extras`) shares these:

| field | type | default | effect |
|---|---|---|---|
| `tuning` | `Tuning<S>` | all `None` | the library's own search settings, overriding what `Depth` sets **only when set**: `sync_min`, `max_cand`, `osd`, `strictness` (`DecodeStrictness`, [§6](#6-engine-primitives)) and `strategy`. Embedded (15 candidates, one pass) and the tier-C sweeps use it |
| `ap_hint` | `Option<ApHint>` | none | a free-form a-priori hint beside upstream's QSO-context AP: the skimmer's "hunt one DX" case, which upstream expresses only through the QSO context. When set it replaces the derived hint. Hypotheses that lock both callsigns run only for a candidate within 50 Hz of `rx_freq_hz` (`ft4_decode.f90` / `ft8b.f90` always have an `nfqso`) |
| `eq` | `EqMode` | `Off` | `Off` / `Local`. A property of the **input audio**, not the search |
| `filter` | `MessageFilter` | `Default` | message acceptance, [§2.6](#26-message-acceptance) |

and, per mode:

| extra | on | effect |
|---|---|---|
| `Tuning::strategy` | FT8: `Ft8Strategy::{SinglePass, SicRounds(n), SicEarly}`; FT4: `Ft4Strategy::{SinglePass, SicRounds(n)}`; FST4: `Fst4Strategy::SinglePass` | an enum dispatched with `match` and monomorphised per mode, so a strategy that is not selected costs nothing. `SicRounds(n)` is flat successive-interference cancellation (n clamped to 1..=3); `SicEarly` is the checkpoint emulation (`jt9 -d2/-d3`), fixed 3-checkpoint structure. FT4 has no `SicEarly` and FST4 no subtraction, mirroring an upstream absence |
| `a7` | FT8 (`bool`, off) | WSJT-X's **a7** list decoder (`ft8_a7.f90`), fed by this decoder's own decodes of period *n* − 2 (the same sequence one cycle earlier; it needs `SlotInput::period`). For each of those pairs the messages it could send next are matched at its old frequency and DT (pass id 30) |
| `sniper` | FT8 (`Option<Sniper { search_hz }>`, 250 Hz) | the roofing-filter mode, below. Needs `rx_freq_hz` |
| `noise_blanker` | FST4 (`Option<NoiseBlanker>`) | WSJT-X's **NB** (`blanker.f90`): zero the loudest samples before the slot FFT. `Percent(n)` blanks `n` % (0..=25); `Sweep { step, ftol_hz }` decodes at 0, step, … 20 %, the levels above 0 only within `ftol_hz` of `rx_freq_hz` (up to 21 decodes). Off, like WSJT-X's default NB 0 % |

**What makes sniper mode is the window, not the hint.** `Sniper` confines the
search to ±`search_hz` around `rx_freq_hz`, and it exists because the
operator narrowed the transceiver's *analogue* roofing filter — the Yaesu
FTDX101MP and FTDX10 are the usual examples at ~500 Hz — and pointed it at a
station whose carrier is already known. The audio arriving is already
band-limited; the decoder is matching the hardware. `eq: EqMode::Local`
flattens the tilt that filter's skirt puts on the passband. It is
**FT8 alone**, and it is not a general "hunt one known station" convenience:
that is `ap_hint` on the wide-band path, which FT8, FT4 and every FST4
sub-mode have. FT4 and FST4 sniper entry points existed until 2026-09-13 and
were removed: the wide-band path is the main path for every mode here, and if
it is not WSJT-X-faithful without a sniper, that is a bug in the wide-band
path. Full reasoning, with the measurements, in
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md).

```rust
use mfsk_core::decoder::{Decoder, DecodeParams, Sniper};
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::ApHint;
use mfsk_core::msg::wsjt77::pack77;

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1000.0, /* amp */ 20_000);
let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
audio[start..start + frame.len()].copy_from_slice(&frame);

let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)).rx_freq(1000.0));
let extras = decoder.extras_mut();
extras.sniper = Some(Sniper { search_hz: 250.0 });
extras.eq = EqMode::Local;
extras.ap_hint = Some(ApHint::new().with_call1("CQ").with_call2("JA1ABC"));

let result = decoder.decode(&mfsk_core::decoder::SlotInput::i16(&audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in &result.rows {
    println!("{:7.1} Hz  {}", row.decoded.freq_hz, row.decoded.text);
}
```

The strategy extensions are where phantom decodes come from ([§2.2](#22-decodeparams-and-depth));
`Tuning::strategy` and `a7` are the non-default code paths.

*WSPR, JT9, JT65* take `WsprExtras`, `Jt9Extras`, `Jt65Extras`. All three
carry `search: SearchTuning` (`time_tolerance_early_sec`,
`time_tolerance_late_sec`, `score_threshold`, `max_candidates`, each
`Option`, over the mode's `default_search_params()` with the block's band).
WSPR adds `max_cycles_per_bit` over the depth's (10000 is `wsprd`'s own
default, which the GUI's Normal and Deep lower to 500; measured: on the
WSJT-X golden 500 loses G8VDQ at −23 dB, which 10000 decodes); JT65 adds
`chase: Option<ChaseParams>` (by default the Chase decoder's trial count is
the depth's `nvec`).

**WSPR** decimates the slot to wsprd's 375 Hz baseband and runs
wsprd's own coarse search and three decode passes there, rather than
the shared FT-style pipeline. The FEC (`ConvFano`) and message codec
(`Wspr50Message`) are still associated types on `impl Protocol for
Wspr`, so the trait surface stays consistent — only the slot-level
decoder differs. The decoder's table lets OSD re-find a station a previous
slot's Fano decode confirmed, which is how `wsprd` reaches W3BI at −25 dB on
its own sample file.

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::msg::WsprMessage;
use mfsk_core::wspr::Wspr;
use mfsk_core::wspr::tx::synthesize_type1;

// Synthesise a WSPR Type 1 frame (120 s @ 12 kHz slot).
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let mut decoder = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1600.0)));
let result = decoder.decode(&SlotInput::f32(&samples_f32));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in result.rows {
    let d = row.native; // WsprResult
    match d.message {
        WsprMessage::Type1 { callsign, grid, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {} {}dBm", d.freq_hz, d.snr_db, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {}dBm", d.freq_hz, d.snr_db, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  <#{:05x}> {} {}dBm",
                     d.freq_hz, d.snr_db, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

**JT9** and **JT65** are the same shape: `Decoder::<Jt9>` and
`Decoder::<Jt65>`, `SlotInput::f32` (or `i16`) of the 60 s period. JT9 at
`rx_freq_hz` also runs upstream's Rx-frequency pass within `tol_hz`
(default 50 Hz). Both return the 72-bit message as text. Since 0.13 neither reports the
all-zero codeword (`000AAA 000AAA RA90`), which a Reed-Solomon or Fano decode
of silence produces.

```rust
# #[cfg(all(feature = "jt9", feature = "jt65"))] {
use mfsk_core::decoder::{DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::{Jt65, Jt9};

let jt9_audio = mfsk_core::jt9::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
    .expect("pack + synth");
let mut jt9 = Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep));
assert!(!jt9.decode(&SlotInput::f32(&jt9_audio)).rows.is_empty(), "roundtrip must decode");

let jt65_audio = mfsk_core::jt65::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let mut jt65 = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)));
let result = jt65.decode(&SlotInput::f32(&jt65_audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in result.rows {
    println!("{:7.2} Hz  {:+.0} dB  {}", row.decoded.freq_hz, row.decoded.snr_db, row.decoded.text);
}
# }
```

The JT65 Chase search (`jt65::chase`, issue #169) is a faithful port of
WSJT-X's `ftrsdap` stochastic Chase decoder, magic numbers included. On the
AWGN sweep it moves the 50% crossing from −22.5 to −23.5 dB, at the cost of
up to `ChaseParams::max_trials` RS attempts per candidate that does not
decode at once.

**JT65 averaging** (`params.averaging`, `ndepth & 16`; `jt65::averaging`, a port
of `avg65` in `jt65_decode.f90`). A candidate the single period fails on is
kept: its period, DT, frequency and 63 × 64 symbol powers, up to 64 periods per
decoder. The saved periods of the same parity, with a DT within 0.2 s and a
frequency within `tol_hz` (default 50 Hz) of the new one, are summed, and with
two or more the sum is decoded as one period would be (the Chase search; the
probabilities are `s1/psum`, so the sum needs no rescaling). It needs
`SlotInput::period` on every call (without it nothing is averaged), and
`clear()` forgets the saved periods. On noise with σ = 2.0 no single period
decodes and the sixth summed one does
(`tests/decoder_depth.rs::jt65_averaging_decodes_what_no_single_period_does`).
Not ported: the JT65B/C smoothing loop (`ismo`) and `nflip`. `deep_search`
(`ndepth & 32`, the call-sign database correlation `hint65`) is carried and not
read.

**Decoding at a known alignment.** `wspr::SniperRequest`, `jt9::SniperRequest`,
`jt65::SniperRequest` and `q65::SniperRequest` stay public: a decode at a
`(start_sample, frequency)` the caller already has is not a search, and the
WSPR boards (`embedded-shared`) build on it. `SniperRequest::new(audio, rate,
start_sample, freq_hz)`; WSPR's `::baseband(idat, qdat, …)` does the same on a
baseband the caller already decimated, with `.drift()`, `.nblocks()`,
`.confirmed(&WsprCallsignTable)` and `.refine_drift()` — the knobs the scan's own passes set per
candidate (that is what the CoreS3 WSPR receiver drives from its own candidate
loop). JT65's takes `.chase(..)` or `.erasures(&[0, 8, 16, 24, 32])` (the last
of the two called wins).

**Q65** is ten sub-mode ZSTs, one `Decoder<Q65a30>` etc. each, with the
richest extras (`Q65Extras`):

| field | default | effect |
|---|---|---|
| `search` | — | `SearchTuning`, as above |
| `ap_hint` | none | free-form hint, as for the frame family. **Each candidate is tried without AP first**, then with the hint, as `q65_decode.f90`'s `ipass` loop does, so one hinted scan returns what a plain scan and a hinted scan returned between them (`Q65Result::ap` says which carried a decode; since #555) |
| `ap_list` | empty | candidate codewords (`Vec<[i32; 63]>`) for the AP-list decode, in place of the list the decoder builds from `station` and `qso` |
| `callers` | none | `Q65Callers`, the contest stations heard (`q65_hist2`); with `Contest::GridExchange` they join the list |
| `pileup` | `false` | Q65 Pileup (WSJT-X 3.2): a Pileup station sets the spare 78th payload bit to say it copied its correspondent's last transmission. `Q65Result::copied_last_tx` reports it (WSJT-X marks the line `#`), and `encode_channel_symbols_flagged` / `synthesize_standard_flagged_for` send it. With `pileup` an `ap_hint` naming both callsigns leaves the bit free instead of locking it to 0, so a flagged reply still matches; the `ap_list` templates carry the bit clear, as `q65_set_list.f90` builds them |
| `max_drift` | `0` | WSJT-X's Max Drift (0..50 bins). The sync search tries a linear tone drift of up to `max_drift` bins across the frame (`q65_ccf_22`) and the grid decode takes the found drift back out (`q65_loops`' `twkfreq`). It costs `2*max_drift+1` times the plain search per frequency bin; WSJT-X narrows its window to Rx ± F Tol while it is on, so narrow `band_hz` to match. Plain and `ap_hint` scans only |
| `fading` | none | `(FadingModel, b90_ts)`: the fast-fading metric with a caller-picked model, for Doppler-spread channels (microwave EME, ≥10 Hz spread) |

Which front end runs (`q65/decode_request.rs`; `.decode()` resolves
precedence as `ap_list > fading (+ ap_hint) > ap_hint > plain`; `ap_list` and
`fading` are mutually exclusive in the engine). Q65's `Decoder` has three
paths, chosen by the block:

| when | strategy | how | threshold gain |
|---|---|---|---|
| default scan | `(Δf,Δt,b90)` grid + Lorentzian fading BP, depth as [§2.2](#22-decodeparams-and-depth) | nothing set | WSJT-X-faithful default |
| known callsign(s) or report, terrestrial | AP-hint BP | `extras.ap_hint` | ~2 dB |
| Doppler-spread channel | fast-fading metric + BP | `extras.fading` | 5–8 dB on spread channels |
| known call pair, no QSO context | AP-list template matching | `extras.ap_list` | ~3 dB |
| known call pair and an Rx frequency (WSJT-X's q3) | 85-symbol sync of every list message near the Rx frequency, then list decode | `station` (+ `qso`) and `rx_freq_hz` (+ `tol_hz`), or `extras.ap_list` with `rx_freq_hz` | `q65sim` Q65-30A, 20 files a level at −24 / −26 / −28 / −30 dB: 20 / 20 / 7 / 2, `jt9 -3 -d 1` the same on the same files |
| weak / ionoscatter signal spanning several periods | running average of the symbol spectra (`averaging`) | `params.averaging = true` and a consecutive `SlotInput::period` | recovers signals no single-period strategy can |

**Q65 averaging** is decoder state, not a separate request (0.12's
`MultiPeriodRequest`): with `averaging` on, each period is added to the
running average (`s1a`, weight `1/min(navg, 4)`, 3-stage cascade; the last
`decoder::MAX_AVERAGED_PERIODS` = 8 periods are kept, older ones weigh
`0.75^8` ≈ 10 % or less), and a gap in `period`, or `None`, restarts it. It
yields one result a period; a q3 hit skips that period's ladder (upstream
goes on to its candidate loop). The averaged path takes the search tuning
and the AP list / q3, not `ap_hint`, `fading`, `pileup` or `max_drift`.

**Q65 q3 list decoding.** With `rx_freq_hz` and a list (built from `station`
and `qso`, or `extras.ap_list`), `tol_hz` (default 10 Hz, the `jt9` CLI's) is
WSJT-X's q3 decode: the 85-symbol sync of every list message within F Tol of
the Rx frequency (`q65_ccf_85`), then the list decode with the fast-fading
metric over the `b90` sweep (`q65_dec_q3`). It runs first, and the scan runs
after it for the rest of the band. At `max_drift` 50, when nothing decoded at
the Rx frequency, it runs again on spectra with the drift found there taken
out (the "w3sz" stage 5). **Not a 1:1 port everywhere an `ap_list` appears**
(issue #522): upstream's list decode only ever runs as q3, gated on the Rx
frequency, so an `extras.ap_list` without `rx_freq_hz` is this crate's own
per-candidate AWGN-metric template match with no upstream counterpart — a
deliberate extension, not a fidelity gap.

`q65::Q65History` is WSJT-X's `q65_hist`, held by the application:
`.record(&result)` after each decode (it keeps the last 100), and
`.lookup(rx_freq_hz)` returns the DX call — plus the grid when the message
carries one — from the most recent decode within 10 Hz. WSJT-X does this
on a manual Decode Again with no DX call entered, to build the full-AP
list (`standard_qso_codewords`, to pass as `extras.ap_list`) without the operator typing the call.
`q65::Q65Callers` and `contest_codewords` are the contest-mode variant
(`q65_hist2` / `q65_set_list2`): up to 50 stations that called with a
grid, kept by the application (`record(freq, msg, now)`, `expire(now)`),
and a full-AP list of every `MyCall Caller Grid` / `R Grid` / `RRR` /
`RR73` / `73` with the 78th bit clear and set. What each front end actually
does, and why the default scan is not the plain Bessel path, is in
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for).

**Q65 time window and EME delay.** `default_search_params()` searches
-1.0 .. +1.0 s around the nominal start, as WSJT-X's GUI does
(`q65.f90`'s `lag1`/`lag2`). `eme_delay` is its "Decode at 52 s" EME delay:
the late edge moves to +5.5 s (+4.0 s on Q65-15) for the Earth-Moon-Earth
round trip. `dt_sec` is measured from the nominal start.

```rust
# #[cfg(feature = "q65")] {
use mfsk_core::decoder::{DecodeParams, Decoder, Depth};
use mfsk_core::q65::Q65a30;

// A Q65-30A station in a QSO: the Rx frequency and tolerance gate the q3
// list decode, and averaging needs `SlotInput::period` on every call.
let params = DecodeParams::for_band((200.0, 3_000.0))
    .depth(Depth::Deep)
    .rx_freq(1_000.0)
    .tol(10.0)
    .station("K1ABC", "FN42")
    .qso("JA1XYZ", "PM95", mfsk_core::decoder::QsoProgress::Report)
    .averaging(true);
let mut decoder = Decoder::<Q65a30>::new(params);
decoder.extras_mut().max_drift = 0;
assert!(decoder.params().averaging);
# }
```

**`dt_sec`, `SearchParams` and `SyncCandidate`.** For WSPR, JT9, JT65 and
Q65 the result's `dt_sec` runs from the **nominal start** — the mode's
`tx_start_offset_s` (WSPR: its fixed 1.0 s `TX_START_OFFSET_S`) — and is
signed, so a frame that begins early reads negative (#397; it was measured
from the buffer start for Q65 and JT65, and a 0.5 s early frame read −0.013 s
and +0.442 s). `to_decoded` on `Q65Result` / `Jt65Result` / `Jt9Result` reads
that field. The scan modes share one coarse-search vocabulary in
`engine::search` (#394): `SearchParams { freq_min_hz, freq_max_hz,
time_tolerance_early_sec, time_tolerance_late_sec, score_threshold,
max_candidates }` (the window is an early/late pair in **seconds** because
Q65's is asymmetric; `SearchParams::symmetric(..)` sets both) and
`SyncCandidate { start_sample, freq_hz, score }`, where `freq_hz` is tone 0
and `.dt_sec(nominal, rate)` converts. There is no `SearchParams::default()`:
the defaults are mode-specific data, so each mode has
`search::default_search_params()` (Q65: 200-3000 Hz, ±1.0 s, 8 candidates,
threshold 0.1) — that is what `SearchTuning` overrides. FT8, FT4 and FST4 keep
`engine::sync::SyncCandidate`, which carries `dt_sec` instead of `start_sample`,
on purpose.

**uvpacket** has its own transmitter and receiver (`uvpacket::tx`,
`uvpacket::rx`), outside `Decoder`: a non-WSJT applied example that reuses only
the FEC mother code. Full account in [`UVPACKET.md`](UVPACKET.md).

**MSK144** is outside `Decoder` too, by design: it is not FSK, and
`msk144::decode::decode_slot` bypasses `engine::pipeline`. It scans the whole
T/R period for pings.

**JTTY** (WSJT-X 3.2.0-rc1's mode for weak-signal keyboard chat; a port of
`lib/jtty/`, tracked in #477 and `docs/notes/JTTY_UPSTREAM.md`) is the one
mode here with **no slot**: 4-GFSK at 31.25 baud, 1.888 s frames (59
symbols: 13 sync + 46 data) that can start at any moment, a message that is
several frames long and is put together by the receiver. From C, Kotlin and
Swift it is a receiver handle: `mfsk_jtty_*`, `MfskJttyReceiver` and
`JttyReceiver`, in [`BINDINGS.md` §2.8.1](BINDINGS.md#281-jtty--a-receiver-handle-instead-of-a-slot-call). So it is outside `Protocol` and `PROTOCOLS`, like
MSK144, and its receiver is *incremental*: `jtty::rx::Stream` takes audio in
chunks of any size and hands each message update to a callback from inside
`push`, on the caller's thread (and rayon's pool, under `parallel`; the
result does not depend on the thread count). The `Receiver` behind it holds
only immutable tables and is shared through an `Arc`, so one `Stream` per
audio channel costs a buffer each.

```rust
# #[cfg(all(feature = "jtty", feature = "fft-rustfft"))] {
use std::sync::Arc;
use mfsk_core::jtty::rx::{Params, Receiver, Stream};
use mfsk_core::jtty::source::{Atom, CallAction};
use mfsk_core::jtty::tx;

// one frame, "CQ K1ABC"; the last frame of a transmission carries end-of-message
let atoms = [Atom::call(CallAction::Cq, "K1ABC")];
let tones = tx::tones(&atoms).expect("encodable");
let mut audio: Vec<i16> = vec![0; 12_000];                 // one second of lead-in
audio.extend(tx::synth_f32(&tones, 1500.0, 3000.0).iter().map(|&x| x as i16));
audio.extend(std::iter::repeat(0).take(4 * 12_000));       // and room to finish

let mut stream = Stream::new(Arc::new(Receiver::new()), Params::default());
let mut updates = Vec::new();
for chunk in audio.chunks(4096) {                          // any chunk size
    stream.push(chunk, &mut |u| updates.push(u));
}
stream.finish(&mut |u| updates.push(u));                   // flush what is still open
assert!(updates.iter().any(|u| u.text.contains("CQ K1ABC")));
# }
```

**Sending** starts from text. `jtty::pack::pack(text, profile)` is upstream's `pack_jtty`:
it normalises the text (upper case, `~` and NUL are spaces, spaces collapse, anything
outside the 64-character alphabet becomes `#`) and picks the **fewest frames** with a
dynamic program — a callsign with its action (`CQ K1ABC CQ`), a control phrase (`TU`), a
number, a grid, `599 <location>`, or a Field Day `3A EMA` is one frame, anything else five
characters a frame. `ExchangeProfile::RttyRoundup` adds serial-number and state candidates
(`599 5` is sent as `599 005`); the other profiles pack alike. It refuses rather than
truncates: over 80 characters, over 16 frames, or a serial that does not fit is a
`PackError`. The result is what `jtty::tx::tones` and `synth_f32` take. It is tested
against upstream's own `pack_jtty` on 3 525 messages × exchange profiles
(`tests/jtty_pack.rs`), the same frames every time. What
WSJT-X's GUI wraps around it — F-key templates, the N1MM tags, deciding the profile from
the operating activity — is host policy and is not in the library (#463 draws the same
line for the QSO-state decoders).

```rust
# #[cfg(feature = "jtty")] {
use mfsk_core::jtty::pack::{self, ExchangeProfile};

let tones = pack::tones("cq k1abc cq", ExchangeProfile::Unknown)
    .expect("packs")
    .expect("not empty");
assert_eq!(tones.len(), 59);                       // one frame
assert_eq!(pack::pack("HELLO WORLD", ExchangeProfile::Unknown).unwrap().len(), 3);
# }
```

`Params` carries what `rjtty` takes: the operator's receive frequency and
±tolerance (channel 0), the sync floor `smin_db`, and the band channels 1
and 2 watch (they look 1350 ± 150 Hz and 1650 ± 150 Hz for stations off the
operator's frequency). `.subtract` (on) takes each decoded frame off the
signal and re-searches the windows before it — that is what lets a weak
station under a strong one through; switching it off is a single-signal
receiver. `Receiver::scan` / `scan_messages` are the one-shot forms for a
whole recording, and give exactly what streaming the same samples does.
`MessageUpdate` is emitted each time a message grows and once more when it
completes (`complete`) or is given up on; `id` is stable for the life of the
message. Measured on the reference machine: 11.7 ms per 0.47 s window on one
thread (43x real time). Known limit, shared with upstream's receiver (#488):
a frequency drift beyond about 12-16 Hz/s is not tracked, which a low-orbit
satellite pass can exceed near closest approach.

**SNR comparability.** `Jt65Result::snr_db` and `Q65`'s are converted
to WSJT-X's 2500 Hz reference bandwidth. `Jt9Result::snr_db` is
**not** — JT9's multi-stage AGC/IFFT/coherent-sum pipeline doesn't
reduce to a simple bandwidth offset, so it is relative-only: compare
JT9 decodes against each other, not against other protocols.
`Wspr`'s is a `wsprd`-calibrated candidate SNR, the same figure
`wsprd` itself prints next to a spot.

### 2.6 Message acceptance

Everything below this point in the stack is error *detection*: the
LDPC parity check, then the CRC. Neither says anything about whether
what comes out is a message someone sent. A CRC-14 false positive is a
codeword the decoder converged on that is not the transmitted one, so
its 77 information bits are effectively uniform, and better than half
of those unpack to a syntactically valid message.

`MessageCodec::is_plausible` is what refuses them. **It has no WSJT-X
counterpart** — `ft8b.f90` accepts on `nbadcrc` and
`nharderrors <= 36`, and this crate's own ceiling is that same 36 — so
it is a judgement call rather than a port, and the judgement belongs
to whoever knows the band.

**It judges fields, not text.** `unpack` returns `Wsjt77Fields`, the
decoded message as its fields, and the verdict reads the *callsign*
fields. Asking anything of the rendered string instead means splitting
it back into tokens and guessing which ones were callsigns: judging
`JA1ABC 3Y0Z 6A EMA` that way tests `6A` and `EMA` against a callsign
grammar, and judging `JA1ABC PM95 20` tests `PM95` and `20`. The
verdict did exactly that until issue #383, and refused ARRL RTTY
Roundup, free text and telemetry outright for as long as it existed.
Everything else a text rule might check — the ARRL section index, the
grid bounds, the RTTY exchange range — was already enforced during
unpacking.

Two of the types have no callsign to check, and are handled by name:
free text and telemetry carry no redundancy at all (nearly every bit
pattern is a valid one), so they are accepted; the EU VHF contest
carries two hashes and nothing else, so it is accepted only when one
of them resolves.

One extra, `filter: MessageFilter`, on `Ft8Extras`, `Ft4Extras` and
`Fst4Extras` — **every frame-family mode** — with four values:

| value | verdict |
|---|---|
| `MessageFilter::Default` | the codec's verdict where the protocol runs it by default (FT8 and FT4, not FST4), none otherwise |
| `MessageFilter::Codec` | the codec's verdict and nothing else — the one-line way to get it on a protocol that does not run it by default |
| `MessageFilter::AlsoAccept(f)` | the codec's verdict **plus** what `f` accepts; widens the verdict and can only add |
| `MessageFilter::Only(f)` | replaces the verdict with `f` outright |

`f` is a `fn(&Wsjt77Fields) -> bool` — a function pointer, so the extras
stay `Clone` and `'static` (a closure that captures nothing coerces to one).

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, MessageFilter, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::Wsjt77Fields;

/// Whatever the deployment knows and the ITU allowlist does not. The
/// message is seen as fields, so `callsigns()` is exactly the callsign
/// fields — never a grid or a report.
fn special_event_only(m: &Wsjt77Fields) -> bool {
    m.callsigns().all(|c| c.starts_with("8J"))
}

let audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let params = DecodeParams::for_band((200.0, 3_000.0));

// The codec verdict, plus callsigns it does not know about.
let mut widened = Decoder::<Ft8>::new(params.clone());
widened.extras_mut().filter = MessageFilter::AlsoAccept(special_event_only);

// No opinion at all — every CRC-passing message, phantoms included.
// This is WSJT-X's own acceptance rule with nothing on top.
let mut unfiltered = Decoder::<Ft8>::new(params);
unfiltered.extras_mut().filter = MessageFilter::Only(|_| true);

// Silence carries neither real signals nor CRC survivors, so even the
// filterless run comes back empty.
assert!(widened.decode(&SlotInput::i16(&audio)).rows.is_empty());
assert!(unfiltered.decode(&SlotInput::i16(&audio)).rows.is_empty());
```

`MessageFilter::Only(f)` replaces a verdict that removes roughly two thirds
of the CRC survivors that reach it, so a permissive `f` will surface phantom
rows.

**FT8 also drops `/R` and `TU; ` messages, whatever the policy.** Since
#439 FT8 does what `ft8b.f90` (WSJT-X 3.0 onward) does right after the
CRC: with no contest active, a standard or RTTY Roundup message carrying
`/R` or starting `TU; ` is discarded and the pass moves on. It sits
before the policy, so `MessageFilter::Only(|_| true)` does not bring those
rows back; a `contest` other than `Contest::None` in the block does, and is the right setting for a
contest, where `CALL1/R CALL2` and `TU; CALL1 CALL2` are real traffic.

**On by default for FT8 and FT4, and the reason is subtraction.** A
wrong decode is not a cosmetic error on a path that subtracts what it
accepts: `SicRounds` and `SicEarly` remove the decoded
waveform from the audio before looking again. Measured on
`qso3_busy.wav`, with the verdict off `SicEarly` accepts the
phantom `CQ G47OXF RD84`, subtracts it, and loses the real
`CQ EA2BFM IN83` underneath — 18/18 becomes 17/18. On the single-pass
path the same verdict removes two garbage rows at `max_cand = 200` and
**nothing at all** at the depth that ships on embedded hardware.

FT4 shares that CRC-14 and that SIC path, and joined it once the same
measurement existed for it — 720 `ft4sim` slots across the threshold
window (−21..−13 dB, four ITU-R channels): phantom rows **7 → 2**,
golden rows **353 → 354**, and all four 50 %-crossing SNRs unchanged
to 0.00 dB. The one recall cell that moves moves *up*, because a
rejection lets the candidate ladder keep going. What that corpus
cannot test is the allowlist's own risk — every slot in it carries the
same callsign — so a deployment seeing unusual prefixes widens it with
`MessageFilter::AlsoAccept`.

**FST4 leaves it off**, and not for want of measuring: CRC-24 puts its
false-positive rate 512x below the other two, so there is little for
the verdict to remove and the recall it could cost is the same.

**Zero-cost when unused.** The policy is a type parameter of the engine,
not a `&dyn Fn` like the row callback and the budget: each `MessageFilter`
variant selects its own monomorphised copy, `Default` carries a zero-sized
`DefaultPolicy`, and for a protocol that does not filter by default the
message is not even decoded — both conditions are compile-time constants
inside that copy. The row callback fires once per decode; this one fires once per candidate
that reaches the message stage, which is why it is worth the type parameter.

---

### 2.7 Wideband IQ input

`mfsk_core::iq` takes a wideband complex-IQ stream (an SDR, an IQ recording)
where every decoder above takes 12 kHz real audio. It is library scope: DSP
plus decoding, no device handling, no UI, no spot upload. What it does *not*
do is find signals: the caller says which dial frequency carries which mode,
and the decoders search the channel's audio themselves, over the band of
their `DecodeParams`.

**One channel: `IqToAudio`.** The audio a transceiver's USB output would have
carried for a dial frequency, from IQ at any integer rate of 12 kHz or more:

```rust
use mfsk_core::iq::{IqError, IqSampleFormat, IqStream, IqToAudio};

let stream = IqStream::new(768_000, 14_200_000.0, IqSampleFormat::Cf32);
// 14.290 MHz is 90 kHz above the centre: inside the band, clear of DC.
let mut ch = IqToAudio::new(stream, 14_290_000.0).unwrap();
let mut audio = Vec::new();
ch.push_cf32(&[0.0; 2 * 1024], &mut audio);   // interleaved I/Q; appends 12 kHz f32 audio
assert_eq!(ch.samples_in(), 1024);
// DC inside the channel's 0-6 kHz window is refused.
assert_eq!(IqToAudio::new(stream, 14_199_000.0).err(), Some(IqError::TooCloseToDc));
```

Placement is checked up front: the channel's 0-6 kHz audio window must lie
inside `±Fs/2` (`IqError::OutsideBand`), must not contain the stream's DC
(`TooCloseToDc`), and the rate must reach 12 kHz through a small rational
factor (`UnsupportedRate` for, say, 999 983 Hz; `RateTooLow` under 12 kHz).
The path is: mix the channel's audio 3 kHz to DC, a cascade of short FIR
decimators down to 24-48 kS/s, a polyphase `L/M` resampler to exactly 12 kHz
complex, one sharp low-pass there, shift back up, take the real part.

**Usable audio starts near 200 Hz.** Taking the real part folds the sideband
*below* the dial onto the wanted one, so the low-pass has to be sharp at audio
0, not at Nyquist: it passes audio 200-5800 Hz and stops at -200 Hz. A signal
at 0-200 Hz is attenuated, not decoded reliably. An SSB receiver's own filter
starts about there; a decoder given complex input directly would not have the
limit, and would be a much larger change than this front end (issue #534).

**N channels, on UTC: `IqReceiver`.** (Needs an FFT backend.) It does
channelization and slot cutting only, and **decodes nothing**: the caller says
which dial frequency carries which mode, tells it what UTC the stream is at,
and pulls each completed slot out as an owned `CompletedSlot`. Decoding is the
caller's, with one `AnyDecoder` per channel — so each channel has its own
options and its own callsign table, and the decode can run on any thread
(a slot is owned and `Send`).

```rust
# #[cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))] {
use std::collections::HashMap;
use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};

let mut rx = IqReceiver::new(IqStream::new(768_000, 14_200_000.0, IqSampleFormat::Cf32));
let ft8 = rx.add_channel(14_074_000.0, Mode::Ft8).unwrap(); // Err if DC is in its window or it is out of band
let ft4 = rx.add_channel(14_080_000.0, Mode::Ft4).unwrap();
// Many channels: share one polyphase filter bank instead (see "Two channelizers" below):
//   IqReceiver::with_channelizer(stream, Channelizer::Pfb)?
let mut decoders = HashMap::from([
    (ft8, AnyDecoder::with_defaults(Mode::Ft8)), // per channel: its own options and hash table
    (ft4, AnyDecoder::with_defaults(Mode::Ft4)),
]);

// rx.set_time(utc_ns, at_sample);   // as often as there is a reading
let mut slots = Vec::new();
rx.push_cf32(&vec![0.0f32; 2 * 4096], &mut slots); // interleaved I/Q; also push_cs16 / push_bytes
for slot in slots {
    let out = decoders.get_mut(&slot.channel).unwrap().decode(&slot.input());
    for d in &out.rows {
        println!("{:.1} Hz  {}", slot.abs_freq_hz(d.freq_hz), d.text);
    }
}
let report = rx.retune(14_201_000.0); // RetuneReport { paused, resumed }
assert!(report.paused.is_empty());
rx.gap(1_000); // a hole in the samples
# }
```

`Mode` is `registry::Mode` (it replaces 0.12's `iq::IqMode`): FT8, FT4, the
five FST4 periods, WSPR, JT9, JT65 and the ten Q65 sub-modes, as the build
has them. A `CompletedSlot` carries `channel`, `mode`, `dial_hz`, `period`
(the slot's index on the grid), `start_sample`, `utc_ns` (with a clock set),
and `audio: Vec<f32>` — 12 kHz from the slot's nominal start, so `dt` reads as
it does on a WAV. `slot.input()` is the `SlotInput` (audio plus `period`, so
averaging and a7 see consecutive slots), and `slot.abs_freq_hz(audio_hz)` is
dial plus audio frequency. The public IQ types are `#[non_exhaustive]`.

*Time.* The sample count is the clock; no time source is read unless you give
one. A channel's audio index `k` is `k/12000` s after sample 0, and slot `j` of
a mode with period `T` covers UTC `[j·T, (j+1)·T)`, computed in integers.
`set_time(utc_ns, at_sample)` takes observations of the clock (nanoseconds
since the Unix epoch; `at_sample` counts input samples, as `samples_in()`
does) and follows them at a bounded rate through `slotgrid::SampleClock`, so a
drifting crystal or host clock moves slot boundaries by milliseconds and no
slot is lost. It returns the `ClockChange` the observation caused: `First`
(the clock is set), `Slewed { by_ns }` (within bounds, moved at most 400 ppm of
the time since the last observation, 6 ms across an FT8 slot) or `Stepped {
by_ns }` (more than a second away: a clock that was set, not one that
drifted; the open slots straddle the jump and are dropped). With no
observation the grid free-runs from sample 0, right for replaying a recording.
A slot always starts on its own boundary: when the stream is slightly faster
than the clock its first samples are the previous slot's last. A slot is
completed once all of it has arrived; the partial slot the stream opened in
the middle of is not. A slot's last audio sample comes out a few filter
lengths after the last IQ sample that carries it, so a recording needs a
moment of padding after its end, as a live stream has.

`mfsk_core::slotgrid` is that arithmetic on its own, in integers with no
`std`, allocation or atomics, so it also fits an embedded board and the C
ABI's audio stream: `SlotGrid::new(period_ns, rate_hz)` (`start_of`,
`next_start`, `follow`), `SampleClock` (`observe`, `utc_of`,
`with_max_slew_ppm`, `with_step_ns`) and `SlotCutter<T>` (feed it samples at
the grid's rate and it hands back each slot on its own boundary, following the
clock's anchor). A simulated day at +13 ppm with 3–11 ms of jitter on the
observations loses no slot (`slotgrid::tests`).

*Retune and gaps.* `retune(center_hz)` moves every channel that still fits the
new centre and **pauses** the rest, returning a `RetuneReport { paused,
resumed }`; a paused channel keeps its dial, the caller keeps its decoder and
its tables, and the channel resumes when a later retune brings it back inside
the band (`channel_state(id)`: `Active` or `Paused(IqError)`). `retune` and
`gap(lost)` both drop every open slot — audio that straddles a change of
centre or a hole in the samples is not a slot — and the sample clock
continues.

*Threading.* Slots are returned from the `push_*` call that completes them and
decoded wherever the caller likes — hundreds of milliseconds on a busy FT8
band, so a caller that cannot block hands them to a worker. Each slot is scaled
to a fixed RMS before the decoders see it; they are scale-free and the IQ's own
level is not something to inherit, and a silent or NaN slot is not returned.

*Formats.* `Cf32`, `Cs16` typed or as bytes; `Cs8` (HackRF), `Cu8` (RTL-SDR,
128 = zero) and `Cs24` as byte streams through `push_bytes`, a sample split
across calls carried over.

*Selectivity.* 120 dB outside the channel's audio −200…6200 Hz, anywhere in
the band (`iq::REJECT_DB`): every filter is a Kaiser design for it, which is
the noise floor of an ideal 16-bit ADC in 2500 Hz at 768 kS/s. Measured worst
over ~1 150 interferer positions per rate, aimed at every decimator's alias
edges as well: −124.0 dB at 192 kS/s, −121.0 at 768 k, −122.0 at 2.4 M on the
`Direct` path; −124.3…−124.4 dB on the `Pfb` path at five rates (window placed
across a whole sub-band, ~1 000 positions each). `tests/iq_front_end.rs` and
`iq::pfb`'s unit tests assert it.

*Two channelizers, one choice.* `IqReceiver::new` uses `Channelizer::Direct`:
one `IqToAudio` per channel, each mixing and decimating from the input rate,
so cost is linear in channels. `IqReceiver::with_channelizer(stream,
Channelizer::Pfb)` shares one polyphase filter bank (`iq::PfbChannelizer`)
among all channels instead: 2x oversampled, sub-bands ~24 kHz apart, each
channel's back end an `IqToAudio` on the sub-band nearest its window. Both give
the decoders the same audio at the same selectivity: every IQ decode test runs
through each and gets the WAV path's set. Measured, one thread, release, `Cf32`
in, % of a core:

| channels | 768 kS/s Direct | 768 kS/s Pfb | 2.4 MS/s Direct | 2.4 MS/s Pfb |
|---:|---:|---:|---:|---:|
| 1 | 0.92 | 2.66 | 2.38 | 9.08 |
| 4 | 3.68 | 3.34 | 9.51 | 9.78 |
| 8 | 7.40 | 4.32 | 19.13 | 10.77 |
| 32 | 29.89 | 10.22 | 76.51 | 16.64 |
| 128 | — | 34.94 | — | 41.15 |

Break-even is about four channels at either rate. An amateur band's handful
of modes fits `Direct`; a skimmer across a band wants `Pfb`. The bank needs a
rate of 40 kS/s or more (`UnsupportedRate` otherwise). Its design, and why it
is a polyphase bank rather than an FFT with a mask (which leaked −71 dB between
bins), is `docs/notes/IQ_CHANNELIZER.md`.

*Evidence.* `tests/iq_front_end.rs` and `tests/iq_receiver.rs` place real
recordings as double-sideband IQ (so a leaking lower sideband would show as
phantoms) at 48 k, 192 k, 250 k, 768 k and 2.4 MS/s, I/Q swapped and off DC, and
require the WAV path's own decode set: 16/16 and 14/14 FT8, 11/11 FT4, with no
phantoms, frequency within 2 Hz and DT within 0.05 s. `tests/iq_receiver_modes.rs`
does the same for WSPR (9/9), JT9 (5/5), JT65 and Q65-120D / -300A. The C ABI is
[`mfsk_iq_*`](BINDINGS.md#282-wideband-iq--a-receiver-handle-for-an-sdr-stream).

### 2.8 Non-standard, compound and suffixed callsigns

A 77-bit message holds at most one full non-standard callsign; the others travel as hashes. This crate unpacks every such message
exactly as WSJT-X v3.2.0-rc1 does. Issue #568 checked that on the 168 messages that WS (the former WSJT-X Improved) composes for QSOs with
non-standard, compound and suffixed calls, in two table states (`tests/ws_77bit_extension.rs`). What a caller should expect from this traffic:

- **`<...>` means "a hash this decoder has not learnt", not an error.** A bystander shows it for the hashed field of a message, and a decoder shows
  it before it has seen the call in full (a type 4 message, or a CQ). A row never carries a guess: `RowDetail::hash_resolved` is set when the text
  needed the table. Keep one `Decoder` per mode for the whole session, so a call heard in one period resolves in the next.
- **The call in a report may be the base call.** Some programs send `DG2YCB` in the report and R-report of a QSO with `DG2YCB/QRP`; the first
  message and the RR73/73 carry the full call. Nothing in a message says the two are one station. Match a partner by its base call, as WSJT-X
  does, and take the call you log from a message that carried it in full.
- **Type 4 has no report and no grid.** A message with a plain non-standard call can be `<HIS> MY`, `... RRR`, `... RR73` or `... 73`.
  WSJT-X sends a report when the two calls fit a standard message: two standard calls (`W1XYZ/P DG2YCB -05` is type 2), or a standard call without a
  suffix and the other call as a 22-bit hash (`W9XYZ <PJ4/K1ABC> -11`, type 1). It cannot when one call is plain non-standard, or when a hash is combined
  with a `/R` or `/P` call (`W1XYZ/P <DG2YCB/MM> -05`): only type 4 holds those, type 4 has no report, and rc1 refuses to pack them. The format can carry such
  a report as type 1 with both calls hashed (`<W250USA> <DG123YCB> -05`; rc1 packs and unpacks it) once both calls are known. WS composes such messages; WSJT-X does not.
- **A hash resolves to whatever stored call has that hash.** A 12-bit hash (type 4) has 4,096 values, so a decoder that holds 500 distinct calls
  resolves about 11% of the unknown 12-bit hashes it meets to some stored call; a 22-bit hash with 1,000 stored calls, about 0.024% (arithmetic from
  the table sizes, not a measurement). Treat a resolved `<CALL>` as evidence; to log a contact, wait for the call in full.
- **`/P` and `/R` are part of the hashed call.** A call is hashed and learnt with its suffix, as in WSJT-X: `learn_callsign("W1XYZ/P")`, not `"W1XYZ"`.
- **QSO sequencing is not in this crate.** WSJT-X's own handling of such messages (for instance, a message with no third word counting as a
  report of 0) is application logic and is not ported.

Details, tables and the evidence: [`docs/notes/WS_77BIT_EXTENSION.md`](../notes/WS_77BIT_EXTENSION.md).

## 3. Protocols

### 3.1 Generic vs bespoke, per protocol

The map to read first. Each cell says whether that layer is **shared**
(reused verbatim from the generic core) or **own** (code in the
protocol's own module).

| Protocol | FEC codec | Message codec | Sync mode | Decode entry point |
|----------|-----------|---------------|-----------|--------------------|
| **FT8**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 3×Costas-7 | `Decoder<Ft8>`, dispatching to FT8's own `ft8::decode_block` engine [^ft8] |
| **FT4**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 4×Costas-4 | `Decoder<Ft4>` over the generic `engine::pipeline` |
| **FST4** | shared `Ldpc240_101` | shared `Wsjt77Message` (77-bit) | `Block` — 5×Costas-8 | `Decoder<P>` over the generic `engine::pipeline` |
| **WSPR** | own `ConvFano` (conv r=½ K=32 + Fano) | own `Wspr50Message` (50-bit) | own `Interleaved` [^wspr] | `Decoder<Wspr>` over the bespoke `wspr::decode` |
| **JT9**  | own `ConvFano232` (conv, 206-bit framing) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | `Decoder<Jt9>` over the bespoke `jt9` entry |
| **JT65** | own `Rs63_12` (RS GF(2⁶), erasure-aware) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | `Decoder<Jt65>` over the bespoke `jt65` entry |
| **Q65**  | own `Q65Fec` + QRA codec over GF(64) [^q65] | own `Q65Message` (77-bit) | `Block` | `Decoder<P>` over the bespoke `q65::rx` |
| **uvpacket** | shared `Ldpc240_101` (punctured) | own `UvPacketRawMessage` (byte-pipe) | `Block` — Costas-4 [^uv] | bespoke `uvpacket::rx` |
| **MSK144** | shared `Ldpc128_90` + CRC-13 | shared `msg::wsjt77` (77-bit) | **none — opts out of `Protocol`** [^msk] | bespoke `msk144::decode::decode_slot` |
| **JTTY** | own tail-biting conv r=½ K=10 (`jtty::tbcc`, list-WAVA in `jtty::trellis`) + CRC-12 | own 32-bit `jtty::source` grammar (`Atom`), several frames a message | **none — opts out of `Protocol`** [^jtty]; 13-tone sync at the head of every frame | bespoke `jtty::rx::{Receiver, Stream}` |

What the table makes visible:

- **FT8 / FT4 / FST4** are the cheap additions — LDPC + 77-bit message
  + block-Costas sync, so almost everything is shared.
- **WSPR** swaps all three of *FEC family*, *message width* and *sync
  mode* independently — the proof that those axes are orthogonal.
- **Q65** adds a third FEC family (non-binary QRA over GF(64)), ten
  sub-modes from one macro, and a family of decoder strategies
  (§3.4), all inside the same `Protocol` super-trait.
- **uvpacket** is a non-WSJT applied example: it reuses only the FEC
  mother code and bypasses the generic TX/RX pipeline. Full account in
  [`UVPACKET.md`](UVPACKET.md).
- **MSK144** opts out of the trait surface entirely, yet still reuses
  the FEC and message layers.
- **JTTY** opts out too, and shares nothing above the DSP: its FEC,
  message grammar and receiver are its own (`jtty::*`), and it is the one
  mode whose receiver is incremental — [§2.5](#25-extras-and-the-protocols-outside-decoder).

> This table is the source of truth that
> `mfsk-core/tests/common_selftest.rs`'s code-sharing ratchet, the
> `README.md` sharing paragraph and `lib.rs`'s own docs all trace back
> to. Change it and those change with it.

[^ft8]: FT8 is driven by the same `Decoder<P>` as FT4/FST4, but
    internally routes through its own hand-tuned `ft8::decode_block`
    engine (host + embedded shared) rather than `engine::pipeline`;
    see [§6](#6-engine-primitives).

[^wspr]: `SyncMode::Interleaved` — the lower bit of every channel
    symbol carries one bit of a fixed 162-bit sync vector, so sync is
    not a block of Costas arrays. WSPR is the only user of this variant.

[^q65]: `Q65Fec::decode_soft` returns `None` **by design** — the real
    decode runs over GF(64) probability vectors via the QRA codec
    (`fec::qra` + `fec::qra15_65_64`), not bit-LLRs. `NTONES = 65` with
    `BITS_PER_SYMBOL = 6` (tone 0 is a reserved sync tone) is the case
    that loosened the `GRAY_MAP` length contract to
    `[2^BITS_PER_SYMBOL, NTONES]`.

[^uv]: uvpacket bypasses the generic pipeline, so several of its
    `ModulationParams` constants are decorative — present only to
    satisfy the trait and the invariant test.

[^msk]: MSK144 (issue #25) is continuous-phase binary MSK sent as
    offset-QPSK, and repeats an 864-sample frame through the whole T/R
    period rather than sitting at a known offset in a fixed slot — so
    neither `ModulationParams`/`FrameLayout` nor `engine::pipeline`
    fit, and no ZST implements `Protocol` for it. Its own
    `msk144::decode::decode_slot` driver scans for pings via
    `msk144::spd`/`msk144::sync`. It still reuses the 77-bit
    `msg::wsjt77` codec and the generic LDPC BP/OSD engine
    (`fec::ldpc_128_90`). Golden-WAV recall vs WSJT-X
    `samples/MSK144/*.wav` is 3/3 (`tests/msk144_wsjtx_samples.rs`).

[^jtty]: JTTY (WSJT-X 3.2.0-rc1, #477) is 4-GFSK at 31.25 baud (`NSPS`
    384, GFSK BT 2, modulation index 1, so tone spacing = baud) in
    self-contained 1.888 s frames — 13 sync + 46 data tones — that may
    start at any moment. There is no T/R slot, so `ModulationParams` /
    `FrameLayout` have nothing to say and no ZST implements `Protocol`
    for it; it is not in `PROTOCOLS` (only `mfsk-ffi` gives it a mode
    number, `MFSK_MODE_JTTY`). `jtty::rx::Receiver` decodes a window and
    `Stream` a live feed, both port `jtty_mdecode` (signal subtraction,
    retro re-sweep, message assembly included), and `jtty::pack` /
    `jtty::tx` send text. Recall against upstream's `rjtty`: identical in
    all 18 cells of an AWGN/fading sweep; tier C 50 % crossing −16.20 dB
    (AWGN) and −15.25 dB (moderate fading), 0 unexpected decodes in 360
    files (`tests/jtty_sweep.rs`).

### 3.2 Geometry

24 wired ZSTs: 20 WSJT-family protocols and sub-modes plus 4
`uvpacket` sub-modes. MSK144 and JTTY are listed last for reference but
are **not** among them — neither implements `Protocol`, so they appear in
neither the registry nor `tests/protocol_invariants.rs`.

| Protocol   | Slot   | Tones | Symbols | Tone Δf    | FEC              | Msg   | Sync       | Notes |
|------------|--------|-------|---------|------------|------------------|-------|------------|--------|
| FT8        | 15 s   | 8     | 79      | 6.25 Hz    | LDPC(174, 91)    | 77 b  | 3×Costas-7 | |
| FT4        | 7.5 s  | 4     | 103     | 20.833 Hz  | LDPC(174, 91)    | 77 b  | 4×Costas-4 | |
| FST4-15    | 15 s   | 4     | 160     | 16.667 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | fastest FST4, ≈−20.7 dB threshold |
| FST4-30    | 30 s   | 4     | 160     | 7.143 Hz   | LDPC(240, 101)   | 77 b  | 5×Costas-8 | ≈−24.2 dB |
| FST4-60A   | 60 s   | 4     | 160     | 3.0864 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | dominant terrestrial sub-mode, ≈−28.1 dB |
| FST4-120   | 120 s  | 4     | 160     | 1.4634 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | ≈−31.3 dB |
| FST4-300   | 300 s  | 4     | 160     | 0.5580 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | ≈−35.3 dB, deepest wired FST4 |
| WSPR       | 120 s  | 4     | 162     | 1.465 Hz   | conv r=½ K=32 + Fano | 50 b | per-symbol LSB (npr3) | |
| JT9        | 60 s   | 9     | 85      | 1.736 Hz   | conv r=½ K=32 + Fano | 72 b  | 16 distributed | |
| JT65       | 60 s   | 65    | 126     | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 distributed | |
| Q65-15A    | 15 s   | 65    | 85      | 6.667 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 distributed | |
| Q65-30A    | 30 s   | 65    | 85      | 3.333 Hz   | (same QRA codec) | 77 b  | (same)     | |
| Q65-60A    | 60 s   | 65    | 85      | 1.667 Hz   | (same QRA codec) | 77 b  | (same)     | 6 m EME |
| Q65-60B    | 60 s   | 65    | 85      | 3.333 Hz   | (same QRA codec) | 77 b  | (same)     | 70 cm / 23 cm EME |
| Q65-60C    | 60 s   | 65    | 85      | 6.667 Hz   | (same QRA codec) | 77 b  | (same)     | ~3 GHz EME |
| Q65-60D    | 60 s   | 65    | 85      | 13.33 Hz   | (same QRA codec) | 77 b  | (same)     | 5.7 / 10 GHz EME |
| Q65-60E    | 60 s   | 65    | 85      | 26.67 Hz   | (same QRA codec) | 77 b  | (same)     | 24 GHz+, extreme spread |
| Q65-120D   | 120 s  | 65    | 85      | 6.0 Hz     | (same QRA codec) | 77 b  | (same)     | 10 GHz rain/troposcatter |
| Q65-120E   | 120 s  | 65    | 85      | 12.0 Hz    | (same QRA codec) | 77 b  | (same)     | 6 m ionoscatter |
| Q65-300A   | 300 s  | 65    | 85      | 0.289 Hz   | (same QRA codec) | 77 b  | (same)     | optical scatter, deepest AWGN |
| MSK144     | T/R period | 2 (MSK) | 144 | — | LDPC(128, 90) + CRC-13 | 77 b | burst scan | not a `Protocol` impl |
| JTTY       | none (any start) | 4 | 59 (13 sync + 46 data) | 31.25 Hz | tail-biting conv r=½ K=10 + CRC-12 | 32 b source + 2 flag b | 13-tone head sync | 1.888 s frame, NSPS 384 (31.25 baud); not a `Protocol` impl |

### 3.3 Per-protocol notes

- **FT8 / FT4 — what WSJT-X 3.x changed, and what this crate follows.**
  Read at the `v2.7.0` and `v3.0.0` tags (3.2.0-rc1 carries the same
  values). FT8 has the SNR floor and the `xsnr2` bail-out at **−25 dB**
  (`FT8_SNR_FLOOR_DB`, was −24), `Ft8::AP_MAG_SCALE` **1.1** (was 1.01),
  `mlag` **13**, three passes with passes 2 and 3 on the squared `|cs|²`
  metric, a fifth LLR variant `llre`, the nsync floor (`> 6`, `> 7` in
  the squared-metric passes, `> 8` at `Depth::Fast` and `Depth::Normal`, as `ndepth <= 2` does), and drops
  `/R` and `TU; ` messages outside a contest (#438, #439; §2.6). FT4's
  published defaults are `sync_min` 1.18, `max_cand` 200 (#440;
  `ft4_decode.f90` moved from 1.2 / 100). The message packer follows
  `pack77_1`: `RR73` goes out as the grid `RR73` (field 32373, not
  `MAXGRID4 + 3`), reports −35..−31 dB wrap by 101 as upstream does, and
  `/P` / `/R` calls pack (Type 2 when either carries `/P`) (#464).
  **OSD runs the way `decode174_91` runs it (#456).** For FT4 (and FT8's
  AP rung) that is BP, then OSD on the BP sum after 1 and after 2
  iterations, a test pattern that flips a locked bit skipped, the CRC
  checked once on the winner (`osd_decode_npre1_masked` on
  `bp_llr_zsum_ap_with_scratch`). Before, the raw LLR was searched with
  the CRC on every candidate: on iid Gaussian LLRs **22.6 %** of
  `osd_depth` 2 calls passed the CRC against `decode174_91`'s 5.8e-5
  (9.7e-5 after). Within 50 Hz of `rx_freq_hz` FT4 also takes a third
  OSD snapshot (`FecOpts::osd_snapshots`, `maxosd = 3`): 41 gained, none
  lost on 20 800 sweep files. The post-OSD `osd_max_errors` gate is gone
  ([§6](#6-engine-primitives)).
- **FST4** — LDPC(240, 101) + 24-bit CRC (`fec::ldpc240_101`); the
  BP/OSD code is the same across LDPC sizes, so the new material is
  just the parity-check/generator tables and code dimensions. The five
  wired sub-modes differ only in `NSPS` / `SYMBOL_DT` /
  `TONE_SPACING_HZ` — plus `TX_START_OFFSET_S` for FST4-15 alone
  (0.5 s rather than 1.0 s into the slot) — and are emitted by the
  `fst4_submode!` macro. FST4-900 / FST4-1800 remain unwired (no user
  demand). FST4W — the WSPR-style one-way 50-bit beacon variant,
  LDPC(240, 74) — is a separate message format and out of scope
  (issue #23). The **OSD searches the (240, 91) subcode**, as
  `fst4_decode.f90:478` (`decode240_101(llr, Keff=91, …)`) does: only the
  message and the first 14 CRC bits are free, the last 10 CRC bits are
  cascaded into the code (`ldpc240_101::FST4_KEFF = 91`, `osd::PartialCrc`).
  On upstream's own `decode240_101`, 4000 BPSK/AWGN draws a point, that
  recovers 3432 / 2071 / 593 at amplitude 1.0 / 0.9 / 0.8 against 2903 /
  1231 / 211 with all 101 bits free, about 0.5 dB. The CRC-24 is checked
  once, on the OSD winner (`osd240_101.f90:285`); with 14 detecting bits the
  wrong-codeword rate per call is FT8's 2⁻¹⁴, so
  `FrameDecodable::REQUIRES_UNPACK` (true for FST4) refuses a decode whose
  77 bits do not unpack, as `fst4_decode.f90:570` does. Tier C against
  `jt9 -7 -d3`, 20 groups: crossing −0.07 dB (this crate minus `jt9`, was
  +0.18), unexpected decodes 27 (was 99; `jt9` 7) (#456).
  The `noise_blanker` extra is WSJT-X's **NB** ([§2.5](#25-extras-and-the-protocols-outside-decoder)): on 50
  FST4-15 slots with 20 full-scale clicks a second, 0 decodes without it, 29
  here and 27 for `jt9` at NB 2 % (#469).
- **WSPR** — `ConvFano` ported from WSJT-X `lib/wsprd/fano.c`;
  `Wspr50Message` covers Types 1 / 2 / 3. The module adds a
  quarter-symbol spectrogram to keep the 120-s-slot coarse search
  within a reasonable time budget.
- **JT9 / JT65** — JT9's `ConvFano232` differs from WSPR's `ConvFano`
  only in its 206-bit codeword framing; both feed the 72-bit
  `Jt72Codec`. JT65's `Rs63_12` does erasure-aware decoding via Karn's
  Berlekamp-Massey.
- **Q65** — QRA over GF(64) (`fec::qra::QraCode` + the code instance
  `fec::qra15_65_64::QRA15_65_64_IRR_E23`); the application layer adds
  a CRC-12 over 13 information symbols and punctures the two CRC
  symbols out of the 65-symbol codeword, leaving 63 channel symbols
  transmitted. Ten sub-modes differ only in `NSPS` and tone spacing
  (×1…×16); all the decoder strategies share the one QRA codec. The
  decoder metric uses the punctured code rate 13/63 as `q65_init` does
  (it was 15/65, 12 % high), and every list decode requires
  `plog > PLOG_MIN` (−242) and a non-zero message, as `q65_dec1` does.
- **JTTY** — see the footnote in [§3.1](#31-generic-vs-bespoke-per-protocol)
  and [§2.5](#25-extras-and-the-protocols-outside-decoder). Its constants
  live in `jtty` (`NSPS`, `SYNC_SYMBOLS`, `FRAME_SYMBOLS`, `MAX_FRAMES` =
  16), not in trait constants. It has its own 1-based GFSK pulse in
  `jtty::tx`, not `engine::dsp::gfsk`: that pulse sat one sample early
  until #482, which JTTY's waveform check (4.9e-2 against upstream) exposed.

### 3.4 Decoder strategies

Every protocol runs the same underlying flow; the *strategy* wrapped
around it varies, and `Depth` ([§2.2](#22-decodeparams-and-depth)) picks it
as `ndepth` does. Most are a single pass. Only Q65 exposes several
parallel receiver chains for one FEC frame, MSK144 replaces the
slot model with a burst scan, and JTTY with an incremental receiver.

| Protocol | Strategy by `Depth` | Optional (extras and params) |
|----------|---------------------|------------------------------|
| **FT8**  | `Fast` 2 flat SIC rounds, no OSD; `Normal` / `Deep` `SicEarly` | `Tuning::strategy` (`SinglePass`, `SicRounds(n)`, `SicEarly`); AP iaptype loop (1–12) from the QSO context or `ap_hint`; the **a7 / a8 list decoders** (pass ids 30 / 31, run at the end of every FT8 strategy; a7 via the `a7` extra and `SlotInput::period`, a8 via MyCall, HisCall and HisGrid plus `rx_freq_hz`); `sniper` |
| **FT4**  | `Fast` single pass, no OSD, no AP; `Normal` `SicRounds(3)`; `Deep` `SicRounds(3)` with OSD | `Tuning::strategy` (`SinglePass`, `SicRounds(n)`); full-slot coherent sync (`sync2d`) |
| **FST4** | single-pass BP + OSD at every depth; the `i0 ± 1` timing retry from `Normal` | full-slot two-stage coherent sync search; `noise_blanker` (fixed % or sweep) |
| **WSPR** | wsprd's `-qB` / `-C 500 -o 4` / `+ -d` passes over the quarter-symbol spectrogram scan | `max_cycles_per_bit` |
| **JT9**  | single bespoke pass, Fano limit by depth; the Rx-frequency pass at `rx_freq_hz` | — |
| **JT65** | 2 / 2 / 4 passes with subtraction; stochastic Chase decoder with the depth's `nvec` trials | `chase` (RS erasure decode is `jt65::SniperRequest::erasures`) |
| **Q65**  | `(Δf,Δt,b90)` grid + Lorentzian fading BP (scan), grid effort by depth | AP-hint, explicit fast-fading, AP-list, averaging; **q3** list decode; Max Drift; Pileup; EME delay ([§2.5](#25-extras-and-the-protocols-outside-decoder)) |
| **MSK144** | burst scan over the whole T/R period | — |
| **JTTY** | streaming: sync surface, candidates, four-rung list-WAVA ladder, gate; decoded frames subtracted, retro re-sweep, frames assembled into messages | `Params::subtract` off (single-signal receiver) |

**A-priori decoding is a general option, not a sniper feature.** AP is
the last rung of the per-candidate ladder — `process_candidate_basic`'s
for FT4 and every FST4 sub-mode, FT8's own for FT8 — and
`msg::pipeline_ap` is hypothesis generation with no engine of its own. It used to be coupled to the sniper by accident
and that cost most of the decodes — the measurement is in
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md).

**Q65's strategies** — which one runs for which block, the q3 list decode,
averaging, Pileup, Max Drift, the time window and EME delay, and the history
and callers helpers — are in [§2.5](#25-extras-and-the-protocols-outside-decoder).

---

## 4. Module and crate map

```text
mfsk_core
├── engine/           Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · cpfsk · envelope · subtract ·
│   │                   msk · analytic · ddc · fir_decimate · polyphase · dotprod ·
│   │                   symbol_fft · blanker · fixed-point FFT kernels
│   ├── fft.rs          FftPlanner trait + the extern factory (see EMBEDDED.md); `with_planner`, a per-thread planner
│   ├── scalar.rs       Q-format fixed-point scalar types
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── sync2d.rs       FT4 / FST4 full-slot coherent sync searches
│   ├── search.rs       SearchParams / SyncCandidate / SearchWindow — the coarse
│   │                   search shared by WSPR, JT9, JT65, Q65 (#394)
│   ├── gray.rs         gray / inv_gray, width-parameterised (`igray.c`) — JT65, JT9
│   ├── ft4_coarse.rs   FT4 coarse candidate generation
│   ├── baseline.rs     spectral baseline fit (FT4 / FST4 normalisation)
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (Wiener per-tone)
│   ├── spectrogram.rs  Spectrogram build/score kernel — JT9, JT65, Q65
│   │                   (WSPR keeps its own: fixed-point FFT backend +
│   │                   baseline-fit normalisation, a real difference)
│   ├── interleave.rs   bit-reversal interleave_bitrev/deinterleave_bitrev
│   │                   — WSPR, JT9 (JT65's is a 7×9 matrix transpose,
│   │                   a different algorithm, and stays in jt65/)
│   ├── tx.rs           message_to_tones / info_to_tones, FskWaveform, and
│   │                   synthesize / synthesize_into / synthesize_i16 / synth_len
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
│                       (pub(crate) internals — call via
│                       decoder::Decoder)
├── decoder/          the public decode API — §2
│   ├── mod.rs          Decoder<P> / Decodable / SlotInput / Audio / Row / RowDetail / SlotResult
│   ├── params.rs       DecodeParams / Depth / Station / QsoContext / ApMode / Contest / SearchTuning
│   ├── frame.rs        FT8 / FT4 / FST4: Ft8Extras · Ft4Extras · Fst4Extras, Tuning, strategies, QSO-context AP
│   ├── slow.rs         WSPR / JT9 / JT65: their extras and state
│   ├── q65.rs          Q65Extras / Q65State (averaging)
│   └── any.rs          AnyDecoder / AnyExtras / Unsupported
├── slotgrid.rs       SlotGrid · SampleClock · ClockChange · SlotCutter — UTC slot arithmetic, integers only, no_std
├── fec/              FecCodec implementations
│   ├── ldpc/           LDPC(174, 91)  — FT8, FT4 (bp.rs / osd.rs / params.rs / tables.rs)
│   ├── ldpc240_101/    LDPC(240, 101) — FST4, uvpacket (punctured)
│   ├── ldpc_128_90/    LDPC(128, 90)  — MSK144
│   ├── conv/           ConvFano r=½ K=32 — WSPR; ConvFano232 — JT9 (fano.rs)
│   ├── rs/             RS(63, 12) GF(2⁶) — JT65
│   ├── qra/            Q-ary RA codec family — Q65
│   │   ├── code.rs       Generic QRA encoder + non-binary BP decoder
│   │   ├── q65.rs        Q65 wrapper (CRC-12 + puncturing) + list-decoding
│   │   ├── fast_fading.rs Doppler-spread-aware intrinsic metric
│   │   ├── fading_tables.rs Gaussian / Lorentzian calibration tables
│   │   ├── npfwht.rs      Non-binary Walsh-Hadamard transform helpers
│   │   └── pdmath.rs      Probability-domain BP math helpers
│   └── qra15_65_64/    the QRA15_65_64_IRR_E23 code instance
├── msg/              Message codecs and the public output row
│   ├── decode_request.rs the frame family's request builders — crate-private (`internal-testing` reopens them); `decoder` drives them
│   ├── decoded.rs      Decoded — the cross-mode output row
│   ├── wsjt77.rs       77-bit WSJT message — FT8, FT4, FST4, Q65, MSK144
│   ├── wspr.rs         50-bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72-bit JT message — JT9, JT65
│   ├── callsign28.rs   shared base-37/36/10/27³ callsign pack/unpack core
│   ├── q65.rs          77-bit ↔ 13× GF(64)-symbol packing for the QRA codec
│   ├── ap.rs           ApHint — a-priori hint builder
│   ├── pipeline_ap.rs  AP hypothesis generation (77-bit-family protocols)
│   ├── packet_bytes.rs PacketBytesMessage — byte-payload example codec
│   └── hash_table.rs   Callsign hash table
├── registry.rs       PROTOCOLS static + ProtocolMeta + by_id / by_name
├── iq/               wideband IQ in — §2.7: IqToAudio (one channel → 12 kHz USB audio, 120 dB), IqReceiver (N channels
│                     cut into CompletedSlots on UTC, no decode; `Channelizer::Direct` or `Pfb`), PfbChannelizer (polyphase filter bank)
├── ft8/              FT8 ZST + decode + decode_block + wave_gen
│   ├── list_decode.rs  WSJT-X's a7 / a8 list decoders (pass ids 30 / 31)
│   └── acquire.rs      cold slot-phase acquisition from off-air audio (#356)
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 family — 5 sub-mode ZSTs (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search + ddc
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ erasure-aware RS, chase)
├── q65/              Q65 family — 10 sub-mode ZSTs + decode + synth
│   ├── decode_request.rs wide-band and multi-period requests (crate-private); SniperRequest (public) — §2.5
│   ├── search.rs       default_search_params, eme_delay_late_sec
│   ├── ap_list.rs      full-AP codeword list (`q65_set_list`)
│   ├── hist.rs         Q65History (`q65_hist`)
│   ├── contest.rs      Q65Callers, contest_codewords (`q65_hist2` / `q65_set_list2`)
│   └── q3.rs           q3 list decode (`q65_dec0` list branch; crate-private)
├── msk144/           MSK144 — no Protocol impl; own top-level driver
├── jtty/             JTTY — no Protocol impl (no slot); wire format, frame decoder, streaming receiver
│   ├── source.rs · crc.rs · tbcc.rs   32-bit grammar, CRC-12, tail-biting encoder
│   ├── pack.rs · tx.rs                text → fewest atoms → tones → audio
│   ├── trellis.rs · correlate.rs · ladder.rs · subtract.rs   list-WAVA decoder, correlators, ladder, subtraction
│   └── dsp.rs · rx.rs · assemble.rs   (FFT feature) window decoder, `Stream`, frame → message
└── uvpacket/         Applied non-WSJT example — 4 sub-mode ZSTs, own tx/rx
```

Each protocol module is gated behind a feature flag of the same name.
`engine`, `fec`, `msg` and `registry` are always available.

### Workspace crates

`[workspace] members` in the root `Cargo.toml`:

| Crate | Responsibility | Published |
|-------|----------------|-----------|
| `mfsk-core` | The library. Host (rustfft) or `no_std` + alloc via a pluggable FFT backend. Everything else is a consumer. | **yes**, crates.io |
| `mfsk-ffi` | The C ABI over **all** protocols: `libmfsk.{so,a,dylib}` + the committed `mfsk.h`. See [`BINDINGS.md`](BINDINGS.md) | no |
| `mfsk-ffi-abi` | Shared `#[repr(C)]` mode / status / params / row types that `mfsk-ffi` re-emits (issue #205) | no |
| `hosttest/mfsk-app-shared` | Runs the host-testable parts of `embedded-poc/mfsk-app-shared` under a normal `cargo test` | no |

`workspace.package.version` is the single source of truth for all four.

Deliberately **outside** the workspace, because a host `cargo build`
would try to compile them with the stable toolchain and fail:
`embedded-poc/` (stand-alone Cargo projects for the M5Stack ESP32
boards, each with a path dep on `mfsk-core` — see
[`EMBEDDED.md`](EMBEDDED.md)) and `bench/wasm/` (a wasm-bindgen
harness, issue #208).

`bindings/kotlin/` and `bindings/swift/` are not Rust and so are in
neither list — see [`BINDINGS.md`](BINDINGS.md).

`mfsk-ffi-ft8`, a smaller FT8-only embedded C ABI, was **retired** in
0.11.0: an ESP-IDF consumer needs a Rust staticlib shim regardless
(pure C cannot define the `extern "Rust"` FFT-planner symbol), and
once a consumer is writing Rust, calling `mfsk-core` directly is
strictly simpler.

#### `FecCodec` is symbol-agnostic

The `FecCodec` trait surface (`engine/protocol.rs`) speaks in **bits**:
`&[u8]` info / codeword, `&[f32]` bit-LLRs, `K` and `N` counted in
bits. Two of the families are non-binary codes — Reed-Solomon over
GF(2⁶) for JT65 and QRA over GF(2⁶) for Q65 — and implement the
bit-level trait by packing / unpacking bits ↔ symbols inside their own
`encode`. Their natural symbol-level decode lives outside
`decode_soft`: `Q65Fec::decode_soft` returns `None` by design, and the
real Q65 decode runs over GF(64) probability vectors via
`fec::qra::Q65Codec`. Counting `K` / `N` in bits keeps the
cross-protocol invariant `FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL`
meaningful for both binary and non-binary codes — see
[§8](#8-runtime-registry-and-trait-surface-verification).

---

## 5. The `Protocol` trait hierarchy

Read the crate bottom-up. There is a **generic core** that knows
nothing about any specific protocol, and each protocol is a thin
plug-in that selects which pieces of that core it uses.

1. **`engine/`** — protocol-agnostic DSP, sync, LLR, the equaliser and
   the decode pipeline. Every function is generic over `P: Protocol`
   and reads the protocol's constants; no per-protocol branches.
2. **`fec/`** — the FEC codec families, each an `impl FecCodec`.
3. **`msg/`** — the message codecs, each an `impl MessageCodec`, plus
   the crate-private request builders that drive the pipeline.
   **`decoder/`** sits on top: `Decoder<P>` (§2) maps WSJT-X's parameter
   block onto those builders and owns the cross-period state.
4. **A protocol** is a zero-sized type implementing three composable
   traits. It carries only constants and two associated-type choices —
   `type Fec` and `type Msg` — plus a `SYNC_MODE`. That is the entire
   act of adding a protocol.

At decode time those layers execute as one receive flow, shared by
every wired protocol:

```text
┌─────────┐  coarse_sync   ┌──────────────┐  refine_candidate  ┌──────────┐
│ samples │ ─────────────▶ │  candidates  │ ─────────────────▶ │ candidate│
│ i16/f32 │  (FFT/Costas)  │ (f, dt, snr) │   (fine sync)      │ refined  │
└─────────┘                └──────────────┘                    └────┬─────┘
                                                                    │  symbol_spectra
                                                                    ▼
                  ┌─────────────┐  compute_llr  ┌──────────────┐  equalize_local
                  │   LLR vec   │ ◀───────────  │     cs[]     │ ◀──────────┐
                  │  (4 vars)   │   (per WSJT)  │   Complex    │ (per-tone  │
                  └──────┬──────┘               │  per-symbol  │  Wiener)   │
                         │                      └──────────────┘            │
                         │  P::Fec::decode_soft  (LDPC BP / Fano / RS /     │
                         │                        QRA-symbol-level)         │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ info bits   │                                           │
                  └──────┬──────┘                                           │
                         │  P::Msg::unpack                                  │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ message txt │ ──── (subtract for next iter) ────────────┘
                  └─────────────┘
```

The traits:

<!-- Not compiled: re-declaring same-named traits here wouldn't
     actually check anything against the real definitions (unlike
     the worked examples below, which `impl` the real imported
     traits and so break if they drift). Kept in sync by hand against
     `engine/protocol.rs` when that file changes. -->

```rust,ignore
pub trait ModulationParams: Copy + Default + 'static {
    const NTONES: u32;
    const BITS_PER_SYMBOL: u32;
    const NSPS: u32;              // samples/symbol @ 12 kHz
    const SYMBOL_DT: f32;
    const TONE_SPACING_HZ: f32;
    const GRAY_MAP: &'static [u8];
    const GFSK_BT: f32;
    const GFSK_HMOD: f32;
    const NFFT_PER_SYMBOL_FACTOR: u32;
    const NSTEP_PER_SYMBOL: u32;
    const NDOWN: u32;
    // Defaulted knobs — a protocol overrides only what its WSJT-X
    // counterpart does differently (`engine/protocol.rs`):
    const LLR_SCALE: f32 = 2.83;
    const LLR_NSYM_MAX: u32 = 3;                    // FT4 4, FST4 8
    const LLR_NSYM_MID: Option<u32> = None;         // FST4 Some(4): the nsym=4 rung
    const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = None;  // FT4, FST4: the 77-element rvec
    const SPECTRUM_WINDOW: SpectrumWindow = SpectrumWindow::Rectangular;  // FT4 Nuttall4
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) or Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
    const CODEWORD_INTERLEAVE: Option<&'static [u16]> = None;  // no wired protocol sets it
}

pub enum SyncMode {
    /// Block-based Costas / pilot arrays at fixed symbol positions.
    /// Used by FT8 / FT4 / FST4.
    Block(&'static [SyncBlock]),
    /// Per-symbol bit-interleaved sync: one bit of a known sync vector
    /// is embedded at `sync_bit_pos` within every channel-symbol tone
    /// index. Used by WSPR (symbol = 2·data + sync_bit).
    Interleaved {
        sync_bit_pos: u8,
        vector: &'static [u8],
    },
}

pub trait Protocol: ModulationParams + FrameLayout + 'static {
    type Fec: FecCodec;
    type Msg: MessageCodec;
    type SyncPhasors: SyncPhasors;   // what the Δt search precomputes; `()` except FT4
    const ID: ProtocolId;
    const AP_MAG_SCALE: f32 = 1.01;  // apmag = max|llr| * this; FT8 1.1, FT4 1.1, FST4 1.1 (3.0 onward)
    const DECODE_FFT1_SIZE: u32 = 0; // forward-FFT length over the slot; 0 = no shared downsampler
}
```

### `Decodable`: putting a protocol behind `Decoder<P>`

<!-- Not compiled: the decode hooks are `#[doc(hidden)]` and crate-internal. -->

```rust,ignore
pub trait Decodable: Sized {
    const MODE: Mode;                        // the registry::Mode this ZST decodes
    type State: Default + Send;              // what upstream keeps across periods
    type Extras: Clone + Default + Send;     // what this library adds, typed per mode
    type Row: Clone + Send;                  // the mode's native result
    // plus hidden hooks: the decode itself, 77-bit unpack against State, learn
}
```

It is implemented for every slot-decoded ZST (the 20 WSJT-family modes; not
`uvpacket`, MSK144 or JTTY). It is what the public API is generic over;
`FrameDecodable` and the per-family request types it replaced are
crate-private. A new mode adds an impl here, a line in the
registry's `modes!` list, and a variant in `any.rs`'s `any_decoder!`.

### Worked examples

Two concrete cases show how the three traits combine on a real ZST.
These compile against the real imported traits, so they break if the
trait surface drifts.

**FT4** — a standard block-Costas protocol that shares its FEC and
message codec with FT8:

```rust
use mfsk_core::engine::{
    FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};
use mfsk_core::fec::Ldpc174_91; // re-exported from fec::ldpc
use mfsk_core::msg::Wsjt77Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const SYMBOL_DT: f32 = 0.048;
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 4;
    const NSTEP_PER_SYMBOL: u32 = 2;
    const NDOWN: u32 = 18;
    // (LLR_NSYM_MAX/INFO_SCRAMBLE_RVEC etc. are recall-tuning knobs
    // with defaults — see the real `ft4::Ft4` for FT4's overrides.)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
    const N_RAMP: u32 = 2;
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 7.5;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft4 {
    type Fec = Ldpc174_91;          // shared with FT8
    type Msg = Wsjt77Message;       // shared with FT8
    // What the Δt search precomputes. `()` is the ordinary answer;
    // FT4 itself carries `Ft4CoarsePhasors`, which is why the real
    // impl differs from this example here.
    type SyncPhasors = ();
    const ID: ProtocolId = ProtocolId::Ft4;
}

const FT4_SYNC_BLOCKS: [SyncBlock; 4] = [
    SyncBlock { start_symbol:  0, pattern: &[0, 1, 3, 2] },
    SyncBlock { start_symbol: 33, pattern: &[1, 0, 2, 3] },
    SyncBlock { start_symbol: 66, pattern: &[2, 3, 1, 0] },
    SyncBlock { start_symbol: 99, pattern: &[3, 2, 0, 1] },
];
```

**WSPR** — structurally different on all three axes. The `Fec` and
`Msg` associated types switch to a new pair, and the sync is expressed
via `SyncMode::Interleaved`:

```rust
use mfsk_core::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // ~683 ms @ 12 kHz
    const SYMBOL_DT: f32 = 8192.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 1;
    const NSTEP_PER_SYMBOL: u32 = 16;
    const NDOWN: u32 = 32;
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync is embedded in data symbols
    const N_SYMBOLS: u32 = 162;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Interleaved {
        sync_bit_pos: 0,                     // LSB of the tone index
        vector: &WSPR_SYNC_VECTOR,           // 162-bit npr3
    };
    const T_SLOT_S: f32 = 120.0;
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Wspr {
    type Fec = ConvFano;                     // convolutional + Fano
    type Msg = Wspr50Message;                // 50-bit message
    type SyncPhasors = ();                   // no Δt search tables
    const ID: ProtocolId = ProtocolId::Wspr;
}

// Illustrative stand-in — the real 162-bit npr3 vector lives in
// `wspr::decode`'s private sync table.
const WSPR_SYNC_VECTOR: [u8; 162] = [0u8; 162];
```

### Transmitting: `FskWaveform`

A protocol that transmits FSK also implements
`engine::tx::FskWaveform`, one constant saying which of WSJT-X's two
transmit families it belongs to (#391):

```rust,ignore
impl FskWaveform for Wspr {
    const WAVEFORM: Waveform = Waveform::Cpfsk;          // WSPR, JT9, JT65, Q65
}
impl FskWaveform for Ft8 {
    const WAVEFORM: Waveform = Waveform::Gfsk(FT8_GFSK); // FT8, FT4, FST4
}
```

That is all `engine::tx` needs: `message_to_tones::<P>` turns a 77-bit
message into tones (FT8 / FT4 / FST4), and `synthesize::<P>` /
`synthesize_into` / `synthesize_i16` / `synth_len` turn tones into audio
at any sample rate, for every mode above. `Cpfsk` is plain
continuous-phase FSK at `TONE_SPACING_HZ` and `SYMBOL_DT`, which WSJT-X
generates in its modulator; `Gfsk` is the pre-computed shaped waveform
of `gen_ft8wave.f90` and friends. A test checks each `Gfsk` config
against its protocol's own `NSPS`, `GFSK_BT` and `GFSK_HMOD`, so the two
cannot drift apart. A protocol whose transmit chain is not FSK — the
`uvpacket` example is π/4-DQPSK — just doesn't implement it.

### Monomorphisation is why this is free

All hot-path functions take `P: Protocol` as a **compile-time** type
parameter. rustc monomorphises one copy per concrete protocol, and LLVM
inlines the trait constants as literals. The generated FT8 code is
byte-identical to the hand-written FT8-only path the library was forked
from, and FT4 benefits from every micro-optimisation applied to the
shared functions.

`dyn Trait` is reserved for cold paths: the FFI boundary and the
`MessageCodec` that unpacks decoded text (once per successful decode,
not once per candidate).

### Adding a protocol

`CONTRIBUTING.md` has the step-by-step. In short, how much work it
takes depends on how much it can reuse:

| case | work |
|---|---|
| Same FEC and message as an existing mode (another FST4 sub-mode) | a new ZST with different numeric constants; `Fec`/`Msg` are type aliases. The full generic pipeline runs unchanged, and a `Decodable` impl (its `State`, `Extras`, `Row`) is what puts it behind `Decoder<P>` |
| New FEC, same message (a different LDPC size) | add a module under `fec/` and implement `FecCodec`. BP/OSD/systematic-encode generalise across LDPC sizes, so the real changes are the tables and dimensions. `fec::ldpc240_101` is the example |
| Both new (WSPR) | add the FEC, add the message codec, and extend `SyncMode` if the sync structure is genuinely different |
| Sub-mode of an existing protocol | the `q65_submode!` / `fst4_submode!` macros emit the ZST and its three trait impls from the differing constants. One line in `tests/protocol_invariants.rs` picks it up |

FST4-60A landed without touching shared code.

---

## 6. Engine primitives

### DSP (`mfsk_core::engine::dsp`)

| Module | Purpose |
|---|---|
| `resample` | linear resampler to 12 kHz |
| `downsample` | FFT-based complex decimation (`DownsampleCfg`) |
| `gfsk` | GFSK tone-to-PCM synthesiser (`GfskCfg`, `GfskStream`). The 3-symbol Gaussian pulse is **1-based** since #482, as `gen_ft8wave.f90` / `gen_ft4wave` / `gen_fst4wave` build it (`tt=(i-1.5*nsps)/nsps`, `i=1..3*nsps`); it had run from `i=0`, one sample early, so every FT8 / FT4 / FST4 waveform was shifted (phase error up to 2π·Δf/fs, 0.023 rad on FT8). Worst normalised sample error against `ft8sim` (v3.2.0-rc1, SNR 99): 1.3e-2 → 1.2e-4 (`tests/gfsk_vs_wsjtx.rs`) |
| `cpfsk` | plain continuous-phase FSK synthesiser — WSJT-X's positive-`toneSpacing` transmit loop, WSPR / JT9 / JT65 / Q65 (`synth_f32`, `synth_f32_into`); one copy, where four `tx.rs` files had each carried one |
| `envelope` | the raised-cosine ramp WSJT-X's modulator puts on those four modes' transmissions (`ramp_samples`, `apply_ramp`; #259) |
| `symbol_fft` | `SymbolFft`: one `nsps`-point FFT reused across a frame's symbols, planned through `engine::fft` — JT9, JT65, Q65 demodulators (#390) |
| `blanker` | `blanker(audio, nz, ndropmax, npct)`: `blanker.f90`'s impulse-noise blanker, behind FST4's `.noise_blanker()` |
| `subtract` | phase-continuous least-squares SIC (`SubtractCfg`) |
| `ddc` | streaming digital down-converter (WSPR's embedded channelizer) |
| `fir_decimate` | `FirStage`, a streaming FIR-and-decimate over complex I/Q, and the low-pass designers: `design_lowpass` (Blackman, ~74 dB) and, for a stated selectivity, `kaiser_order` + `design_lowpass_kaiser` (Kaiser, `f64`, `no_std`). `FirStage::from_taps` takes a designed prototype (#534) |
| `polyphase` | `PolyphaseResampler`, a streaming rational `L/M` resampler over complex I/Q; `from_prototype` takes a designed prototype (#534) |
| `dotprod` | the dot-product kernel the extern hook replaces |

Each takes a runtime `*Cfg` struct rather than `<P>`, because the
tuning parameters include composite-FFT sizes not trivially derived
from trait constants. Protocol modules expose module-level constants —
`ft8::downsample::FT8_CFG`, `ft4::decode::FT4_DOWNSAMPLE`, etc.

**Gray code.** `engine::gray::{gray, inv_gray}(n, bits)` is `igray.c`
for any width 1..=8 (computed in `u32`: the C loop's shift overflows a
`u8` from 4 bits up). JT65 (6 bits) and JT9 (3) use it; FT8 / FT4 / FST4
use their per-protocol `GRAY_MAP` table instead.

### Sync (`mfsk_core::engine::sync`)

* `coarse_sync::<P>(audio: AudioSource, freq_min, freq_max, sync_min,
  freq_hint: Option<f32>, max_cand, grid: RxGrid)` — UTC-aligned 2D
  peak search over `P::SYNC_MODE.blocks()` for non-FT8 protocols.
  `AudioSource` is `Real(&[i16])` or `Complex(&[f32], &[f32])`, paired
  with an `RxGrid` (`RxGrid::real(12_000.0)` for ordinary PCM).
* `refine_candidate::<P>(cd0, cand, search_steps)` — integer-sample
  scan + parabolic sub-sample interpolation.
* `make_costas_ref` / `score_costas_block` — raw correlation helpers
  exposed for diagnostics and custom pipelines.
* `sync_power_cv(per_block)` — the population coefficient of variation
  behind `DecodeResult::sync_cv`, one definition for every protocol
  since #414 (FT8's had been √3 times FT4's and FST4's for the same
  channel; nothing thresholds on it).

**FT8 routes through `ft8::decode_block::coarse_sync` exclusively.**
Calling `engine::sync::coarse_sync::<Ft8>` is still the right path for
hand-rolled non-default usage, but `Decoder<Ft8>` dispatches via
`decode_block::coarse_sync` internally. That is why an FT8 coarse-sync change cannot move an FST4
sensitivity curve: FST4 reaches sync through `engine::sync::coarse_sync`
+ `engine::sync2d::fst4_sync_search` instead.

### Sync2D (`mfsk_core::engine::sync2d`)

Two protocol-specific full-slot coherent searches, both ported from
WSJT-X and both scored via a **phase-continuous** Costas reference
(`make_costas_ref_continuous`, phase accumulating across the whole
8-symbol block instead of resetting per symbol) with
`score_flat_coherent` — ~3 dB better sync-score SNR discrimination
than a non-coherent `Σ|z_k|²` power-sum:

* `ft4_sync_search::<P>` and the windowed `ft4_sync_search_window::<P>`
  — **FT4 only**; a coherent full-slot Δt search (`ft4_decode.f90`'s
  `isync=1`/`isync=2` loop, `sync4d.f90` scorer) over the slot's
  downsampled-sample range rather than a local window around the
  coarse-sync candidate's own frequently-wrong Δt estimate.
* `fst4_sync_search::<P>` — FST4-specific two-stage full-slot search
  (`fst4_decode.f90:657-925`): a coarse pass over the entire T/R slot
  (±1.5 s, step 4, ±12 steps of 0.1·baud) then a fine pass (±7 steps
  of 0.02·baud × ±4 samples). Closed FST4's AWGN sensitivity gap vs
  WSJT-X's published thresholds to ~0.3 dB (issue #146).

`engine::sync::coarse_sync::<P>` also carries an FST4-only
augmentation: a bin can enter the candidate list either via the
short-time Costas-grid threshold *or* by clearing a full-slot
non-coherent 4-tone power check modelled on WSJT-X's
`get_candidates_fst4`. Gated on `P::ID == ProtocolId::Fst4`, so FT8 and
FT4 are byte-identical. It measured as a no-op on the narrow
single-signal AWGN sweep but is a real WSJT-X-faithful coverage
improvement for busy wideband scans.

### LLR (`mfsk_core::engine::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — per-symbol FFT bins. FT8
  callers should prefer `ft8::decode_block::fill_symbol_spectra`, which
  avoids the intermediate `cd0` allocation.
* `compute_llr::<P, T>(cs)` (`T: LlrScalar`, returns `LlrSet<T>`) — four
  WSJT-style LLR variants (a/b/c/d), built from
  `nsym ∈ {1, 2, P::LLR_NSYM_MAX}` correlation-ladder hypotheses, plus
  an `llre` at `nsym = P::LLR_NSYM_MID` when the protocol sets one
  (FST4: 4). `LLR_NSYM_MAX` defaults to 3; FT4 overrides it to 4 and
  FST4 to 8, both matching their own WSJT-X bit-metric code
  (`get_ft4_bitmetrics.f90` / `get_fst4_bitmetrics.f90`).
* `sync_quality::<P>(cs)` — hard-decision sync symbol count.

### Equalise (`mfsk_core::engine::equalize`)

* `equalize_local::<P>(cs)` — per-tone Wiener equaliser driven by
  `P::SYNC_MODE.blocks()` pilot observations; linearly extrapolates any
  tones Costas doesn't visit.

### Pipeline (`mfsk_core::engine::pipeline`)

`decode_frame::<P>` (coarse sync → parallel `process_candidate` →
dedupe), `decode_frame_subtract::<P>` (SIC driver) and
`process_candidate_basic::<P>` (single-candidate BP+OSD) are the raw
engine functions. They are **`pub(crate)`**, or `pub` only under
`internal-testing`. Use `Decoder<P>`.

**`DecodeStrictness` (`Strict`/`Normal`/`Deep`) does not reach every
protocol equally** — check which knob applies before assuming
`.strictness(...)` does anything for a given call:

| method | live for |
|---|---|
| `osd_max_errors()` — post-OSD hard-error ceiling, `osd_depth`-tiered | **No decode path.** `process_candidate_basic` applies it to no protocol any more: FST4 dropped it first (#146) and accepts on CRC-24 plus a successful unpack (`REQUIRES_UNPACK = true`, as `fst4_decode.f90:570`); FT4 followed in #456, once its OSD stopped returning a wrong codeword for 22.6 % of noise candidates (`ft4_decode.f90` has no such gate). FT8 has never called it. The method stays as public API and for the diagnostics that mirror the old ladder |
| `ap_max_errors(locked_bits)` — AP-assisted ceiling, graded by locked-bit count | FT8's per-candidate AP loop and the AP rung of the generic ladder (FT4, every FST4 sub-mode), numerically unified (issue #191). `Normal` is a flat **36**, `ft8b.f90`'s bound (`ft4_decode.f90` has none); 36 against the earlier 30 / 25 found 544 more hits on the FT8 sweep (+6.2 %, none lost) for 1-4 extra phantoms per 12 800 files (#456). `Strict` is 20 from 55 locked bits, else 24; `Deep` is 30 from 55, else 36 |
| `ft8_nharderrors_max()` — FT8's own flat (not tiered) ceiling for the non-AP BP staircase and OSD fallback | FT8 (`ft8::decode_block::process_candidates`/`osd_strategy`). `Normal` returns 36, WSJT-X's own `ft8b.f90:422` ceiling. `Strict = 22` reuses prior art from issue #72; `Deep = 37`, retuned from 40 in #253 by the FT8 sweep (`MFSK_FT8_SWEEP_STRICTNESS`, 16 AWGN/CCIR cells, 320 trials per level per strategy): golden recall was already saturated at 37 (105/320 single-pass, 108/320 `.sic_early()`, identical through 40) while false accepts kept climbing (15 → 16, 20 → 21) |

### FT8 block-decoder entry points (`mfsk_core::ft8::decode_block`)

FT8 exposes a parallel set of entries on top of the shared pipeline,
sharing one `process_one_candidate_inner` body between host and
embedded callers. All operate on the same audio + spectrogram inputs
and differ only in which inner steps they enable:

* `decode_block` / `decode_block_tuned` — pass-1 BP only.
* `decode_block_with_ap` / `decode_block_with_ap_tuned` — pass-1 BP
  followed by the WSJT-X AP iaptype loop (1–12) for any candidate whose
  pass-1 step missed but whose sync quality crosses `q_thresh`.
* `decode_block_into[_tuned]` — the embedded fixed-point entry point
  (`fixed-point` feature); same shape as `decode_block[_tuned]`, kept
  as a distinct name for API stability with `embedded-shared::dual_core`.
* `coarse_sync` / `coarse_sync_with_allsum` — the FT8 sync grid itself.
* `fill_symbol_spectra` / `fill_symbol_spectra_goertzel` — per-symbol
  FFT extraction directly from audio.

FT8 also has `ft8::list_decode` (the a7 / a8 list decoders, run at the end
of every strategy — [§3.4](#34-decoder-strategies)) and `ft8::acquire`
(`acquire_slot_phase`: cold slot-phase acquisition for a receiver with no
clock, three ±2.5 s windows of a longer capture 5 s apart, reduced with
`circular_dt_medoid`; #356).

---

## 7. Feature flags

**`mfsk-core/Cargo.toml` is the authoritative table**, and it is worth
reading as documentation: each flag carries the measurement that
justified it. The summary here covers what a Rust host consumer needs;
[`EMBEDDED.md`](EMBEDDED.md) covers the `no_std` and fixed-point side.

`default = ["std", "ft8", "ft4", "parallel", "fft-rustfft"]`.

| Feature | Default | Effect |
|---|---|---|
| `std` | on | standard library. Off implies `alloc` + an extern FFT backend |
| `alloc` | — | `no_std` with an allocator |
| `ft8` | on | FT8 ZST, decode, wave_gen |
| `ft4` | on | FT4 ZST, decode |
| `fst4` | off | FST4-15/30/60A/120/300 ZSTs, decode. **Not host-only** — routes entirely through the backend-agnostic engine and type-checks clean under `alloc,fst4,fft-extern` (issue #306) |
| `wspr` | off | WSPR ZST, decode, synth, spectrogram search |
| `jt9` / `jt65` / `q65` | off | **Not host-only since #390** — no forced backend, and each type-checks clean under `alloc,<mode>,fft-extern` with no `std`/`rustfft` pulled in (one feature-matrix row each, in `ci.yml` and `scripts/pre-push-check.sh`). Decoders only; no embedded app uses them yet |
| `msk144` | off | MSK144 — no `Protocol` ZST; own top-level driver |
| `jtty` | off | JTTY — no `Protocol` ZST; the FFT-free modules (`source`, `crc`, `tbcc`, `tx`, `pack`, `trellis`, `correlate`, `ladder`, `subtract`) build anywhere, `dsp`, `rx` and `assemble` need a host FFT (`fft-rustfft` or `fft-extern`). Has `alloc,jtty` and `alloc,jtty,fft-extern` rows in the feature matrix |
| `uvpacket` | off | applied non-WSJT example, 4 sub-mode ZSTs. Pulls in `fst4`, and **declares `std` explicitly** (it reaches for `std::f32::consts::PI`) |
| `packet-bytes` | off | `PacketBytesMessage` — byte-payload example `MessageCodec` |
| `full` | off | every protocol + `uvpacket` + `packet-bytes` + `serde` + `parallel` + host FFT |
| `parallel` | on | rayon `par_iter` in the pipeline (no-op under wasm) |
| `fft-rustfft` | on | host FFT backend |
| `fft-extern` | off | the final binary supplies `mfsk_core_make_default_fft_planner`. See `engine::fft` |
| `dotprod-extern` | off | likewise for `mfsk_core_dotprod_f32` |
| `fixed-point` | off | the numeric path embedded ships. **Implies `nstep-half`, and must** |
| `serde` | off | `Serialize`/`Deserialize` on the public result types |
| `internal-testing` | off | reopens `pub(crate)` engine internals for the crate's own integration tests. Deliberately **not** in `full` |

Flags that bite:

- **`internal-testing` is not optional for whole-crate commands.**
  Without it, `cargo clippy --all-targets --features full` reports
  `E0603` private-item errors in `fst4_sweep` / `ft4_sweep` /
  `fst4_wsjtx_samples`. That is a missing flag, not a regression.
- **`fixed-point` implies `nstep-half`.** Decoupled, host fixed-point
  ran a different time grid (NSTEP=NSPS/4) from embedded (NSPS/2) and
  produced a completely different candidate ranking on the same WAV —
  4 vs 7 decodes on `qso3_busy` single-pass. The point of host
  fixed-point is to simulate embedded faithfully.
- **`wspr-ddc` and `wspr-ddc-cascade` are mutually exclusive** —
  `compile_error!` in `decode_scan_inner`. They swap WSPR's channelizer
  from the reference whole-slot FFT to the streaming down-converter
  (single-stage / two-stage cascade). Host default stays on the exact
  reference.
- **`wspr-fano-cap-fast` is not a free knob upward.** Raising the cycle
  budget past the reference starts manufacturing phantom decodes; the
  swept table is in `wspr::decode`.

---

## 8. Runtime registry and trait-surface verification

### 8.1 The `PROTOCOLS` registry

`mfsk_core::PROTOCOLS` is a `&'static [ProtocolMeta]` populated at
compile time from each `Protocol`-impl ZST's associated constants, so a
consumer asking "what does this build support?" doesn't hardcode a list:

```rust
use mfsk_core::PROTOCOLS;

for p in PROTOCOLS {
    println!(
        "{:10}  {:>3}-tone  {:>4} bits/sym  {:>5.1} s slot  ID={:?}",
        p.name, p.ntones, p.bits_per_symbol, p.t_slot_s, p.id,
    );
}
```

Each `ProtocolMeta` carries the protocol's `id` (`ProtocolId`,
family-level), display `name`, and every constant the trait surface
exposes: modulation (`ntones`, `bits_per_symbol`, `nsps`, `symbol_dt`,
`tone_spacing_hz`, `gfsk_bt`, `gfsk_hmod`), frame (`n_data`, `n_sync`,
`n_symbols`, `t_slot_s`) and codec (`fec_k`, `fec_n`, `payload_bits`).
Beyond that geometry it publishes what a host cannot otherwise ask for:

* `tx_start_offset_s` — seconds from the start of the slot buffer to the
  first symbol, the `dt = 0` reference: 0.5 for FT8, FT4, FST4-15 and Q65
  15 / 30 s, 1.0 for the other FST4 sub-modes and Q65 from 60 s up (Q65
  followed `nsps` as `q65.f90:130-131` does; every sub-mode had published
  1.0 until #399).
* `slot_samples_12k` — the slot in samples (FT4 90 000, FT8 180 000,
  FST4-300 3 600 000), and `decode_fft1_size` — the forward FFT the
  decoder takes over the whole slot (`Protocol::DECODE_FFT1_SIZE`; FT4
  92 160, FT8 192 000, **FST4-300 4 194 304**, 0 for the modes with their
  own front end). The second is the number that makes "one call shape for
  every mode" wrong as a memory story.
* `profile: DecodeProfile { caps, defaults, sync_scale, sniper_max_cand_cap }`.

`caps` is a `u32` of bits in `registry::caps` — 17 of them, cross-checked
in both directions against the real trait impls by `tests/registry_caps.rs`
(a bit claimed without the trait fails, and so does a trait without the
bit): `DECODE_HANDLE` 0, `SNIPER` 1, `AP_NARROW` 2, `AP_WIDEBAND` 3,
`SIC_ROUNDS` 4, `SIC_EARLY` 5, `OSD` 6, `EQ_MODE` 7, `STRICTNESS` 8,
`BUDGET` 9, `KNOWN_FILTER` 10, `KNOWN_SUBTRACT` 11, `FFT_CACHE` 12,
`ON_RESULT` 13, `ENCODE` 14, then `NOISE_BLANKER` = 1 << 16 (every FST4
sub-mode) and `TX_FREQ` = 1 << 17 (FT8; no marker trait, so the claim list
pins it). **Bit 15 is skipped on purpose**: it is `MFSK_CAP_STREAM_RECEIVER`,
which only `mfsk-ffi` defines (JTTY has no registry entry to claim it).
`sync_scale` says how to read `sync_min`, because the scales are not the
same number: `CostasAbsolute` (FT8; noise has no fixed value),
`BaselineNormalised` (FT4, and FST4 since #554: the spectrum is divided by a
fitted baseline, so noise sits at ~1.0 and any threshold below that admits
every peak), and
`SyncFraction` (WSPR, JT9, JT65, Q65: sync power over sync plus noise, in
0‥1, default `DEFAULT_SCORE_THRESHOLD` = 0.1). `sniper_max_cand_cap` is the
bound the sniper path silently applies to `max_cand` (FT4: 15).

`profile.defaults` is the host search the registry publishes, and what the
C ABI's `mfsk_mode_defaults` returns. **`Decoder<P>` does not read it:**
`Depth` decides `sync_min` and the candidate count ([§2.2](#22-decodeparams-and-depth)),
and the band is `decoder::default_params(mode)`'s. The table is the registry's
own data (a UI asking "what does this build search by default?"); the scan
modes' time window, score threshold and candidate cap still start from their
`default_search_params()`:

| entry | band (Hz) | `sync_min` | `max_cand` | source |
|---|---|---|---|---|
| FT8 | 100-3000 | 0.8 | 60 | `FT8_PROFILE` |
| FT4 | 300-2700 | 1.18 | 200 | WSJT-X 3.x `ft4_decode.f90` `syncmin` / `MAXCAND` (#440; was 1.2 / 100) |
| FST4 (all five) | 100-3000 | 1.20 (FST4-15: 1.15) | 200 | WSJT-X `fst4_decode.f90` `minsync` and candidate array (#554; was 0.8 / 50 on the Costas scale) |
| WSPR | 1400-1600 | 0.1 | 200 | `wspr::search::default_search_params()`; ±5.46 s (8 symbols) |
| JT9 | 200-4000 | 0.1 | 8 | `jt9::search::default_search_params()`; ±1.728 s |
| JT65 | 1000-2000 | 0.1 | 8 | `jt65::search::default_search_params()`; ±7.62 s |
| Q65 (all ten) | 200-3000 | 0.1 | 8 | `q65::search::default_search_params()`; ±1.0 s (#399, #413) |
| uvpacket | — | 0 | 0 | none of the search machinery applies |

Since #413 WSPR, JT9, JT65 and Q65 read their row from the library's own
`default_search_params()` rather than a hand-kept copy, which had drifted
from all four (WSPR / JT9 / JT65 published no band, and Q65 published
`mfsk-ffi`'s wide EME scan of 32 candidates at 0.05). Without an FFT backend
those four have no `search` module and publish an empty band.

Lookup:

* `by_id(ProtocolId::Q65)` — *every* entry sharing the family-level id.
  Q65 yields ten, FST4 five, others one.
* `by_name("Q65-60D")` — exact-match name lookup.
* `for_protocol_id(id)` — first entry sharing the id; convenient for
  the single-mode-per-family case.

The family / sub-mode distinction matters most for Q65: all ten
sub-modes share `ProtocolId::Q65` but live as distinct registry entries
because their NSPS, tone spacing and slot length differ.

The registry is built by an internal `protocol_meta!` macro in
`mfsk-core/src/registry.rs`; adding a protocol is one line per ZST plus
its display name.

### 8.2 The generic trait-surface checker

`tests/protocol_invariants.rs` runs one generic
`assert_protocol_invariants::<P>` over every wired ZST — 24 of them:
20 WSJT-family, plus `uvpacket`'s four — and pins ~25 trait-level
invariants, among them `FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` and the
`GRAY_MAP` length contract `[2^BITS_PER_SYMBOL, NTONES]`.

A new protocol gets one line there. MSK144 and JTTY do not appear,
because neither implements `Protocol` — that is an architectural
decision, not a gap to close.

---

## License

See [`LICENSE`](../../LICENSE) at the repository root. Every algorithm
here derives from WSJT-X (Joe Taylor K1JT and collaborators), and each
source file cites the `lib/*.f90` / `lib/*.c` it ports.
