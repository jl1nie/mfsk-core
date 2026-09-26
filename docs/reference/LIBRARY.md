# mfsk-core — Rust API reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

A pure-Rust reimplementation of the WSJT-X weak-signal decoders — FT8,
FT4, FST4, WSPR, JT9, JT65, Q65, MSK144 and JTTY — plus the
experimental, non-WSJT `uvpacket` mode, behind one generic core. The
core (`engine` / `fec` / `msg`) is protocol-agnostic; each protocol is
a zero-sized type that plugs a FEC codec, a message codec and a sync
mode into it. One receive flow runs for every wired protocol —
`coarse-sync → refine → LLR → FEC decode → message unpack` — with
per-protocol strategy variations layered on top. MSK144 and JTTY sit
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
- [2. The decode API](#2-the-decode-api)
  - [2.1 `DecodeRequest<P>`](#21-decoderequestp)
  - [2.2 `SniperRequest<P>`](#22-sniperrequestp)
  - [2.3 Compute budget](#23-compute-budget)
  - [2.4 Streaming delivery](#24-streaming-delivery)
  - [2.5 Protocols with their own entry point](#25-protocols-with-their-own-entry-point)
  - [2.6 Message acceptance](#26-message-acceptance)
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
mfsk-core = { version = "0.12", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below enable
several for illustration.

**Decode an FT8 slot.** Synthesise a frame, then decode it back:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. Decode it back. new(audio, freq_min, freq_max, sync_min, max_cand).
// OSD defaults to on; call `.osd(false)` for a cheaper BP-only decode.
let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3_000.0, 1.0, 50)
    .decode()
    .results;
for r in &results {
    if let Some(text) = unpack77(r.message77()) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

Real audio arrives as a 15-second slot at 12 kHz. `engine::dsp::resample`
converts other sample rates; `DecodeRequest` takes `&[i16]`.

---

## 2. The decode API

`DecodeRequest` and `SniperRequest` in `mfsk_core::msg::decode_request`
are the **public** decode entry point for FT8, FT4 and every FST4
sub-mode (the `FrameDecodable` marker trait).

The engine functions underneath (`decode_frame`,
`process_candidate_basic`, the `GenericPipelineProtocol` trait) are
`pub(crate)` since issue #191/#203, so downstream cannot bypass the
request types. The non-default `internal-testing` feature reopens them
for the crate's own integration tests.

Q65, WSPR, JT65, JT9, uvpacket and JTTY keep their own entry points —
[§2.5](#25-protocols-with-their-own-entry-point) (MSK144's is
`msk144::decode::decode_slot`).

### 2.1 `DecodeRequest<P>`

Wide-band search over `freq_min..freq_max`. Construct, chain, decode:

```text
DecodeRequest::<P>::new(audio, freq_min, freq_max, sync_min, max_cand)
    .osd(true)
    .decode()          // -> DecodeOutcome<P>
```

`DecodeOutcome<P>` carries `.results: Vec<P::DecodeResult>`,
`.fft_cache` for a follow-up call, and `.budget: BudgetReport`.

| method | takes | default | available on | effect |
|---|---|---|---|---|
| `new` | `(audio, freq_min, freq_max, sync_min, max_cand)` | — | all | the wide-band search |
| `.freq_hint(hz)` | `f32` | unset | all | prioritise candidates near this frequency. **Also the QSO frequency (`nfqso`) for the a-priori passes:** with an `.ap_hint()` that locks both callsigns, those hypotheses are tried only for a candidate within 50 Hz of it, and not at all without it (`ft4_decode.f90` / `ft8b.f90` always have an `nfqso`). FT4 also decodes candidates in that window with a third OSD snapshot (`maxosd = 3`). CQ and MyCall-only hypotheses run anywhere |
| `.previous_cycle(&[..])` | decoded rows | empty | FT8 | this sequence's decodes of one cycle earlier (the slot 30 s before). Turns on WSJT-X's **a7** list decoder: for each of those pairs, the messages it could send next are matched at its old frequency and DT (pass id 30). **a8** needs no call: it runs with an `.ap_hint()` holding MyCall, HisCall and HisGrid plus a `.freq_hint()` (pass id 31) |
| `.tx_freq(hz)` | `f32` | unset | FT8 | the transmit frequency (`nftx`): FT8 also tries the both-callsigns hypotheses within 50 Hz of it. Ignored by the other modes (`ft4_decode.f90` has no `nftx`) |
| `.osd(bool)` | `bool` | `true` | all | OSD fallback when the BP staircase fails. `LlrEffort` is always `Full` for host decodes |
| `.strictness(s)` | `DecodeStrictness` | `Normal` | all | accept/reject threshold profile — see [§6](#6-engine-primitives) for which protocols each knob actually reaches |
| `.eq_mode(m)` | `EqMode` | `Off` | all | `Off` / `Local`. A property of the **input audio**, not the search |
| `.known(&[..])` | decoded rows | empty | all | skip or subtract messages already found in an earlier pass |
| `.fft_cache(c)` | cache from a previous `DecodeOutcome` | none | all | reuse the forward FFT over the same audio |
| `.noise_blanker(nb)` | `NoiseBlanker` | off | `SupportsNoiseBlanker` — **every FST4 sub-mode** | WSJT-X's **NB** (`blanker.f90`): zero the loudest samples before the slot FFT. `Percent(n)` blanks `n` % (0..=25); `Sweep { step, ftol_hz }` decodes at 0, step, … 20 %, the levels above 0 only within `ftol_hz` of `.freq_hint()` (up to 21 decodes). Off, like WSJT-X's default NB 0 % |
| `.ap_hint(&ApHint)` | `&ApHint` | none | `SupportsWideBandAp` — **FT8, FT4, every FST4 sub-mode** | lock message bits from an a-priori hypothesis |
| `.sic_rounds(n)` | `usize`, clamped `1..=3` | none | `SupportsSicRounds` — **FT8, FT4** | flat successive-interference cancellation |
| `.sic_early()` | — | none | `SupportsSicEarly` — **FT8** | checkpoint-emulation early decode, fixed 3-checkpoint structure |
| `.also_accept(f)` | `Fn(&Wsjt77Fields) -> bool` | none | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | accept what the codec accepts **plus** what `f` accepts — [§2.6](#26-message-acceptance) |
| `.message_filter(f)` | `Fn(&Wsjt77Fields) -> bool` | none | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | replace the codec's verdict with `f` — [§2.6](#26-message-acceptance) |
| `.codec_filter()` | — | on for FT8, off elsewhere | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | apply the codec's own verdict on a protocol that does not by default — [§2.6](#26-message-acceptance) |
| `.contest(on)` | `bool` | `false` | **FT8** | WSJT-X's `ncontest != 0`: keep `/R` and `TU; ` messages, which FT8 otherwise drops after the CRC — [§2.6](#26-message-acceptance) |
| `.on_result(cb)` | `FnMut(&Row)` | none | all | deliver rows as they are found — [§2.4](#24-streaming-delivery) |
| `.budget(check)` | `FnMut() -> bool` | none | all | caller-supplied deadline predicate — [§2.3](#23-compute-budget) |
| `.sniper(...)` | `(audio, target_hz, max_cand)` | — | `SupportsSniper` — **FT8** | build a `SniperRequest` instead |
| `.decode()` | — | — | all | run it |

**`DecodeRequest::<Ft8>::wsjtx_depth(audio, freq_min, freq_max, sync_min,
max_cand, tier, ap)`** is a second *constructor* (not a method), FT8 only,
in `ft8::decode`. It builds the request whose (OSD, SIC strategy, AP)
triple mirrors `jt9 -d1/-d2/-d3`: `WsjtxDepth::D1` is OSD off plus
`.sic_rounds(2)`, `D2` is OSD plus `.sic_early()`, and `D3` adds
`.ap_hint(ap)` (`ap: Option<&ApHint>` is read for `D3` only, ignored for
`D1`/`D2` as `jt9`'s own depth/AP coupling). `D1` and `D2` also raise the
hard-sync (nsync) floor a candidate must clear from 6 (7 in the
squared-metric passes) to 8, as WSJT-X 3.0's `ndepth <= 2` does (#439).
`sync_min` stays yours: on the busy-band corpus 1.3 keeps `.sic_early()`'s
recall while cutting its unexpected decodes from 22 (at 0.8) to 6, and
2.1, WSJT-X 3.x's value for `-d1/-d2`, costs 3-4 points of recall there
([`BENCHMARKS.md`](../notes/BENCHMARKS.md), "The busy-band corpus").

**The strategy extensions are where phantom decodes come from.** Both
false-decode bugs this suite has shipped were in subtraction paths
(#243 in `__staged_sic`, #253 in `.sic_early()`), so a new strategy
ships with its precision guard in the same PR.

### 2.2 `SniperRequest<P>`

Narrow-band, single-target search, gated on `SupportsSniper` and
**implemented for `Ft8` alone**.

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{EqMode, ApHint};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::msg::decode_request::SniperRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1000.0, /* amp */ 20_000);
let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
audio[start..start + frame.len()].copy_from_slice(&frame);

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
let results = SniperRequest::<Ft8>::new(&audio, /*target_hz*/ 1000.0, /*max_cand*/ 15)
    .eq_mode(EqMode::Local)
    .ap_hint(&ap)
    .decode()
    .results;
assert!(!results.is_empty(), "roundtrip must decode");
for r in &results {
    let text = unpack77(r.message77()).unwrap();
    println!("{:7.1} Hz  {}", r.freq_hz, text);
}
```

| method | default | effect |
|---|---|---|
| `new(audio, target_hz, max_cand)` | — | ±250 Hz around `target_hz` |
| `.search_hz(w)` | 250 Hz | widen or narrow the window |
| `.sync_min(v)` | mode default | sync threshold |
| `.osd(bool)` | `true` | as `DecodeRequest` |
| `.strictness(s)` | `Normal` | as `DecodeRequest` |
| `.eq_mode(m)` | `Off` | as `DecodeRequest` |
| `.ap_hint(&h)` | none | as `DecodeRequest` |
| `.also_accept(f)` | none | as `DecodeRequest` |
| `.message_filter(f)` | none | as `DecodeRequest` |
| `.codec_filter()` | on (FT8) | as `DecodeRequest` |
| `.on_result(cb)` | none | as `DecodeRequest` |
| `.budget(check)` | none | as `DecodeRequest` |
| `.decode()` | — | same `DecodeOutcome<P>` shape |

There is no SIC variant — a sniper search is inherently
single-candidate.

**What makes this sniper mode is the window, not the hint.** The
±250 Hz search exists because the operator narrowed the transceiver's
*analogue* roofing filter — the Yaesu FTDX101MP and FTDX10 are the
usual examples at ~500 Hz — and pointed it at a station whose carrier
is already known. The audio arriving is already band-limited; the
decoder is matching the hardware. It is **not** a general "hunt one
known station" convenience.

**`.ap_hint()` works here** — it is in the table above, and
`SniperRequest`'s `.decode()` passes it through to the decoder. But AP
is orthogonal to the window, not part of what sniper *is*: the same
hint reaches the wide-band `DecodeRequest` for FT8, FT4 and every FST4
sub-mode. The two were once coupled — AP was reachable only through an
engine whose candidate loop broke on `if has_ap`, so *holding a hint*
was what made a search single-target — and that coupling is what was
removed, not AP's availability here.

FT4 and FST4 sniper entry points existed until 2026-09-13 and were
removed: the wide-band path is the main path for every mode here, and
if it is not WSJT-X-faithful without a sniper, that is a bug in the
wide-band path. Full reasoning, with the measurements, in
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md).

### 2.3 Compute budget

`.budget(check)` takes a caller-supplied predicate polled between
candidates. **The library reads no clock of its own** — the deadline is
whatever your predicate compares against, which is what keeps it usable
from wasm and from a process that was suspended mid-slot.

`DecodeOutcome::budget` is a `BudgetReport` saying what the cut left
undone: candidates skipped, stages run, and how good the best skipped
candidate was — so a caller can tell "nothing was there" from "we ran
out of time with a promising candidate still queued".

Honoured by FT8, FT4 and every FST4 sub-mode
(`MFSK_CAP_BUDGET` is the same fact published to C).

### 2.4 Streaming delivery

`.on_result(cb)` delivers each row as it is found, on top of the `Vec`
the call returns — for a UI that wants something on screen before a
long slot finishes.

The delivery-order and de-duplication contract is **not repeated
here**: [`STREAMING.md`](STREAMING.md) is the authoritative account,
and `DecodeRequest::on_result`'s own doc comment is the normative one.
In one line: sequential decoding delivers exactly the rows the call
returns, in the same order; parallel decoding delivers in completion
order and may show a transient duplicate that the returned `Vec` has
already deduped.

Every protocol offers the same shape through its own entry point:
`.on_result(cb)` on `wspr::DecodeRequest`, `jt9::DecodeRequest`,
`jt65::DecodeRequest` and `q65::{DecodeRequest, SniperRequest,
MultiPeriodRequest}`. WSPR's is the parallel contract, not the exact
one — see [`STREAMING.md`](STREAMING.md) §3b. JTTY has no request
builder and delivers by callback from inside the audio call:
`jtty::rx::Stream::push(samples, &mut |update| …)` (and `finish`) call it
on the caller's thread — [§2.5](#25-protocols-with-their-own-entry-point).

### 2.5 Protocols with their own entry point

**WSPR** decimates the slot to wsprd's 375 Hz baseband and runs
wsprd's own coarse search and three decode passes there, rather than
the shared FT-style pipeline. The FEC (`ConvFano`) and message codec
(`Wspr50Message`) are still associated types on `impl Protocol for
Wspr`, so the trait surface stays consistent — only the slot-level
decoder differs. Its entry points are `wspr::DecodeRequest` and
`wspr::SniperRequest` (issue #403), which replaced 14 free functions.

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::wspr::DecodeRequest;
use mfsk_core::wspr::tx::synthesize_type1;
use mfsk_core::msg::WsprMessage;

// Synthesise a WSPR Type 1 frame (120 s @ 12 kHz slot).
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let decodes = DecodeRequest::new(&samples_f32, /*sample_rate*/ 12_000).decode();
assert!(!decodes.is_empty(), "roundtrip must decode");
for d in decodes {
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

`DecodeRequest::new` runs the (frequency × time × drift) coarse search
over the whole slot. It also takes `.nominal_start()`, `.params()`,
`.on_result()` and `.table(&mut WsprCallsignTable)`: a table carried
from slot to slot lets OSD re-find a station a previous slot's Fano
decode confirmed, which is how `wsprd` reaches W3BI at −25 dB on its
own sample file.

If the frequency and start sample are already known,
`DecodeRequest::sniper(samples, rate, start_sample, freq_hz).decode()`
bypasses the scan. `SniperRequest::baseband(idat, qdat, …)` does the
same on a baseband the caller already decimated, and takes `.drift()`,
`.nblocks()`, `.confirmed()` and `.refine_drift()` — the knobs the
scan's own passes set per candidate. That is what the CoreS3 WSPR
receiver drives from its own candidate loop.

**JT9** has one builder, `jt9::DecodeRequest` (issue #403), in the
same shape as Q65's below but not generic, since JT9 has one sub-mode.
`DecodeRequest::new(audio, sample_rate)` scans the whole buffer with
`jt9::search::default_search_params()`; `.nominal_start()`,
`.params()`, `.depth(Jt9Depth)` and `.on_result()` adjust it, and
`DecodeRequest::sniper(audio, rate, start_sample, freq_hz).decode()`
is the point decode at a known alignment. It returns the
`Jt72Message` alone: that path has no sync search, AFC or SNR estimate
to report. The six `decode_scan*` / `decode_at` free functions this
replaced are gone.

```rust
# #[cfg(feature = "jt9")] {
use mfsk_core::jt9::{DecodeRequest, Jt9Depth};
use mfsk_core::jt9::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
    .expect("pack + synth");
let decodes = DecodeRequest::new(&audio_f32, 12_000)
    .depth(Jt9Depth::Deep)
    .decode();
assert!(!decodes.is_empty(), "roundtrip must decode");
# }
```

**JT65** has the same pair, `jt65::DecodeRequest` and
`jt65::SniperRequest` (issue #403), replacing nine free functions. The
axis JT65 adds is how Reed-Solomon runs: hard-decision by default,
`.chase(ChaseParams)` on either builder for the stochastic Chase
search, and `.erasures(&[0, 8, 16, 24, 32])` on the sniper for the
deterministic erasure ladder. On the sniper the last of the two called
wins.

```rust
# #[cfg(feature = "jt65")] {
use mfsk_core::jt65::DecodeRequest;
use mfsk_core::jt65::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let decodes = DecodeRequest::new(&audio_f32, 12_000).decode();
assert!(!decodes.is_empty(), "roundtrip must decode");
for d in decodes {
    println!("{:7.2} Hz  {:+.0} dB  {}", d.freq_hz, d.snr_db, d.message);
}
# }
```

The Chase search (`jt65::chase`, issue #169) is a faithful port of
WSJT-X's `ftrsdap` stochastic Chase decoder, magic numbers included.
On the AWGN sweep it moves the 50% crossing from −22.5 to −23.5 dB, at
the cost of up to `ChaseParams::max_trials` RS attempts per candidate
that does not decode at once.

**Q65** has three generic builders in `mfsk_core::q65::decode_request`,
mirroring `msg::decode_request`'s shape and generic over a sealed
`Q65SubMode` marker implemented for all ten sub-mode ZSTs:
`DecodeRequest<P>` (wide-band scan), `SniperRequest<P>` (a known
`(start_sample, base_freq_hz)`), and `MultiPeriodRequest<P>` (averaged
multi-slot). `.ap_hint()`, `.ap_list()` and `.fading()` are plain
inherent methods rather than capability-gated traits, since every Q65
sub-mode supports every capability uniformly. The underlying
`q65::rx` functions are `pub(crate)`. Which builder takes which method
(`q65/decode_request.rs`; a method a builder lacks is a compile error,
not a silent no-op):

| method | `DecodeRequest` | `SniperRequest` | `MultiPeriodRequest` |
|---|---|---|---|
| `.ap_hint(&ApHint)` | yes | yes | no |
| `.ap_list(&codewords)` | yes | yes | yes |
| `.fading(model, b90_ts)` | yes | yes | no |
| `.pileup(bool)` | yes | yes | no |
| `.max_drift(bins)` | yes | no | no |
| `.eme_delay(bool)` | yes | no | yes |
| `.rx_freq(hz)` / `.ftol(hz)` | yes | no | no |
| `.hash_table(Arc<CallsignHashTable>)` | yes | yes | yes |
| `.on_result(cb)` | yes | yes | yes |
| `.decode()` returns | `Vec<Q65Result>` | `Option<Q65Result>` | `Vec<Q65Result>` |

`.hash_table()` is the session `CallsignHashTable` that resolves
`<...>` hashed-callsign (Type 4) placeholders; unset, they stay
unresolved. It is `Arc`-shared, so passing the same table to every
`decode()` of a session is a refcount bump, and the caller owns and grows
it.

**`dt_sec`, `SearchParams` and `SyncCandidate`.** For WSPR, JT9, JT65 and
Q65 the result's `dt_sec` runs from the **nominal start** — the
request's `nominal_start` sample (WSPR: its fixed 1.0 s `TX_START_OFFSET_S`) — and is signed, so
a frame that begins early reads negative (#397; it was measured from the
buffer start for Q65 and JT65, and a 0.5 s early frame read −0.013 s and
+0.442 s). `to_decoded` on `Q65Result` / `Jt65Result` / `Jt9Result` reads
that field and no longer takes `(sample_rate, nominal_start_sample)`;
`Jt9Result` gained `dt_sec`; `msg::decoded::dt_from_samples` is gone.
**Breaking in 0.12**, with the rest in
[0.12 breaking changes](#012-breaking-changes). The scan modes share one
coarse-search vocabulary in `engine::search` (#394):
`SearchParams { freq_min_hz, freq_max_hz, time_tolerance_early_sec,
time_tolerance_late_sec, score_threshold, max_candidates }` (the window
is an early/late pair in **seconds** because Q65's is asymmetric;
`SearchParams::symmetric(..)` sets both) and `SyncCandidate { start_sample,
freq_hz, score }`, where `freq_hz` is tone 0 and `.dt_sec(nominal, rate)`
converts. There is no `SearchParams::default()`: the defaults are
mode-specific data, so each mode has `search::default_search_params()`
(Q65: 200-3000 Hz, ±1.0 s, 8 candidates, threshold 0.1). FT8, FT4 and
FST4 keep `engine::sync::SyncCandidate`, which carries `dt_sec` instead
of `start_sample`, on purpose.

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

Three builder methods, all on `SupportsMessageFilter` — **`Ft8`, `Ft4`
and every FST4 sub-mode**:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;

/// Whatever the deployment knows and the ITU allowlist does not.
fn is_special_event_call(call: &str) -> bool {
    call.starts_with("8J")
}

let audio = vec![0i16; 180_000]; // 15 s @ 12 kHz

// The codec verdict, plus callsigns it does not know about. The
// closure sees the decoded message, so `callsigns()` is exactly the
// callsign fields — never a grid or a report.
let widened = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 20)
    .also_accept(|m| m.callsigns().all(is_special_event_call))
    .decode();

// No opinion at all — every CRC-passing message, phantoms included.
// This is WSJT-X's own acceptance rule with nothing on top.
let unfiltered = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 20)
    .message_filter(|_| true)
    .decode();

// Silence carries neither real signals nor CRC survivors, so even the
// filterless run comes back empty.
assert!(widened.results.is_empty());
assert!(unfiltered.results.is_empty());
```

`.also_accept(f)` widens the verdict and can only add. `.codec_filter()`
applies the verdict and nothing else — the one-line way to get it on a
protocol that does not run it by default. `.message_filter(f)` replaces
it outright, and the thing it replaces removes roughly two thirds of
the CRC survivors that reach it, so a permissive `f` will surface
phantom rows.

**FT8 also drops `/R` and `TU; ` messages, whatever the policy.** Since
#439 FT8 does what `ft8b.f90` (WSJT-X 3.0 onward) does right after the
CRC: with no contest active, a standard or RTTY Roundup message carrying
`/R` or starting `TU; ` is discarded and the pass moves on. It sits
before the policy, so `.message_filter(|_| true)` does not bring those
rows back; `.contest(true)` does, and is the right setting for a
contest, where `CALL1/R CALL2` and `TU; CALL1 CALL2` are real traffic.

**On by default for FT8 and FT4, and the reason is subtraction.** A
wrong decode is not a cosmetic error on a path that subtracts what it
accepts: `.sic_rounds()` and `.sic_early()` remove the decoded
waveform from the audio before looking again. Measured on
`qso3_busy.wav`, with the verdict off `.sic_early()` accepts the
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
`.also_accept()`.

**FST4 leaves it off**, and not for want of measuring: CRC-24 puts its
false-positive rate 512x below the other two, so there is little for
the verdict to remove and the recall it could cost is the same.

**Zero-cost when unused.** The policy is a type parameter, not a
`&dyn Fn` like `.on_result()` and `.budget()`: a request that names
none of the three carries `DefaultPolicy`, a zero-sized type, and for
a protocol that does not filter by default the message is not even
decoded — both conditions are compile-time constants. Those two hooks
fire once per *decode*; this one fires once per candidate that reaches
the message stage, which is why it is worth the type parameter.

---

## 3. Protocols

### 3.1 Generic vs bespoke, per protocol

The map to read first. Each cell says whether that layer is **shared**
(reused verbatim from the generic core) or **own** (code in the
protocol's own module).

| Protocol | FEC codec | Message codec | Sync mode | Decode entry point |
|----------|-----------|---------------|-----------|--------------------|
| **FT8**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 3×Costas-7 | generic `DecodeRequest`, dispatching to FT8's own `ft8::decode_block` engine [^ft8] |
| **FT4**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 4×Costas-4 | generic `DecodeRequest` / `engine::pipeline` |
| **FST4** | shared `Ldpc240_101` | shared `Wsjt77Message` (77-bit) | `Block` — 5×Costas-8 | generic `DecodeRequest` / `engine::pipeline` |
| **WSPR** | own `ConvFano` (conv r=½ K=32 + Fano) | own `Wspr50Message` (50-bit) | own `Interleaved` [^wspr] | bespoke `wspr::decode` |
| **JT9**  | own `ConvFano232` (conv, 206-bit framing) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | bespoke `jt9` entry |
| **JT65** | own `Rs63_12` (RS GF(2⁶), erasure-aware) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | bespoke `jt65` entry |
| **Q65**  | own `Q65Fec` + QRA codec over GF(64) [^q65] | own `Q65Message` (77-bit) | `Block` | bespoke `q65::rx` + Q65-local `DecodeRequest` |
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
  mode whose receiver is incremental — [§2.5](#25-protocols-with-their-own-entry-point).

> This table is the source of truth that
> `mfsk-core/tests/common_selftest.rs`'s code-sharing ratchet, the
> `README.md` sharing paragraph and `lib.rs`'s own docs all trace back
> to. Change it and those change with it.

[^ft8]: FT8 uses the generic `DecodeRequest` builder like FT4/FST4, but
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
  the squared-metric passes, `> 8` for `WsjtxDepth::D1/D2`), and drops
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
  (9.7e-5 after). Within 50 Hz of `.freq_hint()` FT4 also takes a third
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
  `.noise_blanker()` is WSJT-X's **NB** ([§2.1](#21-decoderequestp)): on 50
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
  and [§2.5](#25-protocols-with-their-own-entry-point). Its constants
  live in `jtty` (`NSPS`, `SYNC_SYMBOLS`, `FRAME_SYMBOLS`, `MAX_FRAMES` =
  16), not in trait constants. It has its own 1-based GFSK pulse in
  `jtty::tx`, not `engine::dsp::gfsk`: that pulse sat one sample early
  until #482, which JTTY's waveform check (4.9e-2 against upstream) exposed.

### 3.4 Decoder strategies

Every protocol runs the same underlying flow; the *strategy* wrapped
around it varies. Most are a single pass. Only Q65 exposes several
parallel receiver chains for one FEC frame, MSK144 replaces the
slot model with a burst scan, and JTTY with an incremental receiver.

| Protocol | Default strategy | Optional strategies |
|----------|------------------|---------------------|
| **FT8**  | single-pass BP + OSD | AP iaptype loop (1–12); SIC 1–3 rounds; `.sic_early()`; sniper; the **a7 / a8 list decoders** (pass ids 30 / 31, run at the end of every FT8 strategy; a7 via `.previous_cycle()`, a8 via an `.ap_hint()` with MyCall, HisCall, HisGrid plus `.freq_hint()`); `wsjtx_depth(…)` presets |
| **FT4**  | single-pass BP + OSD | SIC 1–3 rounds; full-slot coherent sync (`sync2d`) |
| **FST4** | single-pass BP + OSD | full-slot two-stage coherent sync search; noise blanker (`.noise_blanker()`, fixed % or sweep) |
| **WSPR** | single bespoke pass (quarter-symbol spectrogram scan) | — |
| **JT9**  | single bespoke pass | — |
| **JT65** | single bespoke pass | RS erasure decode; stochastic Chase decoder |
| **Q65**  | `(Δf,Δt,b90)` grid + Lorentzian fading BP (scan) | AP-hint, explicit fast-fading, AP-list, multi-period; **q3** list decode (`.ap_list().rx_freq()`); Max Drift; Pileup; EME delay |
| **MSK144** | burst scan over the whole T/R period | — |
| **JTTY** | streaming: sync surface, candidates, four-rung list-WAVA ladder, gate; decoded frames subtracted, retro re-sweep, frames assembled into messages | `Params::subtract` off (single-signal receiver) |

**A-priori decoding is a general option, not a sniper feature.** AP is
the last rung of the per-candidate ladder — `process_candidate_basic`'s
for FT4 and every FST4 sub-mode, FT8's own for FT8 — and
`msg::pipeline_ap` is hypothesis generation with no engine of its own. It used to be coupled to the sniper by accident
and that cost most of the decodes — the measurement is in
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md).

**Picking a Q65 strategy:**

| When | Strategy | Builder call | Threshold gain |
|---|---|---|---|
| Single known candidate, unknown content | AWGN Bessel + BP (point-decode only) | `SniperRequest::<P>::new(...).decode()` | baseline |
| Default scan — unknown channel, unknown content | `(Δf,Δt,b90)` grid search + Lorentzian fading BP | `DecodeRequest::<P>::new(...).decode()` | WSJT-X-faithful default |
| Known callsign(s) or report, terrestrial channel | AP-hint BP | `.ap_hint(&ap)` on either builder | ~2 dB |
| Doppler-spread channel, explicit model (microwave EME, ≥10 Hz spread) | Fast-fading metric + BP, caller-picked `(b90_ts, FadingModel)` | `.fading(model, b90_ts)` on either builder | 5–8 dB on spread channels |
| Known call pair, no QSO context, terrestrial | AP-list template matching | `.ap_list(&candidates)` on either builder | ~3 dB |
| Known call pair and an Rx frequency (WSJT-X's q3) | 85-symbol sync of every list message near the Rx frequency, then list decode | `.ap_list(&codewords).rx_freq(hz)` (+ `.ftol(hz)`) on `DecodeRequest` | `q65sim` Q65-30A, 20 files a level at −24 / −26 / −28 / −30 dB: 20 / 20 / 7 / 2, `jt9 -3 -d 1` the same on the same files |
| Weak / ionoscatter signal spanning several T/R periods | Multi-period EMA averaging (3-stage cascade) | `MultiPeriodRequest::<P>::new(...).decode()` | recovers signals no single-period strategy can |

`.ap_list()` and `.fading()` are mutually exclusive in the underlying
engine; `.decode()` resolves precedence as
`ap_list > fading (+ ap_hint) > ap_hint > plain`.
`q65::Q65History` is WSJT-X's `q65_hist`, held by the application:
`.record(&result)` after each decode (it keeps the last 100), and
`.lookup(rx_freq_hz)` returns the DX call — plus the grid when the message
carries one — from the most recent decode within 10 Hz. WSJT-X does this
on a manual Decode Again with no DX call entered, to build the full-AP
list (`standard_qso_codewords`) without the operator typing the call.
`q65::Q65Callers` and `contest_codewords` are the contest-mode variant
(`q65_hist2` / `q65_set_list2`): up to 50 stations that called with a
grid, kept by the application (`record(freq, msg, now)`, `expire(now)`),
and a full-AP list of every `MyCall Caller Grid` / `R Grid` / `RRR` /
`RR73` / `73` with the 78th bit clear and set, to pass to `.ap_list()`.
`MultiPeriodRequest` takes `&[&[f32]]`, one buffer per T/R slot, and is
Rust-only — not in the C ABI. What each front end actually does, and
why the default scan is not the plain Bessel path, is in
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for).

**Q65 Pileup (WSJT-X 3.2).** A Pileup station sets Q65's spare 78th
payload bit to say it copied its correspondent's last transmission.
`Q65Result::copied_last_tx` reports it (WSJT-X marks the line `#`), and
`encode_channel_symbols_flagged` / `synthesize_standard_flagged_for` send
it. `.pileup(true)` on either builder applies upstream's AP policy for
that mode: an `.ap_hint()` naming both callsigns and nothing after them
leaves the bit free instead of locking it to 0, so a flagged reply still
matches. Without it such a hint rejects a flagged reply, exactly as
WSJT-X outside Pileup does. The `.ap_list()` templates carry the bit
clear, as `q65_set_list.f90` builds them.

**Q65 Max Drift.** `.max_drift(bins)` on `q65::DecodeRequest` is
WSJT-X's Max Drift setting (0..50, off by default). The sync search tries
a linear tone drift of up to `bins` spectrum bins (one bin = one baud)
across the frame (`q65_ccf_22`), and the grid decode takes the drift it
found back out (`q65_loops`' `twkfreq`). It costs `2*bins+1` times the
plain search per frequency bin; WSJT-X narrows its window to the Rx
frequency ± F Tol while it is on, so narrow `SearchParams` to match. It
applies to the plain and `.ap_hint()` scans.

**Q65 time window and EME delay.** `default_search_params()` searches
-1.0 .. +1.0 s around the nominal start, as WSJT-X's GUI does
(`q65.f90`'s `lag1`/`lag2`). `.eme_delay(true)` on `q65::DecodeRequest` or
`MultiPeriodRequest` is its "Decode at 52 s" EME delay: the late edge moves
to +5.5 s (+4.0 s on Q65-15) for the Earth-Moon-Earth round trip. `dt_sec`
is measured from the nominal start on both requests; a `SniperRequest`,
which has none, measures it from the start of the buffer.

**Q65 q3 list decoding.** `.ap_list(&codewords).rx_freq(hz)` (with
`.ftol(hz)`, default 10 Hz, the `jt9` CLI's) is WSJT-X's q3 decode:
the 85-symbol sync of every list message within F Tol of the Rx frequency
(`q65_ccf_85`), then the list decode with the fast-fading metric over the
`b90` sweep (`q65_dec_q3`). It runs first, and the scan runs after it for
the rest of the band. At `.max_drift(50)`, when nothing decoded at the Rx
frequency, it runs again on spectra with the drift found there taken out
(the "w3sz" stage 5). Without `.rx_freq()`, `.ap_list()` is this crate's
own per-candidate template match instead of the scan.

---

## 4. Module and crate map

```text
mfsk_core
├── engine/           Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · cpfsk · envelope · subtract ·
│   │                   msk · analytic · ddc · fir_decimate · polyphase · dotprod ·
│   │                   symbol_fft · blanker · fixed-point FFT kernels
│   ├── fft.rs          FftPlanner trait + the extern factory (see EMBEDDED.md)
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
│                       msg::decode_request::DecodeRequest/SniperRequest)
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
├── msg/              Message codecs and the public decode API
│   ├── decode_request.rs DecodeRequest / SniperRequest — §2
│   ├── decoded.rs      Decoded — the public output row
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
├── ft8/              FT8 ZST + decode + decode_block + wave_gen
│   ├── list_decode.rs  WSJT-X's a7 / a8 list decoders (pass ids 30 / 31)
│   └── acquire.rs      cold slot-phase acquisition from off-air audio (#356)
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 family — 5 sub-mode ZSTs (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search + ddc
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ erasure-aware RS, chase)
├── q65/              Q65 family — 10 sub-mode ZSTs + decode + synth
│   ├── decode_request.rs DecodeRequest / SniperRequest / MultiPeriodRequest (§2.5)
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
   the `DecodeRequest`/`SniperRequest` builders that drive the pipeline.
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
| Same FEC and message as an existing mode (another FST4 sub-mode) | a new ZST with different numeric constants; `Fec`/`Msg` are type aliases. The full `DecodeRequest::<P>` pipeline runs unchanged |
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
| `fir` / `dotprod` | polyphase FIR and the dot-product kernel the extern hook replaces |

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
hand-rolled non-default usage, but `DecodeRequest::<Ft8>` and
`SniperRequest::<Ft8>` dispatch via `decode_block::coarse_sync`
internally. That is why an FT8 coarse-sync change cannot move an FST4
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
`internal-testing`. Use `DecodeRequest`/`SniperRequest`.

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

### 0.12 breaking changes

What a 0.11 caller has to change (full text and migration tables:
`CHANGELOG.md`, `## 0.12.0`). Output is bit-identical unless noted.

| area | 0.11 | 0.12 |
|---|---|---|
| decode entry, JT9 (#403) | `decode_scan*`, `decode_at` (6 functions) | `jt9::DecodeRequest::new(..).decode()`, `::sniper(..)` |
| decode entry, JT65 (#403) | `decode_scan*`, `decode_scan_chase*`, `decode_at*`, `chase::decode_at_with_chase` (9) | `jt65::DecodeRequest` / `SniperRequest`, `.chase(..)`, `.erasures(..)` |
| decode entry, WSPR (#403) | 14 functions, `decode_at_baseband_nblocks_gated_drift` and kin; `decode_scan_subtract*` public | `wspr::DecodeRequest` / `SniperRequest`; the SIC pair only behind `internal-testing` |
| synthesis (#391) | `ft8::wave_gen::tones_to_*`, `ft4::encode::*`, `fst4::encode::*`, `wspr::tx::synthesize_audio`, `q65::synthesize_audio_for` … | `engine::tx::synthesize::<P>` / `synthesize_into` / `synthesize_i16[_into]` / `synth_len`, over `FskWaveform` |
| tones (#391) | per-mode `message_to_tones`, FT8's on `&[u8]` → `[u8; 79]` | `engine::tx::message_to_tones::<P>(&[u8; 77]) -> Vec<u8>`; `DecodeResult::message77()` returns `&[u8; 77]` (compare with `*r.message77() == m77`) |
| Gray code (#391) | `jt65::{gray6, inv_gray6}` | `engine::gray::{gray, inv_gray}(n, bits)`; `fst4::encode::append_crc24` → `fec::ldpc240_101::append_crc24` |
| JT65 demodulator (#390) | four `demodulate_aligned*` functions returning tuples | `jt65::demodulate_aligned(..)?` returns a `Jt65Demod` (`.symbols`, `.conf`, `.second_symbols`, `.rel`, `.raw_pwr`, `.snr_db`) |
| `dt_sec` (#397) | Q65 / JT65 from the buffer start; JT9 had none | from the nominal start everywhere; `to_decoded` takes no arguments; `Jt9Result::dt_sec` new; `dt_from_samples` gone |
| search types (#394) | four `SearchParams` / `SyncCandidate`, `SearchParams::default()`, `time_tolerance_sec`, WSPR `time_tolerance_symbols` | `engine::search` re-exports; `default_search_params()` per mode; `time_tolerance_early_sec` / `_late_sec` in seconds |
| Q65 window | `default_search_params()` −1.0 … +5.5 s | −1.0 … +1.0 s as `q65.f90:127-130`; `.eme_delay(true)` restores the late reach |
| LDPC BP (#417) | `fec::ldpc::bp::bp_decode_nms`, `bp_decode_nms_q11`, `llr_f32_to_q11` | `bp_decode_nms_with_scratch`, or `bp_decode_generic_nms::<Ldpc174_91Params, T>`; `Q11i16::from_f32(x).0`; one body per kernel |
| `sync_cv` (#414) | FT8's a root of summed squares | the population CV on every protocol, so FT8's value is 1/√3 of what it was |
| FST4 OSD (#456) | searched all 101 bits | `osd_decode_npre_generic(.., partial_crc: Option<PartialCrc>)`; FST4 passes the (240, 91) subcode |
| default behaviour | FT4 `sync_min` 1.2 / `max_cand` 100; FT4 message policy off | 1.18 / 200 (#440); on (#383) |

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
same number: `CostasAbsolute` (FT8, FST4; noise has no fixed value),
`BaselineNormalised` (FT4: the spectrum is divided by a fitted baseline,
so noise sits at ~1.0 and any threshold below that admits every peak), and
`SyncFraction` (WSPR, JT9, JT65, Q65: sync power over sync plus noise, in
0‥1, default `DEFAULT_SCORE_THRESHOLD` = 0.1). `sniper_max_cand_cap` is the
bound the sniper path silently applies to `max_cand` (FT4: 15).

`profile.defaults` is what a caller that does not choose its own gets
(these are this crate's host-configuration values; the C ABI's
`mfsk_mode_defaults` returns the same):

| entry | band (Hz) | `sync_min` | `max_cand` | source |
|---|---|---|---|---|
| FT8 | 100-3000 | 0.8 | 60 | `FT8_PROFILE` |
| FT4 | 300-2700 | 1.18 | 200 | WSJT-X 3.x `ft4_decode.f90` `syncmin` / `MAXCAND` (#440; was 1.2 / 100) |
| FST4 (all five) | 100-3000 | 0.8 | 50 | `FST4_PROFILE` |
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
