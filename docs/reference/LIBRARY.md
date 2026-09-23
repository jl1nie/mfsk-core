# mfsk-core — Rust API reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

A pure-Rust reimplementation of the WSJT-X weak-signal decoders — FT8,
FT4, FST4, WSPR, JT9, JT65, Q65, MSK144 — behind one generic core. The
core (`engine` / `fec` / `msg`) is protocol-agnostic; each protocol is
a zero-sized type that plugs a FEC codec, a message codec and a sync
mode into it. One receive flow runs for every wired protocol —
`coarse-sync → refine → LLR → FEC decode → message unpack` — with
per-protocol strategy variations layered on top.

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
mfsk-core = { version = "0.11", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below enable
several for illustration.

**Decode an FT8 slot.** Synthesise a frame, then decode it back:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

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

Q65, WSPR, JT65, JT9 and uvpacket keep their own entry points —
[§2.5](#25-protocols-with-their-own-entry-point).

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
| `.freq_hint(hz)` | `f32` | unset | all | prioritise candidates near this frequency |
| `.osd(bool)` | `bool` | `true` | all | OSD fallback when the BP staircase fails. `LlrEffort` is always `Full` for host decodes |
| `.strictness(s)` | `DecodeStrictness` | `Normal` | all | accept/reject threshold profile — see [§6](#6-engine-primitives) for which protocols each knob actually reaches |
| `.eq_mode(m)` | `EqMode` | `Off` | all | `Off` / `Local`. A property of the **input audio**, not the search |
| `.known(&[..])` | decoded rows | empty | all | skip or subtract messages already found in an earlier pass |
| `.fft_cache(c)` | cache from a previous `DecodeOutcome` | none | all | reuse the forward FFT over the same audio |
| `.ap_hint(&ApHint)` | `&ApHint` | none | `SupportsWideBandAp` — **FT8, FT4, every FST4 sub-mode** | lock message bits from an a-priori hypothesis |
| `.sic_rounds(n)` | `usize`, clamped `1..=3` | none | `SupportsSicRounds` — **FT8, FT4** | flat successive-interference cancellation |
| `.sic_early()` | — | none | `SupportsSicEarly` — **FT8** | checkpoint-emulation early decode, fixed 3-checkpoint structure |
| `.also_accept(f)` | `Fn(&Wsjt77Fields) -> bool` | none | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | accept what the codec accepts **plus** what `f` accepts — [§2.6](#26-message-acceptance) |
| `.message_filter(f)` | `Fn(&Wsjt77Fields) -> bool` | none | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | replace the codec's verdict with `f` — [§2.6](#26-message-acceptance) |
| `.codec_filter()` | — | on for FT8, off elsewhere | `SupportsMessageFilter` — **FT8, FT4, every FST4 sub-mode** | apply the codec's own verdict on a protocol that does not by default — [§2.6](#26-message-acceptance) |
| `.on_result(cb)` | `FnMut(&Row)` | none | all | deliver rows as they are found — [§2.4](#24-streaming-delivery) |
| `.budget(check)` | `FnMut() -> bool` | none | all | caller-supplied deadline predicate — [§2.3](#23-compute-budget) |
| `.sniper(...)` | `(audio, target_hz, max_cand)` | — | `SupportsSniper` — **FT8** | build a `SniperRequest` instead |
| `.decode()` | — | — | all | run it |

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
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::SniperRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1000.0, /* amp */ 20_000);
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
`wspr::decode::{decode_scan_streaming, decode_scan_subtract_streaming}`,
`jt9::decode_scan_streaming`, `jt65::decode_scan_streaming`, and
`.on_result(cb)` on `q65::{DecodeRequest, SniperRequest,
MultiPeriodRequest}`.

### 2.5 Protocols with their own entry point

**WSPR** takes symbol-length FFTs directly at 12 kHz rather than
decimating to an FT-style baseband first, so its demodulation is staged
differently. The FEC (`ConvFano`) and message codec (`Wspr50Message`)
are still associated types on `impl Protocol for Wspr`, so the trait
surface stays consistent — only the slot-level decoder differs.

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::wspr::tx::synthesize_type1;
use mfsk_core::msg::WsprMessage;

// Synthesise a WSPR Type 1 frame (120 s @ 12 kHz slot).
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
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

`decode_scan_default` runs the (frequency × time) coarse search over
the whole slot. If the frequency and start sample are already known,
`wspr::decode::decode_at(samples, rate, start_sample, freq_hz)`
bypasses the scan.

**JT9 and JT65** expose the same scan + point-decode pattern:

```rust
# #[cfg(feature = "jt65")] {
use mfsk_core::jt65::decode_scan_default;
use mfsk_core::jt65::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let decodes = decode_scan_default(&audio_f32, 12_000);
assert!(!decodes.is_empty(), "roundtrip must decode");
for d in decodes {
    println!("{:7.2} Hz  {:+.0} dB  {}", d.freq_hz, d.snr_db, d.message);
}
# }
```

JT65 additionally offers `decode_at_with_erasures` (RS erasure
decoding) and, for deeper SNR, `decode_at_with_chase` /
`decode_scan_chase*` (`jt65::chase`, issue #169) — a faithful port of
WSJT-X's `ftrsdap` stochastic Chase decoder, magic numbers included.
Same call shape plus a `&ChaseParams` argument.

**Q65** has three generic builders in `mfsk_core::q65::decode_request`,
mirroring `msg::decode_request`'s shape and generic over a sealed
`Q65SubMode` marker implemented for all ten sub-mode ZSTs:
`DecodeRequest<P>` (wide-band scan), `SniperRequest<P>` (a known
`(start_sample, base_freq_hz)`), and `MultiPeriodRequest<P>` (averaged
multi-slot). `.ap_hint()`, `.ap_list()` and `.fading()` are plain
inherent methods rather than capability-gated traits, since every Q65
sub-mode supports every capability uniformly. The underlying
`q65::rx` functions are `pub(crate)`.

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

What the table makes visible:

- **FT8 / FT4 / FST4** are the cheap additions — LDPC + 77-bit message
  + block-Costas sync, so almost everything is shared.
- **WSPR** swaps all three of *FEC family*, *message width* and *sync
  mode* independently — the proof that those axes are orthogonal.
- **Q65** adds a third FEC family (non-binary QRA over GF(64)), ten
  sub-modes from one macro, and five parallel decoder strategies, all
  inside the same `Protocol` super-trait.
- **uvpacket** is a non-WSJT applied example: it reuses only the FEC
  mother code and bypasses the generic TX/RX pipeline. Full account in
  [`UVPACKET.md`](UVPACKET.md).
- **MSK144** opts out of the trait surface entirely, yet still reuses
  the FEC and message layers.

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

### 3.2 Geometry

24 wired ZSTs: 20 WSJT-family protocols and sub-modes plus 4
`uvpacket` sub-modes. MSK144 is listed last for reference but is **not**
one of them — it implements no `Protocol` impl, so it appears in neither
the registry nor `tests/protocol_invariants.rs`.

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

### 3.3 Per-protocol notes

- **FST4** — LDPC(240, 101) + 24-bit CRC (`fec::ldpc240_101`); the
  BP/OSD code is the same across LDPC sizes, so the new material is
  just the parity-check/generator tables and code dimensions. The five
  wired sub-modes differ only in `NSPS` / `SYMBOL_DT` /
  `TONE_SPACING_HZ` — plus `TX_START_OFFSET_S` for FST4-15 alone
  (0.5 s rather than 1.0 s into the slot) — and are emitted by the
  `fst4_submode!` macro. FST4-900 / FST4-1800 remain unwired (no user
  demand). FST4W — the WSPR-style one-way 50-bit beacon variant,
  LDPC(240, 74) — is a separate message format and out of scope
  (issue #23).
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
  (×1…×16); all five decoder strategies share the one QRA codec.

### 3.4 Decoder strategies

Every protocol runs the same underlying flow; the *strategy* wrapped
around it varies. Most are a single pass. Only Q65 exposes several
parallel receiver chains for one FEC frame, and MSK144 replaces the
slot model with a burst scan.

| Protocol | Default strategy | Optional strategies |
|----------|------------------|---------------------|
| **FT8**  | single-pass BP + OSD | AP iaptype loop (1–12); SIC 1–3 rounds; `.sic_early()`; sniper |
| **FT4**  | single-pass BP + OSD | SIC 1–3 rounds; full-slot coherent sync (`sync2d`) |
| **FST4** | single-pass BP + OSD | full-slot two-stage coherent sync search |
| **WSPR** | single bespoke pass (quarter-symbol spectrogram scan) | — |
| **JT9**  | single bespoke pass | — |
| **JT65** | single bespoke pass | RS erasure decode; stochastic Chase decoder |
| **Q65**  | `(Δf,Δt,b90)` grid + Lorentzian fading BP (scan) | AP-hint, explicit fast-fading, AP-list, multi-period |
| **MSK144** | burst scan over the whole T/R period | — |

**A-priori decoding is a general option, not a sniper feature.** AP is
a rung on `process_candidate_basic`'s own ladder, reaching FT8, FT4 and
every FST4 sub-mode; `msg::pipeline_ap` is hypothesis generation with
no engine of its own. It used to be coupled to the sniper by accident
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
| Weak / ionoscatter signal spanning several T/R periods | Multi-period EMA averaging (3-stage cascade) | `MultiPeriodRequest::<P>::new(...).decode()` | recovers signals no single-period strategy can |

`.ap_list()` and `.fading()` are mutually exclusive in the underlying
engine; `.decode()` resolves precedence as
`ap_list > fading (+ ap_hint) > ap_hint > plain`.
`MultiPeriodRequest` takes `&[&[f32]]`, one buffer per T/R slot, and is
Rust-only — not in the C ABI. What each front end actually does, and
why the default scan is not the plain Bessel path, is in
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for).

---

## 4. Module and crate map

```text
mfsk_core
├── engine/           Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract · msk · analytic ·
│   │                   ddc · fir · dotprod · fixed-point FFT kernels
│   ├── fft.rs          FftPlanner trait + the extern factory (see EMBEDDED.md)
│   ├── scalar.rs       Q-format fixed-point scalar types
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── sync2d.rs       FT4 / FST4 full-slot coherent sync searches
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
│   ├── tx.rs           shared transmit-side helpers
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
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 family — 5 sub-mode ZSTs (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search + ddc
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ erasure-aware RS, chase)
├── q65/              Q65 family — 10 sub-mode ZSTs + decode + synth
├── msk144/           MSK144 — no Protocol impl; own top-level driver
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
    const LLR_SCALE: f32 = 2.83;
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) or Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
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
    const ID: ProtocolId;
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
| `gfsk` | GFSK tone-to-PCM synthesiser (`GfskCfg`) |
| `subtract` | phase-continuous least-squares SIC (`SubtractCfg`) |
| `ddc` | streaming digital down-converter (WSPR's embedded channelizer) |
| `fir` / `dotprod` | polyphase FIR and the dot-product kernel the extern hook replaces |

Each takes a runtime `*Cfg` struct rather than `<P>`, because the
tuning parameters include composite-FFT sizes not trivially derived
from trait constants. Protocol modules expose module-level constants —
`ft8::downsample::FT8_CFG`, `ft4::decode::FT4_DOWNSAMPLE`, etc.

### Sync (`mfsk_core::engine::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC-aligned 2D
  peak search over `P::SYNC_MODE.blocks()` for non-FT8 protocols.
* `refine_candidate::<P>(cd0, cand, search_steps)` — integer-sample
  scan + parabolic sub-sample interpolation.
* `make_costas_ref` / `score_costas_block` — raw correlation helpers
  exposed for diagnostics and custom pipelines.

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
* `compute_llr::<P>(cs)` — four WSJT-style LLR variants (a/b/c/d),
  built from `nsym ∈ {1, 2, P::LLR_NSYM_MAX}` correlation-ladder
  hypotheses. `LLR_NSYM_MAX` defaults to 3; FT4 overrides it to 4 and
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
| `osd_max_errors()` — post-OSD hard-error ceiling, `osd_depth`-tiered | **FT4 only in practice.** FST4 bypasses it (`is_fst4` in `engine/pipeline.rs`) and trusts CRC-24 alone, matching WSJT-X's own FST4 acceptance test (`fst4_decode.f90:570`). FT8 has never called it |
| `ap_max_errors(locked_bits)` — AP-assisted ceiling, graded by locked-bit count | FT8's per-candidate AP loop and FT4/FST4's AP path alike, numerically unified (issue #191) |
| `ft8_nharderrors_max()` — FT8's own flat (not tiered) ceiling for the non-AP BP staircase and OSD fallback | FT8 (`ft8::decode_block::process_candidates`/`osd_strategy`). `Normal` returns 36, WSJT-X's own `ft8b.f90:422` ceiling. `Strict = 22` reuses prior art from issue #72; `Deep = 40` is exploratory and not yet swept against a fading corpus |

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
| `jt9` / `jt65` / `q65` | off | **host-only** — they pull `fft-rustfft` and therefore `std`. Their FFTs go through `engine::fft` since #390, but the modules are not yet `no_std`-clean |
| `msk144` | off | MSK144 — no `Protocol` ZST; own top-level driver |
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

A new protocol gets one line there. MSK144 does not appear, because it
implements no `Protocol` impl — that is an architectural decision, not
a gap to close.

---

## License

See [`LICENSE`](../../LICENSE) at the repository root. Every algorithm
here derives from WSJT-X (Joe Taylor K1JT and collaborators), and each
source file cites the `lib/*.f90` / `lib/*.c` it ports.
