# The 0.14 public API: final proposal (2026-10-17)

Status: **approved 2026-10-17** (with section 7.1 out of scope, the C ABI of section 7.2 a follow-up, and the release later than the 2-week target for A2). Planned by Opus, implemented by Sonnet agents. Source: the pre-release API audit of 2026-10-17
(rustdoc JSON of `--features full` at `6c16ff36`: 1 015 public items, 1 296 paths), and the user's decisions in that review.

## 0. Who the API is for, and what is not changing

The crate serves two kinds of user, and its public surface is wide **on purpose**:

- **application builders**: receive (audio or IQ), decode, transmit, show messages;
- **protocol builders**: a new mode on the existing traits, codecs and DSP (CLAUDE.md §2.6, `CONTRIBUTING.md` "Adding a new protocol").

So nothing below hides or moves building blocks, and items reachable by more than one path stay where the implementation
put them (a protocol module re-exports the shared parts it is built from; a parent module flattens its children). There is no
semver gate (dropped, `15686981`): it would flag every change to a building block.

Decided by the user, all for 0.14:

| # | change | kind |
|---|---|---|
| L1 | tier map: which modules are the application layer, the protocol-building layer, the building blocks | docs |
| L2 | `mfsk_core::prelude` | additive |
| L3 | the three names that collide with application-layer names | breaking |
| L4 | the global instrumentation counters behind a feature | breaking |
| C1 | delete the four `ft8` migration façades (their four users move to the real paths, §5) | breaking |
| C2 | docs that name crates that no longer exist | docs |
| C3 | `ft8::llr::GenericLlrSet` no longer public | breaking |
| R1 | `Debug` on every public type; `Display` + `Error` on every error type | additive (in progress) |
| A1 | a transmit API at the level of `AnyDecoder` | additive |
| A2 | a whole-message 77-bit packer (WSJT-X `pack77(msg)`) | additive |
| A3 | `Decoder<P>` and `AnyDecoder` made symmetric | mostly breaking |

## 1. Tiers (L1)

| tier | for | modules |
|---|---|---|
| **application** | receive, decode, transmit | `prelude`, `decoder`, `encoder` (A1), `iq`, `slotgrid`, `registry`, `msg` (codecs, `Decoded`, `ApHint`, `CallsignHashTable`), the protocol ZSTs at the root; MSK144 and JTTY through their own modules (`msk144::decode::Stream`, `jtty::rx`, `jtty::pack`, `jtty::tx`) |
| **protocol building** | a new mode | the root traits (`ModulationParams`, `FrameLayout`, `Protocol`, `FecCodec`, `MessageCodec`, `SyncFrontEnd`), `decoder::Decodable`, `engine::tx` (`FskWaveform`, `message_to_tones`, `synthesize*`), `fec::*` |
| **building blocks** | replace a stage, embedded | `engine::{dsp, sync, sync2d, llr, equalize, spectrogram, search, …}`, protocol internals (`ft8::decode_block`, `*::search`, `*::ddc`, `*::rx`) |

Where: a "Tier" column in `LIBRARY.md` §4 (and `.ja.md`), the same table near the top of the crate doc (`lib.rs`), and one line
in CLAUDE.md §3, which today says "raw engine functions are `pub(crate)`"; only `engine::pipeline` is.

## 2. `prelude` (L2)

`use mfsk_core::prelude::*;` brings what an application needs and nothing that a glob import is likely to clash with:

```rust
pub use crate::decoder::{AnyDecoder, AnySlotResult, Decoder, DecodeParams, Depth, QsoContext, QsoProgress,
                         Row, RowDetail, SlotInput, SlotResult, Station, Unsupported};   // fft feature
pub use crate::encoder::{encode, EncodeError, EncodeOptions, Encoded, TxAudio};         // A1
pub use crate::iq::{ChannelId, Channelizer, CompletedSlot, IqError, IqReceiver, IqSampleFormat, IqStream};
pub use crate::msg::{ApHint, Decoded};
pub use crate::registry::{Mode, ProtocolMeta};
```

Each line carries the `cfg` of its module. Protocol ZSTs stay at the root (`mfsk_core::Ft8`), not in the prelude.

## 3. Name collisions (L3)

| today | 0.14 | why |
|---|---|---|
| `uvpacket::Mode` (`uvpacket::puncture::Mode`) | `uvpacket::UvMode` | collides with `registry::Mode`, the application's mode type |
| `msk144::decode::Depth` | removed; `msk144` takes `decoder::Depth` | the same three levels with the same meaning (`ndepth` 1/2/3, default Deep, `mskrtd.f90:142-144`); `msk144::decode` and `decoder` have the same feature gate (an FFT backend), so the dependency is free. The pattern count becomes a private `fn npat(Depth)` |
| `jtty::testsig::Station` | `jtty::testsig::Transmission` | collides with `decoder::Station`; its own doc says "One transmission" |

Consumers: `UvMode` 8 files (tests, `uvpacket`); `Depth` 6 files incl. `mfsk-ffi/src/lib.rs:3315` (`mfsk_msk144_open`); `Transmission`
1 file, `embedded-poc/m5stack-cores3-app/src/bin/jtty_demo.rs` (host CI does not build it: compile it with the board's toolchain).

## 4. Instrumentation behind a feature (L4)

`fec::conv::fano::instrument` and `wspr::instrument` (22 process-global `AtomicU32`s and their `reset`/`snapshot`) move behind a new
feature `instrument`. Without it the modules do not exist and the counting sites compile to nothing. Consumers that turn it on:
`embedded-poc/embedded-shared` (`apps/wspr_scan.rs`, `apps/wspr_bench.rs`), `mfsk-core/tests/wspr_wsjtx_samples.rs` and
`tests/wspr_sweep.rs` (`required-features`). `full` does not include it.

## 5. Clean-ups (C1-C3)

- **C1.** Delete `ft8::ldpc`, `ft8::message`, `ft8::resample`, `ft8::hash_table` (each says it is a migration façade). The audit said no
  file uses them; the implementing agent found four the audit's grep had dropped, `mfsk-core/tests/ft8_resample.rs:10` and
  `embedded-poc/m5stack-s3-app/src/{audio.rs:1147, tx.rs:33, tx_scheduler.rs:37}` (not built by host CI). They move to the real
  paths in the same change: the point of 0.14 is a cleaner API, and keeping a path because it is used would defeat it (user,
  2026-10-17). Migration line in `LIBRARY.md` §1.3: `ft8::message::X` → `msg::wsjt77::X`, `ft8::ldpc::X` → `fec::ldpc::X`,
  `ft8::resample::X` → `engine::dsp::resample::X`, `ft8::hash_table::X` → `msg::hash_table::X`.
- **C2.** 13 doc lines in 9 files name `ft8-core`, `mfsk-fec`, `mfsk-msg`, `mfsk-engine`; point them at the modules.
- **C3.** `ft8::llr::GenericLlrSet` (an alias of `engine::llr::LlrSet` beside the different `ft8::llr::LlrSet`) becomes a private `use`.

## 6. `Debug`, `Display`, `Error` (R1, in progress)

`Debug` on the 78 public types without it (derived where cheap; a hand-written impl that prints sizes instead of buffers where a
type holds sample or spectrum buffers, ending `finish_non_exhaustive()`); `Display` + `core::error::Error` on `fec::qra::Q65DecodeError`
and `uvpacket::{framing::PackError, framing::UnpackError, rx::DecodeError}`. A compile-time test asserts both for the application types.

## 7. Transmit (A1): `mfsk_core::encoder`

Today a Rust application transmits by a different route per family: `engine::tx::message_to_tones::<P>` + `engine::tx::synthesize::<P>`
for FT8/FT4/FST4, `wspr::synthesize_type1`, `jt9::synthesize_standard`, `jt65::synthesize_standard`, `q65::synthesize_standard_for`,
`fst4w::encode::message_to_tones`. The C ABI already has one `mfsk_encode_*` per mode; Rust has nothing at the level of `AnyDecoder`.

(0.15: the fields are private behind accessors, #700; `LIBRARY.md` §1.4.)

```rust
/// What `encode` produced: the channel symbols, and what a receiver will print.
#[non_exhaustive] #[derive(Clone, Debug)]
pub struct Encoded {
    pub mode: Mode,
    pub tones: Vec<u8>,
    /// The message as `unpack` renders the transmitted bits (WSJT-X shows this in its Tx box).
    pub text: String,
    /// Calls the message sends as hashes; give them to the decoder (`learn_callsign`) so their
    /// `<...>` resolves when the other station answers (WSJT-X's `record_tx_hashes`).
    pub hashed_calls: Vec<String>,
}

#[non_exhaustive] #[derive(Clone, Copy, Debug, Default)]
pub struct EncodeOptions { pub q65_copied_last_tx: bool }     // builder: .q65_copied_last_tx(bool)

#[non_exhaustive] #[derive(Clone, Copy, Debug)]
pub struct TxAudio { pub sample_rate: u32, pub f0_hz: f32, pub amplitude: f32 }
impl TxAudio { pub fn new(sample_rate: u32, f0_hz: f32) -> Self /* amplitude 1.0 */; pub fn amplitude(self, a: f32) -> Self }

#[non_exhaustive] #[derive(Clone, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The text is not a message this mode can send (77-bit modes: why, from `pack77_message`).
    Message { mode: Mode, reason: Pack77Error },
    /// A message form this mode has but this crate does not encode (section 7.1).
    Unsupported { mode: Mode, what: &'static str },
}

pub fn encode(mode: Mode, text: &str) -> Result<Encoded, EncodeError>;
pub fn encode_with(mode: Mode, text: &str, options: &EncodeOptions) -> Result<Encoded, EncodeError>;
impl Encoded {
    pub fn synthesize(&self, audio: &TxAudio) -> Vec<f32>;
    pub fn synthesize_i16(&self, audio: &TxAudio, amplitude_i16: i16) -> Vec<i16>;
    pub fn synth_len(&self, sample_rate: u32) -> usize;
}
```

Per mode, all through code that exists today except the packer of A2. Every `Mode` ZST implements
`engine::tx::FskWaveform`, so tones → audio is one generic call, `engine::tx::synthesize::<P>`, dispatched over `Mode` the way
`AnyDecoder` is (a macro over the same list):

| modes | text → bits | bits → tones |
|---|---|---|
| FT8, FT4, FST4-15‥1800 | `pack77_message` (A2) | `engine::tx::message_to_tones::<P>` |
| Q65 (10 sub-modes) | `pack77_message` | `q65::encode_channel_symbols_flagged(bits, options.q65_copied_last_tx)` |
| FST4W-120‥1800 | `pack77_message` with `prefer_wspr_50bit` (as `genfst4` asks; today `Fst4wMessage::pack_text` → `pack77_wspr(text, true)`) | `fst4w::encode::payload_to_tones::<P>` on the first 50 bits |
| WSPR | `msg::wspr::pack_type1` on `CALL GRID4 DBM` | `wspr::encode_channel_symbols` |
| JT9, JT65 | `msg::jt72::pack_standard` on `CALL1 CALL2 GRID/RPT` | `jt9`/`jt65::encode_channel_symbols` |

`encode` is stateless; nothing global records the hashes (the decoder owns its table, so the caller hands `hashed_calls` to it).
MSK144 and JTTY are outside `Mode` on the receive side and stay so here (`msk144::tx`, `jtty::pack` + `jtty::tx`).
`engine::tx` stays the protocol builder's generic route.

### 7.1 Not in 0.14 (says `EncodeError::Unsupported`)

*Moved into 0.14 on 2026-10-17: §15.2, §15.3 (as built: §14). The text below is the original plan.*

- **WSPR types 2 and 3** (compound calls, 6-character grids: two transmissions, `wqencode`): not ported.
- **JT9 / JT65 free text and compound calls**: `msg::jt72` packs and unpacks only the standard form today (`Jt72Message::Unsupported`
  on receive). A port of `packmsg` / `unpackmsg` (`lib/packjt.f90:401-661`, 260 lines) is its own job; asking is cheaper than guessing
  how much these legacy modes are still used.

### 7.2 C ABI

*Moved into 0.14 on 2026-10-17: §15.1 (as built: §14).* Not part of this change. `mfsk_encode_*` keep their per-mode signatures. A `mfsk_encode(mode, text, …)` over `encoder::encode` would
bring the bindings level with Rust; it is a follow-up for the FFI (Kotlin/Swift wrappers too), not for this release.

## 8. Whole-message packer (A2): `msg::wsjt77::pack77_message`

A port of WSJT-X v3.2.0-rc1 `pack77_encode_core` (`lib/77bit/packjt77.f90:527-608`) with its result codes (`:66-71`):

```rust
#[non_exhaustive] #[derive(Clone, Copy, Debug, Default)]
pub struct Pack77Options { pub prefer_wspr_50bit: bool }

#[non_exhaustive] #[derive(Clone, Debug)]
pub struct Packed77 { pub bits: [u8; 77], pub i3: u8, pub n3: u8, pub hashed_calls: Vec<String> }

#[non_exhaustive] #[derive(Clone, Copy, Debug, PartialEq, Eq)]   // + Display, Error
pub enum Pack77Error {
    NotEncoded,               // PACK77_STATUS_NOT_ENCODED
    FreeTextTooLong,          // PACK77_STATUS_FREE_TEXT_TOO_LONG
    FreeTextInvalid,          // PACK77_STATUS_FREE_TEXT_INVALID
    PreferredFamilyRejected,  // PACK77_STATUS_PREFERRED_FAMILY_REJECTED
    RoundtripRejected,        // PACK77_STATUS_INTERNAL_ROUNDTRIP_REJECTED
}

pub fn pack77_message(text: &str, options: &Pack77Options) -> Result<Packed77, Pack77Error>;
```

Order as upstream: upper-case and split; over 19 words → telemetry (0.5) else free text; unless the message starts `CQ `/`DE `/`QRZ `:
0.1 DXpedition, 0.3/0.4 ARRL Field Day, 0.5 telemetry (one word); then 0.6 (WSPR form), 1/2 standard (+ `/P`), 3 ARRL RTTY Roundup,
4 one non-standard call, 5 EU VHF; else exact free text. Every candidate is accepted only if its bits unpack to the same text
(`take_candidate`). Today's `pack77`, `pack77_type1`, `pack77_type4`, `pack77_free_text`, `pack77_wspr` stay (they are the
candidates 1, 4, 0.0, 0.6 and their callers keep working); new: candidates 0.1, 0.3/0.4, 0.5, 3, 5. Unpacking of all of them exists.
Not ported: the DX macro expansion (`expand_dx_macro`, a GUI convenience).

Tests (tier A): every candidate round-trips through `unpack77`; a table of messages packed by upstream gives the same 77 bits and
`i3.n3`; malformed messages give the upstream status. The oracle is rc1's own `lib/77bit/encode77.f90` built against rc1's
`packjt77.f90`, `packjt77_grammar.f90` and `packjt77_schema.f90` (the way `target/fst4w-oracle` builds `genfst4`), run over rc1's
`lib/77bit/messages.txt` (100 lines) plus cases of our own. The checked-in `encode77.out` (41 lines, last changed `2da8f51fc`) predates
the 3.x packer and is not the reference.

Size: the candidates call into `packjt77_grammar.f90` (1 530 lines) and `packjt77_schema.f90` (871); `msg/wsjt77.rs` already ports the
parts the 0.6 candidate and the round-trip gate need (`:1785-2115`). What is new is the 0.1, 0.3/0.4, 0.5, 3 and 5 encoders and the
dispatch, several hundred lines of Rust; the largest item of this release.

## 9. `Decoder<P>` and `AnyDecoder` symmetric (A3)

| today | 0.14 | kind |
|---|---|---|
| `AnyDecoder::decode_i16(&[i16], Option<i64>)` only | also `Decoder<P>::decode_i16` | additive |
| `AnyDecoder::extras_mut()` only | also `AnyDecoder::extras(&self) -> AnyExtrasRef<'_>` (read-only twin of `AnyExtras`) | additive |
| `AnyDecoder::set_ap_hint` only | also `Decoder<P>::set_ap_hint` for the modes whose extras carry `ap_hint` (FT8, FT4, FST4, Q65), through a small trait on those extras | additive |
| `AnySlotResult { rows: Vec<Decoded>, details: Vec<RowDetail> }` (two parallel vectors) | `pub type AnySlotResult = SlotResult<()>`: `rows: Vec<Row<()>>`, the same row type as `Decoder<P>` with the native result erased | breaking: `mfsk-ffi/src/decoder.rs` (1 site), `apps/skimmer/core/src/lib.rs` (1), tests (≈11) |
| `AnyDecoder::decode_with(on_row: &dyn Fn(&Decoded, &RowDetail))` | `on_row: OnRow<'_, ()>`, as `Decoder<P>` | breaking: `mfsk-ffi/src/decoder.rs:1197-1199`, `apps/skimmer/core/src/lib.rs:439-441` |
| `AnyDecoder::wcalls() -> Result<Vec<String>, Unsupported>` (a copy) | `-> Result<&[String], Unsupported>` | breaking, 1 site (`mfsk-ffi`) |
| `AnyDecoder::set_wcalls` maps `TooManyCalls` to `Unsupported` (a list that is too long is reported as "the mode has no list") | `-> Result<(), WcallsError>`, `enum WcallsError { Unsupported(Unsupported), TooManyCalls(TooManyCalls) }` | breaking, 1 site; also a wrong message today |

## 10. Re-audit against the crates that use the API (2026-10-17)

Every change above, against what `mfsk-ffi`, `bindings/{kotlin,swift}`, `apps/skimmer`, `embedded-poc`, `hosttest`, `bench/wasm` and
`wasm-smoke` call today (grep over their sources):

| change | where it lands | finding |
|---|---|---|
| A3 `AnySlotResult` → `SlotResult<()>` | `mfsk-ffi/src/decoder.rs:1203` zips `rows` with `details`; `apps/skimmer` reads `details` once; `wasm-smoke` reads only `.rows.len()` | three edits; `wasm-smoke` unchanged |
| A3 `decode_with` callback | `mfsk-ffi/src/decoder.rs:1197-1199`, `apps/skimmer/core/src/lib.rs:439-441` | the closures take `&Row<()>` instead of `(&Decoded, &RowDetail)`; one clone of `Decoded` per delivered row inside `AnyDecoder` (rows per slot are tens) |
| A3 `wcalls` / `set_wcalls` | `mfsk-ffi/src/decoder.rs` (1 each) | the C ABI's status for a too-long list becomes the right one (today it says "unsupported") |
| L3 `msk144` `Depth` | `mfsk-ffi/src/lib.rs:3315` (`mfsk_msk144_open` maps 1/2/3 to it) | the C value is unchanged; only the Rust import moves |
| L3 `testsig::Transmission` | `embedded-poc/m5stack-cores3-app/src/bin/jtty_demo.rs` | not built by host CI: build it with the board's toolchain |
| L4 `instrument` | `embedded-shared` `apps/wspr_scan.rs` returns `instrument::Counts` and `fano::instrument::Snapshot` in its result; the module is gated by its `wspr-bench` feature | add `"mfsk-core/instrument"` to `wspr-bench` (`embedded-shared/Cargo.toml:46`); the CoreS3 WSPR receiver keeps its counters |
| C1 façades | `mfsk-core/tests/ft8_resample.rs`, `embedded-poc/m5stack-s3-app` (3 imports of `ft8::message::pack77`) | the first version of this table said "no consumer"; wrong (see §5). The imports move to `engine::dsp::resample` / `msg::wsjt77` |
| A1 `encoder`, A2 `pack77_message`, R1, L2 | additive | Kotlin / Swift / C unchanged (the C ABI is not touched, §7.2) |

Nothing found that needs the plan changed, apart from the C1 consumers above. Two things the plan relies on and that were checked rather than assumed: all `Mode` ZSTs
implement `FskWaveform` (`ft8/mod.rs:115`, `ft4/mod.rs:112`, `fst4/mod.rs:193`, `fst4w/mod.rs:182`, `wspr/mod.rs:140`, `jt9/mod.rs:256`,
`jt65/mod.rs:514`, `q65/protocol.rs:80`), and `msk144::decode` has the same feature gate as `decoder` (`msk144/mod.rs:18`, `lib.rs:370`).

## 11. Considered and not changed

- `Decoded::dt_sec` beside `*_s` elsewhere: 378 uses in 93 files (and the C/Kotlin/Swift field names); renaming costs more than it gives.
- `DecodeParams::for_band((lo, hi))` (a tuple) beside `.band(lo, hi)`: 33 uses; both read clearly.
- Builder names (`with_*` on `ApHint`, `IqReceiver::with_clock`; bare on `DecodeParams`, `SlotInput`, `IqStream`): the rule in use is
  "bare name on a plain parameter struct, `with_` on a type with behaviour"; written down in `LIBRARY.md` instead of renamed.
- `IqReceiver::{remove_channel, tap_audio, take_audio, set_prefix_points}` returning `bool` for an unknown id: 27 uses in 7 files;
  an unknown id is a caller bug, and `false` is documented.
- `Option` from the existing packers (`pack77`, `pack28`, `msg::wspr::pack_*`, `msg::jt72::pack_*`): the new `pack77_message` returns why;
  the old ones stay as they are.
- Same-name items in different building-block modules (`Spectrogram` ×7, `score_candidate` ×5, …): the module says which.

## 12. Sample apps: a follow-up (user, 2026-10-17)

The sample apps under `embedded-poc/` are updated after the library change, not with it (they are outside the workspace and host CI).
Until then they do not build against `main`. Known places: `m5stack-s3-app/src/{audio.rs:1147, tx.rs:33, tx_scheduler.rs:37}`
(`ft8::message::pack77` → `msg::wsjt77::pack77`, C1), `m5stack-cores3-app/src/bin/jtty_demo.rs` (`testsig::Station` → `Transmission`, L3),
`embedded-shared/Cargo.toml:46` (`wspr-bench` needs `mfsk-core/instrument`, L4); the implementing agent's report completes the list.

## 13. Order of work

1. R1 lands (in progress).
2. C1-C3, L3, L4 (mechanical).
3. A3 (decoder module; FFI and skimmer call sites in the same commit).
4. A2 (the port; tests against upstream vectors).
5. A1 (on A2), then L2 (`prelude` names `encoder`), then L1 and the migration section of `LIBRARY.md` §1.3 (+ `.ja.md`), CHANGELOG.
6. Merge gate (tiers A+B), `scripts/pre-push-check.sh`, `cargo build -p mfsk-ffi`, `cargo test -p skimmer-core`, the CoreS3 build of
   `jtty_demo` (L3) and of `embedded-shared` with `instrument` (L4).

## 14. As implemented (2026-10-17)

| item | commit | note |
|---|---|---|
| R1 `Debug`, `Display`, `Error` | `d8333a60` | §6 |
| C1-C3, L3, L4 | `6cde54ed` | §3-§5; the CI matrix row for embedded WSPR turns `instrument` on; the `embedded-poc` sample apps follow (§12) |
| A2 `pack77_message` | `93f26561` | §8; oracle: rc1's own packer over `messages.txt` and 1 203 cases of ours, fixture `tests/fixtures/pack77_rc1.tsv` (2 163 rows), plus 35 000 random rows once outside the repo |
| `unpack77` fidelity | `51215cf5` | not in the design: `unpack28` follows `unpack28_core` (`callok`, `<...>` for 532444 and up, 18 telemetry hex digits); found while porting A2, whose round-trip gate needed local patches for the first two |
| A1 `encoder` | `56ccfa41` | §7 |
| A3 symmetric decoders | `ac350478` | §9 |
| L1 tier map, L2 `prelude`, migration docs, CHANGELOG | this change | `LIBRARY.md` §1.3, §4 (+ `.ja.md`), crate doc, `src/prelude.rs` |
| upstream baseline | `9e345837` | v3.3.0-beta2, nothing to port (#690) |
| 15.1 C ABI `mfsk_encode`, `mfsk_encode_info` (+ Kotlin, Swift) | `9970c8b9` | §15.1; `mfsk-ffi` tests 6 + C++ smoke driver; Kotlin and Swift written but not built on the author's host (no `jni.h`, `kotlinc`, Xcode): their CI jobs are the first check |
| 15.2 WSPR types 2/3 | `2fc95c22` | §15.2; `msg::wspr::pack_message`, oracle: rc1's own `wqencode` + `wqdecode` (gfortran, `scripts/wspr/`), fixture `tests/fixtures/wspr_pack_rc1.tsv` (2 226 rows), and all 32 400 four-character grids offline |
| 15.3 JT9 / JT65 full messages | `5a4b9ee8` | §15.3; `msg::jt72::pack_message` / `unpack`, oracle: rc1's own `packmsg` / `unpackmsg` (`scripts/jt72/`), fixture `tests/fixtures/jt72_rc1.tsv` (7 651 rows); JT65 / JT9 goldens unchanged (4/5, 7/7, extra 0) |
| WSPR unpack = `wsprd` | `98fe9331` | not in the design; found while checking 15.2's round trip, see below |

**A1 deviations from §7.**

- (a) FST4W refuses a text that does not pack as `i3.n3 = 0.6` (`Pack77Error::PreferredFamilyRejected`), as `Fst4wMessage::pack_text`
  does, instead of sending the first 50 bits of whatever packed. `prefer_wspr_50bit` alone falls back to other types (free text
  `K1ABC FN42 38`) whose first 50 bits are not a message. `tests/encoder.rs` compares 15 texts with `pack_text`: same tones, same refusals.
- (b) `Encoded::text` renders hashed calls from a table seeded with `hashed_calls`, as WSJT-X's Tx box does, not as `<...>`.
- (c) `hashed_calls` holds what rc1 records (`commit_hash_facts`): `<W9XYZ> PJ4/K1ABC RR73` gives `W9XYZ` only. The A2 oracle checks this.

**A3 additions to §9.** `TooManyCalls` gained `Display` and `Error`; `WcallsError::TooManyCalls` is gated on `fst4w` (without it the
variant does not exist); the C ABI maps it to `MFSK_STATUS_INVALID_ARG` (the header is unchanged). `HasApHint` is the small trait
behind `Decoder::set_ap_hint`.

**L2 as built.** `prelude` is §2's list with these differences: the decoder line is split by gate (`Decoder`, `DecodeParams`, `Depth`,
`QsoContext`, `QsoProgress`, `Row`, `RowDetail`, `SlotInput`, `SlotResult`, `Station` need an FFT backend; `AnyDecoder`, `AnySlotResult`,
`Unsupported` also need one protocol feature); the IQ line is split the same way (`IqError`, `IqSampleFormat`, `IqStream` always;
`ChannelId`, `Channelizer`, `CompletedSlot`, `IqReceiver` with the receiver's gate). `WcallsError`, `HasApHint`, `AnyExtras` and
`AnyExtrasRef` are left out: the first is named only by code that matches on `set_wcalls`'s error, the others are a bound and types that are
used without being named. No name clashes inside the prelude (it compiles as a glob import beside `mfsk_core::Ft8` in its doctest).

**15.1 deviations from §15.1.**

- (a) `mfsk_decoder_add_callsign` already exists, so no `mfsk_decoder_learn_callsign` was added: `hashed_calls` goes to it.
- (b) `sample_rate == 0` or a non-finite `freq_hz` is `MFSK_STATUS_INVALID_ARG` (`TxAudio` cannot synthesise at them), and the
  synthesis runs under `catch_unwind`, a panic at an unsupported rate being `InvalidArg` rather than an unwind across the C boundary.
- (c) The `*_len` parameters of the two strings were added after review (the first signature had none), as §15.1 records; the
  header documents "needed bytes including the NUL, set even when too small, nothing written unless all three buffers fit".
- (d) The mode statuses are `mfsk_decoder_open`'s: `InvalidArg` for a number that is no `MfskMode`, `UnknownProtocol` for MSK144,
  JTTY, uvpacket and a mode absent from the build. Header additive: two functions and `MFSK_ENCODE_Q65_COPIED_LAST_TX`; ABI stays 3.

**15.2 deviations from §15.2.**

- (a) A failure is `EncodeError::Wspr { mode, reason: WsprPackError }`; `EncodeError::Message` is left for WSPR only when the packed
  bits do not unpack (`RoundtripRejected`: a `CQ` call or a report in the grid slot, which `wqencode` packs and no receiver reads).
- (b) The encoder normalises the text (upper case, blank runs collapsed) before `pack_message`; `pack_message` itself takes it as typed.
- (c) `WsprPacked` carries `hashed_call` (type 3: the call between `<` and `>`, as typed), since the encoder needs it for
  `Encoded::hashed_calls` and it is not one of `wqencode`'s outputs.
- (d) Rows where upstream is undefined or aborts are refused, not reproduced: `packpfx` / `packcall` flags ignored in the type 2 and 3
  branches (`BadCall`, `BadGrid`), a non-integer power (list-directed read aborts, `BadPower`), a call that overruns the 12-character
  buffers (`CallTooLong`), an unclosed `<` (`UnclosedHashedCall`). The power is clamped to 0..60 and rounded with the `nu` table, as upstream.

**15.3 deviations from §15.3.**

- (a) `Jt72Message` is `Standard`, `Compound(String)` and `FreeText(String)`; `Unsupported` is removed (upstream renders every bit
  pattern), `Display` is upstream's text. `pack_message` returns `Jt72Packed { words, itype }`; its only refusal is `Jt72PackError::NonAscii`,
  reported by the encoder as `EncodeError::Jt72` (`Unsupported` is produced by no mode now).
- (b) An empty text packs as blank free text, as `packmsg` does, not as an error. A text past 22 characters is cut to 22 (the `character*22`
  its callers pass).
- (c) Decode side: JT9 no longer drops non-`Standard` messages (`jt9fano.f90:87-88` filters only `000AAA`); JT65 blanks `000AAA` and `0L6MWK`
  (`decode65b.f90:27-28`). **Open:** the JT65 `hint65` bank still matches standard messages only, as the ported `hint65.f90` does.

**WSPR unpack follows `wsprd`, not `wqdecode` (`98fe9331`).** WSJT-X's WSPR receiver is `wsprd`, so the reference for the text is
`lib/wsprd/wsprd_utils.c` `unpk_` (324-405; identical in rc1 and v3.3.0-beta2), checked row by row against rc1's own `unpk_` built from
`git archive` (`scripts/wsprd/`, fixture `tests/fixtures/wsprd_unpack_rc1.tsv`, 26 046 rows). `unpackcall` is exact, `unpackpfx` keeps the part
after the last blank, `unpackgrid` uses double arithmetic; a Type 3 call is `<...>` unresolved and `<CALL>` when the decoder's
`WsprCallsignTable` holds it (`WsprResult::hashed_call`, `WsprMessage::render`). Kept because `wsprd` does the same: `3DA0` as `3D0`, CQ/QRZ/DE
calls and report grids not decoded. **Deliberate divergence:** `ntype == 63` makes `unpk_` print an empty message; `unpack` returns `None`.
WSPR golden (`150426_0918.wav`): 9/9, 0 extra, texts and freq/dt/snr identical before and after.

## 15. Added to 0.14: the C ABI encoder, WSPR types 2/3, JT9/JT65 full messages (user, 2026-10-17)

§7.1 and §7.2 were out of scope; the user moved them into 0.14. Planned by Opus, implemented by Sonnet agents, as before.

### 15.1 C ABI: one encoder call over the mode (was §7.2)

Additive; the per-mode `mfsk_encode_*` stay (ABI 3 keeps every symbol).

```c
/* flags: MFSK_ENCODE_Q65_COPIED_LAST_TX (bit 0); others reserved, must be 0. */
MfskStatus mfsk_encode(uint32_t mode, const char *text, uint32_t flags,
                       uint32_t sample_rate, float freq_hz,
                       float *out, size_t cap, size_t *out_len);           /* f32 PCM, same buffer contract as mfsk_encode_ft8 */
MfskStatus mfsk_encode_info(uint32_t mode, const char *text, uint32_t flags,
                            uint8_t *tones, size_t tones_cap, size_t *tones_len,
                            char *text_out, size_t text_cap, size_t *text_len,       /* what a receiver prints */
                            char *hashed_out, size_t hashed_cap, size_t *hashed_len); /* hashed calls, space-separated */
```

Both strings follow `put_text`'s contract (`mfsk_decoder_unpack77`): NUL-terminated, `*_len` = bytes needed including the NUL,
set even when the buffer is too small (corrected 2026-10-17: the first version of this signature had no `*_len`). `mode` is the C ABI's `MfskMode` number (as `mfsk_decoder_open` takes it). `EncodeError::Message` → `MFSK_STATUS_INVALID_ARG` with
the `Pack77Error` (or the message family's error) in `mfsk_last_error`; `EncodeError::Unsupported` → `MFSK_STATUS_UNSUPPORTED`. The
header is regenerated with cbindgen and committed. Kotlin (`Mfsk.kt` + `native/mfsk_jni.c`) and Swift (`Sources/MfskCore`) get one
`encode(mode, text, …)` each, in the style of their existing `encode` functions. **Not verifiable on this host** (no `jni.h`/`kotlinc`, no
Xcode): their CI jobs are the check. If the C ABI has no way to feed `hashed_calls` back to a decoder (`learn_callsign`), add
`mfsk_decoder_learn_callsign(dec, call)` beside `mfsk_decoder_unpack77`.

### 15.2 WSPR types 2 and 3 (was §7.1)

Port `wqencode` (`lib/wqencode.f90`, rc1) as `msg::wspr::pack_message(text) -> Result<WsprPacked, WsprPackError>` covering type 1
(`CALL GRID4 DBM`), type 2 (`PFX/CALL DBM`, `CALL/SFX DBM`) and type 3 (`<CALL> GRID6 DBM`), with upstream's power handling (clamp to
0..60 and round to the WSPR set with the `nu` table: **upstream rounds, it does not refuse** — the A1 route refuses a power outside the set
and must follow upstream). The encoder's WSPR arm uses it; `EncodeError::Unsupported` for WSPR goes away. Unpacking of all three types
exists (`msg::wspr::unpack`). A station with a compound call alternates type 2 and type 3 transmissions; that schedule is the
application's (WSJT-X's GUI does it), and `encoder` documents it. Test against rc1's own `wqencode` built with gfortran (as
`scripts/pack77`), vendored fixture.

### 15.3 JT9 / JT65 full messages (was §7.1)

Port `packmsg` / `unpackmsg` / `packtext` / `unpacktext` and the prefix/suffix handling (`lib/packjt.f90:401-709`, rc1) into `msg::jt72`:
`pack_message(text) -> Result<([u8; 12], itype), Jt72PackError>` and an unpack that renders every type as upstream does (standard,
compound calls, free text, …), replacing `Jt72Message::Unsupported` for the forms upstream renders. The JT9 and JT65 decoders then print
what `jt9` prints: compare against the vendored reference outputs (tier B golden tests); a recall or text change is reported with evidence,
not hidden. The encoder's JT arm uses `pack_message`; its `Unsupported` goes away. Test against rc1's own `packmsg`/`unpackmsg` built with
gfortran, vendored fixture.

### 15.4 Order

15.1 (main tree, `mfsk-ffi` + bindings), 15.2 and 15.3 (each in its own worktree; both touch `encoder.rs` in different arms, merged by the
planner) run in parallel. CHANGELOG, LIBRARY.md §1.3 / §2 and BINDINGS.md (+ `.ja.md`) follow the merge.
