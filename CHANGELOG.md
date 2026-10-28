# Changelog

## Unreleased

### Breaking changes

- **`engine::pipeline` is crate-private and no public path depends on `internal-testing` any more** (#693, closes #420). The types it
  produced moved to `engine::frame`: `DecodeResult`, `DecodeStrictness`, `DecodeDepth`, `LlrEffort`, `FftCache`, `BudgetCheck`,
  `BudgetReport` (the `ft8::decode` / `ft4::decode` / `fst4::decode` / `decoder` re-exports are unchanged). What the crate's tests and
  the embedded builds reached through `internal-testing` is now `mfsk_core::__internal::{pipeline, protocol, msg, fst4, wspr, jt9,
  jt65, q65, fec, sync2d, ft8}` (hidden from the docs, no semver promise): `engine::pipeline::process_candidate_basic` is
  `__internal::pipeline::process_candidate_basic`, `msg::decode_request` is `__internal::msg::decode_request`, and so on.
  `Ft4CoarsePhasors::buffer_addrs()` is `__internal::sync2d::buffer_addrs(&phasors)`. `mfsk-ffi` no longer enables the feature.
- **`#[non_exhaustive]`** on the option, result and error types that will grow: `jtty::rx::Params`, `jt65::{ChaseParams, HintSpec}`,
  `uvpacket` `AfcOpts` / `MultiChannelOpts` / `DecodedFrame` / `SlotEnergy` / `SyncStats` and its three error enums, `decoder::{Depth, ApMode,
  QsoProgress}`, `Jt9Depth`, `GridDepth`, `jtty::rx::{SideChannels, Subtracted}`, `jtty::pack::{ExchangeProfile, PackError}`, the msk144 and jtty result structs, `iq::RateTrackConfig`, `q65::{Caller, DxFromHistory}`. Build an
  option from `Default` and assign; add a wildcard arm to a `match`. `WsprMessage` and `Jt72Message` stay exhaustive (the variants are
  the message formats). Tier A+B unchanged. Migration: `LIBRARY.md` §1.4 (and `.ja.md`).
- **Registry capability bits describe the public API** (#698). Bit 0 is `caps::WSJT77_FAMILY` (the FT8 / FT4 / FST4 decoders: `ap_hint`,
  `eq`, `filter`, the QSO-context AP ladder); `DECODE_HANDLE` is a deprecated alias and never meant "a decoder opens", which is the new
  `caps::DECODER` (bit 18: every `Mode`; C `MFSK_CAP_DECODER`, what `mfsk_decoder_open` accepts). Bits 10-12 (`KNOWN_FILTER`,
  `KNOWN_SUBTRACT`, `FFT_CACHE`) described the 0.12 builders, which no public API has had since 0.13: removed from Rust, kept in C /
  Kotlin / Swift as deprecated names that no mode sets, numbers not reused. `tests/registry_caps.rs` checks bits 0 and 18 against
  `AnyDecoder`, `mfsk-ffi/tests/mode_introspection.rs` bit 18 against `mfsk_decoder_open`. C migration: `DECODE_HANDLE` → `WSJT77_FAMILY` or `DECODER`, drop tests of 10-12.
- **The registry carries no default search parameters** (#698). `registry::DecodeDefaults`, `DecodeProfile::defaults` and
  `DecodeProfile::sniper_max_cand_cap` are gone: nothing read them but `default_params`' fallback band, and FT8 / FT4 / FST4's values
  described no decoder (e.g. FT8's 100-3000 Hz and `sync_min` 0.8, FT4's sniper cap 15). `decoder::default_params(mode)` is the one source and
  states each mode's band (WSPR / JT9 / JT65 / Q65 from their `search::default_search_params()`, FST4W `jt9 -W`'s 1400-1600).
  `DecodeProfile` keeps `caps` and `sync_scale`. No mode's defaults moved: `tests/default_params.rs` pins all 26 at their 0.14 values.
  C ABI unchanged. Detail: `LIBRARY.md` §8.1.
- **`encoder::Encoded`'s fields are private** (#700): read `mode()`, `tones()`, `text()`, `hashed_calls()`. The tones are always the ones
  `encode` made for the mode, so `synthesize` cannot be handed a wrong count or tone; its doc claimed a panic on an unsupported sample
  rate, and none was found (one sub-mode per family, 0 Hz to 192 kHz: too low a rate gives degenerate audio, 0 Hz none). The GFSK ramp
  at a rate other than 12 kHz is computed in `u64` (FST4-1800 at 24 kHz overflowed a 32-bit `usize`). Tones from elsewhere
  go to `engine::tx::synthesize::<P>`, which checks them. `Decoded::protocol` stays the family on purpose: the exact sub-mode is the
  decoder's (`AnyDecoder::mode()`), documented. Nothing in the workspace wrote the fields; C / Kotlin / Swift unchanged. `LIBRARY.md` §1.4.

### Added

- **`Row::new(decoded, native)` and `SlotResult::new(rows)`** (#712): an application builds the rows and results it consumes, for a test of a
  display, a logger or a replay, without decoding audio (both are `#[non_exhaustive]`, so a struct expression is closed outside the crate).
  Default `RowDetail` / `BudgetReport`, `candidates` `None`; assign a field when it matters, as `CompletedSlot::new` (#592). `tests/decoder_result_new.rs`
  builds them from outside the crate. Additive.
- **`decoder::params_read(mode)`: which `DecodeParams` fields a mode's decoder can read** (#698), as `ParamFields` / `ParamField`; C
  `mfsk_mode_fields_read` + `MFSK_FIELD_*`, Kotlin `Mfsk.fieldsRead` / `FIELD_*`, Swift `Mode.paramsRead`. A field outside the set changes
  nothing for that mode, so a host can say which settings are effective; conditions per field are in the doc (`LIBRARY.md` §2.2). Found
  while building it: WSPR reads no band (wsprd takes none), JT65 no AP setting (#703), no mode `station.grid`.
  `tests/params_read.rs` sets every unread field to a value that would matter on a synthetic signal of each decoder and requires the
  same decode. Additive; no decode changed.
- **`RowDetail::method` and `SlotResult::candidates`: what an application can read about a decode** (#696). `method: Option<DecodeMethod>` says how a row was decoded (`Bp`, `Osd`, `Ap { iaptype }`,
  `ListA7`, `ListA8`; `None` for WSPR, JT9, JT65, Q65, FST4W) where `pass` is a number that means something else per mode and that FT4 / FST4 give twice (5 is both OSD depth 3 and an a-priori decode);
  `pass` stays, its table is in `docs/notes/PASS_IDS.md`. `candidates` is what the first coarse search of the call kept (FT8, FT4, FST4; per call, so FT8's `SicEarly` reports checkpoint A's in a whole-period call and C's in a prefix sequence's last; `None` for a call that made none, as FT8's checkpoint B); the engine's `DecodeOutcome` gains the same field. **#697: WSPR, JT9, JT65 and Q65 report it too** (every mode but FST4W; per-mode definitions in `docs/notes/CANDIDATES.md`: WSPR's is pass 0's list and is not narrowed to the band as `wsprd`'s `npk` is, JT9's the truncated list, JT65's pass 1's, Q65's the first scan's). `method`'s `iaptype` is WSJT-X's number only for the QSO context's hypotheses: rows of a free-form `ap_hint` are `Ap { iaptype: None }`.
  C ABI (appended, ABI 3): `MfskDecode::{method, iaptype}` (`MFSK_METHOD_*`), `MfskBudgetReport::candidates`; Kotlin `MfskMethod`, `iaptype`, `candidates`; Swift `DecodeMethod`. Additive.
- **`decoder::PeriodBudget`: one compute allowance for the calls of a period** (#626, #627): `share(period, mode, fraction)`, `left()`, `finish(took, &budget_report)` (true once per
  period, on the first call that ran out) and, with `std`, `within(start)`. Moved from the skimmer's `OpenPeriod`; the accounting holds no clock.
- **A channel takes its mode and prefix points from its decoder** (#692): `iq::ChannelSpec` (`From<Mode>`, `From<&AnyDecoder>`, `From<&Decoder<P>>`),
  `IqReceiver::add_channel(dial, impl Into<ChannelSpec>)` (a bare `Mode` value still compiles; an inferred `m.into()` argument now needs its type) and `update_channel` (applies from the next slot; another mode is
  `IqError::ModeChange`). `Decoder::decode_slot` / `decode_slot_with` and the `AnyDecoder` twins take the `CompletedSlot` and pick `decode_prefix` or `decode`
  themselves; a slot of another mode is `Err(ModeMismatch)`. Measured: points on the channel for a decoder that wants none return the same rows
  (`qso3_busy`, 14 / 14, SNR within 0.1 dB; `docs/notes/IQ_PREFIX_DESIGN.md` §10). Skimmer: no second `mode`, no throwaway decoder to ask for points, and the points
  follow the options only once the decoder has them. Additive.

### Documentation

- **Changing only the band keeps the mode's defaults** (#711, part 1): `DecodeParams::for_band`'s rustdoc called its block "the GUI's defaults",
  but it is the bare block (AP `Full`; FT8's and JT65's GUI start with AP off). The protocol modules' rustdoc examples, the prelude's and
  `STREAMING.md`'s (and `.ja.md`) now build from `default_params(mode).band(lo, hi)` or `with_defaults()` (WSPR, which reads no band);
  `tests/default_params.rs` checks that `band()` touches nothing else and that `for_band` is not a mode's defaults. No decoder changed.
- **The quick starts take one path** (#708): README and `LIBRARY.md` §1.1 encode with `encode` + `TxAudio` as rustdoc and §1 already did (README's
  hand-written pack77 → tones → waveform became a pointer to §5), all decode from the mode's defaults (`with_defaults()`, not `for_band`'s AP `Full`),
  and rustdoc / §1 say when to hold an `AnyDecoder` (mode picked at run time, README's path) instead of `Decoder::<Ft8>`.
  The rustdoc `Cargo.toml` snippets said `0.12`; `tests/doc_dependency_versions.rs` reads the `mfsk-core = "X"` / `{ version = "X" … }` lines of
  rustdoc, README and `docs/reference/*.md` against the crate's `major.minor`. §2.4's `AnyDecoder::decode_with` callback is `Row<()>`; §1.4 says the reference describes `main`.

### Deprecated

- **`IqReceiver::set_prefix_points`** (since 0.15, still works): `update_channel` with a `ChannelSpec` from the decoder replaces it (#692). No removal date is set.

### Results

- `RR73` is packed as the acknowledgment `MAXGRID4+3` (32403), not the grid 32373, for every non-CQ type 1/2 message in all 77-bit
  modes (WSJT-X v3.3.0-beta3 `58c7a522f`, #694 A1). The AP lock on `RR73` follows (`2c1b22b6c`, A2): in FT8, FT4 and FST4 it now
  matches beta3+ senders, not <= 3.2 senders. Q65 (fix): `genq65` has always sent 32403 while rc1's `q65_ap.f90` locked the
  grid form, which no Q65 sender transmits; the lock now matches every Q65 sender. Unpack is unchanged (both values read `RR73`);
  `q65_rr73_fixup` stays (`genq65.f90:43`). Tier A+B: no decode or golden change.
- **FT8 a8 and Q65's full-AP list give a standard call with a `/R` or `/P` suffix the full candidate set** (#694, WSJT-X v3.3.0-beta4
  `015b48945`, `6b6124f9d`, `b2204822b`): `stdcall_suffix` keeps such a call standard, and 207 `CQ call`, 208-211 `<MyCall> call` +
  nothing/RRR/RR73/73 and 212-411 `MyCall <call>` + the report ladder (FT8 cuts it at +-30 dB, Q65 does not) are added, 411 in all
  with a standard MyCall. a7 is unchanged, as upstream. Other calls still get 206. **`q65::MAX_AP_CODEWORDS` 206 -> 411**; the list
  grows only for such DX calls.

### Skimmer

- An rtl_tcp connection says what it waits for: `Event::Preparing { stage, up_to_s }` at the start of "waiting for the server to
  apply the settings" (up to 20 s) and "measuring the sample rate" (35 s), shown in the GUI's server line and printed by the CLI;
  the ~56 s between Connected and Decoding were silent.

## 0.14.0 — the API audited and rounded out: one transmit call over every mode (`encoder`, and `mfsk_encode` in the C ABI), WSJT-X's whole-message 77-bit packer, WSPR types 2/3 and JT9/JT65 free text and compound calls sent and received; every packer and unpacker bit-exact with WSJT-X 3.2.0-rc1 (`wsprd` for WSPR text); `Decoder<P>` and `AnyDecoder` symmetric, a `prelude` and a tier map, `Debug`/`Error` on every type (breaking); the skimmer's rtl_tcp client calibrates through pauses

Entries are grouped by what a reader has to do about them:
the API first, then what changes in results, then everything else. Measurements are kept to the one or two numbers that
justify a change; the rest is on the issue or in `docs/notes/`.

### Breaking changes at a glance

- **`ModulationParams`**: `NFFT_PER_SYMBOL_FACTOR`, `NSTEP_PER_SYMBOL` and `NDOWN` move to the new `SyncFrontEnd`; `SYMBOL_DT`,
  `N_SYMBOLS` and `LdpcParams::M` are defaulted; `TONE_SPACING_HZ` is written `tone_spacing_hz(GFSK_HMOD, NSPS)` (#419).
- **Removed**: `osd_decode`, `osd_decode_deep(4)`, `osd_decode_npre1` (`_masked`, `_npre2`) and `osd_decode_npre_generic`, for
  `osd174_91` / `osd_npre` (#417); `jt9::baseband` (#425). `is_plausible_callsign` stays and now behaves as `is_valid_callsign` (#612).
- **`registry::Mode` discriminants moved**: the FST4 and FST4W modes were inserted before WSPR, so `Mode::Wspr as u8` is 13, not 7, and
  `Q65A300` 25, not 19. Nothing in this repository casts a `Mode` to an integer; the C ABI has its own
  `MfskMode` numbers (`MFSK_MODE_WSPR = 7` is unchanged). Name a mode with its variant, `Mode::ALL` or `meta().name`, not a cast.
- **Types**: 41 public types are `#[non_exhaustive]`: 40 in #573 and `jtty::MessageUpdate` in #651; `RowDetail::{sync_score, sync_cv, hard_errors}` are `Option`s (#594).
- **C ABI**: appended fields only, ABI stays 3, but behaviour moves: rows are `out[0].size` apart and keep that `size` (#607, #635), a
  retry after a short buffer is answered without decoding again (#633), `float` at other rates keeps its level (#634), and
  `mfsk_iq_push` decodes FT8 early by default (#601). `MFSK_CAP_BUDGET` is published for four more modes (#593).
- **JTTY times are `f64`**: `FrameDecode::tsync_s`, `MessageUpdate::start_s`, `assemble::FRAME_PERIOD_S`, `Assembler::{prune, continuations}` (#645).
- **Kotlin / Swift**: `syncScore`, `syncCv` / `syncCV` and `hardErrors` are nullable (#594).
- **API clean-up (0.14)**: `ft8::{message, ldpc, resample, hash_table}` are gone (`msg::wsjt77`, `fec::ldpc`, `engine::dsp::resample`,
  `msg::hash_table`); `ft8::llr::GenericLlrSet` is removed; `uvpacket::Mode` is `UvMode`; `msk144::decode::Depth` is `decoder::Depth`;
  `jtty::testsig::Station` is `Transmission`; `wspr::instrument` and `fec::conv::fano::instrument` need the new `instrument` feature.
- **`AnyDecoder` follows `Decoder<P>`**: `AnySlotResult` is `SlotResult<()>` (`rows[i].decoded` / `.detail`, no `details`);
  `decode_with` takes `OnRow<'_, ()>`; `wcalls()` borrows; `set_wcalls` returns `WcallsError`.
- **Fewer messages decode (unpack77)**: a call WSJT-X's `callok` refuses (`AB1`, `FN4`, a leading `Q`, a blank inside) is no longer a decode;
  telemetry prints 18 hex digits; an out-of-range `CQ` token prints `<...>`, not `<?>`.
- **JT9 / JT65 and encoder types**: `Jt72Message::Unsupported` is removed, `Jt72Message::{Compound, FreeText}` are new (a `match` on it changes);
  `EncodeError` gained `Wspr` and `Jt72` (it is `#[non_exhaustive]`, so a `_` arm already covers them); a WSPR power outside the set is rounded, not refused.
- **Migration**: `LIBRARY.md` §1.3 (Rust: what changes in results, in code, what was removed, the old → new paths) and `BINDINGS.md` §3.1 (C, Kotlin,
  Swift), each with its `.ja.md`.

### Public API

- **`mfsk_core::encoder`: one transmit call over `Mode`, as `AnyDecoder` is for receive (A1).** A Rust application sent by a different route
  per family (`message_to_tones` + `synthesize`, `wspr::synthesize_type1`, `jt9`/`jt65::synthesize_standard`, `q65::synthesize_standard_for`,
  `fst4w::encode`); the C ABI had `mfsk_encode_*` per mode and Rust had nothing at that level. `encode(mode, text)` /
  `encode_with` give `Encoded { tones, text, hashed_calls }` (`text` as a receiver prints it, `hashed_calls` for `learn_callsign`), then
  `synthesize` / `synthesize_i16` / `synth_len`; `TxAudio`, `EncodeOptions` (Q65 copied-last-Tx), `EncodeError`. Every `Mode`; no FFT
  backend needed. WSPR types 2/3 and JT9/JT65 free text and compound calls are sent too (below); no mode produces `EncodeError::Unsupported` now.
  Tier A: every mode round-trips through `AnyDecoder`; tones and waveforms equal the per-mode routes; FST4W gives `Fst4wMessage::pack_text`'s
  tones and refusals on 15 texts. `engine::tx` stays the protocol builder's route. Design: `docs/notes/API_0_14_DESIGN.md` §7, §14.
- **C ABI `mfsk_encode` / `mfsk_encode_info`: one encoder call over the mode, with Kotlin `Mfsk.encode` / `encodeInfo` and Swift
  `Mode.encode` / `encodeInfo`.** The C ABI had an `mfsk_encode_*` per mode; now `mfsk_encode(mode, text, flags, sample_rate, freq_hz, out, cap,
  &n)` gives f32 PCM (`mfsk_encode_ft8`'s buffer contract) and `mfsk_encode_info` the tones, the text a receiver prints and the hashed calls
  (to `mfsk_decoder_add_callsign`), strings as `mfsk_decoder_unpack77`'s. Flag `MFSK_ENCODE_Q65_COPIED_LAST_TX`. Additive: two functions and a
  constant in `mfsk.h`, ABI stays 3. `mfsk-ffi` tests 6 (FT8 PCM equals `mfsk_encode_ft8`; info equals `encoder::encode` for FT8, FT4, FST4,
  Q65, WSPR) and the C++ smoke driver pass; Kotlin and Swift are not built on the author's host (no JNI headers, `kotlinc` or Xcode), checked by
  their CI jobs. `BINDINGS.md` §2.6, §3.1, §4, §5; design §15.1.
- **`msg::wspr::pack_message` (types 1, 2, 3) and `msg::jt72::pack_message`: the encoder sends WSPR types 2/3 and JT9/JT65 free text and compound
  calls.** Ports of `wqencode` (with `packcall`, `packgrid`, `packpfx`, `pack50`, `hash`) and `packmsg` / `packtext`; `Jt72Message` gains `Compound`
  and `FreeText` (below). WSPR power is clamped to 0..60 and rounded to the WSPR set as upstream does (38 dBm goes out as 37), not refused; the
  alternation of type 2 and type 3 is the application's schedule. Against rc1's own routines built with gfortran: WSPR 2 226 rows plus all 32 400
  four-character grids offline, JT 7 651 rows (`tests/fixtures/{wspr_pack,jt72}_rc1.tsv`). New `EncodeError::Wspr`, `EncodeError::Jt72`.
  `LIBRARY.md` §1.3; design §15.2, §15.3.
- **`msg::wsjt77::pack77_message`: WSJT-X's whole-message 77-bit packer (A2).** A port of `pack77_encode_core` (rc1 `packjt77.f90:527-627`)
  with its status codes (`Pack77Error`, `Pack77Options`, `Packed77`): the candidates in upstream's order (0.1 DXpedition, 0.3/0.4 Field Day,
  0.5 telemetry, 0.6, 1/2, 3 RTTY Roundup, 4, 5 EU VHF), each accepted only if it unpacks to the same text, then free text. Not ported: the
  `$DX` macro. Checked against rc1's own packer, run over its `messages.txt` and 1 203 cases of ours: `tests/fixtures/pack77_rc1.tsv`
  (2 163 rows, every candidate: status, `i3.n3`, bits, text, hashed calls) all equal, and 35 000 random rows once, outside the repo.
  `docs/notes/API_0_14_DESIGN.md` §8.
- **`Decoder<P>` and `AnyDecoder` are symmetric (A3).** `AnySlotResult` is `SlotResult<()>` and `decode_with` takes the same `OnRow`
  (breaking: the two parallel vectors and the `(&Decoded, &RowDetail)` callback are gone); `Decoder::decode_i16`, `Decoder::set_ap_hint`
  (FT8, FT4, FST4, Q65, through `HasApHint`) and `AnyDecoder::extras()` (`AnyExtrasRef`) are new; `wcalls()` borrows and `set_wcalls`
  returns `WcallsError`, so a too-long list is no longer reported as "unsupported" (the C ABI now answers `InvalidArg`, as its doc said;
  header unchanged). `TooManyCalls` gained `Display` / `Error`. Migration: `LIBRARY.md` §1.3; design §9.
- **API clean-up for 0.14 (C1-C3, L3, L4).** The four `ft8` migration façades (`message`, `ldpc`, `resample`, `hash_table`) are deleted, with
  their users moved to the real paths; `ft8::llr::GenericLlrSet` is gone; three names that collided with application-layer names are
  renamed (`uvpacket::UvMode`, `jtty::testsig::Transmission`, and `msk144` takes `decoder::Depth`, same levels and C values); the 22
  process-global counters of `wspr::instrument` / `fec::conv::fano::instrument` exist only with the new `instrument` feature (not in `full`;
  the CI row for embedded WSPR turns it on); doc lines naming `ft8-core`, `mfsk-fec`, `mfsk-msg`, `mfsk-engine` point at the modules. The
  `embedded-poc` sample apps follow separately. Old → new table: `LIBRARY.md` §1.3; design §3-§5.
- **`mfsk_core::prelude` and the tier map (L1, L2).** `use mfsk_core::prelude::*;` brings `Decoder`, `AnyDecoder`, `DecodeParams`,
  `SlotInput`, `Row`, `encode`, `Encoded`, `TxAudio`, `IqReceiver`, `Mode`, `Decoded`, `ApHint` and the rest an application names, each line under its
  module's feature gate; the protocol ZSTs stay at the root. The crate doc and `LIBRARY.md` §4 (+ `.ja.md`) sort the modules into
  application / protocol building / building blocks (the surface stays wide on purpose), and §4 writes down the builder naming rule
  (bare on a parameter struct, `with_` on a type with behaviour). Design §1, §2.
- **Every public type is `Debug`, every error type is `Display` + `core::error::Error`.** 78 types had no `Debug` (`Decoder<P>`,
  `AnyDecoder`, `SlotInput`, `IqReceiver` among them); a type that holds sample or spectrum buffers prints their lengths, not the
  samples (`Audio` too, which printed every sample). `Q65DecodeError` and the three `uvpacket` errors gain `Display`/`Error`; `IqError`,
  `Unsupported` and `jtty::pack::PackError` are `Error` without `std`.
- **`IqReceiver` can correct a source whose real rate is not its nominal one (#686).** `iq::plannable_rate(real_hz)` gives the integer rate to build the `IqStream` with (within 500 ppm of the real one, which the chain takes: `1 035 949` and `998 444` were `UnsupportedRate`) and the offset that is left; `IqReceiver::set_rate_offset_ppm` / `IqToAudio::set_rate_offset_ppm` (also `PfbChannelizer`) put every channel's audio on a true 12 kHz grid, with a last stage on the 12 kHz baseband (a Kaiser-windowed sinc cut at 6 kHz, phase interpolated linearly, no group delay, 0.016 % of a core a channel at 768 kS/s), the NCO and the PFB sub-bands following the real rate, and `set_time` / `utc_of` reading sample counts at the real rate. It may change while running without losing a slot; up to 50 000 ppm (5 %). `IqReceiver::step_time` sets the clock at once (not used by the skimmer: its real-RSP1 trial of a faster tracker decoded fewer messages, see `docs/notes/RTLTCP_TIMEBASE.md`). `IqReceiver::track_rate(Some(RateTrackConfig::default()))` learns the offset from the readings of `set_time` (a least-squares line through the anchors, three standard errors from zero, 400 ppm at most per fit, a fit a minute at the earliest). With no offset set the arithmetic is the old one. Before: an RSP1 that delivered 994 kS/s for 1 024 000 asked stepped the clock by a second every ~90 s (`SampleClock` slews 400 ppm and steps at 1 s) and warped FT8 by 180-370 ms a slot; a tracker given only the clock learns 1 500 ppm to 1 499.3 with no step. C ABI: `mfsk_iq_plannable_rate`, `mfsk_iq_set_rate_offset_ppm`, `mfsk_iq_rate_offset_ppm`, `mfsk_iq_track_rate` (additive; the ABI revision stays 3; not in Kotlin / Swift yet).
- **MSK144 decodes as audio arrives (#497).** `msk144::decode::Stream` takes 12 kHz `i16` in chunks of any size and reports each
  decode once its 0.6 s window is complete, as WSJT-X's `mskrtd` does; `new_period()` resets the dupe check (`mskrtd.f90:88`), the
  noise floor carries over. `decode_slot` is now a `Stream` fed the whole slot, so its output is unchanged; any chunk size gives
  the same decodes (`tests/msk144_stream.rs`). The C ABI and bindings follow separately.
- **FST4W, and FST4-900 / FST4-1800 (#649).** WSJT-X's WSPR-style beacon on FST4's modulation: `Fst4w120` / `300` / `900` / `1800`
  (feature `fst4w`), LDPC(240,74) + CRC-24 over a 50-bit type-0.6 message, `Decoder<P>` with `wcalls()` / `set_wcalls()` for the Keff-50
  known-call list, `Decoded::hash22` for an unresolved `<...>` row, `Mode::Fst4W*`, `pack77_wspr`. `Fst4s900` / `Fst4s1800` join FST4
  (largest slot transform now 21 591 360 points). C ABI: `MfskMode` 26-31, `hash22` on both rows, `mfsk_fst4w_pack`,
  `mfsk_decoder_get_wcalls` / `set_wcalls`; Kotlin and Swift follow (written without a JVM or a Mac: CI builds them first). Exact against
  upstream: 1 208 FEC inputs, 1 201 `genfst4` messages, `jt9 -W` / `-7` rows, and WSJT-X's own FST4W-1800 sample (`DL0HOT JO60 30`).
  Upstream bug found: `genfst4.f90:40-43` eats characters after two leading blanks (not reproduced). Detail: `LIBRARY.md` §3.3.
  Tier C (`fst4w_sweep`, task `fst4w/t1`, 1 760 files): the same as `jt9 -W` in all 8 conditions (20 upstream-only, 16 crate-only,
  crossings within ±0.17 dB, no extras either side), and 0.30× of upstream's time near the crossing (53 vs 176 ms; first measured 1.77×, #670: OSD pattern loop).

- **`IqReceiver::add_audio_channel`, `take_audio_from` and `utc_of_audio`, for a receiver with no slot; JTTY's
  `MessageUpdate::calls` (#650).** An audio channel is placed as a slotted one but cuts nothing: it keeps its continuous
  12 kHz audio, which `take_audio_from` hands out up to the first break, with the audio index of its first sample, so a
  retune, a gap or a clock re-anchoring is seen and JTTY's `Stream` can be finished and reset there. `take_audio` joins the
  runs as before. `MessageUpdate` gains `calls` (the call atoms' callsigns, in order, each once) and is now
  `#[non_exhaustive]`. For the skimmer's JTTY channels; `tests/iq_audio_channel.rs` runs JTTY through IQ to its calls.
- **`#[non_exhaustive]` on the public types that are expected to grow (#573).** One break, taken once, so that the growth
  itself stops being one: a new protocol is patch-level by this crate's convention, but it adds a `ProtocolId` variant, so each
  had technically been breaking, as is any field added to an output row, `ProtocolMeta` or a mode's `Extras`. 40 types gain it
  (53 now carry it): output rows and native results, registry metadata, the search knobs, `Station` / `QsoContext` / `Contest`, the
  option enums. A caller pays a `_` arm on a `match` and `Default` plus field assignment where it wrote a struct literal;
  `Station::new`, `QsoContext::new` and `Sniper::new` are new. `ApMode`, `Depth`, `QsoProgress` and the `engine` / `fec`
  primitives stay exhaustive. No behaviour changes. List and migration: `LIBRARY.md` §1.3 (and `.ja.md`).
- **The coarse-sync geometry belongs to the modes that use it, and derived constants are derived (#419).** `NFFT_PER_SYMBOL_FACTOR`,
  `NSTEP_PER_SYMBOL` and `NDOWN` move from `ModulationParams` to a new `SyncFrontEnd: Protocol`, implemented by the 7 types that
  decode through the shared front end (FT8, FT4, FST4 ×5); the other 17 carried values nothing read, and JT9's contradicted its
  own search. A mode without it cannot reach `engine::sync` or the generic pipeline: a compile error, not a placeholder.
  `SYMBOL_DT`, `N_SYMBOLS` and `LdpcParams::M` default to their identities; `TONE_SPACING_HZ` stays required (Q65's sub-modes
  B…E are `2^(letter−1)` wider) but is written with `tone_spacing_hz`, so a forgotten value fails to compile. FT4's hand-typed
  `20.833` is the derived `20.833334`; decode output is unchanged. `tests/protocol_invariants.rs` pins all 24 types.
- **`RowDetail` says what the mode reports (#594).** `sync_score`, `sync_cv` and `hard_errors` are `Option`s: WSPR, JT9, JT65 and Q65
  gave `0.0` / `0` on every row, and FT8's a7 and a8 list decodes a placeholder sync score, so a measured zero could not be told
  from a number the mode does not have. They are `None` there and `Some` for FT8, FT4 and FST4 searched decodes; the doc says on
  which scale. The C row keeps its layout and gains `flags` bits 2-4 (`MFSK_DECODE_FLAG_HAS_*`). `Decoded::new` exists, since
  a `#[non_exhaustive]` `Decoded` could not otherwise be built outside the crate.
- **A returned row names the delivery it was, and every mode's row has an identity key (#592).** `RowDetail::delivery` is the
  position of the delivery a returned row came from (compared exactly: no frequency rounded to the Hz, no key to build), `None` for
  a row the callback never saw. `RowDetail::info` is filled for WSPR (50 bits), JT9 and JT65 (72) and Q65 (77), not only FT8, FT4
  and FST4, so two JT65 `K1ABC W9XYZ EN37` rows are no longer one by text. `Q65Result` gains `bits77`; `Jt72Message::bits72()`;
  `AnyDecoder::delivery_is_exact()` says which `decode_with` contract a decoder runs (`STREAMING.md` §3a / §3b).
- **`SlotInput::budget` is honoured by every mode (#593), and FT8 `SicEarly` stops when it does (#587).** Only FT8, FT4 and FST4 polled
  it; WSPR, JT9, JT65 and Q65 ran whole and returned an all-zero `BudgetReport`. Each now polls once per candidate and sets
  `exhausted`; a candidate already running finishes, the coarse search is not cut, Q65's averaged decode (a whole period) is not
  cut. `SicEarly` polled nothing between its checkpoints, so a 300 ms budget ran 641 ms; it now runs 304 ms (`BudgetReport` gains
  `rows_subtracted`). The four `DecodeRequest`s gain `.budget(check)` and `.decode_reported()`. `tests/decoder_budget.rs`.
- **Early decode for live use: `Decoder::decode_prefix` / `decode_prefix_with` (#572).** Call it as audio arrives with the period
  so far and `SlotInput::period`. FT8 returns checkpoint A's rows at 141 696 samples (~11.8 s, `Stage::Early`), subtracts at 162 432,
  and at the whole period returns the same rows in the same order as `decode`; other modes return nothing until then (tests also under
  `fixed-point`). `SicEarly` runs as resumable steps (`ft8::decode::Staged`) with no change to its rows. C
  `mfsk_decoder_decode_prefix_i16` / `_f32` and `MfskDecode::stage`; Kotlin and Swift `decodePrefix`. Design: `EARLY_DECODE_DESIGN.md` §12.
- **`IqReceiver` delivers prefixes (#600).** `set_prefix_points(id, points)` (off by default) cuts a channel's slot at exactly
  those 12 kHz counts, then whole, through the same `push_*` (`CompletedSlot::is_whole()`); the points come from the channel's
  decoder, `Decoder::prefix_points()` (`[141_696, 162_432]` for FT8 under `SicEarly`, empty otherwise). Through IQ `qso3_busy`
  gives 15 rows at 11.8 s and the same 21 at the end as `decode` (`tests/iq_prefix.rs`). Design: `IQ_PREFIX_DESIGN.md`.

### Decoding

- **`pack77`, `pack77_type1`, `pack77_type4`, `pack28`, `pack77_free_text` give rc1's bits.** They were written before `pack77_message` and
  differed from it: a directed CQ (`CQ DX`, `CQ POTA`, `CQ 123`) was left-justified, not `adjustr`ed (4 of the fixture's rows, `pack28_core`
  `packjt77.f90:882-1001`); the 12 bits of a type-4 `CQ` were 0, not the hash of the call (`:2377-2404`); `CQ` + report, `CQ 12`, a `Q` call and a standard call as type 4 were
  accepted, `+50`, `<hash>` calls and lower case refused; the empty text was refused. Now thin wrappers over `pack77_message`'s code (`tests/pack77_forms.rs`,
  the 2 163-row fixture; before, 177 divergences in 18 classes). The C ABI (`mfsk_encode_ft8` etc.) and apps change on air for those inputs only; plain
  CQ / report / RR73 messages are bit for bit as before. The A7 CQ stand-in `QU1RK` keeps its bits. `LIBRARY.md` §1.3.
- **JT9 and JT65 print free text and compound calls as `jt9` does; WSPR text matches `wsprd`.** `Jt72Message::Unsupported` rows are gone:
  JT9 no longer drops non-standard messages (`jt9fano.f90:87-88` filters only `000AAA`), JT65 blanks `000AAA` and the `0L6MWK` birdie
  (`decode65b.f90:27-28`) and no longer emits `<unsupported …>`. WSPR follows `wsprd_utils.c` `unpk_`: a Type 3 call unresolved is `<...>`
  (was `<#hhhhh>`), `<CALL>` once heard; `PFX/CALL` as `wsprd` prints it; `unpackcall` exact, so some calls that were dropped now decode. Checked
  row by row against rc1's own routines (7 651 JT rows, 26 046 WSPR rows); the JT65 / JT9 goldens (4/5, 7/7, extra 0) and the WSPR golden
  (9/9, extra 0, texts and freq/dt/snr identical) are unchanged: the references contain no such messages. One divergence: `ntype == 63` is
  `None`, not an empty message. `LIBRARY.md` §1.3; design §15.3.
- **`unpack77` matches rc1 on `callok`, hash placeholders and telemetry.** `unpack28` follows `unpack28_core` (`packjt77.f90:1013-1096`): a
  standard call `callok` refuses (`AB1`, `FN4`, a leading `Q`) or a call / `CQ` word with a blank inside fails to unpack, so `unpack77`
  returns `None` where rc1 has `ok = .false.` (FST4W transmit, through `pack77_wspr`, refuses such a call too); the 28-bit fields
  532444 and up print `<...>`, not `<?>`; telemetry prints all 18 hex digits (`3z6.6`, `:1640`), not without leading zeros. JTTY keeps the
  unchecked text. Vectors are rc1's own `unpack77`; the 2 163-row `pack77_message` fixture matches with its two local patches removed, and no
  golden expectation moved. `LIBRARY.md` §1.3.
- **JT65 deep search (`hint65`) is ported (#648).** `DecodeParams::deep_search` (`ndepth & 32`) was carried and ignored; it now matches
  a candidate Chase fails on against a bank of known calls (`Jt65Extras::known_calls`, WSJT-X's `CALL3.TXT` that beta1 made a
  caller-supplied list) plus the DX station's reports, from v3.3.0-beta1's `hint65.f90`. On the `jt65sim` corpus -25 dB goes from 0/20 to
  20/20; 0 phantoms on 300 files and on 200 noise periods at 1 000 calls (1 at 10 000). Standard messages only, single periods only;
  `docs/notes/BENCHMARKS.md`. The C ABI does not carry the call list yet.
- **OSD is `osd174_91.f90` / `osd240_101.f90` line for line, and 8× faster than upstream (#417).** FT8, FT4 and FST4 share one
  (`fec::ldpc::osd_npre`), replacing a (174,91)-pinned port and a generic one that matched upstream in neither: both picked the basis
  as "first `k` independent columns" rather than the `id..k+20` column swap, and the pinned one mis-scored candidates when the pivots
  left `0..k`. Against WSJT-X's own Fortran (gfortran, release flags) it agrees on all 18 860 recorded inputs
  (`tests/osd_upstream_fixture.rs`, regenerated by `scripts/osd-upstream/run.sh`). A call takes 24 µs against upstream's 198;
  `qso3_busy` at `Depth::Deep` decodes in 504 ms instead of 570, the FST4-60 golden in 597 instead of 951.
- **The plausibility verdict no longer checks callsign prefixes (#612; absorbs #611's `A5A` fix, thanks BG7JAF).** FT8 and FT4 refused
  a callsign whose one-letter prefix was not F G I K M N R W or whose letter+digit prefix was not in an 82-entry ITU table. That
  refused 13 of the 100 303 callsigns spotted on PSK Reporter in FT4/FT8 since 2024-06-20 (all special-event: `J42A`, `L22D`,
  `U1BD`, `U5WAR`, …), and `A5A` (Bhutan) outright, and the table lacked 57 of the 138 letter+digit prefixes `cty.dat` lists. With it
  off nothing else measured moved (tier C for FT8 and FT4, the busy-band corpus, `qso3_busy`, the FT4 golden, 3 000 FT4 noise slots);
  the grammar check, which does the work, stays. Audit and numbers on #612.
- **JT65, WSPR and JT9 take a carrier's sub-bin offset out through one `SymbolFft::mixed` (#424).** JT65 corrected with per-sample
  `cos`/`sin` of a wrapped f32 phase, WSPR with `step · n` on the absolute sample index (~0.1 rad by the end of a slot), and JT9's
  fixed-carrier path (`SniperRequest`) not at all, losing up to ~3.9 dB for a carrier half a bin off. All three now mix each symbol
  window with the crate's `Mixer`. JT65 and WSPR move by one part in a million (tier C: every crossing +0.00 dB); JT9's sniper path
  gains the correction. Q65 is left as it is: `q65_symspec` itself works on nearest-bin spectra, so correcting it would diverge from WSJT-X.
- **`Decoder<Wspr>` and the MSK144 decoder no longer panic on `wasm32-unknown-unknown` (#583).** Their stage timers called
  `std::time::Instant::now()` whenever `std` was on, and on that target std has no clock: `time not implemented on this platform`,
  on any input (every browser build of WSPR or MSK144 since 0.10.1 at least). The timers go through one `Tick` (`src/clock.rs`) that
  reads zero without a clock; the values only feed benchmark counters, so no decode changes.
- **FT8's symbol scaling and `nsync` gate follow `ft8b.f90` (#423).** `cs = csymb / 1e3` is a true division again (a scratch gfortran
  oracle: `z/1e3` matches componentwise division on 2M of 2M samples, `* (1/1000)` differs on 81%), and the `nsync <= 10 && xsnr < -25`
  gate reads the Costas count of the candidate's refined symbol spectra (`ft8b.f90:164-177`) instead of recounting on the coarse
  spectrogram after decoding. On the goldens and the FT8 sweep (T1, 1 040 trials) no group moves; the gate flips for one candidate
  in the whole tier-A/B suite.
- **An FT8 `decode_prefix` sequence pins the sniper with the rest of its settings (#628).** Turning the sniper on after checkpoint A
  made the final call run one sniper decode instead of finishing the staged sequence, so early rows already returned were missing
  from the "complete" set (21 rows down to 2 on `qso3_busy`). Whether the period runs the sniper, and on which frequency, is now
  fixed by its first call like the strategy and search; a change applies from the next period. AP, QSO context and the message
  filter are still read at each call.
- **Q65 follows WSJT-X v3.3.0-beta1 where its results move (#642).** The symbol-spectra horizon `jz` now covers every complete
  window, `(period*12000-nsps)/istep+1` (60A: 743 columns, was ~693), and the `max_drift` divisor is the 85-symbol length rather
  than `jz`, so the two change together. The contest caller list evicts the least recently heard rather than the first inserted,
  expires before recording, and lists the DX station as the 51st beside 50 callers (`1 + 10 * 51` = 511 codewords, was 411). The
  tier-C Q65 sweep is unchanged to 0.00 dB in all 20 groups: it does not run q3, drift or the caller list, so it shows no
  regression and nothing more; `q65_max_drift` and `q65_q3` still pass.
- **JTTY packs a two-letter section after a class token as an atom, as WSJT-X v3.3.0-beta1 does (#642).** `10A CT` and `1F DX`
  are now one frame; through rc1 upstream's lookup rejected a word shorter than three characters, so they went out as TEXT5
  (two frames), and this crate reproduced that. A transmitted message changes on the wire for these inputs; a receiver decoded
  both. `pack_cases.tsv` is regenerated from a beta1 build: 11 of 3 525 lines change, all this, and `jtty_pack` passes.
- **JTTY reports an SNR a user can read, as WSJT-X v3.3.0-beta1 does (#642).** `FrameDecode::snr_db` is now
  `db((pt − pn) / pn) − db(2500 / baud)` floored at −17 dB (`SNR_FLOOR_DB`), an SNR in 2 500 Hz; through rc1 it was `db(pt / pn)`,
  a tone-bandwidth ratio that upstream showed as `round(snr_db − 20)`. `MessageUpdate::snr_db` is the first frame's, never
  reassigned, appended to `MfskJttyUpdate` (ABI stays 3) and to the Kotlin and Swift updates (`snrDb`). Against a v3.3.0-beta1 `rjtty` on
  `sjtty` files, true −10 / 0 / 10 / 20 / 30 dB read −9 / 0 / 7 / 10 / 11 there and −9.4 / −0.2 / 7.5 / 10.5 / 11.0 here: a strong signal
  reads low, upstream's own limit. The sync gate still uses the old ratio, as upstream does.
- **Degenerate inputs follow WSJT-X v3.3.0-beta1 (#642).** Q65's q3 `better` is 0, not `inf`, when the list has no runner-up, so a
  one-codeword list is no longer accepted (rc1's unguarded divide passed `better >= 1.10`); silence and an empty list decode nothing.
  FST4's sync search stops at `last_start`, the last start with a whole frame behind it, and skips a candidate when none fits
  (`isbest<0`); rc1 scored the blocks past the end as 0 and let a partial score win. The FST4 tier-C sweep is unchanged in all 20
  groups (crossings +0.00 dB, unexpected decodes +0): its full-slot files never reach `last_start`, so this is a no-regression result, and
  the mechanism is pinned by a unit test.
- **JTTY can send a message open, and says where each frame's text starts (#643).** WSJT-X v3.3.0-beta1's `pack_jtty(..., frame_starts,
  is_final)`: `pack::pack_with_starts` returns the 1-indexed source column of each atom, `tx::payloads_with` / `tones_with` and
  `pack::tones_with` take `is_final` (false: no end-of-message flag on the last frame, so a message typed in pieces can be
  closed later), and the C ABI gains `mfsk_jtty_encode_tones_ex` (`mfsk_jtty_encode_tones` is `is_final` = 1). Checked against a
  v3.3.0-beta1 `jtty_pack_oracle` on 4 025 cases (500 of them `is_final` = 0), frames and starts identical. Kotlin and Swift do not
  expose it yet.
- **A JTTY update says why it was emitted (#644).** `MessageUpdate::kind` is `UpdateKind::{Growing, Complete, Expired, ReceptionEnded}`,
  WSJT-X v3.3.0-beta1's `UPDATE_*` with the same values; `Assembler::end` (what a stream's `finish` calls) reports open messages as
  `ReceptionEnded`, where a message given up on is `Expired` and both used to be `complete = false`. Appended to `MfskJttyUpdate`
  (`kind`, with `MFSK_JTTY_UPDATE_*` in `mfsk.h`; ABI stays 3) and to the Kotlin (`kind`) and Swift (`kind`) updates, which are not
  built here.
- **JTTY keeps absolute time in `f64`, as WSJT-X's `real64` (#645).** `FrameDecode::tsync_s`, `MessageUpdate::start_s`,
  `assemble::FRAME_PERIOD_S` and the `Assembler`'s `prune` / `continuations` are `f64`; `Stream` derives window times in `f64`. An `f32` second is 8 ms
  wide at 24 hours and 0.125 s after about 15 days, against 0.05 to 0.1 s continuation tolerances, so a stream left running
  could stop joining frames. Per-column and per-sample loops stay `f32` (the ESP32-S3 has single-precision hardware only).
  C ABI: `MfskJttyUpdate::start_time_s` (`f64`) appended after `kind`; `start_s` stays `f32`. Kotlin and Swift `startSeconds` are
  `Double`. JTTY tier-C sweep and the `rjtty` frame-for-frame tests are unchanged. Breaking for Rust callers that read these as `f32`.
- **The CoreS3 JTTY app and demo show the message's SNR (#646).** `MessageUpdate::snr_db` goes to the row (it was `NAN`, drawn `+0`, and
  `0` in the demo) and to `ALL.TXT`, whose JTTY line now carries the S/N column (`all_txt::message_line` takes `snr_db`; the DT column
  stays blank, a message has no slot). Host-tested in `hosttest/mfsk-app-shared`; `cargo check` of the crate with the `esp` toolchain
  passes. Flashed (`jtty-demo`, the busy-band scene, 2026-10-10): the four messages that complete read -10 / -5 / -4 / -4 dB against true
  -9.6 / -5.3 / -2.8 / -4.7 dB; the value is upstream's simple estimate, so a strong signal reads low (a true +30 dB reads about +11).
- **Four packing bugs found by comparing Q65's caller list with WSJT-X v3.3.0-beta1 (#642).** (1) `pack77_free_text` padded on the right;
  `packtext77` right-justifies (`adjustr`), so free text had other bits than WSJT-X's for the same words (read back the same, which hid it;
  FT8/FT4/Q65 free-text transmit and any exact-codeword comparison). (2) `pack28` dropped the sixth character of a one-letter-prefix call:
  `W1ABCD` and `K1ABCD` packed as `W1ABC` / `K1ABC`, now not standard. (3) `pack77` took `CQ`, `DE`, `QRZ` as the second call. (4) The
  full-AP list skipped a caller whose message would not pack and left out the DX station when the own call was nonstandard
  (`PJ4/K1ABC`); upstream packs it as free text cut to 13 characters (`*** bad message ***` as codewords of -1 when those are not
  free-text characters) and asks only whether the DX call is standard. `pack77_q65_list_message` is that packer; a negative symbol
  scores as never matching in `check_codeword_llh`. `tests/q65_callers_oracle.rs` covers all of it against a beta1 build.
- **The BP uses WSJT-X's `platanh` (#658).** Every WSJT-X belief-propagation decoder calls `lib/platanh.f90`, a piecewise-linear `atanh`
  (one definition, duplicated unchanged in `bpdecode144.f90`; `atanh` is only in comments). This crate's generic BP called `atanh`
  clamped at ±4.6. Same corpora, one binary switched by an environment variable: FT8 -2 net decodes of 592, FT4 +4 of 555, FST4 -1 of
  1881, FT8 busy 0 of 4441 (one more unexpected decode), MSK144 identical; no 50 % crossing moved by 0.5 dB (largest 0.36 dB,
  FST4-300 CCIR-poor); the six suites ran in 139 s against 167 s. FT8 again at 100 trials a cell: 2561 vs 2559 decodes (12 gained, 14 lost, p = 0.85), crossings within 0.04 dB. Messages and their order are unchanged in all 53 decode snapshots;
  `hard_errors` moves by up to 11 and `pass` in 88 rows, so the 22 snapshots where one of them changed were rewritten (the other 31 are untouched).
  `tests/decode_snapshot.rs`'s note that a difference is never a reason to rewrite a fixture was about the 0.13 redesign.

### Internals

- **DSP duplicates become one (#425).** `engine::dsp::window::cosine_sum` replaces the two Nuttall-4 loops (symmetric in
  `engine::sync`, periodic in `ft8::baseline`), bit-identical to both. `engine::dsp::downsample::repack_centered` is the FFT-shift
  re-pack that `wspr::baseband` and `jt9::softsym::downsam9` each wrote. `Mixer` is a const generic over `LEAD` and `RENORM`;
  `Mixer::tweak1` is `tweak1.f90`'s configuration, bit-identical to `msk144::sync`'s loop, and the default instantiation, which the
  CoreS3's FT4 DDC runs, compiles to what it was. `cpfsk::for_each_phase` is the transmitter's phase accumulator, now also walked by
  `subtract`'s non-GFSK rebuild (last bit of `cos`/`sin`; transmit output bit-identical). Each is pinned against the code it replaced.
  `jt9::baseband` is removed (breaking): no callers, and its doc claimed to mirror `downsam9.f90`, which it does not. `uvpacket::rx` mixes
  with `Mixer` instead of `sin_cos(step · n as f32)`, whose carrier sat 0.016 rad off over the last 4 000 samples of a 60 s buffer.
  Left alone on purpose, with the reasons in `window.rs` and `Mixer`'s docs: the oscillators and windows on the CoreS3's decode path, `twkfreq_poly`.
- **FT8's triage shifts the refined carrier with the crate's mixer (#423).** `triage_candidate` rotated `cd0` by per-sample `cos`/`sin`;
  it calls `freq_shift_cd0_into`, the transform FT4 has run since. tier A+B unchanged, tier C for FT8 (every crossing +0.00 dB) and the
  busy-band corpus identical. The rest of #423 is dispositioned on the issue: the two SIC drivers stay apart, and two items that look like
  refactors are fidelity changes.

### C ABI and bindings

- **Release binaries for Windows, Android and iOS (#368).** A tag attached one `mfsk-ffi` build, linux-x86_64, though `ci.yml` already built
  the MinGW DLL, the Android arm64 `.so` and the iOS libraries on every PR and threw them away after checking them. The new
  `ffi-binaries.yml` (called by `release.yml`) keeps them: `windows-x86_64-gnu` (DLL, import library, static library), `android-arm64-v8a`
  (jniLibs tree, `mobile` features, checked 16 KB aligned and rayon-free), and `ios.xcframework.zip` with its SHA-256 for a SwiftPM
  `.binaryTarget`. It adds `windows-x86_64-msvc`: a DLL with the C runtime linked in (`+crt-static`; the default build imports
  `VCRUNTIME140.dll`, which a PC without the VC++ Redistributable lacks) and its import library. The C++ driver is compiled with `cl` and run
  on Windows against that DLL and against the MinGW package's, so a DLL that does not load there fails in CI, not for a user. The driver
  takes the golden recordings' paths from the environment as well as from `build.sh`'s macros, and `build_msvc.bat` builds it. The workflow
  also runs by hand and on a PR that edits it, so the release path is tried before a tag. Still missing: the other Android ABIs, macOS.

- **MSK144 receiver handle (#497).** `mfsk_msk144_open` / `push_i16` / `push_f32` / `poll` / `pending` / `new_period` / `reset` /
  `close` over `msk144::decode::Stream`, rows in `MfskMsk144Decode` (size-versioned, `t_s` a `double`); MSK144 now publishes
  `MFSK_CAP_STREAM_RECEIVER`, and `mfsk-ffi` compiles the MSK144 decoder in (`protocols`). Kotlin `MfskMsk144Receiver`, Swift
  `Msk144Receiver`; the C++ driver, Kotlin and Swift each decode the golden recording's two pings in chunks. `BINDINGS.md` §2.8.3.
- **MSK144 transmit (#497).** `MFSK_CAP_ENCODE` was set for MSK144 with no entry point behind it. `msk144::tx::message_tones` ports
  `genmsk_128_90.f90` and `synthesize` plays the tones as WSJT-X's modulator does (6 samples a tone, 1000/2000 Hz, the frame
  repeated for the period less 0.5 s, its end fade); `mfsk_encode_msk144`, Kotlin `Mfsk.synthesizeMsk144`, Swift `MSK144.encode`.
  Each binding's test sends a message and decodes it back through the receiver handle.
- **An array of `MfskDecode` is stepped by the caller's stride, not the library's (#607).** The decode wrote row `i` at the library's own
  `sizeof(MfskDecode)`, so a program built against an older, shorter header got row 1 onward in the wrong place and wrote past its
  buffer. The stride is now `out[0].size`, applied to every row, with `min(size, sizeof)` bytes written each; a `size` that cannot be a
  struct size is refused with nothing written. `mfsk_q65_history_record` reads by the same rule. `tests/row_stride.rs` (checked by
  reverting the fix); `BINDINGS.md` §2.4 (and `.ja`).
- **Early rows through the push handles, and Kotlin and Swift over them (#601).** `mfsk_iq_push` decodes an FT8 channel early by default:
  checkpoint A's rows reach the callback and `mfsk_iq_poll` at ~11.8 s with `MfskIqDecode::stage` = `MFSK_STAGE_EARLY`, and the whole slot
  queues the rest without repeating them; `mfsk_iq_set_early(rx, ch, false)` restores whole slots. The stream is opt-in
  (`mfsk_decoder_prefix_points`, `mfsk_stream_set_prefix_points`, `mfsk_decoder_decode_stream`, `mfsk_stream_slot_is_whole`).
  Kotlin and Swift likewise; Kotlin's lent channel decoder now takes `onDecode` and `setBudget`. `BINDINGS.md` §2.5, §2.8.2 (and `.ja`).
- **The C ABI and the Kotlin and Swift bindings catch up with #592, #593 and #594.** `MFSK_CAP_BUDGET` is published for WSPR, JT9, JT65 and
  Q65, so `mfsk_decoder_set_budget` no longer refuses them (`registry_caps.rs` ties the bit to "every mode with a `Decoder`").
  `MfskBudgetReport` gains `rows_subtracted`, `mfsk_decoder_delivery_is_exact` is new, and `MfskIqDecode` appends `MfskDecode`'s detail
  (key, flags, sync) and `key_bits` / `key` / `delivery`. All appended to size-versioned structs; ABI stays 3. Kotlin and Swift:
  `syncScore` / `syncCv` / `hardErrors` are nullable, rows gain `key`, `keyBits` and `delivery`. Kotlin's JVM test and the Swift
  suite (four new tests) pass in CI. `CompletedSlot::new` lets a consumer's tests build a slot. Docs: `BINDINGS.md`, `STREAMING.md`.
- **A short output buffer no longer loses the call's result (#633).** `mfsk_decoder_decode_stream` took the stream's ready slot before it
  checked the buffer, so the retry the ABI asks for found no slot (`UNSUPPORTED`): Swift's `decode(CaptureStream)` returned `nil` and
  the Kotlin shim threw. The other `decode_*` calls re-decoded on retry, which fired the callback twice and stepped a Q65 average twice.
  A refused call now keeps its rows (and a stream's period and time), and the retry is answered from them.
- **Float audio at 44.1 / 48 kHz goes the same way as at 12 kHz (#634).** `mfsk_decoder_decode_f32` and `_prefix_f32` peak-normalised each
  buffer to 16 bits when resampling, so a prefix sequence's level followed each prefix's peak instead of being pinned by the first, and
  the float-engine modes (WSPR, JT9, JT65, Q65) were quantised. They now resample keeping the level and hand over `float`, so the
  decoder sets the gain once per period. No decode changed on a synthetic FT8 signal or `qso3_busy.wav` at 48 kHz (21 rows, with
  and without a loud click after B); this is consistency, not a measured recall gain.
- **Rows decoded into a newer caller's array go straight to `mfsk_q65_history_record` (#635).** A decode rewrote every row's `size` to the
  bytes it wrote, so a caller whose `MfskDecode` is longer than the library's saw `size` shrink to the library's `sizeof`, and the history
  (which steps by `rows[0].size`) read row 1 from inside row 0's tail. `size` now comes back as the caller's stride. A single struct
  (`MfskModeInfo`, `MfskParams`) still reports the bytes written. Appended-field callers who read the written length from a row
  should use the header's `sizeof` instead.

### Skimmer

- rtl_tcp (#686, `docs/notes/RTLTCP_EVAL_PLAN.md` section 6): `RateMeter` takes a pause out of its time axis (a 1.4 s reset
  in the 30 s window read 1.7-2.0 % short), `await_rate` waits 20 s of stream when the stream was never at another rate
  (rsp_tcp's Init), and `IqReceiver` applies a learned rate offset on a boundary of every active channel's period
  (60 s / 120 s modes no longer get it mid-slot). Not yet run on an RSP1.
- rtl_tcp rate calibration now separates the requested ADC clock used for RF mixing from payload throughput
  used for audio timing. Normal servers need no patch; the previous ADC-clock model remains selectable.
  Two fixed-IQ comparisons improved FT8 rows from 0 to 10 and 1 to 28; long-pause reanchors remain.
  Methods and limitations: `docs/notes/IQ_RSP1_PR688_2026-10-15.md`.

- **The skimmer can read an `rtl_tcp` server, for a device SpyServer does not serve (#685; first target an SDRplay RSP through `rsp_tcp`).** New `skimmer_core::rtltcp` (greeting `RTL0`, `rsp_tcp`'s `RSP0` capabilities struct, 5-byte commands) and `session_rtl`: one fixed rate, 8-bit or 16-bit IQ (`Cu8` / `Cs16`, chosen by what the server announces; `Auto`, `8` or `16` checks it and refuses a mismatch with the way out), and a retune that waits 600 ms, writes the gain again and drops the old tuning's samples. `Config::protocol` / `rtl_format`; CLI `--protocol rtl_tcp --rtl-format auto|8|16`; GUI *Protocol* and *rtl_tcp IQ* per server (older settings read as SpyServer). The step/hold logic of `session` is now `pick_step`, shared; `stream` takes any `Link`. Tested against `fake::FakeRtlServer` (`tests/fake_rtltcp.rs`). On a real RSP1 it decodes FT8 (JA stations on 7041 kHz, 4-9 per slot). `rsp_tcp` reads no commands until its `Init` is over (10-15 s) and streams meanwhile at its start-up rate: the client waits for the byte rate (`rtltcp::await_rate`) before taking the stream (six `re-anchor: -1.0 s` in the first minute before). The rate it delivers was 994 kS/s for 1 024 000 asked (-2.9 %): the client measures it (`--rtl-calibrate SECS`, 30 by default), gives the receiver the nearest rate it takes and the rest as a rate offset (`IqReceiver::set_rate_offset_ppm` / `track_rate`, #686), and the clock was stepped once in 6 minutes instead of every ~90 s. That RSP1 on a bare USB hub also failed (`RSP failure`) within 1-7 minutes, with or without a client sending anything: give it its own powered port; 8-bit has ~48 dB of range (measured no-signal floor 1.8 LSB). Released in skimmer 1.0.1.
- **The skimmer GUI frames the band being heard in red (rotation).** In the waterfall's thumbnail row, the thumbnails of the channels in the band the server's rotation is on now get a red frame; the focus keeps its blue outline. Nothing is framed when the server does not rotate or is stopped. GUI only (`WaterfallPanel.svelte`); not in a tagged release yet.
- **The skimmer 1.0.0 sends to PSK Reporter itself (#655).** PSK Reporter's author, after the test-listener period, said to keep
  sending in production. `pskreporter::APP_ENDPOINT` is now `Endpoint::Production`, for the CLI and the GUI; the tripwire test,
  the README, `MANUAL_SKIMMER` (EN/JA), the CLI usage and the release text say so. The GUI's *Test only* wording and the header's
  `(test, not recorded)` drop out by themselves. Checked live on the production-pointed 0.9.0 build: spots from JL1NIE appeared on pskreporter.info.
  Tag `skimmer-v1.0.0` (the four version files and `package.json` agree).
- **The skimmer's window says where PSK Reporter spots go.** The switch has no destination to choose (`APP_ENDPOINT`, the test listener until
  PSK Reporter's author approves), but the window did not say what it was, so ticking it looked like reporting to the real service. A new
  command, `psk_destination`, lets the label read *Test only* with the listener's address and the header `PSK Reporter (test, not recorded)`;
  both drop the wording by themselves when `APP_ENDPOINT` becomes production. A unit test pins `APP_ENDPOINT` to the test listener, and its
  failure message is the checklist for flipping it (README, `MANUAL_SKIMMER` section 10, the release text, this file). In 0.9.1; the 0.9.0 window does not say it.
- **The skimmer's first public release is 0.9.0, a beta (#655).** The version goes from the 1.0.0 set in preparation to 0.9.0: 0.9.x is the beta,
  1.0.0 the first release with PSK Reporter in production (its author has not yet said yes, and the destination stays the test listener). A
  reader of the old installers (v0.13.x, which lacked JTTY, PSK Reporter, early FT8 rows and decode lanes) sees how much has changed; Windows'
  installer allows the lower number (`allow_downgrades`, Tauri's default). The release text says what the beta is and is not, and `MANUAL_SKIMMER.md`
  (`.ja.md`) is the user manual. The decode table's *Clear* tooltip named `ALL.TXT`, which only the command line writes; it names the database.
- **PSK Reporter's IPFIX packets, byte for byte (#655, stage 1 of 4).** `skimmer_core::pskreporter::build_packets` builds the receiver
  record, the spots and the two templates exactly as WSJT-X v3.3.0-beta1's `PSKReporterIPFIX.cpp` does (UDP 952-byte and TCP limits, the
  receiver record in every datagram, templates in the first only, a sequence that counts spots, UTF-8 cut at its limits, a five-byte
  frequency, a wrapping SNR). `scripts/pskreporter/ipfix_oracle.cpp` links that file against Qt5Core; `tests/pskreporter_ipfix.rs` requires
  the same bytes on 58 datagrams of 32 builds. Nothing sends yet: the sender, the five-minute batching the protocol page asks for, the
  dedupe and the opt-in setting are the next stages, and nothing goes to PSK Reporter until its author has been asked about skimmers.
- **The skimmer CLI can send spots to PSK Reporter (#655, stage 2 of 4; `--psk-reporter`, off by default).** `PskReporter` batches on a
  thread of its own and sends on one UDP socket, as the protocol page asks: a datagram at most every five minutes plus a random addition,
  a callsign once per band per five minutes, templates in the first three datagrams and hourly, `informationSource` 1. `spot_from_decode`
  and `spot_from_jtty` are WSJT-X's rules (no WSPR, no low-confidence, no own call, a locator or a CQ). It goes to PSK Reporter's **test
  listener** (`pskreporter.info:14739`, nothing recorded) unless `--psk-production`, which waits for PSK Reporter's author (#655);
  `--psk-to HOST:PORT` is a listener of one's own. Checked on a live SpyServer against a local listener: 36 FT8 rows offered, 8 duplicates,
  28 sent in 3 datagrams from one port, with the receiver record and the templates.
- **The skimmer GUI has the PSK Reporter switch (#655, stage 3 of 4).** Per server, under Settings > Server: report under the server's My call
  at its Grid (the antenna's locator), with a free-text antenna; a server missing either stops Connect with the reason. The destination is
  the test listener (`pskEndpoint: "test"`); production is listed greyed out until PSK Reporter's author has been asked. The header shows
  `PSK Reporter: N sent` (red on an error) with the counts in its tooltip. Settings from before read as off.
- **The PSK Reporter sender packs 1200-byte datagrams and sends a full one early (#655).** PSK Reporter's author answered: more often than every
  five minutes is fine once a datagram is full (1200 bytes or more), larger datagrams are fine, `mfsk-skimmer <version>` is the software name, and one
  callsign per antenna location with that antenna's locator (already so, per server in the GUI). The sender's datagram limit goes from 952 to 1200 bytes
  (`FULL_UDP_PAYLOAD_BYTES`; `build_packets` and the WSJT-X byte-for-byte tests keep 952) and a spot that would overfill the queue sends it first.
  The destination is one constant, `pskreporter::APP_ENDPOINT`, for the CLI and the GUI: the test listener until the author says yes to
  production. The GUI's "Send to" choice and the CLI's `--psk-production` are gone (`pskEndpoint` in an old settings file is ignored); `--psk-to` stays.
- **The skimmer is released on its own (`skimmer-vX.Y.Z` tags, version 0.9.0 on).** `skimmer-release.yml` checks the tag against the skimmer's
  version (four files), builds the installers and makes a release not marked latest, without crates.io; the library's `release.yml` no longer
  attaches them. An app fix no longer waits for the library's biweekly cut. PSK Reporter now sees `mfsk-skimmer 0.9.0`: 0.9.x is the beta, 1.0.0 the first release with PSK Reporter in production.
- **PSK Reporter spot rules are WSJT-X's, checked against WSJT-X (#655, stage 4 of 4).** `scripts/pskreporter/spot_oracle.cpp` runs upstream's
  `tokens_re` / `deCallAndGrid`, `Radio::is_standard_callsign`, `decoded_grid_pattern` and the real Fortran `stdmsg_`; the answers for 1 537
  words and 68 messages are vendored and `tests/pskreporter_spot_rules.rs` requires them (143 real decodes off the air agreed too). The
  check found that the skimmer's own heuristic missed `K1ABC W9XYZ R EN37`, took messages upstream calls free text, and that the JTTY
  sender rule refused `/P` and `/R` calls and took `RR73` for a locator; PSK Reporter now uses upstream's rules, and so does the JTTY
  one (`JttyMessage::sender`, which the station list reads too). A slot that began before the radio was last retuned is not reported.
- **skimmer: JTTY (#650).** JTTY has no slot (frames start at any moment and a message grows while it is received), so a JTTY
  channel is not decoded a slot at a time: the receiver keeps its continuous audio (`IqReceiver::add_audio_channel`), a thread
  per channel runs upstream's receiver over it at a slow AGC (`jtty::rx::Stream`; Rx frequency and tolerance are the channel's
  `rx` and `tol`, 1500 ± 50 Hz as upstream, plus the fixed side channels), and a break in the audio (a retune, a gap, the clock
  re-anchored) ends the reception, so a half-heard message is reported cut off and never joined to the next. `Event::Jtty`
  reports a message as it grows and when it is over. The CLI prints it and writes ALL.TXT in WSJT-X's format when it is over;
  the database stores it as a `decodes` row of mode `JTTY` with the sender and locator by WSJT-X's spotting rule, so Analysis,
  the map and the station list take it; the window has a JTTY tab where a message is updated in place; presets carry upstream's
  JTTY frequencies; a JTTY channel has a waterfall like the others, drawn from the audio its thread reads. As upstream: no full-band search, and no drift search. Tested through IQ,
  both channelizers, and across a gap (`jtty::tests`, `store::tests`). A stand-in SpyServer (`skimmer_core::fake`,
  `cargo run -p skimmer-core --release --example fake_spyserver`) speaks the protocol and plays a 48 s script of two QSOs and a CQ
  caller at 1500, 1350 and 1650 Hz, for trying the CLI or the window without a radio; `tests/fake_spyserver.rs` runs the whole
  skimmer against it over a socket.
- **Connect no longer waits 10 s on a dead NTP server (#640).** The first clock measurement ran five queries at 2 s each against the
  first address only, so a pool member that did not answer held Connect for 10.6 s before falling back to the PC clock
  (`pool.ntp.org`, 2026-10-09, while `ntp.nict.jp` answered in 38 ms). A query now waits 1 s (a later reply is over the 1 s
  round-trip limit and discarded anyway), and an address that has not answered is passed over for the next of the ones the name
  resolves to, up to three: the same pool now measures in 1.9 s, and nobody answering costs 3 s. An address that has answered keeps
  being asked, and a name that resolves to one address gets one retry, so a single lost packet does not end the measurement.
- **The skimmer GUI makes an unset grid visible (#640).** With no home grid the map's *Great-circle* button is disabled and a notice
  says why, where a small grey hint used to say it beside a map drawn in Mercator anyway; and the settings' example values
  (placeholders) are light italic, not the browser's dark grey that read as a value already entered.
- **Every slot is decoded, and the next slot does not wait for the last (default time budget off, four decoder threads per channel).**
  A channel had one decoder thread with a queue of four, so a decode that outlasted its slot held up the next and a long one dropped
  slots. Each channel now has `Config::decode_lanes` threads (`--lanes`, GUI *Decode threads*; 4) and a slot goes to the one with the
  least waiting. The per-slot time budget (`Config::slot_budget`, `--slot-budget`) is off by default. A lane has its own decoder, so a
  callsign one lane learned does not resolve a `<...>` in another; a channel with averaging uses its first lane only.
- **Rows go out as the decoder finds them, not when the slot is done, with their detail.** The worker calls `decode_with` and hands each
  row on at once: on `qso3_busy.wav` 13-14 of 21 rows are found by 0.36 s and the last at 1.45 s of a 1.8 s decode (the same through a real
  `IqReceiver`). `Decode` carries the row's key, sync score, FEC-corrected errors, `hash_resolved` and Q65's copied-last-Tx, and the
  database gains `msg_key`, `sync`, `hard_errors`, `resolved` (older files are upgraded). A streamed row is resolved against the callsign
  table as it stood when the period began, so the worker says a row again (`update`) when the returned one reads better; the GUI replaces
  it in place (✓). A parallel strategy's repeat is dropped by `delivery` (#592); `Config::slot_budget` stops a slot's decode and counts it in
  `Status::budget_cut_slots` (#593). Not run against a live SpyServer or through the GUI for these.
- **FT8 rows from ~11.8 s into the slot, as WSJT-X shows them (#600).** Each channel tells the receiver its decoder's `prefix_points()` and a
  lane decodes every delivery of a period with `decode_prefix_with`: checkpoint A's rows go out at the first prefix marked early
  (`DecodeDetail::early`, the GUI's ⏱, `--detail`), the rest at the end, where an early row is said again only if it reads better. A period's
  deliveries stay on the lane its first went to. On by default (`Config::early_decode`, `--no-early`, GUI *Early decode*); FT8 at `Fast` depth
  and every other mode decode the whole slot as before. Tried in the window against a SpyServer on 40 m (2026-10-08): most strong stations arrive early.
- **With early decode, `slot_budget` is a period's, not each call's (#626, #627).** FT8's A, B and final calls of one period each got
  a fresh full allowance, so a period could spend three times its share, and each call that ran out added one to
  `Status::budget_cut_slots`. The calls now share the allowance (each gets what the earlier ones left; waiting for audio between
  them is not counted), and a cut period counts once. A's rows stay in the complete set even when B and the end have nothing left
  (checked on `qso3_busy`: 15 of 15).

### CI, tests and docs

- **The skimmer's JTTY waterfall test no longer depends on the cycle phase.** `a_jtty_channel_has_a_waterfall_and_still_decodes` asked for ten
  rows by the time a message completed, but the stand-in server is paced by the wall clock and its script by the UTC cycle, so that took 2 to
  14 s depending on where the stream began and the assertion failed one start in eight (7 rows seen; measured by starting at chosen phases).
  The properties are now `jtty::tests` in virtual time, 0.5 s for all sixteen: the waterfall's copy is the audio the thread read, keeping it
  changes no decode, and rows drawn from it are lit where the message was and dark elsewhere (checked by breaking `feed` three ways). The
  end-to-end test keeps only the loop's wiring, 1.9 s.
- **Q65's caller list is checked against WSJT-X v3.3.0-beta1 itself (#642).** `scripts/jttysim/q65_callers_oracle.f90` links upstream's
  `q65_callers` and `q65_set_list2`; `tests/q65_callers_oracle.rs` replays a 300-operation script (records, expiry, removal, lists) and
  requires the same caller list after every step and the same codewords in every list, including DX as the 51st beside 50 callers. It
  fails on the pre-beta1 code (first difference: the first-inserted caller is evicted instead of the least recently heard).
  `build_jt9_upstream.sh` builds v3.3.0-beta1, and with `JT9_ORACLE_MAX_DRIFT=1` a `jt9` whose Q65 `max_drift` comes from the
  environment (stage 5 is otherwise unreachable from the command line). With the Rx tolerance matched (`jt9 -F 20` and
  `.ftol(20.0)`), 160 drifting Q65-60A files give the same hit on 159: Max Drift stage 5 agrees with upstream up to 48 Hz/min.
- **CI: a `wasm` job decodes one slot of every mode on `wasm32-unknown-unknown`, in Node (#585).** The #583 panic shipped for five releases
  because nothing ran that target. `wasm-smoke/` (outside the workspace) fails naming the modes that panic; against the tree before #584 it
  fails for WSPR and MSK144 and for no other of the 21. Not a timing or SIMD check: that is `bench/wasm`.
- **`decode_snapshot` and the full-slot subtract pin compare within a tolerance, not bit for bit (#579).** The f32 last bits belong to the
  machine: on Apple M5 50 of 53 snapshot fixtures differ in the last bits (`snr_db` up to 0.038 dB) with the same messages in the same order,
  and the subtract pin written for #567 (a hash of 180 000 samples) differed in 27-36 samples between rustfft kernels. Messages, order and `pass` stay exact; the float
  columns have limits 2.5-5 times the measured gaps (`MFSK_SNAPSHOT_STRICT=1` asks for bits again), and the subtract pin is the output itself
  (`embedded-poc/assets/ft8_subtract_full_slot.bin`, 2 per sample, 100 samples). A guard test shows a changed message still fails.
  Data and `scripts/libm_probe.rs`: `docs/notes/snapshot_platform/`.
- **docs: `LIBRARY.md` §2.3 (and `.ja.md`) is "Early decode and the compute budget".** Why both follow from "a period's state lives in the
  `Decoder`" and "the library reads no clock"; upstream's 41/47/50 checkpoints and what `SicEarly` keeps of them; `decode` against
  `decode_prefix`; why FT4 has none; the budget (the candidate as unit, what is deliberately not cut, which `BudgetReport` fields each mode
  fills; how it follows the checkpoints, #589). §2.1 and §2.4 say `RowDetail`'s `Option`s and `delivery`.

## 0.13.1 — the LPF subtract no longer panics on a slot shorter than the frame (#567, #574), 48 kHz input is low-passed before 12 kHz (#576), hashed calls keep /P and /R (#570), the licence is GPL-3.0-only (#569), the skimmer installers say how to install them

- **48 kHz (and any rate above 12 kHz) is low-passed before going down to 12 kHz, as WSJT-X does (#576).**
  `engine::dsp::resample` had no anti-alias filter: 48 kHz → 12 kHz was a plain decimation by 4, so 6–24 kHz folded into
  the band. It runs in the C ABI's decode and stream entry points at any rate other than 12 kHz, on the boards' USB audio
  and microphone, and in wasm capture. On white noise over the full 48 kHz band FT8's 50 % crossing was -15.3 dB; with
  `fil4` (`lib/fil4.f90`, the 49-tap FIR `Detector.cpp` applies) it is -21.1 dB, against -21.0 dB for the same signal and
  noise generated at 12 kHz. `fil4` is now the filter at 48 kHz, in the batch functions and in `LinearResamplerI16To12k`
  (Q15, FPU-free); other rates get a Kaiser low-pass to its specification. Tests: a 9 kHz tone comes out ≥ 30 dB down
  (it was 0 dB), and FT8 at -19 dB in full-band 48 kHz noise decodes (0 of 4 before).

- **docs: the 0.13 decoder by use case, why it is shaped like WSJT-X's, and how to move to it.** `LIBRARY.md` §1.1 (and
  `.ja.md`) is new: a decoder is something you keep, shown on the common jobs (a live receiver numbering its periods, a
  QSO's context, one decoder per channel, a deadline, streaming rows, a non-GUI search, other sample rates), each with
  the reason for its shape and each a doctest. `LIBRARY.md` §1.2 (and
  `.ja.md`) gathers the migration from 0.12 that sat at the end of §6, adds what changes in results with the
  defaults (FT8 search, band, AP, JT9/JT65, WSPR, with the measurements behind them) and what replaces each removed option;
  §2 opens with the six rules the API follows. `DESIGN_RATIONALE.md` §6 records what went wrong in 0.12, the
  decision, what it cost and what it bought. `ft4::subtract_signal_lpf`'s doc no longer claims a short buffer is
  a no-op (#574).

- **`engine::dsp::subtract`: the LPF subtract no longer panics on a slot buffer shorter than the frame (#567).** `apply_at_offset` sized
  the FFT from `audio.len()`, where `subtractft8.f90` fixes `NFFT = NMAX = 15*12000` (v3.2.0-rc1, lines 10-11) — always at least
  `NFRAME = 1920*79`, so a short `dd` is simply zero-filled. On a shorter buffer `cfilt` was shorter than `nframe`, and both the
  camp build (`cfilt[i]`) and the end correction (`cfilt[nframe-1-d]`) indexed past its end. With end correction on — the FT8
  subtract's own setting — that panicked at *any* `dt`, not only a negative one, and on the `sic_early`/`__staged_sic` paths it
  takes the decode worker thread down with it. `nfft` is now `audio.len().max(nframe)`, which restores the reference's invariant;
  the existing clamps and the end correction are then correct as written. A full slot is unaffected (`audio.len() > nframe`
  already, so `nfft` does not move) and its output stays bit-identical. `residual_band_power` takes the same `nfft` so the
  `sqf` trial search scores on the grid the subtract actually used. New tests: `subtract_short_buffer`, and
  `decoder_input_length`, which runs every mode through `AnyDecoder` on a real frame cut to 0 … 1.5 periods from either end
  (on noise alone no decode happens and nothing after it runs, so it did not catch #567 until it carried a signal).
  **Affected:** every release since the FFT subtract (`28a38e7a`, 2026-07-20): 0.8.0 through 0.13.0. On an
  older version, pad the slot with zeros to its full length (FT8 180 000 samples, FT4 90 000) before decoding: the FFT is then
  at least a frame long and the panic cannot occur. Older versions are not yanked; 0.13.1 is the fixed one.

- **A hashed callsign with `/P` or `/R` now resolves, as in WSJT-X (#570).** `CallsignHashTable::insert` stripped the suffix and the
  type 1/2 learner never read the `/R` / `/P` flag, so `<W1XYZ/P>` stayed `<...>` where WSJT-X v3.2.0-rc1 prints it (32 of 336 outcomes on the
  WS fixtures of #568; no wrong call, a missing resolution). Both now follow `save_hash_call` and the unpacker (`packjt77.f90:356-390`,
  `:1755-1765`). New test `ws_77bit_extension` checks 168 messages in two table states against rc1: 336 of 336.

- **License identifier corrected to `GPL-3.0-only` (#569).** Up to 0.13.0 the crate, its README and the source headers said
  `GPL-3.0-or-later, matching upstream`. WSJT-X states "version 3" (README, About dialog, `COPYING`) and does not add "or later";
  this crate contains code derived from it. `license` in the `Cargo.toml` files, the `SPDX-License-Identifier` headers
  (Rust, Swift, Kotlin, C, C++), the READMEs, `CONTRIBUTING.md` and the reference docs now say GPL-3.0-only. Already published
  versions keep their crates.io metadata; this takes effect with the next release. Whether files that are original work
  (not derived from WSJT-X) should return to "or later" is left for a per-file decision.

- **skimmer: the install guide and the release page say how to install it.** `apps/skimmer/README.md` now lists what the
  app needs (a reachable SpyServer, a right clock), the Windows SmartScreen step, macOS 14 versus 15+ (*Open Anyway*,
  `xattr`), first run, what to check when it will not connect, and updating. The GitHub release body carries the file
  table with the release's version, the same install steps and a link to the README; `attach-skimmer` needs `verify-tag`
  for that version. Not run on a tag yet.

## 0.13.0 — one persistent `Decoder<P>` per mode on the WSJT-X parameter block (breaking), `Depth` as `ndepth`, QSO-context AP for FT8/FT4/FST4, a pull `IqReceiver` with a clock that follows the host, one C decoder handle for every mode (breaking), per-channel decoder threads in the skimmer

### 0.13.0 — decode API redesigned on the WSJT-X decoder model (breaking)

- **`Decoder<P>`: one persistent decoder per mode, as `jt9` runs.** It owns what upstream keeps between periods (the
  callsign hash table, FT8 a7 rows, Q65 period averages, WSPR's call table) and is driven per period by a
  `DecodeParams` that is `jt9com`'s block (`nfa/nfb`, `nfqso`, `nftx`, `ntol`, `ndepth`, `mycall/mygrid`,
  `hiscall/hisgrid`, `nQSOProgress`, `ncontest`, AP, `emedelay`). Library-only options are a typed `Extras` per mode, so an
  option a mode lacks does not compile (`AnyDecoder` returns `Unsupported`). Rows stream through `on_row` as found.
  Removed: every per-family `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` as public API (the wide-band ones are
  crate-private; `SniperRequest` stays for the WSPR boards), `known()`, `previous_cycle()`, `hash_table(Arc)`,
  WSPR `table()`, `wsjtx_depth`, `iq::IqMode` (now `registry::Mode`). Migration table: `docs/reference/LIBRARY.md`.
- **`Depth` decides the search, as `ndepth` does (breaking behaviour).** Default is Deep and the defaults follow the
  WSJT-X GUI (FT8/JT65 AP off, band 200–4000 Hz, FST4 600–1400 Hz). Ported per mode from v3.2.0-rc1: FT8 syncmin/passes/OSD and
  nfqso-first candidate order, FT4 passes and OSD, FST4 OSD without the i0±1 jitter at Fast, Q65 maxiters 40/60/100 and
  `q65_loops` windows, JT65 2/2/4 passes with `subtract65` and nvec 100/1000/1000, JT9 Fano limit and the nqd narrow
  pass, WSPR `-qB` / `-C 500 -o 4` / `-d` with `unpk_`'s Type-3 sanity checks. Tests: `tests/decoder_depth.rs`.
- **QSO-context AP for FT8, FT4 and FST4** from upstream's `naptypes` tables, derived from `station` + `qso` + `ap`
  (`ApHint` stays as the library's "hunt one DX"). A weak-reply sweep on FT8: 0 of 30 decoded with AP off, 20 of 30 on.
- **WSPR now reproduces `wsprd`, signal for signal, on the WSJT-X golden.** Compared against an instrumented
  `wsprd` v3.2.0-rc1: SNR within 0.02 dB (it was 0.05-0.26 dB, and one signal rounded to a different integer), DT
  equal to 3 decimals, frequency to 0.1 Hz, and G8VDQ (-23 dB), which this crate had needed a larger Fano budget for,
  decodes at the default one. Causes, all ported literally from `wsprd.c`: the noise floor is `tmpsort[122]` not 123;
  the sync window constant is 0.006147931; the front pad is a whole number of the 128-sample coarse strides, so every
  lag tried is `wsprd`'s; the whole recording is read; the coarse drift bin truncates `ifr + offset`, not the offset
  (every candidate came out with drift -1); candidates run strongest SNR first; and each decode is subtracted at once,
  with `wsprd`'s float-accumulated reference phase, instead of decoding a pass in parallel and subtracting at its end.
  Reported frequency is the centre of the four tones, and the power prints two wide (` 3`, not `3`). The AWGN sweep:
  50 % crossing -31.40 dB (was -31.33), 1 phantom in 260 trials as before; 20 ms/file at low SNR (was 52). Detail in
  `docs/notes/WSPR_UPSTREAM.md`.
- **JT65 message averaging (`ndepth & 16`), as `avg65`.** A candidate the single period fails on is kept (period, DT,
  frequency, 63 × 64 symbol powers; up to 64 periods per decoder) and summed with the saved periods of the same parity
  within 0.2 s of DT and `tol_hz` of frequency; two or more are decoded as one (the Chase search). Needs
  `SlotInput::period`. On noise where no single period decodes, the sixth summed one does. A JT65 frame also came back
  once per candidate around it (8 rows): the repeat test compared an unpadded start with a padded one since #283.
  Not ported: the JT65B/C smoothing loop; `deep_search` (`hint65`) is carried and not read.
- **JT9 and JT65 drop the all-zero codeword** (`000AAA 000AAA RA90`, `jt9fano.f90:88`, `decode65b.f90:27`): a noise-free JT9
  period returned the true message plus three of these.
- **`IqReceiver` is a pull model on a clock that follows the host.** `push_*` yields owned `CompletedSlot`s; the caller
  owns one decoder per channel, so per-channel options and hash tables follow. `SlotGrid`/`SampleClock`/`SlotCutter`
  (integer arithmetic, no atomics) slew a drifting clock at up to 400 ppm and step past 1 s, starting each slot on its own
  boundary with 0.2 s of overlap: a 24 h simulation at +13 ppm loses no slot (fixed-length slots slid off the UTC grid and lost one
  at about 21 h). `retune()` reports which channels paused or resumed.
- **C ABI 3: one `mfsk_decoder_*` handle for every mode (breaking).** `MfskParams`/`MfskExtras` are size-versioned and
  initialised by `mfsk_params_init`/`mfsk_extras_init`; rows stream through a callback; an option a mode lacks is
  `MFSK_UNSUPPORTED`; `MfskStream` cuts slots on the UTC grid. The session handle and every `mfsk_*_decode*` function are
  gone. cpp_smoke, Kotlin and the Swift package were ported and run; on macOS the Swift suite passes 98 of 98, after
  one test was updated for `mfsk_stream_clear` now dropping the slot being cut as well. Old→new table:
  `docs/reference/BINDINGS.md`.
- **Embedded:** the boards call the stage functions, which did not change; `cargo +esp check` passes on all four boards.
  New CI/pre-push rows build `Decoder`/`AnyDecoder` under `alloc` + `fft-extern` and the union of the board features.
  `Decoder::new` allocates nothing.
- **Skimmer sample app:** one decoder thread per channel behind a bounded queue (a slow decode no longer stalls the socket
  reader or the clock), per-channel band / DX call / depth (CLI `:band=LO-HI:dx=CALL:depth=..`, GUI ⚙ dialog, applied to a
  running skimmer at the channel's next slot), and a `STATUS.log` of push, decode, queue and drop figures for long runs (rolled to `STATUS.log.1` past 5 MB; a server failing the same way repeatedly is one line and a count).

- **0.12.0 is yanked (2026-10-03).** Its `iq::IqReceiver` lost every other slot with a real-clock time anchor and
  never resolved hashed (`<...>`) callsigns, so it was unusable for live reception; the fix is an API redesign, which
  ships as 0.13.0 rather than a 0.12.x patch. A `Cargo.lock` already holding 0.12.0 keeps working; new resolutions of
  `mfsk-core = "0.12"` fail until 0.13.0, and FT8/FT4/WAV users can stay on 0.11.x meanwhile.
- **`IqReceiver` decodes every slot when its time anchor is off the 12 kHz grid (#534).** A slot starts at its
  boundary rounded up to a sample, so the sample after a slot can lie just past the next boundary, and
  `next_boundary` skipped to the boundary after that. With an anchor off the grid, which a wall-clock timestamp
  nearly always is, the receiver decoded every other slot and could also lose the first one. A stream stamped from the
  wall clock decoded 2 slots of 4; it now decodes 4 of 4, 21 FT8 messages each. Every test had
  anchored on a whole second, which is on the grid. New tests cover arbitrary anchors for every slot period and two
  back-to-back slots anchored 40 µs off.
- **Sample app: a SpyServer skimmer, `apps/skimmer/`.** `skimmer-core` drives `IqReceiver` from a SpyServer
  (protocol as SDR++ speaks it) and reports connection, stream, decode and health as events; the `skimmer` CLI prints
  them and appends WSJT-X `ALL.TXT`-style lines; a Tauri + Svelte GUI (`apps/skimmer/gui`, built on Windows) shows
  channels with slot-progress pies and band presets from WSJT-X's frequency list, and decodes banded by slot. It shares the radio with SDR#: it never tunes the
  device unless `--tune`, sets only its own DDC centre, gives control back, and pauses channels outside the band. The
  socket is read on its own thread: the first decode (400-650 ms) had made the server drop IQ on Windows in 3 of 3
  runs, 0 after. Against an Airspy HF+ at 7041 kHz beside SDR#: 19-26 JA FT8 decodes per slot, WSL and Windows.
- **FT8's T1 sensitivity task now asks for what `jt9 -8 -d 3` does.** With no callsign known the AP tries only the CQ
  hypothesis; the 0.12 sweep asked for it with `ApHint::with_call1("CQ")`, which a `Decoder` still accepts and which
  reproduces the old numbers bit for bit. The CQ-only derivation is 0.2-0.5 dB behind that in fading channels
  (ccir_moderate -20.00 against -20.50 dB) and level with WSJT-X in paired counts (5 up-only against 3 crate-only, p =
  0.73); JT65, JT9 and Q65 crossings are unchanged and WSPR's moved -0.07 dB. Against WSJT-X no group is behind except
  JT65's known gap (15 upstream-only against 0, as in 0.12). AP is off by default.
- **The skimmer keeps every decode in SQLite and answers questions about them.** `skimmer.db` (WAL, indexed on time,
  band and call; the GUI no longer writes an ALL.TXT, the CLI's `--log` still does) records each decode with its sender, locator, CQ kind and the server that
  heard it. An Analysis view runs one query (UTC period, regexes on call, locator and text, band, mode, server, SNR,
  distance, bearing sector, CQ kind) over a great-circle or Mercator map that animates in a chosen window, the
  openings of the bands, a per-station hours-on-the-air map with a small map of the great circle, and a Database
  tab (size, clearing out old decodes, compacting, a CSV of the query, a copy of the file).
- **The skimmer runs up to four SpyServers, each in its own place, and can rotate bands.** A server has its own
  locator, network delay, gain and control; bearings are read from the server that heard the signal. A rotation gives
  each band of a server's channels a turn (at least five minutes, from the first band on Connect or on the UTC
  clock) and each band can be given the UTC hours it takes part in (day and night follow the server's local time);
  a band's decoder and waterfall history wait for its next turn. The rotation can be held on a band, or moved
  to the next or previous band at once (its whole turn starts then). Sixteen busy-band FT8 decoders ending together take
  about 3 s on 8 threads (`tests/load.rs`).
- **The skimmer's screen.** A server's dot switches it on or off; beside its channels the rotation shows the band and the
  time left, holds when pressed, and moves to the previous or next band at once (its whole turn starts then). Settings are
  in tabs. The map has a grey line (day, the band where the Sun is within 6 degrees of the horizon, night; sunrise and
  sunset per server are in its Settings), band-coloured paths when bands are chosen, and an Animate that can skip
  windows with no station. The installers can be built by hand (`workflow_dispatch`).
  A band's hours can be "day" or "night" following the calendar: from an hour before sunrise to an hour after sunset
  (or the night between), recomputed for each day at the server, with no moment of renewal.
- **The skimmer's clock can follow NTP, and the delay to the server can be entered.** The PC clock is not touched;
  its measured offset (best of five SNTP queries, every ten minutes) is added, and a fixed network delay is taken off
  the arrival times, which the minimum-delay anchor cannot remove.
- **An XCFramework for iOS apps.** `bindings/swift/scripts/build-xcframework.sh` builds `libmfsk` for the iOS device
  (`aarch64-apple-ios`) and the Apple-silicon simulator (`aarch64-apple-ios-sim`) with the `mobile` features, and
  packages both as `target/xcframework/Mfsk.xcframework`. Each slice carries `mfsk.h` and a module map, so Swift code
  imports it as `CMfsk`, the same name the SwiftPM package uses. Before it reports success, the script compiles and
  links a small Swift program against each slice. It is not run in CI, and `Package.swift` still links by `-L` rather
  than through the XCFramework.
- **Skimmer installers for Windows and macOS, attached to each release.** `skimmer-installers.yml` builds the GUI into
  `mfsk-skimmer-vX.Y.Z-windows-x64-setup.exe` (NSIS) and `mfsk-skimmer-vX.Y.Z-macos-arm64.dmg` (Apple silicon). The
  release workflow attaches both after the crates.io publish without waiting on them, so a GUI build failure does not
  hold the library back. Pull requests that touch `apps/skimmer/` run the same build. The macOS app is signed ad hoc
  and not notarized. It carries `NSLocalNetworkUsageDescription`: without that entry, or with only the linker's
  signature, macOS refused the SpyServer connection with `No route to host` and never asked for permission.
- **The skimmer reconnects when the server goes quiet.** A SpyServer that dropped the client while a Mac slept left
  the socket ESTABLISHED, and the skimmer, which only writes when it changes a setting, sat "connected" with no data
  and never reconnected. `Conn::read` now takes 15 s with nothing from the server as a dropped connection, and the
  usual retry follows. With streaming off, while no channel fits the band, it pings every 5 s so the PONGs keep an idle
  connection from looking dead. The GUI also stops posting a notice for every lost IQ message: the health line counts
  them and the health log still records each one.
- **WSPR: `Decoded::freq_hz` and the text report now match `wsprd`'s own numbers.** `freq_hz` carried tone 0, 2.197 Hz
  below what `wsprd` reports — the centre of the four tones (`wsprd.c:1496`, `1500 + f1`) — so a caller comparing
  against `wsprd`'s output saw every WSPR frequency low by 1.5 tone spacings (ND6P: 1444.04 Hz here, 1446 Hz in
  `wsprd` on the WSJT-X golden). The power field also printed without `wsprd`'s `%2d` padding, dropping the leading
  space on single-digit dBm values (`wsprd_utils.c:353,364,398`). The Type 1/2/3 power arithmetic itself was already
  correct; only `freq_hz` and the two display fixes change. Snapshot fixtures move in their frequency bits only (36
  rows for 36).
- **Skimmer GUI: per-channel decode counts no longer disagree with the table, and the window keeps updating when
  covered or minimised.** Counts were incremented on arrival while the table itself lagged a `requestAnimationFrame`
  queue that macOS/browsers starve for a covered or minimised window — so a count could read higher than the rows the
  table ever showed, and follow-scroll (keyed on the row count, which stops changing once the 5000-row cap is hit)
  could stop following. Counts now come from the same batch the table applies, on a 100 ms timer instead of
  `requestAnimationFrame`, and the health line gains a `window got N` figure — the raw count of decodes the window
  received, to compare against what the table shows.
- **`scripts/release-status.sh` reports a yanked published version.** A tagged version is not necessarily a live one:
  this is exactly what happened to 0.12.0. The script now queries crates.io for the tag's published version and
  prints `YANKED`, its `yank_message` if one was set (else a pointer at this CHANGELOG's own "is yanked" entry), or a
  neutral "could not check" line when crates.io is unreachable — never a false `ok`.

Older releases are archived out of this file to keep it skimmable. The rule:
this file holds the unreleased section and the two latest minor series, and
each new minor's release PR moves the oldest series out.
0.12.0 in
[`docs/historical/CHANGELOG-0.12.md`](docs/historical/CHANGELOG-0.12.md),
0.11.0 in
[`docs/historical/CHANGELOG-0.11.md`](docs/historical/CHANGELOG-0.11.md),
0.8.0 – 0.10.1 in
[`docs/historical/CHANGELOG-0.8-0.10.md`](docs/historical/CHANGELOG-0.8-0.10.md),
0.6.0 – 0.7.4 in
[`docs/historical/CHANGELOG-0.6-0.7.md`](docs/historical/CHANGELOG-0.6-0.7.md),
0.1.0 – 0.5.12 in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
