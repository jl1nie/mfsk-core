# Changelog

## 0.16.0 — every transmit waveform is WSJT-X's own generator's, and one value, `tx_buffer_start`, says where to place it in every mode (breaking for FT4 hosts); JT9 picks its candidates as `jt9` does and JT9/JT65 report DT from WSJT-X's 1.0 s; FT4 subtracts `gen_ft4wave`'s frame; WSJT-X v3.3.0-beta4 → beta6 ported (#726)

WSJT-X v3.3.0-beta4 → beta6, ported (#726): every transmit waveform now follows WSJT-X's own generator, and JT9 decodes as `jt9`
does. Measurements behind each entry: [`docs/notes/WSJTX_BETA6_PORT.md`](docs/notes/WSJTX_BETA6_PORT.md).

### Changed — what a host must change (this is why 0.16 is a minor)

- **Place every transmission at `tx_buffer_start`** (#743): the slot sample the synthesised buffer's first sample goes to, so the
  frame lands at DT = 0 on a WSJT-X receiver. FT4 5424, FT8 6000, the 1 s modes 12 000 (`Encoded::tx_buffer_start(rate)`,
  `ModeMeta::tx_buffer_start_samples_12k`, appended to C `MfskModeInfo`, Swift and Kotlin `txBufferStartSamples12k`).
  `tx_start_offset_s` is the DT reference only. **An FT4 host that placed by it now sends 48 ms late**: FT4's frame is 105 symbols,
  60 480 samples, a ramp symbol before the tones (`gen_ft4wave`). WSJT-X itself starts FT4 at 0.300 s, ~150 ms earlier (marked).
- **JT9 and JT65 `tx_start_offset_s` is 1.0 s, not 0.0** (#742), where WSJT-X starts them (`Modulator.cpp:163`); `Decoder`'s DT was 1.0 s
  above `jt9`'s, and a host placing by it keyed up 1 s early.
- **Types:** `Waveform` is `#[non_exhaustive]` with `Ft4` and `Fst4`; `GfskParams` gains `frame_ramp: bool` (a struct literal needs it);
  `Jt9Extras::search` supplies only the band. `mfsk_synth_lead_len` and the other lead helpers were added and removed within this cycle.

### Fixed — transmit

- **FT4, FST4, FT8 match WSJT-X's generators** (#729, #736): FT4 is `gen_ft4wave`'s 105-symbol frame, FST4 `gen_fst4wave`'s (no
  extended first and last symbol, a half-symbol ramp), and the phase is a `u64` turn accumulator (no `f64` per sample, which the
  LX7 cannot afford). Against beta6's own generators on a fixed tone list: FT8 -68 → -95 dB, FT4 -21 → -95 dB, FST4-15 -16 → -88 dB
  (new tier A `tests/gen_wave_vs_wsjtx.rs`). The CoreS3's live `GfskStream` is unchanged.
- **WSPR, JT9, JT65, Q65 match `Modulator::modulate`** (#737): `sin` of a phase advanced first, the same accumulator, per-symbol
  frequency in `f64`. Was `cos` before the advance in `f32`: -35 dB on a 110 s WSPR frame. Within 2e-4 of an `f64` reference.

### Fixed — receive

- **JT9 picks its candidates as `jt9` does** (#742): `sync9`'s 0.732 Hz bins walked upwards, the first that passes `ccflim`/`red2lim`
  decoded (`jt9::sync9`), not this crate's own search in score order cut at 32. On `130418_1742.wav` the same rows as beta6's `jt9`
  at `-d 1`/`-d 3`, 1050-1550 and 200-4000 Hz, with and without an Rx window (DT within 0.1 s, SNR 0.5 dB); two DTs were 0.2 s off.
  The crate's own `peakdt9` gate is gone. Tier C now decodes as its `jt9 -d 3` side: -26.58 dB (baseline -26.64), `jt9` only 3 of 300.
- **FT4 subtracts `gen_ft4wave`'s frame** (#739, #734), as `subtractft4.f90` does: the residue of a clean signal goes from -20 dB to
  -29 dB. Three marginal messages leave `ft4_eq_local` (Rust-only equaliser), now guarded by a recall floor of 12 of `jt9`'s 14.
- **Q65 decodes a noiseless frame** (#740, #735; **marked divergence**): the `s3lim` clip base is floored at the range `int16` audio
  carries. On float input with no noise the 40th percentile was f32 rounding and the clip flattened the tones' sidelobes; WSJT-X's
  `int16` input cannot reach that. The floor never engages on the vendored recordings.
- **JTTY refuses a misaligned view of a decoded frame** (#728; upstream PR #702, `is_frame_alias`): upstream's own case went from 4
  false frames in 24 trials to 0, with 24/24 still decoded in full.
- **WSPR refuses an out-of-range type-2 suffix and type 63** (#727; upstream `ffa884606`), as beta6's `wsprd` does: `unpackpfx` no
  longer wraps. 1 488 rows of the 26 046-row `unpk_` fixture, rebuilt from beta6, change.

### Performance

- **GFSK pulse and NCO `sin` are beta6's polynomials** (#738): no `exp`, division or libm `sin` per sample; within about 1 dB of the
  `erf`/`sinf` form against beta6's generators (FT8 -94 dB, FT4 -96 dB). FT8 `synthesize` on x86_64: 704 → 535 µs. Not measured on
  the LX7.

### Docs

- **README shortened, 823 → ~230 lines, and its Japanese twin now mirrors it** (was a 58-line summary). It leads with what an operator
  can run and states each fact once. Corrected on the way: FST4W was missing from the protocol and feature tables; FST4 has seven
  periods; the WSJT-X speed column reads `UPSTREAM_EVALUATION.md`'s 2026-10 ratios; the Swift binding is built and tested by CI; the
  design diagram's `core` box is `engine`. Design text moved to `DESIGN_RATIONALE.md` §7.
- **The beta6 port is recorded in `docs/notes/WSJTX_BETA6_PORT.md`**, and the reference docs follow it: TX placement in `LIBRARY`,
  `BINDINGS` and `EMBEDDED` (FT4's citations at beta6's lines), JT9's scan and `SearchTuning` in `LIBRARY` and `CANDIDATES.md`.

## 0.15.1 — FT8 follows `jt9` on real recordings: 401 of `jt9`'s 401 decodes on ft8_lib's 22 recordings (was 383), reported and subtracted where WSJT-X does (`xdt`), `sync8.f90`'s lag grid and candidate dedupe, blind-CQ AP on every candidate, `i3=4` 11-character calls, `ft8b`'s last bit-metric group (#721)

A drop-in patch for 0.15.0: no breaking API change (it adds `engine::LlrTail`, defaulted on the trait). FT8 decodes more on crowded real bands, with no false decode added on the labelled corpora (tier C: no channel worse than the 0.15.0 baseline). Reported FT8 `dt_sec` moves by -5 ms and a few `sync_score` / SNR / hard-error values move; `sic_rounds` and `decode_block` stay the simplified strategies (#722).

- **FT8's multi-symbol bit metrics follow `ft8b.f90` at the end of each half** (new `engine::LlrTail`, defaulted `SyncFrontEnd::LLR_TAIL`; FT8 sets `ContinueIntoNext`, every other protocol keeps the overlap patch it had). `do k=1,29,nsym` runs the last group on into the next (Costas) symbol; the shared sweep re-anchored it on the last data symbols instead (made for FT4, #18). Bits 79-87 and 166-174 of `llrb`/`llrc`/`llre` differed from `jt9`'s in a few signs on identical audio (all-sign agreement with jt9's traced LLRs on 638 candidates: 45 % / 40 % / 66 % before, 94 % / 94 % / 95 % after), enough to lose OSD decodes near 29 hard errors. On ft8_lib's 22 recordings `mfsk-core` now decodes 386 of `jt9 -8 -d 3`'s 401 messages, was 383; FT4/FST4/FST4W unchanged. Also reaches the a7 list decoder (`ft8_a7.f90`/`ft8c.f90` use the same loop).
- **FT8 reports a decode, and subtracts it, where WSJT-X does** (`refine_fine::XDT_OFFSET_S`). `ft8b.f90` / `ft8_a7.f90` return `xdt = (ibest-1)*dt2`, one 200 Hz sample before the start the correlation found, and `subtractft8` starts there; this crate used `ibest`, 5 ms late. A clean +38 dB signal was subtracted to -15.6 dB here and -55.8 dB by `jt9`, now -55.9 dB. Reported `dt_sec` moves by -5 ms (bias against `jt9` +5.1 -> +0.3 ms). On ft8_lib's 22 recordings the default strategy goes 386 -> 390 of `jt9`'s 401 with fewer outputs `jt9` lacks (11 -> 8), 855 -> 861 of 902 crowded synthetic signals (`jt9`: 860). **`sic_rounds(n)` and `decode_block` lose a little in a crowded band** (`qso3_busy.wav`: `sic_rounds(3)` 20 -> 19 messages, the `decode_block` full-parity floor 20 -> 19; 902 synthetic: 838 -> 828): they have no re-subtraction with a refined DT between rounds, which `sic_early` does as `jt9` does, and the old 5 ms error had been hiding it. They are the simplified strategies; `sic_early` is the faithful one (21 of 21 on `qso3_busy.wav`).
- **FT8 closes the remaining gaps to `jt9` on real recordings, plus one in the message filter** (#721). (1) A re-decoded message is subtracted again before it is dropped as a duplicate, as `ft8b.f90:463-467` subtracts every valid decode ahead of `ft8_decode.f90`'s dupe check: a signal left by a shallower first subtraction, or a splattering signal's copies at other frequencies, no longer buries its neighbours. (2) `coarse_sync`'s lag grid is `sync8.f90`'s (`jstrt` truncated to 12, 1-based; `JZ` 62): it was two steps (80 ms) off on identical `sync2d` values, so coarse DT, the `±MLAG` window and `ft8b`'s ±10-sample start all differed. The fixed-point `nstep-half` grid (80 ms steps, no upstream counterpart) keeps its rounded origin: the same formula there costs the board's driver 5 of 7 floor decodes. (3) The blind-CQ AP pass runs on every ladder candidate, as iaptype 1 does upstream; the `nsync >= 12` cost gate is gone. (4) An `i3=4` call may be up to 11 characters (`c58`), so `<...> DA0FONTANE` is reported as `jt9` reports it. (5) `coarse_sync` dedupes on the integer-lag DT `(lag-0.5)*tstep`, as `sync8.f90:143` does, not on the parabolically interpolated one: a weak candidate 3 Hz from a +22 dB signal at exactly the 0.04 s window (`jt9` DT 0.94 vs 0.90; 0.931 here) was dropped, and with it the second and third decode and subtraction of that signal, so `CQ UB3AQS KO85` stayed buried. Only for candidate lists of 60 or more (`FAITHFUL_DEDUPE_MIN_CAND`): at the embedded cap of 30 the extra near-duplicates pushed `OH3NIV ZS6S -03` out of `qso1.wav` (4 -> 3 of 5 in `ft8_decode_block_real_qso`), so a lean list keeps the interpolated-DT dedupe. (6) `decode_block_multipass` (host only) also subtracts a re-found message before dropping it; the `decode_block` full-parity floor is back at 20.
  Measured: ft8_lib's 22 recordings 390 -> **401 of `jt9`'s 401** (default `sic_early`; 7 outputs `jt9` lacks, 6 attested elsewhere in the session, `SP7NHS YC2FCE -15` undecided), `sic_rounds(3)` 381 -> 387; `qso3_busy.wav` `sic_rounds(3)` back to 20 messages; the labelled busy (2900 signals) and dense (4102) corpora: no false decode in any set, none in the 200 noise-only files, recall within +3 / -2 signals of before (dense `real`, 902 signals: identical). Reported SNR on the dense set, 3 644 decodes both make: mean difference to `jt9` +0.02 dB, 90 % identical, all but 4 within 1 dB (both about -0.4 dB against the injected SNR). Decode time about +17 % (1.29 s per file against `jt9`'s 3.57 s, single thread); random-payload survival through the codec verdict +1.3 %. The embedded (non-`fft-rustfft`) `decode_block` never subtracts; the cold-start DT bootstrap moved with the grid (22 recordings: median error 0.140 -> 0.080 s; one fixture's budget 130 -> 180 ms). Detail: `docs/notes/FT8_REAL_VS_JT9.md`.
- **Tests:** `ft8_real_recordings_vs_jt9` also holds five more vendored ft8_lib recordings (`191111_110615/110645`, `websdr_test2/5/11`), one per #721 fix; each fails without its fix. `ft8_noise_slots_decode_nothing` (tier B): 24 noise slots per condition, no hint and a CQ hint, `max_extra: 0`, for the blind-CQ pass that now runs on every candidate. `ft8::llr` checks all five LLR sets against a direct port of `ft8b.f90`; `sic_early_eq_mode_reaches_sic_engine` no longer pins one noise seed (its "reliably" was 1 draw of 12). 18 FT8 snapshot fixtures moved again, no message added or lost (`decode_snapshot.rs` header lists the rows). Measurement and what is still different: `docs/notes/FT8_REAL_VS_JT9.md`.

### Skimmer

- **Skimmer 1.0.2 is the skimmer on `mfsk-core` 0.15.1** (tag `skimmer-v1.0.2`; no skimmer code change, the four version files and `package.json` agree). It takes the library by path, so its FT8 channels get the #721 changes above: the default `sic_early` strategy decodes 401 of `jt9`'s 401 messages on ft8_lib's 22 recordings (383 under 0.15.0), and a reported FT8 `dt` moves by -5 ms. The skimmer uses neither `sic_rounds` nor `decode_block`, so the loss the entry above names for those does not reach it. Checked: `skimmer-core` tests (`--release`), `cargo check` of the workspace, the tag run's installers. Not checked: a live SpyServer or `rtl_tcp` session on this build.

## 0.15.0 — the public surface narrowed to what applications use: `engine::pipeline` crate-private and no public path behind `internal-testing` (breaking), `#[non_exhaustive]` on the growing option/result/error types (breaking), registry capability bits that describe the API (breaking), candidates and decode method readable from a result, `Row::new` / `SlotResult::new`, a parameter block WSJT-X's GUI cannot send no longer crashes a decoder, `RR73` and `/R` `/P` AP follow WSJT-X 3.3 beta

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

- **A block WSJT-X's GUI cannot send no longer crashes or hangs a decoder** (#717). `Decoder` hands its mode the block normalised: band
  edges clamped into 0-6000 Hz (a NaN edge to that end), a non-finite Rx / Tx frequency or a non-finite or negative tolerance unset;
  `params()` still shows what was set. Before, on one synthetic signal per mode, an infinite band top panicked FT8 / FST4 (debug) and Q65
  (release) and did not return from JT9 / JT65; a NaN Rx frequency or tolerance lost the FST4W decode. Every value the GUI can send passes
  through bit-identically: tier A+B unchanged. `tests/decoder_params_normalised.rs`.

### Skimmer

- An rtl_tcp connection says what it waits for: `Event::Preparing { stage, up_to_s }` at the start of "waiting for the server to
  apply the settings" (up to 20 s) and "measuring the sample rate" (35 s), shown in the GUI's server line and printed by the CLI;
  the ~56 s between Connected and Decoding were silent.

Older releases are archived out of this file to keep it skimmable. The rule:
this file holds the unreleased section and the two latest minor series, and
each new minor's release PR moves the oldest series out.
0.14.0 in
[`docs/historical/CHANGELOG-0.14.md`](docs/historical/CHANGELOG-0.14.md),
0.13.0 – 0.13.1 in
[`docs/historical/CHANGELOG-0.13.md`](docs/historical/CHANGELOG-0.13.md),
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
