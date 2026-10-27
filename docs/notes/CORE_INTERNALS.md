# `mfsk-core` internals — the map before you edit

Moved out of the repo-root `CLAUDE.md` (2026-10-12). Read it before editing the decoders, the `Protocol` traits or the sniper / AP code.

## `mfsk-core` internals — the map before you edit

**FST4W** (#649; WSJT-X's WSPR-style beacon, 120/300/900/1800 s) is FST4's modulation
and front end with LDPC(240,74), a 50-bit message and a Keff 66 → Keff 50 ladder
(`fst4w::decode`); the Keff-50 word has no CRC and is accepted only through a
known-call list that is state between candidates, so candidates run one at a
time. Its oracles run upstream: `scripts/osd-upstream/run.sh 74` (FEC),
`scripts/fst4w/build_oracle.sh` (`genfst4`), `scripts/fst4w/gen_decode_vectors.sh`
and `gen_fst4_long_vectors.sh` (`jt9`). Build them from `git archive` of the tag,
never the working tree.

A protocol is a **zero-sized type** implementing `ModulationParams`
(tones, symbol rate, Gray map, GFSK shaping), `FrameLayout` (symbol
counts, slot length, sync blocks), and `Protocol` (which binds
`type Fec` and `type Msg`). Everything generic is monomorphised per `P`,
so the abstraction costs compile time and nothing else. Adding a protocol
is a trait impl plus a registry line — FST4-60A landed without touching
shared code. `CONTRIBUTING.md` "Adding a new protocol" and
`LIBRARY.md` §5 are the step-by-steps.

| module | contents |
|---|---|
| `engine/` | the shared core: `protocol.rs` (the traits), `pipeline.rs`, `sync.rs` / `sync2d.rs`, `spectrogram.rs`, `llr.rs`, `equalize.rs`, `scalar.rs` (Q-format types), `fft.rs` (the `FftPlanner` trait + extern factory), `tx.rs`, and `dsp/` (resample, downsample, GFSK, envelope, subtract, DDC, FIR/polyphase, dotprod, the fixed-point FFT kernels) |
| `fec/` | `ldpc/` (174,91), `ldpc240_101/`, `ldpc240_74/` (FST4W: its own BP loop and `fastosd240_74`), `ldpc_128_90/` (MSK144), `conv/` (r=½ K=32 Fano), `rs/` (63,12 over GF(2⁶)), `qra/` + `qra15_65_64/` (Q65) |
| `msg/` | message codecs — `wsjt77.rs`, `jt72.rs`, `wspr.rs`, `q65.rs`, `packet_bytes.rs`, `callsign28.rs`, `hash_table.rs` — plus `decode_request.rs` and `decoded.rs`, which are the public entry point and the public output row |
| `ft8/ ft4/ fst4/ fst4w/ wspr/ jt9/ jt65/ q65/ msk144/ jtty/ uvpacket/` | per-protocol ZSTs (not for `msk144` / `jtty`, which have none), decoders, synthesisers; each feature-gated by its own name. `fst4w/` reuses `fst4/`'s modulation and front end (feature `fst4w` implies `fst4`) and adds `Ldpc240_74`, the 50-bit type-0.6 message and its own candidate ladder |
| `registry.rs` | `PROTOCOLS: &[ProtocolMeta]` + `by_id` / `by_name` / `for_protocol_id` — how a UI or FFI layer asks "what does this build support?" without hardcoding a list. Since 0.12.0 each entry also carries `profile: DecodeProfile` (`caps`, `sync_scale`; until 0.15.0 also `defaults: DecodeDefaults`, removed in #698 — `decoder::default_params` is the one source of the starting block), and `registry::caps` is the capability-bit table `mfsk-ffi` republishes (`tests/registry_caps.rs` ties it to the trait impls) |

**There is no `src/core/`. The shared module is `src/engine/`.**
The source has zero `crate::core::` paths. The stale `core::` spellings
in `CONTRIBUTING.md` and in the `Cargo.toml` feature comments were
corrected in PR #378; `CHANGELOG.md` and `docs/historical/` still carry
it, where it is a historical record and correct as written. (Rust's own
`core::sync::atomic` is everywhere in `src/` and is a different thing —
don't "correct" it.) Don't "fix" a build error by inventing a `core`
module.

To check: `grep -rn 'core::pipeline\|core::fft\|core::scalar\|crate::core::'`
— anything outside CHANGELOG/historical is a leftover.

**`Decoder<P>` (`mfsk-core/src/decoder/`) is the public decode API since 0.13.0.** One persistent decoder per mode,
as `jt9` runs one: it owns what upstream keeps between periods (callsign hash table, FT8 a7 rows, Q65 averages, WSPR's
call table) and takes `DecodeParams` (= `jt9com`'s block: band, `nfqso`, `ntol`, `nftx`, `Depth` = `ndepth`, station,
QSO context, AP mode, contest, EME delay) plus a typed per-mode `Extras` for what the library adds (`strategy`,
`ApHint`, `eq`, `filter`, sniper, noise blanker, Q65 pileup/drift…). `decode(&SlotInput)` takes a whole period
(`Audio::I16`/`F32`, `period`, `Budget`) and streams rows through `on_row`. `decode_prefix` takes the period so far
instead (#572): FT8's checkpoint-A rows at ~11.8 s, and a final call equal to `decode`; the decoder infers the stage
(where it departs from the design: `docs/notes/EARLY_DECODE_DESIGN.md` §12). `AnyDecoder` picks the mode at run time
(`registry::Mode`); an option the mode lacks is `Unsupported`. `Depth` decides every search setting, mapped per mode from
v3.2.0-rc1 and cited in the code; defaults follow the WSJT-X GUI. The 0.12 per-family `DecodeRequest`/`SniperRequest`/
`MultiPeriodRequest` are crate-private (`internal-testing` reopens them for tests; `SniperRequest` stays public for the
WSPR boards). The raw engine functions (`decode_frame`, `process_candidate_basic`, `GenericPipelineProtocol`) are
`pub(crate)` on purpose. Time is `slotgrid` (`SlotGrid`, `SampleClock`, `SlotCutter`), shared by `IqReceiver` (pull
model: owned `CompletedSlot`s, one decoder per channel kept by the caller) and the C stream. The design, its
embedded checks (E1–E11) and the open questions are in the plan `moonlit-mapping-hopcroft.md` and the memory note
`project_v0130_redesign.md`.

Strategy extensions exist per protocol in its `Extras`:
`Ft8Strategy::SicRounds(n)` / `SicEarly` (FT4 has no `SicEarly`), FT8/FT4 only, mirroring an upstream
absence. **The phantom-prone code lives in these non-default strategies**
(both false-decode bugs this suite has shipped were in subtraction paths:
#243 in `__staged_sic`, #253 in `.sic_early()`), so a new strategy ships
with its precision guard in the same PR.

**Sniper mode is an FT8-only roofing-filter mode. The window is what
makes it one — not the hint.** This has been misread more than once,
each time costing a design decision, so: `SniperRequest`'s ±250 Hz window exists because the
operator narrowed the transceiver's *analogue* roofing filter (FTDX101MP,
FTDX10 — the few radios offering one at ~500 Hz) and pointed it at a DX
station whose carrier is already known. The audio arriving is already
band-limited; the decoder is matching the hardware, not being clever.
`.eq_mode()` sits beside it to flatten the tilt that analogue filter's
skirt puts on the passband — so **EQ is a property of the input audio,
not of the search**, which is why `DecodeRequest` carries it too and why
it costs decodes on flat `ft4sim` corpora.

It is gated on its own trait, `SupportsSniper`, **implemented for FT8
alone** (2026-09-13). The wide-band path is the main path for every mode
here; if it is not WSJT-X-faithful without a sniper, that is a bug in the
wide-band path. FT4 is a contest protocol whose premise is working a full
band, and FST4 narrows through its own DDC channelizer instead.

**`.ap_hint()` does work on `SniperRequest`** — it is set, carried, and
passed to `decode_sniper_inner`. What follows is about where AP *came
from*, not whether it is available there; an earlier phrasing here read
as "AP is unavailable in sniper mode", which is false and contradicts
`LIBRARY.md` §2.5 (the per-mode extras).

**A-priori decoding is a general option that got coupled to sniper by
accident.** The AP engine broke out of its candidate loop on
`if has_ap`, so a hint — not a narrow search — was what made it
single-target; and it ran a *parallel, shallower* per-candidate ladder
(OSD depth-2 only, no Top-K rescue) that cost most of the decodes — 4
against 11 on the FT4 golden, measured. Both are gone: AP is now the
last rung of each per-candidate ladder, and `msg::pipeline_ap` is
~100 lines (`wc -l`: 103) of hypothesis generation with no engine of its own. There are
**two** such ladders, not one: `process_candidate_basic` serves FT4 and
every FST4 sub-mode, while FT8 — which does not implement
`GenericPipelineProtocol` — runs its own in
`ft8::decode_block::process_one_candidate_inner`, with an inline copy of
the hypothesis list (#415; unifying them is #423). Full writeup in
`docs/notes/DESIGN_RATIONALE.md` §3.

**JTTY is outside the `Protocol` trait too, for a different reason: it has
no slot.** Frames start whenever the sender likes and a message is several
of them, so the receiver (`jtty::rx::Stream`, incremental; a callback per
`MessageUpdate`) carries state, and the C ABI gives it a handle of its own
(`mfsk_jtty_*`, poll-based with a per-message-coalescing queue) rather than
a slot call. Port of WSJT-X 3.2.0-rc1's `lib/jtty/`; the plan, the
upstream read-through and every measurement are in
`docs/notes/JTTY_UPSTREAM.md` (issue #477).

**MSK144 is intentionally outside the `Protocol` trait** — it isn't FSK,
and `msk144::decode::decode_slot` bypasses `engine::pipeline` by design.
It therefore doesn't appear in `PROTOCOLS` or `protocol_invariants.rs`.
That is an architectural decision, not a gap to close.

`tests/protocol_invariants.rs` runs one generic
`assert_protocol_invariants::<P>` over every wired ZST — **30 of them**:
26 WSJT-family (FST4 now has seven periods and FST4W four), plus `uvpacket`'s four — and pins ~25 trait-level
invariants. A new protocol gets one line there.

Count them with `grep -c 'assert_protocol_invariants::<' mfsk-core/tests/protocol_invariants.rs` (that is ZSTs; the file has fewer `#[test]` functions than that).


