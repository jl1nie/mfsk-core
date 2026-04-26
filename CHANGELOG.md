# Changelog

## 0.2.1 — 2026-04-26

Patch release with no code changes — README hot-fix only.

The 0.2.0 README's `docs/LIBRARY.{md,ja.md}` links resolved to
`github.com/jl1nie/mfsk-core/blob/HEAD/mfsk-core/docs/LIBRARY.*`
when crates.io rendered the README, which is 404 because `docs/`
lives at the workspace root (the crate's
`readme = "../README.md"` pulls the workspace README in). Switched
both links to absolute `https://github.com/.../blob/main/docs/...`
URLs so they resolve from both crates.io and direct GitHub viewing.

## 0.2.0 — 2026-04-26

The Q65 wave: complete the WSJT-X Q65 family (terrestrial Q65-30A
plus EME Q65-60A‥E), expose all four Q65 decoder strategies through
the C ABI, and validate the trait surface end-to-end with a generic
checker plus a runtime registry. ~330 tests across the workspace
(up from ~230 at 0.1.0).

### Added — Q65 weak-signal decoder family complete

- `fec::qra::fast_fading` + `fading_tables` modules port
  `q65_intrinsics_fastfading`, `q65_esnodb_fastfading`,
  `fadengauss.c` and `fadenlorentz.c` from WSJT-X. Decodes the
  10 GHz EME reference recording (60D, VK7MO ↔ K6QPV) where the
  AWGN Bessel front end fails.
- `q65::ap_list::standard_qso_codewords` + `Q65Codec::decode_with_codeword_list`
  port `q65_decode_fullaplist` and `q65_set_list.f90` (the WSJT-X
  206-codeword "full AP list"). At SNR −25 dB (1 dB below the
  published Q65-30A threshold), AP-list decodes 6/6 frames where
  plain BP fails 0/6.
- New entry-point families in `q65::rx`, generic over the sub-mode
  ZST: `decode_at_fading_for<P>` / `decode_scan_fading_for<P>` and
  `decode_at_with_ap_list_for<P>` / `decode_scan_with_ap_list_for<P>`.

### Added — Q65 reaches C/C++/Kotlin via `mfsk-ffi`

- New `MfskProtocol::Q65a30 = 6` enum variant routes Q65-30A through
  the generic-handle path (`mfsk_decoder_new` + `mfsk_decode_f32`).
- Dedicated `mfsk_q65_decode{,_with_ap,_fading,_with_ap_list}`
  function family takes a `MfskQ65SubMode` parameter
  (`A30 / A60 / B60 / C60 / D60 / E60`) and reaches every sub-mode
  with every decoder strategy. New `MfskQ65FadingModel` enum
  (`Gaussian / Lorentzian`) for the fast-fading entry point.
- `mfsk_encode_q65` synthesises any sub-mode from
  `(call1, call2, grid_or_report)`.
- `mfsk-ffi` remains `publish = false` — consumers clone the
  workspace and `cargo build -p mfsk-ffi`.

### Added — Trait surface verified end-to-end

- New `mfsk_core::PROTOCOLS` static + `ProtocolMeta` struct +
  `by_id` / `by_name` / `for_protocol_id` lookup helpers
  (`mfsk-core/src/registry.rs`). Lets UI layers and FFI bridges
  enumerate the wired protocols at runtime; all six Q65 sub-modes
  appear as distinct entries (different NSPS / tone spacing) sharing
  `ProtocolId::Q65`.
- New `tests/protocol_invariants.rs` runs a single generic
  `assert_protocol_invariants::<P: Protocol>` against every wired
  ZST (FT8, FT4, FST4, WSPR, JT9, JT65, plus all six Q65 sub-modes
  — 11 in total) checking 17 trait-level invariants per ZST.
  Cross-checks every `PROTOCOLS` entry against its ZST through a
  separate code path so registry typos are caught.

### Changed

- `ModulationParams::GRAY_MAP` doc contract loosened from
  `len() == NTONES` to `len() ∈ [2^BITS_PER_SYMBOL, NTONES]` to
  match the actual range across protocols (JT9 trims its map to
  the 8 data tones; JT65 / Q65 extend with identity over the sync
  slots). Surfaced by the new invariants test.
- README + `docs/LIBRARY.{md,ja.md}` extended with new sections on
  Q65 decoder-strategy selection (when to use AWGN vs AP vs
  fast-fading vs AP-list) and on the runtime registry / invariants
  test.

### CI

- Heavy synthetic SNR / AP / fast-fading sweeps gated with
  `#[ignore = "slow: ..."]`; local `cargo test` skips them in
  debug mode (10+ min → seconds), CI runs them in release mode via
  `--include-ignored` (~10 s total).

## 0.1.0 — 2026-04-19

Initial release. Consolidates nine previously-separate workspace
crates from the `jl1nie/webft8` project into a single `mfsk-core`
crate with feature-gated protocol modules:

- `mfsk-core`, `mfsk-fec`, `mfsk-msg` → `core`, `fec`, `msg` modules
- `ft8-core`, `ft4-core`, `fst4-core`, `wspr-core`, `jt9-core`,
  `jt65-core` → per-protocol modules behind features of the same
  name

Features shipped at 0.1.0:

- FT8 (15 s, 8-GFSK, LDPC(174, 91))
- FT4 (7.5 s, 4-GFSK, LDPC(174, 91))
- FST4-60A (60 s, 4-GFSK, LDPC(240, 101))
- WSPR (120 s, 4-FSK, convolutional r=½ K=32 + Fano, incl. Type 1/2/3)
- JT9 (60 s, 9-FSK, convolutional r=½ K=32 + Fano)
- JT65 (60 s, 65-FSK, RS(63, 12) GF(2⁶), incl. erasure-aware decode)

Companion (not published): `mfsk-ffi` sibling crate exposing a
C ABI + `mfsk.h` header via cbindgen, with C++ driver and Kotlin
JNI example scaffolds.

Algorithms derived from WSJT-X (K1JT et al.); each source file
cites the corresponding upstream file.
