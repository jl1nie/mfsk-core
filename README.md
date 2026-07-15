# mfsk-core

<p align="center">
  <img src="https://raw.githubusercontent.com/jl1nie/mfsk-core/main/docs/assets/m5sticks3-ft8-decode.jpg"
       alt="M5StickS3 running embedded-poc/m5stack-s3-app — five real on-air FT8 decodes from a single 15 s slot, IDLE FSM waiting for the operator to pick a callsign"
       width="360">
</p>

<p align="center"><i>M5StickS3 running <code>embedded-poc/m5stack-s3-app</code> — five real on-air FT8 decodes from a single 15 s slot, QSO FSM idle waiting for the operator to pick a callsign. See <a href="https://github.com/jl1nie/mfsk-core/blob/main/docs/MANUAL_M5STICKS3.md">docs/MANUAL_M5STICKS3.md</a>.</i></p>

[![CI](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml/badge.svg)](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/mfsk-core.svg)](https://crates.io/crates/mfsk-core)
[![docs.rs](https://img.shields.io/docsrs/mfsk-core)](https://docs.rs/mfsk-core)
[![License](https://img.shields.io/badge/license-GPL--3.0--or--later-blue.svg)](LICENSE)

## What is this?

`mfsk-core` is a pure-Rust library for **WSJT-family digital amateur-radio
modes** — a single crate that implements FT8, FT4, FST4, WSPR, JT9, JT65
and Q65-30A decode / encode / synthesis on top of a small set of shared
primitives (DSP, sync correlation, LLR, LDPC / convolutional /
Reed-Solomon / QRA FEC, message codecs). It runs anywhere Rust runs:
desktop, WASM in the browser, Android/iOS, and `no_std` embedded MCUs.

The [`embedded-poc/m5stack-s3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-s3-app/)
crate shipped with the source tree is a **working M5StickS3 FT8
controller** running the same library on Xtensa LX7 — LCD UI, BLE CI-V to
IC-705, acoustic mic capture, QSO FSM. The image above is one of its
decode slots.

Every algorithm is a Rust re-implementation of
[WSJT-X](https://sourceforge.net/projects/wsjt/) (Joe Taylor K1JT and
collaborators), which remains the reference implementation — see
[Attribution](#attribution) below.

## Supported protocols

| Protocol   | Slot   | FEC                               | Message | Sync                   | Feature |
|------------|--------|-----------------------------------|---------|------------------------|---------|
| FT8        | 15 s   | LDPC(174, 91) + CRC-14            | 77 bit  | 3 × Costas-7           | `ft8`   |
| FT4        | 7.5 s  | LDPC(174, 91) + CRC-14            | 77 bit  | 4 × Costas-4           | `ft4`   |
| FST4-60A   | 60 s   | LDPC(240, 101) + CRC-24           | 77 bit  | 5 × Costas-8           | `fst4`  |
| FST4-15/30/120/300 | 15-300 s | (same LDPC(240, 101))     | 77 bit  | (same sync layout)     | `fst4`  |
| WSPR       | 120 s  | Convolutional r=½ K=32 + Fano     | 50 bit  | Per-symbol LSB (npr3)  | `wspr`  |
| JT9        | 60 s   | Convolutional r=½ K=32 + Fano     | 72 bit  | 16 distributed slots   | `jt9`   |
| JT65       | 60 s   | Reed-Solomon(63, 12) GF(2⁶)       | 72 bit  | 63 distributed slots   | `jt65`  |
| Q65-30A    | 30 s   | QRA(15, 65) GF(2⁶) + CRC-12       | 77 bit  | 22 distributed slots   | `q65`   |
| Q65-60A‥E  | 60 s   | (same QRA codec)                  | 77 bit  | (same sync layout)     | `q65`   |

Seven protocol families, sixteen wired ZSTs in the registry: FST4
contributes five T/R-period sub-modes (FST4-15, -30, -60A, -120, -300)
and Q65 contributes one 30-s sub-mode (Q65-30A) plus five 60-s EME
sub-modes (Q65-60A‥E) — both families share FEC, message codec and
sync layout across their sub-modes, differing only in NSPS / tone
spacing (and, for FST4-15 alone, the T/R start offset).
[`PROTOCOLS`](https://docs.rs/mfsk-core/latest/mfsk_core/static.PROTOCOLS.html)
exposes one entry per wired ZST; `uvpacket` (when enabled) adds four
more for its rate ladder. See [Static set of protocols](#static-set-of-protocols)
for why there's no runtime `register_protocol()`.

## Quick Start

```toml
# Cargo.toml
[dependencies]
mfsk-core = { version = "0.7", features = ["ft8", "ft4"] }
```

Synthesise an FT8 frame and decode it back:

```rust
use mfsk_core::ft8::{
    decode::{decode_frame, DecodeDepth},
    wave_gen::{message_to_tones, tones_to_i16},
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
let end = (start + frame.len()).min(audio.len());
audio[start..end].copy_from_slice(&frame[..end - start]);

// 2. Decode it back.
for r in decode_frame(&audio, 100.0, 3_000.0, 1.0, None, DecodeDepth::BpAllOsd, 50) {
    if let Some(text) = unpack77(&r.message77) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

That's the whole round trip: pack a message → synthesise 12 kHz PCM →
decode it back. Each protocol module documents its own top-level entry
points and carries its own Quick example:

- [`mfsk_core::ft8`](https://docs.rs/mfsk-core/latest/mfsk_core/ft8/)
  — `decode_frame` + `decode_sniper_ap` (narrow-band "sniper" mode)
- [`mfsk_core::ft4`](https://docs.rs/mfsk-core/latest/mfsk_core/ft4/)
  — `decode_frame`
- [`mfsk_core::fst4`](https://docs.rs/mfsk-core/latest/mfsk_core/fst4/)
  — FST4-60A `decode_frame`; other sub-modes via `decode_frame_for::<Fst4s120>` etc.
- [`mfsk_core::wspr`](https://docs.rs/mfsk-core/latest/mfsk_core/wspr/)
  — `decode::decode_scan_default`
- [`mfsk_core::jt9`](https://docs.rs/mfsk-core/latest/mfsk_core/jt9/)
  — `decode_scan_default`
- [`mfsk_core::jt65`](https://docs.rs/mfsk-core/latest/mfsk_core/jt65/)
  — `decode_scan_default` + `decode_at_with_erasures` (for low SNR)
- [`mfsk_core::q65`](https://docs.rs/mfsk-core/latest/mfsk_core/q65/)
  — `decode_scan_default` (Q65-30A); generic `decode_scan_for<P>`
  for any wired sub-mode including the Q65-60A‥E EME variants;
  `decode_scan_with_ap` / `decode_scan_with_ap_for<P>` for AP-hint
  decoding (~2 dB threshold gain when call signs are known); and
  `decode_scan_fading_for<P>` for the fast-fading metric (Gaussian
  / Lorentzian channel models) that recovers 5–8 dB on Doppler-spread
  channels — required for microwave EME at 5.7 / 10 / 24 GHz; and
  `decode_scan_with_ap_list_for<P>` (paired with `standard_qso_codewords`)
  for BP-free template matching against the full WSJT-X "AP list"
  of standard exchanges (~3 dB threshold gain when the callsign pair
  is known up-front)

## Features

| Feature       | Default | What it enables                              |
|---------------|---------|----------------------------------------------|
| `ft8`         | ✓       | FT8 decode / synth                           |
| `ft4`         | ✓       | FT4 decode / synth                           |
| `fst4`        |         | FST4-60A decode / synth (+ FST4-15/30/120/300) |
| `wspr`        |         | WSPR decode / synth                          |
| `jt9`         |         | JT9 decode / synth                           |
| `jt65`        |         | JT65 decode / synth (+ erasure-aware RS)     |
| `q65`         |         | Q65-30A decode / synth (QRA soft-decision)   |
| `uvpacket`    |         | Applied example *(experimental)*: NFM voice-channel packet protocol (QPSK + LDPC), reuses `Ldpc240_101` |
| `full`        |         | Aggregate of all seven WSJT protocols + uvpacket + packet-bytes |
| `parallel`    | ✓       | Rayon-parallel candidate processing          |
| `fft-rustfft` | ✓       | Default host FFT backend (`rustfft`, requires `std`) |
| `fft-extern`  |         | Pluggable FFT trait — caller binary supplies an `FftPlanner` impl (esp-dsp on ESP32-S3, CMSIS-DSP on RP2350, …) |
| `fixed-point` |         | Embedded integer pipeline: u16 spectrogram + i16 DFT + Q11i16 LLR + integer NMS BP (see [Status](#status) for the Q3i8 → Q11i16 step taken in 0.6.2 / 0.6.3 for recall) |
| `profile-coarse` |      | Always-on coarse_sync sub-stage profiling (automatically disabled on `wasm32-unknown-unknown` to prevent panics) |

## Architecture

Every protocol runs the same DSP pipeline, differing only in the
constants each stage plugs in (tone count, sync pattern, FEC codec,
message layout):

```text
Audio (i16 PCM)
  │
  ▼
FFT              spectrogram over the slot
  │
  ▼
Candidate Search  coarse frequency/time sync — Costas/sync-pattern
  │                correlation across the spectrogram
  ▼
Synchronization   fine time/frequency refinement per candidate
  │
  ▼
Demodulation      tone → LLR (soft bits), GFSK/FSK-aware
  │
  ▼
FEC               LDPC / convolutional+Fano / Reed-Solomon / QRA decode
  │
  ▼
Decoded Message    77-/72-/50-bit unpack → callsign / grid / report
```

This is the same shape for FT8's LDPC(174,91) and JT65's
Reed-Solomon(63,12) — only the boxes' contents change per protocol.
See [Design Philosophy](#design-philosophy) for how that's expressed
in code (a `Protocol` trait, not per-mode copy-paste), and
[`docs/LIBRARY.md` §4](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.md#4-shared-primitives-core)
for the full data-flow diagram down to function level.

## Design Philosophy

### Why this exists

[WSJT-X](https://sourceforge.net/projects/wsjt/) is the reference
implementation of these modes and will stay that way — it is
battle-tested on the desktop, heavily optimised, and the source of
truth for every protocol constant you will find in this crate. But
it is also a mixed Fortran / C / Qt application built around a
specific desktop workflow. That makes it a poor fit whenever you
want to run the decoders *somewhere else*:

- in a **browser** as a WASM PWA,
- on **Android or iOS** for portable operation, where linking a
  Fortran runtime is a non-starter,
- in a **headless Rust application** (skimmer, monitoring station,
  remote SDR front end),
- on **embedded MCUs** (ESP32-S3 with esp-dsp, RP2350 with CMSIS-DSP,
  Cortex-M) via `no_std + alloc` — the M5Stack Core2 PoC decodes 3–7
  FT8 results per 15 s cycle on Xtensa LX6 with the fixed-point hot
  path,
- or as the core of a **new protocol experiment** that reuses FT8's
  LDPC and sync machinery for a different modulation / FEC /
  message recipe.

### Why a `Protocol` trait

The seven protocols share roughly 80 % of their signal path: 8-GFSK /
FSK demodulation, soft-decision LDPC / convolutional / Reed-Solomon /
QRA decoding, 77- / 72- / 50-bit WSJT message packing, spectrogram-based
sync search. In the Fortran codebase that commonality is expressed by
copy-and-paste between per-mode source files; here it is expressed by
traits, split by what actually varies per protocol:

- **Shared** (lives in `core`, generic over any `P: Protocol`): coarse
  sync, fine sync, LLR computation, equalisation, the decode pipeline
  driver, GFSK synthesis.
- **Protocol-specific** (declared as `const` associated items + ZSTs on
  the protocol type): tone count, symbol rate, Gray map, GFSK shaping
  constants (`ModulationParams`); total symbols, sync/data layout, slot
  length (`FrameLayout`); which FEC codec (`Protocol::Fec`, e.g. LDPC
  vs Reed-Solomon) and which message codec (`Protocol::Msg`, e.g. 77-bit
  WSJT vs 50-bit WSPR) the protocol plugs in.

```text
         ┌────────────────────────────────────────────────────────┐
         │   ft8   ft4   fst4   wspr   jt9   jt65   q65           │  per-protocol ZSTs
         │        (each implements Protocol + FrameLayout)         │  (feature-gated)
         └─────────────┬─────────────────┬────────────────────────┘
                       │                 │
              ┌────────▼─────────┐  ┌────▼─────────┐
              │       msg        │  │     fec      │  shared codecs
              │  Wsjt77 · Jt72   │  │ LDPC · RS    │  behind traits
              │  Wspr50  · Q65   │  │ ConvFano·QRA │
              │  · Hash table    │  │              │
              └────────┬─────────┘  └────┬─────────┘
                       │                 │
                   ┌───▼─────────────────▼───┐
                   │          core           │  Protocol trait, DSP
                   │ sync · llr · equalize · │  (resample / GFSK /
                   │  pipeline · tx · dsp    │   downsample / subtract)
                   └─────────────────────────┘
```

### Zero-cost: generic decoder, not a runtime dispatch table

Because everything above is expressed as `const` associated items +
ZSTs, the generic pipeline code — `coarse_sync::<P>`, `decode_frame::<P>`,
the LDPC inner loop — is **monomorphised per protocol**. LLVM sees a
fully specialised function for each `P`, inlines the constants, and
autovectorises the hot loops. The generated machine code is
byte-identical to a hand-written per-protocol decoder; the receive
path is a chain of free functions in `core::sync` → `core::llr` →
`core::equalize` → `core::pipeline` (each generic over `P: Protocol`),
not a `Demodulator` / `Receiver` trait object — there is no vtable, no
dynamic dispatch, on the hot path.

This pays off most clearly when adding a new protocol: it's a trait
impl on a ZST, not a cross-cutting refactor. FST4-60A joined the crate
post-hoc without changing any shared pipeline code — the entire
implementation is the trait impl block on a single ZST plus a
Costas pattern table. Similarly, swapping an LDPC codec between two
LDPC modes, or exposing the same 77-bit message layer to FT8, FT4 and
FST4, are one-line changes, not cross-cutting refactors.

### Why Rust

- **Safety**: bit-level FEC routines (LDPC belief propagation,
  Karn's Berlekamp-Massey + Forney for RS, Fano sequential decoding)
  are textbook index-heavy code. Writing them in safe Rust eliminates
  an entire class of memory-corruption bugs that Fortran / C ports
  have historically hidden.
- **Generics + trait bounds**: describing a protocol family as data +
  traits is natural. The equivalent in C++ would be template
  metaprogramming with subtler error messages; in Fortran, it simply
  isn't on offer.
- **Targets**: the same code compiles to `wasm32-unknown-unknown`
  (WASM SIMD 128-bit via `rustfft`), to Android `arm64-v8a` via the
  NDK (NEON SIMD), to `no_std + alloc` embedded MCUs, and to any
  `x86_64-*-unknown` host for servers — from a single source tree.
- **Ecosystem**: `rustfft`, `num-complex`, `crc`, `rayon` are
  plug-and-play, so the crate's dependency graph is small and
  reviewable.

## Performance

- **Embedded (M5StickS3, Xtensa LX7, fixed-point)**: decodes a real
  on-air busy-band FT8 slot in **~1.19 s post-SlotEnd** via the
  streaming pipeline (FFT overlapped with capture); BP scratch is
  ~12 KB with the `Q11i16` LLR format. M5Stack Core2 (Xtensa LX6) on
  the same recording: ~2.8 s.
- **Host (desktop, f32)**: `decode_frame_subtract_with_ap` (multipass
  + AP hints) reaches 18 decodes on the WSJT-X busy-band reference
  WAV, including 5/6 of the JTDX AP-on extras.
- **Memory**: the embedded fixed-point path is designed to fit inside
  ESP32-S3 internal DRAM without PSRAM for the hot buffers — see
  [`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
  for the byte-level BP-scratch and spectrogram budget.
- Full recall tables against WSJT-X-distributed reference recordings,
  per-protocol golden-WAV results, and the fixed-point Q-format
  history are in [Status](#status) below and
  [`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md).

## Comparison with WSJT-X

Not a competitor — a different point in the design space. WSJT-X is
the reference implementation and the source of truth for every
protocol constant in this crate; `mfsk-core` exists to run the same
algorithms in places a Fortran/C/Qt desktop application can't reach.

| | WSJT-X | mfsk-core |
|---|---|---|
| Language | Fortran + C + Qt | Rust |
| Distribution | Desktop application | Library crate (`cargo add`) |
| `no_std` / embedded | ✗ | ✓ (ESP32-S3, RP2350, Cortex-M) |
| WASM | ✗ | ✓ (`wasm32-unknown-unknown`) |
| Android / iOS | ✗ | ✓ (NDK / FFI) |
| FFT backend | fixed (FFTW) | pluggable (`rustfft` or caller-supplied, e.g. esp-dsp/CMSIS-DSP) |
| Reference implementation | ✓ | derived from WSJT-X, cites source per file |

## FAQ

**How does this differ from WSJT-X?** See
[Comparison with WSJT-X](#comparison-with-wsjt-x) and
[Why this exists](#why-this-exists) above — same algorithms, different
deployment targets (library vs desktop app, `no_std` embedded, WASM).

**Does it support `no_std`?** Yes — `default-features = false, features
= ["alloc", "ft8", "fft-extern"]` (or similar) builds without `std`.
`std` is only required by the default `fft-rustfft` backend; embedded
targets swap in their own FFT via `fft-extern`. See
[`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md).

**Can I swap the FFT backend?** Yes — enable `fft-extern` instead of
`fft-rustfft` and provide an `FftPlanner` impl (`core::fft`); the
embedded ports use this for esp-dsp (ESP32-S3) and CMSIS-DSP (RP2350).

**How do I use this on embedded hardware?** Start with
[`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
(feature-flag map, FFT-extern contract, Q-format reference, full C ABI
tutorial) and, for a complete working example,
[`embedded-poc/m5stack-s3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-s3-app/)
(a shipping M5StickS3 FT8 controller).

**Is the API stable?** Not yet — see [Status](#status). Breaking
changes follow cargo-style minor bumps (`0.x` line).

## License

**GPL-3.0-or-later**, matching upstream WSJT-X. See [LICENSE](LICENSE).

------------------------------------------------------------------------

## Reference

The sections above cover getting started and the design rationale.
The rest of this document is reference material: attribution, module
layout, the FFI surface, contribution workflow, and the detailed
per-protocol status / recall tables.

## Attribution

Every algorithm in this crate is derived from
[WSJT-X](https://sourceforge.net/projects/wsjt/) (Joe Taylor K1JT and
collaborators). Source files cite the corresponding upstream
`lib/ft8/*`, `lib/ft4/*`, `lib/fst4/*`, `lib/wsprd/*`, `lib/jt65_*`,
`lib/jt9_*`, `lib/packjt.f90`, etc. that they port from. This is a
Rust re-implementation aimed at broadening the set of platforms
(browser / WASM, Android, embedded) that can host the decoders —
**not** a replacement for WSJT-X itself, which remains the reference
implementation.

License matches upstream: **GPL-3.0-or-later**.

## Protocol registry details

### Static set of protocols

`PROTOCOLS` is a `const` slice — the set of supported protocols is
fixed at compile time by Cargo features. There is no runtime
`register_protocol()` API by design: every wired ZST is verified by
`tests/protocol_invariants.rs` to satisfy the trait surface, and that
guarantee can't be extended to types unknown at compile time. UI /
FFI consumers should iterate `PROTOCOLS` (or filter via `by_id` /
`by_name`) at startup; if you need a new protocol, add the ZST + a
`protocol_meta!` line and rebuild.

### Applied example: `uvpacket` (experimental)

The `uvpacket` module (`--features uvpacket`, off by default) is
an in-tree example showing the trait abstractions extend beyond
WSJT-X — a π/4-DQPSK packet protocol for NFM / SSB voice channels
that reuses the shared `Ldpc240_101` codec. **Experimental and
its public API may change** — pin to an exact version. See
[`docs/UVPACKET.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/UVPACKET.md)
([日本語](https://github.com/jl1nie/mfsk-core/blob/main/docs/UVPACKET.ja.md))
for the design narrative, modulation / equaliser / framing details
and per-mode performance characterisation.

## Modules

- `mfsk_core::core` — protocol traits, DSP (resample / downsample /
  GFSK / subtract), sync, LLR, equaliser, pipeline driver.
- `mfsk_core::fec` — `Ldpc174_91` / `Ldpc240_101` / `ConvFano` /
  `ConvFano232` / `Rs63_12` / `qra::Q65Codec` (with the
  `qra15_65_64::QRA15_65_64_IRR_E23` code instance) for Q65.
- `mfsk_core::msg` — 77-bit (`Wsjt77Message`), 72-bit (`Jt72Codec`),
  50-bit (`Wspr50Message`) and Q65 (`Q65Message`, 77-bit ↔ 13-symbol
  packing helpers) message codecs; callsign hash table.
- `mfsk_core::{ft8, ft4, fst4, wspr, jt9, jt65, q65}` — per-protocol
  ZSTs, decoders and synthesisers (each feature-gated). The `q65`
  module exposes one ZST per wired sub-mode — `Q65a30` for
  terrestrial work, plus `Q65a60` / `Q65b60` / `Q65c60` / `Q65d60` /
  `Q65e60` for EME at 6 m through 10 GHz+ — with generic
  `synthesize_standard_for<P>` / `decode_at_for<P>` / `decode_scan_for<P>`
  helpers that pick the right NSPS and tone spacing from the type
  parameter.

## C / C++ / Kotlin

The `mfsk-ffi` sibling crate in this repository builds a
`libmfsk.{so,a,dylib}` + `mfsk.h` (via `cbindgen`) that exposes the
same decoder and synthesiser surface through an opaque-handle C ABI.
`mfsk-ffi` is not published to crates.io — consumers clone this repo and run:

```
cargo build -p mfsk-ffi --release
```

See `mfsk-ffi/examples/cpp_smoke/` for an end-to-end driver test
(including multi-threaded usage) and `mfsk-ffi/examples/kotlin_jni/`
for an Android/JNI skeleton.

## Contributing

PRs welcome — recent forks have shipped FT4 SIC, FT4/FST4 depth +
strictness controls, and the FT8 wide-band AP path. The local-fence
+ CI gates are uniform across direct commits and fork PRs:

- **Pre-commit hook**: `.githooks/pre-commit` runs `cargo fmt --check`,
  `cargo clippy --workspace --all-targets --features full -- -D warnings`,
  and `RUSTDOCFLAGS=-D warnings cargo doc -p mfsk-core --features full
  --no-deps` (~10–20 s on a warm cache). Enable once per clone:

  ```sh
  git config core.hooksPath .githooks
  ```

  The hook deliberately skips the full `cargo test` suite (kept in
  CI to keep commits snappy); fmt / clippy / rustdoc each catch a
  failure mode that would otherwise trip CI after the push.
- **CI gates** (`.github/workflows/ci.yml`): same fmt + clippy
  fence, plus `cargo test -p mfsk-core --features full --release --
  --include-ignored` (slow synthetic-SNR / AP / fast-fading sweeps
  enabled), a 13-cell feature matrix that builds every protocol in
  isolation + the embedded `alloc + ft8 + fft-extern + fixed-point`
  preset, the C++ driver against `mfsk-ffi`, rustdoc with `-D warnings`,
  and a `cargo publish --dry-run` for `mfsk-core`.
- **Release**: tag-driven (`v0.6.x`). Pushing a tag that matches
  `mfsk-core/Cargo.toml::version` and is reachable from `main`
  triggers `release.yml`, which publishes to crates.io and cuts a
  GitHub release with auto-generated notes. Embedded `mfsk-ffi-ft8`
  binaries follow on the same tag.

For non-trivial changes, please open an issue first so the
WSJT-X-source-faithfulness lineage of any DSP or FEC change is
visible in review (every protocol constant in this crate cites the
upstream `lib/*.f90` it ports from — drift from that lineage tends
to be the failure mode caught by the WSJT-X golden harnesses).

## Architecture & ABI reference

For a deeper look at the design — trait hierarchy with worked
examples, shared DSP / sync / LLR / pipeline primitives, the C ABI
memory model, Kotlin/Android scaffolding — see the library
reference:

<!-- Absolute URLs so the links resolve from both GitHub and the
     crates.io README renderer (which otherwise rewrites
     "docs/LIBRARY.md" to mfsk-core/docs/... — see workspace
     layout: docs/ lives at the repo root, not under the crate). -->
- **English:** [`docs/LIBRARY.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.md)
- **日本語:** [`docs/LIBRARY.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/LIBRARY.ja.md)
- **Embedded targets:**
  [English `docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
  / [日本語 `docs/EMBEDDED.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.ja.md)
  — generic-scalar architecture (one codebase for f32 host and
  fixed-point embedded), feature-flag map, FFT-extern contract,
  Goertzel per-symbol DFT (zero-scratch, 0.6.4+) with BASIS
  deprecation, Q-format reference, full `mfsk-ffi-ft8` C ABI
  tutorial (streaming + ESP-IDF component layout), performance
  benchmark, streaming RX pipeline, binary footprint.
- **M5StickS3 FT8 controller manual:**
  [English `docs/MANUAL_M5STICKS3.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/MANUAL_M5STICKS3.md)
  / [日本語 `docs/MANUAL_M5STICKS3.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/MANUAL_M5STICKS3.ja.md)
  — build / flash / `cfg.toml` / `BootMode` cycle / UI / QSO
  workflow / troubleshooting.

## Status

**Current line: `0.6.x`** (latest tag `v0.6.5`, 2026-05-18) — API
is deliberately not frozen. Breaking changes follow cargo-style minor
bumps (`0.5 → 0.6`). See `CHANGELOG.md` for the per-release breakdown;
the line history below explains what each series brought in. The
0.5.x line landed the
embedded baseline (`no_std + alloc`, pluggable FFT backend,
caller-buffer TX APIs) and the first end-to-end real-audio embedded
port. The **0.6.x line consolidates the FT8 sync + per-candidate
pipeline**: host (`decode_frame*`) and embedded (`decode_block`) now
share `decode_block::coarse_sync` (WSJT-X `sync8.f90`-faithful 16-bin
allsum estimator) and a unified `process_one_candidate_inner` for the
LLR / BP / OSD / AP staircase. **0.6.3** added WSJT-X-faithful OSD
`npre1`/`npre2` precoding (#63) and carved `decode_block.rs` into
six stage submodules (ε refactor). **0.6.4** (Phase 1.7.7-Stick)
replaced the BASIS Q15 dot-product per-symbol DFT with a Goertzel
recursion that needs **zero scratch** — freeing 120 KB of internal
DRAM on dual-core S3 (which unblocks M5StickS3 Qso-mode I2S
bidirectional DMA) and improving SNR by **+0.16..+0.63 dB** on
real-silicon decodes. See `CHANGELOG.md` for the full per-release
breakdown.

Latest M5StickS3 (Xtensa LX7) ship config decodes **6 / 18 JTDX-golden
FT8 callsigns + 1 bonus = 7 total** on the WSJT-X-distributed busy-band
reference (`samples/FT8/210703_133430.wav`) in **~1.19 s post-SlotEnd**
via the streaming pipeline (FFT overlapped with capture). (These
embedded recall + wall-clock numbers were last formally measured
during the 0.6.2 → 0.6.3 Q11i16 ship sweep — `wav_sim` re-run on
0.6.5 firmware is the right way to confirm whether 0.6.3's host-side
OSD CRC-luck phantom drop applies to the embedded path. 0.6.3
CHANGELOG only re-verified the WSJT-X 8-entry golden at 7/8 on
embedded.) The
embedded LLR scalar settled on `Q11i16` after a two-step path:
0.5.x's Phase 1 used `Q3i8` (i8, ±16, ~1/8 LSB) and a 2026-05-04
host sweep initially read as Q3i8-equivalent. The wider real-silicon
LX7 sweep done in 0.6.3 then showed Q3i8's ~0.875-LLR quantization
step was the dominant recall ceiling on Xtensa — host fixed-point +
rustfft hit 16/18 with f32 but only 9/18 with Q3i8 on this WAV. The
`Q11i16` widening (i16, ±16, ~1/2048 LSB, BP scratch doubled from
~6 KB to ~12 KB, still inside the S3 / Core2 internal-DRAM budget)
removed the LLR-resolution bottleneck. The recovery is asymmetric:
on host `Q11i16` reaches f32-equivalent recall (16/18 ≈ 16/18, full
gap close); on real-silicon embedded the gain is one entry
(6/18 → 6/18 + 1 bonus = 7 total, XE2X HA2NP RR73), with the
remaining headroom blocked by other parts of the embedded pipeline
(NSTEP-half, coarse-sync simplifications, no `fine_refine_pass1`).
`Q3i8` is preserved in
`core::scalar` for the comparison path. M5Stack Core2 (LX6) on the
same WAV ~2.8 s. See
[`docs/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/EMBEDDED.md)
for the integration contract, runtime BP / `nstep-half` tuning knobs,
and the structural recall ceiling (no `fine_refine_pass1` on Xtensa
without 192k FFT — investigated and deferred);
`embedded-poc/m5stack-s3-app/` and `embedded-poc/m5stack-core2-app/`
are the production FT8 controller crates (LCD UI + QSO FSM +
WiFi-UDP log streaming) — both consume the board-agnostic
`embedded-poc/mfsk-app-shared/` (carved out as part of
[#61](https://github.com/jl1nie/mfsk-core/issues/61), closed in
0.6.3). `embedded-poc/m5stack-s3/` is the remaining decoder-only
compute-bench crate for S3 timing-regression tracking; the matching
Core2 bench was retired in #61 Phase 3 (M5Stack Core2 timing is
now measured through `m5stack-core2-app` itself).

Host AP-on multipass recall on the same WAV is materially higher.
After 0.6.2's host pipeline catch-up (cs-source unification +
`subtract_signal_lpf` matching `decode_block_multipass`),
`decode_frame_subtract_with_ap` produces **18 decodes** including
**5 / 6 of the JTDX AP-on extras** (was 1 / 6 pre-0.6.2). The single
remaining AP-on extra (K1BZM DK8NE -19 dB) needs a wider AP-list /
callsign hash table — out of 0.6.x scope.

Algorithm correctness is covered by the workspace test suite:
end-to-end synth → decode roundtrips for every protocol, an AWGN
sensitivity sweep that confirms Q65-30A hits its WSJT-X-published
−24 dB threshold, an AP-vs-plain comparison that shows the expected
~2 dB gain from a-priori call sign information, an AP-list
(template matching) comparison that decodes 6/6 frames at SNR −25 dB
where plain BP fails 0/6, a real 6 m EME recording (W7GJ exchanges
from the WSJT-X reference set), and a real 10 GHz EME recording that
the fast-fading metric is required to decode. The trait surface
itself is pinned by `tests/protocol_invariants.rs` — a single generic
`<P: Protocol>` checker run across every wired ZST. Run with
`--features full` for the eleven-ZST coverage; the default features
(`ft8`, `ft4`) only exercise the two default protocols.

Recall against the **WSJT-X-distributed reference recordings**
(`samples/{WSPR,FT4,JT9}/*.wav`) is locked by golden harnesses in
`tests/{wspr,ft4,jt9}_wsjtx_samples.rs` (run only when the WSJT-X
tree is present at the expected sibling path):

| Reference WAV                       | Recall      | Notes                  |
|-------------------------------------|-------------|------------------------|
| `WSPR/150426_0918.wav` (8 frames)   | **8 / 8**   | sub-bin demod + neg-dt |
| `FT4/000000_000002.wav` (6 frames)  | **6 / 6**   | Nuttall + sync4d       |
| `JT9/130418_1742.wav` (5 frames)    | **5 / 5**   | full WSJT-X-faithful softsym pipeline (afc9 + chkss2 + xx0 mettab + sync9 collapse) |
| `MSK144/181211_120500.wav` (n/a)    | —           | not implemented — [#25](https://github.com/jl1nie/mfsk-core/issues/25) |
| `JT65/*` (n/a)                      | —           | golden harness pending — [#24](https://github.com/jl1nie/mfsk-core/issues/24) |
| `FST4/210115_0058.wav` (1 frame)    | **1 / 1**   | fixed NSPS/NDOWN/BT + rvec message scramble (0.6.8, [#23](https://github.com/jl1nie/mfsk-core/issues/23)) |

#### Known limitations / quirks

- **JT65** — `decode_scan` decodes synthesised JT65A frames cleanly
  via the Reed-Solomon(63, 12) FEC and the AP-list path lifts weak
  signals to within ~2 dB of WSJT-X. There's no golden WAV in this
  crate's regression set because the WSJT-X v3 reference samples
  themselves don't decode cleanly without the soft-symbol erasure
  metadata that lives in private WSJT-X branches. Tracked in
  [#24](https://github.com/jl1nie/mfsk-core/issues/24).
- **FST4** — five T/R-period sub-modes wired: FST4-15,
  -30, -60A, -120, -300 (`fst4::Fst4s15`/`Fst4s30`/`Fst4s60`/
  `Fst4s120`/`Fst4s300`), all sharing frame layout / FEC / message
  codec / GFSK shaping and differing only in `NSPS`/`NDOWN` (and, for
  FST4-15 alone, the T/R start offset) — every constant verified
  directly against WSJT-X `fst4_decode.f90` / `fst4sim.f90` source,
  the same rigor that caught #23's original bug. Only FST4-60A has a
  real-recording golden-WAV lock (`samples/FST4/210115_0058.wav`,
  1/1) — the WSJT-X sample tree ships no FST4-15/30/120/300
  recordings, so those four are validated by synth-roundtrip
  self-consistency plus source cross-checks only; a real recording or
  a WSJT-X `fst4sim`-generated reference WAV would strengthen that.
  FST4-900 / FST4-1800 and FST4W (the WSPR-style 50-bit one-way
  beacon variant, a different message format entirely) remain out of
  scope — no user demand as of writing. See
  [#23](https://github.com/jl1nie/mfsk-core/issues/23).
- **MSK144** — not implemented (out of scope of the 0.5.x line and still as of 0.6.x). The decode path needs a
  different correlator geometry from the rest of the FT/JT/Q-family
  decoders this crate is built around. Tracked in
  [#25](https://github.com/jl1nie/mfsk-core/issues/25).

#### What's solid

- **FT8** — synth → decode round-trip lib tests green, real WSJT-X
  reference (`210703_133430.wav`):
  - **WSJT-X 8-entry golden: 7 / 8** (host `decode_frame_with_ap` and
    embedded `decode_block` both, post-0.6.0 sync consolidation +
    `i_start as i32` fix).
  - **JTDX 18-entry golden: 13 / 18** (`decode_block`). Peaked at
    16/18 in 0.6.2; 0.6.3's WSJT-X-faithful OSD `npre1`/`npre2`
    precoding + `OSD_HARDERRORS_MAX = 22` ceiling identified 3 of
    those as CRC-luck phantoms and dropped them. The 13/18 is true
    positives only.
  - **Host AP-on multipass JTDX-extras: 4 / 6** (was 1 / 6 pre-0.6.2,
    5 / 6 in 0.6.2, 4 / 6 in 0.6.3 after the same phantom drop)
    via `decode_frame_subtract_with_ap` after the cs-source +
    `subtract_signal_lpf` unification.
  - **Embedded S3 fixed-point: 6 / 18 + 1 bonus = 7 total** in
    ~1.19 s post-SlotEnd (last formally measured during the 0.6.2 →
    0.6.3 Q11i16 ship sweep — see *Status* above for re-measurement
    status). AP-hint biasing exposed on the narrow-band
    (`decode_sniper_ap`), wide-band single-pass (`decode_frame_with_ap`),
    multipass (`decode_frame_subtract_with_ap`) and embedded
    (`decode_block_with_ap`, new in 0.6.1) paths.
- **WSPR** — 8 / 8 WSJT-X golden, ~0.88 s end-to-end on a desktop
  build. Sub-bin demod + 2-pass subtract+re-coarse + OSD-2 fallback +
  Type-3 phantom filter.
- **FT4** — 6 / 6 WSJT-X golden after the 0.5.9 multi-slice port of
  the WSJT-X demod path (Nuttall window, `nsym = 4` LLR aggregation,
  `sync4d` 2-pass refine, `rvec` scrambler). Successive-interference
  cancellation primitives (`subtract_signal*`, `refine_signal_freq`)
  ported from `lib/ft4_subtract.f90`. WSJT-X Decode menu (Fast /
  Normal / Deep) exposed via `decode_frame_with_options` for FT4
  and FST4-60A.
- **JT9** — 5 / 5 WSJT-X golden on `samples/JT9/130418_1742.wav`
  via the full WSJT-X-faithful softsym pipeline (`afc9` + `chkss2`
  + `xx0` mettab + `sync9` per-freq collapse). Closed [#19](https://github.com/jl1nie/mfsk-core/issues/19).
- **Q65** — fast-fading + AP-list paths exercise both the WSJT-X
  6 m EME and the 10 GHz EME reference recordings.
- **SNR (FT8)** — `xsnr2_db_simple` calibration (0.5.7 + 0.5.8) lands
  reported SNR within ±3 dB of JTDX absolute on real silicon.
