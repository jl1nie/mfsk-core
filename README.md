# mfsk-core

Pure-Rust library for **WSJT-family digital amateur-radio modes** — a
single crate that implements FT8, FT4, FST4, WSPR, JT9 and JT65
decode / encode / synthesis on top of a small set of shared
primitives (DSP, sync correlation, LLR, LDPC / convolutional / Reed-
Solomon FEC, message codecs).

The architecture is a **zero-cost generic abstraction**: each
protocol is a zero-sized type that implements traits describing its
modulation parameters, frame layout, FEC and message codec. Hot-path
code like `coarse_sync::<P>` and `decode_frame::<P>` is monomorphised
per protocol, so the generated code is byte-identical to a hand-
rolled per-protocol implementation while the library surface stays
cohesive.

```toml
[dependencies]
mfsk-core = { version = "0.1", features = ["ft8", "ft4"] }
```

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

## Protocols

| Protocol   | Slot   | FEC                               | Message | Sync                   | Feature |
|------------|--------|-----------------------------------|---------|------------------------|---------|
| FT8        | 15 s   | LDPC(174, 91) + CRC-14            | 77 bit  | 3 × Costas-7           | `ft8`   |
| FT4        | 7.5 s  | LDPC(174, 91) + CRC-14            | 77 bit  | 4 × Costas-4           | `ft4`   |
| FST4-60A   | 60 s   | LDPC(240, 101) + CRC-24           | 77 bit  | 5 × Costas-8           | `fst4`  |
| WSPR       | 120 s  | Convolutional r=½ K=32 + Fano     | 50 bit  | Per-symbol LSB (npr3)  | `wspr`  |
| JT9        | 60 s   | Convolutional r=½ K=32 + Fano     | 72 bit  | 16 distributed slots   | `jt9`   |
| JT65       | 60 s   | Reed-Solomon(63, 12) GF(2⁶)       | 72 bit  | 63 distributed slots   | `jt65`  |

## Modules

- `mfsk_core::core` — protocol traits, DSP (resample / downsample /
  GFSK / subtract), sync, LLR, equaliser, pipeline driver.
- `mfsk_core::fec` — `Ldpc174_91` / `Ldpc240_101` / `ConvFano` /
  `ConvFano232` / `Rs63_12`.
- `mfsk_core::msg` — 77-bit (`Wsjt77Message`), 72-bit (`Jt72Codec`)
  and 50-bit (`Wspr50Message`) message codecs; callsign hash table.
- `mfsk_core::{ft8, ft4, fst4, wspr, jt9, jt65}` — per-protocol
  ZSTs, decoders and synthesisers (each feature-gated).

## Features

| Feature       | Default | What it enables                              |
|---------------|---------|----------------------------------------------|
| `ft8`         | ✓       | FT8 decode / synth                           |
| `ft4`         | ✓       | FT4 decode / synth                           |
| `fst4`        |         | FST4-60A decode / synth                      |
| `wspr`        |         | WSPR decode / synth                          |
| `jt9`         |         | JT9 decode / synth                           |
| `jt65`        |         | JT65 decode / synth (+ erasure-aware RS)     |
| `full`        |         | Aggregate of all six protocols               |
| `parallel`    | ✓       | Rayon-parallel candidate processing          |
| `osd-deep`    |         | OSD-3 fallback on AP decodes (extra CPU)     |
| `eq-fallback` |         | Non-EQ fallback inside `EqMode::Adaptive`    |

## Quick example

```rust
use mfsk_core::ft8::{Ft8, decode::{decode_frame, DecodeDepth}};
use mfsk_core::msg::wsjt77::unpack77;

let audio: Vec<i16> = /* 12 kHz PCM, 15 s */ vec![];
for r in decode_frame(&audio, 100.0, 3000.0, 1.5, None, DecodeDepth::BpAllOsd, 200) {
    if let Some(text) = unpack77(&r.message77) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

See each protocol module's docstring for the equivalent entry points
(e.g. `mfsk_core::wspr::decode::decode_scan_default`,
`mfsk_core::jt65::decode_at_with_erasures`, etc.).

## C / C++ / Kotlin

The `mfsk-ffi` sibling crate in this repository builds a
`libmfsk.{so,a,dylib}` + `mfsk.h` (via `cbindgen`) that exposes the
same decoder and synthesiser surface through an opaque-handle C ABI.
It is not published to crates.io — consumers clone this repo and run:

```
cargo build -p mfsk-ffi --release
```

See `mfsk-ffi/examples/cpp_smoke/` for an end-to-end driver test
(including multi-threaded usage) and `mfsk-ffi/examples/kotlin_jni/`
for an Android/JNI skeleton.

## Status

`0.1.x` — API is deliberately not frozen. Breaking changes follow
cargo-style minor bumps (`0.1 → 0.2`). Algorithm correctness is
covered by ~150 tests across the workspace, including end-to-end
synth → decode roundtrips for every protocol.
