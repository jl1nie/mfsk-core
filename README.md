# mfsk-core

<p align="center">
  <img src="https://raw.githubusercontent.com/jl1nie/mfsk-core/main/docs/assets/m5sticks3-ft8-decode.jpg"
       alt="M5StickS3 running embedded-poc/m5stack-s3-app — five real on-air FT8 decodes from a single 15 s slot, IDLE FSM waiting for the operator to pick a callsign"
       width="360">
</p>

<p align="center"><i>M5StickS3 running <code>embedded-poc/m5stack-s3-app</code> — five real on-air FT8 decodes from a single 15 s slot, QSO FSM idle waiting for the operator to pick a callsign. See <a href="https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/MANUAL_M5STICKS3.md">docs/reference/MANUAL_M5STICKS3.md</a>.</i></p>

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

## Why mfsk-core

- **At or near WSJT-X sensitivity parity on every mode.**
  FST4 is within 0.1-0.6 dB of WSJT-X's published thresholds across
  all five sub-modes; FT4's AWGN gap is ~0.3 dB; MSK144 matches a real
  WSJT-X `jt9` build on 25/28 AWGN cross-check cells exactly; FT8
  matches the WSJT-X golden set 8/8 and JTDX's 18/18; WSPR
  and JT9 are 8/8 and 7/7 on their WSJT-X reference recordings. JT65's
  own long-disclosed ~7-8 dB gap vs. WSJT-X's stochastic `ftrsdap`
  decoder was closed 2026-08-08
  ([#169](https://github.com/jl1nie/mfsk-core/issues/169)): a faithful
  port of `ftrsdap` itself (`jt65::decode_at_with_chase`, magic numbers
  included) plus an FFT bin-alignment fix that turned out to be the
  bigger factor (affecting every JT65 decode path, not just the new
  one). Full numbers, per protocol, including the honest caveats on
  the WSJT-X comparison methodology:
  [`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md).
- **Runs where WSJT-X can't.** Same algorithms, `no_std`-portable:
  a real shipping product
  ([`embedded-poc/m5stack-s3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-s3-app/),
  pictured above) decodes real on-air FT8 in ~1.2 s post-slot on an
  ESP32-S3, plus WASM in the browser and Android/iOS via FFI — none
  of which a Fortran/C/Qt desktop application can target.
- **A `Protocol` trait, not per-mode copy-paste.** Eight protocol
  families share one generic, monomorphised decode pipeline (no
  vtable, no dynamic dispatch on the hot path) — adding FST4-60A to
  the crate was a trait impl on one ZST, not a cross-cutting refactor.
  See [Design Philosophy](#design-philosophy).

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
| MSK144     | 15 s   | LDPC(128, 90) + CRC-13            | 77 bit  | Meteor-ping burst-scan (matched filter) | `msk144` |

Eight protocol families, sixteen wired `Protocol`-trait ZSTs in the
registry: FST4 contributes five T/R-period sub-modes (FST4-15, -30,
-60A, -120, -300) and Q65 contributes one 30-s sub-mode (Q65-30A) plus
five 60-s EME sub-modes (Q65-60A‥E) — both families share FEC, message
codec and sync layout across their sub-modes, differing only in NSPS /
tone spacing (and, for FST4-15 alone, the T/R start offset). **MSK144
is the exception**: its continuous-phase binary-MSK modulation and
transient-burst timing don't fit the static-slot model every other
protocol here shares, so no ZST implements `Protocol` for it — its own
`msk144::decode::decode_slot` driver bypasses `engine::pipeline`
entirely by design (see
[`docs/reference/LIBRARY.md` §0.5](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/LIBRARY.md#05-generic-vs-bespoke-per-protocol)
for why — see the MSK144 row and its footnote in the generic-vs-bespoke
table).
[`PROTOCOLS`](https://docs.rs/mfsk-core/latest/mfsk_core/static.PROTOCOLS.html)
exposes one entry per wired ZST (so MSK144 doesn't appear there);
`uvpacket` (when enabled) adds four more for its rate ladder. See
[Static set of protocols](#static-set-of-protocols) for why there's no
runtime `register_protocol()`.

## Quick Start

```toml
# Cargo.toml
[dependencies]
mfsk-core = { version = "0.8", features = ["ft8", "ft4"] }
```

New features and fixes land on `main` immediately as PRs merge, but
crates.io releases are cut on a throttled cadence (see
[Status](#status)) — if you want a specific fix or new mode before it
ships to crates.io, point at the git repo instead:

```toml
mfsk-core = { git = "https://github.com/jl1nie/mfsk-core", branch = "main", features = ["ft8", "ft4"] }
```

Synthesise an FT8 frame and decode it back:

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
let end = (start + frame.len()).min(audio.len());
audio[start..end].copy_from_slice(&frame[..end - start]);

// 2. Decode it back.
let results = DecodeRequest::<Ft8>::new(
    &audio,
    /* freq_min */ 100.0,
    /* freq_max */ 3_000.0,
    /* sync_min */ 1.0,
    /* max_cand */ 50,
)
.decode()
.results;
for r in &results {
    if let Some(text) = unpack77(r.message77()) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

That's the whole round trip: pack a message → synthesise 12 kHz PCM →
decode it back. Each protocol module documents its own top-level entry
points and carries its own Quick example:

- [`mfsk_core::ft8`](https://docs.rs/mfsk-core/latest/mfsk_core/ft8/)
  — `DecodeRequest::<Ft8>` (wide-band) + `SniperRequest::<Ft8>`
  (narrow-band "sniper" mode)
- [`mfsk_core::ft4`](https://docs.rs/mfsk-core/latest/mfsk_core/ft4/)
  — `DecodeRequest::<Ft4>`
- [`mfsk_core::fst4`](https://docs.rs/mfsk-core/latest/mfsk_core/fst4/)
  — `DecodeRequest::<Fst4s60>` (FST4-60A); other sub-modes via
  `DecodeRequest::<Fst4s120>` etc.
- [`mfsk_core::wspr`](https://docs.rs/mfsk-core/latest/mfsk_core/wspr/)
  — `decode::decode_scan_default`
- [`mfsk_core::jt9`](https://docs.rs/mfsk-core/latest/mfsk_core/jt9/)
  — `decode_scan_default`
- [`mfsk_core::jt65`](https://docs.rs/mfsk-core/latest/mfsk_core/jt65/)
  — `decode_scan_default` + `decode_at_with_erasures` (for low SNR)
- [`mfsk_core::q65`](https://docs.rs/mfsk-core/latest/mfsk_core/q65/)
  — `DecodeRequest::<P>` (wide-band scan) / `SniperRequest::<P>`
  (narrow-band, known alignment) for any wired sub-mode including the
  Q65-60A‥E EME variants; `.ap_hint(...)` for AP-hint decoding (~2 dB
  threshold gain when call signs are known); `.fading(model, b90_ts)`
  for the fast-fading metric (Gaussian / Lorentzian channel models)
  that recovers 5–8 dB on Doppler-spread channels — required for
  microwave EME at 5.7 / 10 / 24 GHz; `.ap_list(candidates)` (paired
  with `standard_qso_codewords`) for BP-free template matching against
  the full WSJT-X "AP list" of standard exchanges (~3 dB threshold
  gain when the callsign pair is known up-front); and
  `MultiPeriodRequest::<P>` for averaged multi-slot decode
  (ionoscatter / weak-EME signals no single-period decode recovers).
  Q65's own dedicated builders — unlike FT8/FT4/FST4's
  `msg::decode_request::{DecodeRequest, SniperRequest}` — since every
  `q65::rx` function operates on `&[f32]` audio, not `&[i16]` (issue
  #204)

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
| `fixed-point` |         | Embedded integer pipeline: u16 spectrogram + i16 DFT + Q11i16 LLR + integer NMS BP — see [`docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md#q-format-quick-reference) for the Q-format reference |
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
in code (a `Protocol` trait, not per-mode copy-paste),
[`docs/reference/LIBRARY.md` §0.5](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/LIBRARY.md#05-generic-vs-bespoke-per-protocol)
for a per-protocol table of exactly which boxes are shared vs. bespoke,
and
[`docs/reference/LIBRARY.md` §4](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/LIBRARY.md#4-shared-primitives-core)
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

- **Shared** (lives in `engine`, generic over any `P: Protocol`): coarse
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
autovectorises the hot loops **on native targets, where SIMD is enabled
by default** (SSE/AVX on x86_64, NEON on aarch64). The generated machine
code is byte-identical to a hand-written per-protocol decoder; the
receive path is a chain of free functions in `engine::sync` →
`engine::llr` → `engine::equalize` → `engine::pipeline` (each generic
over `P: Protocol`), not a `Demodulator` / `Receiver` trait object —
there is no vtable, no dynamic dispatch, on the hot path. On
`wasm32-unknown-unknown` this autovectorization requires an explicit
build flag — see [Building for WebAssembly](#building-for-webassembly)
below.

This pays off most clearly when adding a new protocol: it's a trait
impl on a ZST, not a cross-cutting refactor. FST4-60A joined the crate
post-hoc without changing any shared pipeline code — the entire
implementation is the trait impl block on a single ZST plus a
Costas pattern table. Similarly, swapping an LDPC codec between two
LDPC modes, or exposing the same 77-bit message layer to FT8, FT4 and
FST4, are one-line changes, not cross-cutting refactors.

### Building for WebAssembly

Unlike `x86_64`/`aarch64` hosts, where SSE/AVX or NEON are enabled by
default, `wasm32-unknown-unknown` ships with **no SIMD by default**.
mfsk-core has zero hand-written SIMD (by design — it keeps the crate
`no_std`/portable), so every hot loop depends entirely on LLVM's
autovectorizer, which on wasm32 only emits `v128` instructions when
`+simd128` is explicitly enabled. This also gates `rustfft` (the
default FFT backend)'s own wasm-SIMD butterfly kernels. Without the
flag, both mfsk-core's DSP and the FFT backend run fully scalar.

Add this to your **consuming project's** `.cargo/config.toml` (this is
a build flag for the binary/wasm-bindgen crate that embeds mfsk-core,
not something the library itself can impose on downstream builds):

```toml
[target.wasm32-unknown-unknown]
rustflags = ["-C", "target-feature=+simd128"]
```

Measured effect (Node, `--target nodejs`, real `decode_wav()` FT8
decode calls, median of 5-7 runs per config, same input, steady-state;
methodology and a reproducible harness in
[`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md)):

| WAV | without `+simd128` | with `+simd128` | speedup |
|---|---:|---:|---:|
| sim_busy_band.wav | 194.2 ms | 156.5 ms | 19.4% |
| sim_extreme_hard.wav | 216.5 ms | 173.1 ms | 20.0% |

Browser support for wasm SIMD has been universal since ~2021
(Chrome/Firefox/Safari 16.4+); in practice there's no reason not to
set this flag for a wasm build.

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
  (WASM SIMD 128-bit via `rustfft`, requires `+simd128` — see
  [Building for WebAssembly](#building-for-webassembly) above), to
  Android `arm64-v8a` via the NDK (NEON SIMD), to `no_std + alloc`
  embedded MCUs, and to any `x86_64-*-unknown` host for servers — from
  a single source tree.
- **Ecosystem**: `rustfft`, `num-complex`, `crc`, `rayon` are
  plug-and-play, so the crate's dependency graph is small and
  reviewable.

## Benchmarks vs. WSJT-X

Not a competitor — a different point in the design space. WSJT-X is
the reference implementation and the source of truth for every
protocol constant in this crate; `mfsk-core` exists to run the same
algorithms in places a Fortran/C/Qt desktop application can't reach.

| | WSJT-X | mfsk-core |
|---|---|---|
| Language | Fortran + C + Qt | Rust |
| Distribution | Desktop application | Library crate (`cargo add`) |
| `no_std` / embedded | ✗ | ✓ (ESP32-S3, RP2350, Cortex-M) |
| WASM | ✗ | ✓ (`wasm32-unknown-unknown`, [`+simd128`](#building-for-webassembly) recommended) |
| Android / iOS | ✗ | ✓ (NDK / FFI) |
| FFT backend | fixed (FFTW) | pluggable (`rustfft` or caller-supplied, e.g. esp-dsp/CMSIS-DSP) |
| Reference implementation | ✓ | derived from WSJT-X, cites source per file |

Headline decode numbers (full per-protocol writeup, including how each
sweep was generated and reproduced, in
[`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md)):

| Protocol | Golden-WAV recall | AWGN gap vs. WSJT-X |
|----------|-------------------|----------------------|
| FT8      | 8/8 host full-parity (WSJT-X), 18/18 (JTDX) | CCIR fading gap closed |
| FT4      | 6/6 | ~0.3 dB |
| FST4     | 1/1 (FST4-60A) | 0.10-0.60 dB across 5 sub-modes |
| WSPR     | 8/8 | matches published sensitivity floor |
| JT9      | 7/7 | no measurable gap |
| JT65     | none available | ~0 dB (2026-08-08, #169: faithful `ftrsdap` port + FFT bin-alignment fix — see BENCHMARKS.md for comparison caveats) |
| Q65      | 2 real EME recordings | matches WSJT-X with AP hint; 2 sub-modes measurably beat WSJT-X's own plain decode |
| MSK144   | 3/3 (incl. exact SNR match) | 25/28 cells exact match vs. a real `jt9` build |

Embedded wall-clock: M5StickS3 (Xtensa LX7, fixed-point) decodes a
real on-air busy-band FT8 slot in **~1.19 s post-SlotEnd** via the
streaming pipeline (FFT overlapped with capture); M5Stack Core2
(Xtensa LX6) on the same recording: ~2.8 s. See
[`docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md)
for the byte-level BP-scratch / spectrogram memory budget.

## FAQ

**How does this differ from WSJT-X?** See
[Benchmarks vs. WSJT-X](#benchmarks-vs-wsjt-x) and
[Why this exists](#why-this-exists) above — same algorithms, different
deployment targets (library vs desktop app, `no_std` embedded, WASM).

**Does it support `no_std`?** Yes — `default-features = false, features
= ["alloc", "ft8", "fft-extern"]` (or similar) builds without `std`.
`std` is only required by the default `fft-rustfft` backend; embedded
targets swap in their own FFT via `fft-extern`. See
[`docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md).

**Can I swap the FFT backend?** Yes — enable `fft-extern` instead of
`fft-rustfft` and provide an `FftPlanner` impl (`engine::fft`); the
embedded ports use this for esp-dsp (ESP32-S3) and CMSIS-DSP (RP2350).

**How do I use this on embedded hardware?** Start with
[`docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md)
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
[`docs/reference/UVPACKET.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/UVPACKET.md)
([日本語](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/UVPACKET.ja.md))
for the design narrative, modulation / equaliser / framing details
and per-mode performance characterisation.

## Modules

- `mfsk_core::engine` — protocol traits, DSP (resample / downsample /
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
  `synthesize_standard_for<P>` helper plus the
  `DecodeRequest<P>`/`SniperRequest<P>` builders that pick the right
  NSPS and tone spacing from the type parameter.

## C / C++ / Kotlin

The `mfsk-ffi` sibling crate in this repository builds a
`libmfsk.{so,a,dylib}` + `mfsk.h` (via `cbindgen`) that exposes the
same decoder and synthesiser surface through an opaque-handle C ABI.
`mfsk-ffi` is not published to crates.io, but every tagged release
attaches a prebuilt `linux-x86_64` tarball to the GitHub Release;
other platforms/ABIs (including Android) build locally:

```sh
cargo build -p mfsk-ffi --release
```

See `mfsk-ffi/examples/cpp_smoke/` for an end-to-end driver test
(including multi-threaded usage) and `mfsk-ffi/examples/kotlin_jni/`
for an Android/JNI skeleton. Embedded targets (ESP32-S3, RP2350,
Cortex-M) instead use the sibling `mfsk-ffi-ft8` crate — see its own
prebuilt binaries below.

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
  preset, `cargo test` + the C++ driver for `mfsk-ffi` and `cargo test`
  for `mfsk-ffi-ft8`, rustdoc with `-D warnings`, and a
  `cargo publish --dry-run` for `mfsk-core`.
- **Release**: tag-driven (`v0.6.x`). Pushing a tag that matches the
  workspace version (`Cargo.toml::[workspace.package].version`,
  inherited by `mfsk-core`/`mfsk-ffi`/`mfsk-ffi-ft8` alike) and is
  reachable from `main` triggers `release.yml`, which publishes
  `mfsk-core` to crates.io and cuts a GitHub release with
  auto-generated notes. Prebuilt `mfsk-ffi` (linux-x86_64) and
  `mfsk-ffi-ft8` (linux-x86_64, esp32-xtensa, esp32s3-xtensa)
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
     "docs/reference/LIBRARY.md" to mfsk-core/docs/... — see workspace
     layout: docs/ lives at the repo root, not under the crate). -->
- **English:** [`docs/reference/LIBRARY.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/LIBRARY.md)
- **日本語:** [`docs/reference/LIBRARY.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/LIBRARY.ja.md)
- **Streaming decode interface (`.on_result` + async bridge):**
  [English `docs/reference/STREAMING.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/STREAMING.md)
  / [日本語 `docs/reference/STREAMING.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/STREAMING.ja.md)
  — per-protocol streaming entry points, the delivery contract
  (sequential exact-match vs. parallel completion-order), why it's a
  synchronous callback rather than `async`/Tokio/a channel, and a
  complete worked example of driving it from a Tokio async client.
- **Benchmarks vs. WSJT-X, full per-protocol detail:**
  [`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md)
  — golden-WAV recall + AWGN sensitivity sweep results for every
  protocol, current-state only (see `CHANGELOG.md` for how each
  number got there).
- **Embedded targets:**
  [English `docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md)
  / [日本語 `docs/reference/EMBEDDED.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.ja.md)
  — generic-scalar architecture (one codebase for f32 host and
  fixed-point embedded), feature-flag map, FFT-extern contract,
  Goertzel per-symbol DFT (zero-scratch, 0.6.4+) with BASIS
  deprecation, Q-format reference, full `mfsk-ffi-ft8` C ABI
  tutorial (streaming + ESP-IDF component layout), performance
  benchmark, streaming RX pipeline, binary footprint.
- **FST4 sensitivity benchmark setup:**
  [English `docs/notes/FST4_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FST4_BENCHMARK.md)
  / [日本語 `docs/notes/FST4_BENCHMARK.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FST4_BENCHMARK.ja.md)
  — reproducing the `fst4sim`-driven AWGN/fading SNR sweep
  (`tests/fst4_sweep.rs`) from a clean checkout on any machine:
  prerequisites, building `fst4sim` from WSJT-X source, generating
  the WAV corpus, and how to avoid grid-censoring artifacts when
  reading off the recall crossing.
- **MSK144 sensitivity benchmark setup:**
  [English `docs/notes/MSK144_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/MSK144_BENCHMARK.md)
  / [日本語 `docs/notes/MSK144_BENCHMARK.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/MSK144_BENCHMARK.ja.md)
  — the self-contained `tests/msk144_snr_sweep.rs` regression sweep
  (no WSJT-X checkout needed), plus how to reproduce the one-time
  apples-to-apples verification against a real WSJT-X `jt9` build
  (building `jt9`/`msk144sim`/Hamlib from source, `msk144sim`'s SNR
  convention, and the measured baseline results).
- **M5StickS3 FT8 controller manual:**
  [English `docs/reference/MANUAL_M5STICKS3.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/MANUAL_M5STICKS3.md)
  / [日本語 `docs/reference/MANUAL_M5STICKS3.ja.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/MANUAL_M5STICKS3.ja.md)
  — build / flash / `cfg.toml` / `BootMode` cycle / UI / QSO
  workflow / troubleshooting.

## Status

**Latest published tag: `v0.9.0`** — API is deliberately not frozen:
breaking changes follow cargo-style minor bumps, while a new
protocol/mode addition on its own is patch-level (e.g. MSK144 shipped
as `0.7.4`, not `0.8.0`) — minor bumps mark more structural changes.
See `CHANGELOG.md` for the per-release breakdown and
`docs/notes/ROADMAP.md` for open follow-ups.

**Release cadence**: PRs merge to `main` continuously — CHANGELOG.md's
top section always reflects the latest unreleased state, so tracking
`main` directly (see [Quick Start](#quick-start)) gets you every
change immediately. Actual crates.io tags/GitHub Releases are cut on a
**biweekly** cadence (bundling everything merged since the last tag)
rather than after every individual change, to keep update
notifications for crates.io consumers from firing too often — an
out-of-cadence release is still fine for a security fix, a serious
correctness bug, or on explicit request.

Algorithm correctness is covered by the workspace test suite:
end-to-end synth → decode roundtrips for every protocol, real
WSJT-X-distributed reference recordings, `*sim`-generated AWGN
sensitivity sweeps, and AP-list / fast-fading comparisons — see
[Benchmarks vs. WSJT-X](#benchmarks-vs-wsjt-x) above and
[`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md)
for the full results. The trait surface itself is pinned by
`tests/protocol_invariants.rs` — a single generic `<P: Protocol>`
checker run across every wired ZST. Run with `--features full` for
full coverage; the default features (`ft8`, `ft4`) only exercise the
two default protocols.

`embedded-poc/m5stack-s3-app/` (demo / acoustic-fallback, since the
StickS3 board can't do USB host) and `embedded-poc/m5stack-core2-app/`
(wav_sim-only LX6 sibling) are production FT8 controller crates (LCD
UI + QSO FSM + WiFi-UDP log streaming), both consuming the
board-agnostic `embedded-poc/mfsk-app-shared/`.
`embedded-poc/m5stack-cores3-app/` is the **main UAC controller
target** (M5Stack CoreS3 has the PMIC/IO-expander wiring StickS3
lacks) — board bring-up and UAC host code are shipped, but live
IC-705 hardware verification hasn't happened yet (issue #163). See
`docs/notes/ROADMAP.md`'s Phase B-Core section for the current
status. `embedded-poc/m5stack-s3/` is a
decoder-only compute-bench crate for S3 timing-regression tracking.
See
[`docs/reference/EMBEDDED.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/EMBEDDED.md)
for the integration contract and runtime tuning knobs, and
[`docs/reference/MANUAL_M5STICKS3.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/MANUAL_M5STICKS3.md)
for the M5StickS3 controller's build/flash/UI workflow.
