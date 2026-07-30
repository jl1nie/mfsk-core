//! # mfsk-core
//!
//! Pure-Rust library for **WSJT-family digital amateur-radio modes**:
//! FT8, FT4, FST4, WSPR, JT9, JT65 and Q65-30A. Decode, encode, and
//! synthesis in a single crate.
//!
//! ## Why this exists
//!
//! [WSJT-X](https://sourceforge.net/projects/wsjt/) is the reference
//! implementation of these modes and will stay that way — it is
//! battle-tested on the desktop, heavily optimised, and the source of
//! truth for every protocol constant you will find in this crate. But
//! it is also a mixed Fortran / C / Qt application built around a
//! specific desktop workflow. That makes it a poor fit whenever you
//! want to run the decoders *somewhere else*:
//!
//! - in a **browser** as a WASM PWA (the original driver for this
//!   library — a waterfall + sniper-mode decoder that runs in Chrome
//!   / Safari without an install step),
//! - on **Android or iOS** for portable operation, where linking a
//!   Fortran runtime is a non-starter,
//! - in a **headless Rust application** (skimmer, monitoring station,
//!   remote SDR front end) that wants async I/O and safe memory
//!   handling,
//! - on a **handheld embedded controller** like
//!   [`embedded-poc/m5stack-s3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-s3-app)
//!   — a working M5StickS3 FT8 controller (LCD UI, BLE CI-V to
//!   IC-705, acoustic mic capture, QSO FSM) running this library on
//!   Xtensa LX7 + esp-dsp + the Goertzel per-symbol DFT, decoding
//!   real on-air signals in ~1.2 s post-SlotEnd; see
//!   [`docs/reference/MANUAL_M5STICKS3.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/reference/MANUAL_M5STICKS3.md)
//!   for build / flash / operation,
//! - or as the core of a **new protocol experiment** that reuses FT8's
//!   LDPC and sync machinery for a different modulation / FEC /
//!   message recipe.
//!
//! Each of the seven protocols here shares roughly 80 % of its signal
//! path with at least one sibling: 8-GFSK / FSK demodulation, soft-
//! decision LDPC / convolutional / Reed-Solomon / QRA decoding,
//! 77- / 72- / 50-bit WSJT message packing, spectrogram-based sync
//! search. In the Fortran codebase that commonality is expressed by
//! copy-and-paste between per-mode source files; here it is expressed
//! by a small set of traits.
//!
//! ## The abstraction
//!
//! A protocol in this crate is a **zero-sized type** (e.g. [`Ft8`])
//! that implements four traits:
//!
//! - [`ModulationParams`] — tone count, symbol rate, Gray map, GFSK
//!   shaping constants.
//! - [`FrameLayout`] — total symbols, sync / data symbol counts, slot
//!   length, sync-block layout.
//! - [`Protocol`] — the top-level trait, tying the above together
//!   with two associated types: [`Protocol::Fec`] (implementing
//!   [`FecCodec`]) and [`Protocol::Msg`] (implementing
//!   [`MessageCodec`]).
//!
//! Because everything is expressed as `const` associated items + ZSTs,
//! the generic pipeline code — `coarse_sync::<P>`, `decode_frame::<P>`,
//! the LDPC inner loop — is **monomorphised per protocol**. LLVM sees
//! a fully specialised function for each `P`, inlines the constants,
//! and autovectorises the hot loops **on native targets, where SIMD is
//! enabled by default**. The generated machine code is byte-identical
//! to a hand-written per-protocol decoder; the only thing the
//! abstraction costs is longer compile times. On
//! `wasm32-unknown-unknown` this autovectorization requires an
//! explicit `+simd128` build flag — see "WebAssembly builds" below.
//!
//! This pays off most clearly when you add a new protocol. FST4-60A
//! joined the library post-hoc without touching any of the shared
//! sync / DSP / FEC code — the entire implementation is the trait
//! impl block on a single ZST plus a ~50-element Costas pattern
//! table. Similarly, swapping an LDPC codec between two LDPC modes or
//! exposing the same 77-bit message layer to FT8, FT4, and FST4 are
//! one-line changes, not cross-cutting refactors.
//!
//! ## Why Rust
//!
//! - **Safety**: bit-level FEC routines (LDPC belief propagation,
//!   Karn's Berlekamp-Massey + Forney for RS, Fano sequential
//!   decoding) are textbook index-heavy code. Writing them in safe
//!   Rust eliminates an entire class of memory-corruption bugs that
//!   Fortran / C ports have historically hidden.
//! - **Generics + trait bounds**: describing a protocol family as
//!   data + traits is natural. The equivalent in C++ would be template
//!   metaprogramming with subtler error messages; in Fortran, it
//!   simply isn't on offer.
//! - **Targets**: the same code compiles to `wasm32-unknown-unknown`
//!   (WASM SIMD 128-bit via `rustfft`, requires `+simd128` — see
//!   "WebAssembly builds" below), to Android `arm64-v8a` via
//!   the NDK (NEON SIMD), and to any `x86_64-*-unknown` host for
//!   servers — from a single source tree.
//! - **Ecosystem**: `rustfft`, `num-complex`, `crc`, `rayon` are
//!   plug-and-play, so the crate's dependency graph is small and
//!   reviewable.
//!
//! ## WebAssembly builds
//!
//! `wasm32-unknown-unknown` ships with no SIMD by default (unlike
//! x86_64/aarch64 hosts). Since this crate has zero hand-written SIMD
//! by design, every hot loop — plus `rustfft`'s own wasm-SIMD
//! butterfly kernels — depends on LLVM's autovectorizer, which only
//! emits `v128` code on wasm32 when `+simd128` is explicitly enabled.
//! Add to your consuming project's `.cargo/config.toml`:
//!
//! ```toml
//! [target.wasm32-unknown-unknown]
//! rustflags = ["-C", "target-feature=+simd128"]
//! ```
//!
//! Measured ~19-20% FT8 decode speedup from this flag alone; see the
//! README's "Building for WebAssembly" section and
//! `docs/notes/BENCHMARKS.md` for the full numbers and methodology.
//!
//! ## Relationship to WSJT-X
//!
//! Every algorithm in this crate is derived from WSJT-X (Joe Taylor
//! K1JT et al.). Source files cite the corresponding upstream file
//! they port (`lib/ft8/…`, `lib/ft4/…`, `lib/fst4/…`, `lib/wsprd/…`,
//! `lib/jt65_*.f90`, `lib/jt9_*.f90`, `lib/packjt.f90`, etc.).
//! Licensed GPL-3.0-or-later, matching upstream.
//!
//! `mfsk-core` is **not** a replacement for WSJT-X. The goal is to
//! broaden the set of platforms and applications that can host WSJT
//! decoding — WSJT-X on the desktop, `mfsk-core` everywhere else.
//!
//! ## Module layout
//!
//! - [`engine`] — protocol traits, DSP (resample / downsample / GFSK /
//!   subtract), sync, LLR, equaliser, pipeline driver.
//! - [`fec`] — LDPC(174, 91), LDPC(240, 101), convolutional r=½ K=32
//!   Fano, Reed-Solomon(63, 12) over GF(2⁶), and the QRA(15, 65)
//!   over GF(2⁶) Q-ary RA codec used by Q65 (belief-propagation
//!   decoder via Walsh-Hadamard messages).
//! - [`msg`] — 77-bit WSJT, 72-bit JT, 50-bit WSPR and Q65 message
//!   codecs + callsign hash table.
//! - [`ft8`] / [`ft4`] / [`fst4`] / [`wspr`] / [`jt9`] / [`jt65`] /
//!   [`q65`] — per-protocol ZSTs, decoders and synthesisers. Each is
//!   gated behind a feature of the same name.
//!
//! ## Feature flags
//!
//! | Feature       | Default? | What it enables                              |
//! |---------------|----------|----------------------------------------------|
//! | `ft8`         | yes      | FT8 (15 s, 8-GFSK, LDPC(174,91))             |
//! | `ft4`         | yes      | FT4 (7.5 s, 4-GFSK, LDPC(174,91))            |
//! | `fst4`        |          | FST4-60A (60 s, 4-GFSK, LDPC(240,101))       |
//! | `wspr`        |          | WSPR (120 s, 4-FSK, conv r=½ K=32 + Fano)    |
//! | `jt9`         |          | JT9 (60 s, 9-FSK, conv r=½ K=32 + Fano)      |
//! | `jt65`        |          | JT65 (60 s, 65-FSK, RS(63,12))               |
//! | `q65`         |          | Q65-30A + Q65-60A‥E (65-FSK, QRA(15,65) GF(64)) |
//! | `full`        |          | Aggregate of all seven protocols + uvpacket + packet-bytes |
//! | `parallel`    | yes      | Rayon-parallel candidate processing          |
//! | `fft-rustfft` | yes      | Default host FFT backend (`rustfft`, requires `std`) |
//! | `fft-extern`  |          | Pluggable FFT trait — caller binary supplies an `FftPlanner` impl |
//! | `fixed-point` |          | Embedded integer pipeline: u16 spec + i16 DFT + Q11i16 LLR + integer NMS BP |
//! | `profile-coarse` |       | Always-on coarse_sync sub-stage profiling               |
//!
//! ## Runtime registry
//!
//! [`PROTOCOLS`] is a `&'static [ProtocolMeta]` listing every
//! `Protocol` impl wired into the current build. Each entry carries
//! the protocol's id, display name, and every constant the trait
//! surface exposes (modulation / frame / FEC / message). Use it
//! when a UI layer or FFI bridge needs to enumerate "what does this
//! build support?" without hardcoding its own list:
//!
//! ```
//! # use mfsk_core::PROTOCOLS;
//! for p in PROTOCOLS {
//!     println!("{}: {} tones × {} bits, {} s slot",
//!              p.name, p.ntones, p.bits_per_symbol, p.t_slot_s);
//! }
//! ```
//!
//! [`by_id`] / [`by_name`] / [`for_protocol_id`] cover the common
//! lookup patterns. All six Q65 sub-modes (Q65-30A, Q65-60A‥E)
//! appear as distinct registry entries because their NSPS and tone
//! spacing differ; they share `ProtocolId::Q65` because the FFI
//! protocol tag is family-level.
//!
//! ## Trait surface verification
//!
//! `tests/protocol_invariants.rs` runs a single generic
//! `assert_protocol_invariants::<P: Protocol>` over every wired ZST
//! (FT8 / FT4 / FST4 / WSPR / JT9 / JT65 plus all six Q65 sub-modes
//! — 11 in total; uvpacket adds four more under `--features uvpacket`).
//! It pins ~25 trait-level invariants split across three layers:
//! modulation (`2^BITS_PER_SYMBOL ≤ NTONES`, `SYMBOL_DT × 12000 ==
//! NSPS`, GRAY_MAP coverage and uniqueness, GFSK / tone-spacing
//! positivity), frame layout (`N_SYMBOLS == N_DATA + N_SYNC`, sync
//! pattern in-range, `T_SLOT_S > 0`), and codec consistency
//! (`FecCodec::K ≥ MessageCodec::PAYLOAD_BITS`, `FecCodec::N ≤
//! N_DATA × BITS_PER_SYMBOL`). Adding a new `Protocol` impl is a
//! one-line registry edit + a one-line test invocation; the same
//! generic body proves the new ZST's constants are internally
//! consistent without any per-protocol glue. Drift between trait
//! doc and implementation is caught mechanically — the work that
//! landed Q65 surfaced one such discrepancy in `GRAY_MAP` and fixed
//! it in the same pass.
//!
//! The default features (`ft8`, `ft4`) only exercise the two default
//! protocols; run `cargo test --features full` (or enable a specific
//! protocol feature) to cover the rest. The registry size and
//! `ProtocolId` uniqueness checks adapt to whatever feature
//! combination is active.
//!
//! ## Library stack
//!
//! ```text
//!          ┌─────────────────────────────────────────────────────┐
//!          │      ft8   ft4   fst4   wspr   jt9   jt65   …       │  per-protocol ZSTs
//!          │        (each implements Protocol + FrameLayout)      │  (feature-gated)
//!          └─────────────┬─────────────────┬─────────────────────┘
//!                        │                 │
//!               ┌────────▼────────┐  ┌─────▼──────┐
//!               │       msg       │  │    fec     │  shared codecs
//!               │  Wsjt77 · Jt72  │  │ LDPC · RS  │  behind traits
//!               │  Wspr50  · Hash │  │ ConvFano   │
//!               └────────┬────────┘  └─────┬──────┘
//!                        │                 │
//!                    ┌───▼─────────────────▼───┐
//!                    │          core           │  Protocol trait, DSP
//!                    │ sync · llr · equalize · │  (resample / GFSK /
//!                    │  pipeline · tx · dsp    │   downsample / subtract)
//!                    └─────────────────────────┘
//! ```
//!
//! Each protocol declares its slot length, tone count, Gray map,
//! Costas / sync pattern, FEC codec and message codec at compile time
//! via the [`Protocol`] trait. The generic code in [`engine`] —
//! coarse sync, fine sync, LLR computation, LDPC / RS / convolutional
//! decode, GFSK synthesis — works for any type that satisfies the
//! trait.
//!
//! ## Quick start
//!
//! ```toml
//! # Cargo.toml
//! [dependencies]
//! mfsk-core = { version = "0.8", features = ["ft8", "ft4"] }
//! ```
//!
//! Round-trip a synthesised FT8 frame through the decoder:
//!
//! ```
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::ft8::{
//!     Ft8,
//!     wave_gen::{message_to_tones, tones_to_i16},
//! };
//! use mfsk_core::msg::decode_request::DecodeRequest;
//! use mfsk_core::msg::wsjt77::{pack77, unpack77};
//!
//! // 1. Pack a standard FT8 message and synthesise 12 kHz i16 PCM.
//! //    The synth produces just the transmitted frame (~12.64 s);
//! //    pad to the full 15 s slot with the signal starting at 0.5 s.
//! let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack");
//! let tones = message_to_tones(&msg77);
//! let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);
//!
//! let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
//! let start = (0.5 * 12_000.0) as usize;
//! for (i, &s) in frame.iter().enumerate() {
//!     if start + i < audio.len() { audio[start + i] = s; }
//! }
//!
//! // 2. Decode it back across the full FT8 band.
//! let results = DecodeRequest::<Ft8>::new(
//!     &audio,
//!     /* freq_min */ 100.0,
//!     /* freq_max */ 3_000.0,
//!     /* sync_min */ 1.0,
//!     /* max_cand */ 50,
//! )
//! .decode()
//! .results;
//! assert!(!results.is_empty(), "roundtrip must decode");
//! let text = unpack77(results[0].message77()).expect("unpack");
//! assert_eq!(text, "CQ JA1ABC PM95");
//! # }
//! ```
//!
//! ## `no_std` usage
//!
//! ```toml
//! # Cargo.toml — TX-only, no_std + alloc, no FFT backend needed
//! [dependencies]
//! mfsk-core = { version = "0.8", default-features = false, features = ["alloc", "ft8"] }
//! ```
//!
//! Encoding (`message_to_tones` / `tones_to_i16`) never touches `std` — no
//! FFT, no heap-backed collections beyond `alloc::vec::Vec`. This is the
//! same call as the [`ft8::wave_gen`] encoder-only example above; the
//! `alloc ft8` and `alloc ft8 fft-extern` legs of CI's feature-matrix build
//! this exact combination (`.github/workflows/ci.yml`) to confirm it
//! compiles under `#![no_std]`. Decoding additionally needs a
//! [`engine::fft::FftPlanner`] impl — bring your own via `fft-extern` (the
//! embedded ports use this for esp-dsp / CMSIS-DSP) since `fft-rustfft`
//! requires `std`.
//!
//! ```
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
//! use mfsk_core::msg::wsjt77::pack77;
//!
//! let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack");
//! let tones = message_to_tones(&msg77);
//! let pcm = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);
//! assert!(!pcm.is_empty());
//! # }
//! ```

// Several clippy lints fight with the style of this crate:
//
// - `too_many_arguments` triggers on inner FEC / DSP helpers that are
//   one-to-one ports of Fortran subroutines; splitting them into
//   "smaller" functions would just obscure the correspondence with
//   the upstream algorithm.
// - `needless_range_loop` flags `for i in 0..N` loops that index into
//   fixed-size arrays. Algorithmic code ported from WSJT-X reads more
//   clearly with the index variable in scope (sync pattern iteration,
//   LDPC check-node passes, Reed-Solomon syndrome computation), so
//   the .iter().enumerate() form is not always an improvement.
// - `unusual_byte_groupings` trips on magic constants where the digit
//   grouping encodes a bit-layout meaning (WSPR bit-reversal constants,
//   LDPC generator polynomial byte boundaries). Normalising the
//   grouping would obscure the intent.
#![allow(
    clippy::too_many_arguments,
    clippy::needless_range_loop,
    clippy::unusual_byte_groupings
)]
// `no_std` build is gated on the absence of the default `std` feature.
// `alloc` is unconditional — every protocol module uses Vec / String,
// so making it optional would just push the dep up to every caller.
// (The `alloc` Cargo feature still exists as a no-op alias kept around
// for back-compat with consumers that listed it explicitly.)
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

/// Crate version string, taken from Cargo.toml at compile time. Useful
/// for FFI / WASM consumers that need to verify which mfsk-core they
/// actually linked against (e.g. through a [patch.crates-io] path
/// override that didn't get re-fingerprinted).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

pub mod engine;
pub mod fec;
pub mod msg;

#[cfg(feature = "ft8")]
pub mod ft8;

#[cfg(feature = "ft4")]
pub mod ft4;

#[cfg(feature = "fst4")]
pub mod fst4;

#[cfg(feature = "wspr")]
pub mod wspr;

#[cfg(feature = "jt9")]
pub mod jt9;

#[cfg(feature = "jt65")]
pub mod jt65;

#[cfg(feature = "q65")]
pub mod q65;

#[cfg(feature = "uvpacket")]
pub mod uvpacket;

#[cfg(feature = "msk144")]
pub mod msk144;

pub mod registry;

// Flatten commonly-used types to the crate root.
pub use crate::engine::{
    DecodeContext, FecCodec, FecOpts, FecResult, FrameLayout, MessageCodec, MessageFields,
    ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};
pub use crate::registry::{PROTOCOLS, ProtocolMeta, by_id, by_name, for_protocol_id};

#[cfg(feature = "fst4")]
pub use crate::fst4::Fst4s60;
#[cfg(feature = "ft4")]
pub use crate::ft4::Ft4;
#[cfg(feature = "ft8")]
pub use crate::ft8::Ft8;
#[cfg(feature = "jt9")]
pub use crate::jt9::Jt9;
#[cfg(feature = "jt65")]
pub use crate::jt65::Jt65;
#[cfg(feature = "q65")]
pub use crate::q65::Q65a30;
#[cfg(feature = "uvpacket")]
pub use crate::uvpacket::{UvExpress, UvRobust, UvStandard, UvUltraRobust};
#[cfg(feature = "wspr")]
pub use crate::wspr::Wspr;

// Markdown-doc doctest hooks: pulls each file's fenced `rust` code
// blocks into `cargo test --doc` so a renamed/removed API breaks the
// build here instead of silently rotting in prose. `#[cfg(doctest)]`
// keeps these out of normal builds and the generated docs.rs page;
// non-`rust`-tagged fences (shell commands, ASCII diagrams) must be
// tagged accordingly in the source file or rustdoc treats them as
// Rust too.
#[doc = include_str!("../../README.md")]
#[cfg(doctest)]
struct MarkdownDoctestsReadme;

#[doc = include_str!("../../docs/reference/LIBRARY.md")]
#[cfg(doctest)]
struct MarkdownDoctestsLibrary;

#[doc = include_str!("../../docs/reference/LIBRARY.ja.md")]
#[cfg(doctest)]
struct MarkdownDoctestsLibraryJa;

#[doc = include_str!("../../docs/reference/EMBEDDED.md")]
#[cfg(doctest)]
struct MarkdownDoctestsEmbedded;

#[doc = include_str!("../../docs/reference/EMBEDDED.ja.md")]
#[cfg(doctest)]
struct MarkdownDoctestsEmbeddedJa;

#[doc = include_str!("../../docs/reference/UVPACKET.md")]
#[cfg(doctest)]
struct MarkdownDoctestsUvpacket;

#[doc = include_str!("../../docs/reference/UVPACKET.ja.md")]
#[cfg(doctest)]
struct MarkdownDoctestsUvpacketJa;

#[doc = include_str!("../../docs/reference/MANUAL_M5STICKS3.md")]
#[cfg(doctest)]
struct MarkdownDoctestsManualM5StickS3;

#[doc = include_str!("../../docs/reference/MANUAL_M5STICKS3.ja.md")]
#[cfg(doctest)]
struct MarkdownDoctestsManualM5StickS3Ja;
