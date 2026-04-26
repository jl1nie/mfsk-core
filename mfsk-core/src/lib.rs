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
//! and autovectorises the hot loops. The generated machine code is
//! byte-identical to a hand-written per-protocol decoder; the only
//! thing the abstraction costs is longer compile times.
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
//!   (WASM SIMD 128-bit via `rustfft`), to Android `arm64-v8a` via
//!   the NDK (NEON SIMD), and to any `x86_64-*-unknown` host for
//!   servers — from a single source tree.
//! - **Ecosystem**: `rustfft`, `num-complex`, `crc`, `rayon` are
//!   plug-and-play, so the crate's dependency graph is small and
//!   reviewable.
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
//! - [`core`] — protocol traits, DSP (resample / downsample / GFSK /
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
//! | `full`        |          | Aggregate of all seven protocols             |
//! | `parallel`    | yes      | Rayon-parallel candidate processing          |
//! | `osd-deep`    |          | OSD-3 fallback on AP decodes (extra CPU)     |
//! | `eq-fallback` |          | Non-EQ fallback inside `EqMode::Adaptive`    |
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
//! via the [`Protocol`] trait. The generic code in [`core`] —
//! coarse sync, fine sync, LLR computation, LDPC / RS / convolutional
//! decode, GFSK synthesis — works for any type that satisfies the
//! trait.
//!
//! ## Quick start
//!
//! ```toml
//! # Cargo.toml
//! [dependencies]
//! mfsk-core = { version = "0.1", features = ["ft8", "ft4"] }
//! ```
//!
//! Round-trip a synthesised FT8 frame through the decoder:
//!
//! ```
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::ft8::{
//!     decode::{decode_frame, DecodeDepth},
//!     wave_gen::{message_to_tones, tones_to_i16},
//! };
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
//! let results = decode_frame(
//!     &audio,
//!     /* freq_min */ 100.0,
//!     /* freq_max */ 3_000.0,
//!     /* sync_min */ 1.0,
//!     /* freq_hint */ None,
//!     DecodeDepth::BpAllOsd,
//!     /* max_cand */ 50,
//! );
//! assert!(!results.is_empty(), "roundtrip must decode");
//! let text = unpack77(&results[0].message77).expect("unpack");
//! assert_eq!(text, "CQ JA1ABC PM95");
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

pub mod core;
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

pub mod registry;

// Flatten commonly-used types to the crate root.
pub use crate::core::{
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
#[cfg(feature = "wspr")]
pub use crate::wspr::Wspr;
