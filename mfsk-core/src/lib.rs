//! # mfsk-core
//!
//! Pure-Rust library for **WSJT-family digital amateur-radio modes**:
//! FT8, FT4, FST4, WSPR, JT9, JT65. Protocol traits, FEC codecs, DSP
//! primitives, and per-protocol decoders/synthesisers are unified
//! behind a zero-cost generic abstraction — each protocol is a ZST
//! that declares its own constants + associated types, and hot-path
//! code like `coarse_sync::<P>` / `decode_frame::<P>` is
//! monomorphised per protocol.
//!
//! Every algorithm in this crate is derived from
//! [WSJT-X](https://sourceforge.net/projects/wsjt/) (Joe Taylor K1JT
//! et al.). Source files cite the corresponding upstream file they
//! port (`lib/ft8/…`, `lib/ft4/…`, `lib/fst4/…`, `lib/wsprd/…`,
//! `lib/jt65_*.f90`, `lib/jt9_*.f90`, `lib/packjt.f90`, etc.).
//! Licensed GPL-3.0-or-later, matching upstream.
//!
//! ## Module layout
//!
//! - [`core`] — protocol traits, DSP (resample / downsample / GFSK /
//!   subtract), sync, LLR, equaliser, pipeline driver.
//! - [`fec`] — LDPC(174, 91), LDPC(240, 101), convolutional r=½ K=32
//!   + Fano, Reed-Solomon(63, 12) over GF(2⁶).
//! - [`msg`] — 77-bit WSJT, 72-bit JT, 50-bit WSPR message codecs
//!   + callsign hash table.
//! - [`ft8`] / [`ft4`] / [`fst4`] / [`wspr`] / [`jt9`] / [`jt65`] —
//!   per-protocol ZSTs, decoders and synthesisers. Each is gated
//!   behind a feature of the same name.
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
//! | `full`        |          | Aggregate of ft8/ft4/fst4/wspr/jt9/jt65      |
//! | `parallel`    | yes      | Rayon-parallel candidate processing          |
//! | `osd-deep`    |          | OSD-3 fallback on AP decodes (extra CPU)     |
//! | `eq-fallback` |          | Non-EQ fallback inside `EqMode::Adaptive`    |
//!
//! ## Quick start
//!
//! ```toml
//! # Cargo.toml
//! [dependencies]
//! mfsk-core = { version = "0.1", features = ["ft8", "ft4"] }
//! ```
//!
//! ```no_run
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::ft8::{Ft8, decode::{decode_frame, DecodeDepth}};
//! let audio: Vec<i16> = /* 12 kHz PCM, 15 s */ vec![];
//! let results = decode_frame(&audio, 100.0, 3000.0, 1.5, None,
//!                            DecodeDepth::BpAllOsd, 200);
//! for r in results {
//!     println!("{:?} Hz  dt={:?}  SNR={:?}", r.freq_hz, r.dt_sec, r.snr_db);
//! }
//! # }
//! ```

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

// Flatten commonly-used types to the crate root.
pub use crate::core::{
    DecodeContext, FecCodec, FecOpts, FecResult, FrameLayout, MessageCodec, MessageFields,
    ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};

#[cfg(feature = "ft8")]
pub use crate::ft8::Ft8;
#[cfg(feature = "ft4")]
pub use crate::ft4::Ft4;
#[cfg(feature = "fst4")]
pub use crate::fst4::Fst4s60;
#[cfg(feature = "wspr")]
pub use crate::wspr::Wspr;
#[cfg(feature = "jt9")]
pub use crate::jt9::Jt9;
#[cfg(feature = "jt65")]
pub use crate::jt65::Jt65;
