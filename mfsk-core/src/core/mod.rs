//! # mfsk-core
//!
//! Generic MFSK (M-ary frequency-shift-keying) primitives for WSJT-family
//! amateur-radio digital modes (FT8, FT4, FT2, FST4, JT9, JT65, WSPR).
//!
//! The crate defines *protocol traits* whose associated constants describe
//! modulation / frame / FEC / message-codec parameters, plus generic pipeline
//! code parameterised by those traits. Concrete protocol crates (`ft8-core`,
//! `ft4-core`, …) provide zero-sized types that implement the traits — all
//! dispatch is monomorphised, so there is no runtime cost vs. hand-written
//! per-protocol code.
//!
//! ## Zero-cost dispatch philosophy
//!
//! - **Hot paths** (sync correlation, LLR, FEC inner loops, DSP) take
//!   `P: Protocol` as a compile-time type parameter. Each concrete protocol
//!   produces its own monomorphised copy — LLVM sees a fully-specialised
//!   function and can autovectorise / drop bounds checks.
//! - **Cold paths** (message codec callbacks, CLI glue, FFI boundary) may
//!   legitimately use `dyn MessageCodec` / `Box<dyn …>` where ergonomics
//!   beat the negligible virtual-call cost.
//!
//! ## Re-export layout
//!
//! ```text
//! mfsk_core
//! ├── protocol   — trait hierarchy
//! └── (future) dsp / sync / llr / pipeline
//! ```

pub mod dsp;
pub mod equalize;
pub mod llr;
pub mod pipeline;
pub mod protocol;
pub mod sync;
pub mod tx;

pub use protocol::{
    DecodeContext, FecCodec, FecOpts, FecResult, FrameLayout, MessageCodec, MessageFields,
    ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};
