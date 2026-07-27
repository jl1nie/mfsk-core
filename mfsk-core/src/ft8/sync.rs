//! FT8 synchronisation — re-exports from the protocol-generic
//! [`crate::engine::sync`] module.
//!
//! As of v0.6.0 this module no longer hosts FT8-specific sync code:
//!
//! - `coarse_sync` was removed in #48 step B (commit history). FT8
//!   coarse-sync is owned exclusively by
//!   [`crate::ft8::decode_block::coarse_sync`], which uses the
//!   WSJT-X `sync8.f90`-faithful 16-bin allsum noise estimator.
//!   `decode.rs` and `decode_block.rs` call it directly.
//!
//! - `compute_spectra`, `fine_sync_power`, `fine_sync_power_split`,
//!   `refine_candidate`, `refine_candidate_double` were removed in
//!   #49 cat B (this commit). They were one-line trampolines into
//!   `crate::engine::sync::*::<Ft8>`. Internal callers
//!   (`ft8::decode::process_candidate`) now turbofish the generic
//!   functions directly. Out-of-tree FT8 callers should do the same.
//!
//! - `refine_candidate_double` and its `FineSyncDetail` result type were
//!   deleted outright from `crate::engine::sync` in issue #192: an
//!   exhaustive call-graph audit found zero callers anywhere in the
//!   crate (production, tests, or benches) — the generic engine's
//!   fallback path that would have used it was unreachable and removed
//!   in the same pass (see `engine::pipeline::GenericPipelineProtocol`).
//!
//! The re-export below preserves the [`SyncCandidate`] type at its
//! FT8-namespaced path so naming stays stable even though no
//! FT8-specific code lives here anymore.

pub use crate::engine::sync::{SyncCandidate, make_costas_ref, parabolic_peak, score_costas_block};
