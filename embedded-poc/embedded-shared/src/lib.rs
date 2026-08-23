//! Shared embedded glue for `m5stack-core2` (LX6) and `m5stack-s3`
//! (LX7) FT8-RX bins. See `Cargo.toml` for module list.

#![no_std]

extern crate alloc;

pub mod apps;
pub mod dual_core;
pub mod esp_dsp_dotprod;
pub mod esp_dsp_fft;
// Behind `fst4-bench` for the same reason `wspr_dual_core` is behind
// `wspr-bench`: both reach into `mfsk_core::fst4`, which only the
// consumers that enable that feature have. Added ungated 2026-08-22,
// which quietly made `m5stack-s3-app` and `m5stack-core2-app`
// unbuildable — neither has changed since, so nobody hit it. There is
// no CI or pre-commit lint over `embedded-poc/` to have caught it (see
// this directory's `CLAUDE.md`); check the sibling crates by hand when
// touching this list.
#[cfg(feature = "fst4-bench")]
pub mod fst4_dual_core;
#[cfg(feature = "fst4-bench")]
pub mod fst4_monitor;
pub mod internal_pool;
pub mod pipeline;
pub mod stage1_inc;
pub mod wav_sim;
// Ungated: `fst4_dual_core` and `internal_pool` use it too, and they
// are not behind `wspr-bench`.
pub mod worker_arena;
#[cfg(feature = "wspr-bench")]
pub mod wspr_dual_core;
