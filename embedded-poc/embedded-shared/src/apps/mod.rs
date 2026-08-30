//! Reusable bin entry points shared between target crates.
//!
//! Each module exposes a `run()` (or similar) that target binaries
//! call after providing target-specific data (e.g. WAV byte slices).
//! All chip-agnostic logic lives here so per-target crates only carry
//! `Cargo.toml` (target / sdkconfig) and a 5-line `bin/*.rs` shim.

pub mod compute_bench;
#[cfg(feature = "fst4-bench")]
pub mod fst4_bench;
#[cfg(feature = "fst4-bench")]
pub mod fst4_ddc_bench;
#[cfg(feature = "ft4-bench")]
pub mod ft4_bench;
#[cfg(feature = "ft4")]
pub mod ft4_rx;
pub mod rx_wavsim;
pub mod scalar_bench;
#[cfg(feature = "wspr-bench")]
pub mod wspr_bench;
#[cfg(feature = "wspr-bench")]
pub mod wspr_scan;
