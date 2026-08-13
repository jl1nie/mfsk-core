//! WSPR candidate-loop bench (M5StickS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::wspr_bench`.
//! See `docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` (issue #260).
//!
//! Build: `cargo build --release --bin wspr-bench`.
//!
//! The baked baseband is generated on the host by the `#[ignore]`d
//! `wspr_bake_golden_baseband` in
//! `mfsk-core/tests/wspr_wsjtx_samples.rs` — 360 KiB, derived from the
//! in-tree WSJT-X golden `150426_0918.wav`.

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");

fn main() -> ! {
    embedded_shared::apps::wspr_bench::run(GOLDEN_BASEBAND)
}
