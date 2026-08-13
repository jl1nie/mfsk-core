//! WSPR candidate-loop bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::wspr_bench`.
//! See `docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` (issue #260).
//!
//! Build: `cargo build --release --bin wspr-bench`.
//!
//! **Why here and not in `m5stack-s3/`**, which is where the plan puts
//! it and where the compute benches otherwise live: the S3 board on the
//! bench is a CoreS3 (16 MB flash, 8 MB **Quad** PSRAM), and
//! `m5stack-s3`'s sdkconfig pins `CONFIG_SPIRAM_MODE_OCT=y` for the
//! M5StickS3. Flashing that build to a CoreS3 boot-loops on
//! `octal_psram: PSRAM chip is not connected, or wrong PSRAM line
//! mode`. Only the sdkconfig differs — the bench body is shared, so an
//! identical binary can be produced from `m5stack-s3` for a StickS3 by
//! its own `wspr-bench` bin.
//!
//! **This matters for Phase 2.** The PSRAM-vs-SRAM arm measures PSRAM
//! bandwidth, and Quad is roughly half of Octal's ~80 MB/s. A Quad
//! result is an upper bound on the PSRAM penalty a StickS3 would show,
//! not the same number.
//!
//! The baked baseband is generated on the host by the `#[ignore]`d
//! `wspr_bake_golden_baseband` in
//! `mfsk-core/tests/wspr_wsjtx_samples.rs` — 360 KiB, derived from the
//! in-tree WSJT-X golden `150426_0918.wav`.

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");

fn main() -> ! {
    embedded_shared::apps::wspr_bench::run(GOLDEN_BASEBAND)
}
