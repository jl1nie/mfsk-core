//! JTTY receive-window cost bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::jtty_bench`. Answers #499
//! E0: does the JTTY receiver's per-window work (sync surface + ladder) fit its
//! 0.472 s window, and what does the decimated search #499 proposes cost instead?
//!
//! Build: `cargo build --release --bin jtty-bench --features jtty`. No baked assets.

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::jtty_bench::init_logger_once();
    embedded_shared::apps::jtty_bench::run()
}
