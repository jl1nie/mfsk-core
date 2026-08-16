//! `LlrScalar` (f32 vs Q11i16 vs Q3i8) microbenchmark (M5Stack CoreS3
//! / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::scalar_bench`.
//! Answers the premise issue #198 was deprioritized on hold pending: is
//! fixed-point arithmetic actually faster than f32 on this specific
//! silicon? Answering that cheaply here is meant to inform whether
//! issue #198's `FecCodec::decode_soft` genericization (unlocking
//! fixed-point BP for FT4/FST4/MSK144, real numerical work) is worth
//! starting, before starting it.
//!
//! Build: `cargo build --release --bin scalar-bench`. No baked assets —
//! synthetic input only.

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::scalar_bench::init_logger_once();
    embedded_shared::apps::scalar_bench::run()
}
