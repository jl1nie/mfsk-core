//! FST4-60 decoder-only bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::fst4_bench`.
//! Answers issue #306: does the FST4-60 decoder (candidate search +
//! LLR/BP/OSD, fed a host-baked FFT cache — see that module's doc
//! comment for why the wideband stage can't run on-device today) fit
//! inside FST4-60's ~7 s post-slot margin.
//!
//! Build: `cargo build --release --bin fst4-bench`.
//!
//! Both baked assets are generated on the host by the `#[ignore]`d
//! `fst4_bake_golden_precomputed` in
//! `mfsk-core/tests/fst4_wsjtx_samples.rs` — from the in-tree WSJT-X
//! golden `210115_0058.wav`. Run it (from the repo root):
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test fst4_wsjtx_samples fst4_bake_golden_precomputed -- --ignored --nocapture
//! ```
//!
//! before building this bin, if `embedded-poc/assets/fst4_60_golden_*.bin`
//! don't already exist.

const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/fst4_60_golden_audio.bin");
const GOLDEN_FFT_CACHE: &[u8] = include_bytes!("../../../assets/fst4_60_golden_fft_cache.bin");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // Must go through the shared guard, not `EspLogger::initialize_
    // default` directly — `run` installs the logger too, and a second
    // install aborts the process.
    embedded_shared::apps::fst4_bench::init_logger_once();
    embedded_shared::apps::fst4_bench::run(GOLDEN_AUDIO, GOLDEN_FFT_CACHE)
}
