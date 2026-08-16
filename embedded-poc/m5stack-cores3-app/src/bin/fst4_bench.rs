//! FST4-60 decoder-only bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::fst4_bench`.
//! Answers issue #306: does FST4-60's LLR/BP/OSD stage fit inside its
//! ~7 s post-slot margin, measured over a real candidate population —
//! with no FFT anywhere in the on-device path (issue #307 found the
//! embedded FFT backend can't serve any of the 3 FFT sites FST4 needs
//! yet; see that module's doc comment for the full account, including
//! why this bin only ships one baked asset, not two).
//!
//! Build: `cargo build --release --bin fst4-bench`.
//!
//! The baked asset is generated on the host from the in-tree WSJT-X
//! golden `210115_0058.wav` (from the repo root):
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test fst4_wsjtx_samples fst4_bake_golden_refined_candidates \
//!     -- --ignored --nocapture
//! ```
//!
//! before building this bin, if
//! `embedded-poc/assets/fst4_60_golden_refined_candidates.bin` doesn't
//! already exist.

const GOLDEN_REFINED_CANDIDATES: &[u8] =
    include_bytes!("../../../assets/fst4_60_golden_refined_candidates.bin");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // Must go through the shared guard, not `EspLogger::initialize_
    // default` directly — `run` installs the logger too, and a second
    // install aborts the process.
    embedded_shared::apps::fst4_bench::init_logger_once();
    embedded_shared::apps::fst4_bench::run(GOLDEN_REFINED_CANDIDATES)
}
