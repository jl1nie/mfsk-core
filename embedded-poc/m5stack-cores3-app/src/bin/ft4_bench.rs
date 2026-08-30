//! FT4 whole-slot bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::ft4_bench`,
//! including why only one of these three assets is an input and what
//! the ~1.96 s budget is.
//!
//! Answers the FT4 half of the question issue #306 asked of FST4: FT4
//! had no embedded path at all (`docs/reference/EMBEDDED.md`), and
//! nothing in this tree had ever compiled `mfsk-core/ft4` for a board.
//!
//! Build: `cargo build --release --bin ft4-bench`.
//!
//! The baked assets are generated on the host from the in-tree WSJT-X
//! golden `000000_000002.wav` (from the repo root):
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test ft4_wsjtx_samples ft4_bake_golden_precomputed \
//!     -- --ignored --nocapture
//! ```
//!
//! before building this bin, if
//! `embedded-poc/assets/ft4_golden_{audio,fft_cache,candidates}.bin`
//! don't already exist.

/// The only *input*: one 7.5 s slot of 12 kHz audio, the same thing a
/// UAC capture would hand the decoder.
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/ft4_golden_audio.bin");
/// Control arm only — the DDC path passes an empty slice instead.
const GOLDEN_FFT_CACHE: &[u8] = include_bytes!("../../../assets/ft4_golden_fft_cache.bin");
/// Control for the on-device coarse stage, not a substitute for it.
const GOLDEN_CANDIDATES: &[u8] = include_bytes!("../../../assets/ft4_golden_candidates.bin");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // Must go through the shared guard, not `EspLogger::initialize_
    // default` directly — `run` installs the logger too, and a second
    // install aborts the process.
    embedded_shared::apps::ft4_bench::init_logger_once();
    embedded_shared::apps::ft4_bench::run(GOLDEN_AUDIO, GOLDEN_FFT_CACHE, GOLDEN_CANDIDATES)
}
