//! FST4-60 DDC-fed coarse-sync + refine + decode bench (M5Stack CoreS3
//! / ESP32-S3 / LX7) — `docs/notes/FST4_DDC_DESIGN.md` stage 5.
//!
//! Thin shim — all logic lives in `embedded_shared::apps::fst4_ddc_bench`.
//! Unlike `fst4-bench` (issue #306/#307, LLR/BP/OSD only, no FFT
//! anywhere), this one runs the *whole* chain on-device: real 12 kHz
//! audio in, through the DDC (mixer + FIR cascade + polyphase
//! resampler, no FFT), `coarse_sync` (one power-of-two FFT, via the
//! DDC's own complex baseband), `fst4_sync_search` refine (no FFT),
//! and the same LLR/BP/OSD decoder `fst4-bench` already proved fits.
//!
//! Build: `cargo build --release --bin fst4-ddc-bench`.
//!
//! The baked asset is the same raw golden audio `fst4-bench` itself
//! documents generating (`fst4_bake_golden_precomputed` in
//! `mfsk-core/tests/fst4_wsjtx_samples.rs`) — only the
//! `fst4_60_golden_audio.bin` half is needed here, not the FFT cache.

const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/fst4_60_golden_audio.bin");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::fst4_ddc_bench::init_logger_once();
    embedded_shared::apps::fst4_ddc_bench::run_task(GOLDEN_AUDIO)
}
