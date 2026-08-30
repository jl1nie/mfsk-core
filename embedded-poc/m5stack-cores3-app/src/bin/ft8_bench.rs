//! FT8 per-stage bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::compute_bench`,
//! which already reports `decode_block`'s phases separately: stage 1
//! (spectrogram), stage 2 (coarse sync), pass 2 (re-rank), stage 3
//! (per-candidate LLR/BP/OSD).
//!
//! **Why a CoreS3 bin now.** FT8 has been the board's main mode since
//! #163, and every stage number for it came from `m5stack-s3`
//! (a StickS3) or from the app's own logs. FT4 got a proper bench on
//! this board and a day of measurement found four things nobody had
//! looked at (`docs/notes/FT4_BENCHMARK.md` §25-34); FT8's own
//! breakdown on the *same* silicon had never been taken, which is what
//! the `fixed-point` question needs — `ft8_qso3_apoff_recall` now shows
//! the quantisation costs no recall, so what remains is whether it buys
//! any speed, and that is a per-stage question.
//!
//! Build: `cargo build --release --features ft8-bench --bin ft8-bench`.

/// The recording every FT8 recall test in this tree scores against.
/// 20 known-real signals, of which the ship config reaches 12.
const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // Through the shared guard, not `initialize_default` directly:
    // `compute_bench::run` installs the logger too, and a second
    // install aborts.
    embedded_shared::apps::compute_bench::init_logger_once();

    // Before anything plans a transform. The f32 mixed-radix wrappers
    // want it; FT8's `fixed-point` spectrogram runs the *sc16* kernel
    // instead, so this is here for the paths around it rather than for
    // stage 1 itself — which is exactly the asymmetry this bench is
    // meant to expose.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft8-bench: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );

    // FT8's two `cs` staging buffers, taken while the heap is whole.
    // `main.rs` does this for the decode/uac boot modes; a bench is no
    // different, and without it stage 3 aborts in `internal_pool` with
    // "cs scratch arena unavailable" — which is the arena's own error
    // path working exactly as designed, several seconds after the run
    // that will need it started.
    if !embedded_shared::internal_pool::reserve_arena() {
        log::error!("ft8-bench: cs scratch reservation failed — stage 3 will abort");
    }

    embedded_shared::apps::compute_bench::run("m5stack-cores3", &[("qso3_busy", QSO3_BUSY)])
}
