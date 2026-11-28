//! The cells flow's QSO-context AP rung on the CoreS3: thin shim over `embedded_shared::apps::ap_bench`, whose module doc has the predictions. Logs only through
//! `embedded-poc/scripts/flash-monitor.sh` / `capture.sh`.
//!
//! Build: `cargo build --release --features ft8-ap-bench --bin ft8-ap-bench`.

const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    // Before anything plans a transform, and the FT8 staging pair, while the heap is whole.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft8-ap-bench: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );
    embedded_shared::esp_dsp_fft::prewarm(2560);
    if !embedded_shared::internal_pool::reserve_arena() {
        log::error!("ft8-ap-bench: cs scratch reservation failed");
    }
    embedded_shared::apps::ap_bench::run("m5stack-cores3", QSO3_BUSY)
}
