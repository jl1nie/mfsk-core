//! SIC on the CoreS3 (S3 plan, SIC-on-ESP32 step S2): thin shim over `embedded_shared::apps::sic_bench`, whose module doc has the arms. Logs only through
//! `embedded-poc/scripts/flash-monitor.sh` / `capture.sh`.
//!
//! Build: `cargo build --release --features ft8-sic-bench --bin ft8-sic-bench`.

const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");
const QSO1: &[u8] = include_bytes!("../../../assets/qso1.wav");
const QSO2: &[u8] = include_bytes!("../../../assets/qso2.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    // Before anything plans a transform, and the FT8 staging pair, while the heap is whole.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft8-sic-bench: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );
    embedded_shared::esp_dsp_fft::prewarm(2560);
    if !embedded_shared::internal_pool::reserve_arena() {
        log::error!("ft8-sic-bench: cs scratch reservation failed");
    }
    embedded_shared::apps::sic_bench::run(
        "m5stack-cores3",
        &[("qso3_busy", QSO3_BUSY), ("qso1", QSO1), ("qso2", QSO2)],
    )
}
