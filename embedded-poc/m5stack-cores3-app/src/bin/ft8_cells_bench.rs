//! FT8 half-dB cell producer bench (M5Stack CoreS3): thin shim over `embedded_shared::apps::cells_bench`, whose module
//! doc says what it measures. The S3 cells plan's step 1. Logs only through `embedded-poc/scripts/flash-monitor.sh`.
//!
//! Build: `cargo build --release --features ft8-cells-bench --bin ft8-cells-bench`.

const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");
const QSO1: &[u8] = include_bytes!("../../../assets/qso1.wav");
const QSO2: &[u8] = include_bytes!("../../../assets/qso2.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    // Before anything plans a transform: the shared mixed-radix scratch, in internal DRAM while the heap is whole.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft8-cells-bench: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );
    embedded_shared::esp_dsp_fft::prewarm(2560);
    embedded_shared::apps::cells_bench::run(
        "m5stack-cores3",
        &[("qso3_busy", QSO3_BUSY), ("qso1", QSO1), ("qso2", QSO2)],
    )
}
