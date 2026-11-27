//! The old S3 flow on one core and on two: thin shim over `embedded_shared::apps::old_s3_bench`, whose module doc has the predictions. Logs only through
//! `embedded-poc/scripts/flash-monitor.sh` / `capture.sh`.
//!
//! Build: `cargo build --release --features ft8-old-s3-bench --bin ft8-old-s3-bench`.

const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");
const QSO1: &[u8] = include_bytes!("../../../assets/qso1.wav");
const QSO2: &[u8] = include_bytes!("../../../assets/qso2.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!("ft8-old-s3-bench: mixed-radix FFT scratch in {} DRAM", if internal { "INTERNAL" } else { "PSRAM" });
    if !embedded_shared::internal_pool::reserve_arena() {
        log::error!("ft8-old-s3-bench: cs scratch reservation failed");
    }
    embedded_shared::apps::old_s3_bench::run("m5stack-cores3", &[("qso3_busy", QSO3_BUSY), ("qso1", QSO1), ("qso2", QSO2)])
}
