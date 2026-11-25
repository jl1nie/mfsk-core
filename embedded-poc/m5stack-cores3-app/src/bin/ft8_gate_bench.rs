//! FT8 gate + `fine_sync_12k` + OSD bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::gate_bench`, whose module doc has the arms and what they
//! mean. This is the S3 plan's stage 1: what the host's `mirror_gate_and_osd` cannot say, the time, the stack and
//! the heap of adding the gate, `fine_sync_12k` and OSD to the shipped pipeline.
//!
//! Run it beside `ft8-bench` (the shipped driver's own stage times, no OSD) on the **same board in the same
//! session**; the logs go through `embedded-poc/scripts/flash-monitor.sh` only (no second `espflash monitor`).
//!
//! Build: `cargo build --release --features ft8-gate-bench --bin ft8-gate-bench`.

const QSO3_BUSY: &[u8] = include_bytes!("../../../assets/qso3_busy.wav");
const QSO1: &[u8] = include_bytes!("../../../assets/qso1.wav");
const QSO2: &[u8] = include_bytes!("../../../assets/qso2.wav");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // The shared guard: `gate_bench::run` installs the logger too, and a second install aborts.
    embedded_shared::apps::compute_bench::init_logger_once();

    // Before anything plans a transform (as `ft8-bench` does).
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft8-gate-bench: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );
    // FT8's `cs` staging buffers, taken while the heap is whole (the arena aborts in `internal_pool` otherwise).
    if !embedded_shared::internal_pool::reserve_arena() {
        log::error!("ft8-gate-bench: cs scratch reservation failed — stage 3 will abort");
    }

    embedded_shared::apps::gate_bench::run(
        "m5stack-cores3",
        &[("qso3_busy", QSO3_BUSY), ("qso1", QSO1), ("qso2", QSO2)],
    )
}
