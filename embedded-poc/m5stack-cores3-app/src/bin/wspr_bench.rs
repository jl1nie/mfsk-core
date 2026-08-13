//! WSPR candidate-loop bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::wspr_bench`.
//! See `docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` (issue #260).
//!
//! Build: `cargo build --release --bin wspr-bench`.
//!
//! **Why here and not in `m5stack-s3/`**, which is where the plan puts
//! it and where the compute benches otherwise live: the S3 board on the
//! bench is a CoreS3 (16 MB flash, 8 MB **Quad** PSRAM), and
//! `m5stack-s3`'s sdkconfig pins `CONFIG_SPIRAM_MODE_OCT=y` for the
//! M5StickS3. Flashing that build to a CoreS3 boot-loops on
//! `octal_psram: PSRAM chip is not connected, or wrong PSRAM line
//! mode`. Only the sdkconfig differs — the bench body is shared, so an
//! identical binary can be produced from `m5stack-s3` for a StickS3 by
//! its own `wspr-bench` bin.
//!
//! **This matters for Phase 2.** The PSRAM-vs-SRAM arm measures PSRAM
//! bandwidth, and Quad is roughly half of Octal's ~80 MB/s. A Quad
//! result is an upper bound on the PSRAM penalty a StickS3 would show,
//! not the same number.
//!
//! The baked baseband is generated on the host by the `#[ignore]`d
//! `wspr_bake_golden_baseband` in
//! `mfsk-core/tests/wspr_wsjtx_samples.rs` — 360 KiB, derived from the
//! in-tree WSJT-X golden `150426_0918.wav`.

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");

/// Bring WiFi up before the bench when `MFSK_WSPR_BENCH_WIFI` is set at
/// build time, so the run measures the decoder **as it would actually
/// ship** rather than on a board with the radio switched off.
///
/// This is a memory question before it is a CPU one. The candidate loop
/// needs two large task stacks live at once — `SCAN_STACK` 90 KB plus
/// `PASS01_WORKER_STACK` 80 KB, then `PASS2_WORKER_STACK` 88 KB — and
/// FreeRTOS task stacks must come out of *internal* DRAM in one
/// contiguous piece. A radio-off boot leaves 328 KB free with a 236 KB
/// largest block, so 178 KB of stacks fits with ~58 KB to spare. The
/// WiFi driver plus LwIP claim tens of KB of that same internal pool,
/// and `wspr_dual_core`'s `have_room_for(PASS2_WORKER_STACK)` guard
/// responds to a shortfall by **silently falling back to sequential** —
/// which costs the 1.17x dual-core speedup and pushes TOTAL back over
/// the 120 s slot. Nothing in the bench log would say why.
///
/// So: measure the heap with the radio up, before deciding how much of
/// the slot to reserve for spot upload.
fn maybe_start_wifi() -> Option<mfsk_app_shared::wifi::WifiHandle> {
    if option_env!("MFSK_WSPR_BENCH_WIFI").is_none() {
        log::info!("wspr_bench: WiFi off (set MFSK_WSPR_BENCH_WIFI=1 to measure with the radio up)");
        return None;
    }
    if WIFI_SSID.is_empty() {
        log::warn!("wspr_bench: MFSK_WSPR_BENCH_WIFI set but WIFI_SSID empty (no cfg.toml)");
        return None;
    }
    let peripherals = esp_idf_hal::peripherals::Peripherals::take().ok()?;
    let sysloop = esp_idf_svc::eventloop::EspSystemEventLoop::take().ok()?;
    let nvs = esp_idf_svc::nvs::EspDefaultNvsPartition::take().ok();
    match mfsk_app_shared::wifi::connect_sta(
        peripherals.modem,
        sysloop,
        nvs,
        WIFI_SSID,
        WIFI_PSK,
    ) {
        Ok(h) => {
            log::info!("wspr_bench: WiFi up, ip {}", h.ip);
            Some(h)
        }
        Err(e) => {
            log::warn!("wspr_bench: WiFi bring-up failed: {e:#}");
            None
        }
    }
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // Must go through the shared guard, not `EspLogger::initialize_
    // default` directly — `run` installs the logger too, and the second
    // install aborts the process.
    embedded_shared::apps::wspr_bench::init_logger_once();
    // Held for the process lifetime — dropping the handle tears the
    // association down, and the point is to keep the radio's memory
    // claimed for the whole measurement.
    let _wifi = maybe_start_wifi();
    embedded_shared::apps::wspr_bench::run(GOLDEN_BASEBAND)
}
