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

/// Bring WiFi up when `MFSK_WSPR_BENCH_WIFI` is set at build time, so
/// the run measures the decoder **as it would actually ship** rather
/// than on a board with the radio switched off. Called from
/// `run_with_post_spawn`'s hook — after both decode task stacks are
/// placed, never before (see `main`).
///
/// This is a memory question before it is a CPU one, and the answer
/// moved twice while issue #260 worked it out:
///
/// - A FreeRTOS task stack is a contiguous *internal* DRAM allocation.
///   With WiFi associated, the largest such block on a CoreS3 is 88 KB,
///   and the loop wanted two stacks at once.
/// - `=cycle` exists because "take the radio down for the decode" was
///   the obvious fix. It is not one: cycling returns free internal DRAM
///   (228 -> 269 KB) while the **largest contiguous block stays at
///   156 KB**, merely linking WiFi costs 236 -> 184 KB before it is ever
///   started, and re-associating takes a measured 7.9 s (and was seen
///   failing outright with `ESP_ERR_TIMEOUT`). Kept as a diagnostic.
/// - What actually fixed it: the dual-core worker's stack moved to
///   `.bss` (`wspr_dual_core::WORKER_STACK`), and the scan task's is now
///   taken before the radio starts. The radio can stay up through the
///   decode, and does.
///
/// `=1` holds the association for the whole run — the shipped
/// arrangement, and the configuration any spot-upload number has to be
/// measured in.
fn log_internal(label: &str) {
    use esp_idf_svc::sys::{
        MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL, heap_caps_get_free_size,
        heap_caps_get_largest_free_block,
    };
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let (free, largest) = unsafe {
        (
            heap_caps_get_free_size(caps),
            heap_caps_get_largest_free_block(caps),
        )
    };
    log::info!(
        "[mem] {label}: free_internal {} KB, largest contig {} KB",
        free / 1024,
        largest / 1024
    );
}

fn maybe_start_wifi() -> Option<mfsk_app_shared::wifi::WifiHandle> {
    let Some(mode) = option_env!("MFSK_WSPR_BENCH_WIFI") else {
        log::info!("wspr_bench: WiFi off (MFSK_WSPR_BENCH_WIFI=1 to hold it up, =cycle to up/down)");
        return None;
    };
    if WIFI_SSID.is_empty() {
        log::warn!("wspr_bench: MFSK_WSPR_BENCH_WIFI set but WIFI_SSID empty (no cfg.toml)");
        return None;
    }
    log_internal("pre-wifi");
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
            log_internal("wifi-up");
            if mode == "cycle" {
                // What a real receiver does between slots: the spot has
                // left, so give the radio's internal DRAM back before
                // the decode claims its stacks.
                drop(h);
                unsafe { esp_idf_svc::sys::vTaskDelay(200) };
                log_internal("wifi-dropped");
                return None;
            }
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
    // WiFi comes up from the post-spawn hook, **not** here: the decode
    // task stacks have to take their contiguous internal DRAM before
    // the radio starts claiming any (issue #260). Leaked rather than
    // held in a local because the hook returns and the bench never
    // exits — dropping the handle would tear the association down.
    embedded_shared::apps::wspr_bench::run_with_hooks(
        GOLDEN_BASEBAND,
        || {
            if let Some(h) = maybe_start_wifi() {
                core::mem::forget(h);
            }
        },
        &report_spots,
    )
}

/// Slot start of the baked golden recording, `150426_0918.wav` — the
/// bench replays one fixed capture, so the timestamp is the file's, not
/// the clock's. A live receiver substitutes its own slot start.
const GOLDEN_SLOT_DATE: &str = "150426";
const GOLDEN_SLOT_TIME: &str = "0918";

/// Dial frequency the golden recording was made on: the 20 m WSPR
/// segment, 14.0956 MHz USB.
const GOLDEN_DIAL_MHZ: f64 = 14.095_600;

/// Hand each slot's decodes to the wsprnet encoder.
///
/// **Nothing is uploaded** — [`SpotSink::Dummy`] formats the request
/// and logs it. This bench replays one recording from 2015 on a loop;
/// posting that to a live database would be asserting receptions that
/// are neither current nor this station's. The encoder is the part
/// worth exercising against real decoder output, and it is exercised
/// fully by building the body.
fn report_spots(results: &[mfsk_core::wspr::decode::WsprResult]) {
    use mfsk_app_shared::wsprnet::{Mode, Reporter, Spot, SpotSink, report_slot};

    let reporter = Reporter {
        call: "N0CALL".into(),
        grid: "PM95".into(),
        dial_mhz: GOLDEN_DIAL_MHZ,
        // The decoder's version, not this bin's — that is what a
        // spot is a claim about.
        version: format!("mfsk-core-{}", mfsk_core::VERSION),
        mode: Mode::Wspr2,
    };

    let spots: Vec<Spot> = results
        .iter()
        .map(|r| {
            // `WsprResult.message` is the decoded 50-bit payload
            // rendered as text — "<call> <grid> <dbm>" for a type-1
            // message. Split rather than re-derive: the decoder already
            // did the unpacking, and wsprnet wants the same three
            // fields back out.
            let mut parts = r.message.to_string();
            let mut it = parts.split_whitespace();
            let call = it.next().unwrap_or("").to_string();
            let grid = it.next().unwrap_or("").to_string();
            let dbm = it.next().and_then(|d| d.parse().ok()).unwrap_or(0);
            parts.clear();
            Spot {
                date: GOLDEN_SLOT_DATE.into(),
                time: GOLDEN_SLOT_TIME.into(),
                call,
                grid,
                dbm,
                snr_db: r.snr_db as i32,
                dt_sec: r.dt_sec,
                drift_hz: r.drift_hz as i32,
                audio_hz: r.freq_hz,
            }
        })
        .collect();

    report_slot(&spots, &reporter, &SpotSink::Dummy);
}
