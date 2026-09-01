//! FT4 receiver, WAV demo — the desk equivalent of a radio on 40 m.
//!
//! Replays the vendored WSJT-X golden at 12 kHz in real time and runs
//! the shipping FT4 slot pipeline over it: the coarse periodogram
//! accumulated **during** capture, then DDC / Δt search / decode after
//! the slot closes, held to the slot budget. The FT8 controller has had
//! `wav_sim` for this since the beginning; FT4 had nothing, which is
//! why every FT4 number in `docs/notes/FT4_BENCHMARK.md` up to §33 came
//! from a bench feeding whole buffers rather than a receiver.
//!
//! ## The screen is the FT8 controller's
//!
//! Waterfall, decoded list and status bar all go through
//! `mfsk_app_shared::ui::state::UI` and are drawn by
//! `display::run_log_panel` — the same state and the same loop the FT8
//! path uses, not a second implementation of either. FT4 only supplies
//! rows and decodes.
//!
//! The one thing FT4 brings is *rate*: the coarse stage transforms 152
//! rows per 7.5 s slot and [`Ft4SavgBuilder::push_with_rows`] hands
//! each one over on the way past, so the waterfall flows at one row
//! per 48 ms during capture. The FT8 controller pushes one row per
//! 15 s slot. No extra transform either way — these are the rows the
//! decoder is already computing.
//!
//! **This is a separate bin rather than a boot mode** so it can be
//! flashed and re-flashed freely: it brings up no USB host, so the
//! serial console stays attached. `BootMode::Decode` is passed to the
//! panel for exactly that reason — it is the mode that does not take
//! the PHY.
//!
//! Build: `cargo build --release --features ft4 --bin ft4-demo`.
//!
//! [`Ft4SavgBuilder::push_with_rows`]:
//!     mfsk_core::engine::ft4_coarse::Ft4SavgBuilder::push_with_rows

use embedded_shared::apps::ft4_rx as ft4;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

use mfsk_core_m5stack_cores3_app as app;

/// The same slot `ft4-bench` uses: WSJT-X's `FT4/000000_000002.wav`,
/// baked to raw `i16` by `ft4_bake_golden_precomputed`. 19 signals, of
/// which 14 are inside the 100-2700 Hz search band and 11 decode in a
/// single pass at `DecodeDepth::EMBEDDED`.
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/ft4_golden_audio.bin");

/// The deadline the candidate loop is held to, measured from slot
/// close. `TX_TURNAROUND_BUDGET_MS` is what a transceiver has; a
/// receive-only demo could use `RX_ONLY_BUDGET_MS` and try every
/// candidate. This runs the transceiver budget because that is the
/// constraint worth seeing.
const BUDGET_MS: i64 = ft4::TX_TURNAROUND_BUDGET_MS;

/// One UAC read's worth of resampled audio — `uac::reader_thread`'s
/// `dst_scratch` is sized for ~256 samples at 48 k → 12 k. Feeding the
/// same size means the demo exercises the block cadence the radio path
/// will, rather than a friendlier one.
const BLOCK: usize = 256;

/// Stack for the feed/decode thread.
///
/// **8 KB, measured.** This thread runs the whole decode — coarse
/// stage, DDC, Δt search, BP — and `board::log_task_stacks` reported
/// it using **2 584 B of its previous 32 KB** (2026-09-01). The big
/// buffers are heap: a slot of audio, a `cd0` per candidate. 32 KB was
/// inherited from `decode_pipeline`'s FT8 path without ever being
/// checked against this workload, and on this board an unused KiB of
/// internal DRAM is not free — see `ft4_rx::WORKER_STACK`.
const FEED_STACK: usize = 8 * 1024;

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== mfsk-core-m5stack-cores3-app ft4-demo ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    // Which front end this image carries, printed at boot because a
    // flash that lands but never boots leaves no other way to tell
    // two images apart on the wire (2026-09-01: three "Flashing has
    // completed" writes with the board parked in DOWNLOAD mode and no
    // boot log to say which one was running).
    log::info!(
        "ft4-demo: front end = shared /2 streamed during capture; window closes at \
         {} samples, budget {} ms to key-up",
        ft4::CAPTURE_CLOSE_SAMPLES,
        ft4::TX_TURNAROUND_BUDGET_MS
    );

    // The candidate loop is compute-bound for hundreds of milliseconds
    // with no yield point; IDLE starves and the watchdog fires. Same
    // call and same reason as every bench and receiver here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("ft4-demo: task watchdog deinit -> {r}");

    // Before anything plans a transform, and before the panel's SPI
    // driver can fragment internal DRAM — the coarse stage's
    // 2304-point workspace is 41 % faster there. See `esp_dsp_fft`.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft4-demo: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );

    let peripherals = Peripherals::take().expect("peripherals taken twice");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");

    // WiFi, through the same `crate::net` path the receivers use, and
    // for one measurement in particular: the candidate worker's stack
    // has to come out of internal DRAM, and §30 recorded the largest
    // free internal block with WiFi up as 31 744 B against a 32 KB
    // ask. This bin is where that gets measured, because the app
    // installs the USB host driver and takes the serial console with
    // it — the same conditions, minus the console this measurement
    // needs. `MFSK_FT4_DEMO_WIFI=1` opts in; the demo is silent by
    // default so a desk without an AP still runs it.
    // Runtime condition, not a build-time one: `option_env!` is not
    // tracked by cargo, so a flag set on one build and not the next is
    // exactly the kind of difference that makes a measurement lie
    // about which arm it measured. cfg.toml's presence is the switch.
    if app::WIFI_SSID.is_empty() {
        log::info!("ft4-demo: no cfg.toml WiFi credentials — running without the network");
    } else {
        log::info!("ft4-demo: bringing WiFi up (measuring the worker stack against it)");
        let sysloop = esp_idf_svc::eventloop::EspSystemEventLoop::take().expect("sysloop");
        match mfsk_app_shared::wifi::wifi_driver_init(
            peripherals.modem,
            sysloop,
            Some(nvs_part.clone()),
        ) {
            Ok(driver) => {
                let settings_nvs = mfsk_app_shared::settings::open_nvs(nvs_part.clone())
                    .expect("settings NVS open");
                app::net::spawn(
                    driver,
                    std::sync::Arc::new(std::sync::Mutex::new(settings_nvs)),
                    app::net::Config {
                        name: "ft4-demo::net",
                        policy: app::net::DECODE_FIRST,
                        power_save: true,
                        on_ntp: |synced| log::info!("ft4-demo: NTP synced = {synced}"),
                    },
                );
            }
            Err(e) => log::error!("ft4-demo: WiFi driver init failed: {e:#}"),
        }
    }

    let spawn = app::board::spawn_named(c"ft4feed", FEED_STACK, feed_loop);
    if let Err(e) = spawn {
        log::error!("ft4-demo: feed thread spawn failed ({e})");
    }

    // The FT8 controller's own panel, unchanged. `BootMode::Decode`
    // keeps it out of USB host mode, which is what leaves the console
    // attached and the board flashable.
    app::display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi2,
        peripherals.pins,
        &app::FANOUT,
        nvs,
        BootMode::Decode,
    )
}

/// Replay the golden forever, decoding each slot and feeding the
/// shared UI.
fn feed_loop() {
    let audio: Vec<i16> = GOLDEN_AUDIO
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    assert_eq!(
        audio.len(),
        ft4::SLOT_SAMPLES,
        "the baked golden must be exactly one FT4 slot"
    );
    log::info!(
        "ft4-demo: replaying {} samples ({}.{} s) in {BLOCK}-sample blocks, forever",
        audio.len(),
        audio.len() / 12_000,
        (audio.len() % 12_000) * 10 / 12_000,
    );

    let mut accum = ft4::SlotAccum::new();
    let mut slot_no: u32 = 0;
    // Absolute pacing: sleeping `BLOCK/12` ms per block would drift by
    // whatever each block's work cost. Anchor to the start instead so
    // the feed stays at 12 kHz however long a block takes.
    let t_start = now_us();
    let mut fed: u64 = 0;

    loop {
        for chunk in audio.chunks(BLOCK) {
            // The rows the coarse stage is already transforming, turned
            // into palette indices on the way past and handed to the
            // shared UI. No extra FFT — see `push_with_rows`.
            let slot = accum.push_with_rows(chunk, &mut |row| {
                let cells = ft4::wf_row(row);
                if let Ok(mut ui) = UI.lock() {
                    ui.push_waterfall(cells);
                }
            });

            if let Some(slot) = slot {
                slot_no += 1;
                let o = ft4::decode_slot(&slot, BUDGET_MS);
                log::info!(
                    "ft4-demo: slot {slot_no} — {} of {} candidates tried, {} decodes in {} ms \
                     of {BUDGET_MS} ms{}",
                    o.tried,
                    o.cands,
                    o.decodes.len(),
                    o.elapsed_us / 1000,
                    match o.cut_at_score {
                        // Coarse scores are baseline-normalised, so 1.2
                        // is WSJT-X's own threshold: a cut landing near
                        // it gave up almost nothing.
                        Some(sc) => format!(" — cut {} weakest at score {sc:.2}", o.cands - o.tried),
                        None => String::new(),
                    },
                );
                if let Ok(mut ui) = UI.lock() {
                    for d in &o.decodes {
                        let mut msg: heapless::String<22> = heapless::String::new();
                        let _ = msg.push_str(&d.msg[..d.msg.len().min(22)]);
                        ui.push_decode(DecodedRow {
                            df_hz: d.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                            snr_db: d.snr_db.round().clamp(-128.0, 127.0) as i8,
                            hard_errors: d.hard_errors.min(255) as u8,
                            msg,
                            slot_seq: slot_no,
                            first_seq: slot_no,
                        });
                        log::info!(
                            "    {:>6.1} Hz  {:>+5.2} s  {:>3.0} dB  {}",
                            d.freq_hz,
                            d.dt_sec,
                            d.snr_db,
                            d.msg,
                        );
                    }
                }
            }

            fed += chunk.len() as u64;

            // Sleep only if the pipeline is ahead of real time; if it
            // is behind, the next block goes straight out and the slot
            // line above shows the overrun rather than this hiding it.
            let due_us = (fed * 1_000_000 / 12_000) as i64;
            let now = now_us() - t_start;
            if due_us > now {
                esp_idf_hal::delay::FreeRtos::delay_ms(((due_us - now) / 1_000).max(1) as u32);
            }
        }
    }
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
