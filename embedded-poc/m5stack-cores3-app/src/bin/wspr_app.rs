//! CoreS3 WSPR receiver app — step 7 of the CoreS3 WSPR app plan
//! (see memory `project_wspr_app_cores3_ui`, plan file
//! `~/.claude/plans/mighty-chasing-castle.md`).
//!
//! Boot sequence: NVS settings → spawn the display task (its own LCD
//! bring-up, core 1) → spawn the scan task (reserving its stack
//! before WiFi, see the comment at its call site, core 0) → WiFi STA
//! → NTP → HTTP config server → release the scan task → `main` itself
//! becomes an idle supervisor loop.
//!
//! **Display runs on its own task, pinned to core 1 — not inline in
//! `main`.** `CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y` (this crate's
//! sdkconfig) pins `main` to core 0, the same core the scan task is
//! pinned to; the scan task runs at FreeRTOS priority 5 and is
//! compute-bound for 90-110 s at a stretch with no yield point (that's
//! *why* the watchdog gets disabled). A first version of this app ran
//! the render loop inline in `main` and was starved almost the entire
//! 2-minute cycle as a result — real-hardware observation (the LCD
//! looked frozen most of the time), not a theoretical concern. Moving
//! the render loop to its own core-1 task fixes it: `wspr_dual_core`'s
//! worker also lands on core 1 (`APP_CPU = 1`), but only for the
//! fraction of each pass spent in `pass01_split`/`pass2_split`, not
//! the whole scan — far less contention than sharing a core with the
//! scan task itself.
//!
//! A **new binary**, not folded into `main.rs`: WSPR never touches
//! the FT8 controller's `decode_block`/UAC stack, and this app's own
//! task-stack placement constraints (see below) are unrelated to that
//! binary's.
//!
//! **Live audio capture is out of scope** (blocked on issue #163, see
//! this crate's `wspr_bench.rs`) — every slot decodes the same baked
//! WAV-derived baseband (`assets/wspr_golden_baseband.bin`) the bench
//! binary uses. One consequence worth being explicit about: the
//! band selector in the web settings form changes what dial frequency
//! gets *reported* to wsprnet, not what actually gets decoded — there
//! is no live receiver behind it yet. Picking a band other than 20 m
//! makes every spot's `tqrg` field describe a signal that was not
//! really heard on that band.
#![allow(dead_code)] // board.rs/pmic.rs carry fields/fns this bin doesn't use (no UAC, no touch).

#[path = "../board.rs"]
mod board;
#[path = "../pmic.rs"]
mod pmic;

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use display_interface_spi::SPIInterface;
use mipidsi::{
    models::ILI9341Rgb565,
    options::{ColorInversion, Orientation, Rotation},
    Builder,
};

use mfsk_core::wspr::decode::WsprResult;

use embedded_shared::apps::wspr_scan::{load_baseband, now_us, run_scan, NBB, SLOT_US};
use embedded_shared::wspr_dual_core;

use mfsk_app_shared::civil_time::civil_from_unix;
use mfsk_app_shared::settings::{self, Settings};
use mfsk_app_shared::wspr_bands::{WsprBand, WSPR_BANDS};
use mfsk_app_shared::ui::wspr_list;
use mfsk_app_shared::ui::wspr_row::WsprSpotRow;
use mfsk_app_shared::ui::wspr_state::WSPR_UI;
use mfsk_app_shared::{http_config, ntp};

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");

/// How long to wait for the boot-time NTP attempt before giving up
/// and running without absolute time (`ntp_synced` stays false, and
/// wsprnet reporting stays off for the whole run — see `scan_loop`).
/// 20 s is generous for `pool.ntp.org` over a home network; unmeasured
/// on real hardware, same caveat as this plan's other untuned
/// timeouts.
const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;

/// Stack for the scan task. Same 72 KiB `wspr-bench` settled on after
/// measuring a 63 192 B peak through `run_scan`'s full pass 0/1/2
/// sequence (see that bin's own `SCAN_STACK` doc comment) — this app
/// calls the identical function, so the identical stack applies.
const SCAN_STACK: u32 = 72 * 1024;

/// Stack for the display task. `embedded_graphics`/`mipidsi` drawing
/// is shallow compared to the decode path; this is roughly what
/// `main`'s own 32 KiB stack (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) was
/// covering for the same code when it ran inline, not an independent
/// hardware measurement.
const DISPLAY_STACK: u32 = 24 * 1024;

/// Below the scan/dual-core-worker priority (5) so the display task
/// never delays a decode — on core 1 it only ever contends with
/// `wspr_dual_core`'s intermittent worker, and losing that contention
/// just means a render waits a few hundred ms, not that a spot gets
/// dropped.
const DISPLAY_PRIORITY: u32 = 3;

/// Internal-DRAM free/largest-contiguous-block snapshot, same shape
/// as `wspr-bench`'s own `log_heap` — evidence for the task-spawn
/// ordering this file's `main` doc comment argues for, not just a
/// "it didn't error this run" assumption.
fn log_heap(tag: &str) {
    let caps = esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT;
    unsafe {
        log::info!(
            "wspr_app heap {tag}: internal free {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(caps) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(caps) / 1024,
        );
    }
}

/// Set once WiFi/NTP/HTTP-server bring-up finishes; the scan task
/// blocks on this before its first `run_scan`. See the comment at
/// `spawn_scan_task`'s call site for why the task is *created* before
/// that bring-up even though it doesn't start working until after.
static SCAN_GO: AtomicBool = AtomicBool::new(false);

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::wspr_bench::init_logger_once();

    log::info!("=== mfsk-core-m5stack-cores3-app wspr-app boot ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop runs compute-bound for tens of seconds at a
    // stretch with no yield point; IDLE0 starves and the task
    // watchdog fires. Same call `wspr-bench` makes, same reason —
    // this app pays the identical decode cost.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");
    wspr_dual_core::init();

    let peripherals = Peripherals::take().expect("peripherals taken twice");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = settings::open_nvs(nvs_part.clone()).expect("settings NVS open");
    let nvs = Arc::new(Mutex::new(nvs));

    // Spawn the scan task's 72 KiB stack **before anything else can
    // fragment internal DRAM** — WiFi is the case `wspr-bench` (issue
    // #260) measured (largest contiguous block 236 -> 184 KB *before
    // the radio even starts*, just from linking), but the display
    // task's own SPI-driver/mipidsi allocations are the same kind of
    // risk if they're left free to race this claim: a first version
    // of this fix spawned the display task first and hit exactly that
    // — `xTaskCreatePinnedToCore` for the scan stack failed on real
    // hardware because the display task's concurrent bring-up (on the
    // other core, but the same heap) was contending for the same
    // block. Scan first, unconditionally, before anything else on
    // either core gets a chance to allocate. The task itself blocks
    // on `SCAN_GO` until bring-up below finishes — this is "reserve
    // the memory now, start the work once settings/WiFi/NTP are
    // ready," not "decode before init is done."
    log_heap("pre-scan-spawn");
    spawn_scan_task(ScanCtx { nvs: nvs.clone() });
    log_heap("post-scan-spawn");

    // Display task: its own LCD bring-up + render loop, pinned to
    // core 1 — see this file's top doc comment for why inline-in-
    // `main` starved it against the scan task on real hardware.
    spawn_display_task(DisplayCtx {
        i2c0: peripherals.i2c0,
        spi2: peripherals.spi2,
        pins: peripherals.pins,
    });
    log_heap("post-display-spawn");

    // ── WiFi STA. Compile-time SSID/PSK, same as every other bin in
    // this crate — provisioning UI is explicitly out of scope. ──────
    let sysloop = EspSystemEventLoop::take().expect("sysloop");
    let wifi_handle = if WIFI_SSID.is_empty() {
        log::warn!("wspr_app: WIFI_SSID empty (no cfg.toml) — NTP/HTTP config/wsprnet all unavailable");
        None
    } else {
        match mfsk_app_shared::wifi::connect_sta(peripherals.modem, sysloop, Some(nvs_part), WIFI_SSID, WIFI_PSK) {
            Ok(h) => {
                log::info!("wspr_app: WiFi up, ip {}", h.ip);
                Some(h)
            }
            Err(e) => {
                log::warn!("wspr_app: WiFi STA failed: {e:#}");
                None
            }
        }
    };
    let wifi_up = wifi_handle.is_some();
    // `_wifi_handle` is never read again but must outlive `main` (its
    // `Drop` tears the association down) — `main` never returns, so a
    // plain binding here already lives for the process's lifetime.
    let _wifi_handle = wifi_handle;

    // ── NTP: one attempt at boot, gated by settings + WiFi. Not
    // per-slot — see `ntp.rs`'s own doc comment for why absolute time
    // can't come from anywhere else on a cold start. ────────────────
    let initial_settings = {
        let g = nvs.lock().expect("settings NVS mutex poisoned");
        settings::load(&g)
    };
    let (ntp_synced, _sntp_keepalive) = if wifi_up && initial_settings.ntp_enabled {
        match ntp::start(&initial_settings.ntp_server) {
            Ok(sntp) => {
                let synced = ntp::wait_synced(&sntp, NTP_SYNC_TIMEOUT_MS);
                (synced, Some(sntp))
            }
            Err(e) => {
                log::warn!("wspr_app: NTP start failed: {e:#}");
                (false, None)
            }
        }
    } else {
        if !initial_settings.ntp_enabled {
            log::info!("wspr_app: NTP sync disabled in settings");
        } else {
            log::info!("wspr_app: NTP skipped — WiFi not up");
        }
        (false, None)
    };
    if !ntp_synced {
        log::warn!("wspr_app: no absolute UTC this run — wsprnet reporting stays off every slot");
    }

    // ── HTTP config server. ──────────────────────────────────────────
    let _http_server = if wifi_up {
        match http_config::start(nvs.clone()) {
            Ok(s) => {
                log::info!("wspr_app: HTTP config server up");
                Some(s)
            }
            Err(e) => {
                log::warn!("wspr_app: HTTP config server failed: {e:#}");
                None
            }
        }
    } else {
        None
    };

    {
        let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
        ui.update_status(|u| u.ntp_synced = ntp_synced);
    }
    SCAN_GO.store(true, Ordering::Release);

    // `main` (pinned to core 0 by sdkconfig) has nothing left to do —
    // the scan task owns core 0's real work, the display task runs
    // independently on core 1. An idle supervisor loop, same shape as
    // `wspr-bench`'s own `main` tail.
    loop {
        FreeRtos::delay_ms(1000);
    }
}

// ── Display task ──────────────────────────────────────────────────────

struct DisplayCtx {
    i2c0: esp_idf_hal::i2c::I2C0<'static>,
    spi2: esp_idf_hal::spi::SPI2<'static>,
    pins: esp_idf_hal::gpio::Pins,
}

extern "C" fn display_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_display_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut DisplayCtx) };
    display_loop(*ctx);
}

fn spawn_display_task(ctx: DisplayCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(display_task_entry),
            c"wspr_display".as_ptr(),
            DISPLAY_STACK,
            ptr,
            DISPLAY_PRIORITY,
            core::ptr::null_mut(),
            1, // core 1 — deliberately NOT the scan task's core 0, see
               // this file's top doc comment.
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_display task");
    }
}

/// LCD bring-up (AXP2101 + AW9523B → SPI2 → mipidsi, mirrors
/// `display.rs`'s FT8-controller sequence minus the BootMode/UAC
/// branches this app has no use for) followed by the render loop.
/// Never returns.
fn display_loop(ctx: DisplayCtx) -> ! {
    let mut display = match pmic::init(ctx.i2c0, ctx.pins.gpio12, ctx.pins.gpio11) {
        Ok(i2c) => {
            drop(i2c); // reset cycle is complete; nothing else on this bus yet (no touch).

            let driver = SpiDriver::new(
                ctx.spi2,
                ctx.pins.gpio36, // SCK  (board::LCD_PIN_SCK)
                ctx.pins.gpio37, // MOSI (board::LCD_PIN_MOSI)
                Option::<AnyIOPin>::None,
                &SpiDriverConfig::new(),
            )
            .expect("SPI2 driver");
            let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
            let spi_dev = SpiDeviceDriver::new(driver, Some(ctx.pins.gpio3), &spi_cfg) // CS (board::LCD_PIN_CS)
                .expect("SPI device (CS=3)");
            let dc = PinDriver::output(ctx.pins.gpio35).expect("DC gpio35"); // board::LCD_PIN_DC
            let di = SPIInterface::new(spi_dev, dc);

            let mut delay = Ets;
            match Builder::new(ILI9341Rgb565, di)
                .display_size(240, 320)
                .orientation(Orientation::new().rotate(Rotation::Deg90))
                .invert_colors(ColorInversion::Inverted)
                .init(&mut delay)
            {
                Ok(d) => d,
                Err(e) => {
                    log::error!("display init failed: {e:?}");
                    loop {
                        log::info!("alive (no LCD)");
                        FreeRtos::delay_ms(2000);
                    }
                }
            }
        }
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            loop {
                log::info!("alive (no PMIC/LCD)");
                FreeRtos::delay_ms(2000);
            }
        }
    };
    display.clear(Rgb565::BLACK).ok();
    log::info!("LCD init OK ({}x{})", board::LCD_WIDTH, board::LCD_HEIGHT);
    {
        let ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
        wspr_list::render_all(&mut display, &ui).ok();
    }

    // Status bar repaints every tick (cheap, one line); discovered/
    // history panes only repaint when `WsprUiState::dirty_seq`
    // actually changed, same gating `decoded_list`/`waterfall` use
    // for the FT8 UI. The lock is held across the SPI draw calls
    // rather than snapshotted out first (as the FT8 display loops
    // do) — contention is the scan task's once-per-slot `set_slot`/
    // `update_status` against this loop's 500 ms tick, which is rare
    // and brief enough that the simpler form was chosen over
    // threading a full `WsprUiState` snapshot type through for this
    // first pass.
    let mut last_dirty = u32::MAX;
    let mut tick: u32 = 0;
    loop {
        let heap_kb = (unsafe { esp_idf_svc::sys::esp_get_free_heap_size() } / 1024) as u32;
        let hhmmss = current_hhmmss();
        let dirty = {
            let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
            ui.free_heap_kb = heap_kb;
            ui.utc_hhmmss = hhmmss.clone();
            ui.dirty_seq()
        };
        {
            let ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
            wspr_list::render_status(&mut display, &ui).ok();
            if dirty != last_dirty {
                wspr_list::render_discovered(&mut display, &ui).ok();
                wspr_list::render_history(&mut display, &ui).ok();
                last_dirty = dirty;
            }
        }
        // ~10 s cadence. Proves this task keeps running (and the UTC
        // clock keeps advancing) through the scan task's 90-110 s
        // compute-bound stretch on the other core, same purpose as
        // the FT8 controller `display.rs`'s own periodic "alive"
        // line — direct evidence over a plausible architecture, after
        // a first cut of this fix turned out to have its own bug
        // (see `main`'s doc comment on spawn ordering).
        if tick % 20 == 0 {
            log::info!("wspr_app::display: alive tick={tick} dirty={dirty} utc={hhmmss} heap={heap_kb}k");
        }
        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(500);
    }
}

// ── Scan task ─────────────────────────────────────────────────────────

struct ScanCtx {
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn scan_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_scan_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut ScanCtx) };
    scan_loop(*ctx);
}

fn spawn_scan_task(ctx: ScanCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(scan_task_entry),
            c"wspr_scan".as_ptr(),
            SCAN_STACK,
            ptr,
            5,
            core::ptr::null_mut(),
            0, // pinned to core 0, matching wspr-bench — wspr_dual_core's
               // worker expects the caller on a known core.
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_scan task");
    }
}

/// One `run_scan` per (simulated) slot, forever. Never returns.
fn scan_loop(ctx: ScanCtx) -> ! {
    while !SCAN_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("wspr_app: scan loop starting");

    // `idat`/`qdat` land in PSRAM automatically: `NBB` f32 samples is
    // 184 320 B, well past this crate's `SPIRAM_MALLOC_ALWAYSINTERNAL`
    // threshold (4096 B), so a plain heap `Vec` gets PSRAM without the
    // manual `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` calls
    // `wspr-bench` uses — that bench explicitly wants to *choose*
    // between SRAM/PSRAM to measure the difference; this app just
    // wants wherever the allocator already puts a buffer this size.
    let mut idat = vec![0.0f32; NBB];
    let mut qdat = vec![0.0f32; NBB];

    loop {
        let slot_start_us = now_us();

        let settings = {
            let g = ctx.nvs.lock().expect("settings NVS mutex poisoned");
            settings::load(&g)
        };
        let band = &WSPR_BANDS[settings.band_idx as usize];

        load_baseband(GOLDEN_BASEBAND, &mut idat, &mut qdat);
        let (stats, results) = run_scan(&mut idat, &mut qdat);
        for s in &stats {
            s.log();
        }
        log::info!("wspr_app: slot decoded {} station(s)", results.len());

        let (date, time) = current_date_time();
        let rows: Vec<WsprSpotRow> = results.iter().map(|r| to_row(r, &time)).collect();

        {
            let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
            ui.set_slot(time.clone(), &rows);
            ui.update_status(|u| {
                u.band_label = heapless::String::try_from(band.label).unwrap_or_default();
                u.dial_mhz = band.dial_mhz;
                u.wsprnet_enabled =
                    mfsk_app_shared::wsprnet::SpotSink::from_config(Some(&settings.wsprnet_spot_config))
                        .is_enabled();
            });
        }

        let ntp_synced = WSPR_UI.lock().expect("WSPR_UI mutex poisoned").ntp_synced;
        if ntp_synced {
            report_to_wsprnet(&results, &settings, band, &date, &time);
        } else {
            log::info!("wspr_app: NTP never synced this run — skipping wsprnet report");
        }

        let elapsed_us = now_us() - slot_start_us;
        let remaining_ms = ((SLOT_US - elapsed_us).max(0) / 1000) as u32;
        if remaining_ms > 0 {
            FreeRtos::delay_ms(remaining_ms);
        }
    }
}

// ── Decode-result plumbing ───────────────────────────────────────────

struct ParsedMsg {
    call: heapless::String<13>,
    grid: heapless::String<6>,
    dbm: i8,
}

/// `WsprResult.message` is the decoded 50-bit payload rendered as
/// text — `"<call> <grid> <dbm>"` for a type-1 message. Split rather
/// than re-derive: the decoder already did the unpacking, and both
/// the UI row and the wsprnet report want the same three fields back
/// out (same approach `wspr_bench.rs`'s `report_spots` uses).
fn parse_message(msg: &str) -> ParsedMsg {
    let mut it = msg.split_whitespace();
    let call = it.next().unwrap_or("");
    let grid = it.next().unwrap_or("");
    let dbm: i8 = it.next().and_then(|d| d.parse().ok()).unwrap_or(0);
    ParsedMsg {
        call: heapless::String::try_from(call).unwrap_or_default(),
        grid: heapless::String::try_from(grid).unwrap_or_default(),
        dbm,
    }
}

fn to_row(r: &WsprResult, hhmm: &heapless::String<4>) -> WsprSpotRow {
    let parsed = parse_message(&r.message.to_string());
    WsprSpotRow {
        utc_hhmm: hhmm.clone(),
        call: parsed.call,
        grid: parsed.grid,
        power_dbm: parsed.dbm,
        freq_offset_hz: r.freq_hz,
        snr_db: r.snr_db as i8,
        dt_sec: r.dt_sec,
        drift_hz: r.drift_hz as i8,
    }
}

/// Off by default (`SpotSink::Disabled`) or missing callsign both
/// skip silently by design — `settings.rs`'s own doc comment: an
/// empty callsign means "not configured," not an error to surface
/// every 2 minutes in the log.
fn report_to_wsprnet(results: &[WsprResult], settings: &Settings, band: &WsprBand, date: &str, time: &str) {
    use mfsk_app_shared::wsprnet::{report_slot, Mode, Reporter, Spot, SpotSink};

    let sink = SpotSink::from_config(Some(settings.wsprnet_spot_config.as_str()));
    if !sink.is_enabled() {
        return;
    }
    if settings.call.is_empty() {
        log::warn!("wspr_app: wsprnet reporting is on but no callsign is set — skipping this slot");
        return;
    }

    let reporter = Reporter {
        call: settings.call.to_string(),
        grid: settings.grid.to_string(),
        dial_mhz: band.dial_mhz,
        version: format!("mfsk-core-{}", mfsk_core::VERSION),
        mode: Mode::Wspr2,
    };

    let spots: Vec<Spot> = results
        .iter()
        .map(|r| {
            let parsed = parse_message(&r.message.to_string());
            Spot {
                date: date.to_string(),
                time: time.to_string(),
                call: parsed.call.to_string(),
                grid: parsed.grid.to_string(),
                dbm: parsed.dbm as i32,
                snr_db: r.snr_db as i32,
                dt_sec: r.dt_sec,
                drift_hz: r.drift_hz as i32,
                audio_hz: r.freq_hz,
            }
        })
        .collect();

    let bodies = report_slot(&spots, &reporter, &sink);
    log::info!("wspr_app: wsprnet reported {} spot(s)", bodies.len());
}

// ── Clock formatting ─────────────────────────────────────────────────

fn current_hhmmss() -> heapless::String<8> {
    let mut s = heapless::String::new();
    if let Ok(dur) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        let (_, _, _, h, m, sec) = civil_from_unix(dur.as_secs() as i64);
        let _ = write!(&mut s, "{h:02}:{m:02}:{sec:02}");
    } else {
        let _ = s.push_str("--:--:--");
    }
    s
}

/// `(yyMMdd, HHmm)` — wsprnet's `date`/`time` fields, and the second
/// also doubles as [`WsprSpotRow::utc_hhmm`]. Meaningless (reads
/// whatever the unsynced system clock happens to hold) unless NTP
/// synced this run — callers gate on that separately rather than this
/// function returning an `Option`, since the WSPR UI still wants
/// *some* 4 digits to show even before/without a sync.
fn current_date_time() -> (heapless::String<6>, heapless::String<4>) {
    let mut date = heapless::String::new();
    let mut time = heapless::String::new();
    if let Ok(dur) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        let (y, mo, d, h, mi, _s) = civil_from_unix(dur.as_secs() as i64);
        let _ = write!(&mut date, "{:02}{mo:02}{d:02}", y.rem_euclid(100));
        let _ = write!(&mut time, "{h:02}{mi:02}");
    } else {
        let _ = date.push_str("000000");
        let _ = time.push_str("0000");
    }
    (date, time)
}
