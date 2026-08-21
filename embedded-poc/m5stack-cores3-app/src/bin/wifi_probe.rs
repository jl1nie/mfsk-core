//! Standalone WiFi-only diagnostic, plus an optional real decode load.
//!
//! No display/DDC/HTTP/NTP/UDP-log — just WiFi driver bring-up, then a
//! tight loop of single connect attempts on their own task, each timed
//! and tallied, with an *optional* second task running the real
//! `run_scan` decode loop (same function `wspr-app`'s `scan_loop`
//! calls, same golden baseband, same core/priority) to see whether
//! that concurrent CPU/internal-DRAM load changes WiFi's success rate.
//! Same "isolate the thing under investigation" rationale
//! `lcd_minimal.rs` used for the LCD investigation.
//!
//! Built 2026-08-15 for the WiFi failure-mode investigation (see
//! `wspr-app`'s own design memory for the three distinct low-level
//! failure points already found). First cut (WiFi-only) measured
//! 27/29 (93%) success with clean ~6 s connects — much better than
//! `wspr-app`'s own repeated 0/4-8 failures in real operation,
//! pointing at decode-workload contention rather than pure AP
//! flakiness. `WITH_DECODE_LOAD` below adds the real decode task to
//! test that directly, on the user's own suggestion ("単に接続してから
//! decode回せば良いだけな気がする") rather than building a synthetic
//! approximation.
//!
//! **Second cut, same day**: the first `WITH_DECODE_LOAD` version ran
//! the probe loop inline on `main` (default priority, same core 0 as
//! the decode task) — measured 4/4 success but ~45 s/connect instead
//! of ~6 s, a real slowdown but not a failure. User's own read: that
//! placement doesn't match `wspr-app`'s actual priorities (network
//! task at [`NETWORK_PRIORITY`], core 1; decode at priority 5, core
//! 0), so it under-tests starvation — `BlockingWifi`'s own internal
//! waits (`CONNECT_TIMEOUT` = 15 s each for `connect()`/
//! `wait_netif_up()`) don't care about wall-clock slowdown *unless*
//! it's severe enough to blow through that 15 s budget, which a
//! same-core low-vs-high-priority fight might not reach even at 45 s
//! total (that total is scan+connect+dhcp summed, not one 15 s wait
//! necessarily breached) but a *true* priority-starved cross-core
//! setup might. The probe loop now runs on its own task, pinned to
//! core 1 at [`NETWORK_PRIORITY`] — the same core/priority
//! `wspr-app`'s real network task uses — instead of inline on `main`,
//! so this is now a faithful placement match, not just "some
//! concurrent load exists."
//!
//! Each WiFi cycle: `connect_with_retry(..., Some(1))` (exactly one
//! attempt, no internal retry — this tool *is* the retry loop, so it
//! can tally each attempt individually) → log outcome + elapsed time
//! → `disconnect()` → 3 s pause → repeat forever. Running totals
//! (successes/attempts) print every cycle so a multi-minute capture
//! gives a real success-rate number instead of one anecdote.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use embedded_shared::apps::wspr_scan::{load_baseband, run_scan, NBB};

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");

/// Pause between cycles — long enough to let the AP's own state
/// settle between a `disconnect()` and the next `connect()` (matches
/// `wifikey2::WifiManager::reconnect()`'s own 3 s), short enough that
/// a multi-minute capture still covers dozens of attempts.
const CYCLE_DELAY_MS: u32 = 3_000;

/// Flip to `true` to spawn the concurrent decode task — see this
/// file's own top doc comment. Off by default so this tool still
/// works as the pure WiFi-only baseline it started as.
const WITH_DECODE_LOAD: bool = true;

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");

/// Same stack `wspr-app`/`wspr-bench` settled on for `run_scan`'s full
/// pass 0/1/2 sequence.
const DECODE_STACK: u32 = 72 * 1024;
/// Stack for the probe task — comparable to `wspr-app`'s own
/// `NETWORK_STACK` (its `connect_with_retry` call site).
const PROBE_STACK: u32 = 24 * 1024;
/// Matches `wspr-app`'s own `NETWORK_PRIORITY` exactly — the whole
/// point of this second cut (see this file's top doc comment) is
/// testing that real priority, not an arbitrary one.
const NETWORK_PRIORITY: u32 = 2;

static DECODE_GO: AtomicBool = AtomicBool::new(false);
static PROBE_GO: AtomicBool = AtomicBool::new(false);

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== wifi-probe boot (WITH_DECODE_LOAD={WITH_DECODE_LOAD}) ===");
    if WIFI_SSID.is_empty() {
        log::error!("wifi-probe: WIFI_SSID empty (no cfg.toml) — nothing to probe");
        loop {
            FreeRtos::delay_ms(60_000);
        }
    }

    if WITH_DECODE_LOAD {
        // Same reasoning `wspr-app`/`wspr-bench` document: the
        // candidate loop is compute-bound for tens of seconds with no
        // yield point.
        let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
        log::info!("wifi-probe: task watchdog deinit -> {r}");
        embedded_shared::wspr_dual_core::init();

        // Claim the decode task's 72 KiB stack before WiFi driver
        // init gets a chance to fragment internal DRAM — same
        // ordering lesson this whole investigation kept re-learning
        // in `wspr-app` itself.
        spawn_decode_task();
    }

    let peripherals = Peripherals::take().expect("peripherals taken twice");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let sysloop = EspSystemEventLoop::take().expect("sysloop");

    let wifi = mfsk_app_shared::wifi::wifi_driver_init(peripherals.modem, sysloop, Some(nvs_part))
        .expect("wifi-probe: WiFi driver init failed — nothing to probe this boot");
    log::info!("wifi-probe: driver up");

    // Probe task: its own stack, PSRAM-backed for the same reason
    // `wspr-app`'s own network task is (internal DRAM is tight once
    // the decode task's stack plus WiFi's own driver buffers are
    // live) — core 1, [`NETWORK_PRIORITY`], matching `wspr-app`'s real
    // placement exactly. See this file's top doc comment for why that
    // match is the point of this second cut.
    spawn_probe_task(ProbeCtx { wifi });

    // Release both tasks together — matches `wspr-app`'s own
    // `SCAN_GO` release point (right after every task is spawned),
    // so decode and the probe loop genuinely overlap the way they
    // would in the real app.
    DECODE_GO.store(true, Ordering::Release);
    PROBE_GO.store(true, Ordering::Release);

    loop {
        FreeRtos::delay_ms(1000);
    }
}

// ── Probe task ────────────────────────────────────────────────────────

struct ProbeCtx {
    wifi: esp_idf_svc::wifi::BlockingWifi<esp_idf_svc::wifi::EspWifi<'static>>,
}

extern "C" fn probe_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_probe_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut ProbeCtx) };
    probe_loop(*ctx);
}

fn spawn_probe_task(ctx: ProbeCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let caps = esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(probe_task_entry),
            c"probe_wifi".as_ptr(),
            PROBE_STACK,
            ptr,
            NETWORK_PRIORITY,
            core::ptr::null_mut(),
            1, // core 1 — matches `wspr-app`'s own network task.
            caps,
        )
    };
    if created != 1 {
        log::error!("wifi-probe: failed to create probe_wifi task");
    }
}

fn probe_loop(mut ctx: ProbeCtx) -> ! {
    while !PROBE_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("wifi-probe: probe task starting");

    let mut attempt: u32 = 0;
    let mut successes: u32 = 0;
    loop {
        attempt += 1;
        let t0_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        let outcome =
            mfsk_app_shared::wifi::connect_with_retry(&mut ctx.wifi, WIFI_SSID, WIFI_PSK, Some(1));
        let elapsed_ms = (unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t0_us) / 1000;

        match outcome {
            Ok(info) => {
                successes += 1;
                log::info!(
                    "PROBE #{attempt}: SUCCESS ip={} in {elapsed_ms} ms — running total {successes}/{attempt}",
                    info.ip
                );
            }
            Err(e) => {
                log::warn!(
                    "PROBE #{attempt}: FAIL ({e:#}) in {elapsed_ms} ms — running total {successes}/{attempt}"
                );
            }
        }

        // Best-effort — if the attempt failed, the driver may already
        // be disconnected; if it succeeded, tear it down deliberately
        // so the next cycle starts from a clean, comparable state
        // rather than measuring a "reconnect while already up" no-op.
        let _ = ctx.wifi.disconnect();
        FreeRtos::delay_ms(CYCLE_DELAY_MS);
    }
}

// ── Decode load task (optional) ──────────────────────────────────────

extern "C" fn decode_task_entry(_arg: *mut core::ffi::c_void) {
    decode_loop();
}

fn spawn_decode_task() {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(decode_task_entry),
            c"probe_decode".as_ptr(),
            DECODE_STACK,
            core::ptr::null_mut(),
            5, // same priority `wspr-app`'s own scan task runs at.
            core::ptr::null_mut(),
            0, // core 0 — same as `wspr-app`'s own scan task; the WiFi
               // probe loop above runs on `main`, also core 0
               // (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y`), so this is
               // deliberately the *worse* case for contention, not
               // `wspr-app`'s actual core-1-for-network split — the
               // point here is measuring the load itself, not
               // reproducing the exact placement.
        )
    };
    if created != 1 {
        log::error!("wifi-probe: failed to create probe_decode task");
    }
}

/// Repeatedly decodes the same golden baseband forever — real
/// `run_scan` cost (coarse search + Fano/OSD + subtract, same as
/// `wspr-app`'s own slot-0 decode), just looped instead of paced to a
/// WSPR slot boundary, to keep the CPU/internal-DRAM load as close to
/// continuous as `wspr-app`'s own DDC-fed slots get.
fn decode_loop() -> ! {
    while !DECODE_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("wifi-probe: decode load task starting");

    let mut idat = vec![0.0f32; NBB];
    let mut qdat = vec![0.0f32; NBB];
    let mut pass: u32 = 0;
    loop {
        pass += 1;
        load_baseband(GOLDEN_BASEBAND, &mut idat, &mut qdat);
        let (stats, results) = run_scan(&mut idat, &mut qdat);
        for s in &stats {
            s.log();
        }
        log::info!(
            "wifi-probe: decode pass {pass} done, {} station(s)",
            results.len()
        );
    }
}
