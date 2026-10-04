// SPDX-License-Identifier: GPL-3.0-only
//! The FT8 controller, and every mode that is not one of the other
//! three receivers.
//!
//! This one used to be `main` itself — the dispatch in `main.rs` sent
//! WSPR, FST4 and FT4 away and then fell through into the FT8
//! controller's own boot inline. So the receiver with the most
//! behaviour was the one with no name, and the differences between it
//! and the other three (a hand-rolled network sequence, no watchdog
//! deinit, a WiFi decision made three steps earlier than everyone
//! else's) read as properties of `main` rather than as choices. They
//! are choices, and they are in one place now — see `crate::boot`.
//!
//! **Not only `Uac`.** `Decode` (the WAV replay demo) and the handful
//! of bring-up modes inherited from `m5stack-s3-app` all land here,
//! because what they have in common is the panel: `display::run_log_panel`
//! is the main loop, and whether a decode pipeline runs underneath it
//! is a per-mode question this file answers rather than a receiver of
//! its own.

use mfsk_app_shared::boot_mode::BootMode;

use crate::boot::{BootCtx, Display, Panel, Receiver};

pub struct Ft8Controller;

impl Receiver for Ft8Controller {
    const TAG: &'static str = "ft8_app";

    /// **Left on, deliberately.** The other three receivers deinit it;
    /// this one never has, and its `task_wdt(IDLE0)` lines during a
    /// cold acquisition are a watched symptom (four per acquisition,
    /// stable across sessions). Turning the watchdog off here would
    /// have deleted that signal as a side effect of a refactor.
    const DEINIT_WDT: bool = false;

    /// In host mode the serial console goes away when
    /// `usb_host_install` returns, and everything interesting about
    /// enumeration is logged in the few hundred milliseconds after it.
    /// Bounded, and skipped entirely when no sink is coming.
    const WAIT_FOR_LOG_SINK: bool = true;

    fn prepare(ctx: &BootCtx) {
        // Ordering, not size, is what makes this work: after WiFi and
        // the USB host have taken their share the largest free internal
        // block is 31,744 B, and before they start it is 155,648 B
        // (both measured on this board, #163). Reserving here also
        // means a binary that carries several modes only ever allocates
        // the one it booted into — see `embedded_shared::worker_arena`.
        if matches!(ctx.mode, BootMode::Decode | BootMode::Uac)
            && !embedded_shared::internal_pool::reserve_arena()
        {
            log::error!("decode scratch reservation failed — the pipeline will abort on first use");
        }
        let _ = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    }

    fn attach_panel(_ctx: &BootCtx, display: Display) -> Panel {
        // The panel *is* this receiver's main loop — it owns the touch
        // poll, the mode picker and the USB host bring-up.
        Panel::DrawsLast(display)
    }

    fn net_config(ctx: &BootCtx) -> Option<crate::net::Config> {
        // Which modes have any use for a network. Unchanged from the
        // `matches!(mode, Wifi | Uac)` this replaced: `Decode` replays
        // a baked recording and wants its slots, not a log sink.
        if !matches!(ctx.mode, BootMode::Wifi | BootMode::Uac) {
            return None;
        }
        Some(crate::net::Config {
            name: "ft8_app::net",
            // **Unmeasured, so unchanged.** `MIN_MODEM` is what cost
            // `fst4_app` nothing and bought it 33 → 53 s; whether the
            // FT8 controller pays the same price with an associated
            // idle STA has never been measured, and this refactor is
            // not the place to find out. Issue #381 carries the A/B.
            power_save: false,
            // `AIR DT` means the phase comes off the band, so NTP would
            // spend its timeout disciplining a clock the grid is not
            // following. The RTC still holds the minute for the log.
            ntp: crate::grid_source() != mfsk_app_shared::grid_src::GridSource::AirDt,
            without: if matches!(ctx.mode, BootMode::Uac) {
                "no UDP log, and in UAC mode that is the only console \
                 (USB-Serial-JTAG detached on usb_host_install)"
            } else {
                "no UDP log, no NTP, no config page"
            },
            bringup: crate::net::Bringup::Connect,
            // Off: measured 2026-09-22 (SIM, `logs/storage_ft8sim*`), a
            // resident httpd costs ~4.3 KB of internal DRAM for a page
            // needed once per activation. The logs are fetched from a
            // server started on demand, not one left listening.
            http: false,
            on_ntp: |synced| log::info!("ft8_app: NTP synced = {synced}"),
        })
    }

    fn start(ctx: &BootCtx) {
        match ctx.mode {
            BootMode::Decode => {
                crate::log_free_internal("pre-thread-spawn");
                spawn_decode(|| crate::decode_pipeline::run());
            }
            BootMode::Uac => {
                crate::log_free_internal("pre-thread-spawn");
                crate::uac::set_grid_fix_nvs(ctx.nvs_part.clone());
                sim_feed_if_asked();
                // Spawned before `uac::start_host()` installs the UAC
                // driver, so the chunk queue is live when the reader
                // thread (created on the first `RxConnected`) calls
                // `set_chunk_q`; until then samples are dropped.
                spawn_decode(|| {
                    crate::decode_pipeline::run_with_source("uac", |q| crate::uac::set_chunk_q(q))
                });
            }
            other => log::info!("decode_pipeline skipped ({})", other.label()),
        }
    }

    fn run_forever(ctx: BootCtx, panel: Panel) -> ! {
        let Panel::DrawsLast(display) = panel else {
            unreachable!("attach_panel hands this receiver's panel back to run_forever");
        };
        // The panel above the decode, so the screen never stops — see
        // `display::PANEL_PRIORITY` for what it costs FT8 (nothing
        // measurable).
        unsafe {
            esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), crate::display::PANEL_PRIORITY)
        };
        crate::display::run_log_panel(
            display.i2c0,
            display.spi2,
            display.pins,
            &crate::FANOUT,
            ctx.nvs,
            ctx.mode,
        )
    }
}

/// **Pinned to PRO_CPU.** The two-core decode split is this thread on
/// one core and `dsp_worker` on the other, and `dsp_worker` is pinned
/// to APP_CPU: left unpinned this thread can be scheduled there too,
/// and the half-and-half of coarse sync and stage 3 becomes serial. It
/// has in practice always run on core 0 (every watchdog dump during
/// acquisition reads `CPU 0: decode`, `CPU 1: IDLE1`), so this pins
/// where it already sits rather than moving it — the guarantee is what
/// is new, not the placement.
fn spawn_decode(body: impl FnOnce() + Send + 'static) {
    let spawned = crate::board::spawn_named_tuned(
        c"decode",
        32 * 1024,
        None,
        Some(esp_idf_svc::hal::cpu::Core::Core0),
        body,
    );
    if let Err(e) = spawned {
        log::error!("decode_pipeline spawn failed ({e})");
    }
}

/// `MFSK_CORES3_SIM`: feed a baked slot through the real `AudioSink`
/// path instead of a radio. The replay routes that bypassed the sink
/// could not exercise the slot machinery at all, which is why this
/// exists — see `uac::spawn_sim_feed`.
fn sim_feed_if_asked() {
    if option_env!("MFSK_CORES3_SIM").is_none() {
        return;
    }
    if option_env!("MFSK_SIM_NO_CLOCK").is_some() {
        mfsk_app_shared::time_sync::suppress_clock(true);
        log::warn!("SIM: clock suppressed — grid must recover from the air");
    }
    let offset_ms: usize = option_env!("MFSK_SIM_OFFSET_MS")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    crate::uac::spawn_sim_feed(
        crate::uac::SimSource::Wav(crate::decode_pipeline::QSO_WAVS[0]),
        crate::uac::SLOT_SAMPLES_12K,
        offset_ms * 12,
    );
}
