// SPDX-License-Identifier: GPL-3.0-or-later
//! The CoreS3 application's library half.
//!
//! Everything the binary is made of lives here so that **other bins in
//! this crate can reach it too** — `ft4-demo` needs `board` and
//! `display`, and a bin cannot see another bin's modules. Before this
//! split there was no lib target and each bin was an island, which is
//! why `audio_out` and the FT4 receiver could be written but not wired
//! to anything that draws a screen.
//!
//! `main.rs` is now only `fn main()`: boot mode, log fanout, WiFi, and
//! the dispatch into `apps`.

pub mod apps;
pub mod audio_out;
pub mod board;
pub mod boot;
pub mod civ_usb;
pub mod coredump;
pub mod decode_pipeline;
pub mod display;
pub mod esp_log_bridge;
pub mod log_slot;
pub mod net;
pub mod pmic;
pub mod rtc;
pub mod storage;
pub mod touch;
pub mod tx_bringup;
pub mod uac;
pub mod waterfall_feed;

use esp_idf_svc::sys::{
    heap_caps_get_free_size, heap_caps_get_largest_free_block, MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use log::LevelFilter;

use mfsk_app_shared::log_sink::{FanoutLogger, LogFanout};

pub fn log_free_internal(label: &str) {
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let free = unsafe { heap_caps_get_free_size(caps) };
    let largest = unsafe { heap_caps_get_largest_free_block(caps) };
    log::info!("[mem] {label} free_internal={free} largest={largest}");
}

pub static FANOUT: LogFanout = LogFanout::new();

/// この起動で WiFi を立ち上げるか。`display` が「ログ送信先を待つか」の
/// 判断に使う — 来ない sink を45秒待つのは、起動が45秒遅い受信機に
/// なるだけ。Refs #163.
pub static WIFI_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Where this boot takes the slot grid's phase from — the CONFIG page's
/// setting, read once at startup and published here so the picker can
/// mark it and `main` can act on it without re-opening NVS.
static GRID_SOURCE: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

pub fn set_grid_source(src: mfsk_app_shared::grid_src::GridSource) {
    GRID_SOURCE.store(
        match src {
            mfsk_app_shared::grid_src::GridSource::Ntp => 0,
            mfsk_app_shared::grid_src::GridSource::AirDt => 1,
        },
        std::sync::atomic::Ordering::Release,
    );
}

pub fn grid_source() -> mfsk_app_shared::grid_src::GridSource {
    match GRID_SOURCE.load(std::sync::atomic::Ordering::Acquire) {
        1 => mfsk_app_shared::grid_src::GridSource::AirDt,
        _ => mfsk_app_shared::grid_src::GridSource::Ntp,
    }
}

/// Whether this boot was *asked* to bring WiFi up — the CONFIG page's
/// other setting, published the same way and for the same two readers
/// (the picker marks it, `main` acts on it).
///
/// Not the same question as [`wifi_enabled_for_this_boot`], which
/// answers "is a log sink coming": a boot mode with no use for WiFi,
/// or a build with an empty `WIFI_SSID`, leaves that false while this
/// still says `On`. Issue #381.
static WIFI_PREF: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

pub fn set_wifi_pref(pref: mfsk_app_shared::wifi_pref::WifiPref) {
    WIFI_PREF.store(
        match pref {
            mfsk_app_shared::wifi_pref::WifiPref::On => 0,
            mfsk_app_shared::wifi_pref::WifiPref::Off => 1,
        },
        std::sync::atomic::Ordering::Release,
    );
}

pub fn wifi_pref() -> mfsk_app_shared::wifi_pref::WifiPref {
    match WIFI_PREF.load(std::sync::atomic::Ordering::Acquire) {
        1 => mfsk_app_shared::wifi_pref::WifiPref::Off,
        _ => mfsk_app_shared::wifi_pref::WifiPref::On,
    }
}

/// What this boot does about WiFi and why: the CONFIG page's `WIFI` row, unless
/// `TIME: AIR DT` is selected, which turns it off (#381,
/// `mfsk_app_shared::wifi_policy`). The one place the rule is applied.
pub fn wifi_decision() -> mfsk_app_shared::wifi_policy::WifiDecision {
    mfsk_app_shared::wifi_policy::decide(
        wifi_pref().enabled(),
        grid_source() == mfsk_app_shared::grid_src::GridSource::AirDt,
    )
}

/// The `WIFI` row as it is *running*, for the panel: `WIFI: OFF` under `TIME:
/// AIR DT` whatever is stored, so the picker does not show ON on a board with
/// no radio. The stored choice is untouched (it is what applies under NTP).
pub fn effective_wifi_pref() -> mfsk_app_shared::wifi_pref::WifiPref {
    if wifi_decision().enabled() {
        mfsk_app_shared::wifi_pref::WifiPref::On
    } else {
        mfsk_app_shared::wifi_pref::WifiPref::Off
    }
}

/// One of the CONFIG page's settings, as handed to
/// [`commit_config_and_restart`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigChoice {
    Grid(mfsk_app_shared::grid_src::GridSource),
    Wifi(mfsk_app_shared::wifi_pref::WifiPref),
}

impl ConfigChoice {
    fn label(self) -> &'static str {
        match self {
            ConfigChoice::Grid(src) => src.label(),
            ConfigChoice::Wifi(pref) => pref.label(),
        }
    }

    fn write(
        self,
        nvs: &esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>,
    ) -> Result<(), esp_idf_svc::sys::EspError> {
        match self {
            ConfigChoice::Grid(src) => mfsk_app_shared::grid_src::write(nvs, src),
            ConfigChoice::Wifi(pref) => mfsk_app_shared::wifi_pref::write(nvs, pref),
        }
    }
}

/// Persist a CONFIG-page choice and restart into it.
///
/// Same shape as `boot_mode::commit_and_restart`, and for the same
/// reason: the panel tasks run on PSRAM stacks and an NVS write
/// disables the flash cache, which a PSRAM stack must not be holding.
pub fn commit_config_and_restart(
    nvs: std::sync::Arc<std::sync::Mutex<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>>,
    choice: ConfigChoice,
) {
    struct Req {
        nvs: std::sync::Arc<
            std::sync::Mutex<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>,
        >,
        choice: ConfigChoice,
    }

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `commit_config_and_restart` leaked exactly this pointer.
        let req = unsafe { Box::from_raw(arg as *mut Req) };
        match req.nvs.lock() {
            Ok(nvs) => match req.choice.write(&nvs) {
                Ok(()) => log::warn!("config: committed {} — restarting", req.choice.label()),
                Err(e) => log::error!("config write failed: {e} — not restarting"),
            },
            Err(e) => log::error!("config: NVS lock poisoned: {e} — not restarting"),
        }
        drop(req);
        // Let the line reach the log sink; in UAC mode that is the only
        // channel out of this board.
        unsafe { esp_idf_svc::sys::vTaskDelay(40) };
        // SAFETY: no arguments, does not return.
        unsafe { esp_idf_svc::sys::esp_restart() };
    }

    let ptr = Box::into_raw(Box::new(Req { nvs, choice })) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"config_save".as_ptr(),
            4096,
            ptr,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("could not spawn the config save task");
        drop(unsafe { Box::from_raw(ptr as *mut Req) });
    }
}

/// Whether this boot's receiver holds the USB host install until the
/// UDP log sink exists.
///
/// Only the FT8 controller does, and it is the reason the flag exists:
/// in host mode the serial console goes away the moment
/// `usb_host_install` returns, so everything interesting about
/// enumeration would land in the staging ring and be overwritten. The
/// other three receivers never waited — they dispatched before the
/// flag that used to gate it was even set — and a 45 s pause before
/// audio is not something a refactor should hand them. Published by
/// `boot::run` from `Receiver::WAIT_FOR_LOG_SINK`.
static WAIT_LOG_SINK: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

pub fn set_wait_for_log_sink(wait: bool) {
    WAIT_LOG_SINK.store(wait, std::sync::atomic::Ordering::Release);
}

pub fn wait_for_log_sink() -> bool {
    WAIT_LOG_SINK.load(std::sync::atomic::Ordering::Acquire)
}

pub fn wifi_enabled_for_this_boot() -> bool {
    WIFI_ENABLED.load(std::sync::atomic::Ordering::Acquire)
}
pub static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

pub const WIFI_SSID: &str = env!("WIFI_SSID");
pub const WIFI_PSK: &str = env!("WIFI_PSK");
pub const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
pub const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");
pub const BOOT_MODE_DEFAULT: &str = env!("BOOT_MODE_DEFAULT");

// `NTP_SERVER` and `NTP_SYNC_TIMEOUT_MS` used to live here, for the FT8
// controller's own NTP wait. That wait is `net::run`'s now, for every
// receiver, and it reads the server from the settings page rather than
// from a constant — the same page the FT8 controller's own network task
// serves. The default behind it is `pool.ntp.org`, which is what the
// constant said, so nothing moved for a board nobody has configured.
