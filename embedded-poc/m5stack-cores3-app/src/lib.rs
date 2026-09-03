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
pub mod coredump;
pub mod decode_pipeline;
pub mod display;
pub mod esp_log_bridge;
pub mod log_slot;
pub mod net;
pub mod pmic;
pub mod rtc;
pub mod touch;
pub mod uac;

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

pub fn wifi_enabled_for_this_boot() -> bool {
    WIFI_ENABLED.load(std::sync::atomic::Ordering::Acquire)
}
pub static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

pub const WIFI_SSID: &str = env!("WIFI_SSID");
pub const WIFI_PSK: &str = env!("WIFI_PSK");
pub const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
pub const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");
pub const BOOT_MODE_DEFAULT: &str = env!("BOOT_MODE_DEFAULT");

/// SNTP server for the FT8 controller. WSPR and FST4 take theirs from
/// NVS settings, which this app has no page for; `pool.ntp.org` is what
/// their own default is.
pub const NTP_SERVER: &str = "pool.ntp.org";
/// Long enough for a first sync over WiFi, short enough that a boot
/// with no route still reaches the decode loop.
pub const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;
