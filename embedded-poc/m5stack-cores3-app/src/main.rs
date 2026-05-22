//! M5Stack CoreS3 FT8 controller — entry point (Phase 0-Core).
//!
//! Mirrors `m5stack-core2-app/src/main.rs`. CoreS3 deltas vs Core2:
//!   - PMIC: AXP2101 + AW9523B (vs AXP192); LCD RST via AW9523B.
//!   - SPI2 (FSPI) on pins 36/37/3/35 for LCD (vs Core2 SPI3 18/23/5/15).
//!   - I2C0: SDA=12, SCL=11 (vs Core2 SDA=21, SCL=22).
//!   - No GPIO buttons (same as Core2); NVS-only boot mode.
//!   - USB-OTG capable (Phase 1-Core); no audio in Phase 0.

#![allow(dead_code)]

mod board;
mod decode_pipeline;
mod display;
mod pmic;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{
    heap_caps_get_free_size, heap_caps_get_largest_free_block, MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use log::LevelFilter;

use mfsk_app_shared::boot_mode;
use mfsk_app_shared::log_sink::{FanoutLogger, LogFanout};
use mfsk_app_shared::udp_log;
use mfsk_app_shared::wifi;

pub fn log_free_internal(label: &str) {
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let free = unsafe { heap_caps_get_free_size(caps) };
    let largest = unsafe { heap_caps_get_largest_free_block(caps) };
    log::info!("[mem] {label} free_internal={free} largest={largest}");
}

static FANOUT: LogFanout = LogFanout::new();
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-cores3-app boot ===");
    log::info!("phase 0-core: NVS boot-mode, wav_sim decode, ILI9342C LCD");
    log::info!("build-stamp 2026-05-23-cores3-phase0");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    let mode = boot_mode::determine_no_override(&nvs);
    log::info!("boot_mode: {} (NVS-only on CoreS3)", mode.label());

    let mut _wifi_slot: Option<wifi::WifiHandle> = None;
    let wifi_should_start = mode == boot_mode::BootMode::Wifi && !WIFI_SSID.is_empty();
    if mode == boot_mode::BootMode::Wifi && WIFI_SSID.is_empty() {
        log::warn!(
            "boot_mode=WIFI but WIFI_SSID empty (no cfg.toml) — flip NVS to DECODE"
        );
    }
    if wifi_should_start {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        match wifi::connect_sta(
            peripherals.modem,
            sysloop,
            Some(nvs_part.clone()),
            WIFI_SSID,
            WIFI_PSK,
        ) {
            Ok(handle) => {
                let target_ip: std::net::IpAddr = if UDP_LOG_TARGET.is_empty()
                    || UDP_LOG_TARGET == "auto"
                {
                    std::net::IpAddr::V4(handle.subnet_broadcast)
                } else {
                    match UDP_LOG_TARGET.parse() {
                        Ok(ip) => ip,
                        Err(e) => {
                            log::warn!(
                                "UDP_LOG_TARGET '{UDP_LOG_TARGET}' parse failed ({e}); using subnet bcast"
                            );
                            std::net::IpAddr::V4(handle.subnet_broadcast)
                        }
                    }
                };
                let port: u16 = UDP_LOG_PORT.parse().unwrap_or(9999);
                let addr = std::net::SocketAddr::new(target_ip, port);
                match udp_log::UdpLogSink::new(addr) {
                    Ok(sink) => {
                        if let Ok(mut slot) = FANOUT.udp.try_lock() {
                            *slot = Some(sink);
                        }
                        FANOUT.drain_staging_to_udp();
                        log::info!("UDP log sink up → {addr}");
                    }
                    Err(e) => log::warn!("UDP socket bind failed: {e}"),
                }
                _wifi_slot = Some(handle);
            }
            Err(e) => log::warn!("WiFi STA failed: {e:#} — UDP log disabled"),
        }
    }

    if mode == boot_mode::BootMode::Decode {
        log_free_internal("pre-thread-spawn");
        let pipeline_spawn = std::thread::Builder::new()
            .stack_size(32 * 1024)
            .spawn(|| decode_pipeline::run());
        if let Err(e) = pipeline_spawn {
            log::error!("decode_pipeline spawn failed ({e})");
        }
    } else {
        log::info!("decode_pipeline skipped (BootMode::Wifi)");
    }

    display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi2,
        peripherals.pins,
        &FANOUT,
        nvs,
        mode,
    )
}
