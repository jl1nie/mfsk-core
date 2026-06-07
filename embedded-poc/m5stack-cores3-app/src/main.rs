//! M5Stack CoreS3 FT8 controller — entry point (Phase 1-Core).
//!
//! Mirrors `m5stack-core2-app/src/main.rs`. CoreS3 deltas vs Core2:
//!   - PMIC: AXP2101 + AW9523B (vs AXP192); LCD RST via AW9523B.
//!   - SPI2 (FSPI) on pins 36/37/3/35 for LCD (vs Core2 SPI3 18/23/5/15).
//!   - I2C0: SDA=12, SCL=11 (vs Core2 SDA=21, SCL=22).
//!   - No GPIO buttons (same as Core2); NVS-only boot mode.
//!   - USB-OTG host via AW9523B P0_1 (BUS_OUT_EN) — Phase 1-Core UAC.

#![allow(dead_code)]

mod board;
mod decode_pipeline;
mod display;
mod pmic;
mod uac;

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
const BOOT_MODE_DEFAULT: &str = env!("BOOT_MODE_DEFAULT");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-cores3-app boot ===");
    log::info!("phase 1-core: UAC host (AW9523B BUS_OUT_EN + usb_host_uac), wav_sim decode, ILI9342C LCD");
    log::info!("build-stamp 2026-05-23-cores3-phase1");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    if !BOOT_MODE_DEFAULT.is_empty() {
        let target = boot_mode::BootMode::from_cfg_str(BOOT_MODE_DEFAULT);
        let current = boot_mode::read(&nvs);
        if current != target {
            log::info!("boot_mode: cfg override {} → {}", current.label(), target.label());
            let _ = boot_mode::write(&nvs, target);
        }
    }
    let mode = boot_mode::determine_no_override(&nvs);
    log::info!("boot_mode: {} (NVS-only on CoreS3)", mode.label());

    let mut _wifi_slot: Option<wifi::WifiHandle> = None;
    let needs_wifi = matches!(mode, boot_mode::BootMode::Wifi | boot_mode::BootMode::Uac);
    let wifi_should_start = needs_wifi && !WIFI_SSID.is_empty();
    if needs_wifi && WIFI_SSID.is_empty() {
        log::warn!(
            "boot_mode={} but WIFI_SSID empty (no cfg.toml) — UDP log unavailable{}",
            mode.label(),
            if mode == boot_mode::BootMode::Uac {
                "; serial console also gone in UAC mode (USB-Serial-JTAG detached on usb_host_install)"
            } else {
                ""
            }
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

    match mode {
        boot_mode::BootMode::Decode => {
            log_free_internal("pre-thread-spawn");
            let pipeline_spawn = std::thread::Builder::new()
                .stack_size(32 * 1024)
                .spawn(|| decode_pipeline::run());
            if let Err(e) = pipeline_spawn {
                log::error!("decode_pipeline spawn failed ({e})");
            }
        }
        boot_mode::BootMode::Uac => {
            // Spawn the decode pipeline first so the chunk_q is live before
            // uac::start_host() installs the UAC driver. The reader thread
            // (spawned on first RxConnected) calls set_chunk_q once the
            // queue handle is registered, dropping samples until then.
            // Note: uac::start_host() itself is called in display::run_log_panel
            // after pmic::init() drives BUS_OUT_EN HIGH.
            log_free_internal("pre-thread-spawn");
            let pipeline_spawn = std::thread::Builder::new()
                .stack_size(32 * 1024)
                .spawn(|| decode_pipeline::run_with_source(|q| uac::set_chunk_q(q)));
            if let Err(e) = pipeline_spawn {
                log::error!("decode_pipeline (Uac) spawn failed ({e})");
            }
        }
        _ => {
            log::info!("decode_pipeline skipped ({})", mode.label());
        }
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
