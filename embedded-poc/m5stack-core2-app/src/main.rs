//! M5Stack Core2 FT8 controller — entry point.
//!
//! Sibling of `m5stack-s3-app/src/main.rs`. Differences in Phase 2:
//!   - No boot-time KEY1 override (Core2 has no GPIO buttons), so we
//!     call `boot_mode::determine_no_override` instead of `determine`.
//!   - LCD bring-up routes through `pmic::init_lcd_power` (AXP192)
//!     instead of the S3 M5PM1 sequence.
//!   - No audio I2S init (decode pipeline uses wav_sim only).

#![allow(dead_code)]

mod board;
mod decode_pipeline;
mod display;
mod pmic;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{
    heap_caps_aligned_alloc, heap_caps_get_free_size, heap_caps_get_largest_free_block,
    MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use log::LevelFilter;
use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;

use mfsk_app_shared::boot_mode;
use mfsk_app_shared::log_sink::{FanoutLogger, LogFanout};
use mfsk_app_shared::udp_log;
use mfsk_app_shared::wifi;

/// Same DRAM probe shape as the s3-app sibling — useful for diffing
/// internal-DRAM headroom between LX6 (~280 KB) and LX7 (~512 KB)
/// when comparing log captures.
pub fn log_free_internal(label: &str) {
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let free = unsafe { heap_caps_get_free_size(caps) };
    let largest = unsafe { heap_caps_get_largest_free_block(caps) };
    log::info!("[mem] {label} free_internal={free} largest={largest}");
}

/// Allocate one 16-byte-aligned `BASIS_SCRATCH_LEN`-element `i16`
/// buffer in internal DRAM. Panics on alloc failure or alignment
/// violation; both indicate the heap is too fragmented for the asm
/// dot product to hit 1 cycle/sample, which would silently regress
/// the decode wall-clock 2× rather than fail loud.
pub fn alloc_basis_dram(label: &str) -> &'static mut [i16] {
    const BYTES: usize = BASIS_SCRATCH_LEN * core::mem::size_of::<i16>();
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let raw = unsafe { heap_caps_aligned_alloc(16, BYTES, caps) } as *mut i16;
    assert!(!raw.is_null(), "BASIS {label} alloc failed ({BYTES} B internal DRAM)");
    assert_eq!(
        raw as usize & 0xF,
        0,
        "BASIS {label} not 16-byte aligned (asm dot product would degrade 2×)"
    );
    unsafe {
        core::ptr::write_bytes(raw, 0, BASIS_SCRATCH_LEN);
        core::slice::from_raw_parts_mut(raw, BASIS_SCRATCH_LEN)
    }
}

static FANOUT: LogFanout = LogFanout::new();
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

// Build-time WiFi config (see `build.rs` + `cfg.toml`). Empty SSID
// means cfg.toml is absent — boot still completes, UDP log just stays
// dark.
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-core2-app boot ===");
    log::info!("phase 2: NVS boot-mode, no GPIO override (Core2 has no buttons)");
    log::info!("build-stamp 2026-05-16-core2-phase2-scaffold");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    let mode = boot_mode::determine_no_override(&nvs);
    log::info!("boot_mode: {} (NVS-only on Core2)", mode.label());

    // ── WiFi STA + UDP log sink — same path as the s3-app sibling. ──
    // `_wifi_slot` is a local `mut Option<WifiHandle>` (not `static
    // mut`) because `main` never returns: the eventual
    // `display::run_log_panel` call below is `-> !`, so the local
    // outlives every consumer of the WiFi handle. Avoids the
    // `unsafe { static mut ... }` block + `allow(static_mut_refs)`
    // that the s3-app sibling needs (s3 cycles modes via reboot, but
    // Core2 stays in one mode for the life of the binary). Gemini PR
    // #76 review.
    let mut _wifi_slot: Option<wifi::WifiHandle> = None;
    let wifi_should_start = mode == boot_mode::BootMode::Wifi && !WIFI_SSID.is_empty();
    if mode == boot_mode::BootMode::Wifi && WIFI_SSID.is_empty() {
        log::warn!(
            "boot_mode=WIFI but WIFI_SSID empty (no cfg.toml) — UDP log unavailable; flip NVS via host to return to DECODE"
        );
    }
    if wifi_should_start {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        match wifi::connect_sta(peripherals.modem, sysloop, Some(nvs_part.clone()), WIFI_SSID, WIFI_PSK) {
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
        log::info!(
            "decode_pipeline skipped (BootMode::Wifi — BASIS alloc skipped, ~120 KB internal DRAM free for WiFi)"
        );
    }

    // Main thread = LCD render loop. Takes I2C0 (PMIC/IMU/touch bus),
    // SPI3 (LCD), and the GPIO bag. `peripherals.modem` is already
    // consumed by WiFi so we split fields individually.
    display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi3,
        peripherals.pins,
        &FANOUT,
        nvs,
        mode,
    )
}
