//! M5StickS3 IC-705 FT8 controller — entry point.

#![allow(dead_code)]

mod audio;
mod board;
mod buttons;
mod civ;
mod decode_pipeline;
mod display;
mod pmic;
mod uac;

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

/// Probe internal DRAM. Phase 0.7: called pre/post BASIS alloc to
/// confirm the heap accounting matches the ~123 KB delta the static
/// `.bss` version reserved at boot.
pub fn log_free_internal(label: &str) {
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let free = unsafe { heap_caps_get_free_size(caps) };
    let largest = unsafe { heap_caps_get_largest_free_block(caps) };
    log::info!("[mem] {label} free_internal={free} largest={largest}");
}

/// Allocate a 16-byte-aligned `BASIS_SCRATCH_LEN`-element `i16` buffer
/// in internal DRAM and return it as a `'static mut` slice. Panics on
/// alloc failure or alignment violation — both indicate the heap is
/// too fragmented for the asm dot product to hit 1 cycle/sample, which
/// would silently regress wall-clock 2× rather than fail loud.
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
    // Zero-init to match the static `.bss` baseline.
    unsafe {
        core::ptr::write_bytes(raw, 0, BASIS_SCRATCH_LEN);
        core::slice::from_raw_parts_mut(raw, BASIS_SCRATCH_LEN)
    }
}

static FANOUT: LogFanout = LogFanout::new();
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

// Phase 0.6 build-time config (see `build.rs` + `cfg.toml`). Empty SSID
// means cfg.toml is absent — boot still completes, UDP log just stays
// dark.
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // EspLogger を init すると log::set_logger を奪われ、自前の
    // FanoutLogger が install 失敗 → LCD に何も流れなくなる。
    // C-side ESP_LOG (タイムスタンプ付き UART 出力) は init せずとも
    // 自動で動作するのでこのままでよい。
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-s3-app boot ===");
    log::info!("phase 0.7c: runtime boot-mode (NVS + KEY1 override + KEY2 long-press)");
    log::info!("build-stamp 2026-05-13-phase07c-bootmode");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    // NVS は (a) WiFi PHY calibration (b) boot_mode flag の 2 用途で
    // 共有。partition take は process 1 回しか通らないので、ここで
    // 取って両方に clone で渡す。
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    let mode = boot_mode::determine(&nvs, board::BTN_A_PIN);
    log::info!("boot_mode: {} (stored + KEY1 override)", mode.label());

    // ── Phase 0.6+: WiFi STA + UDP log sink. 起動条件:
    //   - mode == Wifi かつ WIFI_SSID が空でない (cfg.toml がある)
    //   失敗時は warn だけ出して続行 — boot 完走 > log 経路。
    //   `_wifi` は drop されると association が切れるので static slot に保持。
    static mut WIFI_SLOT: Option<wifi::WifiHandle> = None;
    let wifi_should_start = mode == boot_mode::BootMode::Wifi && !WIFI_SSID.is_empty();
    if mode == boot_mode::BootMode::Wifi && WIFI_SSID.is_empty() {
        log::warn!(
            "boot_mode=WIFI but WIFI_SSID empty (no cfg.toml) — running WiFi-mode shell without STA; flip via KEY2 to return to DECODE"
        );
    }
    if wifi_should_start {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        match wifi::connect_sta(peripherals.modem, sysloop, Some(nvs_part.clone()), WIFI_SSID, WIFI_PSK) {
            Ok(handle) => {
                // `pc_ip = "auto"` (or empty) → use the directed
                // subnet broadcast learned from DHCP. Otherwise treat
                // UDP_LOG_TARGET as a literal IPv4 (unicast or
                // 255.255.255.255). Subnet-broadcast is the safer
                // default since routers/APs often drop limited
                // broadcast but pass directed-broadcast through.
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
                #[allow(static_mut_refs)]
                unsafe {
                    WIFI_SLOT = Some(handle);
                }
            }
            Err(e) => log::warn!("WiFi STA failed: {e:#} — UDP log disabled"),
        }
    }

    // decode_pipeline を spawn するか否か: NVS-stored boot mode で決定。
    //   - BootMode::Decode: BASIS heap alloc (120 KB) + decode_pipeline +
    //     stage1_inc + dsp_worker + wav_sim を起動
    //   - BootMode::Wifi: BASIS alloc skip、WiFi STA + UDP log のみ。
    //     Internal DRAM ~120 KB が WiFi 用に空く
    if mode == boot_mode::BootMode::Decode {
        // Phase 0.7c: BASIS の `heap_caps_aligned_alloc` は thread の中で
        // 行う。0.7c で NVS が常時 init されるようになり pre-alloc free が
        // 234 KB に下がった結果、main で先に BASIS を取ると largest free
        // が 31 KB まで縮み 32 KB thread stack の確保が ENOMEM になる。
        // 順序を逆にして spawn 先で alloc すれば spawn 時点で largest が
        // 139 KB 確保できるので安全。BASIS の DRAM 配置要件は満たす。
        log_free_internal("pre-thread-spawn");
        let pipeline_spawn = std::thread::Builder::new()
            .stack_size(32 * 1024)
            .spawn(|| decode_pipeline::run());
        if let Err(e) = pipeline_spawn {
            log::error!("decode_pipeline spawn failed ({e})");
        }
    } else {
        log::info!(
            "decode_pipeline skipped (BootMode::Wifi — BASIS alloc skipped, 120 KB internal DRAM free for WiFi)"
        );
    }

    // メインタスクは LCD render loop (返らない)。modem は WiFi が
    // consume するので Peripherals 全体は move 不可 — 必要なフィールド
    // (i2c1 / i2s0 / spi3 / pins) を個別に渡す。NVS handle は KEY2
    // long-press 時の flip_and_restart に使う。
    display::run_log_panel(
        peripherals.i2c1,
        peripherals.i2s0,
        peripherals.spi3,
        peripherals.pins,
        &FANOUT,
        nvs,
        mode,
    )
}
