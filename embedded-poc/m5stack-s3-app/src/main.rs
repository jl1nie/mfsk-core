//! M5StickS3 IC-705 FT8 controller — entry point.

#![allow(dead_code)]

mod adif;
mod audio;
mod board;
mod buttons;
mod civ;
mod decode_pipeline;
mod display;
mod flash_log;
mod log_sink;
mod pmic;
mod qso;
mod snr_norm;
mod time_sync;
mod tx_picker;
mod uac;
mod udp_log;
mod ui;
mod wifi;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use log::LevelFilter;

use crate::log_sink::{FanoutLogger, LogFanout};

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
    log::info!("phase 4 + 0.6: QSO FSM + WiFi UDP log");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    // ── Phase 0.6: WiFi STA + UDP log sink. WIFI_SSID 空なら skip
    //   (cfg.toml 不在 = log は USB-CDC + LCD のみ)。失敗時は warn
    //   だけ出して続行 — boot 完走 > log 経路。`_wifi` は drop されると
    //   association が切れるので static slot に保持。
    static mut WIFI_SLOT: Option<wifi::WifiHandle> = None;
    if !WIFI_SSID.is_empty() {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        match wifi::connect_sta(peripherals.modem, sysloop, WIFI_SSID, WIFI_PSK) {
            Ok(handle) => {
                let target = format!("{UDP_LOG_TARGET}:{UDP_LOG_PORT}")
                    .parse::<std::net::SocketAddr>();
                match target {
                    Ok(addr) => match udp_log::UdpLogSink::new(addr) {
                        Ok(sink) => {
                            if let Ok(mut slot) = FANOUT.udp.try_lock() {
                                *slot = Some(sink);
                            }
                            FANOUT.drain_staging_to_udp();
                            log::info!("UDP log sink up → {addr}");
                        }
                        Err(e) => log::warn!("UDP socket bind failed: {e}"),
                    },
                    Err(e) => log::warn!("UDP target parse failed: {e}"),
                }
                #[allow(static_mut_refs)]
                unsafe {
                    WIFI_SLOT = Some(handle);
                }
            }
            Err(e) => log::warn!("WiFi STA failed: {e:#} — UDP log disabled"),
        }
    } else {
        log::info!("WIFI_SSID empty (no cfg.toml) — UDP log disabled");
    }

    // 別スレッドで decode pipeline を走らせる。デコード結果は log::info!
    // → FanoutLogger 経由で LCD scroll panel + UDP に流れる。
    std::thread::Builder::new()
        .stack_size(32 * 1024)
        .spawn(|| decode_pipeline::run())
        .expect("spawn decode pipeline");

    // メインタスクは LCD render loop (返らない)。modem は WiFi が
    // consume するので Peripherals 全体は move 不可 — 必要なフィールド
    // (i2c1 / i2s0 / spi3 / pins) を個別に渡す。
    display::run_log_panel(
        peripherals.i2c1,
        peripherals.i2s0,
        peripherals.spi3,
        peripherals.pins,
        &FANOUT,
    )
}
