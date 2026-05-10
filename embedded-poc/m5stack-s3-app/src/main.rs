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
use esp_idf_svc::nvs::EspDefaultNvsPartition;
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
    log::info!("build-stamp 2026-05-11-cdc-gate");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    // ── Phase 0.6: WiFi STA + UDP log sink. WIFI_SSID 空なら skip
    //   (cfg.toml 不在 = log は USB-CDC + LCD のみ)。失敗時は warn
    //   だけ出して続行 — boot 完走 > log 経路。`_wifi` は drop されると
    //   association が切れるので static slot に保持。
    static mut WIFI_SLOT: Option<wifi::WifiHandle> = None;
    if !WIFI_SSID.is_empty() {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        // NVS partition は WiFi の PHY 校正データを永続化するのに必要。
        // 取得失敗 (= partition 不在) でも `None` で WiFi 自体は動くが、
        // 毎 boot フルキャリブになるので少し遅くなる + DRAM 食う。
        let nvs = EspDefaultNvsPartition::take().ok();
        match wifi::connect_sta(peripherals.modem, sysloop, nvs, WIFI_SSID, WIFI_PSK) {
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
    } else {
        log::info!("WIFI_SSID empty (no cfg.toml) — UDP log disabled");
    }

    // decode_pipeline を spawn するか否か:
    //   - WiFi 有効時 (WIFI_SSID 設定): WiFi 静的バッファ + decode_pipeline
    //     の thread stack 群 (main 24KB + dsp_worker + stage1_inc 16KB +
    //     wav_sim) + BASIS_RE/IM 静的 DRAM (60KB×2) を全部内部 DRAM 161KB
    //     に詰めようとして OOM (Phase 0.6 検証で xTaskCreatePinnedToCore=-1
    //     を実測)。BASIS は移動不可 (cache/DMA 制約、ユーザ指摘) なので
    //     Phase 0.6 検証モードでは decode_pipeline 自体を skip する。
    //   - WiFi 無効時 (cfg.toml 不在): Phase 4 demo として decode_pipeline
    //     を起動する (BT も sdkconfig で無効化済み)。
    if WIFI_SSID.is_empty() {
        let pipeline_spawn = std::thread::Builder::new()
            .stack_size(32 * 1024)
            .spawn(|| decode_pipeline::run());
        if let Err(e) = pipeline_spawn {
            log::error!("decode_pipeline spawn failed ({e})");
        }
    } else {
        log::info!(
            "decode_pipeline skipped (Phase 0.6 mode: WiFi takes DRAM that decoder needs)"
        );
    }

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
