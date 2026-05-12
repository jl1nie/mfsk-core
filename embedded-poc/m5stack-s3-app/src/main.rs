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
use esp_idf_svc::sys::{
    heap_caps_aligned_alloc, heap_caps_get_free_size, heap_caps_get_largest_free_block,
    MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use log::LevelFilter;
use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;

use crate::log_sink::{FanoutLogger, LogFanout};

/// Probe internal DRAM. Phase 0.7: called pre/post BASIS alloc to
/// confirm the heap accounting matches the ~123 KB delta the static
/// `.bss` version reserved at boot.
fn log_free_internal(label: &str) {
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
fn alloc_basis_dram(label: &str) -> &'static mut [i16] {
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
    log::info!("phase 4 + 0.6: QSO FSM + WiFi UDP log");
    log::info!("build-stamp 2026-05-13-phase07a-basis-heap");

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
        // Phase 0.7a: BASIS は heap_caps_aligned_alloc で内部 DRAM に確保
        // (旧 .bss 配置と同等)。WiFi mode で skip するためだけにこの分岐に
        // 移している — 静的版から見て [mem] 行の差分が ~123 KB なら成功。
        log_free_internal("pre-basis-alloc");
        let basis_re_main = alloc_basis_dram("RE_main");
        let basis_im_main = alloc_basis_dram("IM_main");
        let basis_re_c1 = alloc_basis_dram("RE_c1");
        let basis_im_c1 = alloc_basis_dram("IM_c1");
        log_free_internal("post-basis-alloc");
        // Worker pair: hand a raw pointer to `dual_core::init` (called
        // inside `decode_pipeline::run`). The slice references end at
        // this scope; the worker thread reconstructs slices from the
        // raw pointers. `*mut` isn't `Send` so cross the thread
        // boundary as `usize` (same pattern as the `wf_q` drainer).
        let re_c1_addr = basis_re_c1.as_mut_ptr() as usize;
        let im_c1_addr = basis_im_c1.as_mut_ptr() as usize;
        let pipeline_spawn = std::thread::Builder::new()
            .stack_size(32 * 1024)
            .spawn(move || {
                decode_pipeline::run(
                    basis_re_main,
                    basis_im_main,
                    re_c1_addr as *mut i16,
                    im_c1_addr as *mut i16,
                )
            });
        if let Err(e) = pipeline_spawn {
            log::error!("decode_pipeline spawn failed ({e})");
        }
    } else {
        log::info!(
            "decode_pipeline skipped (Phase 0.7a: WiFi mode — BASIS alloc skipped, 120 KB DRAM free for WiFi)"
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
