//! M5StickS3 IC-705 FT8 controller — entry point.

#![allow(dead_code)]

mod audio;
mod board;
mod buttons;
mod civ;
mod decode_pipeline;
mod display;
mod pmic;
mod tx;
mod tx_scheduler;
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
const MY_CALL: &str = env!("MY_CALL");
const MY_GRID: &str = env!("MY_GRID");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    // EspLogger を init すると log::set_logger を奪われ、自前の
    // FanoutLogger が install 失敗 → LCD に何も流れなくなる。
    // C-side ESP_LOG (タイムスタンプ付き UART 出力) は init せずとも
    // 自動で動作するのでこのままでよい。
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-s3-app boot ===");
    log::info!("phase 0.7c: runtime boot-mode (NVS + KEY1 override + KEY2 long-press)");
    log::info!("build-stamp 2026-05-17-phase17-qso");

    // Global QSO FSM instance. Empty MY_CALL keeps the FSM Idle so
    // the TX scheduler / auto-CQ stays a no-op until cfg.toml provides
    // a real callsign. Wrapped in Arc<Mutex> for the scheduler /
    // decode_pipeline shared writes.
    let qso = std::sync::Arc::new(std::sync::Mutex::new(
        mfsk_app_shared::qso::QsoManager::new(MY_CALL, MY_GRID),
    ));
    if MY_CALL.is_empty() {
        log::warn!(
            "[station] cfg.toml MY_CALL empty — QSO FSM disabled, TX scheduler stays idle"
        );
    } else {
        log::info!("[station] {} / {} (from cfg.toml)", MY_CALL, MY_GRID);
    }

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    // NVS は (a) WiFi PHY calibration (b) boot_mode flag の 2 用途で
    // 共有。partition take は process 1 回しか通らないので、ここで
    // 取って両方に clone で渡す。
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    let _stored = boot_mode::determine(&nvs, board::BTN_A_PIN);
    // TEMP 2026-05-17: force Acoustic for live-band sync + decode
    // verify after the text-color UI overhaul. Revert before merging
    // — restore `let mode = _stored;`.
    let mode = boot_mode::BootMode::Acoustic;
    log::info!("boot_mode: {} (TEMP force-override, live mic test)", mode.label());

    // ── Phase 0.6+: WiFi STA + UDP log sink. 起動条件:
    //   - mode == Wifi かつ WIFI_SSID が空でない (cfg.toml がある)
    //   - mode == Uac (UAC host install で USB-Serial-JTAG console が
    //     detach されるため、UDP 経路を必ず up させる。SSID 空でも
    //     試行はせずに warn のみ — UAC 機能は console なし fallback で
    //     継続)
    //   失敗時は warn だけ出して続行 — boot 完走 > log 経路。
    //   `_wifi` は drop されると association が切れるので static slot に保持。
    static mut WIFI_SLOT: Option<wifi::WifiHandle> = None;
    // Acoustic mode は USB-Serial-JTAG が生きているので console は使える。
    // 当初 UDP log のため WiFi 起動したが、WiFi 試行後の residual alloc が
    // capture thread spawn の OOM 原因 (2026-05-17 bring-up 実機検証) なので
    // Acoustic は WiFi 起動しない方針。UAC は USB-Serial-JTAG 奪われるので
    // UDP 必須、Wifi mode は WiFi が主目的。
    // TxTest needs no WiFi (BLE only for PTT). Uac needs WiFi for
    // UDP log path (USB-Serial-JTAG detaches on host install). Wifi
    // mode is the dev/log mode itself.
    let needs_wifi = matches!(mode, boot_mode::BootMode::Wifi | boot_mode::BootMode::Uac);
    let wifi_should_start = needs_wifi && !WIFI_SSID.is_empty();
    if needs_wifi && WIFI_SSID.is_empty() {
        log::warn!(
            "boot_mode={} but WIFI_SSID empty (no cfg.toml) — UDP log unavailable; serial console is also gone in UAC mode (USB-Serial-JTAG detached on usb_host_install)",
            mode.label()
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

    // decode_pipeline / UAC host を spawn するか否か: boot mode で決定。
    //   - BootMode::Decode: BASIS heap alloc (120 KB) + decode_pipeline +
    //     stage1_inc + dsp_worker + wav_sim
    //   - BootMode::Wifi:   pipeline skip、WiFi STA + UDP のみ
    //   - BootMode::Uac:    pipeline skip (#32 で UAC ringbuf に差し替えて
    //     再 spawn 予定)、USB host stack を install。#30 段階では
    //     uac_host_install + driver event callback まで。実際の device
    //     open / streaming は #31 以降。
    match mode {
        boot_mode::BootMode::Decode => {
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
        }
        boot_mode::BootMode::Wifi => {
            log::info!(
                "decode_pipeline skipped (BootMode::Wifi — BASIS alloc skipped, 120 KB internal DRAM free for WiFi)"
            );
        }
        boot_mode::BootMode::Acoustic => {
            // Phase 1.5-Stick: pure RX mode (mic capture → decode
            // pipeline). **No BLE** — adding civ::start() here pushes
            // BASIS 120 KB + BT controller 30 KB past the internal
            // DRAM ceiling (2026-05-17 verify: 4th BASIS alloc IM_c1
            // returns null, panic). BLE coexistence with decode lives
            // in BootMode::Qso, which trades audio-thread complexity
            // (bidir I2S) for the radio control surface. Acoustic
            // stays as the lightweight diagnostic / Phase 1.5 demo.
            log_free_internal("pre-thread-spawn");
            let pipeline_spawn = std::thread::Builder::new()
                .stack_size(32 * 1024)
                .spawn(|| {
                    decode_pipeline::run_with_source(|chunk_q| {
                        audio::set_chunk_q(chunk_q);
                    })
                });
            if let Err(e) = pipeline_spawn {
                log::error!("decode_pipeline (Acoustic) spawn failed ({e})");
            }
        }
        boot_mode::BootMode::CivTest => {
            // Phase 2-Stick bring-up: BLE のみ起動、decode pipeline は
            // skip。BASIS 120 KB 不要なので BT controller + NimBLE が
            // 余裕で fit する。IC-705 pairing + CI-V command 動作確認用。
            // display loop は audio/pipeline 無しで LCD だけ更新。
            if let Err(e) = civ::start() {
                log::error!("BLE CI-V start failed: {e:#}");
            }
            log_free_internal("post-civ-only-start");
        }
        boot_mode::BootMode::Qso => {
            // Phase 1.7.1-Stick: Acoustic capture (Phase 1.5) + BLE
            // CI-V (Phase 2) + TX synth (Phase 1.6) + QSO FSM + menu
            // UX, all sharing one `Arc<Mutex<QsoManager>>`. capture_tx_thread
            // (display.rs) owns the bidir I2S; pipeline consumes the
            // same FSM so decode → process_message updates state the
            // TX side reads back. Init order: civ first (BT controller
            // heap before BASIS alloc).
            if let Err(e) = civ::start() {
                log::error!("BLE CI-V start failed: {e:#}");
            }
            log_free_internal("pre-thread-spawn");
            let qso_for_pipeline = qso.clone();
            let pipeline_spawn = std::thread::Builder::new()
                .stack_size(32 * 1024)
                .spawn(move || {
                    decode_pipeline::run_with_source_qso(
                        |chunk_q| {
                            audio::set_chunk_q(chunk_q);
                        },
                        Some(qso_for_pipeline),
                    )
                });
            if let Err(e) = pipeline_spawn {
                log::error!("decode_pipeline (Qso) spawn failed ({e})");
            }
        }
        boot_mode::BootMode::TxTest => {
            // Phase 1.6-Stick bring-up: BLE (PTT) + TX synth (CQ test
            // loop) のみ。decode pipeline skip = BASIS 不要 = BT
            // controller + NimBLE + I2S TX + 12 kHz mono synth buf
            // (300 KB PSRAM) が全部 fit。display.rs が I2S TX 駆動
            // + tx::start_test_loop を spawn する。
            if let Err(e) = civ::start() {
                log::error!("BLE CI-V start failed: {e:#}");
            }
            log_free_internal("post-civ-tx-start");
        }
        boot_mode::BootMode::Uac => {
            // Spawn the decode pipeline FIRST. The thread allocates
            // BASIS scratch (~120 KB internal DRAM, same as Decode
            // mode), brings up stage1_inc + wf_drain, then calls the
            // source-spawn closure with the chunk_q — that's where
            // UAC gets wired in via `uac::set_chunk_q`. The reader
            // thread (spawned later by `uac::start_host` →
            // `app_task::handle_rx_connected` on first `RxConnected`)
            // holds samples until `CHUNK_Q_ADDR` is populated; lossy
            // by design (a few hundred ms of audio drops while
            // pipeline init races UAC enumeration).
            log_free_internal("pre-thread-spawn");
            let pipeline_spawn = std::thread::Builder::new()
                .stack_size(32 * 1024)
                .spawn(|| {
                    decode_pipeline::run_with_source(|chunk_q| {
                        uac::set_chunk_q(chunk_q);
                    })
                });
            if let Err(e) = pipeline_spawn {
                log::error!("decode_pipeline (Uac) spawn failed ({e})");
            }
            log_free_internal("pre-uac-host-install");
            if let Err(e) = uac::start_host() {
                log::error!("UAC host start failed: {e:#}");
            }
            log_free_internal("post-uac-host-install");
        }
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
        qso,
    )
}
