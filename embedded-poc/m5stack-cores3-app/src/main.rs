//! M5Stack CoreS3 FT8 controller — entry point (Phase 1-Core).
//!
//! Mirrors `m5stack-core2-app/src/main.rs`. CoreS3 deltas vs Core2:
//!   - PMIC: AXP2101 + AW9523B (vs AXP192); LCD RST via AW9523B.
//!   - SPI2 (FSPI) on pins 36/37/3/35 for LCD (vs Core2 SPI3 18/23/5/15).
//!   - I2C0: SDA=12, SCL=11 (vs Core2 SDA=21, SCL=22).
//!   - No GPIO buttons (same as Core2); NVS-only boot mode.
//!   - USB-OTG host via AW9523B P0_1 (BUS_OUT_EN) — Phase 1-Core UAC.

#![allow(dead_code)]

mod apps;
mod board;
mod coredump;
mod decode_pipeline;
mod display;
mod esp_log_bridge;
mod log_slot;
mod pmic;
mod touch;
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

/// この起動で WiFi を立ち上げるか。`display` が「ログ送信先を待つか」の
/// 判断に使う — 来ない sink を45秒待つのは、起動が45秒遅い受信機に
/// なるだけ。Refs #163.
static WIFI_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub(crate) fn wifi_enabled_for_this_boot() -> bool {
    WIFI_ENABLED.load(std::sync::atomic::Ordering::Acquire)
}
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");
const BOOT_MODE_DEFAULT: &str = env!("BOOT_MODE_DEFAULT");

/// SNTP server for the FT8 controller. WSPR and FST4 take theirs from
/// NVS settings, which this app has no page for; `pool.ntp.org` is what
/// their own default is.
const NTP_SERVER: &str = "pool.ntp.org";
/// Long enough for a first sync over WiFi, short enough that a boot
/// with no route still reaches the decode loop.
const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    LOGGER.install();

    // Catch Rust panics on the way out.
    //
    // In host mode there is no serial console — the USB driver has the
    // PHY — and the ESP-IDF panic handler writes straight to that
    // console with `esp_rom_printf`, bypassing the log path entirely.
    // So a panic looks like a spontaneous reboot and nothing else. A
    // Rust panic at least runs this first; the sleep is to let the UDP
    // sink actually put the datagram on the wire before the abort.
    // A hardware exception still slips through — that one shows up as
    // silence, which is itself a clue. Refs #163.
    std::panic::set_hook(Box::new(|info| {
        log::error!("PANIC: {info}");
        std::thread::sleep(std::time::Duration::from_millis(400));
    }));

    log::info!("=== mfsk-core-m5stack-cores3-app boot ===");
    log::info!(
        "phase 1-core: UAC host (AW9523B BUS_OUT_EN + usb_host_uac), wav_sim decode, ILI9342C LCD"
    );
    log::info!("build-stamp 2026-05-23-cores3-phase1");

    // Before anything else can crash: say what the last crash was.
    crate::coredump::report_previous_crash();

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");
    // `cfg.toml`'s `boot_mode` seeds a board that has never been told,
    // and nothing more.
    //
    // It used to be reapplied on every boot, which made the stored
    // value unwritable in practice: picking a mode from the touch
    // panel wrote NVS, restarted, and the restart put `cfg.toml`'s
    // value straight back. A binary that carries every receiver is
    // pointless if the choice cannot outlive a reboot — re-flashing to
    // change mode is exactly what it exists to avoid.
    //
    // Change it deliberately by erasing NVS, or from the panel.
    if !BOOT_MODE_DEFAULT.is_empty() && !boot_mode::is_set(&nvs) {
        let target = boot_mode::BootMode::from_cfg_str(BOOT_MODE_DEFAULT);
        log::info!("boot_mode: unset, seeding from cfg.toml → {}", target.label());
        let _ = boot_mode::write(&nvs, target);
    }
    let mode = boot_mode::determine_no_override(&nvs);
    log::info!("boot_mode: {} (NVS-only on CoreS3)", mode.label());

    // WSPR and FST4 are whole receivers, not modes of the FT8
    // controller — their own tasks, screens, slot grids and stacks.
    // They get the singletons and never come back.
    //
    // This is why they are here at all rather than in their own
    // binaries: changing mode used to mean re-flashing, and on this
    // board that means unplugging the radio, because `usb_host_install`
    // takes the port the flasher would use. Refs #163.
    match mode {
        boot_mode::BootMode::Wspr => apps::wspr::run(peripherals, nvs_part),
        boot_mode::BootMode::Fst4 => apps::fst4::run(peripherals, nvs_part),
        _ => {}
    }

    // Take the decode scratch now, while the heap is still whole.
    //
    // Ordering, not size, is what makes this work: after WiFi and the
    // USB host have taken their share the largest free internal block
    // is 31,744 B, and before they start it is 155,648 B (both
    // measured on this board, #163). Reserving here also means a
    // binary that carries several modes only ever allocates the one it
    // booted into — see `embedded_shared::worker_arena`.
    if matches!(mode, boot_mode::BootMode::Decode | boot_mode::BootMode::Uac)
        && !embedded_shared::internal_pool::reserve_arena()
    {
        log::error!("decode scratch reservation failed — the pipeline will abort on first use");
    }

    // WiFi is not a debug channel. It carries NTP, the HTTP config
    // UI and QSO log upload, and an FST4/WSPR beacon needs all three
    // while the radio is attached — so "host mode" and "networked"
    // are not alternatives.
    //
    // They were treated as alternatives for one night. Bringing WiFi
    // up costs ~105 KB of internal DRAM, the USB host stack another
    // 14 KB, and with a display loop quietly overflowing its stack
    // into the heap on top of that, the board aborted in whichever
    // subsystem allocated next. Turning WiFi off made the crashes
    // stop, which looked like a diagnosis and was not: the overflow
    // was the bug (see `display::run_log_panel`), and once it was
    // fixed the budget was never the problem. Measured 2026-08-23
    // with a radio streaming and WiFi associated: 73 KB of internal
    // DRAM still free, no allocation failures, audio at full rate.
    //
    // WiFi buffers were also sized down for this workload — see
    // `sdkconfig.defaults`. Refs #163.
    let needs_wifi = matches!(
        mode,
        boot_mode::BootMode::Wifi | boot_mode::BootMode::Uac
    );
    WIFI_ENABLED.store(needs_wifi, std::sync::atomic::Ordering::Release);
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
        // **Backgrounded.** This used to run inline, and association
        // took ~30 s — during which the LCD showed nothing and the USB
        // host was not yet installed, so plugging a radio in did
        // nothing and the board was indistinguishable from a crashed
        // one. A receiver has to be up when it is powered on; WiFi
        // buys it a log sink and a clock, and both can arrive late.
        //
        // Same conclusion `wspr_app`/`fst4_app` reached and for the
        // same reason. The handle has to outlive the association, so
        // the thread keeps it and never returns.
        let modem = peripherals.modem;
        let nvs_for_wifi = nvs_part.clone();
        let spawned = crate::board::spawn_named(c"wifi", 24 * 1024, move || {
                let sysloop = match EspSystemEventLoop::take() {
                    Ok(s) => s,
                    Err(e) => {
                        log::error!("sysloop take failed: {e:#} — no WiFi this boot");
                        return;
                    }
                };
                match wifi::connect_sta(modem, sysloop, Some(nvs_for_wifi), WIFI_SSID, WIFI_PSK) {
                    Ok(handle) => {
                        let target_ip: std::net::IpAddr =
                            if UDP_LOG_TARGET.is_empty() || UDP_LOG_TARGET == "auto" {
                                std::net::IpAddr::V4(handle.subnet_broadcast)
                            } else {
                                match UDP_LOG_TARGET.parse() {
                                    Ok(ip) => ip,
                                    Err(e) => {
                                        log::warn!(
                                            "UDP_LOG_TARGET '{UDP_LOG_TARGET}' parse failed ({e}); \
                                             using subnet bcast"
                                        );
                                        std::net::IpAddr::V4(handle.subnet_broadcast)
                                    }
                                }
                            };
                        let port: u16 = UDP_LOG_PORT.parse().unwrap_or(9999);
                        let addr = std::net::SocketAddr::new(target_ip, port);
                        // Install the sink, and keep trying.
                        //
                        // `FANOUT.udp` is an `embassy_sync` mutex with
                        // only `try_lock`, and every log line anywhere
                        // in the process takes it. Inline in `main`
                        // that was safe — nothing else was running
                        // yet. From a background thread it is not: a
                        // single missed `try_lock` used to drop the
                        // sink on the floor and leave the board
                        // reachable by ping and silent on the log,
                        // which is exactly what happened the first
                        // time this moved off the boot path.
                        //
                        // The retry also covers a bind that fails
                        // because the interface is not quite ready.
                        let mut installed = false;
                        for attempt in 0..30u32 {
                            if !installed {
                                match udp_log::UdpLogSink::new(addr) {
                                    Ok(sink) => match FANOUT.udp.try_lock() {
                                        Ok(mut slot) => {
                                            *slot = Some(sink);
                                            installed = true;
                                        }
                                        Err(_) => {
                                            // Sink dropped here; rebuilt next round.
                                        }
                                    },
                                    Err(e) => {
                                        if attempt == 0 {
                                            log::warn!("UDP socket bind failed: {e} — retrying");
                                        }
                                    }
                                }
                            }
                            if installed {
                                FANOUT.drain_staging_to_udp();
                                log::info!("UDP log sink up → {addr} (attempt {})", attempt + 1);
                                break;
                            }
                            std::thread::sleep(std::time::Duration::from_millis(200));
                        }
                        if !installed {
                            log::error!("UDP log sink never installed — board will be silent");
                        }

                        // NTP, which the FT8 controller has never
                        // started and cannot decode reliably without.
                        //
                        // `Ft8ChunkSink` anchors its 15 s slot grid
                        // with `time_sync::samples_to_next_slot_12k`,
                        // and that returns `None` until the system
                        // clock is plausible — which only NTP makes it.
                        // Unanchored, the grid free-runs from whenever
                        // the first USB sample arrived, at a phase
                        // uniform over 15 s, against a mode that
                        // tolerates ±2.5 s: about a one-in-three chance
                        // of decoding anything at all, per boot.
                        //
                        // Measured 2026-08-23 against a real antenna:
                        // audio at -26 dBFS, thirty candidates a slot,
                        // `dec=0` every slot. `slot grid anchored to
                        // UTC` appears exactly once in an evening of
                        // logs — on the one boot that followed a soft
                        // restart out of FST4, which runs NTP, and
                        // whose clock survives `esp_restart` but not
                        // the power-on reset the button performs.
                        //
                        // WSPR and FST4 have done this since they were
                        // written. `time_sync`'s own doc comment says
                        // "NTP is already in every app that has WiFi";
                        // this was the app where that was not true.
                        let _sntp = match mfsk_app_shared::ntp::start(NTP_SERVER) {
                            Ok(sntp) => {
                                if mfsk_app_shared::ntp::wait_synced(&sntp, NTP_SYNC_TIMEOUT_MS) {
                                    log::info!("NTP synced — FT8 slot grid can anchor to UTC");
                                } else {
                                    log::warn!(
                                        "NTP never synced in {NTP_SYNC_TIMEOUT_MS} ms — the slot \
                                         grid stays free-running and decodes are unlikely"
                                    );
                                }
                                Some(sntp)
                            }
                            Err(e) => {
                                log::warn!("NTP start failed: {e:#} — slot grid free-running");
                                None
                            }
                        };

                        // `handle`'s `Drop` tears the association down
                        // and `_sntp`'s stops the periodic re-sync, so
                        // this thread has to hold both forever.
                        loop {
                            std::thread::sleep(std::time::Duration::from_secs(60));
                        }
                    }
                    Err(e) => log::warn!("WiFi STA failed: {e:#} — UDP log disabled"),
                }
        });
        if let Err(e) = spawned {
            log::error!("wifi thread spawn failed ({e}) — continuing without WiFi");
        }
    }

    match mode {
        boot_mode::BootMode::Decode => {
            log_free_internal("pre-thread-spawn");
            let pipeline_spawn = crate::board::spawn_named(c"decode", 32 * 1024, || decode_pipeline::run());
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
            let pipeline_spawn = crate::board::spawn_named(c"decode", 32 * 1024, || {
                decode_pipeline::run_with_source(|q| uac::set_chunk_q(q))
            });
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
