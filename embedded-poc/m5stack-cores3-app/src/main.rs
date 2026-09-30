//! M5Stack CoreS3 FT8 controller — entry point (Phase 1-Core).
//!
//! Mirrors `m5stack-core2-app/src/main.rs`. CoreS3 deltas vs Core2:
//!   - PMIC: AXP2101 + AW9523B (vs AXP192); LCD RST via AW9523B.
//!   - SPI2 (FSPI) on pins 36/37/3/35 for LCD (vs Core2 SPI3 18/23/5/15).
//!   - I2C0: SDA=12, SCL=11 (vs Core2 SDA=21, SCL=22).
//!   - No GPIO buttons (same as Core2); NVS-only boot mode.
//!   - USB-OTG host via AW9523B P0_1 (BUS_OUT_EN) — Phase 1-Core UAC.

#![allow(dead_code)]

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use mfsk_app_shared::boot_mode;

use mfsk_core_m5stack_cores3_app::{
    apps, boot, coredump, set_grid_source, set_wifi_pref, BOOT_MODE_DEFAULT, LOGGER,
};

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
        log::info!(
            "boot_mode: unset, seeding from cfg.toml → {}",
            target.label()
        );
        let _ = boot_mode::write(&nvs, target);
    }
    // `MFSK_CORES3_FORCE_MODE` (compile-time) overrides the NVS mode —
    // for a measurement run that needs a specific mode without erasing
    // NVS (which also holds WiFi creds and the HTTP-config settings).
    // Same "knob for the bench, not for the operator" role as
    // `MFSK_CORES3_FORCE_UAC`; unset in every shipped build.
    let mode = match option_env!("MFSK_CORES3_FORCE_MODE") {
        Some(s) => boot_mode::BootMode::from_cfg_str(s),
        None => boot_mode::determine_no_override(&nvs),
    };
    log::info!("boot_mode: {} (NVS-only on CoreS3)", mode.label());

    // **Whether WiFi comes up at all** — one of the CONFIG page's two
    // settings, read here because it belongs to every receiver. It used
    // to be decided from the boot mode alone, three steps further down
    // and only for the FT8 controller, so a board told to take its
    // phase off the air still ran an association campaign over the
    // slots a cold acquisition needs (#381). What the campaign costs,
    // and the three ways to end up without a radio, are
    // `net::bring_up`'s to say now.
    let wifi_pref = mfsk_app_shared::wifi_pref::read(&nvs);
    set_wifi_pref(wifi_pref);
    log::info!("wifi: {} (CONFIG page; TIME: AIR DT turns it off)", wifi_pref.label());
    // **How the slot grid's phase is kept** — the CONFIG page's other
    // setting, read once here and published for whoever boots.
    //
    // It does *not* choose a time source. The clock is the log's, and
    // it comes from NTP when there is a network and from the RTC when
    // there is not; FT8 logging wants the minute right, which the RTC
    // holds for weeks. What drifts into trouble is the *slot phase*:
    // seconds of it after days off the network, against a mode whose
    // coarse search is ±1 s. `AirDt` says to correct that from the air
    // rather than by waiting on a time server that is not there —
    // which is why it also skips NTP (`net::Config::ntp`), and why it
    // leaves the clock alone.
    let grid_src = mfsk_app_shared::grid_src::read(&nvs);
    // `MFSK_CORES3_FORCE_GRID=ntp|air`: this build's time source,
    // whatever NVS holds — for a measurement run that needs one without
    // changing the operator's setting (NVS is not written), the way
    // `MFSK_CORES3_FORCE_MODE` does for the mode.
    let grid_src = match option_env!("MFSK_CORES3_FORCE_GRID") {
        Some("ntp") => mfsk_app_shared::grid_src::GridSource::Ntp,
        Some("air") => mfsk_app_shared::grid_src::GridSource::AirDt,
        _ => grid_src,
    };
    set_grid_source(grid_src);
    log::info!(
        "grid source: {}{}",
        grid_src.label(),
        if option_env!("MFSK_CORES3_FORCE_GRID").is_some() { " (MFSK_CORES3_FORCE_GRID)" } else { "" }
    );

    // **No seeding from a stored fix.** `AIR DT` is a cold start by
    // definition now: the air places the phase every boot, because a
    // phase inherited from the RTC or from yesterday's acquisition is
    // one nothing has checked, and starting from an unchecked phase is
    // how this receiver spent minutes discovering it was 1.65 s out.
    // The `grid_fix` record is still written — `apps/ft4.rs` reads it
    // for the FT8 → FT4 reboot, where the alternative is no phase at
    // all.

    // **Everything above this line is every receiver's, and everything
    // below is one receiver's.** That split is the whole of `main` now.
    //
    // It carries four receivers rather than four binaries because
    // changing mode used to mean re-flashing, and on this board that
    // means unplugging the radio: `usb_host_install` takes the port the
    // flasher would use. Refs #163.
    //
    // Each of them used to open with its own copy of the same seven
    // steps, in its own order, and the FT8 controller's copy was this
    // function's own body — so the receiver with the most behaviour was
    // the one with no name. `boot::run` is that sequence, written once;
    // `apps::*` are the differences. `Wspr` and `Fst4` are behind
    // default-off features, so say which mode a board asked for and
    // carry on as an FT8 controller rather than appearing to ignore the
    // NVS setting.
    match mode {
        #[cfg(feature = "wspr")]
        boot_mode::BootMode::Wspr => {
            boot::run::<apps::wspr::WsprRx>(mode, peripherals, nvs_part, nvs)
        }
        #[cfg(feature = "fst4")]
        boot_mode::BootMode::Fst4 => {
            boot::run::<apps::fst4::Fst4Rx>(mode, peripherals, nvs_part, nvs)
        }
        #[cfg(feature = "ft4")]
        boot_mode::BootMode::Ft4 => {
            boot::run::<apps::ft4::Ft4Rx>(mode, peripherals, nvs_part, nvs)
        }
        #[cfg(feature = "jtty-rx")]
        boot_mode::BootMode::Jtty => {
            boot::run::<apps::jtty::JttyRx>(mode, peripherals, nvs_part, nvs)
        }
        #[cfg(not(feature = "jtty-rx"))]
        boot_mode::BootMode::Jtty => log::error!(
            "boot_mode=jtty but this image was built without --features jtty-rx — \
             continuing as an FT8 controller"
        ),
        #[cfg(not(feature = "ft4"))]
        boot_mode::BootMode::Ft4 => log::error!(
            "boot_mode=ft4 but this image was built without --features ft4 — \
             continuing as an FT8 controller"
        ),
        #[cfg(not(feature = "wspr"))]
        boot_mode::BootMode::Wspr => log::error!(
            "boot_mode=wspr but this image was built without --features wspr — \
             continuing as an FT8 controller"
        ),
        #[cfg(not(feature = "fst4"))]
        boot_mode::BootMode::Fst4 => log::error!(
            "boot_mode=fst4 but this image was built without --features fst4 — \
             continuing as an FT8 controller"
        ),
        _ => {}
    }

    boot::run::<apps::ft8::Ft8Controller>(mode, peripherals, nvs_part, nvs)
}
