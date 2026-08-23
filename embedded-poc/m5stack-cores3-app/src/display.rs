//! LCD bring-up + render loop for M5Stack CoreS3 (ILI9342C, 320×240).
//!
//! Mirrors `m5stack-core2-app/src/display.rs` structurally. Key CoreS3
//! deltas:
//!   - SPI2 (FSPI) on pins 36/37/3/35 (Core2 used SPI3 on 18/23/5/15).
//!   - PMIC is AXP2101 + AW9523B; RST + BL are driven by `pmic::init`
//!     before SPI init (Core2 used AXP192 GPIO4 for RST).
//!   - Color inversion: ILI9342C on CoreS3 requires Inverted (same as
//!     Core2 whose M5GFX cfg also sets invert=true for ILI9342C).
//!
//! Layout reuse: `mfsk_app_shared::ui::{decoded_list, status_bar,
//! waterfall}` hardcode 135 px width (M5StickS3 panel). Phase 0-Core
//! renders them into the top-left 135×240 corner; widening to runtime
//! canvas dims is a Phase 2.5-Core item.

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

use display_interface_spi::SPIInterface;
use esp_idf_hal::{
    delay::Ets,
    gpio::{AnyIOPin, PinDriver, Pins, Pull},
    i2c::I2C0,
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2},
    units::FromValueType,
};
use mipidsi::{
    models::ILI9342CRgb565,
    options::{ColorInversion, Orientation},
    Builder,
};

use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::log_sink::LogFanout;
use mfsk_app_shared::ui::{decoded_list, link_bar, mode_picker, state::UI, status_bar, waterfall};

/// The TX line sits on the last row of the canvas, not at y=226 —
/// that was where the shared widgets ended on a 240 px-tall panel.
const TX_REGION_H: u32 = 14;
/// The link bar owns the last row, so the TX line sits above it. The
/// same ordering holds in WSPR and FST4 — whichever mode booted, USB
/// and WiFi are in the same place.
const TX_REGION_Y: i32 =
    crate::board::CANVAS_H as i32 - link_bar::HEIGHT as i32 - TX_REGION_H as i32;
const LINK_BAR_Y: i32 = crate::board::CANVAS_H as i32 - link_bar::HEIGHT as i32;

/// The decoded list runs from the bottom of the waterfall to the TX
/// line. On this canvas that is 192 px — twelve rows rather than the
/// seven a 240 px-tall panel fits.
///
/// The stations are what the screen is for. Everything else here has
/// to justify taking a row from them, and the USB/tick panel could
/// not: it is bring-up instrumentation, and it now only draws when
/// [`USB_PANEL`] is built in.
const DECODED_ROWS: usize = ((TX_REGION_Y - decoded_list::ORIGIN_Y) as u32 / decoded_list::ROW_PX) as usize;
/// Panel width in the portrait orientation the three receivers share.
///
/// The FT8 controller used to run this panel unrotated (320x240) and
/// draw the shared widgets into the top-left 135x240 corner, because
/// those widgets were sized for the M5StickS3. WSPR and FST4 already
/// rotated to 240x320 and used all of it; this now matches them.
const SHARED_UI_WIDTH: u32 = crate::board::CANVAS_W as u32;

/// Where the USB status panel lives: the strip below the shared
/// widgets.
///
/// It used to be a right-hand column at x=140, which was correct while
/// this panel ran unrotated at 320x240 and the shared widgets occupied
/// only the left 135 px. Rotating to the 240x320 the other two
/// receivers use left that column hanging off the side — x=140..240 on
/// a 240 px canvas, straight over the full-width status bar and
/// decoded list. Reported from the bench as the tick overlapping the
/// top-right corner.
///
/// The space that is actually free is vertical: `TX_REGION_Y` + its
/// height ends the shared widgets at y=240, and the canvas is 320
/// tall. Full width buys 40 characters a line instead of 30, which is
/// what lets the same fields fit in fewer rows.
const USB_REGION_X: i32 = 0;
/// Sits directly above the TX line, so turning it on shortens the
/// decoded list by exactly its own height and leaves no gap.
const USB_REGION_Y: i32 = TX_REGION_Y - USB_PANEL_LINES as i32 * USB_PANEL_PITCH;

/// Whether to draw the USB/tick diagnostic panel at all.
///
/// It was built for #163, when the board ran on battery in host mode
/// with no serial console and the LCD was the only window — every
/// enable bit, the driver-event count and the last error had to be on
/// screen or they could not be seen. That is a bring-up instrument,
/// not something an operator watching for stations wants over the top
/// of the list, and on the 240 px-wide rotated canvas it landed
/// squarely on the status bar and the decodes.
///
/// Build with `MFSK_CORES3_USB_PANEL=1` for a bench session; the
/// decoded list gives up its bottom rows for it while it is on.
const USB_PANEL: bool = option_env!("MFSK_CORES3_USB_PANEL").is_some();

/// 6 行 × 12 px = 72 px、y=242..314 に収まる (キャンバス高 320)。
const USB_PANEL_LINES: usize = 6;

const USB_PANEL_PITCH: i32 = 12;

/// Characters a panel line holds, at [`SHARED_UI_WIDTH`] / 6 px.
const USB_PANEL_COLS: usize = 40;

/// How long the boot waits before installing the USB host driver.
///
/// Purely a serviceability window: the driver detaches
/// USB-Serial-JTAG, so this is the only time a flasher can connect.
/// Six seconds is invisible to an operator and comfortable for
/// `espflash`, which connects in under one.
const USB_HOST_DELAY_MS: u32 = 6_000;

/// How long to hold the USB host install waiting for the UDP log sink.
/// Long enough for a WiFi association (~30 s measured), short enough
/// that a board with no network still becomes a receiver.
const LOG_SINK_WAIT_MS: u32 = 45_000;

/// `MFSK_CORES3_FORCE_UAC=1` — run as a USB host even while something
/// external is powering the port. For a board fed from M5Bus rather
/// than the USB-C connector, where the two supplies do not collide.
const FORCE_UAC: bool = match option_env!("MFSK_CORES3_FORCE_UAC") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

/// LCD bring-up + render loop. Returns `!`.
#[allow(clippy::too_many_arguments)]
pub fn run_log_panel(
    i2c0: I2C0<'static>,
    spi2: SPI2<'static>,
    pins: Pins,
    fanout: &'static LogFanout,
    nvs: EspNvs<NvsDefault>,
    mode: BootMode,
) -> ! {
    // Set false when the board finds itself on external power — see
    // the VBUS check below.
    let mut host_mode = mode == BootMode::Uac;

    // Boot-time reading only.
    //
    // Polling these from the display loop crashed the board about a
    // second after audio started, and the coredump named it exactly:
    // `pmic::vbus_mv` -> `I2cDriver::write_read` ->
    // `i2c_cmd_link_delete`, aborting in the `main` task. That second
    // is when three USB devices have just taken a 4 KB control buffer
    // each out of internal DRAM, and the I2C command link could no
    // longer be allocated. The audio path had nothing to do with it.
    //
    // No loss: the AXP2101's VBUS ADC cannot see the board's own boost
    // output anyway, so in host mode — the only mode where a live
    // reading would be interesting — the number is meaningless.
    // Refs #163.
    let mut pmic_i2c;

    // ── PMIC: AXP2101 + AW9523B → LCD power rails + RST + BL. ──────────
    match crate::pmic::init(i2c0, pins.gpio12, pins.gpio11) {
        Ok(mut i2c) => {
            // Phase 1-Core: enable the USB VBUS boost BEFORE
            // `usb_host_install()`. AW9523B P0_1 (BUS_OUT_EN) HIGH
            // drives the VBUS switch; omission leaves VBUS floating and
            // the host stack sees no device.
            //
            // **But only when nothing else is driving that port.** One
            // USB-C connector cannot both take power in and hand it
            // out, so "host or peripheral" is a question about the
            // cable, not a build-time choice. Sourcing VBUS into a PC
            // that is already sourcing it browns the board out on a
            // battery that is not full — which is how a bench session
            // was lost, along with the ability to re-flash, since the
            // host firmware never charges and `cfg.toml` writes
            // `boot_mode` to NVS on every boot (#163).
            if mode == BootMode::Uac {
                let external = match crate::pmic::vbus_present(&mut i2c) {
                    Ok((present, raw)) => {
                        log::info!(
                            "AXP2101 status1=0x{raw:02x} — VBUS {} (bit5)",
                            if present {
                                "PRESENT (external power)"
                            } else {
                                "absent (battery)"
                            },
                        );
                        present
                    }
                    Err(e) => {
                        log::warn!("AXP2101 VBUS read failed: {e:#} — assuming battery");
                        false
                    }
                };
                if external && !FORCE_UAC {
                    // Charging is the useful thing to do while plugged
                    // in, and it is the thing this firmware could never
                    // do before.
                    host_mode = false;
                    log::warn!(
                        "external USB power detected — staying a peripheral so the battery \
                         charges and the port stays flashable. Unplug from the PC and reset to \
                         run as a host, or build with MFSK_CORES3_FORCE_UAC=1."
                    );
                } else if let Err(e) = crate::pmic::enable_usb_host_vbus(&mut i2c) {
                    log::error!("BUS_OUT_EN (Phase 1-Core) failed: {e:#}");
                }
            }
            // Let the boost settle, then measure. The output-register
            // readback only says what was written; this is the voltage
            // itself, on the net the connector's VBUS pin sits on.
            std::thread::sleep(std::time::Duration::from_millis(200));
            match (
                crate::pmic::battery_mv(&mut i2c),
                crate::pmic::vbus_mv(&mut i2c),
            ) {
                (Some(bat), Some(vbus)) => {
                    log::info!("AXP2101 battery {bat} mV, VBUS {vbus} mV")
                }
                _ => log::warn!("AXP2101 voltage read failed"),
            }

            // Scan again, now that TP_RST is released.
            //
            // `pmic::init` scans at its start — before it pulses the
            // reset — so that listing cannot see a device held in
            // reset, and the touch controller's absence from it means
            // nothing. This second pass is the one that can answer
            // whether the panel is on this bus at all, and at which
            // address. Cheap, once, at boot.
            crate::pmic::scan(&mut i2c);
            crate::pmic::dump_rails(&mut i2c);
            // `pmic::init` now runs M5GFX's full sequence, rails
            // included, so by here the touch controller should be
            // powered and configured.
            esp_idf_hal::delay::FreeRtos::delay_ms(200);
            crate::pmic::scan(&mut i2c);
            crate::touch::init(&mut i2c);
            pmic_i2c = Some(i2c);
        }
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            loop {
                log::info!("alive (no PMIC/LCD)");
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        }
    }
    // ── SPI2 (FSPI) host for the ILI9342C. ───────────────────────────
    let driver = SpiDriver::new(
        spi2,
        pins.gpio36, // SCK
        pins.gpio37, // MOSI
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new(),
    )
    .expect("SPI2 driver");
    let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
    let spi_dev =
        SpiDeviceDriver::new(driver, Some(pins.gpio3), &spi_cfg).expect("SPI device (CS=3)");

    // Touch interrupt, as an input with a pull-up — `Touch_FT5x06::init`
    // does `pinMode(_cfg.pin_int, input_pullup)` and `getTouchRaw`
    // reads it before it will touch I2C at all. Without it the only
    // way to notice a tap is to poll the bus, and polling the bus
    // slowly is why the panel felt broken: at 2 Hz, any tap shorter
    // than half a second fell between reads. M5GFX puts this on
    // GPIO 21 (not on the expander, whatever `board.rs` used to say).
    let touch_int = PinDriver::input(pins.gpio21, Pull::Up).expect("TP_INT gpio21");

    let dc = PinDriver::output(pins.gpio35).expect("DC gpio35");
    let di = SPIInterface::new(spi_dev, dc);

    // No reset_pin(): RST was already cycled by AW9523B in pmic::init;
    // mipidsi will send SWRESET via the command interface as fallback.
    //
    // **2026-08-15 real-hardware fix** (found chasing `wspr-app`'s own
    // "LCD shows nothing" bug on the same board — see memory
    // `project_wspr_app_cores3_ui`): this panel is genuinely ILI9342C,
    // whose mipidsi model already has `FRAMEBUFFER_SIZE = (320, 240)`
    // — landscape-native. The previous code used the *ILI9341* model
    // (`FRAMEBUFFER_SIZE = (240, 320)`, portrait-native) plus a manual
    // `.orientation(Deg90)` to compensate; that Builder-time rotation
    // never correctly resynced mipidsi's internal width/height
    // bookkeeping on this hardware (confirmed via `lcd_minimal.rs`'s
    // orientation-cycling diagnostic in the wspr-app investigation —
    // not yet independently re-verified with a photo of *this*
    // binary's own screen, since decode_pipeline's `wav_sim` loop was
    // never re-run against real UAC audio here to justify a full
    // hardware session just for this). Using the model whose native
    // framebuffer already matches the physical panel needs no
    // rotation hack at all.
    let mut delay = Ets;
    let mut display = match Builder::new(ILI9342CRgb565, di)
        .display_size(crate::board::NATIVE_W, crate::board::NATIVE_H)
        // Same rotation `wspr_app` and `fst4_app` use, so all three
        // receivers present the same 240x320 canvas.
        .orientation(Orientation::new().rotate(crate::board::ROTATION))
        .invert_colors(ColorInversion::Inverted) // M5GFX CoreS3: cfg.invert = true
        .init(&mut delay)
    {
        Ok(d) => d,
        Err(e) => {
            log::error!("display init failed: {:?}", e);
            loop {
                log::info!("alive (no LCD)");
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        }
    };

    log::info!(
        "LCD init OK (ILI9342C/CoreS3, {}x{})",
        crate::board::CANVAS_W,
        crate::board::CANVAS_H
    );
    // Explicit `Rectangle` fill, not `DrawTarget::clear()` — the
    // latter gave unreliable/partial coverage on this mipidsi/SPI
    // setup every time it was tried during the wspr-app investigation
    // (see the fix note above); every other draw call in this file
    // already used `Rectangle` fills, so this is the one holdout.
    Rectangle::new(
        Point::new(0, 0),
        Size::new(
            crate::board::CANVAS_W as u32,
            crate::board::CANVAS_H as u32,
        ),
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
    .draw(&mut display)
    .ok();

    let tx_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::new(0, 0, 8))
        .build();
    let tx_bg = Rgb565::new(0, 0, 8);

    Rectangle::new(
        Point::new(0, TX_REGION_Y),
        Size::new(SHARED_UI_WIDTH, TX_REGION_H),
    )
    .into_styled(PrimitiveStyle::with_fill(tx_bg))
    .draw(&mut display)
    .ok();
    let mode_line: heapless::String<32> = {
        let mut s: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(&mut s, format_args!("CoreS3 Mode: {}", mode.label()));
        s
    };
    Text::with_baseline(
        mode_line.as_str(),
        Point::new(2, TX_REGION_Y + 2),
        tx_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .ok();

    // Phase 1-Core: USB host + UAC class driver. **After the LCD is
    // up, not before.**
    //
    // This used to run before SPI init, so nothing had been drawn when
    // `usb_host_install()` detached USB-Serial-JTAG — a board that
    // showed a blank screen and had no console was giving the operator
    // nothing at all to go on. The panel is painted first now, so
    // "waiting for device" is on screen from the moment the console
    // goes away.
    //
    // The delay before it is the flashing window. Installing the host
    // driver takes the serial port with it, and with WiFi no longer
    // blocking the boot that now happens ~2 s in — too fast to catch
    // with a flasher. `USB_HOST_DELAY_MS` is the difference between a
    // board you can re-flash and one that needs the download-mode
    // button dance.
    if host_mode && USB_PANEL {
        let mut banner: heapless::String<40> = heapless::String::new();
        {
            use core::fmt::Write as _;
            let _ = write!(&mut banner, "USB: host in {} ms", USB_HOST_DELAY_MS);
        }
        Text::with_baseline(
            banner.as_str(),
            Point::new(USB_REGION_X + 2, USB_REGION_Y + 1),
            tx_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .ok();
        log::info!(
            "uac: installing USB host in {USB_HOST_DELAY_MS} ms — serial console goes away then \
             (this is the window to re-flash)"
        );
        std::thread::sleep(std::time::Duration::from_millis(USB_HOST_DELAY_MS as u64));

        // Hold until the log sink exists, if it is coming.
        //
        // Everything interesting about enumeration is logged in the
        // few hundred milliseconds after `start_host()`, and on
        // battery there is no serial console to catch it — the VBUS
        // gate means a board in host mode is a board with no cable to
        // a PC. WiFi takes ~30 s, so without this wait those lines land
        // in the staging ring and are gone by the time anything can
        // read them. Bounded, because a receiver with no WiFi still has
        // to work.
        // Only worth waiting if a sink is actually coming. UAC mode
        // now leaves WiFi off by default, and waiting 45 s for a sink
        // that will never exist is just a receiver that takes 45 s
        // longer to start. Refs #163.
        let sink_expected = crate::wifi_enabled_for_this_boot();
        let mut waited_ms = 0u32;
        while sink_expected && waited_ms < LOG_SINK_WAIT_MS {
            if crate::FANOUT
                .udp
                .try_lock()
                .map(|g| g.is_some())
                .unwrap_or(false)
            {
                log::info!("uac: log sink up after {waited_ms} ms — installing USB host now");
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(250));
            waited_ms += 250;
        }
        if sink_expected && waited_ms >= LOG_SINK_WAIT_MS {
            log::warn!(
                "uac: no log sink after {waited_ms} ms — installing USB host anyway; \
                 enumeration will only be visible on screen"
            );
        }

        // Turn up ESP-IDF's own USB enumeration logging before the
        // stack starts.
        //
        // The UAC class driver only reports what it recognises, so a
        // device that never finishes enumeration — or a hub whose
        // downstream ports are never walked — is indistinguishable
        // from nothing being plugged in. These tags are the ones the
        // host library uses on that path, and they are quiet outside
        // attach/detach, so leaving them up costs nothing while the
        // board waits. Issue #163.
        // `ENUM` alone at DEBUG. It carries the per-stage verdicts —
        // GET_FULL_DEV_DESC, CHECK_SHORT_CONFIG_DESC, and which one
        // FAILED — which is the whole diagnostic.
        //
        // The rest stay at INFO deliberately. At DEBUG, `EXT_PORT` /
        // `USBH` / `EXT_HUB` emit a "Processing actions" line per state
        // transition, so one attach is several hundred lines inside a
        // few milliseconds. Every one of those becomes a UDP datagram
        // sent from the USB task, and the board reliably fell off the
        // network right after enumeration whenever they were on — the
        // log volume was costing us the log. Refs #163.
        unsafe {
            esp_idf_svc::sys::esp_log_level_set(
                c"ENUM".as_ptr(),
                esp_idf_svc::sys::esp_log_level_t_ESP_LOG_DEBUG,
            );
        }
        for tag in [
            c"USB HOST".as_ptr(),
            c"USBH".as_ptr(),
            c"HUB".as_ptr(),
            c"EXT_HUB".as_ptr(),
            c"EXT_PORT".as_ptr(),
        ] {
            unsafe {
                esp_idf_svc::sys::esp_log_level_set(
                    tag,
                    esp_idf_svc::sys::esp_log_level_t_ESP_LOG_INFO,
                );
            }
        }

        crate::log_free_internal("pre-uac-host-install");
        // Serial is about to go away with the PHY; move the C-side log
        // output somewhere that survives (see `esp_log_bridge`).
        crate::esp_log_bridge::install();

        if let Err(e) = crate::uac::start_host() {
            log::error!("UAC host start failed: {e:#}");
            let mut msg: heapless::String<96> = heapless::String::new();
            {
                use core::fmt::Write as _;
                let _ = write!(&mut msg, "start_host FAILED: {e:#}");
            }
            crate::uac::HOST_RESULT.store(msg.as_str());
        }
        crate::log_free_internal("post-uac-host-install");
    }

    let mut tick: u32 = 0;
    let mut boot_summary_sent = false;

    // The two render snapshots live on the heap, allocated once.
    //
    // They used to be stack locals in this loop, and they are the
    // reason the board was corrupting its own heap. `WfLine` is
    // `[u8; 135]` and `WF_DEPTH` is 100, so the waterfall snapshot is
    // 13,500 B; at `opt-level = 1` (forced on this target by the
    // Xtensa LLVM regression) the `.collect()` temporary does not fold
    // into the destination, so it cost roughly twice that. The
    // 2026-08-23 coredump measured this function's frame at 29,616 B
    // of a 32,768 B main-task stack — 448 B of margin — and a task
    // stack here is heap memory with the internal pool sitting
    // directly below it (stack base 0x3fcb4d20, pool start
    // 0x3fcb1a90). Overflowing did not fault: it rewrote TLSF block
    // headers, and the crash surfaced later and elsewhere, in
    // `tlsf_walk_pool` reading a smashed size field. Every diagnostic
    // probe added to this loop that night made the margin smaller.
    //
    // Boxed and reused, so there is no per-frame allocation either.
    // 13.5 KB is well past `SPIRAM_MALLOC_ALWAYSINTERNAL = 4096`, so
    // this lands in PSRAM and costs no internal DRAM — the resource
    // the USB host path is actually short of. Refs #163.
    let mut wf_snapshot: Box<
        heapless::Vec<mfsk_app_shared::ui::state::WfLine, { mfsk_app_shared::ui::state::WF_DEPTH }>,
    > = Box::new(heapless::Vec::new());
    let mut decoded_snapshot: Box<heapless::Vec<mfsk_app_shared::ui::state::DecodedRow, 16>> =
        Box::new(heapless::Vec::new());

    let mut last_usb_panel: heapless::Vec<heapless::String<USB_PANEL_COLS>, USB_PANEL_LINES> =
        heapless::Vec::new();
    let mut last_touch = crate::touch::Contact::default();
    // The way back out of this mode, shared with the other two
    // receivers so switching is not a one-way trip. Sits under the USB
    // panel (which ends at y=122); the shared widgets own the left
    // 135 px.
    // Centred, like the other two receivers. The origin used to be
    // (140, 134), from when this panel ran unrotated at 320x240 — at
    // 240 wide that put a 208 px widget 108 px off the right edge.
    let mut picker = mode_picker::ModePicker::new(Point::new(
        (crate::board::CANVAS_W as i32 - mode_picker::WIDTH as i32) / 2,
        (crate::board::CANVAS_H as i32 - mode_picker::height() as i32) / 2,
    ));
    let mut touch_read_failed = false;
    let mut last_wf_seq: u32 = u32::MAX;
    let mut last_decoded_fp: (usize, u32) = (usize::MAX, u32::MAX);
    let mut last_tx_seq: u32 = 0;
    loop {
        // Touch, first thing in the frame and only when it changes.
        //
        // Input before rendering, because the render path takes an
        // early `continue` while the picker is up. Reading touch
        // after it meant that opening the overlay switched off the
        // only input that could operate it: the menu appeared and
        // nothing on it could be selected.
        //
        // Deliberately slow and deliberately quiet. Every read is an
        // I2C transaction, every transaction allocates a command link,
        // and that allocation is what aborted this very loop the last
        // time it talked to I2C — `pmic::vbus_mv` polled per second,
        // dying in `i2c_cmd_link_delete` once internal DRAM ran out
        // (#163). There is more headroom now, but this is the first
        // flash that tests it, so the rate is low and the alive tick's
        // `internal=` reading is the thing to watch beside it.
        // Re-read the expander about once a second, from the task that
        // owns the bus. Without this the panel reports what was written
        // minutes ago; with it, an expander that has fallen off its
        // rail shows as `V!!!` on the frame after it happens.
        if tick % 12 == 0 {
            if let Some(i2c) = pmic_i2c.as_mut() {
                crate::pmic::refresh_power_state(i2c);
                // Once, the first frame after a sink exists to receive
                // it. In host mode this is the only record there will
                // ever be of how the boot went.
                if !boot_summary_sent
                    && crate::FANOUT.udp.try_lock().map(|g| g.is_some()).unwrap_or(false)
                {
                    boot_summary_sent = true;
                    let r = crate::uac::HOST_RESULT.read();
                    log::warn!(
                        "[boot-summary] mode={} host_mode={host_mode} start_host: {}",
                        mode.label(),
                        if r.is_empty() { "never called" } else { r.as_str() }
                    );
                    crate::pmic::log_boot_summary(i2c);
                }
            }
        }
        if touch_int.is_low() {
            if let Some(i2c) = pmic_i2c.as_mut() {
                match crate::touch::read(i2c) {
                    Some(c) => {
                        let last_touch_points = last_touch.points;
                        if c != last_touch {
                            if c.points > 0 {
                                log::info!("touch: {} pt at ({}, {})", c.points, c.x, c.y);
                            } else {
                                log::info!("touch: released");
                            }
                            last_touch = c;
                        }
                        let _ = last_touch_points;
                        {
                            if let Some(target) = picker.update(c.points > 0, c.x, c.y) {
                                log::warn!("boot_mode -> {} (touch), restarting", target.label());
                                if let Err(e) = boot_mode::write(&nvs, target) {
                                    log::error!("boot_mode write failed: {e} — not restarting");
                                } else {
                                    // Let the line reach the log sink; in
                                    // UAC mode that is the only channel.
                                    std::thread::sleep(std::time::Duration::from_millis(400));
                                    // SAFETY: no arguments, does not return.
                                    unsafe { esp_idf_svc::sys::esp_restart() };
                                }
                            }
                        }
                    }
                    None => {
                        if !touch_read_failed {
                            log::warn!("touch: read failed — not repeating this");
                            touch_read_failed = true;
                        }
                    }
                }
            }
        } else if last_touch.points > 0 {
            // The pin going high is the release; there is nothing to
            // read for it.
            log::info!("touch: released");
            last_touch = crate::touch::Contact::default();
        }

        // With no finger down the picker still needs a frame to let an
        // arming lapse and to notice a release.
        if last_touch.points == 0 {
            let _ = picker.update(false, 0, 0);
        }
        picker.render(&mut display, mode).ok();
        if picker.take_just_closed() {
            // The overlay covered the panel; force everything back.
            last_usb_panel.clear();
            last_wf_seq = u32::MAX;
            last_decoded_fp = (usize::MAX, u32::MAX);
            last_tx_seq = last_tx_seq.wrapping_add(1);
        }

        let heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
        if tick % 50 == 0 {
            let internal = unsafe {
                esp_idf_svc::sys::heap_caps_get_free_size(
                    esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT,
                )
            };
            let internal_largest = unsafe {
                esp_idf_svc::sys::heap_caps_get_largest_free_block(
                    esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT,
                )
            };
            // Stack headroom, in the same line as the heap numbers.
            //
            // The crash this is here to catch is not a heap bug: the
            // main task's 32 KB stack sits directly above the internal
            // heap pool (stack base 0x3fcb4d20, pool start 0x3fcb1a90),
            // and this very loop's frame measured 29,616 B in the
            // 2026-08-23 coredump — 448 B of margin. Overflowing it
            // writes into the pool's first blocks, and the *next*
            // `heap_caps_get_largest_free_block` faults inside
            // `tlsf_walk_pool` on a smashed size field. The heap walk
            // is the detector; this number is the cause.
            //
            // `uxTaskGetStackHighWaterMark` allocates nothing, blocks
            // on nothing and costs 4 B of the very stack it measures.
            // SAFETY: null = the calling task.
            let stack_hw =
                unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
            log::info!(
                "alive tick={tick} free_heap={heap} internal={internal} \
                 largest={internal_largest} stack_hw={stack_hw}"
            );
            // Every task's headroom, from here, every 30 s. The
            // high-water mark is monotonic, so this is not sampling —
            // one late reading is the whole answer. See
            // `board::log_task_stacks`.
            if tick % 300 == 0 {
                crate::board::log_task_stacks();
            }
        }

        let status_snapshot;
        let decoded_fp;
        let wf_seq;
        let tx_seq;
        let tx_line_snapshot: heapless::String<48>;
        {
            let Ok(mut ui) = UI.lock() else {
                log::warn!("UI mutex poisoned — skipping render frame");
                continue;
            };
            ui.status.free_heap_kb = (heap / 1024) as u32;
            status_snapshot = ui.status.clone();
            decoded_snapshot.clear();
            for row in ui.decoded_iter() {
                if decoded_snapshot.push(row.clone()).is_err() {
                    break;
                }
            }
            wf_seq = ui.wf_push_seq();
            // Copy the waterfall only when it moved.
            //
            // The `wf_seq` gate below was already skipping the *draw*;
            // the 13.5 KB copy ran every frame regardless, ten times a
            // second, for a surface that changes once per slot.
            if wf_seq != last_wf_seq {
                wf_snapshot.clear();
                for line in ui.waterfall_iter() {
                    if wf_snapshot.push(*line).is_err() {
                        break;
                    }
                }
            }
            tx_seq = ui.tx_seq();
            let mut buf: heapless::String<48> = heapless::String::new();
            for ch in ui.tx_line().chars() {
                if buf.push(ch).is_err() {
                    break;
                }
            }
            tx_line_snapshot = buf;
            let max_seq = decoded_snapshot
                .iter()
                .map(|r| r.slot_seq)
                .max()
                .unwrap_or(0);
            decoded_fp = (decoded_snapshot.len(), max_seq);
        }

        // Freeze what is underneath while the overlay is up. The
        // widget only redraws itself on change, so anything painted
        // over it stays painted over it — which is why the picker
        // vanished a moment after opening.
        if picker.is_open() {
            picker.render(&mut display, mode).ok();
            std::thread::sleep(std::time::Duration::from_millis(50));
            tick = tick.wrapping_add(1);
            continue;
        }

        status_bar::render(&mut display, &status_snapshot, SHARED_UI_WIDTH).ok();

        // Cheap enough to repaint every frame (one 240x14 fill plus
        // three short strings), and repainting unconditionally is what
        // keeps it correct after the mode picker closes over it.
        link_bar::render(
            &mut display,
            &crate::uac::link_info(),
            SHARED_UI_WIDTH,
            LINK_BAR_Y,
        )
        .ok();

        if wf_seq != last_wf_seq {
            let wf_refs: heapless::Vec<
                &mfsk_app_shared::ui::state::WfLine,
                { mfsk_app_shared::ui::state::WF_DEPTH },
            > = wf_snapshot.iter().collect();
            waterfall::render(&mut display, &wf_refs, SHARED_UI_WIDTH).ok();
            last_wf_seq = wf_seq;
        }

        if decoded_fp != last_decoded_fp {
            decoded_list::render_in(
                &mut display,
                &decoded_snapshot,
                None,
                None,
                SHARED_UI_WIDTH,
                decoded_list::ORIGIN_Y,
                if USB_PANEL {
                    ((USB_REGION_Y - decoded_list::ORIGIN_Y) as u32 / decoded_list::ROW_PX) as usize
                } else {
                    DECODED_ROWS
                },
            )
            .ok();
            last_decoded_fp = decoded_fp;
        }

        if tx_seq != last_tx_seq {
            Rectangle::new(
                Point::new(0, TX_REGION_Y),
                Size::new(SHARED_UI_WIDTH, TX_REGION_H),
            )
            .into_styled(PrimitiveStyle::with_fill(tx_bg))
            .draw(&mut display)
            .ok();
            let text = if tx_line_snapshot.is_empty() {
                "IDLE: ---"
            } else {
                tx_line_snapshot.as_str()
            };
            Text::with_baseline(
                text,
                Point::new(2, TX_REGION_Y + 2),
                tx_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .ok();
            last_tx_seq = tx_seq;
        }

        // Re-announce the previous boot's crash once the network is
        // there to hear it.
        //
        // `report_previous_crash` runs before anything else can crash,
        // which is right, but that is ~21 s before the UDP sink exists,
        // and the staging ring is deliberately small (it was competing
        // with the audio transfer buffer for internal DRAM). So the one
        // line worth reading was reliably the one that got dropped.
        // Refs #163.
        // Freeze briefly on the reason, once it is on screen.
        //
        // A board that reboots a second and a half in gives nobody time
        // to read why, and with WiFi off in host mode the panel is the
        // only channel there is. Refs #163.
        if tick == 1 && crate::coredump::was_abnormal_reset() {
            log::warn!("holding 10 s so the reset reason stays readable");
            std::thread::sleep(std::time::Duration::from_secs(10));
        }

        if tick == 300 {
            let crash = crate::coredump::last_crash();
            if !crash.is_empty() {
                log::error!("coredump (repeat, for the log sink): {crash}");
            }
        }

        // The USB panel. Everything the USB side knows, on screen,
        // refreshed every loop.
        //
        // This board spends its life on battery with a radio on the
        // only port it has, which means no serial console (the host
        // driver takes the PHY) and often no WiFi either. A log line
        // is not a status display: the log panel holds a handful of
        // lines and scrolls, so a value printed once at boot is gone
        // by the time anyone looks. Anything needed to answer "why is
        // it not enumerating" has to be *standing* somewhere, and the
        // tick has to be there too — a frozen panel and an idle one
        // are otherwise the same picture.
        //
        // Refs #163.
        if USB_PANEL {
            use core::fmt::Write as _;
            let (st, sa, rms) = crate::uac::status();
            let (dev, cli, evts, err) = crate::uac::usb_counters();
            let (vbus_tried, p0, p1, st1) = crate::pmic::power_state();

            let mut panel: heapless::Vec<heapless::String<USB_PANEL_COLS>, USB_PANEL_LINES> =
                heapless::Vec::new();
            let mut line = |args: core::fmt::Arguments| {
                let mut s: heapless::String<USB_PANEL_COLS> = heapless::String::new();
                let _ = s.write_fmt(args);
                let _ = panel.push(s);
            };

            // Compact on purpose: this panel shares the screen with
            // the scrolling log, and at one field per line it grew
            // until it covered it. Everything below fits in the
            // `USB_PANEL_COLS` characters the full-width strip allows.
            // Four fixed rows now, not seven: the strip is 6 rows deep
            // and the two standing failure lines below need the rest.
            // 40 characters is what makes pairing these fields fit.
            line(format_args!(
                "USB t={tick} {} {} dev={dev} cli={cli}",
                if host_mode { "host" } else { "periph" },
                match st {
                    crate::uac::UacState::Off if host_mode => "off",
                    crate::uac::UacState::Off => "chg",
                    crate::uac::UacState::Waiting => "wait",
                    crate::uac::UacState::Streaming => "STREAM",
                    crate::uac::UacState::Error => "ERROR",
                }
            ));
            if vbus_tried {
                line(format_args!(
                    "vbus b{} o{} p0={p0:02x} p1={p1:02x}",
                    if p1 & crate::board::AW9523_P1_BOOST_EN != 0 {
                        1
                    } else {
                        0
                    },
                    if p0 & crate::board::AW9523_P0_USB_OTG_EN != 0 {
                        1
                    } else {
                        0
                    },
                ));
            } else {
                line(format_args!("vbus n/a (periph)"));
            }
            line(format_args!("evts={evts} err={err:x} s1={st1:02x}"));
            match rms {
                Some(db) => line(format_args!("au={sa} rms={db:.0}dBFS")),
                None => line(format_args!("au={sa} rms=--")),
            }
            if last_touch.points > 0 {
                line(format_args!(
                    "bat={}mV T {},{} n={}",
                    crate::pmic::battery_mv_cached(),
                    last_touch.x,
                    last_touch.y,
                    last_touch.points
                ));
            } else {
                line(format_args!(
                    "bat={}mV T --",
                    crate::pmic::battery_mv_cached()
                ));
            }

            // Two standing failure lines, each only when it has
            // something: the previous boot's crash, and the last
            // enumeration failure.
            for text in [crate::coredump::last_crash(), {
                let e = crate::esp_log_bridge::last_enum_line();
                if e.is_empty() {
                    crate::esp_log_bridge::last_error_line()
                } else {
                    e
                }
            }] {
                let mut rest: &str = text.as_str();
                for _ in 0..2 {
                    if rest.is_empty() {
                        break;
                    }
                    let mut cut = rest.len().min(USB_PANEL_COLS - 1);
                    while cut > 0 && !rest.is_char_boundary(cut) {
                        cut -= 1;
                    }
                    let (head, tail) = rest.split_at(cut);
                    line(format_args!("{head}"));
                    rest = tail;
                }
            }

            for (i, text) in panel.iter().enumerate() {
                if last_usb_panel.get(i) == Some(text) {
                    continue;
                }
                let y = USB_REGION_Y + (i as i32) * USB_PANEL_PITCH;
                Rectangle::new(
                    Point::new(USB_REGION_X, y),
                    Size::new(
                        crate::board::CANVAS_W as u32 - USB_REGION_X as u32,
                        USB_PANEL_PITCH as u32,
                    ),
                )
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut display)
                .ok();
                Text::with_baseline(
                    text.as_str(),
                    Point::new(USB_REGION_X + 2, y + 1),
                    tx_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .ok();
            }
            last_usb_panel = panel;
        }


        let _ = fanout;
        // 50 ms, not 100.
        //
        // With the INT pin gating the I2C read, this loop's period is
        // what decides whether a tap is seen at all, and 100 ms was
        // long enough to miss quick ones. The extra iterations cost a
        // GPIO read each; the bus is only touched when a finger is
        // actually down.
        std::thread::sleep(std::time::Duration::from_millis(50));
        tick = tick.wrapping_add(1);
    }
}
