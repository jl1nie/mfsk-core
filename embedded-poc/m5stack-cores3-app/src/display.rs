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
    options::{ColorInversion, ColorOrder, Orientation},
    Builder,
};

use std::sync::{Arc, Mutex};
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
const DECODED_ROWS: usize =
    ((TX_REGION_Y - decoded_list::ORIGIN_Y) as u32 / decoded_list::ROW_PX) as usize;
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

/// `MFSK_CORES3_FORCE_UAC=1` — run as a USB host even while something
/// external is powering the port. For a board fed from M5Bus rather
/// than the USB-C connector, where the two supplies do not collide.
const FORCE_UAC: bool = match option_env!("MFSK_CORES3_FORCE_UAC") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

/// LCD bring-up + render loop. Returns `!`.
#[allow(clippy::too_many_arguments)]
/// The panel's priority on `main` in FT8 mode: above the decoders
/// (5-6), so the screen never stops for a decode —
/// which at `main`'s own 1 it did, 1.2-1.4 s every slot. FST4's display
/// task has always run at 7 for the same reason. **Not FT4**, whose
/// panel stays at 1: its reply deadline is the tight one (`apps::ft4`).
///
/// Affordable because the panel is cheap now — DMA it sleeps through,
/// bars drawn on change, 6 frames/s: 8-13 % of core 0. Measured on FT8
/// SIM with the panel at 1 and at 7: 7 decodes, 6 in time, 0 cut either
/// way (2026-09-21).
///
/// **Below the audio path (8).** It sat above the UAC reader and class
/// driver (then 6) until 2026-09-22 — harmless on the SIM, where the
/// test above ran, and a source of dropped isochronous frames on a
/// radio (`uac::UAC_DRIVER_TASK_PRIORITY`).
pub const PANEL_PRIORITY: u32 = 7;

/// One panel frame, µs: 6 a second, matching the waterfall's 6 rows a
/// second (`embedded_shared::waterfall::WF_HOP`), so each redraw moves
/// it by one row.
///
/// The panel's CPU is per frame — ~18 ms of it at 10 frames/s was
/// 16-19 % of core 0 (FreeRTOS run-time counters, 2026-09-21) — and in
/// FT4 mode the panel runs above the decode so the screen never stops,
/// which makes every one of those milliseconds the decode's. Held to 6,
/// the cost falls with the rate.
const FRAME_US: i64 = 166_667;

/// Waterfall rows per SPI write: 4 rows are 1 920 bytes, 25 writes for
/// the region instead of ~375. Under the 2 KB internal-DRAM threshold on
/// purpose, so the block lands in memory the SPI DMA can read directly
/// rather than a PSRAM buffer the driver would copy into a bounce.
const WF_BLOCK_ROWS: usize = 4;

/// A full-width strip of the panel, drawn in memory and sent the way the
/// waterfall is: one address window, then byte blocks over DMA
/// ([`blit_strip!`]).
///
/// **Why the bars and the list need it.** Drawn straight to the display,
/// `embedded-graphics` text goes through `fill_contiguous`'s per-pixel
/// iterator, which `display-interface-spi` sends 64 pixels at a time —
/// the path the waterfall left because it was half of a 39 ms redraw
/// (see the waterfall block). Measured on a CoreS3 running jtty-demo
/// (2026-09-28), the status bar cost ~20 ms a redraw and the list up to
/// ~40 ms a second of core 0, almost all of it CPU, against 5 ms a second
/// for building every waterfall row. Here the same widgets draw into RAM
/// (a store per pixel) and the bytes go out as DMA the core is free
/// through.
///
/// Starts black, which is what the boot clear leaves on the panel: the
/// status bar paints only its glyph cells and the tail, and its two
/// margin rows show whatever is under them.
struct Strip {
    y0: i32,
    w: u32,
    h: u32,
    /// Big-endian RGB565, row-major — the panel's own byte order, as
    /// `waterfall::row_rgb565_be` writes it.
    buf: Vec<u8>,
}

impl Strip {
    fn new(y0: i32, w: u32, h: u32) -> Self {
        Self { y0, w, h, buf: vec![0u8; (w * h * 2) as usize] }
    }
}

impl Dimensions for Strip {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, self.y0), Size::new(self.w, self.h))
    }
}

impl DrawTarget for Strip {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(p, c) in pixels {
            let (x, y) = (p.x, p.y - self.y0);
            if x >= 0 && y >= 0 && (x as u32) < self.w && (y as u32) < self.h {
                let i = ((y as u32 * self.w + x as u32) * 2) as usize;
                let v = embedded_graphics::pixelcolor::raw::RawU16::from(c).into_inner();
                self.buf[i] = (v >> 8) as u8;
                self.buf[i + 1] = v as u8;
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let a = area.intersection(&self.bounding_box());
        if a.size.width == 0 || a.size.height == 0 {
            return Ok(());
        }
        let v = embedded_graphics::pixelcolor::raw::RawU16::from(color).into_inner();
        let (hi, lo) = ((v >> 8) as u8, v as u8);
        for y in a.top_left.y..a.top_left.y + a.size.height as i32 {
            let row = ((y - self.y0) as u32 * self.w) as usize;
            let x0 = a.top_left.x as usize;
            for x in x0..x0 + a.size.width as usize {
                self.buf[(row + x) * 2] = hi;
                self.buf[(row + x) * 2 + 1] = lo;
            }
        }
        Ok(())
    }
}

/// Send a [`Strip`] to its place on the panel: the address window once,
/// then the pixels through `$bounce`, a block of internal DRAM the SPI
/// DMA can read (the strips themselves are large enough to land in
/// PSRAM). The waterfall's block buffer serves.
macro_rules! blit_strip {
    ($display:expr, $strip:expr, $bounce:expr) => {{
        let s = &$strip;
        let opened = $display.set_pixels(
            0,
            s.y0 as u16,
            (s.w - 1) as u16,
            (s.y0 + s.h as i32 - 1) as u16,
            core::iter::empty(),
        );
        if opened.is_ok() {
            use display_interface::WriteOnlyDataCommand as _;
            for chunk in s.buf.chunks($bounce.len()) {
                $bounce[..chunk.len()].copy_from_slice(chunk);
                // SAFETY: raw data after the RAMWR `set_pixels` just issued,
                // inside the window it set — as the waterfall block does.
                let _ = unsafe { $display.dcs() }
                    .di
                    .send_data(display_interface::DataFormat::U8(&$bounce[..chunk.len()]));
            }
        }
    }};
}

pub fn run_log_panel(
    i2c0: I2C0<'static>,
    spi2: SPI2<'static>,
    pins: Pins,
    fanout: &'static LogFanout,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    mode: BootMode,
) -> ! {
    // Set false when the board finds itself on external power — see
    // the VBUS check below.
    // Which receivers take audio from a radio over USB host. FT4 was
    // missing here until 2026-09-02, which is why `apps::ft4`'s
    // `Ft4Sink` had never been fed: it registered a sink in a mode
    // where the host driver was never installed, so the receiver
    // silently ran on its baked replay slot and every number measured
    // for it came from a recording.
    //
    // WSPR and FST4 joined when they moved onto this panel from their
    // own spot-list screen (2026-09-21), which had always installed the
    // host unconditionally on battery.
    let mut host_mode = matches!(
        mode,
        BootMode::Uac | BootMode::Ft4 | BootMode::Wspr | BootMode::Fst4
    );

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
            //
            // **A board fed from the DIN Base does not have that
            // problem**: the M5Bus supplies it, so the USB-C port is
            // free to source VBUS to the radio. If the AXP2101 reports
            // that supply as VBUS-present and this check therefore
            // refuses host mode, `MFSK_CORES3_FORCE_UAC=1` is the
            // documented way past it — see `FORCE_UAC`.
            if host_mode {
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
    //
    // **DMA, and transfers the task sleeps through.** esp-idf-hal's
    // defaults are no DMA (64-byte transactions) and `polling: true`,
    // i.e. `spi_device_polling_transmit`: the calling task spins until
    // every byte is on the wire. The panel sends ~48 KB for each
    // waterfall redraw — ~20 ms at 20 MHz — and every one of those
    // milliseconds was CPU no other task on the core could have, which
    // is what made the panel cost the decode its own wire time the
    // moment it ran above it (FT4, 2026-09-21). With DMA and queued
    // transactions the task blocks on the driver's semaphore and the
    // core is free while the display is written.
    let driver = SpiDriver::new(
        spi2,
        pins.gpio36, // SCK
        pins.gpio37, // MOSI
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new().dma(esp_idf_hal::spi::Dma::Auto(4096)),
    )
    .expect("SPI2 driver");
    let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into()).polling(false);
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
        // **BGR, not RGB.** The CoreS3's ILI9342C is wired
        // blue-green-red, and mipidsi defaults to RGB, so every colour
        // this UI draws came out with its red and blue exchanged:
        // `CSS_ORANGE` (255, 165, 0) reached the panel as (0, 165, 255)
        // and read as sky blue, which is how the menu's amber hold
        // indicator was reported as "blue". Green, white, grey and
        // black are unaffected, which is why it went unnoticed — those
        // are most of this UI. The waterfall's palette is the other
        // casualty and is corrected by the same line: it is written
        // black → blue → cyan → green → lime → red, and was being
        // displayed with the blue and red ends swapped.
        .color_order(ColorOrder::Bgr)
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
        Size::new(crate::board::CANVAS_W as u32, crate::board::CANVAS_H as u32),
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
    // with a flasher. The host install is the difference between a
    // board you can re-flash and one that needs the download-mode
    // button dance.
    // `host_mode` alone. This block is not the banner — it is the
    // delay, the wait for a log sink, and `start_host()` itself. It
    // read `host_mode && USB_PANEL` for one commit, because retiring
    // the diagnostic panel gated the banner by narrowing the condition
    // of the block the banner happened to start. The USB host then
    // never installed at all: VBUS came up, all three enable bits read
    // back correctly, and nothing was ever asked to enumerate. From the
    // bench that is indistinguishable from a radio that will not talk.
    //
    // Only the drawing is optional.
    if host_mode {
        if USB_PANEL {
            let mut banner: heapless::String<40> = heapless::String::new();
            {
                use core::fmt::Write as _;
                let _ = write!(&mut banner, "USB: starting host");
            }
            Text::with_baseline(
                banner.as_str(),
                Point::new(USB_REGION_X + 2, USB_REGION_Y + 1),
                tx_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .ok();
        }
        crate::civ_usb::set_nvs(nvs.clone());
        crate::uac::start_host_when_ready();
    }

    let mut tick: u32 = 0;
    let mut boot_summary_sent = false;
    let mut rtc_stored = false;

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
    // Slot-start flags for the waterfall, parallel to `wf_snapshot`.
    // 100 bytes on this task's stack rather than a `Box`: the rows
    // beside them are 24 KB and boxed for that reason, this is not.
    let mut wf_marks: heapless::Vec<bool, { mfsk_app_shared::ui::state::WF_DEPTH }> =
        heapless::Vec::new();
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
    let mut picker = mode_picker::ModePicker::new(
        Point::new(
            (crate::board::CANVAS_W as i32 - mode_picker::WIDTH as i32) / 2,
            (crate::board::CANVAS_H as i32 - mode_picker::height() as i32) / 2,
        ),
        Size::new(crate::board::CANVAS_W as u32, crate::board::CANVAS_H as u32),
    );
    picker.set_freq_presets(mfsk_app_shared::freq_presets::for_mode(
        mode_picker::mode_name(mode).unwrap_or(""),
    ));
    let mut touch_read_failed = false;
    let mut last_wf_seq: u32 = u32::MAX;
    let mut last_decoded_fp: (usize, u32, u16) = (usize::MAX, u32::MAX, u16::MAX);
    // One flag per snapshot row: heard this slot. The UI's rule
    // (`UiState::decoded_current_iter`), not a receiver's watermark.
    let mut current_snapshot: heapless::Vec<bool, 16> = heapless::Vec::new();
    let mut last_tx_seq: u32 = 0;
    // One block of waterfall rows in wire format; see the draw site.
    let mut wf_block: Vec<u8> =
        vec![0u8; WF_BLOCK_ROWS * waterfall::WIDTH as usize * 2];
    let list_rows = if USB_PANEL {
        ((USB_REGION_Y - decoded_list::ORIGIN_Y) as u32 / decoded_list::ROW_PX) as usize
    } else {
        DECODED_ROWS
    };
    let mut status_strip = Strip::new(status_bar::ORIGIN_Y, SHARED_UI_WIDTH, status_bar::HEIGHT);
    // The link bar changes about once a second too — the battery and VBUS
    // readings it shows refresh at 1 Hz — so it pays the per-pixel path's
    // ~10 ms as often as the status bar did (jtty-demo, 2026-09-28: `bars`
    // stayed at 11-16 ms/s with only the status bar moved to a strip).
    let mut link_strip = Strip::new(LINK_BAR_Y, SHARED_UI_WIDTH, link_bar::HEIGHT);
    let mut list_strip = Strip::new(
        decoded_list::ORIGIN_Y,
        SHARED_UI_WIDTH,
        list_rows as u32 * decoded_list::ROW_PX,
    );
    let mut last_status: Option<mfsk_app_shared::ui::state::StatusInfo> = None;
    let mut last_link: Option<mfsk_app_shared::ui::link_bar::LinkInfo> = None;
    // The longest time between two frames, every ~10 s: how long the
    // screen stood still — a panel starved by something above it on its
    // core looks hung to the operator however well the rest is doing.
    let mut frame_prev_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let mut frame_gap_max_us: i64 = 0;
    let mut frame_count: u32 = 0;
    let mut frame_report_us = frame_prev_us;
    // Busy time, excluding the touch-poll wait at the end of a frame:
    // the whole frame, the waterfall feed (its FFT), the redraw.
    let mut busy_us: i64 = 0;
    let mut feed_us: i64 = 0;
    let mut wf_draw_us: i64 = 0;
    // The redraw split in two: building the RGB565 rows (CPU) and sending
    // them (the DMA wait, during which the core is free) — `wf_draw_us`
    // alone cannot say how much of the panel's time a decoder on the same
    // core actually loses.
    let mut wf_conv_us: i64 = 0;
    let mut wf_send_us: i64 = 0;
    let mut bars_us: i64 = 0;
    let mut list_us: i64 = 0;
    // Each split into drawing into its strip (CPU) and sending it (DMA).
    let mut bars_send_us: i64 = 0;
    let mut list_send_us: i64 = 0;
    let mut pre_us: i64 = 0;
    let mut frame_top_us: i64;
    loop {
        // The storage task is spawned here, on this task's stack, not
        // on the decoder's that made the first request (`storage.rs`).
        crate::storage::start_if_requested();
        {
            let now = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            frame_gap_max_us = frame_gap_max_us.max(now - frame_prev_us);
            frame_prev_us = now;
            frame_count += 1;
            frame_top_us = now;
            if now - frame_report_us >= 10_000_000 {
                let secs = ((now - frame_report_us) / 1_000_000).max(1);
                log::info!(
                    "panel: {frame_count} frames, longest gap {} ms | busy {} ms/s = feed {} + \
                     waterfall {} (rows {} + send {}) + bars {} (send {}) + list {} (send {}) + pre-draw {} (incl. feed) + rest",
                    frame_gap_max_us / 1_000,
                    busy_us / 1_000 / secs,
                    feed_us / 1_000 / secs,
                    wf_draw_us / 1_000 / secs,
                    wf_conv_us / 1_000 / secs,
                    wf_send_us / 1_000 / secs,
                    bars_us / 1_000 / secs,
                    bars_send_us / 1_000 / secs,
                    list_us / 1_000 / secs,
                    list_send_us / 1_000 / secs,
                    pre_us / 1_000 / secs,
                );
                bars_us = 0;
                list_us = 0;
                bars_send_us = 0;
                list_send_us = 0;
                pre_us = 0;
                // The per-task walk stops the USB host's interrupt for
                // milliseconds and drops isochronous packets
                // (`board::log_task_cpu`); on a radio, idle only.
                if host_mode {
                    crate::board::log_idle_cpu();
                } else {
                    crate::board::log_task_cpu();
                }
                frame_gap_max_us = 0;
                frame_count = 0;
                busy_us = 0;
                feed_us = 0;
                wf_draw_us = 0;
                wf_conv_us = 0;
                wf_send_us = 0;
                frame_report_us = now;
            }
        }
        // The waterfall's rows, built here from the audio itself — the
        // same feed in every mode (`waterfall_feed`).
        {
            let t = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            crate::waterfall_feed::drain_to_ui();
            feed_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t;
        }

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
                // Store the clock once NTP has made it real, so the
                // next boot has one before WiFi does.
                //
                // The predicate is provenance, not plausibility:
                // `utc_now_ms().is_some()` was true a second after
                // boot because `pmic::init` had just seeded the clock
                // from this very chip, so this wrote the RTC's own
                // value back to it and NTP — arriving 30 s later —
                // never reached the register. #354.
                if !rtc_stored && mfsk_app_shared::time_sync::clock_is_disciplined() {
                    rtc_stored = true;
                    if let Err(e) = crate::rtc::write_from_system_clock(i2c) {
                        log::warn!("rtc: could not store the clock: {e:#}");
                    }
                }
                // Once, the first frame after a sink exists to receive
                // it. In host mode this is the only record there will
                // ever be of how the boot went.
                if !boot_summary_sent
                    && crate::FANOUT
                        .udp
                        .try_lock()
                        .map(|g| g.is_some())
                        .unwrap_or(false)
                {
                    boot_summary_sent = true;
                    let r = crate::uac::HOST_RESULT.read();
                    log::warn!(
                        "[boot-summary] mode={} host_mode={host_mode} start_host: {}",
                        mode.label(),
                        if r.is_empty() {
                            "never called"
                        } else {
                            r.as_str()
                        }
                    );
                    let rt = crate::rtc::RTC_RESULT.read();
                    log::warn!(
                        "[boot-summary] rtc: {}",
                        if rt.is_empty() {
                            "no result recorded"
                        } else {
                            rt.as_str()
                        }
                    );
                    crate::pmic::log_boot_summary(i2c);
                }
            }
        }
        if let Some(commit) = poll_touch_once(
            &touch_int,
            pmic_i2c.as_mut(),
            &mut last_touch,
            &mut picker,
            &mut touch_read_failed,
        ) {
            apply_commit(&nvs, commit);
        }
        picker
            .render(&mut display, mode, crate::grid_source(), crate::wifi_pref(), rig_hz())
            .ok();
        if picker.take_just_closed() {
            // The overlay covered the panel; force everything back.
            last_usb_panel.clear();
            last_wf_seq = u32::MAX;
            last_decoded_fp = (usize::MAX, u32::MAX, u16::MAX);
            last_tx_seq = last_tx_seq.wrapping_add(1);
            last_status = None;
            last_link = None;
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
            //
            // **Not while a radio's audio streams**: the same kernel
            // walk as `[cpu]`, under a critical section, and it drops
            // USB audio packets (`board::log_task_cpu`). Tick 0 still
            // reports: that is before any audio can stream, since the
            // radio's enumeration alone takes seconds.
            if tick % 300 == 0 && (!host_mode || tick == 0) {
                crate::board::log_task_stacks();
            }
        }

        let status_snapshot;
        let decoded_fp;
        let wf_seq;
        let wf_due_frame;
        let tx_seq;
        let tx_line_snapshot: heapless::String<48>;
        // What the strip shows: the acquisition, while one is running.
        let acq_line_snapshot: heapless::String<32>;
        {
            let Ok(mut ui) = UI.lock() else {
                log::warn!("UI mutex poisoned — skipping render frame");
                continue;
            };
            ui.status.mode = mode_picker::mode_name(mode);
            // The UTC field has existed in `StatusInfo` since the bar
            // was written and nothing ever wrote it, so the panel read
            // `--:--:--` whatever the clock was doing. That mattered
            // more than a blank field usually does: this receiver's
            // slot grid can only anchor while the clock is plausible,
            // and when it is not, thirty candidates a slot decode to
            // nothing. The one indicator that would have said so was
            // the one that was never connected.
            let sod =
                mfsk_app_shared::time_sync::utc_now_ms().map(|ms| ((ms / 1000) % 86_400) as u32);
            // **The heap figure moves with the clock, not every frame.**
            // The bar redraws only when its snapshot changes, and the
            // free heap (internal + PSRAM) changes by KBs on almost every
            // frame in a mode that churns PSRAM continuously — JTTY
            // allocates and frees a ~605 KB window every 472 ms — which
            // defeated that gate: the status bar repainted at the full
            // 6 frames/s, ~12 ms each, ~70 ms of every second of core 0,
            // above the receiver's Back (jtty-demo, 2026-09-28). The
            // clock needs a redraw once a second anyway; the heap rides
            // it. With no clock, once a second by frame count instead.
            let second_ticked = sod != ui.status.utc_sod
                || (sod.is_none() && tick % (1_000_000 / FRAME_US) as u32 == 0);
            if second_ticked {
                ui.status.free_heap_kb = (heap / 1024) as u32;
            }
            ui.status.utc_sod = sod;
            status_snapshot = ui.status.clone();
            decoded_snapshot.clear();
            current_snapshot.clear();
            let mut current_mask: u16 = 0;
            for (k, (row, current)) in ui.decoded_current_iter().enumerate() {
                if decoded_snapshot.push(row.clone()).is_err() {
                    break;
                }
                let _ = current_snapshot.push(current);
                if current {
                    current_mask |= 1 << k;
                }
            }
            wf_seq = ui.wf_push_seq();
            let wf_due = wf_seq != last_wf_seq;
            wf_due_frame = wf_due;
            // Copy the waterfall only when it moved.
            //
            // The `wf_seq` gate below was already skipping the *draw*;
            // the 13.5 KB copy ran every frame regardless, ten times a
            // second, for a surface that changes once per slot.
            if wf_due {
                wf_snapshot.clear();
                for line in ui.waterfall_iter() {
                    if wf_snapshot.push(*line).is_err() {
                        break;
                    }
                }
                wf_marks.clear();
                for m in ui.waterfall_marks_iter() {
                    if wf_marks.push(*m).is_err() {
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
            let mut abuf: heapless::String<32> = heapless::String::new();
            for ch in ui.acq_line().chars() {
                if abuf.push(ch).is_err() {
                    break;
                }
            }
            acq_line_snapshot = abuf;
            // **Which rows are green is decided here, by time.** A
            // row is heard this slot while it is younger than one slot
            // period, so a slot that decodes nothing turns the last
            // one's stations white by itself. The mask is in the
            // fingerprint for that reason: the rows do not change when
            // they age, and a redraw that never fires cannot repaint
            // them white.
            decoded_fp = (decoded_snapshot.len(), ui.decoded_seq(), current_mask);
        }

        // Freeze what is underneath while the overlay is up. The
        // widget only redraws itself on change, so anything painted
        // over it stays painted over it — which is why the picker
        // vanished a moment after opening.
        if picker.is_open() {
            picker
            .render(&mut display, mode, crate::grid_source(), crate::wifi_pref(), rig_hz())
            .ok();
            // The overlay is the one screen that is nothing but input;
            // spend its idle time sampling rather than sleeping.
            if let Some(commit) = pump_touch(
                50,
                &touch_int,
                pmic_i2c.as_mut(),
                &mut last_touch,
                &mut picker,
                &mut touch_read_failed,
            ) {
                apply_commit(&nvs, commit);
            }
            tick = tick.wrapping_add(1);
            continue;
        }

        let t_bars = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        pre_us += t_bars - frame_top_us;
        // **Both bars only when what they show has changed** — or the
        // picker has closed over them, which clears `last_*`. They used
        // to repaint every frame on the reasoning that they were cheap;
        // measured, the two together were 126-178 ms of every second on
        // a CoreS3 panel, more than the station list and the waterfall
        // feed combined, for text that changes about once a second.
        if last_status.as_ref() != Some(&status_snapshot) {
            status_bar::render(&mut status_strip, &status_snapshot, SHARED_UI_WIDTH).ok();
            let t_send = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            blit_strip!(display, status_strip, wf_block);
            bars_send_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_send;
            last_status = Some(status_snapshot.clone());
        }
        let link = crate::uac::link_info();
        if last_link != Some(link) {
            link_bar::render(&mut link_strip, &link, SHARED_UI_WIDTH, LINK_BAR_Y).ok();
            let t_send = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            blit_strip!(display, link_strip, wf_block);
            bars_send_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_send;
            last_link = Some(link);
        }
        bars_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_bars;

        if wf_due_frame {
            let t = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            // **As byte blocks, not through `fill_contiguous`.** That
            // path hands `display-interface-spi` a per-pixel iterator it
            // sends 64 pixels at a time — ~375 SPI transactions for this
            // region, each with the driver's own overhead, which was
            // half of a ~39 ms redraw against ~20 ms of wire time at
            // 20 MHz. Here the window is opened once (an empty pixel
            // iterator sends only the address window and RAMWR) and the
            // rows follow as prebuilt RGB565 blocks of `WF_BLOCK_ROWS`
            // rows, one `spi.write` each. Same pixels as
            // `waterfall::render_marked` (`row_rgb565_be`).
            let w = SHARED_UI_WIDTH.min(waterfall::WIDTH) as usize;
            let n = wf_snapshot.len().min(mfsk_app_shared::ui::state::WF_DEPTH);
            let blank = waterfall::HEIGHT as usize - n;
            let first = wf_snapshot.len() - n;
            let opened = display.set_pixels(
                0,
                waterfall::ORIGIN_Y as u16,
                (w - 1) as u16,
                (waterfall::ORIGIN_Y + waterfall::HEIGHT as i32 - 1) as u16,
                core::iter::empty(),
            );
            if opened.is_ok() {
                let mut row = 0usize;
                while row < waterfall::HEIGHT as usize {
                    let rows = WF_BLOCK_ROWS.min(waterfall::HEIGHT as usize - row);
                    let t_conv = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                    for k in 0..rows {
                        let r = row + k;
                        let (line, marked) = if r < blank {
                            (None, false)
                        } else {
                            let j = first + r - blank;
                            (wf_snapshot.get(j), wf_marks.get(j).copied().unwrap_or(false))
                        };
                        waterfall::row_rgb565_be(
                            line,
                            marked,
                            w,
                            &mut wf_block[k * w * 2..(k + 1) * w * 2],
                        );
                    }
                    // SAFETY: raw data after the RAMWR `set_pixels` just
                    // issued, inside the window it set — nothing the
                    // driver tracks is changed.
                    use display_interface::WriteOnlyDataCommand as _;
                    let t_send = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
                    wf_conv_us += t_send - t_conv;
                    let _ = unsafe { display.dcs() }
                        .di
                        .send_data(display_interface::DataFormat::U8(&wf_block[..rows * w * 2]));
                    wf_send_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_send;
                    row += rows;
                }
            }
            wf_draw_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t;
            last_wf_seq = wf_seq;
        }

        if decoded_fp != last_decoded_fp {
            let t = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            decoded_list::render_in_flags(
                &mut list_strip,
                &decoded_snapshot,
                &current_snapshot,
                None,
                SHARED_UI_WIDTH,
                decoded_list::ORIGIN_Y,
                list_rows,
            )
            .ok();
            let t_send = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
            blit_strip!(display, list_strip, wf_block);
            list_send_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_send;
            list_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t;
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
            let text = if !acq_line_snapshot.is_empty() {
                acq_line_snapshot.as_str()
            } else if tx_line_snapshot.is_empty() {
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
        busy_us += unsafe { esp_idf_svc::sys::esp_timer_get_time() } - frame_top_us;
        // The frame's idle time, spent sampling touch.
        //
        // It used to be `sleep(50)`, on the reasoning that the loop
        // period decides whether a tap is seen — true, but the period
        // is the render plus the sleep, and the render is the larger
        // half. Polling inside the idle time instead puts three or four
        // samples under a ~100 ms tap however long the frame took.
        // Paced to `FRAME_US` from the top of the frame, not a fixed
        // wait after it: the frame rate is what costs, and holding it
        // is what lets the panel run above FT4's decode without taking
        // the decode's time (see `FRAME_US`). Touch is still sampled
        // every 12 ms of the wait. At least 20 ms, so a slow frame
        // still leaves the lower tasks a gap.
        let spent_ms = (unsafe { esp_idf_svc::sys::esp_timer_get_time() } - frame_top_us) / 1_000;
        let wait_ms = (FRAME_US / 1_000 - spent_ms).max(20) as u64;
        if let Some(commit) = pump_touch(
            wait_ms,
            &touch_int,
            pmic_i2c.as_mut(),
            &mut last_touch,
            &mut picker,
            &mut touch_read_failed,
        ) {
            apply_commit(&nvs, commit);
        }
        tick = tick.wrapping_add(1);
    }
}

/// One touch poll, fed to the picker.
///
/// Split out of the render loop because the loop's period is not its
/// sleep: a frame draws the waterfall, the decoded list and the status
/// bar first, and touch was only sampled once per frame behind all of
/// that. A tap is ~100 ms of contact; a frame that takes longer misses
/// it outright, which is what "the panel is unresponsive" was.
///
/// The I2C bus is still only touched while the INT pin says a finger is
/// down (#163's `i2c_cmd_link` churn is the reason), so idling costs a
/// GPIO read per poll and nothing else.
fn poll_touch_once(
    touch_int: &PinDriver<'_, esp_idf_hal::gpio::Input>,
    i2c: Option<&mut esp_idf_hal::i2c::I2cDriver<'static>>,
    last_touch: &mut crate::touch::Contact,
    picker: &mut mode_picker::ModePicker,
    read_failed: &mut bool,
) -> Option<mode_picker::Commit> {
    if touch_int.is_low() {
        let Some(i2c) = i2c else {
            return None;
        };
        match crate::touch::read(i2c) {
            Some(c) => {
                if c != *last_touch {
                    if c.points > 0 {
                        log::info!("touch: {} pt at ({}, {})", c.points, c.x, c.y);
                    } else {
                        log::info!("touch: released");
                    }
                    *last_touch = c;
                }
                return picker.update(c.points > 0, c.x, c.y);
            }
            None => {
                if !*read_failed {
                    log::warn!("touch: read failed — not repeating this");
                    *read_failed = true;
                }
            }
        }
    } else if last_touch.points > 0 {
        // The pin going high is the release; there is nothing to read
        // for it.
        log::info!("touch: released");
        *last_touch = crate::touch::Contact::default();
    }
    // With no finger down the picker still needs a call to let an
    // arming lapse and to notice the release.
    if last_touch.points == 0 {
        return picker.update(false, 0, 0);
    }
    None
}

/// Idle for `ms`, sampling touch throughout instead of sleeping
/// through it. Returns early on a commit.
fn pump_touch(
    ms: u64,
    touch_int: &PinDriver<'_, esp_idf_hal::gpio::Input>,
    mut i2c: Option<&mut esp_idf_hal::i2c::I2cDriver<'static>>,
    last_touch: &mut crate::touch::Contact,
    picker: &mut mode_picker::ModePicker,
    read_failed: &mut bool,
) -> Option<mode_picker::Commit> {
    /// Fast enough that a short tap lands on at least three polls, slow
    /// enough to stay out of the way — the panel's own report rate is
    /// ~60 Hz and the controller holds the last contact between reads.
    const STEP_MS: u64 = 12;
    let deadline = std::time::Instant::now() + std::time::Duration::from_millis(ms);
    loop {
        if let Some(c) = poll_touch_once(
            touch_int,
            i2c.as_deref_mut(),
            last_touch,
            picker,
            read_failed,
        ) {
            return Some(c);
        }
        if std::time::Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(std::time::Duration::from_millis(STEP_MS));
    }
}

/// Persist what the picker committed and restart into it.
///
/// Through `boot_mode::commit_and_restart` / `commit_config_and_restart`,
/// which write from a task of their own: this panel also runs on a
/// PSRAM stack (see [`spawn_log_panel`]), and a flash write aborts from
/// one. From `main` the hand-off costs nothing and changes nothing.
/// The rig's dial as the status bar has it — CAT's latest reading.
/// `try_lock`: the picker's `*` can wait a frame, the panel cannot.
fn rig_hz() -> Option<u32> {
    UI.try_lock().ok().and_then(|ui| ui.status.rig_freq_hz)
}

fn apply_commit(nvs: &Arc<Mutex<EspNvs<NvsDefault>>>, commit: mode_picker::Commit) {
    // A dial change is not a restart: nothing to flush, nothing to
    // write from here (the CAT task saves it, off this PSRAM stack).
    if let mode_picker::Commit::Freq(hz) = commit {
        log::warn!("freq -> {hz} Hz (touch)");
        crate::civ_usb::request_freq(hz);
        return;
    }
    // Every branch restarts the board, and `all.txt` and `qso.adi` are
    // held in PSRAM until a write point (`storage`) — put them on flash
    // first. A deliberate restart takes the stall.
    if !crate::storage::flush_blocking(std::time::Duration::from_secs(5)) {
        log::warn!("storage: held logs not confirmed written before restart");
    }
    match commit {
        mode_picker::Commit::Mode(target) => {
            log::warn!("boot_mode -> {} (touch), restarting", target.label());
            boot_mode::commit_and_restart(nvs.clone(), target);
        }
        mode_picker::Commit::Grid(src) => {
            log::warn!("grid source -> {} (touch), restarting", src.label());
            crate::commit_config_and_restart(nvs.clone(), crate::ConfigChoice::Grid(src));
        }
        mode_picker::Commit::Wifi(pref) => {
            log::warn!("wifi -> {} (touch), restarting", pref.label());
            crate::commit_config_and_restart(nvs.clone(), crate::ConfigChoice::Wifi(pref));
        }
        mode_picker::Commit::Freq(_) => unreachable!("handled above"),
    }
}

/// Run [`run_log_panel`] on its own core-1 task, stack in PSRAM, for a
/// receiver whose decode keeps core 0 busy.
///
/// **One panel for every mode.** WSPR and FST4 used to draw a spot-list
/// screen of their own (`spot_panel`); they now show exactly what FT8
/// and FT4 do — the same waterfall, decode list, status and link bars,
/// and mode picker — and feed it through `ui::state::UI` the way FT4
/// does. What still differs is *where* the panel runs. FT8 and FT4 call
/// it from `main` (core 0, priority 1); WSPR's scan task holds core 0
/// at priority 5 for ~100 s of every 120, which would freeze the panel
/// and its touch polling — and the mode picker is the only way out of a
/// receiver. So those two start it here, on core 1, at their own
/// display priority, as the spot-list task did.
///
/// The stack is in PSRAM for the reason the spot-list task's was: the
/// USB host's endpoint buffers need DMA-capable internal DRAM, and with
/// a display stack there the IC-705's hub enumerated but its CDC and
/// audio interfaces did not. Nothing on this panel writes flash itself
/// ([`apply_commit`]).
pub fn spawn_log_panel(
    display: crate::boot::Display,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    mode: BootMode,
    stack: u32,
    priority: u32,
) {
    struct Ctx {
        display: crate::boot::Display,
        nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
        mode: BootMode,
    }
    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `spawn_log_panel` leaked exactly this box.
        let ctx = unsafe { Box::from_raw(arg as *mut Ctx) };
        let Ctx { display, nvs, mode } = *ctx;
        run_log_panel(
            display.i2c0,
            display.spi2,
            display.pins,
            &crate::FANOUT,
            nvs,
            mode,
        )
    }
    let ptr = Box::into_raw(Box::new(Ctx { display, nvs, mode })) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(entry),
            c"display".as_ptr(),
            stack,
            ptr,
            priority,
            core::ptr::null_mut(),
            1,
            esp_idf_svc::sys::MALLOC_CAP_SPIRAM,
        )
    };
    if created != 1 {
        log::error!("display: failed to create the panel task ({stack} B PSRAM stack)");
    }
}
