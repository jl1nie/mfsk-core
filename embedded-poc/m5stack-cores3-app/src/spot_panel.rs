// SPDX-License-Identifier: GPL-3.0-or-later
//! The display task the spot-list receivers share (#353 item 2).
//!
//! FST4 and WSPR put the same thing on the screen — a status line, the
//! latest slot's rows, a rolling history, the link bar and the mode
//! picker over the top — and each carried its own 330-line copy of the
//! loop that draws it. Only three things differed: which `BootMode`
//! the picker highlights, which `ui::` render functions get called,
//! and the log prefix. Everything else, including the LCD bring-up,
//! the VBUS/host-mode decision, the boot summary, the RTC store and
//! the touch polling, was the same code twice — so a fix to one (the
//! peripheral-mode check that keeps the port flashable, say) had to be
//! remembered in the other.
//!
//! What is parameterised is what actually differs, in [`SpotPanel`];
//! the loop itself is [`run`]. Same shape `crate::net` uses for
//! network bring-up.
//!
//! **Not FT8/FT4.** Those two share `ui::state::UI` and
//! `display::run_log_panel`, which is a waterfall + decode ring +
//! touch/menu/TX panel — a different screen for a different job, not a
//! spot list. #353 says so explicitly: moving them onto this would be
//! the wrong direction.

use std::sync::{Arc, Mutex};

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::DrawTarget;

use display_interface_spi::SPIInterface;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::MALLOC_CAP_SPIRAM;
use mipidsi::{
    models::ILI9342CRgb565,
    options::{ColorInversion, Orientation},
    Builder,
};

use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::civil_time::civil_from_unix;
use mfsk_app_shared::ui::spot_render::{PANEL_HEIGHT, PANEL_WIDTH};
use mfsk_app_shared::ui::{link_bar, mode_picker};

/// What one receiver's screen does differently from the other's.
///
/// Implementors are unit structs; everything is an associated const or
/// a static method, because the state these reach for is the
/// process-wide `WSPR_UI`/`FST4_UI` the scan task also writes.
pub trait SpotPanel {
    /// The receiver's UI state — `SlotUiState` or `WsprUiState`.
    type Ui;

    /// Which entry the mode picker shows as current, and the name in
    /// the boot summary.
    const MODE: BootMode;
    /// Log prefix, e.g. `"wspr_app::display"`.
    const TAG: &'static str;
    /// FreeRTOS task name.
    const TASK_NAME: &'static core::ffi::CStr;
    /// Task stack, in bytes. Allocated in PSRAM — see [`spawn`].
    const STACK: u32;
    /// Task priority. WSPR's scan task is compute-bound for ~100 s of
    /// every 120, FST4's differs, and each receiver picked its own
    /// number against its own scan task; this is not a constant to
    /// unify without measuring both.
    const PRIORITY: u32;

    /// Run `f` against the receiver's UI state, holding its lock.
    ///
    /// One lock per tick covers the status update and all three draws,
    /// the way each copy of this loop already did it: contention is
    /// the scan task's once-per-slot write against a 500 ms tick.
    fn with_ui<R, F: FnOnce(&mut Self::Ui) -> R>(f: F) -> R;

    /// The two fields every one of these screens shows and neither
    /// scan task sets.
    fn set_status(ui: &mut Self::Ui, heap_kb: u32, utc_hhmmss: &str);

    /// Bumped by the scan task when the rows change; the expensive
    /// panes redraw only when it moves.
    fn dirty_seq(ui: &Self::Ui) -> u32;

    /// First paint: the whole panel, background included.
    fn render_all<D>(display: &mut D, ui: &Self::Ui) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>;

    /// The status line. Cheap, every tick.
    fn render_status<D>(display: &mut D, ui: &Self::Ui) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>;

    /// The latest slot's rows — `render_slot` for FST4,
    /// `render_discovered` for WSPR.
    fn render_rows<D>(display: &mut D, ui: &Self::Ui) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>;

    /// The rolling history pane.
    fn render_history<D>(display: &mut D, ui: &Self::Ui) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>;
}

/// Peripherals the display task takes ownership of, plus the NVS
/// handle the mode picker needs.
pub struct DisplayCtx {
    pub i2c0: esp_idf_hal::i2c::I2C0<'static>,
    pub spi2: esp_idf_hal::spi::SPI2<'static>,
    pub pins: esp_idf_hal::gpio::Pins,
    /// For `boot_mode::commit_and_restart` when the mode picker
    /// commits — this task cannot write flash itself, its stack is in
    /// PSRAM. Same `"mfsk"` namespace `settings` uses, so one handle
    /// serves both.
    pub nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn task_entry<P: SpotPanel>(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn` leaked exactly this pointer via `Box::into_raw`,
    // and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut DisplayCtx) };
    run::<P>(*ctx);
}

/// Start the display task on core 1 — deliberately not the scan
/// task's core 0.
///
/// The stack is in PSRAM. Drawing is shallow and not on any deadline,
/// and the internal DRAM this frees is what the USB host needs later:
/// with the stack in internal memory the hub's interrupt endpoint
/// allocation failed with `ESP_ERR_NO_MEM`, so the IC-705's hub
/// enumerated and its downstream CDC and audio interfaces never did —
/// `num_devices` stuck at 1 where FT8 reaches 3. Endpoint buffers have
/// to be DMA-capable internal memory; a display stack does not.
///
/// Safe because nothing on this task writes flash. The mode commits go
/// through `boot_mode::commit_and_restart`, which exists precisely
/// because a flash write aborts from a PSRAM stack.
pub fn spawn<P: SpotPanel>(ctx: DisplayCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(task_entry::<P>),
            P::TASK_NAME.as_ptr(),
            P::STACK,
            ptr,
            P::PRIORITY,
            core::ptr::null_mut(),
            1,
            MALLOC_CAP_SPIRAM,
        )
    };
    if created != 1 {
        log::error!("{}: failed to create the display task", P::TAG);
    }
}

/// `HH:MM:SS` from the system clock, or midnight if it has none — the
/// status line always shows eight digits, and whether they mean
/// anything is what the `ntp` flag beside them is for.
fn current_hhmmss() -> heapless::String<8> {
    use core::fmt::Write as _;
    let mut s: heapless::String<8> = heapless::String::new();
    match std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        Ok(d) => {
            let (_, _, _, h, m, sec) = civil_from_unix(d.as_secs() as i64);
            let _ = write!(&mut s, "{h:02}:{m:02}:{sec:02}");
        }
        Err(_) => {
            let _ = s.push_str("00:00:00");
        }
    }
    s
}

/// One touch poll: read the panel, log a contact change, and hand the
/// mode picker its update.
///
/// The log line is one per *change* of contact state, so an otherwise
/// silent capture separates "nothing was touched" from "touched, but
/// the hold never reached `OPEN_MS`". The picker's own log only speaks
/// while the overlay is up, which is exactly the case that cannot be
/// reached when the hold is the thing failing.
fn poll_touch(
    picker: &mut mode_picker::ModePicker,
    touch_int: Option<&PinDriver<'static, esp_idf_hal::gpio::Input>>,
    touch_i2c: Option<&mut esp_idf_hal::i2c::I2cDriver<'static>>,
    last_contact: &mut crate::touch::Contact,
    nvs: &Arc<Mutex<EspNvs<NvsDefault>>>,
) {
    let (Some(int), Some(i2c)) = (touch_int, touch_i2c) else {
        return;
    };
    let c = if int.is_low() {
        crate::touch::read(i2c).unwrap_or_default()
    } else {
        crate::touch::Contact::default()
    };
    if c != *last_contact {
        if c.points > 0 {
            log::info!("touch: {} pt at ({}, {})", c.points, c.x, c.y);
        } else {
            log::info!("touch: released");
        }
        *last_contact = c;
    }
    if let Some(target) = picker.update(c.points > 0, c.x, c.y) {
        log::warn!("boot_mode -> {} (touch), restarting", target.label());
        // Not written here: this task's stack is in PSRAM, and a flash
        // write aborts from one. See `boot_mode::commit_and_restart`.
        boot_mode::commit_and_restart(nvs.clone(), target);
    }
}

/// LCD bring-up (AXP2101 + AW9523B → SPI2 → mipidsi, mirroring
/// `display.rs`'s FT8-controller sequence minus the BootMode/UAC
/// branches these receivers have no use for) followed by the render
/// loop. Never returns.
pub fn run<P: SpotPanel>(ctx: DisplayCtx) -> ! {
    // Kept across the whole loop: the touch controller shares this bus
    // and the mode picker is this receiver's only way back out.
    let mut touch_i2c: Option<esp_idf_hal::i2c::I2cDriver<'static>> = None;
    let mut display = match crate::pmic::init(ctx.i2c0, ctx.pins.gpio12, ctx.pins.gpio11) {
        Ok(mut i2c) => {
            // Enable USB VBUS boost **before** `crate::uac::start_host()`
            // — AW9523B P0_1 (BUS_OUT_EN) HIGH drives the VBUS switch;
            // omission leaves VBUS floating and the host stack sees no
            // device.
            //
            // Stay a peripheral while something else is powering the
            // port. One USB-C connector cannot both take power in and
            // hand it out, so "host or peripheral" is a question about
            // the cable, not the build. The FT8 controller has checked
            // this since #163; these receivers did not, and the moment
            // their USB host stopped being opt-in that gap became the
            // board refusing to enumerate on a PC at all — the app
            // takes the PHY before a flasher can reach it, and the only
            // way back is holding the button into DOWNLOAD mode. On WSL
            // every one of those costs a `usbipd attach` as well.
            //
            // Charging is also the useful thing to do while plugged in.
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
            let host_mode = !external;
            if external {
                log::warn!(
                    "external USB power detected — staying a peripheral so the battery charges \
                     and the port stays flashable. Unplug from the PC and reset to take audio \
                     from a radio."
                );
            } else if let Err(e) = crate::pmic::enable_usb_host_vbus(&mut i2c) {
                log::error!("BUS_OUT_EN failed: {e:#}");
            }
            // The bus is kept, not dropped: the FT5x06 is on it, and
            // the mode picker is the only way out of this receiver.
            touch_i2c = Some(i2c);

            // Installing the USB host + UAC class driver **detaches
            // USB-Serial-JTAG**, i.e. the serial console, the moment it
            // returns — which is why UDP log fanout is wired first.
            //
            // Unconditional (given host mode), where it used to sit
            // behind a build-time flag on the stated condition that
            // #163 stayed open. #163 closed 2026-08-23 with ten minutes
            // of live capture, and a build-time flag cannot work at all
            // in a binary that picks its receiver at boot: choosing a
            // mode from the picker would land in a receiver whose radio
            // was compiled out. `start_host_when_ready` waits for the
            // log sink itself and says what it found.
            if host_mode {
                crate::uac::start_host_when_ready();
            } else {
                log::info!(
                    "{}: USB host not installed (peripheral mode) — the serial console stays up \
                     and audio falls back to the synthetic generator",
                    P::TAG,
                );
            }

            let driver = SpiDriver::new(
                ctx.spi2,
                ctx.pins.gpio36, // SCK  (crate::board::LCD_PIN_SCK)
                ctx.pins.gpio37, // MOSI (crate::board::LCD_PIN_MOSI)
                Option::<AnyIOPin>::None,
                &SpiDriverConfig::new(),
            )
            .expect("SPI2 driver");
            let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
            let spi_dev = SpiDeviceDriver::new(driver, Some(ctx.pins.gpio3), &spi_cfg) // CS (crate::board::LCD_PIN_CS)
                .expect("SPI device (CS=3)");
            let dc = PinDriver::output(ctx.pins.gpio35).expect("DC gpio35"); // crate::board::LCD_PIN_DC
            let di = SPIInterface::new(spi_dev, dc);

            // **2026-08-15 real-hardware fix**: this board's chip is
            // ILI9342C (see `board.rs`'s own doc comment), whose native
            // `FRAMEBUFFER_SIZE` in mipidsi is already `(320, 240)` —
            // landscape. The previous code used the *ILI9341* model
            // instead (`FRAMEBUFFER_SIZE = (240, 320)`,
            // portrait-native) plus a manual `.orientation(Deg90)` to
            // compensate — a Builder-time rotation that, empirically,
            // never correctly swapped mipidsi's own internal
            // width/height bookkeeping (every symptom chased that day —
            // partial coverage, stripes, the panel reporting 240×320
            // instead of 320×240 — traces back to this).
            //
            // **Then, per a follow-up layout request, rotated back to
            // portrait on purpose**: `.orientation(Deg90)` on top of
            // this now-correct 320×240 landscape base gives a clean
            // 240×320 canvas — confirmed via `lcd_minimal.rs`'s
            // orientation-cycling diagnostic (`R90 NORMAL`, unmirrored)
            // on real hardware. Different from the old bug: this
            // rotation is layered on the *correct* native-landscape
            // model, not used to fake landscape out of a
            // portrait-native one, so it doesn't hit the same
            // bookkeeping issue.
            let mut delay = Ets;
            match Builder::new(ILI9342CRgb565, di)
                .display_size(crate::board::NATIVE_W, crate::board::NATIVE_H)
                .orientation(Orientation::new().rotate(crate::board::ROTATION))
                .invert_colors(ColorInversion::Inverted)
                .init(&mut delay)
            {
                Ok(d) => d,
                Err(e) => {
                    log::error!("display init failed: {e:?}");
                    loop {
                        log::info!("alive (no LCD)");
                        FreeRtos::delay_ms(2000);
                    }
                }
            }
        }
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            loop {
                log::info!("alive (no PMIC/LCD)");
                FreeRtos::delay_ms(2000);
            }
        }
    };
    log::info!(
        "LCD init OK ({}x{})",
        crate::board::CANVAS_W,
        crate::board::CANVAS_H
    );

    // The 2026-08-15 real-hardware investigation behind the bring-up
    // above (three real bugs: AXP2101 DLDO1/backlight never enabled,
    // `board.rs`'s LCD_RST/TP_RST bits swapped, and `DrawTarget
    // ::clear()` itself giving partial coverage on this mipidsi/SPI
    // setup — see `pmic.rs`'s and `spot_render`'s doc comments) used a
    // standalone `lcd-minimal` bin for the raw-driver / orientation
    // diagnostics rather than growing throwaway test code here.
    // `render_all` is the real first paint.
    P::with_ui(|ui| {
        if let Err(e) = P::render_all(&mut display, ui) {
            log::error!("{}: render_all FAILED: {e:?}", P::TAG);
        }
    });

    // Mode picker: held open, so it costs no layout. Centred on this
    // 320x240 panel.
    let touch_int = PinDriver::input(ctx.pins.gpio21, esp_idf_hal::gpio::Pull::Up).ok();
    let mut boot_summary_sent = false;
    let mut rtc_stored = false;
    let mut last_contact = crate::touch::Contact::default();
    let mut picker = mode_picker::ModePicker::new(embedded_graphics::prelude::Point::new(
        (crate::board::CANVAS_W as i32 - mode_picker::WIDTH as i32) / 2,
        (crate::board::CANVAS_H as i32 - mode_picker::height() as i32) / 2,
    ));

    let mut last_dirty = u32::MAX;
    let mut tick: u32 = 0;
    loop {
        // Freeze the tables while the overlay is up — it only redraws
        // on change, so a repaint underneath erases it and it never
        // comes back.
        if picker.is_open() {
            picker.render(&mut display, P::MODE).ok();
            FreeRtos::delay_ms(50);
            poll_touch(
                &mut picker,
                touch_int.as_ref(),
                touch_i2c.as_mut(),
                &mut last_contact,
                &ctx.nvs,
            );
            if picker.take_just_closed() {
                last_dirty = u32::MAX;
            }
            continue;
        }

        let heap_kb = (unsafe { esp_idf_svc::sys::esp_get_free_heap_size() } / 1024) as u32;
        let hhmmss = current_hhmmss();

        // Same 1 Hz re-read the FT8 controller does: the enable bits
        // are only worth showing if something has looked recently.
        if let Some(i2c) = touch_i2c.as_mut() {
            crate::pmic::refresh_power_state(i2c);
            // Store the clock once NTP has made it real, so the next
            // boot has one before WiFi does.
            //
            // The predicate is provenance, not plausibility:
            // `utc_now_ms().is_some()` was true a second after boot
            // because `pmic::init` had just seeded the clock from this
            // very chip, so this wrote the RTC's own value back to it
            // and NTP — arriving 30 s later — never reached the
            // register. #354.
            if !rtc_stored && mfsk_app_shared::time_sync::clock_is_disciplined() {
                rtc_stored = true;
                if let Err(e) = crate::rtc::write_from_system_clock(i2c) {
                    log::warn!("rtc: could not store the clock: {e:#}");
                }
            }
            // Once, the first frame after a log sink exists. In host
            // mode there is no serial console, and everything this
            // reports is printed seconds before WiFi associates — the
            // staging ring has been overwritten by then. Same one-shot
            // the FT8 controller does.
            if !boot_summary_sent
                && crate::FANOUT
                    .udp
                    .try_lock()
                    .map(|g| g.is_some())
                    .unwrap_or(false)
            {
                boot_summary_sent = true;
                let (host_attempted, ..) = crate::pmic::power_state();
                let r = crate::uac::HOST_RESULT.read();
                log::warn!(
                    "[boot-summary] mode={} host_mode={host_attempted} start_host: {}",
                    P::MODE.label(),
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

        // One lock for the status update and all three draws, as each
        // copy of this loop already did — the lock is held across the
        // SPI calls rather than snapshotting the state out first (as
        // the FT8 display loop does), because contention is the scan
        // task's once-per-slot write against this 500 ms tick.
        let dirty = P::with_ui(|ui| {
            P::set_status(ui, heap_kb, &hhmmss);
            let dirty = P::dirty_seq(ui);
            // Same bar, same place, in every mode — see `link_bar`'s
            // own doc comment for why it is not a field in these
            // receivers' headers.
            link_bar::render(
                &mut display,
                &crate::uac::link_info(),
                PANEL_WIDTH,
                PANEL_HEIGHT as i32 - link_bar::HEIGHT as i32,
            )
            .ok();
            if let Err(e) = P::render_status(&mut display, ui) {
                log::warn!("{}: render_status failed: {e:?}", P::TAG);
            }
            if dirty != last_dirty {
                if let Err(e) = P::render_rows(&mut display, ui) {
                    log::warn!("{}: render_rows failed: {e:?}", P::TAG);
                }
                if let Err(e) = P::render_history(&mut display, ui) {
                    log::warn!("{}: render_history failed: {e:?}", P::TAG);
                }
                last_dirty = dirty;
            }
            dirty
        });

        // ~10 s cadence. Direct evidence that this task keeps running
        // (and the UTC clock keeps advancing) through the scan task's
        // compute-bound stretch on the other core — same purpose as the
        // FT8 controller `display.rs`'s own periodic "alive" line,
        // after a first cut of that fix turned out to have its own bug.
        if tick % 20 == 0 {
            log::info!(
                "{}: alive tick={tick} dirty={dirty} utc={hhmmss} heap={heap_kb}k",
                P::TAG,
            );
        }
        tick = tick.wrapping_add(1);

        // Touch at 50 ms while the render cadence stays at 500.
        //
        // The picker is only as responsive as whatever calls it, and
        // this loop redraws spot tables — it has no business running
        // ten times faster. So the wait polls instead of sleeping
        // through: a tap or an 800 ms hold lands either way. Checking
        // costs one GPIO read while no finger is down.
        for _ in 0..10 {
            if touch_i2c.is_some() {
                poll_touch(
                    &mut picker,
                    touch_int.as_ref(),
                    touch_i2c.as_mut(),
                    &mut last_contact,
                    &ctx.nvs,
                );
                picker.render(&mut display, P::MODE).ok();
                if picker.take_just_closed() {
                    last_dirty = u32::MAX;
                }
            }
            FreeRtos::delay_ms(50);
        }
    }
}
