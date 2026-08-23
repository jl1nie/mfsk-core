//! Absolute-minimum CoreS3 LCD bring-up test.
//!
//! No WiFi, no NVS, no scan/display task split, no WSPR/FT8 logic —
//! just PMIC → SPI2 → mipidsi → full-screen color fills, all inline
//! on the default `app_main` task. Exists to isolate whether the
//! CoreS3's LCD genuinely lights up on this board at all, independent
//! of every other variable `wspr-app`/`mfsk-core-m5stack-cores3-app`
//! carry (2026-08-15: neither showed anything on a real board, and
//! this may be the first time anyone has actually looked at this
//! board's screen rather than trusting a serial "LCD init OK" log
//! line).
//!
//! Also reads the AW9523B `OUT0` register back after writing it, to
//! rule out a silently-failed I2C write as the reason the backlight
//! bit doesn't take.

#![allow(dead_code)]

#[path = "../board.rs"]
mod board;
#[path = "../log_slot.rs"]
mod log_slot;
#[path = "../pmic.rs"]
mod pmic;
// `pmic::init` reads the RTC to set the system clock. This bin has no
// use for the time, but it shares the driver.
#[path = "../rtc.rs"]
mod rtc;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};

use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

use display_interface_spi::SPIInterface;
use mipidsi::{
    models::ILI9342CRgb565,
    options::{ColorInversion, Orientation, Rotation},
    Builder,
};

/// Fill each quarter of the current screen a different colour:
/// top-left RED, top-right GREEN, bottom-left BLUE, bottom-right
/// YELLOW. Reveals rotation, mirroring, *and* coverage gaps all in
/// one pattern — a solid fill can't show any of those.
///
/// `w`/`h` are passed in rather than read from `display.bounding_box()`
/// — that reported a stale `320x240` for every orientation in an
/// earlier version of this test, including ones that should have
/// swapped to `240x320`, and stripes on the real panel (real-hardware
/// observation, 2026-08-15) are exactly what a too-wide rectangle
/// write looks like: the LCD controller wraps the column address
/// once the write exceeds the actual addressable window.
fn draw_quadrants<D>(display: &mut D, w: u32, h: u32, label: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (hw, hh) = (w / 2, h / 2);
    let quads = [
        (Point::new(0, 0), Rgb565::RED),
        (Point::new(hw as i32, 0), Rgb565::GREEN),
        (Point::new(0, hh as i32), Rgb565::BLUE),
        (Point::new(hw as i32, hh as i32), Rgb565::YELLOW),
    ];
    for (origin, color) in quads {
        Rectangle::new(origin, Size::new(hw, hh))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(display)?;
    }
    // 3 px white stroke just inside the full w×h edge — any coverage
    // gap (the "bottom white"/"stripes" bugs already found today)
    // shows up as a broken or missing border segment, not just a
    // vague "does it look right" judgement call.
    Rectangle::new(
        Point::new(1, 1),
        Size::new(w.saturating_sub(2), h.saturating_sub(2)),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 3))
    .draw(display)?;
    // Centered text label — legible only if this orientation's rotation
    // is actually right-side-up; backwards/upside-down text is itself
    // a direct answer about rotation, no color-naming required.
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::BLACK)
        .background_color(Rgb565::WHITE)
        .build();
    let text_w = label.len() as i32 * 10;
    Text::with_baseline(
        label,
        Point::new((w as i32 - text_w) / 2, (h as i32 - 20) / 2),
        style,
        Baseline::Top,
    )
    .draw(display)?;
    Ok(())
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== lcd_minimal boot ===");

    let peripherals = Peripherals::take().expect("peripherals taken twice");

    let mut display = match pmic::init(
        peripherals.i2c0,
        peripherals.pins.gpio12,
        peripherals.pins.gpio11,
    ) {
        Ok(mut i2c) => {
            // Read AW9523B OUT0 back — rules out a silently-failed
            // write as the reason LCD_BL doesn't visibly light.
            match read_reg(&mut i2c, board::AW9523B_I2C_ADDR, 0x02) {
                Ok(v) => log::info!(
                    "lcd_minimal: AW9523B OUT0 readback = {v:#04x} (expect {:#04x}, LCD_BL bit set)",
                    board::AW9523_P0_LCD_BL
                ),
                Err(e) => log::error!("lcd_minimal: AW9523B OUT0 readback failed: {e:#}"),
            }
            drop(i2c);

            let driver = SpiDriver::new(
                peripherals.spi2,
                peripherals.pins.gpio36, // SCK
                peripherals.pins.gpio37, // MOSI
                Option::<AnyIOPin>::None,
                &SpiDriverConfig::new(),
            )
            .expect("SPI2 driver");
            let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
            let spi_dev = SpiDeviceDriver::new(driver, Some(peripherals.pins.gpio3), &spi_cfg)
                .expect("SPI device (CS=3)");
            let dc =
                esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio35).expect("DC gpio35");
            let di = SPIInterface::new(spi_dev, dc);

            // Correct model this time (see `wspr_app.rs`'s 2026-08-15
            // fix note): ILI9342CRgb565's native FRAMEBUFFER_SIZE is
            // already (320, 240), landscape. No orientation baked in
            // here — the loop below cycles `set_orientation` on top of
            // this correct base to find the right *portrait* (240×320)
            // config for the new "rotate 90°" layout request.
            let mut delay = Ets;
            match Builder::new(ILI9342CRgb565, di)
                .display_size(320, 240)
                .invert_colors(ColorInversion::Inverted)
                .init(&mut delay)
            {
                Ok(d) => {
                    log::info!("lcd_minimal: mipidsi init OK");
                    d
                }
                Err(e) => {
                    log::error!("lcd_minimal: mipidsi init FAILED: {e:?}");
                    loop {
                        FreeRtos::delay_ms(2000);
                    }
                }
            }
        }
        Err(e) => {
            log::error!("lcd_minimal: PMIC init FAILED: {e:#}");
            loop {
                FreeRtos::delay_ms(2000);
            }
        }
    };

    log::info!(
        "lcd_minimal: entering PORTRAIT-orientation test (240x320, \
         'rotate 90 deg' request) — read the on-screen label directly \
         (legible + right-side-up only if that orientation is correct), \
         check the white border is unbroken on all 4 edges, 6 s per \
         attempt."
    );
    // Deg90/Deg270 on top of the model's native 320×240 landscape base
    // swap to a 240×320 portrait canvas — the two candidates for the
    // requested "rotate 90°" layout. (Deg0/Deg180 would stay 320×240
    // landscape, not what's being tested here.)
    let candidates = [
        ("R90 NORMAL", Rotation::Deg90, false),
        ("R90 MIRROR", Rotation::Deg90, true),
        ("R270 NORMAL", Rotation::Deg270, false),
        ("R270 MIRROR", Rotation::Deg270, true),
    ];
    loop {
        for (label, rotation, mirrored) in candidates {
            let orientation = if mirrored {
                Orientation::new().rotate(rotation).flip_horizontal()
            } else {
                Orientation::new().rotate(rotation)
            };
            match display.set_orientation(orientation) {
                Ok(()) => match draw_quadrants(&mut display, 240, 320, label) {
                    Ok(()) => log::info!("lcd_minimal: showing [{label}] now"),
                    Err(e) => log::error!("lcd_minimal: [{label}] draw FAILED: {e:?}"),
                },
                Err(e) => log::error!("lcd_minimal: [{label}] set_orientation FAILED: {e:?}"),
            }
            FreeRtos::delay_ms(6000);
        }
    }
}

fn read_reg(i2c: &mut I2cDriver<'_>, addr: u8, reg: u8) -> anyhow::Result<u8> {
    let mut buf = [0u8; 1];
    i2c.write_read(addr, &[reg], &mut buf, 100)
        .map_err(|e| anyhow::anyhow!("I2C read 0x{addr:02x} reg 0x{reg:02x} failed: {e}"))?;
    Ok(buf[0])
}
