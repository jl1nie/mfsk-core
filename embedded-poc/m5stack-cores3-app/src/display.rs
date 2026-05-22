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
    gpio::{AnyIOPin, PinDriver, Pins},
    i2c::I2C0,
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2},
    units::FromValueType,
};
use mipidsi::{
    models::ILI9341Rgb565,
    options::{ColorInversion, Orientation, Rotation},
    Builder,
};

use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use mfsk_app_shared::boot_mode::BootMode;
use mfsk_app_shared::log_sink::LogFanout;
use mfsk_app_shared::ui::{decoded_list, state::UI, status_bar, waterfall};

const TX_REGION_Y: i32 = 226;
const TX_REGION_H: u32 = 14;
const SHARED_UI_WIDTH: u32 = 135;

/// LCD bring-up + render loop. Returns `!`.
#[allow(clippy::too_many_arguments)]
pub fn run_log_panel(
    i2c0: I2C0<'static>,
    spi2: SPI2<'static>,
    pins: Pins,
    fanout: &'static LogFanout,
    _nvs: EspNvs<NvsDefault>,
    mode: BootMode,
) -> ! {
    // ── PMIC: AXP2101 + AW9523B → LCD power rails + RST + BL. ──────────
    match crate::pmic::init(i2c0, pins.gpio12, pins.gpio11) {
        Ok(i2c) => {
            drop(i2c); // Phase 6-Core touch driver reclaims the bus.
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
    let spi_dev = SpiDeviceDriver::new(driver, Some(pins.gpio3), &spi_cfg)
        .expect("SPI device (CS=3)");

    let dc = PinDriver::output(pins.gpio35).expect("DC gpio35");
    let di = SPIInterface::new(spi_dev, dc);

    // ILI9342C is ILI9341-compatible — use mipidsi's ILI9341 model.
    // No reset_pin(): RST was already cycled by AW9523B in pmic::init;
    // mipidsi will send SWRESET via the command interface as fallback.
    // display_size MUST be (240, 320) (native portrait controller dims);
    // Deg90 rotation maps to 320×240 landscape in embedded-graphics space.
    let mut delay = Ets;
    let mut display = match Builder::new(ILI9341Rgb565, di)
        .display_size(240, 320)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
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
        "LCD init OK (ILI9342C/CoreS3 via ILI9341 model, {}x{})",
        crate::board::LCD_WIDTH,
        crate::board::LCD_HEIGHT
    );
    display.clear(Rgb565::BLACK).ok();

    let tx_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::new(0, 0, 8))
        .build();
    let tx_bg = Rgb565::new(0, 0, 8);

    Rectangle::new(Point::new(0, TX_REGION_Y), Size::new(SHARED_UI_WIDTH, TX_REGION_H))
        .into_styled(PrimitiveStyle::with_fill(tx_bg))
        .draw(&mut display)
        .ok();
    let mode_line: heapless::String<32> = {
        let mut s: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!("CoreS3 Mode: {}", mode.label()),
        );
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

    let mut tick: u32 = 0;
    let mut last_wf_seq: u32 = u32::MAX;
    let mut last_decoded_fp: (usize, u32) = (usize::MAX, u32::MAX);
    let mut last_tx_seq: u32 = 0;
    loop {
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
            log::info!(
                "alive tick={tick} free_heap={heap} internal={internal} largest={internal_largest}"
            );
        }

        let status_snapshot;
        let decoded_snapshot;
        let wf_snapshot: heapless::Vec<
            mfsk_app_shared::ui::state::WfLine,
            { mfsk_app_shared::ui::state::WF_DEPTH },
        >;
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
            decoded_snapshot = ui
                .decoded_iter()
                .cloned()
                .collect::<heapless::Vec<_, 16>>();
            wf_snapshot = ui.waterfall_iter().cloned().collect();
            wf_seq = ui.wf_push_seq();
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

        status_bar::render(&mut display, &status_snapshot).ok();

        if wf_seq != last_wf_seq {
            let wf_refs: heapless::Vec<
                &mfsk_app_shared::ui::state::WfLine,
                { mfsk_app_shared::ui::state::WF_DEPTH },
            > = wf_snapshot.iter().collect();
            waterfall::render(&mut display, &wf_refs).ok();
            last_wf_seq = wf_seq;
        }

        if decoded_fp != last_decoded_fp {
            decoded_list::render(&mut display, &decoded_snapshot).ok();
            last_decoded_fp = decoded_fp;
        }

        if tx_seq != last_tx_seq {
            Rectangle::new(Point::new(0, TX_REGION_Y), Size::new(SHARED_UI_WIDTH, TX_REGION_H))
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

        let _ = fanout;
        std::thread::sleep(std::time::Duration::from_millis(100));
        tick = tick.wrapping_add(1);
    }
}
