//! LCD bring-up + render loop for M5Stack Core2 (ILI9342C, 320×240).
//!
//! Sibling of `m5stack-s3-app/src/display.rs`. Two structural deltas
//! vs the S3 version:
//!
//! 1. **No audio/I2S init.** Phase 2 has no speaker output — the
//!    decode pipeline is fed by `embedded_shared::wav_sim`, so the
//!    s3-app's ES8311 + I2S setup is dropped. PDM mic / NS4168 are
//!    Phase 2.5+ items.
//! 2. **No button polling.** Core2 has no GPIO buttons (3 touch zones
//!    via FT6336U, deferred). The boot-mode flip path is gone — mode
//!    is whatever NVS holds at boot, period.
//!
//! Layout reuse: `mfsk_app_shared::ui::{decoded_list, status_bar,
//! waterfall}` hardcode 135 px width (M5StickS3 panel). For Phase 2
//! we render them into the **top-left 135×240 corner** of the
//! 320×240 Core2 panel; the right ~185 px stays black. Widening the
//! shared draw routines to a runtime canvas dim is a Phase 2.5 item.

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
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI3},
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

/// TX-line strip lives at the same y the S3 app uses (the shared UI
/// layout hardcodes y=226). We keep the strip the same 135 px wide
/// for now so the existing draw routines paint it consistently with
/// the rest of the borrowed-from-s3-app layout.
const TX_REGION_Y: i32 = 226;
const TX_REGION_H: u32 = 14;
const SHARED_UI_WIDTH: u32 = 135;

/// LCD bring-up + render loop. Returns `!`.
///
/// Peripherals are split out (i2c0 / spi3 / pins) instead of taking
/// `Peripherals` whole because Phase 2's WiFi path consumes
/// `peripherals.modem` first, which prevents a full-struct move.
/// NVS handle is kept for parity with the s3-app signature even
/// though Phase 2 has no flip path — Phase 2.5 (touch) will use it.
#[allow(clippy::too_many_arguments)]
pub fn run_log_panel(
    i2c0: I2C0<'static>,
    spi3: SPI3<'static>,
    pins: Pins,
    fanout: &'static LogFanout,
    _nvs: EspNvs<NvsDefault>,
    mode: BootMode,
) -> ! {
    // ── PMIC: AXP192 経由で LCD 電源 + reset。 ────────────────────────
    //   I2C bus は AXP192 / MPU6886 / FT6336U の 3 デバイスで共有。
    //   Phase 2 では PMIC 設定後ハンドルを drop してよい (touch / IMU
    //   は未使用)。
    match crate::pmic::init_lcd_power(i2c0, pins.gpio21, pins.gpio22) {
        Ok(i2c) => {
            // Keep the bus alive until LCD init runs, then drop —
            // Phase 2.5 (touch) will refactor to retain it.
            drop(i2c);
        }
        Err(e) => {
            log::error!("AXP192 init failed: {e:#}");
            // Without PMIC the LCD stays dark forever; keep the log
            // path alive so UDP heartbeat still tells the user the
            // device booted.
            loop {
                log::info!("alive (no PMIC/LCD)");
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        }
    }

    // ── SPI3 (VSPI) host for the LCD. ─────────────────────────────────
    let driver = SpiDriver::new(
        spi3,
        pins.gpio18, // SCK
        pins.gpio23, // MOSI
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new(),
    )
    .expect("SPI3 driver");
    // Conservative 20 MHz on the first bring-up — 40 MHz wedged the
    // first 27 KB waterfall fill on the maiden flash (no panic, just
    // silent loop stop after tick=0). Bump back to 40 MHz once Phase
    // 2 has a known-good baseline.
    let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
    let spi_dev = SpiDeviceDriver::new(driver, Some(pins.gpio5), &spi_cfg)
        .expect("SPI device (CS=5)");

    let dc = PinDriver::output(pins.gpio15).expect("DC gpio15");

    let di = SPIInterface::new(spi_dev, dc);

    // ILI9342C is mostly ILI9341-compatible — use mipidsi's ILI9341
    // model with `ILI9341Rgb565` pixel format. No `reset_pin()` since
    // RST is on AXP192 GPIO4 (already cycled in `pmic::init_lcd_power`)
    // and mipidsi will fall back to SWRESET via the command interface.
    //
    // `display_size` MUST match the controller's framebuffer (240×320,
    // the panel's native portrait), and `orientation` then rotates the
    // embedded-graphics coordinate space to landscape 320×240 — which
    // is the panel's physical landscape orientation on the Core2 case.
    let mut delay = Ets;
    let mut display = match Builder::new(ILI9341Rgb565, di)
        .display_size(240, 320)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .invert_colors(ColorInversion::Inverted) // M5GFX core2 cfg.invert = true
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
        "LCD init OK (ILI9342C via ILI9341 model, {}x{})",
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

    // Paint the TX placeholder strip and a Phase-2 boot banner.
    Rectangle::new(Point::new(0, TX_REGION_Y), Size::new(SHARED_UI_WIDTH, TX_REGION_H))
        .into_styled(PrimitiveStyle::with_fill(tx_bg))
        .draw(&mut display)
        .ok();
    let mode_line: heapless::String<32> = {
        let mut s: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!("Core2 Mode: {}", mode.label()),
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

        // Snapshot UI state under the mutex, then render outside it.
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
            // Skip this render iteration if the UI mutex is poisoned
            // (i.e. another thread panicked while holding it). Matches
            // the non-panicking `if let Ok(mut ui) = ...` pattern used
            // in `decode_pipeline.rs` — keeps the render loop alive
            // through transient cross-thread panics instead of taking
            // the main task down with it. Gemini PR #76 review.
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
