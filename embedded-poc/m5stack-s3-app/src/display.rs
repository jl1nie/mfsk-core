//! LCD bring-up + LcdPanel scroll renderer (Phase 0.5).
//!
//! M5StickS3 内蔵 LCD: ST7789P3 (135x240)。`mipidsi::models::ST7789` の
//! 標準 init で動く想定。SPI ピンは `board.rs` で確定済み。
//!
//! Phase 0.5 段階の方針: display と描画ループを 1 つの関数に閉じ込めて
//! 型パラメータを外部に露出しない。Phase 3 で本来 UI へ進化する際に
//! 構造体化する。

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
    i2c::I2C1,
    i2s::I2S0,
    spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI3},
    units::FromValueType,
};
use mipidsi::{models::ST7789, options::ColorInversion, Builder};

use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::buttons::{Buttons, Event as ButtonEvent, Key};
use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::log_sink::LogFanout;
use mfsk_app_shared::ui::{decoded_list, menu, state::UI, status_bar, waterfall};

/// 1 行高 (FONT_6X10)。
pub const LINE_H: u16 = 10;

/// TX-line placeholder (Phase 4 will fill with QSO FSM state).
const TX_REGION_Y: i32 = 226;
const TX_REGION_H: u32 = 14;

/// LCD bring-up + 永続描画ループ。**戻らない**。
///
/// 個別 peripherals (i2c1 / i2s0 / spi3 / pins) を受け取って所有する。
/// `Peripherals` 全体を取らないのは Phase 0.6 で `peripherals.modem`
/// を WiFi が先に consume するため (partial-move エラー回避)。
///
/// 実装メモ:
/// - SPI clock 40 MHz。歪んだら 26 MHz 等に下げる。
/// - ST7789 panel は内部 RAM 240x320 から表示窓を切り出すので
///   `display_offset` を panel 実装に合わせる。M5StickS3 の 135x240 は
///   実機で (52, 40) または (40, 53) のどちらか。一回 flash して縞 or
///   ずれを観察して調整する。
pub fn run_log_panel(
    i2c1: I2C1<'static>,
    i2s0: I2S0<'static>,
    spi3: SPI3<'static>,
    pins: Pins,
    fanout: &'static LogFanout,
    nvs: EspNvs<NvsDefault>,
    mode: BootMode,
    qso: std::sync::Arc<std::sync::Mutex<mfsk_app_shared::qso::QsoManager>>,
) -> ! {
    // ── PMIC: M5PM1 経由で LCD 電源 ON。これを欠くと SPI/GPIO が完璧でも
    //   panel は永久に黒。M5GFX board_M5StickS3 と同シーケンス。
    let mut i2c = match crate::pmic::init_lcd_power(i2c1, pins.gpio47, pins.gpio48) {
        Ok(d) => Some(d),
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            None
        }
    };

    // Backlight ON (gpio38、PMIC 電源 ON 後に有効化)
    let mut bl = PinDriver::output(pins.gpio38).expect("BL gpio38");
    bl.set_high().ok();
    core::mem::forget(bl);

    // ── ES8311 codec init via the same I2C bus, then drop the I2C
    //    handle (codec keeps its config; we won't touch its registers
    //    again at runtime). I2S setup follows.
    //
    // Mode-dependent audio path:
    //   Decode    → I2S TX (qso3_busy WAV playback to speaker, demo)
    //   Acoustic  → I2S RX (mic capture to decode_pipeline, Phase 1.5)
    //   Wifi      → skip (no audio, DRAM dedicated to WiFi)
    //   Uac       → skip (audio comes from USB host stack, not codec)
    //
    // 板の pins struct は partial-move を許さないので、各 mode 内で
    // 必要な gpio フィールドだけ参照する (audio path は gpio14/15/17/18、
    // LCD path は gpio38/39/40/41/45/21 を後段で使用)。
    match mode {
        BootMode::Decode => {
            if let Some(i2c_drv) = i2c.as_mut() {
                if let Err(e) = crate::audio::init_es8311(i2c_drv) {
                    log::warn!("ES8311 init failed (audio disabled): {e:#}");
                } else {
                    // Build the I2S TX channel. Pin assignment matches
                    // M5Unified's `_speaker_enabled_cb_sticks3` board
                    // config: MCK = 18, BCK = 17, WS = 15, DOUT = 14.
                    // 48 kHz stereo so the qso3 mono 12 kHz source
                    // upsamples by 4× (zero-order hold). ES8311 in
                    // MCLK=BCLK mode (reg 0x01=0xB5) follows whatever
                    // rate the I2S master generates.
                    use esp_idf_hal::i2s::{
                        config::{
                            ClockSource, Config as I2sConfig, DataBitWidth, MclkMultiple,
                            SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
                        },
                        I2sDriver,
                    };
                    let i2s_cfg = StdConfig::new(
                        I2sConfig::default(),
                        StdClkConfig::new(48_000, ClockSource::Pll160M, MclkMultiple::M256),
                        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
                        StdGpioConfig::default(),
                    );
                    match I2sDriver::new_std_tx(
                        i2s0,
                        &i2s_cfg,
                        pins.gpio17,         // BCLK
                        pins.gpio14,         // DOUT (S3 → codec)
                        Some(pins.gpio18),   // MCLK
                        pins.gpio15,         // WS / LRCK
                    ) {
                        Ok(i2s) => {
                            if let Err(e) = crate::audio::pa_enable(i2c_drv) {
                                log::warn!("PA enable failed: {e:#}");
                            }
                            static QSO3: &[u8] = include_bytes!("../../assets/qso3_busy.wav");
                            std::thread::Builder::new()
                                .stack_size(8 * 1024)
                                .spawn(move || crate::audio::audio_thread(i2s, QSO3))
                                .expect("spawn audio thread");
                        }
                        Err(e) => log::warn!("I2S TX init failed: {e:?}"),
                    }
                }
            }
        }
        BootMode::Acoustic => {
            // Phase 1.5-Stick: ES8311 ADC mode + I2S RX。Decode path と
            // 同じ I2S clock 設定 (48 kHz stereo / MCLK=BCLK mode) を
            // 使う — codec 側 init 差分を最小化、capture thread 側で L
            // channel 抽出 + 48k→12k linear resample。Speaker PA は
            // 起こさない (mic 単独でハウリング回避)。
            if let Some(i2c_drv) = i2c.as_mut() {
                // init_es8311_capture does its own RESET + power-on
                // (M5Unified canonical mic sequence); do NOT call
                // init_es8311 first or DAC bits will fight the ADC path.
                if let Err(e) = crate::audio::init_es8311_capture(i2c_drv) {
                    log::warn!("ES8311 mic-mode init failed (capture disabled): {e:#}");
                } else {
                    use esp_idf_hal::i2s::{
                        config::{
                            ClockSource, Config as I2sConfig, DataBitWidth, MclkMultiple,
                            SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
                        },
                        I2sDriver,
                    };
                    let i2s_cfg = StdConfig::new(
                        I2sConfig::default(),
                        StdClkConfig::new(48_000, ClockSource::Pll160M, MclkMultiple::M256),
                        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
                        StdGpioConfig::default(),
                    );
                    // Codec DOUT → S3 DIN: M5Unified canonical wiring
                    // is GPIO 16 (= board.rs ES8311_DIN constant; the
                    // M5StickS3 official docs page lists DOUT = GPIO 14
                    // / DIN = GPIO 16 from codec perspective, so for
                    // S3 RX we pick up codec DOUT which connects to S3
                    // via GPIO 16).
                    match I2sDriver::new_std_rx(
                        i2s0,
                        &i2s_cfg,
                        pins.gpio17,         // BCLK
                        pins.gpio16,         // DIN (codec → S3)
                        Some(pins.gpio18),   // MCLK
                        pins.gpio15,         // WS / LRCK
                    ) {
                        Ok(i2s) => {
                            if let Err(e) = std::thread::Builder::new()
                                .stack_size(6 * 1024)
                                .name("audio_cap".into())
                                .spawn(move || crate::audio::capture_thread(i2s))
                            {
                                log::error!("spawn capture thread failed: {e} — audio path dead");
                            }
                        }
                        Err(e) => log::warn!("I2S RX init failed: {e:?}"),
                    }
                }
            }
        }
        BootMode::Wifi => {
            log::info!("audio path skipped (BootMode::Wifi: speaker muted, mic idle)");
        }
        BootMode::CivTest => {
            log::info!("audio path skipped (BootMode::CivTest: BLE-only bring-up)");
        }
        BootMode::Qso => {
            // Phase 1.7.1-Stick: I2S full-duplex on i2s0. ES8311 ADC +
            // DAC simultaneously enabled (init_es8311_capture sets up
            // the mic path; the speaker path piggybacks the same MCLK
            // clock chain since codec is in MCLK=BCLK mode). The
            // bidirectional driver lets a single thread (capture_tx_thread)
            // alternate RX read iterations and TX synth play — no need
            // for i2s1 or driver swap.
            if let Some(i2c_drv) = i2c.as_mut() {
                if let Err(e) = crate::audio::init_es8311_capture(i2c_drv) {
                    log::warn!("ES8311 mic-mode init failed (Qso disabled): {e:#}");
                } else {
                    // Speaker amp enable so the TX synth side actually
                    // drives the audio out the speaker (mic input via
                    // ADC stays unaffected).
                    if let Err(e) = crate::audio::pa_enable(i2c_drv) {
                        log::warn!("PA enable failed: {e:#}");
                    }
                    use esp_idf_hal::i2s::{
                        config::{
                            ClockSource, Config as I2sConfig, DataBitWidth, MclkMultiple,
                            SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
                        },
                        I2sDriver,
                    };
                    let i2s_cfg = StdConfig::new(
                        I2sConfig::default(),
                        StdClkConfig::new(48_000, ClockSource::Pll160M, MclkMultiple::M256),
                        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
                        StdGpioConfig::default(),
                    );
                    // bidir constructor: (port, cfg, bclk, din, dout, mclk, ws)
                    // — din = codec→S3 (GPIO 16), dout = S3→codec (GPIO 14).
                    match I2sDriver::new_std_bidir(
                        i2s0,
                        &i2s_cfg,
                        pins.gpio17,
                        pins.gpio16,
                        pins.gpio14,
                        Some(pins.gpio18),
                        pins.gpio15,
                    ) {
                        Ok(i2s) => {
                            let qso_clone = qso.clone();
                            if let Err(e) = std::thread::Builder::new()
                                .stack_size(8 * 1024)
                                .name("audio_capt_tx".into())
                                .spawn(move || crate::audio::capture_tx_thread(i2s, qso_clone))
                            {
                                log::error!("Qso capture+tx spawn failed: {e}");
                            }
                        }
                        Err(e) => log::warn!("Qso I2S bidir init failed: {e:?}"),
                    }
                }
            }
        }
        BootMode::TxTest => {
            // Phase 1.6-Stick: ES8311 DAC + I2S TX → tx::start_test_loop。
            // Decode mode と同じ I2S TX 設定 (48 kHz stereo, codec
            // MCLK=BCLK mode, GPIO 17/14/18/15) を再利用、出力先だけ
            // qso3_busy WAV から FT8 synth tones に切り替え。
            if let Some(i2c_drv) = i2c.as_mut() {
                if let Err(e) = crate::audio::init_es8311(i2c_drv) {
                    log::warn!("ES8311 init failed (TX disabled): {e:#}");
                } else {
                    use esp_idf_hal::i2s::{
                        config::{
                            ClockSource, Config as I2sConfig, DataBitWidth, MclkMultiple,
                            SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
                        },
                        I2sDriver,
                    };
                    let i2s_cfg = StdConfig::new(
                        I2sConfig::default(),
                        StdClkConfig::new(48_000, ClockSource::Pll160M, MclkMultiple::M256),
                        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
                        StdGpioConfig::default(),
                    );
                    match I2sDriver::new_std_tx(
                        i2s0,
                        &i2s_cfg,
                        pins.gpio17,         // BCLK
                        pins.gpio14,         // DOUT (S3 → codec)
                        Some(pins.gpio18),   // MCLK
                        pins.gpio15,         // WS / LRCK
                    ) {
                        Ok(i2s) => {
                            // PA enable so the synthesised tones hit
                            // a powered amp — same as the WAV demo.
                            if let Err(e) = crate::audio::pa_enable(i2c_drv) {
                                log::warn!("PA enable failed: {e:#}");
                            }
                            // Phase 1.7-Stick: tx_scheduler replaces the
                            // standalone test loop. It owns the I2S TX
                            // driver and pulls TxIntent from the shared
                            // QSO FSM each slot. With no RX in TxTest
                            // mode the FSM only sees auto-CQ; replies
                            // come once we move to BootMode::Qso with
                            // I2S full-duplex (Phase 1.7.1).
                            let qso_clone = qso.clone();
                            if let Err(e) = crate::tx_scheduler::spawn(i2s, qso_clone) {
                                log::error!("tx_scheduler spawn failed: {e:#}");
                            }
                        }
                        Err(e) => log::warn!("I2S TX init failed: {e:?}"),
                    }
                }
            }
        }
        BootMode::Uac => {
            log::info!("audio path skipped (BootMode::Uac: audio via USB host stack)");
        }
    }
    drop(i2c);

    // ── SPI3 host (M5GFX が SPI3_HOST を使用)。SPI2 ではない。
    let driver = SpiDriver::new(
        spi3,
        pins.gpio40, // SCK
        pins.gpio39, // MOSI
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new(),
    )
    .expect("SPI3 driver");
    let spi_cfg = SpiConfig::new().baudrate(40_u32.MHz().into());
    let spi_dev = SpiDeviceDriver::new(driver, Some(pins.gpio41), &spi_cfg)
        .expect("SPI device (CS=41)");

    let dc = PinDriver::output(pins.gpio45).expect("DC gpio45");
    let rst = PinDriver::output(pins.gpio21).expect("RST gpio21");

    let di = SPIInterface::new(spi_dev, dc);

    let mut delay = Ets;
    let mut display = match Builder::new(ST7789, di)
        .reset_pin(rst)
        .display_size(crate::board::LCD_WIDTH, crate::board::LCD_HEIGHT)
        .display_offset(52, 40)
        .invert_colors(ColorInversion::Inverted) // M5GFX cfg.invert = true
        .init(&mut delay)
    {
        Ok(d) => d,
        Err(e) => {
            log::error!("display init failed: {:?}", e);
            // LCD 不在でもログだけ吐き続けるループへ。
            loop {
                log::info!("alive (no LCD)");
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        }
    };

    log::info!("LCD init OK (ST7789 135x240 offset 52,40 invert)");
    display.clear(Rgb565::BLACK).ok();

    let tx_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::new(0, 0, 8)) // dim blue strip
        .build();

    // Paint the TX placeholder strip once (refreshed per redraw cycle).
    let tx_bg = Rgb565::new(0, 0, 8);

    // Paint the TX strip once at boot with a placeholder. The decode
    // pipeline thread later repaints this region whenever the QSO FSM
    // updates `UiState::tx_line` (tx_seq bump triggers redraw below).
    Rectangle::new(
        Point::new(0, TX_REGION_Y),
        Size::new(crate::board::LCD_WIDTH as u32, TX_REGION_H),
    )
    .into_styled(PrimitiveStyle::with_fill(tx_bg))
    .draw(&mut display)
    .ok();
    // Mode label paints in the TX strip at boot. Decode mode's QSO FSM
    // will overwrite this within the first slot; WiFi mode keeps it
    // (no FSM). KEY2 long-press (2 s) flips and restarts.
    let mode_line: heapless::String<32> = {
        let mut s: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!("Mode: {} hold K2→flip", mode.label()),
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
    // Separate dirty fingerprints for the two regions. WF advances
    // every ~80 ms (per-pair WfTick) while decoded-list advances
    // once per slot (~15 s); sharing one dirty_seq would cause the
    // decoded list to re-render at WF cadence and flicker visibly.
    //
    // **WF fingerprint = push count, NOT ring length** — the deque
    // saturates at `WF_DEPTH = 100` after ~8 s, so `len()` plateaus
    // and a `len()`-based check would freeze the WF redraw exactly
    // when the user expects continuous flow.
    let mut last_wf_seq: u32 = u32::MAX;
    let mut last_decoded_fp: (usize, u32) = (usize::MAX, u32::MAX);
    let mut last_tx_seq: u32 = 0;
    let mut last_menu_seq: u32 = u32::MAX;
    // Phase 0.7c: poll KEY1/KEY2 here on the 100 ms cadence. Long-press
    // KEY2 (= 2 s) flips boot mode and restarts; KEY1 is reserved for
    // the boot-time invert override (already consumed at boot).
    let mut buttons = Buttons::init();
    loop {
        buttons.poll();
        if let Some(ev) = buttons.take_event() {
            // Phase 1.7-Stick: route to menu when visible; otherwise
            // KEY2 long-press still does boot-mode flip (legacy path
            // for diagnostic mode switching). BtnA/BtnB short-presses
            // outside the menu remain no-ops for now.
            let menu_visible_now = UI
                .lock()
                .ok()
                .map(|u| u.menu_visible)
                .unwrap_or(false);
            match (ev, menu_visible_now) {
                (ButtonEvent::LongPress(Key::Key2), _) => {
                    if menu_visible_now {
                        // Close menu (don't flip boot mode while menu open).
                        if let Ok(mut ui) = UI.lock() {
                            menu::toggle_visible(&mut ui);
                        }
                        log::info!("menu: closed via BtnB long-press");
                    } else if mode == BootMode::Qso || mode == BootMode::Acoustic {
                        // Open menu only in modes that have RX + control
                        // surface to act on. Other modes keep the legacy
                        // boot-mode flip semantic so the cycle is still
                        // navigable via KEY2 long-press.
                        if let Ok(mut ui) = UI.lock() {
                            menu::toggle_visible(&mut ui);
                        }
                        log::info!("menu: opened via BtnB long-press");
                    } else {
                        log::info!("KEY2 long-press → boot_mode flip");
                        boot_mode::flip_and_restart(&nvs, mode);
                    }
                }
                (ButtonEvent::ShortPress(Key::Key2), true) => {
                    if let Ok(mut ui) = UI.lock() {
                        menu::next_item(&mut ui);
                    }
                }
                (ButtonEvent::ShortPress(Key::Key1), true) => {
                    // Activate selected item. The closures below are
                    // intentionally tiny — they bridge between the
                    // board-agnostic menu module and the s3-app crate's
                    // civ / tx_picker code.
                    let mut actions = menu::MenuActions {
                        set_band: &mut |idx: u8| {
                            let (hz, label) = menu::BANDS[idx as usize];
                            crate::civ::set_freq_hz(hz);
                            crate::civ::set_mode_data_usb();
                            log::info!("menu: band → {label} ({hz} Hz)");
                        },
                        find_df: &mut || {
                            // Phase 1.7 first cut: returns the 1500 Hz
                            // centre default until the occupancy map is
                            // populated from the live spectrogram. The
                            // TX scheduler will recompute this anyway
                            // right before each TX cycle.
                            //
                            // TODO Phase 1.7.1: wire occupancy ingest
                            // from decode_pipeline so this hook returns
                            // the actually-quietest 50 Hz window.
                            let df = 1500u16;
                            log::info!("menu: find_df → {df} Hz (occupancy not yet wired)");
                            df
                        },
                        set_auto_cq: &mut |on: bool| {
                            if let Ok(mut q) = qso.lock() {
                                q.set_auto_cq(on);
                            }
                            log::info!("menu: auto_cq → {}", if on { "ON" } else { "OFF" });
                        },
                    };
                    if let Ok(mut ui) = UI.lock() {
                        menu::activate(&mut ui, &mut actions);
                    }
                }
                (ButtonEvent::LongPress(Key::Key1), _) => {
                    log::info!("KEY1 long-press (no action wired yet)");
                }
                (ButtonEvent::ShortPress(k), false) => {
                    log::info!("button short-press: {k:?} (menu hidden, ignored)");
                }
            }
        }
        let heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
        // 100 ms ループ毎に log すると 10 行/秒 = UDP/LCD が文字で溢れる。
        // 5 秒に 1 行 (= 50 ticks) に間引く。Phase 0.7a: 内部 DRAM 残量も
        // 出す。BASIS heap-alloc 化が効いているか確認するため (decode mode
        // で BASIS alloc 前後の差分 ~120 KB を runtime 値で読みたい)。
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

        // ── status bar + heap update from UI state. We refresh the
        //    status struct here so the UTC / heap stay current even
        //    when no new decodes land.
        let status_snapshot;
        let decoded_snapshot;
        let wf_snapshot: heapless::Vec<mfsk_app_shared::ui::state::WfLine, { mfsk_app_shared::ui::state::WF_DEPTH }>;
        let decoded_fp;
        let wf_seq;
        let tx_seq;
        let tx_line_snapshot: heapless::String<48>;
        // (visible, selected, band_idx, df_hz, auto_cq, seq)
        let menu_snapshot: (bool, u8, u8, u16, bool, u32);
        {
            let mut ui = UI.lock().expect("UI mutex poisoned");
            ui.status.free_heap_kb = (heap / 1024) as u32;
            status_snapshot = ui.status.clone();
            decoded_snapshot = ui
                .decoded_iter()
                .cloned()
                .collect::<heapless::Vec<_, 16>>();
            wf_snapshot = ui.waterfall_iter().cloned().collect();
            wf_seq = ui.wf_push_seq();
            tx_seq = ui.tx_seq();
            menu_snapshot = (
                ui.menu_visible,
                ui.menu_selected,
                ui.current_band_idx,
                ui.current_df_hz,
                ui.auto_cq_enabled,
                ui.menu_seq(),
            );
            let mut buf: heapless::String<48> = heapless::String::new();
            for ch in ui.tx_line().chars() {
                if buf.push(ch).is_err() {
                    break;
                }
            }
            tx_line_snapshot = buf;
            // Fingerprint the decoded ring by (count, max_slot_seq) so
            // we re-render only when a slot completes — not on every
            // per-pair WF tick that bumped the global dirty_seq.
            let max_seq = decoded_snapshot
                .iter()
                .map(|r| r.slot_seq)
                .max()
                .unwrap_or(0);
            decoded_fp = (decoded_snapshot.len(), max_seq);
        }

        // Status bar refreshes every loop tick (cheap; no leading
        // wipe in the renderer so it doesn't flicker).
        status_bar::render(&mut display, &status_snapshot).ok();

        // Menu overlay — render after WF/decoded so it paints on top.
        // Tracking last menu_seq to avoid redraw when nothing changed.
        let menu_seq = menu_snapshot.5;
        if menu_seq != last_menu_seq {
            menu::render(
                &mut display,
                menu_snapshot.0,
                menu_snapshot.1,
                menu_snapshot.2,
                menu_snapshot.3,
                menu_snapshot.4,
            )
            .ok();
            // When menu closes, force a full re-render of WF + decoded
            // next iteration so the overlay's residual pixels go away.
            if !menu_snapshot.0 {
                last_wf_seq = u32::MAX;
                last_decoded_fp = (usize::MAX, u32::MAX);
            }
            last_menu_seq = menu_seq;
        }

        // Waterfall: streams at per-pair cadence. Trigger on
        // `wf_push_seq` so the redraw still fires after the ring
        // saturates at WF_DEPTH (= ~8 s into runtime).
        if wf_seq != last_wf_seq {
            let wf_refs: heapless::Vec<&mfsk_app_shared::ui::state::WfLine, { mfsk_app_shared::ui::state::WF_DEPTH }> =
                wf_snapshot.iter().collect();
            waterfall::render(&mut display, &wf_refs).ok();
            last_wf_seq = wf_seq;
        }

        // Decoded list: redraw only when a new slot's results landed.
        if decoded_fp != last_decoded_fp {
            decoded_list::render(&mut display, &decoded_snapshot).ok();
            last_decoded_fp = decoded_fp;
        }

        // TX line: repaint when the QSO FSM bumps `tx_seq`. Cheap
        // (single 14-px-tall strip) so we always do a fresh wipe + draw.
        if tx_seq != last_tx_seq {
            Rectangle::new(
                Point::new(0, TX_REGION_Y),
                Size::new(crate::board::LCD_WIDTH as u32, TX_REGION_H),
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

        // Phase 0.5 boot-time log scroll has been retired now that
        // the WF region is live. UART log path stays via FanoutLogger.
        let _ = fanout;

        std::thread::sleep(std::time::Duration::from_millis(100));
        tick = tick.wrapping_add(1);
    }
}
