//! Modal menu overlay (Phase 1.7-Stick, 2026-05-17).
//!
//! Three items, 2-button navigation, rendered as a 100×54 overlay
//! box in the LCD's top-right corner (the WF region behind keeps
//! drawing so the user can still see band activity while choosing
//! a DF).
//!
//! - BtnB long-press (2 s) → toggle visibility (caller-handled in
//!   display.rs button switch)
//! - BtnB short → advance `selected` (mod MENU_ITEM_COUNT)
//! - BtnA short → activate the currently-selected item:
//!     Band:    cycle to next preset, push freq to CI-V via setter
//!     FindDf:  recompute auto-DF immediately (TX scheduler also
//!              calls this just before each TX cycle)
//!     AutoCq:  toggle the `auto_cq_enabled` gate on the QSO FSM
//!
//! The menu has no "exit" item — BtnB long again closes it.
//!
//! Cross-thread coupling: the menu reads/writes `UiState` fields
//! under the global `UI` mutex. It does NOT take the QSO FSM lock
//! directly — the activate callback is a closure provided by the
//! display loop so the menu module stays board-agnostic (no
//! `mfsk_core_m5stack_s3_app::civ` dependency).
//!
//! Frequencies are user-confirmed standard FT8 channels (per
//! https://www.physics.princeton.edu/pulsar/k1jt/FT8_Operating_Tips.pdf).

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

/// FT8 band preset table. (freq_hz, label).
/// Index 4 (20m) is the default for fresh boots — populated into
/// `UiState::current_band_idx`'s `::new()` initializer.
pub const BANDS: &[(u32, &str); 11] = &[
    (3_573_000, "80m"),
    (5_357_000, "60m"),
    (7_074_000, "40m"),
    (10_136_000, "30m"),
    (14_074_000, "20m"),
    (18_100_000, "17m"),
    (21_074_000, "15m"),
    (24_915_000, "12m"),
    (28_074_000, "10m"),
    (50_313_000, "6m"),
    (144_174_000, "2m"),
];

/// Menu items in display order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuItem {
    /// Cycle 80m → 60m → 40m → ... → 2m → 80m
    Band,
    /// Trigger auto-DF recompute now (TX scheduler also calls this
    /// per-QSO; this menu item is for the user to preview the chosen
    /// DF before flipping CQ on).
    FindDf,
    /// Toggle auto-CQ master gate on the QSO FSM.
    AutoCq,
    /// Persist the next boot mode to NVS and restart. Lets the user
    /// switch from Acoustic ↔ Qso ↔ … without the 3-reboot KEY dance.
    NextMode,
}

pub const MENU_ITEMS: &[MenuItem] = &[
    MenuItem::Band,
    MenuItem::FindDf,
    MenuItem::AutoCq,
    MenuItem::NextMode,
];
pub const MENU_ITEM_COUNT: u8 = 4;

/// Compact representation of a single menu entry's display row.
fn item_label(item: MenuItem) -> &'static str {
    match item {
        MenuItem::Band => "Band:",
        MenuItem::FindDf => "FindDF:",
        MenuItem::AutoCq => "AutoCQ:",
        MenuItem::NextMode => "Mode→:",
    }
}

/// Action hooks the display loop fills in. Keeps `ui/menu.rs`
/// board-agnostic (no civ / tx_picker / qso imports here).
pub struct MenuActions<'a> {
    /// Called when the user activates the Band item — pass the new
    /// band index. Implementation should `civ::set_freq_hz(BANDS[idx].0)`
    /// and store the index in NVS.
    pub set_band: &'a mut dyn FnMut(u8),
    /// Called when the user activates FindDF — recompute and return
    /// the chosen DF in Hz (200..2700). Implementation typically calls
    /// `tx_picker::find_clear_df`.
    pub find_df: &'a mut dyn FnMut() -> u16,
    /// Called when the user activates AutoCQ — pass the new state
    /// (the menu already toggled `auto_cq_enabled` in UiState; this
    /// callback also propagates to the QSO FSM and NVS).
    pub set_auto_cq: &'a mut dyn FnMut(bool),
    /// Called when the user activates NextMode. Implementation should
    /// persist the next boot mode to NVS and restart (via
    /// `boot_mode::flip_and_restart`). The callback returns `!` so
    /// the device reboots before the menu can repaint.
    pub next_mode: &'a mut dyn FnMut(),
}

/// Advance the highlighted item. Caller already holds the UI mutex.
pub fn next_item(ui: &mut super::state::UiState) {
    ui.menu_selected = (ui.menu_selected + 1) % MENU_ITEM_COUNT;
    ui.bump_menu();
}

/// Toggle visibility. Caller already holds the UI mutex.
pub fn toggle_visible(ui: &mut super::state::UiState) {
    ui.menu_visible = !ui.menu_visible;
    ui.bump_menu();
}

/// Activate the highlighted item. Caller holds the UI mutex; the
/// `actions` closure may call into civ / tx_picker / qso without
/// taking the UI mutex (the closure execution is wrapped by us).
pub fn activate(ui: &mut super::state::UiState, actions: &mut MenuActions<'_>) {
    let item = MENU_ITEMS[ui.menu_selected as usize];
    match item {
        MenuItem::Band => {
            let next = (ui.current_band_idx + 1) % (BANDS.len() as u8);
            ui.current_band_idx = next;
            (actions.set_band)(next);
        }
        MenuItem::FindDf => {
            let df = (actions.find_df)();
            ui.current_df_hz = df;
        }
        MenuItem::AutoCq => {
            ui.auto_cq_enabled = !ui.auto_cq_enabled;
            (actions.set_auto_cq)(ui.auto_cq_enabled);
        }
        MenuItem::NextMode => {
            // Persist next boot mode + restart. The callback never
            // returns (`!`), so bump_menu below is unreachable — the
            // device reboots before the menu repaints.
            (actions.next_mode)();
            // Fallback (unreachable in practice): just drop it.
        }
    }
    ui.bump_menu();
}

// ── Rendering ────────────────────────────────────────────────────────

/// Render the menu overlay on top of whatever the display has drawn
/// so far. Caller passes a snapshot of UiState fields (decoupled
/// from the mutex lock so the render isn't serialised against
/// pipeline writes). No-op when `visible == false`.
///
/// Layout (LCD 135×240, menu top-right of WF region):
/// - origin (32, 16), size 100×68 px
/// - background fill: dark blue (Rgb565::new(0,0,8))
/// - border: white 1-px
/// - 4 rows × 14-px tall, cursor `>` in column 0 of selected row
///
/// `next_mode_label` is the label of the boot mode the device would
/// switch to if the NextMode item is activated (e.g. `"QSO"`).
pub fn render<D, E>(
    display: &mut D,
    visible: bool,
    selected: u8,
    band_idx: u8,
    df_hz: u16,
    auto_cq: bool,
    next_mode_label: &str,
) -> Result<(), E>
where
    D: DrawTarget<Color = Rgb565, Error = E>,
{
    if !visible {
        return Ok(());
    }
    const ORIGIN: Point = Point::new(32, 16);
    const SIZE: Size = Size::new(100, 68);
    const ROW_H: i32 = 14;

    // Background + border.
    let bg = Rgb565::new(0, 0, 8);
    let border_style = PrimitiveStyle::with_fill(bg);
    Rectangle::new(ORIGIN, SIZE)
        .into_styled(border_style)
        .draw(display)?;
    let border = PrimitiveStyle::with_stroke(Rgb565::WHITE, 1);
    Rectangle::new(ORIGIN, SIZE)
        .into_styled(border)
        .draw(display)?;

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(bg)
        .build();

    // Values as fixed-cap strings (avoid std::format heap).
    let band_label = BANDS[band_idx as usize].1;
    let mut band_line: heapless::String<16> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(&mut band_line, format_args!(" Band: {band_label}"));
    let mut df_line: heapless::String<16> = heapless::String::new();
    if df_hz == 0 {
        let _ = df_line.push_str(" FindDF: ---");
    } else {
        let _ = core::fmt::Write::write_fmt(&mut df_line, format_args!(" FindDF:{df_hz}"));
    }
    let mut autocq_line: heapless::String<16> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(
        &mut autocq_line,
        format_args!(" AutoCQ: {}", if auto_cq { "ON" } else { "OFF" }),
    );
    let mut mode_line: heapless::String<16> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(&mut mode_line, format_args!(" Mode→:{next_mode_label}"));

    let rows: [&str; 4] = [
        band_line.as_str(),
        df_line.as_str(),
        autocq_line.as_str(),
        mode_line.as_str(),
    ];

    for (i, text) in rows.iter().enumerate() {
        let y = ORIGIN.y + 2 + (i as i32) * ROW_H;
        // Cursor column 0 of selected row.
        let cursor_ch = if selected as usize == i { ">" } else { " " };
        Text::with_baseline(
            cursor_ch,
            Point::new(ORIGIN.x + 2, y),
            text_style,
            Baseline::Top,
        )
        .draw(display)?;
        Text::with_baseline(text, Point::new(ORIGIN.x + 8, y), text_style, Baseline::Top)
            .draw(display)?;
        // Suppress unused-binding warnings on minimal builds.
        let _ = item_label(MENU_ITEMS[i]);
    }
    Ok(())
}
