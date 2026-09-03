// SPDX-License-Identifier: GPL-3.0-or-later
//! The drawing half both spot lists share: the text style, the
//! rectangle fill, and the row loop.
//!
//! `slot_list` (FST4) and `wspr_list` (WSPR) each had their own copy
//! of all three, identical apart from the row type they iterate. The
//! layout constants were identical too — 11 px headers, 10 px column
//! headers, 12 px rows, a 2 px divider, white on black with a grey
//! column header — but written twice, so a change to one screen's
//! metrics silently stopped matching the other's.
//!
//! What stays with each module is what genuinely differs: where the
//! regions sit, what the header line says, and the columns a row is
//! made of.

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

/// Panel geometry, shared because both screens are the same 240x320
/// list with the same font.
pub const PANEL_WIDTH: u32 = 240;
pub const PANEL_HEIGHT: u32 = 320;
pub const STATUS_ORIGIN_Y: i32 = 0;
pub const STATUS_HEIGHT: u32 = 16;
pub const HEADER_H: u32 = 11;
pub const COL_HEADER_H: u32 = 10;
pub const ROW_PX: u32 = 12;
pub const DIVIDER_H: u32 = 2;

pub const BG: Rgb565 = Rgb565::BLACK;
pub const FG: Rgb565 = Rgb565::WHITE;
pub const COL_HEADER_FG: Rgb565 = Rgb565::CSS_GRAY;

/// One row's worth of text, formatted by the mode that owns the row.
///
/// The renderer never looks inside a row; it asks for a line and draws
/// it. `SlotSpotRow` writes `UTC FREQ SNR DT MESSAGE T+`,
/// `WsprSpotRow` writes call/grid/power/drift — same 56-byte budget,
/// because that is what fits at 6 px per character.
pub trait FormatRow {
    fn format_row(&self, out: &mut heapless::String<56>);
}

pub fn text_style(fg: Rgb565, bg: Rgb565) -> MonoTextStyle<'static, Rgb565> {
    MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(fg)
        .background_color(bg)
        .build()
}

/// Fill a full-width band. Both screens clear by rows rather than
/// clearing the panel, so a redraw touches only what changed.
pub fn fill(display: &mut impl DrawTarget<Color = Rgb565>, y: i32, h: u32, color: Rgb565) {
    let _ = Rectangle::new(Point::new(0, y), Size::new(PANEL_WIDTH, h))
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display);
}

/// Draw up to `max_rows` rows from `origin_y`, blanking the rest of
/// the region so a shorter slot does not leave the previous one's
/// rows on screen.
///
/// `divider` is `Some((y, colour))` for the pane that has a rule under
/// it — the discovered/slot pane on both screens.
pub fn render_rows<'a, R, D>(
    display: &mut D,
    rows: impl Iterator<Item = &'a R>,
    origin_y: i32,
    max_rows: usize,
    divider: Option<(i32, Rgb565)>,
) -> Result<(), D::Error>
where
    R: FormatRow + 'a,
    D: DrawTarget<Color = Rgb565>,
{
    let style = text_style(FG, BG);
    let mut buf: heapless::String<56> = heapless::String::new();
    let mut drawn = 0usize;
    for row in rows.take(max_rows) {
        let y = origin_y + (drawn as i32) * ROW_PX as i32;
        fill(display, y, ROW_PX, BG);
        row.format_row(&mut buf);
        Text::with_baseline(buf.as_str(), Point::new(2, y + 1), style, Baseline::Top)
            .draw(display)?;
        drawn += 1;
    }
    if drawn < max_rows {
        let blank_y = origin_y + (drawn as i32) * ROW_PX as i32;
        let blank_h = ((max_rows - drawn) as u32) * ROW_PX;
        fill(display, blank_y, blank_h, BG);
    }
    if let Some((y, colour)) = divider {
        fill(display, y, DIVIDER_H, colour);
    }
    Ok(())
}
