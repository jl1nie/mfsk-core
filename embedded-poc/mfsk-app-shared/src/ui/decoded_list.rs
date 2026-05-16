//! Decoded message list — 7 rows × 16 px = 112 px tall, drawn at
//! y ∈ [114, 226). Newest decode at the bottom (= row 6); older rows
//! scroll up as new decodes land.
//!
//! Each row is `"-NN ffff  message ..."` at FONT_6X10 (22 chars max,
//! fits 135 px width with 3 px slack). Borderline decodes
//! (`hard_errors ≥ 24`) get a trailing `!` plus amber tint so users
//! can spot CRC-luck candidates without reading the error column.
//! Cursor / "Call this station" wiring is deferred to Phase 6 (button
//! integration); for Phase 3 this is read-only.

use core::fmt::Write as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

use crate::ui::state::DecodedRow;

/// Region geometry on the 135 × 240 panel.
pub const ORIGIN_Y: i32 = 114;
pub const HEIGHT: u32 = 112;
pub const ROW_PX: u32 = 16;
/// Visible rows in the region (HEIGHT / ROW_PX).
pub const ROWS: usize = (HEIGHT / ROW_PX) as usize;
/// FONT_6X10 char width.
pub const CHAR_W: u32 = 6;
/// Max characters per row at 135 px width.
const ROW_CHARS: usize = (135 / CHAR_W) as usize;

/// Clear + repaint the decoded list. Idempotent — caller gates by
/// `UiState::dirty_seq`. Rows whose **first** observation lands in
/// the latest visible slot (= genuinely new callsigns this slot)
/// get inverse video (dim cyan bg); recurring callsigns from a
/// `wav_sim` loop draw plain white-on-black, since their `first_seq`
/// stays at the slot they were originally seen.
///
/// Render direction: top of region = **newest** decode (matches
/// WSJT-X / JTDX UX). Older rows scroll downward as new ones arrive.
///
/// Per-row painting only covers the row's vertical band (16 px ×
/// 135 = 2 160 px), and each glyph cell carries `background_color`
/// so no separate wipe is needed — eliminates the 100 ms-cadence
/// flicker the full-region clear was causing.
pub fn render<D>(display: &mut D, rows: &[DecodedRow]) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let bg = Rgb565::BLACK;
    let fg = Rgb565::WHITE;
    let warn = Rgb565::CSS_ORANGE;
    // First-seen-this-slot highlight: dim cyan band.
    let new_bg = Rgb565::new(2, 14, 12);
    let new_fg = Rgb565::WHITE;

    let n = rows.len();
    let take = n.min(ROWS);
    let start = n - take;
    // Trailing `take` entries = most recently updated. Reverse so
    // the most-recently-updated lands at the *top* of the region.
    let visible: heapless::Vec<&DecodedRow, ROWS> = rows[start..].iter().rev().collect();

    // Highlight rows whose first observation is the latest slot.
    let latest_seq = visible.iter().map(|r| r.slot_seq).max().unwrap_or(0);

    for (i, row) in visible.iter().enumerate() {
        let y = ORIGIN_Y + (i as i32) * ROW_PX as i32;
        let row_y_text = y + 3; // 3 px top padding inside the row band

        // "New" = the row's *first* observation is in the current
        // visible slot. Recurring callsigns (re-decoded each slot of
        // a `wav_sim` loop) carry an older `first_seq` and stay plain.
        let is_new = row.first_seq == latest_seq;
        let row_bg = if is_new { new_bg } else { bg };
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(if is_new {
                new_fg
            } else if row.hard_errors >= 24 {
                warn
            } else {
                fg
            })
            .background_color(row_bg)
            .build();

        // Paint the row band first so the right-margin gap (after the
        // text) gets the highlight too. Tiny single-row wipe (135 ×
        // 16 = 2160 px) — well under the flicker threshold.
        Rectangle::new(Point::new(0, y), Size::new(135, ROW_PX))
            .into_styled(PrimitiveStyle::with_fill(row_bg))
            .draw(display)?;

        let mut s: String<32> = String::new();
        let snr = row.snr_db.clamp(-30, 30);
        let _ = write!(&mut s, "{snr:>+3} {:>4} ", row.df_hz);
        let msg_room = ROW_CHARS.saturating_sub(s.len());
        let msg = row.msg.as_str();
        let msg_take = msg.len().min(msg_room.saturating_sub(1));
        let _ = s.push_str(&msg[..msg_take]);
        if row.hard_errors >= 24 {
            let _ = s.push('!');
        }
        Text::with_baseline(s.as_str(), Point::new(0, row_y_text), style, Baseline::Top)
            .draw(display)?;
    }

    // Blank any rows below the last visible decode (when ring isn't
    // full yet, or a slot dropped older entries off the front).
    let drawn = visible.len();
    if drawn < ROWS {
        let blank_y = ORIGIN_Y + (drawn as i32) * ROW_PX as i32;
        let blank_h = ((ROWS - drawn) as u32) * ROW_PX;
        Rectangle::new(Point::new(0, blank_y), Size::new(135, blank_h))
            .into_styled(PrimitiveStyle::with_fill(bg))
            .draw(display)?;
    }

    Ok(())
}
