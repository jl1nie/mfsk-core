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
/// Max characters a row can hold, at the widest panel this renders
/// on. Callers pass their own width; this only sizes the buffer.
pub const MAX_ROW_CHARS: usize = (crate::ui::state::WF_COLS as u32 / CHAR_W) as usize;
const ROW_CHARS: usize = MAX_ROW_CHARS;

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
pub fn render<D>(display: &mut D, rows: &[DecodedRow], width: u32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Compatibility shim: fall back to "max slot_seq among visible
    // rows" (legacy semantic). Newer callers pass an explicit
    // `latest_slot_seq` via `render_with_cursor` so rows go back to
    // white when no new decode arrives.
    render_with_cursor(display, rows, None, None, width)
}

/// Phase 1.7.2: render variant that overlays a cursor on the
/// `selected_idx`-th row (0 = newest). User picks a peer this way
/// and BtnA calls `qso::call_station(dx)` on the highlighted row.
///
/// `latest_slot_seq`: pipeline-side watermark of the most-recent
/// slot processed. When provided, rows with `slot_seq == watermark`
/// render GREEN ("current slot"); older rows render WHITE.
/// When `None`, falls back to "max slot_seq among visible rows".
pub fn render_with_cursor<D>(
    display: &mut D,
    rows: &[DecodedRow],
    selected_idx: Option<u8>,
    latest_slot_seq: Option<u32>,
    // Panel width in pixels. The row buffer is sized for the widest
    // panel; this is how much of it actually gets painted.
    width: u32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Operator-requested 2026-05-17 final simplification: black bg
    // always (native panel contrast max), distinguish via text color
    // only.
    //   green text  = appeared in CURRENT slot (recent — regardless
    //                 of whether the same callsign appeared earlier)
    //   white text  = carried over from earlier slot (not heard this
    //                 cycle)
    //   orange text = BtnB cursor selection (callable target)
    let bg = Rgb565::BLACK;
    let fg = Rgb565::WHITE;
    let current_fg = Rgb565::GREEN;
    let cur_fg = Rgb565::CSS_ORANGE;

    let n = rows.len();
    let take = n.min(ROWS);
    let start = n - take;
    // Trailing `take` entries = most recently updated. Reverse so
    // the most-recently-updated lands at the *top* of the region.
    let visible: heapless::Vec<&DecodedRow, ROWS> = rows[start..].iter().rev().collect();

    // GREEN = "appeared in pipeline's CURRENT slot". If caller passes
    // the pipeline watermark, use that — otherwise fall back to the
    // legacy "max slot_seq among visible rows" (which makes every
    // row green when no new decodes arrive).
    let latest_seq =
        latest_slot_seq.unwrap_or_else(|| visible.iter().map(|r| r.slot_seq).max().unwrap_or(0));

    for (i, row) in visible.iter().enumerate() {
        let y = ORIGIN_Y + (i as i32) * ROW_PX as i32;
        let row_y_text = y + 3; // 3 px top padding inside the row band

        // "Current slot" = the row was last detected in the latest
        // visible slot, even if the same callsign appeared in earlier
        // slots too (operator semantics 2026-05-17: focus is "did I
        // hear this station this cycle?" not "first time ever").
        // Cursor takes visual precedence — it's the actionable row.
        let is_current_slot = row.slot_seq == latest_seq;
        let is_selected = selected_idx == Some(i as u8);
        let row_fg = if is_selected {
            cur_fg
        } else if is_current_slot {
            current_fg
        } else {
            fg
        };
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(row_fg)
            .background_color(bg)
            .build();

        // Paint the row band black first so the right-margin gap
        // (after the text) is uniformly clean. Tiny single-row wipe
        // (135 × 16 = 2160 px) — well under the flicker threshold.
        // Background is always black now; distinction is text-color
        // only.
        Rectangle::new(Point::new(0, y), Size::new(width, ROW_PX))
            .into_styled(PrimitiveStyle::with_fill(bg))
            .draw(display)?;

        let mut s: String<32> = String::new();
        let snr = row.snr_db.clamp(-30, 30);
        let _ = write!(&mut s, "{snr:>+3} {:>4} ", row.df_hz);
        let msg_room = ROW_CHARS.saturating_sub(s.len());
        let msg = row.msg.as_str();
        let msg_take = msg.len().min(msg_room.saturating_sub(1));
        let _ = s.push_str(&msg[..msg_take]);
        // Trailing `!` for hard_errors >= 24 dropped 2026-05-17 —
        // SNR column already conveys decode quality (user request).
        Text::with_baseline(s.as_str(), Point::new(0, row_y_text), style, Baseline::Top)
            .draw(display)?;
    }

    // Blank any rows below the last visible decode (when ring isn't
    // full yet, or a slot dropped older entries off the front).
    let drawn = visible.len();
    if drawn < ROWS {
        let blank_y = ORIGIN_Y + (drawn as i32) * ROW_PX as i32;
        let blank_h = ((ROWS - drawn) as u32) * ROW_PX;
        Rectangle::new(Point::new(0, blank_y), Size::new(width, blank_h))
            .into_styled(PrimitiveStyle::with_fill(bg))
            .draw(display)?;
    }

    Ok(())
}
