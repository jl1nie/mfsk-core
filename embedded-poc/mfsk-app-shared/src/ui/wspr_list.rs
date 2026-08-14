//! WSPR receiver screen — fixed vertical split for the CoreS3's full
//! 320×240 landscape panel.
//!
//! **Not a reuse of `decoded_list`/`status_bar`**: those hardcode the
//! M5StickS3's 135 px width (see their own doc comments) and this
//! app runs on CoreS3's real 320×240 canvas, wide enough that a
//! WSPR row's full field set (`wspr_row::HEADER`'s columns) fits on
//! one line without the truncation logic the 135 px-wide FT8 list
//! needs. Reused *structurally* instead: per-region self-contained
//! wipe-then-draw (no separate full-frame clear, to avoid the same
//! flicker `decoded_list`'s own doc comment describes), caller gates
//! repaints by `WsprUiState::dirty_seq`.
//!
//! No on-device input drives a screen switch (CoreS3 has none — see
//! this crate's design memory), so the layout is one fixed vertical
//! split rather than tabs:
//!
//! ```text
//! y=0..16    status bar       (band / dial freq / UTC / NTP / net / heap)
//! y=16..38   discovered header + column header
//! y=38..110  discovered stations (6 rows, latest-slot wholesale replace)
//! y=110..112 divider
//! y=112..134 history header + column header
//! y=134..230 spot history (8 rows, append-only, newest at top)
//! y=230..240 unused margin
//! ```

use core::fmt::Write as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

use super::wspr_row::{WsprSpotRow, HEADER as ROW_HEADER};
use super::wspr_state::WsprUiState;

pub const PANEL_WIDTH: u32 = 320;
pub const PANEL_HEIGHT: u32 = 240;

pub const STATUS_ORIGIN_Y: i32 = 0;
pub const STATUS_HEIGHT: u32 = 16;

const DISCOVERED_HEADER_Y: i32 = STATUS_HEIGHT as i32;
const HEADER_H: u32 = 12;
const COL_HEADER_H: u32 = 10;
const ROW_PX: u32 = 12;

const DISCOVERED_COL_HEADER_Y: i32 = DISCOVERED_HEADER_Y + HEADER_H as i32;
pub const DISCOVERED_ROWS_Y: i32 = DISCOVERED_COL_HEADER_Y + COL_HEADER_H as i32;
pub const DISCOVERED_ROWS: usize = 6;
const DISCOVERED_REGION_H: u32 = DISCOVERED_ROWS as u32 * ROW_PX;

const DIVIDER_Y: i32 = DISCOVERED_ROWS_Y + DISCOVERED_REGION_H as i32;
const DIVIDER_H: u32 = 2;

const HISTORY_HEADER_Y: i32 = DIVIDER_Y + DIVIDER_H as i32;
const HISTORY_COL_HEADER_Y: i32 = HISTORY_HEADER_Y + HEADER_H as i32;
pub const HISTORY_ROWS_Y: i32 = HISTORY_COL_HEADER_Y + COL_HEADER_H as i32;
pub const HISTORY_ROWS: usize = 8;

const BG: Rgb565 = Rgb565::BLACK;
const FG: Rgb565 = Rgb565::WHITE;
const HEADER_BG: Rgb565 = Rgb565::new(0, 8, 0); // dark green, matches status_bar's bar colour
const COL_HEADER_FG: Rgb565 = Rgb565::CSS_GRAY;

fn text_style(fg: Rgb565, bg: Rgb565) -> embedded_graphics::mono_font::MonoTextStyle<'static, Rgb565> {
    MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(fg).background_color(bg).build()
}

fn fill(display: &mut impl DrawTarget<Color = Rgb565>, y: i32, h: u32, color: Rgb565) {
    let _ = Rectangle::new(Point::new(0, y), Size::new(PANEL_WIDTH, h))
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display);
}

/// Status bar: band, dial frequency, UTC clock, NTP/wsprnet
/// indicators, free heap. All-in-one line — 320 px is wide enough
/// that this never needs the FT8 status bar's field-dropping.
pub fn render_status<D>(display: &mut D, ui: &WsprUiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = text_style(FG, HEADER_BG);
    let mut s: String<64> = String::new();
    let ntp = if ui.ntp_synced { "OK" } else { "--" };
    let net = if ui.wsprnet_enabled { "ON " } else { "OFF" };
    let _ = write!(
        &mut s,
        "{:<4} {:>9.4}MHz  {}  NTP:{ntp}  NET:{net}  {:>4}k",
        ui.band_label, ui.dial_mhz, ui.utc_hhmmss, ui.free_heap_kb
    );
    fill(display, STATUS_ORIGIN_Y, STATUS_HEIGHT, HEADER_BG);
    Text::with_baseline(s.as_str(), Point::new(2, STATUS_ORIGIN_Y + 3), style, Baseline::Top)
        .draw(display)?;
    Ok(())
}

/// "Discovered stations" header + column header + up to
/// [`DISCOVERED_ROWS`] rows from the latest slot.
pub fn render_discovered<D>(display: &mut D, ui: &WsprUiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (shown, decoded) = ui.stations_counts();
    let mut header: String<48> = String::new();
    match ui.last_slot_hhmm() {
        Some(hhmm) if shown < decoded => {
            let _ = write!(&mut header, "DISCOVERED {shown}/{decoded}  slot {hhmm}Z");
        }
        Some(hhmm) => {
            let _ = write!(&mut header, "DISCOVERED {shown}  slot {hhmm}Z");
        }
        None => {
            let _ = header.push_str("DISCOVERED  (no scan yet)");
        }
    }
    fill(display, DISCOVERED_HEADER_Y, HEADER_H, BG);
    Text::with_baseline(
        header.as_str(),
        Point::new(2, DISCOVERED_HEADER_Y + 2),
        text_style(FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    fill(display, DISCOVERED_COL_HEADER_Y, COL_HEADER_H, BG);
    Text::with_baseline(
        ROW_HEADER,
        Point::new(2, DISCOVERED_COL_HEADER_Y),
        text_style(COL_HEADER_FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    render_rows(display, ui.stations().iter(), DISCOVERED_ROWS_Y, DISCOVERED_ROWS, false)
}

/// "Spot history" header + column header + up to [`HISTORY_ROWS`]
/// most recent entries, newest at the top (matches `decoded_list`'s
/// convention for the FT8 UI).
pub fn render_history<D>(display: &mut D, ui: &WsprUiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut header: String<32> = String::new();
    let _ = write!(&mut header, "HISTORY  ({} since boot)", ui.history_len());
    fill(display, HISTORY_HEADER_Y, HEADER_H, BG);
    Text::with_baseline(
        header.as_str(),
        Point::new(2, HISTORY_HEADER_Y + 2),
        text_style(FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    fill(display, HISTORY_COL_HEADER_Y, COL_HEADER_H, BG);
    Text::with_baseline(
        ROW_HEADER,
        Point::new(2, HISTORY_COL_HEADER_Y),
        text_style(COL_HEADER_FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    // `history_iter()` is oldest-first; take the trailing HISTORY_ROWS
    // and reverse so newest lands at the top of the region.
    let all: heapless::Vec<&WsprSpotRow, { super::wspr_state::HISTORY_CAP }> =
        ui.history_iter().collect();
    let take = all.len().min(HISTORY_ROWS);
    let start = all.len() - take;
    let visible = all[start..].iter().rev().copied();

    render_rows(display, visible, HISTORY_ROWS_Y, HISTORY_ROWS, true)
}

/// Shared row-painting loop for both regions: draw up to `max_rows`
/// entries from `rows` starting at `origin_y`, then blank whatever's
/// left of the region when `rows` doesn't fill it. `divider_after`
/// draws the 2 px separator below the discovered region (the history
/// region has no trailing divider).
fn render_rows<'a, D>(
    display: &mut D,
    rows: impl Iterator<Item = &'a WsprSpotRow>,
    origin_y: i32,
    max_rows: usize,
    divider_after: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = text_style(FG, BG);
    let mut buf: heapless::String<56> = heapless::String::new();
    let mut drawn = 0usize;
    for row in rows.take(max_rows) {
        let y = origin_y + (drawn as i32) * ROW_PX as i32;
        fill(display, y, ROW_PX, BG);
        row.format_row(&mut buf);
        Text::with_baseline(buf.as_str(), Point::new(2, y + 1), style, Baseline::Top).draw(display)?;
        drawn += 1;
    }
    if drawn < max_rows {
        let blank_y = origin_y + (drawn as i32) * ROW_PX as i32;
        let blank_h = ((max_rows - drawn) as u32) * ROW_PX;
        fill(display, blank_y, blank_h, BG);
    }
    if divider_after {
        fill(display, DIVIDER_Y, DIVIDER_H, COL_HEADER_FG);
    }
    Ok(())
}

/// Paint every region unconditionally — used once at boot to draw the
/// static frame (headers, divider) before the first slot completes,
/// so the panel doesn't sit blank for up to 2 minutes.
pub fn render_all<D>(display: &mut D, ui: &WsprUiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    display.clear(BG)?;
    render_status(display, ui)?;
    render_discovered(display, ui)?;
    render_history(display, ui)?;
    Ok(())
}

// Compile-time check that the fixed layout above actually fits the
// panel — a silent off-by-one here would just draw past the bottom
// edge on real hardware rather than fail anything.
const _: () = assert!((HISTORY_ROWS_Y as u32 + HISTORY_ROWS as u32 * ROW_PX) <= PANEL_HEIGHT);
