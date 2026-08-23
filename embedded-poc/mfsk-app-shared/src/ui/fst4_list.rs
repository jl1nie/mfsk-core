//! FST4 monitor screen for the CoreS3's panel, run **portrait**
//! (240×320) — the same geometry, chrome and repaint discipline
//! `wspr_list` established, carrying FST4 rows instead of WSPR spots.
//!
//! ```text
//! y=0..16     status bar        (mode / dial freq / UTC / NTP / heap)
//! y=16..37    this-slot header + column header
//! y=37..145   this slot's decodes (9 rows, wholesale replace)
//! y=145..147  divider
//! y=147..168  history header + column header
//! y=168..312  decode history (12 rows, newest at top)
//! y=312..320  unused margin
//! ```
//!
//! **A sibling of `wspr_list`, not a generalisation of it.** The two
//! could share a row-agnostic renderer — the regions differ only in
//! header text and row type — and that refactor is worth doing. It is
//! deliberately not done here: `wspr_list`'s layout constants were
//! settled by looking at a real panel (see its doc comment, and
//! `render_all`'s note on `DrawTarget::clear()` being unreliable on
//! this mipidsi/SPI setup), and a refactor whose only real test is "does
//! the screen still look right" should not ride along with a new app
//! that cannot verify the old screen. The duplication is one file and
//! is marked, rather than silently accumulated.
//!
//! What it does share, structurally: per-region wipe-then-draw with no
//! full-frame clear (avoids flicker), an explicit `Rectangle` fill
//! rather than `clear()`, and a caller that gates repaints on
//! [`Fst4UiState::dirty_seq`].

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

pub const PANEL_WIDTH: u32 = 240;
pub const PANEL_HEIGHT: u32 = 320;

pub const STATUS_ORIGIN_Y: i32 = 0;
pub const STATUS_HEIGHT: u32 = 16;

const SLOT_HEADER_Y: i32 = STATUS_HEIGHT as i32;
const HEADER_H: u32 = 11;
const COL_HEADER_H: u32 = 10;
const ROW_PX: u32 = 12;

const SLOT_COL_HEADER_Y: i32 = SLOT_HEADER_Y + HEADER_H as i32;
pub const SLOT_ROWS_Y: i32 = SLOT_COL_HEADER_Y + COL_HEADER_H as i32;
pub const SLOT_ROWS: usize = 9;
const SLOT_REGION_H: u32 = SLOT_ROWS as u32 * ROW_PX;

const DIVIDER_Y: i32 = SLOT_ROWS_Y + SLOT_REGION_H as i32;
const DIVIDER_H: u32 = 2;

const HISTORY_HEADER_Y: i32 = DIVIDER_Y + DIVIDER_H as i32;
const HISTORY_COL_HEADER_Y: i32 = HISTORY_HEADER_Y + HEADER_H as i32;
pub const HISTORY_ROWS_Y: i32 = HISTORY_COL_HEADER_Y + COL_HEADER_H as i32;
/// Eleven, not twelve: the bottom 14 px of the panel belong to the
/// shared [`link_bar`](crate::ui::link_bar), which every mode draws in
/// the same place. One history row is what it costs to be able to see,
/// in any mode, whether there is a radio on the USB port.
pub const HISTORY_ROWS: usize = 11;

/// One slot's worth of decodes kept for display. The monitor's own
/// `max_cand` is 50, but distinct *messages* in one 60 s slot are far
/// fewer; 16 is the same headroom `wspr_state` uses.
pub const SLOT_CAP: usize = 16;
pub const HISTORY_CAP: usize = 48;

const BG: Rgb565 = Rgb565::BLACK;
const FG: Rgb565 = Rgb565::WHITE;
const HEADER_BG: Rgb565 = Rgb565::new(0, 8, 0);
const COL_HEADER_FG: Rgb565 = Rgb565::CSS_GRAY;

/// One decoded FST4 transmission, sized for a 40-char panel row.
#[derive(Clone, Debug)]
pub struct Fst4SpotRow {
    /// Slot start, UTC, `HHMM`.
    pub utc_hhmm: String<4>,
    /// Decoded audio frequency, Hz — the refined offset, not the
    /// coarse cell's.
    pub freq_hz: f32,
    /// `None` when the decoder could not report one.
    ///
    /// That is the normal case on the wideband DDC monitor path: FST4's
    /// WSJT-X-faithful SNR estimate reads a baseline out of the
    /// whole-slot forward FFT (`SnrCtx::fft_cache`), and the DDC front
    /// end exists precisely so that FFT is never computed. Reporting
    /// nothing is the honest answer until an SNR estimator that works
    /// from the DDC baseband exists.
    pub snr_db: Option<i8>,
    pub dt_sec: f32,
    /// The 77-bit message as text; WSJT-X messages fit in 22 chars.
    pub msg: String<22>,
    /// Seconds after the slot ended that this decode landed. The
    /// number a monitor is actually judged on, and the one thing a
    /// bench log shows that a receiver screen otherwise would not.
    pub t_s: f32,
}

/// Column header matching [`Fst4SpotRow::format_row`]'s field order.
pub const ROW_HEADER: &str = "UTC  FREQ  SNR   DT  MESSAGE      T+";

impl Fst4SpotRow {
    /// One fixed-width line: `HHMM freq snr dt message t+`.
    ///
    /// `snr` renders as `--` when absent, which on the DDC monitor
    /// path is the normal case rather than an error — see
    /// [`Self::snr_db`].
    pub fn format_row(&self, out: &mut String<56>) {
        out.clear();
        let mut snr: String<4> = String::new();
        match self.snr_db {
            Some(v) => {
                let _ = write!(&mut snr, "{v:>3}");
            }
            None => {
                let _ = snr.push_str(" --");
            }
        }
        let _ = write!(
            out,
            "{:<4} {:>5.0} {:>3} {:>+4.1} {:<13} {:>3.1}",
            self.utc_hhmm.as_str(),
            self.freq_hz,
            snr.as_str(),
            self.dt_sec,
            // Truncated to the column, not to storage: a long message
            // must not push the `t+` column out of alignment.
            &self.msg.as_str()[..self.msg.len().min(13)],
            self.t_s,
        );
    }
}

/// What the FST4 screen renders. Mutated by the scan task once a slot,
/// read by the display task at its own tick — [`Self::dirty_seq`] is
/// what keeps the expensive panes from repainting in between.
pub struct Fst4UiState {
    slot: heapless::Vec<Fst4SpotRow, SLOT_CAP>,
    history: heapless::Deque<Fst4SpotRow, HISTORY_CAP>,
    last_slot_hhmm: Option<String<4>>,
    /// Decodes the last slot produced before [`SLOT_CAP`] truncation,
    /// so a truncated slot is visible rather than looking like a quiet
    /// band.
    last_slot_count: usize,
    /// Candidates the last slot's coarse search produced, and how many
    /// of them the loop actually reached before its deadline — the two
    /// numbers that say whether the monitor is keeping up.
    pub last_slot_cands: usize,
    pub last_slot_tried: usize,
    /// Post-slot milliseconds to the first decode of the last slot.
    pub last_first_decode_ms: i64,
    pub dial_mhz: f64,
    pub utc_hhmmss: String<8>,
    pub ntp_synced: bool,
    pub audio_live: bool,
    pub free_heap_kb: u32,
    dirty_seq: AtomicU32,
}

impl Default for Fst4UiState {
    fn default() -> Self {
        Self::new()
    }
}

impl Fst4UiState {
    pub const fn new() -> Self {
        Self {
            slot: heapless::Vec::new(),
            history: heapless::Deque::new(),
            last_slot_hhmm: None,
            last_slot_count: 0,
            last_slot_cands: 0,
            last_slot_tried: 0,
            last_first_decode_ms: 0,
            dial_mhz: 0.0,
            utc_hhmmss: String::new(),
            ntp_synced: false,
            audio_live: false,
            free_heap_kb: 0,
            dirty_seq: AtomicU32::new(0),
        }
    }

    /// Replace this slot's decodes and append them to the history.
    pub fn set_slot(&mut self, hhmm: &str, rows: &[Fst4SpotRow]) {
        self.slot.clear();
        for r in rows.iter().take(SLOT_CAP) {
            let _ = self.slot.push(r.clone());
        }
        self.last_slot_count = rows.len();
        let mut h: String<4> = String::new();
        let _ = h.push_str(&hhmm[..hhmm.len().min(4)]);
        self.last_slot_hhmm = Some(h);
        for r in rows {
            if self.history.is_full() {
                let _ = self.history.pop_front();
            }
            let _ = self.history.push_back(r.clone());
        }
        self.dirty_seq.fetch_add(1, Ordering::Release);
    }

    pub fn dirty_seq(&self) -> u32 {
        self.dirty_seq.load(Ordering::Acquire)
    }
    pub fn slot_rows(&self) -> &[Fst4SpotRow] {
        &self.slot
    }
    pub fn slot_counts(&self) -> (usize, usize) {
        (self.slot.len(), self.last_slot_count)
    }
    pub fn last_slot_hhmm(&self) -> Option<&str> {
        self.last_slot_hhmm.as_deref()
    }
    pub fn history_len(&self) -> usize {
        self.history.len()
    }
    pub fn history_iter(&self) -> impl Iterator<Item = &Fst4SpotRow> {
        self.history.iter()
    }
}

fn text_style(
    fg: Rgb565,
    bg: Rgb565,
) -> embedded_graphics::mono_font::MonoTextStyle<'static, Rgb565> {
    MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(fg)
        .background_color(bg)
        .build()
}

fn fill(display: &mut impl DrawTarget<Color = Rgb565>, y: i32, h: u32, color: Rgb565) {
    let _ = Rectangle::new(Point::new(0, y), Size::new(PANEL_WIDTH, h))
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display);
}

/// Status bar: mode, dial frequency, UTC, NTP and live-audio flags,
/// free heap. `A` is the flag that says whether the decodes on screen
/// came from a radio or from the built-in golden slot.
pub fn render_status<D>(display: &mut D, ui: &Fst4UiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut s: String<48> = String::new();
    let ntp = if ui.ntp_synced { 'Y' } else { 'N' };
    let aud = if ui.audio_live { 'Y' } else { 'N' };
    let _ = write!(
        &mut s,
        "FST4 {:>8.4}M {} N{ntp} A{aud} {:>4}k",
        ui.dial_mhz, ui.utc_hhmmss, ui.free_heap_kb
    );
    fill(display, STATUS_ORIGIN_Y, STATUS_HEIGHT, HEADER_BG);
    Text::with_baseline(
        s.as_str(),
        Point::new(2, STATUS_ORIGIN_Y + 3),
        text_style(FG, HEADER_BG),
        Baseline::Top,
    )
    .draw(display)?;
    Ok(())
}

/// This slot's decodes, headed by the two numbers that say whether the
/// monitor kept up: candidates reached out of candidates found, and
/// how long the first decode took.
pub fn render_slot<D>(display: &mut D, ui: &Fst4UiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (shown, decoded) = ui.slot_counts();
    let mut header: String<48> = String::new();
    match ui.last_slot_hhmm() {
        Some(hhmm) => {
            let _ = write!(
                &mut header,
                "SLOT {hhmm}Z {shown}/{decoded} {}/{} c {}ms",
                ui.last_slot_tried, ui.last_slot_cands, ui.last_first_decode_ms
            );
        }
        None => {
            let _ = header.push_str("SLOT  (no decode yet)");
        }
    }
    fill(display, SLOT_HEADER_Y, HEADER_H, BG);
    Text::with_baseline(
        header.as_str(),
        Point::new(2, SLOT_HEADER_Y + 2),
        text_style(FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    fill(display, SLOT_COL_HEADER_Y, COL_HEADER_H, BG);
    Text::with_baseline(
        ROW_HEADER,
        Point::new(2, SLOT_COL_HEADER_Y),
        text_style(COL_HEADER_FG, BG),
        Baseline::Top,
    )
    .draw(display)?;

    render_rows(display, ui.slot_rows().iter(), SLOT_ROWS_Y, SLOT_ROWS, false)
}

/// Decode history, newest at the top.
pub fn render_history<D>(display: &mut D, ui: &Fst4UiState) -> Result<(), D::Error>
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

    let all: heapless::Vec<&Fst4SpotRow, HISTORY_CAP> = ui.history_iter().collect();
    let take = all.len().min(HISTORY_ROWS);
    let start = all.len() - take;
    let visible = all[start..].iter().rev().copied();

    render_rows(display, visible, HISTORY_ROWS_Y, HISTORY_ROWS, true)
}

fn render_rows<'a, D>(
    display: &mut D,
    rows: impl Iterator<Item = &'a Fst4SpotRow>,
    origin_y: i32,
    max_rows: usize,
    divider_after: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = text_style(FG, BG);
    let mut buf: String<56> = String::new();
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
    if divider_after {
        fill(display, DIVIDER_Y, DIVIDER_H, COL_HEADER_FG);
    }
    Ok(())
}

/// Paint every region — the first frame, before any slot completes.
/// Explicit `Rectangle` fill rather than `DrawTarget::clear()`, which
/// real-hardware testing found unreliable on this mipidsi/SPI setup
/// (see `wspr_list::render_all` for the full finding).
pub fn render_all<D>(display: &mut D, ui: &Fst4UiState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    fill(display, 0, PANEL_HEIGHT, BG);
    render_status(display, ui)?;
    render_slot(display, ui)?;
    render_history(display, ui)?;
    Ok(())
}

const _: () = assert!((HISTORY_ROWS_Y as u32 + HISTORY_ROWS as u32 * ROW_PX) <= PANEL_HEIGHT);
