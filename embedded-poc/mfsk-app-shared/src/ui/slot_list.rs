//! Slotted-mode decode screen for the CoreS3's panel, run **portrait**
//! (240×320) — the same geometry, chrome and repaint discipline
//! `wspr_list` established, carrying decoded rows instead of WSPR
//! spots. Used by the FST4 monitor and the FT4 receiver.
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
//! [`SlotUiState::dirty_seq`].
//!
//! **Not FST4-specific**, despite where it started: the columns are
//! UTC / frequency / SNR / DT / message / post-slot latency, which is
//! what every slotted mode in this tree decodes into. FT4's receiver
//! renders through the same types (`apps/ft4.rs`), which is why they
//! were renamed off `Fst4*` on 2026-08-30. The one place the origin
//! still shows is [`SlotSpotRow::snr_db`] being optional — see its
//! comment; FT4 always has one, FST4's DDC monitor path does not.

use core::fmt::Write as _;

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;

use super::spot_render::{
    fill, render_rows, text_style, FormatRow, BG, COL_HEADER_FG, COL_HEADER_H, DIVIDER_H, FG,
    HEADER_H, ROW_PX,
};
/// Panel geometry lives in [`super::spot_render`] now; re-exported so
/// the app's own layout maths keeps referring to this screen's module.
pub use super::spot_render::{PANEL_HEIGHT, PANEL_WIDTH, STATUS_HEIGHT, STATUS_ORIGIN_Y};
use super::spot_state::SpotState;

const SLOT_HEADER_Y: i32 = STATUS_HEIGHT as i32;

const SLOT_COL_HEADER_Y: i32 = SLOT_HEADER_Y + HEADER_H as i32;
pub const SLOT_ROWS_Y: i32 = SLOT_COL_HEADER_Y + COL_HEADER_H as i32;
pub const SLOT_ROWS: usize = 9;
const SLOT_REGION_H: u32 = SLOT_ROWS as u32 * ROW_PX;

const DIVIDER_Y: i32 = SLOT_ROWS_Y + SLOT_REGION_H as i32;

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

const HEADER_BG: Rgb565 = Rgb565::new(0, 8, 0);

/// One decoded FST4 transmission, sized for a 40-char panel row.
#[derive(Clone, Debug)]
pub struct SlotSpotRow {
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

/// Column header matching [`SlotSpotRow::format_row`]'s field order.
pub const ROW_HEADER: &str = "UTC  FREQ  SNR   DT  MESSAGE      T+";

impl FormatRow for SlotSpotRow {
    fn format_row(&self, out: &mut heapless::String<56>) {
        SlotSpotRow::format_row(self, out);
    }
}

impl SlotSpotRow {
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
pub struct SlotUiState {
    /// This slot's rows and the rolling history — the half that is
    /// identical to WSPR's screen, and so lives in
    /// [`super::spot_state`] where a host test can reach it.
    spots: SpotState<SlotSpotRow, SLOT_CAP, HISTORY_CAP>,
    /// Candidates the coarse stage produced for the last slot.
    pub last_slot_cands: usize,
    /// How many of them the decode loop started.
    pub last_slot_tried: usize,
    /// Milliseconds from slot close to the first decode.
    pub last_first_decode_ms: i64,
    pub dial_mhz: f64,
    pub utc_hhmmss: String<8>,
    pub ntp_synced: bool,
    pub audio_live: bool,
    pub free_heap_kb: u32,
}

impl Default for SlotUiState {
    fn default() -> Self {
        Self::new()
    }
}

impl SlotUiState {
    pub const fn new() -> Self {
        Self {
            spots: SpotState::new(),
            last_slot_cands: 0,
            last_slot_tried: 0,
            last_first_decode_ms: 0,
            dial_mhz: 0.0,
            utc_hhmmss: String::new(),
            ntp_synced: false,
            audio_live: false,
            free_heap_kb: 0,
        }
    }

    /// Replace this slot's decodes and append them to the history.
    pub fn set_slot(&mut self, hhmm: &str, rows: &[SlotSpotRow]) {
        self.spots.set_slot(hhmm, rows);
    }

    pub fn dirty_seq(&self) -> u32 {
        self.spots.dirty_seq()
    }
    pub fn slot_rows(&self) -> &[SlotSpotRow] {
        self.spots.rows()
    }
    pub fn slot_counts(&self) -> (usize, usize) {
        self.spots.counts()
    }
    pub fn last_slot_hhmm(&self) -> Option<&str> {
        self.spots.last_slot_hhmm()
    }
    pub fn history_len(&self) -> usize {
        self.spots.history_len()
    }
    pub fn history_iter(&self) -> impl Iterator<Item = &SlotSpotRow> {
        self.spots.history_iter()
    }
}

/// Status bar: mode, dial frequency, UTC, NTP and live-audio flags,
/// free heap. `A` is the flag that says whether the decodes on screen
/// came from a radio or from the built-in golden slot.
pub fn render_status<D>(display: &mut D, ui: &SlotUiState) -> Result<(), D::Error>
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
pub fn render_slot<D>(display: &mut D, ui: &SlotUiState) -> Result<(), D::Error>
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

    render_rows(display, ui.slot_rows().iter(), SLOT_ROWS_Y, SLOT_ROWS, None)
}

/// Decode history, newest at the top.
pub fn render_history<D>(display: &mut D, ui: &SlotUiState) -> Result<(), D::Error>
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

    let all: heapless::Vec<&SlotSpotRow, HISTORY_CAP> = ui.history_iter().collect();
    let take = all.len().min(HISTORY_ROWS);
    let start = all.len() - take;
    let visible = all[start..].iter().rev().copied();

    render_rows(display, visible, HISTORY_ROWS_Y, HISTORY_ROWS, None)
}

/// Paint every region — the first frame, before any slot completes.
/// Explicit `Rectangle` fill rather than `DrawTarget::clear()`, which
/// real-hardware testing found unreliable on this mipidsi/SPI setup
/// (see `wspr_list::render_all` for the full finding).
pub fn render_all<D>(display: &mut D, ui: &SlotUiState) -> Result<(), D::Error>
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
