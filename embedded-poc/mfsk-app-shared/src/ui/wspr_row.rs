//! One decoded WSPR transmission, in the shape the WSPR UI screens
//! render.
//!
//! Deliberately a separate type from [`crate::wsprnet::Spot`], not a
//! reuse of it: `Spot` is a wire-format type shaped around what
//! wsprnet.org's POST body wants (`date`/`time` as pre-split
//! `yyMMdd`/`HHmm` strings, `dbm`/`snr_db` as `i32` because that's
//! what `to_string()` needs). This type is shaped around what a
//! 320×240 landscape row needs to paint fast and fit in
//! [`heapless::Vec`]/[`heapless::Deque`] rings cheaply — narrower
//! integer widths, one combined `HHMM` field instead of two strings.
//! Both ultimately describe the same decode; the call site
//! (`wspr_app.rs`, step 7 of the implementation plan) builds one of
//! each from the same decoder output.

use core::fmt::Write as _;

/// One decoded WSPR transmission, sized for display rather than wire
/// transmission.
#[derive(Clone, Debug)]
pub struct WsprSpotRow {
    /// Slot start, UTC, as 4-digit `HHMM` — no seconds, since WSPR's
    /// grid is 2-minute-aligned and wsprnet's own `time` field is the
    /// same precision.
    pub utc_hhmm: heapless::String<4>,
    /// Transmitter callsign, as decoded. May carry `/P`-style compound
    /// forms or `<...>` hashed-callsign brackets, same as
    /// [`crate::wsprnet::Spot::call`].
    pub call: heapless::String<13>,
    /// Transmitter locator, 4 chars. Empty for a type-2 message (no
    /// grid transmitted) — rendered blank, not omitted, so the column
    /// stays aligned.
    pub grid: heapless::String<6>,
    /// Transmitted power, dBm. `i8` comfortably covers WSPR's standard
    /// power set (0-60 dBm in the codebook WSJT-X transmits).
    pub power_dbm: i8,
    /// Decoded audio frequency, Hz — the raw passband offset (centred
    /// near 1500 Hz), same quantity as
    /// [`crate::wsprnet::Spot::audio_hz`]. Kept as `f32` since the
    /// decoder's own frequency resolution is sub-Hz.
    pub freq_offset_hz: f32,
    /// Reported SNR, dB (WSPR SNR is always well within `i8` range).
    pub snr_db: i8,
    /// Time offset, seconds.
    pub dt_sec: f32,
    /// Drift, Hz over the transmission (WSPR reports a small integer
    /// here, never more than a few Hz).
    pub drift_hz: i8,
}

/// Column header matching [`WsprSpotRow::format_row`]'s field order
/// and widths — draw this once above a list of formatted rows.
pub const HEADER: &str = "UTC  CALLSIGN  GRID  DBM FREQ  SNR    DT DRIFT";

impl WsprSpotRow {
    /// Render one fixed-width line: `HHMM call grid dbm freq snr dt
    /// drift`, matching [`HEADER`]'s column layout. Truncates
    /// `call`/`grid` to their display width (9/5 chars) rather than
    /// their storage capacity (13/6) — storage is sized for the
    /// longest realistic value (a hashed callsign, `<PJ4/K1ABC>`, is
    /// 11 chars), display is sized to keep the whole line under the
    /// 320 px / 6 px-per-glyph = 53-char budget of the CoreS3 panel
    /// at `FONT_6X10`.
    pub fn format_row(&self, buf: &mut heapless::String<56>) {
        buf.clear();
        let _ = write!(
            buf,
            "{:<4} {:<9.9} {:<5.5} {:>3} {:>4.0} {:>+4} {:>+5.1} {:>+3}",
            self.utc_hhmm,
            self.call,
            self.grid,
            self.power_dbm,
            self.freq_offset_hz,
            self.snr_db,
            self.dt_sec,
            self.drift_hz,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs on the host via `hosttest/mfsk-app-shared` (this module
    /// has no `esp_idf_svc` dependency, same as `wsprnet`/
    /// `wspr_bands` — see that crate's `#[path]` registration) rather
    /// than being trusted from a `+esp build` pass alone.
    #[test]
    fn format_row_fits_the_53_char_budget() {
        let row = WsprSpotRow {
            utc_hhmm: heapless::String::try_from("1234").unwrap(),
            call: heapless::String::try_from("<PJ4/K1ABC>").unwrap(),
            grid: heapless::String::try_from("PM95").unwrap(),
            power_dbm: 37,
            freq_offset_hz: 1462.9,
            snr_db: -23,
            dt_sec: 2.23,
            drift_hz: -1,
        };
        let mut buf = heapless::String::new();
        row.format_row(&mut buf);
        assert!(buf.len() <= 53, "line too wide for the panel: {buf:?} ({} chars)", buf.len());
        assert!(buf.starts_with("1234 <PJ4/K1AB PM95 "), "{buf}");
    }
}
