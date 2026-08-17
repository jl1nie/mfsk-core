//! Shared WSPR UI state — the WSPR sibling of [`crate::ui::state`].
//!
//! Kept as its own `Mutex` + static rather than folded into the
//! existing `UiState`/`UI`: that type is shaped entirely around the
//! FT8 controller (decode ring keyed by `slot_seq` for a QSO FSM,
//! menu overlay state, TX-line string) and `wspr_app.rs` is a
//! separate binary that never runs the FT8 controller loop
//! alongside it — there is nothing to share except the "one `Mutex`,
//! `AtomicU32` dirty-seq" *pattern*, which this module reuses
//! structurally without reusing the type.
//!
//! Two views over the same underlying decodes, matching the app's
//! original two-screen request:
//! - **Discovered stations** = the latest slot only, wholesale
//!   replaced by [`WsprUiState::set_slot`] every ~2 minutes.
//! - **Spot history** = append-only across every slot since boot,
//!   capped at [`HISTORY_CAP`] (oldest dropped once full — a true
//!   unbounded log isn't feasible without external storage, same
//!   trade-off `ui::state::UiState`'s 16-row decode ring already
//!   makes for FT8).

use core::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;

use super::wspr_row::WsprSpotRow;

/// Visible-at-once cap for the "discovered stations" pane. A busy
/// 20 m WSPR band can decode more than this in one slot; excess
/// entries are dropped (oldest-by-scan-order) rather than silently
/// growing the pane past what 320×240 can show — see
/// [`WsprUiState::set_slot`]'s doc comment for how the drop is
/// logged, not hidden.
pub const STATIONS_CAP: usize = 16;

/// Cumulative spot-history ring depth. At ~2 rows/slot average this
/// is roughly 30-60 minutes of history before the oldest entries
/// start rolling off; a busier band rolls off sooner.
pub const HISTORY_CAP: usize = 64;

/// Everything the WSPR app's display loop needs to paint a frame.
pub struct WsprUiState {
    /// Latest slot's decodes, in decode order (not sorted). Wholesale
    /// replaced each slot by [`Self::set_slot`].
    stations: heapless::Vec<WsprSpotRow, STATIONS_CAP>,
    /// Every decode since boot, oldest first, oldest dropped when
    /// full.
    history: heapless::Deque<WsprSpotRow, HISTORY_CAP>,
    /// `HHMM` of the most recently completed slot — `None` before the
    /// first scan finishes, so the header can show "no scan yet"
    /// rather than a stale/blank time.
    last_slot_hhmm: Option<heapless::String<4>>,
    /// How many decodes the last slot produced, *before* the
    /// [`STATIONS_CAP`] truncation — the "discovered" header shows
    /// this alongside `stations.len()` so a truncated slot is visible
    /// to the operator instead of looking like a quiet band.
    last_slot_decoded_count: usize,
    /// Selected band's display label (e.g. `"20m"`), mirrored from
    /// `Settings::band_idx` + `wspr_bands::WSPR_BANDS` by the caller
    /// so the status bar doesn't need its own NVS read.
    pub band_label: heapless::String<8>,
    pub dial_mhz: f64,
    /// `"HH:MM:SS"`, updated once per display tick from the system
    /// clock (meaningful only once NTP has synced — see
    /// `ntp_synced` below).
    pub utc_hhmmss: heapless::String<8>,
    pub ntp_synced: bool,
    /// Mirrors `Settings::wsprnet_spot_config != "off"` — the status
    /// bar shows a plain on/off indicator, not the raw config string
    /// (which may be a URL not worth rendering at this width).
    pub wsprnet_enabled: bool,
    pub free_heap_kb: u32,
    dirty_seq: AtomicU32,
}

impl WsprUiState {
    pub const fn new() -> Self {
        Self {
            stations: heapless::Vec::new(),
            history: heapless::Deque::new(),
            last_slot_hhmm: None,
            last_slot_decoded_count: 0,
            band_label: heapless::String::new(),
            dial_mhz: 0.0,
            utc_hhmmss: heapless::String::new(),
            ntp_synced: false,
            wsprnet_enabled: false,
            free_heap_kb: 0,
            dirty_seq: AtomicU32::new(0),
        }
    }

    /// Replace the "discovered stations" pane with one slot's worth
    /// of decodes, and append every one of them to the cumulative
    /// history ring.
    ///
    /// `decoded` may exceed [`STATIONS_CAP`] on a busy band; this
    /// stores only the first `STATIONS_CAP` for the discovered pane
    /// (order is whatever the caller's scan produced — typically
    /// frequency- or SNR-sorted already) but still logs every one of
    /// them to `history`, and records the pre-truncation count so the
    /// header can show `"DISCOVERED 16/23"` rather than silently
    /// looking like only 16 were heard.
    pub fn set_slot(&mut self, hhmm: heapless::String<4>, decoded: &[WsprSpotRow]) {
        self.last_slot_hhmm = Some(hhmm);
        self.last_slot_decoded_count = decoded.len();

        self.stations.clear();
        for row in decoded.iter().take(STATIONS_CAP) {
            // `push` only fails on capacity, which `.take(STATIONS_CAP)`
            // already guarantees can't happen here.
            let _ = self.stations.push(row.clone());
        }

        for row in decoded {
            if self.history.is_full() {
                let _ = self.history.pop_front();
            }
            let _ = self.history.push_back(row.clone());
        }

        self.bump();
    }

    pub fn stations(&self) -> &[WsprSpotRow] {
        self.stations.as_slice()
    }

    /// `(shown, decoded_before_truncation)` — the header renders
    /// `"DISCOVERED {shown}/{decoded}"` when they differ, plain
    /// `"DISCOVERED {shown}"` otherwise.
    pub fn stations_counts(&self) -> (usize, usize) {
        (self.stations.len(), self.last_slot_decoded_count)
    }

    pub fn last_slot_hhmm(&self) -> Option<&str> {
        self.last_slot_hhmm.as_deref()
    }

    /// History, oldest first — matches `Deque`'s natural iteration
    /// order. Renderer decides how to slice/reverse for display.
    pub fn history_iter(&self) -> impl Iterator<Item = &WsprSpotRow> {
        self.history.iter()
    }

    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Render-side dirty check, same contract as
    /// `ui::state::UiState::dirty_seq`: readers compare against their
    /// last-seen value and skip the LCD push when unchanged.
    pub fn dirty_seq(&self) -> u32 {
        self.dirty_seq.load(Ordering::Acquire)
    }

    fn bump(&self) {
        self.dirty_seq.fetch_add(1, Ordering::AcqRel);
    }

    /// Update the status-bar-shaped fields (band, clock, sync flags).
    /// Bumps dirty like every other mutation — these fields are cheap
    /// and change every display tick, so this is called far more
    /// often than `set_slot`.
    pub fn update_status(&mut self, f: impl FnOnce(&mut Self)) {
        f(self);
        self.bump();
    }
}

impl Default for WsprUiState {
    fn default() -> Self {
        Self::new()
    }
}

/// Process-wide single instance, same rationale as `ui::state::UI`
/// (`Mutex` not `RwLock` — both sides hold it briefly, esp-idf's
/// `RwLock` has no async advantage here).
pub static WSPR_UI: Mutex<WsprUiState> = Mutex::new(WsprUiState::new());
