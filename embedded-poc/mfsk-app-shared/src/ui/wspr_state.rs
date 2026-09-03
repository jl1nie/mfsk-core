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

use std::sync::Mutex;

use super::spot_state::SpotState;

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
    /// The latest slot's spots and the rolling history — the half
    /// this screen shares with FST4's, in [`super::spot_state`] so a
    /// host test can reach it.
    spots: SpotState<WsprSpotRow, STATIONS_CAP, HISTORY_CAP>,
    /// Selected band's display label (e.g. `"20m"`), mirrored from
    /// `Settings::band_idx` + `wspr_bands::WSPR_BANDS` by the caller
    /// so the status bar doesn't need its own NVS read.
    pub band_label: heapless::String<8>,
    pub dial_mhz: f64,
    pub utc_hhmmss: heapless::String<8>,
    pub ntp_synced: bool,
    pub wsprnet_enabled: bool,
    pub free_heap_kb: u32,
}

impl WsprUiState {
    pub const fn new() -> Self {
        Self {
            spots: SpotState::new(),
            band_label: heapless::String::new(),
            dial_mhz: 0.0,
            utc_hhmmss: heapless::String::new(),
            ntp_synced: false,
            wsprnet_enabled: false,
            free_heap_kb: 0,
        }
    }

    /// Replace the discovered-stations pane with this slot's decodes,
    /// append them to `history`, and record the pre-truncation count
    /// so the header can show `"DISCOVERED 16/23"` rather than
    /// silently looking like only 16 were heard.
    pub fn set_slot(&mut self, hhmm: heapless::String<4>, decoded: &[WsprSpotRow]) {
        self.spots.set_slot(&hhmm, decoded);
    }

    pub fn stations(&self) -> &[WsprSpotRow] {
        self.spots.rows()
    }

    /// `(shown, decoded_before_truncation)` — the header renders
    /// `"DISCOVERED {shown}/{decoded}"` when they differ, plain
    /// `"DISCOVERED {shown}"` otherwise.
    pub fn stations_counts(&self) -> (usize, usize) {
        self.spots.counts()
    }

    pub fn last_slot_hhmm(&self) -> Option<&str> {
        self.spots.last_slot_hhmm()
    }

    /// History, oldest first — matches `Deque`'s natural iteration
    /// order. Renderer decides how to slice/reverse for display.
    pub fn history_iter(&self) -> impl Iterator<Item = &WsprSpotRow> {
        self.spots.history_iter()
    }

    pub fn history_len(&self) -> usize {
        self.spots.history_len()
    }

    /// Render-side dirty check, same contract as
    /// `ui::state::UiState::dirty_seq`: readers compare against their
    /// last-seen value and skip the LCD push when unchanged.
    pub fn dirty_seq(&self) -> u32 {
        self.spots.dirty_seq()
    }

    /// Update the status-bar-shaped fields (band, clock, sync flags).
    /// Bumps dirty like every other mutation — these fields are cheap
    /// and change every display tick, so this is called far more
    /// often than `set_slot`.
    pub fn update_status(&mut self, f: impl FnOnce(&mut Self)) {
        f(self);
        self.spots.bump();
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
