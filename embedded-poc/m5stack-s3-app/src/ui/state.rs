//! Shared UI state across `decode_pipeline` (writer) and `display`
//! (reader). Single `Mutex<UiState>` — coarse-grained but the lock
//! holds for ≪1 ms each side (push 1 row / paint 4 fields).
//!
//! Phase 3 scope: `decoded` ring + `status` only. Waterfall row
//! injection lands in a follow-up iteration once stage 1 surfaces a
//! per-slot averaged spectrum.

use core::sync::atomic::{AtomicU32, Ordering};

use heapless::String;
use std::sync::Mutex;

/// Single decoded FT8 message row, ready to render.
#[derive(Clone, Debug)]
pub struct DecodedRow {
    /// Audio offset (Hz). 200..3000 in practice.
    pub df_hz: u16,
    /// SNR (dB), clamped to i8 range. WSJT-X-style 2-digit signed.
    pub snr_db: i8,
    /// Hard-error count from BP — useful for marking borderline
    /// decodes (`!` if ≥ 24).
    pub hard_errors: u8,
    /// Decoded text. WSJT-X 77-bit packed messages fit in 22 chars.
    pub msg: String<22>,
    /// Latest slot in which this message decoded (refreshed on every
    /// re-detection — used to keep the row "alive" in the LRU dedupe).
    pub slot_seq: u32,
    /// Slot in which the message **first** appeared. The renderer
    /// highlights rows where `first_seq == max_visible_slot_seq` so
    /// genuinely-new callsigns flash the highlight while a recurring
    /// QSO (same callsigns each slot in a `wav_sim` loop) draws plain.
    pub first_seq: u32,
}

/// Status-bar fields. All optional so the bar renders during boot
/// before peripherals are up.
#[derive(Clone, Debug, Default)]
pub struct StatusInfo {
    /// Rig audio band centre (Hz). e.g. 7_074_000 for FT8 40 m.
    pub rig_freq_hz: Option<u32>,
    /// "USB" / "USB-D" / "FM" — IC-705 mode string.
    pub rig_mode: Option<String<8>>,
    /// UTC second-of-day (0..86400). Updated by time_sync.
    pub utc_sod: Option<u32>,
    /// Current free heap (KB) — sanity gauge during dev.
    pub free_heap_kb: u32,
}

/// Single waterfall row — 135 palette indices (0..15) covering the
/// FT8 audio band 200..2700 Hz at ~18.5 Hz/screen-column.
pub type WfLine = [u8; 135];

/// Waterfall depth — 100 rows fits the on-screen 100 px region; with
/// 1 row per FT8 slot that's ~25 min of band history.
pub const WF_DEPTH: usize = 100;

/// Bounded ring of decoded rows. Newest at the back. Capacity 16
/// covers >2 slots of qso3-busy density (= 7 decodes/slot) without
/// dropping; UI picks the trailing 7 to render.
pub struct UiState {
    /// LRU-ordered decode list — oldest at index 0, most-recently
    /// updated at the back. Vec (not Deque) so `push_decode` can
    /// search-and-remove existing entries in place to dedupe by msg.
    decoded: heapless::Vec<DecodedRow, 16>,
    waterfall: heapless::Deque<WfLine, WF_DEPTH>,
    pub status: StatusInfo,
    /// QSO FSM intent line — formatted by `qso::format_tx_line`.
    /// Empty until the first auto-CQ fires.
    tx_line: String<48>,
    /// Bumped each time `tx_line` changes so the display loop can
    /// repaint the TX strip independent of the WF / decode list.
    tx_seq: AtomicU32,
    /// Bumped by writers when state changes; readers compare against
    /// their last-rendered seq to skip the LCD push when nothing new.
    /// `AtomicU32` so the dirty check itself doesn't need the mutex.
    dirty_seq: AtomicU32,
    /// Monotonic count of `push_waterfall` calls. Used by the
    /// display loop as a fingerprint that *keeps advancing* once the
    /// 100-row ring is full — `Deque::len()` plateaus at WF_DEPTH and
    /// is therefore not a usable trigger after the first ~8 seconds.
    wf_push_seq: AtomicU32,
}

impl UiState {
    pub const fn new() -> Self {
        Self {
            decoded: heapless::Vec::new(),
            waterfall: heapless::Deque::new(),
            status: StatusInfo {
                rig_freq_hz: None,
                rig_mode: None,
                utc_sod: None,
                free_heap_kb: 0,
            },
            tx_line: String::new(),
            tx_seq: AtomicU32::new(0),
            dirty_seq: AtomicU32::new(0),
            wf_push_seq: AtomicU32::new(0),
        }
    }

    /// Push a fresh decode. Dedupes by `msg` — if the message text
    /// already lives in the ring, the existing entry is removed and
    /// the new copy appended at the back, preserving its original
    /// `first_seq` (so a recurring callsign doesn't re-trigger the
    /// "new" highlight). When the message is genuinely new, append
    /// and stamp `first_seq = slot_seq`. Drops the front when full.
    pub fn push_decode(&mut self, mut row: DecodedRow) {
        if let Some(idx) = self.decoded.iter().position(|r| r.msg == row.msg) {
            // Carry forward the first-seen seq from the existing
            // entry — its highlight semantics belong to *that* slot,
            // not this re-detection.
            row.first_seq = self.decoded[idx].first_seq;
            self.decoded.remove(idx);
        } else if self.decoded.is_full() {
            self.decoded.remove(0);
        }
        // `push` only fails on saturation; the branches above ensure
        // there's room.
        let _ = self.decoded.push(row);
        self.bump();
    }

    /// Push one fresh waterfall row (= one stage1_inc pair's
    /// decimated spectrum). Drops the oldest row when full and bumps
    /// `wf_push_seq` so the display loop's fingerprint check fires
    /// even after the ring saturates at `WF_DEPTH`.
    pub fn push_waterfall(&mut self, row: WfLine) {
        if self.waterfall.is_full() {
            let _ = self.waterfall.pop_front();
        }
        let _ = self.waterfall.push_back(row);
        self.wf_push_seq.fetch_add(1, Ordering::AcqRel);
        self.bump();
    }

    /// Monotonic count of `push_waterfall` calls. Display loop uses
    /// this to detect new WF rows after the ring saturates at
    /// `WF_DEPTH`; a `len()` comparison alone plateaus at full ring.
    pub fn wf_push_seq(&self) -> u32 {
        self.wf_push_seq.load(Ordering::Acquire)
    }

    /// Newest-last view of all retained rows. Vec → slice iter.
    pub fn decoded_iter(&self) -> impl Iterator<Item = &DecodedRow> {
        self.decoded.as_slice().iter()
    }

    pub fn decoded_len(&self) -> usize {
        self.decoded.len()
    }

    /// Waterfall iterator — oldest first, newest last.
    pub fn waterfall_iter(&self) -> impl Iterator<Item = &WfLine> {
        self.waterfall.iter()
    }

    pub fn waterfall_len(&self) -> usize {
        self.waterfall.len()
    }

    /// Render-side dirty check. Returns the current dirty seq;
    /// readers should compare with their last-seen value and skip the
    /// LCD push when equal.
    pub fn dirty_seq(&self) -> u32 {
        self.dirty_seq.load(Ordering::Acquire)
    }

    fn bump(&self) {
        self.dirty_seq.fetch_add(1, Ordering::AcqRel);
    }

    /// Update status bar from any thread.
    pub fn update_status(&mut self, f: impl FnOnce(&mut StatusInfo)) {
        f(&mut self.status);
        self.bump();
    }

    /// Replace the TX-line text. Truncates silently if `s` overflows
    /// 48 chars (the strip is ~22 cols at 6-px font width).
    pub fn set_tx_line(&mut self, s: &str) {
        self.tx_line.clear();
        for ch in s.chars() {
            if self.tx_line.push(ch).is_err() {
                break;
            }
        }
        self.tx_seq.fetch_add(1, Ordering::AcqRel);
        self.bump();
    }

    pub fn tx_line(&self) -> &str {
        self.tx_line.as_str()
    }

    pub fn tx_seq(&self) -> u32 {
        self.tx_seq.load(Ordering::Acquire)
    }
}

/// Process-wide single instance. `Mutex` not `RwLock` since both
/// sides hold the lock briefly and esp-idf's `RwLock` has no async
/// advantages here.
pub static UI: Mutex<UiState> = Mutex::new(UiState::new());
