// SPDX-License-Identifier: GPL-3.0-or-later
//! The container behind both spot lists: this slot's rows, a rolling
//! history, and a dirty counter.
//!
//! ## Why it is one type now
//!
//! `slot_list::SlotUiState` (FST4) and `wspr_state::WsprUiState`
//! (WSPR) were the same container written twice — latest-slot `Vec`,
//! history `Deque`, `last_slot_hhmm`, a pre-truncation count so the
//! header can say `16/23`, and an `AtomicU32` the render loop compares
//! against. They differ where they should: the row type, and the
//! status-bar fields each mode puts beside the list. Those stay with
//! their modes; this holds what does not vary.
//!
//! The rows are generic rather than trait-bound: this type never looks
//! inside a row. Formatting is `SlotSpotRow::format_row` /
//! `WsprSpotRow::format_row`, called by the renderers.
//!
//! ## Why it is here rather than in either list module
//!
//! Both of those pull in the `embedded-graphics` draw stack, so they
//! cannot be compiled by `hosttest/mfsk-app-shared` and their logic
//! has never been exercised by a test that runs anywhere. This module
//! is `heapless` plus one atomic, which the host harness can build —
//! see `hosttest/mfsk-app-shared/src/lib.rs`.

use core::sync::atomic::{AtomicU32, Ordering};

use heapless::{Deque, String, Vec};

/// `SLOT_CAP` rows for the current slot, `HIST_CAP` for the history.
///
/// Both caps are what fits the panel and the memory budget, not what a
/// band can produce — a slot that decodes more than `SLOT_CAP` keeps
/// the first `SLOT_CAP` for display and still reports the true count
/// through [`counts`](Self::counts), so a truncated slot is visible to
/// the operator instead of looking like a quiet band.
pub struct SpotState<R, const SLOT_CAP: usize, const HIST_CAP: usize> {
    slot: Vec<R, SLOT_CAP>,
    history: Deque<R, HIST_CAP>,
    last_slot_hhmm: Option<String<4>>,
    /// Decodes this slot produced *before* the `SLOT_CAP` truncation.
    last_slot_count: usize,
    dirty_seq: AtomicU32,
}

impl<R, const SLOT_CAP: usize, const HIST_CAP: usize> Default for SpotState<R, SLOT_CAP, HIST_CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, const SLOT_CAP: usize, const HIST_CAP: usize> SpotState<R, SLOT_CAP, HIST_CAP> {
    /// `const` so the app-wide `Mutex<…>` statics can keep their
    /// `Mutex::new(State::new())` form.
    pub const fn new() -> Self {
        Self {
            slot: Vec::new(),
            history: Deque::new(),
            last_slot_hhmm: None,
            last_slot_count: 0,
            dirty_seq: AtomicU32::new(0),
        }
    }

    /// Render-side dirty check: readers compare against their
    /// last-seen value and skip the LCD push when it has not moved.
    pub fn dirty_seq(&self) -> u32 {
        self.dirty_seq.load(Ordering::Acquire)
    }

    /// Mark the state changed. Public because each mode's status
    /// fields live outside this type and still have to bump it.
    pub fn bump(&self) {
        self.dirty_seq.fetch_add(1, Ordering::AcqRel);
    }

    pub fn rows(&self) -> &[R] {
        &self.slot
    }

    /// `(shown, decoded_before_truncation)`.
    pub fn counts(&self) -> (usize, usize) {
        (self.slot.len(), self.last_slot_count)
    }

    pub fn last_slot_hhmm(&self) -> Option<&str> {
        self.last_slot_hhmm.as_deref()
    }

    /// Oldest first — `Deque`'s natural order. The renderer decides
    /// how to slice or reverse it.
    pub fn history_iter(&self) -> impl Iterator<Item = &R> {
        self.history.iter()
    }

    pub fn history_len(&self) -> usize {
        self.history.len()
    }
}

impl<R: Clone, const SLOT_CAP: usize, const HIST_CAP: usize> SpotState<R, SLOT_CAP, HIST_CAP> {
    /// Replace this slot's rows and append them to the history,
    /// dropping the oldest history entries once full.
    pub fn set_slot(&mut self, hhmm: &str, rows: &[R]) {
        self.slot.clear();
        for r in rows.iter().take(SLOT_CAP) {
            // Cannot fail: `take(SLOT_CAP)` is the capacity.
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
        self.bump();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type S = SpotState<u8, 4, 6>;

    #[test]
    fn truncates_the_slot_but_reports_the_true_count() {
        let mut s = S::new();
        s.set_slot("0102", &[1, 2, 3, 4, 5, 6]);
        assert_eq!(s.rows(), &[1, 2, 3, 4]);
        // The header shows 4/6 — a truncated slot must not read as a
        // quiet band.
        assert_eq!(s.counts(), (4, 6));
    }

    #[test]
    fn history_keeps_every_row_and_rolls_the_oldest_out() {
        let mut s = S::new();
        s.set_slot("0102", &[1, 2, 3, 4, 5]);
        // All five entered the history even though only four are shown.
        assert_eq!(
            s.history_iter().copied().collect::<std::vec::Vec<_>>(),
            [1, 2, 3, 4, 5]
        );
        s.set_slot("0104", &[6, 7, 8]);
        // Capacity 6: the two oldest are gone, order preserved.
        assert_eq!(
            s.history_iter().copied().collect::<std::vec::Vec<_>>(),
            [3, 4, 5, 6, 7, 8]
        );
    }

    #[test]
    fn hhmm_is_kept_short_rather_than_dropped() {
        let mut s = S::new();
        s.set_slot("010203", &[1]);
        assert_eq!(s.last_slot_hhmm(), Some("0102"));
        let mut s2 = S::new();
        s2.set_slot("01", &[1]);
        assert_eq!(s2.last_slot_hhmm(), Some("01"));
    }

    #[test]
    fn dirty_seq_moves_on_every_mutation() {
        let mut s = S::new();
        let a = s.dirty_seq();
        s.set_slot("0102", &[1]);
        let b = s.dirty_seq();
        assert!(b > a, "set_slot must bump");
        s.bump();
        assert!(s.dirty_seq() > b, "status updates must bump too");
    }
}
