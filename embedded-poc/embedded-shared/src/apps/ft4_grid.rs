// SPDX-License-Identifier: GPL-3.0-or-later
//! The FT4 capture window's place on the slot grid — the arithmetic
//! half of [`SlotAccum`](super::ft4_rx::SlotAccum), with no DSP and no
//! ESP-IDF in it.
//!
//! Split out so it can be tested. `ft4_rx` pulls in `esp_idf_svc` for
//! `esp_timer_get_time` and spawns a second core, so it only compiles
//! for Xtensa, and `embedded-poc` is outside the host workspace with
//! neither CI lint nor CI test reaching it (`embedded-poc/CLAUDE.md`
//! opens on exactly that). What was left checking this was a live run
//! against a radio, which is the most expensive instrument available
//! and the last one to be pointed at an off-by-one.
//!
//! Everything here is `usize`/`i32` over sample counts, so
//! `hosttest/mfsk-app-shared` compiles it and the cases below run in
//! CI. The window length, the grid period and the re-anchor threshold
//! are constructor arguments rather than constants: their values and
//! the measurements behind them stay in `ft4_rx`, and a test can use
//! small round numbers.
//!
//! # Against `mfsk_app_shared::capture_window::CaptureWindow`
//!
//! The two overlap and are not the same. `CaptureWindow` decides what
//! to do with a batch given a phase reading and is re-armed by its
//! caller each window; this carries the phase itself — a pending
//! correction across window closes, and a threshold below which the
//! clock is ignored in favour of the DT-median trim. Its doc records
//! the position that folding FT4 onto it would be a consolidation
//! rather than a fix, and nothing here changes that: this is the same
//! arithmetic `SlotAccum` already ran, moved where it can be tested.
//!
//! One claim there is worth checking against the code before it is
//! relied on: FT4 does not capture a contiguous grid. `SlotAccum`
//! discards `SLOT_SAMPLES - CAPTURE_CLOSE_SAMPLES` — 8 700 samples,
//! 0.725 s — between windows, because the capture closes at 6.775 s of
//! a 7.5 s slot. FT8 is the contiguous one.
//!
//! # The two reference frames
//!
//! Every phase figure here is counted **from where the accumulator
//! sits in the sample stream**, never from the wall clock. The two are
//! not the same point: audio arrives while the decoder is busy, so the
//! accumulator is normally behind real time by whatever is staged and
//! not yet fed. On FT4 that gap is ~1.4 s once per slot — the decode
//! budget — which is larger than the whole re-anchor threshold and
//! comparable to the Δt search itself.
//!
//! [`SlotGrid::anchor_or_reanchor`] therefore takes
//! `samples_to_boundary_from_here`, and it is the caller's job to
//! convert: a clock that says the boundary is `remain` samples away
//! from *now* puts it `remain + staged_not_yet_fed` samples away from
//! the accumulator.

/// Where the capture window sits on the slot grid, and what is pending
/// against it.
#[derive(Clone, Copy, Debug)]
pub struct SlotGrid {
    window: usize,
    period: usize,
    reanchor_thresh: i32,
    /// Samples of the current window already filled.
    filled: usize,
    /// Samples still to be discarded before the next window opens.
    skip: usize,
    /// Signed correction folded into `skip` the next time a window
    /// closes — from the UTC drift check and from the DT-median trim
    /// alike.
    pending_shift: i32,
    aligned: bool,
}

impl SlotGrid {
    /// `window` is the capture window in samples, `period` the slot
    /// grid's period (window plus the inter-window gap), and
    /// `reanchor_thresh` the phase error past which
    /// [`anchor_or_reanchor`](Self::anchor_or_reanchor) moves the grid
    /// rather than leaving the wobble to the DT trim.
    pub fn new(window: usize, period: usize, reanchor_thresh: i32) -> Self {
        Self {
            window,
            period,
            reanchor_thresh,
            filled: 0,
            skip: 0,
            pending_shift: 0,
            aligned: false,
        }
    }

    /// Whether an external clock has set the grid's phase at least
    /// once. Until it has, the grid free-runs from the first sample the
    /// accumulator ever saw — which is what a receiver replaying a
    /// recording, or one with no clock at all, is left with.
    pub fn is_aligned(&self) -> bool {
        self.aligned
    }

    /// Samples from the accumulator's position to the next window
    /// opening, given where it sits in its skip / fill / skip cycle.
    /// The phase [`anchor_or_reanchor`](Self::anchor_or_reanchor)
    /// compares against the clock's.
    pub fn samples_to_next_window_open(&self) -> i32 {
        if self.skip > 0 {
            self.skip as i32
        } else {
            // Mid-window (or opening now): finish filling it, then the
            // inter-window skip.
            (self.window - self.filled) as i32 + (self.period - self.window) as i32
        }
    }

    /// Set — or, once set, trim — the grid's phase from an external
    /// clock.
    ///
    /// `samples_to_boundary_from_here` is counted from the
    /// **accumulator's** position, not from now; see the module doc for
    /// why the distinction is load-bearing rather than pedantic.
    ///
    /// The first call anchors the grid outright: the next window opens
    /// on that boundary. Later calls move it only when the phase has
    /// drifted past the re-anchor threshold, which is what absorbs the
    /// clock stepping when NTP first disciplines an RTC-seeded clock —
    /// that step can be seconds, well past what the Δt search could
    /// pull back. Jitter below the threshold is left for
    /// [`shift_next_window`](Self::shift_next_window).
    pub fn anchor_or_reanchor(&mut self, samples_to_boundary_from_here: usize) {
        if !self.aligned {
            // Exactly on a boundary reads as a whole period; that means
            // "open now", not "skip a slot".
            self.skip = samples_to_boundary_from_here % self.period;
            self.filled = 0;
            self.pending_shift = 0;
            self.aligned = true;
            return;
        }
        if let Some(delta) = self.phase_error(samples_to_boundary_from_here) {
            self.pending_shift += delta;
        }
    }

    /// The signed phase error the next [`anchor_or_reanchor`] would act
    /// on, or `None` when it is inside the threshold. Positive means
    /// the clock's boundary is later than the grid's — the grid is
    /// running early.
    ///
    /// Public because it is the number worth logging: it is the
    /// disagreement between the clock and the grid, which with a
    /// disciplined clock is the band's offset and with an RTC-seeded
    /// one is that chip's drift accumulating in view.
    pub fn phase_error(&self, samples_to_boundary_from_here: usize) -> Option<i32> {
        let period = self.period as i32;
        let want = (samples_to_boundary_from_here % self.period) as i32;
        let have = self.samples_to_next_window_open();
        // Normalised to (−½ period, +½ period]: the sign says which way
        // the grid is off, not how many slots.
        let mut delta = (want - have) % period;
        if delta > period / 2 {
            delta -= period;
        } else if delta <= -period / 2 {
            delta += period;
        }
        (delta.abs() > self.reanchor_thresh).then_some(delta)
    }

    /// Fold a signed correction into the next inter-window gap — the
    /// DT-median trim. Positive delays the next window: the slot opened
    /// early, so the decoded signals sat late in it (WSJT-X's DT sign).
    pub fn shift_next_window(&mut self, delta_samples: i32) {
        self.pending_shift += delta_samples;
    }

    /// How many of `avail` samples to discard before the next window
    /// opens. Consumes them.
    pub fn take_skip(&mut self, avail: usize) -> usize {
        let take = self.skip.min(avail);
        self.skip -= take;
        take
    }

    /// Room left in the current window.
    pub fn room(&self) -> usize {
        self.window - self.filled
    }

    /// Advance the window by `n` filled samples. Returns `true` on the
    /// call that closes it, having already set the next gap.
    ///
    /// `n` must not exceed [`room`](Self::room) — the caller splits a
    /// block on the boundary, which is what keeps its block size from
    /// shifting the grid.
    pub fn fill(&mut self, n: usize) -> bool {
        self.filled += n;
        debug_assert!(self.filled <= self.window);
        if self.filled < self.window {
            return false;
        }
        self.filled = 0;
        // The inter-window gap, plus whatever correction the clock
        // check and the DT trim asked for. A correction too big for one
        // gap to hold is clamped and the remainder carried into the
        // next, so a shift larger than the gap takes two boundaries
        // instead of being silently truncated.
        let want_skip =
            (self.period - self.window) as i32 + core::mem::take(&mut self.pending_shift);
        self.skip = want_skip.clamp(0, self.period as i32) as usize;
        self.pending_shift = want_skip - self.skip as i32;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Round numbers in the same proportions as FT4's own: a window
    // that is most of the period, and a threshold well under the gap.
    const WINDOW: usize = 800;
    const PERIOD: usize = 1_000;
    const THRESH: i32 = 20;

    fn aligned_grid() -> SlotGrid {
        let mut g = SlotGrid::new(WINDOW, PERIOD, THRESH);
        // Anchor with the boundary right here: the window opens now.
        g.anchor_or_reanchor(0);
        assert!(g.is_aligned());
        assert_eq!(
            g.take_skip(PERIOD),
            0,
            "an anchor at 0 must not skip a slot"
        );
        g
    }

    /// Finish whatever window is open, then discard the gap that
    /// follows it. Returns the samples that gap actually consumed —
    /// which is where a pending trim becomes visible, since a trim is
    /// folded in when a window *closes*, not onto a gap already set.
    fn one_cycle(g: &mut SlotGrid) -> usize {
        g.take_skip(PERIOD);
        let room = g.room();
        assert!(g.fill(room), "filling the room must close the window");
        g.take_skip(PERIOD)
    }

    #[test]
    fn an_anchored_grid_holds_its_period() {
        let mut g = aligned_grid();
        for _ in 0..4 {
            assert_eq!(one_cycle(&mut g), PERIOD - WINDOW);
        }
    }

    /// The case a live run would otherwise be the first thing to check:
    /// a clock figure that was measured at "now" while the accumulator
    /// is a decode's worth of audio behind it.
    #[test]
    fn a_boundary_counted_from_now_is_not_a_boundary_counted_from_here() {
        // A decode's backlog, in the same proportion FT4 has: the
        // window is 800 of a 1 000 period and the decode holds ~1.4 s
        // of a 7.5 s slot, so ~190 here.
        const BACKLOG: usize = 190;

        let mut g = aligned_grid();
        one_cycle(&mut g);
        // Re-open and close one more window so the grid is sitting in
        // its gap, which is where the caller checks the clock.
        let room = g.room();
        g.fill(room);
        let here = g.samples_to_next_window_open();
        assert_eq!(here, (PERIOD - WINDOW) as i32);

        // The clock is right and the grid is right, but the clock was
        // read `BACKLOG` samples further down the stream than the
        // accumulator has reached.
        let remain_from_now = (here as usize + PERIOD - BACKLOG) % PERIOD;

        // Passed as-is, it reads as a phase error of exactly the
        // backlog — every slot, in the same direction, on a grid that
        // was not drifting.
        let mut naive = g;
        assert_eq!(naive.phase_error(remain_from_now), Some(-(BACKLOG as i32)));
        naive.anchor_or_reanchor(remain_from_now);
        // A trim lands at the *next* window close, not on the gap
        // already set, so run one cycle to see it.
        assert_eq!(
            one_cycle(&mut naive),
            (PERIOD - WINDOW) - BACKLOG,
            "the grid walks by the backlog, and would again next slot"
        );

        // Converted to this frame first, it reads as no error.
        let mut correct = g;
        assert_eq!(correct.phase_error(remain_from_now + BACKLOG), None);
        correct.anchor_or_reanchor(remain_from_now + BACKLOG);
        assert_eq!(one_cycle(&mut correct), PERIOD - WINDOW);
    }

    /// The same conversion at the *first* anchor, where getting it
    /// wrong is worse: nothing bounds it, the grid simply opens in the
    /// wrong place.
    #[test]
    fn the_first_anchor_is_counted_from_here_too() {
        const BACKLOG: usize = 190;
        let mut g = SlotGrid::new(WINDOW, PERIOD, THRESH);
        // The boundary is 300 samples ahead of *now*, and the
        // accumulator is BACKLOG behind now.
        g.anchor_or_reanchor(300 + BACKLOG);
        assert_eq!(g.take_skip(PERIOD), 300 + BACKLOG);
    }

    #[test]
    fn a_step_larger_than_the_threshold_moves_the_grid_and_jitter_does_not() {
        let mut g = aligned_grid();
        one_cycle(&mut g);
        let room = g.room();
        g.fill(room);
        let here = g.samples_to_next_window_open() as usize;

        // Inside the threshold: left to the DT trim.
        let mut jitter = g;
        jitter.anchor_or_reanchor(here + (THRESH as usize));
        assert_eq!(one_cycle(&mut jitter), PERIOD - WINDOW);

        // Past it: the next gap moves by exactly the error.
        let mut step = g;
        step.anchor_or_reanchor(here + 100);
        assert_eq!(one_cycle(&mut step), (PERIOD - WINDOW) + 100);
    }

    /// The wrap is the point: a boundary just *behind* the grid must
    /// read as a small negative error, not as nearly a whole period.
    #[test]
    fn phase_error_takes_the_short_way_round() {
        let mut g = aligned_grid();
        one_cycle(&mut g);
        let room = g.room();
        g.fill(room);
        let here = g.samples_to_next_window_open() as usize;

        assert_eq!(g.phase_error(here + PERIOD - 100), Some(-100));
        assert_eq!(g.phase_error(here + 100), Some(100));
        // And the far side of the period is the same point.
        assert_eq!(g.phase_error(here + PERIOD), None);
    }

    /// A correction bigger than one gap can hold is carried, not lost.
    #[test]
    fn an_oversized_shift_is_carried_to_the_next_boundary() {
        let mut g = aligned_grid();
        let gap = (PERIOD - WINDOW) as i32;
        // Ask to pull the next window in by more than the whole gap.
        g.shift_next_window(-gap - 50);
        assert_eq!(one_cycle(&mut g), 0, "the gap cannot go negative");
        // The remainder lands on the following boundary.
        assert_eq!(one_cycle(&mut g), (gap - 50) as usize);
        // And then it is spent.
        assert_eq!(one_cycle(&mut g), gap as usize);
    }

    /// A block straddling the boundary must not shift the grid, which
    /// is why the caller splits it.
    #[test]
    fn filling_in_pieces_is_filling_in_one_go() {
        let mut whole = aligned_grid();
        let mut pieces = aligned_grid();
        assert!(whole.fill(WINDOW));
        for _ in 0..(WINDOW / 8) {
            assert!(!pieces.fill(8) || pieces.room() == WINDOW);
        }
        assert_eq!(whole.room(), pieces.room());
        assert_eq!(whole.take_skip(PERIOD), pieces.take_skip(PERIOD));
    }
}
