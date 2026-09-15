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
    /// How much of `pending_shift` the clock check itself has queued
    /// for the window now in progress. Subtracted in
    /// [`SlotGrid::phase_error`] so that asking again inside the same
    /// window sees the error as already corrected, and cleared by
    /// [`SlotGrid::fill`] when the correction lands in `skip`.
    ///
    /// The receiver asks once per audio block — a UAC read, ~21 ms, so
    /// ~320 times per FT4 window — and the error is a function of the
    /// grid's position alone, so without this the same disagreement is
    /// queued on every one of those calls.
    clock_trim: i32,
    aligned: bool,
}

/// What [`SlotGrid::anchor_or_reanchor`] did, for a caller that holds
/// the audio the grid is only counting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub enum Anchor {
    /// The phase was set or trimmed without moving the window now
    /// filling: nothing to do. A trim queued here lands at the next
    /// window close, which is what keeps windows exactly one window
    /// long.
    Kept,
    /// The grid re-phased under a part-filled window, so whatever the
    /// caller has accumulated for it straddles the new boundary and
    /// must be dropped. Only the first anchor does this: a later trim
    /// goes through `pending_shift` instead.
    DiscardPartial,
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
            clock_trim: 0,
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
    ///
    /// The return value matters only to a caller that is accumulating
    /// audio against this grid: [`Anchor::DiscardPartial`] says the
    /// window it was filling no longer exists. The grid counts
    /// samples and the caller holds them, so only the caller can throw
    /// them away — and it has to, because the first anchor re-phases
    /// under a window that may be part-filled from before there was a
    /// clock at all (`time_sync` reports no phase until the RTC or NTP
    /// lands, while UAC audio arrives regardless).
    ///
    /// A correction the DT-median trim had queued before the first
    /// anchor is dropped with that window: it was measured against a
    /// free-running phase, on audio that is no longer going to be
    /// decoded.
    pub fn anchor_or_reanchor(&mut self, samples_to_boundary_from_here: usize) -> Anchor {
        if !self.aligned {
            // Exactly on a boundary reads as a whole period; that means
            // "open now", not "skip a slot".
            self.skip = samples_to_boundary_from_here % self.period;
            let had_partial = self.filled > 0;
            self.filled = 0;
            self.pending_shift = 0;
            self.clock_trim = 0;
            self.aligned = true;
            return if had_partial {
                Anchor::DiscardPartial
            } else {
                Anchor::Kept
            };
        }
        if let Some(delta) = self.phase_error(samples_to_boundary_from_here) {
            self.pending_shift += delta;
            self.clock_trim += delta;
        }
        Anchor::Kept
    }

    /// The signed phase error the next
    /// [`anchor_or_reanchor`](Self::anchor_or_reanchor) would act on,
    /// or `None` when it is inside the threshold. Positive means the
    /// clock's boundary is later than the grid's — the grid is running
    /// early.
    ///
    /// **Net of what is already queued.** A correction this cycle's
    /// clock check has already folded into `pending_shift` is
    /// subtracted, so the answer is what remains uncorrected rather
    /// than the raw disagreement. That is what makes it safe to ask
    /// per audio block: the first call reports the error and queues
    /// it, and the rest of the window reports `None`. The DT-median
    /// trim is deliberately *not* subtracted — it is small by
    /// construction and well inside the threshold, so it never reads
    /// as a phase error on its own.
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
        let residual = delta - self.clock_trim;
        (residual.abs() > self.reanchor_thresh).then_some(residual)
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
        // What the gap could not hold stays owed, and the next window
        // will measure it again — the grid really is still that far off.
        // So carry it as `clock_trim` rather than clearing: the clock's
        // contribution has been spent only to the extent it was
        // applied. Zeroing here let the next window queue the carried
        // part a second time, on top of the carry, and the grid then
        // overshot by exactly that amount before settling (#376).
        //
        // With nothing clamped this is 0, which is the ordinary case:
        // the gap absorbed the whole correction, the next window
        // measures the disagreement afresh, and drift keeps being
        // corrected without one reading being applied twice.
        //
        // Approximate only in that `pending_shift` may also hold a
        // DT-median trim, which is not a clock error. That trim is
        // small by construction — well inside `reanchor_thresh`, which
        // is why `phase_error` deliberately does not subtract it — and
        // so never causes the clamp; what it can do is leave the
        // residual off by its own size, which stays under the
        // threshold and queues nothing.
        self.clock_trim = self.pending_shift;
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
        assert_eq!(g.anchor_or_reanchor(0), Anchor::Kept);
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
        assert_eq!(naive.anchor_or_reanchor(remain_from_now), Anchor::Kept);
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
        assert_eq!(
            correct.anchor_or_reanchor(remain_from_now + BACKLOG),
            Anchor::Kept
        );
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
        assert_eq!(g.anchor_or_reanchor(300 + BACKLOG), Anchor::Kept);
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
        assert_eq!(
            jitter.anchor_or_reanchor(here + (THRESH as usize)),
            Anchor::Kept
        );
        assert_eq!(one_cycle(&mut jitter), PERIOD - WINDOW);

        // Past it: the next gap moves by exactly the error.
        let mut step = g;
        assert_eq!(step.anchor_or_reanchor(here + 100), Anchor::Kept);
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

    /// The receiver asks once per audio block — a UAC read, ~21 ms,
    /// so ~320 times across an FT4 window — and the error is a
    /// function of the grid's position, so every one of those calls
    /// answers the same number. It must be acted on once: an NTP step
    /// multiplied by the block count would skip whole slots, which is
    /// exactly the case the re-anchor branch exists to absorb.
    #[test]
    fn asking_every_block_queues_the_correction_once() {
        let mut g = aligned_grid();
        one_cycle(&mut g);
        let room = g.room();
        g.fill(room);
        let here = g.samples_to_next_window_open() as usize;

        assert_eq!(g.phase_error(here + 100), Some(100));
        for _ in 0..50 {
            assert_eq!(g.anchor_or_reanchor(here + 100), Anchor::Kept);
        }
        // Queued once. Reading as corrected from here on is also what
        // stops the caller logging the same trim on every block.
        assert_eq!(g.phase_error(here + 100), None);
        assert_eq!(
            one_cycle(&mut g),
            (PERIOD - WINDOW) + 100,
            "one correction, not fifty"
        );
        assert_eq!(one_cycle(&mut g), PERIOD - WINDOW);

        // Spent, not disabled: the next window measures afresh, which
        // is how a drifting RTC keeps being corrected.
        let room = g.room();
        g.fill(room);
        let here = g.samples_to_next_window_open() as usize;
        assert_eq!(g.phase_error(here + 60), Some(60));
    }

    /// A receiver has audio before it has a clock —
    /// `time_sync::samples_to_next_slot_12k_ms` reports nothing until
    /// the RTC or NTP lands, while UAC audio arrives regardless — so
    /// the first anchor commonly re-phases under a part-filled window.
    /// The grid counts samples and the caller holds them, so the grid
    /// has to say so: otherwise the caller's buffer keeps the
    /// pre-anchor audio and the window closes holding that *plus* a
    /// whole window.
    #[test]
    fn the_first_anchor_under_a_part_filled_window_discards_it() {
        let mut g = SlotGrid::new(WINDOW, PERIOD, THRESH);
        // Free-running, no clock yet: audio simply accumulates.
        assert!(!g.fill(300));
        assert_eq!(g.room(), WINDOW - 300);

        assert_eq!(g.anchor_or_reanchor(0), Anchor::DiscardPartial);
        assert_eq!(g.room(), WINDOW, "the window is whole again");
        assert_eq!(g.take_skip(PERIOD), 0, "and opens on the boundary");

        // Nothing part-filled, nothing to discard — the ordinary case,
        // and the one the caller must not pay an allocation for.
        let mut fresh = SlotGrid::new(WINDOW, PERIOD, THRESH);
        assert_eq!(fresh.anchor_or_reanchor(0), Anchor::Kept);
    }

    /// A trim queued before the first anchor was measured against a
    /// free-running phase, on audio that anchor is about to discard.
    /// It goes with it rather than being applied to the new grid.
    #[test]
    fn the_first_anchor_drops_a_trim_queued_before_it() {
        let mut g = SlotGrid::new(WINDOW, PERIOD, THRESH);
        g.shift_next_window(-120);
        assert_eq!(g.anchor_or_reanchor(0), Anchor::Kept);
        assert_eq!(
            one_cycle(&mut g),
            PERIOD - WINDOW,
            "the stale trim does not move the anchored grid"
        );
    }

    /// A block straddling the boundary must not shift the grid, which
    /// is why the caller splits it.
    #[test]
    fn filling_in_pieces_is_filling_in_one_go() {
        let mut whole = aligned_grid();
        let mut pieces = aligned_grid();
        assert!(whole.fill(WINDOW));
        let last = WINDOW / 8;
        for piece in 1..=last {
            assert_eq!(
                pieces.fill(8),
                piece == last,
                "piece {piece} of {last} must close the window only on the last"
            );
        }
        assert_eq!(whole.room(), pieces.room());
        assert_eq!(whole.take_skip(PERIOD), pieces.take_skip(PERIOD));
    }

    /// Fill one whole window the way the reader does — in blocks, with
    /// the clock asked before each — which is what makes a correction
    /// queued per block rather than per window visible at all.
    ///
    /// `clock_off` is where the clock's boundary sits relative to the
    /// grid's own: negative means the grid is running late.
    fn fill_window_asking_clock(g: &mut SlotGrid, clock_off: i32) {
        loop {
            let n = 8.min(g.room());
            let have = g.samples_to_next_window_open();
            let want = (have + clock_off).rem_euclid(PERIOD as i32) as usize;
            // Already aligned, so every call here is a trim: only the
            // first anchor may discard a window, and asserting it says
            // this helper never silently threw audio away.
            assert_eq!(g.anchor_or_reanchor(want), Anchor::Kept);
            if g.fill(n) {
                return;
            }
        }
    }

    /// A correction bigger than the gap is carried to the next close —
    /// and the next window measures the same error again, because the
    /// grid really is still that far off. Issue #376: `clock_trim` was
    /// cleared at the close regardless, so that second measurement
    /// queued the carried part a *second* time and the grid overshot by
    /// exactly it before settling two windows later.
    ///
    /// Reachable from the case the re-anchor branch exists for — NTP
    /// stepping an RTC-seeded clock by seconds. On FT4's own constants
    /// a −1 s step overshoots by 3 300 samples (0.275 s).
    #[test]
    fn a_carried_correction_is_not_queued_twice() {
        const GAP: i32 = (PERIOD - WINDOW) as i32;
        let mut g = aligned_grid();

        // Further than one gap can move (200), leaving 100 owed, which
        // is past the threshold (20) and so measurable again.
        let mut clock_off: i32 = -300;

        fill_window_asking_clock(&mut g, clock_off);
        let gap1 = g.take_skip(PERIOD) as i32;
        assert_eq!(gap1, 0, "the gap clamps at 0: it cannot move -300");
        clock_off -= gap1 - GAP;
        assert_eq!(clock_off, -100, "-100 still owed after the clamp");

        // The whole point: the grid is genuinely -100 off now, the
        // carry already holds -100, and applying it once lands exactly.
        fill_window_asking_clock(&mut g, clock_off);
        let gap2 = g.take_skip(PERIOD) as i32;
        assert_eq!(
            gap2 - GAP,
            -100,
            "the carried correction must be applied once, not twice"
        );
        clock_off -= gap2 - GAP;
        assert_eq!(clock_off, 0, "and the grid is on the clock");

        // Settled: no further correction, no oscillation.
        fill_window_asking_clock(&mut g, clock_off);
        assert_eq!(g.take_skip(PERIOD) as i32, GAP, "nothing left to correct");
    }
}
