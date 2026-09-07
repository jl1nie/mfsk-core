// SPDX-License-Identifier: GPL-3.0-or-later
//! A capture window that opens on a UTC slot boundary.
//!
//! A receiver taking audio from a USB stream is handed samples in
//! whatever batches the reader thread produces, and has to decide for
//! each batch how much of it belongs to the slot being captured, how
//! much is the gap before the next one, and where the boundary between
//! the two falls. That bookkeeping is the same for every mode whose
//! capture is shorter than its slot — WSPR captures 114 s of a 120 s
//! slot — and getting it wrong is not visible in a log line: the
//! decoder still decodes, the spots are just filed against the wrong
//! two minutes.
//!
//! Written for `wspr_app`'s DDC sink (#313 item 1). All three
//! receivers anchor to UTC now, each with its own bookkeeping: FT8's
//! is in `m5stack-cores3-app`'s `uac.rs` (`Ft8ChunkSink`), FT4's in
//! `embedded_shared::apps::ft4_rx::SlotAccum` (`anchor_or_reanchor`,
//! #354). Both of those capture a *contiguous* grid — every sample
//! belongs to some slot — which is why neither needed the gap this
//! type exists to manage. Folding them onto it would be a
//! consolidation, not a fix, and `uac.rs`'s version is
//! hardware-verified, so neither is changed here.
//!
//! The phase itself comes from [`crate::time_sync`]; this type only
//! decides what to do with a batch given a phase reading, which is
//! what makes it testable off-target.

/// What to do with the front of the batch just handed to the sink.
///
/// Always describes a prefix, never the whole batch: a batch that
/// straddles a boundary produces one step per side, so the caller
/// loops until the batch is empty.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Discard this many samples — the gap before the window opens.
    Skip(usize),
    /// Capture this many samples.
    Capture {
        n: usize,
        /// This step opens a window: stamp the slot's start time.
        opens: bool,
        /// This step fills the window: flush the slot, then re-arm.
        closes: bool,
    },
}

/// Tracks one capture window at a time: idle until the boundary,
/// capture `capture_samples`, idle again.
#[derive(Debug)]
pub struct CaptureWindow {
    capture_samples: usize,
    skip_remaining: usize,
    fed: usize,
}

impl CaptureWindow {
    /// A window of `capture_samples`, starting idle with nothing to
    /// skip — the first [`arm`](Self::arm) sets the gap before the
    /// first capture.
    pub fn new(capture_samples: usize) -> Self {
        Self {
            capture_samples,
            skip_remaining: 0,
            fed: 0,
        }
    }

    /// Set the gap before the next window from a phase reading.
    ///
    /// `samples_to_boundary` is what
    /// [`crate::time_sync::samples_to_next_slot_12k`] returned —
    /// `None` when the clock is unset, in which case `fallback` is
    /// waited out instead. Callers pass the slot's idle tail there
    /// between windows (so the cadence stays a whole slot even with an
    /// arbitrary phase) and `0` before the first one, where delaying
    /// an already-unaligned capture buys nothing.
    ///
    /// Re-read the phase at every boundary rather than counting
    /// forward from the last one: an NTP step or the RTC's drift is
    /// then absorbed by one gap instead of accumulating.
    pub fn arm(&mut self, samples_to_boundary: Option<usize>, fallback: usize) {
        self.skip_remaining = samples_to_boundary.unwrap_or(fallback);
    }

    /// Whether a window is part-captured — `false` while idling and at
    /// the instant one closes.
    pub fn is_open(&self) -> bool {
        self.fed > 0
    }

    /// Samples captured into the current window so far.
    pub fn fed(&self) -> usize {
        self.fed
    }

    /// What to do with the first `avail` samples of the batch in hand.
    ///
    /// `avail` must be non-zero; the caller consumes the returned
    /// prefix and calls again with what is left.
    pub fn step(&mut self, avail: usize) -> Step {
        debug_assert!(avail > 0, "step called with an empty batch");
        if self.skip_remaining > 0 {
            let n = avail.min(self.skip_remaining);
            self.skip_remaining -= n;
            return Step::Skip(n);
        }
        let opens = self.fed == 0;
        let n = avail.min(self.capture_samples - self.fed);
        self.fed += n;
        let closes = self.fed >= self.capture_samples;
        if closes {
            self.fed = 0;
        }
        Step::Capture { n, opens, closes }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 114 s captured out of a 120 s slot at 12 kHz, WSPR's numbers.
    const CAPTURE: usize = 114 * 12_000;
    const IDLE: usize = 6 * 12_000;

    /// Feed `batch`-sized batches until `n` windows have closed,
    /// re-arming from `phase` each time, and return `(captured,
    /// skipped, opens, closes)`.
    fn run(
        w: &mut CaptureWindow,
        batch: usize,
        batches: usize,
        mut phase: impl FnMut() -> Option<usize>,
    ) -> (usize, usize, usize, usize) {
        let (mut cap, mut skip, mut opens, mut closes) = (0, 0, 0, 0);
        for _ in 0..batches {
            let mut left = batch;
            while left > 0 {
                match w.step(left) {
                    Step::Skip(n) => {
                        skip += n;
                        left -= n;
                    }
                    Step::Capture {
                        n,
                        opens: o,
                        closes: c,
                    } => {
                        cap += n;
                        left -= n;
                        opens += usize::from(o);
                        if c {
                            closes += 1;
                            w.arm(phase(), IDLE);
                        }
                    }
                }
            }
        }
        (cap, skip, opens, closes)
    }

    /// The bug this type exists for: back-to-back captures with no gap
    /// run a 114 s cadence against a 120 s grid.
    #[test]
    fn a_closed_window_does_not_reopen_until_re_armed() {
        let mut w = CaptureWindow::new(CAPTURE);
        w.arm(Some(0), 0);
        // Two whole slots' worth of audio, arriving 192 samples at a
        // time the way a USB read does.
        let (cap, skip, opens, closes) = run(&mut w, 192, 2 * 120 * 12_000 / 192, || Some(IDLE));
        assert_eq!(closes, 2, "two slots of audio, two windows");
        assert_eq!(opens, 2);
        assert_eq!(cap, 2 * CAPTURE, "captured exactly the window, twice");
        assert_eq!(skip, 2 * IDLE, "and idled the tail after each");
    }

    /// A batch that straddles the boundary is split, not rounded — the
    /// window is exactly `capture_samples`, whatever the batch size.
    #[test]
    fn a_straddling_batch_is_split_at_the_boundary() {
        let mut w = CaptureWindow::new(1_000);
        w.arm(Some(0), 0);
        assert_eq!(
            w.step(600),
            Step::Capture {
                n: 600,
                opens: true,
                closes: false
            }
        );
        // 400 left in the window, 200 of this batch past it.
        assert_eq!(
            w.step(600),
            Step::Capture {
                n: 400,
                opens: false,
                closes: true
            }
        );
        w.arm(Some(300), 0);
        assert_eq!(w.step(200), Step::Skip(200));
        assert_eq!(w.step(500), Step::Skip(100));
        assert_eq!(
            w.step(400),
            Step::Capture {
                n: 400,
                opens: true,
                closes: false
            }
        );
    }

    /// With no clock the cadence still has to be a whole slot — the
    /// phase is arbitrary, the period is not.
    #[test]
    fn without_a_clock_the_fallback_gap_keeps_the_cadence() {
        let mut w = CaptureWindow::new(CAPTURE);
        // First window starts immediately; then exactly three windows
        // and the two gaps between them.
        w.arm(None, 0);
        let total = 3 * CAPTURE + 2 * IDLE;
        let (cap, skip, opens, closes) = run(&mut w, 1_000, total / 1_000, || None);
        assert_eq!((opens, closes), (3, 3));
        assert_eq!(cap, 3 * CAPTURE);
        assert_eq!(skip, 2 * IDLE, "one gap between each pair of windows");
    }

    /// A clock that appears mid-run (NTP associating late) re-phases
    /// the grid at the next boundary, at the cost of one longer gap —
    /// and does not disturb the window length.
    #[test]
    fn a_late_clock_re_phases_at_the_next_boundary() {
        /// What the clock reports once it exists: most of a slot to
        /// wait, because the free-running phase was most of a slot out.
        const LATE: usize = 47 * 12_000;

        let mut w = CaptureWindow::new(CAPTURE);
        w.arm(None, 0);
        let mut armed = 0;
        // Two windows and the two gaps that follow them: the first gap
        // is the clockless fallback, the second is measured from UTC.
        let total = 2 * CAPTURE + IDLE + LATE;
        let (cap, skip, opens, closes) = run(&mut w, 1_000, total / 1_000, || {
            armed += 1;
            (armed >= 2).then_some(LATE)
        });
        assert_eq!((opens, closes), (2, 2));
        assert_eq!(cap, 2 * CAPTURE, "windows stay exactly one window long");
        assert_eq!(
            skip,
            IDLE + LATE,
            "the fallback gap first, then the UTC-measured one"
        );
    }

    /// Re-arming to zero while a window is open must not reopen it
    /// mid-slot: `arm` sets the *gap*, and an open window is finished
    /// first.
    #[test]
    fn arming_mid_window_does_not_truncate_it() {
        let mut w = CaptureWindow::new(1_000);
        w.arm(Some(0), 0);
        assert!(matches!(w.step(400), Step::Capture { n: 400, .. }));
        assert!(w.is_open());
        assert_eq!(w.fed(), 400);
        w.arm(Some(0), 0);
        assert_eq!(
            w.step(1_000),
            Step::Capture {
                n: 600,
                opens: false,
                closes: true
            }
        );
    }
}
