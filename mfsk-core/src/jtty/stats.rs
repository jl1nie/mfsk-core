//! Work counters and stage timers for the receiver, for measuring what a window costs
//! (#499). Compiled only with the `jtty-stats` feature; without it the `stat_add!` and
//! `stat_time!` macros the receiver is instrumented with expand to nothing.
//!
//! Counters are `AtomicU64` and timers accumulate nanoseconds of wall clock, so this is a host
//! tool (a target without 64-bit atomics cannot build it). Run single-threaded (no
//! `parallel` feature) for stage times that add up: under rayon the stages overlap.

use core::sync::atomic::{AtomicU64, Ordering};

/// What is counted.
#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub enum Counter {
    /// Windows analysed as themselves.
    Windows,
    /// Windows analysed again with a signal subtracted (the retro re-sweep).
    RetroWindows,
    /// Analytic-signal computations (one 32 768-point FFT and one 16 384-point inverse each).
    Analytic,
    /// Sync surfaces built (237 columns of one 8 192-point FFT each).
    SurfaceBuilds,
    /// Peaks picked on channel 0.
    PicksCh0,
    /// Peaks picked on channels 1 and 2.
    PicksOther,
    /// Peak-up refinements (channel 0).
    Peakups,
    /// Candidates frequency-shifted to DC (each rotates a whole 14 160-sample window).
    Shifts,
    /// Candidates that passed the sync gate.
    GatePass,
    /// Candidates the sync gate rejected.
    GateFail,
    /// Payload correlations (full and half symbol).
    Correlations,
    /// Ladder calls.
    LadderCalls,
    /// Ladder calls that accepted a word.
    LadderAccepts,
    /// Frames subtracted from the signal.
    Subtractions,
    /// Sticky-sync retries.
    StickyRetries,
    /// Extra decode rounds after a subtraction.
    ExtraRounds,
    /// Candidates decoded again in those extra rounds.
    RetryCandidates,
    /// Ladder calls made by those candidates.
    RetryLadderCalls,
}
const N_COUNTERS: usize = 18;

/// What is timed. Stages do not nest.
#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub enum Stage {
    /// `analytic_6k`.
    Analytic,
    /// `sync_surface`.
    Surface,
    /// Picking peaks and suppressing around them.
    Pick,
    /// `peakup`.
    Peakup,
    /// Mixing the window to DC for a candidate.
    Shift,
    /// The 13-symbol sync gate.
    Gate,
    /// Payload correlation.
    Correlate,
    /// The decode ladder.
    Ladder,
    /// `subtract_frame`.
    Subtract,
}
const N_STAGES: usize = 9;

/// One candidate that passed the sync gate, and what the ladder made of it.
#[derive(Clone, Debug)]
pub struct GatedCandidate {
    /// Start of the window, seconds.
    pub window_s: f32,
    /// 0, 1 or 2.
    pub channel: u8,
    /// Frequency of the lowest tone after peak-up, Hz.
    pub f1_hz: f32,
    /// Start of the frame, seconds from the start of the audio.
    pub tsync_s: f32,
    /// Sync tones right, of 13.
    pub nsync: usize,
    /// S/N of the sync tones, dB.
    pub snr_db: f32,
    /// The ladder accepted a word.
    pub accepted: bool,
}

/// The counters and timers of one receiver.
#[derive(Default)]
pub struct Stats {
    counts: [AtomicU64; N_COUNTERS],
    ns: [AtomicU64; N_STAGES],
    gated: std::sync::Mutex<Vec<GatedCandidate>>,
}

/// A plain copy of the counters and timers.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// Indexed by [`Counter`].
    pub counts: [u64; N_COUNTERS],
    /// Nanoseconds, indexed by [`Stage`].
    pub ns: [u64; N_STAGES],
    /// Trellis rungs run: L=1, L=2, L=4, half-symbol L=1.
    pub rungs: [u64; 4],
}

impl Snapshot {
    /// One counter.
    pub fn count(&self, c: Counter) -> u64 {
        self.counts[c as usize]
    }
    /// One stage's seconds.
    pub fn seconds(&self, s: Stage) -> f64 {
        self.ns[s as usize] as f64 * 1e-9
    }
}

impl Stats {
    /// Add `n` to a counter.
    pub fn add(&self, c: Counter, n: u64) {
        self.counts[c as usize].fetch_add(n, Ordering::Relaxed);
    }
    /// Start timing a stage; the time is added when the guard drops.
    pub fn time(&self, s: Stage) -> Span<'_> {
        Span {
            slot: &self.ns[s as usize],
            t0: std::time::Instant::now(),
        }
    }
    /// Copy the values (the rung counts are the ladder's, filled in by the receiver).
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            counts: core::array::from_fn(|i| self.counts[i].load(Ordering::Relaxed)),
            ns: core::array::from_fn(|i| self.ns[i].load(Ordering::Relaxed)),
            rungs: [0; 4],
        }
    }
    /// Record a candidate that reached the ladder.
    pub fn record(&self, c: GatedCandidate) {
        self.gated.lock().unwrap().push(c);
    }
    /// Every candidate that reached the ladder since the last reset.
    pub fn gated(&self) -> Vec<GatedCandidate> {
        self.gated.lock().unwrap().clone()
    }
    /// Zero everything.
    pub fn reset(&self) {
        self.gated.lock().unwrap().clear();
        self.counts
            .iter()
            .for_each(|c| c.store(0, Ordering::Relaxed));
        self.ns.iter().for_each(|c| c.store(0, Ordering::Relaxed));
    }
}

/// A running stage timer.
pub struct Span<'a> {
    slot: &'a AtomicU64,
    t0: std::time::Instant,
}

impl Drop for Span<'_> {
    fn drop(&mut self) {
        self.slot
            .fetch_add(self.t0.elapsed().as_nanos() as u64, Ordering::Relaxed);
    }
}
