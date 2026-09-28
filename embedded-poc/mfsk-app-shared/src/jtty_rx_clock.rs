//! The JTTY receiver's sample clock on a board: what arrives between the
//! audio source and the front end, and where samples went missing.
//!
//! **The sample count is JTTY's only clock** (`docs/notes/JTTY_CORES3_APP.md`
//! §4): frames continue a message by their spacing in samples, so a run of
//! samples lost without trace shifts every later frame and breaks the
//! message. Two ways they go missing, and what each does here:
//!
//! - **The front end fell behind** and the staging ring filled:
//!   [`Staging::push`] keeps a *positioned* gap — how many samples, and
//!   where in the stream — so [`Staging::drain_into`] puts zeros exactly
//!   where the audio was lost, not a count somewhere else.
//! - **They never arrived** (reader timeouts, dropped isochronous frames, a
//!   flash write stalling both cores — none of it visible in the stream):
//!   [`SampleClock`] compares samples delivered against the time they took
//!   and asks for zeros when a deficit opens, or a reset past ~1 s.
//!
//! A clock that is merely *slow or fast* is not a gap. A radio's audio
//! clock and this board's timer disagree by parts in 10⁴ (12 003.9 sa/s was
//! measured on an IC-705), and zeros inserted every few seconds to chase
//! that would cut holes in frames that were fine. So the deficit is taken
//! against a baseline that follows a steady drift of up to
//! [`DRIFT_PPM`], and only a jump past that baseline is filled.
//!
//! Pure bookkeeping over sample counts and `esp_timer` microseconds passed
//! in, so it is tested in `hosttest/mfsk-app-shared`.

extern crate alloc;
use alloc::vec::Vec;

/// Samples a second.
pub const FS: i64 = 12_000;

/// A deficit past the drift baseline worth filling with zeros: 20 ms
/// (`JTTY_CORES3_APP.md` §4). Below this is delivery jitter — a UAC read
/// or a SIM block is ~21 ms of audio handed over at once, *after* it was
/// recorded, and the deficit is only read right after a hand-over.
pub const FILL_SAMPLES: i64 = FS / 50;

/// A deficit past which the stream is not repaired but restarted: ~1 s
/// (§4). Zeros covering longer than that would stand in for most of a
/// frame, and the receiver's message state is better rebuilt.
pub const RESET_SAMPLES: i64 = FS;

/// How fast the baseline may follow a deficit that grows steadily — a
/// source clock slower than the board's. 1 000 ppm, ~3× the largest
/// disagreement measured between an IC-705 and this board.
pub const DRIFT_PPM: i64 = 1_000;

/// What [`SampleClock::delivered`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// On time, or late by no more than jitter and drift explain.
    Ok,
    /// This many samples are missing just before what was delivered: put
    /// that many zeros in, then [`SampleClock::filled`].
    Fill(usize),
    /// Too much is missing to fill: restart the stream.
    Reset,
}

/// Samples delivered against the time they took.
#[derive(Clone, Debug, Default)]
pub struct SampleClock {
    /// `esp_timer` µs of the stream's sample 0, from the first hand-over.
    t0_us: Option<i64>,
    /// Samples handed over, zeros filled included.
    delivered: i64,
    /// The deficit a steady drift accounts for, in thousandths of a
    /// sample: a block's worth of drift allowance is ~0.26 samples, which
    /// whole samples would round away.
    base_milli: i64,
    last_us: i64,
}

impl SampleClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// `esp_timer` µs of the stream's sample 0; `None` before the first
    /// hand-over.
    pub fn t0_us(&self) -> Option<i64> {
        self.t0_us
    }

    /// Record `n` samples handed over at `now_us`, the moment the last of
    /// them had been recorded.
    pub fn delivered(&mut self, n: usize, now_us: i64) -> Verdict {
        let n = n as i64;
        let t0 = *self.t0_us.get_or_insert(now_us - n * 1_000_000 / FS);
        self.delivered += n;
        let expected = (now_us - t0) * FS / 1_000_000;
        let deficit_milli = (expected - self.delivered) * 1_000;
        if deficit_milli < self.base_milli {
            // Ahead of the timer (a fast source, or a burst catching up):
            // the baseline comes straight down to meet it.
            self.base_milli = deficit_milli;
        } else {
            let allowed = (now_us - self.last_us).max(0) * FS * DRIFT_PPM / 1_000_000_000;
            self.base_milli += (deficit_milli - self.base_milli).min(allowed);
        }
        self.last_us = now_us;
        let gap = (deficit_milli - self.base_milli) / 1_000;
        if gap > RESET_SAMPLES {
            Verdict::Reset
        } else if gap > FILL_SAMPLES {
            Verdict::Fill(gap as usize)
        } else {
            Verdict::Ok
        }
    }

    /// Count `n` zeros as delivered, after a [`Verdict::Fill`].
    pub fn filled(&mut self, n: usize) {
        self.delivered += n as i64;
    }
}

/// Samples waiting for the front end, with the gaps where some were lost.
#[derive(Clone, Debug)]
pub struct Staging {
    buf: Vec<i16>,
    cap: usize,
    /// `(position in buf, zeros)`, in order: the zeros go *before*
    /// `buf[position]`.
    gaps: Vec<(usize, usize)>,
}

impl Staging {
    /// Room for `cap` samples of audio (gaps take none of it).
    pub fn new(cap: usize) -> Self {
        Self {
            buf: Vec::with_capacity(cap),
            cap,
            gaps: Vec::new(),
        }
    }

    /// Samples held.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty() && self.gaps.is_empty()
    }

    /// Append audio. What does not fit becomes a gap at the end — the
    /// samples are gone, but not where they were. Returns how many were
    /// lost that way.
    pub fn push(&mut self, samples: &[i16]) -> usize {
        let take = samples.len().min(self.cap - self.buf.len());
        self.buf.extend_from_slice(&samples[..take]);
        let lost = samples.len() - take;
        if lost > 0 {
            self.gap(lost);
        }
        lost
    }

    /// `n` samples missing at the current end.
    pub fn gap(&mut self, n: usize) {
        let pos = self.buf.len();
        match self.gaps.last_mut() {
            Some((p, z)) if *p == pos => *z += n,
            _ => self.gaps.push((pos, n)),
        }
    }

    /// Move everything to `out` in stream order, the gaps as zeros, and
    /// empty this. Returns the zeros written.
    pub fn drain_into(&mut self, out: &mut Vec<i16>) -> usize {
        let mut zeros = 0;
        let mut from = 0;
        for &(pos, n) in &self.gaps {
            out.extend_from_slice(&self.buf[from..pos]);
            out.resize(out.len() + n, 0);
            zeros += n;
            from = pos;
        }
        out.extend_from_slice(&self.buf[from..]);
        self.buf.clear();
        self.gaps.clear();
        zeros
    }

    /// Discard everything, for a stream reset.
    pub fn clear(&mut self) {
        self.buf.clear();
        self.gaps.clear();
    }
}
