//! A waterfall of one channel's audio, fine enough to see the signals in it.
//!
//! The decode band of a channel is a few kHz of 12 kHz audio. A 4096-point
//! FFT gives 2.93 Hz bins (8192: 1.46 Hz), against FT8's 6.25 Hz tone spacing
//! and ~50 Hz signals, so neighbours that a 100 Hz-per-bin display runs
//! together stay apart. Rows come every `hop` samples (2048: 0.17 s).
//!
//! Each row is `u8`: dB above the row's own noise floor (its 30th percentile,
//! as `wsprd` takes it) plus a margin, mapped over [`RANGE_DB`]; noise is 0. That keeps the background
//! the same grey as the band fills and empties and the AGC moves, and a row
//! small enough to ship to a window six times a second.

use mfsk_core::engine::fft::{Fft, FftPlanner, RustFftPlanner};
use num_complex::Complex32;

/// The audio rate of a channel.
pub const RATE_HZ: f32 = 12_000.0;
/// dB between level 0 and full scale (255).
pub const RANGE_DB: f32 = 42.0;
/// Level 0 sits this far above the row's floor (its 30th percentile). Noise
/// has a median 2-3 dB above that percentile, so the background is black and
/// only what stands out of it takes a colour.
const FLOOR_MARGIN_DB: f32 = 3.0;

/// One spectrum row.
#[derive(Clone, Debug)]
pub struct Row {
    /// UTC of the middle of the row's window, ns.
    pub utc_ns: i64,
    /// Audio frequency of `levels[0]`, Hz.
    pub f_lo_hz: f32,
    /// Hz per `levels` entry.
    pub bin_hz: f32,
    pub levels: Vec<u8>,
}

impl Row {
    /// The same row `by` times coarser, each entry the strongest of `by`
    /// bins: a thumbnail has to keep a narrow signal visible.
    pub fn pooled(&self, by: usize) -> Row {
        let by = by.max(1);
        Row {
            utc_ns: self.utc_ns,
            f_lo_hz: self.f_lo_hz,
            bin_hz: self.bin_hz * by as f32,
            levels: self
                .levels
                .chunks(by)
                .map(|c| c.iter().copied().max().unwrap_or(0))
                .collect(),
        }
    }
}

pub struct Waterfall {
    n: usize,
    hop: usize,
    lo: usize,
    hi: usize,
    window: Vec<f32>,
    fft: Box<dyn Fft>,
    /// Audio not yet consumed by a row.
    pending: Vec<f32>,
    /// Samples pushed, and the index `pending[0]` has in that count.
    pushed: u64,
    start: u64,
    buf: Vec<Complex32>,
    db: Vec<f32>,
}

impl Waterfall {
    /// `n`-point FFT every `hop` samples, rows covering `lo_hz..hi_hz`.
    pub fn new(n: usize, hop: usize, lo_hz: f32, hi_hz: f32) -> Self {
        let bin = RATE_HZ / n as f32;
        let lo = ((lo_hz / bin).floor() as usize).min(n / 2 - 1);
        let hi = ((hi_hz / bin).ceil() as usize).clamp(lo + 1, n / 2);
        let window = (0..n)
            .map(|i| 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / n as f32).cos())
            .collect();
        Waterfall {
            n,
            hop,
            lo,
            hi,
            window,
            fft: RustFftPlanner::new().plan_forward(n),
            pending: Vec::new(),
            pushed: 0,
            start: 0,
            buf: vec![Complex32::new(0.0, 0.0); n],
            db: Vec::new(),
        }
    }

    pub fn bin_hz(&self) -> f32 {
        RATE_HZ / self.n as f32
    }

    /// Add audio; `end_utc_ns` is the UTC of its last sample, `None` while the
    /// clock is not set (the audio is dropped then, a row without a time is of
    /// no use). Complete rows go to `out`.
    pub fn push(&mut self, audio: &[f32], end_utc_ns: Option<i64>, out: &mut Vec<Row>) {
        self.pushed += audio.len() as u64;
        let Some(end) = end_utc_ns else {
            self.pending.clear();
            self.start = self.pushed;
            return;
        };
        self.pending.extend_from_slice(audio);
        let mut at = 0usize;
        while self.pending.len() - at >= self.n {
            let seg = &self.pending[at..at + self.n];
            for (i, b) in self.buf.iter_mut().enumerate() {
                *b = Complex32::new(seg[i] * self.window[i], 0.0);
            }
            self.fft.process(&mut self.buf);
            self.db.clear();
            self.db.extend(
                self.buf[self.lo..self.hi]
                    .iter()
                    .map(|c| 10.0 * (c.norm_sqr() + 1e-20).log10()),
            );
            let mut sorted = self.db.clone();
            sorted.sort_by(f32::total_cmp);
            let floor = sorted[sorted.len() * 3 / 10];
            let levels = self
                .db
                .iter()
                .map(|&d| {
                    (((d - floor - FLOOR_MARGIN_DB) / RANGE_DB) * 255.0).clamp(0.0, 255.0) as u8
                })
                .collect();
            // The window's middle, in samples before the last pushed one.
            let middle = self.start + (at + self.n / 2) as u64;
            let ago = (self.pushed - middle) as f64 / f64::from(RATE_HZ);
            out.push(Row {
                utc_ns: end - (ago * 1e9) as i64,
                f_lo_hz: self.lo as f32 * self.bin_hz(),
                bin_hz: self.bin_hz(),
                levels,
            });
            at += self.hop;
        }
        self.pending.drain(..at.min(self.pending.len()));
        self.start += at as u64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (std::f32::consts::TAU * hz * i as f32 / RATE_HZ).sin())
            .collect()
    }

    /// Two FT8-spaced tones 18.75 Hz apart stay two peaks with a gap between
    /// them at 2.9 Hz/bin, and a coarse 23 Hz/bin row runs them into one.
    #[test]
    fn neighbours_stay_apart() {
        // With a noise floor, as a band has: with none the floor is the window's
        // leakage and everything is far above it.
        let mut s = 99u32;
        let mut a: Vec<f32> = (0..24_000)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                0.05 * ((s >> 8) as f32 / 8_388_608.0 - 1.0)
            })
            .collect();
        for (x, (y, z)) in a
            .iter_mut()
            .zip(tone(1500.0, 24_000).into_iter().zip(tone(1518.75, 24_000)))
        {
            *x += y + z;
        }
        let mut w = Waterfall::new(4096, 2048, 1400.0, 1600.0);
        let mut rows = Vec::new();
        w.push(&a, Some(0), &mut rows);
        assert!(rows.len() >= 8, "{} rows", rows.len());
        let r = &rows[rows.len() / 2];
        let at = |hz: f32| ((hz - r.f_lo_hz) / r.bin_hz).round() as usize;
        let (p1, p2, gap) = (at(1500.0), at(1518.75), at(1509.4));
        assert!(
            r.levels[p1] > 200 && r.levels[p2] > 200,
            "peaks {:?}",
            (r.levels[p1], r.levels[p2])
        );
        assert!(r.levels[gap] < r.levels[p1] - 100, "gap {}", r.levels[gap]);
        // 8x coarser: the pair is one blob.
        let c = r.pooled(8);
        assert!(c.levels.iter().copied().max().unwrap() > 200);
    }

    /// The noise floor sits at the same grey whatever the level: a row of
    /// quiet noise and one of loud noise read alike.
    #[test]
    fn the_floor_is_level_independent() {
        let noise = |g: f32| -> Vec<f32> {
            let mut s = 12345u32;
            (0..8192)
                .map(|_| {
                    s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                    g * ((s >> 8) as f32 / 8_388_608.0 - 1.0)
                })
                .collect()
        };
        let mean = |g: f32| {
            let mut w = Waterfall::new(4096, 2048, 200.0, 3000.0);
            let mut rows = Vec::new();
            w.push(&noise(g), Some(0), &mut rows);
            let r = &rows[0];
            r.levels.iter().map(|&v| f32::from(v)).sum::<f32>() / r.levels.len() as f32
        };
        assert!((mean(0.01) - mean(1.0)).abs() < 3.0);
    }

    /// Row times follow the audio: a row ends no later than the last sample,
    /// and successive rows are `hop` samples apart.
    #[test]
    fn rows_are_timed_from_the_audio() {
        let mut w = Waterfall::new(4096, 2048, 200.0, 3000.0);
        let mut rows = Vec::new();
        let end = 1_000_000_000_000i64;
        w.push(&tone(1000.0, 12_000), Some(end), &mut rows);
        assert!(rows.iter().all(|r| r.utc_ns <= end));
        let step = rows[1].utc_ns - rows[0].utc_ns;
        assert!((step - 170_666_666).abs() < 2_000, "{step}");
    }
}
