// SPDX-License-Identifier: GPL-3.0-only
//! Learn a stream's rate offset from the readings of the clock (#686).
//!
//! [`IqReceiver::set_time`](super::IqReceiver::set_time) is told, now and then, that input sample `s` was at
//! UTC `t`. Each reading puts the stream's start at `t - s/Fs` (the anchor). If the rate offset in force is
//! right the anchors are all the same; if the real rate is `delta` (a fraction) above it, the time `s/Fs`
//! the receiver assumes for `s` samples is too long by `delta` of itself, so the anchors fall by `delta` per
//! second: **the slope of the anchors against time is `-delta`**. A least-squares line through a few minutes
//! of readings finds it, and the offset is moved by that much, a limited step at a time; the readings in the
//! window were made against the old offset, so the window starts again after each move.
//!
//! The readings are the caller's: network delay and its jitter are the caller's to remove (the skimmer's
//! minimum-delay filter); a reading that is not a good one only costs a slow convergence, since the line
//! is fitted to many.

use alloc::collections::VecDeque;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// How the tracker works. The defaults suit a server's readings once a second or so.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateTrackConfig {
    /// Readings older than this (s of stream time) are forgotten.
    pub window_s: f64,
    /// The least a line is fitted over, s: a short line is a noisy slope. Readings that come from
    /// minimum-delay filtering over a jittery link wander by 100 ms and more, in steps, so their
    /// errors are correlated and the fit's own standard error is optimistic: a 30 s line was seen
    /// to swing +-1000 ppm and step the clock eleven times in eight minutes (#686).
    pub min_span_s: f64,
    /// The shortest line that may act on a **large** slope (`fast_min_ppm`, at four standard
    /// errors): a start that is far off (a measurement 2 % low) is found in half a minute.
    pub fast_span_s: f64,
    /// How large a slope must be, ppm, for the line over `fast_span_s` to act.
    pub fast_min_ppm: f64,
    /// The offset moves at most this much (ppm) per fit. A device that is restarted (an RSP behind
    /// `rsp_tcp`) can come back 2.5 % faster than it went down (#686): a fit must be able to follow.
    pub max_step_ppm: f64,
    /// A reading is kept at most this often, s: a thousand readings a second are no better than ten.
    pub keep_every_s: f64,
    /// A move smaller than this (ppm) is not worth the window it costs.
    pub dead_band_ppm: f64,
}

impl Default for RateTrackConfig {
    fn default() -> Self {
        Self {
            window_s: 300.0,
            min_span_s: 120.0,
            fast_span_s: 30.0,
            fast_min_ppm: 5_000.0,
            max_step_ppm: 30_000.0,
            keep_every_s: 1.0,
            dead_band_ppm: 100.0,
        }
    }
}

pub(crate) struct RateTracker {
    cfg: RateTrackConfig,
    /// `(stream time s, anchor s)`, the anchor relative to the window's first reading.
    pts: VecDeque<(f64, f64)>,
    anchor0: Option<f64>,
    last_kept: f64,
}

impl RateTracker {
    pub(crate) fn new(cfg: RateTrackConfig) -> Self {
        Self {
            cfg,
            pts: VecDeque::new(),
            anchor0: None,
            last_kept: f64::NEG_INFINITY,
        }
    }

    /// Forget the window: the clock was set or stepped, the stream was cut.
    pub(crate) fn reset(&mut self) {
        self.pts.clear();
        self.anchor0 = None;
        self.last_kept = f64::NEG_INFINITY;
    }

    /// A reading: the stream was `x_s` seconds along (by the rate offset in force) at an anchor of
    /// `anchor_s`. Returns how much to move the offset, in ppm, when the window has a fit worth acting on.
    pub(crate) fn observe(&mut self, x_s: f64, anchor_s: f64) -> Option<f64> {
        if x_s - self.last_kept < self.cfg.keep_every_s {
            return None;
        }
        self.last_kept = x_s;
        let a0 = *self.anchor0.get_or_insert(anchor_s);
        self.pts.push_back((x_s, anchor_s - a0));
        while self
            .pts
            .front()
            .is_some_and(|p| x_s - p.0 > self.cfg.window_s)
        {
            self.pts.pop_front();
        }
        let first = self.pts.front()?.0;
        let span = x_s - first;
        if self.pts.len() < 8 || span < self.cfg.fast_span_s.min(self.cfg.min_span_s) {
            return None;
        }
        // Least squares of anchor on time, about the window's mean (for precision).
        let n = self.pts.len() as f64;
        let mx = self.pts.iter().map(|p| p.0).sum::<f64>() / n;
        let my = self.pts.iter().map(|p| p.1).sum::<f64>() / n;
        let (mut sxx, mut sxy) = (0.0, 0.0);
        for &(x, y) in &self.pts {
            sxx += (x - mx) * (x - mx);
            sxy += (x - mx) * (y - my);
        }
        if sxx <= 0.0 {
            return None;
        }
        // The anchors fall by `delta` per second: the stream is `delta` faster than the offset says.
        let slope = sxy / sxx;
        let delta_ppm = -slope * 1e6;
        // Act on a slope that is more than noise: three standard errors of the fit (from its own
        // residuals), and the dead band.
        let res: f64 = self
            .pts
            .iter()
            .map(|&(x, y)| {
                let r = y - my - slope * (x - mx);
                r * r
            })
            .sum();
        let sigma_ppm = (res / (n - 2.0) / sxx).sqrt() * 1e6;
        let steady = span >= self.cfg.min_span_s
            && delta_ppm.abs() >= self.cfg.dead_band_ppm.max(3.0 * sigma_ppm);
        let fast = delta_ppm.abs() >= self.cfg.fast_min_ppm.max(4.0 * sigma_ppm);
        if !(steady || fast) {
            return None;
        }
        let step = delta_ppm.clamp(-self.cfg.max_step_ppm, self.cfg.max_step_ppm);
        // The readings were made against the offset being replaced.
        self.reset();
        Some(step)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stream 2000 ppm faster than the offset in force (0): readings once a second with 20 ms of
    /// jitter. The tracker finds the offset in one fit and the next fit finds nothing left.
    #[test]
    fn finds_the_slope_through_jitter() {
        let mut t = RateTracker::new(RateTrackConfig::default());
        let delta = 2.0e-3;
        let mut rng = 7u32;
        let mut moved = 0.0;
        let mut fits = 0;
        for k in 0..700 {
            let x = k as f64; // s of stream time at the offset in force
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let jitter = ((rng >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 0.04;
            // The anchor falls by delta per second of stream time; the offset already moved is taken out.
            let anchor = 1.7e9 - delta * x + moved * 1e-6 * x + jitter;
            if let Some(step) = t.observe(x, anchor) {
                fits += 1;
                moved += step;
                // A step is at most the cap.
                assert!(step.abs() <= 2_000.0 + 1e-9, "{step}");
            }
        }
        // 2000 ppm is found by the line over two minutes, once; then nothing is left to move.
        assert!(fits >= 1, "{fits}");
        assert!((moved - 2000.0).abs() < 100.0, "{moved}");
    }

    /// Readings that agree with the offset in force move nothing.
    #[test]
    fn a_right_offset_stays() {
        let mut t = RateTracker::new(RateTrackConfig::default());
        let mut rng = 3u32;
        for k in 0..900 {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = ((rng >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 0.004;
            assert_eq!(t.observe(k as f64, 1.7e9 + noise), None, "at {k}");
        }
    }

    /// A step is at most `max_step_ppm`, and the window starts again after it.
    #[test]
    fn a_big_slope_is_taken_in_steps() {
        let mut t = RateTracker::new(RateTrackConfig::default());
        let mut steps = Vec::new();
        for k in 0..200 {
            let x = k as f64;
            if let Some(s) = t.observe(x, 1.7e9 - 0.04 * x) {
                steps.push(s);
            }
        }
        // 40 000 ppm of slope: found in half a minute, and taken at the cap, 30 000 ppm.
        assert!(
            !steps.is_empty() && (steps[0] - 30_000.0).abs() < 1e-6,
            "{steps:?}"
        );
    }
}
