// SPDX-License-Identifier: GPL-3.0-or-later
//! The UTC of IQ sample 0, from arrival times alone.
//!
//! SpyServer sends no timestamps. Each IQ message gives a candidate,
//! `arrival - samples_so_far / fs`; network and buffering delay only ever make a
//! candidate late, so the estimate is the *minimum* candidate over the last
//! `window_s` seconds of samples. Against an Airspy HF+ on the LAN
//! (2026-10-03) it moved 2 ms in 120 s and 7 ms in 9 min (the host clock
//! against the device's sample clock, ~13 ppm), delivery delay 3-11 ms.

use std::collections::VecDeque;

pub struct AnchorEstimate {
    window_samples: u64,
    /// (sample index the candidate was taken at, candidate ns), increasing
    /// in both: only candidates that can still become the minimum.
    window: VecDeque<(u64, i64)>,
}

impl AnchorEstimate {
    pub fn new(fs: u32, window_s: f64) -> Self {
        AnchorEstimate {
            window_samples: (window_s * fs as f64) as u64,
            window: VecDeque::new(),
        }
    }

    /// Add the message whose last sample is sample `samples - 1`, which
    /// arrived at `arrival_ns`; the current estimate.
    pub fn push(&mut self, samples: u64, fs: u32, arrival_ns: i64) -> i64 {
        let cand = arrival_ns - (samples as f64 * 1e9 / fs as f64) as i64;
        while self
            .window
            .front()
            .is_some_and(|&(k, _)| k + self.window_samples < samples)
        {
            self.window.pop_front();
        }
        while self.window.back().is_some_and(|&(_, c)| c >= cand) {
            self.window.pop_back();
        }
        self.window.push_back((samples, cand));
        self.window.front().unwrap().1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: u32 = 1_000;
    const T0: i64 = 1_700_000_000_000_000_000;

    /// Jittered delays never pull the estimate later than the least-delayed
    /// message, and a quiet moment is forgotten once out of the window.
    #[test]
    fn minimum_delay_within_the_window() {
        let mut e = AnchorEstimate::new(FS, 10.0);
        let delays_ms = [30, 12, 45, 5, 20, 60, 8];
        let mut best = i64::MAX;
        for (i, d) in delays_ms.iter().enumerate() {
            let samples = (i as u64 + 1) * FS as u64; // one message per second
            let arrival = T0 + (i as i64 + 1) * 1_000_000_000 + d * 1_000_000;
            best = e.push(samples, FS, arrival);
        }
        assert_eq!(best, T0 + 5_000_000);
        // 20 s later, every message 25 ms late: the 5 ms one has aged out.
        for i in 7..30u64 {
            let arrival = T0 + (i as i64 + 1) * 1_000_000_000 + 25_000_000;
            best = e.push((i + 1) * FS as u64, FS, arrival);
        }
        assert_eq!(best, T0 + 25_000_000);
    }
}
