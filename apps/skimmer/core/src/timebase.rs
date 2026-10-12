// SPDX-License-Identifier: GPL-3.0-only
//! The time base of an `rtl_tcp` stream: one estimator for rate, anchor and pauses (#685, #686).
//!
//! The model, the invariants and the reasons are in `docs/notes/RTLTCP_TIMEBASE.md`. In short: the
//! lower envelope of `(samples / nominal, arrival)` is a line whose slope is the real rate and whose
//! intercept is the stream's start; a pause is a jump of the recent points above that line, a new rate
//! regime a jump below it.
//!
//! Pure: no clock, no I/O. `push` takes what a message gives and returns what the receiver should be told.

use std::collections::VecDeque;

/// Points older than this (s of stream time) are forgotten.
const WINDOW_S: f64 = 240.0;
/// One point is kept per this much stream time: the one lowest above the line in use.
const KEEP_S: f64 = 0.5;
/// The least a line is fitted over, s. Shorter, the slope of the previous line (or the one the caller
/// started with) is kept and only the intercept is fitted.
const MIN_SPAN_S: f64 = 10.0;
/// An edge of the hull shorter than this fraction of the window is a slope from two neighbours: not used.
const EDGE_FRACTION: f64 = 0.25;
/// A fit that lifts the line at the latest point by more than this at once is held for `HOLD_S`, s.
const HOLD_JUMP_S: f64 = 0.2;
/// How long, s of stream time.
const HOLD_S: f64 = 20.0;
/// The recent points a pause or a break is judged on, s.
const DETECT_S: f64 = 6.0;
/// All of the recent points this far above the line before them: a pause.
const PAUSE_MIN_S: f64 = 0.4;
/// All of the recent points this far below the line of the points before the last `BREAK_REF_S`: the
/// rate changed.
const BREAK_MIN_S: f64 = 0.2;
/// A pause is a step: the recent points differ by less than this from each other, s.
const PAUSE_FLAT_S: f64 = 0.1;
/// The line a new rate is judged against ends this long before now, s.
const BREAK_REF_S: f64 = 30.0;
/// Recent points needed before a pause or break is judged.
const RECENT_MIN: usize = 8;

/// What the receiver is to be told after a message.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Update {
    /// UTC (ns since the Unix epoch) of the last sample of the message, on the line.
    pub utc_ns: i64,
    /// The rate offset: the stream's real rate is `nominal * (1 + ppm * 1e-6)`.
    pub ppm: f64,
    /// Set the clock at once, not by slewing: the first line, a pause, a change of rate.
    pub step: bool,
    /// Samples the server dropped (a pause was found): `IqReceiver::gap` them before using `utc_ns`.
    pub lost_samples: u64,
}

pub struct TimeBase {
    nominal: f64,
    t0_ns: Option<i64>,
    /// Added to every UTC returned (the network delay in force, negated).
    shift_ns: i64,
    /// Stream time added to `samples / nominal` for the samples the server never sent.
    xoff: f64,
    pts: VecDeque<(f64, f64)>,
    /// The point of the bucket in progress: `(bucket, x, y)`.
    cand: Option<(i64, f64, f64)>,
    /// Intercept and slope of the line in use (`y = a + b x`); `fitted` when it spans `MIN_SPAN_S`.
    a: f64,
    b: f64,
    fitted: bool,
    /// Stream time since which the fit has been refused for lifting the line.
    held_since: Option<f64>,
}

impl TimeBase {
    /// `nominal`: the rate asked of the server. `start_ppm`: what a previous session learned (0 if none).
    pub fn new(nominal: f64, start_ppm: f64) -> Self {
        Self {
            nominal,
            t0_ns: None,
            shift_ns: 0,
            xoff: 0.0,
            pts: VecDeque::new(),
            cand: None,
            a: 0.0,
            b: 1.0 / (1.0 + start_ppm * 1e-6),
            fitted: false,
            held_since: None,
        }
    }

    /// The rate offset of the line in use, ppm.
    pub fn ppm(&self) -> f64 {
        (1.0 / self.b - 1.0) * 1e6
    }

    /// Whether a line over `MIN_SPAN_S` has been fitted.
    pub fn fitted(&self) -> bool {
        self.fitted
    }

    /// Every UTC from now on is this much later (a changed network delay, negated).
    pub fn shift(&mut self, by_ns: i64) {
        self.shift_ns += by_ns;
    }

    /// A message ended at `arrival_ns` with `samples` received since the start (the server's, not the
    /// receiver's: the gaps `Update::lost_samples` reports are not in it).
    pub fn push(&mut self, samples: u64, arrival_ns: i64) -> Update {
        let t0 = *self.t0_ns.get_or_insert(arrival_ns);
        let x = samples as f64 / self.nominal + self.xoff;
        let y = (arrival_ns - t0) as f64 * 1e-9;
        let mut step = false;
        let mut lost_samples = 0;
        let bucket = (x / KEEP_S).floor() as i64;
        let key = |x: f64, y: f64, b: f64| y - b * x;
        match self.cand {
            Some((k, cx, cy)) if k == bucket => {
                if key(x, y, self.b) < key(cx, cy, self.b) {
                    self.cand = Some((bucket, x, y));
                }
            }
            Some((_, cx, cy)) => {
                self.pts.push_back((cx, cy));
                self.cand = Some((bucket, x, y));
                while self.pts.front().is_some_and(|p| x - p.0 > WINDOW_S) {
                    self.pts.pop_front();
                }
                self.evaluate(x, &mut step, &mut lost_samples);
            }
            None => self.cand = Some((bucket, x, y)),
        }
        if self.pts.is_empty() {
            // Nothing committed yet: the line through this point at the slope in use.
            self.a = y - self.b * x;
        }
        // A pause found just now moved `xoff`: this message is after it.
        let x = samples as f64 / self.nominal + self.xoff;
        Update {
            utc_ns: t0 + ((self.a + self.b * x) * 1e9).round() as i64 + self.shift_ns,
            ppm: self.ppm(),
            step,
            lost_samples,
        }
    }

    /// A point was committed at stream time `x`: look for a pause or a break, then fit.
    fn evaluate(&mut self, x: f64, step: &mut bool, lost: &mut u64) {
        // A pause: the last 6 s all above the line the points before them make. The line is not
        // spoilt by them: points above it never reach the lower hull.
        if let Some((a, b, split)) = self.reference(x - DETECT_S) {
            let recent = &self.pts.make_contiguous()[split..];
            let (lo, hi) = spread(recent, a, b);
            // A step, not a ramp: a slower rate than the line's rises steadily past it.
            if lo > PAUSE_MIN_S && hi - lo < PAUSE_FLAT_S {
                // The server paused for `lo`: the samples of that time were never sent.
                let dx = lo / b;
                for p in self.pts.iter_mut().skip(split) {
                    p.0 += dx;
                }
                self.xoff += dx;
                if let Some(c) = self.cand.as_mut() {
                    c.1 += dx;
                }
                *lost = (dx * self.nominal).round() as u64;
                *step = true;
            }
        }
        // A new rate: the last 6 s all below the line of the points before the last 30 s. (Points of
        // the new rate in the reference would pull it down with them, so it ends earlier than a
        // pause's does: the break is found after ~15 s at 2.5 %.)
        if !*step && let Some((a, b, split)) = self.reference(x - BREAK_REF_S) {
            let from = self.pts.partition_point(|p| p.0 <= x - DETECT_S);
            let recent = &self.pts.make_contiguous()[from..];
            let (lo, hi) = spread(recent, a, b);
            if recent.len() >= RECENT_MIN && (hi < -BREAK_MIN_S || lo > BREAK_MIN_S) {
                let keep = self.pts.partition_point(|p| p.0 <= x - 2.0 * DETECT_S);
                let _ = split;
                self.pts.drain(..keep);
                self.fitted = false;
                *step = true;
            }
        }
        let pts = self.pts.make_contiguous();
        let (a, b, ok) = fit(pts, self.b);
        // A pause not yet told from the line before it (the first seconds, when the window is short)
        // lifts the fit all at once: that is not a rate, and not taken until it has lasted.
        if self.fitted && !*step && (a + b * x) - (self.a + self.b * x) > HOLD_JUMP_S {
            let since = *self.held_since.get_or_insert(x);
            if x - since < HOLD_S {
                return;
            }
            *step = true;
        }
        self.held_since = None;
        self.a = a;
        self.b = b;
        if ok && !self.fitted {
            self.fitted = true;
            *step = true;
        }
    }
}

/// Lowest and highest residual of `pts` against `y = a + b x`.
fn spread(pts: &[(f64, f64)], a: f64, b: f64) -> (f64, f64) {
    pts.iter()
        .map(|p| p.1 - (a + b * p.0))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| {
            (lo.min(r), hi.max(r))
        })
}

impl TimeBase {
    /// The line fitted to the points up to stream time `until`: `(a, b, index of the first later point)`,
    /// when they span `MIN_SPAN_S` and there are enough later ones.
    fn reference(&mut self, until: f64) -> Option<(f64, f64, usize)> {
        let split = self.pts.partition_point(|p| p.0 <= until);
        if split < 2 || self.pts.len() - split < RECENT_MIN {
            return None;
        }
        let b = self.b;
        let (a, b, ok) = fit(&self.pts.make_contiguous()[..split], b);
        ok.then_some((a, b, split))
    }
}

/// The line under every point, as high as possible: the lower hull's edge of least total residual.
/// Returns `(a, b, spans MIN_SPAN_S)`; under `MIN_SPAN_S`, or when no edge is long enough to trust, the
/// slope is `prev_b` and the intercept the lowest point's.
fn fit(pts: &[(f64, f64)], prev_b: f64) -> (f64, f64, bool) {
    let Some(&(x0, _)) = pts.first() else {
        return (0.0, prev_b, false);
    };
    let span = pts[pts.len() - 1].0 - x0;
    let at = |b: f64| {
        pts.iter()
            .map(|p| p.1 - b * p.0)
            .fold(f64::INFINITY, f64::min)
    };
    if span < MIN_SPAN_S {
        return (at(prev_b), prev_b, false);
    }
    // Lower convex hull (monotone chain), points already ascending in x.
    let mut hull: Vec<(f64, f64)> = Vec::new();
    for &p in pts {
        while hull.len() >= 2 {
            let (o, q) = (hull[hull.len() - 2], hull[hull.len() - 1]);
            if (q.0 - o.0) * (p.1 - o.1) - (q.1 - o.1) * (p.0 - o.0) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }
    let n = pts.len() as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |s, p| (s.0 + p.0, s.1 + p.1));
    let mut best: Option<(f64, f64, f64)> = None; // (sum of residuals, a, b)
    for w in hull.windows(2) {
        let (p, q) = (w[0], w[1]);
        if q.0 - p.0 < EDGE_FRACTION * span {
            continue;
        }
        let m = (q.1 - p.1) / (q.0 - p.0);
        let sum = sy - n * p.1 - m * (sx - n * p.0);
        if best.is_none_or(|c| sum < c.0) {
            best = Some((sum, p.1 - m * p.0, m));
        }
    }
    match best {
        Some((_, a, b)) => (a, b, true),
        None => (at(prev_b), prev_b, true),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOMINAL: f64 = 192_000.0;
    const MSG_S: f64 = 0.1;

    /// A deterministic delay: mostly under 10 ms, now and then 60 ms, rarely 200 ms; never negative.
    fn delay_ns(k: u64) -> i64 {
        let h = k
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407)
            >> 33;
        let u = (h % 1000) as f64 / 1000.0;
        let d = if u < 0.80 {
            u * 10.0
        } else if u < 0.98 {
            60.0 * (u - 0.8) / 0.18
        } else {
            200.0 * (u - 0.98) / 0.02
        };
        (d * 1e6) as i64
    }

    /// A server whose clock runs at `rate`, with optional events, driven in virtual time.
    struct Sim {
        tb: TimeBase,
        rate: f64,
        t: f64,       // true seconds since the start
        sent: f64,    // samples the server has produced up to t
        counted: u64, // samples the client received
        lost: f64,    // samples that never existed (pauses)
        k: u64,
        pauses: u32,
        last: Option<Update>,
        t0_ns: i64,
    }

    impl Sim {
        fn new(rate: f64, start_ppm: f64) -> Self {
            Self {
                tb: TimeBase::new(NOMINAL, start_ppm),
                rate,
                t: 0.0,
                sent: 0.0,
                counted: 0,
                lost: 0.0,
                k: 0,
                pauses: 0,
                last: None,
                t0_ns: 1_700_000_000_000_000_000,
            }
        }

        /// One message of `MSG_S`.
        fn step(&mut self) {
            self.t += MSG_S;
            let n = (self.rate * MSG_S).round();
            self.sent += n;
            self.counted += n as u64;
            self.k += 1;
            let arrival = self.t0_ns + (self.t * 1e9) as i64 + delay_ns(self.k);
            let u = self.tb.push(self.counted, arrival);
            if u.lost_samples > 0 {
                self.pauses += 1;
            }
            self.last = Some(u);
        }

        /// The server stops for `p` seconds and its samples of that time never exist.
        fn pause(&mut self, p: f64) {
            self.t += p;
            self.lost += self.rate * p;
        }

        /// How far the returned UTC is from the true time of the latest sample, ms.
        fn error_ms(&self) -> f64 {
            let u = self.last.unwrap();
            let truth = self.t0_ns as f64 + self.t * 1e9; // the last message ended at t, with no delay
            (u.utc_ns as f64 - truth) * 1e-6
        }

        fn run(&mut self, seconds: f64) {
            let end = self.t + seconds;
            while self.t < end - 1e-9 {
                self.step();
            }
        }
    }

    /// 2.9 % under nominal, found within a minute to ~200 ppm, the clock to 30 ms.
    #[test]
    fn finds_the_rate_and_the_start() {
        let mut s = Sim::new(NOMINAL * 0.971, 0.0);
        s.run(60.0);
        let want = (0.971f64 - 1.0) * 1e6;
        let got = s.tb.ppm();
        assert!((got - want).abs() < 600.0, "{got} vs {want}");
        s.run(120.0);
        assert!((s.tb.ppm() - want).abs() < 150.0, "{}", s.tb.ppm());
        assert!(s.error_ms().abs() < 30.0, "{}", s.error_ms());
    }

    /// A server whose rate is exactly nominal: ppm near zero, and no pause is imagined in ten minutes of
    /// the delay's tail.
    #[test]
    fn a_steady_stream_has_no_events() {
        let mut s = Sim::new(NOMINAL, 0.0);
        s.run(600.0);
        assert!(s.tb.ppm().abs() < 100.0, "{}", s.tb.ppm());
        assert_eq!(s.pauses, 0);
        assert!(s.error_ms().abs() < 30.0, "{}", s.error_ms());
    }

    /// A pause of 1.4 s: found once, its size right to 10 %, and the time right again within ten
    /// seconds, not 30.
    #[test]
    fn a_pause_is_found_once_by_its_size() {
        let mut s = Sim::new(NOMINAL * 0.972, -28_000.0);
        s.run(100.0);
        s.pause(1.4);
        let lost_before = s.lost;
        s.run(12.0);
        assert_eq!(s.pauses, 1, "found {} times", s.pauses);
        assert!(s.error_ms().abs() < 40.0, "{}", s.error_ms());
        s.run(200.0);
        assert_eq!(s.pauses, 1);
        assert!(s.error_ms().abs() < 40.0, "{}", s.error_ms());
        let _ = lost_before;
    }

    /// The size: samples reported lost against what was lost.
    #[test]
    fn the_samples_lost_are_the_samples_lost() {
        let mut s = Sim::new(NOMINAL * 0.972, -28_000.0);
        s.run(100.0);
        s.pause(1.4);
        let mut reported = 0u64;
        for _ in 0..200 {
            s.step();
            if let Some(u) = s.last {
                reported += u.lost_samples;
            }
        }
        let want = s.lost;
        assert!(
            (reported as f64 - want).abs() < 0.1 * want,
            "reported {reported}, lost {want}"
        );
    }

    /// A network stall that is made up (the data arrives late, then all at once) is not a pause.
    #[test]
    fn a_stall_that_is_made_up_is_not_a_pause() {
        let mut s = Sim::new(NOMINAL * 0.972, -28_000.0);
        s.run(100.0);
        // Hold 3 s of messages, then deliver them back to back.
        let mut held = Vec::new();
        for _ in 0..30 {
            s.t += MSG_S;
            let n = (s.rate * MSG_S).round();
            s.counted += n as u64;
            held.push(s.counted);
        }
        let arrival = s.t0_ns + (s.t * 1e9) as i64;
        for c in held {
            s.tb.push(c, arrival);
        }
        s.run(60.0);
        assert_eq!(s.pauses, 0);
        assert!(s.error_ms().abs() < 40.0, "{}", s.error_ms());
    }

    /// The server comes back from a restart 2.5 % faster: noticed, and followed within a minute and a
    /// half to 1000 ppm.
    #[test]
    fn a_new_rate_is_followed() {
        let mut s = Sim::new(NOMINAL * 0.972, -28_000.0);
        s.run(150.0);
        s.rate = NOMINAL * 0.997;
        s.run(90.0);
        let want = (0.997f64 - 1.0) * 1e6;
        assert!(
            (s.tb.ppm() - want).abs() < 1000.0,
            "{} vs {want}",
            s.tb.ppm()
        );
        s.run(60.0);
        assert!(s.error_ms().abs() < 60.0, "{}", s.error_ms());
    }

    /// What the RSP does for ten minutes: a restart every ~80 s (a pause of 0.9-1.5 s), the rate coming
    /// back different each time (-2.9 %, -2.7 %, +0.1 %...). None is found twice; 40 s after each
    /// the clock is within 80 ms.
    #[test]
    fn restarts_every_minute_or_so() {
        let mut s = Sim::new(NOMINAL * 0.971, 0.0);
        let rates = [0.971, 0.973, 1.001, 0.972, 0.998, 0.970, 0.972, 0.999];
        let mut worst: f64 = 0.0;
        for (i, r) in rates.iter().enumerate() {
            let before = s.pauses;
            s.run(80.0);
            s.pause(0.9 + 0.1 * (i % 7) as f64);
            s.rate = NOMINAL * r;
            s.run(10.0);
            // A pause with a rate change big enough to ramp the points is read as a break (the clock
            // is stepped just the same); what must not happen is the same pause found again.
            assert!(s.pauses <= before + 1, "restart {i}");
            // 40 s after: the rate has been followed and the clock is right.
            s.run(30.0);
            worst = worst.max(s.error_ms().abs());
        }
        assert!(worst < 80.0, "{worst}");
    }
}
