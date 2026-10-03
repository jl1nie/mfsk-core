// SPDX-License-Identifier: GPL-3.0-or-later
//! Slot arithmetic on a sample clock: which sample opens which UTC slot, and
//! how a clock that is observed to drift is followed without losing one.
//!
//! Pure integer arithmetic, no `std`, no allocation, no atomics, so the
//! same code runs in the IQ receiver, in the C ABI's stream and on an
//! embedded board.
//!
//! - [`SlotGrid`] is the grid of one period at one sample rate: slot `j`
//!   covers UTC `[j·T, (j+1)·T)`, and starts at the first sample at or after
//!   that boundary.
//! - [`SampleClock`] says what UTC sample 0 fell on. It is told what the
//!   clock reads now ([`SampleClock::observe`]) and moves its answer towards
//!   it at a bounded rate, so a slot boundary slides by milliseconds
//!   instead of jumping and no slot is dropped. A difference past a step
//!   threshold is taken as the clock having been set, and re-anchors at
//!   once.

const NS: i128 = 1_000_000_000;

fn ceil_div(a: i128, b: i128) -> i128 {
    a.div_euclid(b) + i128::from(a.rem_euclid(b) != 0)
}

/// The slots of one period at one sample rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotGrid {
    period_ns: i128,
    rate_hz: u32,
}

impl SlotGrid {
    /// Slots of `period_ns` at `rate_hz` samples per second.
    pub const fn new(period_ns: i64, rate_hz: u32) -> Self {
        Self {
            period_ns: period_ns as i128,
            rate_hz,
        }
    }

    /// A slot's length in samples.
    pub const fn slot_samples(&self) -> u64 {
        (self.period_ns * self.rate_hz as i128 / NS) as u64
    }

    /// First sample of slot `j` when sample 0 is at UTC `anchor_ns`: the
    /// boundary, rounded up to a whole sample.
    pub fn start_of(&self, j: i64, anchor_ns: i64) -> i64 {
        let (j, p, a) = (j as i128, self.period_ns, anchor_ns as i128);
        ceil_div(j * p * self.rate_hz as i128 - a * self.rate_hz as i128, NS) as i64
    }

    /// The slot index whose start is the first at or after sample `k`, and
    /// that start: `(j, start_of(j))`.
    ///
    /// A start is its boundary rounded *up* to a sample, so the sample after
    /// one slot's last can lie up to a sample past the next boundary.
    /// Choosing the boundary by time alone would then skip that slot and open
    /// the one after, losing every other slot of a continuous stream, which
    /// is what a wall-clock anchor off the sample grid used to do. The
    /// candidate one slot earlier is taken when its rounded start is still
    /// at or after `k`.
    pub fn next_start(&self, k: u64, anchor_ns: i64) -> (i64, u64) {
        let (p, a, r) = (self.period_ns, anchor_ns as i128, self.rate_hz as i128);
        let mut j = ceil_div(a * r + k as i128 * NS, p * r) as i64;
        if self.start_of(j - 1, anchor_ns) >= k as i64 {
            j -= 1;
        }
        (j, self.start_of(j, anchor_ns).max(0) as u64)
    }

    /// The slot that follows slot `prev`, which ended just before sample
    /// `prev_end`: slot `prev + 1`, starting at its boundary, which is at
    /// `prev_end` or later (the anchor moved later, or the stream is slower
    /// than the clock: samples between are skipped) or up to `overlap`
    /// samples earlier (the stream is faster: the slot's first samples are
    /// the previous slot's last). `None` if the boundary is further behind
    /// than that, when [`Self::next_start`] should decide.
    ///
    /// This is what lets a clock slew, and a crystal run fast or slow, without
    /// losing a slot or drifting off the grid: a slot always starts on its
    /// own boundary, whichever way the anchor moved, instead of on whichever
    /// boundary the time then points at, or a fixed slot length after the
    /// last (which slides off by the crystal's ppm until it overruns).
    pub fn follow(
        &self,
        prev: i64,
        prev_end: u64,
        anchor_ns: i64,
        overlap: u64,
    ) -> Option<(i64, u64)> {
        let j = prev + 1;
        let start = self.start_of(j, anchor_ns);
        (start >= prev_end as i64 || (prev_end as i64 - start) as u64 <= overlap)
            .then_some((j, start.max(0) as u64))
    }
}

/// What an observation did to a [`SampleClock`].
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockChange {
    /// The first observation: the clock is set.
    First,
    /// Within bounds: the clock moved `by_ns` (possibly 0) towards the
    /// observation. Open slots are unaffected.
    Slewed { by_ns: i64 },
    /// The observation was more than the step threshold away: the clock
    /// re-anchored on it, moving `by_ns`. Slots that were open straddle the
    /// jump and are not slots any more.
    Stepped { by_ns: i64 },
}

/// What UTC sample 0 of a stream fell on, followed through drift.
#[derive(Clone, Copy, Debug)]
pub struct SampleClock {
    rate_hz: u32,
    /// UTC of sample 0 in use.
    anchor_ns: Option<i64>,
    /// Sample index of the last observation, for the slew budget.
    last_sample: u64,
    max_slew_ppm: u32,
    step_ns: i64,
}

impl SampleClock {
    /// A clock for a stream of `rate_hz` samples per second, slewing at most
    /// [`Self::DEFAULT_MAX_SLEW_PPM`] and stepping past
    /// [`Self::DEFAULT_STEP_NS`].
    pub const fn new(rate_hz: u32) -> Self {
        Self {
            rate_hz,
            anchor_ns: None,
            last_sample: 0,
            max_slew_ppm: Self::DEFAULT_MAX_SLEW_PPM,
            step_ns: Self::DEFAULT_STEP_NS,
        }
    }

    /// 400 ppm: 6 ms across an FT8 slot, 60 ms across a minute. A
    /// quartz-and-NTP drift of 13 ppm (1.6 ms per 2 minutes) is absorbed
    /// with room to spare, and a 50 ms host step is followed in two minutes
    /// without moving any one boundary by more than the slot's slack.
    pub const DEFAULT_MAX_SLEW_PPM: u32 = 400;

    /// A difference of more than a second is a clock that was set, not one
    /// that drifted.
    pub const DEFAULT_STEP_NS: i64 = 1_000_000_000;

    pub const fn with_max_slew_ppm(mut self, ppm: u32) -> Self {
        self.max_slew_ppm = ppm;
        self
    }

    pub const fn with_step_ns(mut self, ns: i64) -> Self {
        self.step_ns = ns;
        self
    }

    /// UTC of sample 0 in use, if the clock has been set.
    pub fn anchor_ns(&self) -> Option<i64> {
        self.anchor_ns
    }

    /// The UTC the stream's sample `k` is at, if the clock has been set.
    pub fn utc_of(&self, k: u64) -> Option<i64> {
        self.anchor_ns
            .map(|a| a + (k as i128 * NS / self.rate_hz as i128) as i64)
    }

    /// The stream's sample `at_sample` was at UTC `utc_ns` (nanoseconds
    /// since the Unix epoch).
    pub fn observe(&mut self, utc_ns: i64, at_sample: u64) -> ClockChange {
        let implied = utc_ns - (at_sample as i128 * NS / self.rate_hz as i128) as i64;
        let Some(current) = self.anchor_ns else {
            self.anchor_ns = Some(implied);
            self.last_sample = at_sample;
            return ClockChange::First;
        };
        let error = implied - current;
        let elapsed_ns =
            (at_sample.saturating_sub(self.last_sample) as i128 * NS / self.rate_hz as i128) as i64;
        self.last_sample = at_sample;
        if error.abs() > self.step_ns {
            self.anchor_ns = Some(implied);
            return ClockChange::Stepped { by_ns: error };
        }
        let budget = (elapsed_ns as i128 * self.max_slew_ppm as i128 / 1_000_000) as i64;
        let by_ns = error.clamp(-budget, budget);
        self.anchor_ns = Some(current + by_ns);
        ClockChange::Slewed { by_ns }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: u32 = 12_000;
    const FT8: SlotGrid = SlotGrid::new(15_000_000_000, FS);

    /// Consecutive slots on any anchor, however far off the sample grid:
    /// every slot opens exactly where the previous closed or at its own
    /// rounded boundary, never one slot late.
    #[test]
    fn consecutive_slots_follow_on_any_anchor() {
        for off in [0, 1, 41_667, 83_333, 1_000_000, 999_999_999] {
            let anchor = 1_700_000_010_000_000_000 + off;
            let (j0, s0) = FT8.next_start(0, anchor);
            let (j1, s1) = FT8.next_start(s0 + FT8.slot_samples(), anchor);
            assert_eq!((j1, s1), (j0 + 1, FT8.start_of(j0 + 1, anchor) as u64));
            assert!(s1 >= s0 + FT8.slot_samples());
        }
    }

    /// 24 hours at +13 ppm, the clock observed once a second with 3-11 ms of
    /// jitter: driven the way a receiver drives it (one second of samples,
    /// then an observation), no slot is lost, indices are consecutive, and
    /// every slot opens within the slew budget of its true boundary.
    #[test]
    fn a_day_of_drift_loses_no_slot() {
        let ppm = 13i128;
        let t0: i64 = 1_700_000_010_000_000_000;
        // True UTC of sample k: the crystal runs 13 ppm fast.
        let true_utc =
            |k: u64| t0 + (k as i128 * NS * (1_000_000 + ppm) / (FS as i128 * 1_000_000)) as i64;
        let mut clock = SampleClock::new(FS);
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        let mut jitter = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            3_000_000 + (rng % 8_000_000) as i64
        };
        let slot = FT8.slot_samples();
        // The receiver's state: the open slot's index and last sample + 1.
        let mut open: Option<(i64, u64)> = None;
        let mut prev_closed: Option<(i64, u64)> = None;
        let (mut k, mut slots, mut worst_ns) = (0u64, 0u64, 0i64);
        let day = 24 * 3600u64;
        for sec in 0..day {
            // One second of samples. A slot that ends inside it closes, and
            // the next opens with the anchor as it stands now.
            let end_of_second = k + FS as u64;
            let anchor = clock.anchor_ns().unwrap_or(t0);
            loop {
                match open {
                    Some((j, end)) if end <= end_of_second => {
                        open = None;
                        prev_closed = Some((j, end));
                        k = end;
                    }
                    Some(_) => break,
                    None => {
                        let next = prev_closed
                            .and_then(|(p, e)| FT8.follow(p, e, anchor, 12_000))
                            .unwrap_or_else(|| {
                                let (j, s) = FT8.next_start(k, anchor);
                                (j, s)
                            });
                        if next.1 >= end_of_second {
                            break;
                        }
                        if let Some((p, e)) = prev_closed {
                            assert_eq!(
                                next.0,
                                p + 1,
                                "slot lost at {sec} s: prev_end {e}, k {k}, start_of(p+1) {}, anchor {anchor}",
                                FT8.start_of(p + 1, anchor)
                            );
                        }
                        // Against the true boundary of that slot.
                        let boundary = next.0 as i128 * 15 * NS;
                        worst_ns = worst_ns
                            .max((true_utc(next.1) as i128 - boundary).unsigned_abs() as i64);
                        open = Some((next.0, next.1 + slot));
                        slots += 1;
                    }
                }
            }
            k = end_of_second;
            clock.observe(true_utc(k) + jitter(), k);
        }
        assert!(slots >= day / 15 - 2, "{slots} slots in a day");
        // The clock follows the crystal's drift with the observation noise on
        // top: far inside a 15 s slot's tolerance.
        assert!(
            worst_ns < 60_000_000,
            "worst boundary error {} ms",
            worst_ns / 1_000_000
        );
    }

    #[test]
    fn a_two_second_step_is_one_step() {
        let mut c = SampleClock::new(FS);
        assert_eq!(c.observe(1_000_000_000_000, 0), ClockChange::First);
        assert!(matches!(
            c.observe(1_000_000_000_000 + 10 * 1_000_000_000 + 2_000_000_000, 10 * FS as u64),
            ClockChange::Stepped { by_ns } if by_ns == 2_000_000_000
        ));
        assert!(matches!(
            c.observe(
                1_000_000_000_000 + 11 * 1_000_000_000 + 2_000_000_000,
                11 * FS as u64
            ),
            ClockChange::Slewed { by_ns: 0 }
        ));
    }
}
