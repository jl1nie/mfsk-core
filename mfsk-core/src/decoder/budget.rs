//! One period's compute allowance, shared by the calls of its sequence (#626, #627).
//!
//! A period reaches a decoder in several calls when early decode is on
//! ([`Decoder::decode_prefix`](super::Decoder::decode_prefix), or
//! [`Decoder::decode_slot`](super::Decoder::decode_slot) on an
//! [`IqReceiver`](crate::iq::IqReceiver) channel with prefix points): FT8's checkpoint A, B and
//! the end. `SlotInput::budget` is per call, so a caller that builds a fresh deadline for each
//! call gives the period an allowance per delivery, not one, and counts a period it cut
//! once per delivery that ran out. [`PeriodBudget`] is the accounting that keeps both right.
//! It holds no clock (the library has none: `Instant` is absent on `no_std` and unimplemented
//! on `wasm32-unknown-unknown`), only `Duration`s the caller measures.

use core::time::Duration;

use super::BudgetReport;
use crate::registry::Mode;

/// What one period may spend, what its calls have spent, and whether it was already counted as cut.
///
/// Keep one per open period (a new one when the period changes), give each call
/// [`PeriodBudget::left`] as its allowance, and book the call with [`PeriodBudget::finish`]:
///
/// ```
/// # #[cfg(all(feature = "std", feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))] {
/// use std::time::Instant;
/// use mfsk_core::Mode;
/// use mfsk_core::decoder::{AnyDecoder, PeriodBudget};
/// use mfsk_core::iq::CompletedSlot;
/// use mfsk_core::prelude::ChannelId;
///
/// let mut decoder = AnyDecoder::with_defaults(Mode::Ft8);
/// // 2 % of FT8's 15 s: 300 ms for A, B and the end together.
/// let mut budget = PeriodBudget::share(7, Mode::Ft8, Some(0.02));
/// let mut cut = 0;
/// for len in [141_696, 162_432, 180_000] {
///     let slot = CompletedSlot::new(ChannelId(0), Mode::Ft8, 7, vec![0.0; len]);
///     let start = Instant::now();
///     let within = budget.within(start); // what is left of the allowance, from now
///     let out = decoder
///         .decode_slot_with(&slot, within.as_ref().map(|w| w as _), None)
///         .unwrap();
///     cut += usize::from(budget.finish(start.elapsed(), &out.budget)); // once per period
/// }
/// assert_eq!(cut, 0);
/// # }
/// ```
#[derive(Clone, Debug)]
pub struct PeriodBudget {
    period: i64,
    allowance: Option<Duration>,
    spent: Duration,
    counted: bool,
}

impl PeriodBudget {
    /// A period's budget of `allowance` for all its calls together; `None` is no budget.
    pub fn new(period: i64, allowance: Option<Duration>) -> Self {
        Self {
            period,
            allowance,
            spent: Duration::ZERO,
            counted: false,
        }
    }

    /// A budget of `fraction` of `mode`'s slot (`0.02` is 300 ms of FT8's 15 s); `None`, or a
    /// fraction that is negative, not a number or too large for a `Duration`, is no budget.
    pub fn share(period: i64, mode: Mode, fraction: Option<f32>) -> Self {
        let allowance =
            fraction.and_then(|f| Duration::try_from_secs_f32(mode.meta().t_slot_s * f).ok());
        Self::new(period, allowance)
    }

    /// The period this budget is for.
    pub fn period(&self) -> i64 {
        self.period
    }

    /// Decoding time the period's calls have used. The time between calls, waiting for
    /// audio, is not decoding and is not counted.
    pub fn spent(&self) -> Duration {
        self.spent
    }

    /// What the next call may use: the allowance less what the earlier calls spent, never
    /// below zero (a period that has used it all starts its next call already out of time).
    /// `None` without a budget.
    pub fn left(&self) -> Option<Duration> {
        self.allowance.map(|a| a.saturating_sub(self.spent))
    }

    /// Book a call that took `took`. `true` when `report` is the period's **first** to run
    /// out of budget: count the period as cut then, and not on the later calls that run
    /// out as well. Never `true` without a budget.
    pub fn finish(&mut self, took: Duration, report: &BudgetReport) -> bool {
        self.spent += took;
        let first = report.exhausted && !self.counted;
        self.counted |= report.exhausted;
        first
    }

    /// A [`BudgetCheck`](super::BudgetCheck) for a call that starts at `start`: `true` until
    /// [`PeriodBudget::left`] has passed since then. `None` without a budget. On a target
    /// without `Instant`, build the predicate from `left()` and the clock you have.
    #[cfg(feature = "std")]
    pub fn within(&self, start: std::time::Instant) -> Option<impl Fn() -> bool + Sync + use<>> {
        let deadline = start + self.left()?;
        Some(move || std::time::Instant::now() < deadline)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exhausted(yes: bool) -> BudgetReport {
        BudgetReport {
            exhausted: yes,
            ..BudgetReport::default()
        }
    }

    /// #626: a period's calls share one allowance. Each call gets what the earlier ones left,
    /// the time between calls costs nothing, and a period that has used it all starts its next
    /// call already out of time.
    #[test]
    fn a_periods_calls_share_one_allowance() {
        let ms = Duration::from_millis;
        let mut p = PeriodBudget::new(7, Some(ms(300)));
        assert_eq!(p.left(), Some(ms(300)), "A: the whole share");
        p.finish(ms(120), &exhausted(false));
        // B starts 1.7 s later, waiting for audio; none of that is spent.
        assert_eq!(p.left(), Some(ms(180)));
        p.finish(ms(200), &exhausted(true));
        assert_eq!(p.left(), Some(Duration::ZERO), "the end: nothing left");
        assert_eq!(p.spent(), ms(320));
    }

    /// #627: a period whose A, B and final calls all run out is one cut slot; a second period is
    /// a second; a period cut at a prefix and never finished was counted when it was cut.
    #[test]
    fn a_cut_period_is_counted_once() {
        let ms = Duration::from_millis;
        let mut count = 0;
        let mut p = PeriodBudget::new(7, Some(ms(300)));
        for _ in 0..3 {
            count += usize::from(p.finish(ms(100), &exhausted(true)));
        }
        assert_eq!(count, 1);
        let mut q = PeriodBudget::new(8, Some(ms(300)));
        count += usize::from(q.finish(ms(100), &exhausted(false)));
        count += usize::from(q.finish(ms(100), &exhausted(true)));
        assert_eq!(count, 2);
        let mut r = PeriodBudget::new(9, Some(ms(300)));
        count += usize::from(r.finish(ms(100), &exhausted(true)));
        // `r` is abandoned here, with no final call: it was counted when it was cut.
        assert_eq!(count, 3);
    }

    #[test]
    fn no_budget_has_no_deadline_and_nothing_to_count() {
        let mut p = PeriodBudget::new(1, None);
        assert_eq!(p.left(), None);
        #[cfg(feature = "std")]
        assert!(p.within(std::time::Instant::now()).is_none());
        // An exhausted report without a budget (a caller's own predicate) is still counted once.
        assert!(p.finish(Duration::from_millis(5), &exhausted(true)));
        assert!(!p.finish(Duration::from_millis(5), &exhausted(true)));
    }

    #[cfg(feature = "ft8")]
    #[test]
    fn a_share_is_a_fraction_of_the_slot() {
        let p = PeriodBudget::share(3, Mode::Ft8, Some(0.02));
        let left = p.left().unwrap();
        assert!(
            left > Duration::from_millis(299) && left < Duration::from_millis(301),
            "{left:?}"
        );
        assert_eq!(p.period(), 3);
        for bad in [
            None,
            Some(-1.0),
            Some(f32::NAN),
            Some(f32::INFINITY),
            Some(1e30),
        ] {
            assert_eq!(
                PeriodBudget::share(3, Mode::Ft8, bad).left(),
                None,
                "{bad:?}"
            );
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn within_flips_at_the_deadline() {
        let ms = Duration::from_millis;
        let mut p = PeriodBudget::new(1, Some(ms(40)));
        let start = std::time::Instant::now();
        let w = p.within(start).unwrap();
        assert!(w(), "inside the allowance");
        std::thread::sleep(ms(60));
        assert!(!w(), "past it");
        // After a call that used it all, the next call's predicate is false from the start.
        p.finish(ms(40), &exhausted(true));
        assert!(!p.within(std::time::Instant::now()).unwrap()());
    }
}
