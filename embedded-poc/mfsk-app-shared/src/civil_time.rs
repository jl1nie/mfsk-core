//! Unix-epoch-seconds → UTC calendar time, without a `chrono`/`time`
//! dependency.
//!
//! Pure integer arithmetic — Howard Hinnant's `civil_from_days`
//! algorithm (public domain,
//! <http://howardhinnant.github.io/date_algorithms.html>), the same
//! one `chrono`, Abseil's `civil_time`, and most C++ standard
//! libraries use internally. `wspr_app.rs` needs exactly two
//! formatted fields out of a system clock (`yyMMdd` for wsprnet's
//! `date`, `HHmm` for its `time` and the WSPR UI's `utc_hhmm`) and
//! nothing else in this crate needs a calendar at all, so this earns
//! its keep over pulling in a whole date crate for an embedded target.

/// `(year, month, day, hour, minute, second)`, all UTC. `year` is the
/// full year (e.g. `2026`) — callers that want a 2-digit field
/// (wsprnet's `yyMMdd`) take `year.rem_euclid(100)` themselves.
pub fn civil_from_unix(epoch_secs: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = epoch_secs.div_euclid(86_400);
    let secs_of_day = epoch_secs.rem_euclid(86_400);
    let hour = (secs_of_day / 3600) as u32;
    let minute = ((secs_of_day / 60) % 60) as u32;
    let second = (secs_of_day % 60) as u32;

    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    let year = if month <= 2 { y + 1 } else { y };

    (year, month, day, hour, minute, second)
}

/// The start of the `period_secs`-second UTC slot `epoch_secs` belongs
/// to, rounded to the **nearest** boundary rather than floored.
///
/// A spot database wants the slot's start time, and a receiver knows
/// that time as "whenever my capture window opened" — which is a clock
/// read taken a few milliseconds either side of the boundary it was
/// aiming at. Flooring turns a read that lands a millisecond *early*
/// into the previous slot, reporting a WSPR reception two minutes
/// before it happened; rounding is wrong only if the capture is more
/// than half a period out of phase, at which point the slot label is
/// the smaller problem.
///
/// `period_secs` is expected positive; a non-positive value returns
/// `epoch_secs` unchanged rather than dividing by zero.
pub fn slot_start_unix(epoch_secs: i64, period_secs: i64) -> i64 {
    if period_secs <= 0 {
        return epoch_secs;
    }
    let half = period_secs / 2;
    (epoch_secs + half).div_euclid(period_secs) * period_secs
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every expected tuple below was cross-checked independently via
    /// `date -u -d "<timestamp>" +%s` (GNU coreutils), not derived
    /// from this function itself — the point is to catch an error in
    /// this implementation, not confirm it against its own logic.
    #[test]
    fn epoch_zero_is_1970_01_01_midnight() {
        assert_eq!(civil_from_unix(0), (1970, 1, 1, 0, 0, 0));
    }

    #[test]
    fn wspr_golden_recording_slot_start() {
        // 2015-04-26 09:18:00 UTC — 150426_0918.wav's own slot start
        // (see reference_fst4_wspr_wsjtx_sample_decode memory / the
        // WSPR golden test fixtures).
        assert_eq!(civil_from_unix(1_430_039_880), (2015, 4, 26, 9, 18, 0));
    }

    #[test]
    fn recent_date_far_from_the_epoch() {
        assert_eq!(civil_from_unix(1_786_752_000), (2026, 8, 15, 0, 0, 0));
    }

    #[test]
    fn rolls_over_year_boundary() {
        assert_eq!(civil_from_unix(1_577_836_799), (2019, 12, 31, 23, 59, 59));
        assert_eq!(civil_from_unix(1_577_836_800), (2020, 1, 1, 0, 0, 0));
    }

    #[test]
    fn leap_day_in_a_leap_year() {
        assert_eq!(civil_from_unix(951_827_696), (2000, 2, 29, 12, 34, 56));
    }

    #[test]
    fn negative_timestamp_before_the_epoch() {
        // 1969-07-20 20:17:00 UTC — Apollo 11 touchdown, a
        // well-known pre-epoch date and a check that `div_euclid`/
        // `rem_euclid` (not plain `/`/`%`, which round toward zero
        // and would misplace both the day boundary and the
        // time-of-day for a negative input) are doing their job.
        assert_eq!(civil_from_unix(-14_182_980), (1969, 7, 20, 20, 17, 0));
    }

    /// WSPR's 120 s grid, the caller this was added for (#313).
    #[test]
    fn slot_start_rounds_to_the_nearest_boundary() {
        // 2026-09-07 12:34:00 UTC, an even minute — a boundary.
        let boundary = 1_788_784_440;
        assert_eq!(civil_from_unix(boundary), (2026, 9, 7, 12, 34, 0));

        // Exactly on it, and a hair either side: all three name the
        // same slot. The "a hair early" case is the one flooring gets
        // wrong, and it is the likely one — the capture opens on a
        // clock read taken just before the boundary it aimed at.
        assert_eq!(slot_start_unix(boundary, 120), boundary);
        assert_eq!(slot_start_unix(boundary - 1, 120), boundary);
        assert_eq!(slot_start_unix(boundary + 1, 120), boundary);

        // Up to half a period late still reads as this slot; past that
        // it is the next one, which is what rounding means.
        assert_eq!(slot_start_unix(boundary + 59, 120), boundary);
        assert_eq!(slot_start_unix(boundary + 60, 120), boundary + 120);
        assert_eq!(slot_start_unix(boundary + 119, 120), boundary + 120);
    }

    #[test]
    fn slot_start_handles_pre_epoch_and_a_degenerate_period() {
        // `div_euclid`, not `/`: a negative input must round down
        // toward the earlier boundary, not toward zero.
        assert_eq!(slot_start_unix(-121, 120), -120);
        // Exactly half a period out rounds up, like every other tie.
        assert_eq!(slot_start_unix(-180, 120), -120);
        assert_eq!(slot_start_unix(-181, 120), -240);
        // Non-positive period: return the input rather than panic.
        assert_eq!(slot_start_unix(1_788_784_441, 0), 1_788_784_441);
        assert_eq!(slot_start_unix(1_788_784_441, -120), 1_788_784_441);
    }
}
