//! `wsprd`'s output lines, byte for byte, from a [`WsprResult`]: what a `wsprd`-compatible
//! program prints on stdout, appends to `ALL_WSPR.TXT` and writes to `wspr_spots.txt`
//! (`wsprd.c:1577-1594`, v3.3.0-beta6). `std` only: the formats go through `format!`, whose
//! float output is C's (`printf` rounds the exact binary value, to nearest even on a tie;
//! `-0` prints as `-0` in both).
//!
//! The caller supplies what the result does not know: the file's date (`150426`) and time
//! (`0918`), parsed from its name, and the dial frequency in MHz, which with `-e x` is
//! `dialfreq - x * 1e-6` (`wsprd.c:1003`). `wsprd` lists its decodes in order of increasing
//! printed frequency ([`in_wsprd_order`]).
//!
//! Measured against the vendored `wsprd` v3.3.0-beta6 output on the golden by
//! `tests/wsprd_beta6_parity.rs`.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use super::decode::WsprResult;

/// `wsprd.c:1600`, printed once after the decodes.
pub const DECODE_FINISHED: &str = "<DecodeFinished>\n";

/// `freq_print` (`wsprd.c:1522-1525`): `dialfreq + (1500 + f1) / 1e6`, in MHz. `1500 + f1` is a
/// single-precision sum in the C (`int + float`), then widened.
pub fn freq_mhz(r: &WsprResult, dial_mhz: f64) -> f64 {
    dial_mhz + f64::from(1500.0f32 + r.f1_hz) / 1e6
}

/// The message as `wsprd` prints it (`call_loc_pow`, with a Type 3 call resolved from the
/// table as it stood).
pub fn message_text(r: &WsprResult) -> String {
    r.message.render(r.hashed_call.as_deref())
}

/// `r` in the order `wsprd` lists decodes: increasing printed frequency, ties in decode
/// order (`wsprd.c:1559-1566`, a bubble sort that swaps only on `>`).
pub fn in_wsprd_order(results: &[WsprResult], dial_mhz: f64) -> Vec<&WsprResult> {
    let mut v: Vec<&WsprResult> = results.iter().collect();
    v.sort_by(|a, b| {
        freq_mhz(a, dial_mhz)
            .partial_cmp(&freq_mhz(b, dial_mhz))
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    v
}

/// `(int) drift` — C truncation toward zero, not rounding.
fn drift_int(r: &WsprResult) -> i32 {
    r.drift_hz as i32
}

/// `cycles / 81`, the unsigned integer division of `wsprd.c:1581`.
fn cycles_per_bit(r: &WsprResult) -> u32 {
    r.cycles / 81
}

/// The line on stdout (`"%4s %3.0f %4.1f %10.6f %2d  %-s \n"`, `wsprd.c:1577`).
pub fn stdout_line(r: &WsprResult, time: &str, dial_mhz: f64) -> String {
    format!(
        "{:>4} {:>3.0} {:>4.1} {:>10.6} {:>2}  {} \n",
        time,
        f64::from(r.snr_db),
        f64::from(r.dt_sec),
        freq_mhz(r, dial_mhz),
        drift_int(r),
        message_text(r),
    )
}

/// The line `ALL_WSPR.TXT` gets
/// (`"%6s %4s %3.0f %5.2f %11.7f  %-22s %2d %5.2f %2d %2d %4d %2d %3d %5u %5d\n"`,
/// `wsprd.c:1581`).
pub fn all_wspr_line(r: &WsprResult, date: &str, time: &str, dial_mhz: f64) -> String {
    format!(
        "{:>6} {:>4} {:>3.0} {:>5.2} {:>11.7}  {:<22} {:>2} {:>5.2} {:>2} {:>2} {:>4} {:>2} {:>3} {:>5} {:>5}\n",
        date,
        time,
        f64::from(r.snr_db),
        f64::from(r.dt_sec),
        freq_mhz(r, dial_mhz),
        message_text(r),
        drift_int(r),
        f64::from(r.sync),
        u32::from(r.pass) + 1,
        r.blocksize,
        r.jitter,
        r.decode_type,
        r.nhardmin,
        cycles_per_bit(r),
        r.metric,
    )
}

/// The line `wspr_spots.txt` gets
/// (`"%6s %4s %3d %3.0f %4.1f %10.6f  %-22s %2d %5u %4d\n"`, `wsprd.c:1589`); the sync is
/// `(int)(10 * sync)`.
pub fn spots_line(r: &WsprResult, date: &str, time: &str, dial_mhz: f64) -> String {
    format!(
        "{:>6} {:>4} {:>3} {:>3.0} {:>4.1} {:>10.6}  {:<22} {:>2} {:>5} {:>4}\n",
        date,
        time,
        (10.0f32 * r.sync) as i32,
        f64::from(r.snr_db),
        f64::from(r.dt_sec),
        freq_mhz(r, dial_mhz),
        message_text(r),
        drift_int(r),
        cycles_per_bit(r),
        r.jitter,
    )
}
