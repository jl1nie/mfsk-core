//! `ALL.TXT` lines: every decode, and every transmission, one line
//! each — pure formatting, tested in `hosttest/mfsk-app-shared`; the
//! board appends them (`m5stack-cores3-app` `storage.rs`).
//!
//! The layout is WSJT-X's `MainWindow::write_all`
//! (`widgets/mainwindow.cpp` 10574-10640), which trims the decoder's
//! own output line (`lib/decoder.f90`'s `format(i6.6,i4,f5.1,i5,' ~
//! ',1x,a37,1x,a2)`) down to
//!
//! ```text
//! 260922_101500    14.074 Rx FT8    -12  0.3 1234 CQ JA1ABC PM95
//! 260922_101515    14.074 Tx FT8      0  0.0 1500 JA1ABC JL1NIE/P -12
//! ```
//!
//! so a file pulled off the board reads in any tool that reads
//! WSJT-X's. Two divergences: WSPR and FST4 go in the same file and
//! layout as FT8 (WSJT-X gives WSPR its own `ALL_WSPR.TXT`, in
//! `wsprd`'s format), since one file is what the board can serve; and
//! an unknown dial frequency — the CoreS3 has no CAT link — is written
//! `0.000` rather than guessed.

use std::fmt::Write as _;

use crate::civil_time::civil_from_unix;

fn prefix(out: &mut String, unix: i64, dial_hz: Option<u64>, dir: &str, mode: &str) {
    let (y, mo, d, h, mi, s) = civil_from_unix(unix);
    let mhz = dial_hz.map_or(0.0, |hz| hz as f64 / 1e6);
    let _ = write!(
        out,
        "{:02}{mo:02}{d:02}_{h:02}{mi:02}{s:02}{mhz:10.3} {dir} {mode:<6}",
        y.rem_euclid(100)
    );
}

/// A decode heard in the slot that began at `slot_unix`.
pub fn rx_line(
    slot_unix: i64,
    dial_hz: Option<u64>,
    mode: &str,
    snr_db: i32,
    dt_sec: f32,
    df_hz: i32,
    msg: &str,
) -> String {
    let mut s = String::with_capacity(80);
    prefix(&mut s, slot_unix, dial_hz, "Rx", mode);
    let _ = write!(s, "{snr_db:4}{dt_sec:5.1}{df_hz:5} {msg}");
    s.truncate(s.trim_end().len());
    s.push('\n');
    s
}

/// A received message with no report to give — JTTY's, whose decoder
/// carries neither an S/N nor a DT (`docs/notes/JTTY_CORES3_APP.md`
/// §5): the two columns are left blank, in their widths, rather than
/// written `0` / `0.0`, which would read as measurements. `unix` is when
/// the message began.
pub fn message_line(unix: i64, dial_hz: Option<u64>, mode: &str, df_hz: i32, msg: &str) -> String {
    let mut s = String::with_capacity(80);
    prefix(&mut s, unix, dial_hz, "Rx", mode);
    let _ = write!(s, "         {df_hz:5} {msg}");
    s.truncate(s.trim_end().len());
    s.push('\n');
    s
}

/// A transmission beginning at `unix`, at audio offset `df_hz`.
pub fn tx_line(unix: i64, dial_hz: Option<u64>, mode: &str, df_hz: i32, msg: &str) -> String {
    let mut s = String::with_capacity(80);
    prefix(&mut s, unix, dial_hz, "Tx", mode);
    let _ = write!(s, "   0  0.0{df_hz:5} {msg}");
    s.truncate(s.trim_end().len());
    s.push('\n');
    s
}
