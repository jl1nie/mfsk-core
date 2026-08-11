// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 77-bit message decoder.
//!
//! Ported from WSJT-X `lib/77bit/packjt77.f90` (subroutines `unpack77`,
//! `unpack28`, `to_grid4`, `unpacktext77`).
//!
//! Only the most common message types are decoded:
//! - Type 0 n3=0 : Free text (71 bits → 13 chars)
//! - Type 1       : Standard (callsign + callsign + grid/report)
//! - Type 2       : Standard with /P suffix (EU VHF contest)
//! - Type 4       : One non-standard call + one hashed call
//!
//! For message types that require a hash table (22-bit hashed callsigns),
//! `<...>` is returned as a placeholder unless a [`CallsignHashTable`] is
//! provided via [`unpack77_with_hash`].

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use super::hash_table::CallsignHashTable;

// ── Character sets (match WSJT-X packjt77.f90) ──────────────────────────────

/// c1 in Fortran: 37 chars for callsign position 1
const C1: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";
/// c2: 36 chars for position 2
const C2: &[u8] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
/// c3: 10 chars for position 3 (digit only)
const C3: &[u8] = b"0123456789";
/// c4: 27 chars for positions 4-6 (space + A-Z)
const C4: &[u8] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ";
/// c (38 chars) used for non-standard callsign in Type 4
const C38: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";
/// 42-char alphabet for free-text messages
const FREE_TEXT: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";

/// US states + Canadian provinces + DX-region tags used by the ARRL
/// RTTY Roundup Type-3 message format. Mirrors WSJT-X
/// `packjt77.f90:240-258` `cmult` table (NUSCAN=171). Index 0 = "AL",
/// 4 = "CA", 20 = "MA", etc.; entries past index 71 are "X01"…"X99"
/// placeholders.
const RTTY_STATES: &[&str] = &[
    "AL", "AK", "AZ", "AR", "CA", "CO", "CT", "DE", "FL", "GA", "HI", "ID", "IL", "IN", "IA", "KS",
    "KY", "LA", "ME", "MD", "MA", "MI", "MN", "MS", "MO", "MT", "NE", "NV", "NH", "NJ", "NM", "NY",
    "NC", "ND", "OH", "OK", "OR", "PA", "RI", "SC", "SD", "TN", "TX", "UT", "VT", "VA", "WA", "WV",
    "WI", "WY", "NB", "NS", "QC", "ON", "MB", "SK", "AB", "BC", "NWT", "NF", "LB", "NU", "YT",
    "PEI", "DC", "DR", "FR", "GD", "GR", "OV", "ZH", "ZL", "X01", "X02", "X03", "X04", "X05",
    "X06", "X07", "X08", "X09", "X10", "X11", "X12", "X13", "X14", "X15", "X16", "X17", "X18",
    "X19", "X20", "X21", "X22", "X23", "X24", "X25", "X26", "X27", "X28", "X29", "X30", "X31",
    "X32", "X33", "X34", "X35", "X36", "X37", "X38", "X39", "X40", "X41", "X42", "X43", "X44",
    "X45", "X46", "X47", "X48", "X49", "X50", "X51", "X52", "X53", "X54", "X55", "X56", "X57",
    "X58", "X59", "X60", "X61", "X62", "X63", "X64", "X65", "X66", "X67", "X68", "X69", "X70",
    "X71", "X72", "X73", "X74", "X75", "X76", "X77", "X78", "X79", "X80", "X81", "X82", "X83",
    "X84", "X85", "X86", "X87", "X88", "X89", "X90", "X91", "X92", "X93", "X94", "X95", "X96",
    "X97", "X98", "X99",
];

// ── Token boundaries ─────────────────────────────────────────────────────────

const NTOKENS: u32 = 2_063_592;
const MAX22: u32 = 4_194_304;
const MAX_GRID4: u32 = 32_400;

// ── Internal helpers ─────────────────────────────────────────────────────────

/// Read `len` bits starting at `start` from `msg` (MSB first) into a u32.
fn read_bits(msg: &[u8], start: usize, len: usize) -> u32 {
    let mut n = 0u32;
    for i in start..start + len {
        n = (n << 1) | (msg[i] & 1) as u32;
    }
    n
}

/// Same as `read_bits` but returns u64 (for the 58-bit field in Type 4).
fn read_bits_u64(msg: &[u8], start: usize, len: usize) -> u64 {
    let mut n = 0u64;
    for i in start..start + len {
        n = (n << 1) | (msg[i] & 1) as u64;
    }
    n
}

/// Render a signal report the way WSJT-X does.
///
/// `packjt77.f90:504-505`:
///
/// ```fortran
/// write(crpt,'(i3.2)') isnr
/// if(crpt(1:1).eq.' ') crpt(1:1)='+'
/// ```
///
/// Fortran's `i3.2` is width 3 with a **minimum of two digits**, so
/// both signs come out two-digit: `-8` → `-08`, `8` → ` 08` → `+08`.
///
/// This existed inline at two call sites and got it wrong at both, in
/// a way that only showed on single-digit magnitudes: the positive
/// branch put `+` in a separate literal so `{:02}` padded the digits,
/// but the negative branch left `-` inside the formatted integer,
/// where Rust counts it toward the width — `-8` is already 2 wide, so
/// nothing was padded. Formatting the *magnitude* and prepending the
/// sign makes both branches the same shape and the bug unrepresentable.
///
/// Caught measuring FT8 recall against a real `jt9` build, where four
/// decodes looked like misses purely because of this (`W1FC F5BZB -8`
/// vs `-08`).
fn fmt_report(isnr: i32, ir: u8) -> String {
    let sign = if isnr >= 0 { '+' } else { '-' };
    let prefix = if ir == 1 { "R" } else { "" };
    format!("{prefix}{sign}{:02}", isnr.abs())
}

/// Decode a 28-bit packed callsign token.
///
/// Returns the human-readable callsign, "DE", "QRZ", "CQ", "CQ NNN",
/// "CQ XXXX", or "<...>" when the token is a 22-bit hash that cannot be
/// resolved without a call-sign database.
fn unpack28(n28: u32) -> String {
    if n28 < NTOKENS {
        return match n28 {
            0 => "DE".to_string(),
            1 => "QRZ".to_string(),
            2 => "CQ".to_string(),
            3..=1002 => format!("CQ {:03}", n28 - 3),
            _ => {
                // 1003..=532443: "CQ XXXX" (4-char directional CQ). The
                // n28 < NTOKENS check above also permits values 532444..
                // NTOKENS where i1 overflows C4 — bounds-check and fall
                // back to a placeholder.
                let n = n28 - 1003;
                let i1 = (n / (27 * 27 * 27)) as usize;
                let n = n % (27 * 27 * 27);
                let i2 = (n / (27 * 27)) as usize;
                let n = n % (27 * 27);
                let i3 = (n / 27) as usize;
                let i4 = (n % 27) as usize;
                if i1 >= C4.len() || i2 >= C4.len() || i3 >= C4.len() || i4 >= C4.len() {
                    return "<?>".to_string();
                }
                let suffix: String = [C4[i1], C4[i2], C4[i3], C4[i4]]
                    .iter()
                    .map(|&b| b as char)
                    .collect();
                format!("CQ {}", suffix.trim())
            }
        };
    }

    let n = n28 - NTOKENS;
    if n < MAX22 {
        // 22-bit hash — no call-sign database available
        return "<...>".to_string();
    }

    // Standard callsign: 6 characters from mixed alphabets
    let n = n - MAX22;
    let i1 = (n / (36 * 10 * 27 * 27 * 27)) as usize;
    let n = n % (36 * 10 * 27 * 27 * 27);
    let i2 = (n / (10 * 27 * 27 * 27)) as usize;
    let n = n % (10 * 27 * 27 * 27);
    let i3 = (n / (27 * 27 * 27)) as usize;
    let n = n % (27 * 27 * 27);
    let i4 = (n / (27 * 27)) as usize;
    let n = n % (27 * 27);
    let i5 = (n / 27) as usize;
    let i6 = (n % 27) as usize;

    if i1 >= C1.len()
        || i2 >= C2.len()
        || i3 >= C3.len()
        || i4 >= C4.len()
        || i5 >= C4.len()
        || i6 >= C4.len()
    {
        return "?????".to_string();
    }

    let s: String = [C1[i1], C2[i2], C3[i3], C4[i4], C4[i5], C4[i6]]
        .iter()
        .map(|&b| b as char)
        .collect();
    s.trim().to_string()
}

/// Decode a 28-bit packed callsign token, with hash table lookup.
fn unpack28_h(n28: u32, ht: &CallsignHashTable) -> String {
    if n28 >= NTOKENS {
        let n = n28 - NTOKENS;
        if n < MAX22 {
            // 22-bit hash — try table lookup
            if let Some(resolved) = ht.lookup22(n) {
                return resolved;
            }
            return "<...>".to_string();
        }
    }
    unpack28(n28)
}

/// Decode a 12-bit hash with table lookup.
fn resolve_hash12(n12: u32, ht: &CallsignHashTable) -> String {
    if let Some(call) = ht.lookup12(n12) {
        format!("<{}>", call)
    } else {
        "<...>".to_string()
    }
}

/// Decode a 15-bit Maidenhead grid square index.
fn to_grid4(n: u32) -> Option<String> {
    if n > MAX_GRID4 {
        return None;
    }
    let j1 = n / (18 * 10 * 10);
    let n = n % (18 * 10 * 10);
    let j2 = n / (10 * 10);
    let n = n % (10 * 10);
    let j3 = n / 10;
    let j4 = n % 10;
    if j1 > 17 || j2 > 17 {
        return None;
    }
    Some(format!(
        "{}{}{}{}",
        (b'A' + j1 as u8) as char,
        (b'A' + j2 as u8) as char,
        (b'0' + j3 as u8) as char,
        (b'0' + j4 as u8) as char,
    ))
}

/// Decode a 71-bit free-text message (13 chars from a 42-char alphabet).
fn unpack_free_text(msg: &[u8]) -> String {
    let mut n = 0u128;
    for i in 0..71 {
        n = (n << 1) | (msg[i] & 1) as u128;
    }
    let mut chars = [b' '; 13];
    for i in (0..13).rev() {
        chars[i] = FREE_TEXT[(n % 42) as usize];
        n /= 42;
    }
    String::from_utf8(chars.to_vec())
        .unwrap_or_default()
        .trim()
        .to_string()
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Decode a 77-bit FT8 message into a human-readable string.
///
/// Returns `None` if the message type is unsupported or the bits are
/// inconsistent (e.g. unused type codes, bad grid index).
///
/// Supported types:
/// - `0/0`  Free text
/// - `0/1`  DXpedition RR73
/// - `0/3`, `0/4`  ARRL Field Day (callsigns only, exchange shown as `[FD]`)
/// - `1`    Standard: `CALL1 CALL2 GRID` or `CALL1 CALL2 REPORT`
/// - `2`    Standard with `/P`
/// - `4`    One non-standard callsign + 12-bit hashed counterpart
pub fn unpack77(msg: &[u8]) -> Option<String> {
    let n3 = read_bits(msg, 71, 3);
    let i3 = read_bits(msg, 74, 3);

    match i3 {
        // ── Type 0: various sub-types ────────────────────────────────────
        0 => match n3 {
            0 => {
                let text = unpack_free_text(msg);
                if text.is_empty() { None } else { Some(text) }
            }
            1 => {
                // DXpedition: CALL1 RR73; CALL2 <hash> REPORT
                // Format: b28 b28 b10 b5
                let n28a = read_bits(msg, 0, 28);
                let n28b = read_bits(msg, 28, 28);
                let n5 = read_bits(msg, 66, 5);
                let irpt = 2 * n5 as i32 - 30;
                let crpt = if irpt >= 0 {
                    format!("+{:02}", irpt)
                } else {
                    format!("{:03}", irpt)
                };
                let c1 = unpack28(n28a);
                let c2 = unpack28(n28b);
                Some(format!("{} RR73; {} <...> {}", c1, c2, crpt))
            }
            3 | 4 => {
                // ARRL Field Day — show callsigns + tag
                let c1 = unpack28(read_bits(msg, 0, 28));
                let c2 = unpack28(read_bits(msg, 28, 28));
                Some(format!("{} {} [FD]", c1, c2))
            }
            _ => None,
        },

        // ── Type 1 / 2: standard or /P message ───────────────────────────
        1 | 2 => {
            // Format: b28 b1 b28 b1 b1 b15 b3
            let n28a = read_bits(msg, 0, 28);
            let ipa = msg[28] & 1;
            let n28b = read_bits(msg, 29, 28);
            let ipb = msg[57] & 1;
            let ir = msg[58] & 1;
            let igrid = read_bits(msg, 59, 15);

            let mut c1 = unpack28(n28a);
            let mut c2 = unpack28(n28b);

            // Append /R or /P if the flag bit is set (but not for CQ-type tokens)
            if ipa == 1 && !c1.starts_with('<') && !c1.starts_with("CQ") {
                c1.push_str(if i3 == 1 { "/R" } else { "/P" });
            }
            if ipb == 1 && !c2.starts_with('<') {
                c2.push_str(if i3 == 1 { "/R" } else { "/P" });
            }

            let report = if igrid <= MAX_GRID4 {
                let grid = to_grid4(igrid)?;
                if ir == 0 { grid } else { format!("R {}", grid) }
            } else {
                let irpt = igrid - MAX_GRID4;
                match irpt {
                    1 => String::new(),
                    2 => "RRR".to_string(),
                    3 => "RR73".to_string(),
                    4 => "73".to_string(),
                    n => {
                        let mut isnr = n as i32 - 35;
                        if isnr > 50 {
                            isnr -= 101;
                        }
                        fmt_report(isnr, ir)
                    }
                }
            };

            if report.is_empty() {
                Some(format!("{} {}", c1, c2))
            } else {
                Some(format!("{} {} {}", c1, c2, report))
            }
        }

        // ── Type 3: ARRL RTTY Contest ─────────────────────────────────────
        3 => {
            // Format (WSJT-X `packjt77.f90:514` `b1,2b28.28,b1,b3.3,b13.13,b3.3`):
            //   b1: itu (0 = US/Can, 1 = TU; prefix)
            //   b28: call1
            //   b28: call2
            //   b1: ir (0 = no prefix, 1 = "R " prefix on RST)
            //   b3: irpt → RST = 5{irpt+2}9 (e.g. irpt=6 → "589")
            //   b13: nexch → if `> 8000`, `imult = nexch-8000` indexes RTTY_STATES;
            //        else `nserial = nexch`, formatted as 4-digit serial
            //   b3: i3 (= 3, type marker)
            let itu = msg[0] & 1;
            let n28a = read_bits(msg, 1, 28);
            let n28b = read_bits(msg, 29, 28);
            let ir = msg[57] & 1;
            let irpt = read_bits(msg, 58, 3) as u8;
            let nexch = read_bits(msg, 61, 13);
            let c1 = unpack28(n28a);
            let c2 = unpack28(n28b);

            let rst = format!("5{}9", irpt + 2);
            let exch = if nexch > 8000 && (nexch as usize - 8000) <= RTTY_STATES.len() {
                RTTY_STATES[(nexch as usize - 8000) - 1].to_string()
            } else if (1..=7999).contains(&nexch) {
                format!("{:04}", nexch)
            } else {
                // Out-of-range exchange: keep the [RTTY] placeholder so callers
                // can still see the callsign pair without misleading state codes.
                return Some(format!("{} {} [RTTY]", c1, c2));
            };
            let prefix = if itu == 1 { "TU; " } else { "" };
            let r_prefix = if ir == 1 { "R " } else { "" };
            Some(format!(
                "{}{} {} {}{} {}",
                prefix, c1, c2, r_prefix, rst, exch
            ))
        }

        // ── Type 4: one non-standard call + 12-bit hash ───────────────────
        4 => {
            // Format: b12 b58 b1 b2 b1 (b3 = i3)
            let n58 = read_bits_u64(msg, 12, 58);
            let iflip = msg[70] & 1;
            let nrpt = read_bits(msg, 71, 2);
            let icq = msg[73] & 1;

            // Decode 11-char non-standard callsign from 58-bit base-38 number
            let mut n = n58;
            let mut buf = [b' '; 11];
            for i in (0..11).rev() {
                buf[i] = C38[(n % 38) as usize];
                n /= 38;
            }
            let nonstd = String::from_utf8(buf.to_vec())
                .unwrap_or_default()
                .trim()
                .to_string();

            if icq == 1 {
                return Some(format!("CQ {}", nonstd));
            }

            let (c1, c2) = if iflip == 0 {
                ("<...>".to_string(), nonstd)
            } else {
                (nonstd, "<...>".to_string())
            };

            match nrpt {
                0 => Some(format!("{} {}", c1, c2)),
                1 => Some(format!("{} {} RRR", c1, c2)),
                2 => Some(format!("{} {} RR73", c1, c2)),
                3 => Some(format!("{} {} 73", c1, c2)),
                _ => None,
            }
        }

        _ => None,
    }
}

/// Decode a 77-bit FT8 message, resolving hashed callsigns via a lookup table.
///
/// Behaves identically to [`unpack77`] but replaces `<...>` placeholders with
/// actual callsigns when they are found in the hash table.
pub fn unpack77_with_hash(msg: &[u8], ht: &CallsignHashTable) -> Option<String> {
    let n3 = read_bits(msg, 71, 3);
    let i3 = read_bits(msg, 74, 3);

    match i3 {
        0 => match n3 {
            0 => {
                let text = unpack_free_text(msg);
                if text.is_empty() { None } else { Some(text) }
            }
            1 => {
                // DXpedition: CALL1 RR73; CALL2 <hash10> REPORT
                let n28a = read_bits(msg, 0, 28);
                let n28b = read_bits(msg, 28, 28);
                let n10 = read_bits(msg, 56, 10);
                let n5 = read_bits(msg, 66, 5);
                let irpt = 2 * n5 as i32 - 30;
                let crpt = if irpt >= 0 {
                    format!("+{:02}", irpt)
                } else {
                    format!("{:03}", irpt)
                };
                let c1 = unpack28_h(n28a, ht);
                let c2 = unpack28_h(n28b, ht);
                let c3 = if let Some(call) = ht.lookup10(n10) {
                    format!("<{}>", call)
                } else {
                    "<...>".to_string()
                };
                Some(format!("{} RR73; {} {} {}", c1, c2, c3, crpt))
            }
            3 | 4 => {
                let c1 = unpack28_h(read_bits(msg, 0, 28), ht);
                let c2 = unpack28_h(read_bits(msg, 28, 28), ht);
                Some(format!("{} {} [FD]", c1, c2))
            }
            _ => None,
        },

        1 | 2 => {
            let n28a = read_bits(msg, 0, 28);
            let ipa = msg[28] & 1;
            let n28b = read_bits(msg, 29, 28);
            let ipb = msg[57] & 1;
            let ir = msg[58] & 1;
            let igrid = read_bits(msg, 59, 15);

            let mut c1 = unpack28_h(n28a, ht);
            let mut c2 = unpack28_h(n28b, ht);

            if ipa == 1 && !c1.starts_with('<') && !c1.starts_with("CQ") {
                c1.push_str(if i3 == 1 { "/R" } else { "/P" });
            }
            if ipb == 1 && !c2.starts_with('<') {
                c2.push_str(if i3 == 1 { "/R" } else { "/P" });
            }

            let report = if igrid <= MAX_GRID4 {
                let grid = to_grid4(igrid)?;
                if ir == 0 { grid } else { format!("R {}", grid) }
            } else {
                let irpt = igrid - MAX_GRID4;
                match irpt {
                    1 => String::new(),
                    2 => "RRR".to_string(),
                    3 => "RR73".to_string(),
                    4 => "73".to_string(),
                    n => {
                        let mut isnr = n as i32 - 35;
                        if isnr > 50 {
                            isnr -= 101;
                        }
                        fmt_report(isnr, ir)
                    }
                }
            };

            if report.is_empty() {
                Some(format!("{} {}", c1, c2))
            } else {
                Some(format!("{} {} {}", c1, c2, report))
            }
        }

        3 => {
            // Hashed-callsign variant of the ARRL RTTY Roundup unpack;
            // see `unpack77` for the bit-layout commentary.
            let itu = msg[0] & 1;
            let n28a = read_bits(msg, 1, 28);
            let n28b = read_bits(msg, 29, 28);
            let ir = msg[57] & 1;
            let irpt = read_bits(msg, 58, 3) as u8;
            let nexch = read_bits(msg, 61, 13);
            let c1 = unpack28_h(n28a, ht);
            let c2 = unpack28_h(n28b, ht);
            let rst = format!("5{}9", irpt + 2);
            let exch = if nexch > 8000 && (nexch as usize - 8000) <= RTTY_STATES.len() {
                RTTY_STATES[(nexch as usize - 8000) - 1].to_string()
            } else if (1..=7999).contains(&nexch) {
                format!("{:04}", nexch)
            } else {
                return Some(format!("{} {} [RTTY]", c1, c2));
            };
            let prefix = if itu == 1 { "TU; " } else { "" };
            let r_prefix = if ir == 1 { "R " } else { "" };
            Some(format!(
                "{}{} {} {}{} {}",
                prefix, c1, c2, r_prefix, rst, exch
            ))
        }

        4 => {
            let n12 = read_bits(msg, 0, 12);
            let n58 = read_bits_u64(msg, 12, 58);
            let iflip = msg[70] & 1;
            let nrpt = read_bits(msg, 71, 2);
            let icq = msg[73] & 1;

            let mut n = n58;
            let mut buf = [b' '; 11];
            for i in (0..11).rev() {
                buf[i] = C38[(n % 38) as usize];
                n /= 38;
            }
            let nonstd = String::from_utf8(buf.to_vec())
                .unwrap_or_default()
                .trim()
                .to_string();

            if icq == 1 {
                return Some(format!("CQ {}", nonstd));
            }

            let hashed = resolve_hash12(n12, ht);
            let (c1, c2) = if iflip == 0 {
                (hashed, nonstd)
            } else {
                (nonstd, hashed)
            };

            match nrpt {
                0 => Some(format!("{} {}", c1, c2)),
                1 => Some(format!("{} {} RRR", c1, c2)),
                2 => Some(format!("{} {} RR73", c1, c2)),
                3 => Some(format!("{} {} 73", c1, c2)),
                _ => None,
            }
        }

        _ => None,
    }
}

// ── Callsign validation ─────────────────────────────────────────────────────

/// Check if a callsign matches the standard amateur radio format.
///
/// Based on WSJT-X `MainWindow::stdCall` regex:
/// ```text
/// (part1)(part2)(/R|/P)?
/// part1: [A-Z]{0,2} | [A-Z][0-9] | [0-9][A-Z]
/// part2: [0-9][A-Z]{0,3}
/// ```
///
/// Examples: JA1ABC, 3Y0Z, W1AW, VK2RG/P
pub fn is_standard_callsign(call: &str) -> bool {
    let call = call.trim();
    // Strip /R or /P suffix
    let base = if call.ends_with("/R") || call.ends_with("/P") {
        &call[..call.len() - 2]
    } else {
        call
    };

    let b = base.as_bytes();
    if b.is_empty() || b.len() > 6 {
        return false;
    }

    // Find the boundary: part2 starts with a digit followed by letters
    // Scan from right to find the digit that starts part2
    // part2 = [0-9][A-Z]{0,3}
    let mut split = None;
    for i in (0..b.len()).rev() {
        if b[i].is_ascii_digit() {
            // Check remaining chars after this digit are all A-Z
            if b[i + 1..].iter().all(|&c| c.is_ascii_uppercase()) {
                split = Some(i);
                break;
            }
        }
    }
    let split = match split {
        Some(s) => s,
        None => return false,
    };

    let part1 = &b[..split];
    let part2 = &b[split..]; // [0-9][A-Z]{0,3}

    // Validate part2: digit + 0-3 uppercase letters
    if part2.is_empty() || !part2[0].is_ascii_digit() {
        return false;
    }
    if part2.len() > 4 {
        return false;
    }
    if !part2[1..].iter().all(|c| c.is_ascii_uppercase()) {
        return false;
    }

    // Validate part1: [A-Z]{0,2} | [A-Z][0-9] | [0-9][A-Z]
    match part1.len() {
        0 => true, // empty part1 is allowed
        1 => part1[0].is_ascii_uppercase() || part1[0].is_ascii_digit(),
        2 => {
            let (a, b) = (part1[0], part1[1]);
            (a.is_ascii_uppercase() && b.is_ascii_uppercase()) // [A-Z][A-Z]
            || (a.is_ascii_uppercase() && b.is_ascii_digit())  // [A-Z][0-9]
            || (a.is_ascii_digit() && b.is_ascii_uppercase()) // [0-9][A-Z]
        }
        _ => false,
    }
}

/// Check if a string has the structure of an amateur radio callsign base
/// (without portable/CEPT modifiers).
///
/// ITU Radio Regulations Article 19: a callsign consists of
/// `[prefix][digit][suffix]` where:
/// - prefix: 1-3 alphanumeric chars, at least one letter
/// - digit: one separating digit
/// - suffix: 1-4 uppercase letters (1x1 special stations have 1 letter)
fn is_base_callsign(s: &str) -> bool {
    let b = s.as_bytes();
    if b.len() < 2 || b.len() > 7 {
        return false;
    }

    // Find the rightmost digit followed by only letters — that's the
    // separating digit between prefix and suffix.
    let mut split = None;
    for i in (0..b.len()).rev() {
        if b[i].is_ascii_digit() && b[i + 1..].iter().all(|c| c.is_ascii_uppercase()) {
            split = Some(i);
            break;
        }
    }
    let split = match split {
        Some(s) if s + 1 < b.len() => s, // must have ≥1 letter suffix
        _ => return false,
    };

    let prefix = &b[..split];
    let suffix = &b[split + 1..];

    // Prefix: 1-3 chars, alphanumeric, at least one letter
    if prefix.is_empty() || prefix.len() > 3 {
        return false;
    }
    if !prefix.iter().all(|c| c.is_ascii_alphanumeric()) {
        return false;
    }
    if !prefix.iter().any(|c| c.is_ascii_alphabetic()) {
        return false;
    }

    // Suffix: 1-4 uppercase letters
    suffix.len() <= 4 && suffix.iter().all(|c| c.is_ascii_uppercase())
}

/// Check whether a string is a valid FT8 callsign (standard or non-standard).
///
/// Accepts callsigns per ITU Radio Regulations and FT8 encoding:
///
/// 1. **Standard** (pack28 format): handled by [`is_standard_callsign`].
/// 2. **Base callsign** without modifiers: e.g. `3DA0WPX` (7-char, Type 4).
/// 3. **Compound callsign** with `/`:
///    - `CALL/mod`: portable/mobile (`JA1ABC/P`, `JA1ABC/1`, `JA1ABC/QRP`)
///    - `prefix/CALL`: CEPT (`F/JA1ABC`, `ZS6/JA1ABC`)
///    - At least one side must be a valid base callsign; the other must be
///      a short modifier (1-3 alphanumeric chars).
pub fn is_valid_callsign(call: &str) -> bool {
    if is_standard_callsign(call) {
        return true;
    }

    let parts: Vec<&str> = call.split('/').collect();
    match parts.len() {
        1 => is_base_callsign(parts[0]),
        2 => {
            let (a, b) = (parts[0], parts[1]);
            let a_base = is_base_callsign(a);
            let b_base = is_base_callsign(b);
            // Short modifier: 1-3 alphanumeric chars (P, M, MM, AM, QRP, 1, etc.)
            let a_mod = !a.is_empty()
                && a.len() <= 3
                && a.as_bytes().iter().all(|c| c.is_ascii_alphanumeric());
            let b_mod = !b.is_empty()
                && b.len() <= 3
                && b.as_bytes().iter().all(|c| c.is_ascii_alphanumeric());

            (a_base && b_mod) || (a_mod && b_base) || (a_base && b_base)
        }
        _ => false,
    }
}

/// ITU-allocated **letter+digit** 2-char prefix list. The structural
/// `is_valid_callsign` accepts any letter+digit pair (e.g. `Z7` from
/// `Z74QTJ`), but real ITU amateur prefix series only allocate
/// specific letter+digit blocks (mostly digits 2-9 for small countries).
/// `Z7` and similar gaps are common landing spots for CRC-14
/// false-positive bit patterns, so allow-listing the real entries
/// catches garbage on the busy-band block-decode path without
/// needing the full ITU table for the (numerous) letter+letter and
/// digit+letter cases.
///
/// Source: ITU Radio Regulations Appendix 42 / DXCC entity prefixes,
/// 2024 revision. Sorted for binary search.
const VALID_LETTER_DIGIT_PREFIXES: &[&[u8; 2]] = &[
    b"A2", b"A3", b"A4", b"A5", b"A6", b"A7", b"A8", b"A9", b"B0", b"B1", b"B2", b"B3", b"B4",
    b"B5", b"B6", b"B7", b"B8", b"B9", b"C2", b"C3", b"C4", b"C5", b"C6", b"C7", b"C8", b"C9",
    b"D2", b"D3", b"D4", b"D6", b"D7", b"D8", b"D9", b"E2", b"E3", b"E4", b"E5", b"E6", b"E7",
    b"H2", b"H4", b"H6", b"H7", b"H8", b"H9", b"J2", b"J3", b"J5", b"J6", b"J7", b"J8", b"P2",
    b"P3", b"P4", b"P5", b"P6", b"P7", b"P8", b"P9", b"S0", b"S2", b"S5", b"S7", b"S9", b"T2",
    b"T3", b"T4", b"T5", b"T6", b"T7", b"T8", b"V2", b"V3", b"V4", b"V5", b"V6", b"V7", b"V8",
    b"Z2", b"Z3", b"Z6", b"Z8",
];

#[inline]
fn is_known_letter_digit_prefix(prefix: &[u8]) -> bool {
    if prefix.len() != 2 {
        return false;
    }
    let key: &[u8; 2] = match prefix.try_into() {
        Ok(k) => k,
        Err(_) => return false,
    };
    VALID_LETTER_DIGIT_PREFIXES.binary_search(&key).is_ok()
}

/// Stricter callsign validator than [`is_valid_callsign`] — gates the
/// CRC-14 false-positive filter in the FT8 block decoder.
///
/// The internal structural validator (`is_base_callsign`) accepts
/// any alphanumeric prefix that has at least one letter, including
/// letter+digit pairs the ITU never allocates for amateur use
/// (e.g. `Z7`, `Q4`). Random codewords passing CRC-14 land in those
/// gaps disproportionately often (`Z74QTJ/R`, `Q1FOO` — observed in
/// the qso3 busy-band block-decode path before this filter).
///
/// Compared to [`is_valid_callsign`]:
/// - Accepts standard callsigns ([`is_standard_callsign`]) and
///   letter+letter / digit+letter prefix base callsigns unchanged
///   (~all ITU 2-char allocations are letter+letter blocks).
/// - **Letter+digit 2-char prefixes** (the gap-prone case) must
///   appear in an internal ITU Appendix-42 allowlist (~80 entries).
/// - Compound `A/B`: at least one side must pass
///   `is_plausible_callsign`; the modifier side stays as today.
pub fn is_plausible_callsign(call: &str) -> bool {
    if !is_valid_callsign(call) {
        return false;
    }
    // Apply prefix allowlist on top of structural validation.
    let parts: Vec<&str> = call.split('/').collect();
    match parts.len() {
        1 => has_plausible_prefix(parts[0]),
        2 => {
            // Compound — accept iff at least one side is a base
            // callsign with a plausible ITU prefix. The modifier
            // side ("R", "P", "QRP", etc.) is short by structure
            // but doesn't qualify on its own; the base side carries
            // the country.
            let a_plausible = is_base_callsign(parts[0]) && has_plausible_prefix(parts[0]);
            let b_plausible = is_base_callsign(parts[1]) && has_plausible_prefix(parts[1]);
            a_plausible || b_plausible
        }
        _ => false,
    }
}

/// Locate the prefix of a base callsign (or a /-side that looks like
/// one) and check it against the letter+digit ITU allowlist. Other
/// prefix shapes (1-char letter, letter+letter, digit+letter, 3-char)
/// pass through — they cover ~all real ITU allocations.
fn has_plausible_prefix(s: &str) -> bool {
    let b = s.as_bytes();
    if b.len() < 2 || b.len() > 7 {
        // Short modifier or out-of-spec — defer to caller's compound
        // logic; `is_valid_callsign` already validated shape.
        return true;
    }
    // Strip trailing /R or /P (only meaningful on a full callsign,
    // but harmless to apply here).
    let b = if b.len() >= 2
        && b[b.len() - 2] == b'/'
        && (b[b.len() - 1] == b'R' || b[b.len() - 1] == b'P')
    {
        &b[..b.len() - 2]
    } else {
        b
    };
    // Find the rightmost digit followed by only letters → that's
    // the separator between prefix and suffix.
    let mut split = None;
    for i in (0..b.len()).rev() {
        if b[i].is_ascii_digit() && b[i + 1..].iter().all(|c| c.is_ascii_uppercase()) {
            split = Some(i);
            break;
        }
    }
    let split = match split {
        Some(s) => s,
        None => return true, // no separator → caller already handles
    };
    let prefix = &b[..split];
    // 1-char letter prefix: only F, G, I, K, M, N, R, W are
    // assigned to amateur as standalone (everything else uses a
    // 2-char prefix in practice). Q especially is reserved for
    // Q-codes — common landing spot for CRC false positives.
    if prefix.len() == 1 && prefix[0].is_ascii_uppercase() {
        return matches!(
            prefix[0],
            b'F' | b'G' | b'I' | b'K' | b'M' | b'N' | b'R' | b'W'
        );
    }
    // Letter+digit 2-char prefix: must be in the ITU allowlist
    // (the other gap-prone shape that catches CRC false-positives).
    if prefix.len() == 2 && prefix[0].is_ascii_uppercase() && prefix[1].is_ascii_digit() {
        return is_known_letter_digit_prefix(prefix);
    }
    true
}

/// Check if a decoded FT8 message looks plausible (not a false positive).
///
/// CRC-14 provides 1/16384 false-positive probability per candidate.  This
/// function adds a secondary filter by validating that callsign-like tokens
/// follow ITU format rules (must contain a digit) and use the FT8 character
/// set.  Special tokens (CQ, reports, grids, hash placeholders) are skipped.
pub fn is_plausible_message(text: &str) -> bool {
    let words: Vec<&str> = text.split_whitespace().collect();
    if words.is_empty() {
        return false;
    }

    // Contest/DXpedition markers — trust the unpack result
    if text.contains("[FD]") || text.contains("[RTTY]") || text.contains("RR73;") {
        return true;
    }

    for (idx, &w) in words.iter().enumerate() {
        // Known non-callsign tokens
        if matches!(
            w,
            "CQ" | "DE" | "QRZ" | "RRR" | "RR73" | "73" | "R" | "" | "DX"
        ) {
            continue;
        }
        // "CQ NNN" compound tokens
        if w.starts_with("CQ") {
            continue;
        }
        // CQ activity suffix: token right after CQ, all uppercase ≤4 chars
        // e.g., POTA, SOTA, NA, EU (unpack28 directional CQ, C4 alphabet)
        if idx == 1 && words[0] == "CQ" && w.len() <= 4 && w.bytes().all(|b| b.is_ascii_uppercase())
        {
            continue;
        }
        // Hash placeholder
        if w.starts_with('<') && w.ends_with('>') {
            continue;
        }
        // Reports: R+NN, R-NN, +NN, -NN
        if w.starts_with("R+") || w.starts_with("R-") {
            continue;
        }
        if (w.starts_with('+') || w.starts_with('-')) && w[1..].parse::<i32>().is_ok() {
            continue;
        }
        // 4-char grid locator
        if w.len() == 4 {
            let b = w.as_bytes();
            if b[0].is_ascii_uppercase()
                && b[1].is_ascii_uppercase()
                && b[2].is_ascii_digit()
                && b[3].is_ascii_digit()
            {
                continue;
            }
        }

        // Remaining tokens should be callsigns — validate against
        // the ITU prefix allowlist (stricter than is_valid_callsign,
        // catches CRC-14 false positives whose decoded callsign-like
        // tokens land in unallocated letter+digit prefix gaps).
        if !is_plausible_callsign(w) {
            return false;
        }
    }
    true
}

// ── Packing (encode) ────────────────────────────────────────────────────────

/// Write `len` bits of `val` (MSB first) into `msg` starting at `start`.
fn write_bits(msg: &mut [u8; 77], start: usize, len: usize, val: u32) {
    for i in 0..len {
        msg[start + i] = ((val >> (len - 1 - i)) & 1) as u8;
    }
}

/// Pack a callsign into a 28-bit token (inverse of `unpack28`).
///
/// Supports `"DE"`, `"QRZ"`, `"CQ"`, and standard 1–6 character callsigns
/// whose 3rd character (1-indexed) is a digit (e.g. `"JQ1QSO"`, `"3Y0Z"`).
///
/// Returns `None` if the callsign contains characters outside the FT8 alphabet
/// or cannot be encoded in the standard 28-bit field.
pub fn pack28(call: &str) -> Option<u32> {
    let call = call.trim();
    match call {
        "DE" => return Some(0),
        "QRZ" => return Some(1),
        "CQ" => return Some(2),
        _ => {}
    }

    // CQ with suffix: "CQ NNN" or "CQ XXXX"
    if let Some(suffix) = call.strip_prefix("CQ ") {
        let suffix = suffix.trim();
        if !suffix.is_empty() {
            // Numeric suffix: "CQ 001" - "CQ 999"
            if let Ok(n) = suffix.parse::<u32>()
                && n <= 999
            {
                return Some(3 + n);
            }
            // Directional suffix: "CQ POTA", "CQ DX", etc. (1-4 uppercase letters)
            let sb = suffix.as_bytes();
            if sb.len() <= 4 && sb.iter().all(|c| c.is_ascii_uppercase()) {
                let mut buf = [b' '; 4];
                for (i, &b) in sb.iter().enumerate() {
                    buf[i] = b;
                }
                let i1 = C4.iter().position(|&c| c == buf[0])?;
                let i2 = C4.iter().position(|&c| c == buf[1])?;
                let i3 = C4.iter().position(|&c| c == buf[2])?;
                let i4 = C4.iter().position(|&c| c == buf[3])?;
                return Some(1003 + ((i1 * 27 + i2) * 27 + i3) as u32 * 27 + i4 as u32);
            }
            return None; // Invalid CQ suffix
        }
    }

    let bytes = call.as_bytes();
    if bytes.is_empty() || bytes.len() > 6 {
        return None;
    }

    // Pad to 6 characters: if position 3 (1-indexed) is not a digit, prepend space.
    let mut buf = [b' '; 6];
    if bytes.len() >= 3 && bytes[2].is_ascii_digit() {
        // Digit already at position 3 — left-align
        for (i, &b) in bytes.iter().enumerate().take(6) {
            buf[i] = b.to_ascii_uppercase();
        }
    } else if bytes.len() >= 2 && bytes[1].is_ascii_digit() {
        // Digit at position 2 — shift right by 1 so digit lands at position 3
        buf[0] = b' ';
        for (i, &b) in bytes.iter().enumerate() {
            if i + 1 < 6 {
                buf[i + 1] = b.to_ascii_uppercase();
            }
        }
    } else {
        return None; // Cannot form a valid 6-char callsign
    }

    // Position 3 (index 2) must be a digit
    if !buf[2].is_ascii_digit() {
        return None;
    }

    let i1 = C1.iter().position(|&c| c == buf[0])?;
    let i2 = C2.iter().position(|&c| c == buf[1])?;
    let i3 = C3.iter().position(|&c| c == buf[2])?;
    let i4 = C4.iter().position(|&c| c == buf[3])?;
    let i5 = C4.iter().position(|&c| c == buf[4])?;
    let i6 = C4.iter().position(|&c| c == buf[5])?;

    let n = ((((i1 as u32 * 36 + i2 as u32) * 10 + i3 as u32) * 27 + i4 as u32) * 27 + i5 as u32)
        * 27
        + i6 as u32;
    Some(NTOKENS + MAX22 + n)
}

/// Pack a 4-character Maidenhead grid locator into a 15-bit index.
pub fn pack_grid4(grid: &str) -> Option<u32> {
    let g = grid.as_bytes();
    if g.len() != 4 {
        return None;
    }
    let j1 = g[0].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    let j2 = g[1].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    let j3 = g[2].wrapping_sub(b'0') as u32;
    let j4 = g[3].wrapping_sub(b'0') as u32;
    if j1 > 17 || j2 > 17 || j3 > 9 || j4 > 9 {
        return None;
    }
    Some(((j1 * 18 + j2) * 10 + j3) * 10 + j4)
}

/// Pack a Type 1 standard message: `"CALL1 CALL2 GRID"`.
///
/// Both callsigns must be packable via [`pack28`], and `grid` must be a valid
/// 4-character Maidenhead locator.  Returns the 77-bit message array.
pub fn pack77_type1(call1: &str, call2: &str, grid: &str) -> Option<[u8; 77]> {
    let n28a = pack28(call1)?;
    let n28b = pack28(call2)?;
    let igrid = pack_grid4(grid)?;

    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, n28a); // call1 (bits 0–27)
    // ipa = 0 (bit 28) — already zero
    write_bits(&mut msg, 29, 28, n28b); // call2 (bits 29–56)
    // ipb = 0 (bit 57) — already zero
    // ir  = 0 (bit 58) — already zero
    write_bits(&mut msg, 59, 15, igrid); // grid  (bits 59–73)
    write_bits(&mut msg, 74, 3, 1); // i3=1  (bits 74–76)
    Some(msg)
}

/// Pack a Type 1 standard message with any report/grid field.
///
/// `report` can be:
/// - A 4-char grid locator: `"PM95"`
/// - A dB signal report: `"-12"`, `"+05"`
/// - An R-prefixed report: `"R-12"`, `"R+05"`
/// - A standard response: `"RRR"`, `"RR73"`, `"73"`
/// - Empty string (no report)
///
/// # Examples
/// ```
/// # use mfsk_core::msg::wsjt77::pack77;
/// let msg = pack77("CQ", "JA1ABC", "PM95").unwrap();
/// let msg = pack77("JA1ABC", "3Y0Z", "-12").unwrap();
/// let msg = pack77("3Y0Z", "JA1ABC", "R-12").unwrap();
/// let msg = pack77("JA1ABC", "3Y0Z", "RR73").unwrap();
/// ```
pub fn pack77(call1: &str, call2: &str, report: &str) -> Option<[u8; 77]> {
    let n28a = pack28(call1)?;
    let n28b = pack28(call2)?;

    let report = report.trim();

    // Determine igrid and ir flag
    let (igrid, ir): (u32, u8) = if report.is_empty() {
        (MAX_GRID4 + 1, 0)
    } else if report == "RRR" {
        (MAX_GRID4 + 2, 0)
    } else if report == "RR73" {
        (MAX_GRID4 + 3, 0)
    } else if report == "73" {
        (MAX_GRID4 + 4, 0)
    } else if report.len() == 4 && pack_grid4(report).is_some() {
        // Grid locator (e.g. "PM95")
        (pack_grid4(report).unwrap(), 0)
    } else {
        // dB report: "-12", "+05", "R-12", "R+05"
        let (r_prefix, num_str) = if let Some(s) = report.strip_prefix('R') {
            (1u8, s)
        } else {
            (0u8, report)
        };
        let snr: i32 = num_str.parse().ok()?;
        if !(-50..=49).contains(&snr) {
            return None;
        }
        let mut isnr = snr + 35;
        if isnr < 0 {
            isnr += 101;
        }
        (MAX_GRID4 + isnr as u32, r_prefix)
    };

    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, n28a);
    // ipa = 0 (bit 28)
    write_bits(&mut msg, 29, 28, n28b);
    // ipb = 0 (bit 57)
    msg[58] = ir; // ir (bit 58)
    write_bits(&mut msg, 59, 15, igrid);
    write_bits(&mut msg, 74, 3, 1); // i3=1
    Some(msg)
}

/// Write `len` bits of a u64 `val` (MSB first) into `msg` starting at `start`.
fn write_bits_u64(msg: &mut [u8; 77], start: usize, len: usize, val: u64) {
    for i in 0..len {
        msg[start + i] = ((val >> (len - 1 - i)) & 1) as u8;
    }
}

/// Pack a Type 4 message: one non-standard callsign + one hashed standard
/// callsign, or `CQ nonstd`.
///
/// # Arguments
/// * `nonstd` — non-standard callsign (1-11 chars from C38 alphabet)
/// * `std_call` — standard callsign to 12-bit hash (ignored when `is_cq`)
/// * `report` — `""`, `"RRR"`, `"RR73"`, or `"73"`
/// * `is_cq` — if true, packs `"CQ nonstd"` (CQ flag set)
///
/// # Layout (77 bits)
/// ```text
/// [12-bit hash][58-bit base-38 nonstd][1-bit iflip][2-bit nrpt][1-bit icq][3-bit i3=4]
/// ```
pub fn pack77_type4(nonstd: &str, std_call: &str, report: &str, is_cq: bool) -> Option<[u8; 77]> {
    let nonstd = nonstd.trim().to_ascii_uppercase();
    let nb = nonstd.as_bytes();
    if nb.is_empty() || nb.len() > 11 {
        return None;
    }
    if !nb.iter().all(|c| C38.contains(c)) {
        return None;
    }

    // Encode non-standard callsign as 58-bit base-38 number
    let mut n58: u64 = 0;
    // Pad to 11 characters with leading spaces
    let mut padded = [b' '; 11];
    let offset = 11 - nb.len();
    for (i, &b) in nb.iter().enumerate() {
        padded[offset + i] = b;
    }
    for &ch in &padded {
        let idx = C38.iter().position(|&c| c == ch)?;
        n58 = n58 * 38 + idx as u64;
    }

    // 12-bit hash of standard callsign
    let n12 = if is_cq {
        0u32 // unused when CQ flag is set
    } else {
        use super::hash_table::ihashcall;
        ihashcall(std_call, 12)
    };

    // Report encoding
    let nrpt: u32 = match report.trim() {
        "" => 0,
        "RRR" => 1,
        "RR73" => 2,
        "73" => 3,
        _ => return None,
    };

    // iflip: 0 = <hash> nonstd, 1 = nonstd <hash>
    // When std_call packs via pack28, place hash first (iflip=0).
    // Otherwise nonstd first (iflip=1).
    let iflip: u8 = if is_cq || pack28(std_call).is_some() {
        0
    } else {
        1
    };

    let icq: u8 = if is_cq { 1 } else { 0 };

    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 12, n12); // 12-bit hash (bits 0-11)
    write_bits_u64(&mut msg, 12, 58, n58); // 58-bit base-38 (bits 12-69)
    msg[70] = iflip; // iflip (bit 70)
    write_bits(&mut msg, 71, 2, nrpt); // nrpt (bits 71-72)
    msg[73] = icq; // icq (bit 73)
    write_bits(&mut msg, 74, 3, 4); // i3=4 (bits 74-76)
    Some(msg)
}

/// Pack a free-text message (Type 0, n3=0).
///
/// `text` — up to 13 characters from the FREE_TEXT alphabet
/// (`0-9 A-Z + - . / ?` and space).  Shorter text is right-padded with spaces.
///
/// # Examples
/// ```
/// # use mfsk_core::msg::wsjt77::{pack77_free_text, unpack77};
/// let msg = pack77_free_text("JA/TK-001").unwrap();
/// assert_eq!(unpack77(&msg).unwrap(), "JA/TK-001");
/// ```
pub fn pack77_free_text(text: &str) -> Option<[u8; 77]> {
    let text = text.to_ascii_uppercase();
    let bytes = text.as_bytes();
    if bytes.is_empty() || bytes.len() > 13 {
        return None;
    }

    // Pad to 13 characters with trailing spaces
    let mut padded = [b' '; 13];
    for (i, &b) in bytes.iter().enumerate() {
        padded[i] = b;
    }

    // Encode as base-42 number (fits in 71 bits: 42^13 ≈ 2^71.4)
    let mut n: u128 = 0;
    for &ch in &padded {
        let idx = FREE_TEXT.iter().position(|&c| c == ch)? as u128;
        n = n * 42 + idx;
    }

    let mut msg = [0u8; 77];
    for i in 0..71 {
        msg[i] = ((n >> (70 - i)) & 1) as u8;
    }
    // bits 71-76 = 0 (i3=0, n3=0) — already zero
    Some(msg)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod report_format_tests {
    use super::fmt_report;

    /// WSJT-X renders both signs with two digits (`packjt77.f90:504`'s
    /// `i3.2`). Single-digit magnitudes are the only ones this ever got
    /// wrong, and they are common in real traffic.
    #[test]
    fn single_digit_reports_are_zero_padded_like_wsjtx() {
        assert_eq!(fmt_report(-8, 0), "-08");
        assert_eq!(fmt_report(-8, 1), "R-08");
        assert_eq!(fmt_report(8, 0), "+08");
        assert_eq!(fmt_report(8, 1), "R+08");
        assert_eq!(fmt_report(0, 0), "+00");
    }

    /// Two-digit magnitudes were already correct; pin them so a fix to
    /// the above can't regress them.
    #[test]
    fn two_digit_reports_are_unchanged() {
        assert_eq!(fmt_report(-23, 0), "-23");
        assert_eq!(fmt_report(-23, 1), "R-23");
        assert_eq!(fmt_report(15, 0), "+15");
        assert_eq!(fmt_report(-30, 0), "-30");
    }

    /// The exact strings real `jt9` printed for the four FT8 decodes
    /// that exposed this, on `qso3_busy.wav`.
    #[test]
    fn matches_the_jt9_strings_that_exposed_the_bug() {
        assert_eq!(
            format!("W1FC F5BZB {}", fmt_report(-8, 0)),
            "W1FC F5BZB -08"
        );
        assert_eq!(
            format!("WM3PEN EA6VQ {}", fmt_report(-9, 0)),
            "WM3PEN EA6VQ -09"
        );
        assert_eq!(
            format!("N1JFU EA6EE {}", fmt_report(-7, 1)),
            "N1JFU EA6EE R-07"
        );
        assert_eq!(
            format!("K1BZM EA3GP {}", fmt_report(-9, 0)),
            "K1BZM EA3GP -09"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plausible_callsign_accepts_real_calls() {
        // Standard 2-char letter+letter prefixes — most real amateur calls.
        for c in [
            "W1AW", "JA1XYZ", "DL3DB", "EA6VQ", "HB9CQK", "F5RXL", "G3WDG", "VK6ABC", "K1JT",
            "N1PJT", "JL1NIE", "WM3PEN",
        ] {
            assert!(is_plausible_callsign(c), "should accept {c}");
        }
        // Letter+digit prefix (ITU-allocated): A2 (Botswana), V5 (Namibia),
        // S5 (Slovenia), T7 (San Marino).
        for c in ["A22ZZ", "V51AAA", "S55BC", "T77QQ"] {
            assert!(is_plausible_callsign(c), "should accept {c}");
        }
        // Digit+letter prefix: 3D2, 4X, 5B, 9V — all real.
        for c in ["3D2RA", "4X4ABC", "5B4XYZ", "9V1ABC"] {
            assert!(is_plausible_callsign(c), "should accept {c}");
        }
        // Compound / portable
        for c in ["JA1XYZ/P", "JA1XYZ/QRP", "F/JA1XYZ", "KH6/N1ABC"] {
            assert!(is_plausible_callsign(c), "should accept {c}");
        }
    }

    #[test]
    fn plausible_callsign_rejects_letter_digit_gaps() {
        // Prefixes outside the ITU letter+digit allowlist — common
        // landing spots for CRC-14 false-positive bit patterns.
        for c in [
            "Z74QTJ", // observed qso3 garbage
            "Q1ABC",  // Q reserved (no amateur)
            "Q4ABCD", "X0FOO", // X+digit unassigned
            "Y0ABC",
        ] {
            assert!(
                !is_plausible_callsign(c),
                "should reject {c} (unallocated letter+digit prefix)"
            );
        }
    }

    #[test]
    fn plausible_callsign_compound_garbage() {
        // Compound where one side is garbage but the other passes —
        // accept (mirrors WSJT-X's tolerance for portable modifiers).
        assert!(is_plausible_callsign("JA1XYZ/P"));
        // Compound where both sides have unallocated letter+digit
        // prefixes — reject.
        assert!(!is_plausible_callsign("Z74QTJ/Q4ABCD"));
        // Compound with one Z7-prefix base + valid mod token — reject
        // (mod alone can't make Z74QTJ plausible).
        assert!(!is_plausible_callsign("Z74QTJ/R"));
    }

    /// Regression: `n28` in the extended CQ-XXXX region (3..NTOKENS) could
    /// panic with an out-of-bounds C4 access. AP-decoded garbage codewords
    /// can land there; unpack28 must degrade gracefully.
    #[test]
    fn unpack28_does_not_panic_for_extended_range() {
        for n28 in [1003u32, 532443, 532444, 1_000_000, NTOKENS - 1] {
            let _ = unpack28(n28);
        }
    }

    /// Unpack a hex string (20 hex chars = 10 bytes) into a [u8; 77] bit array.
    fn hex_to_msg77(hex: &str) -> [u8; 77] {
        assert_eq!(hex.len(), 20, "need exactly 20 hex chars (10 bytes)");
        let bytes: Vec<u8> = (0..10)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap())
            .collect();
        let mut msg = [0u8; 77];
        for (j, bit) in msg.iter_mut().enumerate() {
            *bit = (bytes[j / 8] >> (7 - j % 8)) & 1;
        }
        msg
    }

    #[test]
    fn decode_cq_r7iw_ln35() {
        // From 191111_110200.wav @ 1290.6 Hz (errors=1, BP)
        let msg = hex_to_msg77("0000002059654a94a3c8");
        let text = unpack77(&msg).expect("should decode");
        assert_eq!(text, "CQ R7IW LN35");
    }

    #[test]
    fn decode_cq_dx_r6wa_ln32() {
        // From 191111_110200.wav @ 2096.9 Hz (errors=0, BP)
        let msg = hex_to_msg77("000046f059519f14a308");
        let text = unpack77(&msg).expect("should decode");
        assert_eq!(text, "CQ DX R6WA LN32");
    }

    #[test]
    fn silence_bits_returns_none_or_empty() {
        let msg = [0u8; 77];
        // i3=0, n3=0 → free text, but all-zero = all-spaces → empty → None
        assert!(unpack77(&msg).is_none());
    }

    #[test]
    fn pack28_roundtrip() {
        // Standard callsigns
        for call in &["JQ1QSO", "3Y0Z", "R7IW", "JA1ABC", "W1AW", "VK2RG"] {
            let n = pack28(call).unwrap_or_else(|| panic!("pack28 failed for {call}"));
            let decoded = unpack28(n);
            assert_eq!(
                decoded,
                call.trim(),
                "roundtrip mismatch for {call}: got {decoded}"
            );
        }
        // Special tokens
        assert_eq!(pack28("CQ"), Some(2));
        assert_eq!(pack28("DE"), Some(0));
        assert_eq!(pack28("QRZ"), Some(1));

        // CQ with directional suffix — roundtrip
        for cq in &["CQ POTA", "CQ SOTA", "CQ DX", "CQ NA", "CQ EU"] {
            let n = pack28(cq).unwrap_or_else(|| panic!("pack28 failed for {cq}"));
            let decoded = unpack28(n);
            assert_eq!(decoded, *cq, "CQ suffix roundtrip mismatch for {cq}");
        }

        // CQ with numeric suffix
        let n = pack28("CQ 001").unwrap();
        assert_eq!(unpack28(n), "CQ 001");
        let n = pack28("CQ 999").unwrap();
        assert_eq!(unpack28(n), "CQ 999");
    }

    #[test]
    fn pack77_type1_roundtrip() {
        let msg = pack77_type1("CQ", "3Y0Z", "JD34").expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert_eq!(text, "CQ 3Y0Z JD34");

        let msg2 = pack77_type1("CQ", "JQ1QSO", "PM95").expect("pack failed");
        let text2 = unpack77(&msg2).expect("unpack failed");
        assert_eq!(text2, "CQ JQ1QSO PM95");
    }

    #[test]
    fn standard_callsign_valid() {
        assert!(is_standard_callsign("JA1ABC"));
        assert!(is_standard_callsign("3Y0Z"));
        assert!(is_standard_callsign("W1AW"));
        assert!(is_standard_callsign("VK2RG"));
        assert!(is_standard_callsign("R7IW"));
        assert!(is_standard_callsign("JQ1QSO"));
        assert!(is_standard_callsign("TA6CQ"));
        assert!(is_standard_callsign("JA1ABC/P"));
        assert!(is_standard_callsign("JM1VWQ/R"));
    }

    #[test]
    fn standard_callsign_invalid() {
        assert!(!is_standard_callsign("NFW/0811"));
        assert!(!is_standard_callsign("791JLI"));
        assert!(!is_standard_callsign(""));
        assert!(!is_standard_callsign("ABCDEFG"));
        assert!(!is_standard_callsign("123"));
    }

    #[test]
    fn standard_callsign_edge_cases() {
        assert!(is_standard_callsign("SY2XHO")); // SY prefix (Greece)
        assert!(is_standard_callsign("8I9NIH")); // 8I prefix
    }

    #[test]
    fn valid_callsign_standard() {
        // Standard pack28 format
        assert!(is_valid_callsign("JA1ABC"));
        assert!(is_valid_callsign("3Y0Z"));
        assert!(is_valid_callsign("W1AW"));
        assert!(is_valid_callsign("W1AW/P"));
        assert!(is_valid_callsign("JM1VWQ/R"));
        assert!(is_valid_callsign("W1A")); // 1x1 special event
    }

    #[test]
    fn valid_callsign_nonstandard() {
        // Type 4: CEPT, area indicators, long prefixes
        assert!(is_valid_callsign("JL1NIE/1")); // area indicator
        assert!(is_valid_callsign("JL1NIE/P")); // portable (also standard)
        assert!(is_valid_callsign("F/JA1ABC")); // CEPT prefix
        assert!(is_valid_callsign("ZS6/JA1ABC")); // country/call
        assert!(is_valid_callsign("JR9ECD/P")); // portable
        assert!(is_valid_callsign("3DA0WPX")); // 7-char call (3-char prefix)
        assert!(is_valid_callsign("JA1ABC/QRP")); // QRP modifier
    }

    #[test]
    fn valid_callsign_rejected() {
        assert!(!is_valid_callsign("NFW/0811")); // no valid base call on either side
        assert!(!is_valid_callsign("ABCDEF")); // no digit
        assert!(!is_valid_callsign(""));
        assert!(!is_valid_callsign("A")); // too short
        assert!(!is_valid_callsign("HELLO+WORLD")); // non-C38 characters
        assert!(!is_valid_callsign("123")); // no letter suffix
        assert!(!is_valid_callsign("//////")); // nonsense
    }

    #[test]
    fn plausible_message_standard() {
        assert!(is_plausible_message("CQ JA1ABC PM95"));
        assert!(is_plausible_message("CQ DX R6WA LN32"));
        assert!(is_plausible_message("JA1ABC 3Y0Z -12"));
        assert!(is_plausible_message("JA1ABC 3Y0Z RRR"));
        assert!(is_plausible_message("JA1ABC 3Y0Z 73"));
        assert!(is_plausible_message("CQ 3Y0Z JD34"));
        assert!(is_plausible_message("OH3NIV ZS6S R-12"));
    }

    #[test]
    fn plausible_message_nonstandard() {
        // Type 4 non-standard callsigns
        assert!(is_plausible_message("JR1UJX/P JH1GIN PM96"));
        assert!(is_plausible_message("<...> JH4IUV/P RR73"));
        assert!(is_plausible_message("CQ JR9ECD/P"));
        assert!(is_plausible_message("F/JA1ABC 3Y0Z -12"));
        assert!(is_plausible_message("CQ SOTA JL1NIE/1"));

        // Hash placeholders
        assert!(is_plausible_message("<...> JA1ABC -12"));
        assert!(is_plausible_message("JA1ABC <...> RRR"));

        // CQ with activity suffix
        assert!(is_plausible_message("CQ POTA JA1ABC PM95"));
        assert!(is_plausible_message("CQ NA W1AW FN31"));
        assert!(is_plausible_message("CQ SOTA JL1NIE/P"));

        // Contest/DXpedition markers
        assert!(is_plausible_message("JA1ABC 3Y0Z [FD]"));
    }

    #[test]
    fn plausible_message_rejected() {
        // No valid callsign structure
        assert!(!is_plausible_message("NFW/0811 73"));
        assert!(!is_plausible_message("ABCDEF GHIJKL"));
        assert!(!is_plausible_message(""));
    }

    #[test]
    fn pack77_type4_roundtrip() {
        // CQ with non-standard callsign
        let msg = pack77_type4("JL1NIE/P", "", "", true).expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert_eq!(text, "CQ JL1NIE/P");

        // Non-standard + hashed, no report
        let msg = pack77_type4("JL1NIE/1", "JA1ABC", "", false).expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert!(
            text.contains("JL1NIE/1"),
            "should contain non-std call: {text}"
        );
        assert!(
            text.contains("<...>"),
            "should contain hash placeholder: {text}"
        );

        // Non-standard + hashed, with 73
        let msg = pack77_type4("JR9ECD/P", "W1AW", "73", false).expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert!(text.contains("JR9ECD/P"), "got: {text}");
        assert!(text.contains("73"), "got: {text}");

        // F/JA1ABC (CEPT)
        let msg = pack77_type4("F/JA1ABC", "W1AW", "RR73", false).expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert!(text.contains("F/JA1ABC"), "got: {text}");
        assert!(text.contains("RR73"), "got: {text}");
    }

    #[test]
    fn type4_hash_register_then_resolve() {
        // Simulate the real flow: pack Type 4 → register std_call in hash table
        // → unpack with hash table → hashed callsign should resolve.
        let mut ht = CallsignHashTable::new();
        ht.insert("JA1ABC");

        // pack: JL1NIE/1 (non-std) + JA1ABC (std, will be 12-bit hashed)
        let msg = pack77_type4("JL1NIE/1", "JA1ABC", "", false).expect("pack failed");

        // unpack WITHOUT hash table → shows <...>
        let text_no_ht = unpack77(&msg).expect("unpack failed");
        assert!(
            text_no_ht.contains("<...>"),
            "without hash table: {text_no_ht}"
        );
        assert!(
            text_no_ht.contains("JL1NIE/1"),
            "without hash table: {text_no_ht}"
        );

        // unpack WITH hash table → resolves <JA1ABC>
        let text_ht = unpack77_with_hash(&msg, &ht).expect("unpack failed");
        assert!(
            text_ht.contains("<JA1ABC>"),
            "with hash table should resolve: {text_ht}"
        );
        assert!(text_ht.contains("JL1NIE/1"), "with hash table: {text_ht}");

        // Verify the resolved message passes plausibility
        assert!(
            is_plausible_message(&text_ht),
            "resolved message should be plausible: {text_ht}"
        );
    }

    #[test]
    fn pack77_type4_cq_with_pack77() {
        // pack77 should work with CQ + non-standard callsign that doesn't pack via pack28
        // This test ensures the Type 4 path produces valid messages
        let msg = pack77_type4("JL1NIE/1", "", "", true).expect("pack failed");
        let text = unpack77(&msg).expect("unpack failed");
        assert_eq!(text, "CQ JL1NIE/1");

        // Verify it passes plausibility
        assert!(is_plausible_message(&text));
    }

    #[test]
    fn pack77_free_text_roundtrip() {
        // SOTA references
        let msg = pack77_free_text("JA/TK-001").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA/TK-001");

        // POTA references
        let msg = pack77_free_text("JP-1001").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JP-1001");

        // JCC number
        let msg = pack77_free_text("JCC 100110").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JCC 100110");

        // Max length (13 chars)
        let msg = pack77_free_text("HELLO FT8 WLD").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "HELLO FT8 WLD");

        // Invalid: too long
        assert!(pack77_free_text("ABCDEFGHIJKLMN").is_none()); // 14 chars

        // Invalid: non-FREE_TEXT character
        assert!(pack77_free_text("HELLO!").is_none()); // '!' not in alphabet
    }

    #[test]
    fn pack77_report_roundtrip() {
        // Grid
        let msg = pack77("CQ", "JA1ABC", "PM95").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "CQ JA1ABC PM95");

        // dB report
        let msg = pack77("JA1ABC", "3Y0Z", "-12").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA1ABC 3Y0Z -12");

        let msg = pack77("JA1ABC", "3Y0Z", "+05").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA1ABC 3Y0Z +05");

        // R-report
        let msg = pack77("3Y0Z", "JA1ABC", "R-12").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "3Y0Z JA1ABC R-12");

        // RRR / RR73 / 73
        let msg = pack77("JA1ABC", "3Y0Z", "RRR").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA1ABC 3Y0Z RRR");

        let msg = pack77("JA1ABC", "3Y0Z", "RR73").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA1ABC 3Y0Z RR73");

        let msg = pack77("3Y0Z", "JA1ABC", "73").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "3Y0Z JA1ABC 73");

        // Empty report
        let msg = pack77("JA1ABC", "3Y0Z", "").unwrap();
        assert_eq!(unpack77(&msg).unwrap(), "JA1ABC 3Y0Z");
    }
}
