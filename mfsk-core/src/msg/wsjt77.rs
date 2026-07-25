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

/// ARRL/RAC Field Day sections used by WSJT-X Type 0.3/0.4 messages.
const FIELD_DAY_SECTIONS: &[&str] = &[
    "AB", "AK", "AL", "AR", "AZ", "BC", "CO", "CT", "DE", "EB", "EMA", "ENY", "EPA", "EWA", "GA",
    "GH", "IA", "ID", "IL", "IN", "KS", "KY", "LA", "LAX", "NS", "MB", "MDC", "ME", "MI", "MN",
    "MO", "MS", "MT", "NC", "ND", "NE", "NFL", "NH", "NL", "NLI", "NM", "NNJ", "NNY", "TER", "NTX",
    "NV", "OH", "OK", "ONE", "ONN", "ONS", "OR", "ORG", "PAC", "PR", "QC", "RI", "SB", "SC", "SCV",
    "SD", "SDG", "SF", "SFL", "SJV", "SK", "SNJ", "STX", "SV", "TN", "UT", "VA", "VI", "VT", "WCF",
    "WI", "WMA", "WNY", "WPA", "WTX", "WV", "WWA", "WY", "DX", "PE", "NB",
];

// ── Token boundaries ─────────────────────────────────────────────────────────

const NTOKENS: u32 = 2_063_592;
const MAX22: u32 = 4_194_304;
const MAX_GRID4: u32 = 32_400;

// ── Internal helpers ─────────────────────────────────────────────────────────

/// Read `len` bits starting at `start` from `msg` (MSB first) into a u32.
fn read_bits(msg: &[u8; 77], start: usize, len: usize) -> u32 {
    let mut n = 0u32;
    for i in start..start + len {
        n = (n << 1) | (msg[i] & 1) as u32;
    }
    n
}

/// Same as `read_bits` but returns u64 (for the 58-bit field in Type 4).
fn read_bits_u64(msg: &[u8; 77], start: usize, len: usize) -> u64 {
    let mut n = 0u64;
    for i in start..start + len {
        n = (n << 1) | (msg[i] & 1) as u64;
    }
    n
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

/// Decode the 25-bit six-character grid used by WSJT77 Type 5.
fn to_grid6(n: u32) -> Option<String> {
    if n > 18_662_399 {
        return None;
    }
    let j1 = n / (18 * 10 * 10 * 24 * 24);
    let rest = n % (18 * 10 * 10 * 24 * 24);
    let j2 = rest / (10 * 10 * 24 * 24);
    let rest = rest % (10 * 10 * 24 * 24);
    let j3 = rest / (10 * 24 * 24);
    let rest = rest % (10 * 24 * 24);
    let j4 = rest / (24 * 24);
    let rest = rest % (24 * 24);
    let j5 = rest / 24;
    let j6 = rest % 24;
    if j1 > 17 || j2 > 17 || j3 > 9 || j4 > 9 || j5 > 23 || j6 > 23 {
        return None;
    }
    Some(format!(
        "{}{}{}{}{}{}",
        (b'A' + j1 as u8) as char,
        (b'A' + j2 as u8) as char,
        (b'0' + j3 as u8) as char,
        (b'0' + j4 as u8) as char,
        (b'A' + j5 as u8) as char,
        (b'A' + j6 as u8) as char,
    ))
}

fn unpack_field_day(
    msg: &[u8; 77],
    hash_table: Option<&CallsignHashTable>,
    n3: u32,
) -> Option<String> {
    let decode_call =
        |packed| hash_table.map_or_else(|| unpack28(packed), |table| unpack28_h(packed, table));
    let call1 = decode_call(read_bits(msg, 0, 28));
    let call2 = decode_call(read_bits(msg, 28, 28));
    let ir = msg[56] & 1;
    let mut transmitters = read_bits(msg, 57, 4) + 1;
    if n3 == 4 {
        transmitters += 16;
    }
    let class_index = read_bits(msg, 61, 3);
    let section_index = read_bits(msg, 64, 7);
    let section = section_index
        .checked_sub(1)
        .and_then(|index| FIELD_DAY_SECTIONS.get(index as usize))?;
    let class = (b'A' + class_index as u8) as char;
    let exchange = format!("{transmitters}{class}");
    if ir == 1 {
        Some(format!("{call1} {call2} R {exchange} {section}"))
    } else {
        Some(format!("{call1} {call2} {exchange} {section}"))
    }
}

fn unpack_telemetry(msg: &[u8; 77]) -> String {
    let mut value = 0u128;
    for bit in &msg[..71] {
        value = (value << 1) | u128::from(*bit & 1);
    }
    let padded = format!("{value:018X}");
    padded.trim_start_matches('0').to_string()
}

fn unpack_type5(msg: &[u8; 77], hash_table: Option<&CallsignHashTable>) -> Option<String> {
    let n12 = read_bits(msg, 0, 12);
    let n22 = read_bits(msg, 12, 22);
    let ir = msg[34] & 1;
    let report = read_bits(msg, 35, 3) + 52;
    let serial = read_bits(msg, 38, 11);
    let grid = to_grid6(read_bits(msg, 49, 25))?;
    let call1 = hash_table
        .map(|table| resolve_hash12(n12, table))
        .unwrap_or_else(|| "<...>".to_string());
    let call2 = hash_table
        .and_then(|table| table.lookup22(n22))
        .unwrap_or_else(|| "<...>".to_string());
    let exchange = format!("{report:02}{serial:04}");
    if ir == 1 {
        Some(format!("{call1} {call2} R {exchange} {grid}"))
    } else {
        Some(format!("{call1} {call2} {exchange} {grid}"))
    }
}

/// Decode a 71-bit free-text message (13 chars from a 42-char alphabet).
fn unpack_free_text(msg: &[u8; 77]) -> String {
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

/// Decode the 50 significant bits of WSJT77 subtype 0.6 used by FST4W.
///
/// Pinned WSJT-X `packjt77.f90::pack77_06` stores these WSPR-style
/// messages in the first 50 bits and marks the containing word with
/// `(n3, i3) = (6, 0)`. This is not the classic WSPR 50-bit layout.
fn unpack_wspr50(msg: &[u8; 77], ht: Option<&CallsignHashTable>) -> Option<String> {
    const NZZZ: u32 = 46_656;

    let decode_power = |encoded: u32| -> Option<u32> {
        // Exact integer equivalent of `nint(encoded * 10.0 / 3.0)`.
        let power = (encoded * 10 + 1) / 3;
        (power <= 60).then_some(power)
    };
    let decode_call = |n28: u32| -> Option<String> {
        let call = if let Some(table) = ht {
            unpack28_h(n28, table)
        } else {
            unpack28(n28)
        };
        (!matches!(call.as_str(), "<?>" | "?????")).then_some(call)
    };

    // Bits 48..50 in Fortran numbering classify the payload:
    // Type 1 = x00, Type 2 = xx1, Type 3 = 010.
    if msg[49] == 1 {
        let call = decode_call(read_bits(msg, 0, 28))?;
        let mut npfx = read_bits(msg, 28, 16);
        let power = decode_power(read_bits(msg, 44, 5))?;

        if npfx < NZZZ {
            let mut prefix = [b' '; 3];
            for index in (0..3).rev() {
                prefix[index] = C2[(npfx % 36) as usize];
                npfx /= 36;
                if npfx == 0 {
                    break;
                }
            }
            let prefix = core::str::from_utf8(&prefix).ok()?.trim();
            return Some(format!("{prefix}/{call} {power}"));
        }

        npfx -= NZZZ;
        let suffix = if npfx <= 35 {
            String::from(C2[npfx as usize] as char)
        } else if npfx <= 1_295 {
            let chars = [C2[(npfx / 36) as usize], C2[(npfx % 36) as usize]];
            String::from_utf8(chars.to_vec()).ok()?
        } else if npfx <= 12_959 {
            let chars = [
                C2[(npfx / 360) as usize],
                C2[((npfx / 10) % 36) as usize],
                C2[(npfx % 10) as usize],
            ];
            String::from_utf8(chars.to_vec()).ok()?
        } else {
            return None;
        };
        return Some(format!("{call}/{suffix} {power}"));
    }

    if msg[48] == 0 {
        let call = decode_call(read_bits(msg, 0, 28))?;
        let grid = to_grid4(read_bits(msg, 28, 15))?;
        let power = decode_power(read_bits(msg, 43, 5))?;
        return Some(format!("{call} {grid} {power}"));
    }

    if msg[47] != 0 {
        return None;
    }

    let hash22 = read_bits(msg, 0, 22);
    let grid_index = read_bits(msg, 22, 25);
    let j1 = grid_index / (18 * 10 * 10 * 25 * 25);
    let rest = grid_index % (18 * 10 * 10 * 25 * 25);
    let j2 = rest / (10 * 10 * 25 * 25);
    let rest = rest % (10 * 10 * 25 * 25);
    let j3 = rest / (10 * 25 * 25);
    let rest = rest % (10 * 25 * 25);
    let j4 = rest / (25 * 25);
    let rest = rest % (25 * 25);
    let j5 = rest / 25;
    let j6 = rest % 25;
    if j1 > 17 || j2 > 17 || j3 > 9 || j4 > 9 || j5 > 24 || j6 > 24 {
        return None;
    }
    let mut grid = format!(
        "{}{}{}{}",
        (b'A' + j1 as u8) as char,
        (b'A' + j2 as u8) as char,
        (b'0' + j3 as u8) as char,
        (b'0' + j4 as u8) as char,
    );
    if j5 != 24 || j6 != 24 {
        grid.push((b'A' + j5 as u8) as char);
        grid.push((b'A' + j6 as u8) as char);
    }
    let call = ht
        .and_then(|table| table.lookup22(hash22))
        .unwrap_or_else(|| "<...>".to_string());
    Some(format!("{call} {grid}"))
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
pub fn unpack77(msg: &[u8; 77]) -> Option<String> {
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
            3 | 4 => unpack_field_day(msg, None, n3),
            5 => Some(unpack_telemetry(msg)),
            6 => unpack_wspr50(msg, None),
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
                        let sign = if isnr >= 0 { "+" } else { "" };
                        if ir == 1 {
                            format!("R{}{:02}", sign, isnr)
                        } else {
                            format!("{}{:02}", sign, isnr)
                        }
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

        5 => unpack_type5(msg, None),

        _ => None,
    }
}

/// Decode a 77-bit FT8 message, resolving hashed callsigns via a lookup table.
///
/// Behaves identically to [`unpack77`] but replaces `<...>` placeholders with
/// actual callsigns when they are found in the hash table.
pub fn unpack77_with_hash(msg: &[u8; 77], ht: &CallsignHashTable) -> Option<String> {
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
            3 | 4 => unpack_field_day(msg, Some(ht), n3),
            5 => Some(unpack_telemetry(msg)),
            6 => unpack_wspr50(msg, Some(ht)),
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
                        let sign = if isnr >= 0 { "+" } else { "" };
                        if ir == 1 {
                            format!("R{}{:02}", sign, isnr)
                        } else {
                            format!("{}{:02}", sign, isnr)
                        }
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

        5 => unpack_type5(msg, Some(ht)),

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

    let valid_call =
        |word: &str| (word.starts_with('<') && word.ends_with('>')) || is_plausible_callsign(word);

    // Complete ARRL Field Day exchange.
    if (words.len() == 4 || words.len() == 5)
        && valid_call(words[0])
        && valid_call(words[1])
        && FIELD_DAY_SECTIONS.contains(words.last().unwrap_or(&""))
    {
        let reply_offset = usize::from(words.len() == 5 && words[2] == "R");
        if words.len() == 4 + reply_offset {
            let exchange = words[2 + reply_offset].as_bytes();
            if (2..=3).contains(&exchange.len())
                && exchange[..exchange.len() - 1]
                    .iter()
                    .all(u8::is_ascii_digit)
                && (b'A'..=b'H').contains(&exchange[exchange.len() - 1])
            {
                return true;
            }
        }
    }

    // Complete ARRL RTTY Roundup exchange.
    let tu_offset = usize::from(words.first() == Some(&"TU;"));
    if words.len() >= tu_offset + 4
        && words.len() <= tu_offset + 5
        && valid_call(words[tu_offset])
        && valid_call(words[tu_offset + 1])
    {
        let reply = words.get(tu_offset + 2) == Some(&"R");
        let rst_index = tu_offset + 2 + usize::from(reply);
        if words.len() == rst_index + 2 {
            let rst = words[rst_index].as_bytes();
            let exchange = words[rst_index + 1];
            if rst.len() == 3
                && rst[0] == b'5'
                && (b'2'..=b'9').contains(&rst[1])
                && rst[2] == b'9'
                && (RTTY_STATES.contains(&exchange)
                    || (exchange.len() == 4 && exchange.bytes().all(|byte| byte.is_ascii_digit())))
            {
                return true;
            }
        }
    }

    // Complete EU VHF Type-5 exchange.
    if (words.len() == 4 || words.len() == 5)
        && words[0].starts_with('<')
        && words[1].starts_with('<')
    {
        let reply_offset = usize::from(words.len() == 5 && words[2] == "R");
        if words.len() == 4 + reply_offset {
            let exchange = words[2 + reply_offset];
            let grid = words[3 + reply_offset];
            if exchange.len() == 6
                && exchange.bytes().all(|byte| byte.is_ascii_digit())
                && pack_grid6(grid).is_some()
            {
                return true;
            }
        }
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
    let upper = call.trim().to_ascii_uppercase();
    let mapped = if upper.starts_with("3DA0") && upper.len() >= 5 {
        Some(format!("3D0{}", &upper[4..upper.len().min(7)]))
    } else if upper.starts_with("3X") && upper.as_bytes().get(2).is_some_and(u8::is_ascii_uppercase)
    {
        Some(format!("Q{}", &upper[2..upper.len().min(6)]))
    } else {
        None
    };
    let call = mapped.as_deref().unwrap_or(&upper);
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
                // WSJT-X `pack28` applies Fortran `adjustr(c4)` before
                // base-27 packing, so suffixes shorter than four
                // characters are right-aligned (for example
                // `CQ DX` becomes `"  DX"`).  Left alignment happens
                // to unpack to the same display text after trimming,
                // but produces a different on-air codeword and breaks
                // AP matching.
                let start = buf.len() - sb.len();
                for (i, &b) in sb.iter().enumerate() {
                    buf[start + i] = b;
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

/// Pack a six-character Maidenhead locator into the Type-5 25-bit field.
pub fn pack_grid6(grid: &str) -> Option<u32> {
    let g = grid.as_bytes();
    if g.len() != 6 {
        return None;
    }
    let j1 = g[0].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    let j2 = g[1].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    let j3 = g[2].wrapping_sub(b'0') as u32;
    let j4 = g[3].wrapping_sub(b'0') as u32;
    let j5 = g[4].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    let j6 = g[5].to_ascii_uppercase().wrapping_sub(b'A') as u32;
    if j1 > 17 || j2 > 17 || j3 > 9 || j4 > 9 || j5 > 23 || j6 > 23 {
        return None;
    }
    Some(
        j1 * 18 * 10 * 10 * 24 * 24
            + j2 * 10 * 10 * 24 * 24
            + j3 * 10 * 24 * 24
            + j4 * 24 * 24
            + j5 * 24
            + j6,
    )
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

fn bracketed_call(call: &str) -> Option<&str> {
    let inner = call.strip_prefix('<')?.strip_suffix('>')?;
    (!inner.is_empty() && inner != "...").then_some(inner)
}

fn pack28_message_call(call: &str) -> Option<u32> {
    if let Some(inner) = bracketed_call(call) {
        return Some(NTOKENS + super::hash_table::ihashcall(inner, 22));
    }
    pack28(call)
}

fn portable_base(call: &str) -> (&str, Option<u8>) {
    if let Some(base) = call.strip_suffix("/R") {
        (base, Some(b'R'))
    } else if let Some(base) = call.strip_suffix("/P") {
        (base, Some(b'P'))
    } else {
        (call, None)
    }
}

fn encode_standard_field(report: &str, reply_grid: bool) -> Option<(u32, u8)> {
    if report.is_empty() {
        return Some((MAX_GRID4 + 1, 0));
    }
    if let Some(grid) = pack_grid4(report) {
        return Some((grid, u8::from(reply_grid)));
    }
    if reply_grid {
        return None;
    }
    match report {
        "RRR" => return Some((MAX_GRID4 + 2, 0)),
        "RR73" => return Some((MAX_GRID4 + 3, 0)),
        "73" => return Some((MAX_GRID4 + 4, 0)),
        _ => {}
    }
    let (ir, numeric) = report
        .strip_prefix('R')
        .map_or((0, report), |numeric| (1, numeric));
    if !numeric.starts_with(['+', '-']) {
        return None;
    }
    let snr: i32 = numeric.parse().ok()?;
    if !(-50..=49).contains(&snr) {
        return None;
    }
    let adjusted = if snr <= -31 { snr + 101 } else { snr };
    Some((MAX_GRID4 + (adjusted + 35) as u32, ir))
}

fn pack77_standard_words(words: &[String]) -> Option<[u8; 77]> {
    if !(2..=4).contains(&words.len()) {
        return None;
    }
    let (call1, suffix1) = portable_base(&words[0]);
    let (call2, suffix2) = portable_base(&words[1]);
    if (bracketed_call(call1).is_some() && suffix2.is_some())
        || (bracketed_call(call2).is_some() && suffix1.is_some())
    {
        return None;
    }
    if words.len() == 2 && suffix2.is_some() {
        return None;
    }
    let n28a = pack28_message_call(call1)?;
    let n28b = pack28_message_call(call2)?;
    let (report, reply_grid) = match words.len() {
        2 => ("", false),
        3 => (words[2].as_str(), false),
        4 if words[2] == "R" => (words[3].as_str(), true),
        _ => return None,
    };
    let (igrid, ir) = encode_standard_field(report, reply_grid)?;
    if words[0].starts_with("CQ ") && (ir == 1 || igrid > MAX_GRID4 + 1) {
        return None;
    }
    let i3 = if suffix1 == Some(b'P') || suffix2 == Some(b'P') {
        2
    } else {
        1
    };
    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, n28a);
    msg[28] = u8::from(suffix1.is_some());
    write_bits(&mut msg, 29, 28, n28b);
    msg[57] = u8::from(suffix2.is_some());
    msg[58] = ir;
    write_bits(&mut msg, 59, 15, igrid);
    write_bits(&mut msg, 74, 3, i3);
    Some(msg)
}

/// Pack the WSJT-X Type 0.1 DXpedition exchange.
pub fn pack77_dxpedition(
    call1: &str,
    call2: &str,
    hashed_call: &str,
    report: i32,
) -> Option<[u8; 77]> {
    let inner = bracketed_call(hashed_call).unwrap_or(hashed_call);
    if inner.len() < 3 {
        return None;
    }
    let n28a = pack28(call1)?;
    let n28b = pack28(call2)?;
    let n5 = ((report + 30) / 2).clamp(0, 31) as u32;
    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, n28a);
    write_bits(&mut msg, 28, 28, n28b);
    write_bits(&mut msg, 56, 10, super::hash_table::ihashcall(inner, 10));
    write_bits(&mut msg, 66, 5, n5);
    write_bits(&mut msg, 71, 3, 1);
    Some(msg)
}

/// Pack an ARRL Field Day Type 0.3/0.4 exchange.
pub fn pack77_field_day(
    call1: &str,
    call2: &str,
    reply: bool,
    transmitters: u8,
    class: char,
    section: &str,
) -> Option<[u8; 77]> {
    if !(1..=32).contains(&transmitters) || !('A'..='H').contains(&class) {
        return None;
    }
    let section = section.trim().to_ascii_uppercase();
    let section_index = FIELD_DAY_SECTIONS
        .iter()
        .position(|candidate| *candidate == section)?
        + 1;
    let (n3, transmitter_index) = if transmitters <= 16 {
        (3, transmitters - 1)
    } else {
        (4, transmitters - 17)
    };
    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, pack28(call1)?);
    write_bits(&mut msg, 28, 28, pack28(call2)?);
    msg[56] = u8::from(reply);
    write_bits(&mut msg, 57, 4, u32::from(transmitter_index));
    write_bits(&mut msg, 61, 3, u32::from(class as u8 - b'A'));
    write_bits(&mut msg, 64, 7, section_index as u32);
    write_bits(&mut msg, 71, 3, n3);
    Some(msg)
}

/// Pack a WSJT-X Type 0.5 telemetry word (up to 71 significant bits).
pub fn pack77_telemetry(hex: &str) -> Option<[u8; 77]> {
    let hex = hex.trim();
    if hex.is_empty() || hex.len() > 18 || !hex.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return None;
    }
    let value = u128::from_str_radix(hex, 16).ok()?;
    if value >= (1u128 << 71) {
        return None;
    }
    let mut msg = [0u8; 77];
    for (index, bit) in msg[..71].iter_mut().enumerate() {
        *bit = ((value >> (70 - index)) & 1) as u8;
    }
    write_bits(&mut msg, 71, 3, 5);
    Some(msg)
}

/// Pack a WSJT-X Type 3 ARRL RTTY Roundup exchange.
pub fn pack77_rtty(
    call1: &str,
    call2: &str,
    reply: bool,
    rst: &str,
    exchange: &str,
    tu: bool,
) -> Option<[u8; 77]> {
    let rst_bytes = rst.as_bytes();
    if rst_bytes.len() != 3
        || rst_bytes[0] != b'5'
        || !(b'2'..=b'9').contains(&rst_bytes[1])
        || rst_bytes[2] != b'9'
    {
        return None;
    }
    let nexch = if let Some(index) = RTTY_STATES
        .iter()
        .position(|candidate| *candidate == exchange)
    {
        8_001 + index as u32
    } else {
        let serial: u32 = exchange.parse().ok()?;
        if !(1..=7_999).contains(&serial) {
            return None;
        }
        serial
    };
    let rst_value: i32 = rst.parse().ok()?;
    let irpt = ((rst_value - 509) / 10 - 2).clamp(0, 7) as u32;
    let mut msg = [0u8; 77];
    msg[0] = u8::from(tu);
    write_bits(&mut msg, 1, 28, pack28(call1)?);
    write_bits(&mut msg, 29, 28, pack28(call2)?);
    msg[57] = u8::from(reply);
    write_bits(&mut msg, 58, 3, irpt);
    write_bits(&mut msg, 61, 13, nexch);
    write_bits(&mut msg, 74, 3, 3);
    Some(msg)
}

/// Pack a WSJT-X Type 5 EU VHF exchange with two explicitly hashed calls.
pub fn pack77_eu_vhf(
    call1: &str,
    call2: &str,
    reply: bool,
    exchange: u32,
    grid: &str,
) -> Option<[u8; 77]> {
    let call1 = bracketed_call(call1)?;
    let call2 = bracketed_call(call2)?;
    if !(520_001..=594_095).contains(&exchange) {
        return None;
    }
    let report = exchange / 10_000 - 52;
    let serial = (exchange % 10_000).min(2_047);
    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 12, super::hash_table::ihashcall(call1, 12));
    write_bits(&mut msg, 12, 22, super::hash_table::ihashcall(call2, 22));
    msg[34] = u8::from(reply);
    write_bits(&mut msg, 35, 3, report);
    write_bits(&mut msg, 38, 11, serial);
    write_bits(&mut msg, 49, 25, pack_grid6(grid)?);
    write_bits(&mut msg, 74, 3, 5);
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
    pack77_type4_ordered(nonstd, std_call, report, is_cq, u8::from(!is_cq))
}

fn pack77_type4_ordered(
    nonstd: &str,
    hashed_call: &str,
    report: &str,
    is_cq: bool,
    iflip: u8,
) -> Option<[u8; 77]> {
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

    // WSJT-X also stores the CQ target's hash even though the decoder
    // ignores it while `icq=1`.
    let n12 = if is_cq {
        super::hash_table::ihashcall(&nonstd, 12)
    } else {
        super::hash_table::ihashcall(hashed_call, 12)
    };

    // Report encoding
    let nrpt: u32 = match report.trim() {
        "" => 0,
        "RRR" => 1,
        "RR73" => 2,
        "73" => 3,
        _ => return None,
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

fn normalize_message_words(text: &str) -> (String, Vec<String>) {
    let normalized = text
        .split_whitespace()
        .map(str::to_ascii_uppercase)
        .collect::<Vec<_>>()
        .join(" ");
    let mut words = normalized
        .split_whitespace()
        .map(ToString::to_string)
        .collect::<Vec<_>>();
    if words.len() >= 3
        && words[0] == "CQ"
        && words[1].len() <= 4
        && (is_valid_callsign(&words[2]) || bracketed_call(&words[2]).is_some())
    {
        words[0] = format!("CQ {}", words[1]);
        words.remove(1);
    }
    (normalized, words)
}

fn pack77_type4_words(words: &[String]) -> Option<[u8; 77]> {
    if !(2..=3).contains(&words.len()) {
        return None;
    }
    let report = words.get(2).map_or("", String::as_str);
    if words[0] == "CQ" {
        if words[1].len() <= 4 {
            return None;
        }
        return pack77_type4_ordered(&words[1], "", "", true, 0);
    }
    let first = bracketed_call(&words[0]);
    let second = bracketed_call(&words[1]);
    match (first, second) {
        (Some(hashed), None) => pack77_type4_ordered(&words[1], hashed, report, false, 0),
        (None, Some(hashed)) => pack77_type4_ordered(&words[0], hashed, report, false, 1),
        _ => None,
    }
}

fn pack77_dxpedition_words(words: &[String]) -> Option<[u8; 77]> {
    if words.len() != 5 || words[1] != "RR73;" || bracketed_call(&words[3]).is_none() {
        return None;
    }
    pack77_dxpedition(&words[0], &words[2], &words[3], words[4].parse().ok()?)
}

fn pack77_field_day_words(words: &[String]) -> Option<[u8; 77]> {
    if !(4..=5).contains(&words.len()) {
        return None;
    }
    let reply = words.len() == 5;
    if reply && words[2] != "R" {
        return None;
    }
    let exchange = &words[words.len() - 2];
    let class = exchange.chars().last()?;
    let transmitters = exchange[..exchange.len().checked_sub(1)?].parse().ok()?;
    pack77_field_day(
        &words[0],
        &words[1],
        reply,
        transmitters,
        class,
        &words[words.len() - 1],
    )
}

fn pack77_rtty_words(words: &[String]) -> Option<[u8; 77]> {
    if !(4..=6).contains(&words.len()) {
        return None;
    }
    let tu = words[0] == "TU;";
    let offset = usize::from(tu);
    if words.len() < offset + 4 {
        return None;
    }
    let reply = words.get(offset + 2).is_some_and(|word| word == "R");
    let rst_index = offset + 2 + usize::from(reply);
    if words.len() != rst_index + 2 {
        return None;
    }
    pack77_rtty(
        &words[offset],
        &words[offset + 1],
        reply,
        &words[rst_index],
        &words[rst_index + 1],
        tu,
    )
}

fn pack77_eu_vhf_words(words: &[String]) -> Option<[u8; 77]> {
    if !(4..=5).contains(&words.len()) {
        return None;
    }
    let reply = words.len() == 5;
    if reply && words[2] != "R" {
        return None;
    }
    let exchange_index = 2 + usize::from(reply);
    pack77_eu_vhf(
        &words[0],
        &words[1],
        reply,
        words[exchange_index].parse().ok()?,
        &words[exchange_index + 1],
    )
}

/// Pack a complete user-facing WSJT77 message using the same type-selection
/// order as pinned WSJT-X `packjt77.f90::pack77`.
///
/// Messages that do not match a structured form fall back to the first
/// 13 characters of the normalized text, matching WSJT-X free-text behavior.
pub fn pack77_message(text: &str) -> Option<[u8; 77]> {
    let (normalized, words) = normalize_message_words(text);
    if normalized.is_empty() {
        return None;
    }
    let starts_special = normalized.starts_with("CQ ")
        || normalized.starts_with("DE ")
        || normalized.starts_with("QRZ ");
    if !starts_special {
        if let Some(message) = pack77_dxpedition_words(&words) {
            return Some(message);
        }
        if let Some(message) = pack77_field_day_words(&words) {
            return Some(message);
        }
        if words.len() == 1
            && let Some(message) = pack77_telemetry(&words[0])
        {
            return Some(message);
        }
    }
    if let Some(message) = pack77_standard_words(&words) {
        return Some(message);
    }
    if let Some(message) = pack77_rtty_words(&words) {
        return Some(message);
    }
    if let Some(message) = pack77_type4_words(&words) {
        return Some(message);
    }
    if let Some(message) = pack77_eu_vhf_words(&words) {
        return Some(message);
    }
    let free_text = normalized.chars().take(13).collect::<String>();
    pack77_free_text(&free_text)
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

    fn bits_to_msg77(bits: &str) -> [u8; 77] {
        assert_eq!(bits.len(), 77);
        let mut message = [0u8; 77];
        for (target, source) in message.iter_mut().zip(bits.bytes()) {
            *target = match source {
                b'0' => 0,
                b'1' => 1,
                _ => panic!("invalid bit"),
            };
        }
        message
    }

    #[test]
    fn pinned_wsjtx_structured_message_vectors() {
        // Generated by tools/wsjtx_wsjt77_oracle.f90 against pinned WSJT-X
        // v3.0.2 commit ccdfaf3c1c109010d15399674ce278167cfde848.
        let vectors = [
            (
                "K1ABC RR73; W9XYZ <KH1/KH7Z> -11",
                "K1ABC RR73; W9XYZ <...> -12",
                "00001001101111011110001101010000110000101001001110111000001100100101001001000",
            ),
            (
                "WA9XYZ KA1ABC R 16A EMA",
                "WA9XYZ KA1ABC R 16A EMA",
                "11100111000010000110110111001001010111000110010100100001111110000001011011000",
            ),
            (
                "WA9XYZ KA1ABC 32A EMA",
                "WA9XYZ KA1ABC 32A EMA",
                "11100111000010000110110111001001010111000110010100100001011110000001011100000",
            ),
            (
                "PA3XYZ/P GM4ABC/P R JO22",
                "PA3XYZ/P GM4ABC/P R JO22",
                "10110111100111011111000000101011111010000110110010101001011100010011010110010",
            ),
            (
                "TU; W9XYZ K1ABC R 579 MA",
                "TU; W9XYZ K1ABC R 579 MA",
                "10000110000101001001110111000000010011011110111100011010111011111101010101011",
            ),
            (
                "CQ PJ4/KA1ABC",
                "CQ PJ4/KA1ABC",
                "00011100000100000011111001001010001101001010100001101110111010111000010001100",
            ),
            (
                "<WA9XYZ> PJ4/KA1ABC RR73",
                "<...> PJ4/KA1ABC RR73",
                "11000001100000000011111001001010001101001010100001101110111010111000010100100",
            ),
            (
                "PJ4/KA1ABC <WA9XYZ> 73",
                "PJ4/KA1ABC <...> 73",
                "11000001100000000011111001001010001101001010100001101110111010111000011110100",
            ),
            (
                "<PA3XYZ> <G4ABC/P> R 590003 IO91NP",
                "<...> <...> R 590003 IO91NP",
                "11001000101111001000101111100100111111000000000110100010111010110000000111101",
            ),
            (
                "K1ABC W9XYZ",
                "K1ABC W9XYZ",
                "00001001101111011110001101010000011000010100100111011100000111111010010001001",
            ),
            (
                "0123456789ABCDEF01",
                "123456789ABCDEF01",
                "00000010010001101000101011001111000100110101011110011011110111100000001101000",
            ),
        ];

        for (input, expected_decode, bits) in vectors {
            let expected = bits_to_msg77(bits);
            let actual = pack77_message(input)
                .unwrap_or_else(|| panic!("failed to pack structured message: {input}"));
            assert_eq!(actual, expected, "packed bits differ for {input}");
            assert_eq!(
                unpack77(&actual).as_deref(),
                Some(expected_decode),
                "canonical decode differs for {input}"
            );
        }
    }

    #[test]
    fn type5_hash_resolution_matches_upstream_collision_semantics() {
        let message = pack77_message("<PA3XYZ> <G4ABC/P> R 590003 IO91NP").expect("pack Type 5");
        let mut hashes = CallsignHashTable::new();
        hashes.insert("PA3XYZ");
        hashes.insert("G4ABC/P");
        // These calls collide at 12 bits. WSJT-X's direct-indexed calls12
        // table is last-writer-wins, so its pinned oracle emits this text.
        assert_eq!(
            unpack77_with_hash(&message, &hashes).as_deref(),
            Some("<G4ABC/P> <G4ABC/P> R 590003 IO91NP")
        );
    }

    #[test]
    fn telemetry_uses_the_wsjt_x_type_05_layout() {
        let message = pack77_telemetry("0123456789ABCDEF01").expect("pack telemetry");
        assert_eq!(read_bits(&message, 71, 3), 5);
        assert_eq!(read_bits(&message, 74, 3), 0);
        assert_eq!(unpack77(&message).as_deref(), Some("123456789ABCDEF01"));
        assert_eq!(
            pack77_message("0123456789ABCDEF01"),
            Some(message),
            "single-word hexadecimal input must select Type 0.5"
        );
        assert!(pack77_telemetry("8123456789ABCDEF01").is_none());
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
        // A roundtrip alone cannot detect left- vs right-alignment,
        // because `unpack28` trims both forms to the same text.
        assert_eq!(pack28("CQ DX"), Some(1135));

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
