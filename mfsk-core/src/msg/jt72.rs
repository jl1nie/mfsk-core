//! JT 72-bit message codec, shared by JT65 and JT9.
//!
//! Ported from WSJT-X `lib/packjt.f90` — in particular
//! `packmsg` / `unpackmsg`, `packcall` / `unpackcall`,
//! and `packgrid` / `unpackgrid`. The 72-bit payload layout is
//! identical between JT65 and JT9; it packs as
//!
//! ```text
//! |---- nc1 (28) ---|--- nc2 (28) ---|-- ng (16) --|
//! ```
//!
//! where `nc1` / `nc2` are the two callsigns (base-37 / base-36 /
//! base-10 / base-27^3 packed) and `ng` is the 4-character
//! Maidenhead grid or an encoded report code.
//!
//! The bytes then get laid out as 12 × 6-bit symbols. That shape
//! matches what JT65's Reed-Solomon and JT9's convolutional encoder
//! ingest. This module does **not** speak symbols directly — callers
//! are expected to unpack the 72-bit byte stream into whatever FEC
//! wants.
//!
//! Standard messages, compound-callsign Types 2–5, and the pinned
//! WSJT-X 13-character free-text format are implemented.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

/// Base used to pack a 6-character callsign into a 28-bit integer.
/// Matches `NBASE` in WSJT-X: `37 * 36 * 10 * 27 * 27 * 27 = 262 177 560`.
const NBASE: u32 = 37 * 36 * 10 * 27 * 27 * 27;

/// First value after the original call / CQ-frequency namespace.
const NBASE2: u32 = NBASE + 1_002;

/// Base used for 4-character Maidenhead grids: `180 * 180 = 32 400`.
/// Values above this encode report codes (see `unpack_grid`).
const NGBASE: u32 = 180 * 180;

/// Decoded JT 72-bit message payload.
///
/// The enum shape mirrors the `itype` classification in WSJT-X
/// `packmsg` (Type 1 = standard, Types 2–5 = compound-callsign
/// variants, Type 6 = free text). Compound calls are returned in the
/// same `Standard` fields as ordinary calls because callers should not
/// need to know which wire representation produced the callsign.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Jt72Message {
    /// Standard two-callsign + grid / report message.
    Standard {
        call1: String,
        call2: String,
        /// Human-readable representation of the `ng` field: either a
        /// 4-char grid ("FN42"), a report ("-15", "R-05"), or one of
        /// the short tokens ("RO", "RRR", "73").
        grid_or_report: String,
    },
    /// Pinned WSJT-X Type 6 base-42 free text (up to 13 characters).
    FreeText { text: String },
    /// A malformed or reserved payload that cannot be rendered as a
    /// valid WSJT-X message. Raw integer fields are retained for
    /// diagnostics.
    Unsupported { nc1: u32, nc2: u32, ng: u32 },
}

impl fmt::Display for Jt72Message {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                write!(f, "{} {}", call1, call2)?;
                if !grid_or_report.is_empty() {
                    write!(f, " {grid_or_report}")?;
                }
                Ok(())
            }
            Jt72Message::FreeText { text } => f.write_str(text),
            Jt72Message::Unsupported { nc1, nc2, ng } => {
                write!(f, "<unsupported nc1={nc1} nc2={nc2} ng={ng}>")
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Character helpers (WSJT-X `nchar` / `unpackcall` tables)
// ─────────────────────────────────────────────────────────────────────────

/// 37-char callsign alphabet: digits, uppercase letters, space.
const CALL_ALPHA: &[u8; 37] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";
const TEXT_ALPHA: &[u8; 42] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?";
const LEGACY_SUFFIXES: &[u8; 12] = b"P0123456789A";
const LEGACY_PREFIXES: &str = "1A 1S 3A 3B6 3B8 3B9 3C 3C0 3D2 3D2C 3D2R 3DA 3V 3W 3X 3Y 3YB 3YP 4J 4L 4S 4U1I 4U1U 4W 4X 5A 5B 5H 5N 5R 5T 5U 5V 5W 5X 5Z 6W 6Y 7O 7P 7Q 7X 8P 8Q 8R 9A 9G 9H 9J 9K 9L 9M2 9M6 9N 9Q 9U 9V 9X 9Y A2 A3 A4 A5 A6 A7 A9 AP BS7 BV BV9 BY C2 C3 C5 C6 C9 CE CE0X CE0Y CE0Z CE9 CM CN CP CT CT3 CU CX CY0 CY9 D2 D4 D6 DL DU E3 E4 EA EA6 EA8 EA9 EI EK EL EP ER ES ET EU EX EY EZ F FG FH FJ FK FKC FM FO FOA FOC FOM FP FR FRG FRJ FRT FT5W FT5X FT5Z FW FY M MD MI MJ MM MU MW H4 H40 HA HB HB0 HC HC8 HH HI HK HK0 HK0M HL HM HP HR HS HV HZ I IS IS0 J2 J3 J5 J6 J7 J8 JA JDM JDO JT JW JX JY K KG4 KH0 KH1 KH2 KH3 KH4 KH5 KH5K KH6 KH7 KH8 KH9 KL KP1 KP2 KP4 KP5 LA LU LX LY LZ OA OD OE OH OH0 OJ0 OK OM ON OX OY OZ P2 P4 PA PJ2 PJ7 PY PY0F PT0S PY0T PZ R1F R1M S0 S2 S5 S7 S9 SM SP ST SU SV SVA SV5 SV9 T2 T30 T31 T32 T33 T5 T7 T8 T9 TA TF TG TI TI9 TJ TK TL TN TR TT TU TY TZ UA UA2 UA9 UK UN UR V2 V3 V4 V5 V6 V7 V8 VE VK VK0H VK0M VK9C VK9L VK9M VK9N VK9W VK9X VP2E VP2M VP2V VP5 VP6 VP6D VP8 VP8G VP8H VP8O VP8S VP9 VQ9 VR VU VU4 VU7 XE XF4 XT XU XW XX9 XZ YA YB YI YJ YK YL YN YO YS YU YV YV0 Z2 Z3 ZA ZB ZC4 ZD7 ZD8 ZD9 ZF ZK1N ZK1S ZK2 ZK3 ZL ZL7 ZL8 ZL9 ZP ZS ZS8 KC4 E5";

#[derive(Clone, Debug)]
struct CompoundCall {
    base: String,
    /// WSJT-X `nv2`: 2/3 legacy prefix/suffix, 4/5 generic prefix/suffix.
    kind: u8,
    k: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum GenericLeader {
    Cq,
    Qrz,
    De,
}

impl GenericLeader {
    fn text(self) -> &'static str {
        match self {
            Self::Cq => "CQ",
            Self::Qrz => "QRZ",
            Self::De => "DE",
        }
    }
}

fn parse_compound_call(call: &str) -> Option<CompoundCall> {
    let call = call.trim().to_ascii_uppercase();
    let (left, right) = call.split_once('/')?;
    if left.is_empty() || right.is_empty() || right.contains('/') {
        return None;
    }
    if let Some(index) = LEGACY_PREFIXES
        .split_ascii_whitespace()
        .position(|prefix| prefix == left)
    {
        return Some(CompoundCall {
            base: right.to_owned(),
            kind: 2,
            k: index as u32 + 1,
        });
    }
    if right.len() == 1
        && let Some(index) = LEGACY_SUFFIXES
            .iter()
            .position(|suffix| *suffix == right.as_bytes()[0])
    {
        return Some(CompoundCall {
            base: left.to_owned(),
            kind: 3,
            k: 401 + index as u32,
        });
    }

    let prefix_shape = (1..=4).contains(&left.len());
    let suffix_shape = (1..=3).contains(&right.len());
    let is_prefix = match (prefix_shape, suffix_shape) {
        (false, false) => return None,
        (true, false) => true,
        (false, true) => false,
        (true, true) if left.len() < 3 && right.len() < 3 => return None,
        (true, true) if left.len() < 3 => true,
        (true, true) if right.len() < 3 => false,
        (true, true) => left.as_bytes().last().is_some_and(u8::is_ascii_digit),
    };
    let encode_base37 = |text: &str, width: usize| {
        let mut value = 0u32;
        for index in 0..width {
            value = 37 * value + nchar(text.as_bytes().get(index).copied().unwrap_or(b' '))?;
        }
        Some(value)
    };
    if is_prefix {
        Some(CompoundCall {
            base: right.to_owned(),
            kind: 4,
            k: encode_base37(left, 4)?,
        })
    } else {
        Some(CompoundCall {
            base: left.to_owned(),
            kind: 5,
            k: encode_base37(right, 3)?,
        })
    }
}

fn apply_legacy_affix(call: &str, mut k: u32) -> Option<String> {
    if k > 450 {
        k -= 450;
    }
    if (1..=339).contains(&k) {
        let prefix = LEGACY_PREFIXES
            .split_ascii_whitespace()
            .nth(k as usize - 1)?;
        return Some(format!("{prefix}/{call}"));
    }
    if (401..=412).contains(&k) {
        return Some(format!(
            "{call}/{}",
            LEGACY_SUFFIXES[(k - 401) as usize] as char
        ));
    }
    None
}

fn unpack_base37(mut value: u32, width: usize) -> Option<String> {
    let mut characters = [b' '; 4];
    if width == 0 || width > characters.len() {
        return None;
    }
    for index in (0..width).rev() {
        characters[index] = CALL_ALPHA[(value % 37) as usize];
        value /= 37;
    }
    if value != 0 {
        return None;
    }
    Some(
        core::str::from_utf8(&characters[..width])
            .ok()?
            .trim_end()
            .to_owned(),
    )
}

/// Decode the six JT65v2 ranges embedded in `nc1` by WSJT-X
/// `unpackcall`: CQ/QRZ/DE, each with either a generic prefix or
/// suffix on the second callsign.
fn unpack_generic_nc1(nc1: u32) -> Option<(GenericLeader, bool, String)> {
    const PREFIX_SPAN: u32 = 1_823_509;
    const SUFFIX_SPAN: u32 = 49_285;
    let (leader, is_prefix, value) = if (NBASE2 + 1..=NBASE2 + PREFIX_SPAN).contains(&nc1) {
        (GenericLeader::Cq, true, nc1 - (NBASE2 + 1))
    } else if (NBASE2 + PREFIX_SPAN + 1..=NBASE2 + 2 * PREFIX_SPAN).contains(&nc1) {
        (GenericLeader::Qrz, true, nc1 - (NBASE2 + PREFIX_SPAN + 1))
    } else if (NBASE2 + 2 * PREFIX_SPAN + 1..=NBASE2 + 3 * PREFIX_SPAN).contains(&nc1) {
        (
            GenericLeader::De,
            true,
            nc1 - (NBASE2 + 2 * PREFIX_SPAN + 1),
        )
    } else if (NBASE2 + 3 * PREFIX_SPAN + 1..=NBASE2 + 3 * PREFIX_SPAN + SUFFIX_SPAN).contains(&nc1)
    {
        (
            GenericLeader::Cq,
            false,
            nc1 - (NBASE2 + 3 * PREFIX_SPAN + 1),
        )
    } else if (NBASE2 + 3 * PREFIX_SPAN + SUFFIX_SPAN + 1
        ..=NBASE2 + 3 * PREFIX_SPAN + 2 * SUFFIX_SPAN)
        .contains(&nc1)
    {
        (
            GenericLeader::Qrz,
            false,
            nc1 - (NBASE2 + 3 * PREFIX_SPAN + SUFFIX_SPAN + 1),
        )
    } else if (NBASE2 + 3 * PREFIX_SPAN + 2 * SUFFIX_SPAN + 1
        ..=NBASE2 + 3 * PREFIX_SPAN + 3 * SUFFIX_SPAN)
        .contains(&nc1)
    {
        (
            GenericLeader::De,
            false,
            nc1 - (NBASE2 + 3 * PREFIX_SPAN + 2 * SUFFIX_SPAN + 1),
        )
    } else {
        return None;
    };
    Some((
        leader,
        is_prefix,
        unpack_base37(value, if is_prefix { 4 } else { 3 })?,
    ))
}

fn k_to_grid(k: u32) -> Option<String> {
    if !(1..=900).contains(&k) {
        return None;
    }
    let mut longitude = 2 * (((k - 1) / 5) % 90) as i32 - 179;
    if k > 450 {
        longitude += 180;
    }
    let latitude = ((k - 1) % 5) as i32 + 85;
    let longitude_units = 12 * (180 - longitude);
    let latitude_units = 24 * (latitude + 90);
    let field_longitude = longitude_units / 240;
    let square_longitude = (longitude_units - 240 * field_longitude) / 24;
    let field_latitude = latitude_units / 240;
    let square_latitude = (latitude_units - 240 * field_latitude) / 24;
    Some(format!(
        "{}{}{}{}",
        (b'A' + field_longitude as u8) as char,
        (b'A' + field_latitude as u8) as char,
        square_longitude,
        square_latitude,
    ))
}

fn grid_to_k(grid: &str) -> u32 {
    let bytes = grid.as_bytes();
    if bytes.len() != 4
        || !matches!(bytes[0], b'A'..=b'R')
        || !matches!(bytes[1], b'A'..=b'R')
        || !bytes[2].is_ascii_digit()
        || !bytes[3].is_ascii_digit()
    {
        return 0;
    }
    let longitude = 179 - 20 * i32::from(bytes[0] - b'A') - 2 * i32::from(bytes[2] - b'0');
    // `unpackmsg` probes the four-character grid at subsquare `ma`:
    // longitude is centered by `m`, while latitude remains just above
    // the square's southern edge because `a` is used there.
    let latitude = -90 + 10 * i32::from(bytes[1] - b'A') + i32::from(bytes[3] - b'0');
    if latitude < 85 {
        return 0;
    }
    (5 * (longitude + 179) / 2 + latitude - 84) as u32
}

/// Translate a callsign char to its `nchar` index: digit→0..9,
/// letter→10..35, space→36. Returns `None` for anything else.
fn nchar(c: u8) -> Option<u32> {
    match c {
        b'0'..=b'9' => Some((c - b'0') as u32),
        b'A'..=b'Z' => Some((c - b'A' + 10) as u32),
        b'a'..=b'z' => Some((c - b'a' + 10) as u32),
        b' ' => Some(36),
        _ => None,
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Callsign (28-bit `nc`)
// ─────────────────────────────────────────────────────────────────────────

/// Pack a ≤ 6-character callsign into a 28-bit integer. The standard
/// layout expects the digit in position 3 (`K1ABC`) or position 2
/// (`K9AN`); the latter gets a leading space inserted so the digit
/// lands at index 2.
///
/// Returns `None` if the callsign doesn't fit the base-37/36/10/27³
/// schema — those cases trigger the "text / compound" fallbacks in
/// `packcall` that this MVP doesn't yet model.
pub fn pack_call(call: &str) -> Option<u32> {
    let mut normalized = call.trim().to_ascii_uppercase();
    // Historical mappings retained verbatim by WSJT-X `packcall`.
    if normalized.starts_with("3DA0") {
        normalized = format!("3D0{}", &normalized[4..]);
    } else if normalized.starts_with("3X")
        && normalized
            .as_bytes()
            .get(2)
            .is_some_and(u8::is_ascii_uppercase)
    {
        normalized = format!("Q{}", &normalized[2..]);
    }
    let call = normalized.as_str();
    let bytes = call.as_bytes();
    // Special tokens handled by WSJT-X's `packcall`.
    match call {
        "CQ" => return Some(NBASE + 1),
        "QRZ" => return Some(NBASE + 2),
        "DE" => return Some(267_796_945),
        _ => {}
    }
    if let Some(frequency) = call.strip_prefix("CQ ")
        && frequency.len() == 3
        && frequency.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Some(NBASE + 3 + frequency.parse::<u32>().ok()?);
    }
    if bytes.is_empty() || bytes.len() > 6 {
        return None;
    }

    // Build the 6-char right-aligned working copy `tmp`.
    let mut tmp = [b' '; 6];
    if bytes.len() >= 3 && bytes[2].is_ascii_digit() {
        // Digit at position 3 (0-indexed 2) — left-aligned as-is.
        for (i, &b) in bytes.iter().enumerate() {
            tmp[i] = b;
        }
    } else if bytes.len() >= 2 && bytes[1].is_ascii_digit() {
        // Digit at position 2 — shift right by one so digit lands at
        // tmp[2]. Max source length becomes 5.
        if bytes.len() > 5 {
            return None;
        }
        for (i, &b) in bytes.iter().enumerate() {
            tmp[i + 1] = b;
        }
    } else {
        return None;
    }

    // Uppercase.
    for t in tmp.iter_mut() {
        if t.is_ascii_lowercase() {
            *t -= b'a' - b'A';
        }
    }

    // Validate slot alphabets.
    let n = [
        nchar(tmp[0])?,
        nchar(tmp[1])?,
        nchar(tmp[2])?,
        nchar(tmp[3])?,
        nchar(tmp[4])?,
        nchar(tmp[5])?,
    ];
    // Slot 0: letter/digit/space (0..=36)
    // Slot 1: letter/digit (0..=35)
    if n[1] == 36 {
        return None;
    }
    // Slot 2: digit (0..=9)
    if n[2] >= 10 {
        return None;
    }
    // Slots 3..=5: letter/space (10..=36)
    for k in 3..6 {
        if n[k] < 10 {
            return None;
        }
    }

    let mut ncall = n[0];
    ncall = 36 * ncall + n[1];
    ncall = 10 * ncall + n[2];
    ncall = 27 * ncall + n[3] - 10;
    ncall = 27 * ncall + n[4] - 10;
    ncall = 27 * ncall + n[5] - 10;
    Some(ncall)
}

/// Unpack a 28-bit integer back into a callsign or special token.
/// Returns `None` for values outside the base-37/36/10/27³ range
/// (those encode compound-callsign variants).
pub fn unpack_call(ncall: u32) -> Option<String> {
    // Special tokens.
    match ncall {
        v if v == NBASE + 1 => return Some("CQ".into()),
        v if v == NBASE + 2 => return Some("QRZ".into()),
        267_796_945 => return Some("DE".into()),
        _ => {}
    }
    if (NBASE + 3..=NBASE + 1_002).contains(&ncall) {
        return Some(format!("CQ {:03}", ncall - NBASE - 3));
    }
    if ncall >= NBASE {
        return None;
    }
    let mut n = ncall;
    let mut chars = [b' '; 6];
    let c6 = (n % 27) + 10;
    chars[5] = CALL_ALPHA[c6 as usize];
    n /= 27;
    let c5 = (n % 27) + 10;
    chars[4] = CALL_ALPHA[c5 as usize];
    n /= 27;
    let c4 = (n % 27) + 10;
    chars[3] = CALL_ALPHA[c4 as usize];
    n /= 27;
    let c3 = n % 10;
    chars[2] = CALL_ALPHA[c3 as usize];
    n /= 10;
    let c2 = n % 36;
    chars[1] = CALL_ALPHA[c2 as usize];
    n /= 36;
    let c1 = n; // 0..=36
    chars[0] = CALL_ALPHA[c1 as usize];

    let s = core::str::from_utf8(&chars).ok()?.trim();
    if let Some(rest) = s.strip_prefix("3D0") {
        Some(format!("3DA0{rest}"))
    } else if let Some(rest) = s.strip_prefix('Q')
        && rest.as_bytes().first().is_some_and(u8::is_ascii_uppercase)
    {
        Some(format!("3X{rest}"))
    } else {
        Some(s.to_string())
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Grid / report (16-bit `ng`)
// ─────────────────────────────────────────────────────────────────────────

/// Pack a 4-character grid locator into `ng` via the Maidenhead →
/// integer mapping used by WSJT-X `packgrid` (without the
/// extended-range report tricks — callers can build those up
/// manually).
///
/// WSJT-X reference (`lib/packjt.f90:341-345` + `grid2deg.f90`):
/// `dlong = (180 - 20*fl - 2*sl) - 62.5/60`, `lat = 10*fla + sla`,
/// `ng = ((int(dlong) + 180) / 2) * 180 + lat`. For valid grids
/// (fl ∈ 0..=17, sl ∈ 0..=9) the integer-arithmetic equivalent
/// collapses to `ng_long_part = 179 - 10*fl - sl`.
fn pack_grid4_plain(grid: &str) -> Option<u32> {
    let b = grid.as_bytes();
    if b.len() != 4 {
        return None;
    }
    let fl = match b[0] {
        c @ b'A'..=b'R' => (c - b'A') as i32,
        _ => return None,
    };
    let fla = match b[1] {
        c @ b'A'..=b'R' => (c - b'A') as i32,
        _ => return None,
    };
    let sl = match b[2] {
        c @ b'0'..=b'9' => (c - b'0') as i32,
        _ => return None,
    };
    let sla = match b[3] {
        c @ b'0'..=b'9' => (c - b'0') as i32,
        _ => return None,
    };
    let ng_long_part = 179 - 10 * fl - sl;
    let lat_int = 10 * fla + sla;
    let ng = ng_long_part * 180 + lat_int;
    Some(ng as u32)
}

/// Pack a 4-char grid OR a report/token into `ng`. Supported short
/// forms: "RO", "RRR", "73", "-NN" (01..30), "R-NN" (01..30), empty
/// (= "   ").
pub fn pack_grid_or_report(s: &str) -> Option<u32> {
    match s.trim_end() {
        "" => Some(NGBASE + 1),
        "RO" => Some(NGBASE + 62),
        "RRR" => Some(NGBASE + 63),
        "73" => Some(NGBASE + 64),
        other => {
            if let Some(rest) = other.strip_prefix('-')
                && let Ok(n) = rest.parse::<i32>()
                && (1..=30).contains(&n)
            {
                return Some(NGBASE + 1 + n as u32);
            }
            if let Some(rest) = other.strip_prefix("R-")
                && let Ok(n) = rest.parse::<i32>()
                && (1..=30).contains(&n)
            {
                return Some(NGBASE + 31 + n as u32);
            }
            let (is_acknowledged, numeric) = if let Some(report) = other.strip_prefix('R') {
                (true, report)
            } else {
                (false, other)
            };
            if let Ok(report) = numeric.parse::<i32>()
                && (-50..=49).contains(&report)
            {
                let prefix = if is_acknowledged { 'L' } else { 'K' };
                return pack_grid4_plain(&format!("{prefix}A{:02}", report + 50));
            }
            pack_grid4_plain(other)
        }
    }
}

/// Inverse of `pack_grid_or_report`. Unknown codes (extended-range
/// reports, free-text `ng + 32768`) decode as "?".
pub fn unpack_grid(ng: u32) -> String {
    if ng == NGBASE + 1 {
        return String::new();
    }
    match ng {
        v if v == NGBASE + 62 => return "RO".into(),
        v if v == NGBASE + 63 => return "RRR".into(),
        v if v == NGBASE + 64 => return "73".into(),
        _ => {}
    }
    if ng > NGBASE && ng <= NGBASE + 30 + 1 {
        let n = ng - NGBASE - 1;
        return format!("-{:02}", n);
    }
    if ng > NGBASE + 31 && ng <= NGBASE + 61 {
        let n = ng - NGBASE - 31;
        return format!("R-{:02}", n);
    }
    if ng < NGBASE {
        // Standard grid. Inverse of pack_grid4_plain:
        //   long_part = ng / 180 = 179 - 10*fl - sl
        //   lat       = ng % 180 = 10*fla + sla
        let long_part = (ng / 180) as i32;
        let lat = (ng % 180) as i32;
        let fl_sl = 179 - long_part;
        let fl = fl_sl / 10;
        let sl = fl_sl % 10;
        let fla = lat / 10;
        let sla = lat % 10;
        let mut g = [0u8; 4];
        g[0] = b'A' + fl as u8;
        g[1] = b'A' + fla as u8;
        g[2] = b'0' + sl as u8;
        g[3] = b'0' + sla as u8;
        let grid = core::str::from_utf8(&g).unwrap_or("????");
        if matches!(&grid[..2], "KA" | "LA") {
            let report = grid[2..].parse::<i32>().unwrap_or(50) - 50;
            let prefix = if grid.starts_with('L') { "R" } else { "" };
            return if report >= 0 {
                format!("{prefix}+{report:02}")
            } else {
                format!("{prefix}-{abs:02}", abs = report.unsigned_abs())
            };
        }
        return grid.to_string();
    }
    "?".into()
}

// ─────────────────────────────────────────────────────────────────────────
// 72-bit pack / unpack
// ─────────────────────────────────────────────────────────────────────────

/// Pack (nc1, nc2, ng) into 12 × 6-bit symbols (`[u8; 12]`, values
/// 0..=63). Matches the dat(1..12) layout in WSJT-X `packmsg` lines
/// 521–532.
pub fn pack_words(nc1: u32, nc2: u32, ng: u32) -> [u8; 12] {
    let mut d = [0u8; 12];
    d[0] = ((nc1 >> 22) & 0x3f) as u8;
    d[1] = ((nc1 >> 16) & 0x3f) as u8;
    d[2] = ((nc1 >> 10) & 0x3f) as u8;
    d[3] = ((nc1 >> 4) & 0x3f) as u8;
    d[4] = (((nc1 & 0xf) << 2) | ((nc2 >> 26) & 0x3)) as u8;
    d[5] = ((nc2 >> 20) & 0x3f) as u8;
    d[6] = ((nc2 >> 14) & 0x3f) as u8;
    d[7] = ((nc2 >> 8) & 0x3f) as u8;
    d[8] = ((nc2 >> 2) & 0x3f) as u8;
    d[9] = (((nc2 & 0x3) << 4) | ((ng >> 12) & 0xf)) as u8;
    d[10] = ((ng >> 6) & 0x3f) as u8;
    d[11] = (ng & 0x3f) as u8;
    d
}

/// Inverse of [`pack_words`]. Returns the packed-field tuple
/// `(nc1, nc2, ng)` — widths 28 / 28 / 16 bits.
pub fn unpack_words(d: &[u8; 12]) -> (u32, u32, u32) {
    let nc1 = ((d[0] as u32) << 22)
        | ((d[1] as u32) << 16)
        | ((d[2] as u32) << 10)
        | ((d[3] as u32) << 4)
        | (((d[4] as u32) >> 2) & 0xf);
    let nc2 = (((d[4] as u32) & 0x3) << 26)
        | ((d[5] as u32) << 20)
        | ((d[6] as u32) << 14)
        | ((d[7] as u32) << 8)
        | ((d[8] as u32) << 2)
        | (((d[9] as u32) >> 4) & 0x3);
    let ng = (((d[9] as u32) & 0xf) << 12) | ((d[10] as u32) << 6) | (d[11] as u32);
    (nc1, nc2, ng)
}

/// Convenience: pack a standard message (call1, call2, grid_or_report)
/// into 12 six-bit words.
pub fn pack_standard(call1: &str, call2: &str, grid_or_report: &str) -> Option<[u8; 12]> {
    const PREFIX_SPAN: u32 = 1_823_509;
    const SUFFIX_SPAN: u32 = 49_285;

    let call1 = call1.trim().to_ascii_uppercase();
    let call2 = call2.trim().to_ascii_uppercase();
    let compound1 = parse_compound_call(&call1);
    let compound2 = parse_compound_call(&call2);
    let base1 = compound1
        .as_ref()
        .map_or(call1.as_str(), |compound| compound.base.as_str());
    let base2 = compound2
        .as_ref()
        .map_or(call2.as_str(), |compound| compound.base.as_str());

    // Pinned `packmsg` never permits a generic Type 4/5 affix in the
    // first call field. It falls back to Type 6 text instead.
    if compound1
        .as_ref()
        .is_some_and(|compound| compound.kind >= 4)
    {
        return None;
    }

    let ordinary_nc1 = pack_call(base1)?;
    let nc2 = pack_call(base2)?;
    let ng = pack_grid_or_report(grid_or_report)?;

    let legacy1 = compound1
        .as_ref()
        .filter(|compound| matches!(compound.kind, 2 | 3));
    let legacy2 = compound2
        .as_ref()
        .filter(|compound| matches!(compound.kind, 2 | 3));
    if legacy1.is_some() || legacy2.is_some() {
        // WSJT-X has only one legacy affix channel: k=1..450 is
        // carried in the first call, k=451..900 in the second. The
        // synthetic polar grid replaces any supplied grid/report.
        if legacy1.is_some() && legacy2.is_some() {
            return None;
        }
        if compound2
            .as_ref()
            .is_some_and(|compound| compound.kind >= 4)
        {
            return None;
        }
        let k = legacy1
            .map(|compound| compound.k)
            .or_else(|| legacy2.map(|compound| compound.k + 450))?;
        let synthetic_grid = k_to_grid(k)?;
        let synthetic_ng = pack_grid_or_report(&synthetic_grid)?;
        return Some(pack_words(ordinary_nc1, nc2, synthetic_ng));
    }

    if let Some(compound) = compound2.as_ref() {
        let leader = match call1.as_str() {
            "CQ" => GenericLeader::Cq,
            "QRZ" => GenericLeader::Qrz,
            "DE" => GenericLeader::De,
            _ => return None,
        };
        let nc1 = match (leader, compound.kind) {
            (GenericLeader::Cq, 4) => NBASE2 + 1 + compound.k,
            (GenericLeader::Qrz, 4) => NBASE2 + PREFIX_SPAN + 1 + compound.k,
            (GenericLeader::De, 4) => NBASE2 + 2 * PREFIX_SPAN + 1 + compound.k,
            (GenericLeader::Cq, 5) => NBASE2 + 3 * PREFIX_SPAN + 1 + compound.k,
            (GenericLeader::Qrz, 5) => NBASE2 + 3 * PREFIX_SPAN + SUFFIX_SPAN + 1 + compound.k,
            (GenericLeader::De, 5) => NBASE2 + 3 * PREFIX_SPAN + 2 * SUFFIX_SPAN + 1 + compound.k,
            _ => return None,
        };
        return Some(pack_words(nc1, nc2, ng));
    }

    Some(pack_words(ordinary_nc1, nc2, ng))
}

/// Pack the pinned WSJT-X Type 6 free-text format.
///
/// Text is uppercased, internal whitespace is collapsed like `fmtmsg`,
/// padded to 13 characters, and truncated after character 13.
pub fn pack_free_text(text: &str) -> [u8; 12] {
    let normalized = text
        .split_ascii_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_ascii_uppercase();
    let mut characters = [b' '; 13];
    for (slot, character) in characters.iter_mut().zip(normalized.bytes()) {
        *slot = if TEXT_ALPHA.contains(&character) {
            character
        } else {
            b' '
        };
    }
    let code = |character: u8| {
        TEXT_ALPHA
            .iter()
            .position(|candidate| *candidate == character)
            .unwrap_or(36) as u32
    };
    let mut nc1 = 0u32;
    for &character in &characters[..5] {
        nc1 = 42 * nc1 + code(character);
    }
    let mut nc2 = 0u32;
    for &character in &characters[5..10] {
        nc2 = 42 * nc2 + code(character);
    }
    let mut nc3 = 0u32;
    for &character in &characters[10..] {
        nc3 = 42 * nc3 + code(character);
    }
    nc1 = 2 * nc1 + u32::from(nc3 & 32_768 != 0);
    nc2 = 2 * nc2 + u32::from(nc3 & 65_536 != 0);
    nc3 &= 32_767;
    pack_words(nc1, nc2, nc3 + 32_768)
}

fn unpack_free_text(nc1: u32, nc2: u32, ng: u32) -> String {
    let mut first = nc1;
    let mut second = nc2;
    let mut third = ng & 32_767;
    if first & 1 != 0 {
        third += 32_768;
    }
    first /= 2;
    if second & 1 != 0 {
        third += 65_536;
    }
    second /= 2;

    let mut characters = [b' '; 13];
    for index in (0..5).rev() {
        characters[index] = TEXT_ALPHA[(first % 42) as usize];
        first /= 42;
    }
    for index in (5..10).rev() {
        characters[index] = TEXT_ALPHA[(second % 42) as usize];
        second /= 42;
    }
    for index in (10..13).rev() {
        characters[index] = TEXT_ALPHA[(third % 42) as usize];
        third /= 42;
    }
    core::str::from_utf8(&characters)
        .unwrap_or_default()
        .trim_end()
        .to_owned()
}

/// Pack a complete JT4/JT9/JT65 message, falling back to Type 6 free
/// text when it is not a valid two-callsign standard message.
pub fn pack_message(message: &str) -> [u8; 12] {
    let normalized = message
        .split_ascii_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_ascii_uppercase()
        .chars()
        .take(22)
        .collect::<String>();
    let fields = normalized.split_ascii_whitespace().collect::<Vec<_>>();
    if fields.len() == 4
        && fields[0] == "CQ"
        && matches!(fields[1].len(), 1 | 3)
        && fields[1].bytes().all(|byte| byte.is_ascii_digit())
        && let Some(frequency) = fields[1].parse::<u16>().ok()
        && let Some(words) = pack_standard(&format!("CQ {frequency:03}"), fields[2], fields[3])
    {
        return words;
    }
    if fields.len() == 4 && fields[0] == "CQ" {
        let encoded_cq = if fields[1] == "DX" {
            Some("CQ9DX".to_owned())
        } else if fields[1].len() == 2 && fields[1].bytes().all(|byte| byte.is_ascii_uppercase()) {
            Some(format!("E9{}", fields[1]))
        } else {
            None
        };
        if let Some(encoded_cq) = encoded_cq
            && let Some(words) = pack_standard(&encoded_cq, fields[2], fields[3])
        {
            return words;
        }
    }
    if (2..=3).contains(&fields.len())
        && let Some(words) =
            pack_standard(fields[0], fields[1], fields.get(2).copied().unwrap_or(""))
    {
        return words;
    }
    pack_free_text(&normalized)
}

/// Convenience: unpack 12 six-bit words into a `Jt72Message`.
pub fn unpack(d: &[u8; 12]) -> Jt72Message {
    let (nc1, nc2, ng) = unpack_words(d);
    if ng >= 32768 {
        return Jt72Message::FreeText {
            text: unpack_free_text(nc1, nc2, ng),
        };
    }
    let Some(mut call2) = unpack_call(nc2) else {
        return Jt72Message::Unsupported { nc1, nc2, ng };
    };
    let grid = unpack_grid(ng);

    if let Some((leader, is_prefix, affix)) = unpack_generic_nc1(nc1) {
        call2 = if is_prefix {
            format!("{affix}/{call2}")
        } else {
            format!("{call2}/{affix}")
        };
        return Jt72Message::Standard {
            call1: leader.text().to_owned(),
            call2,
            grid_or_report: grid,
        };
    }

    let Some(mut call1) = unpack_call(nc1) else {
        return Jt72Message::Unsupported { nc1, nc2, ng };
    };
    let k = grid_to_k(&grid);

    // `DE` is the final special value in `unpackcall`. WSJT-X gives
    // it a dedicated legacy-second-call branch.
    if nc1 == 267_796_945 {
        let has_legacy_second_call = (451..=900).contains(&k);
        if has_legacy_second_call && let Some(compound) = apply_legacy_affix(&call2, k) {
            call2 = compound;
        }
        return Jt72Message::Standard {
            call1,
            call2,
            grid_or_report: if has_legacy_second_call {
                String::new()
            } else {
                grid
            },
        };
    }

    if (1..=450).contains(&k) {
        if let Some(compound) = apply_legacy_affix(&call1, k) {
            call1 = compound;
        }
    } else if (451..=900).contains(&k)
        && let Some(compound) = apply_legacy_affix(&call2, k)
    {
        call2 = compound;
    }

    // Reverse the two historical CQ activity shims in `packmsg`.
    if call1 == "CQ9DX" {
        call1 = "CQ DX".to_owned();
    } else if call1.len() == 4
        && call1.starts_with("E9")
        && call1[2..].bytes().all(|byte| byte.is_ascii_uppercase())
    {
        call1 = format!("CQ {}", &call1[2..]);
    }

    Jt72Message::Standard {
        call1,
        call2,
        grid_or_report: if k == 0 { grid } else { String::new() },
    }
}

// ─────────────────────────────────────────────────────────────────────────
// MessageCodec impl
// ─────────────────────────────────────────────────────────────────────────

use crate::core::{DecodeContext, MessageCodec, MessageFields};

/// JT 72-bit message codec. Used by JT65 and JT9.
#[derive(Copy, Clone, Debug, Default)]
pub struct Jt72Message_;

// The struct name `Jt72Message` is already taken by the output enum,
// so the codec type lives under a trailing underscore and is
// re-exported as `Jt72Codec` for callers.
pub type Jt72Codec = Jt72Message_;

impl MessageCodec for Jt72Message_ {
    type Unpacked = Jt72Message;
    const PAYLOAD_BITS: u32 = 72;
    const CRC_BITS: u32 = 0;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        let words = if let (Some(c1), Some(c2)) = (fields.call1.as_deref(), fields.call2.as_deref())
        {
            let rep = fields
                .grid
                .as_deref()
                .or(fields.free_text.as_deref())
                .unwrap_or("");
            pack_standard(c1, c2, rep)?
        } else {
            pack_free_text(fields.free_text.as_deref()?)
        };
        // Flatten the 12 × 6-bit words into 72 individual bits
        // (MSB-first within each word), matching how FEC stages
        // consume them elsewhere in mfsk-*.
        let mut bits = Vec::with_capacity(72);
        for &w in &words {
            for b in (0..6).rev() {
                bits.push((w >> b) & 1);
            }
        }
        Some(bits)
    }

    fn unpack(&self, payload: &[u8], _ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != 72 {
            return None;
        }
        let mut words = [0u8; 12];
        for (i, slot) in words.iter_mut().enumerate() {
            let mut w = 0u8;
            for b in 0..6 {
                w = (w << 1) | (payload[6 * i + b] & 1);
            }
            *slot = w;
        }
        Some(unpack(&words))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn call_roundtrip_standard() {
        for call in ["K1ABC", "K9AN", "JA1ABC", "VK3KCN", "G4BWP", "W7AV"] {
            let n = pack_call(call).unwrap_or_else(|| panic!("pack {call}"));
            let back = unpack_call(n).unwrap_or_else(|| panic!("unpack {call}"));
            assert_eq!(back, call, "roundtrip: {call}");
        }
    }

    #[test]
    fn call_special_tokens() {
        assert_eq!(pack_call("CQ"), Some(NBASE + 1));
        assert_eq!(pack_call("QRZ"), Some(NBASE + 2));
        assert_eq!(unpack_call(NBASE + 1).as_deref(), Some("CQ"));
        assert_eq!(unpack_call(NBASE + 2).as_deref(), Some("QRZ"));
    }

    #[test]
    fn grid_roundtrip() {
        for grid in ["FN42", "PM95", "JN58", "AA00", "RR99"] {
            let ng = pack_grid_or_report(grid).unwrap_or_else(|| panic!("pack {grid}"));
            let back = unpack_grid(ng);
            assert_eq!(back, grid, "roundtrip {grid}");
        }
    }

    /// Pin the canonical WSJT-X `ng` values for a few grids so we cannot
    /// regress on the `packgrid` formula again. Values derived directly
    /// from `lib/packjt.f90:341-345` + `grid2deg.f90`.
    #[test]
    fn grid_wsjtx_canonical_ng() {
        for (grid, expected_ng) in [
            ("EM41", 24421u32),
            ("FN42", 22632),
            ("PM95", 3725),
            ("JN58", 15258),
            ("AA00", 32220),
            ("RR99", 179),
        ] {
            let ng = pack_grid_or_report(grid).unwrap_or_else(|| panic!("pack {grid}"));
            assert_eq!(ng, expected_ng, "WSJT-X canonical ng for {grid}");
        }
    }

    #[test]
    fn grid_reports_and_tokens() {
        for s in ["RO", "RRR", "73", "-15", "R-05"] {
            let ng = pack_grid_or_report(s).unwrap_or_else(|| panic!("pack {s}"));
            assert_eq!(unpack_grid(ng), s);
        }
    }

    #[test]
    fn standard_message_roundtrip() {
        let words = pack_standard("K1ABC", "JA1ABC", "FN42").expect("pack");
        let m = unpack(&words);
        assert_eq!(
            m,
            Jt72Message::Standard {
                call1: "K1ABC".into(),
                call2: "JA1ABC".into(),
                grid_or_report: "FN42".into(),
            }
        );
    }

    #[test]
    fn free_text_roundtrip_uses_wsjt_base42_alphabet() {
        for (input, expected) in [
            ("TNX JOE -14 73", "TNX JOE -14 7"),
            ("RO", "RO"),
            ("A+B/C.D? 123", "A+B/C.D? 123"),
            ("lower   case", "LOWER CASE"),
        ] {
            let words = pack_free_text(input);
            assert_eq!(
                unpack(&words),
                Jt72Message::FreeText {
                    text: expected.to_owned()
                }
            );
            assert_eq!(unpack(&words).to_string(), expected);
        }
    }

    #[test]
    fn complete_message_prefers_standard_then_falls_back_to_text() {
        assert!(matches!(
            unpack(&pack_message("CQ K1ABC FN42")),
            Jt72Message::Standard { .. }
        ));
        assert_eq!(
            unpack(&pack_message("TNX JOE -14 73")),
            Jt72Message::FreeText {
                text: "TNX JOE -14 7".to_owned()
            }
        );
    }

    #[test]
    fn wsjtx_v3_0_2_testmsg_vectors_match_exactly() {
        let fixture = include_str!("../../tests/data/jt72_wsjtx_v3_0_2.tsv");
        let mut checked = 0usize;
        for (line_number, line) in fixture.lines().enumerate() {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields = line.split('\t').collect::<Vec<_>>();
            assert_eq!(
                fields.len(),
                4,
                "malformed fixture line {}",
                line_number + 1
            );
            let message = fields[0];
            let expected_decode = fields[1];
            let message_type = fields[2].parse::<u8>().expect("valid itype");
            assert!((1..=6).contains(&message_type));
            let expected_words = fields[3]
                .split_ascii_whitespace()
                .map(|word| word.parse::<u8>().expect("valid six-bit word"))
                .collect::<Vec<_>>();
            let expected_words: [u8; 12] =
                expected_words.try_into().unwrap_or_else(|words: Vec<u8>| {
                    panic!(
                        "fixture line {} has {} words instead of 12",
                        line_number + 1,
                        words.len()
                    )
                });

            let actual_words = pack_message(message);
            assert_eq!(
                actual_words, expected_words,
                "packed words differ from pinned WSJT-X for {message:?} (itype {message_type})"
            );
            assert_eq!(
                unpack(&actual_words).to_string(),
                expected_decode,
                "decoded text differs from pinned WSJT-X for {message:?} (itype {message_type})"
            );
            checked += 1;
        }
        assert_eq!(
            checked, 68,
            "the complete pinned upstream test set must run"
        );
    }

    #[test]
    fn compound_calls_use_only_the_wire_forms_allowed_by_packmsg() {
        assert_eq!(LEGACY_PREFIXES.split_ascii_whitespace().count(), 339);
        assert!(matches!(
            unpack(&pack_message("1A/KA1ABC WB9XYZ EN34")),
            Jt72Message::Standard {
                grid_or_report,
                ..
            } if grid_or_report.is_empty()
        ));
        assert!(matches!(
            unpack(&pack_message("A000/KA1ABC WB9XYZ FM07")),
            Jt72Message::FreeText { .. }
        ));
        assert!(matches!(
            unpack(&pack_message("KA1ABC/P WB9XYZ/A")),
            Jt72Message::FreeText { .. }
        ));
    }

    #[test]
    fn codec_trait_roundtrip() {
        let codec = Jt72Message_;
        let fields = MessageFields {
            call1: Some("K1ABC".into()),
            call2: Some("JA1ABC".into()),
            grid: Some("PM95".into()),
            ..MessageFields::default()
        };
        let payload = codec.pack(&fields).expect("pack");
        assert_eq!(payload.len(), 72);
        let ctx = DecodeContext::default();
        let m = codec.unpack(&payload, &ctx).expect("unpack");
        assert!(matches!(m, Jt72Message::Standard { .. }));
    }

    #[test]
    fn pack_words_bit_layout() {
        // Sentinel values let us check the bit routing into dat(1..12).
        let nc1 = 0x0F00_00F0u32; // 28-bit field exercising edges
        let nc2 = 0x0A00_000Au32;
        let ng = 0x0F0Fu32;
        let words = pack_words(nc1 & 0x0fff_ffff, nc2 & 0x0fff_ffff, ng & 0xffff);
        let (n1b, n2b, ngb) = unpack_words(&words);
        assert_eq!(n1b, nc1 & 0x0fff_ffff);
        assert_eq!(n2b, nc2 & 0x0fff_ffff);
        assert_eq!(ngb, ng & 0xffff);
    }

    #[test]
    fn cq_standard_message() {
        let words = pack_standard("CQ", "K1ABC", "FN42").expect("pack CQ");
        let m = unpack(&words);
        match m {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                assert_eq!(call1, "CQ");
                assert_eq!(call2, "K1ABC");
                assert_eq!(grid_or_report, "FN42");
            }
            other => panic!("expected Standard, got {:?}", other),
        }
    }
}
