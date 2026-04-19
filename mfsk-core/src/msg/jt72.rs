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
//! ## Scope
//!
//! The MVP covers the **standard message** (two callsigns plus a
//! grid / report) and its documented report-code variants (plain
//! `-NN` / `RNN` / `RO` / `RRR` / `73`). Free text (Type 6) and the
//! compound-callsign Type 2–5 cases are detected but reported as
//! `Standard { .., grid: "…" }` rather than fully unpacked; those
//! less common paths can be ported from `getpfx1` / `getpfx2` when
//! needed.

use core::fmt;

/// Base used to pack a 6-character callsign into a 28-bit integer.
/// Matches `NBASE` in WSJT-X: `37 * 36 * 10 * 27 * 27 * 27 = 262 177 560`.
const NBASE: u32 = 37 * 36 * 10 * 27 * 27 * 27;

/// Base used for 4-character Maidenhead grids: `180 * 180 = 32 400`.
/// Values above this encode report codes (see `unpack_grid`).
const NGBASE: u32 = 180 * 180;

/// Decoded JT 72-bit message payload.
///
/// The enum shape mirrors the `itype` classification in WSJT-X
/// `packmsg` (Type 1 = standard, Types 2–5 = compound-callsign
/// variants, Type 6 = free text) but for the MVP everything that
/// isn't a plain standard message is collapsed into `Unsupported`.
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
    /// A message whose fields decode but don't fit the standard
    /// pattern yet (compound callsign prefix/suffix, free text).
    /// Raw integer fields are exposed for callers that want to dig in.
    Unsupported { nc1: u32, nc2: u32, ng: u32 },
}

impl fmt::Display for Jt72Message {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => write!(f, "{} {} {}", call1, call2, grid_or_report),
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
    let bytes = call.as_bytes();
    // Special tokens handled by WSJT-X's `packcall`.
    match call {
        "CQ" => return Some(NBASE + 1),
        "QRZ" => return Some(NBASE + 2),
        "DE" => return Some(267_796_945),
        _ => {}
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
        if (b'a'..=b'z').contains(t) {
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

    let s = core::str::from_utf8(&chars).ok()?;
    Some(s.trim().to_string())
}

// ─────────────────────────────────────────────────────────────────────────
// Grid / report (16-bit `ng`)
// ─────────────────────────────────────────────────────────────────────────

/// Pack a 4-character grid locator into `ng` via the Maidenhead →
/// integer mapping used by WSJT-X `packgrid` (without the
/// extended-range report tricks — callers can build those up
/// manually).
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
    // Mirror the int(dlong) / int(dlat+90) arithmetic.
    let dlong_int = -180 + fl * 20 + sl * 2 + 1;
    let lat_int = fla * 10 + sla;
    let ng = ((dlong_int + 180) / 2) * 180 + lat_int;
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
            if let Some(rest) = other.strip_prefix('-') {
                if let Ok(n) = rest.parse::<i32>() {
                    if (1..=30).contains(&n) {
                        return Some(NGBASE + 1 + n as u32);
                    }
                }
            }
            if let Some(rest) = other.strip_prefix("R-") {
                if let Ok(n) = rest.parse::<i32>() {
                    if (1..=30).contains(&n) {
                        return Some(NGBASE + 31 + n as u32);
                    }
                }
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
        // Standard grid. Reverse the (int(dlong), int(dlat+90)) path.
        let long = (ng / 180) as i32;
        let lat = (ng % 180) as i32;
        // long = (dlong_int + 180) / 2 (integer division).
        // To recover a valid grid letter/digit, step by 2° per sub.
        let fl = long / 10;
        let sl = (long % 10) / 1; // each step is 2° long = 1 sub step
        let fla = lat / 10;
        let sla = lat % 10;
        let mut g = [0u8; 4];
        g[0] = b'A' + fl as u8;
        g[1] = b'A' + fla as u8;
        g[2] = b'0' + sl as u8;
        g[3] = b'0' + sla as u8;
        return core::str::from_utf8(&g).unwrap_or("????").to_string();
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
    let nc1 = pack_call(call1)?;
    let nc2 = pack_call(call2)?;
    let ng = pack_grid_or_report(grid_or_report)?;
    Some(pack_words(nc1, nc2, ng))
}

/// Convenience: unpack 12 six-bit words into a `Jt72Message`.
pub fn unpack(d: &[u8; 12]) -> Jt72Message {
    let (nc1, nc2, ng) = unpack_words(d);
    let c1 = unpack_call(nc1);
    let c2 = unpack_call(nc2);
    // Text / free-form messages set a `ng + 32768` high bit that
    // this MVP doesn't decode — collapse those and anything outside
    // the standard NBASE range into `Unsupported`.
    if ng >= 32768 {
        return Jt72Message::Unsupported { nc1, nc2, ng };
    }
    match (c1, c2) {
        (Some(call1), Some(call2)) => Jt72Message::Standard {
            call1,
            call2,
            grid_or_report: unpack_grid(ng),
        },
        _ => Jt72Message::Unsupported { nc1, nc2, ng },
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
        let c1 = fields.call1.as_deref()?;
        let c2 = fields.call2.as_deref()?;
        let rep = fields
            .grid
            .as_deref()
            .or(fields.free_text.as_deref())
            .unwrap_or("");
        let words = pack_standard(c1, c2, rep)?;
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
    fn codec_trait_roundtrip() {
        let codec = Jt72Message_::default();
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
