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
//! [`pack_message`] / [`unpack`] are a port of the whole of `packmsg` /
//! `unpackmsg` (`lib/packjt.f90`, v3.2.0-rc1): standard messages, `CQ nnn` /
//! `CQ DX` / `CQ xx`, add-on prefixes and suffixes (types 2-5) and free text
//! (type 6), tested row by row against rc1's own routines
//! (`tests/fixtures/jt72_rc1.tsv`). [`pack_standard`], [`pack_call`] and
//! friends remain the cheaper route for the plain two-call form.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

/// Base used to pack a 6-character callsign into a 28-bit integer.
/// Matches `NBASE` in WSJT-X: `37 * 36 * 10 * 27 * 27 * 27 = 262 177 560`.
const NBASE: u32 = 37 * 36 * 10 * 27 * 27 * 27;

/// Base used for 4-character Maidenhead grids: `180 * 180 = 32 400`.
/// Values above this encode report codes (see `unpack_grid`).
const NGBASE: u32 = 180 * 180;

/// Decoded JT 72-bit message payload, rendered as WSJT-X `unpackmsg` prints it
/// (`lib/packjt.f90:537-661`, `unpacktext` `:711-741`). Every 72-bit pattern
/// renders to some text upstream, so every pattern is one of these variants.
///
/// `Display` is exactly upstream's text with its trailing blanks removed.
/// The `Unsupported { nc1, nc2, ng }` variant of 0.13 is gone: upstream has no
/// bit pattern it cannot render.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Jt72Message {
    /// Two calls and a grid / report (`itype` 1): `call1 call2 grid`, with an empty
    /// `grid_or_report` when the message has none. `call1` is as printed, so
    /// `CQ DX`, `CQ NA` (WSJT-X's `CQ9DX` / `E9xx` encodings) and `CQ 123` (a reply
    /// frequency) appear in their printed forms.
    Standard {
        call1: String,
        call2: String,
        /// A 4-char grid ("FN42"), a report ("-15", "R-05", "+05"), or "RO" / "RRR" / "73".
        grid_or_report: String,
    },
    /// Every other message built from callsigns: an add-on prefix or suffix
    /// (`CQ PJ4/K1ABC FK52`, `K1ABC/P W9XYZ`, `DE W9XYZ/QRP`), or a grid that
    /// `unpackmsg` reads as a prefix location. The upstream text.
    Compound(String),
    /// Free text (`ng >= 32768`): up to 13 characters of
    /// `0-9 A-Z space + - . / ?`. The upstream text.
    FreeText(String),
}

impl Jt72Message {
    /// The all-zero codeword's text, `000AAA 000AAA RA90`. A decoder that
    /// converged on it found nothing: `jt9fano.f90:88`
    /// (`index(msg,'000AAA ')`, anywhere in the 22-character message),
    /// `extract4.f90:61` and `decode65b.f90:27` all blank it. Measured: a
    /// noise-free JT9 period decoded the true message plus three of these at
    /// 17, -10 and -15 dB.
    pub fn is_all_zero_codeword(&self) -> bool {
        let mut t = self.to_string();
        while t.len() < 22 {
            t.push(' ');
        }
        t.contains("000AAA ")
    }

    /// `decode65b.f90:27-28`: JT65 blanks a message that starts `000AAA ` or
    /// `0L6MWK ` (a "birdie") as no decode.
    #[allow(dead_code)] // the JT65 scan that calls it needs an FFT backend
    pub(crate) fn is_jt65_suppressed(&self) -> bool {
        let mut t = self.to_string();
        t.push(' ');
        t.starts_with("000AAA ") || t.starts_with("0L6MWK ")
    }

    /// The message's 72 bits, one per byte, most significant first: the
    /// identity key a decoded row carries in `RowDetail::info` (#592).
    ///
    /// Re-packed from the message rather than kept from the decode, because
    /// the message is the granularity the JT9 and JT65 de-duplication works
    /// at (`Jt72Message` equality). A repack is used only when it unpacks to
    /// this very message; `None` for a message whose bits the packer would not
    /// reproduce (a non-canonical bit pattern of a corrupt decode).
    pub fn bits72(&self) -> Option<Vec<u8>> {
        let mut cands: Vec<[u8; 12]> = Vec::new();
        if let Jt72Message::Standard {
            call1,
            call2,
            grid_or_report,
        } = self
            && let Some(w) = pack_standard(call1, call2, grid_or_report)
        {
            cands.push(w);
        }
        if let Ok(p) = pack_message(&self.to_string()) {
            cands.push(p.words);
        }
        let words = cands.into_iter().find(|w| unpack(w) == *self)?;
        Some(
            words
                .iter()
                .flat_map(|w| (0..6).rev().map(move |b| (w >> b) & 1))
                .collect(),
        )
    }
}

impl fmt::Display for Jt72Message {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                let t = format!("{} {} {}", call1, call2, grid_or_report);
                f.write_str(t.trim_end_matches(' '))
            }
            Jt72Message::Compound(t) | Jt72Message::FreeText(t) => f.write_str(t),
        }
    }
}

/// What [`pack_message`] produced: the twelve 6-bit words and `packmsg`'s `itype`.
///
/// `itype`: 1 standard, 2 add-on prefix (from the table), 3 add-on suffix (from the
/// table), 4 other prefix, 5 other suffix, 6 free text.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct Jt72Packed {
    pub words: [u8; 12],
    pub itype: i32,
}

/// Why [`pack_message`] refused a text. `packmsg` itself refuses nothing: it falls
/// back to free text, and a text of more than 22 characters is cut to 22 by the
/// `character*22` its callers pass.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Jt72PackError {
    /// The text has a non-ASCII character. Divergence: Fortran would take its UTF-8
    /// bytes one by one as unknown characters (blanks); a message that cannot be sent
    /// as written is refused instead.
    NonAscii,
}

impl fmt::Display for Jt72PackError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Jt72PackError::NonAscii => f.write_str("the message has a non-ASCII character"),
        }
    }
}

impl core::error::Error for Jt72PackError {}

// ─────────────────────────────────────────────────────────────────────────
// Character helpers (WSJT-X `nchar` / `unpackcall` tables)
// ─────────────────────────────────────────────────────────────────────────

/// Translate a callsign char to its `nchar` index: digit→0..9,
/// letter→10..35, space→36. Returns `None` for anything else.
/// Accepts lowercase too — defensive; every caller here already
/// uppercases before this runs (see [`pack_call`]'s digit-position
/// step), so the lowercase branch never actually fires on live
/// input, but the extraction into [`crate::msg::callsign28`]
/// preserves it rather than narrowing behaviour.
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
///
/// Special tokens (`CQ`/`QRZ`/`DE`) are JT9/JT65-specific — WSPR has
/// no bare-CQ message type — so they're handled here, above
/// [`crate::msg::callsign28::pack_call28`]'s shared core.
pub fn pack_call(call: &str) -> Option<u32> {
    match call {
        "CQ" => return Some(NBASE + 1),
        "QRZ" => return Some(NBASE + 2),
        "DE" => return Some(267_796_945),
        _ => {}
    }
    crate::msg::callsign28::pack_call28(call, nchar)
}

/// Unpack a 28-bit integer back into a callsign or special token.
/// Returns `None` for values outside the base-37/36/10/27³ range
/// (those encode compound-callsign variants).
pub fn unpack_call(ncall: u32) -> Option<String> {
    match ncall {
        v if v == NBASE + 1 => return Some("CQ".into()),
        v if v == NBASE + 2 => return Some("QRZ".into()),
        267_796_945 => return Some("DE".into()),
        _ => {}
    }
    crate::msg::callsign28::unpack_call28(ncall)
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

// ─────────────────────────────────────────────────────────────────────────
// packmsg / unpackmsg: the whole 72-bit message, a port of lib/packjt.f90 (rc1)
// ─────────────────────────────────────────────────────────────────────────
//
// The Fortran works on blank-padded `character*N` variables; the port keeps
// fixed-size byte arrays with the same padding and truncation, so the odd cases
// (a 7th callsign character dropped, a 5th grid character dropped, a fourth word
// ignored) come out as upstream's do. Comments name the Fortran line they port;
// indices there are 1-based.
//
// Not changed between v3.2.0-rc1 and v3.3.0-beta2: packjt.f90, fmtmsg.f90,
// grid2deg.f90, deg2grid.f90, pfx.f90 (git diff, empty).

/// `pfx.f90`: the add-on prefixes `getpfx1` looks up (`NZ = 339`), trailing blanks removed.
const PFX: [&str; 339] = [
    "1A", "1S", "3A", "3B6", "3B8", "3B9", "3C", "3C0", "3D2", "3D2C", "3D2R", "3DA", "3V", "3W",
    "3X", "3Y", "3YB", "3YP", "4J", "4L", "4S", "4U1I", "4U1U", "4W", "4X", "5A", "5B", "5H", "5N",
    "5R", "5T", "5U", "5V", "5W", "5X", "5Z", "6W", "6Y", "7O", "7P", "7Q", "7X", "8P", "8Q", "8R",
    "9A", "9G", "9H", "9J", "9K", "9L", "9M2", "9M6", "9N", "9Q", "9U", "9V", "9X", "9Y", "A2",
    "A3", "A4", "A5", "A6", "A7", "A9", "AP", "BS7", "BV", "BV9", "BY", "C2", "C3", "C5", "C6",
    "C9", "CE", "CE0X", "CE0Y", "CE0Z", "CE9", "CM", "CN", "CP", "CT", "CT3", "CU", "CX", "CY0",
    "CY9", "D2", "D4", "D6", "DL", "DU", "E3", "E4", "EA", "EA6", "EA8", "EA9", "EI", "EK", "EL",
    "EP", "ER", "ES", "ET", "EU", "EX", "EY", "EZ", "F", "FG", "FH", "FJ", "FK", "FKC", "FM", "FO",
    "FOA", "FOC", "FOM", "FP", "FR", "FRG", "FRJ", "FRT", "FT5W", "FT5X", "FT5Z", "FW", "FY", "M",
    "MD", "MI", "MJ", "MM", "MU", "MW", "H4", "H40", "HA", "HB", "HB0", "HC", "HC8", "HH", "HI",
    "HK", "HK0", "HK0M", "HL", "HM", "HP", "HR", "HS", "HV", "HZ", "I", "IS", "IS0", "J2", "J3",
    "J5", "J6", "J7", "J8", "JA", "JDM", "JDO", "JT", "JW", "JX", "JY", "K", "KG4", "KH0", "KH1",
    "KH2", "KH3", "KH4", "KH5", "KH5K", "KH6", "KH7", "KH8", "KH9", "KL", "KP1", "KP2", "KP4",
    "KP5", "LA", "LU", "LX", "LY", "LZ", "OA", "OD", "OE", "OH", "OH0", "OJ0", "OK", "OM", "ON",
    "OX", "OY", "OZ", "P2", "P4", "PA", "PJ2", "PJ7", "PY", "PY0F", "PT0S", "PY0T", "PZ", "R1F",
    "R1M", "S0", "S2", "S5", "S7", "S9", "SM", "SP", "ST", "SU", "SV", "SVA", "SV5", "SV9", "T2",
    "T30", "T31", "T32", "T33", "T5", "T7", "T8", "T9", "TA", "TF", "TG", "TI", "TI9", "TJ", "TK",
    "TL", "TN", "TR", "TT", "TU", "TY", "TZ", "UA", "UA2", "UA9", "UK", "UN", "UR", "V2", "V3",
    "V4", "V5", "V6", "V7", "V8", "VE", "VK", "VK0H", "VK0M", "VK9C", "VK9L", "VK9M", "VK9N",
    "VK9W", "VK9X", "VP2E", "VP2M", "VP2V", "VP5", "VP6", "VP6D", "VP8", "VP8G", "VP8H", "VP8O",
    "VP8S", "VP9", "VQ9", "VR", "VU", "VU4", "VU7", "XE", "XF4", "XT", "XU", "XW", "XX9", "XZ",
    "YA", "YB", "YI", "YJ", "YK", "YL", "YN", "YO", "YS", "YU", "YV", "YV0", "Z2", "Z3", "ZA",
    "ZB", "ZC4", "ZD7", "ZD8", "ZD9", "ZF", "ZK1N", "ZK1S", "ZK2", "ZK3", "ZL", "ZL7", "ZL8",
    "ZL9", "ZP", "ZS", "ZS8", "KC4", "E5",
];

/// `pfx.f90`: the add-on suffixes (`NZ2 = 12`).
const SFX: [u8; 12] = *b"P0123456789A";

/// `addpfx` of `common/pfxcom/`: nothing in `jt9` / `wsjtx` assigns it (only
/// map65's `ftninit.f90:16` does), so the common block keeps its zero fill.
const ADDPFX: [u8; 8] = [0; 8];

const NZ: i32 = 339;
const NZ2: i32 = 12;

/// Fortran `character*N` assignment: truncate or blank-pad.
fn pad<const N: usize>(s: &[u8]) -> [u8; N] {
    let mut a = [b' '; N];
    let n = s.len().min(N);
    a[..n].copy_from_slice(&s[..n]);
    a
}

/// Fortran `len_trim`.
fn len_trim(s: &[u8]) -> usize {
    s.iter().rposition(|&c| c != b' ').map_or(0, |p| p + 1)
}

/// Fortran `index(s,' ')`: 1-based, 0 when there is none.
fn index_blank(s: &[u8]) -> usize {
    s.iter().position(|&c| c == b' ').map_or(0, |p| p + 1)
}

fn is_digit(c: u8) -> bool {
    c.is_ascii_digit()
}

fn is_upper(c: u8) -> bool {
    c.is_ascii_uppercase()
}

/// `packjt.f90` `nchar` (`:~960`): 0-9 -> 0-9, letters -> 10-35, any other
/// printable -> 36, control characters -> -1.
fn nchar_f(c: u8) -> i32 {
    match c {
        b'0'..=b'9' => (c - b'0') as i32,
        b'A'..=b'Z' => (c - b'A') as i32 + 10,
        b'a'..=b'z' => (c - b'a') as i32 + 10,
        c if c >= b' ' => 36,
        _ => -1,
    }
}

/// `fmtmsg.f90`: upper-case, then collapse runs of blanks (at most 37 times).
fn fmtmsg(msg: &mut [u8; 22]) {
    let mut iz = 22usize;
    for (i, c) in msg.iter_mut().enumerate() {
        c.make_ascii_uppercase();
        if *c != b' ' {
            iz = i + 1;
        }
    }
    for _ in 0..37 {
        let hay = &msg[..iz];
        let Some(p) = hay.windows(2).position(|w| w == b"  ") else {
            break;
        };
        // msg = msg(1:ib2)//msg(ib2+2:) with ib2 = p + 1
        msg.copy_within(p + 2.., p + 1);
        msg[21] = b' ';
        iz -= 1;
    }
}

/// `packjt.f90` `packcall` (`:~74-147`). `callsign` is changed in place, as upstream's
/// actual argument `c1(1:6)` is. Returns `(ncall, text)`; `text` means "not a callsign".
fn packcall(cs: &mut [u8; 6]) -> (u32, bool) {
    // Work-around for Swaziland prefix.
    if &cs[..4] == b"3DA0" {
        *cs = pad(&[b"3D0".as_slice(), &cs[4..6]].concat());
    }
    // Work-around for Guinea prefixes.
    if &cs[..2] == b"3X" && is_upper(cs[2]) {
        *cs = pad(&[b"Q".as_slice(), &cs[2..6]].concat());
    }
    if &cs[..3] == b"CQ " {
        let mut ncall = NBASE + 1;
        if is_digit(cs[3]) && is_digit(cs[4]) && is_digit(cs[5]) {
            let nfreq =
                (cs[3] - b'0') as u32 * 100 + (cs[4] - b'0') as u32 * 10 + (cs[5] - b'0') as u32;
            ncall = NBASE + 3 + nfreq;
        }
        return (ncall, false);
    } else if &cs[..4] == b"QRZ " {
        return (NBASE + 2, false);
    } else if &cs[..3] == b"DE " {
        return (267_796_945, false);
    }

    let mut tmp: [u8; 6];
    if is_digit(cs[2]) {
        tmp = *cs;
    } else if is_digit(cs[1]) {
        if cs[5] != b' ' {
            return (0, true);
        }
        tmp = pad(&[b" ".as_slice(), &cs[..5]].concat());
    } else {
        return (0, true);
    }
    tmp.make_ascii_uppercase();

    let n1 = is_upper(tmp[0]) || tmp[0] == b' ' || is_digit(tmp[0]);
    let n2 = is_upper(tmp[1]) || is_digit(tmp[1]);
    let n3 = is_digit(tmp[2]);
    let n4 = is_upper(tmp[3]) || tmp[3] == b' ';
    let n5 = is_upper(tmp[4]) || tmp[4] == b' ';
    let n6 = is_upper(tmp[5]) || tmp[5] == b' ';
    if !(n1 && n2 && n3 && n4 && n5 && n6) {
        return (0, true);
    }
    let mut n = nchar_f(tmp[0]);
    n = 36 * n + nchar_f(tmp[1]);
    n = 10 * n + nchar_f(tmp[2]);
    n = 27 * n + nchar_f(tmp[3]) - 10;
    n = 27 * n + nchar_f(tmp[4]) - 10;
    n = 27 * n + nchar_f(tmp[5]) - 10;
    (n as u32, false)
}

/// Result of a Fortran list-directed `read(str,*) n` of one integer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ListInt {
    /// A value was read.
    Val(i32),
    /// A null value (leading `,` `/` `;`): `n` keeps what it had.
    Null,
    /// `err=` taken.
    Err,
    /// `end=` taken (nothing but blanks).
    End,
}

/// libgfortran list-directed integer read (`list_read.c` `read_integer`) on an
/// internal unit, as far as the characters a callsign-grid token can hold need.
fn list_read_int(s: &[u8]) -> ListInt {
    let sep = |c: u8| matches!(c, b' ' | b',' | b'/' | b';' | b'\t' | b'\r' | b'\n');
    let mut i = 0;
    while i < s.len() && (s[i] == b' ' || s[i] == b'\t') {
        i += 1;
    }
    if i >= s.len() {
        return ListInt::End;
    }
    if sep(s[i]) {
        return ListInt::Null;
    }
    let mut neg = false;
    if s[i] == b'-' || s[i] == b'+' {
        neg = s[i] == b'-';
        i += 1;
        if i >= s.len() || !is_digit(s[i]) {
            return ListInt::Err;
        }
    }
    if !is_digit(s[i]) {
        return ListInt::Err;
    }
    let mut v: i64 = 0;
    while i < s.len() && is_digit(s[i]) {
        v = v * 10 + (s[i] - b'0') as i64;
        i += 1;
    }
    if i >= s.len() || sep(s[i]) {
        return ListInt::Val(if neg { -(v as i32) } else { v as i32 });
    }
    if s[i] == b'*' && !neg {
        // r*c: a repeat count, then the value (or nothing: a null value)
        if v == 0 {
            return ListInt::Err;
        }
        let rest = &s[i + 1..];
        if rest.is_empty() || sep(rest[0]) {
            return ListInt::Null;
        }
        return match list_read_int(rest) {
            ListInt::End => ListInt::Null,
            o => o,
        };
    }
    ListInt::Err
}

/// `grid2deg.f90` on `grid//'mm55'`, `packgrid`'s `:40` branch: `ng` of a 4-character grid.
fn ng_of_grid(g: &[u8; 4]) -> u32 {
    let nlong = 180 - 20 * (g[0] as i32 - b'A' as i32);
    let n20d = 2 * (g[2] as i32 - b'0' as i32);
    let xminlong = 5.0f32 * ((b'm' - b'a') as f32 + 0.5);
    let dlong = (nlong - n20d) as f32 - xminlong / 60.0;
    let nlat = -90 + 10 * (g[1] as i32 - b'A' as i32) + g[3] as i32 - b'0' as i32;
    let xminlat = 2.5f32 * ((b'm' - b'a') as f32 + 0.5);
    let dlat = nlat as f32 + xminlat / 60.0;
    let long = dlong as i32;
    let lat = (dlat + 90.0) as i32;
    (((long + 180) / 2) * 180 + lat) as u32
}

/// `packjt.f90` `packgrid` (`:~149-218`). Returns `(ng, text)`.
fn packgrid(grid: &[u8; 4]) -> (u32, bool) {
    if grid == b"    " {
        return (NGBASE + 1, false);
    }
    // Reports in the original range, -01 to -30 dB. An unset `n` after a null
    // value is undefined upstream; 0 here.
    if grid[0] == b'-' {
        match list_read_int(&grid[1..3]) {
            ListInt::Err | ListInt::End => return (0, true),
            ListInt::Val(n) if (1..=30).contains(&n) => return (NGBASE + 1 + n as u32, false),
            _ => {}
        }
    } else if &grid[..2] == b"R-" {
        match list_read_int(&grid[2..4]) {
            ListInt::Err | ListInt::End => return (0, true),
            ListInt::Val(n) if (1..=30).contains(&n) => return (NGBASE + 31 + n as u32, false),
            _ => {}
        }
    } else if grid == b"RO  " {
        return (NGBASE + 62, false);
    } else if grid == b"RRR " {
        return (NGBASE + 63, false);
    } else if grid == b"73  " {
        return (NGBASE + 64, false);
    }

    // 10: extended-range reports, -50 to -31 and 0 to +49.
    let mut n: i32 = 99;
    let c1 = grid[0];
    match list_read_int(grid) {
        ListInt::Val(v) => n = v,
        ListInt::Null => {}
        ListInt::Err | ListInt::End => {
            if let ListInt::Val(v) = list_read_int(&grid[1..4]) {
                n = v;
            }
        }
    }
    let mut g = *grid;
    if (-50..=49).contains(&n) {
        let v = (n + 50) as u8;
        g = [
            if c1 == b'R' { b'L' } else { b'K' },
            b'A',
            b'0' + v / 10,
            b'0' + v % 10,
        ];
    } else {
        // Maybe it's free text?
        let text = !(b'A'..=b'R').contains(&g[0])
            || !(b'A'..=b'R').contains(&g[1])
            || !is_digit(g[2])
            || !is_digit(g[3]);
        if text {
            return (0, true);
        }
    }
    (ng_of_grid(&g), false)
}

/// `packjt.f90` `unpackcall` (`:~149-262`): `(word, iv2, psfx)`.
fn unpackcall(ncall: u32) -> ([u8; 12], i32, [u8; 4]) {
    const C: &[u8; 37] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";
    let mut word: [u8; 12] = pad(b"......");
    let mut psfx = [b' '; 4];
    let mut iv2 = 0;
    let mut n = ncall;
    if n < 262_177_560 {
        word = pad(b"......");
        word[5] = C[(n % 27) as usize + 10];
        n /= 27;
        word[4] = C[(n % 27) as usize + 10];
        n /= 27;
        word[3] = C[(n % 27) as usize + 10];
        n /= 27;
        word[2] = C[(n % 10) as usize];
        n /= 10;
        word[1] = C[(n % 36) as usize];
        n /= 36;
        word[0] = C[n as usize];
        if let Some(i) = word[..4].iter().position(|&c| c != b' ') {
            word = pad(&word[i..]);
        }
    } else if n < 267_796_946 {
        // A JT65v2 message: CQ / QRZ / DE with a prefix or suffix.
        let mut put = |lo: u32, len: usize, k: i32| {
            iv2 = k;
            let mut m = n - lo;
            // psfx(len), ..., psfx(2) take mod 37; psfx(1) takes the rest (`i=n+1`).
            for p in (1..len).rev() {
                psfx[p] = C[(m % 37) as usize];
                m /= 37;
            }
            psfx[0] = C[m as usize];
        };
        match n {
            262_178_563..=264_002_071 => put(262_178_563, 4, 1),
            264_002_072..=265_825_580 => put(264_002_072, 4, 2),
            265_825_581..=267_649_089 => put(265_825_581, 4, 3),
            267_649_090..=267_698_374 => put(267_649_090, 3, 4),
            267_698_375..=267_747_659 => put(267_698_375, 3, 5),
            267_747_660..=267_796_944 => put(267_747_660, 3, 6),
            267_796_945 => iv2 = 7,
            _ => {}
        }
    }
    // 999:
    if &word[..3] == b"3D0" {
        word = pad(&[b"3DA0".as_slice(), &word[3..]].concat());
    }
    if word[0] == b'Q' && is_upper(word[1]) {
        word = pad(&[b"3X".as_slice(), &word[1..]].concat());
    }
    (word, iv2, psfx)
}

/// `packjt.f90` `getpfx1` (`:~743-835`): the add-on prefix / suffix of a callsign.
/// `callsign` is changed in place (to the bare call); returns `(k, nv2)`.
fn getpfx1(callsign: &mut [u8; 12]) -> (i32, i32) {
    let callsign0 = *callsign;
    let mut nv2 = 1;
    let mut iz = index_blank(callsign) as i32 - 1;
    if iz < 0 {
        iz = 12;
    }
    let iz_u = iz as usize;
    let islash = callsign[..iz_u]
        .iter()
        .position(|&c| c == b'/')
        .map_or(0, |p| p + 1) as i32;
    let mut k = 0i32;

    if islash > 0 && islash <= iz - 4 {
        // Add-on prefix
        let c: [u8; 8] = pad(&callsign[..islash as usize - 1]);
        *callsign = pad(&callsign0[islash as usize..iz_u]);
        for (i, p) in PFX.iter().enumerate() {
            if pad::<8>(p.as_bytes()) == c {
                k = i as i32 + 1;
                nv2 = 2;
                break;
            }
        }
        if k == 0 && ADDPFX == c {
            k = 449;
            nv2 = 2;
        }
    } else if islash == iz - 1 {
        // Add-on suffix
        let c1 = if islash as usize >= callsign0.len() {
            b' '
        } else {
            callsign0[islash as usize]
        };
        *callsign = pad(&callsign0[..(islash - 1).max(0) as usize]);
        for (i, &s) in SFX.iter().enumerate() {
            if s == c1 {
                k = 400 + i as i32 + 1;
                nv2 = 3;
                break;
            }
        }
    }

    // 10:
    if islash != 0 && k == 0 {
        let sl = islash as usize;
        let lof: [u8; 12] = pad(&callsign0[..sl - 1]);
        let rof: [u8; 12] = pad(&callsign0[sl..]);
        let llof = len_trim(&lof);
        let lrof = len_trim(&rof);
        let mut ispfx = llof > 0 && llof <= 4;
        let mut issfx = lrof > 0 && lrof <= 3;
        if lof[..llof].iter().any(|&c| c < b' ') {
            ispfx = false;
        }
        if rof[..lrof].iter().any(|&c| c < b' ') {
            issfx = false;
        }
        let invalid = !(ispfx || issfx);
        if ispfx && issfx {
            if llof < 3 {
                issfx = false;
            }
            if lrof < 3 {
                ispfx = false;
            }
            if ispfx && issfx {
                let c = callsign0[sl - 2];
                if is_digit(c) {
                    issfx = false;
                } else {
                    ispfx = false;
                }
            }
        }
        if invalid {
            k = -1;
            *callsign = callsign0;
        } else {
            if ispfx {
                k = nchar_f(lof[0]);
                k = 37 * k + nchar_f(lof[1]);
                k = 37 * k + nchar_f(lof[2]);
                k = 37 * k + nchar_f(lof[3]);
                nv2 = 4;
                *callsign = pad(&callsign0[sl..]);
            }
            if issfx {
                k = nchar_f(rof[0]);
                k = 37 * k + nchar_f(rof[1]);
                k = 37 * k + nchar_f(rof[2]);
                nv2 = 5;
                *callsign = pad(&callsign0[..sl - 1]);
            }
        }
    }
    (k, nv2)
}

/// `packjt.f90` `getpfx2` (`:~837-857`): put the prefix / suffix of table index `k0` on `callsign`.
fn getpfx2(k0: i32, callsign: &mut [u8; 12]) {
    let mut k = k0;
    if k > 450 {
        k -= 450;
    }
    if (1..=NZ).contains(&k) {
        let p = PFX[k as usize - 1].as_bytes();
        let mut v = p.to_vec();
        v.push(b'/');
        v.extend_from_slice(&callsign[..]);
        *callsign = pad(&v);
    } else if (401..=400 + NZ2).contains(&k) {
        let iz = (index_blank(callsign) as i32 - 1).max(0) as usize;
        let mut v = callsign[..iz].to_vec();
        v.push(b'/');
        v.push(SFX[(k - 401) as usize]);
        *callsign = pad(&v);
    } else if k == 449 {
        let mut iz = index_blank(&ADDPFX) as i32 - 1;
        if iz < 1 {
            iz = 8;
        }
        let mut v = ADDPFX[..iz as usize].to_vec();
        v.push(b'/');
        v.extend_from_slice(&callsign[..]);
        *callsign = pad(&v);
    }
}

/// `deg2grid.f90` on the whole-degree points `k2grid` gives it; the first four characters.
fn deg2grid4(dlong0: i32, dlat: i32) -> [u8; 4] {
    let mut dlong = dlong0;
    if dlong < -180 {
        dlong += 360;
    }
    if dlong > 180 {
        dlong -= 360;
    }
    // nlong = int(60.0*(180.0-dlong)/5.0) = 12*(180-dlong), exact
    let nlong = 12 * (180 - dlong);
    let n1 = nlong / 240;
    let n2 = (nlong - 240 * n1) / 24;
    // nlat = int(60.0*(dlat+90)/2.5) = 24*(dlat+90), exact
    let nlat = 24 * (dlat + 90);
    let m1 = nlat / 240;
    let m2 = (nlat - 240 * m1) / 24;
    [
        (b'A' as i32 + n1) as u8,
        (b'A' as i32 + m1) as u8,
        (b'0' as i32 + n2) as u8,
        (b'0' as i32 + m2) as u8,
    ]
}

/// `packjt.f90` `k2grid` (`:~877-889`).
fn k2grid(k: i32) -> [u8; 4] {
    let mut nlong = 2 * ((k - 1) / 5 % 90) - 179;
    if k > 450 {
        nlong += 180;
    }
    let nlat = (k - 1) % 5 + 85;
    deg2grid4(nlong, nlat)
}

/// Fortran `nint` of a value that is never within 1e-3 of a half.
fn nint(x: f32) -> i32 {
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        (x - 0.5) as i32
    }
}

/// `packjt.f90` `grid2k` (`:~859-875`) on `grid//'ma'` (via `grid2deg.f90`).
fn grid2k(grid: &[u8; 4]) -> i32 {
    let mut g1 = grid[0];
    let mut g2 = grid[1];
    if g1.is_ascii_lowercase() {
        g1 -= 32;
    }
    if g2.is_ascii_lowercase() {
        g2 -= 32;
    }
    let nlong0 = 180 - 20 * (g1 as i32 - b'A' as i32);
    let n20d = 2 * (grid[2] as i32 - b'0' as i32);
    let xminlong = 5.0f32 * ((b'm' - b'a') as f32 + 0.5);
    let xlong = (nlong0 - n20d) as f32 - xminlong / 60.0;
    let nlat0 = -90 + 10 * (g2 as i32 - b'A' as i32) + grid[3] as i32 - b'0' as i32;
    let xminlat = 2.5f32 * (0.5f32);
    let xlat = nlat0 as f32 + xminlat / 60.0;
    let nlong = nint(xlong);
    let nlat = nint(xlat);
    if nlat >= 85 {
        5 * (nlong + 179) / 2 + nlat - 84
    } else {
        0
    }
}

/// `packjt.f90` `packtext` (`:663-709`): 13 characters of the 42-character alphabet
/// in `(nc1, nc2, nc3)`.
fn packtext(msg: &[u8; 22]) -> (u32, u32, u32) {
    const C: &[u8; 42] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?";
    let code = |ch: u8| -> u32 { C.iter().position(|&x| x == ch).map_or(36, |p| p as u32) };
    let mut nc1 = 0u32;
    let mut nc2 = 0u32;
    let mut nc3 = 0u32;
    for &ch in &msg[0..5] {
        nc1 = 42 * nc1 + code(ch);
    }
    for &ch in &msg[5..10] {
        nc2 = 42 * nc2 + code(ch);
    }
    for &ch in &msg[10..13] {
        nc3 = 42 * nc3 + code(ch);
    }
    // We now have used 17 bits in nc3.  Must move one each to nc1 and nc2.
    nc1 += nc1;
    if nc3 & 32768 != 0 {
        nc1 += 1;
    }
    nc2 += nc2;
    if nc3 & 65536 != 0 {
        nc2 += 1;
    }
    (nc1, nc2, nc3 & 32767)
}

/// `packjt.f90` `unpacktext` (`:711-741`).
fn unpacktext(nc1a: u32, nc2a: u32, nc3a: u32) -> [u8; 22] {
    const C: &[u8; 42] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?";
    let mut msg = [b' '; 22];
    let mut nc1 = nc1a;
    let mut nc2 = nc2a;
    let mut nc3 = nc3a & 32767;
    if nc1 & 1 != 0 {
        nc3 += 32768;
    }
    nc1 /= 2;
    if nc2 & 1 != 0 {
        nc3 += 65536;
    }
    nc2 /= 2;
    for i in (0..5).rev() {
        msg[i] = C[(nc1 % 42) as usize];
        nc1 /= 42;
    }
    for i in (5..10).rev() {
        msg[i] = C[(nc2 % 42) as usize];
        nc2 /= 42;
    }
    for i in (10..13).rev() {
        msg[i] = C[(nc3 % 42) as usize];
        nc3 /= 42;
    }
    msg
}

/// `packjt.f90` `packmsg` (`:401-535`): `(nc1, nc2, ng, itype)`.
fn packmsg(msg: &mut [u8; 22]) -> (u32, u32, u32, i32) {
    fmtmsg(msg);
    if &msg[..3] == b"CQ " && is_digit(msg[3]) && msg[4] == b' ' {
        *msg = pad(&[b"CQ 00".as_slice(), &msg[3..]].concat());
    }
    if &msg[..6] == b"CQ DX " {
        msg[2] = b'9';
    }
    if &msg[..3] == b"CQ " && is_upper(msg[3]) && is_upper(msg[4]) && msg[5] == b' ' {
        *msg = pad(&[b"E9".as_slice(), &msg[3..]].concat());
    }

    // Free text: `go to 10`.
    let as_text = |msg: &[u8; 22]| -> (u32, u32, u32, i32) {
        let (nc1, nc2, nc3) = packtext(msg);
        (nc1, nc2, nc3 + 32768, 6)
    };

    // See if it's a CQ message; does it have a reply frequency?
    let ia: usize;
    if &msg[..3] == b"CQ " {
        let mut i = 3;
        if is_digit(msg[3]) && is_digit(msg[4]) && is_digit(msg[5]) {
            i = 7;
        }
        ia = i;
    } else {
        match msg.iter().position(|&c| c == b' ') {
            Some(p) => ia = p + 1,
            None => return as_text(msg),
        }
    }
    let mut c1: [u8; 12] = pad(&msg[..ia - 1]);
    // 2nd blank
    let Some(ib) = (ia + 1..=22).find(|&i| msg[i - 1] == b' ') else {
        return as_text(msg);
    };
    let mut c2: [u8; 12] = pad(&msg[ia..ib - 1]);
    // 3rd blank
    let Some(ic) = (ib + 1..=22).find(|&i| msg[i - 1] == b' ') else {
        return as_text(msg);
    };
    let mut c3: [u8; 4] = pad(&msg[ib..ic]);
    if &c3 == b"OOO " {
        c3 = *b"    ";
    }

    let (k1, nv2a) = getpfx1(&mut c1);
    if nv2a >= 4 {
        return as_text(msg);
    }
    let mut c1_6: [u8; 6] = c1[..6].try_into().unwrap();
    let (nc1, text1) = packcall(&mut c1_6);
    c1[..6].copy_from_slice(&c1_6);
    if text1 {
        return as_text(msg);
    }
    let (mut k2, nv2b) = getpfx1(&mut c2);
    let mut c2_6: [u8; 6] = c2[..6].try_into().unwrap();
    let (nc2, text2) = packcall(&mut c2_6);
    c2[..6].copy_from_slice(&c2_6);
    if text2 {
        return as_text(msg);
    }
    if nv2a == 2 || nv2a == 3 || nv2b == 2 || nv2b == 3 {
        if k1 < 0 || k2 < 0 || k1 * k2 != 0 {
            return as_text(msg);
        }
        if k2 > 0 {
            k2 += 450;
        }
        let k = k1.max(k2);
        if k > 0 {
            c3 = k2grid(k);
        }
    }
    let (ng, text3) = packgrid(&c3);

    if nv2a < 4 && nv2b < 4 && !text3 {
        return (nc1, nc2, ng, nv2a.max(nv2b));
    }

    let mut nc1 = 0i64;
    if nv2b == 4 {
        if &c1[..3] == b"CQ " && !text3 {
            nc1 = 262_178_563 + k2 as i64;
        }
        if &c1[..4] == b"QRZ " && !text3 {
            nc1 = 264_002_072 + k2 as i64;
        }
        if &c1[..3] == b"DE " && !text3 {
            nc1 = 265_825_581 + k2 as i64;
        }
    } else if nv2b == 5 {
        if &c1[..3] == b"CQ " && !text3 {
            nc1 = 267_649_090 + k2 as i64;
        }
        if &c1[..4] == b"QRZ " && !text3 {
            nc1 = 267_698_375 + k2 as i64;
        }
        if &c1[..3] == b"DE " && !text3 {
            nc1 = 267_747_660 + k2 as i64;
        }
    }
    if nc1 != 0 {
        return (nc1 as u32, nc2, ng, nv2a.max(nv2b));
    }
    // The message will be treated as plain text.
    as_text(msg)
}

/// `packjt.f90` `unpackgrid` (`:~278-318`): the 4-character field.
fn unpackgrid(ng: u32) -> [u8; 4] {
    let mut grid = [b' '; 4];
    if ng < 32400 {
        let dlat = (ng % 180) as i32 - 90;
        let dlong = (ng / 180) as i32 * 2 - 180 + 2;
        grid = deg2grid4(dlong, dlat);
        if &grid[..2] == b"KA" {
            let n = (grid[2] - b'0') as i32 * 10 + (grid[3] - b'0') as i32 - 50;
            // write(grid,'(i3.2)') n; a blank sign becomes '+'
            let s = if n < 0 { b'-' } else { b'+' };
            let a = n.unsigned_abs();
            grid = [s, b'0' + (a / 10) as u8, b'0' + (a % 10) as u8, b' '];
        } else if &grid[..2] == b"LA" {
            let n = (grid[2] - b'0') as i32 * 10 + (grid[3] - b'0') as i32 - 50;
            let s = if n < 0 { b'-' } else { b'+' };
            let a = n.unsigned_abs();
            grid = [b'R', s, b'0' + (a / 10) as u8, b'0' + (a % 10) as u8];
        }
        return grid;
    }
    let mut n = ng as i32 - NGBASE as i32 - 1;
    if (1..=30).contains(&n) {
        grid = [b'-', b'0' + (n / 10) as u8, b'0' + (n % 10) as u8, b' '];
    } else if (31..=60).contains(&n) {
        n -= 30;
        grid = [b'R', b'-', b'0' + (n / 10) as u8, b'0' + (n % 10) as u8];
    } else if n == 61 {
        grid = *b"RO  ";
    } else if n == 62 {
        grid = *b"RRR ";
    } else if n == 63 {
        grid = *b"73  ";
    }
    grid
}

/// What `unpackmsg` produced: its 22-character message and how it got there.
struct Unpacked {
    msg: [u8; 22],
    /// ng >= 32768
    text: bool,
    iv2: i32,
    k: i32,
    cqnnn: bool,
    /// the two calls and the grid, as the standard form assembles them
    c1: [u8; 12],
    c2: [u8; 12],
    grid: [u8; 4],
}

/// `packjt.f90` `unpackmsg` (`:537-661`).
fn unpackmsg(d: &[u8; 12]) -> Unpacked {
    let (nc1, nc2, ng) = unpack_words(d);
    let mut u = Unpacked {
        msg: [b' '; 22],
        text: false,
        iv2: 0,
        k: 0,
        cqnnn: false,
        c1: [b' '; 12],
        c2: [b' '; 12],
        grid: [b' '; 4],
    };
    if ng >= 32768 {
        u.text = true;
        u.msg = unpacktext(nc1, nc2, ng);
    } else {
        let (mut c1, iv2, psfx) = unpackcall(nc1);
        u.iv2 = iv2;
        if iv2 == 0 {
            // This is an "original JT65" message
            if nc1 == NBASE + 1 {
                c1 = pad(b"CQ    ");
            }
            if nc1 == NBASE + 2 {
                c1 = pad(b"QRZ   ");
            }
            if nc1 >= NBASE + 3 && nc1 - NBASE - 3 <= 999 {
                let nfreq = nc1 - NBASE - 3;
                c1 = pad(format!("CQ {nfreq:03}").as_bytes());
                u.cqnnn = true;
            }
        }
        let (mut c2, _, _) = unpackcall(nc2);
        let grid = unpackgrid(ng);
        u.grid = grid;
        if iv2 > 0 {
            // This is a JT65v2 message
            let ps: Vec<u8> = psfx.to_vec();
            let n1 = len_trim(&ps);
            let n2 = len_trim(&c2);
            let p = &ps[..n1];
            let c = &c2[..n2];
            let join = |parts: &[&[u8]]| -> Vec<u8> { parts.concat() };
            let v: Vec<u8> = match iv2 {
                1 => join(&[b"CQ ", p, b"/", c, b" ", &grid]),
                2 => join(&[b"QRZ ", p, b"/", c, b" ", &grid]),
                3 => join(&[b"DE ", p, b"/", c, b" ", &grid]),
                4 => join(&[b"CQ ", c, b"/", p, b" ", &grid]),
                5 => join(&[b"QRZ ", c, b"/", p, b" ", &grid]),
                6 => join(&[b"DE ", c, b"/", p, b" ", &grid]),
                _ => {
                    let k = grid2k(&grid);
                    if (451..=900).contains(&k) {
                        getpfx2(k, &mut c2);
                        let n2 = len_trim(&c2);
                        join(&[b"DE ", &c2[..n2]])
                    } else {
                        join(&[b"DE ", c, b" ", &grid])
                    }
                }
            };
            u.msg = pad(&v);
        } else {
            let k = grid2k(&grid);
            u.k = k;
            if (1..=450).contains(&k) {
                getpfx2(k, &mut c1);
            }
            if (451..=900).contains(&k) {
                getpfx2(k, &mut c2);
            }
            if let Some(p) = c1.iter().position(|&c| c == 0)
                && p + 1 >= 3
            {
                c1 = pad(&c1[..p]);
            }
            if let Some(p) = c2.iter().position(|&c| c == 0)
                && p + 1 >= 3
            {
                c2 = pad(&c2[..p]);
            }
            u.c1 = c1;
            u.c2 = c2;

            let mut msg = [b' '; 22];
            let mut j = 0usize;
            let mut skip_c1 = false;
            if u.cqnnn {
                msg[..12].copy_from_slice(&c1);
                j = 7;
                skip_c1 = true;
            }
            if !skip_c1 {
                let mut blank = false;
                for &ch in &c1 {
                    j += 1;
                    msg[j - 1] = ch;
                    if ch == b' ' {
                        blank = true;
                        break;
                    }
                }
                if !blank {
                    j += 1;
                    msg[j - 1] = b' ';
                }
            }
            // 10:
            let mut blank = false;
            for &ch in &c2 {
                if j <= 21 {
                    j += 1;
                }
                msg[j - 1] = ch;
                if ch == b' ' {
                    blank = true;
                    break;
                }
            }
            if !blank {
                if j <= 21 {
                    j += 1;
                }
                msg[j - 1] = b' ';
            }
            // 20:
            if k == 0 {
                for &ch in &grid {
                    if j <= 21 {
                        j += 1;
                    }
                    msg[j - 1] = ch;
                }
                if j <= 21 {
                    j += 1;
                }
                msg[j - 1] = b' ';
            }
            u.msg = msg;
        }
    }
    // 100:
    let m = &mut u.msg;
    if &m[..6] == b"CQ9DX " {
        m[2] = b' ';
    }
    if &m[..2] == b"E9" && is_upper(m[2]) && is_upper(m[3]) && m[4] == b' ' {
        *m = pad(&[b"CQ ".as_slice(), &m[2..]].concat());
    }
    if &m[..5] == b"CQ 00" && is_digit(m[5]) {
        *m = pad(&[b"CQ ".as_slice(), &m[5..]].concat());
    }
    u
}

fn bytes_to_string(b: &[u8]) -> String {
    b.iter().map(|&c| c as char).collect()
}

/// Port of WSJT-X `packmsg` (`lib/packjt.f90:401-535`, v3.2.0-rc1) with `packtext`
/// (`:663-709`), `packcall`, `packgrid`, `getpfx1` and `fmtmsg.f90`: pack a JT9 /
/// JT65 message into twelve 6-bit words.
///
/// The text is upper-cased, runs of blanks collapse, and anything past the 22nd
/// character is dropped, as for the `character*22` its callers pass. A standard
/// message (two calls and a grid / report, `CQ` / `QRZ` / `DE`, `CQ nnn`, `CQ DX`,
/// two-letter `CQ xx`), one with an add-on prefix or suffix, packs as such; any
/// other text falls back to free text (13 characters of `0-9 A-Z space + - . / ?`;
/// others become blanks, the rest is cut). The words after the grid are ignored when
/// the first three parse, as upstream does.
pub fn pack_message(text: &str) -> Result<Jt72Packed, Jt72PackError> {
    if !text.is_ascii() {
        return Err(Jt72PackError::NonAscii);
    }
    let mut msg: [u8; 22] = pad(text.as_bytes());
    let (nc1, nc2, ng, itype) = packmsg(&mut msg);
    Ok(Jt72Packed {
        words: pack_words(nc1, nc2, ng),
        itype,
    })
}

/// Port of WSJT-X `unpackmsg` (`lib/packjt.f90:537-661`, v3.2.0-rc1) with `unpacktext`
/// (`:711-741`), `unpackcall`, `unpackgrid`, `getpfx2` and `grid2k`: render twelve
/// 6-bit words exactly as `jt9` prints them (trailing blanks removed).
///
/// Changed from 0.13: every word pattern renders, so the result is no longer
/// `Unsupported` for free text and compound calls.
pub fn unpack(d: &[u8; 12]) -> Jt72Message {
    let u = unpackmsg(d);
    let text = bytes_to_string(&u.msg[..len_trim(&u.msg)]);
    if u.text {
        return Jt72Message::FreeText(text);
    }
    if u.iv2 == 0 && u.k == 0 {
        // The standard form: the calls and grid as the text assembles them.
        let first = |c: &[u8; 12]| -> String {
            let n = c.iter().position(|&x| x == b' ').unwrap_or(12);
            bytes_to_string(&c[..n])
        };
        let mut call1 = if u.cqnnn {
            bytes_to_string(&u.c1[..len_trim(&u.c1)])
        } else {
            first(&u.c1)
        };
        // The three text fix-ups of `unpackmsg` (:651-659) on the first word.
        let b = call1.as_bytes();
        if call1 == "CQ9DX" {
            call1 = "CQ DX".into();
        } else if b.len() == 4 && &b[..2] == b"E9" && is_upper(b[2]) && is_upper(b[3]) {
            call1 = format!("CQ {}", &call1[2..]);
        } else if b.len() == 6 && &b[..5] == b"CQ 00" && is_digit(b[5]) {
            call1 = format!("CQ {}", &call1[5..]);
        }
        let call2 = first(&u.c2);
        let grid = bytes_to_string(&u.grid[..len_trim(&u.grid)]);
        let cand = Jt72Message::Standard {
            call1,
            call2,
            grid_or_report: grid,
        };
        if cand.to_string() == text {
            return cand;
        }
    }
    Jt72Message::Compound(text)
}

// ─────────────────────────────────────────────────────────────────────────
// MessageCodec impl
// ─────────────────────────────────────────────────────────────────────────

use crate::engine::{DecodeContext, MessageCodec, MessageFields};

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
