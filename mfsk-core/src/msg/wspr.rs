//! WSPR 50-bit message codec.
//!
//! Ports `lib/wsprd/wsprd_utils.c` (unpack) and `lib/wsprd/wsprsim_utils.c`
//! (pack) from WSJT-X. The 50-bit payload carries one of three message
//! types:
//!
//! | Type | Contents                       | n1 (28 bit) | n2 (22 bit) |
//! |------|--------------------------------|-------------|-------------|
//! | 1    | 6-char callsign + grid4 + dBm  | packed call | grid+power  |
//! | 2    | prefix/suffix callsign + dBm   | packed call | prefix+type |
//! | 3    | hashed call + grid6 + dBm      | packed grid6| hash+type   |
//!
//! Type discrimination happens on the decode side: `ntype = (n2 & 127) - 64`.
//! Valid "power-in-dBm" values (0, 3, 7, 10, …, 60) mark Type 1; other
//! positive ntype is Type 2; negative ntype is Type 3.
//!
//! Currently Type 1 and Type 3 are implemented end-to-end. Type 2 is
//! detected but reported as a placeholder — the prefix/suffix unpack
//! logic can be ported verbatim when a test corpus materialises.
//!
//! The decoded representation is a `WsprMessage` enum so callers can
//! distinguish the types; the convenience `to_string()` impl yields the
//! familiar `"CALL GRID DBM"` tuple layout that WSPRnet expects.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

const POWERS: &[i32] = &[
    0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60,
];

/// Decoded WSPR message payload.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum WsprMessage {
    /// Standard Type-1 message: 6-char callsign, 4-char grid, transmit power.
    Type1 {
        callsign: String,
        grid: String,
        power_dbm: i32,
    },
    /// Type-2 prefix/suffix callsign (e.g. `PJ4/K1ABC 37`).
    Type2 {
        /// Fully reconstructed callsign with the prefix or suffix baked in
        /// (`"PJ4/K1ABC"`, `"K1ABC/7"`, etc).
        callsign: String,
        power_dbm: i32,
    },
    /// Type-3 hashed callsign + 6-char grid. The hash is exposed raw so
    /// callers with a compatible WSPR hash table can resolve it.
    Type3 {
        /// 15-bit callsign hash derived from `nhash(callsign, 146)` at TX.
        callsign_hash: u32,
        /// 6-character Maidenhead locator.
        grid6: String,
        power_dbm: i32,
    },
}

/// The power is two characters wide, as `wsprd` prints it (`wsprd_utils.c:353,
/// 364,398`: `%2d`), so 0, 3 and 7 dBm carry a leading space: `K1ABC FN42  3`.
impl fmt::Display for WsprMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WsprMessage::Type1 {
                callsign,
                grid,
                power_dbm,
            } => write!(f, "{} {} {:>2}", callsign, grid, power_dbm),
            WsprMessage::Type2 {
                callsign,
                power_dbm,
            } => write!(f, "{} {:>2}", callsign, power_dbm),
            WsprMessage::Type3 {
                callsign_hash,
                grid6,
                power_dbm,
            } => write!(f, "<#{:05x}> {} {:>2}", callsign_hash, grid6, power_dbm),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Character tables
// ─────────────────────────────────────────────────────────────────────────

/// 37-entry table used by callsign/grid unpacking — digits, uppercase
/// letters, and space. Matches `c[]` in `wsprd_utils.c::unpackcall`.
const CHAR37: &[u8; 37] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";

fn callsign_char_code(ch: u8) -> Option<u8> {
    match ch {
        b'0'..=b'9' => Some(ch - b'0'),
        b'A'..=b'Z' => Some(ch - b'A' + 10),
        b' ' => Some(36),
        _ => None,
    }
}

fn locator_char_code(ch: u8) -> Option<u8> {
    match ch {
        b'0'..=b'9' => Some(ch - b'0'),
        b'A'..=b'R' => Some(ch - b'A'),
        b' ' => Some(36),
        _ => None,
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Pack / unpack 50 bits ↔ (n1, n2)
// ─────────────────────────────────────────────────────────────────────────

/// Pack (n1, n2) into 50 bits laid out across 7 bytes + 1 bit.
/// Byte layout matches `wsprsim_utils.c`:
/// ```text
/// data[0]=n1[27..20]  data[1]=n1[19..12]  data[2]=n1[11..4]
/// data[3]=n1[3..0]<<4 | n2[21..18]
/// data[4]=n2[17..10]  data[5]=n2[9..2]    data[6]=n2[1..0]<<6
/// ```
pub fn pack50(n1: u32, n2: u32) -> [u8; 7] {
    [
        ((n1 >> 20) & 0xff) as u8,
        ((n1 >> 12) & 0xff) as u8,
        ((n1 >> 4) & 0xff) as u8,
        (((n1 & 0x0f) << 4) | ((n2 >> 18) & 0x0f)) as u8,
        ((n2 >> 10) & 0xff) as u8,
        ((n2 >> 2) & 0xff) as u8,
        (((n2 & 0x03) << 6) & 0xff) as u8,
    ]
}

/// Inverse of [`pack50`]: 7-byte packed word → (n1, n2).
/// Tolerates the 7th byte carrying only the top 2 bits.
pub fn unpack50(data: &[u8; 7]) -> (u32, u32) {
    let mut n1: u32 = (data[0] as u32) << 20;
    n1 |= (data[1] as u32) << 12;
    n1 |= (data[2] as u32) << 4;
    n1 |= ((data[3] >> 4) & 0x0f) as u32;

    let mut n2: u32 = ((data[3] & 0x0f) as u32) << 18;
    n2 |= (data[4] as u32) << 10;
    n2 |= (data[5] as u32) << 2;
    n2 |= ((data[6] >> 6) & 0x03) as u32;

    (n1, n2)
}

// ─────────────────────────────────────────────────────────────────────────
// Callsign + grid packing (Type 1)
// ─────────────────────────────────────────────────────────────────────────

/// Encode a callsign into a 28-bit integer. Returns `None` if the callsign
/// doesn't fit the compressed form (must be ≤ 6 chars with a digit in
/// position 1 or 2, and only A-Z / 0-9 / space).
///
/// Thin wrapper over [`crate::msg::callsign28::pack_call28`] — WSPR
/// shares this encoding with JT9/JT65 (`crate::msg::jt72`), both
/// tracing to WSJT-X's `packjt.f90::packcall` (called from
/// `wsprcode/wspr_old_subs.f90` too, not just the JT65/JT9 message
/// layer). WSPR has no CQ/QRZ/DE special-token layer (Type-1 WSPR
/// messages are always callsign+grid+power), so there's nothing to
/// check above the shared core here, unlike [`crate::msg::jt72::pack_call`].
pub fn pack_call(callsign: &str) -> Option<u32> {
    crate::msg::callsign28::pack_call28(callsign, |c| callsign_char_code(c).map(u32::from))
}

/// Unpack a 28-bit callsign integer. Returns `None` for the "reserved"
/// range (≥ 262_177_560) that WSJT-X treats as non-Type-1.
pub fn unpack_call(ncall: u32) -> Option<String> {
    crate::msg::callsign28::unpack_call28(ncall)
}

/// Pack a 4-char grid and transmit power into a 22-bit integer.
pub fn pack_grid4_power(grid: &str, power_dbm: i32) -> Option<u32> {
    let bytes = grid.as_bytes();
    if bytes.len() != 4 {
        return None;
    }
    let g0 = locator_char_code(bytes[0])? as u32;
    let g1 = locator_char_code(bytes[1])? as u32;
    let g2 = locator_char_code(bytes[2])? as u32;
    let g3 = locator_char_code(bytes[3])? as u32;
    let m = (179 - 10 * g0 - g2) * 180 + 10 * g1 + g3;
    Some(m * 128 + (power_dbm as u32) + 64)
}

/// Unpack the 22-bit grid+power integer. Returns `(grid, ntype)` where
/// `ntype = (n2 & 127) - 64` — the caller decides whether `ntype` names a
/// Type 1 dBm value, a Type 2 suffix count, or a Type 3 negative tag.
pub fn unpack_grid(ngrid_full: u32) -> Option<(String, i32)> {
    let ntype = (ngrid_full & 127) as i32 - 64;
    let ngrid = ngrid_full >> 7;
    if ngrid >= 32_400 {
        return None;
    }
    let dlat = (ngrid % 180) as i32 - 90;
    let mut dlong = (ngrid / 180) as i32 * 2 - 180 + 2;
    if dlong < -180 {
        dlong += 360;
    }
    if dlong > 180 {
        dlong += 360;
    }
    let nlong = (60.0 * (180.0 - dlong as f32) / 5.0) as i32;
    let ln1 = nlong / 240;
    let ln2 = (nlong - 240 * ln1) / 24;

    let nlat = (60.0 * (dlat + 90) as f32 / 2.5) as i32;
    let la1 = nlat / 240;
    let la2 = (nlat - 240 * la1) / 24;

    let mut grid = [b'0'; 4];
    grid[0] = CHAR37[(10 + ln1) as usize];
    grid[2] = CHAR37[ln2 as usize];
    grid[1] = CHAR37[(10 + la1) as usize];
    grid[3] = CHAR37[la2 as usize];
    Some((core::str::from_utf8(&grid).ok()?.to_string(), ntype))
}

// ─────────────────────────────────────────────────────────────────────────
// Public encode / decode entry points
// ─────────────────────────────────────────────────────────────────────────

/// Pack a Type-1 WSPR message (callsign + 4-char grid + power in dBm) into
/// 50 bits, stored MSB-first across a 50-element `[u8; 50]` of 0/1 values —
/// the form required by [`crate::fec::ConvFano`]'s encode path.
pub fn pack_type1(callsign: &str, grid: &str, power_dbm: i32) -> Option<[u8; 50]> {
    if !POWERS.contains(&power_dbm) {
        return None;
    }
    let n1 = pack_call(callsign)?;
    let n2 = pack_grid4_power(grid, power_dbm)?;
    let bytes = pack50(n1, n2);
    let mut bits = [0u8; 50];
    for i in 0..50 {
        let byte = bytes[i / 8];
        bits[i] = (byte >> (7 - (i % 8))) & 1;
    }
    Some(bits)
}

/// Add a prefix or suffix to a callsign according to the 16-bit
/// `nprefix` field carried in Type-2 messages. Ports
/// `wsprd_utils.c::unpackpfx`.
///
/// * `nprefix < 60000` → prefix of 1-3 chars, packed base-37
/// * `60000 ≤ nprefix ≤ 60035` → single-char digit/letter suffix
/// * `60036 ≤ nprefix ≤ 60125` → two-digit suffix
fn apply_prefix(nprefix: u32, base_call: &str) -> Option<String> {
    const A37: &[u8] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";
    if nprefix < 60_000 {
        // Prefix, 1-3 chars.
        let mut n = nprefix;
        let mut pfx = [b' '; 3];
        for i in (0..3).rev() {
            let nc = (n % 37) as usize;
            pfx[i] = A37[nc];
            n /= 37;
        }
        // Strip leading spaces.
        let start = pfx.iter().position(|&b| b != b' ')?;
        let pfx_str = core::str::from_utf8(&pfx[start..]).ok()?;
        Some(format!("{}/{}", pfx_str, base_call))
    } else {
        let nc = nprefix - 60_000;
        if nc <= 9 {
            Some(format!("{}/{}", base_call, (b'0' + nc as u8) as char))
        } else if nc <= 35 {
            Some(format!(
                "{}/{}",
                base_call,
                (b'A' + (nc - 10) as u8) as char
            ))
        } else if nc <= 125 {
            let d1 = (nc - 26) / 10;
            let d2 = (nc - 26) % 10;
            Some(format!(
                "{}/{}{}",
                base_call,
                (b'0' + d1 as u8) as char,
                (b'0' + d2 as u8) as char
            ))
        } else {
            None
        }
    }
}

/// Unpack 50 info bits into a [`WsprMessage`]. Returns `None` for
/// pathological ntype/ngrid combinations.
pub fn unpack(bits: &[u8; 50]) -> Option<WsprMessage> {
    // Pack bit vector back into the 7-byte word format unpack50 expects.
    let mut data = [0u8; 7];
    for i in 0..50 {
        if bits[i] & 1 != 0 {
            data[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    let (n1, n2) = unpack50(&data);

    let (maybe_grid, ntype) = unpack_grid(n2).unzip();

    // Type 3: negative ntype → hashed callsign + grid6.
    // The 6-char grid is stored via pack_call with a rotated layout:
    // grid6[..5] holds the last 5 chars of the grid, grid6[5] holds the
    // first. We recover the packed string via unpack_call then rotate
    // the tail char back to the front.
    if let Some(t) = ntype
        && t < 0
    {
        let power_dbm = -(t + 1);
        // Reconstruct grid6 from the "callsign-slot" encoding.
        let pseudo_call = unpack_call(n1).unwrap_or_default();
        let mut grid6 = String::new();
        if pseudo_call.len() == 6 {
            let bytes = pseudo_call.as_bytes();
            grid6.push(bytes[5] as char); // rotated-back first char
            grid6.push_str(core::str::from_utf8(&bytes[..5]).ok()?);
        }
        // Hash extraction: ihash = (n2 - ntype - 64) / 128. Since
        // ntype is negative, this is (n2 + (-ntype) - 64) / 128; with
        // n2 raw, it equals n2 >> 7 exactly.
        let hash = n2 >> 7;
        // `unpk_` (`wsprd_utils.c:372-404`) prints a Type 3 only if its power
        // is one WSPR sends (0, 3, 7 mod 10), its locator starts with two
        // letters and two digits, and it is not `ntype == -64` — which "show
        // up as A000AA grids" and are what Fano's all-zero path decodes to.
        let nu = power_dbm % 10;
        let b = grid6.as_bytes();
        let sane = matches!(nu, 0 | 3 | 7)
            && b.len() >= 4
            && b[0].is_ascii_alphabetic()
            && b[1].is_ascii_alphabetic()
            && b[2].is_ascii_digit()
            && b[3].is_ascii_digit()
            && t != -64;
        if !sane {
            return None;
        }
        return Some(WsprMessage::Type3 {
            callsign_hash: hash,
            grid6,
            power_dbm,
        });
    }

    let ntype_val = ntype?;
    let grid = maybe_grid?;

    // Type 1 test: nu = ntype % 10 ∈ {0,3,7} AND ntype ≤ 62.
    if (0..=62).contains(&ntype_val) {
        let nu = ntype_val % 10;
        if nu == 0 || nu == 3 || nu == 7 {
            let callsign = unpack_call(n1)?;
            return Some(WsprMessage::Type1 {
                callsign,
                grid,
                power_dbm: ntype_val,
            });
        }
        // Type 2: positive ntype but power-digit not in {0,3,7}.
        // nadd encodes "this is a compound call" — recover by
        //   n3 = n2 / 128 + 32768 * (nadd - 1)
        //   actual_dbm = ntype - nadd
        let nadd = if nu > 7 {
            nu - 7
        } else if nu > 3 {
            nu - 3
        } else {
            nu
        };
        let n3 = (n2 >> 7) + 32_768 * (nadd as u32 - 1);
        let base_call = unpack_call(n1)?;
        let full_call = apply_prefix(n3, &base_call)?;
        let power_dbm = ntype_val - nadd;
        // Plausibility: the recovered power digit must land on {0,3,7,10}.
        let pu = power_dbm.rem_euclid(10);
        if pu != 0 && pu != 3 && pu != 7 {
            return None;
        }
        return Some(WsprMessage::Type2 {
            callsign: full_call,
            power_dbm,
        });
    }

    None
}

// ─────────────────────────────────────────────────────────────────────────
// pack_message: wqencode (types 1, 2 and 3)
// ─────────────────────────────────────────────────────────────────────────

/// What [`pack_message`] returns: `wqencode`'s `data0` and `ntype`.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct WsprPacked {
    /// The 50 information bits, one per element, MSB first (the layout of
    /// [`pack_type1`]).
    pub bits: [u8; 50],
    /// `wqencode`'s `ntype`: the rounded power (0..=60) for type 1,
    /// `power + 1 + nadd` for type 2, `-(power + 1)` for type 3.
    pub ntype: i32,
    /// Type 3 only: the call between `<` and `>`, as typed (what `wqencode`
    /// hashes; not one of its outputs, the encoder needs it).
    pub hashed_call: Option<String>,
}

/// Why [`pack_message`] refused a text. Each variant is a way `wqencode`
/// (`lib/wqencode.f90`, rc1) fails: it returns without writing `data0`,
/// aborts with a Fortran runtime error, or reads memory it never wrote (see
/// the divergence note on [`pack_message`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum WsprPackError {
    /// No form of `wqencode` takes the text (not `CALL GRID4 DBM`,
    /// `PFX/CALL DBM`, `CALL/SFX DBM` nor `<CALL> GRID6 DBM`); upstream
    /// returns with nothing written.
    NotWsprMessage,
    /// The call does not pack (`packcall`).
    BadCall,
    /// The 4-character grid (type 1) or the 6-character grid (type 3) does
    /// not pack.
    BadGrid,
    /// The power is missing or is not a list-directed integer; upstream's
    /// `read(...,*) ndbm` aborts.
    BadPower,
    /// A `<` at the start but no closing `>`.
    UnclosedHashedCall,
    /// A call longer than `wqencode`'s 12-character buffers (a `<CALL>` of
    /// more than 12 characters, a `/` at position 11 or later of the first
    /// word): upstream reads past the buffer.
    CallTooLong,
}

impl core::fmt::Display for WsprPackError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::NotWsprMessage => {
                "not a WSPR message (CALL GRID4 DBM, PFX/CALL DBM, CALL/SFX DBM or <CALL> GRID6 DBM)"
            }
            Self::BadCall => "the callsign cannot be packed",
            Self::BadGrid => "the grid locator cannot be packed",
            Self::BadPower => "the power is missing or not an integer",
            Self::UnclosedHashedCall => "a '<' without a closing '>'",
            Self::CallTooLong => "a callsign longer than WSPR's 12-character buffer",
        })
    }
}

impl core::error::Error for WsprPackError {}

/// `nhash(key, len, 146) & 0x7fff` as `lib/hash.f90` calls it
/// (`lib/wsprd/nhash.c`, `lib/lookup3.c:160` `wsjt_hashlittle`, Jenkins lookup3).
fn hash15(key: &[u8]) -> u32 {
    fn rot(x: u32, k: u32) -> u32 {
        x.rotate_left(k)
    }
    let mut a = 0xdead_beefu32
        .wrapping_add(key.len() as u32)
        .wrapping_add(146);
    let mut b = a;
    let mut c = a;
    let mut k = key;
    while k.len() > 12 {
        let w = |i: usize| u32::from_le_bytes([k[i], k[i + 1], k[i + 2], k[i + 3]]);
        a = a.wrapping_add(w(0));
        b = b.wrapping_add(w(4));
        c = c.wrapping_add(w(8));
        a = a.wrapping_sub(c);
        a ^= rot(c, 4);
        c = c.wrapping_add(b);
        b = b.wrapping_sub(a);
        b ^= rot(a, 6);
        a = a.wrapping_add(c);
        c = c.wrapping_sub(b);
        c ^= rot(b, 8);
        b = b.wrapping_add(a);
        a = a.wrapping_sub(c);
        a ^= rot(c, 16);
        c = c.wrapping_add(b);
        b = b.wrapping_sub(a);
        b ^= rot(a, 19);
        a = a.wrapping_add(c);
        c = c.wrapping_sub(b);
        c ^= rot(b, 4);
        b = b.wrapping_add(a);
        k = &k[12..];
    }
    if k.is_empty() {
        return c & 0x7fff;
    }
    for (i, &byte) in k.iter().enumerate() {
        let v = u32::from(byte) << (8 * (i % 4));
        match i / 4 {
            0 => a = a.wrapping_add(v),
            1 => b = b.wrapping_add(v),
            _ => c = c.wrapping_add(v),
        }
    }
    c ^= b;
    c = c.wrapping_sub(rot(b, 14));
    a ^= c;
    a = a.wrapping_sub(rot(c, 11));
    b ^= a;
    b = b.wrapping_sub(rot(a, 25));
    c ^= b;
    c = c.wrapping_sub(rot(b, 16));
    a ^= c;
    a = a.wrapping_sub(rot(c, 4));
    b ^= a;
    b = b.wrapping_sub(rot(a, 14));
    c ^= b;
    c = c.wrapping_sub(rot(b, 24));
    c & 0x7fff
}

/// Result of a Fortran list-directed `read(str,*) integer`.
enum ListInt {
    Value(i32),
    /// A null value (`,` or `/` first): the variable keeps its value.
    Null,
    /// End of file or a bad item: the read fails.
    Error,
}

/// The subset of gfortran's list-directed integer input the WSPR parsers
/// reach: blanks and tabs skipped, one item ended by a separator, `r*c`
/// repeat counts and null values (checked against the rc1 oracle in
/// `tests/wspr_pack_message.rs`).
fn list_read_int(s: &[u8]) -> ListInt {
    let sep = |c: u8| matches!(c, b' ' | b'\t' | b',' | b'/' | b'\r' | b'\n');
    let mut i = 0;
    while i < s.len() && matches!(s[i], b' ' | b'\t') {
        i += 1;
    }
    if i == s.len() {
        return ListInt::Error;
    }
    if s[i] == b',' || s[i] == b'/' {
        return ListInt::Null;
    }
    let mut j = i;
    let neg = s[j] == b'-';
    let signed = neg || s[j] == b'+';
    if signed {
        j += 1;
    }
    let mut v: i64 = 0;
    let d0 = j;
    while j < s.len() && s[j].is_ascii_digit() {
        v = (v * 10 + i64::from(s[j] - b'0')).min(1 << 40);
        j += 1;
    }
    if j == d0 {
        return ListInt::Error;
    }
    if j < s.len() && s[j] == b'*' && !signed {
        // r*c: `v` copies of the value after the star; r must be positive.
        if v == 0 || v > i64::from(i32::MAX) {
            return ListInt::Error;
        }
        j += 1;
        if j == s.len() || sep(s[j]) {
            return ListInt::Null;
        }
        let neg2 = s[j] == b'-';
        if neg2 || s[j] == b'+' {
            j += 1;
        }
        let d1 = j;
        let mut w: i64 = 0;
        while j < s.len() && s[j].is_ascii_digit() {
            w = (w * 10 + i64::from(s[j] - b'0')).min(1 << 40);
            j += 1;
        }
        if j == d1 || (j < s.len() && !sep(s[j])) {
            return ListInt::Error;
        }
        v = if neg2 { -w } else { w };
    } else {
        if j < s.len() && !sep(s[j]) {
            return ListInt::Error;
        }
        if neg {
            v = -v;
        }
    }
    match i32::try_from(v) {
        Ok(x) => ListInt::Value(x),
        Err(_) => ListInt::Error,
    }
}

const NGBASE: u32 = 180 * 180;

/// `packcall` (`lib/packjt.f90:54-136`), for a 6-character field. `None` is
/// upstream's `text = .true.`.
///
/// Ported from lib/packjt.f90:54-136 (rc1). The `CQ nnn` frequency form
/// (`:73-77`) cannot arise here: the field has no blank inside.
fn packcall(field: &[u8; 6]) -> Option<u32> {
    let mut c = *field;
    // Swaziland, Guinea work-arounds (:61-65).
    if &c[..4] == b"3DA0" {
        c = [b'3', b'D', b'0', c[4], c[5], b' '];
    }
    if &c[..2] == b"3X" && c[2].is_ascii_uppercase() {
        c = [b'Q', c[2], c[3], c[4], c[5], b' '];
    }
    if &c[..3] == b"CQ " {
        return Some(crate::msg::callsign28::NBASE + 1);
    }
    if &c[..4] == b"QRZ " {
        return Some(crate::msg::callsign28::NBASE + 2);
    }
    if &c[..3] == b"DE " {
        return Some(267_796_945);
    }
    let mut t = c;
    t.make_ascii_uppercase(); // :90-93 (after the digit-position tests, which are case blind)
    let end = t.iter().rposition(|&b| b != b' ').map_or(0, |p| p + 1);
    if !t.is_ascii() {
        return None;
    }
    let s = core::str::from_utf8(&t[..end]).ok()?;
    pack_call(s)
}

/// `packgrid` (`lib/packjt.f90:279-354`) for a 4-character grid; `None` is
/// `text = .true.`.
///
/// Ported from lib/packjt.f90:279-354 (rc1).
fn packgrid(grid: &[u8; 4]) -> Option<u32> {
    if grid == b"    " {
        return Some(NGBASE + 1);
    }
    let in_range = |n: i32| (1..=30).contains(&n);
    if grid[0] == b'-' {
        match list_read_int(&grid[1..3]) {
            ListInt::Error => return None,
            ListInt::Value(n) if in_range(n) => return Some(NGBASE + 1 + n as u32),
            ListInt::Null | ListInt::Value(_) => {}
        }
    } else if &grid[..2] == b"R-" {
        match list_read_int(&grid[2..4]) {
            ListInt::Error => return None,
            ListInt::Value(n) if in_range(n) => return Some(NGBASE + 31 + n as u32),
            ListInt::Null | ListInt::Value(_) => {}
        }
    } else if grid == b"RO  " {
        return Some(NGBASE + 62);
    } else if grid == b"RRR " {
        return Some(NGBASE + 63);
    } else if grid == b"73  " {
        return Some(NGBASE + 64);
    }
    // Extended-range reports (:75-95): -50..-31 and 0..+49 become KAnn / LAnn.
    let mut n = 99;
    match list_read_int(grid) {
        ListInt::Value(v) => n = v,
        ListInt::Null => {}
        ListInt::Error => {
            if let ListInt::Value(v) = list_read_int(&grid[1..4]) {
                n = v;
            }
        }
    }
    let mut g = *grid;
    if (-50..=49).contains(&n) {
        let k = (n + 50) as u8;
        g = [
            if grid[0] == b'R' { b'L' } else { b'K' },
            b'A',
            b'0' + k / 10,
            b'0' + k % 10,
        ];
    }
    let ok = (b'A'..=b'R').contains(&g[0])
        && (b'A'..=b'R').contains(&g[1])
        && g[2].is_ascii_digit()
        && g[3].is_ascii_digit();
    if !ok {
        return None;
    }
    // `grid2deg(grid//'mm55')`, `ng = ((long+180)/2)*180 + lat` is the `m` of pack_grid4_power.
    let n2 = pack_grid4_power(core::str::from_utf8(&g).ok()?, 0)?;
    Some((n2 - 64) / 128)
}

/// 1-based `index(s, c)`; 0 when absent.
fn findex(s: &[u8], c: u8) -> usize {
    s.iter().position(|&b| b == c).map_or(0, |p| p + 1)
}

/// Fortran assignment to `character*12`: truncate, blank pad.
fn field12(s: &[u8]) -> [u8; 12] {
    let mut f = [b' '; 12];
    for (d, &b) in f.iter_mut().zip(s) {
        *d = b;
    }
    f
}

fn first6(s: &[u8; 12]) -> [u8; 6] {
    let mut f = [b' '; 6];
    f.copy_from_slice(&s[..6]);
    f
}

/// `msg(a:b)` with 1-based bounds; empty when `a > b`, `Err` when outside the buffer
/// (upstream reads past it).
fn sub(m: &[u8], a: isize, b: isize) -> Result<&[u8], WsprPackError> {
    if a > b {
        return Ok(&[]);
    }
    if a < 1 || b as usize > m.len() {
        return Err(WsprPackError::CallTooLong);
    }
    Ok(&m[a as usize - 1..b as usize])
}

/// `ndbm` of `read(str,*) ndbm`, clamped to 0..=60 and rounded with `nu`
/// (`lib/wqencode.f90:25-29`).
fn read_power(s: &[u8]) -> Result<i32, WsprPackError> {
    const NU: [i32; 10] = [0, -1, 1, 0, -1, 2, 1, 0, -1, 1];
    let n = match list_read_int(s) {
        ListInt::Value(v) => v,
        ListInt::Null => 0,
        ListInt::Error => return Err(WsprPackError::BadPower),
    };
    let n = n.clamp(0, 60);
    Ok(n + NU[(n % 10) as usize])
}

/// `packpfx` (`lib/packjt.f90:979-1037`): `(n1, ng, nadd)`; `n1` is `None`
/// where upstream leaves it unset.
fn packpfx(call1: &[u8; 12]) -> Result<(Option<u32>, i32, i32), WsprPackError> {
    let i1 = findex(call1, b'/');
    let at = |k: usize| -> Result<u8, WsprPackError> {
        call1.get(k - 1).copied().ok_or(WsprPackError::CallTooLong)
    };
    let head = &call1[..i1.saturating_sub(1)];
    if at(i1 + 2)? == b' ' {
        // Single-character add-on suffix (:988-1004).
        let n1 = packcall(&first6(&field12(head)));
        let nc = i32::from(at(i1 + 1)?);
        let n = if (48..=57).contains(&nc) {
            nc - 48
        } else if (65..=90).contains(&nc) {
            nc - 65 + 10
        } else {
            38
        };
        Ok((n1, 60000 - 32768 + n, 1))
    } else if at(i1 + 3)? == b' ' {
        // Two-character numerical suffix, /10 to /99 (:1005-1012).
        let n1 = packcall(&first6(&field12(head)));
        let n = 10 * (i32::from(at(i1 + 1)?) - 48) + i32::from(at(i1 + 2)?) - 48;
        Ok((n1, 60000 + 26 + n, 1))
    } else {
        // Prefix of 1 to 3 characters (:1013-1035).
        let mut pfx = [b' '; 3];
        for (d, &b) in pfx.iter_mut().zip(head) {
            *d = b;
        }
        for _ in 0..2 {
            if pfx[2] == b' ' {
                pfx = [b' ', pfx[0], pfx[1]];
            }
        }
        let call0 = field12(&call1[i1..]);
        let n1 = packcall(&first6(&call0));
        let mut ng = 0i32;
        for &c in &pfx {
            let nc = i32::from(c);
            let n = if (48..=57).contains(&nc) {
                nc - 48
            } else if (65..=90).contains(&nc) {
                nc - 65 + 10
            } else {
                36
            };
            ng = 37 * ng + n;
        }
        let mut nadd = 0;
        if ng >= 32768 {
            ng -= 32768;
            nadd = 1;
        }
        Ok((n1, ng, nadd))
    }
}

fn bits_of(n1: u32, n2: u32) -> [u8; 50] {
    let bytes = pack50(n1, n2);
    let mut bits = [0u8; 50];
    for (i, b) in bits.iter_mut().enumerate() {
        *b = (bytes[i / 8] >> (7 - (i % 8))) & 1;
    }
    bits
}

/// Pack a WSPR message text, as WSJT-X's `wqencode` does.
///
/// Forms (all on the first 22 bytes of `text`, blank padded, no case
/// folding: the caller upper-cases, as WSJT-X's GUI does):
///
/// - type 1, `CALL GRID4 DBM`: `ntype` is the power;
/// - type 2, `PFX/CALL DBM` or `CALL/SFX DBM`: `ntype = power + 1 + nadd`;
/// - type 3, `<CALL> GRID6 DBM`: the call is sent as a 15-bit hash,
///   `ntype = -(power + 1)`.
///
/// The power is read as Fortran list-directed input, clamped to 0..=60 and
/// rounded to the nearest power WSPR sends (`nu` table): a power such as 38
/// is sent as 37, not refused.
///
/// Ported from lib/wqencode.f90:15-61 (rc1; unchanged in v3.3.0-beta2),
/// with `packcall` (lib/packjt.f90:54-136), `packgrid` (:279-354), `packpfx`
/// (:979-1037), `pack50` (:953-977), `hash` (lib/hash.f90) over
/// `wsjt_hashlittle` (lib/lookup3.c:160).
///
/// **Divergence (deliberate).** Upstream ignores the `lbad`/`text` flag of
/// `packpfx` and `packcall` in the type 2 and 3 branches and packs an unset
/// `n1`, and hashes `len` bytes of a 12-byte buffer. These return
/// [`WsprPackError::BadCall`], [`WsprPackError::BadGrid`] and
/// [`WsprPackError::CallTooLong`] here instead of undefined bits.
pub fn pack_message(text: &str) -> Result<WsprPacked, WsprPackError> {
    let mut msg = [b' '; 22];
    for (d, &s) in msg.iter_mut().zip(text.as_bytes()) {
        *d = s;
    }
    let i1 = findex(&msg, b' ');
    let i2 = findex(&msg, b'/');
    let i3 = findex(&msg, b'<');
    let ii1 = i1 as isize;
    let call1 = field12(sub(&msg, 1, ii1 - 1)?);

    // Standard WSPR message (wqencode.f90:16-33).
    if (3..=7).contains(&i1) && i2 == 0 && i3 == 0 {
        let mut grid4 = [b' '; 4];
        grid4.copy_from_slice(sub(&msg, ii1 + 1, ii1 + 4)?);
        let n1 = packcall(&first6(&call1)).ok_or(WsprPackError::BadCall)?;
        let ng = packgrid(&grid4).ok_or(WsprPackError::BadGrid)?;
        let ndbm = read_power(sub(&msg, ii1 + 5, 22)?)?;
        let n2 = 128 * ng + (ndbm + 64) as u32;
        return Ok(WsprPacked {
            bits: bits_of(n1, n2),
            ntype: ndbm,
            hashed_call: None,
        });
    }

    if i2 >= 2 && i3 < 1 {
        // Type 2 (:35-44).
        let (n1, ng, nadd) = packpfx(&call1)?;
        let ndbm = read_power(sub(&msg, ii1 + 1, 22)?)?;
        let n1 = n1.ok_or(WsprPackError::BadCall)?;
        let ntype = ndbm + 1 + nadd;
        let n2 = (128 * ng + ntype + 64) as u32;
        Ok(WsprPacked {
            bits: bits_of(n1, n2),
            ntype,
            hashed_call: None,
        })
    } else if i3 == 1 {
        // Type 3 (:45-60).
        let i4 = findex(&msg, b'>');
        if i4 == 0 {
            return Err(WsprPackError::UnclosedHashedCall);
        }
        let hl = i4 - 2;
        if hl > 12 {
            return Err(WsprPackError::CallTooLong);
        }
        let call = field12(sub(&msg, 2, i4 as isize - 1)?);
        let ih = hash15(&call[..hl]);
        let tail = sub(&msg, ii1 + 1, 22)?;
        let tail = &tail[..tail.iter().rposition(|&b| b != b' ').map_or(0, |p| p + 1)];
        let i5 = findex(tail, b' ') as isize;
        let mut c2 = Vec::new();
        c2.extend_from_slice(sub(&msg, ii1 + 2, ii1 + i5 - 1)?);
        c2.extend_from_slice(sub(&msg, ii1 + 1, ii1 + 1)?);
        let n1 = packcall(&first6(&field12(&c2)));
        let ndbm = read_power(sub(&msg, ii1 + i5 + 1, 22)?)?;
        let n1 = n1.ok_or(WsprPackError::BadGrid)?;
        let ntype = -(ndbm + 1);
        let n2 = (128 * ih as i32 + ntype + 64) as u32;
        Ok(WsprPacked {
            bits: bits_of(n1, n2),
            ntype,
            hashed_call: Some(String::from_utf8_lossy(&call[..hl]).into_owned()),
        })
    } else {
        Err(WsprPackError::NotWsprMessage)
    }
}

// ─────────────────────────────────────────────────────────────────────────
// MessageCodec trait impl
// ─────────────────────────────────────────────────────────────────────────

use crate::engine::{DecodeContext, MessageCodec, MessageFields};

#[derive(Copy, Clone, Debug, Default)]
pub struct Wspr50Message;

impl MessageCodec for Wspr50Message {
    type Unpacked = WsprMessage;
    const PAYLOAD_BITS: u32 = 50;
    const CRC_BITS: u32 = 0;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        let call = fields.call1.as_deref()?;
        let grid = fields.grid.as_deref()?;
        let power = fields.report?; // re-using MessageFields.report for dBm
        let bits = pack_type1(call, grid, power)?;
        Some(bits.to_vec())
    }

    fn unpack(&self, payload: &[u8], _ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != 50 {
            return None;
        }
        let mut buf = [0u8; 50];
        buf.copy_from_slice(payload);
        unpack(&buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type1_roundtrip_callsign() {
        let bits = pack_type1("K1ABC", "FN42", 37).expect("pack");
        let m = unpack(&bits).expect("unpack");
        assert_eq!(
            m,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn type1_roundtrip_with_digit_in_second_slot() {
        // Callsigns with digit at position 1 (e.g. "K9AN") shift into the
        // "right-aligned" form, which is a known WSJT-X pack-call path.
        let bits = pack_type1("K9AN", "EN50", 33).expect("pack");
        let m = unpack(&bits).expect("unpack");
        match m {
            WsprMessage::Type1 {
                callsign,
                grid,
                power_dbm,
            } => {
                assert_eq!(callsign, "K9AN");
                assert_eq!(grid, "EN50");
                assert_eq!(power_dbm, 33);
            }
            other => panic!("expected Type 1, got {:?}", other),
        }
    }

    #[test]
    fn invalid_power_rejected() {
        assert!(pack_type1("K1ABC", "FN42", 42).is_none());
    }

    #[test]
    fn invalid_grid_rejected() {
        // Grid chars beyond 'R' are out of WSJT's locator alphabet.
        assert!(pack_type1("K1ABC", "SS01", 37).is_none());
    }

    #[test]
    fn unpack_rejects_reserved_call_range() {
        // n1 values ≥ 262177560 have no Type-1 interpretation; when ntype
        // looks like a Type 1 dBm we bail out to None.
        let bits = {
            let mut b = [0u8; 50];
            // Set n1 = all ones = 0x0fff_ffff (28-bit) → well into reserved.
            let n1 = 0x0fff_ffffu32;
            let n2 = pack_grid4_power("FN42", 37).unwrap();
            let bytes = pack50(n1, n2);
            for i in 0..50 {
                b[i] = (bytes[i / 8] >> (7 - (i % 8))) & 1;
            }
            b
        };
        // Should not produce Type 1 — either None or Type 2/3.
        if let Some(WsprMessage::Type1 { .. }) = unpack(&bits) {
            panic!("shouldn't be Type 1");
        }
    }

    #[test]
    fn type2_single_char_suffix() {
        // Port WSJT-X's single-char-suffix encoding for `K1ABC/7` at 37 dBm
        // and verify our `unpack` reverses it:
        //   encode:
        //     base_call = "K1ABC"
        //     m_local   = 60000 - 32768 + 7 = 27239
        //     nadd_enc  = 1
        //     ntype     = power + 1 + nadd_enc = 39
        //     n2        = 128 * m_local + ntype + 64 = 3_486_695
        //   decode:
        //     nu        = 39 % 10 = 9 → nadd_dec = 9 - 7 = 2
        //     n3        = n2>>7 + 32768*(nadd_dec - 1) = 27239 + 32768 = 60007
        //     → apply_prefix(60007) → "K1ABC/7", power = 39 - 2 = 37
        let n1 = pack_call("K1ABC").expect("pack call");
        let m_local = 60_000 - 32_768 + 7; // 27239
        let ntype = 37 + 1 + 1; // 39
        let n2 = 128 * m_local + (ntype + 64);
        let bytes = pack50(n1, n2);
        let mut bits = [0u8; 50];
        for i in 0..50 {
            bits[i] = (bytes[i / 8] >> (7 - (i % 8))) & 1;
        }
        let m = unpack(&bits).expect("unpack");
        assert_eq!(
            m,
            WsprMessage::Type2 {
                callsign: "K1ABC/7".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn type2_prefix_pj4() {
        // Port WSJT-X's prefix encoding for `PJ4/K1ABC` at 37 dBm:
        //   prefix "PJ4" → packed as base-37 digits, length 3
        //     start m = 0 (3-char prefix base)
        //     for each char: m = 37*m + nc
        //       P (25) → 25
        //       J (19) → 25*37 + 19 = 944
        //       4  (4) → 944*37 + 4 = 34932
        //     m > 32768 → m -= 32768 = 2164, nadd_enc = 1
        //   ntype = power + 1 + nadd_enc = 39
        //   n2 = 128 * 2164 + ntype + 64 = 277095
        //   decode:
        //     nu = 39 % 10 = 9 → nadd_dec = 2
        //     n3 = 2164 + 32768 = 34932 → < 60000 → prefix path
        //     "PJ4" recovered
        let n1 = pack_call("K1ABC").expect("pack call");
        let m_local = {
            let mut m: u32 = 0;
            for &ch in b"PJ4" {
                let nc = match ch {
                    b'0'..=b'9' => ch - b'0',
                    b'A'..=b'Z' => ch - b'A' + 10,
                    _ => 36,
                };
                m = 37 * m + nc as u32;
            }
            assert!(m > 32_768, "PJ4 should land above 32768");
            m - 32_768
        };
        let ntype = 37 + 1 + 1;
        let n2 = 128 * m_local + (ntype + 64);
        let bytes = pack50(n1, n2);
        let mut bits = [0u8; 50];
        for i in 0..50 {
            bits[i] = (bytes[i / 8] >> (7 - (i % 8))) & 1;
        }
        let m = unpack(&bits).expect("unpack");
        assert_eq!(
            m,
            WsprMessage::Type2 {
                callsign: "PJ4/K1ABC".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn type3_hashed_call_grid6() {
        // Build a Type-3 message: hash=12345, grid6="FN42LX", power=27.
        // Encoding:
        //   grid6_rotated = "N42LXF"   (last-5 + first char)
        //   n1 = pack_call(grid6_rotated)
        //   ntype = -(power + 1) = -28
        //   n2 = 128*hash + ntype + 64  (i.e. (n2 & 127) - 64 == -28)
        let hash = 12_345u32;
        let grid6 = "FN42LX";
        let power = 27i32;
        let rotated = {
            let b = grid6.as_bytes();
            format!(
                "{}{}",
                core::str::from_utf8(&b[1..6]).unwrap(),
                b[0] as char
            )
        };
        assert_eq!(rotated, "N42LXF");
        // Hmm — "N42LXF" has a digit at position 1 (char '4'), which
        // pack_call handles (right-aligned digit form not triggered).
        // Verify pack_call accepts the rotated grid6.
        let n1 = pack_call(&rotated).expect("pack call(grid6)");
        let ntype: i32 = -(power + 1); // -28
        // n2 = 128*hash + (ntype + 64) where ntype + 64 = 36, all positive
        let n2 = hash * 128 + (ntype + 64) as u32;
        let bytes = pack50(n1, n2);
        let mut bits = [0u8; 50];
        for i in 0..50 {
            bits[i] = (bytes[i / 8] >> (7 - (i % 8))) & 1;
        }
        let m = unpack(&bits).expect("unpack");
        assert_eq!(
            m,
            WsprMessage::Type3 {
                callsign_hash: hash,
                grid6: grid6.into(),
                power_dbm: power,
            }
        );
    }

    #[test]
    fn pack50_unpack50_all_bits() {
        let n1 = 0x0deadb3u32;
        let n2 = 0x001abcdu32 & 0x003f_ffff;
        let bytes = pack50(n1, n2);
        let (rn1, rn2) = unpack50(&bytes);
        assert_eq!(rn1, n1);
        assert_eq!(rn2, n2);
    }
}
