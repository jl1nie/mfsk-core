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

/// ARRL Sections used by the Type-0.3 / 0.4 (Field Day) message format.
/// Mirrors WSJT-X `packjt77.f90:227-236` `csec` (NSEC=86), in order, so
/// the 7-bit `isec` field indexes it 1-based exactly as upstream does.
///
/// The length is load-bearing, not decorative: `packjt77.f90:338`
/// refuses the whole message when `isec` falls outside `1..=86`, and a
/// 7-bit field spans `0..=127` — so 42 of the 128 codes are invalid and
/// rejecting them is 33 % of the Field Day phantom surface.
pub(crate) const ARRL_SECTIONS: &[&str] = &[
    "AB", "AK", "AL", "AR", "AZ", "BC", "CO", "CT", "DE", "EB", "EMA", "ENY", "EPA", "EWA", "GA",
    "GH", "IA", "ID", "IL", "IN", "KS", "KY", "LA", "LAX", "NS", "MB", "MDC", "ME", "MI", "MN",
    "MO", "MS", "MT", "NC", "ND", "NE", "NFL", "NH", "NL", "NLI", "NM", "NNJ", "NNY", "TER", "NTX",
    "NV", "OH", "OK", "ONE", "ONN", "ONS", "OR", "ORG", "PAC", "PR", "QC", "RI", "SB", "SC", "SCV",
    "SD", "SDG", "SF", "SFL", "SJV", "SK", "SNJ", "STX", "SV", "TN", "UT", "VA", "VI", "VT", "WCF",
    "WI", "WMA", "WNY", "WPA", "WTX", "WV", "WWA", "WY", "DX", "PE", "NB",
];

// ── Token boundaries ─────────────────────────────────────────────────────────

pub(crate) const NTOKENS: u32 = 2_063_592;
pub(crate) const MAX22: u32 = 4_194_304;
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
pub(crate) fn unpack28(n28: u32) -> String {
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
                return alloc::format!("<{}>", resolved);
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

/// [`resolve_hash12`] for the 22-bit hash the Type-5 message carries in
/// its second callsign field (`packjt77.f90:605` `hash22`).
///
fn resolve_hash22(n22: u32, ht: &CallsignHashTable) -> String {
    match ht.lookup22(n22) {
        Some(call) => format!("<{}>", call),
        None => UNRESOLVED_HASH.to_string(),
    }
}

/// Decode a 15-bit Maidenhead grid square index.
/// The 5-bit power field of a WSPR-type message, in dBm.
///
/// `packjt77.f90:395` — `idbm=nint(idbm*10.0/3.0)` then a `0..60` range
/// check, which is the check that makes a random 5-bit field fail
/// rather than render.
fn wspr_dbm(raw: u32) -> Option<u32> {
    // `(20 * raw + 3) / 6` is `nint(raw * 10 / 3)` exactly, in integers:
    // a half-way case would need `2 * raw ≡ 3 (mod 6)`, whose left side
    // is even and right side odd, so there are none and any correct
    // rounding agrees. Integer because `f32::round` is `std`-only here
    // and this file compiles under `no_std` — which the feature matrix
    // caught and a `full`-only build would not have.
    let dbm = (20 * raw + 3) / 6;
    if dbm > 60 {
        return None;
    }
    Some(dbm)
}

/// The 16-bit add-on field of a WSPR type-2 message: a base-36 prefix
/// below `NZZZ`, a 1-3 character suffix above it.
///
/// Ported from `packjt77.f90:413-441`, including the `npfx > 12959`
/// rejection — the one branch there that sets `unpk77_success=.false.`
/// and returns.
fn wspr_prefix_suffix(npfx: u32, call: &str) -> Option<String> {
    const A2: &[u8] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const NZZZ: u32 = 46_656; // 36^3
    if npfx < NZZZ {
        let mut n = npfx;
        let mut cpfx = [b' '; 3];
        for i in (0..3).rev() {
            cpfx[i] = A2[(n % 36) as usize];
            n /= 36;
            if n == 0 {
                break;
            }
        }
        let pfx = core::str::from_utf8(&cpfx).ok()?.trim();
        return Some(format!("{}/{}", pfx, call));
    }
    let n = npfx - NZZZ;
    // At most three characters, so a fixed buffer rather than a `Vec`:
    // `vec!` is not in scope under `no_std` here, and nothing about a
    // 3-byte suffix wants the heap.
    let mut buf = [0u8; 3];
    let sfx: &[u8] = if n <= 35 {
        buf[0] = A2[n as usize];
        &buf[..1]
    } else if n <= 1295 {
        buf[0] = A2[(n / 36) as usize];
        buf[1] = A2[(n % 36) as usize];
        &buf[..2]
    } else if n <= 12_959 {
        buf[0] = A2[(n / 360) as usize];
        buf[1] = A2[((n / 10) % 36) as usize];
        buf[2] = A2[(n % 10) as usize];
        &buf[..3]
    } else {
        return None;
    };
    Some(format!("{}/{}", call, core::str::from_utf8(sfx).ok()?))
}

/// 6-character Maidenhead grid from the 25-bit field of a WSPR-type
/// (`i3=0, n3=6`) message.
///
/// Ported from `packjt77.f90`'s `to_grid`, bounds included: every digit
/// is range-checked, and `j5 == j6 == 24` is the sentinel for a
/// four-character grid, which upstream renders by leaving characters
/// 5-6 blank.
///
/// **Not** upstream's `to_grid6`, which is a different function on a
/// different base — see [`to_grid6`]. This was named `to_grid6` until
/// the type-5 port needed the real one and the collision surfaced.
fn to_grid(n: u32) -> Option<String> {
    let mut n = n;
    let j1 = n / (18 * 10 * 10 * 25 * 25);
    if j1 > 17 {
        return None;
    }
    n -= j1 * (18 * 10 * 10 * 25 * 25);
    let j2 = n / (10 * 10 * 25 * 25);
    if j2 > 17 {
        return None;
    }
    n -= j2 * (10 * 10 * 25 * 25);
    let j3 = n / (10 * 25 * 25);
    if j3 > 9 {
        return None;
    }
    n -= j3 * (10 * 25 * 25);
    let j4 = n / (25 * 25);
    if j4 > 9 {
        return None;
    }
    n -= j4 * (25 * 25);
    let j5 = n / 25;
    let j6 = n - j5 * 25;
    if j5 > 24 || j6 > 24 {
        return None;
    }
    let mut g = String::with_capacity(6);
    g.push((b'A' + j1 as u8) as char);
    g.push((b'A' + j2 as u8) as char);
    g.push((b'0' + j3 as u8) as char);
    g.push((b'0' + j4 as u8) as char);
    if j5 != 24 || j6 != 24 {
        g.push((b'A' + j5 as u8) as char);
        g.push((b'A' + j6 as u8) as char);
    }
    Some(g)
}

/// Ported from `packjt77.f90`'s `to_grid6` — the strictly
/// six-character form used by the Type-5 (EU VHF contest) message.
///
/// Distinct from [`to_grid`] in both base and range: the subsquare
/// digits run `0..=23` (`A`..`X`) against a base of 24 rather than
/// `0..=24` against 25, and there is no four-character sentinel. The
/// consequence is a tight bound — `18*18*10*10*24*24 = 18_662_400`
/// valid codes in a 25-bit field, so 44 % of the field names no grid
/// at all and is refused, which `packjt77.f90:599` checks up front.
fn to_grid6(n: u32) -> Option<String> {
    let mut n = n;
    let j1 = n / (18 * 10 * 10 * 24 * 24);
    if j1 > 17 {
        return None;
    }
    n -= j1 * (18 * 10 * 10 * 24 * 24);
    let j2 = n / (10 * 10 * 24 * 24);
    if j2 > 17 {
        return None;
    }
    n -= j2 * (10 * 10 * 24 * 24);
    let j3 = n / (10 * 24 * 24);
    if j3 > 9 {
        return None;
    }
    n -= j3 * (10 * 24 * 24);
    let j4 = n / (24 * 24);
    if j4 > 9 {
        return None;
    }
    n -= j4 * (24 * 24);
    let j5 = n / 24;
    let j6 = n - j5 * 24;
    if j5 > 23 || j6 > 23 {
        return None;
    }
    let mut g = String::with_capacity(6);
    g.push((b'A' + j1 as u8) as char);
    g.push((b'A' + j2 as u8) as char);
    g.push((b'0' + j3 as u8) as char);
    g.push((b'0' + j4 as u8) as char);
    g.push((b'A' + j5 as u8) as char);
    g.push((b'A' + j6 as u8) as char);
    Some(g)
}

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
/// Supported types — every one `packjt77.f90` defines, since the
/// Type-5 port (issue #383); `0/2` and `i3 >= 6` are the codes
/// upstream itself marks unused and refuses.
/// - `0/0`  Free text
/// - `0/1`  DXpedition RR73
/// - `0/3`, `0/4`  ARRL Field Day: `CALL1 CALL2 [R] <ntx><class> SEC`
/// - `0/5`  Telemetry, 71 bits as up to 18 hex digits
/// - `0/6`  WSPR types 1/2/3
/// - `1`    Standard: `CALL1 CALL2 GRID` or `CALL1 CALL2 REPORT`
/// - `2`    Standard with `/P`
/// - `3`    ARRL RTTY Roundup
/// - `4`    One non-standard callsign + 12-bit hashed counterpart
/// - `5`    EU VHF contest: `<CALL> <CALL> [R] <rst><serial> GRID6`
pub fn unpack77(msg: &[u8]) -> Option<String> {
    // One implementation, not two. This used to carry a full copy of
    // the branch table that `unpack77_with_hash` also carries, the
    // two differing only in `unpack28` vs `unpack28_h` and the 10-bit
    // hash lookup — so every per-type validity check had to be written
    // twice or the two would disagree about what a valid message is.
    // An empty table makes `unpack28_h` identical to `unpack28` (its
    // only divergence is a `lookup22` hit, which an empty table cannot
    // produce), and `CallsignHashTable::new()` allocates nothing.
    unpack77_with_hash(msg, &CallsignHashTable::new())
}

/// Decode a 77-bit FT8 message, resolving hashed callsigns via a lookup table.
///
/// Behaves identically to [`unpack77`] but replaces `<...>` placeholders with
/// actual callsigns when they are found in the hash table.
/// Unpack, resolving hashed callsigns, **and register the callsigns
/// this message carries** so later messages can resolve *their*
/// hashes against them.
///
/// This is the flow WSJT-X runs: `save_hash_call` is invoked with the
/// callsign fields as they are unpacked, and the table it fills is
/// what turns a later `<...>` into a name. Without the registering
/// half a table stays empty and [`unpack77_with_hash`] can only ever
/// return placeholders — which is exactly what shipped until
/// 2026-09-20, when a CoreS3 on 7041 kHz rendered `<...>` in 69 of 380
/// decodes with no code anywhere in this crate or its consumers
/// calling [`CallsignHashTable::insert`] outside a test.
///
/// **Registration is by field, never by parsing the rendered text.**
/// A token scan cannot tell `PM95` from a callsign without duplicating
/// this walker's knowledge, and [`CallsignHashTable::insert`] rejects
/// only `CQ…` and strings under two characters — so `RRR`, `DX` and
/// `TU` would all be registered as callsigns. The failure that causes
/// is worse than a placeholder: a polluted entry resolves a later hash
/// to the *wrong* callsign, silently.
///
/// [`unpack77_with_hash`] keeps its shared borrow and stays
/// resolve-only, because `DecodeContext::callsign_hash_table` is an
/// `Arc<dyn Any + Send + Sync>` shared across the parallel decode path
/// and cannot hand out `&mut` without a mutex in a `no_std`,
/// `Send + Sync` context. Callers that own their table use this one.
pub fn unpack77_learn(msg: &[u8], ht: &mut CallsignHashTable) -> Option<String> {
    // Resolve first, learn second: a message must not be allowed to
    // resolve its own hash field against a call it is itself
    // introducing. WSJT-X has the same ordering for the same reason.
    let text = unpack77_with_hash(msg, ht);
    register_callsigns(msg, ht);
    text
}

/// Register the callsign fields of a 77-bit message into `ht`.
///
/// **Public because resolving and learning cannot always happen in the
/// same place.** The FT4 receiver runs `decode_candidate` on two cores
/// at once and its own comment names the reason it can: "no shared
/// mutable state". A `&mut` table there is not an option, so that
/// caller resolves with a shared borrow inside the workers and calls
/// this once per decode afterwards, single-threaded.
/// [`unpack77_learn`] is the convenience form for callers that have
/// no such constraint.
///
/// Walks the same `i3`/`n3` layout [`unpack77_with_hash`] does, but
/// only the callsign fields — the 28-bit standard-call tokens and, for
/// `i3 = 4`, the 58-bit nonstandard call, which is the field whose
/// hash other stations will be transmitting.
///
/// `unpack28` also yields `CQ`, `DE`, `QRZ`, `CQ DX` and `CQ 001` for
/// low tokens, so every candidate is filtered through
/// [`is_standard_callsign`]; the nonstandard call is registered as-is,
/// which is the whole point of it.
pub fn register_callsigns(msg: &[u8], ht: &mut CallsignHashTable) {
    if msg.len() != 77 {
        return;
    }
    let n3 = read_bits(msg, 71, 3);
    let i3 = read_bits(msg, 74, 3);

    let learn_std = |n28: u32, ht: &mut CallsignHashTable| {
        let call = unpack28(n28);
        if is_standard_callsign(&call) {
            ht.insert(&call);
        }
    };

    match i3 {
        0 => match n3 {
            // DXpedition and Field Day both carry two 28-bit calls at
            // the same offsets; free text (n3 = 0) carries none.
            1 | 3 | 4 => {
                learn_std(read_bits(msg, 0, 28), ht);
                learn_std(read_bits(msg, 28, 28), ht);
            }
            _ => {}
        },
        1 | 2 => {
            learn_std(read_bits(msg, 0, 28), ht);
            learn_std(read_bits(msg, 29, 28), ht);
        }
        // ARRL RTTY Roundup: one bit of ITU flag first, so the fields
        // sit at 1 and 29 rather than 0 and 28.
        3 => {
            learn_std(read_bits(msg, 1, 28), ht);
            learn_std(read_bits(msg, 29, 28), ht);
        }
        // Nonstandard call. The 12-bit field is a *hash* of the other
        // station and carries no callsign to learn; the 58-bit field
        // is the call itself.
        4 => {
            let n58 = read_bits_u64(msg, 12, 58);
            let mut n = n58;
            let mut buf = [b' '; 11];
            for i in (0..11).rev() {
                buf[i] = C38[(n % 38) as usize];
                n /= 38;
            }
            if let Ok(s) = core::str::from_utf8(&buf) {
                let call = s.trim();
                // Two characters is `insert`'s own floor; below it
                // there is nothing a hash could usefully name.
                if call.len() >= 2 {
                    ht.insert(call);
                }
            }
        }
        _ => {}
    }
}

pub fn unpack77_with_hash(msg: &[u8], ht: &CallsignHashTable) -> Option<String> {
    unpack77_fields(msg, ht).map(|m| m.to_string())
}

/// Whether one callsign *field* is plausible.
///
/// A field, not a token: [`Wsjt77Fields::callsigns`] yields exactly the
/// places a callsign can appear, so this never has to guess whether it
/// is looking at a grid or a report. Three things can legitimately be
/// there:
///
/// - a directed-CQ token (`CQ`, `DE`, `QRZ`, `CQ DX`, `CQ POTA`,
///   `CQ 123`) — `unpack28` renders the first three token codes and
///   the directed-CQ range as words;
/// - a hashed callsign, `<CALL>` or the unresolved `<...>`;
/// - an actual callsign, which goes to the ITU prefix allowlist.
pub fn is_plausible_call(field: &str) -> bool {
    if matches!(field, "CQ" | "DE" | "QRZ") || field.starts_with("CQ ") {
        return true;
    }
    if field.starts_with('<') && field.ends_with('>') {
        return true;
    }
    is_plausible_callsign(field)
}

/// A decoded 77-bit message, **as fields** rather than as the string
/// they render to.
///
/// This is what [`unpack77_fields`] produces and what every later stage
/// consumes. The rendered form (its [`Display`](core::fmt::Display)) is for
/// humans, and asking a question of it means splitting it back into
/// tokens and guessing which ones were callsigns — the inverse of the
/// work `unpack77` just did, and wrong in ways that are invisible:
/// judging `JA1ABC 3Y0Z 6A EMA` by its tokens tests `6A` and `EMA`
/// against a callsign grammar, and judging `JA1ABC PM95 20` tests
/// `PM95` and `20`. Issue #383's filter did exactly that, and refused
/// three message types outright for as long as it existed.
///
/// Every field here was range-checked during unpacking — the ARRL
/// section index, the grid indices, the RTTY exchange — so the only
/// question left for a consumer is whether the *callsigns* are
/// plausible, which is what [`Wsjt77Fields::callsigns`] is for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Wsjt77Fields {
    /// `i3=0, n3=0` — 13 characters from a 42-symbol alphabet. Nearly
    /// every bit pattern is one, so there is nothing here to check.
    FreeText(String),
    /// `i3=0, n3=1` — `CALL1 RR73; CALL2 <hash10> REPORT`.
    DxPedition {
        call1: String,
        call2: String,
        /// The 10-bit hash, rendered `<CALL>` or `<...>`.
        call3: String,
        report: String,
    },
    /// `i3=0, n3=3|4` — `CALL1 CALL2 [R] <ntx><class> SECTION`.
    FieldDay {
        call1: String,
        call2: String,
        ack: bool,
        transmitters: u32,
        class: char,
        section: &'static str,
    },
    /// `i3=0, n3=5` — 71 bits as up to 18 hex digits. Valid by
    /// construction, like [`Self::FreeText`].
    Telemetry(String),
    /// `i3=0, n3=6` — the WSPR types. `call` is a plain or hashed
    /// callsign; `tail` is the grid and power as rendered.
    Wspr { call: String, tail: String },
    /// `i3=1|2` — the standard message. `exchange` is the grid, the
    /// report, `RRR`/`RR73`/`73`, or empty for a bare `CQ CALL`.
    Standard {
        call1: String,
        call2: String,
        exchange: String,
    },
    /// `i3=3` — ARRL RTTY Roundup.
    RttyRoundup {
        thank_you: bool,
        call1: String,
        call2: String,
        ack: bool,
        rst: String,
        exchange: String,
    },
    /// `i3=4` — one non-standard callsign and one 12-bit hash, or a
    /// `CQ` from the non-standard one.
    Nonstandard {
        call1: String,
        call2: Option<String>,
        exchange: &'static str,
    },
    /// `i3=5` — EU VHF contest. **Both callsigns are hashes**, so the
    /// only evidence the message is real is that one of them resolved.
    EuVhfContest {
        call1: String,
        call2: String,
        ack: bool,
        exchange: String,
        grid6: String,
    },
}

/// The placeholder this module's hash resolvers emit when a
/// hash is not in the table.
pub const UNRESOLVED_HASH: &str = "<...>";

impl Wsjt77Fields {
    /// The callsign-bearing fields, in the order they render.
    ///
    /// Tokens like `CQ`, `CQ DX` and `<...>` come through as they are;
    /// [`is_plausible_call`] is what decides whether each is
    /// acceptable.
    pub fn callsigns(&self) -> impl Iterator<Item = &str> {
        let (a, b, c) = match self {
            Self::FreeText(_) | Self::Telemetry(_) => (None, None, None),
            Self::DxPedition {
                call1,
                call2,
                call3,
                ..
            } => (Some(&**call1), Some(&**call2), Some(&**call3)),
            Self::FieldDay { call1, call2, .. }
            | Self::Standard { call1, call2, .. }
            | Self::RttyRoundup { call1, call2, .. }
            | Self::EuVhfContest { call1, call2, .. } => (Some(&**call1), Some(&**call2), None),
            Self::Wspr { call, .. } => (Some(&**call), None, None),
            Self::Nonstandard { call1, call2, .. } => (Some(&**call1), call2.as_deref(), None),
        };
        [a, b, c].into_iter().flatten()
    }

    /// Whether this looks like a message someone sent, rather than a
    /// CRC survivor.
    ///
    /// The layers below are error *detection* — LDPC parity, then the
    /// CRC — and neither says anything about the message. A CRC-14
    /// false positive is a codeword the decoder converged on that is
    /// not the transmitted one, so its 77 bits are effectively
    /// uniform, and better than half of those unpack to a
    /// syntactically valid message.
    ///
    /// **This has no WSJT-X counterpart** — `ft8b.f90` accepts on
    /// `nbadcrc` and `nharderrors <= 36`. It exists because this
    /// crate's strategies *subtract* each decode from the audio before
    /// looking again, so a wrong one is not a cosmetic error: its
    /// waveform is removed from the residual and takes the real signal
    /// underneath with it. See
    /// `FrameDecodable::MESSAGE_FILTER_DEFAULT` for the measurement.
    pub fn is_plausible(&self) -> bool {
        match self {
            // Nothing to check — see the variant docs.
            Self::FreeText(_) | Self::Telemetry(_) => true,
            // Both callsigns are hashes; the message names neither
            // station unless one resolved, and an unresolved pair is
            // indistinguishable from a CRC survivor. 138 700 of every
            // 2 M uniform payloads reach this type
            // (`phantom_survival_rates`), so accepting it blind is a
            // 44 % rise in the surviving phantom population in
            // exchange for rows nobody can read.
            Self::EuVhfContest { call1, call2, .. } => {
                call1 != UNRESOLVED_HASH || call2 != UNRESOLVED_HASH
            }
            _ => self.callsigns().all(is_plausible_call),
        }
    }
}

impl core::fmt::Display for Wsjt77Fields {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::FreeText(t) | Self::Telemetry(t) => write!(f, "{t}"),
            Self::DxPedition {
                call1,
                call2,
                call3,
                report,
            } => write!(f, "{call1} RR73; {call2} {call3} {report}"),
            Self::FieldDay {
                call1,
                call2,
                ack,
                transmitters,
                class,
                section,
            } => {
                let r = if *ack { "R " } else { "" };
                write!(f, "{call1} {call2} {r}{transmitters}{class} {section}")
            }
            Self::Wspr { call, tail } => write!(f, "{call} {tail}"),
            Self::Standard {
                call1,
                call2,
                exchange,
            } => {
                if exchange.is_empty() {
                    write!(f, "{call1} {call2}")
                } else {
                    write!(f, "{call1} {call2} {exchange}")
                }
            }
            Self::RttyRoundup {
                thank_you,
                call1,
                call2,
                ack,
                rst,
                exchange,
            } => {
                let tu = if *thank_you { "TU; " } else { "" };
                let r = if *ack { "R " } else { "" };
                write!(f, "{tu}{call1} {call2} {r}{rst} {exchange}")
            }
            Self::Nonstandard {
                call1,
                call2,
                exchange,
            } => match call2 {
                None => write!(f, "{call1}"),
                Some(c2) if exchange.is_empty() => write!(f, "{call1} {c2}"),
                Some(c2) => write!(f, "{call1} {c2} {exchange}"),
            },
            Self::EuVhfContest {
                call1,
                call2,
                ack,
                exchange,
                grid6,
            } => {
                let r = if *ack { "R " } else { "" };
                write!(f, "{call1} {call2} {r}{exchange} {grid6}")
            }
        }
    }
}

/// Whether a decoded payload looks like a message someone sent.
///
/// Decodes once with [`unpack77_fields`] and asks
/// [`Wsjt77Fields::is_plausible`]. There is no text stage: nothing
/// here renders a message and splits it back into tokens.
pub fn is_plausible_payload(msg: &[u8]) -> bool {
    is_plausible_payload_with_hash(msg, &CallsignHashTable::new())
}

/// [`is_plausible_payload`] with the caller's hashed-callsign table.
///
/// It matters for exactly one type. The EU VHF contest message
/// (`i3=5`) carries **two hashes and no plain callsign**, so the only
/// evidence it is real is that one of them resolves — and against an
/// empty table none can, which refuses the type outright.
///
/// **No decode path supplies a table here yet**, so that is the
/// shipped behaviour: `i3=5` is refused wherever the verdict is
/// applied. That is deliberate rather than overlooked — an unresolved
/// `<...> <...> R 590003 IO91NP` names neither station, so it is not a
/// message an operator can act on. Threading the table that
/// [`unpack77_with_hash`] already takes down to this point is the fix,
/// and it is a change to the request API rather than to this function.
pub fn is_plausible_payload_with_hash(msg: &[u8], ht: &CallsignHashTable) -> bool {
    unpack77_fields(msg, ht).is_some_and(|m| m.is_plausible())
}

/// The one decode: bits to fields. `unpack77`/`unpack77_with_hash`
/// render the result; `Wsjt77Fields::is_plausible` judges it. Nothing
/// downstream re-parses the rendered string.
pub fn unpack77_fields(msg: &[u8], ht: &CallsignHashTable) -> Option<Wsjt77Fields> {
    let n3 = read_bits(msg, 71, 3);
    let i3 = read_bits(msg, 74, 3);
    let decoded = unpack77_dispatch(msg, ht, i3, n3)?;
    // `packjt77.f90:616` — the last thing upstream's `unpack77` does,
    // for every type: `if(msg(1:4).eq.'CQ <') unpk77_success=.false.`
    // A CQ is addressed to nobody, so the station it names cannot be a
    // hash: nothing can have introduced one. Upstream tests the
    // assembled string's first four characters; the fields say the
    // same thing without the substring.
    let refuse = {
        let mut calls = decoded.callsigns();
        matches!((calls.next(), calls.next()), (Some(a), Some(b))
            if (a == "CQ" || a.starts_with("CQ ")) && b.starts_with('<'))
    };
    if refuse {
        return None;
    }
    Some(decoded)
}

fn unpack77_dispatch(msg: &[u8], ht: &CallsignHashTable, i3: u32, n3: u32) -> Option<Wsjt77Fields> {
    match i3 {
        0 => match n3 {
            0 => {
                let text = unpack_free_text(msg);
                if text.is_empty() {
                    None
                } else {
                    Some(Wsjt77Fields::FreeText(text))
                }
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
                // `packjt77.f90:318,320` — both callsign fields are
                // checked against the token range. `n28 <= 2` is
                // `DE` / `QRZ` / `CQ`, which `unpack28` renders as a
                // word: a DXpedition participant cannot be one, so a
                // field that lands there is a CRC-14 survivor rather
                // than a message.
                if n28a <= 2 || n28b <= 2 {
                    return None;
                }
                let c1 = unpack28_h(n28a, ht);
                let c2 = unpack28_h(n28b, ht);
                let c3 = if let Some(call) = ht.lookup10(n10) {
                    format!("<{}>", call)
                } else {
                    "<...>".to_string()
                };
                Some(Wsjt77Fields::DxPedition {
                    call1: c1,
                    call2: c2,
                    call3: c3,
                    report: crpt,
                })
            }
            5 => {
                // `packjt77.f90:360` — telemetry, 71 bits shown as 18 hex
                // digits with leading zeros blanked. Not implemented here
                // until now, so a telemetry message was a dropped decode
                // rather than a rejected one.
                let hex = format!(
                    "{:06X}{:06X}{:06X}",
                    read_bits(msg, 0, 23),
                    read_bits(msg, 23, 24),
                    read_bits(msg, 47, 24)
                );
                // Upstream blanks leading '0's and left-justifies, which
                // for an all-zero payload leaves an empty message — it
                // reports success either way, and so do we.
                Some(Wsjt77Fields::Telemetry(
                    hex.trim_start_matches('0').to_string(),
                ))
            }
            6 => {
                // `packjt77.f90:372` — WSPR-type. `itype` comes from bits
                // 48..50 (1-based in Fortran), i.e. `msg[47..50]` here.
                let (j48, j49, j50) = (msg[47] & 1, msg[48] & 1, msg[49] & 1);
                let itype = if j50 == 1 {
                    2
                } else if j49 == 0 {
                    1
                } else if j48 == 0 {
                    3
                } else {
                    return None;
                };
                match itype {
                    1 => {
                        let n28 = read_bits(msg, 0, 28);
                        let igrid4 = read_bits(msg, 28, 15);
                        let idbm = wspr_dbm(read_bits(msg, 43, 5))?;
                        let grid = to_grid4(igrid4)?;
                        Some(Wsjt77Fields::Wspr {
                            call: unpack28_h(n28, ht),
                            tail: format!("{} {}", grid, idbm),
                        })
                    }
                    2 => {
                        let n28 = read_bits(msg, 0, 28);
                        let npfx = read_bits(msg, 28, 16);
                        let idbm = wspr_dbm(read_bits(msg, 44, 5))?;
                        let call = unpack28_h(n28, ht);
                        let composed = wspr_prefix_suffix(npfx, &call)?;
                        Some(Wsjt77Fields::Wspr {
                            call: composed,
                            tail: idbm.to_string(),
                        })
                    }
                    _ => {
                        let n22 = read_bits(msg, 0, 22);
                        let igrid6 = read_bits(msg, 22, 25);
                        let grid = to_grid(igrid6)?;
                        Some(Wsjt77Fields::Wspr {
                            call: unpack28_h(n22 + NTOKENS, ht),
                            tail: grid,
                        })
                    }
                }
            }
            3 | 4 => {
                // `packjt77.f90:333-357` — ARRL Field Day, laid out
                // `2b28,b1,b4,b3,b7` (`:337`) over the 71 payload bits:
                // two callsigns, the `R` flag, the transmitter count,
                // the class letter and the ARRL section.
                let (n28a, n28b) = (read_bits(msg, 0, 28), read_bits(msg, 28, 28));
                // `packjt77.f90:343,345` — the same token-range check
                // upstream applies to Field Day's two callsign fields.
                if n28a <= 2 || n28b <= 2 {
                    return None;
                }
                let ir = msg[56] & 1;
                let intx = read_bits(msg, 57, 4);
                let nclass = read_bits(msg, 61, 3);
                let isec = read_bits(msg, 64, 7) as usize;
                // `packjt77.f90:338` — `isec` is a 1-based index into an
                // 86-entry table carried in a 7-bit field, so 42 of its
                // 128 codes name no section at all. Upstream refuses the
                // message; this was the last unported check of the four
                // the type carries, and it is the largest single filter
                // in the family (see `phantom_survival_rates`).
                if isec < 1 || isec > ARRL_SECTIONS.len() {
                    return None;
                }
                let sec = ARRL_SECTIONS[isec - 1];
                // `packjt77.f90:346-347`: the count is 1-based, and
                // n3=4 is simply the +16 continuation of n3=3's range.
                let ntx = intx + 1 + if n3 == 4 { 16 } else { 0 };
                let class = (b'A' + nclass as u8) as char;
                let c1 = unpack28_h(n28a, ht);
                let c2 = unpack28_h(n28b, ht);
                // `packjt77.f90:350-357` writes this as four separate
                // cases because its `cntx` is a fixed-width `character*3`
                // whose leading blank doubles as the separator for
                // `ntx < 10`. Formatting the number directly makes all
                // four collapse into one, with identical output.
                Some(Wsjt77Fields::FieldDay {
                    call1: c1,
                    call2: c2,
                    ack: ir == 1,
                    transmitters: ntx,
                    class,
                    section: sec,
                })
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

            // `packjt77.f90:494,509` — a CQ is a call to no one in
            // particular, so it cannot acknowledge (`R`) and it cannot
            // carry a report. Upstream tests the assembled message's
            // first three characters; `c1` is what those come from, and
            // every CQ token `unpack28` produces (`CQ`, `CQ 123`,
            // `CQ DX`) starts the message with `CQ `.
            let is_cq = c1 == "CQ" || c1.starts_with("CQ ");
            let report = if igrid <= MAX_GRID4 {
                if is_cq && ir == 1 {
                    return None;
                }
                let grid = to_grid4(igrid)?;
                if ir == 0 { grid } else { format!("R {}", grid) }
            } else {
                let irpt = igrid - MAX_GRID4;
                // `irpt == 1` is the bare `CQ CALL` form and is the only
                // one a CQ may take; 2..4 are RRR / RR73 / 73 and 5+ is
                // a signal report.
                if is_cq && irpt >= 2 {
                    return None;
                }
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

            Some(Wsjt77Fields::Standard {
                call1: c1,
                call2: c2,
                exchange: report,
            })
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
                // `packjt77.f90:532-551` builds `msg` only inside the
                // `imult` / `nserial` range arms, so an exchange
                // outside both leaves it blank — upstream still reports
                // success and prints an empty line. **Deliberate
                // divergence**: refuse instead. `pack77_3` cannot
                // produce such a value, so the only thing that reaches
                // here is a CRC survivor, and the `[RTTY]` marker this
                // used to return made the verdict of the day — then a
                // text rule — short-circuit past the callsign check.
                // Those 22 of 8192 exchange codes were the entire
                // surviving i3=3 phantom population
                // (`phantom_survival_rates`).
                return None;
            };
            Some(Wsjt77Fields::RttyRoundup {
                thank_you: itu == 1,
                call1: c1,
                call2: c2,
                ack: ir == 1,
                rst,
                exchange: exch,
            })
        }

        5 => {
            // `packjt77.f90:593-610` — EU VHF contest, laid out
            // `b12,b22,b1,b3,b11,b25` over the 74 payload bits: two
            // *always-hashed* callsigns (`pack77_5` refuses the type
            // unless both are written `<CALL>`), the `R` flag, the
            // RS(T) report, a serial and a six-character grid.
            //
            // Unimplemented until now — the outer `match` fell through
            // to `_ => None`, so an EU VHF contest frame was a dropped
            // decode rather than a rejected one, the same gap the
            // telemetry and WSPR types had.
            let n12 = read_bits(msg, 0, 12);
            let n22 = read_bits(msg, 12, 22);
            let ir = msg[34] & 1;
            let irpt = read_bits(msg, 35, 3);
            let iserial = read_bits(msg, 38, 11);
            let igrid6 = read_bits(msg, 49, 25);
            // `packjt77.f90:599` guards this before unpacking anything
            // else; [`to_grid6`]'s own bounds are the same test, so
            // running it first is equivalent and keeps one copy.
            let grid = to_grid6(igrid6)?;
            let c1 = resolve_hash12(n12, ht);
            let c2 = resolve_hash22(n22, ht);
            // `packjt77.f90:607` — `write(cexch,'(i2,i4.4)') nrs,iserial`.
            let nrs = 52 + irpt;
            Some(Wsjt77Fields::EuVhfContest {
                call1: c1,
                call2: c2,
                ack: ir == 1,
                exchange: format!("{:02}{:04}", nrs, iserial),
                grid6: grid,
            })
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
                return Some(Wsjt77Fields::Nonstandard {
                    call1: "CQ".to_string(),
                    call2: Some(nonstd),
                    exchange: "",
                });
            }

            let hashed = resolve_hash12(n12, ht);
            let (c1, c2) = if iflip == 0 {
                (hashed, nonstd)
            } else {
                (nonstd, hashed)
            };

            let exchange = match nrpt {
                0 => "",
                1 => "RRR",
                2 => "RR73",
                3 => "73",
                _ => return None,
            };
            Some(Wsjt77Fields::Nonstandard {
                call1: c1,
                call2: Some(c2),
                exchange,
            })
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
    pack_grid4(grid)?;
    pack77(call1, call2, grid)
}

/// `JA1ABC/P` → `("JA1ABC", true, true)`, `W9XYZ/R` → `("W9XYZ", true,
/// false)`, anything else unchanged with `(false, false)`.
///
/// `packjt77.f90:1176-1193`: the suffix counts only when it starts at
/// column 4 or later (`index(w//' ','/P ') >= 4`), i.e. behind a base
/// of at least three characters — `CQ` tokens and the like never have
/// one.
fn split_rp_suffix(call: &str) -> (&str, bool, bool) {
    let c = call.trim();
    if c.len() >= 5 {
        if let Some(base) = c.strip_suffix("/P") {
            return (base, true, true);
        }
        if let Some(base) = c.strip_suffix("/R") {
            return (base, true, false);
        }
    }
    (c, false, false)
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
/// Either callsign may carry a `/R` or `/P` suffix, set as the `ipa` /
/// `ipb` flag beside its 28-bit field. A `/P` on either makes the
/// message Type 2 (i3=2), where both flags read as `/P`; otherwise it is
/// Type 1, where they read as `/R` — `pack77_1` in `packjt77.f90`
/// (1176-1193), mirrored by [`unpack77`]'s `1 | 2` arm. Before this, a
/// suffixed call made `pack28` fail and the whole message with it, so a
/// portable station could not send a standard message at all.
///
/// # Examples
/// ```
/// # use mfsk_core::msg::wsjt77::{pack77, unpack77};
/// let msg = pack77("CQ", "JA1ABC", "PM95").unwrap();
/// let msg = pack77("JA1ABC", "3Y0Z", "-12").unwrap();
/// let msg = pack77("3Y0Z", "JA1ABC", "R-12").unwrap();
/// let msg = pack77("JA1ABC", "3Y0Z", "RR73").unwrap();
/// let msg = pack77("CQ SOTA", "JA1ABC/P", "PM95").unwrap();
/// assert_eq!(unpack77(&msg).unwrap(), "CQ SOTA JA1ABC/P PM95");
/// ```
pub fn pack77(call1: &str, call2: &str, report: &str) -> Option<[u8; 77]> {
    let (base1, ipa, p1) = split_rp_suffix(call1);
    let (base2, ipb, p2) = split_rp_suffix(call2);
    let n28a = pack28(base1)?;
    let n28b = pack28(base2)?;
    let i3 = if p1 || p2 { 2 } else { 1 };

    let report = report.trim();

    // Determine igrid and ir flag
    // `pack77_1` (`packjt77.f90`): the last word is tried as a grid *first*
    // (`is_grid4(w(nwords)(1:4))`), so `RR73`, which has a grid's shape, goes
    // out as the grid RR73 (15-bit value 32373), not as `MAXGRID4 + 3`. Only
    // `RRR` and `73` take the `MAXGRID4 + irpt` values. Both forms unpack as
    // "RR73", which is how this crate packed the second one without noticing;
    // on the air, and in upstream's AP pattern for RR73 (`mrr73` in `ft8b.f90`),
    // it is the grid (#464).
    let (igrid, ir): (u32, u8) = if report.is_empty() {
        (MAX_GRID4 + 1, 0)
    } else if report.len() == 4 && pack_grid4(report).is_some() {
        // Grid locator (e.g. "PM95"), and "RR73".
        (pack_grid4(report).unwrap(), 0)
    } else if let Some(g) = report.strip_prefix("R ")
        && g.len() == 4
        && let Some(igrid) = pack_grid4(g)
    {
        // "R EN37": the grid with `ir = 1`, as `pack77_1` packs a
        // two-word `R <grid>` ending (`packjt77.f90`) and as unpack prints
        // it. It fell through to the report parser and failed.
        (igrid, 1)
    } else if report == "RRR" {
        (MAX_GRID4 + 2, 0)
    } else if report == "73" {
        (MAX_GRID4 + 4, 0)
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
        // `if(irpt.ge.-50 .and. irpt.le.-31) irpt=irpt+101; irpt=irpt+35`: -50..-31
        // go to 86..105. This used to add 101 only when `snr + 35` was negative,
        // so -35..-31 packed as 0..4, and -34..-31 collided with the bare / RRR /
        // RR73 / 73 values (a -34 dB report read back as no report at all).
        let irpt = if (-50..=-31).contains(&snr) {
            snr + 101
        } else {
            snr
        } + 35;
        (MAX_GRID4 + irpt as u32, r_prefix)
    };

    let mut msg = [0u8; 77];
    write_bits(&mut msg, 0, 28, n28a);
    msg[28] = ipa as u8;
    write_bits(&mut msg, 29, 28, n28b);
    msg[57] = ipb as u8;
    msg[58] = ir; // ir (bit 58)
    write_bits(&mut msg, 59, 15, igrid);
    write_bits(&mut msg, 74, 3, i3);
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

    /// Every shape a callsign *field* can legitimately hold.
    ///
    /// These used to be whole-message strings run through a token
    /// loop, which had to guess which tokens were callsigns — and got
    /// it wrong for three message types. `Wsjt77Fields::callsigns`
    /// yields the fields, so the question is per field, which is the
    /// question that was always being asked.
    #[test]
    fn plausible_call_accepts_every_shape_a_field_can_hold() {
        // Token codes and the directed-CQ range, which `unpack28`
        // renders as words rather than callsigns.
        for f in [
            "CQ", "DE", "QRZ", "CQ DX", "CQ POTA", "CQ NA", "CQ SOTA", "CQ 123",
        ] {
            assert!(is_plausible_call(f), "{f} is a token, not a callsign");
        }
        // Hashes, resolved and not.
        for f in ["<...>", "<JA1ABC>", "<KH1/KH7Z>", "<G4ABC/P>"] {
            assert!(is_plausible_call(f), "{f} is a hash");
        }
        // Ordinary calls, compound calls, and CEPT prefixes.
        for f in [
            "JA1ABC", "3Y0Z", "R6WA", "ZS6S", "W1AW", "JR1UJX/P", "JH4IUV/P", "JL1NIE/1",
            "F/JA1ABC", "JR9ECD/P", "K1ABC", "W9XYZ",
        ] {
            assert!(is_plausible_call(f), "{f} is a real callsign");
        }
    }

    /// The other half: fields that no allocation could produce. The
    /// last three are CRC survivors observed on `qso3_busy.wav`.
    #[test]
    fn plausible_call_rejects_what_no_allocation_produces() {
        for f in [
            "",
            "NFW/0811",
            "ABCDEF",
            "GHIJKL",
            "294TOW/R",
            "G47OXF",
            "HVA1DPFV3L",
        ] {
            assert!(!is_plausible_call(f), "{f:?} must be refused");
        }
    }

    /// `/P` and `/R` ride as the ipa/ipb flag; any `/P` makes it i3=2
    /// (`packjt77.f90:1176-1193`). A portable activator's whole
    /// exchange depends on this — `pack77` used to refuse every one.
    #[test]
    fn pack77_suffixed_calls_set_the_flag_and_i3() {
        let i3 = |m: &[u8; 77]| read_bits(m, 74, 3);
        for (c1, c2, rpt, want_i3, ipa, ipb) in [
            ("CQ SOTA", "JL1NIE/P", "PM95", 2, 0, 1),
            ("W1AW", "JL1NIE/P", "-07", 2, 0, 1),
            ("JL1NIE/P", "W1AW", "R-12", 2, 1, 0),
            ("W1AW", "JL1NIE/P", "RR73", 2, 0, 1),
            ("G4ABC/P", "PA3XYZ/P", "JO22", 2, 1, 1),
            ("W9XYZ/R", "K1ABC", "FN42", 1, 1, 0),
            ("CQ", "K1ABC/R", "FN42", 1, 0, 1),
            ("W1AW", "JL1NIE", "-07", 1, 0, 0),
        ] {
            let m = pack77(c1, c2, rpt).unwrap_or_else(|| panic!("{c1} {c2} {rpt}"));
            assert_eq!(
                (i3(&m), m[28], m[57]),
                (want_i3, ipa, ipb),
                "{c1} {c2} {rpt}"
            );
            assert_eq!(unpack77(&m).unwrap(), format!("{c1} {c2} {rpt}"));
        }
        // Nothing to split in a bare short call or a CQ token.
        assert_eq!(split_rp_suffix("CQ"), ("CQ", false, false));
        assert_eq!(split_rp_suffix("K1/P"), ("K1/P", false, false));
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
            is_plausible_payload_with_hash(&msg, &ht),
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
        assert!(is_plausible_payload(&msg));
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
    /// One bit per byte, MSB first — the layout `read_bits` reads.
    fn bits77(spec: &[(usize, usize, u32)]) -> [u8; 77] {
        let mut m = [0u8; 77];
        for &(start, len, v) in spec {
            for i in 0..len {
                m[start + i] = ((v >> (len - 1 - i)) & 1) as u8;
            }
        }
        m
    }

    /// A real callsign token, safely past the hash range.
    fn call28() -> u32 {
        pack28("JA1ABC").expect("JA1ABC packs")
    }

    /// `packjt77.f90:318,320` — a DXpedition callsign field in the
    /// `DE`/`QRZ`/`CQ` token range is not a message.
    ///
    /// These pin that the port *fires*. Tier B proves it costs no
    /// golden decode; without these, a check that silently never ran
    /// would look exactly the same.
    #[test]
    fn dxpedition_rejects_a_token_in_a_callsign_field() {
        let good = bits77(&[
            (0, 28, call28()),
            (28, 28, call28()),
            (66, 5, 20),
            (71, 3, 1),
        ]);
        assert!(unpack77(&good).is_some(), "control must decode");
        for (name, a, b) in [
            ("DE", 0, call28()),
            ("QRZ", 1, call28()),
            ("CQ", 2, call28()),
        ] {
            let m = bits77(&[(0, 28, a), (28, 28, b), (66, 5, 20), (71, 3, 1)]);
            assert!(
                unpack77(&m).is_none(),
                "DXpedition call1 = {name} must be refused"
            );
        }
        let m = bits77(&[(0, 28, call28()), (28, 28, 2), (66, 5, 20), (71, 3, 1)]);
        assert!(
            unpack77(&m).is_none(),
            "DXpedition call2 = CQ must be refused"
        );
    }

    /// `packjt77.f90:343,345` — the same two checks for ARRL Field Day.
    #[test]
    fn field_day_rejects_a_token_in_a_callsign_field() {
        for n3 in [3u32, 4] {
            // `isec = 11` ("EMA") — any value in `1..=86` will do, but
            // it can no longer be left at 0: `packjt77.f90:338` refuses
            // that, so the control needs a real section.
            let good = bits77(&[
                (0, 28, call28()),
                (28, 28, call28()),
                (64, 7, 11),
                (71, 3, n3),
            ]);
            assert!(unpack77(&good).is_some(), "control must decode (n3={n3})");
            let m = bits77(&[(0, 28, 0), (28, 28, call28()), (64, 7, 11), (71, 3, n3)]);
            assert!(
                unpack77(&m).is_none(),
                "Field Day call1 = DE must be refused"
            );
        }
    }

    /// `packjt77.f90:337-357` — the Field Day body, which was a bare
    /// `[FD]` marker until the exchange fields were ported.
    #[test]
    fn field_day_decodes_the_exchange() {
        let call = pack28("JA1ABC").expect("JA1ABC packs");
        let dx = pack28("3Y0Z").expect("3Y0Z packs");
        // n3=3, R=0, intx=5 (=> ntx 6), class=0 ('A'), isec=11 ("EMA").
        let m = bits77(&[
            (0, 28, call),
            (28, 28, dx),
            (57, 4, 5),
            (61, 3, 0),
            (64, 7, 11),
            (71, 3, 3),
        ]);
        assert_eq!(unpack77(&m).as_deref(), Some("JA1ABC 3Y0Z 6A EMA"));

        // n3=4 is the same layout with +16 transmitters; R=1 inserts the
        // acknowledgement, class=7 is 'H', isec=84 is the "DX" catch-all.
        let m = bits77(&[
            (0, 28, call),
            (28, 28, dx),
            (56, 1, 1),
            (57, 4, 15),
            (61, 3, 7),
            (64, 7, 84),
            (71, 3, 4),
        ]);
        assert_eq!(unpack77(&m).as_deref(), Some("JA1ABC 3Y0Z R 32H DX"));
    }

    /// Inverse of [`to_grid6`] for the tests above.
    fn grid6_index(g: &str) -> u32 {
        let b = g.as_bytes();
        let j = |i: usize, base: u8| (b[i] - base) as u32;
        j(0, b'A') * 18 * 10 * 10 * 24 * 24
            + j(1, b'A') * 10 * 10 * 24 * 24
            + j(2, b'0') * 10 * 24 * 24
            + j(3, b'0') * 24 * 24
            + j(4, b'A') * 24
            + j(5, b'A')
    }

    /// `packjt77.f90:593-610` — the EU VHF contest message, which the
    /// outer `match` used to drop through `_ => None`.
    #[test]
    fn eu_vhf_contest_decodes_the_exchange() {
        // irpt=7 => nrs 59, iserial=3, grid "IO91NP", R set.
        let igrid6 = grid6_index("IO91NP");
        let m = bits77(&[
            (0, 12, 0x123),
            (12, 22, 0x2_3456),
            (34, 1, 1),
            (35, 3, 7),
            (38, 11, 3),
            (49, 25, igrid6),
            (74, 3, 5),
        ]);
        assert_eq!(unpack77(&m).as_deref(), Some("<...> <...> R 590003 IO91NP"));

        // `ir = 0` drops the acknowledgement; irpt=0 is RS(T) 52.
        let m = bits77(&[(35, 3, 0), (38, 11, 2047), (49, 25, igrid6), (74, 3, 5)]);
        assert_eq!(unpack77(&m).as_deref(), Some("<...> <...> 522047 IO91NP"));
    }

    /// End-to-end: a Type-5 frame whose hashes are in the table
    /// resolves to real callsigns and survives the phantom filter,
    /// which is the whole basis on which the type is accepted.
    #[test]
    fn eu_vhf_contest_resolves_hashes_from_the_table() {
        use crate::msg::hash_table::ihashcall;
        let mut ht = CallsignHashTable::new();
        ht.insert("PA3XYZ");
        let m = bits77(&[
            (0, 12, ihashcall("PA3XYZ", 12)),
            (12, 22, 0x2_3456),
            (34, 1, 1),
            (35, 3, 7),
            (38, 11, 3),
            (49, 25, grid6_index("IO91NP")),
            (74, 3, 5),
        ]);
        let text = unpack77_with_hash(&m, &ht).expect("type 5 unpacks");
        assert_eq!(text, "<PA3XYZ> <...> R 590003 IO91NP");
        assert!(
            is_plausible_payload_with_hash(&m, &ht),
            "a resolved hash is what makes it plausible: {text}"
        );
        // Without the table the same bits are indistinguishable from a
        // CRC survivor.
        let blind = unpack77(&m).expect("type 5 unpacks");
        assert_eq!(blind, "<...> <...> R 590003 IO91NP");
        assert!(!is_plausible_payload(&m));
    }

    /// `packjt77.f90:599` — the 25-bit grid field has 18_662_400 valid
    /// codes, so 44 % of it names no square and is refused.
    #[test]
    fn eu_vhf_contest_rejects_an_out_of_range_grid() {
        let g = |igrid6: u32| bits77(&[(49, 25, igrid6), (74, 3, 5)]);
        assert!(unpack77(&g(18_662_399)).is_some(), "last valid code");
        assert!(unpack77(&g(18_662_400)).is_none(), "one past the table");
        assert!(unpack77(&g(33_554_431)).is_none(), "top of the field");
    }

    /// Both of this type's callsigns are hashes, so the only evidence a
    /// message carries is whether they resolve. An unresolved pair is
    /// indistinguishable from a CRC survivor and must not pass.
    #[test]
    fn eu_vhf_contest_needs_a_resolved_hash() {
        let eu = |c1: &str, c2: &str| Wsjt77Fields::EuVhfContest {
            call1: c1.to_string(),
            call2: c2.to_string(),
            ack: true,
            exchange: "590003".to_string(),
            grid6: "IO91NP".to_string(),
        };
        assert!(!eu(UNRESOLVED_HASH, UNRESOLVED_HASH).is_plausible());
        assert!(eu("<PA3XYZ>", UNRESOLVED_HASH).is_plausible());
        assert!(eu(UNRESOLVED_HASH, "<G4ABC/P>").is_plausible());
        // The exchange and grid need no check here: `irpt` is 3 bits so
        // the RS(T) is always 52..=59, the serial is an 11-bit field,
        // and `to_grid6` already refused 44 % of the grid field during
        // unpacking. Re-testing them was what the text rule did.
    }

    /// The DXpedition body was already ported; what was missing was
    /// anyone looking at it. The verdict of the day short-circuited on
    /// the literal `RR73;` in the rendered string, so both callsign
    /// fields went unchecked.
    #[test]
    fn dxpedition_is_judged_on_its_callsigns() {
        let dx = |c1: &str, c2: &str, c3: &str| Wsjt77Fields::DxPedition {
            call1: c1.to_string(),
            call2: c2.to_string(),
            call3: c3.to_string(),
            report: "-11".to_string(),
        };
        assert!(dx("K1ABC", "W9XYZ", "<KH1/KH7Z>").is_plausible());
        assert!(dx("K1ABC", "W9XYZ", UNRESOLVED_HASH).is_plausible());
        assert!(!dx("NFW/0811", "W9XYZ", UNRESOLVED_HASH).is_plausible());
        assert!(!dx("K1ABC", "NFW/0811", UNRESOLVED_HASH).is_plausible());
    }

    /// `packjt77.f90:616` — a CQ names no station, so the station field
    /// cannot be a hash. Upstream tests the assembled message's first
    /// four characters; this crate did too, and the RTTY Roundup's
    /// optional `TU; ` prefix shifted the message far enough to defeat
    /// it. Asking the callsign *fields* has no such blind spot.
    #[test]
    fn a_cq_to_a_hash_is_refused_even_behind_a_prefix() {
        // i3=3, itu=1 (the `TU; ` prefix), call1 = the CQ token,
        // call2 = a 28-bit value inside the 22-bit hash range, and an
        // exchange that is otherwise valid.
        let hashed = NTOKENS + 1;
        let m = bits77(&[
            (0, 1, 1),
            (1, 28, 2),
            (29, 28, hashed),
            (58, 3, 7),
            (61, 13, 8005),
            (74, 3, 3),
        ]);
        assert_eq!(
            unpack77(&m),
            None,
            "a `TU; CQ <...>` message must be refused"
        );

        // Same message without the prefix, to show the refusal is
        // about the fields rather than about `TU;`.
        let m = bits77(&[
            (1, 28, 2),
            (29, 28, hashed),
            (58, 3, 7),
            (61, 13, 8005),
            (74, 3, 3),
        ]);
        assert_eq!(unpack77(&m), None);
    }

    /// `packjt77.f90:532-551` — the RTTY Roundup exchange is built only
    /// inside the `imult` (8001..=8171) and `nserial` (1..=7999) range
    /// arms. 22 of the 8192 codes fall outside both; upstream leaves
    /// the message blank there, and this refuses it.
    #[test]
    fn rtty_roundup_refuses_an_out_of_range_exchange() {
        let rr = |nexch: u32| {
            bits77(&[
                (1, 28, call28()),
                (29, 28, call28()),
                (61, 13, nexch),
                (74, 3, 3),
            ])
        };
        for nexch in [0, 8000, 8172, 8191] {
            assert!(
                unpack77(&rr(nexch)).is_none(),
                "nexch = {nexch} is unusable"
            );
        }
        for nexch in [1, 7999, 8001, 8171] {
            assert!(
                unpack77(&rr(nexch)).is_some(),
                "nexch = {nexch} is in range"
            );
        }
    }

    /// `packjt77.f90:338` — `isec` indexes an 86-entry table from a
    /// 7-bit field, so 42 of its 128 codes name no ARRL section. This
    /// was the last unported check of the four the type carries.
    #[test]
    fn field_day_rejects_an_out_of_range_section() {
        let fd = |isec: u32| {
            bits77(&[
                (0, 28, call28()),
                (28, 28, call28()),
                (64, 7, isec),
                (71, 3, 3),
            ])
        };
        assert!(unpack77(&fd(0)).is_none(), "isec = 0 is below the table");
        assert!(unpack77(&fd(1)).is_some(), "isec = 1 is \"AB\"");
        assert!(unpack77(&fd(86)).is_some(), "isec = 86 is \"NB\"");
        for isec in 87..128 {
            assert!(
                unpack77(&fd(isec)).is_none(),
                "isec = {isec} names no section"
            );
        }
    }

    /// The Field Day exchange has to survive
    /// [`Wsjt77Fields::is_plausible`] now that it no longer carries a
    /// `[FD]` marker to short-circuit on — and a garbage callsign in
    /// it has to not.
    #[test]
    fn field_day_is_judged_on_its_callsigns() {
        let fd = |c1: &str, c2: &str| Wsjt77Fields::FieldDay {
            call1: c1.to_string(),
            call2: c2.to_string(),
            ack: false,
            transmitters: 6,
            class: 'A',
            section: "EMA",
        };
        assert!(fd("JA1ABC", "3Y0Z").is_plausible());
        assert!(fd(UNRESOLVED_HASH, "3Y0Z").is_plausible());
        // Same shape, but the first token is not a plausible callsign:
        // the old marker path returned `true` without looking.
        assert!(!fd("NFW/0811", "3Y0Z").is_plausible());
        // Right shape, wrong section / class / count.
        // `ZZZ` / `6J` / `33A` used to be tested here. They are now
        // unrepresentable: `section` comes from the 86-entry table,
        // `class` from `b'A' + nclass` over 3 bits, and `transmitters`
        // from a 4-bit field plus the n3=4 offset. The text rule was
        // re-checking ranges `unpack77` had already enforced.
    }

    /// `packjt77.f90:494,509` — a CQ cannot acknowledge and cannot
    /// The 15-bit grid/report field `pack77` writes, against what WSJT-X puts on
    /// the air: each value is the field of `ft8sim` (3.2.0-rc1) transmitting that
    /// message, decoded (#464). `RR73` is the grid RR73, and -50..-31 dB reports
    /// wrap by 101; this crate had `MAXGRID4 + 3` and a -35..-31 collision with
    /// the bare / RRR / RR73 / 73 values.
    #[test]
    fn r_grid_packs_with_ir_set() {
        let bits = pack77("K1ABC", "W9XYZ", "R EN37").unwrap();
        assert_eq!(unpack77(&bits).as_deref(), Some("K1ABC W9XYZ R EN37"));
        assert_eq!(bits[58], 1, "ir");
        let plain = pack77("K1ABC", "W9XYZ", "EN37").unwrap();
        assert_eq!(bits[59..74], plain[59..74]);
    }

    #[test]
    fn report_field_matches_wsjtx() {
        let g15 = |m: &[u8; 77]| m[59..74].iter().fold(0u32, |a, &b| a * 2 + b as u32);
        for (report, field, ir) in [
            ("RR73", 32373, 0),
            ("RRR", 32402, 0),
            ("73", 32404, 0),
            ("", 32401, 0),
            ("-33", 32503, 0),
            ("R-33", 32503, 1),
            ("-50", 32486, 0),
            ("-35", 32501, 0),
            ("-31", 32505, 0),
            ("+49", 32484, 0),
            ("EN37", 8537, 0),
        ] {
            let m = pack77("K1ABC", "W9XYZ", report).unwrap();
            assert_eq!(g15(&m), field, "{report}");
            assert_eq!(m[58], ir, "{report} ir");
            let text = unpack77(&m).unwrap();
            let want = if report.is_empty() {
                "K1ABC W9XYZ".to_string()
            } else {
                format!("K1ABC W9XYZ {report}")
            };
            assert_eq!(text, want);
        }
    }

    /// carry a report.
    #[test]
    fn cq_rejects_r_and_any_report() {
        const CQ: u32 = 2;
        let grid = pack_grid4("PM95").expect("PM95 packs");
        // Control: plain `CQ JA1ABC PM95`.
        let ok = bits77(&[(0, 28, CQ), (29, 28, call28()), (59, 15, grid), (74, 3, 1)]);
        assert_eq!(unpack77(&ok).as_deref(), Some("CQ JA1ABC PM95"));
        // `CQ ... R PM95` — ir = 1 on the grid path.
        let r = bits77(&[
            (0, 28, CQ),
            (29, 28, call28()),
            (58, 1, 1),
            (59, 15, grid),
            (74, 3, 1),
        ]);
        assert!(unpack77(&r).is_none(), "CQ with R must be refused");
        // Reports: irpt 2..4 are RRR / RR73 / 73, 5+ is a signal report.
        // irpt 1 (the bare form) stays legal.
        let bare = bits77(&[
            (0, 28, CQ),
            (29, 28, call28()),
            (59, 15, MAX_GRID4 + 1),
            (74, 3, 1),
        ]);
        assert_eq!(unpack77(&bare).as_deref(), Some("CQ JA1ABC"));
        for irpt in [2u32, 3, 4, 40] {
            let m = bits77(&[
                (0, 28, CQ),
                (29, 28, call28()),
                (59, 15, MAX_GRID4 + irpt),
                (74, 3, 1),
            ]);
            assert!(
                unpack77(&m).is_none(),
                "CQ with irpt={irpt} must be refused"
            );
        }
    }

    /// One valid message of **every type `unpack77` produces**, through
    /// the codec verdict.
    ///
    /// A filter that refuses a whole message type is not strict, it is
    /// broken: the type's traffic is lost outright, and no amount of
    /// phantom rejection pays for that. This is the inventory.
    ///
    /// It exists because ARRL RTTY Roundup was in exactly that state and
    /// nobody noticed — FT8 was the only protocol applying the filter,
    /// and the FT8 recordings this suite has carry no contest exchange.
    /// It surfaced the moment FT4 was wired up (issue #383 step 5):
    /// half of FT4's own golden recording is RTTY Roundup, it being a
    /// contest mode.
    #[test]
    fn every_message_type_survives_its_own_filter() {
        let call = pack28("JA1ABC").expect("packs");
        let dx = pack28("3Y0Z").expect("packs");
        let grid = pack_grid4("PM95").expect("packs");

        let mut cases: Vec<(&str, [u8; 77])> = alloc::vec![
            // 0.0 free text
            ("free text", pack77_free_text("HELLO WORLD").expect("packs")),
            // 0.1 DXpedition: n5 = 19 => report -8
            (
                "DXpedition",
                bits77(&[(0, 28, call), (28, 28, dx), (66, 5, 19), (71, 3, 1)]),
            ),
        ];
        // 0.3 ARRL Field Day
        cases.push((
            "Field Day",
            bits77(&[
                (0, 28, call),
                (28, 28, dx),
                (57, 4, 5),
                (64, 7, 11),
                (71, 3, 3),
            ]),
        ));
        // 0.5 telemetry
        cases.push((
            "telemetry",
            bits77(&[
                (0, 23, 0x12_3456),
                (23, 24, 0x78_9ABC),
                (47, 24, 0xDE_F012),
                (71, 3, 5),
            ]),
        ));
        // 0.6 WSPR type 1: CALL GRID4 DBM
        cases.push((
            "WSPR type 1",
            bits77(&[(0, 28, call), (28, 15, grid), (43, 5, 10), (71, 3, 6)]),
        ));
        // 1 standard, with a grid
        cases.push((
            "standard",
            bits77(&[(0, 28, call), (29, 28, dx), (58, 15, grid), (74, 3, 1)]),
        ));
        // 3 ARRL RTTY Roundup: irpt=7 => 599, nexch=8005 => a state
        cases.push((
            "RTTY Roundup",
            bits77(&[
                (1, 28, call),
                (29, 28, dx),
                (58, 3, 7),
                (61, 13, 8005),
                (74, 3, 3),
            ]),
        ));
        // 4 one nonstandard call + one hashed
        cases.push((
            "nonstandard call",
            pack77_type4("JL1NIE/1", "JA1ABC", "RR73", false).expect("packs"),
        ));
        // Type 5 carries two hashes and nothing else, so it is
        // plausible only once one of them resolves — see
        // `eu_vhf_contest_plausible`. Give it a table, as a real
        // receiver would have by the time the exchange arrives.
        let mut ht = CallsignHashTable::new();
        ht.insert("PA3XYZ");
        cases.push((
            "EU VHF contest",
            bits77(&[
                (0, 12, crate::msg::hash_table::ihashcall("PA3XYZ", 12)),
                (12, 22, 0x2_3456),
                (34, 1, 1),
                (35, 3, 7),
                (38, 11, 3),
                (49, 25, grid6_index("IO91NP")),
                (74, 3, 5),
            ]),
        ));

        let mut refused = Vec::new();
        for (name, m) in &cases {
            let text = unpack77_with_hash(m, &ht)
                .unwrap_or_else(|| panic!("{name} must unpack — fix the decoder, not the filter"));
            // The shipped verdict is `is_plausible_payload`, which
            // decodes once and asks `Wsjt77Fields::is_plausible`.
            if !is_plausible_payload_with_hash(m, &ht) {
                refused.push(alloc::format!("{name}: {text:?}"));
            }
        }
        assert!(
            refused.is_empty(),
            "the codec verdict refuses whole message types:\n  {}",
            refused.join("\n  ")
        );

        // And the one that legitimately depends on the table: without
        // it the EU VHF exchange names neither station, and is
        // indistinguishable from a CRC survivor. See
        // `is_plausible_payload_with_hash`.
        let eu = cases.last().expect("EU VHF case").1;
        assert!(
            !is_plausible_payload(&eu),
            "type 5 must stay refused while no table reaches the filter"
        );
    }

    /// How much of the phantom population each stage removes, and what
    /// a **per-mode** `(i3, n3)` pre-gate would add on top.
    ///
    /// A CRC false positive is a codeword the decoder converged on that
    /// is not the transmitted one, so its 77 information bits are
    /// effectively uniform — which makes uniform random payloads the
    /// right model for the population both `unpack77`'s per-type
    /// validity checks and the codec verdict exist to reject.
    ///
    /// The `(i3, n3)` breakdown is here because WSJT-X does **not**
    /// treat the acceptance surface as mode-independent, even though
    /// `unpack77` itself is shared:
    ///
    /// - `msk144decodeframe.f90:103` rejects `i3=0 & n3∈{1,3,4,>5}`,
    ///   `i3=3` and `i3>5` *before* calling `unpack77` — DXpedition,
    ///   ARRL Field Day, WSPR-type and RTTY Roundup exchanges, none of
    ///   which are ever sent on a meteor-scatter link. Ported as
    ///   `msk144::frame_decode::n3_i3_plausible`.
    /// - `ft8b.f90:510-511` rejects `i3>5`, `i3=0 & n3>6` and
    ///   `i3=0 & n3=2` — but every one of those is already refused by
    ///   `unpack77` itself (`packjt77.f90:613`, `:457`, `:330`), so it
    ///   removes nothing the shared layer had not already removed.
    /// - `ft4_decode.f90:432` and `fst4_decode.f90:489` gate on
    ///   nothing but the all-zero codeword.
    ///
    /// So the question this answers is: for a mode that transmits only
    /// a subset of the 77-bit message styles, how much of the surviving
    /// phantom population is reachable *only* through the styles it
    /// never sends?
    ///
    /// Run it against this commit and against the tree before the
    /// `unpack77` port to see what the port moved:
    ///
    /// ```sh
    /// cargo test -p mfsk-core --features full,internal-testing --release \
    ///     --lib phantom_survival -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "diagnostic — phantom survival through unpack77 and the codec verdict"]
    fn phantom_survival_rates() {
        const N: usize = 2_000_000;
        // A deterministic LCG, so the number is comparable across
        // commits without a dev-dependency.
        let mut x: u64 = 0x2026_0920_0000_0001;
        let mut next_bit = || {
            x = x
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((x >> 33) & 1) as u8
        };

        /// `ft8b.f90:510-511`.
        fn ft8_gate(i3: u8, n3: u8) -> bool {
            !(i3 > 5 || (i3 == 0 && n3 > 6) || (i3 == 0 && n3 == 2))
        }
        /// `msk144decodeframe.f90:103`.
        fn msk144_gate(i3: u8, n3: u8) -> bool {
            !((i3 == 0 && (n3 == 1 || n3 == 3 || n3 == 4 || n3 > 5)) || i3 == 3 || i3 > 5)
        }
        /// FT4 / FST4: no `(i3, n3)` pre-gate at all.
        fn no_gate(_i3: u8, _n3: u8) -> bool {
            true
        }
        /// Named alias purely to keep `clippy::type_complexity` quiet
        /// at the array below.
        type Gate = fn(u8, u8) -> bool;
        let gates: [(&str, Gate); 3] = [
            ("none (FT4/FST4)", no_gate),
            ("ft8b.f90:510", ft8_gate),
            ("msk144decodeframe:103", msk144_gate),
        ];

        let (mut unpacked, mut plausible) = (0usize, 0usize);
        // Survivors of both stages, counted under each pre-gate.
        let mut survived_gated = [0usize; 3];
        // Per `(i3, n3)` cell, keyed `i3 * 8 + n3`. Only `i3 = 0`
        // varies in `n3` (every other type reuses those three bits as
        // payload), so the other rows are collapsed on print.
        let mut cell_unpacked = [0usize; 64];
        let mut cell_plausible = [0usize; 64];

        for _ in 0..N {
            let mut m = [0u8; 77];
            for b in m.iter_mut() {
                *b = next_bit();
            }
            let n3 = read_bits(&m, 71, 3) as u8;
            let i3 = read_bits(&m, 74, 3) as u8;
            if unpack77(&m).is_none() {
                continue;
            }
            unpacked += 1;
            let cell = (i3 as usize) * 8 + n3 as usize;
            cell_unpacked[cell] += 1;
            if !is_plausible_payload(&m) {
                continue;
            }
            plausible += 1;
            cell_plausible[cell] += 1;
            for (k, (_, gate)) in gates.iter().enumerate() {
                if gate(i3, n3) {
                    survived_gated[k] += 1;
                }
            }
        }

        let pct = |a: usize, b: usize| {
            if b == 0 {
                0.0
            } else {
                100.0 * a as f64 / b as f64
            }
        };
        println!("  random 77-bit payloads: {N}");
        println!(
            "  unpack77 accepts          {unpacked:>9}  ({:.3} % of payloads)",
            pct(unpacked, N)
        );
        println!(
            "  the codec verdict keeps   {plausible:>9}  ({:.3} % of those unpack77 accepted)",
            pct(plausible, unpacked)
        );
        println!(
            "  surviving both            {plausible:>9}  ({:.4} % of payloads)",
            pct(plausible, N)
        );

        println!("\n  what each mode's own (i3,n3) pre-gate removes from those survivors:");
        println!("  {:<24} {:>9} {:>9}", "pre-gate", "survive", "vs none");
        for (k, (name, _)) in gates.iter().enumerate() {
            println!(
                "  {name:<24} {:>9} {:>8.1} %",
                survived_gated[k],
                pct(survived_gated[k], survived_gated[0])
            );
        }

        // `n3` is only a type selector for `i3 = 0`; for every other
        // type those three bits carry payload, so that row is printed
        // once with `n3` collapsed rather than as eight meaningless
        // sub-rows.
        println!("\n  i3   n3   unpack77     kept   kept%   style");
        let row = |i3: u8, n3: Option<u8>, u: usize, p: usize, style: &str| {
            let n3s = match n3 {
                Some(v) => alloc::format!("{v}"),
                None => "*".into(),
            };
            println!(
                "  {i3:<4} {n3s:<4} {u:>8} {p:>8} {:>6.1}   {style}",
                pct(p, u)
            );
        };
        for n3 in 0u8..8 {
            let cell = n3 as usize;
            if cell_unpacked[cell] == 0 {
                continue;
            }
            let style = match n3 {
                0 => "free text",
                1 => "DXpedition",
                3 | 4 => "ARRL Field Day",
                5 => "telemetry",
                6 => "WSPR type 1/2/3",
                _ => "?",
            };
            row(
                0,
                Some(n3),
                cell_unpacked[cell],
                cell_plausible[cell],
                style,
            );
        }
        for i3 in 1u8..8 {
            let base = (i3 as usize) * 8;
            let u: usize = cell_unpacked[base..base + 8].iter().sum();
            if u == 0 {
                continue;
            }
            let p: usize = cell_plausible[base..base + 8].iter().sum();
            let style = match i3 {
                1 => "standard",
                2 => "EU VHF /P",
                3 => "ARRL RTTY Roundup",
                4 => "nonstandard call",
                5 => "EU VHF contest",
                _ => "?",
            };
            row(i3, None, u, p, style);
        }
    }

    /// `packjt77.f90:360` — telemetry, 71 bits as 18 hex digits.
    /// Returned `None` before this was ported: a dropped decode.
    #[test]
    fn telemetry_decodes_as_eighteen_hex_digits() {
        let m = bits77(&[
            (0, 23, 0x12_3456),
            (23, 24, 0x78_9ABC),
            (47, 24, 0xDE_F012),
            (71, 3, 5),
        ]);
        assert_eq!(unpack77(&m).as_deref(), Some("123456789ABCDEF012"));
        // Leading zeros are blanked, as upstream's loop does — *all* of
        // them, across the group boundary: the three fields render as
        // `000000` `0000AB` `CD0000` and ten zeros come off the front.
        let z = bits77(&[(23, 24, 0x00_00AB), (47, 24, 0xCD_0000), (71, 3, 5)]);
        assert_eq!(unpack77(&z).as_deref(), Some("ABCD0000"));
    }

    /// `packjt77.f90:387` — WSPR type 1, `CALL GRID4 DBM`.
    #[test]
    fn wspr_type1_decodes_call_grid_power() {
        let grid = pack_grid4("PM95").expect("PM95 packs");
        // idbm raw 6 -> round(6*10/3) = 20 dBm. itype 1 needs j49 = j50 = 0.
        let m = bits77(&[(0, 28, call28()), (28, 15, grid), (43, 5, 6), (71, 3, 6)]);
        assert_eq!(unpack77(&m).as_deref(), Some("JA1ABC PM95 20"));
        // `idbm` out of the 0..60 range is upstream's rejection, and the
        // reason a random 5-bit field fails instead of rendering.
        let bad = bits77(&[(0, 28, call28()), (28, 15, grid), (43, 5, 31), (71, 3, 6)]);
        assert!(
            unpack77(&bad).is_none(),
            "idbm 31 -> 103 dBm must be refused"
        );
    }

    /// `packjt77.f90:403` — WSPR type 2, base-36 prefix or suffix.
    #[test]
    fn wspr_type2_decodes_prefix_and_suffix() {
        // "ABC" = 10*36^2 + 11*36 + 12. itype 2 needs j50 = 1.
        let pfx = bits77(&[
            (0, 28, call28()),
            (28, 16, 13_368),
            (44, 5, 6),
            (49, 1, 1),
            (71, 3, 6),
        ]);
        assert_eq!(unpack77(&pfx).as_deref(), Some("ABC/JA1ABC 20"));
        // Suffix form: npfx - NZZZ = 10 -> 'A'.
        let sfx = bits77(&[
            (0, 28, call28()),
            (28, 16, 46_656 + 10),
            (44, 5, 6),
            (49, 1, 1),
            (71, 3, 6),
        ]);
        assert_eq!(unpack77(&sfx).as_deref(), Some("JA1ABC/A 20"));
    }

    /// `packjt77.f90:444` — WSPR type 3, hashed call plus a 6-char grid.
    #[test]
    fn wspr_type3_decodes_hashed_call_and_grid() {
        // PM95 in the 25-bit grid field, with the j5 = j6 = 24 sentinel
        // that upstream uses for a four-character grid.
        let igrid6 = 15 * 1_125_000 + 12 * 62_500 + 9 * 6_250 + 5 * 625 + 24 * 25 + 24;
        // itype 3 needs j50 = 0, j49 = 1, j48 = 0.
        let m = bits77(&[(0, 22, 1234), (22, 25, igrid6), (48, 1, 1), (71, 3, 6)]);
        assert_eq!(unpack77(&m).as_deref(), Some("<...> PM95"));
    }

    /// `packjt77.f90:616` — nothing can have introduced the hash a
    /// `CQ <...>` would need, so the shape is never a message.
    #[test]
    fn cq_rejects_a_hashed_second_call() {
        let hashed = NTOKENS + 7;
        let grid = pack_grid4("PM95").expect("PM95 packs");
        let m = bits77(&[(0, 28, 2), (29, 28, hashed), (59, 15, grid), (74, 3, 1)]);
        assert!(
            unpack77(&m).is_none(),
            "CQ <...> must be refused; got {:?}",
            unpack77(&m)
        );
    }
}
