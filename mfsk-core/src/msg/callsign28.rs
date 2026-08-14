// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared base-37/36/10/27³ callsign encoding — the scheme WSJT-X's
//! `lib/packjt.f90::packcall`/`unpackcall` implements and which is
//! called, per that file, from both `wqencode.f90`/`wqdecode.f90`
//! (the JT65/JT9 message layer) and `wsprcode/wspr_old_subs.f90`
//! (WSPR) — one upstream routine, not two.
//!
//! Extracted (2026-08-14, code-sharing audit, msg-layer sweep) from
//! two independent Rust ports that were byte-identical on this core:
//! [`super::jt72::pack_call`]/`unpack_call` and
//! [`super::wspr::pack_call`]/`unpack_call`. The two callers differ
//! on exactly one axis — which characters a slot accepts — so that's
//! the one thing left as a parameter rather than folded in:
//!
//! - JT9/JT65's own `nchar` accepts `a`-`z` as well as `A`-`Z`
//!   (defensive; every caller already uppercases first, so this
//!   never actually fires on live input, but preserving it here
//!   keeps the extraction behaviour-neutral).
//! - WSPR's `callsign_char_code` accepts `A`-`Z` only.
//!
//! Special tokens above `NBASE` (JT9/JT65's `CQ`/`QRZ`/`DE`) have no
//! WSPR analogue — WSPR Type-1 messages are always callsign+grid+
//! power, never a bare CQ token — so that layer stays in
//! [`super::jt72`] alone, checked before falling through to
//! [`pack_call28`]/[`unpack_call28`].

use alloc::string::{String, ToString};

/// 37-entry table used to render an unpacked slot back to a
/// character: digits, uppercase letters, space. Matches `c[]` in
/// WSJT-X's `unpackcall`.
pub const CALL_ALPHA: &[u8; 37] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";

/// `NBASE` in WSJT-X: `37 * 36 * 10 * 27³ = 262_177_560` — the
/// exclusive upper bound of the plain 6-character callsign encoding
/// range. Values at or above this are protocol-specific special
/// tokens or reserved; callers check against this before/instead of
/// calling [`unpack_call28`].
pub const NBASE: u32 = 37 * 36 * 10 * 27 * 27 * 27;

/// Pack a ≤6-character callsign into its base-37/36/10/27³ integer,
/// given a per-character `nchar` lookup (digit→0..9, letter→10..35,
/// space→36, anything else→`None`).
///
/// The standard layout expects the digit in position 3 (`K1ABC`) or
/// position 2 (`K9AN`, right-shifted so the digit lands at index 2).
/// Returns `None` if the callsign doesn't fit that digit-position
/// rule, exceeds 6 characters, or any character is rejected by
/// `nchar`.
pub fn pack_call28(call: &str, nchar: impl Fn(u8) -> Option<u32>) -> Option<u32> {
    let bytes = call.as_bytes();
    if bytes.is_empty() || bytes.len() > 6 {
        return None;
    }

    let mut tmp = [b' '; 6];
    if bytes.len() >= 3 && bytes[2].is_ascii_digit() {
        for (i, &b) in bytes.iter().enumerate() {
            tmp[i] = b;
        }
    } else if bytes.len() >= 2 && bytes[1].is_ascii_digit() {
        if bytes.len() > 5 {
            return None;
        }
        for (i, &b) in bytes.iter().enumerate() {
            tmp[i + 1] = b;
        }
    } else {
        return None;
    }

    let n = [
        nchar(tmp[0])?,
        nchar(tmp[1])?,
        nchar(tmp[2])?,
        nchar(tmp[3])?,
        nchar(tmp[4])?,
        nchar(tmp[5])?,
    ];
    // Slot 1: letter/digit only (space forbidden).
    if n[1] == 36 {
        return None;
    }
    // Slot 2: digit only.
    if n[2] >= 10 {
        return None;
    }
    // Slots 3..=5: letter/space only (digit forbidden).
    for v in &n[3..6] {
        if *v < 10 {
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

/// Inverse of [`pack_call28`]. Returns `None` for `ncall >= NBASE`
/// (special tokens / reserved values are the caller's concern,
/// checked before calling this).
pub fn unpack_call28(ncall: u32) -> Option<String> {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn nchar_upper_only(c: u8) -> Option<u32> {
        match c {
            b'0'..=b'9' => Some((c - b'0') as u32),
            b'A'..=b'Z' => Some((c - b'A' + 10) as u32),
            b' ' => Some(36),
            _ => None,
        }
    }

    #[test]
    fn round_trip_k1abc() {
        let n = pack_call28("K1ABC", nchar_upper_only).unwrap();
        assert_eq!(unpack_call28(n).as_deref(), Some("K1ABC"));
    }

    #[test]
    fn round_trip_digit_in_slot_2() {
        // "K9AN" — digit at position 2, gets shifted right by one.
        let n = pack_call28("K9AN", nchar_upper_only).unwrap();
        assert_eq!(unpack_call28(n).as_deref(), Some("K9AN"));
    }

    #[test]
    fn nbase_is_wsjtx_constant() {
        assert_eq!(NBASE, 262_177_560);
    }

    #[test]
    fn rejects_lowercase_when_nchar_is_upper_only() {
        assert_eq!(pack_call28("k1abc", nchar_upper_only), None);
    }
}
