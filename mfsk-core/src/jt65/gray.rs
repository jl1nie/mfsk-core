//! 6-bit Gray code for JT65 symbol mapping.
//!
//! Forward Gray: `g = n ^ (n >> 1)` — a single XOR.
//! Inverse Gray: successive right-shift XORs, matching
//! WSJT-X `igray.c` for the `idir < 0` branch.

/// Gray-encode a 6-bit symbol (0..=63).
#[inline]
pub fn gray6(n: u8) -> u8 {
    (n ^ (n >> 1)) & 0x3f
}

/// Inverse of [`gray6`].
#[inline]
pub fn inv_gray6(g: u8) -> u8 {
    let mut n = g & 0x3f;
    n ^= n >> 1;
    n ^= n >> 2;
    n ^= n >> 4;
    n & 0x3f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_is_bijective_on_6_bits() {
        let mut seen = [false; 64];
        for n in 0u8..64 {
            let g = gray6(n);
            assert!(!seen[g as usize], "duplicate Gray for n={n}");
            seen[g as usize] = true;
            assert_eq!(inv_gray6(g), n, "inverse roundtrip failed for n={n}");
        }
        assert!(seen.iter().all(|&b| b));
    }
}
