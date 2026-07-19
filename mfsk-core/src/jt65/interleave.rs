//! JT65 interleaver: 7×9 matrix transpose over the 63 RS symbols.
//!
//! Ported from WSJT-X `interleave63.f90`. The encoder writes the
//! linear 63-symbol array into a 7×9 matrix in row-major order and
//! reads back in column-major order (equivalently: transposes to a
//! 9×7 matrix). The decoder performs the inverse transpose.

/// Interleave 63 symbols: write as 7 rows × 9 cols, read as 9 rows ×
/// 7 cols. Invertible by [`deinterleave`].
///
/// Matches WSJT-X `interleave63(d1, idir=1)`, called at TX time
/// (`gen65.f90`, `jt65sim.f90`) to go from RS-codeword order to
/// channel order. Fortran's column-major `d1(0:6,0:8)` /
/// `d2(0:8,0:6)` gives `output[9*i+j] = input[7*j+i]`.
pub fn interleave(sym: &mut [u8; 63]) {
    let mut tmp = [0u8; 63];
    for i in 0..7 {
        for j in 0..9 {
            tmp[i * 9 + j] = sym[j * 7 + i];
        }
    }
    *sym = tmp;
}

/// Inverse of [`interleave`].
///
/// Matches WSJT-X `interleave63(d1, idir=-1)`, called at RX time
/// (`extract.f90`) to go from channel order back to RS-codeword
/// order.
pub fn deinterleave(sym: &mut [u8; 63]) {
    let mut tmp = [0u8; 63];
    for i in 0..7 {
        for j in 0..9 {
            tmp[j * 7 + i] = sym[i * 9 + j];
        }
    }
    *sym = tmp;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut s = [0u8; 63];
        for i in 0..63 {
            s[i] = i as u8;
        }
        let original = s;
        interleave(&mut s);
        assert_ne!(s, original, "permutation must change order");
        deinterleave(&mut s);
        assert_eq!(s, original, "deinterleave must invert");
    }
}
