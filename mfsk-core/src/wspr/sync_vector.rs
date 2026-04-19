//! WSPR 162-bit synchronisation vector (`pr3` / `npr3` in WSJT-X Fortran
//! and C sources). The LSB of every transmitted 4-FSK symbol reproduces
//! one bit of this vector; the receiver recovers frame timing and
//! frequency by correlating per-symbol LSBs against it.
//!
//! Source: `lib/wsprd/wsprsim_utils.c::get_wspr_channel_symbols::pr3`
//! (and the identical `npr3` in Fortran). Do not modify — these 162 bits
//! are part of the protocol.

pub const WSPR_SYNC_VECTOR: [u8; 162] = [
    1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0,
    0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
    1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1,
    0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0,
    0, 0,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_is_162() {
        assert_eq!(WSPR_SYNC_VECTOR.len(), 162);
    }

    #[test]
    fn only_zeros_and_ones() {
        for &b in &WSPR_SYNC_VECTOR {
            assert!(b == 0 || b == 1);
        }
    }
}
