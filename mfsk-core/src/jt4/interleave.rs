//! JT4's 206-bit bit-reversal interleaver.
//!
//! This is the exact permutation in pinned WSJT-X
//! `lib/interleave4.f90`: walk the 8-bit integers, reverse their bits,
//! and retain destinations 0 through 205.

const FRAME: usize = 206;

#[inline]
fn reverse_8(value: u8) -> u8 {
    value.reverse_bits()
}

/// Apply the transmit permutation.
pub fn interleave(bits: &mut [u8; FRAME]) {
    let input = *bits;
    let mut output = [0u8; FRAME];
    let mut source = 0usize;
    for value in 0u16..=255 {
        let destination = reverse_8(value as u8) as usize;
        if destination < FRAME {
            output[destination] = input[source];
            source += 1;
        }
    }
    debug_assert_eq!(source, FRAME);
    *bits = output;
}

/// Remove the transmit permutation from hard bits.
pub fn deinterleave(bits: &mut [u8; FRAME]) {
    let input = *bits;
    let mut output = [0u8; FRAME];
    let mut destination = 0usize;
    for value in 0u16..=255 {
        let source = reverse_8(value as u8) as usize;
        if source < FRAME {
            output[destination] = input[source];
            destination += 1;
        }
    }
    debug_assert_eq!(destination, FRAME);
    *bits = output;
}

/// Remove the transmit permutation from soft bits.
pub fn deinterleave_llrs(llrs: &mut [f32; FRAME]) {
    let input = *llrs;
    let mut output = [0f32; FRAME];
    let mut destination = 0usize;
    for value in 0u16..=255 {
        let source = reverse_8(value as u8) as usize;
        if source < FRAME {
            output[destination] = input[source];
            destination += 1;
        }
    }
    debug_assert_eq!(destination, FRAME);
    *llrs = output;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hard_and_soft_round_trip() {
        let original = core::array::from_fn(|i| ((i * 13 + 7) & 1) as u8);
        let mut channel = original;
        interleave(&mut channel);
        assert_ne!(channel, original);

        let mut soft = channel.map(|bit| if bit == 0 { 8.0 } else { -8.0 });
        deinterleave_llrs(&mut soft);
        for (index, &bit) in original.iter().enumerate() {
            assert_eq!(soft[index] < 0.0, bit == 1);
        }

        deinterleave(&mut channel);
        assert_eq!(channel, original);
    }
}
