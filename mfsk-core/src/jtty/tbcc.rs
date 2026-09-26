//! The JTTY convolutional code: tail-biting, rate 1/2, K = 10.
//!
//! Ported from WSJT-X `lib/jtty/tbcc.f90` (`tbcc_encode`) and
//! `jtty_tbcc_list_decoder.f90` (`jtty_tbcc_transition`), with the code
//! parameters of `JTTY_TBCC_PROFILE_1167_1545_80F` in
//! `jtty_tbcc_code_profile.f90`.
//!
//! 46 information bits go in, one 4-FSK tone per bit comes out (two coded
//! bits, Gray-mapped). The code is tail-biting: there are no tail bits; the
//! encoder starts in the state the message itself ends in, so the trellis
//! closes on its start state. The decoder (a later phase) exploits that.

use super::INFO_BITS;

/// Encoder memory (constraint length − 1); the state is the last 9 input bits.
pub const MEMORY: u32 = 9;
/// Number of trellis states, `2^MEMORY`.
pub const STATES: usize = 1 << MEMORY;
/// Generator polynomials (octal 1167 and 1545), applied to the 10-bit register
/// `(state << 1) | input`.
pub const GENERATORS: [u32; 2] = [0o1167, 0o1545];

const REGISTER_MASK: u32 = (1 << (MEMORY + 1)) - 1;
const STATE_MASK: u32 = (1 << MEMORY) - 1;

/// One trellis step: from `state` with input bit `input`, the next state and
/// the tone emitted.
///
/// The tone is the Gray mapping of the two coded bits `(b0, b1)` from the two
/// generators: `00 → 0`, `01 → 1`, `11 → 2`, `10 → 3`, i.e. `2·b0 + (b0 ⊕ b1)`.
pub fn transition(state: u32, input: u8) -> (u32, u8) {
    let register = ((state << 1) | u32::from(input & 1)) & REGISTER_MASK;
    let b0 = (register & GENERATORS[0]).count_ones() & 1;
    let b1 = (register & GENERATORS[1]).count_ones() & 1;
    ((register & STATE_MASK), (2 * b0 + (b0 ^ b1)) as u8)
}

/// The state a tail-biting codeword of `info` starts (and ends) in: the last
/// [`MEMORY`] information bits, the earliest of them most significant.
pub fn tailbiting_state(info: &[u8; INFO_BITS]) -> u32 {
    info[INFO_BITS - MEMORY as usize..]
        .iter()
        .fold(0u32, |s, &b| ((s << 1) | u32::from(b & 1)) & STATE_MASK)
}

/// Encode 46 information bits (payload + CRC, see [`super::crc::append`]) to
/// 46 data tones, each 0‥3.
pub fn encode(info: &[u8; INFO_BITS]) -> [u8; INFO_BITS] {
    let start = tailbiting_state(info);
    let mut state = start;
    let mut tones = [0u8; INFO_BITS];
    tones.iter_mut().zip(info).for_each(|(tone, &bit)| {
        let (next, t) = transition(state, bit);
        *tone = t;
        state = next;
    });
    debug_assert_eq!(state, start, "a tail-biting path must close");
    tones
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_zero_input_is_all_zero_tones() {
        assert_eq!(encode(&[0u8; INFO_BITS]), [0u8; INFO_BITS]);
    }

    #[test]
    fn path_closes_for_arbitrary_input() {
        for seed in 1u64..300 {
            let mut info = [0u8; INFO_BITS];
            let mut x = seed.wrapping_mul(0x2545_F491_4F6C_DD1D) | 1;
            for b in info.iter_mut() {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                *b = (x & 1) as u8;
            }
            // encode() debug-asserts closure; check it independently too.
            let mut state = tailbiting_state(&info);
            let start = state;
            for &bit in &info {
                state = transition(state, bit).0;
            }
            assert_eq!(state, start, "seed {seed}");
            let tones = encode(&info);
            assert!(tones.iter().all(|&t| t < 4));
        }
    }

    #[test]
    fn gray_map_of_the_two_coded_bits() {
        // Pick registers whose (b0, b1) cover all four combinations.
        let mut seen = [false; 4];
        for state in 0..STATES as u32 {
            for input in 0..2u8 {
                let register = ((state << 1) | u32::from(input)) & REGISTER_MASK;
                let b0 = (register & GENERATORS[0]).count_ones() & 1;
                let b1 = (register & GENERATORS[1]).count_ones() & 1;
                let expect = match (b0, b1) {
                    (0, 0) => 0,
                    (0, 1) => 1,
                    (1, 1) => 2,
                    _ => 3,
                };
                let (_, tone) = transition(state, input);
                assert_eq!(tone, expect);
                seen[tone as usize] = true;
            }
        }
        assert!(seen.iter().all(|&s| s));
    }

    #[test]
    fn each_input_bit_reaches_two_distinct_states() {
        // From any state the two inputs lead to different states (the trellis
        // is a shift register), and the same tone-pair structure repeats.
        for state in 0..STATES as u32 {
            let a = transition(state, 0).0;
            let b = transition(state, 1).0;
            assert_ne!(a, b);
            assert_eq!(a >> 1, b >> 1);
        }
    }
}
