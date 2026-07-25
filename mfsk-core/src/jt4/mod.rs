//! JT4: 206-symbol, 4-FSK, convolutionally coded weak-signal mode.
//!
//! Protocol authority: pinned WSJT-X `lib/jt4.f90`, `encode4.f90`,
//! `gen4.f90`, `interleave4.f90`, `decode4.f90`, and `jt4sim.f90`.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::ConvFano232Jt4;
use crate::msg::Jt72Codec;

pub mod interleave;
pub mod rx;
pub mod search;
pub mod tx;

pub use rx::{Jt4Decode, decode_aligned, demodulate_aligned};
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use tx::{
    encode_channel_symbols, synthesize_audio, synthesize_audio_len, synthesize_message,
    synthesize_standard,
};

/// Exact JT4 keying rate.
pub const JT4_BAUD: f32 = 4.375;

/// `npr(2:)` from WSJT-X `lib/jt4.f90`.
#[rustfmt::skip]
pub const JT4_SYNC_VECTOR: [u8; 206] = [
    0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0,
    0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0,
    0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1,
    0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1,
];

/// WSJT-X's normal `*` and inverted `#` sync conventions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SyncPolarity {
    Normal,
    Inverted,
}

impl SyncPolarity {
    pub fn for_standard_report(grid_or_report: &str) -> Self {
        let bytes = grid_or_report.as_bytes();
        if bytes.len() >= 2 && bytes[0] == b'-' && matches!(bytes[1], b'0'..=b'3') {
            Self::Inverted
        } else {
            Self::Normal
        }
    }
}

/// JT4 tone-spacing selection.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Jt4Submode {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

impl Jt4Submode {
    pub const ALL: [Self; 7] = [
        Self::A,
        Self::B,
        Self::C,
        Self::D,
        Self::E,
        Self::F,
        Self::G,
    ];

    pub const fn spacing_multiplier(self) -> u32 {
        match self {
            Self::A => 1,
            Self::B => 2,
            Self::C => 4,
            Self::D => 9,
            Self::E => 18,
            Self::F => 36,
            Self::G => 72,
        }
    }
}

macro_rules! jt4_protocol {
    ($name:ident, $multiplier:literal) => {
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            // JT4 has fractional 12-kHz symbol boundaries. NSPS is the
            // nearest integer solely for trait metadata; TX/RX use
            // SYMBOL_DT cumulatively and never accumulate this rounding.
            const NSPS: u32 = 2743;
            const SYMBOL_DT: f32 = 1.0 / JT4_BAUD;
            const TONE_SPACING_HZ: f32 = JT4_BAUD * $multiplier as f32;
            const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
            const GFSK_BT: f32 = 0.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 1;
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = 1;
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 206;
            const N_SYNC: u32 = 0;
            const N_SYMBOLS: u32 = 206;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Interleaved {
                sync_bit_pos: 0,
                vector: &JT4_SYNC_VECTOR,
            };
            const T_SLOT_S: f32 = 60.0;
            const TX_START_OFFSET_S: f32 = 1.0;
        }

        impl Protocol for $name {
            type Fec = ConvFano232Jt4;
            type Msg = Jt72Codec;
            const ID: ProtocolId = ProtocolId::Jt4;
        }
    };
}

jt4_protocol!(Jt4a, 1);
jt4_protocol!(Jt4b, 2);
jt4_protocol!(Jt4c, 4);
jt4_protocol!(Jt4d, 9);
jt4_protocol!(Jt4e, 18);
jt4_protocol!(Jt4f, 36);
jt4_protocol!(Jt4g, 72);

/// Acquire and decode JT4 from a slot recording.
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    submode: Jt4Submode,
    params: &SearchParams,
) -> alloc::vec::Vec<Jt4Decode> {
    let mut decoded = alloc::vec::Vec::new();
    let samples_per_symbol = sample_rate as f64 / JT4_BAUD as f64;
    let eighth_symbol = (samples_per_symbol / 8.0).round() as i64;
    // Coarse acquisition is deliberately cheap and can land on a
    // neighbouring sync-correlation lobe in weak/fading recordings.
    // Refine one full symbol around it on the upstream 1/8-symbol grid.
    let time_refinements = [
        0,
        -eighth_symbol,
        eighth_symbol,
        -2 * eighth_symbol,
        2 * eighth_symbol,
        -3 * eighth_symbol,
        3 * eighth_symbol,
        -4 * eighth_symbol,
        4 * eighth_symbol,
        -5 * eighth_symbol,
        5 * eighth_symbol,
        -6 * eighth_symbol,
        6 * eighth_symbol,
        -7 * eighth_symbol,
        7 * eighth_symbol,
        -8 * eighth_symbol,
        8 * eighth_symbol,
    ];
    let frequency_step = JT4_BAUD / 4.0;

    for candidate in coarse_search(audio, sample_rate, nominal_start_sample, submode, params) {
        let mut found = None;
        for &time_delta in &time_refinements {
            let start = candidate.start_sample as i64 + time_delta;
            if start < 0 {
                continue;
            }
            // The CCF bin centre is already the pinned WSJT-X frequency
            // estimate. Try it before neighbouring quarter-baud offsets:
            // a failed marginal Fano search can consume its full cycle
            // budget, while the centred reference alignment decodes
            // immediately.
            for frequency_delta in [0.0, -frequency_step, frequency_step] {
                if let Some(result) = decode_aligned(
                    audio,
                    sample_rate,
                    start as usize,
                    candidate.base_frequency_hz + frequency_delta,
                    submode,
                ) {
                    found = Some(result);
                    break;
                }
            }
            if found.is_some() {
                break;
            }
        }
        if let Some(result) = found {
            decoded.push(result);
            // Pinned WSJT-X deliberately operates JT4 in single-decode
            // mode. Continuing through weak secondary CCF peaks after a
            // valid Fano result adds no protocol capability and can spend
            // the full sequential-decoder budget on every noise peak.
            break;
        }
    }
    decoded
}

pub fn decode_scan_default(
    audio: &[f32],
    sample_rate: u32,
    submode: Jt4Submode,
) -> alloc::vec::Vec<Jt4Decode> {
    decode_scan(
        audio,
        sample_rate,
        sample_rate as usize,
        submode,
        &SearchParams::default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn submode_geometry_matches_pinned_wsjt_x() {
        assert_eq!(
            Jt4Submode::ALL.map(Jt4Submode::spacing_multiplier),
            [1, 2, 4, 9, 18, 36, 72],
        );
        assert_eq!(JT4_SYNC_VECTOR.len(), 206);
        assert_eq!(JT4_SYNC_VECTOR.iter().filter(|&&bit| bit == 1).count(), 103);
    }
}
