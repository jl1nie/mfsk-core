// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65 protocol markers + trait wiring.
//!
//! Each Q65 sub-mode is exposed as its own zero-sized type. Sub-modes
//! vary along two orthogonal axes (per `q65params.f90`):
//!
//! - **T/R period** (15 / 30 / 60 / 120 / 300 s) — controls `NSPS`.
//! - **Tone-spacing letter** A / B / C / D / E — controls
//!   `TONE_SPACING_HZ` via `baud × 2^(letter − 1)` multipliers
//!   (×1, ×2, ×4, ×8, ×16).
//!
//! All sub-modes share the same FEC, sync layout, message format and
//! tone numbering — only NSPS and tone spacing change. The wired
//! sub-modes:
//!
//! | ZST          | T/R   | spacing      | typical use                     |
//! |--------------|-------|--------------|---------------------------------|
//! | [`Q65a30`]   | 30 s  | 3.333 Hz     | terrestrial HF/VHF, ionoscatter |
//! | [`Q65a60`]   | 60 s  | 1.667 Hz     | 6 m EME                         |
//! | [`Q65b60`]   | 60 s  | 3.333 Hz     | 70 cm – 23 cm EME               |
//! | [`Q65c60`]   | 60 s  | 6.667 Hz     | microwave EME                   |
//! | [`Q65d60`]   | 60 s  | 13.33 Hz     | 5.7 / 10 GHz EME                |
//! | [`Q65e60`]   | 60 s  | 26.67 Hz     | extreme Doppler / wide spread   |
//!
//! Adding a new sub-mode is a one-line invocation of the
//! `q65_submode!` macro defined further down in this file.

use crate::core::{
    FecCodec, FecOpts, FecResult, FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode,
};
use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::Q65Message;

use super::sync_pattern::Q65_SYNC_BLOCKS;

const IDENTITY_65: [u8; 65] = {
    let mut m = [0u8; 65];
    let mut i = 0usize;
    while i < 65 {
        m[i] = i as u8;
        i += 1;
    }
    m
};

/// Define a Q65 sub-mode ZST with its `ModulationParams`,
/// `FrameLayout` and `Protocol` impls. All sub-modes share NTONES,
/// BITS_PER_SYMBOL, sync pattern, FEC, message codec, and the 22
/// sync / 63 data symbol layout — only NSPS (driven by T/R period)
/// and TONE_SPACING_HZ (driven by sub-mode letter) differ.
macro_rules! q65_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        nsps = $nsps:literal,
        spacing_mult = $mult:literal,
        tr_period_s = $period:literal,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 65;
            const BITS_PER_SYMBOL: u32 = 6;
            const NSPS: u32 = $nsps;
            const SYMBOL_DT: f32 = ($nsps as f32) / 12_000.0;
            /// Tone spacing = baud × multiplier where baud = 12000 / NSPS.
            const TONE_SPACING_HZ: f32 = (12_000.0 / ($nsps as f32)) * ($mult as f32);
            const GRAY_MAP: &'static [u8] = &IDENTITY_65;
            /// Plain FSK — Q65 does not Gaussian-shape its tones.
            const GFSK_BT: f32 = 0.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 2;
            /// 4 kHz baseband (12000 / 3) — wider than every Q65
            /// sub-mode's worst-case occupancy of 65 × 26.67 Hz =
            /// 1733 Hz (Q65-?E).
            const NDOWN: u32 = 3;
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 63;
            const N_SYNC: u32 = 22;
            const N_SYMBOLS: u32 = 85;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&Q65_SYNC_BLOCKS);
            const T_SLOT_S: f32 = $period as f32;
            /// 1.0 s start offset matches WSJT-X's Q65 slot timing.
            const TX_START_OFFSET_S: f32 = 1.0;
        }

        impl Protocol for $name {
            type Fec = Q65Fec;
            type Msg = Q65Message;
            const ID: ProtocolId = ProtocolId::Q65;
        }
    };
}

q65_submode! {
    /// Q65-30A: 30 s T/R period, sub-mode A (tone spacing = baud
    /// × 1 = 3.333 Hz). The most common terrestrial Q65 mode; suits
    /// HF and VHF ionoscatter / weak-signal QSOs.
    Q65a30,
    nsps = 3600,
    spacing_mult = 1,
    tr_period_s = 30,
}

q65_submode! {
    /// Q65-60A: 60 s T/R period, sub-mode A (tone spacing = baud
    /// × 1 = 1.667 Hz). Typical for **6 m EME** — narrow spacing
    /// keeps the signal inside the residual Doppler at low VHF.
    Q65a60,
    nsps = 7200,
    spacing_mult = 1,
    tr_period_s = 60,
}

q65_submode! {
    /// Q65-60B: 60 s T/R period, sub-mode B (tone spacing = baud
    /// × 2 = 3.333 Hz). Typical for **70 cm and 23 cm EME** where
    /// Doppler is wider than at 6 m but still moderate.
    Q65b60,
    nsps = 7200,
    spacing_mult = 2,
    tr_period_s = 60,
}

q65_submode! {
    /// Q65-60C: 60 s T/R period, sub-mode C (tone spacing = baud
    /// × 4 = 6.667 Hz). Microwave EME (~3 GHz) where Doppler
    /// spread starts to dominate.
    Q65c60,
    nsps = 7200,
    spacing_mult = 4,
    tr_period_s = 60,
}

q65_submode! {
    /// Q65-60D: 60 s T/R period, sub-mode D (tone spacing = baud
    /// × 8 = 13.33 Hz). Used for **5.7 / 10 GHz EME** where lunar
    /// libration spreads tones by tens of Hz.
    Q65d60,
    nsps = 7200,
    spacing_mult = 8,
    tr_period_s = 60,
}

q65_submode! {
    /// Q65-60E: 60 s T/R period, sub-mode E (tone spacing = baud
    /// × 16 = 26.67 Hz). Extreme-Doppler / wide-spread channels;
    /// useful at 24 GHz and above or for fast aircraft scatter.
    Q65e60,
    nsps = 7200,
    spacing_mult = 16,
    tr_period_s = 60,
}

/// FecCodec stub for Q65 — present so [`Q65a30`] can satisfy the
/// `Protocol::Fec: FecCodec` bound; the real soft-decision decode
/// path lives in [`Q65Codec`] and is invoked from
/// [`crate::q65::rx`].
///
/// `decode_soft` always returns `None` because the QRA decoder
/// consumes per-symbol probability distributions over GF(64), not
/// bit-level LLRs. `encode` is implemented faithfully (bit-level in,
/// bit-level out) by routing through a transient [`Q65Codec`], so
/// callers that want a quick reference encoding via the generic
/// trait still get the right answer.
#[derive(Copy, Clone, Debug, Default)]
pub struct Q65Fec;

impl FecCodec for Q65Fec {
    /// 63 transmitted channel symbols × 6 bits.
    const N: usize = 63 * 6;
    /// 13 user info symbols × 6 bits = 78 bits (77-bit Wsjt77 +
    /// 1-bit zero padding handled at the message-codec boundary).
    const K: usize = 13 * 6;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), Self::K, "encode: info.len() != K");
        assert_eq!(codeword.len(), Self::N, "encode: codeword.len() != N");

        // bits → 13 GF(64) symbols (MSB-first within each 6-bit group).
        let mut info_syms = [0_i32; 13];
        for (i, slot) in info_syms.iter_mut().enumerate() {
            let mut s = 0_i32;
            for b in 0..6 {
                s = (s << 1) | (info[6 * i + b] & 1) as i32;
            }
            *slot = s;
        }

        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut channel = [0_i32; 63];
        codec.encode(&info_syms, &mut channel);

        // 63 GF(64) symbols → bits (MSB-first within each symbol).
        for (i, &sym) in channel.iter().enumerate() {
            for b in 0..6 {
                codeword[6 * i + b] = ((sym >> (5 - b)) & 1) as u8;
            }
        }
    }

    fn decode_soft(&self, _llr: &[f32], _opts: &FecOpts) -> Option<FecResult> {
        // Bit-LLR soft decoding is not the natural API for Q65's
        // GF(64) belief propagation. Use `Q65Codec::decode` (or the
        // protocol-level helpers in `crate::q65::rx`) instead.
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modulation_constants_match_spec() {
        assert_eq!(<Q65a30 as ModulationParams>::NTONES, 65);
        assert_eq!(<Q65a30 as ModulationParams>::BITS_PER_SYMBOL, 6);
        assert_eq!(<Q65a30 as ModulationParams>::NSPS, 3600);
        assert!(
            (<Q65a30 as ModulationParams>::SYMBOL_DT - 0.3).abs() < 1e-6,
            "SYMBOL_DT must be 0.3 s for the 30-s T/R period"
        );
        let spacing = <Q65a30 as ModulationParams>::TONE_SPACING_HZ;
        assert!(
            (spacing - 12_000.0 / 3600.0).abs() < 1e-3,
            "Q65-30A tone spacing must be 12000/3600 ≈ 3.333 Hz, got {spacing}"
        );
    }

    #[test]
    fn eme_submode_constants_match_q65params_f90() {
        // q65params.f90 maps T/R → NSPS and letter → spacing
        // multiplier 2^(letter − 1). Spot-check each EME sub-mode's
        // NSPS = 7200 and tone spacing = (12000/7200) × multiplier.
        let baud_60 = 12_000.0 / 7200.0; // 1.667 Hz
        for (name, spacing, mult) in [
            ("Q65-60A", <Q65a60 as ModulationParams>::TONE_SPACING_HZ, 1),
            ("Q65-60B", <Q65b60 as ModulationParams>::TONE_SPACING_HZ, 2),
            ("Q65-60C", <Q65c60 as ModulationParams>::TONE_SPACING_HZ, 4),
            ("Q65-60D", <Q65d60 as ModulationParams>::TONE_SPACING_HZ, 8),
            ("Q65-60E", <Q65e60 as ModulationParams>::TONE_SPACING_HZ, 16),
        ] {
            let expected = baud_60 * mult as f32;
            assert!(
                (spacing - expected).abs() < 1e-3,
                "{name} spacing {spacing} != expected {expected}"
            );
        }
        // All 60 s sub-modes share NSPS = 7200.
        assert_eq!(<Q65a60 as ModulationParams>::NSPS, 7200);
        assert_eq!(<Q65b60 as ModulationParams>::NSPS, 7200);
        assert_eq!(<Q65c60 as ModulationParams>::NSPS, 7200);
        assert_eq!(<Q65d60 as ModulationParams>::NSPS, 7200);
        assert_eq!(<Q65e60 as ModulationParams>::NSPS, 7200);
        // …and a 60-second slot.
        assert_eq!(<Q65a60 as FrameLayout>::T_SLOT_S, 60.0);
        assert_eq!(<Q65e60 as FrameLayout>::T_SLOT_S, 60.0);
    }

    #[test]
    fn all_q65_submodes_share_frame_layout() {
        // Every Q65 sub-mode MUST share the same frame structure so
        // that the generic tx/rx code can switch between them on the
        // type parameter alone (only timing / spacing changes).
        for (name, n_data, n_sync, n_symbols) in [
            (
                "Q65a30",
                <Q65a30 as FrameLayout>::N_DATA,
                <Q65a30 as FrameLayout>::N_SYNC,
                <Q65a30 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "Q65a60",
                <Q65a60 as FrameLayout>::N_DATA,
                <Q65a60 as FrameLayout>::N_SYNC,
                <Q65a60 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "Q65b60",
                <Q65b60 as FrameLayout>::N_DATA,
                <Q65b60 as FrameLayout>::N_SYNC,
                <Q65b60 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "Q65c60",
                <Q65c60 as FrameLayout>::N_DATA,
                <Q65c60 as FrameLayout>::N_SYNC,
                <Q65c60 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "Q65d60",
                <Q65d60 as FrameLayout>::N_DATA,
                <Q65d60 as FrameLayout>::N_SYNC,
                <Q65d60 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "Q65e60",
                <Q65e60 as FrameLayout>::N_DATA,
                <Q65e60 as FrameLayout>::N_SYNC,
                <Q65e60 as FrameLayout>::N_SYMBOLS,
            ),
        ] {
            assert_eq!(n_data, 63, "{name} N_DATA");
            assert_eq!(n_sync, 22, "{name} N_SYNC");
            assert_eq!(n_symbols, 85, "{name} N_SYMBOLS");
        }
    }

    #[test]
    fn frame_layout_constants_match_spec() {
        assert_eq!(<Q65a30 as FrameLayout>::N_DATA, 63);
        assert_eq!(<Q65a30 as FrameLayout>::N_SYNC, 22);
        assert_eq!(<Q65a30 as FrameLayout>::N_SYMBOLS, 85);
        assert_eq!(<Q65a30 as FrameLayout>::N_RAMP, 0);
        assert_eq!(<Q65a30 as FrameLayout>::T_SLOT_S, 30.0);
        match <Q65a30 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 22, "Q65 has 22 distributed sync symbols");
                for b in blocks {
                    assert_eq!(b.pattern, &[0u8], "every Q65 sync symbol is tone 0");
                }
            }
            SyncMode::Interleaved { .. } => {
                panic!("Q65 must use Block sync, not Interleaved")
            }
        }
    }

    #[test]
    fn protocol_id_is_q65() {
        assert_eq!(<Q65a30 as Protocol>::ID, ProtocolId::Q65);
    }

    #[test]
    fn q65fec_encode_matches_q65codec_direct() {
        // The bit-level FecCodec stub must agree with calling
        // `Q65Codec::encode` directly on the same payload.
        let fec = Q65Fec;
        // Pseudo-random 78-bit info pattern.
        let info: Vec<u8> = (0..Q65Fec::K)
            .map(|i| ((i.wrapping_mul(13) ^ 0x55) & 1) as u8)
            .collect();
        let mut codeword = vec![0u8; Q65Fec::N];
        fec.encode(&info, &mut codeword);

        // Compute the ground truth via Q65Codec on the same 13-symbol
        // info vector.
        let mut info_syms = [0_i32; 13];
        for (i, slot) in info_syms.iter_mut().enumerate() {
            let mut s = 0_i32;
            for b in 0..6 {
                s = (s << 1) | (info[6 * i + b] & 1) as i32;
            }
            *slot = s;
        }
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut expected_channel = [0_i32; 63];
        codec.encode(&info_syms, &mut expected_channel);

        // Reconstruct the bit-level codeword from expected_channel
        // and compare.
        let mut expected_bits = vec![0u8; Q65Fec::N];
        for (i, &sym) in expected_channel.iter().enumerate() {
            for b in 0..6 {
                expected_bits[6 * i + b] = ((sym >> (5 - b)) & 1) as u8;
            }
        }
        assert_eq!(codeword, expected_bits);
    }

    #[test]
    fn decode_soft_is_a_stub() {
        let fec = Q65Fec;
        let llr = vec![0.0_f32; Q65Fec::N];
        assert!(fec.decode_soft(&llr, &FecOpts::default()).is_none());
    }
}
