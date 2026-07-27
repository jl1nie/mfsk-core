// SPDX-License-Identifier: GPL-3.0-or-later
//! Protocol markers + trait wiring for the four uvpacket modes.
//!
//! Phase 2 modulation pivot (see `docs/0.3.1_PLAN.md`): the modem
//! is **single-carrier coherent QPSK at 1200 baud + RRC pulse + 31-
//! bit m-sequence preamble + periodic pilots**. The first 0.3.1
//! attempt with non-coherent 4-FSK at h=0.5 broke down on tone
//! orthogonality; QPSK's I/Q axes are orthogonal by construction.
//!
//! All four modes share the same modem (1200 baud QPSK, 1500 Hz
//! audio centre, RRC α=0.5), the same FEC mother code
//! (`Ldpc240_101`), the same preamble + pilot scheme, and the same
//! per-LDPC-block frame layout at the [`Protocol`] trait level.
//! They differ only in the puncturing applied to the FEC parity
//! bits, which lives outside the trait constants in
//! [`crate::uvpacket::puncture`].
//!
//! ## Scope boundary: decorative trait constants
//!
//! The `mfsk-core` `Protocol` trait surface was designed to express
//! the WSJT-X family of M-ary tone-FSK modes. uvpacket lives at
//! the boundary of that abstraction: it reuses the FEC layer but
//! its modulation (single-carrier coherent QPSK + RRC) and demod
//! (matched filter + pilot-aided phase track) bypass the generic
//! mfsk-core TX / RX pipeline entirely. The natural consequence is
//! that several `ModulationParams` constants — `NTONES = 4`,
//! `TONE_SPACING_HZ`, `GFSK_BT`, `GFSK_HMOD` — are **decorative**
//! for this module: they exist solely to satisfy the trait signature
//! and the `protocol_invariants` test. They are **not** consulted
//! by [`crate::uvpacket::tx::encode`] or
//! [`crate::uvpacket::rx::decode_known_layout`].
//!
//! See [`crate::uvpacket`]'s module docs for the full scope-note
//! table and the rationale for keeping uvpacket in-tree as an
//! "applied example of FEC reuse" rather than a peer WSJT-family
//! mode.
//!
//! | ZST            | rate | net bps (at 4-GFSK 2400 ch bps) | use |
//! |----------------|-----:|--------------------------------:|-----|
//! | [`UvRobust`]   | 0.42 | 1008 | mountain / weak signal / deep fading |
//! | [`UvStandard`] | 0.50 | 1200 | typical NFM with fading             |
//! | [`UvUltraRobust`] | 0.42 | 504 (half baud) | marathon / weakest-signal posture |
//! | [`UvExpress`]  | 0.75 | 1800 | strong-signal headline-fast mode (OSD-2 essentially mandatory) |
//!
//! Higher-rate modes use kSR-greedy puncture-set selection (see
//! [`crate::uvpacket::puncture`]) — the empirical AWGN sweep showed
//! ~1–3 dB Eb/N0 gain over uniform-spread at the deeper puncture
//! rates, which makes `UvExpress` (76 % parity puncturing) viable.
//!
//! Note: at the [`Protocol`] level, all four ZSTs claim the same
//! `N_DATA = 120` (= unpunctured codeword 240 ch bits / 2 bits/sym).
//! The actual on-air block length post-puncture is shorter for
//! Standard / Fast / Express and is handled by the bespoke TX/RX
//! paths in [`crate::uvpacket::tx`] / [`crate::uvpacket::rx`]. The
//! Protocol-level constants describe the *unpunctured* codeword so
//! the standard mfsk-core invariants (FEC fits in N_DATA × bits/sym)
//! hold.

use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Ldpc240_101;

use super::message::UvPacketRawMessage;
use super::puncture::Mode;
use super::sync_pattern::UVPACKET_SYNC_BLOCKS;

/// Identity Gray map for 4-FSK (FT4 uses the same).
const GRAY_4: [u8; 4] = [0, 1, 3, 2];

/// Audio-domain centre frequency at synth time (Hz). Tones land at
/// 800 / 1400 / 2000 / 2600 Hz, comfortably inside the typical NFM
/// HT audio passband while clearing the 300–500 Hz HPF found on
/// cheaper handhelds.
pub const AUDIO_CENTRE_HZ: f32 = 1700.0;

/// Define a uvpacket sub-mode ZST with all four trait impls.
///
/// All sub-modes share modulation, frame layout, FEC, message codec,
/// and sync. The only per-mode datum is the inherent `MODE` constant
/// pointing at the puncturing variant.
macro_rules! uvpacket_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        mode = $mode:expr,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl $name {
            /// Puncturing posture for this sub-mode. Used by the
            /// bespoke TX / RX paths to pick the right puncture
            /// table.
            pub const MODE: Mode = $mode;
        }

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            /// 1200 baud at 12 kHz sample rate → 10 samples / symbol.
            const NSPS: u32 = 10;
            const SYMBOL_DT: f32 = 1.0 / 1200.0;
            /// h = 0.5 → tone spacing = baud × h = 600 Hz.
            const TONE_SPACING_HZ: f32 = 600.0;
            const GRAY_MAP: &'static [u8] = &GRAY_4;
            const GFSK_BT: f32 = 0.5;
            const GFSK_HMOD: f32 = 0.5;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 4;
            const NSTEP_PER_SYMBOL: u32 = 2;
            /// 12000 / 4 = 3000 Hz baseband window — clears the
            /// 800–2600 Hz tone span with margin.
            const NDOWN: u32 = 4;
        }

        impl FrameLayout for $name {
            /// 240 codeword bits / 2 bits-per-symbol = 120 data symbols
            /// per LDPC block. (Unpunctured. Higher-rate modes
            /// transmit fewer ch bits per block but the trait-level
            /// constant describes the mother codeword.)
            const N_DATA: u32 = 120;
            /// One Costas-4 at the head of each LDPC block.
            const N_SYNC: u32 = 4;
            const N_SYMBOLS: u32 = 124;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&UVPACKET_SYNC_BLOCKS);
            /// uvpacket frames are not slot-aligned — value is
            /// informational only. Use the duration of one
            /// LDPC-block-sized "protocol unit" so callers that
            /// expect a non-zero T_SLOT_S see something reasonable.
            const T_SLOT_S: f32 = 124.0 / 1200.0;
            const TX_START_OFFSET_S: f32 = 0.0;
        }

        impl Protocol for $name {
            type Fec = Ldpc240_101;
            type Msg = UvPacketRawMessage;
            const ID: ProtocolId = ProtocolId::UvPacket;
        }
    };
}

uvpacket_submode! {
    /// **Robust** — rate 0.42 (unpunctured `Ldpc240_101`).
    /// 1008 net bps. For mountain / weak-signal / deep-fading
    /// channels where AFSK 1200 cannot deliver. AFSK has no
    /// equivalent mode — this is the design's headline value-prop.
    UvRobust, mode = Mode::Robust,
}

uvpacket_submode! {
    /// **Standard** — punctured to rate 1/2. 1200 net bps.
    /// Throughput parity with AFSK 1200 plus FEC for typical NFM
    /// channels.
    UvStandard, mode = Mode::Standard,
}

uvpacket_submode! {
    /// **UltraRobust** — unpunctured `Ldpc240_101` at half the
    /// canonical symbol rate (600 baud). 504 net bps,
    /// ≈ −1.75 dB SNR_3kHz threshold. The lowest-threshold mode in
    /// the lineup; targets the marginal-link niche where Robust
    /// can't quite hold but a slower-but-tougher path can. Half-
    /// baud chip duration also halves per-symbol phase walk and
    /// the relative size of any multipath delay, giving real-
    /// channel margin on top of the 3 dB symbol-energy gain.
    UvUltraRobust, mode = Mode::UltraRobust,
}

uvpacket_submode! {
    /// **Express** — punctured to rate 3/4. 1800 net bps (+50 % vs
    /// AFSK 1200). Strong-signal headline-fast mode. 76 % parity
    /// puncturing — OSD-2 is essentially mandatory at the BP
    /// threshold (~+3 dB Eb/N0 with OSD-2; BP-only needs ~+5 dB).
    /// Viable only thanks to kSR-greedy puncture selection
    /// (uniform-spread fails at this rate).
    UvExpress, mode = Mode::Express,
}
