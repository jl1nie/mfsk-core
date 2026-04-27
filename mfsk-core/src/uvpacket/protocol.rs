// SPDX-License-Identifier: GPL-3.0-or-later
//! uvpacket sub-mode trait impls.
//!
//! Each sub-mode is a zero-sized type emitted by the
//! [`uvpacket_submode!`] macro from a small set of (baud, NSPS,
//! tone-spacing) parameters. All sub-modes share the same FEC
//! ([`Ldpc174_91`]), message codec ([`PacketBytesMessage`]), sync
//! pattern ([`super::sync_pattern::UVPACKET_SYNC_BLOCKS`]),
//! interleaver ([`UVPACKET_INTERLEAVE`]) and frame shape — only
//! the time-domain rate parameters differ.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Ldpc174_91;
use crate::msg::PacketBytesMessage;

use super::sync_pattern::UVPACKET_SYNC_BLOCKS;

/// 4-FSK GFSK Gray map — same `[0, 1, 3, 2]` as FT4.
const UVPACKET_GRAY_MAP: [u8; 4] = [0, 1, 3, 2];

/// Stride-25 polynomial bit interleaver over the 174-bit codeword.
///
/// Construction: `INTERLEAVE[j] = (7 * j) mod 174` — the multiplicative
/// inverse of the forward stride 25 (since `25 * 7 = 175 ≡ 1 (mod 174)`).
/// `gcd(25, 174) = 1` so the table is a full permutation of `0..174`.
///
/// What this buys you: the TX-side
/// [`crate::core::tx::codeword_to_itone`] reads codeword bit
/// `INTERLEAVE[j]` into channel-bit position `j`, so consecutive
/// codeword bits land **25 channel positions apart**. Inversely, a
/// burst of `B` consecutive corrupted channel bits decoheres into
/// `B` codeword bits at stride 7 — for B = 14 (≈ a 50 ms fade null
/// at UvPacket150), the affected codeword positions span 98 bits
/// out of 174, well within LDPC(174, 91)'s sparse-error correction
/// envelope.
pub const UVPACKET_INTERLEAVE: [u16; 174] = {
    let mut arr = [0u16; 174];
    let mut j = 0usize;
    while j < 174 {
        arr[j] = ((j * 7) % 174) as u16;
        j += 1;
    }
    arr
};

/// Define one uvpacket sub-mode ZST. All four trait impls
/// (`ModulationParams`, `FrameLayout`, `Protocol`) are emitted in
/// one shot from the input parameters — the only differences across
/// sub-modes are NSPS / SYMBOL_DT / TONE_SPACING_HZ. See
/// `docs/ADDING_A_PROTOCOL.md` for the macro walk-through.
macro_rules! uvpacket_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        nsps = $nsps:literal,
        tone_spacing_hz = $tone_spacing:literal,
        nfft_factor = $nfft_factor:literal,
        nstep = $nstep:literal,
        ndown = $ndown:literal,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            const NSPS: u32 = $nsps;
            const SYMBOL_DT: f32 = ($nsps as f32) / 12_000.0;
            const TONE_SPACING_HZ: f32 = $tone_spacing;
            const GRAY_MAP: &'static [u8] = &UVPACKET_GRAY_MAP;
            /// Modulation index 1.0 keeps tones orthogonal at non-coherent
            /// detection — the existing FFT-based pipeline works without
            /// a coherent demodulator.
            const GFSK_HMOD: f32 = 1.0;
            /// BT = 1.0 matches FT4. Slightly relaxes side-lobes vs
            /// pure FSK, helps fit inside the SSB voice passband.
            const GFSK_BT: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = $nfft_factor;
            const NSTEP_PER_SYMBOL: u32 = $nstep;
            const NDOWN: u32 = $ndown;
        }

        impl FrameLayout for $name {
            /// 87 data symbols × 2 bits-per-symbol = 174 = `Ldpc174_91::N`.
            const N_DATA: u32 = 87;
            /// Two Costas-4 blocks (head + middle) → 8 sync symbols.
            const N_SYNC: u32 = 8;
            /// 8 sync + 87 data = 95 channel symbols per frame.
            const N_SYMBOLS: u32 = 95;
            /// No GFSK ramp symbols — synthesiser handles ramp internally.
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&UVPACKET_SYNC_BLOCKS);
            /// Display only — uvpacket has no UTC slot. Set to 1.0 s
            /// for registry display; the receiver finds frames anywhere
            /// in the audio buffer via sliding-window coarse search.
            const T_SLOT_S: f32 = 1.0;
            const TX_START_OFFSET_S: f32 = 0.0;

            /// Stride-25 polynomial interleaver — see
            /// [`super::UVPACKET_INTERLEAVE`].
            const CODEWORD_INTERLEAVE: Option<&'static [u16]> = Some(&UVPACKET_INTERLEAVE);
        }

        impl Protocol for $name {
            type Fec = Ldpc174_91;
            type Msg = PacketBytesMessage;
            const ID: ProtocolId = ProtocolId::UvPacket;
        }
    };
}

uvpacket_submode! {
    /// Slowest variant: 150 baud, 600 Hz total bandwidth. Best for
    /// weak-signal / heavy-multipath conditions on SSB. Frame
    /// duration 633 ms.
    UvPacket150,
    nsps = 80,
    tone_spacing_hz = 150.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
}

uvpacket_submode! {
    /// Default SSB variant: 300 baud, 1200 Hz total bandwidth. Common
    /// V/UHF SSB conditions. Frame duration 317 ms.
    UvPacket300,
    nsps = 40,
    tone_spacing_hz = 300.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
}

uvpacket_submode! {
    /// Fastest SSB variant: 600 baud, 2400 Hz total bandwidth. Tight
    /// fit inside a typical 3 kHz SSB audio passband; needs decent
    /// SNR. Frame duration 158 ms.
    UvPacket600,
    nsps = 20,
    tone_spacing_hz = 600.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
}

uvpacket_submode! {
    /// FM-data-channel variant: 1200 baud, 4800 Hz total bandwidth.
    /// Exceeds the SSB passband; intended for FM data channels at
    /// U/VHF (12.5 / 25 kHz channel spacing) where the wider audio
    /// bandwidth is available. Frame duration 79 ms; gross 2400 bps,
    /// net ≈ 1.2 kbps after Ldpc174_91 (R ≈ 0.523).
    UvPacket1200,
    nsps = 10,
    tone_spacing_hz = 1200.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 1,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modulation_constants_for_uv_packet300() {
        assert_eq!(<UvPacket300 as ModulationParams>::NTONES, 4);
        assert_eq!(<UvPacket300 as ModulationParams>::BITS_PER_SYMBOL, 2);
        assert_eq!(<UvPacket300 as ModulationParams>::NSPS, 40);
        // SYMBOL_DT = 40 / 12000 = 1/300 s
        let dt = <UvPacket300 as ModulationParams>::SYMBOL_DT;
        assert!((dt - 1.0 / 300.0).abs() < 1e-7, "SYMBOL_DT off: {dt}");
        // 300 baud × 1 bps-tone-spacing-multiplier = 300 Hz tone spacing.
        assert_eq!(<UvPacket300 as ModulationParams>::TONE_SPACING_HZ, 300.0);
    }

    #[test]
    fn modulation_constants_for_uv_packet1200() {
        // 1200 baud → NSPS = 12000 / 1200 = 10
        assert_eq!(<UvPacket1200 as ModulationParams>::NSPS, 10);
        let dt = <UvPacket1200 as ModulationParams>::SYMBOL_DT;
        assert!((dt - 1.0 / 1200.0).abs() < 1e-7, "SYMBOL_DT off: {dt}");
        // BW = 4 tones × 1200 Hz = 4800 Hz — exceeds SSB but fits FM
        // data channels.
        assert_eq!(<UvPacket1200 as ModulationParams>::TONE_SPACING_HZ, 1200.0);
    }

    #[test]
    fn frame_layout_consistent_across_sub_modes() {
        // All four sub-modes share the same frame shape — only
        // time-domain (NSPS) differs.
        for (name, n_data, n_sync, n_symbols) in [
            (
                "UvPacket150",
                <UvPacket150 as FrameLayout>::N_DATA,
                <UvPacket150 as FrameLayout>::N_SYNC,
                <UvPacket150 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "UvPacket300",
                <UvPacket300 as FrameLayout>::N_DATA,
                <UvPacket300 as FrameLayout>::N_SYNC,
                <UvPacket300 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "UvPacket600",
                <UvPacket600 as FrameLayout>::N_DATA,
                <UvPacket600 as FrameLayout>::N_SYNC,
                <UvPacket600 as FrameLayout>::N_SYMBOLS,
            ),
            (
                "UvPacket1200",
                <UvPacket1200 as FrameLayout>::N_DATA,
                <UvPacket1200 as FrameLayout>::N_SYNC,
                <UvPacket1200 as FrameLayout>::N_SYMBOLS,
            ),
        ] {
            assert_eq!(n_data, 87, "{name} N_DATA");
            assert_eq!(n_sync, 8, "{name} N_SYNC");
            assert_eq!(n_symbols, 95, "{name} N_SYMBOLS");
        }
    }

    #[test]
    fn protocol_id_is_uv_packet_for_every_sub_mode() {
        assert_eq!(<UvPacket150 as Protocol>::ID, ProtocolId::UvPacket);
        assert_eq!(<UvPacket300 as Protocol>::ID, ProtocolId::UvPacket);
        assert_eq!(<UvPacket600 as Protocol>::ID, ProtocolId::UvPacket);
        assert_eq!(<UvPacket1200 as Protocol>::ID, ProtocolId::UvPacket);
    }

    #[test]
    fn sync_layout_is_two_costas_blocks() {
        match <UvPacket300 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 2, "exactly two sync blocks");
                assert_eq!(blocks[0].start_symbol, 0);
                assert_eq!(blocks[0].pattern.len(), 4);
                assert_eq!(blocks[1].start_symbol, 47);
                assert_eq!(blocks[1].pattern.len(), 4);
            }
            SyncMode::Interleaved { .. } => panic!("uvpacket must use Block sync"),
        }
    }

    #[test]
    fn interleave_table_is_a_permutation() {
        // Stride-25 polynomial: every output index 0..174 must appear
        // exactly once. If gcd(25, 174) ≠ 1 (or the table generator
        // drifts) this test exposes the duplicate / missing entry.
        let mut seen = [false; 174];
        for &idx in &UVPACKET_INTERLEAVE {
            let i = idx as usize;
            assert!(i < 174, "interleave entry {i} out of range");
            assert!(!seen[i], "interleave entry {i} duplicated");
            seen[i] = true;
        }
        assert!(seen.iter().all(|&b| b), "some indices in 0..174 missing");
    }

    #[test]
    fn interleave_is_advertised_via_frame_layout() {
        // The trait constant must point at the same table the module
        // exposes — drift between them is a silent bug that would
        // fail decode without a clear error.
        let from_trait =
            <UvPacket300 as FrameLayout>::CODEWORD_INTERLEAVE.expect("uvpacket interleaves");
        assert_eq!(from_trait.as_ptr(), UVPACKET_INTERLEAVE.as_ptr());
    }
}
