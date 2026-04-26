// SPDX-License-Identifier: GPL-3.0-or-later
//! uvpacket sub-mode trait impls.
//!
//! Each sub-mode is a zero-sized type emitted by the
//! [`uvpacket_submode!`] macro from a small set of (baud, NSPS,
//! tone-spacing) parameters. All sub-modes share the same FEC
//! ([`Ldpc174_91`]), message codec ([`PacketBytesMessage`]), sync
//! pattern ([`super::sync_pattern::UVPACKET_SYNC_BLOCKS`]) and frame
//! shape — only the time-domain rate parameters differ.

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Ldpc174_91;
use crate::msg::PacketBytesMessage;

use super::sync_pattern::UVPACKET_SYNC_BLOCKS;

/// 4-FSK GFSK Gray map — same `[0, 1, 3, 2]` as FT4.
const UVPACKET_GRAY_MAP: [u8; 4] = [0, 1, 3, 2];

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
            /// One Costas-4 block.
            const N_SYNC: u32 = 4;
            /// 4 sync + 87 data = 91 channel symbols per frame.
            const N_SYMBOLS: u32 = 91;
            /// No GFSK ramp symbols — synthesiser handles ramp internally.
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&UVPACKET_SYNC_BLOCKS);
            /// Display only — uvpacket has no UTC slot. Set to 1.0 s
            /// for registry display; the receiver finds frames anywhere
            /// in the audio buffer via sliding-window coarse search.
            const T_SLOT_S: f32 = 1.0;
            const TX_START_OFFSET_S: f32 = 0.0;
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
    /// weak-signal / heavy-multipath conditions. Frame duration
    /// 607 ms.
    UvPacket150,
    nsps = 80,
    tone_spacing_hz = 150.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
}

uvpacket_submode! {
    /// Default variant: 300 baud, 1200 Hz total bandwidth. Common
    /// V/UHF SSB conditions. Frame duration 303 ms.
    UvPacket300,
    nsps = 40,
    tone_spacing_hz = 300.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
}

uvpacket_submode! {
    /// Fastest variant: 600 baud, 2400 Hz total bandwidth. Tight fit
    /// inside a typical 3 kHz SSB audio passband; needs decent SNR.
    /// Frame duration 152 ms.
    UvPacket600,
    nsps = 20,
    tone_spacing_hz = 600.0,
    nfft_factor = 4,
    nstep = 2,
    ndown = 2,
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
    fn frame_layout_consistent_across_sub_modes() {
        // All three sub-modes share the same frame shape — only
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
        ] {
            assert_eq!(n_data, 87, "{name} N_DATA");
            assert_eq!(n_sync, 4, "{name} N_SYNC");
            assert_eq!(n_symbols, 91, "{name} N_SYMBOLS");
        }
    }

    #[test]
    fn protocol_id_is_uv_packet_for_every_sub_mode() {
        assert_eq!(<UvPacket150 as Protocol>::ID, ProtocolId::UvPacket);
        assert_eq!(<UvPacket300 as Protocol>::ID, ProtocolId::UvPacket);
        assert_eq!(<UvPacket600 as Protocol>::ID, ProtocolId::UvPacket);
    }

    #[test]
    fn sync_layout_is_one_costas_block() {
        match <UvPacket300 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 1, "exactly one sync block");
                assert_eq!(blocks[0].start_symbol, 0);
                assert_eq!(blocks[0].pattern.len(), 4);
            }
            SyncMode::Interleaved { .. } => panic!("uvpacket must use Block sync"),
        }
    }
}
