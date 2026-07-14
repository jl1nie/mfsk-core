//! # `fst4` — FST4-60A decoder and synthesiser
//!
//! FST4 is a weak-signal slow-speed mode used for EME / troposcatter /
//! LF-MF propagation experiments. This module targets the **FST4-60A**
//! sub-mode (60-second T/R period, minimum tone spacing). Trait surface,
//! frame layout, Costas positions, DSP routing, and LDPC(240, 101) +
//! CRC-24 codec ([`crate::fec::Ldpc240_101`]) are all wired. The 77-bit
//! message layer is shared verbatim with FT8 / FT4.
//!
//! ## Covered sub-mode
//!
//! This module ships the **FST4-60A** sub-mode as [`Fst4s60`]. Other
//! sub-modes (FST4-15, -30, -120, -300, -900, -1800) differ only in
//! [`ModulationParams::NSPS`] / `SYMBOL_DT` / `TONE_SPACING_HZ` and
//! can be added as additional ZSTs using the same trait impl pattern.
//!
//! ## References
//!
//! - K1JT et al., "The FST4 and FST4W Protocols", QEX 2021
//! - WSJT-X `lib/fst4/` — `fst4_params.f90`, `genfst4.f90`
//!
//! ## Quick example
//!
//! ```no_run
//! use mfsk_core::fst4::decode::decode_frame;
//! use mfsk_core::msg::wsjt77::unpack77;
//!
//! # let audio: Vec<i16> = vec![];
//! // `audio` is 720_000 i16 samples at 12 kHz (60 s FST4-60A slot).
//! for r in decode_frame(&audio, 100.0, 3_000.0, 0.8, /* max_cand */ 30) {
//!     let msg77: &[u8; 77] = r.message77().try_into().unwrap();
//!     if let Some(text) = unpack77(msg77) {
//!         println!("{:7.1} Hz  dt={:+.2} s  {}", r.freq_hz, r.dt_sec, text);
//!     }
//! }
//! ```

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode};
use crate::fec::Ldpc240_101;
use crate::msg::Wsjt77Message;

pub mod decode;
pub mod encode;

/// FST4-60A: 4-GFSK, 60-second T/R period, 3.0864 baud, minimum tone
/// spacing (12.35 Hz occupied bandwidth). Uses LDPC(240, 101) + CRC-24
/// over the same 77-bit WSJT message payload that FT8 / FT4 use.
#[derive(Copy, Clone, Debug, Default)]
pub struct Fst4s60;

impl ModulationParams for Fst4s60 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    // WSJT-X `fst4_decode.f90`/`fst4sim.f90` (`ntrperiod.eq.60`):
    // nsps=3888, ndown=108, hmod=1, bt=2.0 (`gen_fst4wave.f90:37`
    // `gfsk_pulse(2.0,tt)`, unconditional for all FST4 sub-modes).
    // Symbol length 3888/12000 = 324 ms → baud = 12000/3888 ≈
    // 3.0864 Hz tone spacing. The previous values here (NSPS=3840,
    // NDOWN=192, GFSK_BT=1.0, 3.125 Hz) were placeholders that never
    // got revisited (see git history) — issue #23: they self-decode
    // fine in the synth roundtrip (encode/decode share the same wrong
    // constants) but drift ~0.3-0.6 s against real WSJT-X-generated
    // audio because the wrong NSPS accumulates timing error linearly
    // across the 160-symbol frame.
    const NSPS: u32 = 3_888;
    const SYMBOL_DT: f32 = 3_888.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 3_888.0;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    // BT=2.0 (see NSPS doc comment above) — matches FT8's GFSK_BT,
    // not FT4's 1.0. The previous BT=1.0 here was wrong (see NSPS
    // comment).
    const GFSK_BT: f32 = 2.0;
    const GFSK_HMOD: f32 = 1.0;
    // NFFT window = 2 × NSPS (same convention as FT8) — longer windows
    // don't help FST4-60 because the channel is assumed quasi-static
    // across the 60 s slot.
    const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
    // Half-symbol coarse grid (matches FT4 practice).
    const NSTEP_PER_SYMBOL: u32 = 2;
    // 12 000 / 108 = 111.11 Hz baseband (WSJT-X `fs2`) — enough for
    // 4-tone signal at 3.0864 Hz spacing plus guard band.
    const NDOWN: u32 = 108;
    // WSJT-X `genfst4.f90:63` XORs the 77-bit message with the same
    // `rvec` sequence FT4 uses (`genft4.f90:64`) before CRC-24 +
    // LDPC encode; the receiver must undo it after decode
    // (`fst4_decode.f90` unpack path). This was missing entirely —
    // CRC-24 still passes on real WSJT-X audio without it (the check
    // is self-referential: it just confirms LDPC decoded whatever 77
    // bits were actually sent, scrambled or not), but every decoded
    // message came out as scrambled garbage instead of a real
    // callsign. Same root cause bucket as issue #23.
    const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = Some(&FST4_RVEC);
}

/// FST4's 77-bit pre-LDPC scrambler — identical to [`crate::ft4::FT4_RVEC`]
/// (WSJT-X `genfst4.f90:29-31` uses the same literal array as
/// `genft4.f90`), duplicated here rather than imported so `fst4` doesn't
/// pull in the `ft4` feature.
const FST4_RVEC: [u8; 77] = [
    0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
];

impl FrameLayout for Fst4s60 {
    const N_DATA: u32 = 120;
    const N_SYNC: u32 = 40; // 5 × 8
    const N_SYMBOLS: u32 = 160;
    const N_RAMP: u32 = 0; // GFSK synth handles ramp internally
    const SYNC_MODE: SyncMode = SyncMode::Block(&FST4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 60.0;
    // FST4 transmissions start ~1 s after the slot boundary (per WSJT-X).
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Fst4s60 {
    /// LDPC(240, 101) + CRC-24 — see [`crate::fec::Ldpc240_101`].
    type Fec = Ldpc240_101;
    /// Same 77-bit WSJT message layout as FT8 / FT4 — fully reused.
    type Msg = Wsjt77Message;
    const ID: ProtocolId = ProtocolId::Fst4;
}

// Two alternating Costas patterns, each 8 symbols long, at symbols
// 0 / 38 / 76 / 114 / 152 (0-indexed).
const FST4_SYNC_A: [u8; 8] = [0, 1, 3, 2, 1, 0, 2, 3];
const FST4_SYNC_B: [u8; 8] = [2, 3, 1, 0, 3, 2, 0, 1];

const FST4_SYNC_BLOCKS: [SyncBlock; 5] = [
    SyncBlock {
        start_symbol: 0,
        pattern: &FST4_SYNC_A,
    },
    SyncBlock {
        start_symbol: 38,
        pattern: &FST4_SYNC_B,
    },
    SyncBlock {
        start_symbol: 76,
        pattern: &FST4_SYNC_A,
    },
    SyncBlock {
        start_symbol: 114,
        pattern: &FST4_SYNC_B,
    },
    SyncBlock {
        start_symbol: 152,
        pattern: &FST4_SYNC_A,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fst4s60_trait_surface() {
        assert_eq!(<Fst4s60 as ModulationParams>::NTONES, 4);
        assert_eq!(<Fst4s60 as ModulationParams>::NSPS, 3_888);
        assert!((<Fst4s60 as ModulationParams>::SYMBOL_DT - 0.324).abs() < 1e-6,);
        assert_eq!(<Fst4s60 as FrameLayout>::N_SYMBOLS, 160);
        assert_eq!(<Fst4s60 as FrameLayout>::N_DATA, 120);
        assert_eq!(<Fst4s60 as FrameLayout>::N_SYNC, 40);
        let blocks = <Fst4s60 as FrameLayout>::SYNC_MODE.blocks();
        assert_eq!(blocks.len(), 5);
        assert_eq!(
            blocks.iter().map(|b| b.start_symbol).collect::<Vec<_>>(),
            vec![0, 38, 76, 114, 152],
        );
        assert_eq!(blocks[0].pattern.len(), 8);

        use crate::core::FecCodec;
        assert_eq!(<<Fst4s60 as Protocol>::Fec as FecCodec>::N, 240);
        assert_eq!(<<Fst4s60 as Protocol>::Fec as FecCodec>::K, 101);
    }
}
