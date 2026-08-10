//! # `fst4` — FST4 decoder and synthesiser
//!
//! FST4 is a weak-signal slow-speed mode used for EME / troposcatter /
//! LF-MF propagation experiments. Trait surface, frame layout, Costas
//! positions, DSP routing, and LDPC(240, 101) + CRC-24 codec
//! ([`crate::fec::Ldpc240_101`]) are all wired. The 77-bit message
//! layer is shared verbatim with FT8 / FT4.
//!
//! ## Covered sub-modes
//!
//! Five T/R-period sub-modes are wired: [`Fst4s15`], [`Fst4s30`],
//! [`Fst4s60`], [`Fst4s120`], [`Fst4s300`]. They differ only in
//! [`ModulationParams::NSPS`] / `NDOWN` / `SYMBOL_DT` /
//! `TONE_SPACING_HZ` (and `Fst4s15` alone in
//! [`FrameLayout::TX_START_OFFSET_S`] — see the `fst4_submode!`
//! invocations below for the WSJT-X-sourced values) — frame layout,
//! sync pattern, FEC, message codec, and GFSK shaping (BT=2.0) are
//! identical across all of them, mirroring the
//! [`crate::q65::Q65a30`]-family `q65_submode!` pattern. FST4-900 and
//! FST4-1800 are not wired (no user demand as of writing). FST4W (the
//! WSPR-style 50-bit one-way beacon variant, LDPC(240,74), periods
//! 120/300/900/1800 s) is a separate message format entirely — not
//! covered here; see issue #23 for status.
//!
//! ## References
//!
//! - K1JT et al., "The FST4 and FST4W Protocols", QEX 2021
//! - WSJT-X `lib/fst4_decode.f90`, `lib/fst4/fst4sim.f90`,
//!   `lib/fst4/fst4_params.f90`, `lib/fst4/genfst4.f90`,
//!   `lib/fst4/gen_fst4wave.f90`
//!
//! ## Quick example
//!
//! ```no_run
//! use mfsk_core::fst4::Fst4s60;
//! use mfsk_core::msg::decode_request::DecodeRequest;
//! use mfsk_core::msg::wsjt77::unpack77;
//!
//! # let audio: Vec<i16> = vec![];
//! // `audio` is 720_000 i16 samples at 12 kHz (60 s FST4-60A slot).
//! let results = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3_000.0, 0.8, /* max_cand */ 30)
//!     .decode()
//!     .results;
//! for r in &results {
//!     let msg77: &[u8; 77] = r.message77().try_into().unwrap();
//!     if let Some(text) = unpack77(msg77) {
//!         println!("{:7.1} Hz  dt={:+.2} s  {}", r.freq_hz, r.dt_sec, text);
//!     }
//! }
//! ```

use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode};
use crate::fec::Ldpc240_101;
use crate::msg::Wsjt77Message;

pub(crate) mod baseline;
pub mod decode;
pub mod encode;

/// FST4's 77-bit pre-LDPC scrambler — identical to [`crate::ft4::FT4_RVEC`]
/// (WSJT-X `genfst4.f90:29-31` uses the same literal array as
/// `genft4.f90`), duplicated here rather than imported so `fst4` doesn't
/// pull in the `ft4` feature. WSJT-X `genfst4.f90:63` XORs the 77-bit
/// message with this sequence before CRC-24 + LDPC encode; the receiver
/// must undo it after decode. Shared by every FST4 sub-mode (unconditional
/// on T/R period).
const FST4_RVEC: [u8; 77] = [
    0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
];

// Two alternating Costas patterns, each 8 symbols long, at symbols
// 0 / 38 / 76 / 114 / 152 (0-indexed). Shared by every FST4 sub-mode —
// `genfst4.f90` / `get_fst4_bitmetrics.f90` have no T/R-period branches.
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

/// Define an FST4 sub-mode ZST with its `ModulationParams`,
/// `FrameLayout` and `Protocol` impls. Every FST4 sub-mode shares
/// NTONES, BITS_PER_SYMBOL, Gray map, GFSK BT/hmod, sync pattern, FEC,
/// message codec, the 120-data/40-sync/160-total symbol layout, and the
/// `rvec` message scramble (`fst4_params.f90` / `genfst4.f90` have no
/// T/R-period branches) — only `NSPS` (⇒ `SYMBOL_DT` / `TONE_SPACING_HZ`),
/// `NDOWN`, `T_SLOT_S`, and (for FST4-15 alone) `TX_START_OFFSET_S`
/// differ, all taken directly from WSJT-X `fst4_decode.f90` /
/// `fst4sim.f90`'s per-`ntrperiod` branches.
macro_rules! fst4_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        nsps = $nsps:literal,
        ndown = $ndown:literal,
        tr_period_s = $period:literal,
        tx_start_offset_s = $tx_start:literal,
        snr_calfac = $snr_calfac:literal,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl $name {
            /// `fst4_decode.f90`'s `select case (ntrperiod)` constant
            /// feeding [`crate::fst4::baseline::fst4_snr_db`]'s `arg =
            /// snr_calfac·xsig/base - 1.0`. Per sub-mode, not derived
            /// from anything else — see the `fst4_submode!` invocation
            /// below for this sub-mode's literal.
            pub(crate) const SNR_CALFAC: f32 = $snr_calfac;
        }

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            const NSPS: u32 = $nsps;
            const SYMBOL_DT: f32 = ($nsps as f32) / 12_000.0;
            const TONE_SPACING_HZ: f32 = 12_000.0 / ($nsps as f32);
            const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
            /// BT=2.0 for every FST4 sub-mode (`gen_fst4wave.f90:37`
            /// `gfsk_pulse(2.0,tt)`, unconditional on T/R period) —
            /// matches FT8's `GFSK_BT`, not FT4's 1.0.
            const GFSK_BT: f32 = 2.0;
            const GFSK_HMOD: f32 = 1.0;
            // NFFT window = 2 × NSPS (same convention as FT8) — longer
            // windows don't help FST4 because the channel is assumed
            // quasi-static across the slot.
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            // Half-symbol coarse grid (matches FT4 practice).
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = $ndown;
            const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = Some(&FST4_RVEC);
            /// Matches WSJT-X `get_fst4_bitmetrics.f90`'s 1/2/4/8-symbol
            /// correlation ladder (issue #146) — was silently inheriting
            /// the FT8-calibrated default of 3.
            const LLR_NSYM_MAX: u32 = 8;
            /// The nsym=4 rung of that same ladder — see
            /// `ModulationParams::LLR_NSYM_MID` doc for the measured
            /// recall effect.
            const LLR_NSYM_MID: Option<u32> = Some(4);
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 120;
            const N_SYNC: u32 = 40; // 5 × 8
            const N_SYMBOLS: u32 = 160;
            const N_RAMP: u32 = 0; // GFSK synth handles ramp internally
            const SYNC_MODE: SyncMode = SyncMode::Block(&FST4_SYNC_BLOCKS);
            const T_SLOT_S: f32 = $period as f32;
            const TX_START_OFFSET_S: f32 = $tx_start;
        }

        impl Protocol for $name {
            /// LDPC(240, 101) + CRC-24 — see [`crate::fec::Ldpc240_101`].
            type Fec = Ldpc240_101;
            /// Same 77-bit WSJT message layout as FT8 / FT4 — fully reused.
            type Msg = Wsjt77Message;
            const ID: ProtocolId = ProtocolId::Fst4;
        }
    };
}

fst4_submode! {
    /// FST4-15: 15 s T/R period, 16.667 Hz tone spacing (66.7 Hz
    /// occupied). The fastest FST4 sub-mode; S/N threshold ≈ -20.7 dB.
    /// WSJT-X `fst4_decode.f90` (`ntrperiod.eq.15`): `nsps=720`,
    /// `ndown=18`. Uniquely among FST4 sub-modes, transmissions start
    /// 0.5 s (not 1.0 s) after the slot boundary
    /// (`fst4sim.f90`: `if(nsec.eq.15) k=nint((xdt+0.5)/dt)`;
    /// `fst4_decode.f90`: `if(ntrperiod.eq.15) xdt=(isbest-real(nspsec)/2.0)/fs2`).
    Fst4s15,
    nsps = 720,
    ndown = 18,
    tr_period_s = 15,
    tx_start_offset_s = 0.5,
    snr_calfac = 800.0,
}

fst4_submode! {
    /// FST4-30: 30 s T/R period, 7.143 Hz tone spacing (28.6 Hz
    /// occupied). S/N threshold ≈ -24.2 dB. WSJT-X `fst4_decode.f90`
    /// (`ntrperiod.eq.30`): `nsps=1680`, `ndown=42`.
    Fst4s30,
    nsps = 1_680,
    ndown = 42,
    tr_period_s = 30,
    tx_start_offset_s = 1.0,
    snr_calfac = 600.0,
}

fst4_submode! {
    /// FST4-60A: 60 s T/R period, 3.0864 Hz tone spacing (12.35 Hz
    /// occupied). The dominant terrestrial FST4 sub-mode; S/N
    /// threshold ≈ -28.1 dB. WSJT-X `fst4_decode.f90`
    /// (`ntrperiod.eq.60`): `nsps=3888`, `ndown=108`.
    Fst4s60,
    nsps = 3_888,
    ndown = 108,
    tr_period_s = 60,
    tx_start_offset_s = 1.0,
    snr_calfac = 430.0,
}

fst4_submode! {
    /// FST4-120: 120 s T/R period, 1.4634 Hz tone spacing (5.9 Hz
    /// occupied). S/N threshold ≈ -31.3 dB. WSJT-X `fst4_decode.f90`
    /// (`ntrperiod.eq.120`): `nsps=8200`, `ndown=205`.
    Fst4s120,
    nsps = 8_200,
    ndown = 205,
    tr_period_s = 120,
    tx_start_offset_s = 1.0,
    snr_calfac = 390.0,
}

fst4_submode! {
    /// FST4-300: 300 s (5 min) T/R period, 0.5580 Hz tone spacing
    /// (2.2 Hz occupied). S/N threshold ≈ -35.3 dB. WSJT-X
    /// `fst4_decode.f90` (`ntrperiod.eq.300`): `nsps=21504`,
    /// `ndown=512`.
    Fst4s300,
    nsps = 21_504,
    ndown = 512,
    tr_period_s = 300,
    tx_start_offset_s = 1.0,
    snr_calfac = 340.0,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::FecCodec;

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

        assert_eq!(<<Fst4s60 as Protocol>::Fec as FecCodec>::N, 240);
        assert_eq!(<<Fst4s60 as Protocol>::Fec as FecCodec>::K, 101);
    }

    /// Every sub-mode's WSJT-X-sourced `NSPS`/`NDOWN` from the
    /// `fst4_decode.f90` per-`ntrperiod` branches, spot-checked
    /// against the derived `SYMBOL_DT`/`TONE_SPACING_HZ` and against
    /// `NSPS % NDOWN == 0` (the generic sync pipeline assumes an exact
    /// downsampled-samples-per-symbol ratio).
    #[test]
    fn all_submodes_match_wsjtx_fst4_decode_f90() {
        fn check<P: ModulationParams + FrameLayout>(
            name: &str,
            nsps: u32,
            ndown: u32,
            t_slot_s: f32,
            tx_start_offset_s: f32,
        ) {
            assert_eq!(P::NSPS, nsps, "{name} NSPS");
            assert_eq!(P::NDOWN, ndown, "{name} NDOWN");
            assert_eq!(
                P::NSPS % P::NDOWN,
                0,
                "{name}: NSPS must be an exact multiple of NDOWN"
            );
            assert!(
                (P::SYMBOL_DT - nsps as f32 / 12_000.0).abs() < 1e-6,
                "{name} SYMBOL_DT"
            );
            assert!(
                (P::TONE_SPACING_HZ - 12_000.0 / nsps as f32).abs() < 1e-3,
                "{name} TONE_SPACING_HZ"
            );
            assert_eq!(P::T_SLOT_S, t_slot_s, "{name} T_SLOT_S");
            assert_eq!(
                P::TX_START_OFFSET_S,
                tx_start_offset_s,
                "{name} TX_START_OFFSET_S"
            );
        }
        check::<Fst4s15>("Fst4s15", 720, 18, 15.0, 0.5);
        check::<Fst4s30>("Fst4s30", 1_680, 42, 30.0, 1.0);
        check::<Fst4s60>("Fst4s60", 3_888, 108, 60.0, 1.0);
        check::<Fst4s120>("Fst4s120", 8_200, 205, 120.0, 1.0);
        check::<Fst4s300>("Fst4s300", 21_504, 512, 300.0, 1.0);
    }

    /// Every FST4 sub-mode MUST share the same frame structure, sync
    /// pattern, GFSK shaping, and message scramble so the generic
    /// tx/rx code can switch between them on the type parameter alone.
    #[test]
    fn all_submodes_share_frame_layout_and_fec() {
        fn check<P: ModulationParams + FrameLayout + Protocol>(name: &str) {
            assert_eq!(P::N_DATA, 120, "{name} N_DATA");
            assert_eq!(P::N_SYNC, 40, "{name} N_SYNC");
            assert_eq!(P::N_SYMBOLS, 160, "{name} N_SYMBOLS");
            assert_eq!(P::GFSK_BT, 2.0, "{name} GFSK_BT");
            assert_eq!(P::GFSK_HMOD, 1.0, "{name} GFSK_HMOD");
            assert_eq!(P::GRAY_MAP, &[0, 1, 3, 2], "{name} GRAY_MAP");
            assert_eq!(<P::Fec as FecCodec>::N, 240, "{name} Fec::N");
            assert_eq!(<P::Fec as FecCodec>::K, 101, "{name} Fec::K");
            assert_eq!(P::ID, ProtocolId::Fst4, "{name} ID");
            match P::SYNC_MODE {
                SyncMode::Block(blocks) => assert_eq!(blocks.len(), 5, "{name} sync blocks"),
                SyncMode::Interleaved { .. } => panic!("{name}: FST4 must use Block sync"),
            }
        }
        check::<Fst4s15>("Fst4s15");
        check::<Fst4s30>("Fst4s30");
        check::<Fst4s60>("Fst4s60");
        check::<Fst4s120>("Fst4s120");
        check::<Fst4s300>("Fst4s300");
    }

    /// Cross-checks every sub-mode's `DownsampleCfg` / `GfskCfg`
    /// (hand-picked `fft1_size`/`fft2_size`/`tone_spacing_hz` and
    /// `samples_per_symbol`/`ramp_samples` respectively) against the
    /// `ModulationParams`/`FrameLayout` trait constants they must
    /// stay consistent with. Guards against exactly the class of
    /// silent transcription bug that caused issue #23 in the first
    /// place: a hand-typed constant in `decode.rs`/`encode.rs` that
    /// quietly drifts from the ZST it's paired with.
    #[test]
    fn downsample_and_gfsk_configs_match_submode_constants() {
        use crate::engine::dsp::downsample::DownsampleCfg;
        use crate::engine::dsp::gfsk::GfskCfg;

        fn check_downsample<P: ModulationParams + FrameLayout>(name: &str, cfg: &DownsampleCfg) {
            let ndown = P::NDOWN as usize;
            assert_eq!(
                cfg.fft1_size % ndown,
                0,
                "{name}: fft1_size {} not divisible by NDOWN {ndown}",
                cfg.fft1_size
            );
            assert_eq!(
                cfg.fft1_size / ndown,
                cfg.fft2_size,
                "{name}: fft1_size/NDOWN != fft2_size"
            );
            let nmax = (P::T_SLOT_S * 12_000.0).round() as usize;
            assert!(
                cfg.fft1_size >= nmax,
                "{name}: fft1_size {} < nmax {nmax} (slot won't fit)",
                cfg.fft1_size
            );
            assert!(
                (cfg.tone_spacing_hz - P::TONE_SPACING_HZ).abs() < 1e-3,
                "{name}: cfg.tone_spacing_hz {} != ModulationParams::TONE_SPACING_HZ {}",
                cfg.tone_spacing_hz,
                P::TONE_SPACING_HZ
            );
        }

        fn check_gfsk<P: ModulationParams>(name: &str, cfg: &GfskCfg) {
            assert_eq!(
                cfg.samples_per_symbol as u32,
                P::NSPS,
                "{name}: gfsk samples_per_symbol != NSPS"
            );
            assert_eq!(cfg.bt, P::GFSK_BT, "{name}: gfsk bt != GFSK_BT");
            assert_eq!(cfg.hmod, P::GFSK_HMOD, "{name}: gfsk hmod != GFSK_HMOD");
            assert_eq!(
                cfg.ramp_samples as u32,
                P::NSPS / 8,
                "{name}: gfsk ramp_samples != NSPS/8"
            );
        }

        check_downsample::<Fst4s15>("Fst4s15", &decode::FST4_15_DOWNSAMPLE);
        check_downsample::<Fst4s30>("Fst4s30", &decode::FST4_30_DOWNSAMPLE);
        check_downsample::<Fst4s60>("Fst4s60", &decode::FST4_60A_DOWNSAMPLE);
        check_downsample::<Fst4s120>("Fst4s120", &decode::FST4_120_DOWNSAMPLE);
        check_downsample::<Fst4s300>("Fst4s300", &decode::FST4_300_DOWNSAMPLE);

        check_gfsk::<Fst4s15>("Fst4s15", &encode::FST4_15_GFSK);
        check_gfsk::<Fst4s30>("Fst4s30", &encode::FST4_30_GFSK);
        check_gfsk::<Fst4s60>("Fst4s60", &encode::FST4_60A_GFSK);
        check_gfsk::<Fst4s120>("Fst4s120", &encode::FST4_120_GFSK);
        check_gfsk::<Fst4s300>("Fst4s300", &encode::FST4_300_GFSK);
    }
}
