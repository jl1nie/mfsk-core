//! FST4's GFSK configurations, one per sub-mode — what each sub-mode's
//! [`crate::engine::tx::FskWaveform`] impl points at.
//!
//! The transmit chain itself is generic since #391:
//! [`crate::engine::tx::message_to_tones`] gives the (period-independent)
//! 160-symbol tone sequence and [`crate::engine::tx::synthesize`] turns it
//! into audio for a named sub-mode. That also retired the trap the old
//! un-suffixed `tones_to_f32` carried: it silently meant FST4-60A, where
//! `synthesize::<P>` cannot be called without naming one.

use crate::engine::dsp::gfsk::GfskCfg;

/// FST4-15 GFSK configuration: 12 kHz, 720 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_15_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 720,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 720 / 8,
};

/// FST4-30 GFSK configuration: 12 kHz, 1680 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_30_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 1_680,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 1_680 / 8,
};

/// FST4-60A GFSK configuration: 12 kHz, 3888 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
///
/// Was `samples_per_symbol: 3840, bt: 1.0` — hardcoded independently
/// of [`crate::engine::ModulationParams`] and never updated to match WSJT-X
/// `fst4_decode.f90`'s `nsps=3888` / `gen_fst4wave.f90`'s
/// `gfsk_pulse(2.0,tt)` for `ntrperiod.eq.60` (issue #23 root cause).
pub const FST4_60A_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 3_888,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 3_888 / 8,
};

/// FST4-120 GFSK configuration: 12 kHz, 8200 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_120_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 8_200,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 8_200 / 8,
};

/// FST4-300 GFSK configuration: 12 kHz, 21504 samples/symbol, BT=2.0,
/// hmod=1.0, NSPS/8-sample cosine ramp.
pub const FST4_300_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 21_504,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 21_504 / 8,
};

/// FST4-900 / FST4W-900 GFSK configuration: 12 kHz, 66560 samples/symbol,
/// BT=2.0, hmod=1.0, NSPS/8-sample cosine ramp (`fst4_decode.f90:420`
/// `nsps=66560`; `gen_fst4wave.f90` has no period branch).
pub const FST4_900_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 66_560,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 66_560 / 8,
};

/// FST4-1800 / FST4W-1800 GFSK configuration: 12 kHz, 134400 samples/symbol,
/// BT=2.0, hmod=1.0, NSPS/8-sample cosine ramp (`fst4_decode.f90:426`
/// `nsps=134400`).
pub const FST4_1800_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 134_400,
    bt: 2.0,
    hmod: 1.0,
    ramp_samples: 134_400 / 8,
};
