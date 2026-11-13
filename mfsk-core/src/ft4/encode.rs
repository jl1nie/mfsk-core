//! FT4's GFSK configuration — what `Ft4`'s
//! [`crate::engine::tx::FskWaveform`] impl points at. The transmit chain
//! is generic since #391: [`crate::engine::tx::message_to_tones`] then
//! [`crate::engine::tx::synthesize`].

use crate::engine::dsp::gfsk::GfskCfg;

/// FT4 GFSK configuration: 12 kHz, 576 samples/symbol, BT=1.0, hmod=1.0.
/// BT=1.0 matches WSJT-X `lib/ft4/gen_ft4wave.f90` (`gfsk_pulse(1.0, tt)`).
/// The waveform is `gen_ft4wave`'s: one ramp symbol each side of the 103 tone
/// symbols (`FrameLayout::N_RAMP`), the ramp a whole symbol long
/// (`ramp_samples` = `samples_per_symbol`), see
/// [`crate::engine::dsp::gfsk::synth_f32_lead_tail_into`].
pub const FT4_GFSK: GfskCfg = GfskCfg {
    sample_rate: 12_000.0,
    samples_per_symbol: 576,
    bt: 1.0,
    hmod: 1.0,
    ramp_samples: 576,
};
