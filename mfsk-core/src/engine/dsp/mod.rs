//! Generic DSP primitives shared by every MFSK protocol.
//!
//! Nothing in this module knows about FT8, FT4 or any specific modulation —
//! it operates on raw sample buffers, sample rates, and target frequencies.
//! Protocol-aware DSP (sync correlators, LLR, etc.) lives outside `dsp`.

// `downsample` requires an `Fft` backend; gate on the meta-feature
// (true if any of fft-rustfft / fft-microfft / fft-extern is on).
// `subtract` is FFT-free now (no rustfft consumer); always available
// once we have alloc.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod analytic;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod downsample;
pub mod envelope;
pub mod fft_15;
pub mod fft_mixed_3840;
pub mod fft_mixed_3840_sc16;
pub mod fft_sc16_r2;
pub mod gfsk;
pub mod msk;
pub mod resample;
pub mod subtract;

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use analytic::analytic_signal;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use downsample::{DownsampleCfg, build_fft_cache, downsample, downsample_cached};
pub use gfsk::{GfskCfg, synth_f32, synth_i16};
pub use resample::{LinearResamplerI16To12k, resample_f32_to_12k, resample_to_12k};
pub use subtract::{SubtractCfg, subtract_tones};
