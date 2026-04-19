//! WSPR receiver path: audio samples → 162 per-symbol data-bit LLRs.
//!
//! ## Geometry
//!
//! The only piece of luck the protocol hands us: at 12 kHz sample rate
//! and `NSPS = 8192`, a single-symbol FFT has bin width `12000/8192 =
//! 1.4648 Hz`, exactly one WSPR tone spacing. So a 256-sample FFT at
//! 375 Hz, or an 8192-sample FFT at 12 kHz, lands each tone on its own
//! bin with no leakage between tones. We take the 12 kHz version
//! directly — no downsampling step, no polyphase filter — one FFT per
//! symbol gives the four tone powers we need.
//!
//! ## What this module does
//!
//! Given already-aligned audio (caller knows the start sample and base
//! frequency), emit 162 LLRs — one per channel symbol, in **coded-bit
//! order** (i.e. still interleaved, matching the order the convolutional
//! encoder produced). The caller runs [`super::deinterleave`] on the
//! LLRs and feeds them to the Fano decoder.
//!
//! ## What this module does *not* do
//!
//! No coarse frequency search, no time-offset refinement. The caller
//! must supply the approximate base frequency (the "tone 0" bin) and
//! the nominal audio start index. A follow-up module will wrap this
//! with a peak-search over the sync-vector correlation metric.

use crate::core::ModulationParams;
use num_complex::Complex;
use rustfft::FftPlanner;

use super::{WSPR_SYNC_VECTOR, Wspr};

/// Per-symbol 4-tone magnitudes at a hypothesised alignment.
///
/// Returned entry `[mags, noise_est]`: `mags[i][t]` is the FFT-bin
/// magnitude at `base_bin + t` for symbol `i`; `noise_est` is the mean
/// |bin|² across a few off-tone reference bins, used both for LLR
/// scaling and as a cheap noise floor for sync-score thresholding.
#[derive(Clone)]
pub struct ToneMagnitudes {
    pub mags: Vec<[f32; 4]>, // 162 entries
    pub noise_power_est: f32,
}

/// Run 162 symbol-length FFTs at the hypothesised (start_sample, freq)
/// and collect the four tone magnitudes per symbol. No LLR conversion,
/// no sync information — this is the primitive that both coarse search
/// and final demod build on.
pub fn extract_tone_magnitudes(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<ToneMagnitudes> {
    let nsps = (sample_rate as f32 * <Wspr as ModulationParams>::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps as f32;
    let base_bin = (base_freq_hz / df).round() as usize;
    // Bail out if the caller asked for a window that doesn't fit.
    if start_sample + 162 * nsps > audio.len() || base_bin + 4 >= nsps / 2 {
        return None;
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0.0f32, 0.0); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0.0f32, 0.0); nsps];

    let mut mags = Vec::with_capacity(162);
    let mut noise_acc = 0.0f32;
    let mut noise_count = 0u32;

    for i in 0..162 {
        let sym_start = start_sample + i * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(&mut buf, &mut scratch);

        mags.push([
            buf[base_bin].norm(),
            buf[base_bin + 1].norm(),
            buf[base_bin + 2].norm(),
            buf[base_bin + 3].norm(),
        ]);

        // Noise reference: a few bins just above the signal passband.
        for k in 4..8 {
            let bin = base_bin + k;
            if bin < nsps / 2 {
                noise_acc += buf[bin].norm_sqr();
                noise_count += 1;
            }
        }
    }

    let noise_power_est = if noise_count > 0 {
        noise_acc / noise_count as f32
    } else {
        1.0
    };
    Some(ToneMagnitudes {
        mags,
        noise_power_est,
    })
}

/// Convert per-symbol tone magnitudes to 162 data-bit LLRs using the
/// known sync vector to pick which pair of tones carries the data bit.
/// Returns LLRs clamped to ±20 for numeric stability in the downstream
/// integer-metric Fano decoder.
pub fn mags_to_llrs(tm: &ToneMagnitudes) -> [f32; 162] {
    let mut m_even = [0.0f32; 162];
    let mut m_odd = [0.0f32; 162];
    for i in 0..162 {
        let sync = WSPR_SYNC_VECTOR[i];
        let (e, o) = if sync == 0 {
            (tm.mags[i][0], tm.mags[i][2])
        } else {
            (tm.mags[i][1], tm.mags[i][3])
        };
        m_even[i] = e;
        m_odd[i] = o;
    }

    let mean_sig_power = m_even
        .iter()
        .chain(m_odd.iter())
        .map(|&m| m * m)
        .sum::<f32>()
        / (2.0 * 162.0);
    let sigma2 = tm.noise_power_est.max(mean_sig_power * 1e-4);

    let mut llrs = [0f32; 162];
    for i in 0..162 {
        let raw = (m_even[i] * m_even[i] - m_odd[i] * m_odd[i]) / sigma2;
        llrs[i] = raw.clamp(-20.0, 20.0);
    }
    llrs
}

/// Coarse sync score at a hypothesised alignment.
///
/// Computes two quantities over the 162 symbols: **sync-consistent
/// power** (sum of squared magnitudes in the two tones whose LSB
/// matches the known `WSPR_SYNC_VECTOR`) and **off-power** (same sum
/// for the two sync-inconsistent tones). The score is the normalised
/// *excess* of sync power over off power:
///
/// ```text
/// score = (sync - off) / (sync + off + noise_floor_162x)
/// ```
///
/// where `noise_floor_162x = 162 * noise_power_est` acts as a floor so
/// that empty / low-SNR candidates get squashed toward zero instead of
/// producing noisy ±1 scores. A correctly-aligned clean signal scores
/// near 1.0; an alignment where no signal lands in the window scores
/// near 0; a misaligned window that accidentally routes all captured
/// signal into sync tones still scores *lower* than the true alignment
/// because its absolute sync power is smaller.
pub fn sync_score(tm: &ToneMagnitudes) -> f32 {
    let mut sync_pwr = 0.0f32;
    let mut off_pwr = 0.0f32;
    for i in 0..162 {
        let mags = tm.mags[i];
        let (s_a, s_b, o_a, o_b) = if WSPR_SYNC_VECTOR[i] == 0 {
            (mags[0], mags[2], mags[1], mags[3])
        } else {
            (mags[1], mags[3], mags[0], mags[2])
        };
        sync_pwr += s_a * s_a + s_b * s_b;
        off_pwr += o_a * o_a + o_b * o_b;
    }
    let noise_floor = tm.noise_power_est * 162.0;
    let denom = sync_pwr + off_pwr + noise_floor;
    if denom > 0.0 {
        (sync_pwr - off_pwr) / denom
    } else {
        0.0
    }
}

/// Back-compat wrapper: the original "demodulate aligned → LLRs" path.
/// Equivalent to `mags_to_llrs(&extract_tone_magnitudes(..).unwrap_or_zero())`.
pub fn demodulate_aligned(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> [f32; 162] {
    match extract_tone_magnitudes(audio, sample_rate, start_sample, base_freq_hz) {
        Some(tm) => mags_to_llrs(&tm),
        None => [0f32; 162],
    }
}

#[cfg(test)]
mod tests {
    use super::super::tx::synthesize_audio;
    use super::*;

    #[test]
    fn recovers_llr_sign_noise_free() {
        // Symbols with alternating data bits, sync forced to zero for
        // simplicity (fake sync — real sync comes from WSPR_SYNC_VECTOR).
        let mut symbols = [0u8; 162];
        for i in 0..162 {
            let data_bit = (i & 1) as u8;
            let sync = WSPR_SYNC_VECTOR[i];
            symbols[i] = 2 * data_bit + sync;
        }
        let audio = synthesize_audio(&symbols, 12_000, 1500.0, 0.3);
        let llrs = demodulate_aligned(&audio, 12_000, 0, 1500.0);

        // Each LLR's sign should match the data bit: bit=0 → positive.
        for i in 0..162 {
            let expect_positive = (i & 1) == 0;
            if expect_positive {
                assert!(
                    llrs[i] > 0.0,
                    "symbol {} LLR should be > 0, got {}",
                    i,
                    llrs[i]
                );
            } else {
                assert!(
                    llrs[i] < 0.0,
                    "symbol {} LLR should be < 0, got {}",
                    i,
                    llrs[i]
                );
            }
        }
    }
}
