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

use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;
use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::ModulationParams;
use crate::engine::fft::default_planner;

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
    // Sub-bin freq accuracy: split base_freq_hz into the nearest bin
    // plus a residual offset, then mix the audio by `-residual` so the
    // carrier of tone 0 lands exactly on `base_bin` regardless of the
    // sub-bin offset. Without this, an off-bin carrier (typical for
    // WSPR — 1.46 Hz bins, signals routinely sit ±0.7 Hz off-bin)
    // loses up to ~50 % of its energy to FFT scalloping and the
    // Fano decoder converges to noise instead of the correct payload.
    let bin_pos = base_freq_hz / df;
    let base_bin = bin_pos.round() as usize;
    let residual_hz = base_freq_hz - base_bin as f32 * df;
    // Bail out if the caller asked for a window that doesn't fit.
    if start_sample + 162 * nsps > audio.len() || base_bin + 4 >= nsps / 2 {
        return None;
    }

    let mut planner = default_planner();
    let fft = planner.plan_forward(nsps);
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0.0f32, 0.0); nsps];

    let mut mags = Vec::with_capacity(162);
    let mut noise_acc = 0.0f32;
    let mut noise_count = 0u32;

    let mix_w = -2.0 * PI * residual_hz / sample_rate as f32;

    for i in 0..162 {
        let sym_start = start_sample + i * nsps;
        // Mix `audio[sym_start..]` by `exp(-j 2π residual t)` (absolute
        // sample index `t` so phase is continuous across symbols).
        if residual_hz.abs() > 1e-6 {
            for k in 0..nsps {
                let abs_n = sym_start + k;
                let ph = mix_w * abs_n as f32;
                let s = audio[abs_n];
                buf[k] = Complex::new(s * ph.cos(), s * ph.sin());
            }
        } else {
            for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
                *slot = Complex::new(s, 0.0);
            }
        }
        // The trait does in-place; allocates its own scratch internally
        // (rustfft) or operates true in-place (microfft).
        fft.process(&mut buf);

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

/// Convert per-symbol tone magnitudes to 162 data-bit soft metrics.
///
/// Mirrors WSJT-X `wsprd.c::noncoherent_sequence_detection` (line 465+):
/// `bm[i] = max_one_magnitude − max_zero_magnitude`, then z-score
/// normalise by the population standard deviation. Magnitude-based
/// (NOT power-based — earlier `(m_e² − m_o²) / σ²` was a square-law
/// metric that doesn't match wsprd's linear-difference detector and
/// lost weak signals like NM7J at -1 dB SNR).
///
/// The sync vector (`WSPR_SYNC_VECTOR`) selects which pair of tones
/// carries the data bit at symbol `i`:
///   sync = 0 → tones (0, 2) carry data bit (= 0 vs 1)
///   sync = 1 → tones (1, 3) carry data bit
pub fn mags_to_llrs(tm: &ToneMagnitudes) -> [f32; 162] {
    let mut bmet = [0.0f32; 162];
    for i in 0..162 {
        let sync = WSPR_SYNC_VECTOR[i];
        let (e, o) = if sync == 0 {
            (tm.mags[i][0], tm.mags[i][2])
        } else {
            (tm.mags[i][1], tm.mags[i][3])
        };
        bmet[i] = e - o;
    }

    // z-score normalise by population std (matches WSJT-X
    // `normalizebmet` in `ft8b.f90:466-479`).
    let n = bmet.len() as f32;
    let mean = bmet.iter().sum::<f32>() / n;
    let mean_sq = bmet.iter().map(|&x| x * x).sum::<f32>() / n;
    let var = mean_sq - mean * mean;
    let sig = if var > 0.0 {
        var.sqrt()
    } else {
        mean_sq.sqrt()
    };
    if sig > 0.0 {
        for x in bmet.iter_mut() {
            *x /= sig;
        }
    }
    // Scale to match wsprd. wsprd `wsprd.c:477` uses `symfac=50`
    // then clamps to ±127 before quantising as u8. Our Fano takes
    // float LLRs scaled by `SCALE`; matching `symfac` directly gives
    // the FEC the same effective signal magnitude.
    let _ = tm.noise_power_est; // no longer used; kept to avoid struct churn
    const SCALE: f32 = 2.83;
    for x in bmet.iter_mut() {
        *x *= SCALE;
    }
    bmet
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
