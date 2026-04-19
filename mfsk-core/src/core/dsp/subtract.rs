//! Successive interference cancellation (SIC) for phase-continuous MFSK.
//!
//! Given a tone sequence plus time/frequency coordinates, reconstructs the
//! ideal IQ waveform, estimates its complex amplitude by least-squares
//! projection onto the received signal, and subtracts a scaled copy in place.
//! Protocol-agnostic — the caller supplies tone sequence, sample rate, tone
//! spacing and timing so the same routine serves FT8/FT4/FT2/FST4.

use std::f32::consts::PI;

/// Fixed DSP parameters for a single subtraction call.
#[derive(Clone, Copy, Debug)]
pub struct SubtractCfg {
    /// PCM sample rate (Hz), e.g. 12 000 for the WSJT pipeline.
    pub sample_rate: f32,
    /// Tone spacing (Hz). FT8 = 6.25, FT4 = 20.833, …
    pub tone_spacing_hz: f32,
    /// Samples per FT symbol at `sample_rate`. FT8 = 1920, FT4 = 576, …
    pub samples_per_symbol: usize,
    /// Frame origin offset within the slot buffer, seconds. WSJT convention
    /// places `t = 0` of the transmitted frame at 0.5 s for FT8, 0.5 s for
    /// FT4 as well — so typically 0.5.
    pub base_offset_s: f32,
}

/// Generate phase-continuous cosine/sine references for a symbol stream.
///
/// Returns `(w_cos, w_sin)` each of length `tones.len() * cfg.samples_per_symbol`.
/// `freq_hz` is the carrier of tone 0.
fn generate_iq(tones: &[u8], freq_hz: f32, cfg: &SubtractCfg) -> (Vec<f32>, Vec<f32>) {
    let n = tones.len() * cfg.samples_per_symbol;
    let mut w_cos = vec![0.0f32; n];
    let mut w_sin = vec![0.0f32; n];
    let mut phase = 0.0f32;
    for (sym, &tone) in tones.iter().enumerate() {
        let freq = freq_hz + tone as f32 * cfg.tone_spacing_hz;
        let dphi = 2.0 * PI * freq / cfg.sample_rate;
        let base = sym * cfg.samples_per_symbol;
        for j in 0..cfg.samples_per_symbol {
            w_cos[base + j] = phase.cos();
            w_sin[base + j] = phase.sin();
            phase += dphi;
            if phase > PI {
                phase -= 2.0 * PI;
            }
        }
    }
    (w_cos, w_sin)
}

/// Subtract a tone sequence from `audio` in place, with a fractional gain.
///
/// `gain = 1.0` performs full least-squares subtraction. `gain < 1.0` is
/// useful when the channel is time-varying: over-subtraction would introduce
/// a negative-amplitude residual that poisons subsequent decode passes.
#[inline]
pub fn subtract_tones(
    audio: &mut [i16],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    gain: f32,
    cfg: &SubtractCfg,
) {
    let (w_cos, w_sin) = generate_iq(tones, freq_hz, cfg);

    let start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as usize;
    let len = w_cos.len().min(audio.len().saturating_sub(start));
    if len == 0 {
        return;
    }

    // Complex least-squares: rx[t] ≈ a·cos(φ(t)) + b·sin(φ(t))
    // cos / sin are near-orthogonal over the full frame so the closed-form
    // per-component projection matches the joint solve to floating-point
    // precision.
    let (num_a, num_b, den_a, den_b) =
        (0..len).fold((0.0f32, 0.0f32, 0.0f32, 0.0f32), |(na, nb, da, db), i| {
            let rx = audio[start + i] as f32;
            (
                na + rx * w_cos[i],
                nb + rx * w_sin[i],
                da + w_cos[i] * w_cos[i],
                db + w_sin[i] * w_sin[i],
            )
        });

    let a = if den_a > f32::EPSILON {
        num_a / den_a
    } else {
        0.0
    };
    let b = if den_b > f32::EPSILON {
        num_b / den_b
    } else {
        0.0
    };

    for i in 0..len {
        let sub = gain * (a * w_cos[i] + b * w_sin[i]);
        let new_val = audio[start + i] as f32 - sub;
        audio[start + i] = new_val.clamp(-32_768.0, 32_767.0) as i16;
    }
}
