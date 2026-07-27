//! Gaussian Frequency-Shift-Keying (GFSK) waveform synthesis.
//!
//! Protocol-agnostic: given an FSK tone sequence, produces phase-continuous
//! PCM with Gaussian-shaped frequency transitions. FT8/FT4/FT2/FST4 all use
//! this shape and differ only in samples-per-symbol, BT product and
//! modulation index (`hmod`). Tone *spacing* is implicitly
//! `sample_rate · hmod / samples_per_symbol` — no separate parameter needed.
//!
//! Ported from WSJT-X `gen_ft8wave.f90` + `gfsk_pulse.f90`.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::PI;
#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Runtime parameters of a GFSK waveform generator.
#[derive(Clone, Copy, Debug)]
pub struct GfskCfg {
    /// PCM sample rate in Hz (12 000 for WSJT).
    pub sample_rate: f32,
    /// Samples per modulation symbol (FT8 = 1920, FT4 = 576, …).
    pub samples_per_symbol: usize,
    /// Bandwidth-time product. FT8/FT4 use 2.0 (fairly wide Gaussian);
    /// FST4 uses 1.0.
    pub bt: f32,
    /// Modulation index. 1.0 for FT8 (orthogonal tones at `1/T` spacing).
    pub hmod: f32,
    /// Cosine ramp length at start/end of the waveform, in samples.
    /// `0` disables ramping. FT8 uses `samples_per_symbol / 8`.
    pub ramp_samples: usize,
}

/// Gaussian pulse matching WSJT-X `gfsk_pulse` (3-symbol wide).
#[inline]
fn gfsk_pulse(bt: f32, t: f32) -> f32 {
    let c = PI * (2.0_f32 / 2.0_f32.ln()).sqrt();
    0.5 * (erf(c * bt * (t + 0.5)) - erf(c * bt * (t - 0.5)))
}

/// Approximate erf(x) — Abramowitz & Stegun 7.1.26, accurate to ~1e-5.
#[inline]
fn erf(x: f32) -> f32 {
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254_829_6
            + t * (-0.284_496_72 + t * (1.421_413_8 + t * (-1.453_152_1 + t * 1.061_405_4))));
    sign * (1.0 - poly * (-x * x).exp())
}

/// Output sample count for [`synth_f32`] / [`synth_f32_into`] given a
/// tone-sequence length and the per-symbol sample count.
#[inline]
pub const fn synth_output_len(nsym: usize, samples_per_symbol: usize) -> usize {
    nsym * samples_per_symbol
}

/// Synthesise a PCM waveform from an FSK tone sequence into a caller-
/// provided output buffer. **No allocation** — `out` must already be
/// sized to [`synth_output_len`]`(tones.len(), cfg.samples_per_symbol)`.
/// Two `Vec`s are still allocated internally for the Gaussian pulse
/// table and the per-sample phase-rate buffer; the [`synth_f32`]
/// wrapper additionally allocates the output. Embedded callers driving
/// I2S DMA buffers should prefer this entry point.
///
/// - `tones[j]` is the integer tone index for symbol `j` (0..NTONES).
/// - `f0_hz` is the carrier (tone-0) frequency.
/// - `amplitude` is the peak of the f32 signal written to `out`
///   (typically 1.0).
///
/// Pipeline: build a per-sample phase-rate array `dphi` via a 3-symbol
/// Gaussian pulse shape, add the carrier offset, integrate → phase,
/// take `sin`. Finally, a half-cosine envelope of length
/// `cfg.ramp_samples` smooths both ends.
///
/// # Panics
///
/// Panics if `out.len() != synth_output_len(tones.len(),
/// cfg.samples_per_symbol)` or if `tones` is empty.
pub fn synth_f32_into(out: &mut [f32], tones: &[u8], f0_hz: f32, amplitude: f32, cfg: &GfskCfg) {
    let nsps = cfg.samples_per_symbol;
    let nsym = tones.len();
    assert!(nsym > 0, "synth_f32_into: empty tone sequence");
    let nwave = synth_output_len(nsym, nsps);
    assert_eq!(
        out.len(),
        nwave,
        "synth_f32_into: out.len() must equal synth_output_len()"
    );
    let twopi = 2.0 * PI;
    let dt = 1.0 / cfg.sample_rate;

    let pulse_len = 3 * nsps;
    let pulse: Vec<f32> = (0..pulse_len)
        .map(|i| {
            let tt = (i as f32 - 1.5 * nsps as f32) / nsps as f32;
            gfsk_pulse(cfg.bt, tt)
        })
        .collect();

    let total = (nsym + 2) * nsps;
    let mut dphi = vec![0.0f32; total];
    let dphi_peak = twopi * cfg.hmod / nsps as f32;

    for (j, &tone) in tones.iter().enumerate() {
        let ib = j * nsps;
        for i in 0..pulse_len {
            if ib + i < total {
                dphi[ib + i] += dphi_peak * pulse[i] * tone as f32;
            }
        }
    }

    // Dummy symbols (ramp-in / ramp-out for smooth pulse overlap)
    for i in 0..(2 * nsps).min(total) {
        dphi[i] += dphi_peak * tones[0] as f32 * pulse[nsps + i];
    }
    let ofs = nsym * nsps;
    for i in 0..(2 * nsps) {
        if ofs + i < total {
            dphi[ofs + i] += dphi_peak * tones[nsym - 1] as f32 * pulse[i];
        }
    }

    // Carrier
    for d in dphi.iter_mut() {
        *d += twopi * f0_hz * dt;
    }

    let mut phi = 0.0f32;
    for k in 0..nwave {
        out[k] = amplitude * phi.sin();
        phi += dphi[nsps + k];
        if phi > twopi {
            phi -= twopi;
        }
    }

    // Half-cosine envelope on each end
    let nramp = cfg.ramp_samples.min(nwave / 2);
    if nramp > 0 {
        for i in 0..nramp {
            let env = (1.0 - (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            out[i] *= env;
        }
        let k1 = nwave - nramp;
        for i in 0..nramp {
            let env = (1.0 + (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            out[k1 + i] *= env;
        }
    }
}

/// Complex GFSK synthesis: writes both `cos(phi)` and `sin(phi)` into
/// caller-provided buffers using the same phase progression as
/// [`synth_f32_into`]. Used by signal-cancellation paths that need an
/// IQ pair for least-squares amplitude estimation against arbitrary
/// channel phase.
///
/// Both `out_cos` and `out_sin` must have length
/// [`synth_output_len`]`(tones.len(), cfg.samples_per_symbol)`.
pub fn synth_complex_f32_into(
    out_cos: &mut [f32],
    out_sin: &mut [f32],
    tones: &[u8],
    f0_hz: f32,
    amplitude: f32,
    cfg: &GfskCfg,
) {
    let nsps = cfg.samples_per_symbol;
    let nsym = tones.len();
    assert!(nsym > 0, "synth_complex_f32_into: empty tone sequence");
    let nwave = synth_output_len(nsym, nsps);
    assert_eq!(
        out_cos.len(),
        nwave,
        "synth_complex_f32_into: out_cos.len() must equal synth_output_len()"
    );
    assert_eq!(
        out_sin.len(),
        nwave,
        "synth_complex_f32_into: out_sin.len() must equal synth_output_len()"
    );
    let twopi = 2.0 * PI;
    let dt = 1.0 / cfg.sample_rate;

    let pulse_len = 3 * nsps;
    let pulse: Vec<f32> = (0..pulse_len)
        .map(|i| {
            let tt = (i as f32 - 1.5 * nsps as f32) / nsps as f32;
            gfsk_pulse(cfg.bt, tt)
        })
        .collect();

    let total = (nsym + 2) * nsps;
    let mut dphi = vec![0.0f32; total];
    let dphi_peak = twopi * cfg.hmod / nsps as f32;

    for (j, &tone) in tones.iter().enumerate() {
        let ib = j * nsps;
        for i in 0..pulse_len {
            if ib + i < total {
                dphi[ib + i] += dphi_peak * pulse[i] * tone as f32;
            }
        }
    }
    for i in 0..(2 * nsps).min(total) {
        dphi[i] += dphi_peak * tones[0] as f32 * pulse[nsps + i];
    }
    let ofs = nsym * nsps;
    for i in 0..(2 * nsps) {
        if ofs + i < total {
            dphi[ofs + i] += dphi_peak * tones[nsym - 1] as f32 * pulse[i];
        }
    }
    for d in dphi.iter_mut() {
        *d += twopi * f0_hz * dt;
    }

    let mut phi = 0.0f32;
    for k in 0..nwave {
        out_cos[k] = amplitude * phi.cos();
        out_sin[k] = amplitude * phi.sin();
        phi += dphi[nsps + k];
        if phi > twopi {
            phi -= twopi;
        }
    }

    // Half-cosine envelope on each end (same as synth_f32_into).
    let nramp = cfg.ramp_samples.min(nwave / 2);
    if nramp > 0 {
        for i in 0..nramp {
            let env = (1.0 - (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            out_cos[i] *= env;
            out_sin[i] *= env;
        }
        let k1 = nwave - nramp;
        for i in 0..nramp {
            let env = (1.0 + (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            out_cos[k1 + i] *= env;
            out_sin[k1 + i] *= env;
        }
    }
}

/// Synthesise a PCM waveform from an FSK tone sequence.
///
/// Vec-returning convenience wrapper for [`synth_f32_into`]. Allocates
/// the output, then forwards.
#[inline]
pub fn synth_f32(tones: &[u8], f0_hz: f32, amplitude: f32, cfg: &GfskCfg) -> Vec<f32> {
    let nwave = synth_output_len(tones.len(), cfg.samples_per_symbol);
    let mut out = vec![0.0f32; nwave];
    synth_f32_into(&mut out, tones, f0_hz, amplitude, cfg);
    out
}

/// i16 variant of [`synth_f32_into`]. The peak value of the PCM written
/// to `out` equals `amplitude_i16`.
pub fn synth_i16_into(
    out: &mut [i16],
    tones: &[u8],
    f0_hz: f32,
    amplitude_i16: i16,
    cfg: &GfskCfg,
) {
    let nsps = cfg.samples_per_symbol;
    let nwave = synth_output_len(tones.len(), nsps);
    assert_eq!(
        out.len(),
        nwave,
        "synth_i16_into: out.len() must equal synth_output_len()"
    );
    let mut tmp = vec![0.0f32; nwave];
    synth_f32_into(&mut tmp, tones, f0_hz, 1.0, cfg);
    let scale = amplitude_i16 as f32;
    for (dst, &src) in out.iter_mut().zip(tmp.iter()) {
        *dst = (src * scale) as i16;
    }
}

/// i16 variant: peak value of the returned PCM equals `amplitude_i16`.
#[inline]
pub fn synth_i16(tones: &[u8], f0_hz: f32, amplitude_i16: i16, cfg: &GfskCfg) -> Vec<i16> {
    let nwave = synth_output_len(tones.len(), cfg.samples_per_symbol);
    let mut out = vec![0i16; nwave];
    synth_i16_into(&mut out, tones, f0_hz, amplitude_i16, cfg);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ft8_cfg() -> GfskCfg {
        GfskCfg {
            sample_rate: 12_000.0,
            samples_per_symbol: 1920,
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 240,
        }
    }

    #[test]
    fn synth_into_matches_vec_returning_variant() {
        // The caller-buffer API must be byte-identical to the
        // Vec-returning convenience wrapper.
        let cfg = ft8_cfg();
        let tones: [u8; 8] = [0, 1, 7, 3, 4, 5, 6, 2];
        let f0 = 1500.0;
        let amp = 0.7;
        let from_vec = synth_f32(&tones, f0, amp, &cfg);
        let mut into_buf = vec![0.0f32; synth_output_len(tones.len(), cfg.samples_per_symbol)];
        synth_f32_into(&mut into_buf, &tones, f0, amp, &cfg);
        assert_eq!(from_vec, into_buf);
    }

    #[test]
    #[should_panic(expected = "out.len()")]
    fn synth_into_panics_on_wrong_buffer_size() {
        let cfg = ft8_cfg();
        let tones: [u8; 4] = [0, 1, 2, 3];
        let mut buf = vec![0.0f32; 100]; // wrong size
        synth_f32_into(&mut buf, &tones, 1500.0, 1.0, &cfg);
    }
}
