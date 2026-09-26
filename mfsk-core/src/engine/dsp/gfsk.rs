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
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
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
pub(crate) fn gfsk_pulse(bt: f32, t: f32) -> f32 {
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

/// How often the streaming synthesiser renormalises its phasor — the
/// same period [`super::ddc`]'s mixer uses, for the same reason.
const STREAM_RENORM_PERIOD: u32 = 4096;

/// A GMFSK waveform produced in chunks, holding no buffer of its own.
///
/// **Why this exists.** [`synth_f32_into`] builds the whole waveform
/// before returning a sample: a `dphi` array of `(nsym + 2) * nsps`
/// floats, and [`synth_i16_into`] adds a second full-length f32
/// temporary on top. For FT8 that is 622 KB and 607 KB, and on a
/// CoreS3 the pair costs **472 ms** for one 12.64 s frame — measured
/// 2026-09-20. Against WSJT-X's own transmit schedule (audio at
/// `slot end + 0.5 s`, PTT at the boundary, ~100 ms of rig settle)
/// that puts the decoder's deadline *before* the slot it is decoding
/// has ended. There is no schedule that works with the synthesis on
/// the critical path.
///
/// A transmitter streams to a DMA in chunks — `tx::play` already sends
/// 20 ms at a time — so the waveform never has to exist all at once.
/// Per chunk the same work spreads across the 12.64 s of playback: a
/// 3.7 % duty instead of a deadline.
///
/// **Nothing about the waveform changes.** The 3-symbol Gaussian pulse
/// overlap, the dummy ramp-in and ramp-out symbols that WSJT-X's
/// `gen_ft8wave.f90` uses to keep the pulse train continuous at the
/// ends, and the half-cosine envelope over `cfg.ramp_samples` are all
/// reproduced; `stream_matches_the_reference_synthesiser` pins the
/// output against [`synth_f32`] sample for sample.
///
/// **No `sin` per sample.** The phase increment varies, so the fixed
/// step `super::ddc`'s mixer uses does not apply directly — but it
/// splits: a constant carrier rotation, times a *small* modulation
/// rotation. The modulation increment is at most
/// `dphi_peak * (NTONES - 1)` ≈ 0.023 rad for FT8, where a three-term
/// series for `cos`/`sin` is accurate to ~1e-8, so the per-sample cost
/// is two complex multiplies rather than a libm call. The phasor is
/// renormalised on the same schedule the DDC uses, for the same
/// accumulation reason.
pub struct GfskStream {
    nsps: usize,
    nsym: usize,
    nwave: usize,
    ramp: usize,
    dphi_peak: f32,
    /// The 3-symbol Gaussian pulse, `3 * nsps` long. The only
    /// allocation, and it is 23 KB for FT8 against the 1.2 MB the
    /// batch path holds.
    pulse: Vec<f32>,
    tones: Vec<u8>,
    /// `exp(i · 2π f0 / Fs)`, the part of the phase increment that
    /// does not depend on the sample.
    carrier: (f32, f32),
    phasor: (f32, f32),
    k: usize,
    since_renorm: u32,
}

impl GfskStream {
    /// Prepare to synthesise `tones` at carrier `f0_hz`.
    ///
    /// # Panics
    ///
    /// Panics if `tones` is empty.
    pub fn new(tones: &[u8], f0_hz: f32, cfg: &GfskCfg) -> Self {
        let nsym = tones.len();
        assert!(nsym > 0, "GfskStream::new: empty tone sequence");
        let nsps = cfg.samples_per_symbol;
        let pulse_len = 3 * nsps;
        let pulse: Vec<f32> = (0..pulse_len)
            .map(|i| {
                let tt = (i as f32 - 1.5 * nsps as f32) / nsps as f32;
                gfsk_pulse(cfg.bt, tt)
            })
            .collect();
        let dphi_carrier = 2.0 * PI * f0_hz / cfg.sample_rate;
        Self {
            nsps,
            nsym,
            nwave: synth_output_len(nsym, nsps),
            ramp: cfg.ramp_samples.min(synth_output_len(nsym, nsps) / 2),
            dphi_peak: 2.0 * PI * cfg.hmod / nsps as f32,
            pulse,
            tones: tones.to_vec(),
            carrier: (dphi_carrier.cos(), dphi_carrier.sin()),
            phasor: (1.0, 0.0),
            k: 0,
            since_renorm: 0,
        }
    }

    /// Samples not yet produced.
    pub fn remaining(&self) -> usize {
        self.nwave - self.k
    }

    /// The modulation part of the phase increment for output sample
    /// `k`, i.e. `dphi[nsps + k]` in [`synth_f32_into`]'s array minus
    /// the carrier term.
    ///
    /// At most three symbols' pulses overlap any sample, so this is
    /// O(1) where the batch path pays a full array. The two dummy
    /// terms are the ramp-in and ramp-out symbols.
    fn modulation(&self, k: usize) -> f32 {
        let nsps = self.nsps;
        let m = nsps + k;
        let mut d = 0.0f32;
        let jhi = m / nsps;
        for j in jhi.saturating_sub(2)..=jhi {
            if j < self.nsym {
                let idx = m - j * nsps;
                if idx < self.pulse.len() {
                    d += self.dphi_peak * self.pulse[idx] * self.tones[j] as f32;
                }
            }
        }
        if m < 2 * nsps {
            d += self.dphi_peak * self.tones[0] as f32 * self.pulse[nsps + m];
        }
        let ofs = self.nsym * nsps;
        if m >= ofs && m - ofs < 2 * nsps {
            d += self.dphi_peak * self.tones[self.nsym - 1] as f32 * self.pulse[m - ofs];
        }
        d
    }

    /// The half-cosine envelope at sample `k`, `1.0` in the middle.
    fn envelope(&self, k: usize) -> f32 {
        if self.ramp == 0 {
            return 1.0;
        }
        let twopi = 2.0 * PI;
        let n = self.ramp as f32;
        if k < self.ramp {
            (1.0 - (twopi * k as f32 / (2.0 * n)).cos()) / 2.0
        } else if k >= self.nwave - self.ramp {
            let i = k - (self.nwave - self.ramp);
            (1.0 + (twopi * i as f32 / (2.0 * n)).cos()) / 2.0
        } else {
            1.0
        }
    }

    #[inline]
    fn advance(&mut self) {
        // `exp(i·d) = exp(i·carrier) · exp(i·mod)`, the second from a
        // three-term series — see the type's doc for why that is
        // enough here and a libm call is not needed.
        let md = self.modulation(self.k);
        let md2 = md * md;
        let mod_rot = (1.0 - 0.5 * md2 + md2 * md2 / 24.0, md - md2 * md / 6.0);
        let (pr, pi) = self.phasor;
        let (cr, ci) = self.carrier;
        let (ar, ai) = (pr * cr - pi * ci, pr * ci + pi * cr);
        self.phasor = (
            ar * mod_rot.0 - ai * mod_rot.1,
            ar * mod_rot.1 + ai * mod_rot.0,
        );
        self.k += 1;
        self.since_renorm += 1;
        if self.since_renorm >= STREAM_RENORM_PERIOD {
            let (r, i) = self.phasor;
            let mag = (r * r + i * i).sqrt();
            if mag > 0.0 {
                self.phasor = (r / mag, i / mag);
            }
            self.since_renorm = 0;
        }
    }

    /// Fill `out` with the next samples, returning how many were
    /// written. Short only at the end of the waveform.
    pub fn fill_f32(&mut self, out: &mut [f32], amplitude: f32) -> usize {
        let n = out.len().min(self.remaining());
        for slot in out.iter_mut().take(n) {
            *slot = amplitude * self.phasor.1 * self.envelope(self.k);
            self.advance();
        }
        n
    }

    /// [`Self::fill_f32`] straight to i16, with no intermediate buffer
    /// — the 607 KB temporary and the 82 ms copy `synth_i16_into`
    /// spends are what this exists to avoid.
    pub fn fill_i16(&mut self, out: &mut [i16], amplitude_i16: i16) -> usize {
        let scale = amplitude_i16 as f32;
        let n = out.len().min(self.remaining());
        for slot in out.iter_mut().take(n) {
            *slot = (scale * self.phasor.1 * self.envelope(self.k)) as i16;
            self.advance();
        }
        n
    }
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

#[cfg(test)]
mod stream_tests {
    use super::*;

    /// FT8's own configuration — the one the measurement that
    /// motivated the stream was taken on.
    const FT8: GfskCfg = GfskCfg {
        sample_rate: 12_000.0,
        samples_per_symbol: 1920,
        bt: 2.0,
        hmod: 1.0,
        ramp_samples: 1920 / 8,
    };

    fn tones() -> Vec<u8> {
        // A Costas-ish spread so every tone index is exercised and the
        // pulse overlap has something to do at each boundary.
        (0..79u32).map(|i| ((i * 5 + 3) % 8) as u8).collect()
    }

    /// **The stream is the same waveform.** Not "close enough": the
    /// Gaussian overlap, the dummy ramp symbols and the half-cosine
    /// envelope all have to survive being produced a chunk at a time,
    /// and the phasor has to track a `sin` it never calls.
    #[test]
    fn stream_matches_the_reference_synthesiser() {
        let t = tones();
        let reference = synth_f32(&t, 1_500.0, 1.0, &FT8);
        let mut stream = GfskStream::new(&t, 1_500.0, &FT8);
        let mut got = vec![0f32; reference.len()];
        // Deliberately awkward chunking: 20 ms is what `tx::play`
        // sends, and a ragged first chunk proves the state carries.
        let mut at = 0usize;
        for chunk in [7usize, 240, 1, 1920, 240].into_iter().cycle() {
            if at >= got.len() {
                break;
            }
            let end = (at + chunk).min(got.len());
            let n = stream.fill_f32(&mut got[at..end], 1.0);
            assert_eq!(n, end - at, "stream ran short at {at}");
            at = end;
        }
        assert_eq!(stream.remaining(), 0);

        let worst = reference
            .iter()
            .zip(got.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f32, f32::max);
        // The phasor accumulates where the reference re-derives `sin`
        // from a running scalar, so exact equality is not the claim.
        // 1e-3 of full scale is ~60 dB below the signal and two orders
        // below the quantisation an i16 transmit path applies next.
        assert!(
            worst < 1e-3,
            "stream deviates from the reference by {worst} of full scale"
        );
    }

    /// The envelope is not optional: without it the transmitter keys a
    /// step, and the splatter is the reason WSJT-X ramps at all.
    #[test]
    fn the_ramp_survives_chunking() {
        let t = tones();
        let mut stream = GfskStream::new(&t, 1_500.0, &FT8);
        let mut got = vec![0f32; synth_output_len(t.len(), FT8.samples_per_symbol)];
        let mut at = 0;
        while at < got.len() {
            let end = (at + 240).min(got.len());
            at += stream.fill_f32(&mut got[at..end], 1.0);
        }
        let n = FT8.ramp_samples;
        // First and last samples are inside the ramp's zero end.
        assert!(got[0].abs() < 1e-6, "waveform starts at {}", got[0]);
        assert!(
            got[got.len() - 1].abs() < 0.02,
            "waveform ends at {}",
            got[got.len() - 1]
        );
        // And the middle of the ramp is genuinely attenuated, not just
        // the endpoints.
        let mid_ramp = got[..n].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        let body = got[n..got.len() - n]
            .iter()
            .fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(
            mid_ramp < body,
            "ramp peak {mid_ramp} not below body peak {body}"
        );
    }

    /// i16 without the f32 round trip must land where the f32 path
    /// scaled to i16 would have.
    #[test]
    fn i16_fill_matches_scaling_the_f32_fill() {
        let t = tones();
        let f = synth_f32(&t, 1_500.0, 1.0, &FT8);
        let mut stream = GfskStream::new(&t, 1_500.0, &FT8);
        let mut got = vec![0i16; f.len()];
        let mut at = 0;
        while at < got.len() {
            let end = (at + 960).min(got.len());
            at += stream.fill_i16(&mut got[at..end], 20_000);
        }
        let worst = f
            .iter()
            .zip(got.iter())
            .map(|(a, b)| ((a * 20_000.0) as i32 - *b as i32).abs())
            .max()
            .unwrap_or(0);
        assert!(worst <= 24, "i16 stream differs by {worst} counts");
    }
}
