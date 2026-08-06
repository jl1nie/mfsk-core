//! WSJT-X-equivalent coarse search on the 375 Hz complex baseband.
//!
//! Faithful port of the candidate-detection + 3-D refinement stages of
//! `wsprd.c::main` (lines 980-1197). The stock 12 kHz `Spectrogram` /
//! `coarse_search` pipeline is geometrically wrong for weak WSPR
//! signals: the 8192-pt FFT puts each tone on its own bin (1.46 Hz)
//! but loses ~3 dB of sub-bin signals to scalloping, and the score
//! landscape often peaks at the wrong dt for low-SNR candidates next
//! to strong ones (W5BIT and W3BI both fail this way on the WSJT-X
//! golden sample). wsprd uses a half-resolution baseband FFT (0.73 Hz/
//! bin) with a sin-window, time-averages first to estimate noise, and
//! only then runs the per-candidate (freq, time, drift) refinement —
//! a structure this module reproduces.
//!
//! # Pipeline
//! 1. **Stride-128 / 512-pt FFT** on the complex baseband, windowed by
//!    `w[j] = sin(π·j/512)`. ~359 time slices for a full 122 s slot,
//!    `df_baseband = 375/512 ≈ 0.7324 Hz`.
//! 2. **Time-averaged power spectrum** `psavg[512]`, then 7-pt smooth
//!    restricted to `±150 Hz` around 1500 Hz → `smspec[411]`.
//! 3. **Noise floor** = 30 th-percentile of smspec. Renormalise:
//!    `smspec[j] = smspec[j] / noise_level − 1`, clamped to `min_snr`.
//! 4. **Local-maxima peak detection** on smspec, ranked by SNR.
//! 5. **Per-peak 3-D refinement** over `ifr ± 2`, `k0 ∈ [−10, 22)`,
//!    `idrift ∈ ±max_drift`, scoring
//!    `ss = Σ_k (2·pr3[k]−1)·((p1+p3)−(p0+p2))` with the four sub-bin
//!    tones at `ifr ± 0.5·df`, `ifr ± 1.5·df` (= bins `ifd ± 1, 3` on
//!    the 0.73 Hz/bin baseband grid).
//!
//! Output: a `Vec<BasebandCandidate>` ranked by sync score, each with
//! the **tone-0** audio frequency (matching `coarse_search::SyncCandidate`
//! convention) and an audio-rate `start_sample` to feed
//! `decode_at_baseband`.

use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;
use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::dsp::downsample::with_default_planner;

use super::WSPR_SYNC_VECTOR;
use super::baseband::{BASEBAND_RATE, CENTER_HZ};
use super::demod::TONE_SPACING_HZ;

/// FFT size for the baseband spectrogram. Matches `wsprd.c:976`.
const NFFT: usize = 512;
/// Stride between successive FFTs (in baseband samples). 128 samples
/// = 512 / 4 = half a WSPR symbol on the baseband. Matches `wsprd.c`.
const STRIDE: usize = 128;
/// Half-bandwidth of the working band, in bins around 1500 Hz center.
/// 411 bins × `375/512 Hz/bin` ≈ ±150 Hz. Matches `wsprd.c:1037` (411).
const WORKING_BINS: usize = 411;
/// Minimum SNR floor in linear (`10^(-8/10)`); any smspec point below
/// this gets clamped to `0.1·min_snr`. Matches `wsprd.c:1058`.
const MIN_SNR_LIN: f32 = 0.158_489_32; // 10^(-8/10)
/// SNR scaling factor applied to the local SNR estimate, in dB.
/// `wsprd.c:1063` uses 26.3 for WSPR-2 to convert from the WSPR
/// bandwidth to a 2500-Hz reference. We pass it through as-is so the
/// `snr_db` field has the same calibration as wsprd's spot output.
const SNR_SCALING_DB: f32 = 26.3;

/// Candidate alignment from the wsprd-equivalent coarse path.
///
/// `freq_hz` follows the **tone-0** convention (matches the existing
/// `coarse_search::SyncCandidate.freq_hz`); `decode_at_baseband` adds
/// `+1.5·tone_spacing` internally to recover the signal centre.
#[derive(Clone, Copy, Debug)]
pub struct BasebandCandidate {
    /// Audio-rate sample where symbol 0 starts. Can be 0 if the signal
    /// began before the buffer; callers (`decode_scan`) typically prepend
    /// a 3 s zero pad to make all alignments addressable.
    pub start_sample: usize,
    /// Tone-0 audio frequency in Hz (signal centre minus
    /// `1.5·TONE_SPACING_HZ`).
    pub freq_hz: f32,
    /// Linear-drift coefficient. wsprd searches `±maxdrift` Hz over
    /// the slot; 0 means a stable carrier.
    pub drift_hz: f32,
    /// Sync-vector correlation score in `[0, 1]`. ≈ 1.0 is a clean
    /// alignment, ≈ 0 is empty.
    pub sync: f32,
    /// Per-bin SNR estimate from `smspec`, in dB and **WSPR-bandwidth
    /// referenced** (already passed through `−SNR_SCALING_DB`).
    pub snr_db: f32,
}

/// Time-averaged baseband spectrogram + smoothed/normalised spectrum.
struct Spectro {
    /// `ps[t * NFFT + j]` = |FFT|² at time slice `t`, bin `j` (DC at
    /// bin `NFFT/2 = 256`, matching wsprd's k+256 mod 512 rotation).
    ps: Vec<f32>,
    n_time: usize,
    /// Smoothed + renormalised spectrum, length `WORKING_BINS = 411`.
    /// Indexed so `smspec[i]` corresponds to baseband bin
    /// `256 − 205 + i = 51 + i`. Frequency offset from 1500 Hz is
    /// `(i − 205) · DF_BASEBAND` Hz.
    smspec: [f32; WORKING_BINS],
    /// Noise floor used to normalise smspec. Linear power. Kept for
    /// debugging/diagnostic; not currently consumed by the public API.
    #[allow(dead_code)]
    noise_level: f32,
}

const DF_BASEBAND: f32 = BASEBAND_RATE / NFFT as f32; // ≈ 0.7324 Hz/bin

fn build_spectro(idat: &[f32], qdat: &[f32]) -> Spectro {
    debug_assert_eq!(idat.len(), qdat.len());
    let np = idat.len();
    if np < NFFT {
        return Spectro {
            ps: Vec::new(),
            n_time: 0,
            smspec: [0.0; WORKING_BINS],
            noise_level: 1.0,
        };
    }
    // wsprd: nffts = 4 * floor(npoints / 512) - 1. The factor 4 reflects
    // the 128-sample stride (stride = NFFT/4).
    let n_time = 4 * (np / NFFT) - 1;

    // sin window: w[j] = sin(π · j / NFFT). Matches `wsprd.c:984`
    // (`sin(0.006147931 * i)` where 0.006147931 ≈ π/512).
    let mut window = [0.0f32; NFFT];
    for (j, w) in window.iter_mut().enumerate() {
        *w = (PI * j as f32 / NFFT as f32).sin();
    }

    let fft = with_default_planner(|planner| planner.plan_forward(NFFT));
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); NFFT];
    let mut ps = vec![0.0f32; n_time * NFFT];

    for t in 0..n_time {
        let start = t * STRIDE;
        for j in 0..NFFT {
            let s = if start + j < np {
                Complex::new(idat[start + j] * window[j], qdat[start + j] * window[j])
            } else {
                Complex::new(0.0, 0.0)
            };
            buf[j] = s;
        }
        fft.process(&mut buf);
        // wsprd `wsprd.c:1019`: k = j+256; if k>511 then k -= 512.
        // Equivalent: rotate by NFFT/2 to put DC at the centre.
        let row = &mut ps[t * NFFT..(t + 1) * NFFT];
        for j in 0..NFFT {
            let k = (j + NFFT / 2) % NFFT;
            row[j] = buf[k].norm_sqr();
        }
    }

    // Time-averaged power spectrum.
    let mut psavg = [0.0f32; NFFT];
    for t in 0..n_time {
        let row = &ps[t * NFFT..(t + 1) * NFFT];
        for j in 0..NFFT {
            psavg[j] += row[j];
        }
    }

    // 7-pt smooth, restricted to ±150 Hz (411 bins around DC bin 256).
    // wsprd `wsprd.c:1041`: `k = 256 - 205 + i + j`, j ∈ -3..=3.
    let mut smspec = [0.0f32; WORKING_BINS];
    for i in 0..WORKING_BINS {
        let mut acc = 0.0f32;
        for jw in -3i32..=3i32 {
            let k = (NFFT as i32) / 2 - 205 + i as i32 + jw;
            if k >= 0 && (k as usize) < NFFT {
                acc += psavg[k as usize];
            }
        }
        smspec[i] = acc;
    }

    // Noise floor: 30 th-percentile of smspec. wsprd uses
    // `tmpsort[122]/411` (= 122/411 ≈ 30 th percentile).
    let mut sorted: Vec<f32> = smspec.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let noise_level = sorted[(WORKING_BINS as f32 * 30.0 / 100.0) as usize].max(1e-30);

    // Renormalise: smspec[j] = smspec[j]/noise_level - 1, clamped to
    // 0.1·min_snr if below min_snr. Matches `wsprd.c:1067-1071`.
    for v in smspec.iter_mut() {
        *v = *v / noise_level - 1.0;
        if *v < MIN_SNR_LIN {
            *v = 0.1 * MIN_SNR_LIN;
        }
    }

    Spectro {
        ps,
        n_time,
        smspec,
        noise_level,
    }
}

/// Local-maxima peak detection on `smspec`. Returns up to `max_peaks`
/// peaks ranked by SNR in dB (descending). `(smspec_index, snr_db)`.
fn find_peaks(spec: &Spectro, max_peaks: usize) -> Vec<(usize, f32)> {
    let mut peaks: Vec<(usize, f32)> = Vec::new();
    for j in 1..(WORKING_BINS - 1) {
        let v = spec.smspec[j];
        if v > spec.smspec[j - 1] && v > spec.smspec[j + 1] {
            // wsprd `wsprd.c:1093`: snr = 10·log10(smspec) − snr_scaling.
            let snr_db = 10.0 * v.max(1e-30).log10() - SNR_SCALING_DB;
            peaks.push((j, snr_db));
        }
    }
    peaks.sort_unstable_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(core::cmp::Ordering::Equal));
    peaks.truncate(max_peaks);
    peaks
}

/// Convert smspec index `j` ∈ `[0, 411)` to baseband bin index in `ps`
/// (where DC = bin 256). `bin = 51 + j`.
#[inline]
fn smspec_to_bin(j: usize) -> usize {
    NFFT / 2 - 205 + j
}

/// One refined alignment cell: `(shift_baseband, dfreq, idrift, sync)`.
type RefinedCell = (i32, i32, i32, f32);

/// 3-D coarse refinement around a peak: search `ifr ∈ ±2`,
/// `k0 ∈ [−10, 22)`, `idrift ∈ ±max_drift`. Returns the **top
/// `top_k` cells** ranked by sync, descending.
///
/// Why top-K instead of just the best: at low SNR, the score landscape
/// is noisy and the absolute peak often lands ~1 s off the true
/// alignment. wsprd's mode-0 lag refinement only re-searches ±0.34 s
/// around the coarse pick, so a wrong-by-1-s coarse pick can never
/// reach the true alignment. Emitting alternate (k0, drift) cells lets
/// the demod sweep try each in turn — the true alignment usually
/// shows up as one of the secondary local peaks.
fn refine_alignment_top_k(
    spec: &Spectro,
    bin0: usize,
    max_drift: i32,
    top_k: usize,
) -> Vec<RefinedCell> {
    let mut cells: Vec<RefinedCell> = Vec::new();
    for dfreq in -2i32..=2i32 {
        let ifr = bin0 as i32 + dfreq;
        if ifr < 4 || (ifr as usize) + 4 >= NFFT {
            continue;
        }
        for k0 in -10i32..22i32 {
            for idrift in -max_drift..=max_drift {
                let mut ss = 0.0f32;
                let mut pow = 0.0f32;
                for k in 0..162i32 {
                    let drift_offset =
                        ((k as f32 - 81.0) / 81.0) * (idrift as f32) / (2.0 * DF_BASEBAND);
                    // C truncates float→int toward zero; Rust's `as i32`
                    // does the same. Using `.round()` would shift `ifd`
                    // by ±1 bin for nonzero drift in most symbols and
                    // distort the score landscape vs wsprd.
                    let ifd = ifr + drift_offset as i32;
                    if ifd - 3 < 0 || (ifd + 3) as usize >= NFFT {
                        continue;
                    }
                    let kindex = k0 + 2 * k;
                    if kindex < 0 || (kindex as usize) >= spec.n_time {
                        continue;
                    }
                    let row = &spec.ps[kindex as usize * NFFT..(kindex as usize + 1) * NFFT];
                    let p0 = row[(ifd - 3) as usize].sqrt();
                    let p1 = row[(ifd - 1) as usize].sqrt();
                    let p2 = row[(ifd + 1) as usize].sqrt();
                    let p3 = row[(ifd + 3) as usize].sqrt();
                    let pr3 = WSPR_SYNC_VECTOR[k as usize] as f32;
                    ss += (2.0 * pr3 - 1.0) * ((p1 + p3) - (p0 + p2));
                    pow += p0 + p1 + p2 + p3;
                }
                if pow <= 0.0 {
                    continue;
                }
                let sync = ss / pow;
                cells.push((STRIDE as i32 * (k0 + 1), dfreq, idrift, sync));
            }
        }
    }
    cells.sort_unstable_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(core::cmp::Ordering::Equal));
    cells.truncate(top_k);
    cells
}

/// Run the full wsprd-equivalent coarse on a 375 Hz baseband and
/// return ranked candidates ready for `decode_at_baseband`.
///
/// `pad_samples_audio` is the number of audio-rate samples of pre-pad
/// the caller prepended to the audio before decimation; the returned
/// `start_sample` includes this offset (so it indexes into the padded
/// buffer that fed `decimate_to_baseband`).
///
/// `max_peaks` caps the smspec peak count fed into refinement; wsprd
/// uses 200, which is fine for both SNR ranking and the per-peak
/// runtime budget.
pub fn coarse_baseband(
    idat: &[f32],
    qdat: &[f32],
    pad_samples_audio: usize,
    max_peaks: usize,
    max_drift_hz: i32,
) -> Vec<BasebandCandidate> {
    let spec = build_spectro(idat, qdat);
    if spec.n_time == 0 {
        return Vec::new();
    }
    let peaks = find_peaks(&spec, max_peaks);

    let pad_baseband = (pad_samples_audio as f32 / 32.0).round() as i32;

    // 1 cell per peak: matches wsprd, keeps runtime bounded. Top-K > 1
    // was tried (K=8) and didn't recover W3BI either — at -27 dB SNR
    // the score landscape doesn't have the true alignment as ANY
    // local maximum, so emitting more cells can't help. The wsprd
    // path that does recover such weak signals is the 3-pass
    // subtract-then-re-coarse loop (`subtract_signal2`), not coarse
    // ranking.
    const TOP_K_PER_PEAK: usize = 1;

    let mut out = Vec::with_capacity(peaks.len() * TOP_K_PER_PEAK);
    let _ = pad_baseband; // kept for API symmetry; shift_baseband is absolute
    for (j, snr_db) in peaks {
        let bin0 = smspec_to_bin(j);
        let cells = refine_alignment_top_k(&spec, bin0, max_drift_hz, TOP_K_PER_PEAK);
        for (shift_baseband, dfreq, idrift, sync) in cells {
            if !sync.is_finite() {
                continue;
            }
            let ifr = bin0 as i32 + dfreq;
            let freq_offset_hz = (ifr - NFFT as i32 / 2) as f32 * DF_BASEBAND;
            let centre_audio_hz = CENTER_HZ + freq_offset_hz;
            let tone0_audio_hz = centre_audio_hz - 1.5 * TONE_SPACING_HZ;
            let start_audio_signed = shift_baseband as i64 * 32;
            let start_sample = start_audio_signed.max(0) as usize;
            out.push(BasebandCandidate {
                start_sample,
                freq_hz: tone0_audio_hz,
                drift_hz: idrift as f32,
                sync,
                snr_db,
            });
        }
    }

    // Rank by sync score (the candidate-detection SNR is already
    // baked into peak selection; sync is the alignment-quality metric
    // that decode_at_baseband actually cares about).
    out.sort_unstable_by(|a, b| {
        b.sync
            .partial_cmp(&a.sync)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wspr::baseband::decimate_to_baseband;
    use crate::wspr::tx::synthesize_type1;

    #[test]
    fn finds_synth_signal_near_centre() {
        // A clean synth signal at 1500 Hz should appear as the top
        // candidate (or very near it) with the right freq.
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        // Pad to NPOINTS_MAX so decimation has a full buffer.
        let mut padded = vec![0.0f32; super::super::baseband::NPOINTS_MAX];
        padded[..audio.len()].copy_from_slice(&audio);
        let (idat, qdat) = decimate_to_baseband(&padded);
        let cands = coarse_baseband(&idat, &qdat, 0, 50, 0);
        assert!(!cands.is_empty(), "should find at least one candidate");
        let top = cands[0];
        // Tone-0 of a synth at base_freq=1500 Hz is at 1500 Hz exactly
        // (synthesize_type1 uses base_freq as tone-0). Allow ±1 Hz.
        assert!(
            (top.freq_hz - 1500.0).abs() < 1.5,
            "expected ~1500 Hz, got {}",
            top.freq_hz
        );
    }
}
