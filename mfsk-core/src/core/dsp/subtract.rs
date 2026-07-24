//! Successive interference cancellation (SIC) for phase-continuous MFSK.
//!
//! Given a tone sequence plus time/frequency coordinates, reconstructs the
//! ideal IQ waveform, estimates its complex amplitude by least-squares
//! projection onto the received signal, and subtracts a scaled copy in place.
//! Protocol-agnostic — the caller supplies tone sequence, sample rate, tone
//! spacing and timing so the same routine serves FT8/FT4/FT2/FST4.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::PI;
#[cfg(not(feature = "std"))]
use num_traits::Float;

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
    /// GFSK pulse-shaping parameters. `Some` → match real WSJT-style
    /// transmissions (FT8 BT=2.0, FT4 BT=2.0, FST4 BT=1.0). `None` → use
    /// abrupt phase transitions; reference will not match real signals
    /// — only retained for non-GFSK protocols and round-trip tests.
    pub gfsk: Option<GfskParams>,
}

/// Subset of [`crate::core::dsp::gfsk::GfskCfg`] needed to shape the
/// subtract reference. `sample_rate` and `samples_per_symbol` are
/// already on `SubtractCfg` so we only carry the GFSK-specific knobs.
#[derive(Clone, Copy, Debug)]
pub struct GfskParams {
    /// Bandwidth-time product. FT8/FT4 use 2.0.
    pub bt: f32,
    /// Modulation index. 1.0 for FT8.
    pub hmod: f32,
    /// Cosine ramp length at start/end (samples). FT8 uses `nsps / 8`.
    pub ramp_samples: usize,
}

/// Generate phase-continuous cosine/sine references for a symbol stream.
///
/// Returns `(w_cos, w_sin)` each of length `tones.len() * cfg.samples_per_symbol`.
/// `freq_hz` is the carrier of tone 0.
///
/// When `cfg.gfsk` is `Some`, the references are GFSK-shaped (matches
/// the actual transmitted FT8/FT4 waveform — required for any subtract
/// against real WSJT-style signals). Otherwise the legacy abrupt
/// per-symbol frequency switching is used.
fn generate_iq(tones: &[u8], freq_hz: f32, cfg: &SubtractCfg) -> (Vec<f32>, Vec<f32>) {
    let n = tones.len() * cfg.samples_per_symbol;
    if let Some(g) = cfg.gfsk {
        let gfsk_cfg = crate::core::dsp::gfsk::GfskCfg {
            sample_rate: cfg.sample_rate,
            samples_per_symbol: cfg.samples_per_symbol,
            bt: g.bt,
            hmod: g.hmod,
            ramp_samples: g.ramp_samples,
        };
        let mut w_cos = vec![0.0f32; n];
        let mut w_sin = vec![0.0f32; n];
        crate::core::dsp::gfsk::synth_complex_f32_into(
            &mut w_cos, &mut w_sin, tones, freq_hz, 1.0, &gfsk_cfg,
        );
        return (w_cos, w_sin);
    }
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

/// LS amplitude magnitude for a candidate `(freq_hz, dt_sec)`. Returns
/// `sqrt(a² + b²)` where `(a, b)` are the cos/sin LS projections of
/// `audio` onto the GFSK reference at `freq_hz`. Higher = closer match.
fn ls_amp_mag(audio: &[i16], tones: &[u8], freq_hz: f32, dt_sec: f32, cfg: &SubtractCfg) -> f32 {
    let (w_cos, w_sin) = generate_iq(tones, freq_hz, cfg);
    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= w_cos.len() {
        return 0.0;
    }
    let len = (w_cos.len() - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return 0.0;
    }
    let (mut na, mut nb, mut da, mut db) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for i in 0..len {
        let rx = audio[audio_off + i] as f64;
        let c = w_cos[ref_off + i] as f64;
        let s = w_sin[ref_off + i] as f64;
        na += rx * c;
        nb += rx * s;
        da += c * c;
        db += s * s;
    }
    let a = if da > 0.0 { na / da } else { 0.0 };
    let b = if db > 0.0 { nb / db } else { 0.0 };
    ((a * a + b * b) as f32).sqrt()
}

#[cfg(feature = "fft-rustfft")]
fn ls_amp_mag_f32(
    audio: &[f32],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
) -> f32 {
    let (w_cos, w_sin) = generate_iq(tones, freq_hz, cfg);
    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= w_cos.len() {
        return 0.0;
    }
    let len = (w_cos.len() - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return 0.0;
    }
    let (mut na, mut nb, mut da, mut db) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for i in 0..len {
        let rx = audio[audio_off + i] as f64;
        let c = w_cos[ref_off + i] as f64;
        let s = w_sin[ref_off + i] as f64;
        na += rx * c;
        nb += rx * s;
        da += c * c;
        db += s * s;
    }
    let a = if da > 0.0 { na / da } else { 0.0 };
    let b = if db > 0.0 { nb / db } else { 0.0 };
    ((a * a + b * b) as f32).sqrt()
}

/// Refine the carrier frequency around `freq_hz_init` by grid search,
/// returning the freq that maximises the LS amplitude magnitude of the
/// GFSK reference vs `audio`.
///
/// This compensates for mfsk-core's bin-resolution coarse_sync output:
/// at NFFT_SPEC=4096 / 12 kHz the carrier is reported on a 2.93 Hz grid,
/// but real signals routinely sit ±0.5..3 Hz off-bin. Over a 12.7 s FT8
/// frame even 1 Hz of error accumulates ~25π rad of phase rotation,
/// enough to wipe out a constant-amplitude LS projection — so subtract
/// can't actually remove the signal until the freq is refined.
///
/// `radius_hz` is the half-window (recommend 2.5 Hz to cover one bin
/// either side); `step_hz` is the grid resolution (0.1 Hz works on
/// host f32, embedded paths may use coarser steps for speed).
///
/// Mirrors WSJT-X's `ft8b.f90` `sync8d` + `ctwk` step (32-element
/// twiddle table over ±2.5 Hz).
pub fn refine_freq(
    audio: &[i16],
    tones: &[u8],
    freq_hz_init: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
    radius_hz: f32,
    step_hz: f32,
) -> f32 {
    debug_assert!(cfg.gfsk.is_some(), "refine_freq requires GFSK shaping");
    let mut best_freq = freq_hz_init;
    let mut best_amp = ls_amp_mag(audio, tones, freq_hz_init, dt_sec, cfg);
    let mut df = -radius_hz;
    while df <= radius_hz {
        if df.abs() > f32::EPSILON {
            let a = ls_amp_mag(audio, tones, freq_hz_init + df, dt_sec, cfg);
            if a > best_amp {
                best_amp = a;
                best_freq = freq_hz_init + df;
            }
        }
        df += step_hz;
    }
    best_freq
}

/// Refine the frame start time by fitting a parabola through complex
/// reference-correlation magnitudes at `-radius_samples`, zero, and
/// `+radius_samples`.
///
/// Returns `None` when the fitted vertex lies outside the probe interval
/// or the three points do not define a stable parabola.
#[cfg(feature = "fft-rustfft")]
pub fn refine_dt_f32(
    audio: &[f32],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
    radius_samples: i32,
) -> Option<f32> {
    if radius_samples <= 0 {
        return None;
    }
    let radius_sec = radius_samples as f32 / cfg.sample_rate;
    let minus = ls_amp_mag_f32(audio, tones, freq_hz, dt_sec - radius_sec, cfg);
    let zero = ls_amp_mag_f32(audio, tones, freq_hz, dt_sec, cfg);
    let plus = ls_amp_mag_f32(audio, tones, freq_hz, dt_sec + radius_sec, cfg);
    let curvature = plus + minus - 2.0 * zero;
    if !curvature.is_finite() || curvature.abs() <= f32::EPSILON {
        return None;
    }
    let dx = -(plus - minus) / (2.0 * curvature);
    if !dx.is_finite() || dx.abs() > 1.0 {
        return None;
    }
    let sample_offset = (radius_samples as f32 * dx).round() as i32;
    Some(dt_sec + sample_offset as f32 / cfg.sample_rate)
}

/// WSJT-X-style channel-aware subtract.
///
/// Estimates a **time-varying** complex amplitude `cfilt(t) = LPF[audio(t) *
/// conjg(cref(t))]` instead of a single global LS amplitude, then
/// subtracts `2 * Re[cfilt * cref]` sample-by-sample. The LPF tracks
/// channel fading (QSB) and slow phase drift that defeat the
/// constant-amplitude path in [`subtract_tones`].
///
/// Mirrors `WSJT-X/lib/ft8/subtractft8.f90` / `lib/ft4/subtractft4.f90`.
/// The LPF kernel is a cosine² window of length `2 * lpf_half + 1`.
/// WSJT-X uses `lpf_half = 2000` for FT8 (NFILT=4000) and `700` for
/// FT4 (NFILT=1400).
///
/// `endcorrection` mirrors WSJT-X's per-protocol choice: `subtractft8.f90`
/// boosts the near-edge (within `lpf_half` samples of the frame start/end)
/// LPF estimate to compensate for the truncated filter response there;
/// `subtractft4.f90` does not. Pass `true` for FT8, `false` for FT4.
///
/// # Cost
///
/// With the `fft-rustfft` feature (host), this runs WSJT-X's own
/// algorithm: one forward + inverse FFT of the full audio length per
/// call, against a filter-response FFT that's computed once and
/// cached (`OnceLock`) — matching `subtractft8.f90`'s `first`-gated
/// `cw` setup. O(N log N) instead of the O(`lpf_half` × NFRAME) direct
/// convolution (~608 M MACs / ~300 ms per call for FT8 on host —
/// measured as the dominant cost of `decode_block`'s multi-pass
/// subtract loop on busy-band recordings before this fix).
///
/// Without `fft-rustfft` (embedded / fixed-point, `no_std`) this falls
/// back to the direct time-domain convolution — unchanged from before
/// this function gained the FFT fast path. No embedded caller
/// currently invokes this function: `decode_block`'s embedded
/// (`not(fft-rustfft)`) variant of `decode_block_multipass` has no
/// subtract loop at all (single-pass, no SIC), so the fallback exists
/// only for other `no_std` callers (if any) and to keep the function
/// universally available.
///
/// # Requirements
///
/// Works for both GFSK (FT8/FT4 — `cfg.gfsk = Some`) and continuous-
/// phase MSK / abrupt-tone (WSPR — `cfg.gfsk = None`) modulations.
/// The reference is built via `generate_iq`, which dispatches on
/// `cfg.gfsk`.
pub fn subtract_tones_lpf(
    audio: &mut [i16],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
    lpf_half: usize,
    endcorrection: bool,
) {
    #[cfg(feature = "fft-rustfft")]
    {
        fft_lpf::subtract_tones_lpf_fft(
            audio,
            tones,
            freq_hz,
            dt_sec,
            cfg,
            lpf_half,
            endcorrection,
        );
    }
    #[cfg(not(feature = "fft-rustfft"))]
    {
        let _ = endcorrection;
        subtract_tones_lpf_direct(audio, tones, freq_hz, dt_sec, cfg, lpf_half);
    }
}

/// Floating-point residual variant of [`subtract_tones_lpf`].
///
/// WSJT-X keeps its successive-interference-cancellation buffer as `real`
/// throughout all decode passes. Keeping the residual in `f32` avoids
/// rounding and clipping the full slot after every successful subtraction.
#[cfg(feature = "fft-rustfft")]
pub fn subtract_tones_lpf_f32(
    audio: &mut [f32],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
    lpf_half: usize,
    endcorrection: bool,
) {
    fft_lpf::subtract_tones_lpf_fft_f32(
        audio,
        tones,
        freq_hz,
        dt_sec,
        cfg,
        lpf_half,
        endcorrection,
    );
}

/// Direct time-domain convolution fallback for `no_std` / non-`fft-rustfft`
/// builds. See [`subtract_tones_lpf`]'s doc comment for when this is used.
#[cfg(not(feature = "fft-rustfft"))]
fn subtract_tones_lpf_direct(
    audio: &mut [i16],
    tones: &[u8],
    freq_hz: f32,
    dt_sec: f32,
    cfg: &SubtractCfg,
    lpf_half: usize,
) {
    let nframe = tones.len() * cfg.samples_per_symbol;
    let (cref_re, cref_im) = generate_iq(tones, freq_hz, cfg);

    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= nframe {
        return;
    }
    let len = (nframe - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return;
    }

    // camp(t) = audio(t) * conjg(cref(t)) = audio*(cref_re - j*cref_im)
    let mut camp_re = vec![0.0f32; len];
    let mut camp_im = vec![0.0f32; len];
    for i in 0..len {
        let rx = audio[audio_off + i] as f32;
        camp_re[i] = rx * cref_re[ref_off + i];
        camp_im[i] = -rx * cref_im[ref_off + i];
    }

    // Cosine² LPF kernel of length 2*lpf_half + 1, normalized so sum = 1.
    let nk = 2 * lpf_half + 1;
    let mut kern = vec![0.0f32; nk];
    let mut sumw = 0.0f32;
    for j in 0..nk {
        let x = (j as f32 - lpf_half as f32) * PI / (2.0 * lpf_half as f32);
        let w = x.cos().powi(2);
        kern[j] = w;
        sumw += w;
    }
    for w in kern.iter_mut() {
        *w /= sumw;
    }

    // Direct convolution: cfilt[i] = sum_k kern[k] * camp[i + k - lpf_half],
    // clamping out-of-range indices to nearest edge (effectively reflecting
    // the truncated filter response — WSJT-X uses an explicit
    // `endcorrection` factor for the same effect).
    let mut cfilt_re = vec![0.0f32; len];
    let mut cfilt_im = vec![0.0f32; len];
    for i in 0..len {
        let (mut sr, mut si, mut sw) = (0.0f32, 0.0f32, 0.0f32);
        let lo = (i as i64 - lpf_half as i64).max(0) as usize;
        let hi = (i + lpf_half + 1).min(len);
        for j in lo..hi {
            let k = j as i64 - i as i64 + lpf_half as i64;
            if k < 0 || k as usize >= nk {
                continue;
            }
            let w = kern[k as usize];
            sr += w * camp_re[j];
            si += w * camp_im[j];
            sw += w;
        }
        // End correction: renormalize for the truncated filter window.
        if sw > f32::EPSILON {
            cfilt_re[i] = sr / sw;
            cfilt_im[i] = si / sw;
        }
    }

    // Subtract: audio[i] -= 2 * Re[cfilt(t) * cref(t)]
    //         = 2 * (cfilt_re * cref_re - cfilt_im * cref_im)
    for i in 0..len {
        let cr = cref_re[ref_off + i];
        let ci = cref_im[ref_off + i];
        let sub = 2.0 * (cfilt_re[i] * cr - cfilt_im[i] * ci);
        let v = audio[audio_off + i] as f32 - sub;
        audio[audio_off + i] = v.clamp(-32_768.0, 32_767.0) as i16;
    }
}

/// FFT-based fast convolution for [`subtract_tones_lpf`], mirroring
/// `WSJT-X/lib/ft8/subtractft8.f90` / `lib/ft4/subtractft4.f90`'s own
/// `cw`-cached frequency-domain LPF exactly (not just "an" FFT
/// convolution — the same algorithm, same normalization, same
/// end-correction).
#[cfg(feature = "fft-rustfft")]
mod fft_lpf {
    use super::{SubtractCfg, generate_iq};
    use core::f32::consts::PI;
    use rustfft::FftPlanner;
    use rustfft::num_complex::Complex32;
    use std::sync::{Mutex, OnceLock};
    use std::vec;
    use std::vec::Vec;

    /// Cached filter-response FFT, keyed by `(nfft, lpf_half)`. WSJT-X
    /// builds this once per program run (`first`/`save` in the
    /// Fortran); we build it once per distinct `(nfft, lpf_half)` pair
    /// seen (in practice exactly one for FT8, one for FT4) and reuse
    /// it for every subtract call after that.
    struct CachedWindow {
        nfft: usize,
        lpf_half: usize,
        cw: std::sync::Arc<Vec<Complex32>>,
    }

    static WINDOW_CACHE: OnceLock<Mutex<Vec<CachedWindow>>> = OnceLock::new();

    /// Cosine² LPF kernel of length `2*lpf_half+1`, normalised so the
    /// weights sum to 1. Shared by the window-FFT builder and the
    /// end-correction computation — both need the same un-FFT'd taps.
    fn normalized_kernel(lpf_half: usize) -> Vec<f32> {
        let nk = 2 * lpf_half + 1;
        let mut kern = vec![0.0f32; nk];
        let mut sumw = 0.0f32;
        for (j, k) in kern.iter_mut().enumerate() {
            let x = (j as f32 - lpf_half as f32) * PI / (2.0 * lpf_half as f32);
            let w = x.cos().powi(2);
            *k = w;
            sumw += w;
        }
        for w in kern.iter_mut() {
            *w /= sumw;
        }
        kern
    }

    /// `cw` from `subtractft8.f90`: the cosine² window placed at its
    /// circular delay positions (tap for relative offset `m` goes at
    /// index `m mod nfft`, matching Fortran's `cshift(cw, NFILT/2+1)`
    /// applied to a window initially placed at `cw(1:NFILT+1)`), FFT'd
    /// once, scaled by `1/nfft` (WSJT-X's `fac`) so the un-normalised
    /// forward/inverse FFT pair used per call needs no further scaling.
    fn cached_window_fft(nfft: usize, lpf_half: usize) -> std::sync::Arc<Vec<Complex32>> {
        let cache = WINDOW_CACHE.get_or_init(|| Mutex::new(Vec::new()));
        {
            let guard = cache.lock().unwrap();
            if let Some(e) = guard
                .iter()
                .find(|e| e.nfft == nfft && e.lpf_half == lpf_half)
            {
                return e.cw.clone();
            }
        }

        let kern = normalized_kernel(lpf_half);
        let mut cw = vec![Complex32::new(0.0, 0.0); nfft];
        for (j, &w) in kern.iter().enumerate() {
            let offset = j as isize - lpf_half as isize; // -lpf_half..=lpf_half
            // Fortran places the taps at cw(1:NFILT+1) and applies
            // cshift(..., NFILT/2+1). In zero-based coordinates an
            // original tap at `offset` therefore lands at offset-1.
            let idx = (offset - 1).rem_euclid(nfft as isize) as usize;
            cw[idx] = Complex32::new(w, 0.0);
        }
        let mut planner = FftPlanner::<f32>::new();
        planner.plan_fft_forward(nfft).process(&mut cw);
        let fac = 1.0 / nfft as f32;
        for c in cw.iter_mut() {
            *c *= fac;
        }

        let arc = std::sync::Arc::new(cw);
        let mut guard = cache.lock().unwrap();
        if let Some(e) = guard
            .iter()
            .find(|e| e.nfft == nfft && e.lpf_half == lpf_half)
        {
            return e.cw.clone();
        }
        guard.push(CachedWindow {
            nfft,
            lpf_half,
            cw: arc.clone(),
        });
        arc
    }

    /// Per-edge-distance boost factor, `endcorrection(j)` in
    /// `subtractft8.f90` (`j` there is 1-indexed 1..=lpf_half+1; here
    /// `d = j-1` is the 0-indexed distance from the frame edge,
    /// `0..=lpf_half`). Literal port of:
    /// `endcorrection(j) = 1/(1 - sum(window(j-1:NFILT/2))/sumw)`.
    fn end_correction(lpf_half: usize) -> Vec<f32> {
        let kern = normalized_kernel(lpf_half); // already sums to 1
        let nk = kern.len();
        let mut ec = vec![1.0f32; lpf_half + 1];
        for (d, e) in ec.iter_mut().enumerate() {
            let k_from = d + lpf_half; // window(j-1 : NFILT/2), j-1=d
            let s: f32 = kern[k_from..nk].iter().sum();
            *e = 1.0 / (1.0 - s).max(1e-6);
        }
        ec
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn subtract_tones_lpf_fft(
        audio: &mut [i16],
        tones: &[u8],
        freq_hz: f32,
        dt_sec: f32,
        cfg: &SubtractCfg,
        lpf_half: usize,
        endcorrection: bool,
    ) {
        let nframe = tones.len() * cfg.samples_per_symbol;
        let nfft = audio.len();
        if nframe == 0 || nfft == 0 || lpf_half == 0 {
            return;
        }
        let (cref_re, cref_im) = generate_iq(tones, freq_hz, cfg);

        // `nstart - 1` in WSJT-X's 1-indexed Fortran == `signed_start`
        // here: camp(i) (0-indexed i=0..nframe) reads audio at absolute
        // sample `signed_start + i`, zero when that index is out of
        // bounds (matches `if(j.ge.1.and.j.le.NMAX) camp(i)=...`).
        let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;

        let mut cfilt = vec![Complex32::new(0.0, 0.0); nfft];
        for i in 0..nframe {
            let j = signed_start + i as i64;
            if j >= 0 && (j as usize) < audio.len() {
                let rx = audio[j as usize] as f32;
                // camp = audio * conjg(cref) = rx*(cref_re - j*cref_im)
                cfilt[i] = Complex32::new(rx * cref_re[i], -rx * cref_im[i]);
            }
        }

        let cw = cached_window_fft(nfft, lpf_half);
        let mut planner = FftPlanner::<f32>::new();
        planner.plan_fft_forward(nfft).process(&mut cfilt);
        for (c, w) in cfilt.iter_mut().zip(cw.iter()) {
            *c *= *w;
        }
        planner.plan_fft_inverse(nfft).process(&mut cfilt);

        if endcorrection && lpf_half < nframe {
            let ec = end_correction(lpf_half);
            for (d, &factor) in ec.iter().enumerate() {
                cfilt[d] *= factor;
                let end_idx = nframe - 1 - d;
                if end_idx != d {
                    cfilt[end_idx] *= factor;
                }
            }
        }

        // Subtract: audio[j] -= 2*Re[cfilt(i) * cref(i)] for i=0..nframe,
        // j = signed_start + i (same bounds check as the camp build above).
        for i in 0..nframe {
            let j = signed_start + i as i64;
            if j >= 0 && (j as usize) < audio.len() {
                let z = cfilt[i] * Complex32::new(cref_re[i], cref_im[i]);
                let sub = 2.0 * z.re;
                let v = audio[j as usize] as f32 - sub;
                audio[j as usize] = v.clamp(-32_768.0, 32_767.0) as i16;
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn subtract_tones_lpf_fft_f32(
        audio: &mut [f32],
        tones: &[u8],
        freq_hz: f32,
        dt_sec: f32,
        cfg: &SubtractCfg,
        lpf_half: usize,
        endcorrection: bool,
    ) {
        let nframe = tones.len() * cfg.samples_per_symbol;
        let nfft = audio.len();
        if nframe == 0 || nfft == 0 || lpf_half == 0 {
            return;
        }
        let (cref_re, cref_im) = generate_iq(tones, freq_hz, cfg);
        let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;

        let mut cfilt = vec![Complex32::new(0.0, 0.0); nfft];
        for i in 0..nframe {
            let j = signed_start + i as i64;
            if j >= 0 && (j as usize) < audio.len() {
                let rx = audio[j as usize];
                cfilt[i] = Complex32::new(rx * cref_re[i], -rx * cref_im[i]);
            }
        }

        let cw = cached_window_fft(nfft, lpf_half);
        let mut planner = FftPlanner::<f32>::new();
        planner.plan_fft_forward(nfft).process(&mut cfilt);
        for (c, w) in cfilt.iter_mut().zip(cw.iter()) {
            *c *= *w;
        }
        planner.plan_fft_inverse(nfft).process(&mut cfilt);

        if endcorrection && lpf_half < nframe {
            let ec = end_correction(lpf_half);
            for (d, &factor) in ec.iter().enumerate() {
                cfilt[d] *= factor;
                let end_idx = nframe - 1 - d;
                if end_idx != d {
                    cfilt[end_idx] *= factor;
                }
            }
        }

        for i in 0..nframe {
            let j = signed_start + i as i64;
            if j >= 0 && (j as usize) < audio.len() {
                let z = cfilt[i] * Complex32::new(cref_re[i], cref_im[i]);
                audio[j as usize] -= 2.0 * z.re;
            }
        }
    }
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

    // Signed start (samples). For `dt_sec < -base_offset_s` the FT8 frame
    // begins **before** the audio buffer — the leading portion of the
    // reference falls outside `audio[..]` and must be clipped.
    //
    // Pre-fix: `as usize` saturated negative values to 0, leaving
    // `start = 0` and the entire reference aligned to `audio[0..]`.
    // For e.g. `dt_sec = -0.78` (FT8 reports up to ±2.5 s) this misaligned
    // the reference by 3360 samples → near-zero LS amplitude → effectively
    // no-op subtract on legitimately-decoded signals at large negative DT.
    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= w_cos.len() {
        return;
    }
    let len = (w_cos.len() - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return;
    }

    // Complex least-squares: rx[t] ≈ a·cos(φ(t)) + b·sin(φ(t))
    // cos / sin are near-orthogonal over the full frame so the closed-form
    // per-component projection matches the joint solve to floating-point
    // precision.
    let (num_a, num_b, den_a, den_b) =
        (0..len).fold((0.0f32, 0.0f32, 0.0f32, 0.0f32), |(na, nb, da, db), i| {
            let rx = audio[audio_off + i] as f32;
            let wc = w_cos[ref_off + i];
            let ws = w_sin[ref_off + i];
            (na + rx * wc, nb + rx * ws, da + wc * wc, db + ws * ws)
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
        let sub = gain * (a * w_cos[ref_off + i] + b * w_sin[ref_off + i]);
        let new_val = audio[audio_off + i] as f32 - sub;
        audio[audio_off + i] = new_val.clamp(-32_768.0, 32_767.0) as i16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Regression test for `subtract_tones` start-clipping with negative DT.
    ///
    /// Pre-fix: `((base_offset_s + dt_sec) * sample_rate) as usize` silently
    /// saturated negative values to 0, leaving the reference misaligned by
    /// thousands of samples — the LS projection then estimated near-zero
    /// amplitude and the subtract was effectively a no-op for any decode
    /// reporting `dt_sec < -base_offset_s`.
    #[test]
    fn subtract_tones_negative_dt_aligns_via_ref_offset() {
        // Fake FT8-shape config: small frame for cheap test.
        // Tests intentionally exercise the abrupt-transition path so audio
        // and reference share identical shape (cleanest > -100 dB drop).
        // The GFSK path is exercised by `tests/ft8_subtract_self_test.rs`.
        let cfg = SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: None,
        };
        let tones: Vec<u8> = (0..79).map(|k| (k % 8) as u8).collect();
        let (w_cos, _w_sin) = generate_iq(&tones, 1500.0, &cfg);

        // Build audio that starts mid-frame (signal began before sample 0):
        // shift the reference left by 3360 samples and take only the
        // overlapping tail. amplitude = 5000.
        let shift = 3360usize;
        let amp = 5000.0f32;
        let mut audio: Vec<i16> = vec![0i16; 180_000];
        let n = w_cos.len() - shift;
        for i in 0..n.min(audio.len()) {
            audio[i] = (amp * w_cos[shift + i]).clamp(-32_768.0, 32_767.0) as i16;
        }
        let pre_energy: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();

        // Reported dt corresponds to signal-start at sample -3360, i.e.
        // dt_sec = -3360 / 12_000 = -0.28 → start = (0.5 - 0.28) * 12k = 2640.
        // With shift=3360 the actual signal-start is -3360 from sample 0, so
        // the equivalent decoded dt_sec is (0 - shift) / sample_rate -
        // base_offset_s = -shift/sr - 0.5 = -0.78 (mirroring qso3 CQ F5RXL).
        let dt_sec = -(shift as f32) / cfg.sample_rate - cfg.base_offset_s;
        subtract_tones(&mut audio, &tones, 1500.0, dt_sec, 1.0, &cfg);

        let post_energy: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();
        let drop_db = 10.0 * (post_energy / pre_energy).log10();

        // With proper alignment: post energy should drop by ≥ 30 dB
        // (clean signal, exact reference match — only quantization noise left).
        assert!(
            drop_db < -30.0,
            "subtract_tones failed to remove signal at dt_sec={dt_sec:.3} \
             (drop only {drop_db:.1} dB; expected < -30 dB). \
             Pre-fix bug: `start as usize` saturated negative to 0."
        );
    }

    /// Sanity: positive DT path also works (regression check).
    #[test]
    fn subtract_tones_positive_dt_works() {
        // Tests intentionally exercise the abrupt-transition path so audio
        // and reference share identical shape (cleanest > -100 dB drop).
        // The GFSK path is exercised by `tests/ft8_subtract_self_test.rs`.
        let cfg = SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: None,
        };
        let tones: Vec<u8> = (0..79).map(|k| (k % 8) as u8).collect();
        let (w_cos, _) = generate_iq(&tones, 1500.0, &cfg);

        let dt_sec: f32 = 0.2;
        let start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as usize;
        let amp = 5000.0f32;
        let mut audio: Vec<i16> = vec![0i16; 180_000];
        for i in 0..w_cos.len() {
            audio[start + i] = (amp * w_cos[i]).clamp(-32_768.0, 32_767.0) as i16;
        }
        let pre: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();

        subtract_tones(&mut audio, &tones, 1500.0, dt_sec, 1.0, &cfg);
        let post: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();
        let drop_db = 10.0 * (post / pre).log10();
        assert!(
            drop_db < -30.0,
            "positive-DT subtract drop only {drop_db:.1} dB"
        );
    }
}
