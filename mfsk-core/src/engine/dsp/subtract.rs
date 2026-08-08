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

/// Subset of [`crate::engine::dsp::gfsk::GfskCfg`] needed to shape the
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
        let gfsk_cfg = crate::engine::dsp::gfsk::GfskCfg {
            sample_rate: cfg.sample_rate,
            samples_per_symbol: cfg.samples_per_symbol,
            bt: g.bt,
            hmod: g.hmod,
            ramp_samples: g.ramp_samples,
        };
        let mut w_cos = vec![0.0f32; n];
        let mut w_sin = vec![0.0f32; n];
        crate::engine::dsp::gfsk::synth_complex_f32_into(
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
///
/// `refine_freq` no longer calls this directly (see
/// [`ls_amp_mag_tweaked`]) — kept `#[cfg(test)]`-only as the frozen,
/// trusted full-resynthesis reference its differential test compares
/// against.
#[cfg(test)]
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

/// Number of samples between rotor-magnitude renormalizations in
/// [`ls_amp_mag_tweaked`]'s incremental NCO. Bounds f32 rounding drift
/// over the ~59k (FT4) / ~334k (FT8) sample buffers `refine_freq`
/// sweeps; cheap relative to the multiply-adds it guards (one `norm()`
/// + divide every 256 samples).
const NCO_RESYNC_SAMPLES: usize = 256;

/// Same LS amplitude magnitude as [`ls_amp_mag`], but takes a
/// precomputed carrier-free (`freq_hz = 0.0`) reference phasor
/// `(base_cos, base_sin)` and applies `freq_hz` via a cheap per-sample
/// NCO rotation instead of resynthesizing the whole GFSK waveform.
///
/// Valid because `generate_iq`'s carrier term is added *uniformly* to
/// every entry of its internal phase-rate array before integration, so
/// `phi(k; f0) = phi_mod(k) + k · (2π · f0 · dt)` — i.e. any carrier
/// just adds a pure linear phase ramp on top of the carrier-free
/// (tone-modulation-only) phase `phi_mod(k)` that `base_cos`/`base_sin`
/// already encode. Rotating by that ramp via `nco(k) = exp(j · k · 2π ·
/// f0 · dt)` and combining via the angle-addition identities
/// (`cos(a+b)`, `sin(a+b)`) reproduces `generate_iq(tones, f0, cfg)`
/// without repeating its `O(nsym · pulse_len)` Gaussian-pulse
/// convolution or its `O(nwave)` `sin`/`cos` integration — both
/// independent of `f0`, and (measured) the dominant cost of
/// `refine_freq`'s ~50-100-point frequency grid search (issue #182:
/// FT4's `decode_frame_subtract` regressed from 48.8ms to ~530ms after
/// issues #178-180 wired `refine_freq` into FT4's subtract path; ~35ms
/// of that was this resynthesis, repeated per real decode).
fn ls_amp_mag_tweaked(
    audio: &[i16],
    base_cos: &[f32],
    base_sin: &[f32],
    freq_hz: f32,
    dt_sec: f32,
    dt: f32,
    cfg: &SubtractCfg,
) -> f32 {
    let signed_start = ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64;
    let (audio_off, ref_off) = if signed_start < 0 {
        (0usize, (-signed_start) as usize)
    } else {
        (signed_start as usize, 0usize)
    };
    if ref_off >= base_cos.len() {
        return 0.0;
    }
    let len = (base_cos.len() - ref_off).min(audio.len().saturating_sub(audio_off));
    if len == 0 {
        return 0.0;
    }

    let step_phase = 2.0 * PI * freq_hz * dt;
    let (step_cos, step_sin) = (step_phase.cos(), step_phase.sin());
    // Seed the rotor at absolute index `ref_off` (one trig call), then
    // advance it by incremental complex multiply — matching the
    // absolute-index carrier phase `generate_iq(tones, freq_hz, cfg)`
    // would have produced at the same position.
    let seed_phase = step_phase * ref_off as f32;
    let (mut nco_re, mut nco_im) = (seed_phase.cos(), seed_phase.sin());

    let (mut na, mut nb, mut da, mut db) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for i in 0..len {
        let bc = base_cos[ref_off + i];
        let bs = base_sin[ref_off + i];
        let c = bc * nco_re - bs * nco_im;
        let s = bs * nco_re + bc * nco_im;

        let rx = audio[audio_off + i] as f64;
        na += rx * c as f64;
        nb += rx * s as f64;
        da += (c * c) as f64;
        db += (s * s) as f64;

        let (new_re, new_im) = (
            nco_re * step_cos - nco_im * step_sin,
            nco_re * step_sin + nco_im * step_cos,
        );
        nco_re = new_re;
        nco_im = new_im;
        if (i + 1) % NCO_RESYNC_SAMPLES == 0 {
            let norm = (nco_re * nco_re + nco_im * nco_im).sqrt();
            if norm > f32::EPSILON {
                nco_re /= norm;
                nco_im /= norm;
            }
        }
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
/// twiddle table over ±2.5 Hz). Builds the carrier-free GFSK reference
/// once (`ls_amp_mag_tweaked`) rather than resynthesizing it at every
/// grid step — see that function's doc comment for why this is safe.
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
    let (base_cos, base_sin) = generate_iq(tones, 0.0, cfg);
    let dt = 1.0 / cfg.sample_rate;
    let mut best_freq = freq_hz_init;
    let mut best_amp =
        ls_amp_mag_tweaked(audio, &base_cos, &base_sin, freq_hz_init, dt_sec, dt, cfg);
    let mut df = -radius_hz;
    while df <= radius_hz {
        if df.abs() > f32::EPSILON {
            let a = ls_amp_mag_tweaked(
                audio,
                &base_cos,
                &base_sin,
                freq_hz_init + df,
                dt_sec,
                dt,
                cfg,
            );
            if a > best_amp {
                best_amp = a;
                best_freq = freq_hz_init + df;
            }
        }
        df += step_hz;
    }
    best_freq
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

/// `subtractft8.f90`'s `lrefinedt=.true.` subtract: probes a small
/// range of `dt` offsets around `dt_sec` and subtracts at whichever one
/// leaves the least residual energy in the signal's own tone band,
/// instead of trusting `dt_sec` exactly. See the private
/// `fft_lpf::subtract_tones_lpf_refine_dt_fft` (this function's
/// `fft-rustfft` backend) for the full port of WSJT-X's `sqf`/`peakup`
/// search.
///
/// Intended for subtracting candidates whose `dt` hasn't been through a
/// full decode pass yet — e.g. issue #180's staged-checkpoint SIC,
/// which (like `ft8_decode.f90`'s own early-decode-and-subtract
/// checkpoints) subtracts still-early candidates before the harder,
/// final search runs. Use plain [`subtract_tones_lpf`] once a
/// candidate's `dt` is already the final decode's own value
/// (`ft8b.f90:474`'s `lrefinedt=.false.` case).
///
/// Without `fft-rustfft` this falls back to the plain (non-refining)
/// direct-convolution subtract — the dt-search itself needs the
/// in-band-power FFT metric, which the `no_std` direct path has no
/// cheap equivalent for, and (per [`subtract_tones_lpf`]'s doc comment)
/// no embedded caller uses either path today.
pub fn subtract_tones_lpf_refine_dt(
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
        fft_lpf::subtract_tones_lpf_refine_dt_fft(
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
    // Argument spans -pi/2..=pi/2 (NFILT = 2*lpf_half), matching
    // subtractft8.f90's `cos(pi*j/NFILT)**2` — see `fft_lpf::normalized_kernel`'s
    // doc comment (issue #180) for why the divisor must be `2*lpf_half`, not
    // `lpf_half`.
    let nk = 2 * lpf_half + 1;
    let nfilt = 2.0 * lpf_half as f32;
    let mut kern = vec![0.0f32; nk];
    let mut sumw = 0.0f32;
    for j in 0..nk {
        let x = (j as f32 - lpf_half as f32) * PI / nfilt;
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
    use std::sync::{Arc, Mutex, OnceLock};
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
    ///
    /// `subtractft8.f90`/`subtractft4.f90`: `window(j) = cos(pi*j/NFILT)**2`
    /// for `j` in `-NFILT/2 ..= NFILT/2`, where `NFILT = 2*lpf_half` — i.e.
    /// the cosine argument spans `-pi/2 ..= pi/2` (a single taper, 1 at the
    /// centre, 0 at the edges). A prior version of this function divided
    /// by `lpf_half` instead of `2*lpf_half` (`NFILT`), doubling the
    /// argument to `-pi ..= pi`: `cos²` at that range is *not* monotonic
    /// (it dips to 0 at the quarter points and rises back to 1 at the
    /// true edges), so the kernel that shipped was two humps with a null
    /// between them, not a lowpass at all — full-weight was given to
    /// samples `lpf_half` away (166 ms, for FT8's `lpf_half=2000`), same
    /// as the centre sample. See issue #180.
    fn normalized_kernel(lpf_half: usize) -> Vec<f32> {
        let nk = 2 * lpf_half + 1;
        let nfilt = 2.0 * lpf_half as f32;
        let mut kern = vec![0.0f32; nk];
        let mut sumw = 0.0f32;
        for (j, k) in kern.iter_mut().enumerate() {
            let x = (j as f32 - lpf_half as f32) * PI / nfilt;
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
            let idx = offset.rem_euclid(nfft as isize) as usize;
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

    /// Cached forward/inverse FFT plans, keyed by `nfft`. Same idea as
    /// `cached_window_fft` (and `fill_symbol_spectra.rs`'s
    /// `SYMBOL_FFT_32`): `FftPlanner::plan_fft_{forward,inverse}`
    /// re-derives the same `nfft`-point twiddle tables on every call —
    /// measured at ~2.8 ms/call (vs ~0.7 ms for the FFT itself) for
    /// FT8's `nfft = 180 000`, i.e. plan construction cost ~4x the
    /// actual transform. `subtract_tones_lpf_fft` runs this twice
    /// (forward + inverse) per accepted decode subtracted, so this
    /// dominated the per-call cost even after `cached_window_fft`
    /// removed the filter-response rebuild.
    struct CachedPlans {
        nfft: usize,
        forward: Arc<dyn rustfft::Fft<f32>>,
        inverse: Arc<dyn rustfft::Fft<f32>>,
    }

    static PLAN_CACHE: OnceLock<Mutex<Vec<CachedPlans>>> = OnceLock::new();

    fn cached_plans(nfft: usize) -> (Arc<dyn rustfft::Fft<f32>>, Arc<dyn rustfft::Fft<f32>>) {
        let cache = PLAN_CACHE.get_or_init(|| Mutex::new(Vec::new()));
        {
            let guard = cache.lock().unwrap();
            if let Some(e) = guard.iter().find(|e| e.nfft == nfft) {
                return (e.forward.clone(), e.inverse.clone());
            }
        }

        let mut planner = FftPlanner::<f32>::new();
        let forward = planner.plan_fft_forward(nfft);
        let inverse = planner.plan_fft_inverse(nfft);

        let mut guard = cache.lock().unwrap();
        if let Some(e) = guard.iter().find(|e| e.nfft == nfft) {
            return (e.forward.clone(), e.inverse.clone());
        }
        guard.push(CachedPlans {
            nfft,
            forward: forward.clone(),
            inverse: inverse.clone(),
        });
        (forward, inverse)
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
        apply_at_offset(
            audio,
            tones,
            freq_hz,
            dt_sec,
            cfg,
            lpf_half,
            endcorrection,
            0,
        );
    }

    /// Core of [`subtract_tones_lpf_fft`], with an extra `idt_offset`
    /// (samples at `cfg.sample_rate`) added to the reference alignment
    /// before subtracting. `idt_offset = 0` is byte-identical to the
    /// former single-purpose `subtract_tones_lpf_fft` body. Shared by
    /// the plain subtract and by [`subtract_tones_lpf_refine_dt_fft`]'s
    /// `-90/0/+90`-sample trial search (`subtractft8.f90`'s `sqf`).
    #[allow(clippy::too_many_arguments)]
    fn apply_at_offset(
        audio: &mut [i16],
        tones: &[u8],
        freq_hz: f32,
        dt_sec: f32,
        cfg: &SubtractCfg,
        lpf_half: usize,
        endcorrection: bool,
        idt_offset: i64,
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
        // `idt_offset` mirrors `subtractft8.f90`'s `sqf(idt)` inner
        // function, which probes `nstart=dt*12000+1+idt`.
        let signed_start =
            ((cfg.base_offset_s + dt_sec) * cfg.sample_rate).round() as i64 + idt_offset;

        // `j = signed_start + i` is in-bounds (`0..audio.len()`) only for
        // `i` in `i_lo..i_hi` — clamp once instead of re-checking
        // `j >= 0 && j < audio.len()` on every one of the `2 × nframe`
        // iterations below. Out-of-range `i` has nothing to do in either
        // loop (the camp build leaves `cfilt[i]` at its zero-init value;
        // the subtract loop has no `audio[j]` to touch), so clamping the
        // range is a complete substitute for the branch, not just a fast
        // path alongside it. Only non-empty near a slot's start/end.
        let i_lo = (-signed_start).clamp(0, nframe as i64) as usize;
        let i_hi = (audio.len() as i64 - signed_start).clamp(0, nframe as i64) as usize;

        let mut cfilt = vec![Complex32::new(0.0, 0.0); nfft];
        for i in i_lo..i_hi {
            let j = (signed_start + i as i64) as usize;
            let rx = audio[j] as f32;
            // camp = audio * conjg(cref) = rx*(cref_re - j*cref_im)
            cfilt[i] = Complex32::new(rx * cref_re[i], -rx * cref_im[i]);
        }

        let cw = cached_window_fft(nfft, lpf_half);
        let (forward, inverse) = cached_plans(nfft);
        forward.process(&mut cfilt);
        for (c, w) in cfilt.iter_mut().zip(cw.iter()) {
            *c *= *w;
        }
        inverse.process(&mut cfilt);

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
        // j = signed_start + i (same `i_lo..i_hi` range as the camp build
        // above — identical `j` formula, so identical bounds).
        for i in i_lo..i_hi {
            let j = (signed_start + i as i64) as usize;
            let z = cfilt[i] * Complex32::new(cref_re[i], cref_im[i]);
            let sub = 2.0 * z.re;
            let v = audio[j] as f32 - sub;
            audio[j] = v.clamp(-32_768.0, 32_767.0) as i16;
        }
    }

    /// In-band residual power after a trial subtraction, `sqq` in
    /// `subtractft8.f90::sqf`. Computes a full-length forward FFT of
    /// `audio` (real input, zero imaginary — the Fortran side gets the
    /// same positive-frequency half-spectrum via a real-FFT/equivalence
    /// trick) and sums `|X(k)|^2` over the bins spanning
    /// `[freq_hz - 1.5*tone_spacing_hz, freq_hz + 8.5*tone_spacing_hz]`
    /// — the candidate's own tone band. Used only to *rank* three trial
    /// offsets against each other, so the missing real-FFT packing is
    /// immaterial (both sides of that symmetric band are scaled
    /// identically for every trial).
    fn residual_band_power(
        audio: &[i16],
        freq_hz: f32,
        tone_spacing_hz: f32,
        sample_rate: f32,
    ) -> f32 {
        let nfft = audio.len();
        if nfft == 0 {
            return 0.0;
        }
        let mut buf: Vec<Complex32> = audio
            .iter()
            .map(|&s| Complex32::new(s as f32, 0.0))
            .collect();
        let (forward, _) = cached_plans(nfft);
        forward.process(&mut buf);

        let df = sample_rate / nfft as f32;
        // Fortran `ia=(f0-1.5*baud)/df` / `ib=(f0+8.5*baud)/df`: real-to-
        // integer assignment truncates toward zero, not rounds.
        let ia = ((freq_hz - 1.5 * tone_spacing_hz) / df) as i64;
        let ib = ((freq_hz + 8.5 * tone_spacing_hz) / df) as i64;
        let lo = ia.max(0) as usize;
        let hi = (ib.max(0) as usize).min(nfft.saturating_sub(1));
        let mut sqq = 0.0f32;
        for k in lo..=hi {
            let c = buf[k];
            sqq += c.re * c.re + c.im * c.im;
        }
        sqq
    }

    /// `subtractft8.f90`'s `lrefinedt=.true.` path: probes `idt` offsets
    /// of `-90`/`0`/`+90` samples (±7.5 ms at 12 kHz) around the
    /// candidate's own `dt_sec`, scores each trial by its resulting
    /// in-band residual power ([`residual_band_power`]), fits a
    /// parabola through the 3 points (`peakup.f90`) and subtracts at
    /// the fitted vertex offset. If the fitted vertex falls outside
    /// `±90` samples (`|dx| > 1.0`), the signal is left untouched —
    /// matching WSJT-X's `if(abs(dx).gt.1.0) return`, which treats that
    /// case as "no acceptable minimum".
    ///
    /// WSJT-X reserves this for its two "early decode, subtract before
    /// the next checkpoint" call sites (`ft8_decode.f90:132`, `:162`) —
    /// candidates whose `dt` hasn't yet had a full decode pass to lock
    /// it down. The single subtract-after-a-confirmed-decode call
    /// (`ft8b.f90:474`) always uses `lrefinedt=.false.` (plain
    /// [`subtract_tones_lpf_fft`]) since that `dt` is already final.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn subtract_tones_lpf_refine_dt_fft(
        audio: &mut [i16],
        tones: &[u8],
        freq_hz: f32,
        dt_sec: f32,
        cfg: &SubtractCfg,
        lpf_half: usize,
        endcorrection: bool,
    ) {
        const SEARCH_SAMPLES: i64 = 90;

        let trial = |offset: i64| -> Vec<i16> {
            let mut copy = audio.to_vec();
            apply_at_offset(
                &mut copy,
                tones,
                freq_hz,
                dt_sec,
                cfg,
                lpf_half,
                endcorrection,
                offset,
            );
            copy
        };

        let cand_m = trial(-SEARCH_SAMPLES);
        let cand_p = trial(SEARCH_SAMPLES);
        let cand_0 = trial(0);

        let sq_m = residual_band_power(&cand_m, freq_hz, cfg.tone_spacing_hz, cfg.sample_rate);
        let sq_p = residual_band_power(&cand_p, freq_hz, cfg.tone_spacing_hz, cfg.sample_rate);
        let sq_0 = residual_band_power(&cand_0, freq_hz, cfg.tone_spacing_hz, cfg.sample_rate);

        // peakup.f90: b=(yp-ym)/2; c=(yp+ym-2*y0)/2; dx=-b/(2*c).
        let b = (sq_p - sq_m) / 2.0;
        let c = (sq_p + sq_m - 2.0 * sq_0) / 2.0;
        if c.abs() < f32::EPSILON {
            // No curvature: fall back to the untried, unshifted subtract
            // rather than leaving the signal untouched.
            audio.copy_from_slice(&cand_0);
            return;
        }
        let dx = -b / (2.0 * c);
        if dx.abs() > 1.0 {
            // No acceptable minimum in range: don't subtract at all.
            return;
        }
        let best_offset = (SEARCH_SAMPLES as f32 * dx).round() as i64;
        apply_at_offset(
            audio,
            tones,
            freq_hz,
            dt_sec,
            cfg,
            lpf_half,
            endcorrection,
            best_offset,
        );
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

#[cfg(all(test, feature = "fft-rustfft"))]
mod refine_dt_tests {
    use super::*;
    use crate::engine::dsp::gfsk::{GfskCfg, synth_i16};

    fn ft8_cfg() -> SubtractCfg {
        SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: Some(GfskParams {
                bt: 2.0,
                hmod: 1.0,
                ramp_samples: 1920 / 8,
            }),
        }
    }

    /// Issue #180: when a candidate's reported `dt` is a few ms off from
    /// the signal's true position (as happens with early/coarse
    /// checkpoint-SIC candidates — see `ft8::decode`'s staged-checkpoint
    /// SIC), `subtract_tones_lpf_refine_dt` should leave less residual
    /// energy behind than the plain, non-refining `subtract_tones_lpf`,
    /// by re-searching ±90 samples for the best-cancelling alignment
    /// (mirrors `subtractft8.f90`'s `lrefinedt=.true.` path).
    #[test]
    fn refine_dt_beats_plain_subtract_when_dt_is_off() {
        let cfg = ft8_cfg();
        let gfsk_cfg = GfskCfg {
            sample_rate: cfg.sample_rate,
            samples_per_symbol: cfg.samples_per_symbol,
            bt: cfg.gfsk.unwrap().bt,
            hmod: cfg.gfsk.unwrap().hmod,
            ramp_samples: cfg.gfsk.unwrap().ramp_samples,
        };
        let tones: Vec<u8> = (0..79).map(|k| (k % 8) as u8).collect();
        let freq_hz = 1500.0f32;
        let true_dt_sec = 0.2f32;

        let samples = synth_i16(&tones, freq_hz, 5000, &gfsk_cfg);
        let mut audio = vec![0i16; 180_000];
        let start = ((cfg.base_offset_s + true_dt_sec) * cfg.sample_rate).round() as usize;
        for (i, &s) in samples.iter().enumerate() {
            audio[start + i] = s;
        }

        // Candidate's reported dt is 40 samples (~3.3 ms) off true —
        // within subtractft8.f90's ±90-sample search radius, comparable
        // to the timing slop an early/unrefined checkpoint-SIC candidate
        // can carry.
        let reported_dt_sec = true_dt_sec + 40.0 / cfg.sample_rate;

        let mut audio_plain = audio.clone();
        subtract_tones_lpf(
            &mut audio_plain,
            &tones,
            freq_hz,
            reported_dt_sec,
            &cfg,
            2000,
            true,
        );
        let energy_plain: f64 = audio_plain.iter().map(|&s| (s as f64).powi(2)).sum();

        let mut audio_refined = audio.clone();
        subtract_tones_lpf_refine_dt(
            &mut audio_refined,
            &tones,
            freq_hz,
            reported_dt_sec,
            &cfg,
            2000,
            true,
        );
        let energy_refined: f64 = audio_refined.iter().map(|&s| (s as f64).powi(2)).sum();

        assert!(
            energy_refined < energy_plain,
            "dt-refining subtract left more residual energy ({energy_refined:.0}) than the \
             plain subtract ({energy_plain:.0}) at a 40-sample dt offset"
        );
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

    fn ft4_like_cfg() -> SubtractCfg {
        SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 20.833,
            samples_per_symbol: 576,
            base_offset_s: 0.5,
            gfsk: Some(GfskParams {
                bt: 1.0,
                hmod: 1.0,
                ramp_samples: 576 / 8,
            }),
        }
    }

    fn ft8_like_cfg() -> SubtractCfg {
        SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: Some(GfskParams {
                bt: 2.0,
                hmod: 1.0,
                ramp_samples: 1920 / 8,
            }),
        }
    }

    /// `ls_amp_mag_tweaked`'s NCO-rotation fast path must match
    /// `ls_amp_mag`'s full-resynthesis result (issue #182's FT4
    /// decode-speed fix) within a tight, measured relative tolerance
    /// across both protocols' real `refine_freq` sweep ranges.
    #[test]
    fn ls_amp_mag_tweaked_matches_full_resynthesis() {
        // `ls_amp_mag`'s LS amplitude has a comb of deep nulls a few
        // tenths of a Hz apart (an inherent property of correlating a
        // GFSK reference against itself at a near-but-not-exact carrier,
        // not an artifact of either implementation) where a purely
        // relative-error metric blows up even though the *absolute*
        // discrepancy between the two implementations stays tiny
        // relative to the signal's own amplitude scale. Use a combined
        // absolute-or-relative tolerance, with the absolute floor tied
        // to the known synth amplitude below (measured: away from
        // nulls, relative error is ~0.001-2.3%; at nulls, absolute error
        // stays under ~0.2% of the synth amplitude).
        const AMPLITUDE: f32 = 8000.0;
        const ABS_TOL: f32 = 1e-3 * AMPLITUDE;
        const REL_TOL: f32 = 3e-2;
        let mut max_abs_err = 0.0f32;
        let mut max_rel_err = 0.0f32;

        for (cfg, radius_hz, nsym) in [(ft4_like_cfg(), 5.0f32, 103), (ft8_like_cfg(), 2.5f32, 79)]
        {
            let tones: Vec<u8> = (0..nsym).map(|k| ((k * 3 + 1) % 8) as u8).collect();
            let samples = {
                let n = tones.len() * cfg.samples_per_symbol;
                let mut out = vec![0.0f32; n];
                let gfsk_cfg = crate::engine::dsp::gfsk::GfskCfg {
                    sample_rate: cfg.sample_rate,
                    samples_per_symbol: cfg.samples_per_symbol,
                    bt: cfg.gfsk.unwrap().bt,
                    hmod: cfg.gfsk.unwrap().hmod,
                    ramp_samples: cfg.gfsk.unwrap().ramp_samples,
                };
                crate::engine::dsp::gfsk::synth_f32_into(
                    &mut out, &tones, 1500.0, AMPLITUDE, &gfsk_cfg,
                );
                out
            };
            let offset = (cfg.base_offset_s * cfg.sample_rate) as usize;
            let mut audio = vec![0i16; samples.len() + offset + 4000];
            for (i, &s) in samples.iter().enumerate() {
                audio[offset + i] = s.clamp(-32_768.0, 32_767.0) as i16;
            }

            let (base_cos, base_sin) = generate_iq(&tones, 0.0, &cfg);
            let dt = 1.0 / cfg.sample_rate;

            for dt_sec in [0.0f32, 0.15, -0.2] {
                let mut df = -radius_hz;
                while df <= radius_hz {
                    let freq = 1500.0 + df;
                    let expected = ls_amp_mag(&audio, &tones, freq, dt_sec, &cfg);
                    let actual =
                        ls_amp_mag_tweaked(&audio, &base_cos, &base_sin, freq, dt_sec, dt, &cfg);
                    let abs_err = (actual - expected).abs();
                    let rel_err = abs_err / expected.abs().max(f32::EPSILON);
                    max_abs_err = max_abs_err.max(abs_err);
                    if abs_err > ABS_TOL && rel_err > REL_TOL {
                        panic!(
                            "df={df} dt_sec={dt_sec}: expected={expected} actual={actual} \
                             abs_err={abs_err} (tol {ABS_TOL}) rel_err={rel_err} (tol {REL_TOL})"
                        );
                    }
                    if abs_err > ABS_TOL {
                        // Away from a null (else the ABS_TOL branch above
                        // would have already accepted it) — relative
                        // error is meaningful here.
                        max_rel_err = max_rel_err.max(rel_err);
                    }
                    df += 0.1;
                }
            }
        }
        eprintln!("MEASURED max_abs_err={max_abs_err} max_rel_err (away from nulls)={max_rel_err}");
    }

    /// `refine_freq` must still find the true off-grid carrier after
    /// switching to the NCO-tweaked fast path, not just stay numerically
    /// close to the old per-step resynthesis on average.
    #[test]
    fn refine_freq_finds_true_offgrid_carrier() {
        let cfg = ft4_like_cfg();
        let tones: Vec<u8> = (0..103).map(|k| ((k * 5 + 2) % 8) as u8).collect();
        let freq_hz_init = 1500.0f32;
        let freq_true = freq_hz_init + 1.7;

        let samples = {
            let n = tones.len() * cfg.samples_per_symbol;
            let mut out = vec![0.0f32; n];
            let g = cfg.gfsk.unwrap();
            let gfsk_cfg = crate::engine::dsp::gfsk::GfskCfg {
                sample_rate: cfg.sample_rate,
                samples_per_symbol: cfg.samples_per_symbol,
                bt: g.bt,
                hmod: g.hmod,
                ramp_samples: g.ramp_samples,
            };
            crate::engine::dsp::gfsk::synth_f32_into(
                &mut out, &tones, freq_true, 8000.0, &gfsk_cfg,
            );
            out
        };
        let offset = (cfg.base_offset_s * cfg.sample_rate) as usize;
        let mut audio = vec![0i16; samples.len() + offset + 4000];
        for (i, &s) in samples.iter().enumerate() {
            audio[offset + i] = s.clamp(-32_768.0, 32_767.0) as i16;
        }

        let refined = refine_freq(&audio, &tones, freq_hz_init, 0.0, &cfg, 5.0, 0.1);
        assert!(
            (refined - freq_true).abs() <= 0.1 + 1e-3,
            "refine_freq picked {refined}, expected within 0.1 Hz of true carrier {freq_true}"
        );
    }
}
