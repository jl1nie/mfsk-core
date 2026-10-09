//! FFT-based downconversion / decimation.
//!
//! Ported from WSJT-X `ft8_downsample.f90` and generalised to arbitrary MFSK
//! signals. The algorithm is:
//!
//! 1. Zero-pad the real input to [`DownsampleCfg::fft1_size`] samples and take
//!    a forward FFT.
//! 2. Extract the positive-frequency bins covering
//!    `[f0 - leading_pad·Δf, f0 + trailing_pad·Δf]`, where the pads are
//!    expressed in tone-spacing units.
//! 3. Hann-taper [`DownsampleCfg::edge_taper_bins`] on each side of the
//!    extracted block to suppress FFT ringing.
//! 4. Cyclic-rotate so that `f0` lands at DC.
//! 5. Inverse FFT of size [`DownsampleCfg::fft2_size`] gives the decimated
//!    complex baseband.
//! 6. Scale by `1 / sqrt(fft1_size · fft2_size)`.
//!
//! The output sample rate is `input_rate · fft2_size / fft1_size`. For the FT8
//! tuning (12 000 → 200 Hz) it is 200 Hz; FT4 will pick a different
//! `fft2_size` to keep a wider baseband.
//!
//! ## Why a config struct rather than `<P>` ?
//!
//! `fft1_size` is chosen per protocol for FFT efficiency (highly-composite
//! numbers close to the slot length). It is not simply derived from
//! `SLOT_S · sample_rate` — it is a tunable tied to the FFT backend's
//! strengths. Keeping it in a runtime struct lets callers express values that
//! are awkward to pin to associated constants.

use alloc::vec;
use alloc::vec::Vec;
use core::iter;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use crate::engine::fft::{FftPlanner, default_planner};

// `default_planner()`'s own doc comment says to "reuse the same instance
// across all decodes in a session so rustfft's twiddle cache hits" — but
// `build_fft_cache`/`downsample_cached` used to construct a fresh one on
// every call instead. `downsample_cached` alone is called once per FT8
// candidate (~30 cand × 3 pass = ~90×/slot, see
// `fill_symbol_spectra_via_cd0`'s neighboring `SYMBOL_FFT_32` comment for
// the same anti-pattern already fixed at a different call site), so every
// call rebuilt the `fft2_size`-point inverse-FFT plan's twiddle table
// from scratch via scalar sin/cos, even though `fft2_size` never varies
// within a decode session. A wasm --prof profile (issue #208, Stage C)
// attributed ~12% of total decode wall-clock to exactly this — bigger
// than any dense-kernel vectorization target found in that pass. Fixed
// the same way `subtract_tones_lpf_fft`'s filter-response FFT plan was
// (see docs/notes/BENCHMARKS.md, 2026-07-25 entry): cache the planner
// itself, not just its output, keyed by nothing since one thread only
// ever needs one live planner regardless of how many distinct sizes it's
// asked to plan (rustfft's own `FftPlanner` caches per size internally).
//
// `std`-gated because `thread_local!` needs std; `no_std` (embedded,
// `fft-extern`) callers keep constructing a fresh planner per call —
// those call sites don't reach this function at all today (FT8's
// `fill_symbol_spectra_via_cd0`, the only `downsample_cached` caller in
// the hot per-candidate loop, is itself `fft-rustfft`-gated), so this is
// out of scope for embedded rather than a regression there.
#[cfg(feature = "std")]
std::thread_local! {
    static DOWNSAMPLE_PLANNER: core::cell::RefCell<Box<dyn FftPlanner>> =
        core::cell::RefCell::new(default_planner());
}

#[cfg(feature = "std")]
#[inline]
pub(crate) fn with_default_planner<R>(f: impl FnOnce(&mut dyn FftPlanner) -> R) -> R {
    DOWNSAMPLE_PLANNER.with_borrow_mut(|planner| f(planner.as_mut()))
}

#[cfg(not(feature = "std"))]
#[inline]
pub(crate) fn with_default_planner<R>(f: impl FnOnce(&mut dyn FftPlanner) -> R) -> R {
    f(default_planner().as_mut())
}

/// Runtime parameters shared by [`downsample`], [`downsample_cached`], and
/// [`build_fft_cache`]. Callers typically keep one instance per protocol.
#[derive(Clone, Copy, Debug)]
pub struct DownsampleCfg {
    /// Input sample rate in Hz (12 000 for the WSJT pipeline).
    pub input_rate: u32,

    /// Zero-padded forward-FFT length.
    pub fft1_size: usize,

    /// Inverse-FFT length = output sample count per call.
    pub fft2_size: usize,

    /// Tone spacing of the modulation in Hz (passed as the bandwidth unit so
    /// the extracted band scales with the protocol).
    pub tone_spacing_hz: f32,

    /// Bins of headroom below `f0` in tone-spacing units (FT8 uses 1.5).
    pub leading_pad_tones: f32,

    /// Bins of headroom above `f0 + (ntones-1)·tone_spacing` in tone-spacing
    /// units (FT8 uses 1.5 past tone 7 → 8.5 total).
    pub trailing_pad_tones: f32,

    /// Number of data tones; used together with `trailing_pad_tones` to
    /// determine the upper edge of the extracted band.
    pub ntones: u32,

    /// Length of the raised-cosine taper applied to each edge (typically 101).
    pub edge_taper_bins: usize,
}

impl DownsampleCfg {
    #[inline]
    fn bin_hz(&self) -> f32 {
        self.input_rate as f32 / self.fft1_size as f32
    }
}

/// Compute only the large forward-FFT cache. Subsequent calls to
/// [`downsample_cached`] reuse it across all candidate frequencies, which is
/// the expensive operation amortised by `ft8-core::decode::process_candidate`.
#[inline]
pub fn build_fft_cache(audio: &[i16], cfg: &DownsampleCfg) -> Vec<Complex<f32>> {
    let mut x: Vec<Complex<f32>> = audio
        .iter()
        .map(|&s| Complex::new(s as f32, 0.0))
        .chain(iter::repeat(Complex::new(0.0, 0.0)))
        .take(cfg.fft1_size)
        .collect();
    with_default_planner(|planner| planner.plan_forward(cfg.fft1_size).process(&mut x));
    x
}

/// Downconvert `audio` to a complex baseband centred on `f0`.
///
/// Returns the decimated signal plus the forward-FFT cache so the caller can
/// feed it to subsequent frequency-shifted calls without recomputing the
/// 192 k-point transform.
#[inline]
pub fn downsample(
    audio: &[i16],
    f0: f32,
    cfg: &DownsampleCfg,
) -> (Vec<Complex<f32>>, Vec<Complex<f32>>) {
    let cache = build_fft_cache(audio, cfg);
    let out = downsample_cached(&cache, f0, cfg);
    (out, cache)
}

/// Downconvert using a pre-computed forward-FFT cache.
#[inline]
pub fn downsample_cached(
    fft_cache: &[Complex<f32>],
    f0: f32,
    cfg: &DownsampleCfg,
) -> Vec<Complex<f32>> {
    debug_assert_eq!(fft_cache.len(), cfg.fft1_size);

    let df = cfg.bin_hz();
    let baud = cfg.tone_spacing_hz;

    let i0 = (f0 / df).round() as usize;
    let ft = f0 + (cfg.ntones as f32 - 1.0 + cfg.trailing_pad_tones) * baud;
    let fb = f0 - cfg.leading_pad_tones * baud;
    let it = ((ft / df).round() as usize).min(cfg.fft1_size / 2);
    let ib = ((fb / df).round() as usize).max(1);
    let k = it.saturating_sub(ib) + 1;

    let mut c1 = vec![Complex::new(0.0f32, 0.0); cfg.fft2_size];
    for (dst, src) in c1[..k.min(cfg.fft2_size)]
        .iter_mut()
        .zip(fft_cache[ib..=it].iter())
    {
        *dst = *src;
    }

    // Raised-cosine taper on leading and trailing edges.
    let et = cfg.edge_taper_bins;
    if et > 1 {
        let n = et - 1;
        let taper: Vec<f32> = (0..et)
            .map(|i| 0.5 * (1.0 + (i as f32 * core::f32::consts::PI / n as f32).cos()))
            .collect();
        for i in 0..et.min(k) {
            c1[i] *= taper[n - i];
        }
        if k > et {
            for i in 0..et {
                c1[k - et + i] *= taper[i];
            }
        }
    }

    // Cyclic shift so f0 lands on DC.
    let shift = i0.saturating_sub(ib) % cfg.fft2_size;
    c1.rotate_left(shift);

    // Inverse FFT.
    with_default_planner(|planner| planner.plan_inverse(cfg.fft2_size).process(&mut c1));

    // Combined scale factor.
    let fac = 1.0 / ((cfg.fft1_size as f32) * (cfg.fft2_size as f32)).sqrt();
    for s in c1.iter_mut() {
        *s *= fac;
    }

    c1
}

/// Pack `nfft2` bins of `spectrum` into FFT-shift order around `i0`, each
/// multiplied by `scale`, ready for an inverse FFT of that size: bins
/// `i0..=i0 + nfft2/2` go to `out[0..=nfft2/2]` and the `nfft2/2 − 1` bins
/// below `i0` go to the top of `out`, so a band centred on bin `i0` comes out
/// of the inverse transform as a complex baseband centred on DC. Bins outside
/// `spectrum` (below 0, past its end) leave their slot zero.
///
/// This is the re-pack `wsprd`'s `readwavfile` (`wsprd.c:172-177`) and WSJT-X's
/// `downsam9` both perform; they differ in what they scale by and in where `i0`
/// comes from (rounded there, truncated here), which stay with their callers
/// (#425). It is not [`downsample_cached`]'s layout, which copies a
/// tone-spacing-sized window `ib..=it` with a raised-cosine taper and
/// rotates it, so the FT8-family path is left as it is.
///
/// `scale == 1.0` is bit-exact: multiplying an `f32` by one returns it.
pub(crate) fn repack_centered(
    spectrum: &[Complex<f32>],
    i0: i64,
    nfft2: usize,
    scale: f32,
) -> Vec<Complex<f32>> {
    let mut out = vec![Complex::new(0.0f32, 0.0); nfft2];
    let nh2 = (nfft2 / 2) as i64;
    let len = spectrum.len() as i64;
    let nfft2 = nfft2 as i64;

    // Piece 1: i in [0, nh2], j = i0 + i. Each piece is a contiguous run, so
    // clamp its valid `i` range to `0 <= j < len` once instead of testing the
    // bound on every one of the nfft2 iterations.
    let p1_lo = (-i0).clamp(0, nh2 + 1);
    let p1_hi = (len - i0).clamp(0, nh2 + 1);
    for i in p1_lo..p1_hi {
        out[i as usize] = spectrum[(i0 + i) as usize] * scale;
    }

    // Piece 2: i in (nh2, nfft2), j = i0 + i − nfft2.
    let p2_lo = (nfft2 - i0).clamp(nh2 + 1, nfft2);
    let p2_hi = (len - i0 + nfft2).clamp(nh2 + 1, nfft2);
    for i in p2_lo..p2_hi {
        out[i as usize] = spectrum[(i0 + i - nfft2) as usize] * scale;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `wspr::baseband::decimate_to_baseband`'s loop as it was.
    fn wspr_loop(buf: &[Complex<f32>], i0: usize, nfft2: usize) -> Vec<Complex<f32>> {
        let nh2 = nfft2 / 2;
        let mut fftin = vec![Complex::new(0.0f32, 0.0); nfft2];
        for i in 0..nfft2 {
            let j = if i > nh2 {
                i0.wrapping_add(i).wrapping_sub(nfft2)
            } else {
                i0 + i
            };
            if j < buf.len() {
                fftin[i] = buf[j];
            }
        }
        fftin
    }

    /// `jt9::softsym::AudioFft::downsam9`'s two clamped pieces as they were.
    fn jt9_pieces(c1: &[Complex<f32>], i0: i64, nfft2: usize, fac: f32) -> Vec<Complex<f32>> {
        let nh2 = (nfft2 / 2) as i64;
        let mut c2 = vec![Complex::new(0.0f32, 0.0); nfft2];
        let c1_len = c1.len() as i64;
        let nfft2 = nfft2 as i64;
        let p1_lo = (-i0).clamp(0, nh2 + 1);
        let p1_hi = (c1_len - i0).clamp(0, nh2 + 1);
        for i in p1_lo..p1_hi {
            c2[i as usize] = c1[(i0 + i) as usize] * fac;
        }
        let p2_lo = (nfft2 - i0).clamp(nh2 + 1, nfft2);
        let p2_hi = (c1_len - i0 + nfft2).clamp(nh2 + 1, nfft2);
        for i in p2_lo..p2_hi {
            c2[i as usize] = c1[(i0 + i - nfft2) as usize] * fac;
        }
        c2
    }

    fn spectrum(n: usize) -> Vec<Complex<f32>> {
        let mut x = 0x2026_1009u32;
        (0..n)
            .map(|_| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let re = (x >> 8) as f32 / 8_388_608.0 - 1.0;
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                Complex::new(re, (x >> 8) as f32 / 8_388_608.0 - 1.0)
            })
            .collect()
    }

    fn bits(v: &[Complex<f32>]) -> Vec<(u32, u32)> {
        v.iter().map(|c| (c.re.to_bits(), c.im.to_bits())).collect()
    }

    /// Both users' loops, bit for bit, for `i0` inside the spectrum, at either
    /// edge, and past either end.
    #[test]
    fn repack_is_bit_identical_to_both_loops_it_replaced() {
        for (len, nfft2) in [(10_000usize, 512usize), (6_145, 1_512), (400, 64)] {
            let sp = spectrum(len);
            for i0 in [
                -(nfft2 as i64),
                -5,
                0,
                1,
                (nfft2 / 2) as i64,
                (len / 2) as i64,
                len as i64 - nfft2 as i64,
                len as i64 - 3,
                len as i64,
                len as i64 + 7,
            ] {
                let new = repack_centered(&sp, i0, nfft2, 1.0);
                if i0 >= 0 {
                    assert_eq!(
                        bits(&new),
                        bits(&wspr_loop(&sp, i0 as usize, nfft2)),
                        "wspr loop, len {len}, nfft2 {nfft2}, i0 {i0}"
                    );
                }
                let fac = 0.731_f32;
                assert_eq!(
                    bits(&repack_centered(&sp, i0, nfft2, fac)),
                    bits(&jt9_pieces(&sp, i0, nfft2, fac)),
                    "jt9 pieces, len {len}, nfft2 {nfft2}, i0 {i0}"
                );
            }
        }
    }

    #[test]
    fn a_band_centred_on_i0_lands_on_dc_after_the_inverse() {
        // A single bin at `i0` must come out as a constant (DC) baseband.
        let nfft2 = 64;
        let mut sp = vec![Complex::new(0.0f32, 0.0); 1_000];
        sp[300] = Complex::new(1.0, 0.0);
        let mut packed = repack_centered(&sp, 300, nfft2, 1.0);
        with_default_planner(|p| p.plan_inverse(nfft2).process(&mut packed));
        for c in &packed {
            assert!((c.re - 1.0).abs() < 1e-6 && c.im.abs() < 1e-6, "{c}");
        }
    }
}
