//! Protocol-agnostic soft-decision LLR computation.
//!
//! Extracts complex tone spectra for each data symbol, then computes four
//! log-likelihood ratio variants (llra/b/c/d) matching WSJT-X `ft8b.f90`
//! convention — three `nsym = 1, 2, 3` grouping schemes plus a bit-by-bit
//! normalised variant. Parameterised over any [`Protocol`]: NTONES,
//! BITS_PER_SYMBOL, and the SYNC_BLOCKS layout drive the inner loops.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::Protocol;
use crate::core::fft::default_planner;
use crate::core::scalar::{Cmplx, ComplexSpec, LlrScalar, SpecScalar};

// ──────────────────────────────────────────────────────────────────────────
// LLR bundle
// ──────────────────────────────────────────────────────────────────────────

/// Four LLR vectors (a, b, c, d) of length `codeword_len()` bits in
/// LDPC bit-index order, generic over the [`LlrScalar`] storage type.
/// `LlrSet<f32>` (default) is the host path; `LlrSet<Q11i16>` is the
/// embedded fixed-point path used together with `bp_decode_generic_nms`.
#[derive(Clone)]
pub struct LlrSet<T: LlrScalar = f32> {
    /// nsym=1 soft metrics, scaled (matches WSJT-X llra).
    pub llra: Vec<T>,
    /// nsym=2 soft metrics, scaled (matches WSJT-X llrb).
    pub llrb: Vec<T>,
    /// nsym=3 soft metrics, scaled (matches WSJT-X llrc).
    pub llrc: Vec<T>,
    /// nsym=1 bit-normalised (matches WSJT-X llrd).
    pub llrd: Vec<T>,
    /// Optional extra coherent-integration depth strictly between
    /// nsym=2 and nsym=`LLR_NSYM_MAX` (see
    /// [`ModulationParams::LLR_NSYM_MID`](super::ModulationParams::LLR_NSYM_MID)).
    /// Empty unless the protocol sets `LLR_NSYM_MID` and the caller used
    /// [`compute_llr`]. The FT8-specific wrappers also use this slot for
    /// WSJT-X's combined `bmete` vector.
    pub llre: Vec<T>,
}

/// Default LLR scale factor from WSJT-X ft8b.f90. Individual protocols may
/// override via `ModulationParams::LLR_SCALE`.
pub const LLR_SCALE: f32 = 2.83;

// ──────────────────────────────────────────────────────────────────────────
// Pre-LDPC info scrambler (FT4-only)
// ──────────────────────────────────────────────────────────────────────────

/// Apply `Protocol::INFO_SCRAMBLE_RVEC` to the first
/// `min(rvec.len(), info.len())` bits of `info`, in place. No-op when
/// the protocol doesn't define a scrambler.
///
/// Must be called *after* the FEC layer accepts the candidate and
/// produces SNR-estimation tones (since those tones must be encoded
/// from the still-scrambled `info`), but *before* the result is
/// presented to the caller / message codec — the unpacker expects
/// raw 77-bit payload, not its rvec-XORed counterpart.
///
/// Mirrors WSJT-X `ft4_decode.f90:430`
/// (`message77 = mod(message77 + rvec, 2)`).
#[inline]
pub fn descramble_info<P: super::Protocol>(info: &mut [u8]) {
    if let Some(rvec) = <P as super::ModulationParams>::INFO_SCRAMBLE_RVEC {
        let n = rvec.len().min(info.len());
        for (b, &r) in info[..n].iter_mut().zip(rvec.iter()) {
            *b = (*b ^ r) & 1;
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Symbol spectra
// ──────────────────────────────────────────────────────────────────────────

/// Extract complex tone spectra for every channel symbol.
///
/// Returns a flat row-major `Vec<Complex<f32>>` of length `N_SYMBOLS × NTONES`;
/// row `k` / column `t` holds the k-th symbol's t-th tone amplitude, scaled
/// by 1/1000 (matching WSJT-X).
///
/// `i_start` is the sample index in `cd0` of the first symbol, from fine sync.
/// Signed to support candidates whose dt < -0.5 s (signal starts before the
/// cd0 window — first sync block is partially or wholly truncated, but later
/// blocks may still be valid). WSJT-X `ft8b.f90:155-157` boundary policy is
/// all-or-nothing per symbol: when any of the `ds_spb` samples for a symbol
/// would fall outside `cd0`, that symbol's window is zero-filled (rather
/// than partially read). Per-element bounds checking lets edge symbols pull
/// extra signal energy and shifts the LLR sign pattern away from WSJT-X.
pub fn symbol_spectra<P: Protocol>(cd0: &[Complex<f32>], i_start: i32) -> Vec<Cmplx<f32>> {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let ds_spb = (P::NSPS / P::NDOWN) as usize;

    let mut planner = default_planner();
    let fft = planner.plan_forward(ds_spb);

    // cs is spec-scalar storage (`Cmplx<f32>`); buf is an FFT scratch
    // (`Complex<f32>`) — the alias makes them the same machine type
    // but the naming preserves the semantic distinction.
    let mut cs: Vec<Cmplx<f32>> = vec![Cmplx::new(0.0f32, 0.0); n_sym * ntones];
    let mut buf = vec![Complex::new(0.0f32, 0.0); ds_spb];
    let np2 = cd0.len() as i32;

    for k in 0..n_sym {
        let i1 = i_start + (k * ds_spb) as i32;
        if i1 >= 0 && i1 + ds_spb as i32 <= np2 {
            for (j, b) in buf.iter_mut().enumerate() {
                *b = cd0[(i1 as usize) + j];
            }
        } else {
            for b in buf.iter_mut() {
                *b = Complex::new(0.0, 0.0);
            }
        }
        fft.process(&mut buf);
        for (t, bin) in buf.iter().take(ntones).enumerate() {
            cs[k * ntones + t] = *bin / 1000.0;
        }
    }
    cs
}

// ──────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────

// Data-chunk layout (slots between / around sync blocks) is shared
// with the TX side; reuse [`crate::core::tx::data_chunks`] so any
// frame layout the encoder honours is decoded the same way.
use crate::core::tx::data_chunks;

#[inline]
fn normalize_bmet(bmet: &mut [f32]) {
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
        bmet.iter_mut().for_each(|x| *x /= sig);
    }
}

// ──────────────────────────────────────────────────────────────────────────
// LLR computation
// ──────────────────────────────────────────────────────────────────────────

/// Compute soft LLRs from the flat symbol-spectra vector.
///
/// All four LLR variants (llra, llrb, llrc, llrd) are produced.
/// Costs grow with `nsym`: nsym=3 alone is ~80 % of the work
/// because nt = 512 = 8³ tone-combinations. Callers that only need
/// `llra`/`llrd` (the BP-only path) should use [`compute_llr_fast`]
/// instead — it caps at nsym=1 and skips the heavy nsym=2/3 loops.
///
/// The `Cmplx<f32>` (= `Complex<f32>` via the type alias in
/// `core::scalar`) cs entry is a thin convenience wrapper around
/// the generic `compute_llr_generic` — use the generic form when
/// the caller already holds [`Cmplx<S>`] storage for some other
/// `S: SpecScalar` (e.g. `Q14i16` on the embedded fixed-point path).
pub fn compute_llr<P: Protocol, T: LlrScalar>(cs: &[Cmplx<f32>]) -> LlrSet<T> {
    let mut set = compute_llr_generic::<P, f32, T>(cs, P::LLR_NSYM_MAX as usize);
    if let Some(mid) = P::LLR_NSYM_MID {
        let mut bmete = vec![0.0f32; codeword_bit_len::<P>()];
        fill_bmet_for_nsym::<P, f32>(cs, mid as usize, &mut bmete, None, false);
        set.llre = scale_bmet::<T>(bmete, P::LLR_SCALE);
    }
    set
}

/// Same as [`compute_llr`] but stops at nsym=1. `llrb`/`llrc` come
/// back zero-filled. Use when the caller will only ever read
/// `llra` (or `llrd`), e.g. Step 1 of the BP staircase. ~5× faster
/// than the full computation.
pub fn compute_llr_fast<P: Protocol, T: LlrScalar>(cs: &[Cmplx<f32>]) -> LlrSet<T> {
    compute_llr_generic::<P, f32, T>(cs, 1)
}

/// Compute the unnormalised, unscaled bit-metric arrays for ONE
/// `nsym` level. Internal helper shared by [`compute_llr_generic`]
/// (which calls it 1..=max_nsym times) and the lazy single-nsym
/// [`compute_llr_partial`] used to feed the BP staircase one variant
/// at a time without re-doing the cheaper levels.
///
/// `bmet_primary` receives `max_one - max_zero` (= llra at nsym=1,
/// llrb at nsym=2, llrc at nsym=3). At nsym=1 only, `bmet_norm`
/// receives the bit-normalised variant (= llrd). For nsym ≥ 2 pass
/// `bmet_norm = None`.
fn fill_bmet_for_nsym<P: Protocol, S: SpecScalar>(
    cs: &[Cmplx<S>],
    nsym: usize,
    bmet_primary: &mut [f32],
    bmet_norm: Option<&mut [f32]>,
    square_metric: bool,
) {
    let ntones = P::NTONES as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray_map = P::GRAY_MAP;
    let chunks = data_chunks::<P>();
    let codeword_len = bmet_primary.len();

    let nt = ntones.pow(nsym as u32);
    let ibmax = bps * nsym - 1;
    let mut s2 = vec![0.0f32; nt];
    debug_assert!(nsym <= 8, "supported coherent LLR depth is at most 8");

    // Bit-normalised array (llrd) is only produced for nsym=1.
    let mut bmet_norm_holder = bmet_norm;

    // Inner closure: compute bit metrics for one nsym-symbol group
    // starting at frame symbol `ks`, write into `bmet_primary` /
    // `bmet_norm_holder` at relative offset `i_bit_base`. Used by
    // both the regular non-overlapping group sweep and the
    // overlap-tail patch below.
    let mut process_group =
        |ks: usize,
         i_bit_base: usize,
         bmet_primary: &mut [f32],
         bmet_norm_holder: &mut Option<&mut [f32]>| {
            for (i, s2_i) in s2.iter_mut().enumerate() {
                let mut digits = [0usize; 8];
                let mut state = i;
                for j in (0..nsym).rev() {
                    digits[j] = state % ntones;
                    state /= ntones;
                }
                let mut sum_re = 0.0f32;
                let mut sum_im = 0.0f32;
                for j in 0..nsym {
                    let entry = cs[(ks + j) * ntones + gray_map[digits[j]] as usize];
                    sum_re += entry.re.to_f32();
                    sum_im += entry.im.to_f32();
                }
                let magnitude_squared = sum_re * sum_re + sum_im * sum_im;
                *s2_i = if square_metric {
                    magnitude_squared
                } else {
                    magnitude_squared.sqrt()
                };
            }
            for ib in 0..=ibmax {
                let bit_idx = i_bit_base + ib;
                if bit_idx >= codeword_len {
                    break;
                }
                let bit_sel = ibmax - ib;
                let max_one = s2
                    .iter()
                    .enumerate()
                    .filter(|&(i, _)| (i >> bit_sel) & 1 == 1)
                    .map(|(_, &v)| v)
                    .fold(f32::NEG_INFINITY, f32::max);
                let max_zero = s2
                    .iter()
                    .enumerate()
                    .filter(|&(i, _)| (i >> bit_sel) & 1 == 0)
                    .map(|(_, &v)| v)
                    .fold(f32::NEG_INFINITY, f32::max);
                let bm = max_one - max_zero;
                bmet_primary[bit_idx] = bm;
                if let Some(b) = bmet_norm_holder.as_deref_mut() {
                    let den = max_one.max(max_zero);
                    b[bit_idx] = if den > 0.0 { bm / den } else { 0.0 };
                }
            }
        };

    let mut chunk_bit_base = 0usize;
    for &(chunk_start_sym, chunk_len) in &chunks {
        let mut k = 0usize;
        // WSJT-X walks `k = 1, 1+nsym, ... <= chunk_len` and does
        // not require the whole coherent group to remain inside the
        // data chunk.  Consequently the final group can include one
        // or two symbols from the following Costas block.  Its bit
        // writes are clipped by the codeword boundary (and, for the
        // first FT8 half, the next half overwrites any spill).
        //
        // The former overlap-tail patch moved the final group
        // backwards to keep it wholly in data.  That changed bits
        // 82..87 for nsym=2/3 and diverged from `ft8b.f90`.
        while k < chunk_len {
            let ks = chunk_start_sym + k;
            let i_bit_base = chunk_bit_base + k * bps;
            process_group(ks, i_bit_base, bmet_primary, &mut bmet_norm_holder);
            k += nsym;
        }
        chunk_bit_base += chunk_len * bps;
    }
}

#[inline]
fn scale_bmet<T: LlrScalar>(mut v: Vec<f32>, scale: f32) -> Vec<T> {
    normalize_bmet(&mut v);
    v.into_iter().map(|x| T::from_f32(x * scale)).collect()
}

#[inline]
fn codeword_bit_len<P: Protocol>() -> usize {
    let bps = P::BITS_PER_SYMBOL as usize;
    data_chunks::<P>().iter().map(|&(_, l)| l).sum::<usize>() * bps
}

/// Generic LLR computation accepting any [`Cmplx<S>`] cs storage.
/// Inner `bmet` arithmetic stays in `f32` (norms / max-min are
/// awkward to quantise mid-stream) — `S` only changes how we read
/// each cs entry (`S::to_f32` per component). Final scale-and-round
/// to `T` happens at the bundle boundary.
pub fn compute_llr_generic<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    max_nsym: usize,
) -> LlrSet<T> {
    compute_llr_generic_with_metric::<P, S, T>(cs, max_nsym, false, false)
}

/// FT8 amplitude-metric bundle including WSJT-X's combined `bmete`
/// vector in `llre`.
#[doc(hidden)]
pub fn compute_llr_generic_ft8<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    max_nsym: usize,
) -> LlrSet<T> {
    compute_llr_generic_with_metric::<P, S, T>(cs, max_nsym, false, true)
}

/// WSJT-X `imetric=2` variant used by later FT8 subtraction passes.
/// It squares each coherent-group magnitude before forming max-log
/// bit metrics.
#[doc(hidden)]
pub fn compute_llr_generic_power<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    max_nsym: usize,
) -> LlrSet<T> {
    compute_llr_generic_with_metric::<P, S, T>(cs, max_nsym, true, true)
}

fn compute_llr_generic_with_metric<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    max_nsym: usize,
    square_metric: bool,
    include_combined: bool,
) -> LlrSet<T> {
    let codeword_len = codeword_bit_len::<P>();
    let mut bmeta = vec![0.0f32; codeword_len];
    let mut bmetb = vec![0.0f32; codeword_len];
    let mut bmetc = vec![0.0f32; codeword_len];
    let mut bmetd = vec![0.0f32; codeword_len];

    // `max_nsym=3` (FT8 / default) → run nsym ∈ {1, 2, 3}, store
    //   results in {bmeta, bmetb, bmetc} respectively.
    // `max_nsym=4` (FT4, matching WSJT-X `get_ft4_bitmetrics.f90:71`)
    //   → run nsym ∈ {1, 2, 4}, store in {bmeta, bmetb, bmetc}; skip
    //   nsym=3. The 4-symbol coherent integration gives ~3 dB SNR
    //   gain on stable signals vs nsym=3.
    // `max_nsym=8` (FST4, matching WSJT-X `get_fst4_bitmetrics.f90`'s
    //   1/2/4/8-symbol correlation ladder) → run nsym ∈ {1, 2, 8},
    //   store in {bmeta, bmetb, bmetc}; skip 3..7. `nt = NTONES^nsym`
    //   reaches 4^8 = 65536 group hypotheses, same cost WSJT-X pays
    //   (`s2(0:65535)` in the Fortran source) — the `data_chunks`
    //   grouping + tail-patch logic below is already nsym-generic, so
    //   no new per-nsym combination table is needed.
    for nsym in 1usize..=max_nsym {
        if nsym > 2 && nsym != max_nsym {
            continue; // only the two shallow depths + the deepest (max_nsym) run
        }
        let primary: &mut [f32] = match nsym {
            1 => &mut bmeta,
            2 => &mut bmetb,
            n if n == max_nsym => &mut bmetc,
            _ => unreachable!(),
        };
        if nsym == 1 {
            // Split bmeta/bmetd borrow trick: both come from the same
            // function-local Vecs but we need disjoint &mut. The
            // explicit shadow keeps the borrow checker happy.
            let (bmeta_slice, bmetd_slice) = (&mut bmeta[..], &mut bmetd[..]);
            fill_bmet_for_nsym::<P, S>(cs, 1, bmeta_slice, Some(bmetd_slice), square_metric);
        } else {
            fill_bmet_for_nsym::<P, S>(cs, nsym, primary, None, square_metric);
        }
    }

    // WSJT-X FT8 `bmete`: choose the largest-magnitude value from the
    // three *raw* nsym=1/2/3 metric vectors for each bit, then normalize
    // the combined vector once. Choosing after each source vector has
    // already been normalized changes both the winner and OSD reliability
    // ordering on weak signals.
    let bmete = if include_combined && max_nsym >= 3 {
        (0..codeword_len)
            .map(|i| {
                [bmeta[i], bmetb[i], bmetc[i]]
                    .into_iter()
                    .max_by(|left, right| {
                        left.abs()
                            .partial_cmp(&right.abs())
                            .unwrap_or(core::cmp::Ordering::Equal)
                    })
                    .unwrap_or(0.0)
            })
            .collect()
    } else {
        Vec::new()
    };

    let s = P::LLR_SCALE;
    LlrSet {
        llra: scale_bmet::<T>(bmeta, s),
        llrb: scale_bmet::<T>(bmetb, s),
        llrc: scale_bmet::<T>(bmetc, s),
        llrd: scale_bmet::<T>(bmetd, s),
        llre: scale_bmet::<T>(bmete, s),
    }
}

/// Compute a single LLR variant by `nsym` level. Returns the bmet
/// vector matching that nsym (llra at 1, llrb at 2, llrc at 3),
/// normalised + scaled exactly the way [`compute_llr_generic`] would
/// produce that array on its own. nsym=1 returns the same llra
/// [`compute_llr_generic`] produces; the bit-normalised variant
/// (llrd) isn't separately exposed because the staircase consumer
/// already obtains llrd from Step 1's [`compute_llr_fast`] output.
///
/// Used by the FT8 stage-3 BP staircase to lazy-compute Step-2
/// variants — Step-1 already did the nsym=1 work, so Step 2 only
/// pays the (cheap) nsym=2 if variant b is tried, the (expensive)
/// nsym=3 only if variant c is needed. Empirically variant a from
/// Step 1 fails identically in Step 2 (same input, same BP), so
/// this path skips it — see `process_candidates_with` in
/// `ft8::decode_block`.
pub fn compute_llr_partial<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    nsym: usize,
) -> Vec<T> {
    debug_assert!((1..=3).contains(&nsym));
    let codeword_len = codeword_bit_len::<P>();
    let mut bmet = vec![0.0f32; codeword_len];
    fill_bmet_for_nsym::<P, S>(cs, nsym, &mut bmet, None, false);
    scale_bmet::<T>(bmet, P::LLR_SCALE)
}

// ──────────────────────────────────────────────────────────────────────────
// SNR estimation
// ──────────────────────────────────────────────────────────────────────────

/// WSJT-X compatible SNR (dB) estimate from symbol spectra + decoded tones.
///
/// Signal: `Σ |cs[k][itone[k]]|²`. Noise reference: `Σ |cs[k][(itone[k] +
/// NTONES/2) mod NTONES]|²` (tone on the "opposite side" of the comb).
/// SNR_dB = `10·log10(sig/noi − 1) − 27` clamped to −24 dB floor (WSJT-X
/// convention, applied per-tone bandwidth → 2500 Hz reference).
pub fn compute_snr_db<P: Protocol>(cs: &[Cmplx<f32>], itone: &[u8]) -> f32 {
    compute_snr_db_generic::<P, f32>(cs, itone)
}

/// Same as [`compute_snr_db`] but generic over the [`SpecScalar`]
/// type. The signal/noise sums use `S::Wide` accumulator and convert
/// to f32 at the boundary, so a `Cmplx<Q14i16>` cs gives a sane SNR
/// without intermediate f32 quantisation.
pub fn compute_snr_db_generic<P: Protocol, S: SpecScalar>(cs: &[Cmplx<S>], itone: &[u8]) -> f32 {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let mut xsig = 0.0f32;
    let mut xnoi = 0.0f32;
    let offset = ntones / 2;
    for k in 0..n_sym.min(itone.len()) {
        let t = itone[k] as usize % ntones;
        xsig += cs[k * ntones + t].norm_sqr_f32();
        xnoi += cs[k * ntones + (t + offset) % ntones].norm_sqr_f32();
    }
    if xnoi < f32::EPSILON {
        return -24.0;
    }
    let ratio = xsig / xnoi - 1.0;
    if ratio <= 0.001 {
        return -24.0;
    }
    (10.0 * ratio.log10() - 27.0_f32).max(-24.0)
}

/// Hard-decision sync quality — count sync symbols whose dominant tone
/// matches the protocol's Costas pattern. Range is 0..N_SYNC; callers
/// typically threshold on this.
pub fn sync_quality<P: Protocol>(cs: &[Cmplx<f32>]) -> u32 {
    sync_quality_generic::<P, f32>(cs)
}

/// Generic version of [`sync_quality`]. Accepts any [`Cmplx<S>`]
/// slice; the per-tone "is this the dominant magnitude?" comparison
/// uses `S::Wide` (i32 for Q14i16) so no f32 round-trip is needed
/// on the embedded fixed-point path.
pub fn sync_quality_generic<P: Protocol, S: SpecScalar>(cs: &[Cmplx<S>]) -> u32
where
    S::Wide: PartialOrd,
{
    let ntones = P::NTONES as usize;
    let mut count = 0u32;
    for block in P::SYNC_MODE.blocks() {
        let start = block.start_symbol as usize;
        for (t, &expected) in block.pattern.iter().enumerate() {
            let sym = start + t;
            // Fortran `maxloc(s8(:,k))` returns the FIRST index of the
            // maximum on ties. `Iterator::max_by` returns the LAST.
            // Use a manual loop with strict `>` to match WSJT-X.
            let mut best = 0usize;
            let mut best_val = cs[sym * ntones].norm_sqr_wide();
            for a in 1..ntones {
                let v = cs[sym * ntones + a].norm_sqr_wide();
                if v > best_val {
                    best_val = v;
                    best = a;
                }
            }
            if best == expected as usize {
                count += 1;
            }
        }
    }
    count
}
