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
use crate::engine::dsp::downsample::with_default_planner;
use crate::engine::scalar::{Cmplx, ComplexSpec, LlrScalar, SpecScalar};

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
    /// [`compute_llr`] (not [`compute_llr_generic`]/[`compute_llr_fast`]
    /// directly, which never populate this field).
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

    // Reuse the same thread_local planner `downsample_cached` caches
    // (#211) instead of building a fresh one every call — rustfft's own
    // `FftPlanner` caches per size internally, so sharing one instance
    // across both call sites' different sizes is free. `symbol_spectra`
    // runs once per candidate from `engine/pipeline.rs` and
    // `msg/pipeline_ap.rs`, rebuilding this size's twiddle table from
    // scratch every time before this fix (smaller than
    // `downsample_cached`'s FFT, so a smaller absolute cost per call,
    // but the same anti-pattern at the same call frequency).
    let fft = with_default_planner(|planner| planner.plan_forward(ds_spb));

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
// with the TX side; reuse [`crate::engine::tx::data_chunks`] so any
// frame layout the encoder honours is decoded the same way.
use crate::engine::tx::data_chunks;

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
/// `engine::scalar`) cs entry is a thin convenience wrapper around
/// the generic `compute_llr_generic` — use the generic form when
/// the caller already holds [`Cmplx<S>`] storage for some other
/// `S: SpecScalar` (e.g. `Q14i16` on the embedded fixed-point path).
pub fn compute_llr<P: Protocol, T: LlrScalar>(cs: &[Cmplx<f32>]) -> LlrSet<T> {
    let mut set = compute_llr_generic::<P, f32, T>(cs, P::LLR_NSYM_MAX as usize);
    if let Some(mid) = P::LLR_NSYM_MID {
        let mut bmete = vec![0.0f32; codeword_bit_len::<P>()];
        fill_bmet_for_nsym::<P, f32>(cs, mid as usize, &mut bmete, None);
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
///
/// Stack-array bounds for the per-hypothesis scratch below. FST4's
/// `LLR_NSYM_MAX=8` (`ModulationParams::LLR_NSYM_MAX` doc) is the
/// largest `nsym` any protocol uses today; both bounds are sized with
/// headroom (indexing panics if a future protocol ever exceeds them,
/// rather than silently truncating).
const MAX_NSYM: usize = 16;
const MAX_IBMAX_PLUS_1: usize = 32;

/// Complex coherent-sum table for `span` consecutive symbols starting
/// at frame symbol `ks` — one entry per `ntones^span` tone-combination
/// hypothesis, left un-normalised (magnitude is taken by the caller
/// once, at the top level only — matches WSJT-X's own `c1`/`c2`/`c4`
/// staying complex until the final `abs()`,
/// `get_fst4_bitmetrics.f90:94-162`).
///
/// Recursively splits `span` into `hi = span/2`, `lo = span - hi`,
/// builds each half, and combines via outer-sum:
/// `combined[i * lo_len + j] = half_hi[i] + half_lo[j]` — `i` (the
/// earlier `hi` symbols) in the high-order position, `j` (the later
/// `lo` symbols) in the low-order position. This matches the same
/// big-endian mixed-radix digit convention the flat direct-sum
/// computation used (symbol 0 of a group = most significant digit of
/// the hypothesis index), so index `i` means exactly the same
/// tone-combination hypothesis either way.
///
/// Base case `span == 1` is the trivial per-tone lookup (`O(ntones)`,
/// no recursion). Reduces the s2-fill loop's `O(nt * nsym)` direct-sum
/// cost to `O(nt)` at the top-level combine (plus lower-order cost
/// building the halves) — WSJT-X's own reasoning ("eliminates
/// redundant calculations") ported to this per-`nsym`-call shape. For
/// FST4's `nsym=8` rung (`nt=65536`) this cuts the s2-fill cost from
/// `65536*8=524288` adds to `65536 + 256 + 16 ≈ 65808`, ~8x.
fn build_group_amplitudes<S: SpecScalar>(
    cs: &[Cmplx<S>],
    ks: usize,
    span: usize,
    ntones: usize,
    gray_map: &[u8],
) -> Vec<Complex<f32>> {
    if span == 1 {
        return (0..ntones)
            .map(|t| {
                let entry = cs[ks * ntones + gray_map[t] as usize];
                Complex::new(entry.re.to_f32(), entry.im.to_f32())
            })
            .collect();
    }
    let hi = span / 2;
    let lo = span - hi;
    let half_hi = build_group_amplitudes::<S>(cs, ks, hi, ntones, gray_map);
    let half_lo = build_group_amplitudes::<S>(cs, ks + hi, lo, ntones, gray_map);
    let mut out = Vec::with_capacity(half_hi.len() * half_lo.len());
    for &a in &half_hi {
        for &b in &half_lo {
            out.push(a + b);
        }
    }
    out
}

fn fill_bmet_for_nsym<P: Protocol, S: SpecScalar>(
    cs: &[Cmplx<S>],
    nsym: usize,
    bmet_primary: &mut [f32],
    bmet_norm: Option<&mut [f32]>,
) {
    let ntones = P::NTONES as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray_map = P::GRAY_MAP;
    let chunks = data_chunks::<P>();
    let codeword_len = bmet_primary.len();

    let nt = ntones.pow(nsym as u32);
    let ibmax = bps * nsym - 1;
    let mut s2 = vec![0.0f32; nt];

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
            let table = build_group_amplitudes::<S>(cs, ks, nsym, ntones, gray_map);
            for (s2_i, entry) in s2.iter_mut().zip(&table) {
                *s2_i = (entry.re * entry.re + entry.im * entry.im).sqrt();
            }
            // Updates every bit position's running max-when-1 /
            // max-when-0 in one sweep, instead of the previous
            // `2 * (ibmax + 1)` independent full scans (one
            // `filter().fold()` pair per bit position) — `nt * (ibmax +
            // 1)` element visits here vs. `2 * (ibmax + 1) * nt` before.
            //
            // Loop order is `bit_sel` outer, `i` inner (issue #208 Stage
            // D — swapped from the original `i`-outer/`bit_sel`-inner
            // nesting). With `i` outer, LLVM would need to vectorize
            // across `i` while re-running the whole (runtime-bounded)
            // `bit_sel` loop per lane — an outer-loop vectorization it
            // doesn't attempt here regardless of the inner branch shape
            // (confirmed empirically: making the original branch
            // branchless without also flipping the loop order left the
            // compiled `v128` count for this function unchanged). With
            // `bit_sel` outer, the inner loop is a single flat max-
            // reduction over `s2` with a loop-invariant `bit_sel` and a
            // regular period-`2^(bit_sel+1)` predicate on `i` — the
            // classic shape LLVM's inner-loop vectorizer handles, and
            // the branchless select (`f32::NEG_INFINITY` fed to
            // whichever accumulator the bit doesn't select, then an
            // unconditional `max()`) is what lets it lower the
            // per-element predicate to a vector compare-and-blend
            // instead of scalarizing. Confirmed via `wasm-objdump`.
            let mut max_one = [f32::NEG_INFINITY; MAX_IBMAX_PLUS_1];
            let mut max_zero = [f32::NEG_INFINITY; MAX_IBMAX_PLUS_1];
            for bit_sel in 0..=ibmax {
                let mut mo = f32::NEG_INFINITY;
                let mut mz = f32::NEG_INFINITY;
                for (i, &v) in s2.iter().enumerate() {
                    let bit_is_one = (i >> bit_sel) & 1 == 1;
                    let v_for_one = if bit_is_one { v } else { f32::NEG_INFINITY };
                    let v_for_zero = if bit_is_one { f32::NEG_INFINITY } else { v };
                    mo = mo.max(v_for_one);
                    mz = mz.max(v_for_zero);
                }
                max_one[bit_sel] = mo;
                max_zero[bit_sel] = mz;
            }
            for ib in 0..=ibmax {
                let bit_idx = i_bit_base + ib;
                if bit_idx >= codeword_len {
                    break;
                }
                let bit_sel = ibmax - ib;
                let (max_one, max_zero) = (max_one[bit_sel], max_zero[bit_sel]);
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
        while k + nsym <= chunk_len {
            let ks = chunk_start_sym + k;
            let i_bit_base = chunk_bit_base + k * bps;
            process_group(ks, i_bit_base, bmet_primary, &mut bmet_norm_holder);
            k += nsym;
        }
        // Tail patch (issue #18, slice 5):
        // when `chunk_len` is not a multiple of `nsym`, the last
        // `chunk_len - k` symbols are uncovered by the non-overlapping
        // sweep above. WSJT-X's `get_ft4_bitmetrics.f90` iterates over
        // the **whole** frame (including sync symbols) so the loop
        // structure naturally covers everything; our chunk-restricted
        // loop drops `chunk_len % nsym` data symbols per chunk = 6
        // bits total for FT4 (chunk_len=29, nsym=4). We patch by
        // running one final overlapping group at `k = chunk_len -
        // nsym` whose later bits overwrite the prior group's overlap
        // and whose new bits fill the tail. Identity for chunks that
        // are exact multiples of `nsym` (k == chunk_len there).
        if k < chunk_len && chunk_len >= nsym {
            let tail_k = chunk_len - nsym;
            let ks = chunk_start_sym + tail_k;
            let i_bit_base = chunk_bit_base + tail_k * bps;
            process_group(ks, i_bit_base, bmet_primary, &mut bmet_norm_holder);
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
            fill_bmet_for_nsym::<P, S>(cs, 1, bmeta_slice, Some(bmetd_slice));
        } else {
            fill_bmet_for_nsym::<P, S>(cs, nsym, primary, None);
        }
    }

    let s = P::LLR_SCALE;
    LlrSet {
        llra: scale_bmet::<T>(bmeta, s),
        llrb: scale_bmet::<T>(bmetb, s),
        llrc: scale_bmet::<T>(bmetc, s),
        llrd: scale_bmet::<T>(bmetd, s),
        llre: Vec::new(),
    }
}

/// Compute a single LLR variant at one `nsym` level, normalised +
/// scaled exactly the way [`compute_llr_generic`] would produce that
/// array on its own (nsym=1 returns the same `llra` `compute_llr_generic`
/// produces; the bit-normalised variant `llrd` isn't separately exposed
/// because callers already obtain it from [`compute_llr_fast`]'s
/// output).
///
/// Used by staircase-style decode paths to lazy-compute deeper variants
/// only once shallower ones have already failed BP — e.g. the FT8
/// stage-3 BP staircase (`nsym` up to `LLR_NSYM_MAX=3`) and the generic
/// `engine::pipeline` engine's own lazy variant loop (FST4's `nsym` up to
/// `LLR_NSYM_MAX=8`, plus `LLR_NSYM_MID=4`).
pub fn compute_llr_partial<P: Protocol, S: SpecScalar, T: LlrScalar>(
    cs: &[Cmplx<S>],
    nsym: usize,
) -> Vec<T> {
    debug_assert!((1..=MAX_NSYM).contains(&nsym));
    let codeword_len = codeword_bit_len::<P>();
    let mut bmet = vec![0.0f32; codeword_len];
    fill_bmet_for_nsym::<P, S>(cs, nsym, &mut bmet, None);
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
