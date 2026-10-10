//! Generic 2-D (Δf, Δt) fine-sync refine — port of WSJT-X `sync4d.f90`
//! (FT4) and `fst4_sync_search` (FST4).
//!
//! **FT4** uses [`ft4_sync_search`]: faithful port of WSJT-X
//! `ft4_decode.f90`'s `isync=1`/`isync=2` loop (`sync4d.f90` scorer) —
//! a coherent full-slot Δt search, not a local window.
//!
//! **FST4** uses [`fst4_sync_search`]: faithful port of WSJT-X
//! `fst4_decode.f90:879-925`.  Coarse pass sweeps ±1.5 s of the full
//! slot (not just a local coarse_sync window) so the winner is always
//! near the true peak, then a fine pass ±7 × 0.02·baud × ±4 samples
//! locks in.
//!
//! Both protocols previously used a shared two-pass *local* refine
//! (`sync2d_refine`/`Sync2dConfig`, ±10-±20 downsampled-sample window
//! around the coarse-sync candidate) — removed (2026-07-20, no call
//! sites left) once both FST4 (#146) and FT4 (issue #72, FT4_BENCHMARK.md
//! section 7) moved to their own full-slot coherent searches; a local
//! window at the coarse-sync candidate's position couldn't recover from
//! cases where that non-coherent Δt estimate was wrong by more than the
//! window's own radius (see the two sections above for the measurements
//! that motivated each protocol's switch).
//!
//! The output is a [`Sync2dResult`] with refined `(freq_hz, i0, score)`;
//! downstream `symbol_spectra` is invoked on a freq-twiddled `cd0` so
//! per-symbol FFT bins land on the correct tones for the refined carrier.

use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use crate::engine::SyncFrontEnd;
use crate::engine::dsp::dotprod::{AlignedF32, dot_f32};
use crate::engine::sync::{SyncCandidate, SyncDims};

/// Output of [`fst4_sync_search`] / [`ft4_sync_search`].
#[derive(Clone, Debug)]
pub struct Sync2dResult {
    /// Refined carrier frequency in Hz (= initial + Δf_best).
    pub freq_hz: f32,
    /// Refined symbol-0 sample offset in `cd0` (signed; negative means
    /// the frame nominally started before sample 0 of the baseband).
    pub i0: i32,
    /// Peak sync power summed across all Costas blocks at (Δf, Δt)_best.
    ///
    /// [`f32::NEG_INFINITY`] means no start fits ([`fst4_sync_search`] only): WSJT-X
    /// v3.3.0-beta1's `isbest=-1`, which `decode_kernel` skips. `i0` is then the nominal one.
    pub score: f32,
}

// ──────────────────────────────────────────────────────────────────────────
// Coherent block correlator — FST4-specific
//
// WSJT-X `sync_fst4` (fst4_decode.f90:657) computes the sync score by:
//   1. Building a phase-continuous FSK reference (`csync1`/`csync2`) for
//      the entire 8-symbol Costas block with continuous phase accumulation
//      across symbol boundaries.
//   2. Computing ONE coherent inner product over all 8*nss samples:
//      z = sum(cd0 * conjg(csynct1))
//   3. Score = |z| / nz   (AMPLITUDE, not power)
//
// Our previous `score_costas_block` computed the SUM OF PER-TONE POWERS:
//   sum_k |inner_product_k|²  (8 separate dot-products, then sum of powers)
// which gives ~3 dB worse SNR discrimination in the sync score vs the
// coherent approach, causing noise peaks to win at near-threshold SNR even
// with the full-slot time search.
// ──────────────────────────────────────────────────────────────────────────

/// Create a phase-continuous FSK reference for one Costas block.
///
/// Unlike `make_costas_ref` (per-tone, phase reset to 0 each symbol),
/// this accumulates phase continuously — matching the actual phase-
/// continuous FSK modulation of the FST4 signal.  Returned length is
/// `pattern.len() * ds_spb`.
fn make_costas_ref_continuous(pattern: &[u8], ds_spb: usize) -> Vec<Complex<f32>> {
    let mut out = Vec::with_capacity(pattern.len() * ds_spb);
    let mut phi = 0.0f64;
    for &tone in pattern {
        let dphi = core::f64::consts::TAU * (tone as f64) / (ds_spb as f64);
        for _ in 0..ds_spb {
            out.push(Complex::new(phi.cos() as f32, phi.sin() as f32));
            phi += dphi;
        }
    }
    out
}

// `ft4_sync_search_window` calls `make_costas_ref_continuous` once per
// call (not per grid cell — that per-cell cost was already eliminated,
// see the module comments inside that function) to build `blocks_ref`,
// but the function itself is called once per *candidate* (~31/decode
// on the real FT4 golden WAV), and `(pattern, ds_spb)` never varies
// within one decode call — every candidate for a given protocol `P`
// rebuilds an identical reference from scratch. Mirrors
// `engine::sync::cached_costas_ref`'s exact rationale and shape
// (2-slot thread_local, content-keyed, std-gated with an uncached
// no_std fallback) for this module's own phase-continuous variant.
#[cfg(feature = "std")]
type CostasRefContinuousCacheEntry = (&'static [u8], usize, Vec<Complex<f32>>);

#[cfg(feature = "std")]
std::thread_local! {
    static COSTAS_REF_CONTINUOUS_CACHE: core::cell::RefCell<Vec<CostasRefContinuousCacheEntry>> =
        const { core::cell::RefCell::new(Vec::new()) };
}

#[cfg(feature = "std")]
fn cached_costas_ref_continuous(pattern: &'static [u8], ds_spb: usize) -> Vec<Complex<f32>> {
    COSTAS_REF_CONTINUOUS_CACHE.with_borrow_mut(|cache| {
        if let Some((_, _, flat)) = cache.iter().find(|(p, d, _)| *p == pattern && *d == ds_spb) {
            return flat.clone();
        }
        let flat = make_costas_ref_continuous(pattern, ds_spb);
        if cache.len() >= 2 {
            cache.remove(0);
        }
        cache.push((pattern, ds_spb, flat.clone()));
        flat
    })
}

/// `no_std` (embedded) fallback — `thread_local!` needs `std`. FT4's
/// generic pipeline doesn't reach embedded builds today (same
/// reasoning as `cached_costas_ref`'s own no_std fallback), so a plain
/// uncached rebuild is fine here.
#[cfg(not(feature = "std"))]
fn cached_costas_ref_continuous(pattern: &[u8], ds_spb: usize) -> Vec<Complex<f32>> {
    make_costas_ref_continuous(pattern, ds_spb)
}

/// A Costas block's reference, laid out for [`dot_f32`].
///
/// The scorer wants `Σ cd0[n] · conj(ref[n])`, a complex accumulation.
/// Written out, its two halves are each a **real** dot product over the
/// same interleaved `(re, im, re, im, …)` memory the complex arrays
/// already are:
///
/// - `Re z = Σ (c_re·r_re + c_im·r_im)` — the interleaved arrays dotted
///   directly;
/// - `Im z = Σ (c_im·r_re − c_re·r_im)` — the same, against a reference
///   whose every sample has been rewritten as `(−im, re)`.
///
/// So one scalar complex loop becomes two calls into whatever
/// `dot_f32` is backed by, at identical flop count. On the CoreS3 that
/// backend is `dsps_dotprod_f32_aes3`, measured at 3.6x the portable
/// loop for a misaligned 288-deep dot — and 288 is exactly this
/// block's length for FST4-60.
///
/// Both layouts are built once per `(block, df)` in the twiddle step,
/// where the reference is already being rebuilt anyway.
struct FlatRef {
    /// Reference length in complex samples. Not derived from
    /// `plain.len()`: [`AlignedF32`] rounds its allocation up to a
    /// multiple of four, so that would over-report for odd block
    /// lengths.
    n: usize,
    /// The reference, interleaved.
    ///
    /// 16-byte aligned, which a plain `Vec<f32>` is not: `dot_f32`'s
    /// esp-dsp backend needs **both** operands aligned and the length a
    /// multiple of four, or it silently takes a scalar body ~2.3x
    /// slower. The length is already fine (a Costas block is
    /// `nsym · ds_spb · 2` f32), so alignment was the whole gap —
    /// measured on a CoreS3 at **22 % of this scorer's 284 688 dots per
    /// slot reaching the fast path**, the other 78 % failing on
    /// alignment alone (`docs/notes/FT4_BENCHMARK.md` §29).
    plain: AlignedF32,
    /// The same reference with each `(re, im)` rewritten as `(−im, re)`.
    swapped: AlignedF32,
    /// The same two, shifted right by one complex sample (two leading
    /// zero `f32`), for windows that start at an **odd** `cd0` index.
    ///
    /// `score_flat_coherent` reads `cd0[s0..]` reinterpreted as `f32`,
    /// so its byte offset is `s0 · 8` and it is 16-byte aligned only
    /// when `s0` is even. Rather than copy the window — which loses,
    /// measured: the same trade cost more than it saved in
    /// `ft4::ddc`'s FIR (§27) — the odd case reads from `s0 - 1`,
    /// which *is* aligned, against a reference whose first sample is
    /// zero. Every added term is exactly `0.0 · x`; only the rounding
    /// of the sum moves.
    ///
    /// Same trick as `FirStage`'s per-phase tap tables, with two
    /// phases instead of four because the unit here is a complex
    /// sample rather than a single `f32`.
    #[cfg(feature = "dotprod-extern")]
    plain_odd: AlignedF32,
    #[cfg(feature = "dotprod-extern")]
    swapped_odd: AlignedF32,
}

impl FlatRef {
    /// Allocated once per block and refilled for every frequency
    /// offset — **not** rebuilt per offset.
    ///
    /// The search sweeps 40 offsets per candidate, so a fresh pair of
    /// buffers per offset is 400 allocations of ~2.3 KB each, per
    /// candidate. That is not free on this target: with
    /// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` those land in
    /// internal DRAM while there is any, and in PSRAM once there is
    /// not — so the same code measured 780 ms per candidate in a bench
    /// with internal DRAM to spare and 1131 ms in an application that
    /// had spent it on WiFi and task stacks. Filling in place removes
    /// the question.
    fn with_len(n: usize) -> Self {
        Self::with_len_min_alloc(n, 0)
    }

    /// [`Self::with_len`] with every buffer allocated at no less than
    /// `min_bytes` — same contents, placed past an allocator's
    /// internal-DRAM threshold. See [`Ft4SweepScratch::new_with_min_alloc`].
    fn with_len_min_alloc(n: usize, min_bytes: usize) -> Self {
        Self {
            n,
            plain: AlignedF32::with_min_alloc(n * 2, min_bytes),
            swapped: AlignedF32::with_min_alloc(n * 2, min_bytes),
            // Two leading zeros, then the same `n * 2` samples.
            #[cfg(feature = "dotprod-extern")]
            plain_odd: AlignedF32::with_min_alloc(n * 2 + 2, min_bytes),
            #[cfg(feature = "dotprod-extern")]
            swapped_odd: AlignedF32::with_min_alloc(n * 2 + 2, min_bytes),
        }
    }

    /// Overwrite with `flat_ref` carrier-shifted by `df_hz`.
    fn fill(&mut self, flat_ref: &[Complex<f32>], df_hz: f32, ds_rate: f32) {
        let omega = 2.0 * PI * df_hz / ds_rate;
        let shift = df_hz.abs() >= f32::EPSILON;
        for (n, &r) in flat_ref.iter().enumerate() {
            let r = if shift {
                let p = omega * n as f32;
                r * Complex::new(p.cos(), p.sin())
            } else {
                r
            };
            self.plain.as_mut_slice()[2 * n] = r.re;
            self.plain.as_mut_slice()[2 * n + 1] = r.im;
            self.swapped.as_mut_slice()[2 * n] = -r.im;
            self.swapped.as_mut_slice()[2 * n + 1] = r.re;
            #[cfg(feature = "dotprod-extern")]
            {
                self.plain_odd.as_mut_slice()[2 * n + 2] = r.re;
                self.plain_odd.as_mut_slice()[2 * n + 3] = r.im;
                self.swapped_odd.as_mut_slice()[2 * n + 2] = -r.im;
                self.swapped_odd.as_mut_slice()[2 * n + 3] = r.re;
            }
        }
    }

    fn len(&self) -> usize {
        self.n
    }

    /// [`fill`](Self::fill) with the carrier phasor read from a table
    /// instead of evaluated per sample.
    ///
    /// `phasor[n]` must be `Complex::new((omega·n).cos(),
    /// (omega·n).sin())` for the same `omega` — then this writes the
    /// same bytes `fill` would, and the search finds the same argmax
    /// bit for bit.
    ///
    /// The `df == 0` case keeps `fill`'s own branch rather than
    /// multiplying by `1 + 0j`, so that path is untouched.
    fn fill_with(&mut self, flat_ref: &[Complex<f32>], df_hz: f32, phasor: &[Complex<f32>]) {
        let shift = df_hz.abs() >= f32::EPSILON;
        for (n, &r) in flat_ref.iter().enumerate() {
            let r = if shift { r * phasor[n] } else { r };
            self.plain.as_mut_slice()[2 * n] = r.re;
            self.plain.as_mut_slice()[2 * n + 1] = r.im;
            self.swapped.as_mut_slice()[2 * n] = -r.im;
            self.swapped.as_mut_slice()[2 * n + 1] = r.re;
            #[cfg(feature = "dotprod-extern")]
            {
                self.plain_odd.as_mut_slice()[2 * n + 2] = r.re;
                self.plain_odd.as_mut_slice()[2 * n + 3] = r.im;
                self.swapped_odd.as_mut_slice()[2 * n + 2] = -r.im;
                self.swapped_odd.as_mut_slice()[2 * n + 3] = r.re;
            }
        }
    }
}

/// `cd0` with its base guaranteed 16-byte aligned, copying only if it
/// is not already.
///
/// [`score_flat_coherent`]'s odd-phase reference handles the *parity*
/// of `s0`, but only if the buffer's base is aligned to start with: at
/// an 8-mod-16 base **no** `s0` works and every dot falls to the scalar
/// path. `cd0` reaches here as a `Vec<Complex<f32>>` from
/// `downsample_cached` or `ft4::ddc`, and `Complex<f32>` has alignment
/// 4, so nothing guarantees it.
///
/// This was not hypothetical. The §29 measurement showed 100 % of the
/// scorer's 284 688 dots on the PIE path — and that held only because
/// the allocator happened to hand back an aligned 40 KB block. Leaking
/// one more 18 KB internal buffer elsewhere in the same binary shifted
/// the heap, and the next run measured **0 %**, with the search back at
/// 1 789 ms from 1 049 (`docs/notes/FT4_BENCHMARK.md` §32). A measured
/// 100 % that depends on an uncontrolled allocator is not a guarantee.
///
/// The copy is per *call* — once per candidate, amortised over the
/// ~25 000 dots the grid search then does — which is the case §29
/// identified as affordable, unlike `ft4::ddc`'s per-dot staging that
/// §27 measured and rejected.
struct AlignedCd0 {
    store: Option<AlignedF32>,
}

impl AlignedCd0 {
    fn new(cd0: &[Complex<f32>]) -> Self {
        if (cd0.as_ptr() as usize).is_multiple_of(16) {
            return Self { store: None };
        }
        let n2 = cd0.len() * 2;
        let mut a = AlignedF32::with_min_alloc(n2, 0);
        // SAFETY: `Complex<f32>` is `repr(C)` over two `f32`, so this
        // is exactly the interleaved view of the same samples.
        let src = unsafe { core::slice::from_raw_parts(cd0.as_ptr() as *const f32, n2) };
        a.as_mut_slice()[..n2].copy_from_slice(src);
        Self { store: Some(a) }
    }

    fn get<'a>(&'a self, orig: &'a [Complex<f32>]) -> &'a [Complex<f32>] {
        match &self.store {
            // SAFETY: the store holds `orig.len() * 2` `f32` copied
            // from `orig`, 16-byte aligned; reinterpreting back to
            // `Complex<f32>` is the inverse of the view taken in `new`.
            Some(a) => unsafe {
                core::slice::from_raw_parts(
                    a.as_slice().as_ptr() as *const Complex<f32>,
                    orig.len(),
                )
            },
            None => orig,
        }
    }
}

/// Coherent inner product for one Costas block: returns amplitude |z|.
/// Matches WSJT-X: `abs(sum(cd0 * conjg(csynct))) / nz`
/// (normalization by block length omitted here — comparison is relative).
/// Returns 0.0 if the block's samples fall outside `cd0`.
fn score_flat_coherent(cd0: &[Complex<f32>], flat_ref: &FlatRef, cd0_start: i32) -> f32 {
    let np = cd0.len() as i32;
    let len = flat_ref.len() as i32;
    if cd0_start < 0 || cd0_start + len > np {
        return 0.0;
    }
    let s0 = cd0_start as usize;

    // Odd `s0` puts the `f32` view at an 8-mod-16 address, which costs
    // the backend its PIE path. Start one complex sample earlier —
    // aligned — against the zero-led reference instead. Needs one more
    // complex sample in range than the plain path, so the last legal
    // window still takes the plain one.
    // How many complex samples the zero-led reference spans: its
    // padded `f32` length halved. Stated from the buffer rather than
    // as `n + 1`, because `AlignedF32` rounds up to a multiple of four
    // and how much it adds depends on the parity of `n` — which is
    // `ds_spb`-dependent and so differs between FT4 and FST4.
    #[cfg(feature = "dotprod-extern")]
    let odd_span = (flat_ref.plain_odd.as_slice().len() / 2) as i32;
    #[cfg(feature = "dotprod-extern")]
    if s0 % 2 == 1 && cd0_start - 1 + odd_span <= np {
        // The *padded* length, not `n * 2 + 2`. A length that is not a
        // multiple of four fails the backend's other precondition, and
        // slicing back to 258 is exactly what the first flash of this
        // did: 5 184 calls still on the scalar path. The tail of the
        // reference is zero, and `odd_span` above is what puts the
        // extra `cd0` samples in range.
        //
        // SAFETY: as the plain branch below, starting one sample
        // earlier; the `odd_span` test establishes the length.
        let pad = flat_ref.plain_odd.as_slice().len();
        let c: &[f32] =
            unsafe { core::slice::from_raw_parts(cd0[s0 - 1..].as_ptr() as *const f32, pad) };
        let zr = dot_f32(c, flat_ref.plain_odd.as_slice());
        let zi = dot_f32(c, flat_ref.swapped_odd.as_slice());
        return (zr * zr + zi * zi).sqrt();
    }

    // SAFETY: `Complex<f32>` is `#[repr(C)]` over two `f32`, so a
    // slice of them is exactly the interleaved layout `FlatRef` was
    // built to match, with the same alignment. The bounds check above
    // establishes the length.
    let c: &[f32] =
        unsafe { core::slice::from_raw_parts(cd0[s0..].as_ptr() as *const f32, flat_ref.n * 2) };
    let zr = dot_f32(c, &flat_ref.plain.as_slice()[..flat_ref.n * 2]);
    let zi = dot_f32(c, &flat_ref.swapped.as_slice()[..flat_ref.n * 2]);
    (zr * zr + zi * zi).sqrt()
}

/// FST4-specific sync: faithful port of WSJT-X `fst4_sync_search`
/// (`fst4_decode.f90:879-925`), with the coherent amplitude scorer matching
/// `sync_fst4` (`fst4_decode.f90:657`, `nsyncoh=8`).
///
/// **Scorer**: phase-continuous FSK reference, one inner product per Costas
/// block (8×ds_spb samples), return amplitude |z| and sum across 5 blocks.
/// This is `~3 dB` better SNR discrimination than the per-tone power-sum
/// (`score_costas_block`) at near-threshold SNR.
///
/// **Coarse pass**: ±12 × 0.1·baud Hz × ±1.5 s time range (step 4).
/// Pre-twiddled once per freq offset to avoid per-cell re-allocation.
///
/// **Fine pass**: ±7 × 0.02·baud Hz × ±4 samples around coarse winner,
/// step 1.  `sbest` reset to 0.0 before fine pass (WSJT-X convention).
pub fn fst4_sync_search<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
) -> Sync2dResult {
    // Only `d.ds_spb`/`d.ds_rate` are read below — governed by
    // `downsample_cached`'s own rate, not `SyncDims::of`'s
    // `sample_rate_hz` parameter (see that doc comment), so the
    // argument here is inert.
    // Aligned once per call, before anything reads it — see
    // [`AlignedCd0`]. Copies only when the caller's buffer is not
    // already 16-byte aligned.
    let aligned = AlignedCd0::new(cd0);
    let cd0 = aligned.get(cd0);

    let d = SyncDims::of::<P>(12_000.0);
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    let baud = P::TONE_SPACING_HZ;
    let init_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;

    // WSJT-X: ishw = 1.5 * floor(fs2) samples.
    let ishw = (1.5 * ds_rate as f64).floor() as i32;
    // v3.3.0-beta1 `last_start=nfft2-160*nss`: the last start at which a whole frame is still
    // inside the baseband; both passes stop there (`min(is0+ishw,last_start)`). rc1 let a late
    // start run on, scoring the blocks past the end as 0, so a partial score could win.
    let last_start = cd0.len() as i32 - P::N_SYMBOLS as i32 * ds_spb as i32;

    // Pre-build flat phase-continuous references for each Costas block.
    // (start_sample_offset, flat_ref)
    let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * ds_spb as i32;
            let flat = make_costas_ref_continuous(b.pattern, ds_spb);
            (off, flat)
        })
        .collect();

    // Scratch for the twiddled references — allocated once here and
    // refilled per frequency offset; see `FlatRef::with_len`.
    let mut twiddled: Vec<(i32, FlatRef)> = flat_blocks
        .iter()
        .map(|(off, flat)| (*off, FlatRef::with_len(flat.len())))
        .collect();

    // Helper: score all 5 blocks with pre-twiddled refs at given i0.
    let score_flat = |twiddled: &Vec<(i32, FlatRef)>, i0: i32| -> f32 {
        twiddled
            .iter()
            .map(|(off, flat)| score_flat_coherent(cd0, flat, i0 + off))
            .sum::<f32>()
    };

    let retwiddle = |twiddled: &mut Vec<(i32, FlatRef)>, df: f32| {
        for ((_, dst), (_, src)) in twiddled.iter_mut().zip(flat_blocks.iter()) {
            dst.fill(src, df, ds_rate);
        }
    };

    // Coarse pass: sweep ±ishw time × ±12×0.1·baud freq.
    let mut best_df = 0.0f32;
    let mut best_i0 = init_i0;
    let mut best_score = f32::NEG_INFINITY;

    for si in -12i32..=12 {
        let df = si as f32 * 0.1 * baud;
        retwiddle(&mut twiddled, df);

        let mut di = -ishw;
        while di <= ishw {
            let i0 = init_i0 + di;
            if i0 > last_start {
                break;
            }
            let s = score_flat(&twiddled, i0);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += 4;
        }
    }

    // No start fits (`if(min(is0+ishw,last_start)<max(1,is0-ishw)) return` with `isbest=-1`).
    if best_score == f32::NEG_INFINITY {
        return Sync2dResult {
            freq_hz: candidate.freq_hz,
            i0: init_i0,
            score: f32::NEG_INFINITY,
        };
    }

    // Fine pass: ±7×0.02·baud Hz × ±4 samples.  WSJT-X resets sbest=0.0.
    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;
    best_score = 0.0;

    for si in -7i32..=7 {
        let df = coarse_winner_df + si as f32 * 0.02 * baud;
        retwiddle(&mut twiddled, df);

        for di in -4i32..=4 {
            let i0 = coarse_winner_i0 + di;
            if i0 > last_start {
                break;
            }
            let s = score_flat(&twiddled, i0);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
        }
    }

    Sync2dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

/// FT4-specific sync: coherent full-slot Δt search, faithful port of
/// WSJT-X `ft4_decode.f90`'s `isync=1`/`isync=2` loop (`sync4d.f90` scorer)
/// — added 2026-07-18 after a diagnostic
/// (`tests/ft4_coherent_wide_search_diag.rs`) confirmed the hypothesis:
/// `engine::sync::coarse_sync`'s non-coherent (power-spectrogram) Δt
/// estimate can be wrong by more than a second under CCIR fading, and
/// the previous local `sync2d_refine` (`Sync2dConfig::for_ft4`, ±20
/// downsampled samples ≈ ±30 ms) could never recover from an error that
/// large — even though the true peak's *coherent* score was consistently
/// higher than whatever the non-coherent stage picked instead.
///
/// **Scorer**: one coherent dot product per FT4 Costas block (4 blocks:
/// symbols 0, 33, 66, 99), magnitude-summed across blocks — matches
/// `sync4d.f90`'s `sync = p(z1)+p(z2)+p(z3)+p(z4)` (`p(z)=|z*fac|`,
/// magnitude not power) where each `z_k` is itself ONE coherent dot
/// product spanning all 4 symbols of block k
/// (`z1=sum(cd0(i1:i1+4*NSS-1:2)*conjg(csync2))`, `sync4d.f90:64`) — i.e.
/// coherent *within* each block, magnitude-summed (non-coherent) *across*
/// the 4 blocks, the same combining style [`fst4_sync_search`] already
/// uses. [`ft4_sync_search_window`] twiddles each candidate `(Δf, Δt)`
/// cell's dot product inline rather than calling `score_flat_coherent`
/// on a pre-twiddled reference (as an earlier revision did) — same
/// arithmetic, but avoids re-allocating a twiddled `Vec<Complex<f32>>`
/// per block on every one of the ~19,900 grid cells the coarse+fine
/// passes evaluate. **Originally shipped using `score_costas_block`**
/// (per-symbol power-sum, correct for FT8's `sync8d.f90` — verified
/// against `/home/minoru/src/WSJT-X/lib/ft8/sync8d.f90`, which really
/// does non-coherent per-symbol power summing) — a ~3 dB-class
/// discrimination gap at near-threshold SNR (same mechanism as the
/// FST4 fix, issue #146), caught during the issue #72 AWGN-gap
/// diagnostic (`docs/notes/FT4_BENCHMARK.md` section 9) by reading
/// `sync4d.f90`'s inner `z1=sum(...)` line rather than stopping at the
/// outer `sync=p(z1)+...` formula that (correctly) matched at a glance.
///
/// **Coarse pass**: ±12 Hz / 3 Hz step (`ft4_decode.f90` isync=1:
/// `idfmin=-12,idfmax=12,idfstp=3`) × a *fixed absolute* Δt window,
/// step 4 downsampled samples (`ibstp=4`). The absolute window
/// `[-344, 1012]` downsampled samples is WSJT-X's combined 3-segment
/// coverage (`iseg=1..3`, `ibmin`/`ibmax` per segment) collapsed into
/// one pass — deliberately centred on the *nominal* frame position
/// (`i0` for `dt_sec=0`), not on `candidate.dt_sec`, since that
/// non-coherent estimate is exactly what this function exists to
/// override.
///
/// **Fine pass**: ±4 Hz / 1 Hz × ±5 samples step 1 around the coarse
/// winner (`ft4_decode.f90` isync=2).
pub fn ft4_sync_search<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
) -> Sync2dResult {
    // WSJT-X `ft4_decode.f90`: iseg=1 ibmin=108/ibmax=560, iseg=2
    // ibmin=560/ibmax=1012, iseg=3 ibmin=-344/ibmax=108 — union is
    // [-344, 1012], an absolute downsampled-sample range independent of
    // any candidate dt guess. Collapsed into one pass here (see module
    // doc above `ft4_sync_search`) rather than WSJT-X's literal 3-segment
    // loop with a per-segment decode attempt — [`ft4_sync_search_window`]
    // exposes the windowed search directly for diagnosing whether that
    // collapse loses anything (issue #72, `FT4_BENCHMARK.md` section 11).
    ft4_sync_search_window::<P>(cd0, candidate, -344, 1012)
}

/// The Δt search's coarse-pass carrier phasors, built once and read by
/// every candidate in a slot.
///
/// ## What this is, and what it is not
///
/// [`ft4_sync_search_window`] spends 30 % of itself in `FlatRef::fill`
/// — 18 calls per candidate, each over four Costas blocks with a
/// `cos`/`sin` **per sample**. Measured on a CoreS3 (2026-09-02):
/// 25.7 ms of the call's 86.7 (`docs/notes/FT4_BENCHMARK.md` §47).
///
/// The first attempt at removing it cached the *filled references* —
/// nine `df` x four blocks of `FlatRef`, ~144 KB. That made the whole
/// search **2.6x slower**, and the diagnosis is the interesting part:
/// the dots stayed on the PIE path (23 724 fast, 0 slow) and the
/// buffers straddled internal DRAM and PSRAM, but the *uncached* path
/// in the same binary slowed down identically — 223 ms against 87 —
/// because 144 KB of cache displaced the per-call `FlatRef`s
/// (~1 KB each, small enough that `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`
/// keeps them in internal DRAM) into PSRAM. Same lesson §29 and §32
/// record from the other direction: this search runs at its measured
/// speed only while its working set is internal.
///
/// So what is cached here is the part that is expensive to compute and
/// cheap to hold: the phasor `e^{j·2π·df·n/ds_rate}`, `n` over one
/// Costas block. All four blocks are the same length, so one table per
/// `df` serves all of them — **128 complex samples x 9 `df` ≈ 9 KB**,
/// against 144 KB for the references they build.
///
/// The fills still happen, still write into the caller's own scratch,
/// and still produce the same bytes; only the transcendentals are
/// gone.
pub struct Ft4CoarsePhasors {
    /// One table per coarse `df`, in sweep order.
    tables: Vec<(f32, Vec<Complex<f32>>)>,
}

use super::protocol::SyncPhasors;

impl SyncPhasors for Ft4CoarsePhasors {
    fn build(ds_rate: f32, n: usize) -> Self {
        Self::build_for(ds_rate, n)
    }

    /// Exact `f32` comparison, deliberately: the sweep stores
    /// `idf as f32` for integer `idf` and the fine pass asks with
    /// `coarse_winner_df + si as f32`, a sum of two exactly
    /// representable integers. A `df` that misses is built instead,
    /// which is slower and not wrong.
    fn table_for(&self, df: f32) -> Option<&[Complex<f32>]> {
        self.tables
            .iter()
            .find(|(d, _)| *d == df)
            .map(|(_, t)| t.as_slice())
    }
}

impl Ft4CoarsePhasors {
    /// Build the nine coarse-sweep phasor tables for `P`.
    pub fn new<P: SyncFrontEnd>() -> Self {
        let d = SyncDims::of::<P>(12_000.0);
        // Every Costas block is `nsym · ds_spb` samples and they are
        // all the same length, so one table covers all four.
        let n = P::SYNC_MODE
            .blocks()
            .first()
            .map(|b| b.pattern.len() * d.ds_spb)
            .unwrap_or(0);
        Self::build_for(d.ds_rate, n)
    }

    /// A set that holds nothing, so every lookup misses and the
    /// search builds each `df` on demand.
    ///
    /// The path `()` gives every other protocol, reachable for `Ft4`
    /// — which is what lets the bit-identity test compare the two and
    /// `ft4-bench` keep an uncached arm now that the tables are a
    /// protocol's associated type rather than an `Option` argument.
    pub fn empty() -> Self {
        Self { tables: Vec::new() }
    }

    fn build_for(ds_rate: f32, n: usize) -> Self {
        let mut tables = Vec::new();
        let mut idf = COARSE_DF_MIN;
        while idf <= COARSE_DF_MAX {
            let df = idf as f32;
            let omega = 2.0 * PI * df / ds_rate;
            // Exactly `fill`'s own expression, so the products match.
            let t = (0..n)
                .map(|k| {
                    let p = omega * k as f32;
                    Complex::new(p.cos(), p.sin())
                })
                .collect();
            tables.push((df, t));
            idf += COARSE_DF_STEP;
        }
        Self { tables }
    }

    /// Where the tables live (`internal-testing`) — the question the
    /// 144 KB version got wrong. S3 internal DRAM is
    /// `0x3FC8_0000..0x3FD0_0000`, PSRAM `0x3C00_0000..0x3E00_0000`.
    #[cfg(feature = "internal-testing")]
    pub fn buffer_addrs(&self) -> (usize, usize) {
        (
            self.tables[0].1.as_ptr() as usize,
            self.tables[self.tables.len() - 1].1.as_ptr() as usize,
        )
    }
}

/// The coarse `df` sweep, as `ft4_decode.f90` runs it: -12..=12 Hz in
/// steps of 3. Named because [`Ft4CoarsePhasors`] has to visit exactly
/// the same values in the same order.
/// Δt step of the coarse sweep, in `cd0` samples. Also the bin width
/// `ft4_sync_search_window_binned` uses, which is what makes its bins
/// line up with the grid for free.
const COARSE_DT_STEP: i32 = 4;

const COARSE_DF_MIN: i32 = -12;
const COARSE_DF_MAX: i32 = 12;
const COARSE_DF_STEP: i32 = 3;

/// Time the Δt search's window-independent half, in two pieces
/// (`internal-testing`).
///
/// `ft4_sync_search_window` costs `fixed + cells x per_cell`, and a
/// degenerate window measures `fixed` without saying what is in it
/// (`docs/notes/FT4_BENCHMARK.md` §47). These two entry points run
/// exactly the pieces that function runs, so a bench can time them
/// without this module exposing `FlatRef` or `AlignedCd0` themselves.
///
/// Returns a value derived from the result so the work cannot be
/// optimised away.
#[cfg(feature = "internal-testing")]
pub fn ft4_sync_ref_prep_bench<P: SyncFrontEnd>(fills: usize) -> f32 {
    let d = SyncDims::of::<P>(12_000.0);
    let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * d.ds_spb as i32;
            (off, cached_costas_ref_continuous(b.pattern, d.ds_spb))
        })
        .collect();
    let mut twiddled: Vec<(i32, FlatRef)> = flat_blocks
        .iter()
        .map(|(off, flat)| (*off, FlatRef::with_len(flat.len())))
        .collect();
    let mut sink = 0.0f32;
    for k in 0..fills {
        // The same df values the coarse pass sweeps: -12..=12 step 3.
        let df = (-12 + 3 * (k % 9) as i32) as f32;
        for ((_, dst), (_, src)) in twiddled.iter_mut().zip(flat_blocks.iter()) {
            dst.fill(src, df, d.ds_rate);
        }
        sink += twiddled[0].1.len() as f32;
    }
    sink
}

/// The other piece: `cd0`'s alignment copy, once per candidate
/// (`internal-testing`). See [`ft4_sync_ref_prep_bench`].
#[cfg(feature = "internal-testing")]
pub fn ft4_aligned_cd0_bench(cd0: &[Complex<f32>]) -> f32 {
    let aligned = AlignedCd0::new(cd0);
    let s = aligned.get(cd0);
    s[0].re + s[s.len() - 1].im
}

/// Same coherent full-slot Δt search as [`ft4_sync_search`], but over an
/// explicit `[ib_min, ib_max]` downsampled-sample window instead of the
/// hardcoded full-union range. Lets callers (tests, diagnostics) replicate
/// WSJT-X's literal per-segment search — `ft4_decode.f90`'s `iseg=1,2,3`
/// loop, each with its own `ibmin`/`ibmax` — to check whether the
/// collapsed single-pass search in [`ft4_sync_search`] ever misses a
/// position that a per-segment search plus a per-segment decode attempt
/// would have found.
pub fn ft4_sync_search_window<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
) -> Sync2dResult {
    let d = SyncDims::of::<P>(12_000.0);
    let n = P::SYNC_MODE
        .blocks()
        .first()
        .map(|b| b.pattern.len() * d.ds_spb)
        .unwrap_or(0);
    let refs = P::SyncPhasors::build(d.ds_rate, n);
    ft4_sync_search_window_with::<P>(cd0, candidate, ib_min, ib_max, &refs)
}

/// Fill every Costas block's carrier-shifted reference for one `df`.
///
/// One table per `df`, then four `fill_with` from it: `fill`
/// evaluates the phasor per sample and every block indexes it from
/// zero, so filling four blocks used to evaluate the same 128
/// `cos`/`sin` pairs four times.
///
/// At module scope because both FT4 coarse passes and the shared
/// fine pass call it.
fn twiddle_all<S: SyncPhasors>(
    twiddled: &mut [(i32, FlatRef)],
    flat_blocks: &[(i32, Vec<Complex<f32>>)],
    scratch: &mut [Complex<f32>],
    refs: &S,
    df: f32,
    ds_rate: f32,
) {
    // `fill_with` leaves the reference alone at `df == 0` — the
    // same branch `fill` has — so that case needs no table.
    let table: &[Complex<f32>] = if df.abs() < f32::EPSILON {
        &[]
    } else if let Some(t) = refs.table_for(df) {
        t
    } else {
        // Exactly `fill`'s expression, which is what keeps this
        // bit-identical to evaluating it inside the block loop.
        let omega = 2.0 * PI * df / ds_rate;
        for (k, slot) in scratch.iter_mut().enumerate() {
            let p = omega * k as f32;
            *slot = Complex::new(p.cos(), p.sin());
        }
        &scratch[..]
    };
    for ((_, dst), (_, src)) in twiddled.iter_mut().zip(flat_blocks.iter()) {
        dst.fill_with(src, df, table);
    }
}

/// [`ft4_sync_search_window`] against phasor tables the caller keeps
/// across candidates.
///
/// **Both passes read them.** This took `Option<&Ft4CoarsePhasors>`
/// until 2026-09-21, so the coarse sweep read a table and the fine
/// pass rebuilt its nine `df` from `cos`/`sin` — four times each, once
/// per Costas block, because `fill` evaluates the phasor per sample
/// and every block indexes it from zero.
///
/// Now the tables are [`crate::engine::Protocol::SyncPhasors`], both passes ask the
/// same way, and a `df` the set does not hold is built once into a
/// scratch rather than four times into the blocks. `()` — every
/// protocol but FT4 — holds nothing and takes that path for every
/// `df`, which is what they all did before.
pub fn ft4_sync_search_window_with<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
    refs: &P::SyncPhasors,
) -> Sync2dResult {
    // See [`AlignedCd0`]; same reasoning as `fst4_sync_search`.
    let aligned = AlignedCd0::new(cd0);
    let cd0 = aligned.get(cd0);

    // Only `d.ds_spb`/`d.ds_rate` are read below — see
    // `fst4_sync_search`'s identical comment.
    let d = SyncDims::of::<P>(12_000.0);
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;

    // Pre-built phase-continuous references, one per Costas block.
    let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
        .blocks()
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * ds_spb as i32;
            (off, cached_costas_ref_continuous(b.pattern, ds_spb))
        })
        .collect();

    // Scratch for the carrier-shifted references, allocated once and
    // refilled per `df` — the same `FlatRef` machinery
    // `fst4_sync_search` uses, adopted here 2026-08-29 after the first
    // FT4 hardware measurement put this function at 76% of a slot
    // budget it overran 8.8x (`docs/notes/FT4_BENCHMARK.md` §17).
    //
    // What changed, and why it is the same arithmetic: this loop used
    // to apply the frequency shift to `cd0` *inside* the innermost
    // sample loop, as a rotating phasor (`twid *= step`) restarted at
    // every `(df, i0)` cell. But `twid[n] = step^n` is indexed by the
    // offset **within the block**, not by `i0` — so it is identical
    // across the ~340 `i0` positions each `df` sweeps, and rebuilding
    // it per cell was that many times redundant. Folding it into the
    // reference instead (`FlatRef::fill`) hoists it out of the `i0`
    // loop entirely and leaves a plain complex inner product, which is
    // what `dot_f32` — and therefore `dotprod-extern`'s
    // `dsps_dotprod_f32_aes3` on LX7 — can serve. This function's own
    // earlier comment already recorded the identity ("the same dot
    // product as twiddling each sample of `cd0` in place"); FST4 was
    // simply on the right side of it and FT4 was not.
    //
    // **Not bit-identical**, deliberately: the products reassociate
    // (`c*conj(r)*twid` becomes `c*conj(r*e^{jp})`), and `fill`
    // evaluates the phasor per sample instead of accumulating a
    // recurrence, so it carries *less* rounding error, not more.
    // Verified against the WSJT-X golden and the full AWGN/CCIR sweep
    // before landing — see `docs/notes/FT4_BENCHMARK.md` §19.
    let mut twiddled: Vec<(i32, FlatRef)> = flat_blocks
        .iter()
        .map(|(off, flat)| (*off, FlatRef::with_len(flat.len())))
        .collect();

    // `FlatRef::fill` applies `e^{+j.2pi.df.n/ds_rate}` to the reference
    // and `score_flat_coherent` conjugates it, giving
    // `sum c[n].conj(r[n]).e^{-j.2pi.df.n/ds_rate}` — exactly the sign
    // convention the replaced `phasor_for` used
    // (`omega = -2pi.df/ds_rate`).
    // **One table per `df`, then four `fill_with` from it.**
    //
    // `fill` evaluates the phasor per sample and every Costas block
    // indexes it from zero, so filling four blocks evaluated the same
    // 128 `cos`/`sin` pairs four times. The coarse pass stopped doing
    // that when `Ft4CoarsePhasors` arrived; the fine pass could not,
    // because it asks for `df` the coarse sweep never visits.
    // **On the stack, not the heap.** The first cut allocated a `Vec`
    // per call, which is per candidate, on both cores. Measured on the
    // board: one core went 1 739 -> 1 622 ms over twelve candidates,
    // and two cores went 1 167 -> 1 178 — the serial work fell and the
    // scaling fell further, 1.48x to 1.37x. Two cores taking the IDF
    // heap lock once a candidate is the shared resource that pattern
    // points at, and it is the same lesson §47 records: in this
    // function, what the working set does matters more than what the
    // arithmetic does.
    //
    // A Costas block is `pattern.len() * ds_spb` samples — 128 for
    // FT4, which is the only protocol that reaches here. The array is
    // sized for that with room, and a protocol that needs more falls
    // back to a `Vec` rather than being refused.
    const STACK_TABLE: usize = 160;
    let table_n = flat_blocks.iter().map(|(_, b)| b.len()).max().unwrap_or(0);
    let mut stack_scratch = [Complex::new(0.0f32, 0.0f32); STACK_TABLE];
    let mut heap_scratch: Vec<Complex<f32>> = if table_n > STACK_TABLE {
        alloc::vec![Complex::new(0.0, 0.0); table_n]
    } else {
        Vec::new()
    };
    let scratch: &mut [Complex<f32>] = if table_n > STACK_TABLE {
        &mut heap_scratch[..]
    } else {
        &mut stack_scratch[..table_n]
    };

    let mut best_df = 0.0f32;
    let mut best_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;
    let mut best_score = f32::NEG_INFINITY;

    // The coarse sweep, from the cache when there is one. The two
    // arms visit the same `df` values in the same order and score with
    // the same references, so they agree bit for bit; only the fills
    // are skipped.
    let scan_coarse = |twiddled: &Vec<(i32, FlatRef)>,
                       df: f32,
                       best_score: &mut f32,
                       best_df: &mut f32,
                       best_i0: &mut i32| {
        let mut i0 = ib_min;
        while i0 <= ib_max {
            let s = twiddled
                .iter()
                .map(|(off, flat)| score_flat_coherent(cd0, flat, i0 + off))
                .sum::<f32>();
            if s > *best_score {
                *best_score = s;
                *best_df = df;
                *best_i0 = i0;
            }
            i0 += COARSE_DT_STEP;
        }
    };
    // One loop whether or not the protocol precomputed anything. The
    // cached arm used to iterate `cache.tables` instead — the same
    // values in the same order, which is exactly what
    // `cached_coarse_refs_are_bit_identical` had to assert and is now
    // structural.
    let mut idf = COARSE_DF_MIN;
    while idf <= COARSE_DF_MAX {
        let df = idf as f32;
        twiddle_all(&mut twiddled, &flat_blocks, scratch, refs, df, ds_rate);
        scan_coarse(&twiddled, df, &mut best_score, &mut best_df, &mut best_i0);
        idf += COARSE_DF_STEP;
    }

    // Fine pass around the coarse winner.
    let (best_score, best_df, best_i0) = ft4_fine_pass::<P>(
        cd0,
        &flat_blocks,
        &mut twiddled,
        scratch,
        refs,
        ds_rate,
        best_df,
        best_i0,
    );

    Sync2dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

/// The references and scratch one worker needs to run
/// [`Ft4CoarseSweep`]s: the four Costas blocks, and four carrier-shifted
/// copies refilled per `df`.
///
/// Separate from the sweep because a receiver holds one sweep per
/// candidate at once and a worker only ever scores one of them at a
/// time: the twiddled copies are ~16 KB of small allocations, and
/// twelve sets of them would sit in internal DRAM on an ESP32-S3.
pub struct Ft4SweepScratch {
    flat_blocks: Vec<(i32, Vec<Complex<f32>>)>,
    twiddled: Vec<(i32, FlatRef)>,
    table: Vec<Complex<f32>>,
    ds_rate: f32,
}

impl Ft4SweepScratch {
    pub fn new<P: SyncFrontEnd>() -> Self {
        Self::new_with_min_alloc::<P>(0)
    }

    /// [`Self::new`] with every buffer at least `min_alloc_bytes`.
    ///
    /// On the host mirror one scratch is ~10 KB of allocations under
    /// the CoreS3's 2 048-byte internal-DRAM threshold, and a receiver
    /// running the sweep during capture holds one per worker — beside
    /// WiFi, whose floor at the slot boundary is ~16 KB. Past the
    /// threshold they go to PSRAM instead; ~9 KB a worker, read in a
    /// tight loop, which the data cache holds.
    pub fn new_with_min_alloc<P: SyncFrontEnd>(min_alloc_bytes: usize) -> Self {
        let d = SyncDims::of::<P>(12_000.0);
        let min_elems = min_alloc_bytes.div_ceil(core::mem::size_of::<Complex<f32>>());
        let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = P::SYNC_MODE
            .blocks()
            .iter()
            .map(|b| {
                let off = b.start_symbol as i32 * d.ds_spb as i32;
                let src = cached_costas_ref_continuous(b.pattern, d.ds_spb);
                let mut v = Vec::with_capacity(src.len().max(min_elems));
                v.extend_from_slice(&src);
                (off, v)
            })
            .collect();
        let twiddled = flat_blocks
            .iter()
            .map(|(off, flat)| {
                (
                    *off,
                    FlatRef::with_len_min_alloc(flat.len(), min_alloc_bytes),
                )
            })
            .collect();
        let n = flat_blocks.iter().map(|(_, b)| b.len()).max().unwrap_or(0);
        let mut table = Vec::with_capacity(n.max(min_elems));
        table.resize(n, Complex::new(0.0, 0.0));
        Self {
            flat_blocks,
            twiddled,
            table,
            ds_rate: d.ds_rate,
        }
    }
}

/// [`ft4_sync_search_window_with`]'s coarse sweep, run **while `cd0` is
/// still being built**, then its fine pass at the end.
///
/// A cell's coarse score is the sum of four Costas-block correlations,
/// and block `k` of cell `i0` reads `cd0[i0 + off_k ..][..128]` and
/// nothing else — so it can be scored the moment those samples exist.
/// [`advance`](Self::advance) scores every block that has become
/// readable since the last call, in the order the one-shot sweep adds
/// them (A, B, C, D per cell), so each cell's partial sum is formed in
/// the same order. On a 5.04 s frame the last block of the latest cell
/// the ±1.0 s window reaches ends ~0.2 s before the capture closes, so
/// the whole sweep can be done before the slot ends.
///
/// **One difference from the one-shot search, and where it can show.**
/// That search runs on the RMS-normalised `cd0`, and the normalisation
/// needs the whole slot. This sweep scores the raw one. A cell's score
/// is `|Σ cd0·conj(ref)|`, linear in the scale, so the ranking is the
/// same except where two cells tie to within rounding; the fine pass,
/// which produces the score and position the decoder uses, runs on the
/// normalised `cd0` exactly as before
/// (`ft4_incremental_sweep_matches_the_one_shot_search`).
///
/// [`complete`](Self::complete) then [`fine`](Self::fine) at the close.
pub struct Ft4CoarseSweep {
    ib_min: i32,
    n_i0: usize,
    /// `n_df × n_i0` partial sums, `df`-major — the one-shot sweep's
    /// visiting order.
    partial: Vec<f32>,
    /// Per block, how many `i0` cells (from `ib_min`) have had that
    /// block added. Never ahead of the block before it.
    done: [usize; 4],
    /// The raw `cd0` so far, 16-byte aligned for `dot_f32`'s PIE path.
    cd0: crate::engine::dsp::dotprod::AlignedF32,
    /// Complex samples copied into `cd0`.
    have: usize,
}

const N_COARSE_DF: usize = ((COARSE_DF_MAX - COARSE_DF_MIN) / COARSE_DF_STEP + 1) as usize;

impl Ft4CoarseSweep {
    /// A sweep over `[ib_min, ib_max]` of a `cd0` that will end up
    /// `cd0_len` samples long. Allocations are at least `min_alloc_bytes`
    /// each — see `CandidateDdc::new_half_rate_with_min_alloc`.
    pub fn new(ib_min: i32, ib_max: i32, cd0_len: usize, min_alloc_bytes: usize) -> Self {
        let n_i0 = ((ib_max - ib_min) / COARSE_DT_STEP + 1).max(0) as usize;
        let mut partial = Vec::with_capacity(
            (N_COARSE_DF * n_i0).max(min_alloc_bytes.div_ceil(core::mem::size_of::<f32>())),
        );
        partial.resize(N_COARSE_DF * n_i0, 0.0);
        Self {
            ib_min,
            n_i0,
            partial,
            done: [0; 4],
            cd0: crate::engine::dsp::dotprod::AlignedF32::with_min_alloc(
                cd0_len * 2,
                min_alloc_bytes,
            ),
            have: 0,
        }
    }

    /// Coarse cells scored so far, over all blocks, out of
    /// `4 × n_df × n_i0` — for a receiver to report how much of the
    /// sweep the capture paid for.
    pub fn progress(&self) -> (usize, usize) {
        (
            self.done.iter().sum::<usize>() * N_COARSE_DF,
            4 * N_COARSE_DF * self.n_i0,
        )
    }

    /// Take `cd0`'s samples so far — a prefix of the final baseband; any
    /// samples already taken must be unchanged — and score every block
    /// that has become readable. `max_cells` bounds the block-cells
    /// scored in this call (one block of one cell at one `df` each), so
    /// a caller can stay responsive; `usize::MAX` for no bound. Returns
    /// whether anything readable is still unscored.
    pub fn advance<P: SyncFrontEnd>(
        &mut self,
        cd0: &[Complex<f32>],
        final_len: usize,
        scratch: &mut Ft4SweepScratch,
        refs: &P::SyncPhasors,
        max_cells: usize,
    ) -> bool {
        let cap = self.cd0.as_slice().len() / 2;
        let take = cd0.len().min(cap);
        if take > self.have {
            // SAFETY: `Complex<f32>` is `repr(C)` over two `f32`.
            let src = unsafe {
                core::slice::from_raw_parts(
                    cd0[self.have..take].as_ptr() as *const f32,
                    (take - self.have) * 2,
                )
            };
            self.cd0.as_mut_slice()[self.have * 2..take * 2].copy_from_slice(src);
            self.have = take;
        }
        let have = self.have;
        let mut budget = max_cells;
        for k in 0..scratch.flat_blocks.len().min(4) {
            let (off, ref_len) = (
                scratch.flat_blocks[k].0,
                scratch.flat_blocks[k].1.len() as i32,
            );
            // Cells whose block `k` window is final: it ends inside
            // what has arrived, or starts past where `cd0` will end (so
            // it scores 0 however long we wait). Never past block `k-1`.
            let limit = if k == 0 { self.n_i0 } else { self.done[k - 1] };
            let mut upto = self.done[k];
            while upto < limit {
                let start = self.ib_min + upto as i32 * COARSE_DT_STEP + off;
                // Four samples of margin: `score_flat_coherent`'s
                // odd-offset path needs up to two more than the window
                // and tests them against the slice it is given, so a
                // block at the edge of what has arrived would otherwise
                // take the other path than it takes against the whole
                // `cd0` — making the result depend on how the input
                // was chunked.
                let end = start + ref_len;
                let readable =
                    end + 4 <= have as i32 || have == final_len || end > final_len as i32;
                if !readable {
                    break;
                }
                upto += 1;
            }
            if upto <= self.done[k] {
                continue;
            }
            let n = (upto - self.done[k]).min((budget / N_COARSE_DF).max(1));
            let from = self.done[k];
            let view_len = if have == final_len { final_len } else { have };
            for idf in 0..N_COARSE_DF {
                let df = (COARSE_DF_MIN + idf as i32 * COARSE_DF_STEP) as f32;
                twiddle_one::<P>(scratch, k, refs, df);
                let flat = &scratch.twiddled[k].1;
                // The one-shot sweep's range test is against the final
                // length; a window past `have` but inside `final_len`
                // was excluded by `readable` above.
                // SAFETY: `AlignedF32` is contiguous `f32` and
                // `Complex<f32>` is `repr(C)` over two of them;
                // `view_len <= have` complex samples were copied.
                let cd0v = unsafe {
                    core::slice::from_raw_parts(
                        self.cd0.as_slice().as_ptr() as *const Complex<f32>,
                        view_len,
                    )
                };
                for c in from..from + n {
                    let i0 = self.ib_min + c as i32 * COARSE_DT_STEP;
                    let st = i0 + off;
                    let v = if st + ref_len > final_len as i32 {
                        0.0
                    } else {
                        score_flat_coherent(cd0v, flat, st)
                    };
                    let cell = &mut self.partial[idf * self.n_i0 + c];
                    // `Iterator::sum` over the four blocks, spelled out:
                    // it starts from -0.0 and adds in block order.
                    *cell = if k == 0 { -0.0 + v } else { *cell + v };
                }
            }
            self.done[k] = from + n;
            budget = budget.saturating_sub(n * N_COARSE_DF);
            if budget == 0 {
                break;
            }
        }
        self.done.iter().any(|&d| d < self.n_i0)
    }

    /// Score whatever is left against the whole raw `cd0` — the
    /// baseband including its flushed tail.
    pub fn complete<P: SyncFrontEnd>(
        &mut self,
        cd0_raw: &[Complex<f32>],
        scratch: &mut Ft4SweepScratch,
        refs: &P::SyncPhasors,
    ) {
        self.advance::<P>(cd0_raw, cd0_raw.len(), scratch, refs, usize::MAX);
        debug_assert!(self.done.iter().all(|&d| d == self.n_i0));
    }

    /// Pick the coarse winner in the one-shot sweep's order and run the
    /// fine pass on `cd0_normalised`. Call [`complete`](Self::complete)
    /// first; split from it so a caller can normalise its one `cd0` in
    /// place between the two rather than keep a copy.
    pub fn fine<P: SyncFrontEnd>(
        self,
        cd0_normalised: &[Complex<f32>],
        candidate: &SyncCandidate,
        scratch: &mut Ft4SweepScratch,
        refs: &P::SyncPhasors,
    ) -> Sync2dResult {
        let mut best_score = f32::NEG_INFINITY;
        let mut best_df = 0.0f32;
        let mut best_i0 = 0i32;
        for idf in 0..N_COARSE_DF {
            let df = (COARSE_DF_MIN + idf as i32 * COARSE_DF_STEP) as f32;
            for c in 0..self.n_i0 {
                let s = self.partial[idf * self.n_i0 + c];
                if s > best_score {
                    best_score = s;
                    best_df = df;
                    best_i0 = self.ib_min + c as i32 * COARSE_DT_STEP;
                }
            }
        }
        let aligned = AlignedCd0::new(cd0_normalised);
        let cd0n = aligned.get(cd0_normalised);
        let Ft4SweepScratch {
            flat_blocks,
            twiddled,
            table,
            ds_rate,
        } = scratch;
        let (best_score, best_df, best_i0) = ft4_fine_pass::<P>(
            cd0n,
            flat_blocks,
            twiddled,
            table,
            refs,
            *ds_rate,
            best_df,
            best_i0,
        );
        Sync2dResult {
            freq_hz: candidate.freq_hz + best_df,
            i0: best_i0,
            score: best_score,
        }
    }
}

/// [`twiddle_all`] for one block.
fn twiddle_one<P: SyncFrontEnd>(
    scratch: &mut Ft4SweepScratch,
    k: usize,
    refs: &P::SyncPhasors,
    df: f32,
) {
    let Ft4SweepScratch {
        flat_blocks,
        twiddled,
        table,
        ds_rate,
    } = scratch;
    let n = flat_blocks[k].1.len();
    let t: &[Complex<f32>] = if df.abs() < f32::EPSILON {
        &[]
    } else if let Some(t) = refs.table_for(df) {
        t
    } else {
        let omega = 2.0 * PI * df / *ds_rate;
        for (i, slot) in table[..n].iter_mut().enumerate() {
            let p = omega * i as f32;
            *slot = Complex::new(p.cos(), p.sin());
        }
        &table[..n]
    };
    twiddled[k].1.fill_with(&flat_blocks[k].1, df, t);
}

/// **Experimental**: the coarse sweep over tone-demodulated bins.
///
/// Off by default; `ft4_rx` selects it with `MFSK_FT4_BINNED_SEARCH`.
/// It exists to measure the trade, not as a recommendation.
///
/// ## What it does differently
///
/// [`ft4_sync_search_window_with`] scores a cell by correlating every
/// one of a Costas block's `4 * ds_spb` samples against a reference —
/// 512 complex multiply-adds a cell for FT4, 3 060 cells in the coarse
/// sweep. The reference is a tone times a carrier offset, and the tone
/// part is the fast one, so demodulating by it *first* leaves a
/// residual that barely turns inside four samples:
/// `2*pi * 12 Hz * 4 / 666.667 = 0.45 rad` at the widest `df` the
/// sweep visits, and exactly zero at the winning one.
///
/// So this demodulates `cd0` by each tone on the absolute sample grid,
/// sums the result in fours, and scores from those bins. The coarse
/// grid steps `i0` by `COARSE_DT_STEP` = 4 and every Costas block
/// starts at a multiple of `ds_spb`, so a bin never straddles a cell
/// boundary and the same bins serve every `i0`. A cell becomes
/// `4 blocks * 4 symbols * 8 bins` = **128 complex multiply-adds**
/// plus 16 per-symbol constants, against 512.
///
/// The per-symbol constant is what makes the demodulation legal:
/// writing `n` for the absolute sample and `start` for the block's
/// position, `exp(-j2*pi*t*(n - start - k*ds_spb)/ds_spb)` factors
/// into `exp(-j2*pi*t*n/ds_spb)` — which is the demodulation, free of
/// `start` — times `exp(+j2*pi*t*start/ds_spb)`, which depends only on
/// `start mod ds_spb`, i.e. on `i0 mod ds_spb`, i.e. on one of eight
/// values. `k*ds_spb` drops out because `t*k` is an integer.
///
/// ## What it approximates
///
/// One thing only: the carrier-offset twiddle is applied at each bin's
/// centre rather than per sample. Over four samples at 12 Hz that is
/// +-0.22 rad at the edges, ~0.07 dB of coherent loss at the widest
/// `df` and none at the winner.
///
/// The fine pass is **not** binned — it steps `i0` by one, so nothing
/// lines up — and runs `ft4_fine_pass` exactly as the shipped search
/// does. The refined position this returns is therefore produced by
/// the same code either way; only which cell the fine pass is centred
/// on can differ.
pub fn ft4_sync_search_window_binned<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
    refs: &P::SyncPhasors,
) -> Sync2dResult {
    const BIN: usize = COARSE_DT_STEP as usize;

    let aligned = AlignedCd0::new(cd0);
    let cd0 = aligned.get(cd0);

    let d = SyncDims::of::<P>(12_000.0);
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    let ntones = P::NTONES as usize;
    let blocks = P::SYNC_MODE.blocks();

    // Needed twice: for each symbol's start phase here, and by the
    // fine pass below.
    let flat_blocks: Vec<(i32, Vec<Complex<f32>>)> = blocks
        .iter()
        .map(|b| {
            let off = b.start_symbol as i32 * ds_spb as i32;
            (off, cached_costas_ref_continuous(b.pattern, ds_spb))
        })
        .collect();

    // One binned, tone-demodulated copy of `cd0` per tone. `ntones *
    // cd0.len() / BIN` complex — the same size as `cd0` itself for
    // FT4, since `ntones == BIN`.
    let nbins = cd0.len() / BIN;
    let mut binned: Vec<Complex<f32>> = alloc::vec![Complex::new(0.0f32, 0.0); ntones * nbins];
    for t in 0..ntones {
        // `exp(-j2*pi*t*n/ds_spb)` repeats every `ds_spb` samples.
        let tp: Vec<Complex<f32>> = (0..ds_spb)
            .map(|n| {
                let a = -2.0 * PI * t as f32 * n as f32 / ds_spb as f32;
                Complex::new(a.cos(), a.sin())
            })
            .collect();
        let dst = &mut binned[t * nbins..(t + 1) * nbins];
        for (j, slot) in dst.iter_mut().enumerate() {
            let base = j * BIN;
            let mut acc = Complex::new(0.0f32, 0.0);
            for n in base..base + BIN {
                acc += cd0[n] * tp[n % ds_spb];
            }
            *slot = acc;
        }
    }

    // `exp(+j2*pi*t*p/ds_spb)` for the eight `p = i0 mod ds_spb` the
    // coarse grid can produce.
    let nph = ds_spb / BIN;
    let tf: Vec<Complex<f32>> = (0..ntones)
        .flat_map(|t| {
            (0..nph).map(move |pi| {
                let a = 2.0 * PI * t as f32 * (pi * BIN) as f32 / ds_spb as f32;
                Complex::new(a.cos(), a.sin())
            })
        })
        .collect();

    let nsym = blocks[0].pattern.len();
    let bins_per_sym = ds_spb / BIN;
    let bins_per_block = nsym * bins_per_sym;
    let block_len = (nsym * ds_spb) as i32;
    let np = cd0.len() as i32;

    let mut twb: Vec<Complex<f32>> = alloc::vec![Complex::new(0.0f32, 0.0); bins_per_block];
    let mut best_df = 0.0f32;
    let mut best_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * ds_rate).round() as i32;
    let mut best_score = f32::NEG_INFINITY;

    let mut idf = COARSE_DF_MIN;
    while idf <= COARSE_DF_MAX {
        let df = idf as f32;
        let omega = 2.0 * PI * df / ds_rate;
        for (i, slot) in twb.iter_mut().enumerate() {
            // Conjugated, at the bin's centre.
            let a = -omega * ((i * BIN) as f32 + (BIN as f32 - 1.0) * 0.5);
            *slot = Complex::new(a.cos(), a.sin());
        }
        let mut i0 = ib_min;
        while i0 <= ib_max {
            let p_idx = (i0.rem_euclid(ds_spb as i32) as usize) / BIN;
            let mut total = 0.0f32;
            for (bi, b) in blocks.iter().enumerate() {
                let start = i0 + flat_blocks[bi].0;
                // `score_flat_coherent` scores an out-of-range window
                // as zero; so does this.
                if start < 0 || start + block_len > np {
                    continue;
                }
                let mut acc = Complex::new(0.0f32, 0.0);
                for k in 0..nsym {
                    let t = b.pattern[k] as usize;
                    let c = flat_blocks[bi].1[k * ds_spb].conj() * tf[t * nph + p_idx];
                    let base = (start as usize + k * ds_spb) / BIN;
                    let row = &binned[t * nbins..];
                    let mut sacc = Complex::new(0.0f32, 0.0);
                    for j in 0..bins_per_sym {
                        sacc += row[base + j] * twb[k * bins_per_sym + j];
                    }
                    acc += c * sacc;
                }
                total += acc.norm_sqr();
            }
            if total > best_score {
                best_score = total;
                best_df = df;
                best_i0 = i0;
            }
            i0 += COARSE_DT_STEP;
        }
        idf += COARSE_DF_STEP;
    }

    // Fine pass, exact and shared.
    let mut twiddled: Vec<(i32, FlatRef)> = flat_blocks
        .iter()
        .map(|(off, flat)| (*off, FlatRef::with_len(flat.len())))
        .collect();
    let table_n = flat_blocks.iter().map(|(_, b)| b.len()).max().unwrap_or(0);
    let mut scratch: Vec<Complex<f32>> = alloc::vec![Complex::new(0.0f32, 0.0); table_n];
    let (best_score, best_df, best_i0) = ft4_fine_pass::<P>(
        cd0,
        &flat_blocks,
        &mut twiddled,
        &mut scratch,
        refs,
        ds_rate,
        best_df,
        best_i0,
    );

    Sync2dResult {
        freq_hz: candidate.freq_hz + best_df,
        i0: best_i0,
        score: best_score,
    }
}

/// The exact +-4 Hz / +-5 sample refinement both FT4 coarse passes end
/// with, extracted so the binned one can share it rather than carry a
/// second copy. Bit-identical to what it replaced: same loop bounds,
/// same order, same `score_flat_coherent`.
///
/// Takes the already-built references rather than building its own —
/// `cached_costas_ref_continuous` is uncached on `no_std`, so a second
/// build would be four allocations and four trig sweeps per candidate
/// on the board.
#[allow(clippy::too_many_arguments)]
fn ft4_fine_pass<P: SyncFrontEnd>(
    cd0: &[Complex<f32>],
    flat_blocks: &[(i32, Vec<Complex<f32>>)],
    twiddled: &mut [(i32, FlatRef)],
    scratch: &mut [Complex<f32>],
    refs: &P::SyncPhasors,
    ds_rate: f32,
    coarse_df: f32,
    coarse_i0: i32,
) -> (f32, f32, i32) {
    let mut best_score = f32::NEG_INFINITY;
    let mut best_df = coarse_df;
    let mut best_i0 = coarse_i0;
    for si in -4i32..=4 {
        let df = coarse_df + si as f32;
        twiddle_all(twiddled, flat_blocks, scratch, refs, df, ds_rate);
        for di in -5i32..=5 {
            let i0 = coarse_i0 + di;
            let s = twiddled
                .iter()
                .map(|(off, flat)| score_flat_coherent(cd0, flat, i0 + off))
                .sum::<f32>();
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
        }
    }
    (best_score, best_df, best_i0)
}

/// Apply a complex-phasor freq shift to `cd0`. Used by callers that
/// take the [`Sync2dResult::freq_hz`] from this module and want to
/// run [`crate::engine::llr::symbol_spectra`] on a baseband whose
/// carrier sits at the refined freq.
pub fn freq_shift_cd0(cd0: &[Complex<f32>], df_hz: f32, ds_rate: f32) -> Vec<Complex<f32>> {
    let mut out = Vec::new();
    freq_shift_cd0_into(cd0, df_hz, ds_rate, &mut out);
    out
}

/// [`freq_shift_cd0`] into a buffer the caller keeps.
///
/// **The allocation is 18 % of the call.** Measured on a CoreS3
/// (2026-09-21, `ft4-bench`'s `llr_bp_probe`): the whole call is
/// 13 629 µs per FT4 candidate, and the same arithmetic written into a
/// buffer that already exists is 11 181 µs — so the fresh 40 KB `Vec`
/// costs **2 448 µs a candidate**, ~29 ms a slot at twelve.
///
/// That is worth a second entry point on its own, but it is also the
/// shape the rest of the work needs: a 40 KB allocation per candidate
/// is exactly the heap traffic §29/§32/§47 of `FT4_BENCHMARK.md` all
/// turn on, where one extra internal-DRAM block moved elsewhere took
/// the search from 100 % PIE to 0 % with no change to the code that
/// ran.
///
/// `out` is resized to `cd0.len()`; its previous contents are not read.
/// [`freq_shift_cd0`] is a wrapper over this.
pub fn freq_shift_cd0_into(
    cd0: &[Complex<f32>],
    df_hz: f32,
    ds_rate: f32,
    out: &mut Vec<Complex<f32>>,
) {
    out.clear();
    out.reserve(cd0.len());
    if df_hz.abs() < f32::EPSILON {
        out.extend_from_slice(cd0);
        return;
    }
    // **The crate's mixer, not a third one.** This used to evaluate
    // `cos` and `sin` per sample — 11 175 µs for FT4's 5 120-sample
    // `cd0`, 2.18 µs a sample, measured on a CoreS3 2026-09-21. Every
    // other place in this crate that multiplies a signal by a complex
    // exponential had already stopped doing that: `wspr::ddc` uses a
    // period-8 table (its centre is exactly `Fs/8`), and
    // `engine::dsp::ddc::Mixer` is the general form, a rotating-phasor
    // NCO renormalised every 4 096 samples. `ft4::ddc` mixes with it
    // over 40 609 samples per candidate, and `fst4::ddc`'s refine
    // stage uses `mix_complex` for exactly this operation.
    //
    // This function was the one that never adopted it, because it is
    // the only rotation in the crate that belongs to no DSP module —
    // it sits in the generic pipeline's refine step, which FT8 bypasses
    // (own engine) and WSPR never reaches (own decoder), so the three
    // optimisation campaigns that built the other mixers each stopped
    // at their own module boundary.
    //
    // Same transform and the same sign convention: `Mixer::new`'s
    // `dphi = -2π·center/Fs` is this function's own `omega`, `cur`
    // starts at `1+0j`, and `mix_complex` emits before advancing, so
    // sample 0 is unrotated in both.
    let mut mixer = super::dsp::ddc::Mixer::new(df_hz, ds_rate);
    out.extend(cd0.iter().map(|&c| {
        let (re, im) = mixer.mix_complex(c.re, c.im);
        Complex::new(re, im)
    }));
}

#[cfg(test)]
mod ft4_coarse_ref_cache_tests {
    use super::*;
    use crate::ft4::Ft4;

    /// A candidate-shaped `cd0`: a Costas-flavoured tone at the
    /// baseband rate with a little structure, enough that the search
    /// has a peak to find and enough noise that the argmax is not
    /// trivially degenerate.
    fn synthetic_cd0(n: usize) -> Vec<Complex<f32>> {
        let mut state = 0x1234_5678u32;
        (0..n)
            .map(|k| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = (state >> 16) as i16 as f32 / 32_768.0;
                let p = 0.031 * k as f32;
                Complex::new(p.cos() + 0.3 * noise, p.sin() - 0.3 * noise)
            })
            .collect()
    }

    /// The tabled search must not merely agree with the per-`df` one —
    /// it must be the same number.
    ///
    /// **Both passes, not just the coarse one.** This used to compare
    /// `ft4_sync_search_window` (no tables at all) against
    /// `..._cached` (tables for the coarse sweep), which left the fine
    /// pass out of the claim entirely — it rebuilt its nine `df` from
    /// `cos`/`sin` on both sides, so any difference there was
    /// invisible. Now the tables are `Ft4::SyncPhasors` and the fine
    /// pass reads them too, so the two arms are "holds nothing" against
    /// "holds the coarse nine", and the fine pass differs between them
    /// on the three `si` that land back on the coarse grid.
    ///
    /// The candidate frequencies are chosen to drive a non-zero coarse
    /// winner, so the fine sweep visits `df` both on and off the grid.
    #[test]
    fn cached_coarse_refs_are_bit_identical() {
        let cd0 = synthetic_cd0(5_120);
        let refs = Ft4CoarsePhasors::new::<Ft4>();
        let none = Ft4CoarsePhasors::empty();
        for freq in [700.0f32, 1_500.0, 2_310.5] {
            let cand = SyncCandidate {
                freq_hz: freq,
                dt_sec: 0.0,
                score: 1.0,
            };
            for (lo, hi) in [(-344, 1012), (0, 667), (0, 0)] {
                let plain = ft4_sync_search_window_with::<Ft4>(&cd0, &cand, lo, hi, &none);
                let cached = ft4_sync_search_window_with::<Ft4>(&cd0, &cand, lo, hi, &refs);
                assert_eq!(
                    plain.i0, cached.i0,
                    "i0 differs at {freq} Hz, window ({lo}, {hi})"
                );
                assert_eq!(
                    plain.freq_hz.to_bits(),
                    cached.freq_hz.to_bits(),
                    "freq differs at {freq} Hz, window ({lo}, {hi})"
                );
                assert_eq!(
                    plain.score.to_bits(),
                    cached.score.to_bits(),
                    "score differs at {freq} Hz, window ({lo}, {hi})"
                );
            }
        }
    }
}

#[cfg(test)]
mod rotator_tests {
    use super::*;

    /// Exactly what `freq_shift_cd0` used to compute, per sample.
    fn shift_exact(cd0: &[Complex<f32>], df_hz: f32, ds_rate: f32) -> Vec<Complex<f32>> {
        let omega = -2.0 * PI * df_hz / ds_rate;
        cd0.iter()
            .enumerate()
            .map(|(n, &c)| {
                let p = omega * n as f32;
                c * Complex::new(p.cos(), p.sin())
            })
            .collect()
    }

    fn ramp(n: usize) -> Vec<Complex<f32>> {
        (0..n)
            .map(|k| {
                let a = k as f32 * 0.017;
                Complex::new(a.cos(), a.sin())
            })
            .collect()
    }

    /// **The rotator is not bit-identical, so this bounds it.**
    ///
    /// `freq_shift_cd0` advances a phasor by one complex multiply per
    /// sample instead of evaluating `cos`/`sin`, which is what every
    /// other mixer in this crate already did. The error that buys back
    /// is rounding in the recurrence, renormalised in magnitude every
    /// `RENORM_PERIOD` samples but not in phase.
    ///
    /// Bounded at both lengths that matter: FT4's `CD0_LEN = 5 120`,
    /// and a 30 000-sample stand-in for FST4's long sub-modes, which
    /// reach this through `fst4::baseline` and `fst4::rung_major`.
    ///
    /// Measured (the samples are unit-magnitude, so a complex
    /// difference is a phase error in radians):
    ///
    /// | | df 0.37 | df −3.7 | df 11.9 |
    /// |---|---|---|---|
    /// | n = 5 120 | 2.7e-6 | 5.1e-5 | 8.1e-5 |
    /// | n = 30 000 | 8.0e-6 | 7.8e-5 | 1.7e-4 |
    ///
    /// The worst of those is 0.0096°. As a frequency error it is the
    /// ramp divided by the span: 1.7e-4 rad over 45 s is **5.9e-7 Hz**,
    /// against FT4's 20.83 Hz tone spacing and the 1 Hz grid the argmax
    /// above resolves — six orders down. `ft4::ddc` already mixes
    /// 40 609 samples a candidate through the same `Mixer` and is swept.
    ///
    /// The bar is 5e-4, which fails on a 3x regression (the
    /// renormalisation going away, say) while sitting far above the
    /// arithmetic's own floor. It is not 1e-4: that was a number
    /// picked before measuring, and 30 000 samples exceeded it.
    #[test]
    fn the_rotator_tracks_the_exact_rotation() {
        for &n in &[5_120usize, 30_000] {
            for &df in &[0.37f32, -3.7, 11.9] {
                let x = ramp(n);
                let got = freq_shift_cd0(&x, df, 666.666_7);
                let want = shift_exact(&x, df, 666.666_7);
                let worst = got
                    .iter()
                    .zip(&want)
                    .map(|(a, b)| (a - b).norm())
                    .fold(0.0f32, f32::max);
                assert!(
                    worst < 5e-4,
                    "n={n} df={df}: worst deviation {worst:e} — the recurrence has drifted"
                );
            }
        }
    }

    /// A zero shift still short-circuits to a copy, exactly.
    #[test]
    fn a_zero_shift_is_a_copy() {
        let x = ramp(64);
        assert_eq!(freq_shift_cd0(&x, 0.0, 666.666_7), x);
    }
}

#[cfg(all(test, feature = "fst4"))]
mod fst4_last_start_tests {
    use super::*;
    use crate::fst4::Fst4s60;

    /// WSJT-X v3.3.0-beta1 `last_start=nfft2-160*nss`, `isbest=-1`: a baseband too short to hold
    /// a whole frame has no start to try, so the search says so (`NEG_INFINITY`) instead of
    /// scoring the missing blocks as 0 and letting a partial score win, as rc1 did.
    #[test]
    fn a_baseband_shorter_than_a_frame_has_no_start() {
        let d = SyncDims::of::<Fst4s60>(12_000.0);
        let frame = <Fst4s60 as crate::engine::FrameLayout>::N_SYMBOLS as usize * d.ds_spb;
        let cand = SyncCandidate {
            freq_hz: 1500.0,
            dt_sec: 0.5,
            score: 1.0,
        };
        let short = vec![Complex::new(0.1f32, 0.0); frame - 1];
        let r = fst4_sync_search::<Fst4s60>(&short, &cand);
        assert_eq!(r.score, f32::NEG_INFINITY);
        assert_eq!(r.freq_hz, 1500.0);

        // Room for exactly one start: it is found, and every start scored lies at or before it.
        let long = vec![Complex::new(0.1f32, 0.0); frame + 40];
        let r = fst4_sync_search::<Fst4s60>(&long, &cand);
        assert!(r.score.is_finite());
        assert!(r.i0 <= 40, "i0 {} past the last start 40", r.i0);
    }
}
