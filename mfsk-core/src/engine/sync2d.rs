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
use num_traits::Float;

use crate::engine::Protocol;
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
        Self {
            n,
            plain: AlignedF32::new(n * 2),
            swapped: AlignedF32::new(n * 2),
            // Two leading zeros, then the same `n * 2` samples.
            #[cfg(feature = "dotprod-extern")]
            plain_odd: AlignedF32::new(n * 2 + 2),
            #[cfg(feature = "dotprod-extern")]
            swapped_odd: AlignedF32::new(n * 2 + 2),
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
        let mut a = AlignedF32::new(n2);
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
pub fn fst4_sync_search<P: Protocol>(
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
            let s = score_flat(&twiddled, i0);
            if s > best_score {
                best_score = s;
                best_df = df;
                best_i0 = i0;
            }
            di += 4;
        }
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
pub fn ft4_sync_search<P: Protocol>(
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

impl Ft4CoarsePhasors {
    /// Build the nine coarse-sweep phasor tables for `P`.
    pub fn new<P: Protocol>() -> Self {
        let d = SyncDims::of::<P>(12_000.0);
        // Every Costas block is `nsym · ds_spb` samples and they are
        // all the same length, so one table covers all four.
        let n = P::SYNC_MODE
            .blocks()
            .first()
            .map(|b| b.pattern.len() * d.ds_spb)
            .unwrap_or(0);
        let mut tables = Vec::new();
        let mut idf = COARSE_DF_MIN;
        while idf <= COARSE_DF_MAX {
            let df = idf as f32;
            let omega = 2.0 * PI * df / d.ds_rate;
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
pub fn ft4_sync_ref_prep_bench<P: Protocol>(fills: usize) -> f32 {
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
pub fn ft4_sync_search_window<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
) -> Sync2dResult {
    ft4_sync_search_window_cached::<P>(cd0, candidate, ib_min, ib_max, None)
}

/// [`ft4_sync_search_window`], reading the coarse pass's references
/// from `refs` instead of rebuilding them.
///
/// `None` is the original behaviour. `Some` is bit-identical to it —
/// the same `df` values in the same order, so the same references and
/// the same coarse argmax — and skips the nine fills that make up
/// half of this function's 30 % reference-building overhead. See
/// [`Ft4CoarsePhasors`] for the measurement.
pub fn ft4_sync_search_window_cached<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    ib_min: i32,
    ib_max: i32,
    refs: Option<&Ft4CoarsePhasors>,
) -> Sync2dResult {
    // See [`AlignedCd0`]; same reasoning as `fst4_sync_search`.
    let aligned = AlignedCd0::new(cd0);
    let cd0 = aligned.get(cd0);

    // Only `d.ds_spb`/`d.ds_rate` are read below — see
    // `fst4_sync_search`'s identical comment.
    let d = SyncDims::of::<P>(12_000.0);
    let ds_spb = d.ds_spb;
    let ds_rate = d.ds_rate;
    const COARSE_DT_STEP: i32 = 4;

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

    let score_flat = |twiddled: &Vec<(i32, FlatRef)>, i0: i32| -> f32 {
        twiddled
            .iter()
            .map(|(off, flat)| score_flat_coherent(cd0, flat, i0 + off))
            .sum::<f32>()
    };

    // `FlatRef::fill` applies `e^{+j.2pi.df.n/ds_rate}` to the reference
    // and `score_flat_coherent` conjugates it, giving
    // `sum c[n].conj(r[n]).e^{-j.2pi.df.n/ds_rate}` — exactly the sign
    // convention the replaced `phasor_for` used
    // (`omega = -2pi.df/ds_rate`).
    let retwiddle = |twiddled: &mut Vec<(i32, FlatRef)>, df: f32| {
        for ((_, dst), (_, src)) in twiddled.iter_mut().zip(flat_blocks.iter()) {
            dst.fill(src, df, ds_rate);
        }
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
    match refs {
        Some(cache) => {
            for (df, table) in &cache.tables {
                for ((_, dst), (_, src)) in twiddled.iter_mut().zip(flat_blocks.iter()) {
                    dst.fill_with(src, *df, table);
                }
                scan_coarse(&twiddled, *df, &mut best_score, &mut best_df, &mut best_i0);
            }
        }
        None => {
            let mut idf = COARSE_DF_MIN;
            while idf <= COARSE_DF_MAX {
                let df = idf as f32;
                retwiddle(&mut twiddled, df);
                scan_coarse(&twiddled, df, &mut best_score, &mut best_df, &mut best_i0);
                idf += COARSE_DF_STEP;
            }
        }
    }

    // Fine pass around the coarse winner.
    let coarse_winner_df = best_df;
    let coarse_winner_i0 = best_i0;
    best_score = f32::NEG_INFINITY;

    for si in -4i32..=4 {
        let df = coarse_winner_df + si as f32;
        retwiddle(&mut twiddled, df);
        for di in -5i32..=5 {
            let i0 = coarse_winner_i0 + di;
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

/// Apply a complex-phasor freq shift to `cd0`. Used by callers that
/// take the [`Sync2dResult::freq_hz`] from this module and want to
/// run [`crate::engine::llr::symbol_spectra`] on a baseband whose
/// carrier sits at the refined freq.
pub fn freq_shift_cd0(cd0: &[Complex<f32>], df_hz: f32, ds_rate: f32) -> Vec<Complex<f32>> {
    if df_hz.abs() < f32::EPSILON {
        return cd0.to_vec();
    }
    let omega = -2.0 * PI * df_hz / ds_rate;
    cd0.iter()
        .enumerate()
        .map(|(n, &c)| {
            let p = omega * n as f32;
            c * Complex::new(p.cos(), p.sin())
        })
        .collect()
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

    /// The cached coarse pass must not merely agree — it must be the
    /// same number. It visits the same `df` values in the same order
    /// and scores against references built by the same `fill`, so
    /// anything less than exact equality means the cache changed what
    /// the search sees.
    #[test]
    fn cached_coarse_refs_are_bit_identical() {
        let cd0 = synthetic_cd0(5_120);
        let refs = Ft4CoarsePhasors::new::<Ft4>();
        for freq in [700.0f32, 1_500.0, 2_310.5] {
            let cand = SyncCandidate {
                freq_hz: freq,
                dt_sec: 0.0,
                score: 1.0,
            };
            for (lo, hi) in [(-344, 1012), (0, 667), (0, 0)] {
                let plain = ft4_sync_search_window::<Ft4>(&cd0, &cand, lo, hi);
                let cached = ft4_sync_search_window_cached::<Ft4>(&cd0, &cand, lo, hi, Some(&refs));
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
