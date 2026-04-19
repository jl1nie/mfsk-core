//! Protocol-agnostic soft-decision LLR computation.
//!
//! Extracts complex tone spectra for each data symbol, then computes four
//! log-likelihood ratio variants (llra/b/c/d) matching WSJT-X `ft8b.f90`
//! convention — three `nsym = 1, 2, 3` grouping schemes plus a bit-by-bit
//! normalised variant. Parameterised over any [`Protocol`]: NTONES,
//! BITS_PER_SYMBOL, and the SYNC_BLOCKS layout drive the inner loops.

use super::Protocol;
use num_complex::Complex;
use rustfft::FftPlanner;

// ──────────────────────────────────────────────────────────────────────────
// LLR bundle
// ──────────────────────────────────────────────────────────────────────────

/// Four LLR vectors (a, b, c, d) of length `codeword_len()` bits in LDPC
/// bit-index order.
#[derive(Clone)]
pub struct LlrSet {
    /// nsym=1 soft metrics, scaled (matches WSJT-X llra).
    pub llra: Vec<f32>,
    /// nsym=2 soft metrics, scaled (matches WSJT-X llrb).
    pub llrb: Vec<f32>,
    /// nsym=3 soft metrics, scaled (matches WSJT-X llrc).
    pub llrc: Vec<f32>,
    /// nsym=1 bit-normalised (matches WSJT-X llrd).
    pub llrd: Vec<f32>,
}

/// Default LLR scale factor from WSJT-X ft8b.f90. Individual protocols may
/// override via `ModulationParams::LLR_SCALE`.
pub const LLR_SCALE: f32 = 2.83;

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
pub fn symbol_spectra<P: Protocol>(cd0: &[Complex<f32>], i_start: usize) -> Vec<Complex<f32>> {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let ds_spb = (P::NSPS / P::NDOWN) as usize;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(ds_spb);

    let mut cs = vec![Complex::new(0.0f32, 0.0); n_sym * ntones];
    let mut buf = vec![Complex::new(0.0f32, 0.0); ds_spb];

    for k in 0..n_sym {
        let i1 = i_start + k * ds_spb;
        for (j, b) in buf.iter_mut().enumerate() {
            *b = if i1 + j < cd0.len() {
                cd0[i1 + j]
            } else {
                Complex::new(0.0, 0.0)
            };
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

/// Ordered list of data chunks: `(first_data_symbol, chunk_len_in_symbols)`.
///
/// FT8: 2 chunks `[(7, 29), (43, 29)]`. FT4: 3 chunks `[(4, 29), (37, 29), (70, 29)]`.
fn data_chunks<P: Protocol>() -> Vec<(usize, usize)> {
    let blocks = P::SYNC_MODE.blocks();
    let mut chunks = Vec::with_capacity(blocks.len().saturating_sub(1));
    for i in 0..blocks.len().saturating_sub(1) {
        let after = blocks[i].start_symbol as usize + blocks[i].pattern.len();
        let before_next = blocks[i + 1].start_symbol as usize;
        if before_next > after {
            chunks.push((after, before_next - after));
        }
    }
    chunks
}

/// Decompose `i` into `nsym` base-`ntones` digits, most significant first.
#[inline]
fn base_digits(mut i: usize, ntones: usize, nsym: usize) -> Vec<usize> {
    let mut out = vec![0usize; nsym];
    for j in (0..nsym).rev() {
        out[j] = i % ntones;
        i /= ntones;
    }
    out
}

#[inline]
fn normalize_bmet(bmet: &mut [f32]) {
    let n = bmet.len() as f32;
    let mean = bmet.iter().sum::<f32>() / n;
    let mean_sq = bmet.iter().map(|&x| x * x).sum::<f32>() / n;
    let var = mean_sq - mean * mean;
    let sig = if var > 0.0 { var.sqrt() } else { mean_sq.sqrt() };
    if sig > 0.0 {
        bmet.iter_mut().for_each(|x| *x /= sig);
    }
}

// ──────────────────────────────────────────────────────────────────────────
// LLR computation
// ──────────────────────────────────────────────────────────────────────────

/// Compute soft LLRs from the flat symbol-spectra vector.
pub fn compute_llr<P: Protocol>(cs: &[Complex<f32>]) -> LlrSet {
    let ntones = P::NTONES as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray_map = P::GRAY_MAP;
    let chunks = data_chunks::<P>();
    let codeword_len: usize = chunks.iter().map(|&(_, l)| l).sum::<usize>() * bps;

    let mut bmeta = vec![0.0f32; codeword_len];
    let mut bmetb = vec![0.0f32; codeword_len];
    let mut bmetc = vec![0.0f32; codeword_len];
    let mut bmetd = vec![0.0f32; codeword_len];

    for nsym in 1usize..=3 {
        // Number of tone-combinations over `nsym` symbols.
        let nt = ntones.pow(nsym as u32);
        let ibmax = bps * nsym - 1;
        let mut s2 = vec![0.0f32; nt];

        // Walk the data symbols chunk by chunk to build the contiguous
        // 174-bit codeword layout used by the LDPC decoder.
        let mut chunk_bit_base = 0usize;
        for &(chunk_start_sym, chunk_len) in &chunks {
            let mut k = 0usize; // symbol index within this chunk
            while k + nsym <= chunk_len {
                let ks = chunk_start_sym + k;

                // Precompute |Σ cs_k[gray[idx_k]]| for each tone-combination.
                for (i, s2_i) in s2.iter_mut().enumerate() {
                    let digits = base_digits(i, ntones, nsym);
                    let sum: Complex<f32> = (0..nsym)
                        .map(|j| cs[(ks + j) * ntones + gray_map[digits[j]] as usize])
                        .sum();
                    *s2_i = sum.norm();
                }

                // Map each of the `ibmax+1` bits into the codeword.
                let i_bit_base = chunk_bit_base + k * bps;
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

                    match nsym {
                        1 => {
                            bmeta[bit_idx] = bm;
                            let den = max_one.max(max_zero);
                            bmetd[bit_idx] = if den > 0.0 { bm / den } else { 0.0 };
                        }
                        2 => bmetb[bit_idx] = bm,
                        3 => bmetc[bit_idx] = bm,
                        _ => unreachable!(),
                    }
                }

                k += nsym;
            }
            chunk_bit_base += chunk_len * bps;
        }
    }

    normalize_bmet(&mut bmeta);
    normalize_bmet(&mut bmetb);
    normalize_bmet(&mut bmetc);
    normalize_bmet(&mut bmetd);

    let s = P::LLR_SCALE;
    let scale = |v: Vec<f32>| -> Vec<f32> { v.into_iter().map(|x| x * s).collect() };

    LlrSet {
        llra: scale(bmeta),
        llrb: scale(bmetb),
        llrc: scale(bmetc),
        llrd: scale(bmetd),
    }
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
pub fn compute_snr_db<P: Protocol>(cs: &[Complex<f32>], itone: &[u8]) -> f32 {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let mut xsig = 0.0f32;
    let mut xnoi = 0.0f32;
    let offset = ntones / 2;
    for k in 0..n_sym.min(itone.len()) {
        let t = itone[k] as usize % ntones;
        xsig += cs[k * ntones + t].norm_sqr();
        xnoi += cs[k * ntones + (t + offset) % ntones].norm_sqr();
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
pub fn sync_quality<P: Protocol>(cs: &[Complex<f32>]) -> u32 {
    let ntones = P::NTONES as usize;
    let mut count = 0u32;
    for block in P::SYNC_MODE.blocks() {
        let start = block.start_symbol as usize;
        for (t, &expected) in block.pattern.iter().enumerate() {
            let sym = start + t;
            let best = (0..ntones)
                .max_by(|&a, &b| {
                    cs[sym * ntones + a]
                        .norm()
                        .partial_cmp(&cs[sym * ntones + b].norm())
                        .unwrap()
                })
                .unwrap_or(0);
            if best == expected as usize {
                count += 1;
            }
        }
    }
    count
}
