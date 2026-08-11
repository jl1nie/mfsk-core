//! Convolutional + Fano sequential decoder, shared across WSPR / JT9.
//!
//! The Fano algorithm (see [`fano`]) runs bit-by-bit on a rate-1/2 K=32
//! convolutional code. Only the Layland–Lushbaugh generator pair is wired
//! for now (that's what WSPR uses); JT9 uses the same pair, so adding it
//! will be a no-op on this module.
//!
//! The `ConvFano` type implements [`crate::engine::FecCodec`] for the specific
//! shape WSPR needs: 50 info bits, 31 zero-tail bits, 162 coded bits.

pub mod fano;
mod metric_table;

use alloc::vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::FecCodec;
use crate::engine::{FecOpts, FecResult};

/// WSPR convolutional codec: 50 info bits + 31 zero-tail → 162 coded bits.
///
/// The 31-bit tail is an implementation detail of the Fano decoder (it lets
/// the search terminate in known state); callers see `K = 50` information
/// bits and `N = 162` channel bits.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConvFano;

impl ConvFano {
    /// Total input bits the Fano decoder runs over (50 message + 31 tail).
    pub const NBITS: usize = 81;
    /// Fano threshold step — `wsprd.c:822`'s `delta = 60`, usable
    /// verbatim now that [`fano::build_branch_metrics_wsprd`] puts the
    /// branch metrics on wsprd's own `10 ×` quantisation. (It was 17
    /// while the metric was a linear approximation on an
    /// input-dependent scale.)
    pub const DEFAULT_DELTA: i32 = 60;
    /// Default "max cycles per bit" — 10000 matches WSJT-X's wsprd default.
    pub const DEFAULT_MAX_CYCLES: u64 = 10_000;
    /// LLR → branch-metric quantisation scale.
    pub const METRIC_SCALE: f32 = 16.0;
    /// Fano metric bias — `wsprd.c:823`'s `bias = 0.45`, applied
    /// inside [`fano::build_branch_metrics_wsprd`] against the
    /// empirical table. The old value (1.0) belonged to the linear
    /// metric and was, per its own comment, capped by the lack of a
    /// normalisation pass; that pass now exists.
    pub const METRIC_BIAS: f32 = 0.45;
}

/// Pack the message bits + 31 zero tail into the 11-byte buffer that
/// [`conv_encode`](fano::conv_encode) consumes.
fn pack_msg_with_tail(info: &[u8]) -> [u8; 11] {
    assert_eq!(info.len(), 50, "WSPR info payload must be 50 bits");
    let mut packed = [0u8; 11];
    for (i, &b) in info.iter().enumerate() {
        if b & 1 != 0 {
            packed[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    // Bits 50..81 are the zero tail; bits 81..88 are padding and ignored.
    packed
}

impl crate::engine::protocol::sealed::Sealed for ConvFano {}
impl FecCodec for ConvFano {
    const N: usize = 162;
    const K: usize = 50;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), Self::K);
        assert_eq!(codeword.len(), Self::N);
        let packed = pack_msg_with_tail(info);
        let mut out = vec![0u8; 2 * Self::NBITS];
        fano::conv_encode(&packed, Self::NBITS, &mut out);
        codeword.copy_from_slice(&out);
    }

    fn decode_soft(&self, llr: &[f32], _opts: &FecOpts) -> Option<FecResult> {
        assert_eq!(llr.len(), Self::N);
        let bm = fano::build_branch_metrics_wsprd(llr, Self::METRIC_BIAS);
        let res = fano::fano_decode(
            &bm,
            Self::NBITS,
            Self::DEFAULT_DELTA,
            Self::DEFAULT_MAX_CYCLES,
        );
        self.finish_decode(llr, res)
    }
}

impl ConvFano {
    /// [`FecCodec::decode_soft`] with caller-provided [`fano::FanoScratch`],
    /// reused across every call for one candidate's `nblock` sweep
    /// instead of reallocated per call — see
    /// [`fano::fano_decode_with_scratch`]'s doc comment for why this is
    /// safe to pool.
    pub fn decode_soft_pooled(
        &self,
        llr: &[f32],
        _opts: &FecOpts,
        scratch: &mut fano::FanoScratch,
    ) -> Option<FecResult> {
        assert_eq!(llr.len(), Self::N);
        let bm = fano::build_branch_metrics_wsprd(llr, Self::METRIC_BIAS);
        let res = fano::fano_decode_with_scratch(
            scratch,
            &bm,
            Self::NBITS,
            Self::DEFAULT_DELTA,
            Self::DEFAULT_MAX_CYCLES,
        );
        self.finish_decode(llr, res)
    }

    /// Shared post-processing for [`FecCodec::decode_soft`] /
    /// [`Self::decode_soft_pooled`]: recover the info vector, re-encode
    /// to count hard errors, and reject on non-convergence.
    fn finish_decode(&self, llr: &[f32], res: fano::FanoDecodeResult) -> Option<FecResult> {
        if !res.converged {
            return None;
        }

        // Recover 50-bit info vector (drop the 31-bit zero tail).
        let mut info = vec![0u8; Self::K];
        for i in 0..Self::K {
            info[i] = (res.data[i / 8] >> (7 - (i % 8))) & 1;
        }

        // Re-encode to check consistency and count hard errors.
        let mut reencoded = vec![0u8; Self::N];
        self.encode(&info, &mut reencoded);
        let hard_errors = llr
            .iter()
            .zip(reencoded.iter())
            .filter(|&(&l, &c)| (c == 1) != (l < 0.0))
            .count() as u32;

        Some(FecResult {
            info,
            hard_errors,
            iterations: 0,
        })
    }
}

/// JT9 convolutional codec: 72 info bits + 31 zero-tail → 206 coded bits.
///
/// Shares generator polynomials with [`ConvFano`] (the Layland-Lushbaugh
/// r=½ K=32 pair, POLY1 = 0xf2d0_5351, POLY2 = 0xe461_3c47); only the
/// code dimensions differ. Naming echoes WSJT-X's `fano232.f90`, which
/// is the module this one is modelled on.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConvFano232;

impl ConvFano232 {
    /// Total input bits the Fano decoder runs over (72 message + 31 tail).
    pub const NBITS: usize = 103;
    /// Fano threshold step — `nint(3.4·50) = 170`, matching
    /// `lib/jt9fano.f90` `ndelta`. The 50× scale on the WSJT-X metric
    /// is the `scale` used to quantise the `xx0` LUT below.
    pub const DEFAULT_DELTA: i32 = 170;
    /// Max cycles per bit. WSJT-X's jt9_decode varies this with depth
    /// (5 000–100 000); 10 000 matches the wsprd default and decodes
    /// reliably for clean / moderate-SNR signals.
    pub const DEFAULT_MAX_CYCLES: u64 = 10_000;
}

/// `xx0` log-likelihood lookup table from `lib/jt9fano.f90`. Indexed by
/// the WSJT-X soft-symbol value `i ∈ 0..=255` (corresponding to signed
/// `i4 = i − 128 ∈ −128..=127`). For positive `i4` (bit=1 evidence)
/// the table saturates at strongly-negative values; for negative `i4`
/// (bit=0 evidence) it saturates at +1.0. This asymmetry is what
/// gives Fano its WSJT-X-faithful path-search behaviour at low SNR
/// and is **not** captured by the linear `m = 0.5·l − bias` form used
/// in [`fano::build_branch_metrics`].
#[rustfmt::skip]
const JT9_XX0: [f32; 256] = [
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
    0.988, 1.000, 0.991, 0.993, 1.000, 0.995, 1.000, 0.991,
    1.000, 0.991, 0.992, 0.991, 0.990, 0.990, 0.992, 0.996,
    0.990, 0.994, 0.993, 0.991, 0.992, 0.989, 0.991, 0.987,
    0.985, 0.989, 0.984, 0.983, 0.979, 0.977, 0.971, 0.975,
    0.974, 0.970, 0.970, 0.970, 0.967, 0.962, 0.960, 0.957,
    0.956, 0.953, 0.942, 0.946, 0.937, 0.933, 0.929, 0.920,
    0.917, 0.911, 0.903, 0.895, 0.884, 0.877, 0.869, 0.858,
    0.846, 0.834, 0.821, 0.806, 0.790, 0.775, 0.755, 0.737,
    0.713, 0.691, 0.667, 0.640, 0.612, 0.581, 0.548, 0.510,
    0.472, 0.425, 0.378, 0.328, 0.274, 0.212, 0.146, 0.075,
    0.000,-0.079,-0.163,-0.249,-0.338,-0.425,-0.514,-0.606,
   -0.706,-0.796,-0.895,-0.987,-1.084,-1.181,-1.280,-1.376,
   -1.473,-1.587,-1.678,-1.790,-1.882,-1.992,-2.096,-2.201,
   -2.301,-2.411,-2.531,-2.608,-2.690,-2.829,-2.939,-3.058,
   -3.164,-3.212,-3.377,-3.463,-3.550,-3.768,-3.677,-3.975,
   -4.062,-4.098,-4.186,-4.261,-4.472,-4.621,-4.623,-4.608,
   -4.822,-4.870,-4.652,-4.954,-5.108,-5.377,-5.544,-5.995,
   -5.632,-5.826,-6.304,-6.002,-6.559,-6.369,-6.658,-7.016,
   -6.184,-7.332,-6.534,-6.152,-6.113,-6.288,-6.426,-6.313,
   -9.966,-6.371,-9.966,-7.055,-9.966,-6.629,-6.313,-9.966,
   -5.858,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
   -9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
   -9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
   -9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
   -9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
   -9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,-9.966,
];

/// Build the 256×2 mettab used by `jt9fano`. Mirrors lines 65–77 of
/// `lib/jt9fano.f90`:
///
/// ```fortran
/// bias=0.5; scale=50; ndelta=nint(3.4*scale); ib=160; slope=2
/// do i=0,255
///   mettab(i-128,0)=nint(scale*(xx0(i)-bias))
///   if(i.gt.ib) mettab(i-128,0)=mettab(ib-128,0) - slope*(i-ib)
///   if(i.ge.1)  mettab(128-i,1)=mettab(i-128,0)
/// enddo
/// mettab(-128,1)=mettab(-127,1)
/// ```
fn build_jt9_mettab() -> [[i32; 2]; 256] {
    const BIAS: f32 = 0.5;
    const SCALE: f32 = 50.0;
    const IB: usize = 160;
    const SLOPE: i32 = 2;
    let mut mettab = [[0i32; 2]; 256];
    // Column 0: m_if_sent_bit_was_0 (raw + linear extension beyond IB).
    let mut col0 = [0i32; 256];
    for (i, val) in JT9_XX0.iter().enumerate() {
        col0[i] = (SCALE * (val - BIAS)).round() as i32;
    }
    let pivot = col0[IB];
    for i in (IB + 1)..=255 {
        col0[i] = pivot - SLOPE * (i as i32 - IB as i32);
    }
    for i in 0..=255 {
        mettab[i][0] = col0[i];
    }
    // Column 1 mirrors column 0 by Fortran rule
    // `mettab(128−i, 1) = mettab(i−128, 0)` for i ∈ 1..=255 — i.e.
    // `mettab[256−i][1] = col0[i]`. Then `mettab(−128, 1)` falls back
    // to `mettab(−127, 1)` because (256 − 0) = 256 is out of range.
    for i in 1..=255 {
        mettab[256 - i][1] = col0[i];
    }
    mettab[0][1] = mettab[1][1];
    mettab
}

/// Convert mfsk-core LLRs (positive ⇒ bit=0, scale ≈ 10) into Fano
/// branch metrics using the WSJT-X `jt9fano` mettab. Each LLR is
/// negated to match WSJT-X's "positive ⇒ bit=1" soft-symbol sign,
/// clipped to `i4 ∈ [−127, 127]`, and looked up in the mettab.
fn jt9_branch_metrics(llrs: &[f32]) -> alloc::vec::Vec<[i32; 2]> {
    let mettab = build_jt9_mettab();
    llrs.iter()
        .map(|&l| {
            // mfsk-core LLR sign is opposite of WSJT-X i4: flip.
            let i4 = (-l).round().clamp(-127.0, 127.0) as i32;
            let idx = (i4 + 128) as usize;
            // Fano expects [m_if_0, m_if_1] per coded bit.
            [mettab[idx][0], mettab[idx][1]]
        })
        .collect()
}

/// Pack 72 message bits + 31-bit zero tail into the 13-byte buffer that
/// [`conv_encode`](fano::conv_encode) consumes (NBITS = 103 → 13 bytes
/// with the last 4 bits unused).
fn pack_msg_with_tail_jt9(info: &[u8]) -> [u8; 13] {
    assert_eq!(info.len(), 72, "JT9 info payload must be 72 bits");
    let mut packed = [0u8; 13];
    for (i, &b) in info.iter().enumerate() {
        if b & 1 != 0 {
            packed[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    // Bits 72..103 are the zero tail; bits 103..104 are padding.
    packed
}

impl crate::engine::protocol::sealed::Sealed for ConvFano232 {}
impl FecCodec for ConvFano232 {
    const N: usize = 206;
    const K: usize = 72;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), Self::K);
        assert_eq!(codeword.len(), Self::N);
        let packed = pack_msg_with_tail_jt9(info);
        let mut out = vec![0u8; 2 * Self::NBITS];
        fano::conv_encode(&packed, Self::NBITS, &mut out);
        codeword.copy_from_slice(&out);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts) -> Option<FecResult> {
        assert_eq!(llr.len(), Self::N);
        // JT9 uses the WSJT-X-calibrated `xx0` mettab rather than the
        // linear `m = 0.5·l − bias` form — the asymmetric saturation
        // (heavy penalty for high-confidence disagreement, flat
        // reward for high-confidence agreement) is what stops Fano
        // from latching onto plausible-looking neighbour codewords
        // at marginal SNR.
        let bm = jt9_branch_metrics(llr);
        let max_cycles = opts.max_cycles_per_bit.unwrap_or(Self::DEFAULT_MAX_CYCLES);
        let res = fano::fano_decode(&bm, Self::NBITS, Self::DEFAULT_DELTA, max_cycles);
        if !res.converged {
            return None;
        }
        let mut info = vec![0u8; Self::K];
        for i in 0..Self::K {
            info[i] = (res.data[i / 8] >> (7 - (i % 8))) & 1;
        }
        let mut reencoded = vec![0u8; Self::N];
        self.encode(&info, &mut reencoded);
        let hard_errors = llr
            .iter()
            .zip(reencoded.iter())
            .filter(|&(&l, &c)| (c == 1) != (l < 0.0))
            .count() as u32;
        Some(FecResult {
            info,
            hard_errors,
            iterations: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_then_decode_roundtrip() {
        let codec = ConvFano;
        // Arbitrary 50-bit info word.
        let mut info = vec![0u8; 50];
        for (i, slot) in info.iter_mut().enumerate() {
            *slot = (((i * 7) ^ 0x2a) & 1) as u8;
        }
        let mut cw = vec![0u8; 162];
        codec.encode(&info, &mut cw);

        // Perfect LLRs.
        let llr: Vec<f32> = cw
            .iter()
            .map(|&b| if b == 0 { 8.0 } else { -8.0 })
            .collect();
        let r = codec
            .decode_soft(&llr, &FecOpts::default())
            .expect("perfect LLRs must decode");
        assert_eq!(r.info, info);
        assert_eq!(r.hard_errors, 0);
    }

    #[test]
    fn jt9_encode_decode_roundtrip() {
        let codec = ConvFano232;
        let mut info = vec![0u8; 72];
        for (i, slot) in info.iter_mut().enumerate() {
            *slot = (((i * 11) ^ 0x55) & 1) as u8;
        }
        let mut cw = vec![0u8; 206];
        codec.encode(&info, &mut cw);
        let llr: Vec<f32> = cw
            .iter()
            .map(|&b| if b == 0 { 8.0 } else { -8.0 })
            .collect();
        let r = codec
            .decode_soft(&llr, &FecOpts::default())
            .expect("perfect LLRs must decode");
        assert_eq!(r.info, info);
        assert_eq!(r.hard_errors, 0);
    }

    #[test]
    fn jt9_tolerates_a_few_errors() {
        let codec = ConvFano232;
        let info: Vec<u8> = (0..72).map(|i| i as u8 & 1).collect();
        let mut cw = vec![0u8; 206];
        codec.encode(&info, &mut cw);
        // Real-pipeline `symspec2` emits LLRs in ±127 (matching WSJT-X
        // soft sym scale=10), and the `jt9fano` mettab below is
        // calibrated for that range. Use a strong magnitude here so
        // the synthetic LLRs land in the same regime.
        let mut llr: Vec<f32> = cw
            .iter()
            .map(|&b| if b == 0 { 100.0 } else { -100.0 })
            .collect();
        for &pos in &[3usize, 17, 42, 91, 155, 199] {
            llr[pos] = -llr[pos] * 0.3;
        }
        let r = codec
            .decode_soft(&llr, &FecOpts::default())
            .expect("should correct 6 weak errors");
        assert_eq!(r.info, info);
    }

    #[test]
    fn tolerates_a_few_errors() {
        let codec = ConvFano;
        let info: Vec<u8> = (0..50).map(|i| i as u8 & 1).collect();
        let mut cw = vec![0u8; 162];
        codec.encode(&info, &mut cw);
        // Strong LLRs.
        let mut llr: Vec<f32> = cw
            .iter()
            .map(|&b| if b == 0 { 6.0 } else { -6.0 })
            .collect();
        // Flip 5 LLRs to the wrong side with lower magnitude — simulates noise
        // on a handful of coded bits.
        for &pos in &[3usize, 17, 42, 91, 155] {
            llr[pos] = -llr[pos] * 0.3;
        }
        let r = codec
            .decode_soft(&llr, &FecOpts::default())
            .expect("should correct 5 weak errors");
        assert_eq!(r.info, info);
    }
}
