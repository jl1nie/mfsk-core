//! Convolutional + Fano sequential decoder, shared across WSPR / JT9.
//!
//! The Fano algorithm (see [`fano`]) runs bit-by-bit on a rate-1/2 K=32
//! convolutional code. Only the Layland–Lushbaugh generator pair is wired
//! for now (that's what WSPR uses); JT9 uses the same pair, so adding it
//! will be a no-op on this module.
//!
//! The `ConvFano` type implements [`crate::core::FecCodec`] for the specific
//! shape WSPR needs: 50 info bits, 31 zero-tail bits, 162 coded bits.

pub mod fano;

use super::FecCodec;
use crate::core::{FecOpts, FecResult};

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
    /// Default Fano threshold step. 17 is a pragmatic starting point for
    /// our `build_branch_metrics` scale (16.0) and closely mirrors WSJT-X's
    /// 60/10 ≈ 6 ratio when you account for the different quantisation.
    pub const DEFAULT_DELTA: i32 = 17;
    /// Default "max cycles per bit" — 10000 matches WSJT-X's wsprd default.
    pub const DEFAULT_MAX_CYCLES: u64 = 10_000;
    /// LLR → branch-metric quantisation scale.
    pub const METRIC_SCALE: f32 = 16.0;
    /// Fano bias, subtracted from each per-bit metric.
    pub const METRIC_BIAS: f32 = 0.0;
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
        let bm = fano::build_branch_metrics(llr, Self::METRIC_BIAS, Self::METRIC_SCALE);
        let res = fano::fano_decode(
            &bm,
            Self::NBITS,
            Self::DEFAULT_DELTA,
            Self::DEFAULT_MAX_CYCLES,
        );
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
    /// Fano threshold step — same scale as `ConvFano` since the metric
    /// computation hasn't changed.
    pub const DEFAULT_DELTA: i32 = 17;
    /// Max cycles per bit. WSJT-X's jt9_decode varies this with depth
    /// (5 000–100 000); 10 000 matches the wsprd default and decodes
    /// reliably for clean / moderate-SNR signals.
    pub const DEFAULT_MAX_CYCLES: u64 = 10_000;
    pub const METRIC_SCALE: f32 = 16.0;
    pub const METRIC_BIAS: f32 = 0.0;
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

    fn decode_soft(&self, llr: &[f32], _opts: &FecOpts) -> Option<FecResult> {
        assert_eq!(llr.len(), Self::N);
        let bm = fano::build_branch_metrics(llr, Self::METRIC_BIAS, Self::METRIC_SCALE);
        let res = fano::fano_decode(
            &bm,
            Self::NBITS,
            Self::DEFAULT_DELTA,
            Self::DEFAULT_MAX_CYCLES,
        );
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
        let mut llr: Vec<f32> = cw
            .iter()
            .map(|&b| if b == 0 { 6.0 } else { -6.0 })
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
