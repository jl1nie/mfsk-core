//! MSK144 TX synthesis: a 128-bit LDPC(128,90) codeword -> 144-bit
//! channel bit sequence -> 864-sample complex baseband frame.
//!
//! Ported from WSJT-X `genmsk_128_90.f90`. Message packing
//! (`pack77`/CRC-13) and LDPC encoding are the caller's job — see
//! [`crate::msg::wsjt77`] and [`crate::fec::Ldpc128_90`] — this module
//! covers only the MSK144-specific channel framing, delegating the
//! OQPSK waveform math to [`crate::engine::dsp::msk`].

use num_complex::Complex32;

use crate::engine::dsp::msk::{NSPM, build_bitseq, synth_frame};

/// Synthesise the 864-sample complex baseband frame for a 128-bit
/// LDPC(128,90) codeword (already produced by
/// [`crate::fec::Ldpc128_90::encode`](crate::engine::FecCodec::encode)).
/// Convenience wrapper combining [`build_bitseq`] + [`synth_frame`].
pub fn synth_codeword_frame(codeword: &[u8; 128]) -> [Complex32; NSPM] {
    let bitseq = build_bitseq(codeword);
    synth_frame(&bitseq)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::dsp::msk::{build_bitseq, matched_filter_softbits};
    use crate::engine::{FecCodec, FecOpts};
    use crate::fec::Ldpc128_90;
    use crate::fec::ldpc_128_90::check_crc13;

    fn build_info_with_crc(pattern: impl Fn(usize) -> u8) -> [u8; 90] {
        let mut info = [0u8; 90];
        for i in 0..77 {
            info[i] = pattern(i);
        }
        let mut bytes = [0u8; 12];
        for (i, &b) in info[..77].iter().enumerate() {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            bytes[byte_idx] |= (b & 1) << bit_pos;
        }
        let crc = crate::fec::ldpc_128_90::crc13(&bytes);
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }
        info
    }

    /// WSJT-X normalization (`msk144decodeframe.f90:85-93`): zero-mean
    /// / unit-variance softbits, then `llr = 2*softbit/sigma^2` with
    /// the fixed `sigma=0.60` magic constant, over just the 128
    /// data-bit softbits (both embedded sync words are skipped: bits
    /// 8..56 and 64..144, 0-based).
    fn softbits_to_data_llr(softbits: &[f32; 144]) -> [f32; 128] {
        let mean: f32 = softbits.iter().sum::<f32>() / 144.0;
        let var: f32 = softbits
            .iter()
            .map(|s| (s - mean) * (s - mean))
            .sum::<f32>()
            / 144.0;
        let sigma_sig = var.sqrt();

        const SIGMA: f32 = 0.60;
        let mut llr = [0.0f32; 128];
        for (i, v) in llr[0..48].iter_mut().enumerate() {
            *v = 2.0 * (softbits[8 + i] / sigma_sig) / (SIGMA * SIGMA);
        }
        for (i, v) in llr[48..128].iter_mut().enumerate() {
            *v = 2.0 * (softbits[64 + i] / sigma_sig) / (SIGMA * SIGMA);
        }
        llr
    }

    /// End-to-end: encode -> build channel frame -> matched filter ->
    /// scale to LLR -> LDPC decode, at zero CFO / perfect timing /
    /// zero carrier phase. Exercises the full DSP chain added in
    /// Phase 2 plus Phase 1's codec.
    #[test]
    fn clean_frame_roundtrips_through_ldpc() {
        let info = build_info_with_crc(|i| ((i * 7 + 3) & 1) as u8);
        assert!(check_crc13(&info));

        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);

        let frame = synth_codeword_frame(&codeword);
        let softbits = matched_filter_softbits(&frame);

        // Hard-decision check: every softbit's sign must match its
        // transmitted bipolar bit.
        let bitseq = build_bitseq(&codeword);
        for i in 0..144 {
            assert_eq!(
                softbits[i] > 0.0,
                bitseq[i] == 1,
                "softbit {i} sign mismatch"
            );
        }

        let llr = softbits_to_data_llr(&softbits);
        let opts = FecOpts {
            verify_info: Some(check_crc13),
            ..Default::default()
        };
        let r = Ldpc128_90
            .decode_soft(&llr, &opts)
            .expect("BP converges on clean synth frame");
        assert_eq!(&r.info[..77], &info[..77]);
    }

    /// Same chain, but with an arbitrary constant carrier-phase offset
    /// applied before demodulation, to genuinely exercise the
    /// carrier-phase estimate/derotation step (a phase-0 test alone
    /// wouldn't meaningfully exercise it since derotation would be a
    /// near-no-op).
    #[test]
    fn phase_rotated_frame_still_decodes() {
        let info = build_info_with_crc(|i| ((i * 3 + 5) & 1) as u8);
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);

        let mut frame = synth_codeword_frame(&codeword);
        let phase = 0.7_f32; // arbitrary, non-trivial constant phase offset
        let rot = Complex32::new(phase.cos(), phase.sin());
        for c in frame.iter_mut() {
            *c *= rot;
        }

        let softbits = matched_filter_softbits(&frame);
        let bitseq = build_bitseq(&codeword);
        for i in 0..144 {
            assert_eq!(
                softbits[i] > 0.0,
                bitseq[i] == 1,
                "softbit {i} sign mismatch under phase rotation"
            );
        }

        let llr = softbits_to_data_llr(&softbits);
        let opts = FecOpts {
            verify_info: Some(check_crc13),
            ..Default::default()
        };
        let r = Ldpc128_90
            .decode_soft(&llr, &opts)
            .expect("BP converges under constant phase rotation");
        assert_eq!(&r.info[..77], &info[..77]);
    }

    /// Deterministic small perturbation (not true AWGN -- this crate's
    /// convention for fast unit tests is a reproducible per-index
    /// pattern rather than an RNG dependency, see
    /// `fec::ldpc_128_90::tests::roundtrip_noisy_llr_recovers`) added
    /// to a handful of samples, confirming the full chain tolerates
    /// small distortion rather than requiring bit-exact synth input.
    #[test]
    fn mildly_perturbed_frame_still_decodes() {
        let info = build_info_with_crc(|i| ((i * 5 + 1) & 1) as u8);
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);

        let mut frame = synth_codeword_frame(&codeword);
        for (k, c) in frame.iter_mut().enumerate() {
            // Small deterministic perturbation, amplitude well below
            // the unit-pulse peak, applied to every 7th sample.
            if k % 7 == 0 {
                let n = (k as f32 * 12.9898).sin() * 43_758.547;
                let pseudo = n - n.floor() - 0.5; // in [-0.5, 0.5)
                *c += Complex32::new(0.15 * pseudo, -0.15 * pseudo);
            }
        }

        let softbits = matched_filter_softbits(&frame);
        let llr = softbits_to_data_llr(&softbits);
        let opts = FecOpts {
            verify_info: Some(check_crc13),
            ..Default::default()
        };
        let r = Ldpc128_90
            .decode_soft(&llr, &opts)
            .expect("BP converges under mild perturbation");
        assert_eq!(&r.info[..77], &info[..77]);
    }
}
