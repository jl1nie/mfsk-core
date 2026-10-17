//! MSK144 transmit: a 77-bit message to 144 channel tones to audio, as
//! WSJT-X sends it (#497).
//!
//! - [`message_tones`] ports `genmsk_128_90.f90`: CRC-13 and LDPC(128,90)
//!   over the 77 message bits, the 144-bit channel vector (8-bit sync, 48
//!   bits, 8-bit sync, 80 bits), and the MSK-to-tone mapping that makes the
//!   offset-QPSK signal a binary CPFSK one (`:109-118`, polarity flipped).
//! - [`synthesize`] is what WSJT-X's modulator does with those tones in
//!   MSK144 mode (`mainwindow.cpp`, `m_mode == "MSK144"`; `Modulator.cpp`):
//!   6 samples a tone at 12 kHz (2000 baud), tone 0 at `f0` and tone 1 at
//!   `f0 + 1000` Hz with `f0 = 1000`, so the signal is centred on 1500 Hz
//!   whatever the Tx frequency; the frame repeated, phase-continuous, for the
//!   T/R period less 0.5 s, the last 816 samples at 48 kHz (204 at 12 kHz)
//!   faded by 0.98 a sample.
//! - [`synth_codeword_frame`] is the complex-baseband form of one frame, which
//!   the decoder's own tests use.
//!
//! The short `<CALL1 CALL2> RPT` form (MSK40, `genmsk40`) is not ported,
//! as on the receive side (#25).

use alloc::vec::Vec;
use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::FecCodec;
use crate::engine::dsp::msk::{NSPM, build_bitseq, synth_frame};
use crate::fec::Ldpc128_90;

/// Samples per tone at 12 kHz (`m_nsps = 6`).
pub const SAMPLES_PER_TONE: usize = 6;
/// Tones per frame.
pub const FRAME_TONES: usize = 144;
/// Centre of the transmitted signal, Hz: WSJT-X's MSK144 modulator uses
/// `f0 = 1000` and a 1000 Hz tone spacing regardless of the Tx frequency.
pub const CENTER_HZ: f32 = 1500.0;

/// Synthesise the 864-sample complex baseband frame for a 128-bit
/// LDPC(128,90) codeword (already produced by
/// [`crate::fec::Ldpc128_90::encode`](crate::engine::FecCodec::encode)).
/// Convenience wrapper combining [`build_bitseq`] + [`synth_frame`].
pub fn synth_codeword_frame(codeword: &[u8; 128]) -> [Complex32; NSPM] {
    let bitseq = build_bitseq(codeword);
    synth_frame(&bitseq)
}

/// The 128-bit codeword for a 77-bit message: CRC-13 appended
/// (`encode_128_90`), then LDPC(128,90).
pub fn codeword(msg77: &[u8; 77]) -> [u8; 128] {
    let mut info = [0u8; 90];
    info[..77].copy_from_slice(msg77);
    let mut bytes = [0u8; 12];
    for (i, &b) in msg77.iter().enumerate() {
        bytes[i / 8] |= (b & 1) << (7 - (i % 8));
    }
    let crc = crate::fec::ldpc_128_90::crc13(&bytes);
    for i in 0..13 {
        info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
    }
    let mut cw = [0u8; 128];
    Ldpc128_90.encode(&info, &mut cw);
    cw
}

/// The 144 audio tones (0 or 1) for a 77-bit message, `genmsk_128_90.f90`'s
/// `i4tone`.
pub fn message_tones(msg77: &[u8; 77]) -> [u8; FRAME_TONES] {
    let bitseq = build_bitseq(&codeword(msg77));
    let b = |i: usize| 2 * i32::from(bitseq[i]) - 1;
    let mut tones = [0u8; FRAME_TONES];
    for i in 1..=72usize {
        // `i4tone(2i-1) = (b(2i) b(2i-1) + 1)/2`,
        // `i4tone(2i) = -(b(2i) b(mod(2i,144)+1) - 1)/2`, then `1 - i4tone`.
        let t1 = (b(2 * i - 1) * b(2 * i - 2) + 1) / 2;
        let t2 = -(b(2 * i - 1) * b((2 * i) % 144) - 1) / 2;
        tones[2 * i - 2] = (1 - t1) as u8;
        tones[2 * i - 1] = (1 - t2) as u8;
    }
    tones
}

/// Samples [`synthesize`] makes for a T/R period of `period_s` seconds:
/// the period less 0.5 s (`i1 = TRperiod*48000 - 24000` at 48 kHz).
pub fn transmission_len(period_s: u32) -> usize {
    (period_s as usize * 12_000).saturating_sub(6_000)
}

/// 12 kHz audio for one transmission: `tones` repeated for
/// [`transmission_len`]`(period_s)` samples, peak `amplitude`, as WSJT-X's
/// modulator plays MSK144. The signal sits at [`CENTER_HZ`] ± 500 Hz.
pub fn synthesize(tones: &[u8; FRAME_TONES], period_s: u32, amplitude: f32) -> Vec<f32> {
    let n = transmission_len(period_s);
    let twopi = 2.0 * core::f64::consts::PI;
    let f0 = f64::from(CENTER_HZ) - 500.0;
    let mut out = Vec::with_capacity(n);
    let mut phi = 0.0f64;
    // `Modulator.cpp` fades over its last 816 samples at 48 kHz, 0.98 a
    // sample: 204 samples at 12 kHz, 0.98^4 each.
    let fade_from = n.saturating_sub(204);
    let mut amp = f64::from(amplitude);
    for k in 0..n {
        let tone = tones[(k / SAMPLES_PER_TONE) % FRAME_TONES];
        let dphi = twopi * (f0 + 1000.0 * f64::from(tone)) / 12_000.0;
        phi += dphi;
        if phi > twopi {
            phi -= twopi;
        }
        if k >= fade_from {
            amp *= 0.98f64.powi(4);
        }
        out.push((amp * phi.sin()) as f32);
    }
    out
}

/// [`message_tones`] then [`synthesize`] for a standard message
/// (`CALL1 CALL2 REPORT`, as [`crate::msg::wsjt77::pack77`] takes it).
/// `None` if the message does not pack.
pub fn synthesize_standard(
    call1: &str,
    call2: &str,
    report: &str,
    period_s: u32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let msg77 = crate::msg::wsjt77::pack77(call1, call2, report)?;
    Some(synthesize(&message_tones(&msg77), period_s, amplitude))
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

    /// The production tone map against the decoder tests' own copy of
    /// `genmsk_128_90.f90:109-118` (`test_util::build_i4tone`).
    #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
    #[test]
    fn message_tones_match_the_test_oracle() {
        let msg77 = crate::msg::wsjt77::pack77("K1ABC", "W9XYZ", "EN37").unwrap();
        let oracle = super::super::test_util::build_i4tone(&build_bitseq(&codeword(&msg77)));
        assert_eq!(message_tones(&msg77), oracle);
    }

    /// One transmission decodes back to its message through the real-time
    /// receiver, at the time and frequency it was sent.
    #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
    #[test]
    fn a_transmission_decodes_back() {
        let audio = synthesize_standard("K1ABC", "W9XYZ", "R-07", 15, 0.3).unwrap();
        assert_eq!(audio.len(), 15 * 12_000 - 6_000);
        let pcm: Vec<i16> = audio.iter().map(|&x| (x * 32767.0) as i16).collect();
        let got =
            crate::msk144::decode::decode_slot(&pcm, 1500.0, 100.0, crate::decoder::Depth::Deep);
        assert!(
            got.iter()
                .any(|d| d.message == "K1ABC W9XYZ R-07" && (d.freq_hz - CENTER_HZ).abs() < 5.0),
            "{got:?}"
        );
    }
}
