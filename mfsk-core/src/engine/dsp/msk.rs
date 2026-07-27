//! MSK144 OQPSK waveform primitives shared by TX synthesis and RX
//! matched-filter demodulation.
//!
//! MSK144 (2000 baud, 6 samples/symbol @ 12 kHz, 144 symbols = 864
//! samples = 72 ms/frame) is transmitted as offset-QPSK: bits are
//! mapped alternately onto I/Q "rails", each rail bit shaped with a
//! 12-sample half-sine pulse, the Q rail delayed 6 samples relative to
//! I. This is a fundamentally different modulation family from the
//! Gaussian-shaped M-ary FSK the rest of `engine::dsp` targets
//! ([`super::gfsk`]) — MSK144 needs its own primitives rather than
//! reusing GFSK synth/subtract.
//!
//! Ported from the identical shared preamble in WSJT-X
//! `msk144sync.f90:30-54`, `msk144decodeframe.f90:22-46`, and
//! `genmsk_128_90.f90:39-47` — all three build the same half-sine
//! pulse table and sync-word waveform; this module is the single home
//! for them so TX (`genmsk_128_90`), the RX search correlator
//! (`msk144sync`/`msk144_freq_search`), and the per-frame carrier-phase
//! estimator (`msk144decodeframe`) share one definition.

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Samples per MSK144 frame at 12 kHz (144 symbols x 6 samples/symbol).
pub const NSPM: usize = 864;

/// PCM sample rate MSK144 operates at (Hz).
pub const FS_HZ: f32 = 12_000.0;

/// 8-bit MSK144 sync word (natural 0/1 form). Appears twice per
/// 144-bit frame, at symbol offsets 0 and 56 (`msk144sync.f90:27`).
pub const S8: [u8; 8] = [0, 1, 1, 1, 0, 0, 1, 0];

/// Half-sine pulse shape spanning one MSK144 rail-bit duration (12
/// samples @ 12 kHz = 2 MSK symbol periods). `pp[i] = sin(i * pi /
/// 12)` for `i = 0..12` (Fortran `pp(i)=sin((i-1)*pi/12)`,
/// `msk144sync.f90:36-39`).
pub fn half_sine_pulse() -> [f32; 12] {
    let mut pp = [0.0f32; 12];
    for (i, p) in pp.iter_mut().enumerate() {
        *p = (i as f32 * core::f32::consts::PI / 12.0).sin();
    }
    pp
}

/// Bipolar sync word (`2*S8 - 1`, i.e. +-1 values) — matches Fortran's
/// in-place `s8=2*s8-1` (`msk144sync.f90:42`).
pub fn bipolar_sync_word() -> [i8; 8] {
    let mut s8 = [0i8; 8];
    for (i, s) in s8.iter_mut().enumerate() {
        *s = 2 * S8[i] as i8 - 1;
    }
    s8
}

/// Complex sync-word matched-filter template (42 samples): the
/// pulse-shaped OQPSK waveform corresponding to the two embedded 8-bit
/// sync words' worth of I/Q rail bits. Shared byte-for-byte by
/// WSJT-X's `msk144sync.f90` (frequency/timing search correlator) and
/// `msk144decodeframe.f90` (per-frame carrier-phase estimate).
///
/// Real part = I rail, imaginary part = Q rail. Ported from
/// `msk144sync.f90:43-51`:
///
/// ```text
/// cbq(1:6)  = pp(7:12)*s8(1)     cbi(1:12)  = pp*s8(2)
/// cbq(7:18) = pp*s8(3)           cbi(13:24) = pp*s8(4)
/// cbq(19:30)= pp*s8(5)           cbi(25:36) = pp*s8(6)
/// cbq(31:42)= pp*s8(7)           cbi(37:42) = pp(1:6)*s8(8)
/// ```
///
/// (Fortran 1-based indices/subscripts above; the 0-based translation
/// below is covered index-by-index in `tests::sync_waveform_matches_fortran_layout`.)
pub fn sync_waveform() -> [Complex32; 42] {
    let pp = half_sine_pulse();
    let s8 = bipolar_sync_word();

    let mut cbi = [0.0f32; 42];
    let mut cbq = [0.0f32; 42];

    for i in 0..12 {
        cbi[i] = pp[i] * s8[1] as f32;
        cbi[12 + i] = pp[i] * s8[3] as f32;
        cbi[24 + i] = pp[i] * s8[5] as f32;
    }
    for i in 0..6 {
        cbi[36 + i] = pp[i] * s8[7] as f32;
    }

    for i in 0..6 {
        cbq[i] = pp[6 + i] * s8[0] as f32;
    }
    for i in 0..12 {
        cbq[6 + i] = pp[i] * s8[2] as f32;
        cbq[18 + i] = pp[i] * s8[4] as f32;
        cbq[30 + i] = pp[i] * s8[6] as f32;
    }

    let mut cb = [Complex32::new(0.0, 0.0); 42];
    for i in 0..42 {
        cb[i] = Complex32::new(cbi[i], cbq[i]);
    }
    cb
}

/// Build the 144-bit MSK144 channel bit sequence (natural 0/1 form:
/// 8-bit sync word + 48 data bits + 8-bit sync word + 80 data bits)
/// from a 128-bit LDPC(128,90) codeword. Matches `genmsk_128_90.f90:92-96`.
pub fn build_bitseq(codeword: &[u8; 128]) -> [u8; 144] {
    let mut bitseq = [0u8; 144];
    bitseq[0..8].copy_from_slice(&S8);
    bitseq[8..56].copy_from_slice(&codeword[0..48]);
    bitseq[56..64].copy_from_slice(&S8);
    bitseq[64..144].copy_from_slice(&codeword[48..128]);
    bitseq
}

/// Synthesise the 864-sample complex baseband (OQPSK I/Q) frame for a
/// 144-bit natural-0/1 channel bit sequence (as produced by
/// [`build_bitseq`]). This is the complex-baseband equivalent of
/// WSJT-X `genmsk_128_90.f90:97-108` — it stops short of the
/// audio-domain tone synthesis (`i4tone`, lines 109-114), producing
/// the already-downconverted-and-aligned frame a zero-CFO,
/// perfectly-timed receiver would see. Real part = I rail, imaginary
/// part = Q rail.
///
/// The Q rail's first bit (`bitseq[0]`) wraps around the frame
/// boundary: its second pulse-half opens the frame (samples 0..6) and
/// its first pulse-half closes it (samples 858..864) — the inherent
/// OQPSK half-symbol offset between the I and Q rails.
pub fn synth_frame(bitseq: &[u8; 144]) -> [Complex32; NSPM] {
    let pp = half_sine_pulse();
    let mut bp = [0i8; 144];
    for i in 0..144 {
        bp[i] = 2 * bitseq[i] as i8 - 1;
    }

    let mut xi = [0.0f32; NSPM];
    let mut xq = [0.0f32; NSPM];

    // xq(1:6) = bitseq(1)*pp(7:12)  [Fortran 1-based]
    for k in 0..6 {
        xq[k] = bp[0] as f32 * pp[6 + k];
    }
    // do i=1,71: is=(i-1)*12+7; xq(is:is+11)=bitseq(2*i+1)*pp
    for i in 1..=71usize {
        let is0 = (i - 1) * 12 + 6; // 0-based start = Fortran is - 1
        let bit = bp[2 * i] as f32; // bitseq(2*i+1) 1-based -> 0-based index 2*i
        for k in 0..12 {
            xq[is0 + k] = bit * pp[k];
        }
    }
    // xq(864-5:864) = bitseq(1)*pp(1:6)
    for k in 0..6 {
        xq[858 + k] = bp[0] as f32 * pp[k];
    }

    // do i=1,72: is=(i-1)*12+1; xi(is:is+11)=bitseq(2*i)*pp
    for i in 1..=72usize {
        let is0 = (i - 1) * 12; // 0-based start = Fortran is - 1
        let bit = bp[2 * i - 1] as f32; // bitseq(2*i) 1-based -> 0-based index 2*i-1
        for k in 0..12 {
            xi[is0 + k] = bit * pp[k];
        }
    }

    let mut c = [Complex32::new(0.0, 0.0); NSPM];
    for k in 0..NSPM {
        c[k] = Complex32::new(xi[k], xq[k]);
    }
    c
}

/// Recover the 144 matched-filter soft symbols from a received
/// (already time-aligned, arbitrary-constant-phase) 864-sample complex
/// baseband frame. Ported from `msk144decodeframe.f90:51-67`:
///
/// 1. Estimate carrier phase from the two embedded sync words'
///    correlation against [`sync_waveform`] (at symbol offsets 0 and
///    56) and derotate the frame so the constellation lands on the
///    I/Q axes.
/// 2. Matched-filter each rail bit: a half-sine-weighted sum of the
///    derotated frame's imaginary part (Q rail, odd bit positions) or
///    real part (I rail, even bit positions) over its 12-sample
///    window, with the same bit-1 wraparound special case as
///    [`synth_frame`].
///
/// Does **not** handle carrier frequency (CFO) error or unknown timing
/// offset — those are estimated by the sync/burst-scan search (a
/// later phase), which is expected to hand this function an
/// already-aligned, downconverted frame.
pub fn matched_filter_softbits(c: &[Complex32; NSPM]) -> [f32; 144] {
    let pp = half_sine_pulse();
    let cb = sync_waveform();

    // Carrier phase estimate from the two sync-word correlations
    // (msk144decodeframe.f90:52-58).
    let mut cca = Complex32::new(0.0, 0.0);
    let mut ccb = Complex32::new(0.0, 0.0);
    for k in 0..42 {
        cca += c[k] * cb[k].conj();
        ccb += c[336 + k] * cb[k].conj();
    }
    let s = cca + ccb;
    let phase0 = s.im.atan2(s.re);
    let cfac = Complex32::new(phase0.cos(), phase0.sin());

    let mut cr = [Complex32::new(0.0, 0.0); NSPM];
    for k in 0..NSPM {
        cr[k] = c[k] * cfac.conj();
    }

    let mut softbits = [0.0f32; 144];

    // softbits(1): Q rail, bit 1 -- wraps around the frame boundary.
    softbits[0] = (0..6).map(|k| cr[k].im * pp[6 + k]).sum::<f32>()
        + (0..6).map(|k| cr[858 + k].im * pp[k]).sum::<f32>();

    // softbits(2): I rail, symbol 1 (same formula as the i=1 case of
    // the general I-rail loop below, but Fortran special-cases it
    // alongside softbits(1) rather than starting the loop at i=1).
    softbits[1] = (0..12).map(|k| cr[k].re * pp[k]).sum::<f32>();

    for i in 2..=72usize {
        // softbits(2*i-1): Q rail.
        let start_q = (i - 1) * 12 - 6; // 0-based start
        softbits[2 * i - 2] = (0..12).map(|k| cr[start_q + k].im * pp[k]).sum::<f32>();

        // softbits(2*i): I rail.
        let start_i = (i - 1) * 12; // 0-based start
        softbits[2 * i - 1] = (0..12).map(|k| cr[start_i + k].re * pp[k]).sum::<f32>();
    }

    softbits
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn half_sine_pulse_matches_analytic_sin() {
        let pp = half_sine_pulse();
        for (i, &p) in pp.iter().enumerate() {
            let expected = (i as f32 * core::f32::consts::PI / 12.0).sin();
            assert!(
                (p - expected).abs() < 1e-6,
                "pp[{i}] = {p}, expected {expected}"
            );
        }
        // Endpoints of a half-sine over [0, pi): pp[0] = sin(0) = 0,
        // pp[6] = sin(pi/2) = 1 (peak at the midpoint of the 12-sample span).
        assert!((pp[0] - 0.0).abs() < 1e-6);
        assert!((pp[6] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn bipolar_sync_word_matches_fortran() {
        // s8 = [0,1,1,1,0,0,1,0] -> 2*s8-1 = [-1,1,1,1,-1,-1,1,-1]
        assert_eq!(bipolar_sync_word(), [-1, 1, 1, 1, -1, -1, 1, -1]);
    }

    #[test]
    fn sync_waveform_matches_fortran_layout() {
        let pp = half_sine_pulse();
        let cb = sync_waveform();

        // I rail (real part), s8 bipolar = [-1,1,1,1,-1,-1,1,-1] (0-based).
        // cbi(1:12)=pp*s8(2)=pp*1        -> cb[0..12].re  ==  pp
        // cbi(13:24)=pp*s8(4)=pp*1       -> cb[12..24].re ==  pp
        // cbi(25:36)=pp*s8(6)=pp*(-1)    -> cb[24..36].re == -pp
        // cbi(37:42)=pp(1:6)*s8(8)=-pp   -> cb[36..42].re == -pp[0..6]
        for i in 0..12 {
            assert!((cb[i].re - pp[i]).abs() < 1e-6, "cb[{i}].re");
            assert!((cb[12 + i].re - pp[i]).abs() < 1e-6, "cb[{}].re", 12 + i);
            assert!((cb[24 + i].re + pp[i]).abs() < 1e-6, "cb[{}].re", 24 + i);
        }
        for i in 0..6 {
            assert!((cb[36 + i].re + pp[i]).abs() < 1e-6, "cb[{}].re", 36 + i);
        }

        // Q rail (imag part):
        // cbq(1:6)=pp(7:12)*s8(1)=-pp(7:12)  -> cb[0..6].im   == -pp[6..12]
        // cbq(7:18)=pp*s8(3)=pp              -> cb[6..18].im  ==  pp
        // cbq(19:30)=pp*s8(5)=-pp            -> cb[18..30].im == -pp
        // cbq(31:42)=pp*s8(7)=pp             -> cb[30..42].im ==  pp
        for i in 0..6 {
            assert!((cb[i].im + pp[6 + i]).abs() < 1e-6, "cb[{i}].im");
        }
        for i in 0..12 {
            assert!((cb[6 + i].im - pp[i]).abs() < 1e-6, "cb[{}].im", 6 + i);
            assert!((cb[18 + i].im + pp[i]).abs() < 1e-6, "cb[{}].im", 18 + i);
            assert!((cb[30 + i].im - pp[i]).abs() < 1e-6, "cb[{}].im", 30 + i);
        }
    }

    #[test]
    fn frame_constants_match_wsjtx() {
        assert_eq!(NSPM, 864);
        assert_eq!(FS_HZ, 12_000.0);
        assert_eq!(S8, [0, 1, 1, 1, 0, 0, 1, 0]);
    }

    /// Direct-arithmetic check of bit 1 (Q rail)'s frame-boundary
    /// wraparound in isolation: flipping only `bitseq[0]` must flip
    /// the sign of exactly the two wraparound windows
    /// (`c[0..6].im`, `c[858..864].im`) and nothing else, matching
    /// `genmsk_128_90.f90:99,104` (`xq(1:6)=bitseq(1)*pp(7:12)` /
    /// `xq(864-5:864)=bitseq(1)*pp(1:6)`).
    #[test]
    fn synth_frame_wraps_bit_one_across_boundary() {
        let pp = half_sine_pulse();
        let mut bitseq = [1u8; 144]; // all-ones -> all bits bipolar +1
        bitseq[0] = 0; // bit 0 -> bipolar -1

        let c = synth_frame(&bitseq);
        for k in 0..6 {
            assert!(
                (c[k].im + pp[6 + k]).abs() < 1e-6,
                "c[{k}].im (start wrap), got {}",
                c[k].im
            );
            assert!(
                (c[858 + k].im + pp[k]).abs() < 1e-6,
                "c[{}].im (end wrap), got {}",
                858 + k,
                c[858 + k].im
            );
        }

        // Flip bit 0 back to +1: both wraparound windows flip sign,
        // matching bitseq(1) appearing as a common factor in both
        // Fortran expressions.
        bitseq[0] = 1;
        let c2 = synth_frame(&bitseq);
        for k in 0..6 {
            assert!(
                (c2[k].im - pp[6 + k]).abs() < 1e-6,
                "c2[{k}].im (start wrap)"
            );
            assert!(
                (c2[858 + k].im - pp[k]).abs() < 1e-6,
                "c2[{}].im (end wrap)",
                858 + k
            );
        }

        // The rest of the frame (I rail throughout, and the Q rail's
        // 71 non-wraparound windows) must be unaffected by bit 0.
        for k in 6..858 {
            assert!(
                (c[k].im - c2[k].im).abs() < 1e-6,
                "c[{k}].im should not depend on bit 0"
            );
        }
        for k in 0..NSPM {
            assert!(
                (c[k].re - c2[k].re).abs() < 1e-6,
                "c[{k}].re (I rail) should not depend on bit 0"
            );
        }
    }
}
