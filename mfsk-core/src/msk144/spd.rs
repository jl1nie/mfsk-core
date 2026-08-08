//! MSK144 short-ping burst detector: scans a multi-frame analytic-
//! signal buffer for transient (meteor-scatter ping) energy via a
//! squared-signal two-tone spectral search, producing a list of
//! burst-start candidates with frequency-error and SNR estimates.
//!
//! Ported from WSJT-X `msk144spd.f90` in full: the detection portion
//! (`msk144spd.f90:64-153`, [`detect_burst_candidates`]) plus the
//! "sync/demod/decode each candidate" outer loop
//! (`msk144spd.f90:155-193`, [`short_ping_decode`]) that ties it to
//! [`crate::msk144::sync`] and [`crate::msk144::frame_decode`].
//!
//! MSK144 is nominally 2-FSK at ±500 Hz deviation about a center
//! frequency `fc`; squaring the signal doubles every frequency
//! component, so a clean tone at `fc+500` becomes a spectral line at
//! `2*(fc+500)`, and likewise `fc-500` -> `2*(fc-500)` — a
//! non-coherent, phase-independent way to detect a burst and estimate
//! its frequency without needing symbol-level sync first.

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::DecodeContext;
use crate::engine::dsp::msk::{NSPM, matched_filter_softbits};
use crate::engine::fft::default_planner;
use crate::msk144::frame_decode::decode_frame;
use crate::msk144::sync::{msk144_sync, rotate_to_shift};

const NFFT: usize = NSPM; // 864
const MAXCAND: usize = 5;
const STEP: usize = 216; // NSPM/4, 18 ms @ 12 kHz

/// The 3-frame averaging masks `msk144spd.f90:35-41` tries per
/// candidate (`navpatterns`): single frames, pairs, and all three.
const NAVPATTERNS: [[bool; 3]; 6] = [
    [false, true, false],
    [true, false, false],
    [false, false, true],
    [true, true, false],
    [false, true, true],
    [true, true, true],
];

/// Raised-cosine edge window (`rcw` in `msk144spd.f90:56-59`):
/// `rcw[k] = (1 - cos(k*pi/12)) / 2` for `k = 0..12`.
fn raised_cosine_window() -> [f32; 12] {
    let mut rcw = [0.0f32; 12];
    for (k, r) in rcw.iter_mut().enumerate() {
        *r = (1.0 - (k as f32 * core::f32::consts::PI / 12.0).cos()) / 2.0;
    }
    rcw
}

/// argmax of `tonespec` over the (possibly bin-wrapping) window
/// `[lo, hi]` inclusive. Returns `(peak bin, peak value, average
/// power over the window including the peak bin)` — WSJT-X divides
/// by the *full* window count even though the numerator excludes the
/// peak (`msk144spd.f90:99`: `(sum(tonespec,ismask)-ah)/count(ismask)`),
/// ported literally rather than "corrected" to `count-1`.
fn peak_in_window(tonespec: &[f32], lo: isize, hi: isize, len: usize) -> (usize, f32, f32) {
    let mut best_idx = 0usize;
    let mut best_val = f32::MIN;
    let mut sum = 0.0f32;
    let mut count = 0usize;
    let mut b = lo;
    while b <= hi {
        let idx = b.rem_euclid(len as isize) as usize;
        let v = tonespec[idx];
        sum += v;
        count += 1;
        if v > best_val {
            best_val = v;
            best_idx = idx;
        }
        b += 1;
    }
    let avg = if count > 0 {
        (sum - best_val) / count as f32
    } else {
        0.0
    };
    (best_idx, best_val, avg)
}

/// Sub-bin parabolic interpolation using the complex spectrum values
/// around `peak_bin` directly (not just their magnitudes). Matches
/// `msk144spd.f90:97`:
/// `deltah = -real((ctmp(ihpk-1)-ctmp(ihpk+1)) / (2*ctmp(ihpk)-ctmp(ihpk-1)-ctmp(ihpk+1)))`.
fn parabolic_delta(ctmp: &[Complex32], peak_bin: usize, len: usize) -> f32 {
    let im1 = (peak_bin + len - 1) % len;
    let ip1 = (peak_bin + 1) % len;
    let num = ctmp[im1] - ctmp[ip1];
    let den = ctmp[peak_bin] * 2.0 - ctmp[im1] - ctmp[ip1];
    -(num / den).re
}

/// One burst-start candidate.
#[derive(Clone, Copy, Debug)]
pub struct BurstCandidate {
    /// 0-based sample index (within the scanned buffer) marking the
    /// start of the `NSPM`-sample analysis window that triggered this
    /// candidate. `msk144spd.f90:161-162`'s caller backs up a full
    /// frame and takes a 3-frame window centered near here for the
    /// sync/decode attempt (a later phase).
    pub start_sample: usize,
    /// Frequency error estimate (Hz) relative to the nominal `fc`.
    pub freq_err_hz: f32,
    /// SNR estimate (dB): WSJT-X's `12*log10(detmet)/2 - 9` /
    /// `12*log10(detmet2)/2 - 9` formula.
    pub snr_db: f32,
}

/// Scan `cbig` (an analytic-signal buffer at the native 12 kHz rate,
/// still at its passband frequency — *not* baseband) for burst-start
/// candidates near center frequency `fc`, within `±ntol` Hz.
///
/// Ported from `msk144spd.f90:64-153`. Returns up to 5 (`MAXCAND`)
/// candidates, strongest-first within each of the two detection
/// stages (the `detmet` peak-power stage, then — only if fewer than 3
/// candidates were found — the `detmet2` peak-to-local-average stage).
pub fn detect_burst_candidates(cbig: &[Complex32], fc: f32, ntol: f32) -> Vec<BurstCandidate> {
    let n = cbig.len();
    if n < NSPM {
        return Vec::new();
    }

    let rcw = raised_cosine_window();
    let fs = 12_000.0f32;
    let df = fs / NFFT as f32;

    let nfhi = 2.0 * (fc + 500.0);
    let nflo = 2.0 * (fc - 500.0);
    // Fortran 1-based bin index -> 0-based: drop the literal "+1".
    let ihlo = ((nfhi - 2.0 * ntol) / df).round() as isize;
    let ihhi = ((nfhi + 2.0 * ntol) / df).round() as isize;
    let illo = ((nflo - 2.0 * ntol) / df).round() as isize;
    let ilhi = ((nflo + 2.0 * ntol) / df).round() as isize;
    let i2000 = (nflo / df).round() as isize;
    let i4000 = (nfhi / df).round() as isize;

    let nstep = (n - NSPM) / STEP;
    if nstep == 0 {
        return Vec::new();
    }

    let mut planner = default_planner();
    let fft = planner.plan_forward(NFFT);

    let mut detmet = vec![0.0f32; nstep];
    let mut detmet2 = vec![0.0f32; nstep];
    let mut detfer = vec![-999.99f32; nstep];

    // `ctmp`/`tonespec` reused across every offset in the scan below
    // instead of allocated fresh per offset — `NFFT == NSPM`, so both
    // are fixed-size for the whole loop. Same allocation-in-hot-loop
    // class as `engine::sync::coarse_sync`'s `fill_sync2d_row!` fix;
    // smaller magnitude here (2 allocs/offset vs. 4 for
    // `msk144_freq_search`'s CFO loop) but the same pattern.
    let mut ctmp: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); NSPM];
    let mut tonespec: Vec<f32> = vec![0.0f32; NFFT];

    for (istp, (dm, (dm2, df_))) in detmet
        .iter_mut()
        .zip(detmet2.iter_mut().zip(detfer.iter_mut()))
        .enumerate()
    {
        let ns = istp * STEP;
        let ne = ns + NSPM;
        if ne > n {
            break;
        }

        ctmp.copy_from_slice(&cbig[ns..ne]);
        for c in ctmp.iter_mut() {
            *c *= *c;
        }
        for k in 0..12 {
            ctmp[k] *= rcw[k];
        }
        for k in 0..12 {
            ctmp[NSPM - 12 + k] *= rcw[11 - k];
        }

        fft.process(&mut ctmp);
        for (t, c) in tonespec.iter_mut().zip(ctmp.iter()) {
            *t = c.norm_sqr();
        }

        let (ihpk, ah, ahavp) = peak_in_window(&tonespec, ihlo, ihhi, NFFT);
        let deltah = parabolic_delta(&ctmp, ihpk, NFFT);
        let trath = ah / (ahavp + 0.01);

        let (ilpk, al, alavp) = peak_in_window(&tonespec, illo, ilhi, NFFT);
        let deltal = parabolic_delta(&ctmp, ilpk, NFFT);
        let tratl = al / (alavp + 0.01);

        let ferrh = (ihpk as f32 + deltah - i4000 as f32) * df / 2.0;
        let ferrl = (ilpk as f32 + deltal - i2000 as f32) * df / 2.0;

        *dm = ah.max(al);
        *dm2 = trath.max(tratl);
        *df_ = if ah >= al { ferrh } else { ferrl };
    }

    // Noise floor: WSJT-X picks the value at ascending-sorted position
    // `nstep/4` (1-based) — a low quantile, not the median despite the
    // comment (`msk144spd.f90:122-124`). Only that single order
    // statistic is needed, not a full ascending order —
    // `select_nth_unstable_by` finds it in O(n) average instead of
    // `sort_by`'s O(n log n), same class of fix applied to
    // JT65/JT9/Q65's `Spectrogram::build`.
    let mut sorted = detmet.clone();
    let pos0 = (nstep / 4).saturating_sub(1);
    sorted.select_nth_unstable_by(pos0, |a, b| a.partial_cmp(b).unwrap());
    let xmed = sorted[pos0].max(1e-9);
    for v in detmet.iter_mut() {
        *v /= xmed;
    }

    let mut candidates = Vec::new();

    let mut stage1 = detmet.clone();
    for _ in 0..MAXCAND {
        let (il, &val) = stage1
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .expect("nstep > 0");
        if val < 3.0 {
            break;
        }
        if detfer[il].abs() <= ntol {
            candidates.push(BurstCandidate {
                start_sample: il * STEP,
                freq_err_hz: detfer[il],
                snr_db: 12.0 * val.log10() / 2.0 - 9.0,
            });
        }
        stage1[il] = 0.0;
    }

    if candidates.len() < 3 {
        let mut stage2 = detmet2.clone();
        for _ in 0..(MAXCAND - candidates.len()) {
            let (il, &val) = stage2
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .expect("nstep > 0");
            if val < 12.0 {
                break;
            }
            if detfer[il].abs() <= ntol {
                candidates.push(BurstCandidate {
                    start_sample: il * STEP,
                    freq_err_hz: detfer[il],
                    snr_db: 12.0 * val.log10() / 2.0 - 9.0,
                });
            }
            stage2[il] = 0.0;
        }
    }

    candidates
}

/// Result of a successful [`short_ping_decode`].
#[derive(Clone, Debug)]
pub struct ShortPingDecode {
    pub message: String,
    pub info77: [u8; 77],
    /// Refined frequency estimate (Hz) from the winning sync trial.
    pub fest: f32,
    /// Decode time (seconds, relative to the start of `cbig`):
    /// `(candidate.start_sample + NSPM/2) / 12000`, matching
    /// `msk144spd.f90:184`'s `tret`.
    pub tdec_sec: f32,
    /// Number of frames coherently averaged for the winning trial (1-3).
    pub navg: usize,
}

/// Try to sync, demodulate, and decode each burst candidate found by
/// [`detect_burst_candidates`]. For each candidate, backs up one frame
/// and takes a 3-frame window around it (`msk144spd.f90:161-166`),
/// then tries each of the 6 `NAVPATTERNS` frame-averaging masks,
/// each of up to 2 sync peaks, and 3 slicer-dither offsets per peak —
/// returning on the first successful decode. Ported from
/// `msk144spd.f90:155-193`.
pub fn short_ping_decode(
    cbig: &[Complex32],
    fc: f32,
    ntol: f32,
    ctx: &DecodeContext,
) -> Option<ShortPingDecode> {
    const NPEAKS: usize = 2;
    const NTOL0: f32 = 8.0;
    const DELTAF: f32 = 2.0;

    for cand in detect_burst_candidates(cbig, fc, ntol) {
        let ib = cand.start_sample.saturating_sub(NSPM);
        let ie = (ib + 3 * NSPM).min(cbig.len());
        let ib = ie.saturating_sub(3 * NSPM);
        if ie - ib < 3 * NSPM {
            continue;
        }
        let window = &cbig[ib..ie];
        let fo = fc + cand.freq_err_hz;

        for navmask in NAVPATTERNS {
            let sync_result = msk144_sync(window, 3, NTOL0, DELTAF, &navmask, NPEAKS, fo);
            if !sync_result.success {
                continue;
            }
            for peak in &sync_result.peaks {
                for dither in 0..3 {
                    let ic0 = match dither {
                        0 => peak.shift,
                        1 => peak.shift.saturating_sub(1),
                        _ => (peak.shift + 1).min(NSPM - 1),
                    };
                    let aligned = rotate_to_shift(&sync_result.frame, ic0);
                    let softbits = matched_filter_softbits(
                        aligned.as_slice().try_into().expect("NSPM samples"),
                    );
                    if let Some(result) = decode_frame(&softbits, ctx) {
                        return Some(ShortPingDecode {
                            message: result.message,
                            info77: result.info77,
                            fest: sync_result.fest,
                            tdec_sec: (cand.start_sample as f32 + NSPM as f32 / 2.0) / 12_000.0,
                            navg: navmask.iter().filter(|&&b| b).count(),
                        });
                    }
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::FecCodec;
    use crate::engine::dsp::msk::{build_bitseq, synth_frame};
    use crate::fec::Ldpc128_90;
    use crate::msk144::sync::tweak1;

    fn synth_clean_frame(pattern: impl Fn(usize) -> u8) -> Vec<Complex32> {
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
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);
        synth_frame(&bitseq).to_vec()
    }

    /// Deterministic complex Gaussian-ish noise: a seeded xorshift32
    /// PRNG feeding a Box-Muller transform. Reproducible (no external
    /// `rand` dependency, matching this crate's convention) but, unlike
    /// a plain sine-based hash, has genuinely flat/white spectral
    /// statistics — needed here because [`detect_burst_candidates`]
    /// runs an FFT-based spectral-peak search that a tonal fake-noise
    /// generator would trip on by construction.
    fn pseudo_gaussian_noise(n: usize, amp: f32, seed: u32) -> Vec<Complex32> {
        let mut state = seed | 1;
        let mut next_u32 = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        let mut uniform = move || (next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
        (0..n)
            .map(|_| {
                let u1 = uniform();
                let u2 = uniform();
                let r = (-2.0 * u1.ln()).sqrt();
                let theta = 2.0 * core::f32::consts::PI * u2;
                Complex32::new(amp * r * theta.cos(), amp * r * theta.sin())
            })
            .collect()
    }

    /// A single clean burst, upconverted to a known passband
    /// frequency, embedded in an otherwise-quiet 8-frame buffer (the
    /// size `mskrtd.f90` scans per step): the detector should report a
    /// candidate whose `start_sample` is within one frame of the true
    /// burst position and whose `freq_err_hz` is small.
    #[test]
    fn detects_clean_burst_near_true_position_and_frequency() {
        let frame = synth_clean_frame(|i| ((i * 7 + 3) & 1) as u8);
        let fc_true = 1500.0f32;
        let burst = tweak1(&frame, fc_true);

        let total = 8 * NSPM;
        let mut cbig = pseudo_gaussian_noise(total, 0.05, 12345);
        let true_start = 3 * NSPM + 100; // arbitrary offset within the buffer
        for (k, &s) in burst.iter().enumerate() {
            cbig[true_start + k] += s;
        }

        let candidates = detect_burst_candidates(&cbig, fc_true, 8.0);
        assert!(!candidates.is_empty(), "expected at least one candidate");

        let best = candidates[0];
        let dist = (best.start_sample as isize - true_start as isize).unsigned_abs();
        assert!(
            dist < NSPM,
            "candidate start {} too far from true burst start {}",
            best.start_sample,
            true_start
        );
        assert!(
            best.freq_err_hz.abs() <= 8.0,
            "freq_err_hz = {} out of tolerance",
            best.freq_err_hz
        );
    }

    /// This is a detection-*only* stage: WSJT-X's own design tolerates
    /// candidates arising from noise alone here, relying on the
    /// downstream sync + CRC-verified decode attempt (a later phase)
    /// to reject the false ones — so "zero candidates in noise" is a
    /// stronger claim than the algorithm actually makes. What must
    /// hold instead is discriminative power: a real burst's strongest
    /// candidate should clearly outrank whatever the *same* noise
    /// realization (no burst added) produces on its own.
    #[test]
    fn burst_candidate_outranks_same_noise_without_it() {
        let noise_seed = 7u32;
        let noise_only = pseudo_gaussian_noise(8 * NSPM, 0.3, noise_seed);
        let noise_best_snr = detect_burst_candidates(&noise_only, 1500.0, 8.0)
            .iter()
            .map(|c| c.snr_db)
            .fold(f32::MIN, f32::max);

        let frame = synth_clean_frame(|i| ((i * 5 + 1) & 1) as u8);
        let burst = tweak1(&frame, 1500.0);
        let mut with_burst = pseudo_gaussian_noise(8 * NSPM, 0.3, noise_seed);
        let true_start = 3 * NSPM;
        for (k, &s) in burst.iter().enumerate() {
            with_burst[true_start + k] += s;
        }
        let burst_best_snr = detect_burst_candidates(&with_burst, 1500.0, 8.0)
            .iter()
            .map(|c| c.snr_db)
            .fold(f32::MIN, f32::max);

        assert!(
            burst_best_snr > noise_best_snr + 3.0,
            "burst_best_snr={burst_best_snr}, noise_best_snr={noise_best_snr}"
        );
    }

    /// Independent WSJT-X-style reference synthesizer, mirroring
    /// `msk144sim.f90:52-76` (simple continuous-phase binary FSK at
    /// ±500 Hz deviation, driven by the *tone* sequence `itone` — not
    /// this crate's own OQPSK/complex-baseband
    /// `crate::engine::dsp::msk::synth_frame`). Used only to build a
    /// test signal whose generation shares no code with the
    /// `synth_frame`/`matched_filter_softbits` pair under test, so a
    /// bug shared between TX and RX can't silently cancel out in a
    /// same-code roundtrip.
    fn msk144sim_reference_audio(itone: &[u8], freq_hz: f32) -> Vec<f32> {
        let twopi = 2.0 * core::f32::consts::PI;
        let baud = 2000.0f32;
        let dphi0 = twopi * (freq_hz - 0.25 * baud) / 12_000.0;
        let dphi1 = twopi * (freq_hz + 0.25 * baud) / 12_000.0;
        let mut phi = 0.0f32;
        let mut out = Vec::with_capacity(144 * 6);
        for &tone in itone {
            let dphi = if tone == 0 { dphi0 } else { dphi1 };
            for _ in 0..6 {
                out.push(phi.cos());
                phi = (phi + dphi) % twopi;
            }
        }
        out
    }

    /// The `itone` audio-tone sequence msk144sim/genmsk_128_90 actually
    /// transmits is *not* the raw channel bit sequence — MSK's
    /// continuous-phase property means the instantaneous tone at each
    /// symbol depends on the product of adjacent bipolar rail bits.
    /// Ported from `genmsk_128_90.f90:109-114,118`.
    fn build_i4tone(bitseq_natural: &[u8; 144]) -> [u8; 144] {
        let mut bp = [0i8; 144];
        for i in 0..144 {
            bp[i] = 2 * bitseq_natural[i] as i8 - 1;
        }
        let mut i4tone = [0i8; 144];
        for i in 1..=72usize {
            let b_2i_minus_1 = bp[2 * i - 2]; // bitseq(2*i-1)
            let b_2i = bp[2 * i - 1]; // bitseq(2*i)
            let b_wrap = bp[(2 * i) % 144]; // bitseq(mod(2*i,144)+1)
            i4tone[2 * i - 2] = (b_2i * b_2i_minus_1 + 1) / 2; // i4tone(2*i-1)
            i4tone[2 * i - 1] = -((b_2i * b_wrap - 1) / 2); // i4tone(2*i)
        }
        let mut out = [0u8; 144];
        for i in 0..144 {
            out[i] = (-i4tone[i] + 1) as u8; // "Flip polarity": i4tone=-i4tone+1
        }
        out
    }

    /// The full loop, closed with a genuinely independent oracle:
    /// message -> codeword -> channel bit sequence (production code,
    /// shared with the real TX path) -> `itone` tone sequence ->
    /// simple binary-FSK real-audio synthesis (independent of
    /// `synth_frame`) -> FFT-based analytic-signal construction
    /// (independent of the matched filter) -> the actual sync search
    /// and OQPSK matched filter under test -> recovered bits must
    /// match what was transmitted.
    #[test]
    fn independent_fsk_oracle_decodes_through_full_rx_chain() {
        let mut info = [0u8; 90];
        for i in 0..77 {
            info[i] = ((i * 11 + 2) & 1) as u8;
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
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);

        let fc_true = 1500.0f32;
        let itone = build_i4tone(&bitseq);
        let audio = msk144sim_reference_audio(&itone, fc_true);
        let analytic = crate::engine::dsp::analytic_signal(&audio);
        assert_eq!(analytic.len(), NSPM);

        let navmask = [true];
        let result = crate::msk144::sync::msk144_sync(&analytic, 1, 8.0, 2.0, &navmask, 2, fc_true);
        assert!(result.success, "xmax = {}", result.xmax);

        let aligned = crate::msk144::sync::rotate_to_shift(&result.frame, result.peaks[0].shift);
        let softbits = crate::engine::dsp::msk::matched_filter_softbits(
            aligned.as_slice().try_into().expect("NSPM samples"),
        );
        for i in 0..144 {
            assert_eq!(
                softbits[i] > 0.0,
                bitseq[i] == 1,
                "softbit {i} mismatch against independent FSK oracle"
            );
        }
    }

    /// Phase 3 + Phase 4 capstone: a real standard message, synthesized
    /// via the independent FSK oracle, embedded at an unknown position
    /// inside a multi-frame noisy buffer -- burst detection, sync
    /// search, matched filter, and frame decode must all compose
    /// correctly end to end, with no step told in advance where the
    /// burst is or what CFO it carries.
    #[test]
    fn full_chain_detects_syncs_and_decodes_a_real_message() {
        use crate::msg::wsjt77;

        let msg77 = wsjt77::pack77("K1ABC", "W9XYZ", "EN37").expect("valid standard message");
        let mut info = [0u8; 90];
        info[..77].copy_from_slice(&msg77);
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
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);

        // Real MSK144 transmits the same 864-sample frame continuously
        // for the whole T/R period (`msk144sim.f90`'s `nreps` loop) --
        // not a single isolated 864-sample shot -- which is what lets
        // `msk144_sync`'s multi-frame coherent averaging make sense in
        // the first place. Simulate a several-repetition ping so an
        // arbitrary 3-frame analysis window has a real chance of
        // landing on repeat content regardless of exactly where the
        // (216-sample-quantized) detector places it.
        const NREPS: usize = 6;
        let fc_true = 1500.0f32;
        let itone = build_i4tone(&bitseq);
        let itone_repeated: alloc::vec::Vec<u8> =
            itone.iter().copied().cycle().take(144 * NREPS).collect();
        let audio = msk144sim_reference_audio(&itone_repeated, fc_true);
        let analytic = crate::engine::dsp::analytic_signal(&audio);

        let total = 8 * NSPM;
        let mut cbig = pseudo_gaussian_noise(total, 0.05, 999);
        let true_start = NSPM + 300; // unknown to the detector
        for (k, &s) in analytic.iter().enumerate() {
            cbig[true_start + k] += s;
        }

        let ctx = DecodeContext::default();
        let decoded = short_ping_decode(&cbig, fc_true, 8.0, &ctx)
            .expect("short_ping_decode should find and decode the embedded message");
        assert!(decoded.message.contains("K1ABC"));
        assert!(decoded.message.contains("W9XYZ"));
        assert!(decoded.message.contains("EN37"));
    }
}
