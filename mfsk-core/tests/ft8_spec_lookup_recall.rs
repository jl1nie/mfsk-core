//! Phase 1.7.7-Stick host validation — end-to-end **recall** comparison
//! of BASIS-based vs spec-lookup pass-2 fill, on the qso3_busy.wav
//! reference (WSJT-X AP-off golden = 8 entries).
//!
//! Both paths start from the SAME coarse_sync candidate list (so pass-1
//! is held constant). The difference is only the pass-2 + BP per-cell
//! cs fill algorithm:
//!   - BASIS: existing `fill_symbol_spectra` (per-symbol direct DFT)
//!   - SPEC LOOKUP: complex spec at NSTEP-spaced cols × tone bins
//!     with sub-NSTEP DT phase correction (re-uses the slot-wide
//!     3840-pt FFT, zero extra scratch on embedded)
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_spec_lookup_recall -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]
#![allow(
    clippy::needless_range_loop,
    clippy::manual_memcpy,
    clippy::explicit_auto_deref,
    clippy::unnecessary_sort_by
)]

use mfsk_core::core::scalar::Cmplx;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode::DecodeResult;
#[cfg(feature = "fixed-point")]
use mfsk_core::ft8::decode_block::{BASIS_SCRATCH_LEN, fill_symbol_spectra_into};
use mfsk_core::ft8::decode_block::{
    DEFAULT_Q_THRESH, RefinedCandidate, SymMask, coarse_sync_with_allsum, compute_spectrogram,
    fill_symbol_spectra, precompute_coarse_allsum,
    process_candidates_into_with_cs_scratch_tuned_with_fill, sync_quality_block0,
};
use mfsk_core::ft8::params::{NSPS, NSTEP, NTONES};
use mfsk_core::msg::wsjt77::unpack77;
use num_complex::Complex;
use rustfft::FftPlanner;
use std::collections::BTreeSet;

const NN: usize = 79;
const NFFT_SPEC: usize = 3840;
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const TONE_SPACING_HZ: f32 = 6.25;
const TX_START_OFFSET_S: f32 = 0.5;
const NMAX: usize = 180_000;

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let mut i = 12usize;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len.min(bytes.len() - i);
        }
        i += len + if len % 2 == 1 { 1 } else { 0 };
    }
    bytes[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

fn build_complex_spectrogram(audio: &[i16], max_freq_hz: f32) -> (Vec<Complex<f32>>, usize, usize) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NFFT_SPEC);

    let mut data = vec![Complex::new(0.0_f32, 0.0); n_freq * n_time];
    let mut buf = vec![Complex::new(0.0_f32, 0.0); NFFT_SPEC];

    for j in 0..n_time {
        let ia = j * NSTEP;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < NSPS {
                let sample = if ia + k < audio.len() {
                    (audio[ia + k] as f32) * scale
                } else {
                    0.0
                };
                Complex::new(sample, 0.0)
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        let row_base = j * n_freq;
        for i in 0..n_freq {
            data[row_base + i] = buf[i];
        }
    }
    (data, n_freq, n_time)
}

/// Mirror of `fill_symbol_spectra_generic` but using spec lookup +
/// DT phase correction. `mask` controls which symbols get filled
/// (others are left untouched, matching the BASIS path's contract).
fn fill_from_complex_spec(
    out: &mut [[Cmplx<f32>; 8]; 79],
    spec: &[Complex<f32>],
    n_freq: usize,
    n_time: usize,
    freq_hz: f32,
    dt_sec: f32,
    mask: SymMask,
) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let bin_step = (TONE_SPACING_HZ / df).round() as i32; // = 2 for FT8
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let bin0 = (freq_hz / df).round() as i32;
    let two_pi_over_nfft = -2.0_f32 * core::f32::consts::PI / NFFT_SPEC as f32;

    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let target_start: i64 = i0 + (sym as i64) * (NSPS as i64);
        let col_frac = target_start as f64 / NSTEP as f64;
        let col_int = col_frac.round() as i64;
        let delta_samples: i64 = target_start - col_int * (NSTEP as i64);

        if col_int < 0 || col_int >= n_time as i64 {
            for tone in 0..NTONES {
                out[sym][tone] = Complex::new(0.0, 0.0);
            }
            continue;
        }
        let col_int = col_int as usize;

        for tone in 0..NTONES {
            let bin = bin0 + (tone as i32) * bin_step;
            if bin < 0 || (bin as usize) >= n_freq {
                out[sym][tone] = Complex::new(0.0, 0.0);
                continue;
            }
            let bin_u = bin as usize;
            // Compensate /300 spec scaling so magnitudes match BASIS.
            let cell = spec[col_int * n_freq + bin_u] * 300.0;
            let theta = two_pi_over_nfft * (delta_samples as f32) * (bin as f32);
            let phase = Complex::from_polar(1.0, theta);
            out[sym][tone] = cell * phase;
        }
    }
}

fn sym_in_mask(sym: usize, mask: SymMask) -> bool {
    // FT8 Costas blocks at [0..7], [36..43], [72..79].
    let in_block_a = sym < 7;
    let in_block_b = (36..43).contains(&sym);
    let in_block_c = (72..79).contains(&sym);
    let is_sync = in_block_a || in_block_b || in_block_c;
    match mask {
        SymMask::SyncOnly => is_sync,
        SymMask::SyncBlock0 => in_block_a,
        SymMask::NotBlock0 => !in_block_a,
        SymMask::DataOnly => !is_sync,
        SymMask::SyncBlocks12 => in_block_b || in_block_c,
    }
}

/// Build RefinedCandidate Vec (= pass2 output) from SyncCandidates,
/// using the supplied `cs_for` closure to compute each candidate's
/// SyncBlock0 cs box. Mirrors private `refine_candidates_with`.
fn refine_candidates_pub<F>(
    cands: Vec<mfsk_core::core::sync::SyncCandidate>,
    max_cand: usize,
    mut cs_for: F,
) -> Vec<RefinedCandidate>
where
    F: FnMut(&mfsk_core::core::sync::SyncCandidate) -> Box<[[Cmplx<f32>; 8]; 79]>,
{
    use std::cmp::{Ordering, Reverse};
    use std::collections::BinaryHeap;

    struct Slot {
        q: u32,
        idx: u32,
        cand: mfsk_core::core::sync::SyncCandidate,
        cs: Box<[[Cmplx<f32>; 8]; 79]>,
    }
    impl PartialEq for Slot {
        fn eq(&self, o: &Self) -> bool {
            self.q == o.q && self.idx == o.idx
        }
    }
    impl Eq for Slot {}
    impl Ord for Slot {
        fn cmp(&self, o: &Self) -> Ordering {
            self.q.cmp(&o.q).then_with(|| self.idx.cmp(&o.idx))
        }
    }
    impl PartialOrd for Slot {
        fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
            Some(self.cmp(o))
        }
    }

    let mut heap: BinaryHeap<Reverse<Slot>> = BinaryHeap::with_capacity(max_cand + 1);
    for (idx, c) in cands.into_iter().enumerate() {
        let cs = cs_for(&c);
        let q = sync_quality_block0(&cs);
        let slot = Slot {
            q,
            idx: idx as u32,
            cand: c,
            cs,
        };
        if heap.len() < max_cand {
            heap.push(Reverse(slot));
        } else if let Some(Reverse(top)) = heap.peek()
            && slot.q > top.q
        {
            heap.pop();
            heap.push(Reverse(slot));
        }
    }
    let mut out: Vec<_> = heap
        .into_iter()
        .map(|Reverse(s)| (s.cand, s.cs, s.q))
        .collect();
    out.sort_by(|a, b| b.2.cmp(&a.2));
    out
}

fn decode_messages(results: &[DecodeResult]) -> Vec<String> {
    results
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect()
}

#[test]
fn recall_basis_vs_spec_lookup() {
    println!("\n=== Phase 1.7.7-Stick: BASIS vs spec-lookup recall ===\n");
    let audio = load_wav_i16(QSO3_PATH);
    println!(
        "audio.len() = {} samples ({:.2} s)",
        audio.len(),
        audio.len() as f32 / SAMPLE_RATE_HZ
    );

    // Pass 1 — coarse_sync (shared between both paths)
    let spec_mag = compute_spectrogram(&audio, 3000.0);
    // Match decode_block behaviour: single full-band coarse_sync_with_allsum
    // (NOT split). Embedded splits via dual_core for parallelism but
    // produces equivalent candidate set; we keep the simple unified
    // call here to mirror host `decode_block` exactly.
    let allsum = precompute_coarse_allsum(&spec_mag, 100.0, 3000.0);
    let cands = coarse_sync_with_allsum(&spec_mag, 100.0, 3000.0, 1.0, 30, &allsum);
    println!("coarse_sync: {} candidates", cands.len());

    // Build complex spec for the spec-lookup path
    let (spec_cplx, n_freq, n_time) = build_complex_spectrogram(&audio, 3000.0);
    assert_eq!(n_freq, spec_mag.n_freq);
    assert_eq!(n_time, spec_mag.n_time);

    // ── Path A: BASIS pass2 + BASIS BP (embedded-equivalent) ────────
    //
    // When `fixed-point` is enabled this uses `fill_symbol_spectra_into`
    // = the BASIS sin/cos dot-product path (bit-identical to embedded
    // M5StickS3 / Core2 numerical behaviour). When `fixed-point` is
    // disabled this falls back to the via_cd0 host path through
    // `fill_symbol_spectra` (rustfft cd0 + 32-pt FFT).
    #[cfg(feature = "fixed-point")]
    let mut basis_re_a = vec![0i16; BASIS_SCRATCH_LEN];
    #[cfg(feature = "fixed-point")]
    let mut basis_im_a = vec![0i16; BASIS_SCRATCH_LEN];

    let cands_a = cands.clone();
    let refined_a = refine_candidates_pub(cands_a, 15, |c| {
        let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        #[cfg(feature = "fixed-point")]
        {
            fill_symbol_spectra_into::<i16>(
                &mut *cs,
                &audio,
                c.freq_hz,
                c.dt_sec,
                SymMask::SyncBlock0,
                &mut basis_re_a,
                &mut basis_im_a,
            );
        }
        #[cfg(not(feature = "fixed-point"))]
        fill_symbol_spectra::<i16>(
            &mut cs,
            &audio,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
            None,
        );
        cs
    });
    let mut cs_a: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
    let results_a = process_candidates_into_with_cs_scratch_tuned_with_fill(
        &audio,
        refined_a,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        30,
        &mut cs_a,
        |cs, cand, mask| {
            #[cfg(feature = "fixed-point")]
            fill_symbol_spectra_into::<i16>(
                cs,
                &audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                &mut basis_re_a,
                &mut basis_im_a,
            );
            #[cfg(not(feature = "fixed-point"))]
            fill_symbol_spectra::<i16>(cs, &audio, cand.freq_hz, cand.dt_sec, mask, None);
        },
    );

    // ── Path B: SPEC-LOOKUP pass2 + SPEC-LOOKUP BP ───────────────────
    let cands_b = cands.clone();
    let refined_b = refine_candidates_pub(cands_b, 15, |c| {
        let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        fill_from_complex_spec(
            &mut cs,
            &spec_cplx,
            n_freq,
            n_time,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
        );
        cs
    });
    let mut cs_b: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
    let results_b = process_candidates_into_with_cs_scratch_tuned_with_fill(
        &audio,
        refined_b,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        30,
        &mut cs_b,
        |cs, cand, mask| {
            fill_from_complex_spec(
                cs,
                &spec_cplx,
                n_freq,
                n_time,
                cand.freq_hz,
                cand.dt_sec,
                mask,
            );
        },
    );

    // ── Compare ──────────────────────────────────────────────────────
    println!("\nBASIS path:        {} raw DecodeResults", results_a.len());
    for r in &results_a {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }
    println!("\nSPEC-LOOKUP path:  {} raw DecodeResults", results_b.len());
    for r in &results_b {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }

    let msgs_a: BTreeSet<String> = decode_messages(&results_a).into_iter().collect();
    let msgs_b: BTreeSet<String> = decode_messages(&results_b).into_iter().collect();

    let intersection: BTreeSet<_> = msgs_a.intersection(&msgs_b).collect();
    let only_a: BTreeSet<_> = msgs_a.difference(&msgs_b).collect();
    let only_b: BTreeSet<_> = msgs_b.difference(&msgs_a).collect();

    println!("\nSummary:");
    println!("  shared:        {}", intersection.len());
    println!("  only in BASIS: {}  ({:?})", only_a.len(), only_a);
    println!("  only in LOOK:  {}  ({:?})", only_b.len(), only_b);

    // We don't assert exact equality (the algorithms differ
    // subtly per-cell); we just want the recall to be comparable.
    // Test passes if SPEC LOOKUP achieves at least 80% of BASIS recall.
    let recall_pct = if msgs_a.is_empty() {
        100.0
    } else {
        100.0 * intersection.len() as f32 / msgs_a.len() as f32
    };
    println!("\nRecall (SPEC vs BASIS): {:.1}%", recall_pct);
    assert!(
        recall_pct >= 80.0 || msgs_b.len() >= msgs_a.len(),
        "SPEC-LOOKUP recall {recall_pct:.1}% < 80% of BASIS path"
    );
}
