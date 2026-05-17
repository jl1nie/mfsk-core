//! Phase 1.7.7-Stick host validation: **Goertzel vs BASIS** end-to-end
//! recall + SNR comparison on qso3_busy.wav. Goal: confirm Goertzel
//! is a drop-in replacement for BASIS — same DFT output (mathematical
//! identity), same recall, no SNR degradation. If host shows 7/7 +
//! per-station |ΔSNR| < 0.5 dB, we can ship the embedded swap without
//! touching spec format / NSTEP / PSRAM / stage1_inc.
//!
//! Why Goertzel and not the BASIS sin/cos table approach?
//! Both compute the same DFT bin `Σ x[n] exp(-jωn)` for a tone at
//! arbitrary frequency `ω = 2π·f/Fs`. BASIS precomputes the 16
//! cos/sin vectors (8 tones × {re, im}) of length NSPS=1920 and does
//! a dot product per (sym, tone) — that's the 60 KB × 2 (re, im) × 2
//! (dual_core) = 120 KB internal DRAM scratch we want to free.
//! Goertzel uses a 2-tap IIR recursion (3 state values per (sym,
//! tone), thrown away each call) so the scratch is zero. Same math,
//! same output, same recall.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_goertzel_vs_basis -- --nocapture
//! ```
#![cfg(all(feature = "fixed-point", feature = "fft-rustfft"))]
#![allow(clippy::needless_range_loop)]

use mfsk_core::core::scalar::Cmplx;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    BASIS_SCRATCH_LEN, DEFAULT_Q_THRESH, RefinedCandidate, SymMask, coarse_sync_with_allsum,
    compute_spectrogram, fill_symbol_spectra_goertzel, fill_symbol_spectra_into,
    precompute_coarse_allsum, process_candidates_into_with_cs_scratch_tuned_with_fill,
    sync_quality_block0,
};
use mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER;
use mfsk_core::msg::wsjt77::unpack77;
use std::collections::BTreeSet;

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap();
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

// The Goertzel fill kernel under test is the actual library function
// `mfsk_core::ft8::decode_block::fill_symbol_spectra_goertzel`,
// imported at the top of this file — NOT a local re-implementation.
// (Earlier drafts of this test defined a local copy of the algorithm,
// which made the regression guard tautological: it would only catch
// bugs in the local copy, not in the library impl. Gemini PR #104
// review caught the duplication; the local copy is now removed.)

fn refine_pub<F>(
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

fn run_basis(audio: &[i16]) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, 3000.0);
    let allsum = precompute_coarse_allsum(&spec, 100.0, 3000.0);
    let cands = coarse_sync_with_allsum(&spec, 100.0, 3000.0, 1.0, 30, &allsum);
    let mut basis_re = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im = vec![0i16; BASIS_SCRATCH_LEN];
    let refined = refine_pub(cands, 15, |c| {
        let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        fill_symbol_spectra_into::<i16>(
            &mut *cs,
            audio,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
            &mut basis_re,
            &mut basis_im,
        );
        cs
    });
    let mut cs_scratch: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    process_candidates_into_with_cs_scratch_tuned_with_fill(
        audio,
        refined,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        &mut cs_scratch,
        |cs, cand, mask| {
            fill_symbol_spectra_into::<i16>(
                cs,
                audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                &mut basis_re,
                &mut basis_im,
            );
        },
    )
}

fn run_goertzel(audio: &[i16]) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, 3000.0);
    let allsum = precompute_coarse_allsum(&spec, 100.0, 3000.0);
    let cands = coarse_sync_with_allsum(&spec, 100.0, 3000.0, 1.0, 30, &allsum);
    let refined = refine_pub(cands, 15, |c| {
        let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        fill_symbol_spectra_goertzel(&mut cs, audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0);
        cs
    });
    let mut cs_scratch: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    process_candidates_into_with_cs_scratch_tuned_with_fill(
        audio,
        refined,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        &mut cs_scratch,
        |cs, cand, mask| {
            fill_symbol_spectra_goertzel(cs, audio, cand.freq_hz, cand.dt_sec, mask);
        },
    )
}

#[test]
fn goertzel_vs_basis_recall_and_snr() {
    let audio = load_wav_i16(asset_path!("qso3_busy.wav"));

    let basis = run_basis(&audio);
    let goertzel = run_goertzel(&audio);

    println!("\n=== BASIS reference ({} decodes) ===", basis.len());
    for r in &basis {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "?".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1} '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }
    println!("\n=== Goertzel ({} decodes) ===", goertzel.len());
    for r in &goertzel {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "?".into());
        let delta = basis
            .iter()
            .find(|b| unpack77(&b.message77).unwrap_or_default() == msg)
            .map(|b| format!("Δ={:+.2} dB", r.snr_db - b.snr_db))
            .unwrap_or_else(|| "(extra)".to_string());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1} '{}' {}",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg, delta
        );
    }

    let basis_msgs: BTreeSet<String> = basis
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();
    let goertzel_msgs: BTreeSet<String> = goertzel
        .iter()
        .filter_map(|r| unpack77(&r.message77))
        .collect();
    let shared: BTreeSet<&String> = goertzel_msgs.intersection(&basis_msgs).collect();
    let only_basis: BTreeSet<&String> = basis_msgs.difference(&goertzel_msgs).collect();
    let only_goertzel: BTreeSet<&String> = goertzel_msgs.difference(&basis_msgs).collect();

    println!("\nSummary:");
    println!("  shared:           {}", shared.len());
    println!("  only BASIS:       {} {:?}", only_basis.len(), only_basis);
    println!(
        "  only Goertzel:    {} {:?}",
        only_goertzel.len(),
        only_goertzel
    );

    // Goertzel == BASIS mathematically. f32 vs Q15 rounding may shift
    // SNR by a fraction of a dB, hard_err by 1-2 on borderline
    // candidates, but recall must be identical (within ±1 decode if
    // a borderline candidate flips both ways).
    let recall_pct = if basis_msgs.is_empty() {
        100.0
    } else {
        100.0 * shared.len() as f32 / basis_msgs.len() as f32
    };
    println!("\nRecall (Goertzel vs BASIS): {recall_pct:.1}%");
    assert!(
        recall_pct >= 100.0,
        "Goertzel must match BASIS recall (got {recall_pct:.1}%)"
    );
}
