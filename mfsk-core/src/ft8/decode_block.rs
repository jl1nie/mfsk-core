//! Embedded-friendly FT8 decode (esp-dsp pow-of-2 FFT only).
//!
//! Mirrors the host `decode_frame` pipeline but skips the 192_000-pt
//! wide-band FFT cache and the 3_840-pt per-symbol FFT — both of
//! which are non-power-of-two. Uses an 8192-pt per-symbol FFT for
//! the spectrogram (1920-sample input zero-padded) and a brute-force
//! per-tone DFT for the per-candidate LLR pass. Calls `bp_decode_kind`
//! with `BpKind::NormalizedMinSum` so the BP step skips the
//! `tanh`/`atanh` cache.
//!
//! Because the FFT bin width (≈ 1.465 Hz at 8192-pt) does not divide
//! the 6.25 Hz tone spacing evenly, Costas tone positions are computed
//! at fractional bins and rounded to the nearest integer. The
//! resulting bin-alignment jitter (≤ 0.7 Hz) is below FT8's
//! frequency-search tolerance.
//!
//! Same FFT trait, same compute path on host (rustfft) and on target
//! (esp-dsp) — sensitivity sweeps run on host and the result transfers
//! directly to hardware.

// ── Submodule layout (`docs/CLEANUP_2026_05.md` ε) ──────────────────────────
//
// `decode_block.rs` is the parent / facade for a per-stage submodule
// tree carved out by `docs/CLEANUP_2026_05.md` ε:
//   ε.1 types  · ε.2 spectrogram  · ε.3 coarse_sync
//   ε.4 fill_symbol_spectra  · ε.5 process_candidates
//   ε.6 osd_strategy (the WSJT-X-faithful precoding hook for #63)
//
// `process_candidates` and `osd_strategy` are siblings; the rest of
// the tree is stage-shaped (spectrogram → coarse_sync →
// fill_symbol_spectra → per-candidate processing). The parent file
// owns only module declarations + re-exports + tests.
mod coarse_sync;
mod fill_symbol_spectra;
mod osd_strategy;
mod process_candidates;
mod spectrogram;
mod types;

// Re-export the items that were previously defined inline in this
// file so external callers (`mfsk-ffi-ft8`, `embedded-shared`,
// integration tests, sibling modules under `ft8/`) see the same
// `mfsk_core::ft8::decode_block::*` paths.
pub use coarse_sync::{
    coarse_allsum_len, coarse_sync, coarse_sync_with_allsum, precompute_coarse_allsum,
    precompute_coarse_allsum_into,
};
pub use fill_symbol_spectra::{
    SymMask, fill_symbol_spectra, fill_symbol_spectra_generic, fill_symbol_spectra_goertzel,
    goertzel_window_end_sample, symbol_spectra_direct,
};
pub use process_candidates::decode_block_streaming;
/// Phase 1.7.7-Stick: fill-closure variant for host research /
/// regression tests (spec-lookup vs BASIS dot product comparison).
pub use process_candidates::process_candidates_into_with_cs_scratch_tuned_with_fill;
pub use process_candidates::{RefinedCandidate, decode_block, decode_block_tuned, xsnr2_db_simple};
#[cfg(feature = "fixed-point")]
pub use process_candidates::{
    decode_block_into, decode_block_into_tuned, process_candidates_into,
    process_candidates_into_tuned, process_candidates_into_with_cs_scratch,
    process_candidates_into_with_cs_scratch_tuned, refine_candidates_into,
};
#[cfg(feature = "fft-rustfft")]
pub use process_candidates::{decode_block_with_ap, decode_block_with_ap_tuned};
#[doc(hidden)]
pub use process_candidates::{process_candidates, process_candidates_tuned, sync_quality_block0};
// `super::decode::*` (host frame decoder) reaches into this internal
// per-candidate inner entry — keep visible to the parent `ft8` module.
pub(in crate::ft8) use process_candidates::process_one_candidate_inner;
// Same reason (issue #253 SNR-calibration follow-up, 2026-08-10):
// `super::decode::*`'s own SIC/single-pass/sniper engines share the
// WSJT-X-faithful xsnr2 gate `decode_block_multipass` uses, so every
// FT8 entry point reports the same SNR for the same signal.
#[cfg(feature = "std")]
pub(in crate::ft8) use process_candidates::{
    TRACE_NSYNC_FAIL, TRACE_NSYNC_PASS, TRACE_OSD_ATTEMPT, stage_trace_enabled,
};
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
pub(in crate::ft8) use process_candidates::{apply_wsjtx_xsnr2, compute_xsig_wsjtx};
pub use spectrogram::{SpecCell, Spectrogram, compute_spectrogram};
pub use types::{AudioSample, DEFAULT_Q_THRESH, NFFT_SPEC};

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::super::decode::DecodeDepth;
    use super::super::params::{NMAX, NTONES};
    use super::types::{SAMPLE_RATE_HZ, TONE_SPACING_HZ, TX_START_OFFSET_S};
    use super::*;
    use crate::engine::scalar::Cmplx;
    use crate::engine::{MessageCodec, MessageFields};
    use crate::ft8::wave_gen::{message_to_tones, tones_to_f32};
    use crate::msg::Wsjt77Message;

    fn pack_cq() -> [u8; 77] {
        let bits = Wsjt77Message
            .pack(&MessageFields {
                call1: Some("CQ".into()),
                call2: Some("JA1ABC".into()),
                grid: Some("PM95".into()),
                ..Default::default()
            })
            .unwrap();
        let mut out = [0u8; 77];
        out.copy_from_slice(&bits);
        out
    }

    fn synth_clean(msg77: &[u8; 77], freq_hz: f32) -> Vec<i16> {
        let itone = message_to_tones(msg77);
        let pcm = tones_to_f32(&itone, freq_hz, 0.5);
        let mut slot = vec![0.0f32; NMAX];
        let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize;
        let n = pcm.len().min(NMAX - start);
        slot[start..start + n].copy_from_slice(&pcm[..n]);
        slot.iter()
            .map(|&s| (s * 25_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect()
    }

    /// Reproduce the dual_core::coarse_sync_split_with_allsum
    /// pattern on host: build a full-band allsum, slice it for the
    /// head [100, mid] and tail [mid, freq_max] sub-bands, run
    /// coarse_sync_with_allsum on each, compare against running
    /// coarse_sync on each sub-band without allsum. Synthesises 5
    /// signals across the band so both halves have real candidates
    /// (mirrors the qso3 busy-band failure case).
    /// Verify that filling the allsum **column-by-column** (mirroring
    /// what `embedded-poc/m5stack-core2::stage1_inc::update_one_half`
    /// does as new spectrogram rows arrive) produces the same buffer
    /// as the one-shot `precompute_coarse_allsum`.
    /// **Test mirrors only the FT8 contiguous (16-bin) path.**
    #[test]
    fn fill_coarse_allsum_column_by_column_matches_oneshot() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax) = (1550.0_f32, 3000.0_f32);
        let oneshot = precompute_coarse_allsum(&spec, fmin, fmax);

        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
        let tone_step_bins = TONE_SPACING_HZ / df;
        let ia = (fmin / df).round() as usize;
        let max_tone_off = ((NTONES - 1) as f32 * tone_step_bins).ceil() as usize + 1;
        let nh1 = spec.n_freq;
        let ib_unbounded = (fmax / df).round() as usize;
        let ib = ib_unbounded.min(nh1.saturating_sub(max_tone_off));
        let n_freq = ib - ia + 1;
        let n_time = spec.n_time;

        // Column-by-column: simulate incremental fill, walking m
        // outer, fi inner. NFFT=3840 → 7-tone every-other gather per
        // carrier (k=0..NTONES-1; tone 7 is data-only, never a Costas
        // position — matches WSJT-X sync8.f90:66 and stage1_inc's
        // `update_one_half`).
        let mut col: Vec<f32> = vec![0.0; n_freq * n_time];
        for m in 0..n_time {
            for fi in 0..n_freq {
                let i_carrier = ia + fi;
                let mut s = 0.0_f32;
                for k in 0..(NTONES - 1) {
                    let bin = (i_carrier + 2 * k).min(nh1 - 1);
                    s += spec.power_acc(bin, m);
                }
                col[fi * n_time + m] = s;
            }
        }

        for fi in 0..n_freq {
            for m in 0..n_time {
                let a = oneshot[fi * n_time + m];
                let b = col[fi * n_time + m];
                assert!(
                    (a - b).abs() < 1e-3,
                    "fi={fi} m={m}: oneshot={a} col-by-col={b}"
                );
            }
        }
    }

    /// Same as `coarse_sync_with_allsum_matches_internal` but for a
    /// non-default sub-band (tail of [1550, 3000]) — isolates whether
    /// the coarse_sync_with_allsum path itself works for arbitrary
    /// freq ranges, independent of slicing logic.
    #[test]
    fn coarse_sync_with_allsum_tail_band_only() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 2000.0); // signal in tail band
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (1550.0_f32, 3000.0_f32, 1.0_f32, 30usize);

        let a = coarse_sync(&spec, fmin, fmax, smin, ncand);
        let allsum = precompute_coarse_allsum(&spec, fmin, fmax);
        let b = coarse_sync_with_allsum(&spec, fmin, fmax, smin, ncand, &allsum);

        assert_eq!(
            a.len(),
            b.len(),
            "tail band candidate count A={} B={}",
            a.len(),
            b.len()
        );
        for (ai, bi) in a.iter().zip(b.iter()) {
            assert!(
                (ai.freq_hz - bi.freq_hz).abs() < 1e-3 && (ai.score - bi.score).abs() < 1e-4,
                "tail-band mismatch: A={ai:?} B={bi:?}"
            );
        }
    }

    /// Reproduce the dual_core::coarse_sync_split_with_allsum
    /// pattern using per-half allsums (the API supports any sub-band
    /// directly). Synthesises 5 signals across the band so both
    /// halves have real candidates (mirrors qso3 busy band).
    /// **Per-half precompute avoids sliding-window f32 drift across
    /// the full band that breaks slice-based approaches.**
    #[test]
    fn coarse_sync_split_with_allsum_per_half_busy_band() {
        let msg = pack_cq();
        let freqs = [400.0_f32, 1100.0, 1700.0, 2200.0, 2700.0];
        let mut mix = vec![0.0f32; NMAX];
        for (i, &f) in freqs.iter().enumerate() {
            let itone = message_to_tones(&msg);
            let pcm = tones_to_f32(&itone, f, 0.5);
            let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize + i * 100;
            let n = pcm.len().min(NMAX - start);
            for k in 0..n {
                mix[start + k] += pcm[k];
            }
        }
        let audio: Vec<i16> = mix
            .iter()
            .map(|&s| (s * 5_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (100.0_f32, 3_000.0_f32, 1.0_f32, 30usize);
        let mid = 0.5 * (fmin + fmax);

        // Path A: baseline coarse_sync per half.
        let head_a = coarse_sync(&spec, fmin, mid, smin, ncand);
        let tail_a = coarse_sync(&spec, mid, fmax, smin, ncand);

        // Path B: per-half precompute_coarse_allsum.
        let allsum_head = precompute_coarse_allsum(&spec, fmin, mid);
        let allsum_tail = precompute_coarse_allsum(&spec, mid, fmax);
        let head_b = coarse_sync_with_allsum(&spec, fmin, mid, smin, ncand, &allsum_head);
        let tail_b = coarse_sync_with_allsum(&spec, mid, fmax, smin, ncand, &allsum_tail);

        assert_eq!(head_a.len(), head_b.len());
        assert_eq!(tail_a.len(), tail_b.len());
        for (a, b) in head_a.iter().zip(head_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3 && (a.score - b.score).abs() < 1e-4,
                "head per-half mismatch: A={a:?} B={b:?}"
            );
        }
        for (a, b) in tail_a.iter().zip(tail_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3 && (a.score - b.score).abs() < 1e-4,
                "tail per-half mismatch: A={a:?} B={b:?}"
            );
        }
    }

    /// **Documents a known limitation, not a regression**: building a
    /// full-band allsum and slicing it for sub-band consumption
    /// Full-band allsum sliced for each half MUST match per-half
    /// precompute. With NFFT=3840 the 8-bin every-other gather (no
    /// sliding window) is f32-stable, so slicing a full-band allsum
    /// reproduces the per-half result bit-for-bit. The previous
    /// `#[should_panic]` documented an f32 sliding-window drift that
    /// existed at NFFT=4096 + 16-bin sliding sum and is now gone.
    #[test]
    fn coarse_sync_split_with_allsum_busy_band() {
        let msg = pack_cq();
        // 5 signals across [100, 3000] Hz band — 2 in head [100, 1550]
        // and 3 in tail [1550, 3000].
        let freqs = [400.0_f32, 1100.0, 1700.0, 2200.0, 2700.0];
        let mut mix = vec![0.0f32; NMAX];
        for (i, &f) in freqs.iter().enumerate() {
            let itone = message_to_tones(&msg);
            let pcm = tones_to_f32(&itone, f, 0.5);
            let start = (TX_START_OFFSET_S * SAMPLE_RATE_HZ) as usize + i * 100;
            let n = pcm.len().min(NMAX - start);
            for k in 0..n {
                mix[start + k] += pcm[k];
            }
        }
        let audio: Vec<i16> = mix
            .iter()
            .map(|&s| (s * 5_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let spec = compute_spectrogram(&audio, 3_000.0);

        let (fmin, fmax, smin, ncand) = (100.0_f32, 3_000.0_f32, 1.0_f32, 30usize);
        let mid = 0.5 * (fmin + fmax);
        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;

        // Path A: baseline — coarse_sync per half.
        let head_a = coarse_sync(&spec, fmin, mid, smin, ncand);
        let tail_a = coarse_sync(&spec, mid, fmax, smin, ncand);

        // Path B: precompute full-band allsum, slice, run coarse_sync_with_allsum.
        let allsum_full = precompute_coarse_allsum(&spec, fmin, fmax);
        let allsum_ia = (fmin / df).round() as usize;
        let head_len = coarse_allsum_len(spec.n_freq, spec.n_time, fmin, mid);
        let tail_len = coarse_allsum_len(spec.n_freq, spec.n_time, mid, fmax);
        let ia_head = (fmin / df).round() as usize;
        let ia_tail = (mid / df).round() as usize;
        let head_off = (ia_head - allsum_ia) * spec.n_time;
        let tail_off = (ia_tail - allsum_ia) * spec.n_time;
        let head_slice = &allsum_full[head_off..head_off + head_len];
        let tail_slice = &allsum_full[tail_off..tail_off + tail_len];
        let head_b = coarse_sync_with_allsum(&spec, fmin, mid, smin, ncand, head_slice);
        let tail_b = coarse_sync_with_allsum(&spec, mid, fmax, smin, ncand, tail_slice);

        assert_eq!(
            head_a.len(),
            head_b.len(),
            "head candidate count differs: A={} B={}",
            head_a.len(),
            head_b.len()
        );
        assert_eq!(
            tail_a.len(),
            tail_b.len(),
            "tail candidate count differs: A={} B={}",
            tail_a.len(),
            tail_b.len()
        );
        for (a, b) in head_a.iter().zip(head_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "head mismatch: A={a:?} B={b:?}"
            );
        }
        for (a, b) in tail_a.iter().zip(tail_b.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "tail mismatch: A={a:?} B={b:?}"
            );
        }
    }

    #[test]
    fn coarse_sync_with_allsum_matches_internal() {
        // Phase-E2 sister API equivalence: compute the same allsum that
        // the internal precompute would produce, feed it via
        // `coarse_sync_with_allsum`, and verify identical candidate output.
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let spec = compute_spectrogram(&audio, 3_000.0);
        let (fmin, fmax, smin, ncand) = (100.0_f32, 3000.0_f32, 1.0_f32, 30usize);
        let internal = coarse_sync(&spec, fmin, fmax, smin, ncand);
        let allsum = precompute_coarse_allsum(&spec, fmin, fmax);
        assert_eq!(
            allsum.len(),
            coarse_allsum_len(spec.n_freq, spec.n_time, fmin, fmax),
            "allsum length must match coarse_allsum_len"
        );
        let external = coarse_sync_with_allsum(&spec, fmin, fmax, smin, ncand, &allsum);
        assert_eq!(internal.len(), external.len(), "candidate count differs");
        for (a, b) in internal.iter().zip(external.iter()) {
            assert!(
                (a.freq_hz - b.freq_hz).abs() < 1e-3
                    && (a.dt_sec - b.dt_sec).abs() < 1e-6
                    && (a.score - b.score).abs() < 1e-4,
                "candidate mismatch: internal={a:?} external={b:?}"
            );
        }
    }

    #[test]
    fn roundtrip_clean_signal() {
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);
        let results = decode_block(&audio, 100.0, 3000.0, 1.0, DecodeDepth::BP_ONLY, 30);
        assert!(
            results.iter().any(|r| r.message77() == msg),
            "decode_block should recover clean CQ; got {} results",
            results.len()
        );
    }

    /// Fill a clean CQ signal with `Sc = Q14i16` cs storage and
    /// verify the autogain produces non-saturated output that the
    /// generic `sync_quality_block0` recognises as a Costas hit.
    /// Exercises the Phase 2.6 fill / autogain path end-to-end.
    #[test]
    fn fill_q14i16_autogain_recovers_costas() {
        use crate::engine::scalar::Q14i16;
        let msg = pack_cq();
        let audio = synth_clean(&msg, 1500.0);

        let mut cs_q14: alloc::boxed::Box<[[Cmplx<Q14i16>; 8]; 79]> =
            alloc::vec![[Cmplx::<Q14i16>::default(); 8]; 79]
                .try_into()
                .unwrap();
        // The generic fill's 2-pass auto-gain path must yield a
        // Costas-perfect block 0 sync_quality regardless of build.
        fill_symbol_spectra_generic::<Q14i16, i16>(
            &mut cs_q14,
            &audio,
            1500.0,
            0.0,
            SymMask::SyncBlock0,
        );

        // Spot-check: non-zero entries (autogain didn't kill the signal).
        let nonzero = cs_q14.iter().flatten().any(|c| c.re.0 != 0 || c.im.0 != 0);
        assert!(nonzero, "Q14 autogain produced all-zero cs");

        // sync_quality_block0 generic with S=Q14i16 should recognise
        // the Costas pattern at full strength on a clean signal.
        let q = sync_quality_block0(&cs_q14);
        assert_eq!(q, 7, "expected perfect Costas block-0 hit, got q={q}");
    }
}
