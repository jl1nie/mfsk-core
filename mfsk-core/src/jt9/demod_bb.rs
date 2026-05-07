//! Legacy box-car demod path. Superseded by `softsym.rs` for `decode_scan`
//! in 0.5.9 — kept only for the `synth_demod_bb_roundtrip` and
//! `jt9_golden_grid_roundtrips` tests that pin the encode-side
//! invariants. Will be removed once #19 is closed and the new pipeline
//! has reached steady-state recall.
#![allow(dead_code)]

//! JT9 baseband demodulator: 1500 Hz complex → 206 bit LLRs.
//!
//! Operates on `(idat, qdat)` produced by `baseband::mix_to_baseband`.
//! At 1500 Hz with NSPS_BB=864, a 864-pt FFT gives bin width
//! 1500/864 ≈ 1.736 Hz = one JT9 tone spacing.
//! After mixing to the candidate's centre frequency, tone 0 (sync)
//! lands at DC (bin 0) and data tones 1..=8 land at bins 1..=8.
//! The LLR formula and de-interleaver are identical to `rx.rs`.

use num_complex::Complex;
use rustfft::FftPlanner;

use super::baseband::NSPS_BB;
use super::interleave::deinterleave_llrs;
use super::sync_pattern::JT9_ISYNC;

const LLR_CLAMP: f32 = 20.0;

/// Compute a differential sync score at a given baseband offset.
///
/// For each of the 85 symbols, compute the coherent-sum magnitude (= bin 0
/// of an 864-pt FFT, but computed cheaply as a simple sum). Then:
///   score = sync_avg / (sync_avg + data_avg)
///
/// At the true frame alignment, sync symbols carry tone 0 (DC) → large
/// coherent sum; data symbols carry tones 1..8 → sums cancel → small.
/// At wrong alignments or pure noise, both averages are similar → score ≈ 0.5.
/// This is insensitive to the absolute noise level, solving the
/// "noise denominator varies with position" problem of single-FFT estimates.
pub fn sync_score(idat: &[f32], qdat: &[f32], start_bb: usize) -> f32 {
    let n = idat.len();
    let mut sync_sum = 0.0f32;
    let mut data_sum = 0.0f32;
    let mut ns = 0usize;
    let mut nd = 0usize;

    for sym_idx in 0..85usize {
        let s0 = start_bb + sym_idx * NSPS_BB;
        let s1 = s0 + NSPS_BB;
        if s1 > n {
            break;
        }
        let si: f32 = idat[s0..s1].iter().sum();
        let sq: f32 = qdat[s0..s1].iter().sum();
        let mag = (si * si + sq * sq).sqrt();
        if JT9_ISYNC[sym_idx] == 1 {
            sync_sum += mag;
            ns += 1;
        } else {
            data_sum += mag;
            nd += 1;
        }
    }

    if ns == 0 {
        return 0.0;
    }
    let sync_avg = sync_sum / ns as f32;
    let data_avg = if nd > 0 { data_sum / nd as f32 } else { 0.0 };
    sync_avg / (sync_avg + data_avg + 1e-6)
}

/// Inverse Gray code on 3-bit values.
#[inline]
fn inv_gray3(g: u8) -> u8 {
    let mut n = g & 0x7;
    n ^= n >> 1;
    n ^= n >> 2;
    n & 0x7
}

/// Demodulate 85 channel symbols from baseband and return 206 deinterleaved
/// bit LLRs for [`crate::fec::ConvFano232::decode_soft`].
///
/// `start_bb` is the index into `idat`/`qdat` (1500 Hz) where symbol 0 begins.
/// Convention: positive LLR ⇒ bit 0 is more likely (matches `rx.rs`).
pub fn demodulate(idat: &[f32], qdat: &[f32], start_bb: usize) -> [f32; 206] {
    let n = idat.len();
    let end = start_bb + 85 * NSPS_BB;
    if end > n {
        return [0f32; 206];
    }

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NSPS_BB);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];

    let mut llrs207 = [0f32; 207];
    let mut noise_acc = 0.0f32;
    let mut noise_count = 0u32;
    let mut j = 0usize; // data-symbol counter

    for sym_idx in 0..85usize {
        let s0 = start_bb + sym_idx * NSPS_BB;
        let mut buf: Vec<Complex<f32>> = (s0..s0 + NSPS_BB)
            .map(|k| Complex::new(idat[k], qdat[k]))
            .collect();
        fft.process_with_scratch(&mut buf, &mut scratch);

        // Noise: bins 9..14 above the 9-tone passband.
        for b in 9..14 {
            noise_acc += buf[b].norm_sqr();
            noise_count += 1;
        }

        if JT9_ISYNC[sym_idx] == 1 {
            continue;
        }

        // Data tones 1..=8 are at bins 1..=8 (tone 0 = DC is sync).
        let mut mags = [0f32; 8];
        for t in 0..8 {
            mags[t] = buf[t + 1].norm();
        }

        // Max-log-MAP LLR (same as rx.rs, positive ⇒ bit=0 more likely).
        for bit_pos in 0..3 {
            let mask = 1u8 << (2 - bit_pos);
            let mut max0 = f32::NEG_INFINITY;
            let mut max1 = f32::NEG_INFINITY;
            for tone in 0u8..8 {
                let data_bits = inv_gray3(tone);
                let p = mags[tone as usize] * mags[tone as usize];
                if data_bits & mask == 0 {
                    max0 = max0.max(p);
                } else {
                    max1 = max1.max(p);
                }
            }
            llrs207[3 * j + bit_pos] = max0 - max1;
        }
        j += 1;
    }
    debug_assert_eq!(j, 69);

    let noise_var = if noise_count > 0 {
        (noise_acc / noise_count as f32).max(1e-6)
    } else {
        1.0
    };

    let mut out = [0f32; 206];
    for i in 0..206 {
        out[i] = (llrs207[i] / noise_var).clamp(-LLR_CLAMP, LLR_CLAMP);
    }
    deinterleave_llrs(&mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jt9::baseband::mix_to_baseband;
    use crate::jt9::tx::synthesize_standard;

    #[test]
    fn synth_demod_bb_roundtrip() {
        use crate::core::{DecodeContext, FecOpts, MessageCodec};
        use crate::fec::{ConvFano232, FecCodec};
        use crate::msg::{Jt72Codec, Jt72Message};

        let freq = 1200.0f32;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");

        let (idat, qdat) = mix_to_baseband(&audio, freq);
        let llrs = demodulate(&idat, &qdat, 0);

        let codec = ConvFano232;
        let res = codec
            .decode_soft(&llrs, &FecOpts::default())
            .expect("Fano must converge on clean synth");
        let mut payload = [0u8; 72];
        payload.copy_from_slice(&res.info);
        let msg = Jt72Codec::default()
            .unpack(&payload, &DecodeContext::default())
            .expect("unpack");
        match msg {
            Jt72Message::Standard {
                call1,
                call2,
                grid_or_report,
            } => {
                assert_eq!(call1, "CQ");
                assert_eq!(call2, "K1ABC");
                assert_eq!(grid_or_report, "FN42");
            }
            other => panic!("expected Standard, got {:?}", other),
        }
    }

    /// Roundtrip for all 5 golden messages to catch any Jt72Codec grid-square bugs.
    #[test]
    fn jt9_golden_grid_roundtrips() {
        use crate::core::{DecodeContext, FecOpts, MessageCodec};
        use crate::fec::{ConvFano232, FecCodec};
        use crate::msg::{Jt72Codec, Jt72Message};

        let cases = &[
            ("CQ", "GM7GAX", "IO75"),
            ("TF3G", "N7MQ", "CN84"),
            ("K1JT", "KF4RWA", "73"),
            ("CQ", "M0WAY", "IO82"),
            ("K1JT", "N5KDV", "EM41"),
        ];
        for &(c1, c2, grid) in cases {
            let audio = synthesize_standard(c1, c2, grid, 12_000, 1346.0, 0.5).expect("synth");
            let (idat, qdat) = mix_to_baseband(&audio, 1346.0);
            let llrs = demodulate(&idat, &qdat, 0);
            let res = ConvFano232
                .decode_soft(&llrs, &FecOpts::default())
                .unwrap_or_else(|| panic!("Fano failed for {} {} {}", c1, c2, grid));
            let mut payload = [0u8; 72];
            payload.copy_from_slice(&res.info);
            let msg = Jt72Codec::default()
                .unpack(&payload, &DecodeContext::default())
                .unwrap_or_else(|| {
                    panic!(
                        "unpack failed for {} {} {} hard={}",
                        c1, c2, grid, res.hard_errors
                    )
                });
            match msg {
                Jt72Message::Standard {
                    call1,
                    call2,
                    grid_or_report,
                } => {
                    assert_eq!(call1, c1, "call1 mismatch for {} {} {}", c1, c2, grid);
                    assert_eq!(call2, c2, "call2 mismatch for {} {} {}", c1, c2, grid);
                    assert_eq!(
                        grid_or_report, grid,
                        "grid mismatch: got {} for {} {} {}",
                        grid_or_report, c1, c2, grid
                    );
                }
                other => panic!(
                    "expected Standard for {} {} {}, got {:?}",
                    c1, c2, grid, other
                ),
            }
        }
    }
}

#[cfg(test)]
mod sync_diag {
    use super::*;
    use crate::jt9::baseband::mix_to_baseband;
    use std::path::Path;

    fn load_wav(path: &Path) -> Option<Vec<f32>> {
        let bytes = std::fs::read(path).ok()?;
        let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let data = &bytes[44..44 + data_len];
        Some(
            data.chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                .collect(),
        )
    }

    #[test]
    #[ignore]
    fn print_sync_scores_dense() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !path.exists() {
            eprintln!("WAV not found");
            return;
        }
        let audio = load_wav(path).unwrap();

        // For 1346 Hz: compare mixing at 1345.5 (coarse) vs 1345.5+0.434=1345.934 Hz
        eprintln!("=== 1346 Hz: score at start_bb=1296..1566 for different mix freqs ===");
        for &mix_freq in &[1345.5f32, 1345.934f32, 1346.0f32, 1346.434f32] {
            let (idat, qdat) = mix_to_baseband(&audio, mix_freq);
            let score_1320 = sync_score(&idat, &qdat, 1320);
            let score_1512 = sync_score(&idat, &qdat, 1512);
            let score_1566 = sync_score(&idat, &qdat, 1566);
            eprintln!(
                "  mix={:.3} Hz: score@1320={:.4}  score@1512={:.4}  score@1566={:.4}",
                mix_freq, score_1320, score_1512, score_1566
            );
        }

        // Dense scan from 0..2000 for all golden signals, step 54 (⅟₁₆ symbol)
        for &(freq, label) in &[
            (1224.0f32, "1224.0 Hz (golden K1JT KF4RWA 73, dt=0.1)"),
            (1345.5f32, "1345.5 Hz (golden K1JT N5KDV EM41, dt=0.1)"),
            (1185.8f32, "1185.8 Hz (golden TF3G N7MQ CN84, dt=0.0)"),
            (1289.9f32, "1289.9 Hz (golden CQ M0WAY IO82, dt=0.1)"),
            (1119.0f32, "1119.0 Hz (golden CQ GM7GAX IO75, dt=0.0)"),
        ] {
            let (idat, qdat) = mix_to_baseband(&audio, freq);
            let mut peak_bb = 0usize;
            let mut peak_score = 0.0f32;
            for start_bb in (0..=2200usize).step_by(54) {
                if start_bb + 85 * NSPS_BB > idat.len() {
                    break;
                }
                let score = sync_score(&idat, &qdat, start_bb);
                if score > peak_score {
                    peak_score = score;
                    peak_bb = start_bb;
                }
            }
            eprintln!(
                "\n{label}:  PEAK at start_bb={} ({:.3}s) score={:.4}",
                peak_bb,
                peak_bb as f32 / 1500.0,
                peak_score
            );
        }
    }

    /// Diagnostic: compare expected vs detected tones. Kept as scaffolding
    /// for verifying the new softsym pipeline.
    #[test]
    #[ignore]
    fn compare_tx_tones_vs_wav_tones() {
        use crate::jt9::tx::{encode_channel_symbols, synthesize_standard};
        use crate::msg::jt72::pack_standard;
        use num_complex::Complex;
        use rustfft::FftPlanner;

        // 1) Get expected tone sequence from our TX for both EM41 and NM51
        let words_em = pack_standard("K1JT", "N5KDV", "EM41").expect("pack EM");
        let words_nm = pack_standard("K1JT", "N5KDV", "NM51").expect("pack NM");
        let mut info_em = [0u8; 72];
        let mut info_nm = [0u8; 72];
        for (i, b) in info_em.iter_mut().enumerate() {
            *b = (words_em[i / 6] >> (5 - (i % 6))) & 1;
        }
        for (i, b) in info_nm.iter_mut().enumerate() {
            *b = (words_nm[i / 6] >> (5 - (i % 6))) & 1;
        }
        let tones_em = encode_channel_symbols(&info_em);
        let tones_nm = encode_channel_symbols(&info_nm);
        eprintln!("EM41 tones (first 30): {:?}", &tones_em[..30]);
        eprintln!("NM51 tones (first 30): {:?}", &tones_nm[..30]);

        let words = words_em;
        let mut info_bits = [0u8; 72];
        for (i, bit) in info_bits.iter_mut().enumerate() {
            let word = words[i / 6];
            let bit_in_word = 5 - (i % 6);
            *bit = (word >> bit_in_word) & 1;
        }
        let expected_tones = encode_channel_symbols(&info_bits);
        eprintln!("Expected tones (first 20): {:?}", &expected_tones[..20]);

        // 2) Sanity: synth that signal and confirm our DEMOD gets the same tones
        let synth_audio =
            synthesize_standard("K1JT", "N5KDV", "EM41", 12_000, 1346.0, 0.5).expect("synth");
        let (synth_id, synth_qd) = mix_to_baseband(&synth_audio, 1346.0);

        // Print dominant bins for synth signal at start_bb=0
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(NSPS_BB);
        let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];

        eprintln!("\n=== SYNTH 'K1JT N5KDV EM41' at 1346 Hz, start_bb=0 ===");
        eprintln!("sym | expected | dominant_bin | top_3_mags");
        for sym_idx in 0..15usize {
            let s0 = sym_idx * NSPS_BB;
            let mut buf: Vec<Complex<f32>> = (s0..s0 + NSPS_BB)
                .map(|k| Complex::new(synth_id[k], synth_qd[k]))
                .collect();
            fft.process_with_scratch(&mut buf, &mut scratch);
            let mut mags = [0f32; 9];
            for t in 0..9 {
                mags[t] = buf[t].norm();
            }
            let mut best = 0usize;
            for t in 1..9 {
                if mags[t] > mags[best] {
                    best = t;
                }
            }
            eprintln!(
                "  {:2} |    {}     |      {}       | {:?}",
                sym_idx,
                expected_tones[sym_idx],
                best,
                mags.iter()
                    .map(|m| (*m * 1.0).round() / 1.0)
                    .collect::<Vec<_>>()
            );
        }

        // 3) Now do same for WAV at 1346 Hz, best position
        let wav_path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !wav_path.exists() {
            eprintln!("WAV not found");
            return;
        }
        let audio = load_wav(wav_path).unwrap();
        let (wav_id, wav_qd) = mix_to_baseband(&audio, 1346.0);

        // Scan widely for: (a) sync_score peak; (b) max signal energy in tone bins
        let mut best_start_sync = 0usize;
        let mut best_sync_sc = 0.0f32;
        let mut best_start_pwr = 0usize;
        let mut best_pwr = 0.0f32;
        for start_bb in (0..(wav_id.len().saturating_sub(85 * NSPS_BB))).step_by(8) {
            let s = sync_score(&wav_id, &wav_qd, start_bb);
            if s > best_sync_sc {
                best_sync_sc = s;
                best_start_sync = start_bb;
            }
            // Total signal power in tone bins 0..=8 across all 85 symbols (bin-0 magnitude only)
            let mut pwr = 0.0f32;
            for sym_idx in 0..85usize {
                let s0 = start_bb + sym_idx * NSPS_BB;
                if s0 + NSPS_BB > wav_id.len() {
                    break;
                }
                let si: f32 = wav_id[s0..s0 + NSPS_BB].iter().sum();
                let sq: f32 = wav_qd[s0..s0 + NSPS_BB].iter().sum();
                pwr += si * si + sq * sq;
            }
            if pwr > best_pwr {
                best_pwr = pwr;
                best_start_pwr = start_bb;
            }
        }
        eprintln!(
            "\nSync-score peak: start_bb={} ({:.3}s) score={:.4}",
            best_start_sync,
            best_start_sync as f32 / 1500.0,
            best_sync_sc
        );
        eprintln!(
            "DC-power peak:   start_bb={} ({:.3}s) pwr={:.0}",
            best_start_pwr,
            best_start_pwr as f32 / 1500.0,
            best_pwr
        );

        let best_start = best_start_sync;
        let best_sc = best_sync_sc;
        eprintln!(
            "\n=== WAV 1346 Hz at best sync, start_bb={} score={:.4} ===",
            best_start, best_sc
        );
        eprintln!("EM41 first 10 expected tones: {:?}", &tones_em[..10]);
        eprintln!("NM51 first 10 expected tones: {:?}", &tones_nm[..10]);

        // ALSO compare to 1224 Hz (the signal we DO decode correctly).
        // This tells us what a "good" signal looks like.
        let words_1224 = pack_standard("K1JT", "KF4RWA", "73").expect("pack 1224");
        let mut info_1224 = [0u8; 72];
        for (i, b) in info_1224.iter_mut().enumerate() {
            *b = (words_1224[i / 6] >> (5 - (i % 6))) & 1;
        }
        let tones_1224 = encode_channel_symbols(&info_1224);
        let (wav_id_1224, wav_qd_1224) = mix_to_baseband(&audio, 1224.0);
        let mut best_1224_start = 0usize;
        let mut best_1224_sc = 0.0f32;
        for start_bb in (0..(wav_id_1224.len().saturating_sub(85 * NSPS_BB))).step_by(8) {
            let s = sync_score(&wav_id_1224, &wav_qd_1224, start_bb);
            if s > best_1224_sc {
                best_1224_sc = s;
                best_1224_start = start_bb;
            }
        }
        eprintln!(
            "\n=== WAV 1224 Hz (correctly decoded reference) at start_bb={} score={:.4} ===",
            best_1224_start, best_1224_sc
        );
        eprintln!("Expected tones (first 15): {:?}", &tones_1224[..15]);
        eprintln!("sym | expected | dominant_bin | top mags");
        for sym_idx in 0..15usize {
            let s0 = best_1224_start + sym_idx * NSPS_BB;
            if s0 + NSPS_BB > wav_id_1224.len() {
                break;
            }
            let mut buf: Vec<Complex<f32>> = (s0..s0 + NSPS_BB)
                .map(|k| Complex::new(wav_id_1224[k], wav_qd_1224[k]))
                .collect();
            fft.process_with_scratch(&mut buf, &mut scratch);
            let mut mags = [0f32; 9];
            for t in 0..9 {
                mags[t] = buf[t].norm();
            }
            let mut best = 0usize;
            for t in 1..9 {
                if mags[t] > mags[best] {
                    best = t;
                }
            }
            let exp = tones_1224[sym_idx];
            let mark = if exp == best as u8 { "✓" } else { "✗" };
            eprintln!(
                "  {:2} |    {}     |      {}    {}  | {:?}",
                sym_idx,
                exp,
                best,
                mark,
                mags.iter().map(|m| (*m).round()).collect::<Vec<_>>()
            );
        }
        eprintln!("sym | expected | dominant_bin | top mags");
        for sym_idx in 0..15usize {
            let s0 = best_start + sym_idx * NSPS_BB;
            if s0 + NSPS_BB > wav_id.len() {
                break;
            }
            let mut buf: Vec<Complex<f32>> = (s0..s0 + NSPS_BB)
                .map(|k| Complex::new(wav_id[k], wav_qd[k]))
                .collect();
            fft.process_with_scratch(&mut buf, &mut scratch);
            let mut mags = [0f32; 9];
            for t in 0..9 {
                mags[t] = buf[t].norm();
            }
            let mut best = 0usize;
            for t in 1..9 {
                if mags[t] > mags[best] {
                    best = t;
                }
            }
            let exp = expected_tones[sym_idx];
            let mark = if exp == best as u8 { "✓" } else { "✗" };
            eprintln!(
                "  {:2} |    {}     |      {}    {}  | {:?}",
                sym_idx,
                exp,
                best,
                mark,
                mags.iter().map(|m| (*m).round()).collect::<Vec<_>>()
            );
        }
    }

    /// Answer the key question: where does each golden signal ACTUALLY decode?
    /// Try Fano at every start_bb from 900..2200 (step 54) and report successes.
    #[test]
    #[ignore]
    fn fano_sweep_golden() {
        use crate::core::{DecodeContext, FecOpts, MessageCodec};
        use crate::fec::{ConvFano232, FecCodec};
        use crate::msg::Jt72Codec;

        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !path.exists() {
            eprintln!("WAV not found");
            return;
        }
        let audio = load_wav(path).unwrap();

        for &(nom_freq, label) in &[
            (1224.0f32, "1224 Hz K1JT KF4RWA 73 dt=0.1"),
            (1346.0f32, "1346 Hz K1JT N5KDV EM41 dt=0.1"),
            (1186.0f32, "1186 Hz TF3G N7MQ CN84  dt=0.0"),
            (1290.0f32, "1290 Hz CQ M0WAY IO82   dt=0.1"),
            (1119.0f32, "1119 Hz CQ GM7GAX IO75  dt=0.0"),
        ] {
            eprintln!("\n=== {} ===", label);
            // Try small freq deltas around the nominal frequency
            let freq_deltas = [0.0f32, 0.434, -0.434, 0.868, -0.868];
            for &df in &freq_deltas {
                let freq = nom_freq + df;
                let (idat, qdat) = mix_to_baseband(&audio, freq);
                for start_bb in (900..=2200usize).step_by(54) {
                    if start_bb + 85 * NSPS_BB > idat.len() {
                        break;
                    }
                    let score = sync_score(&idat, &qdat, start_bb);
                    if score < 0.50 {
                        continue;
                    }
                    let llrs = demodulate(&idat, &qdat, start_bb);
                    if let Some(res) = ConvFano232.decode_soft(&llrs, &FecOpts::default()) {
                        let mut payload = [0u8; 72];
                        payload.copy_from_slice(&res.info);
                        if let Some(msg) =
                            Jt72Codec::default().unpack(&payload, &DecodeContext::default())
                        {
                            let t_sec = start_bb as f32 / 1500.0;
                            eprintln!(
                                "  freq={:.3} Hz start_bb={} ({:.3}s) score={:.4} hard={}: {:?}",
                                freq, start_bb, t_sec, score, res.hard_errors, msg
                            );
                        }
                    }
                }
            }
        }
    }
}
