// SPDX-License-Identifier: GPL-3.0-only
//! Decode one Q65 candidate from a [`PcmSource`] read forward: [`decode_candidate_streamed`].
//!
//! The same decode as [`super::decode_candidate`] for a slot that cannot be held (an SD card read forward). It never holds
//! the audio: it reads one symbol's `nsps` samples at a time, makes the symbol's row, and keeps only what the metric needs.
//! The plain, AP-hint and AP-list decodes keep the 63 narrow rows (63 x 64 `f32`, 16 KB); the **fast-fading metric streams**
//! its wide rows into a [`FastFadingStream`], so what it keeps is the 63 x 64 metric, 16 128 B whatever the sub-mode, where
//! the whole wide spectrogram is 48 KB (sub-mode A) to 290 KB (E).
//!
//! **Reads**: one forward sweep for the decode (the 63 data symbols, the 22 sync positions skipped). A decode that succeeds
//! reads once more, from the start, for the SNR (`q65_snr_db`'s composite spectrum, over all 85 symbols along the decoded
//! codeword): most candidates fail, and cost one sweep.

use alloc::vec;

use super::Q65Result;
use super::candidate::Q65Candidate;
use super::decode_request::{Q65SubMode, ctx_from_hash_table};
use super::rx::{
    MAXITERS_DEPTH1, ap_used, bp_decode, collect_data_rows, default_es_no_metric,
    extract_data_energies_w, finish_with, for_each_data_row, list_decode, reencode, snr_db_narrow,
    snr_db_wide, submode_index_from_params, wide_layout,
};
use super::snr::q65_snr_db_w;
use super::windows::{PcmWindows, SymbolWindows};
use crate::engine::source::PcmSource;
use crate::fec::qra::{FastFadingStream, Q65Codec};
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;

/// Decode `cand` from `src`, read forward, for sub-mode `P`; the sample values are `i16 / 32768`, the scale the decoder works
/// in. Same precedence as [`super::decode_candidate`] (`ap_list`, then `fading` with `ap_hint`, then `ap_hint`, then plain)
/// and the same result: the plain, hint and list decodes to the bit, the fading decode to the float rounding of
/// [`FastFadingStream`] against the whole-spectrogram metric. `None` when the frame does not fit in `src`, BP fails or the
/// CRC rejects it, or the message does not unpack.
pub fn decode_candidate_streamed<P: Q65SubMode, A: PcmSource>(
    src: &mut A,
    sample_rate: u32,
    cand: &Q65Candidate<'_>,
) -> Option<Q65Result> {
    let ctx = ctx_from_hash_table(cand.hash_table.as_ref());
    let (start, freq) = (cand.at.start_sample, cand.at.freq_hz);
    let mut w = PcmWindows::new(src);
    let code = &QRA15_65_64_IRR_E23;

    if let Some(candidates) = cand.ap_list {
        if candidates.is_empty() {
            return None;
        }
        let energies = extract_data_energies_w::<P, _>(&mut w, sample_rate, start, freq)?;
        let mut intrinsics = vec![0.0_f32; 64 * 63];
        code.mfsk_bessel_metric(&mut intrinsics, &energies, 63, default_es_no_metric());
        let codec = Q65Codec::new(code);
        let (idx, info_syms) = list_decode(&codec, &intrinsics, candidates)?;
        let codeword = candidates[idx];
        return finish_with(&info_syms, 0, sample_rate, start, freq, &ctx, || {
            q65_snr_db_w::<P, _>(&mut w, sample_rate, start, freq, &codeword, |_| {
                snr_db_narrow::<P>(&energies, &codeword)
            })
        })
        .map(|mut r| {
            r.ap = true;
            r
        });
    }

    let ap = cand.ap();
    if let Some((model, b90_ts)) = cand.fading {
        let lay = wide_layout::<P>(sample_rate, freq, w.len(), start)?;
        let mut stream = FastFadingStream::new(
            code,
            submode_index_from_params::<P>(),
            b90_ts,
            model,
            default_es_no_metric(),
        );
        for_each_data_row(
            &mut w,
            sample_rate,
            start,
            lay.nsps,
            lay.bins_per_symbol,
            |i| lay.wide_start + i,
            None,
            |row| stream.push_row(row),
        );
        let mut intrinsics = vec![0.0_f32; 64 * 63];
        let _state = stream.finish(&mut intrinsics);
        let mut codec = Q65Codec::new(code);
        let mut info_syms = [0_i32; 13];
        let iterations =
            bp_decode(&mut codec, &intrinsics, &mut info_syms, ap, MAXITERS_DEPTH1).ok()?;
        let codeword = reencode(&mut codec, &info_syms);
        return finish_with(
            &info_syms,
            iterations,
            sample_rate,
            start,
            freq,
            &ctx,
            || {
                q65_snr_db_w::<P, _>(&mut w, sample_rate, start, freq, &codeword, |w| {
                    // the fallback is made from the wide energies, which were not held: read them again
                    let wide = collect_data_rows(
                        w,
                        sample_rate,
                        start,
                        lay.nsps,
                        lay.bins_per_symbol,
                        |i| lay.wide_start + i,
                        None,
                    );
                    snr_db_wide::<P>(&wide, sample_rate, &codeword)
                })
            },
        )
        .map(|mut r| {
            r.ap = ap_used(ap);
            r
        });
    }

    let energies = extract_data_energies_w::<P, _>(&mut w, sample_rate, start, freq)?;
    let mut intrinsics = vec![0.0_f32; 64 * 63];
    code.mfsk_bessel_metric(&mut intrinsics, &energies, 63, default_es_no_metric());
    let mut codec = Q65Codec::new(code);
    let mut info_syms = [0_i32; 13];
    let iterations =
        bp_decode(&mut codec, &intrinsics, &mut info_syms, ap, MAXITERS_DEPTH1).ok()?;
    let codeword = reencode(&mut codec, &info_syms);
    finish_with(
        &info_syms,
        iterations,
        sample_rate,
        start,
        freq,
        &ctx,
        || {
            q65_snr_db_w::<P, _>(&mut w, sample_rate, start, freq, &codeword, |_| {
                snr_db_narrow::<P>(&energies, &codeword)
            })
        },
    )
    .map(|mut r| {
        r.ap = ap_used(ap);
        r
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::alignment::Alignment;
    use crate::engine::source::{CountingSource, SliceSource};
    use crate::fec::qra::FadingModel;
    use crate::msg::ApHint;
    use crate::q65::{Q65a30, decode_candidate, standard_qso_codewords, synthesize_standard};

    fn gauss_noise(n: usize, seed: u64) -> Vec<f32> {
        let mut s = seed.wrapping_add(1);
        let mut next = || {
            s = s
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((s >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
        };
        let mut out = alloc::vec::Vec::with_capacity(n);
        while out.len() < n {
            let (u, v) = (next(), next());
            let m = (-2.0 * u.ln()).sqrt();
            out.push(m * (core::f32::consts::TAU * v).cos());
            out.push(m * (core::f32::consts::TAU * v).sin());
        }
        out.truncate(n);
        out
    }

    /// A Q65-30A slot of "K1ABC JA1ABC PM95" at 1500 Hz with noise of `sigma` (a fixed generator), scaled into i16 range.
    /// Returns the i16 audio and the f32 a slice decode reads from it: exactly `i16 / 32768`, what the streamed one reads.
    fn slot(sigma: f32, seed: u64) -> (Vec<i16>, Vec<f32>) {
        let clean = synthesize_standard("K1ABC", "JA1ABC", "PM95", 12_000, 1500.0, 0.3).unwrap();
        let nz = gauss_noise(clean.len(), seed);
        let scale = 1.0 / (4.0 + sigma * 4.0); // keep the peaks inside [-1, 1)
        let raw: Vec<i16> = clean
            .iter()
            .zip(&nz)
            .map(|(c, n)| {
                ((c + n * sigma) * scale * 32_768.0)
                    .round()
                    .clamp(-32_768.0, 32_767.0) as i16
            })
            .collect();
        let f: Vec<f32> = raw.iter().map(|&v| f32::from(v) / 32_768.0).collect();
        (raw, f)
    }

    fn at() -> Alignment {
        Alignment::new(0, 1500.0)
    }

    fn both(raw: &[i16], f: &[f32], cand: &Q65Candidate) -> (Option<Q65Result>, Option<Q65Result>) {
        let slice = decode_candidate::<Q65a30>(f, 12_000, cand);
        let streamed = decode_candidate_streamed::<Q65a30, _>(&mut SliceSource(raw), 12_000, cand);
        (slice, streamed)
    }

    /// Same result from the slice and from a source read forward: the message, frequency, time, the SNR (a composite
    /// spectrum read the second time) and the AP flag to the bit; `iterations` too except under the fading metric, whose
    /// intrinsics differ from the whole-spectrogram ones by float rounding.
    fn same(a: &Option<Q65Result>, b: &Option<Q65Result>, exact_iterations: bool, what: &str) {
        match (a, b) {
            (None, None) => {}
            (Some(a), Some(b)) => {
                assert_eq!(a.message, b.message, "{what}");
                assert_eq!(a.bits77, b.bits77, "{what}");
                assert_eq!(a.freq_hz.to_bits(), b.freq_hz.to_bits(), "{what}");
                assert_eq!(a.start_sample, b.start_sample, "{what}");
                assert_eq!(a.dt_sec.to_bits(), b.dt_sec.to_bits(), "{what}");
                assert_eq!(
                    a.snr_db.to_bits(),
                    b.snr_db.to_bits(),
                    "{what}: snr {} vs {}",
                    a.snr_db,
                    b.snr_db
                );
                assert_eq!(a.ap, b.ap, "{what}");
                if exact_iterations {
                    assert_eq!(a.iterations, b.iterations, "{what}");
                }
            }
            (a, b) => panic!(
                "{what}: slice {:?}, streamed {:?}",
                a.is_some(),
                b.is_some()
            ),
        }
    }

    #[test]
    fn the_streamed_decode_is_the_slice_decode() {
        let hint = ApHint::new().with_call1("K1ABC").with_call2("JA1ABC");
        let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        let wrong = standard_qso_codewords("W9XYZ", "N0CALL", "EN52");

        // a clean slot: plain, hint, the fast-fading metric (Lorentzian decodes at b90*Ts 500, the Gaussian does not), a list
        let (raw, f) = slot(0.0, 0);
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()));
        assert!(s.is_some(), "the clean slot decodes");
        same(&s, &t, true, "clean plain");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()).ap_hint(&hint));
        same(&s, &t, true, "clean hint");
        let (s, t) = both(
            &raw,
            &f,
            &Q65Candidate::new(at()).fading(FadingModel::Lorentzian, 500.0),
        );
        assert!(s.is_some());
        same(&s, &t, false, "clean fading lorentzian");
        let (s, t) = both(
            &raw,
            &f,
            &Q65Candidate::new(at()).fading(FadingModel::Gaussian, 500.0),
        );
        assert!(
            s.is_none(),
            "the Gaussian of that width loses the clean slot"
        );
        same(&s, &t, false, "clean fading gaussian");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()).ap_list(&list));
        assert!(s.is_some());
        same(&s, &t, true, "clean list");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()).ap_list(&wrong));
        assert!(s.is_none());
        same(&s, &t, true, "clean wrong list");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(Alignment::new(0, 1503.0)));
        assert!(s.is_none());
        same(&s, &t, true, "clean, 3 Hz off");

        // a noisy slot where plain BP loses and the options find it
        let (raw, f) = slot(6.5, 0);
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()));
        assert!(s.is_none(), "plain BP loses the noisy slot");
        same(&s, &t, true, "noisy plain");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()).ap_hint(&hint));
        assert!(s.is_some(), "the AP hint finds the noisy slot");
        same(&s, &t, true, "noisy hint");
        let (s, t) = both(&raw, &f, &Q65Candidate::new(at()).ap_list(&list));
        assert!(s.is_some(), "the list finds the noisy slot");
        same(&s, &t, true, "noisy list");
        let (s, t) = both(
            &raw,
            &f,
            &Q65Candidate::new(at())
                .fading(FadingModel::Lorentzian, 0.5)
                .ap_hint(&hint),
        );
        assert!(s.is_some(), "fading and the hint find the noisy slot");
        same(&s, &t, false, "noisy fading + hint");
    }

    /// The reads of a streamed decode: a candidate that fails costs one forward sweep over the 63 data symbols (the 22 sync
    /// positions skipped, no restart); one that succeeds reads once more, from the start and over all 85 symbols, for the SNR.
    /// 30A at 12 kHz is 3 600 samples a symbol.
    #[test]
    fn the_reads_are_one_forward_sweep_and_one_more_for_a_decode() {
        const NSPS: u64 = 3_600;
        let (raw, _) = slot(0.0, 0);

        let mut miss = CountingSource::new(SliceSource(&raw[..]));
        let r = decode_candidate_streamed::<Q65a30, _>(
            &mut miss,
            12_000,
            &Q65Candidate::new(Alignment::new(0, 1503.0)),
        );
        assert!(r.is_none());
        let c = miss.cost();
        assert_eq!(c.restarts, 0, "{c:?}");
        assert_eq!(c.samples_read, 63 * NSPS, "{c:?}");

        let mut hit = CountingSource::new(SliceSource(&raw[..]));
        let r = decode_candidate_streamed::<Q65a30, _>(&mut hit, 12_000, &Q65Candidate::new(at()));
        assert!(r.is_some());
        let c = hit.cost();
        assert_eq!(c.restarts, 1, "the SNR reads from the start again: {c:?}");
        assert_eq!(c.samples_read, (63 + 85) * NSPS, "{c:?}");
    }

    /// A source shorter than the frame is not decoded (the frame must fit), under every strategy.
    #[test]
    fn a_source_shorter_than_the_frame_gives_nothing() {
        let (raw, _) = slot(0.0, 0);
        let short = &raw[..84 * 3_600];
        let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        for c in [
            Q65Candidate::new(at()),
            Q65Candidate::new(at()).ap_list(&list),
            Q65Candidate::new(at()).fading(FadingModel::Lorentzian, 0.5),
        ] {
            assert!(
                decode_candidate_streamed::<Q65a30, _>(&mut SliceSource(short), 12_000, &c)
                    .is_none()
            );
        }
    }
}
