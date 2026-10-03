// SPDX-License-Identifier: GPL-3.0-or-later
//! Frozen decode output of the 0.12 request API, so the 0.13.0 redesign
//! (one persistent `Decoder<P>` per mode, upstream parameter block, every
//! per-family request type deleted) can be proven to change nothing: every
//! request shape below is decoded on the golden recordings and compared,
//! bit for bit, with `tests/fixtures/decode_snapshot/<case>.txt`.
//!
//! The fixtures were written by the 0.12 API before the redesign
//! (`MFSK_WRITE_SNAPSHOT=1`); after it, only the calls change. A
//! difference is a behaviour change, never a reason to rewrite a fixture.
//!
//! Row format, one per line, sorted. Frame family (FT8/FT4/FST4): text,
//! then `freq_hz`, `dt_sec`, `snr_db`, `sync_score` as f32 bit patterns in
//! hex, then `hard_errors` and `pass`. Other families and the IQ path: the
//! common `Decoded` row, text then `freq_hz`, `dt_sec`, `snr_db` as bits.
#![cfg(feature = "full")]

use std::path::PathBuf;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeResult, DecodeStrictness};
use mfsk_core::fst4::Fst4s60;
use mfsk_core::msg::ApHint;
use mfsk_core::msg::decode_request::{DecodeRequest, NoiseBlanker};
use mfsk_core::msg::wsjt77::unpack77;
use mfsk_core::{Ft4, Ft8};

#[allow(dead_code)]
mod common;

const FT8_WAV: &str = asset_path!("qso3_busy.wav");
const FT4_WAV: &str = asset_path!("golden/ft4/000000_000002.wav");
const FST4_WAV: &str = asset_path!("golden/fst4/210115_0058.wav");

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/decode_snapshot")
}

fn rows(results: &[DecodeResult]) -> String {
    let mut lines: Vec<String> = results
        .iter()
        .map(|r| {
            format!(
                "{}\t{:08x}\t{:08x}\t{:08x}\t{:08x}\t{}\t{}",
                unpack77(r.message77()).unwrap_or_else(|| "<unpack failed>".into()),
                r.freq_hz.to_bits(),
                r.dt_sec.to_bits(),
                r.snr_db.to_bits(),
                r.sync_score.to_bits(),
                r.hard_errors,
                r.pass
            )
        })
        .collect();
    lines.sort();
    lines.join("\n") + "\n"
}

/// One common `Decoded` row per line, sorted.
#[allow(dead_code)]
fn decoded_rows(rows: impl IntoIterator<Item = mfsk_core::msg::Decoded>) -> String {
    let mut lines: Vec<String> = rows
        .into_iter()
        .map(|d| {
            format!(
                "{}\t{:08x}\t{:08x}\t{:08x}",
                d.text,
                d.freq_hz.to_bits(),
                d.dt_sec.to_bits(),
                d.snr_db.to_bits()
            )
        })
        .collect();
    lines.sort();
    lines.join("\n") + "\n"
}

/// Write the fixture (`MFSK_WRITE_SNAPSHOT=1`) or compare with it.
fn check(case: &str, results: &[DecodeResult]) {
    check_text(case, rows(results));
}

fn check_text(case: &str, got: String) {
    let path = fixture_dir().join(format!("{case}.txt"));
    if std::env::var_os("MFSK_WRITE_SNAPSHOT").is_some() {
        std::fs::create_dir_all(fixture_dir()).unwrap();
        std::fs::write(&path, &got).unwrap();
        return;
    }
    let want = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} (write it with MFSK_WRITE_SNAPSHOT=1)",
            path.display()
        )
    });
    assert!(
        got == want,
        "{case}: decode output changed\n--- fixture\n{want}--- now\n{got}"
    );
}

fn load(path: &str) -> Option<Vec<i16>> {
    let audio = common::load_wav_i16_opt(path);
    if audio.is_none() {
        common::skip_or_fail(path);
    }
    audio
}

/// A station heard in `qso3_busy.wav`, so AP has something to lock onto.
fn ft8_hint() -> ApHint {
    ApHint::new().with_call1("K1JT")
}

#[test]
fn ft8_request_shapes() {
    let Some(a) = load(FT8_WAV) else { return };
    let (lo, hi, sync, n) = (100.0, 3000.0, 1.0, 200);
    let req = || DecodeRequest::<Ft8>::new(&a, lo, hi, sync, n);
    let hint = ft8_hint();

    let default = req().decode().results;
    check("ft8_default", &default);
    check("ft8_single_pass", &req().single_pass().decode().results);
    for k in 1..=3 {
        check(
            &format!("ft8_sic_rounds_{k}"),
            &req().sic_rounds(k).decode().results,
        );
    }
    check("ft8_sic_early", &req().sic_early().decode().results);
    check("ft8_ap_hint", &req().ap_hint(&hint).decode().results);
    check("ft8_osd_off", &req().osd(false).decode().results);
    check(
        "ft8_strict_deep",
        &req().strictness(DecodeStrictness::Deep).decode().results,
    );
    check(
        "ft8_eq_local",
        &req().eq_mode(EqMode::Local).decode().results,
    );
    check("ft8_freq_hint", &req().freq_hint(1_500.0).decode().results);
    check("ft8_tx_freq", &req().tx_freq(1_200.0).decode().results);
    check("ft8_contest", &req().contest(true).decode().results);
    check("ft8_eme_delay", &req().eme_delay(true).decode().results);
    check(
        "ft8_also_accept",
        &req().also_accept(|_| true).decode().results,
    );
    check(
        "ft8_message_filter",
        &req().message_filter(|_| true).decode().results,
    );
    check("ft8_codec_filter", &req().codec_filter().decode().results);
    check(
        "ft8_previous_cycle",
        &req().previous_cycle(&default).decode().results,
    );
    let sniper = DecodeRequest::<Ft8>::sniper(&a, 1_500.0, 50)
        .ap_hint(&hint)
        .decode();
    check("ft8_sniper_ap", &sniper.results);
}

#[test]
fn ft4_request_shapes() {
    let Some(a) = load(FT4_WAV) else { return };
    let req = || DecodeRequest::<Ft4>::new(&a, 100.0, 3000.0, 1.2, 200);
    check("ft4_default", &req().decode().results);
    check("ft4_single_pass", &req().single_pass().decode().results);
    for k in 1..=3 {
        check(
            &format!("ft4_sic_rounds_{k}"),
            &req().sic_rounds(k).decode().results,
        );
    }
    let hint = ApHint::new().with_call1("CQ");
    check("ft4_ap_hint", &req().ap_hint(&hint).decode().results);
    check("ft4_codec_filter", &req().codec_filter().decode().results);
    check(
        "ft4_eq_local",
        &req().eq_mode(EqMode::Local).decode().results,
    );
}

#[test]
fn fst4_request_shapes() {
    let Some(a) = load(FST4_WAV) else { return };
    let req = || DecodeRequest::<Fst4s60>::new(&a, 100.0, 3000.0, 1.2, 200);
    check("fst4_60_default", &req().decode().results);
    check(
        "fst4_60_nb_percent",
        &req()
            .noise_blanker(NoiseBlanker::Percent(5))
            .decode()
            .results,
    );
    check(
        "fst4_60_nb_sweep",
        &req()
            .noise_blanker(NoiseBlanker::Sweep {
                step: 5,
                ftol_hz: 50.0,
            })
            .decode()
            .results,
    );
    let hint = ApHint::new().with_call1("CQ");
    check("fst4_60_ap_hint", &req().ap_hint(&hint).decode().results);
}

#[test]
fn wspr_request_shapes() {
    use mfsk_core::msg::WsprMessage;
    use mfsk_core::wspr::search::{SearchParams, default_search_params};
    use mfsk_core::wspr::{DecodeRequest, WsprCallsignTable};

    let Some(path) = common::corpus::golden_path("wspr/150426_0918.wav") else {
        common::skip_or_fail("WSPR golden");
        return;
    };
    let Some(a) = common::load_wav_f32_opt(&path) else {
        return;
    };
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1620.0,
        max_candidates: 100,
        ..default_search_params()
    };
    let text = |r: Vec<mfsk_core::wspr::WsprResult>| decoded_rows(r.iter().map(|d| d.to_decoded()));
    check_text(
        "wspr_default",
        text(DecodeRequest::new(&a, 12_000).params(params).decode()),
    );
    check_text(
        "wspr_defaults_only",
        text(DecodeRequest::new(&a, 12_000).decode()),
    );
    // The table is decoder state in 0.13: decode the slot twice through one
    // table and record both passes.
    let mut table = WsprCallsignTable::new();
    table.record(&WsprMessage::Type1 {
        callsign: "W3BI".into(),
        grid: "FN20".into(),
        power_dbm: 30,
    });
    let first = DecodeRequest::new(&a, 12_000)
        .params(params)
        .table(&mut table)
        .decode();
    let second = DecodeRequest::new(&a, 12_000)
        .params(params)
        .table(&mut table)
        .decode();
    check_text("wspr_table_first", text(first));
    check_text("wspr_table_second", text(second));
}

#[test]
fn jt9_request_shapes() {
    use mfsk_core::jt9::search::SearchParams;
    use mfsk_core::jt9::{DecodeRequest, Jt9Depth};

    let Some(a) = common::load_wav_f32_opt(asset_path!("130418_1742.wav")) else {
        common::skip_or_fail("JT9 recording");
        return;
    };
    let params = SearchParams {
        freq_min_hz: 1050.0,
        freq_max_hz: 1550.0,
        time_tolerance_early_sec: 1.728,
        time_tolerance_late_sec: 1.728,
        score_threshold: 0.05,
        max_candidates: 200,
    };
    for (depth, name) in [
        (Jt9Depth::Fast, "fast"),
        (Jt9Depth::Normal, "normal"),
        (Jt9Depth::Deep, "deep"),
    ] {
        let r = DecodeRequest::new(&a, 12_000)
            .params(params)
            .depth(depth)
            .decode();
        check_text(
            &format!("jt9_{name}"),
            decoded_rows(r.iter().map(|d| d.to_decoded())),
        );
    }
}

#[test]
fn jt65_request_shapes() {
    use mfsk_core::jt65::search::{SearchParams, default_search_params};
    use mfsk_core::jt65::{ChaseParams, DecodeRequest};

    let Some(path) = common::corpus::golden_path("jt65/jt65a_5sig_m18.wav") else {
        common::skip_or_fail("JT65 golden");
        return;
    };
    let Some(a) = common::load_wav_f32_opt(&path) else {
        return;
    };
    let params = SearchParams {
        freq_min_hz: 300.0,
        freq_max_hz: 2700.0,
        ..default_search_params()
    };
    let text = |r: Vec<mfsk_core::jt65::Jt65Result>| decoded_rows(r.iter().map(|d| d.to_decoded()));
    check_text(
        "jt65_default",
        text(DecodeRequest::new(&a, 12_000).params(params).decode()),
    );
    check_text(
        "jt65_chase",
        text(
            DecodeRequest::new(&a, 12_000)
                .params(params)
                .chase(ChaseParams::default())
                .decode(),
        ),
    );
}

fn wavs_in(rel: &str) -> Vec<Vec<f32>> {
    let Some(dir) = common::corpus::golden_subdir(rel) else {
        common::skip_or_fail(rel);
        return Vec::new();
    };
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    paths.sort();
    paths.iter().filter_map(common::load_wav_f32_opt).collect()
}

#[test]
fn q65_request_shapes() {
    use mfsk_core::fec::qra::FadingModel;
    use mfsk_core::q65::search::SearchParams;
    use mfsk_core::q65::{
        DecodeRequest, MultiPeriodRequest, Q65Result, Q65a30, Q65a60, Q65d60,
        standard_qso_codewords,
    };
    let text = |r: Vec<Q65Result>| decoded_rows(r.iter().map(|d| d.to_decoded()));

    // Single period, fast-fading metric (60D 10 GHz EME).
    if let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") {
        let a = common::load_wav_f32_opt(&path).unwrap();
        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 6.0,
            time_tolerance_late_sec: 6.0,
            score_threshold: 0.05,
            max_candidates: 8,
        };
        check_text(
            "q65_60d_plain",
            text(DecodeRequest::<Q65d60>::new(&a, 12_000, 0, params).decode()),
        );
        check_text(
            "q65_60d_fading",
            text(
                DecodeRequest::<Q65d60>::new(&a, 12_000, 0, params)
                    .fading(FadingModel::Gaussian, 10.0)
                    .decode(),
            ),
        );
    } else {
        common::skip_or_fail("Q65 60D golden");
    }

    // Single period with an AP hint (60A 6 m EME, first recording).
    let eme = wavs_in("q65/60A_EME_6m");
    if let Some(a) = eme.first() {
        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 30.0,
            time_tolerance_late_sec: 30.0,
            score_threshold: 0.05,
            max_candidates: 16,
        };
        let hint = ApHint::new().with_call1("W7GJ");
        check_text(
            "q65_60a_ap_hint",
            text(
                DecodeRequest::<Q65a60>::new(a, 12_000, 12_000 * 30, params)
                    .ap_hint(&hint)
                    .decode(),
            ),
        );
    }

    // Averaging over periods (30A ionoscatter): averaging becomes decoder
    // state in 0.13.
    let slots = wavs_in("q65/30A_Ionoscatter_6m");
    if !slots.is_empty() {
        let refs: Vec<&[f32]> = slots.iter().map(|v| v.as_slice()).collect();
        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 15.0,
            time_tolerance_late_sec: 15.0,
            score_threshold: 0.05,
            max_candidates: 8,
        };
        check_text(
            "q65_30a_averaged",
            text(MultiPeriodRequest::<Q65a30>::new(&refs, 12_000, 12_000 * 15, params).decode()),
        );
        let ap = standard_qso_codewords("K1JT", "K9AN", "");
        check_text(
            "q65_30a_averaged_ap_list",
            text(
                MultiPeriodRequest::<Q65a30>::new(&refs, 12_000, 12_000 * 15, params)
                    .ap_list(&ap)
                    .rx_freq(1010.0)
                    .decode(),
            ),
        );
    }
}

/// The IQ path as 0.12 ships it: FT8 and FT4 recordings in one 192 kS/s
/// stream, decoded by `IqReceiver` with its frozen defaults. 0.13's
/// receiver plus a default `AnyDecoder` per channel must reproduce these
/// rows, except that `<...>` may now resolve.
#[test]
fn iq_receiver_rows() {
    use common::iq::{add_into, interleave, synth_iq};
    use mfsk_core::Mode;
    use mfsk_core::iq::{Channelizer, IqReceiver, IqSampleFormat, IqStream};
    use std::sync::{Arc, Mutex};

    const FS: u32 = 192_000;
    const CENTER: f64 = 14_077_000.0;
    let Some(ft8) = load(FT8_WAV) else { return };
    let Some(mut ft4) = load(FT4_WAV) else { return };
    ft4.resize(90_000, 0);
    let (ft8_dial, ft4_dial) = (CENTER + 20_000.0, CENTER - 50_000.0);
    let mut iq = synth_iq(&ft8, FS, CENTER, ft8_dial);
    add_into(&mut iq, &synth_iq(&ft4, FS, CENTER, ft4_dial));
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));

    for (kind, name) in [(Channelizer::Direct, "direct"), (Channelizer::Pfb, "pfb")] {
        let stream = IqStream {
            sample_rate: FS,
            center_hz: CENTER,
            format: IqSampleFormat::Cf32,
            iq_swap: false,
        };
        let mut rx = IqReceiver::with_channelizer(stream, kind).unwrap();
        let rows = Arc::new(Mutex::new(Vec::new()));
        let sink = rows.clone();
        rx.on_decode(move |r| sink.lock().unwrap().push(r.clone()));
        let c8 = rx.add_channel(ft8_dial, Mode::Ft8).unwrap();
        let c4 = rx.add_channel(ft4_dial, Mode::Ft4).unwrap();
        rx.set_time_anchor(1_700_000_010 * 1_000_000_000);
        for chunk in interleave(&iq).chunks(2 * 77_777) {
            rx.push_cf32(chunk);
        }
        let rows = rows.lock().unwrap();
        for (ch, mode) in [(c8, "ft8"), (c4, "ft4")] {
            let got = decoded_rows(
                rows.iter()
                    .filter(|r| r.channel == ch)
                    .map(|r| r.decoded.clone()),
            );
            check_text(&format!("iq_{name}_{mode}"), got);
        }
    }
}

// ── The same shapes through `Decoder<P>` (0.13). Each must reproduce the
// fixture its 0.12 request wrote. `known()` has no counterpart: a whole
// period is decoded once.

mod via_decoder {
    use super::*;
    use mfsk_core::decoder::{
        ApMode, Contest, Decodable, DecodeParams, Decoder, Depth, Ft4Strategy, Ft8Strategy,
        MessageFilter, SlotInput, Sniper, Tuning,
    };

    fn native<P: Decodable<Row = DecodeResult>>(
        params: DecodeParams,
        extras: P::Extras,
        audio: &[i16],
    ) -> Vec<DecodeResult> {
        let mut d = Decoder::<P>::new(params).with_extras(extras);
        d.decode(&SlotInput::i16(audio))
            .rows
            .into_iter()
            .map(|r| r.native)
            .collect()
    }

    fn block() -> DecodeParams {
        DecodeParams::for_band((100.0, 3000.0))
            .depth(Depth::Deep)
            .ap(ApMode::Off)
    }

    fn t8(sync: f32, n: usize) -> Tuning<Ft8Strategy> {
        Tuning {
            sync_min: Some(sync),
            max_cand: Some(n),
            ..Default::default()
        }
    }

    #[test]
    fn ft8() {
        use mfsk_core::decoder::Ft8Extras;
        let Some(a) = load(FT8_WAV) else { return };
        let x = || Ft8Extras {
            tuning: t8(1.0, 200),
            ..Default::default()
        };
        let run = |p: DecodeParams, e: Ft8Extras| native::<Ft8>(p, e, &a);
        let with = |f: fn(&mut Ft8Extras)| {
            let mut e = x();
            f(&mut e);
            e
        };
        check("ft8_default", &run(block(), x()));
        check(
            "ft8_single_pass",
            &run(
                block(),
                with(|e| e.tuning.strategy = Some(Ft8Strategy::SinglePass)),
            ),
        );
        for k in 1..=3 {
            let mut e = x();
            e.tuning.strategy = Some(Ft8Strategy::SicRounds(k));
            check(&format!("ft8_sic_rounds_{k}"), &run(block(), e));
        }
        check(
            "ft8_sic_early",
            &run(
                block(),
                with(|e| e.tuning.strategy = Some(Ft8Strategy::SicEarly)),
            ),
        );
        check(
            "ft8_ap_hint",
            &run(
                block(),
                with(|e| e.ap_hint = Some(ApHint::new().with_call1("K1JT"))),
            ),
        );
        check(
            "ft8_osd_off",
            &run(block(), with(|e| e.tuning.osd = Some(false))),
        );
        check(
            "ft8_strict_deep",
            &run(
                block(),
                with(|e| e.tuning.strictness = Some(DecodeStrictness::Deep)),
            ),
        );
        check(
            "ft8_eq_local",
            &run(block(), with(|e| e.eq = EqMode::Local)),
        );
        check("ft8_freq_hint", &run(block().rx_freq(1_500.0), x()));
        check("ft8_tx_freq", &run(block().tx_freq(1_200.0), x()));
        check(
            "ft8_contest",
            &run(block().contest(Contest::GridExchange), x()),
        );
        check("ft8_eme_delay", &run(block().eme_delay(true), x()));
        check(
            "ft8_also_accept",
            &run(
                block(),
                with(|e| e.filter = MessageFilter::AlsoAccept(|_| true)),
            ),
        );
        check(
            "ft8_message_filter",
            &run(block(), with(|e| e.filter = MessageFilter::Only(|_| true))),
        );
        check(
            "ft8_codec_filter",
            &run(block(), with(|e| e.filter = MessageFilter::Codec)),
        );
        // a7: the decoder's own decodes of period n - 2.
        let mut d = Decoder::<Ft8>::new(block()).with_extras(with(|e| e.a7 = true));
        d.decode(&SlotInput::i16(&a).period(0));
        let second: Vec<DecodeResult> = d
            .decode(&SlotInput::i16(&a).period(2))
            .rows
            .into_iter()
            .map(|r| r.native)
            .collect();
        check("ft8_previous_cycle", &second);
        for (depth, name) in [
            (Depth::Fast, "d1"),
            (Depth::Normal, "d2"),
            (Depth::Deep, "d3"),
        ] {
            let mut e = x();
            if depth == Depth::Deep {
                e.ap_hint = Some(ApHint::new().with_call1("K1JT"));
            }
            check(&format!("ft8_wsjtx_{name}"), &run(block().depth(depth), e));
        }
        let sniper = Ft8Extras {
            tuning: t8(0.8, 50),
            ap_hint: Some(ApHint::new().with_call1("K1JT")),
            sniper: Some(Sniper::default()),
            ..Default::default()
        };
        check("ft8_sniper_ap", &run(block().rx_freq(1_500.0), sniper));
    }

    #[test]
    fn ft4() {
        use mfsk_core::decoder::Ft4Extras;
        let Some(a) = load(FT4_WAV) else { return };
        let x = || Ft4Extras {
            tuning: Tuning {
                sync_min: Some(1.2),
                max_cand: Some(200),
                ..Default::default()
            },
            ..Default::default()
        };
        let run = |e: Ft4Extras| native::<Ft4>(block(), e, &a);
        check("ft4_default", &run(x()));
        let mut e = x();
        e.tuning.strategy = Some(Ft4Strategy::SinglePass);
        check("ft4_single_pass", &run(e));
        for k in 1..=3 {
            let mut e = x();
            e.tuning.strategy = Some(Ft4Strategy::SicRounds(k));
            check(&format!("ft4_sic_rounds_{k}"), &run(e));
        }
        let mut e = x();
        e.ap_hint = Some(ApHint::new().with_call1("CQ"));
        check("ft4_ap_hint", &run(e));
        let mut e = x();
        e.filter = MessageFilter::Codec;
        check("ft4_codec_filter", &run(e));
        let mut e = x();
        e.eq = EqMode::Local;
        check("ft4_eq_local", &run(e));
    }

    #[test]
    fn fst4() {
        use mfsk_core::decoder::Fst4Extras;
        let Some(a) = load(FST4_WAV) else { return };
        let x = || Fst4Extras {
            tuning: Tuning {
                sync_min: Some(1.2),
                max_cand: Some(200),
                ..Default::default()
            },
            ..Default::default()
        };
        let run = |e: Fst4Extras| native::<Fst4s60>(block(), e, &a);
        check("fst4_60_default", &run(x()));
        let mut e = x();
        e.noise_blanker = Some(NoiseBlanker::Percent(5));
        check("fst4_60_nb_percent", &run(e));
        let mut e = x();
        e.noise_blanker = Some(NoiseBlanker::Sweep {
            step: 5,
            ftol_hz: 50.0,
        });
        check("fst4_60_nb_sweep", &run(e));
        let mut e = x();
        e.ap_hint = Some(ApHint::new().with_call1("CQ"));
        check("fst4_60_ap_hint", &run(e));
    }
}
