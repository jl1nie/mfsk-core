// SPDX-License-Identifier: GPL-3.0-only
//! Frozen decode output of the 0.12 request API, so the 0.13.0 redesign
//! (one persistent `Decoder<P>` per mode, upstream parameter block, every
//! per-family request type deleted) can be proven to change nothing: every
//! request shape below is decoded on the golden recordings and compared
//! with `tests/fixtures/decode_snapshot/<case>.txt`: the messages, their order,
//! `pass` exactly, the float columns within [`Tol`] (below), because the
//! fixtures hold f32 bit patterns and the last bits belong to the machine
//! (#579). `MFSK_SNAPSHOT_STRICT=1` asks for the bits again.
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

/// How far a row may sit from its fixture on a machine other than the one that wrote it (#579).
///
/// The fixtures pin f32 bit patterns written on x86_64 Linux (glibc 2.35, rustfft's AVX2
/// kernel). The same decode on Apple M5 (aarch64: rustfft's NEON kernel, Apple's libm) has the
/// same messages in the same order in all 53 cases, and 50 of them differ in the last bits:
/// `freq_hz` by up to 1.2e-3 Hz (FT4), `dt_sec` by 0, `snr_db` by up to 0.038 dB (the IQ
/// path; 0.036 on `ft8_wsjtx_d2`, 1.5e-3 elsewhere), `sync_score` by up to 1.9e-4 relative,
/// `hard_errors` by 1 in one FT8 row. A Zen 2 Ryzen on that glibc matches all 53 bit for bit;
/// a Ryzen 7 7435HS on glibc 2.44 differs in 4 rows of 4 fixtures, `snr_db` by 1-2 ULP in three
/// and `freq_hz` by 1.9e-4 Hz in `ft4_default` (its `log10f` and `atan2f` return other bits than
/// glibc 2.35's). Data, probe and the account: `docs/notes/snapshot_platform/`. Each limit below is 2.5-5 times the
/// largest measured gap; a changed message, a moved row or a different `pass` is still a failure.
struct Tol {
    freq_hz: f32,
    dt_sec: f32,
    snr_db: f32,
    /// Relative: `sync_score` runs from 1e4 to 1e8.
    sync_score: f32,
    hard_errors: u32,
}

impl Tol {
    const PLATFORM: Tol = Tol {
        freq_hz: 5e-3,
        dt_sec: 1e-5,
        snr_db: 0.1,
        sync_score: 1e-3,
        hard_errors: 2,
    };
    const EXACT: Tol = Tol {
        freq_hz: 0.0,
        dt_sec: 0.0,
        snr_db: 0.0,
        sync_score: 0.0,
        hard_errors: 0,
    };
}

fn tol() -> &'static Tol {
    if std::env::var_os("MFSK_SNAPSHOT_STRICT").is_some() {
        &Tol::EXACT
    } else {
        &Tol::PLATFORM
    }
}

fn near(a: f32, b: f32, abs: f32, rel: f32) -> bool {
    a.to_bits() == b.to_bits() || (a - b).abs() <= abs + rel * a.abs().max(b.abs())
}

/// Compare two row texts (the format of `rows()` / `decoded_rows()`): `Err` says which row
/// and which column.
fn within_tolerance(want: &str, got: &str) -> Result<(), String> {
    let (w, g): (Vec<&str>, Vec<&str>) = (want.lines().collect(), got.lines().collect());
    if w.len() != g.len() {
        return Err(format!("row count {} vs {}", w.len(), g.len()));
    }
    let t = tol();
    let bits = |s: &str| u32::from_str_radix(s, 16).map(f32::from_bits);
    for (i, (w, g)) in w.iter().zip(&g).enumerate() {
        let (wf, gf): (Vec<&str>, Vec<&str>) = (w.split('\t').collect(), g.split('\t').collect());
        if wf.len() != gf.len() || !(wf.len() == 4 || wf.len() == 7) {
            if w != g {
                return Err(format!("row {i}: {w:?} vs {g:?}"));
            }
            continue;
        }
        if wf[0] != gf[0] {
            return Err(format!("row {i}: message {:?} vs {:?}", wf[0], gf[0]));
        }
        // 1 freq_hz, 2 dt_sec, 3 snr_db, then 4 sync_score (frame families only).
        let columns: [(&str, usize, f32, f32); 4] = [
            ("freq_hz", 1, t.freq_hz, 0.0),
            ("dt_sec", 2, t.dt_sec, 0.0),
            ("snr_db", 3, t.snr_db, 0.0),
            ("sync_score", 4, 0.0, t.sync_score),
        ];
        for (name, c, abs, rel) in columns.into_iter().take(if wf.len() == 7 { 4 } else { 3 }) {
            let (a, b) = (
                bits(wf[c]).map_err(|e| format!("row {i}: {name}: {e}"))?,
                bits(gf[c]).map_err(|e| format!("row {i}: {name}: {e}"))?,
            );
            if !near(a, b, abs, rel) {
                return Err(format!("row {i} ({}): {name} {a} vs {b}", wf[0]));
            }
        }
        if wf.len() == 7 {
            let (a, b): (u32, u32) = (
                wf[5].parse().unwrap_or(u32::MAX),
                gf[5].parse().unwrap_or(0),
            );
            if a.abs_diff(b) > t.hard_errors {
                return Err(format!("row {i} ({}): hard_errors {a} vs {b}", wf[0]));
            }
            if wf[6] != gf[6] {
                return Err(format!("row {i} ({}): pass {} vs {}", wf[0], wf[6], gf[6]));
            }
        }
    }
    Ok(())
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
    // `MFSK_SNAPSHOT_REPORT=1`: print each mismatch as a `SNAPDIFF` block and carry on,
    // so one run measures how far this machine is from the fixtures in every case
    // instead of stopping at the first (#579; `scripts/snapshot-diff-stats.py` reads it).
    // Unset, a mismatch fails as before.
    if got != want && std::env::var_os("MFSK_SNAPSHOT_REPORT").is_some() {
        eprintln!("SNAPDIFF {case}\n--- fixture\n{want}--- now\n{got}SNAPEND");
        return;
    }
    if let Err(why) = within_tolerance(&want, &got) {
        panic!("{case}: decode output changed ({why})\n--- fixture\n{want}--- now\n{got}");
    }
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

/// The IQ path as 0.12 shipped it: FT8 and FT4 recordings in one 192 kS/s
/// stream, cut into slots by `IqReceiver` and decoded with the registry's
/// 0.12 search (FT8 100-3000 Hz, sync 0.8, 60 candidates; FT4 300-2700 Hz,
/// sync 1.18, 200). The receiver no longer decodes: one decoder per channel
/// does, and must reproduce those rows.
#[test]
fn iq_receiver_rows() {
    use common::iq::{add_into, interleave, synth_iq};
    use mfsk_core::Mode;
    use mfsk_core::decoder::{AnyDecoder, AnyExtras, DecodeParams, Depth};
    use mfsk_core::iq::{Channelizer, IqReceiver, IqSampleFormat, IqStream};

    const FS: u32 = 192_000;
    const CENTER: f64 = 14_077_000.0;
    let Some(ft8) = load(FT8_WAV) else { return };
    let Some(mut ft4) = load(FT4_WAV) else { return };
    ft4.resize(90_000, 0);
    let (ft8_dial, ft4_dial) = (CENTER + 20_000.0, CENTER - 50_000.0);
    let mut iq = synth_iq(&ft8, FS, CENTER, ft8_dial);
    add_into(&mut iq, &synth_iq(&ft4, FS, CENTER, ft4_dial));
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));

    let legacy = |mode: Mode| {
        let (band, sync, n) = match mode {
            Mode::Ft8 => ((100.0, 3000.0), 0.8, 60),
            _ => ((300.0, 2700.0), 1.18, 200),
        };
        let mut d = AnyDecoder::new(mode, DecodeParams::for_band(band).depth(Depth::Deep));
        match d.extras_mut() {
            AnyExtras::Ft8(e) => {
                e.tuning.sync_min = Some(sync);
                e.tuning.max_cand = Some(n);
            }
            AnyExtras::Ft4(e) => {
                e.tuning.sync_min = Some(sync);
                e.tuning.max_cand = Some(n);
            }
            _ => unreachable!(),
        }
        d
    };

    for (kind, name) in [(Channelizer::Direct, "direct"), (Channelizer::Pfb, "pfb")] {
        let stream = IqStream::new(FS, CENTER, IqSampleFormat::Cf32);
        let mut rx = IqReceiver::with_channelizer(stream, kind).unwrap();
        let c8 = rx.add_channel(ft8_dial, Mode::Ft8).unwrap();
        let c4 = rx.add_channel(ft4_dial, Mode::Ft4).unwrap();
        rx.set_time(1_700_000_010 * 1_000_000_000, 0);
        let mut slots = Vec::new();
        for chunk in interleave(&iq).chunks(2 * 77_777) {
            rx.push_cf32(chunk, &mut slots);
        }
        for (ch, mode, label) in [(c8, Mode::Ft8, "ft8"), (c4, Mode::Ft4, "ft4")] {
            let mut d = legacy(mode);
            let mut rows = Vec::new();
            for slot in slots.iter().filter(|s| s.channel == ch) {
                rows.extend(d.decode(&slot.input()).rows);
            }
            check_text(&format!("iq_{name}_{label}"), decoded_rows(rows));
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

    /// The 0.12 requests always tried the blind `CQ` pass: AP on, with no
    /// station or QSO, which is exactly that and nothing else.
    fn block() -> DecodeParams {
        DecodeParams::for_band((100.0, 3000.0))
            .depth(Depth::Deep)
            .ap(ApMode::Full)
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

    fn f32_rows<R>(rows: Vec<mfsk_core::decoder::Row<R>>) -> String {
        decoded_rows(rows.into_iter().map(|r| r.decoded))
    }

    #[test]
    fn wspr() {
        use mfsk_core::Wspr;
        use mfsk_core::decoder::{SearchTuning, WsprExtras};
        let Some(path) = common::corpus::golden_path("wspr/150426_0918.wav") else {
            common::skip_or_fail("WSPR golden");
            return;
        };
        let Some(a) = common::load_wav_f32_opt(&path) else {
            return;
        };
        // The 0.12 scan is wsprd's defaults: three passes with DT jitter and
        // `-C 10000`. Normal is that, but for the GUI's `-C 500`.
        let params = DecodeParams::for_band((1400.0, 1620.0)).depth(Depth::Normal);
        let extras = WsprExtras {
            search: SearchTuning {
                max_candidates: Some(100),
                ..Default::default()
            },
            max_cycles_per_bit: Some(10_000),
        };
        // One decoder over two periods: the table is state.
        let mut d = Decoder::<Wspr>::new(params).with_extras(extras);
        let first = d.decode(&SlotInput::f32(&a)).rows;
        check_text("wspr_default", f32_rows(first));
        // The second period sees the stations the first confirmed.
        let second = d.decode(&SlotInput::f32(&a)).rows;
        check_text("wspr_table_second", f32_rows(second));
    }

    #[test]
    fn jt9() {
        use mfsk_core::Jt9;
        use mfsk_core::decoder::{Jt9Extras, SearchTuning};
        let Some(a) = common::load_wav_f32_opt(asset_path!("130418_1742.wav")) else {
            common::skip_or_fail("JT9 recording");
            return;
        };
        let params = DecodeParams::for_band((1050.0, 1550.0));
        let extras = Jt9Extras {
            search: SearchTuning {
                time_tolerance_early_sec: Some(1.728),
                time_tolerance_late_sec: Some(1.728),
                score_threshold: Some(0.05),
                max_candidates: Some(200),
            },
        };
        for (depth, name) in [
            (Depth::Fast, "fast"),
            (Depth::Normal, "normal"),
            (Depth::Deep, "deep"),
        ] {
            let mut d =
                Decoder::<Jt9>::new(params.clone().depth(depth)).with_extras(extras.clone());
            let rows = d.decode(&SlotInput::f32(&a)).rows;
            check_text(&format!("jt9_{name}"), f32_rows(rows));
        }
    }

    /// Compare Q65 rows with a fixture whose request used another nominal
    /// start: `dt` is relative to it, so it moves by the difference; every
    /// other column is within [`Tol`].
    fn check_q65(
        case: &str,
        rows: Vec<mfsk_core::decoder::Row<mfsk_core::q65::Q65Result>>,
        dt_shift: f32,
        dt_tol: f32,
        snr_tol_db: f32,
    ) {
        let want = std::fs::read_to_string(fixture_dir().join(format!("{case}.txt"))).unwrap();
        let mut got: Vec<(String, u32, f32, u32)> = rows
            .into_iter()
            .map(|r| {
                let d = r.decoded;
                (d.text, d.freq_hz.to_bits(), d.dt_sec, d.snr_db.to_bits())
            })
            .collect();
        got.sort_by(|a, b| a.0.cmp(&b.0));
        let mut lines: Vec<&str> = want.lines().collect();
        lines.sort();
        assert_eq!(got.len(), lines.len(), "{case}: row count");
        for (g, w) in got.iter().zip(lines) {
            let f: Vec<&str> = w.split('\t').collect();
            assert_eq!(g.0, f[0], "{case}: text");
            let old_freq = f32::from_bits(u32::from_str_radix(f[1], 16).unwrap());
            assert!(
                near(f32::from_bits(g.1), old_freq, tol().freq_hz, 0.0),
                "{case}: freq {} vs {old_freq}",
                f32::from_bits(g.1)
            );
            let old_dt = f32::from_bits(u32::from_str_radix(f[2], 16).unwrap());
            assert!(
                (g.2 - (old_dt + dt_shift)).abs() < dt_tol,
                "{case}: dt {} vs {}",
                g.2,
                old_dt + dt_shift
            );
            let old_snr = f32::from_bits(u32::from_str_radix(f[3], 16).unwrap());
            let snr = f32::from_bits(g.3);
            assert!(
                near(snr, old_snr, snr_tol_db.max(tol().snr_db), 0.0),
                "{case}: snr {snr} vs {old_snr}"
            );
        }
    }

    #[test]
    fn q65() {
        use mfsk_core::decoder::{Q65Extras, SearchTuning};
        use mfsk_core::fec::qra::FadingModel;
        use mfsk_core::q65::{Q65a30, Q65a60, Q65d60, standard_qso_codewords};

        let tune = |early: f32, late: f32, n: usize| SearchTuning {
            time_tolerance_early_sec: Some(early),
            time_tolerance_late_sec: Some(late),
            score_threshold: Some(0.05),
            max_candidates: Some(n),
        };
        // The 0.12 request ran the grid search at its Fast depth.
        let params = DecodeParams::for_band((200.0, 3000.0)).depth(Depth::Fast);

        // 60D EME, single period; the 0.12 request used nominal 0 and
        // +-6 s, the decoder's nominal is the frame's 1.0 s.
        if let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") {
            let a = common::load_wav_f32_opt(&path).unwrap();
            let x = |fading| Q65Extras {
                search: tune(7.0, 5.0, 8),
                fading,
                ..Default::default()
            };
            let rows = Decoder::<Q65d60>::new(params.clone())
                .with_extras(x(None))
                .decode(&SlotInput::f32(&a))
                .rows;
            check_q65("q65_60d_plain", rows, -1.0, 1e-3, 0.0);
            let rows = Decoder::<Q65d60>::new(params.clone())
                .with_extras(x(Some((FadingModel::Gaussian, 10.0))))
                .decode(&SlotInput::f32(&a))
                .rows;
            check_q65("q65_60d_fading", rows, -1.0, 1e-3, 0.0);
        } else {
            common::skip_or_fail("Q65 60D golden");
        }

        // 60A EME with an AP hint; old nominal 30 s +-30 s, now 1.0 s.
        let eme = wavs_in("q65/60A_EME_6m");
        if let Some(a) = eme.first() {
            let extras = Q65Extras {
                search: tune(1.0, 59.0, 16),
                ap_hint: Some(ApHint::new().with_call1("W7GJ")),
                ..Default::default()
            };
            let rows = Decoder::<Q65a60>::new(params.clone())
                .with_extras(extras)
                .decode(&SlotInput::f32(a))
                .rows;
            check_q65("q65_60a_ap_hint", rows, 29.0, 1e-3, 0.0);
        }

        // Averaging over the four periods of 30A ionoscatter: state in the
        // decoder, one `decode` per period. Old nominal 15 s +-15 s, now 0.5 s.
        let slots = wavs_in("q65/30A_Ionoscatter_6m");
        if !slots.is_empty() {
            let averaging = params.clone().averaging(true);
            let x = |ap: bool| Q65Extras {
                search: tune(0.5, 29.5, 8),
                ap_list: if ap {
                    standard_qso_codewords("K1JT", "K9AN", "")
                } else {
                    Vec::new()
                },
                ..Default::default()
            };
            // The AP-list (q3) decode places its `dt` on the grid from the
            // period's start. The 0.12 test passed a mid-period nominal, so
            // that start was 174000 samples late and its `dt` is off by up to
            // one grid step (here 0.0375 s); text and frequency are unaffected
            // and compared exactly. The q3 SNR is measured at the best grid
            // point, so it moves with the grid: 1.7 dB here (-19.4 dB before,
            // -21.1 dB now). The new value is the right one: real `jt9` prints
            // -21 for this period (and -18 for the next q3 decode), `dt` 0.3 -
            // `decoder_depth::q65_q3_snr_and_dt_are_jt9s` checks that. This is
            // the one place this suite does not reproduce a 0.12 number bit
            // for bit, and it is the 0.12 test's geometry that was off, not
            // the decoder.
            for (case, ap, p, tol, snr_tol) in [
                ("q65_30a_averaged", false, averaging.clone(), 1e-3, 0.0),
                (
                    "q65_30a_averaged_ap_list",
                    true,
                    averaging.clone().rx_freq(1010.0),
                    0.05,
                    2.0,
                ),
            ] {
                let mut d = Decoder::<Q65a30>::new(p).with_extras(x(ap));
                let mut all = Vec::new();
                for (n, a) in slots.iter().enumerate() {
                    all.extend(d.decode(&SlotInput::f32(a).period(n as i64)).rows);
                }
                // The batch form de-duplicated across periods.
                let mut kept: Vec<_> = Vec::new();
                for r in all {
                    if !kept.iter().any(|k: &mfsk_core::decoder::Row<_>| {
                        k.decoded.text == r.decoded.text
                            && (k.decoded.freq_hz - r.decoded.freq_hz).abs() <= 4.0
                    }) {
                        kept.push(r);
                    }
                }
                check_q65(case, kept, 14.5, tol, snr_tol);
            }
        }
    }
}

/// The comparator accepts the platform's last-bit noise and refuses a changed decode (#579).
#[test]
fn tolerance_accepts_last_bits_and_refuses_a_changed_decode() {
    let row = |text: &str, snr: f32, sync: f32, he: u32, pass: u32| {
        format!(
            "{text}\t{:08x}\t{:08x}\t{:08x}\t{:08x}\t{he}\t{pass}\n",
            1500.0f32.to_bits(),
            0.25f32.to_bits(),
            snr.to_bits(),
            sync.to_bits()
        )
    };
    let base = row("CQ K1ABC FN42", -9.148_531, 5.0e7, 27, 15);
    let ulp = |x: f32| f32::from_bits(x.to_bits() + 1);
    if std::env::var_os("MFSK_SNAPSHOT_STRICT").is_none() {
        assert!(within_tolerance(&base, &base).is_ok());
        assert!(
            within_tolerance(&base, &row("CQ K1ABC FN42", ulp(-9.148_531), 5.0e7, 27, 15)).is_ok()
        );
        assert!(
            within_tolerance(
                &base,
                &row("CQ K1ABC FN42", -9.148_531 + 0.04, 5.0e7 * 1.0002, 26, 15)
            )
            .is_ok()
        );
    }
    // Always refused, strict or not.
    assert!(within_tolerance(&base, &row("CQ K1ABC FN43", -9.148_531, 5.0e7, 27, 15)).is_err());
    assert!(within_tolerance(&base, &row("CQ K1ABC FN42", -8.5, 5.0e7, 27, 15)).is_err());
    assert!(within_tolerance(&base, &row("CQ K1ABC FN42", -9.148_531, 5.1e7, 27, 15)).is_err());
    assert!(within_tolerance(&base, &row("CQ K1ABC FN42", -9.148_531, 5.0e7, 30, 15)).is_err());
    assert!(within_tolerance(&base, &row("CQ K1ABC FN42", -9.148_531, 5.0e7, 27, 14)).is_err());
    assert!(within_tolerance(&base, &format!("{base}{base}")).is_err());
}
