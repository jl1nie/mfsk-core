//! Every decoded row carries an identity key (`RowDetail::info`), so a
//! mode-generic caller can tell the streamed rows from the returned ones
//! without comparing text (#592).
//!
//! Driven through the public `AnyDecoder::decode_with`, one recording per
//! mode. Before #592 `info` was empty for WSPR, JT9, JT65 and Q65, and a
//! text-keyed guard dropped two of the JT65 golden's three
//! `K1ABC W9XYZ EN37` rows.
#![cfg(feature = "full")]

#[allow(dead_code)]
mod common;

use std::collections::BTreeMap;
use std::sync::Mutex;

use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, SlotInput};

/// A row's identity as a caller can build it from the public row: the key,
/// and the frequency to the Hz, because one message can be on the air at
/// several frequencies (the JT65 golden sends it three times).
type Id = (Vec<u8>, i64);

fn count(ids: impl IntoIterator<Item = Id>) -> BTreeMap<Id, usize> {
    let mut m = BTreeMap::new();
    for id in ids {
        *m.entry(id).or_insert(0) += 1;
    }
    m
}

/// Decode `path` as `mode`; assert each row has a key of `key_len` bits and
/// that the streamed rows cover the returned ones, any extra being a
/// duplicate by the key (`STREAMING.md` §3b).
fn check(mode: Mode, path: &str, key_len: usize) {
    let Some(a) = common::load_wav_f32_opt(path) else {
        common::skip_or_fail(path);
        return;
    };
    let mut d = AnyDecoder::with_defaults(mode);
    let streamed = Mutex::new(Vec::<Id>::new());
    let out = d.decode_with(&SlotInput::f32(&a).period(100), &|row, det| {
        streamed
            .lock()
            .unwrap()
            .push((det.info.clone(), row.freq_hz.round() as i64));
    });
    let name = mode.name();
    assert!(
        !out.rows.is_empty(),
        "{name}: the recording decoded nothing"
    );
    assert_eq!(out.rows.len(), out.details.len());

    let returned: Vec<Id> = out
        .rows
        .iter()
        .zip(&out.details)
        .map(|(r, det)| (det.info.clone(), r.freq_hz.round() as i64))
        .collect();
    for ((key, _), row) in returned.iter().zip(&out.rows) {
        assert_eq!(
            key.len(),
            key_len,
            "{name}: {:?} has a {}-bit key, expected {key_len}",
            row.text,
            key.len()
        );
        assert!(
            key.iter().all(|&b| b <= 1),
            "{name}: key is one bit per byte"
        );
    }

    let streamed = streamed.into_inner().unwrap();
    let (s, r) = (count(streamed), count(returned));
    for (id, &n) in &r {
        let got = s.get(id).copied().unwrap_or(0);
        assert!(
            got >= n,
            "{name}: a returned row was never streamed: {id:?}"
        );
    }
    // Streamed extras exist only as repeats of a row that was returned.
    for id in s.keys() {
        assert!(
            r.contains_key(id),
            "{name}: a streamed row is not in the returned set: {id:?}"
        );
    }
}

#[test]
fn ft8_rows_carry_their_91_information_bits() {
    check(Mode::Ft8, asset_path!("qso3_busy.wav"), 91);
}

#[test]
fn ft4_rows_carry_their_91_information_bits() {
    check(Mode::Ft4, asset_path!("golden/ft4/000000_000002.wav"), 91);
}

#[test]
fn fst4_rows_carry_their_information_bits() {
    check(
        Mode::Fst4S60,
        asset_path!("golden/fst4/210115_0058.wav"),
        101,
    );
}

#[test]
fn wspr_rows_carry_their_50_bits() {
    check(Mode::Wspr, asset_path!("golden/wspr/150426_0918.wav"), 50);
}

#[test]
fn jt9_rows_carry_their_72_bits() {
    check(Mode::Jt9, asset_path!("130418_1742.wav"), 72);
}

/// The golden returns one message at three frequencies; each is its own row,
/// and each streams.
#[test]
fn jt65_rows_carry_their_72_bits_and_the_repeated_message_is_three_rows() {
    let path = asset_path!("golden/jt65/jt65a_5sig_m18.wav");
    check(Mode::Jt65, path, 72);

    let Some(a) = common::load_wav_f32_opt(path) else {
        return;
    };
    let out = AnyDecoder::with_defaults(Mode::Jt65).decode(&SlotInput::f32(&a).period(100));
    let k1abc: Vec<_> = out
        .rows
        .iter()
        .zip(&out.details)
        .filter(|(r, _)| r.text == "K1ABC W9XYZ EN37")
        .collect();
    assert_eq!(k1abc.len(), 3, "{:?}", out.rows);
    assert!(k1abc.iter().all(|(_, d)| d.info == k1abc[0].1.info));
}

/// Q65 needs the search window and nominal start of `decoder_depth.rs`; at the
/// default block most recordings decode nothing.
#[test]
fn q65_rows_carry_their_77_bits() {
    use mfsk_core::decoder::{DecodeParams, Decoder, Q65Extras};
    use mfsk_core::q65::Q65d60;
    let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") else {
        common::skip_or_fail("Q65 60D golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    let mut e = Q65Extras::default();
    e.search.time_tolerance_early_sec = Some(7.0);
    e.search.time_tolerance_late_sec = Some(5.0);
    e.search.score_threshold = Some(0.05);
    e.search.max_candidates = Some(8);
    e.fading = Some((mfsk_core::fec::qra::FadingModel::Gaussian, 10.0));
    let mut d = Decoder::<Q65d60>::new(DecodeParams::for_band((200.0, 3000.0))).with_extras(e);
    let streamed = Mutex::new(Vec::<Vec<u8>>::new());
    let out = d.decode_with(&SlotInput::f32(&a), &|row| {
        streamed.lock().unwrap().push(row.detail.info.clone());
    });
    assert!(!out.rows.is_empty());
    let streamed = streamed.into_inner().unwrap();
    for row in &out.rows {
        assert_eq!(row.detail.info.len(), 77, "{:?}", row.decoded.text);
        assert_eq!(row.detail.info, row.native.bits77.to_vec());
        assert!(streamed.contains(&row.detail.info));
    }
}

// ── delivery_is_exact (#592 B) ───────────────────────────────────────────

use mfsk_core::decoder::{AnyExtras, Depth, Ft4Strategy, Ft8Strategy, Sniper};

fn exact(mode: Mode, depth: Depth) -> bool {
    let mut d = AnyDecoder::with_defaults(mode);
    d.params_mut().depth = depth;
    d.delivery_is_exact()
}

/// `STREAMING.md` §3: sequential for FT8 (every depth: `SicRounds` / `SicEarly`),
/// FT4 `Normal` / `Deep`, JT9, JT65 and Q65; the parallel contract for FT4
/// `Fast`, every FST4 mode and WSPR.
#[test]
fn delivery_is_exact_follows_the_mode_and_depth() {
    for depth in [Depth::Fast, Depth::Normal, Depth::Deep] {
        assert!(exact(Mode::Ft8, depth), "FT8 {depth:?}");
        assert!(exact(Mode::Jt9, depth), "JT9 {depth:?}");
        assert!(exact(Mode::Jt65, depth), "JT65 {depth:?}");
        assert!(exact(Mode::Q65D60, depth), "Q65 {depth:?}");
        assert!(!exact(Mode::Wspr, depth), "WSPR {depth:?}");
        assert!(!exact(Mode::Fst4S60, depth), "FST4 {depth:?}");
    }
    assert!(!exact(Mode::Ft4, Depth::Fast));
    assert!(exact(Mode::Ft4, Depth::Normal));
    assert!(exact(Mode::Ft4, Depth::Deep));
}

/// The settings that move the answer are the extras: a pinned `SinglePass`
/// and FT8's sniper (which needs a target frequency) are the parallel
/// contract; the same decoder is exact again when they go.
#[test]
fn delivery_is_exact_follows_the_extras() {
    let mut d = AnyDecoder::with_defaults(Mode::Ft8);
    assert!(d.delivery_is_exact());
    if let AnyExtras::Ft8(x) = d.extras_mut() {
        x.tuning.strategy = Some(Ft8Strategy::SinglePass);
    }
    assert!(!d.delivery_is_exact(), "FT8 SinglePass");
    if let AnyExtras::Ft8(x) = d.extras_mut() {
        x.tuning.strategy = None;
        x.sniper = Some(Sniper::default());
    }
    assert!(
        d.delivery_is_exact(),
        "a sniper with no target is not sniping"
    );
    d.params_mut().rx_freq_hz = Some(1500.0);
    assert!(!d.delivery_is_exact(), "FT8 sniper");
    if let AnyExtras::Ft8(x) = d.extras_mut() {
        x.sniper = None;
    }
    assert!(d.delivery_is_exact());

    let mut d = AnyDecoder::with_defaults(Mode::Ft4);
    if let AnyExtras::Ft4(x) = d.extras_mut() {
        x.tuning.strategy = Some(Ft4Strategy::SinglePass);
    }
    assert!(!d.delivery_is_exact(), "FT4 SinglePass at Deep");
}

/// What the flag promises, on the recordings: when it is `true`, the streamed
/// rows are the returned rows, once each, in the same order.
#[test]
fn when_delivery_is_exact_the_stream_is_the_returned_rows_in_order() {
    let cases = [
        (Mode::Ft8, asset_path!("qso3_busy.wav"), Depth::Fast),
        (Mode::Ft8, asset_path!("qso3_busy.wav"), Depth::Normal),
        (Mode::Ft8, asset_path!("qso3_busy.wav"), Depth::Deep),
        (
            Mode::Ft4,
            asset_path!("golden/ft4/000000_000002.wav"),
            Depth::Normal,
        ),
        (
            Mode::Ft4,
            asset_path!("golden/ft4/000000_000002.wav"),
            Depth::Deep,
        ),
        (Mode::Jt9, asset_path!("130418_1742.wav"), Depth::Deep),
        (
            Mode::Jt65,
            asset_path!("golden/jt65/jt65a_5sig_m18.wav"),
            Depth::Deep,
        ),
    ];
    for (mode, path, depth) in cases {
        let Some(a) = common::load_wav_f32_opt(path) else {
            common::skip_or_fail(path);
            continue;
        };
        let mut d = AnyDecoder::with_defaults(mode);
        d.params_mut().depth = depth;
        assert!(d.delivery_is_exact(), "{} {depth:?}", mode.name());
        let streamed = Mutex::new(Vec::<Id>::new());
        let out = d.decode_with(&SlotInput::f32(&a).period(100), &|row, det| {
            streamed
                .lock()
                .unwrap()
                .push((det.info.clone(), row.freq_hz.round() as i64));
        });
        let returned: Vec<Id> = out
            .rows
            .iter()
            .zip(&out.details)
            .map(|(r, det)| (det.info.clone(), r.freq_hz.round() as i64))
            .collect();
        assert!(
            !returned.is_empty(),
            "{} {depth:?} decoded nothing",
            mode.name()
        );
        assert_eq!(
            streamed.into_inner().unwrap(),
            returned,
            "{} {depth:?}",
            mode.name()
        );
    }
}

// ── what a row reports (#594) ───────────────────────────────────────────────

/// `(sync_score, sync_cv, hard_errors)` present, of every returned row.
fn provided(mode: Mode, path: &str) -> Option<Vec<(bool, bool, bool)>> {
    let a = common::load_wav_f32_opt(path)?;
    let out = AnyDecoder::with_defaults(mode).decode(&SlotInput::f32(&a).period(100));
    assert!(!out.details.is_empty(), "{}: nothing decoded", mode.name());
    Some(
        out.details
            .iter()
            .map(|d| {
                (
                    d.sync_score.is_some(),
                    d.sync_cv.is_some(),
                    d.hard_errors.is_some(),
                )
            })
            .collect(),
    )
}

/// FT8, FT4 and FST4 report all three; WSPR, JT9 and JT65 report none, so a
/// consumer can tell a measured `0` from a mode that has no such number.
#[test]
fn a_row_says_which_of_its_numbers_the_mode_reports() {
    let cases = [
        (Mode::Ft8, asset_path!("qso3_busy.wav"), (true, true, true)),
        (
            Mode::Ft4,
            asset_path!("golden/ft4/000000_000002.wav"),
            (true, true, true),
        ),
        (
            Mode::Fst4S60,
            asset_path!("golden/fst4/210115_0058.wav"),
            (true, true, true),
        ),
        (
            Mode::Wspr,
            asset_path!("golden/wspr/150426_0918.wav"),
            (false, false, false),
        ),
        (
            Mode::Jt9,
            asset_path!("130418_1742.wav"),
            (false, false, false),
        ),
        (
            Mode::Jt65,
            asset_path!("golden/jt65/jt65a_5sig_m18.wav"),
            (false, false, false),
        ),
    ];
    for (mode, path, want) in cases {
        let Some(rows) = provided(mode, path) else {
            common::skip_or_fail(path);
            continue;
        };
        // FT8's a7 rows (no sync search) are the one exception inside a mode.
        let searched: Vec<_> = rows.iter().filter(|r| r.0 == want.0).collect();
        assert!(!searched.is_empty(), "{}: {rows:?}", mode.name());
        for r in &rows {
            assert_eq!(r.2, want.2, "{}: hard_errors {rows:?}", mode.name());
        }
    }
}

#[test]
fn q65_rows_report_no_sync_or_error_count() {
    use mfsk_core::decoder::{DecodeParams, Decoder, Q65Extras};
    use mfsk_core::q65::Q65d60;
    let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") else {
        common::skip_or_fail("Q65 60D golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    let mut e = Q65Extras::default();
    e.search.time_tolerance_early_sec = Some(7.0);
    e.search.time_tolerance_late_sec = Some(5.0);
    e.search.score_threshold = Some(0.05);
    e.search.max_candidates = Some(8);
    e.fading = Some((mfsk_core::fec::qra::FadingModel::Gaussian, 10.0));
    let out = Decoder::<Q65d60>::new(DecodeParams::for_band((200.0, 3000.0)))
        .with_extras(e)
        .decode(&SlotInput::f32(&a));
    assert!(!out.rows.is_empty());
    for r in &out.rows {
        assert_eq!(
            (r.detail.sync_score, r.detail.sync_cv, r.detail.hard_errors),
            (None, None, None)
        );
    }
}
