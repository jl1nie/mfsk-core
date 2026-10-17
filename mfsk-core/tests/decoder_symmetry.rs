//! `Decoder<P>` and `AnyDecoder` offer the same things (API 0.14 §9): the
//! same rows, the same `decode_i16`, the read-only extras, one AP-hint setter,
//! and a known-call list that says why it refused.
#![cfg(feature = "full")]

#[allow(dead_code)]
mod common;

use std::sync::Mutex;

use mfsk_core::decoder::{AnyDecoder, AnyExtras, AnyExtrasRef, Decoder, SlotInput, WcallsError};
use mfsk_core::msg::{ApHint, Decoded};
use mfsk_core::{Ft8, Jt65, Mode};

fn pairs<R>(rows: &[mfsk_core::decoder::Row<R>]) -> Vec<(Decoded, mfsk_core::decoder::RowDetail)> {
    rows.iter()
        .map(|r| (r.decoded.clone(), r.detail.clone()))
        .collect()
}

#[test]
fn decode_i16_is_decode_of_an_i16_slot() {
    let Some(a) = common::load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let via_slot = Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&a).period(7));
    let via_i16 = Decoder::<Ft8>::with_defaults().decode_i16(&a, Some(7));
    assert!(!via_slot.rows.is_empty());
    assert_eq!(pairs(&via_slot.rows), pairs(&via_i16.rows));
}

/// The erased rows are the typed rows, field for field, and `decode_with`
/// delivers them in the same order.
fn any_rows_are_typed_rows<P>(mode: Mode, path: &str)
where
    P: mfsk_core::decoder::Decodable,
{
    let Some(a) = common::load_wav_f32_opt(path) else {
        common::skip_or_fail(path);
        return;
    };
    let slot = SlotInput::f32(&a).period(100);
    let typed = Decoder::<P>::with_defaults().decode(&slot);
    let any = AnyDecoder::with_defaults(mode).decode(&slot);
    assert!(!typed.rows.is_empty(), "{}", mode.name());
    assert_eq!(pairs(&typed.rows), pairs(&any.rows), "{}", mode.name());

    let seen = Mutex::new(Vec::new());
    let out = AnyDecoder::with_defaults(mode).decode_with(&slot, &|r| {
        seen.lock()
            .unwrap()
            .push((r.decoded.clone(), r.detail.clone()));
    });
    // `decode_with` names each returned row's delivery; nothing else differs.
    let mut with = pairs(&out.rows);
    assert!(with.iter().all(|r| r.1.delivery.is_some()));
    with.iter_mut().for_each(|r| r.1.delivery = None);
    assert_eq!(with, pairs(&any.rows), "{}", mode.name());
    let seen = seen.into_inner().unwrap();
    // Delivery may repeat a row under a parallel strategy; none may be new.
    assert!(
        seen.iter()
            .all(|s| any.rows.iter().any(|r| r.decoded.text == s.0.text))
    );
    assert!(!seen.is_empty(), "{}", mode.name());
}

#[test]
fn any_rows_equal_typed_rows_ft8() {
    any_rows_are_typed_rows::<Ft8>(Mode::Ft8, asset_path!("qso3_busy.wav"));
}

#[test]
fn any_rows_equal_typed_rows_jt65() {
    any_rows_are_typed_rows::<Jt65>(Mode::Jt65, asset_path!("golden/jt65/jt65a_5sig_m18.wav"));
}

#[test]
fn extras_reflects_extras_mut() {
    let mut d = AnyDecoder::with_defaults(Mode::Ft8);
    assert!(matches!(d.extras(), AnyExtrasRef::Ft8(e) if !e.a7));
    match d.extras_mut() {
        AnyExtras::Ft8(e) => e.a7 = true,
        other => panic!("{other:?}"),
    }
    assert!(matches!(d.extras(), AnyExtrasRef::Ft8(e) if e.a7));
}

#[test]
fn set_ap_hint_is_the_same_on_both() {
    let mut hint = ApHint::default();
    hint.call2 = Some("JA1ABC".into());

    let mut typed = Decoder::<Ft8>::with_defaults();
    typed.set_ap_hint(Some(hint.clone()));
    let mut any = AnyDecoder::with_defaults(Mode::Ft8);
    any.set_ap_hint(Some(hint)).unwrap();
    let AnyExtrasRef::Ft8(e) = any.extras() else {
        panic!("Ft8")
    };
    assert_eq!(
        typed.extras().ap_hint.as_ref().unwrap().call2,
        e.ap_hint.as_ref().unwrap().call2
    );
    assert_eq!(e.ap_hint.as_ref().unwrap().call2.as_deref(), Some("JA1ABC"));

    // Same rows on a recording.
    let Some(a) = common::load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let t = typed.decode_i16(&a, Some(1));
    let n = any.decode_i16(&a, Some(1));
    assert_eq!(pairs(&t.rows), pairs(&n.rows));

    typed.set_ap_hint(None);
    assert!(typed.extras().ap_hint.is_none());
    // A mode without the field refuses, by name.
    assert!(
        AnyDecoder::with_defaults(Mode::Wspr)
            .set_ap_hint(None)
            .is_err()
    );
}

#[test]
fn set_wcalls_says_why() {
    let many: Vec<String> = (0..101).map(|i| format!("K{i}ABC FN42")).collect();
    let mut w = AnyDecoder::with_defaults(Mode::Fst4W120);
    assert!(matches!(
        w.set_wcalls(&many),
        Err(WcallsError::TooManyCalls(_))
    ));
    assert!(w.wcalls().unwrap().is_empty());
    w.set_wcalls(&many[..2]).unwrap();
    assert_eq!(w.wcalls().unwrap(), &many[..2]);

    let mut f = AnyDecoder::with_defaults(Mode::Ft8);
    assert!(matches!(
        f.set_wcalls(&many[..1]),
        Err(WcallsError::Unsupported(_))
    ));
    assert!(f.wcalls().is_err());
    let e = f.set_wcalls(&[]).unwrap_err();
    assert!(std::error::Error::source(&e).is_some());
    assert!(!e.to_string().is_empty());
}
