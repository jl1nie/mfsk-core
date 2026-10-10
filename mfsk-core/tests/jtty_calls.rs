//! #650: a JTTY message carries the callsigns of its call atoms, in arrival
//! order and each once, so a station list takes them without parsing text.
#![cfg(all(feature = "jtty", feature = "fft-rustfft"))]

use std::sync::Arc;

use mfsk_core::jtty::assemble::UpdateKind;
use mfsk_core::jtty::rx::{Params, Receiver, Stream};
use mfsk_core::jtty::source::{Atom, CallAction};
use mfsk_core::jtty::tx;

fn updates_for(atoms: &[Atom]) -> Vec<mfsk_core::jtty::assemble::MessageUpdate> {
    let tones = tx::tones(atoms).expect("encodable");
    let mut audio: Vec<i16> = vec![0; 12_000];
    audio.extend(
        tx::synth_f32(&tones, 1500.0, 3000.0)
            .iter()
            .map(|&x| x as i16),
    );
    audio.extend(std::iter::repeat_n(0, 4 * 12_000));
    let mut stream = Stream::new(Arc::new(Receiver::new()), Params::default());
    let mut updates = Vec::new();
    for chunk in audio.chunks(4096) {
        stream.push(chunk, &mut |u| updates.push(u));
    }
    stream.finish(&mut |u| updates.push(u));
    updates
}

#[test]
fn a_message_carries_its_call_atoms_once_each_in_order() {
    let atoms = [
        Atom::call(CallAction::Call, "JA1ABC"),
        Atom::call(CallAction::Call, "K1ABC"),
        Atom::call(CallAction::Call, "JA1ABC"),
    ];
    let ups = updates_for(&atoms);
    let done = ups
        .iter()
        .find(|u| u.kind == UpdateKind::Complete)
        .unwrap_or_else(|| panic!("no complete message: {ups:?}"));
    assert_eq!(done.calls, ["JA1ABC", "K1ABC"], "{:?}", done.text);
    // Each growing update has the calls arrived so far, never more.
    for u in ups.iter().filter(|u| u.id == done.id) {
        assert!(done.calls.starts_with(&u.calls), "{u:?}");
    }
}

#[test]
fn a_cq_carries_its_call() {
    let atoms = [Atom::call(CallAction::Cq, "K1ABC")];
    let ups = updates_for(&atoms);
    assert!(ups.iter().any(|u| u.text.contains("CQ K1ABC")), "{ups:?}");
    // A CQ is still a call atom: its call is the station's.
    assert!(ups.iter().all(|u| u.calls == ["K1ABC"]), "{ups:?}");
}
