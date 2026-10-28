//! A parameter block WSJT-X's GUI could not send (its `nfa`, `nfb`, `nfqso` and
//! `ntol` are integers) neither crashes nor hangs a decoder, and decodes what the
//! normalised block decodes (#717).
//!
//! Before #717, on one strong synthetic signal per mode (`ed97d013`): an infinite
//! band top panicked FT8 and FST4 (debug; release returned no row) and Q65
//! (release), and JT9 and JT65 did not return within 90 s; a NaN Rx frequency or
//! tolerance returned no row from FST4W.
#![cfg(all(feature = "full", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::decoder::default_params;
use mfsk_core::prelude::*;

fn slot(mode: Mode, text: &str, f0: f32) -> Vec<f32> {
    let n = (mode.meta().t_slot_s * 12_000.0).round() as usize;
    let frame = encode(mode, text)
        .unwrap()
        .synthesize(&TxAudio::new(12_000, f0));
    let mut audio = vec![0.0f32; n];
    let len = frame.len().min(n - 6_000);
    audio[6_000..6_000 + len].copy_from_slice(&frame[..len]);
    audio
}

fn rows(mode: Mode, p: DecodeParams, audio: &[f32]) -> Vec<(String, i32)> {
    let mut d = AnyDecoder::new(mode, p);
    let r = d.decode(&SlotInput::f32(audio).period(0));
    r.rows
        .iter()
        .map(|r| (r.decoded.text.clone(), r.decoded.freq_hz.round() as i32))
        .collect()
}

#[test]
fn values_the_gui_cannot_send_decode_as_their_normalised_block() {
    let modes = [
        (Mode::Ft8, "CQ JA1ABC PM95", 1500.0),
        (Mode::Ft4, "CQ JA1ABC PM95", 1500.0),
        (Mode::Fst4S60, "CQ JA1ABC PM95", 1000.0),
        (Mode::Fst4W120, "JA1ABC PM95 30", 1500.0),
        (Mode::Wspr, "JA1ABC PM95 30", 1500.0),
        (Mode::Jt9, "CQ JA1ABC PM95", 1500.0),
        (Mode::Jt65, "CQ JA1ABC PM95", 1500.0),
        (Mode::Q65A60, "CQ JA1ABC PM95", 1500.0),
    ];
    for (mode, text, f0) in modes {
        let audio = slot(mode, text, f0);
        let base = default_params(mode);
        let bad = base
            .clone()
            .band(f32::NAN, f32::INFINITY)
            .rx_freq(f32::NAN)
            .tol(-50.0)
            .tx_freq(f32::INFINITY);
        // What `bad` normalises to: the whole baseband, nothing else set.
        let same = base.band(0.0, 6_000.0);
        let want = rows(mode, same, &audio);
        assert!(
            want.iter().any(|(t, _)| t.contains("JA1ABC")),
            "{}: the normalised block must decode the signal: {want:?}",
            mode.name()
        );
        assert_eq!(rows(mode, bad, &audio), want, "{}", mode.name());
    }
}
