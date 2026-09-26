//! The GFSK synthesiser against WSJT-X's own noiseless waveform (#482).
//!
//! `ft8sim "K1ABC W9XYZ EN37" 1500 0 0 0 1 99` (SNR > 90: no noise, peak
//! normalised to 32767, frame at 0.5 s) is compared sample by sample with
//! this crate's FT8 synthesis of the same message, both peak-normalised.
//! Set `MFSK_FT8SIM_NOISELESS_WAV` to that file; the test prints the worst
//! normalised error and skips when unset.

#![cfg(feature = "ft8")]

#[allow(dead_code)]
mod common;

use mfsk_core::ft8::Ft8;

#[test]
#[ignore]
fn ft8_matches_ft8sim_noiseless() {
    let Ok(path) = std::env::var("MFSK_FT8SIM_NOISELESS_WAV") else {
        return;
    };
    let up = common::load_wav_f32(&path);
    let msg = mfsk_core::msg::wsjt77::pack77("K1ABC", "W9XYZ", "EN37").unwrap();
    let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&msg);
    let ours = mfsk_core::engine::tx::synthesize::<Ft8>(&tones, 12_000, 1500.0, 1.0);
    let off = 6_000;
    let peak_up = up.iter().fold(0f32, |a, &b| a.max(b.abs()));
    let peak_ours = ours.iter().fold(0f32, |a, &b| a.max(b.abs()));
    let mut worst = 0f32;
    let mut at = 0;
    for (k, &o) in ours.iter().enumerate() {
        let e = (o / peak_ours - up[off + k] / peak_up).abs();
        if e > worst {
            worst = e;
            at = k;
        }
    }
    println!("worst normalised error {worst:.3e} at sample {at}");
}
