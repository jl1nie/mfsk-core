// SPDX-License-Identifier: GPL-3.0-or-later
//! Synthetic wideband IQ from 12 kHz audio, shared by the `iq_*` tests.

use mfsk_core::engine::dsp::polyphase::PolyphaseResampler;

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// `audio` (12 kHz) as double-sideband IQ at `fs` centred on `center_hz`,
/// with the audio's 0 Hz at `dial_hz`. The lower sideband lands below the
/// dial, where a front end that leaks would show it.
pub fn synth_iq(audio: &[i16], fs: u32, center_hz: f64, dial_hz: f64) -> Vec<(f32, f32)> {
    let g = gcd(fs as u64, 12_000);
    let (l, m) = ((fs as u64 / g) as u32, (12_000 / g) as u32);
    let mut rs = PolyphaseResampler::new(l, m, 32 * l as usize + 1, 4096);
    let (mut ri, mut rq) = (Vec::new(), Vec::new());
    for &s in audio {
        rs.push(s as f32 / 32_768.0, 0.0, &mut ri, &mut rq);
    }
    // Drop the upsampler's group delay so the audio keeps its timing.
    let skip = rs.group_delay_output();
    let w = std::f64::consts::TAU * (dial_hz - center_hz) / fs as f64;
    ri.iter()
        .skip(skip)
        .enumerate()
        .map(|(n, &a)| {
            let p = w * n as f64;
            (a * p.cos() as f32, a * p.sin() as f32)
        })
        .collect()
}

/// `dst += src`, growing `dst` with zeros as needed.
pub fn add_into(dst: &mut Vec<(f32, f32)>, src: &[(f32, f32)]) {
    if dst.len() < src.len() {
        dst.resize(src.len(), (0.0, 0.0));
    }
    for (d, s) in dst.iter_mut().zip(src) {
        d.0 += s.0;
        d.1 += s.1;
    }
}

/// Interleave `iq` as `f32` I/Q.
pub fn interleave(iq: &[(f32, f32)]) -> Vec<f32> {
    iq.iter().flat_map(|&(i, q)| [i, q]).collect()
}
