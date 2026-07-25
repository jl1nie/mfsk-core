//! JT65 transmitter: 72-bit message → 126 channel tones → audio.
//!
//! Mirrors the stages in WSJT-X `jt65sim.f90` lines 172–190:
//! 1. Pack the message into 12 × 6-bit symbols (`Jt72` words).
//! 2. Encode with RS(63, 12) using the JT65 byte ordering
//!    (`Rs63_12::encode_jt65`) → 63 codeword symbols.
//! 3. Interleave (7×9 transpose).
//! 4. Gray-code each 6-bit symbol.
//! 5. Splice into the 126-slot frame: sync positions emit tone 0,
//!    data positions emit `gray(sent[k]) + 2`.
//! 6. Emit CPFSK audio at the JT65A baud (≈ 2.69 Hz tone spacing).

use core::f32::consts::TAU;

use crate::core::ModulationParams;
use crate::fec::Rs63_12;

use super::gray::gray6;
use super::interleave::interleave;
use super::sync_pattern::JT65_NPRC;
use super::{Jt65, Jt65Shorthand, SyncPolarity};

/// Encode a 12-symbol info payload into 126 channel tones
/// (values 0 or 2..=65 where 0 = sync, 2..=65 = data + 2).
pub fn encode_channel_symbols(info: &[u8; 12]) -> [u8; 126] {
    encode_channel_symbols_with_polarity(info, SyncPolarity::Normal)
}

/// Encode a normal JT65 frame or the inverted-sync `OOO` report form.
pub fn encode_channel_symbols_with_polarity(info: &[u8; 12], polarity: SyncPolarity) -> [u8; 126] {
    let rs = Rs63_12::new();
    let mut sent = rs.encode_jt65(info);
    interleave(&mut sent);
    for s in sent.iter_mut() {
        *s = gray6(*s);
    }
    let mut tones = [0u8; 126];
    let mut k = 0usize;
    for i in 0..126 {
        let is_sync = match polarity {
            SyncPolarity::Normal => JT65_NPRC[i] == 1,
            SyncPolarity::Inverted => JT65_NPRC[i] == 0,
        };
        if is_sync {
            tones[i] = 0; // sync
        } else {
            // Data tones are the 64 Gray-coded values, offset by +2
            // (WSJT-X `jt65sim.f90` line 186: itone(j)=sent(k)+2).
            tones[i] = sent[k] + 2;
            k += 1;
        }
    }
    debug_assert_eq!(k, 63, "data positions must total 63");
    tones
}

/// Synthesize JT65A audio: one CPFSK tone per symbol at
/// `base_freq + tone * 2.69 Hz`. `base_freq` is the frequency of
/// tone 0 (the sync tone).
pub fn synthesize_audio(
    tones: &[u8; 126],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    synthesize_audio_for::<Jt65>(tones, sample_rate, base_freq_hz, amplitude)
}

/// Synthesize one JT65 sub-mode selected by protocol marker `P`.
pub fn synthesize_audio_for<P: ModulationParams>(
    tones: &[u8; 126],
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let tone_spacing = P::TONE_SPACING_HZ;
    let mut out = Vec::with_capacity(nsps * 126);
    let mut phase = 0.0f32;
    for &sym in tones {
        assert!(sym <= 65, "JT65 tone must be in 0..=65");
        let freq = base_freq_hz + sym as f32 * tone_spacing;
        let dphi = TAU * freq / sample_rate as f32;
        for _ in 0..nsps {
            out.push(amplitude * phase.cos());
            phase += dphi;
            if phase > TAU {
                phase -= TAU;
            } else if phase < -TAU {
                phase += TAU;
            }
        }
    }
    out
}

/// Build the 126-symbol two-tone EME shorthand waveform used for
/// `RO`, `RRR`, and `73`.
///
/// Pinned WSJT-X `gen65.f90` alternates groups of four symbols between
/// tone 0 and tone `10 * nspecial`, where `nspecial` is 2, 3, or 4.
pub fn encode_shorthand_symbols(message: Jt65Shorthand) -> [u8; 126] {
    let upper_tone = 10 * message.code();
    let mut tones = [0u8; 126];
    for (index, tone) in tones.iter_mut().enumerate() {
        if (index / 4) % 2 == 1 {
            *tone = upper_tone;
        }
    }
    tones
}

/// Synthesize an EME shorthand for one JT65 sub-mode.
pub fn synthesize_shorthand_for<P: ModulationParams>(
    message: Jt65Shorthand,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    synthesize_audio_for::<P>(
        &encode_shorthand_symbols(message),
        sample_rate,
        base_freq_hz,
        amplitude,
    )
}

/// Convenience: pack a standard message via `Jt72` and synthesize.
pub fn synthesize_standard(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    synthesize_standard_for::<Jt65>(
        call1,
        call2,
        grid_or_report,
        sample_rate,
        base_freq_hz,
        amplitude,
    )
}

/// Pack and synthesize a standard message for JT65 sub-mode `P`.
pub fn synthesize_standard_for<P: ModulationParams>(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    synthesize_standard_for_with_polarity::<P>(
        call1,
        call2,
        grid_or_report,
        sample_rate,
        base_freq_hz,
        amplitude,
        SyncPolarity::Normal,
    )
}

/// Pack and synthesize a standard message with explicit JT65 sync polarity.
pub fn synthesize_standard_for_with_polarity<P: ModulationParams>(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
    polarity: SyncPolarity,
) -> Option<Vec<f32>> {
    let words = crate::msg::jt72::pack_standard(call1, call2, grid_or_report)?;
    let tones = encode_channel_symbols_with_polarity(&words, polarity);
    Some(synthesize_audio_for::<P>(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

/// Parse and synthesize one complete JT65 message exactly as entered in
/// WSJT-X: a normal packed message, an `OOO` inverted-sync report, or one
/// of the two-tone EME shorthands `RO`, `RRR`, and `73`.
pub fn synthesize_message_for<P: ModulationParams>(
    message: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let message = message.trim().to_ascii_uppercase();
    if let Some(shorthand) = Jt65Shorthand::parse(&message) {
        return Some(synthesize_shorthand_for::<P>(
            shorthand,
            sample_rate,
            base_freq_hz,
            amplitude,
        ));
    }

    let mut fields = message.split_ascii_whitespace().collect::<Vec<_>>();
    let polarity = if fields.last() == Some(&"OOO") {
        fields.pop();
        SyncPolarity::Inverted
    } else {
        SyncPolarity::Normal
    };
    let words = if polarity == SyncPolarity::Inverted {
        if !(2..=3).contains(&fields.len()) {
            return None;
        }
        crate::msg::jt72::pack_standard(fields[0], fields[1], fields.get(2).copied().unwrap_or(""))?
    } else {
        crate::msg::jt72::pack_message(&message)
    };
    let tones = encode_channel_symbols_with_polarity(&words, polarity);
    Some(synthesize_audio_for::<P>(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_splits_63_sync_63_data() {
        let info = [0u8; 12];
        let tones = encode_channel_symbols(&info);
        let sync_count = tones.iter().filter(|&&t| t == 0).count();
        assert_eq!(sync_count, 63, "expected exactly 63 sync tones");
        let data_count = tones.iter().filter(|&&t| (2..=65).contains(&t)).count();
        assert_eq!(data_count, 63, "expected exactly 63 data tones");
    }

    #[test]
    fn synthesize_produces_expected_length() {
        let tones = [0u8; 126];
        let audio = synthesize_audio(&tones, 12_000, 1270.0, 0.3);
        assert_eq!(audio.len(), 4460 * 126);
    }

    #[test]
    fn synthesize_standard_message_ok() {
        let audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3).expect("pack + synth");
        assert_eq!(audio.len(), 4460 * 126);
    }

    #[test]
    fn shorthand_tones_match_pinned_wsjtx_gen65() {
        for (message, upper_tone) in [
            (Jt65Shorthand::Ro, 20),
            (Jt65Shorthand::Rrr, 30),
            (Jt65Shorthand::SeventyThree, 40),
        ] {
            let tones = encode_shorthand_symbols(message);
            for (index, tone) in tones.into_iter().enumerate() {
                assert_eq!(tone, if (index / 4) % 2 == 0 { 0 } else { upper_tone });
            }
        }
    }

    #[test]
    fn message_parser_handles_ooo_and_shorthand() {
        let ooo = synthesize_message_for::<Jt65>("KA1ABC WB9XYZ EN34 OOO", 12_000, 1_270.0, 0.3)
            .expect("OOO message");
        let shorthand =
            synthesize_message_for::<Jt65>("RRR", 12_000, 1_270.0, 0.3).expect("shorthand");
        assert_eq!(ooo.len(), 4460 * 126);
        assert_eq!(shorthand.len(), 4460 * 126);
    }

    #[test]
    fn message_parser_transmits_free_text() {
        let audio = synthesize_message_for::<Jt65>("TNX JOE -14 73", 12_000, 1_270.0, 0.3)
            .expect("free-text message");
        let decoded =
            super::super::decode_at(&audio, 12_000, 0, 1_270.0).expect("decode free-text synth");
        assert_eq!(decoded.to_string(), "TNX JOE -14 7");
    }
}
