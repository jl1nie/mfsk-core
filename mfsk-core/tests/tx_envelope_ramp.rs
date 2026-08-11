// SPDX-License-Identifier: GPL-3.0-or-later
//! Every transmit path must ramp its burst envelope (issue #259).
//!
//! Without a ramp, `synthesize_audio` writes `amplitude · cos(phase)`
//! from the first sample to the last, so a transmission begins and
//! ends on a step discontinuity in the envelope — a broadband click at
//! both edges, entirely separate from symbol-transition shaping.
//! Measured on WSPR before the fix, windowing the burst *start*
//! embedded in silence (16384-pt Hann, 12 kHz, dBc relative to the
//! in-band peak):
//!
//! | offset | before | after |
//! |---|---:|---:|
//! | +100 Hz | -46.2 | -55.2 |
//! | +250 Hz | -53.9 | -101.7 |
//! | +500 Hz | -59.5 | -100.3 |
//! | +1000 Hz | -64.7 | -118.4 |
//!
//! Close-in (+50 Hz) barely moves, correctly: that region is set by
//! the CPFSK symbol structure, not by the switch-on transient.
//!
//! Scope note — these four protocols are exactly the ones WSJT-X
//! transmits via `Modulator::modulate`'s plain-CPFSK branch (positive
//! `toneSpacing`). FT8/FT4/FST4 take the pre-computed-waveform branch
//! and have had an equivalent ramp all along, via
//! `engine::dsp::gfsk::GfskCfg::ramp_samples`; they are covered by
//! their own roundtrip tests rather than here. See
//! `engine::dsp::envelope`'s module doc.

#![cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]

/// Envelope must start at (near) zero, reach full scale, and return to
/// (near) zero — checked as a fraction of the waveform's own peak so
/// the assertion is independent of the `amplitude` argument.
fn assert_ramped(name: &str, audio: &[f32], nsps: usize) {
    assert!(
        audio.len() > 4 * nsps,
        "{name}: waveform too short to be meaningful"
    );
    let peak = audio.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    assert!(peak > 0.0, "{name}: waveform is silent");

    // First and last samples sit at the very start of the raised
    // cosine, so they must be a tiny fraction of peak. A un-ramped
    // path fails this immediately: it starts at `amplitude · cos(0)`
    // = full scale.
    assert!(
        audio[0].abs() < 0.01 * peak,
        "{name}: first sample is {:.4} ({:.1}% of peak {peak:.4}) — envelope is not ramped in",
        audio[0],
        100.0 * audio[0].abs() / peak
    );
    let last = *audio.last().unwrap();
    assert!(
        last.abs() < 0.05 * peak,
        "{name}: last sample is {last:.4} ({:.1}% of peak {peak:.4}) — envelope is not ramped out",
        100.0 * last.abs() / peak
    );

    // The ramp must be *short*: the body of the transmission has to
    // still reach full amplitude, or we have attenuated the signal
    // rather than shaped its edges. Checked a full symbol in from each
    // end, which is well past any ramp this crate applies (capped at
    // nsps/8).
    let body_peak = audio[nsps..audio.len() - nsps]
        .iter()
        .fold(0.0f32, |a, &b| a.max(b.abs()));
    assert!(
        body_peak > 0.99 * peak,
        "{name}: body peak {body_peak:.4} is below the overall peak {peak:.4} — \
         the ramp is eating into the transmission body"
    );
}

/// Number of samples per symbol at 12 kHz, from the protocol's own
/// `SYMBOL_DT`, so the "one symbol in" probe above stays correct if a
/// protocol's timing constants change.
fn nsps_at_12k(symbol_dt: f32) -> usize {
    (12_000.0 * symbol_dt).round() as usize
}

#[cfg(feature = "wspr")]
#[test]
fn wspr_tx_envelope_is_ramped() {
    use mfsk_core::engine::ModulationParams;
    let symbols = [1u8; 162];
    let audio = mfsk_core::wspr::tx::synthesize_audio(&symbols, 12_000, 1500.0, 0.5);
    assert_ramped(
        "WSPR",
        &audio,
        nsps_at_12k(<mfsk_core::wspr::Wspr as ModulationParams>::SYMBOL_DT),
    );
}

#[cfg(feature = "jt65")]
#[test]
fn jt65_tx_envelope_is_ramped() {
    use mfsk_core::engine::ModulationParams;
    let tones = [1u8; 126];
    let audio = mfsk_core::jt65::tx::synthesize_audio(&tones, 12_000, 1000.0, 0.5);
    assert_ramped(
        "JT65",
        &audio,
        nsps_at_12k(<mfsk_core::jt65::Jt65 as ModulationParams>::SYMBOL_DT),
    );
}

#[cfg(feature = "jt9")]
#[test]
fn jt9_tx_envelope_is_ramped() {
    use mfsk_core::engine::ModulationParams;
    let tones = [1u8; 85];
    let audio = mfsk_core::jt9::tx::synthesize_audio(&tones, 12_000, 1000.0, 0.5);
    assert_ramped(
        "JT9",
        &audio,
        nsps_at_12k(<mfsk_core::jt9::Jt9 as ModulationParams>::SYMBOL_DT),
    );
}

#[cfg(feature = "q65")]
#[test]
fn q65_tx_envelope_is_ramped() {
    use mfsk_core::engine::ModulationParams;
    let tones = [1u8; 85];
    let audio = mfsk_core::q65::tx::synthesize_audio(&tones, 12_000, 1000.0, 0.5);
    assert_ramped(
        "Q65",
        &audio,
        nsps_at_12k(<mfsk_core::q65::Q65a30 as ModulationParams>::SYMBOL_DT),
    );
}

/// The no-alloc embedded entry point must ramp identically to the
/// `Vec`-returning wrapper — they are separate code paths only in who
/// owns the buffer, and an embedded transmitter is precisely the
/// caller most likely to be driving a real PA.
#[cfg(feature = "wspr")]
#[test]
fn wspr_no_alloc_path_ramps_identically() {
    let symbols = [2u8; 162];
    let owned = mfsk_core::wspr::tx::synthesize_audio(&symbols, 12_000, 1500.0, 0.5);
    let mut buf = vec![0.0f32; mfsk_core::wspr::tx::synthesize_audio_len(12_000)];
    mfsk_core::wspr::tx::synthesize_audio_into(&mut buf, &symbols, 12_000, 1500.0, 0.5);
    assert_eq!(
        buf, owned,
        "synthesize_audio_into must produce the same samples as synthesize_audio"
    );
}

/// uvpacket needs no explicit ramp — its RRC pulse (α = 0.5, spanning
/// several symbols) tapers the burst edges on its own, because the
/// first and last symbols convolve with the pulse tails rather than
/// switching on at full amplitude. Measured: first sample 0.21 % of
/// peak, last 0.00 %, reaching 50 % of peak 27 samples in.
///
/// Pinned anyway so that shortening the RRC span (or moving to a
/// different pulse) can't silently reintroduce the step discontinuity
/// that issue #259 found in the CPFSK paths. This is the one transmit
/// path whose edge behaviour is an emergent property of its pulse
/// shaping rather than an explicit ramp, so it is the one most likely
/// to lose it by accident.
#[cfg(feature = "uvpacket")]
#[test]
fn uvpacket_rrc_tapers_its_own_burst_edges() {
    use mfsk_core::uvpacket::framing::FrameHeader;
    use mfsk_core::uvpacket::puncture::Mode;

    let header = FrameHeader {
        mode: Mode::Standard,
        block_count: 4,
        app_type: 1,
        sequence: 0,
    };
    let audio = mfsk_core::uvpacket::tx::encode(&header, &[0x5A; 4], 1500.0).expect("encode");
    let peak = audio.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    assert!(peak > 0.0, "uvpacket burst is silent");

    assert!(
        audio[0].abs() < 0.01 * peak,
        "uvpacket first sample is {:.1}% of peak — the RRC tails no longer taper the burst start",
        100.0 * audio[0].abs() / peak
    );
    let last = *audio.last().unwrap();
    assert!(
        last.abs() < 0.01 * peak,
        "uvpacket last sample is {:.1}% of peak — the RRC tails no longer taper the burst end",
        100.0 * last.abs() / peak
    );
}
