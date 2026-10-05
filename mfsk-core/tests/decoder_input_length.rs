//! No mode panics on a period of the wrong length.
//!
//! `SlotInput` is documented as one whole period, but nothing enforces it,
//! and live callers do not always have one: a receiver that attaches
//! mid-slot, a controller that dispatches once half a slot is in (#567's
//! reporter), a sound card that drops a block, or a short WAV. #567 was the
//! FT8 / FT4 subtract indexing past a buffer shorter than the frame. It had
//! been there since 0.8.0, because every test, corpus and the crate's own
//! `SlotCutter` only ever hand over full slots.
//!
//! This drives every mode the build has through `AnyDecoder` — the path the
//! C ABI and the IQ receiver's callers take. Each period holds one real frame
//! of the mode under low-level noise, so the decoders find something and the
//! paths after a decode (subtraction, hash learning, averaging) run; on
//! noise alone they never would, and this test did not catch #567 until it
//! carried a signal. Each length is cut two ways: the period's start (a
//! slot dispatched early) and its end (a receiver that attached late, so
//! the frame starts before the buffer and `dt` is negative). Consecutive
//! periods go through one decoder, so state carried between periods also
//! sees mixed lengths.
//!
//! Decode counts are not checked on the cut periods. A panic is a failure,
//! and so is a full period that decodes nothing: then the paths this exists
//! for were never reached.

use std::panic::{AssertUnwindSafe, catch_unwind};

use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::engine::protocol::{FrameLayout, Protocol};
use mfsk_core::engine::tx::{FskWaveform, message_to_tones, synthesize};
use mfsk_core::msg::wsjt77::pack77;

const FS: u32 = 12_000;
/// Peak amplitude of the frame, against noise of about ±300.
const AMP: f32 = 3_000.0;

/// One frame of a 77-bit mode at its nominal start, in an otherwise empty
/// period of `slot` samples.
fn frame77<P>(slot: usize, freq_hz: f32) -> Vec<f32>
where
    P: Protocol + FskWaveform,
{
    let msg = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
    let tones = message_to_tones::<P>(&msg);
    place::<P>(synthesize::<P>(&tones, FS, freq_hz, 1.0), slot)
}

/// `wave` at `P`'s nominal start in a period of `slot` samples.
fn place<P: FrameLayout>(wave: Vec<f32>, slot: usize) -> Vec<f32> {
    let start = (P::TX_START_OFFSET_S * FS as f32).round() as usize;
    let mut v = vec![0.0f32; slot];
    for (i, s) in wave.into_iter().enumerate() {
        if let Some(d) = v.get_mut(start + i) {
            *d = s;
        }
    }
    v
}

/// The period each mode is tested with: one frame of it, or `None` for a
/// mode this file does not know how to synthesise (it is then run on noise
/// only, and the test says so).
fn period_with_signal(mode: Mode, slot: usize) -> Option<Vec<f32>> {
    use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
    use mfsk_core::q65::tx::synthesize_standard_for;
    use mfsk_core::q65::{
        Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60, Q65e120,
    };
    use mfsk_core::{Jt9, Jt65, Wspr};

    fn q65<P: FrameLayout + FskWaveform>(slot: usize) -> Option<Vec<f32>> {
        let w = synthesize_standard_for::<P>("CQ", "JA1ABC", "PM95", FS, 1_500.0, 1.0)?;
        Some(place::<P>(w, slot))
    }

    Some(match mode.name() {
        "FT8" => frame77::<mfsk_core::Ft8>(slot, 1_500.0),
        "FT4" => frame77::<mfsk_core::Ft4>(slot, 1_500.0),
        // FST4's default band is 600–1400 Hz.
        "FST4-15" => frame77::<Fst4s15>(slot, 1_000.0),
        "FST4-30" => frame77::<Fst4s30>(slot, 1_000.0),
        "FST4-60A" => frame77::<Fst4s60>(slot, 1_000.0),
        "FST4-120" => frame77::<Fst4s120>(slot, 1_000.0),
        "FST4-300" => frame77::<Fst4s300>(slot, 1_000.0),
        "WSPR" => place::<Wspr>(
            mfsk_core::wspr::synthesize_type1("JA1ABC", "PM95", 37, FS, 1_500.0, 1.0)?,
            slot,
        ),
        "JT9" => place::<Jt9>(
            mfsk_core::jt9::synthesize_standard("CQ", "JA1ABC", "PM95", FS, 1_500.0, 1.0)?,
            slot,
        ),
        "JT65" => place::<Jt65>(
            mfsk_core::jt65::synthesize_standard("CQ", "JA1ABC", "PM95", FS, 1_500.0, 1.0)?,
            slot,
        ),
        "Q65-15A" => q65::<Q65a15>(slot)?,
        "Q65-30A" => q65::<Q65a30>(slot)?,
        "Q65-60A" => q65::<Q65a60>(slot)?,
        "Q65-60B" => q65::<Q65b60>(slot)?,
        "Q65-60C" => q65::<Q65c60>(slot)?,
        "Q65-60D" => q65::<Q65d60>(slot)?,
        "Q65-60E" => q65::<Q65e60>(slot)?,
        "Q65-120D" => q65::<Q65d120>(slot)?,
        "Q65-120E" => q65::<Q65e120>(slot)?,
        "Q65-300A" => q65::<Q65a300>(slot)?,
        _ => return None,
    })
}

/// `signal` (one period) under deterministic noise of about ±300, as i16.
fn with_noise(signal: &[f32], seed: u32) -> Vec<i16> {
    let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    signal
        .iter()
        .map(|&s| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let n = ((x >> 16) as i16 / 109) as f32;
            (s * AMP + n).clamp(-32_768.0, 32_767.0) as i16
        })
        .collect()
}

/// The lengths tried for a period of `slot` samples.
fn lengths(slot: usize) -> Vec<usize> {
    let mut v = vec![
        0,
        1,
        999,
        slot / 4,
        slot / 2,
        slot * 3 / 4,
        slot * 9 / 10,
        slot - 1,
        slot + 1,
        slot * 3 / 2,
    ];
    v.sort_unstable();
    v.dedup();
    v
}

#[test]
fn every_mode_survives_every_input_length() {
    let mut failures = Vec::new();
    let mut no_signal = Vec::new();
    let mut runs = 0usize;

    for &mode in Mode::ALL {
        let slot = mode.meta().slot_samples_12k as usize;
        let Some(signal) = period_with_signal(mode, slot) else {
            no_signal.push(mode.name());
            continue;
        };
        let full = with_noise(&signal, 7);
        let mut dec = AnyDecoder::with_defaults(mode);
        let mut period = 0i64;

        // The full period first: it has to decode, or the cut periods below
        // would never reach the paths that run after a decode.
        let n = dec.decode_i16(&full, Some(period)).rows.len();
        period += 1;
        if n == 0 {
            failures.push(format!("{}: a full period decoded nothing", mode.name()));
            continue;
        }

        for len in lengths(slot) {
            let cuts: [(&str, Vec<i16>); 2] = if len <= slot {
                [
                    ("start", full[..len].to_vec()),
                    ("end", full[slot - len..].to_vec()),
                ]
            } else {
                let mut long = full.clone();
                long.resize(len, 0);
                [("padded", long.clone()), ("padded", long)]
            };
            for (cut, audio) in cuts.iter() {
                let ok = catch_unwind(AssertUnwindSafe(|| {
                    dec.decode_i16(audio, Some(period));
                }))
                .is_ok();
                period += 1;
                runs += 1;
                if !ok {
                    failures.push(format!("{} len={len} {cut} (period {slot})", mode.name()));
                    // A decoder that panicked mid-decode is not reused.
                    dec = AnyDecoder::with_defaults(mode);
                }
            }
        }
    }

    assert!(runs > 0, "no protocol feature enabled");
    assert!(
        no_signal.is_empty(),
        "no synthesiser for {no_signal:?}: add one so the test reaches its decode paths"
    );
    assert!(
        failures.is_empty(),
        "{} of {runs} decodes failed:\n  {}",
        failures.len(),
        failures.join("\n  ")
    );
}

/// The geometry #567 was reported from, through the decoder: a receiver
/// that attached late, so the buffer is shorter than the frame **and** the
/// frame started before it (`dt < 0`). The test above never produces it on
/// the SIC modes: a frame missing more than a few percent of its start does
/// not decode, so nothing is subtracted. Here the frame ends just before the
/// period does, and the cut keeps all but its first 1000 or 3000 samples, so
/// it still decodes and the subtract runs on a buffer shorter than the frame
/// with a negative start. FT8 and FT4 are the modes whose default search
/// subtracts with the engine's LPF subtract (WSPR and JT65 have their own,
/// FST4, JT9 and Q65 none). The cut period must decode, or the subtract was
/// never reached and the case proves nothing.
#[test]
fn late_attach_shorter_than_the_frame_decodes_and_subtracts() {
    fn run<P>(mode: Mode, freq_hz: f32) -> Vec<String>
    where
        P: Protocol + FskWaveform,
    {
        let slot = mode.meta().slot_samples_12k as usize;
        let msg = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let wave = synthesize::<P>(&message_to_tones::<P>(&msg), FS, freq_hz, 1.0);
        let nframe = wave.len();
        let start = slot - nframe - 500;
        let mut period = vec![0.0f32; slot];
        period[start..start + nframe].copy_from_slice(&wave);
        let full = with_noise(&period, 11);

        let mut out = Vec::new();
        for missing in [1_000usize, 3_000] {
            // Keep the frame minus its first `missing` samples, and the 500
            // after it: the buffer is `nframe - missing + 500` long.
            let len = nframe - missing + 500;
            assert!(len < nframe);
            let audio = &full[slot - len..];
            let mut dec = AnyDecoder::with_defaults(mode);
            match catch_unwind(AssertUnwindSafe(|| dec.decode_i16(audio, None).rows.len())) {
                Err(_) => out.push(format!("{} missing={missing}: panicked", mode.name())),
                Ok(0) => out.push(format!(
                    "{} missing={missing}: decoded nothing, so nothing was subtracted",
                    mode.name()
                )),
                Ok(_) => {}
            }
        }
        out
    }

    let mut failures = Vec::new();
    failures.extend(run::<mfsk_core::Ft8>(Mode::Ft8, 1_500.0));
    failures.extend(run::<mfsk_core::Ft4>(Mode::Ft4, 1_500.0));
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
