//! JT9 decode pipeline: WSJT-X-faithful softsym (downsam9 → peakdt9 →
//! AFC → twkfreq → symspec2 → fano232).
//!
//! Replaces the earlier box-car path in `baseband.rs` + `demod_bb.rs`.
//! The big audio FFT is computed once and reused across candidates.

use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};
use crate::fec::ConvFano232;
use crate::msg::{Jt72Codec, Jt72Message};

use super::Jt9Decode;
use super::softsym::{AudioFft, FSAMPLE_DOWN, afc9, llrs_from_c5, peakdt9, twkfreq_poly};

/// Sync-score gate: peakdt9 returns `(sync_avg/data_avg)−1`. For pure
/// noise this is ≈ 0; clean JT9 frames sit in the 0.3–10 range. WSJT-X
/// uses ccfbest > 30 (raw integrated power) on its own scale; the 1.5
/// threshold here is calibrated from the 130418_1742 sample (golden
/// signals score 1.5..15, phantoms < 0.8).
const SYNC_GATE: f32 = 1.5;

/// Try to decode a JT9 signal centred at `freq_hz` using a pre-built
/// audio FFT. Returns `None` when the sync gate is missed, Fano fails
/// to converge, or the message is not `Jt72Message::Standard`.
pub fn decode_at_baseband_with_fft(big_fft: &AudioFft, freq_hz: f32) -> Option<Jt9Decode> {
    if freq_hz <= 0.0 {
        return None;
    }

    let c2 = big_fft.downsam9(freq_hz);
    let (lagpk, sync_score, mut c3) = peakdt9(&c2);
    if !sync_score.is_finite() || sync_score < SYNC_GATE {
        return None;
    }

    // WSJT-X-faithful AFC: 3-parameter chi-square optimisation over
    // (frequency, drift, integer-sample time shift). `afc9` mutates
    // `c3` to apply the discovered integer shift; the caller mixes
    // out the residual frequency + drift with `twkfreq_poly`.
    let afc = afc9(&mut c3);
    twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);

    // WSJT-X two-stage sync gate (`lib/jt9_decode.f90:139`): both
    // sync = (syncpk + 1)/4 and schk (the chkss2 normalised tone-0
    // sync power) must clear their thresholds before we spend Fano
    // cycles. Drops phantom convergences in busy bands.
    let sync = (afc.syncpk + 1.0) / 4.0;
    if !sync.is_finite() || sync < 1.0 {
        return None;
    }
    let (schk, llrs) = llrs_from_c5(&c3);
    if !schk.is_finite() || schk < 1.5 {
        return None;
    }
    let res = ConvFano232.decode_soft(&llrs, &FecOpts::default())?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    let msg = Jt72Codec::default().unpack(&payload, &DecodeContext::default())?;

    // JT9 has no CRC — Fano can converge on plausible-looking junk.
    // Accepting only Standard form drops most hashed-callsign garbage.
    match &msg {
        Jt72Message::Standard { .. } => {}
        _ => return None,
    }

    // Translate (lagpk, df) back to audio-sample / Hz coordinates so
    // downstream consumers see a consistent timing/freq report.
    let start_sample = lag_to_audio_sample(lagpk);
    let freq_corrected = freq_hz - afc.a0;
    Some(Jt9Decode {
        message: msg,
        freq_hz: freq_corrected,
        start_sample,
    })
}

/// One-shot variant: builds the big FFT inline. Useful for tests and
/// callers that don't reuse the same audio across many candidates.
#[cfg(test)]
#[allow(dead_code)]
pub fn decode_at_baseband(
    audio: &[f32],
    _sample_rate: u32,
    _start_sample: usize,
    freq_hz: f32,
) -> Option<Jt9Decode> {
    let big = AudioFft::build(audio);
    decode_at_baseband_with_fft(&big, freq_hz)
}

/// Convert peakdt9 lag (in 27.78 Hz baseband samples) back to a
/// 12 kHz audio-sample index. Reverses the offsetting that peakdt9
/// applies when slicing c3 out of c2.
fn lag_to_audio_sample(lagpk: i64) -> usize {
    // peakdt9 places sample 0 of c3 at c2 index `lagpk - i0 - NSPSD + 1`
    // where i0 = 5 * NSPSD. Each 27.78 Hz sample equals NDOWN = 432
    // audio samples at 12 kHz.
    let i0 = 5 * 16i64;
    let nspsd = 16i64;
    let ndown = 432i64;
    let c2_offset = lagpk - i0 - nspsd + 1;
    // Audio-sample index of symbol-0 within the original audio.
    let sample = c2_offset.max(0) * ndown;
    let _ = FSAMPLE_DOWN; // silence unused
    sample as usize
}

#[cfg(test)]
#[allow(dead_code)]
mod gate_diag {
    use super::*;
    use crate::core::DecodeContext;
    use crate::msg::Jt72Codec;
    use std::path::Path;

    /// Walk specific frequencies (the missing JT9 goldens) through the
    /// pipeline manually so we can see *where* they drop: peakdt9
    /// sync_score → afc9 syncpk + sync = (syncpk+1)/4 → schk → Fano
    /// hard_errors at three retry depths.
    #[test]
    #[ignore]
    fn probe_missing_goldens() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !path.exists() {
            eprintln!("skipping — sample not found");
            return;
        }
        let bytes = std::fs::read(path).unwrap();
        let dl = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let audio: Vec<f32> = bytes[44..44 + dl]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0)
            .collect();

        let big_fft = AudioFft::build(&audio);

        for &(label, nominal_freq) in &[
            ("CQ GM7GAX IO75", 1119.0f32),
            ("CQ M0WAY IO82", 1290.0f32),
            ("TF3G N7MQ CN84 (control: passes)", 1186.0f32),
            ("K1JT KF4RWA 73 (control: passes)", 1224.0f32),
        ] {
            eprintln!("\n=== {} @ {:.1} Hz ===", label, nominal_freq);
            // Sweep ±2 Hz in 0.5 Hz steps so we see the freq with the
            // best sync, not just the nominal.
            for df_i in -4..=4i32 {
                let freq = nominal_freq + df_i as f32 * 0.5;
                let c2 = big_fft.downsam9(freq);
                let (_lagpk, peakdt_score, mut c3) = super::super::softsym::peakdt9(&c2);
                if !peakdt_score.is_finite() || peakdt_score < 0.3 {
                    continue;
                }
                let afc = super::super::softsym::afc9(&mut c3);
                super::super::softsym::twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);
                let sync = (afc.syncpk + 1.0) / 4.0;
                let (schk, llrs) = super::super::softsym::llrs_from_c5(&c3);

                let mut hits = Vec::new();
                for &lim in &[10_000u64, 30_000, 100_000] {
                    let opts = FecOpts {
                        max_cycles_per_bit: Some(lim),
                        ..FecOpts::default()
                    };
                    let dec = ConvFano232.decode_soft(&llrs, &opts);
                    let s = match dec {
                        Some(r) => {
                            let mut p = [0u8; 72];
                            p.copy_from_slice(&r.info);
                            let m = Jt72Codec::default().unpack(&p, &DecodeContext::default());
                            format!(
                                "limit={} hard={} msg={:?}",
                                lim,
                                r.hard_errors,
                                m.map(|x| x.to_string())
                            )
                        }
                        None => format!("limit={} no-converge", lim),
                    };
                    hits.push(s);
                }
                eprintln!(
                    "  freq={:.1} peakdt={:.3} sync={:.3} schk={:.3} | {}",
                    freq,
                    peakdt_score,
                    sync,
                    schk,
                    hits.join(" / ")
                );
            }
        }
    }
}
