//! MSK144 top-level decode driver: a 7168-sample sliding-window scan
//! over a whole Rx slot, calling [`crate::msk144::spd::short_ping_decode`]
//! (Stage A) then, on failure, a longer frame-averaging fallback
//! (Stage B) per analysis block.
//!
//! Ported from WSJT-X `mskrtd.f90` (the per-block real-time decoder)
//! and `decode_msk144.f90` (the sliding-window outer loop). Two
//! deliberate simplifications from the WSJT-X reference, both
//! documented at the point they matter below:
//!
//! - No adaptive RX-equalizer training (`beq`/`pcoeffs`/`corr(i)` in
//!   `mskrtd.f90`, trained by `msk144signalquality.f90`):
//!   [`crate::core::dsp::analytic_signal`] applies WSJT-X's *fixed*
//!   1500 Hz-centered bandpass filter (`analytic.f90`'s `h(i)`,
//!   unconditional in the reference), but not the *adaptive*
//!   session-trained phase/amplitude correction layered on top of it.
//!   Signal-quality estimation/training is not ported.
//! - MSK40 (the legacy shorthand mode, `msk40spd.f90`) and SWL/hash
//!   dedup bookkeeping are out of scope (matches the original scoping
//!   in this crate's issue #25).
//!
//! Noise-floor (`pnoise`) and dupe-check (`msglast`/`nsnrlast`) state
//! is scoped to one [`decode_slot`] call rather than persisted across
//! calls — WSJT-X's real-time decoder smooths `pnoise` across an
//! entire session, but every other protocol in this crate treats "one
//! slot" as the unit of a decode call, and this is a deliberate
//! simplification in that direction rather than an attempt at full
//! real-time fidelity.

use alloc::string::String;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::core::DecodeContext;
use crate::core::dsp::analytic_signal;
use crate::core::dsp::msk::NSPM;
use crate::msk144::sync::{msk144_sync, rotate_to_shift};
use crate::msk144::{frame_decode::decode_frame, spd::short_ping_decode};

const BLOCK_SIZE: usize = 7168;
const STEP_SIZE: usize = BLOCK_SIZE / 2; // 3584

/// The 8-frame averaging masks `mskrtd.f90:49-53` tries in Stage B
/// (`iavpatterns`) when Stage A (the short-ping decoder) fails to
/// find anything.
const IAVPATTERNS: [[bool; 8]; 4] = [
    [true, true, true, true, false, false, false, false],
    [false, false, true, true, true, true, false, false],
    [true, true, true, true, true, false, false, false],
    [true, true, true, true, true, true, true, false],
];

/// Nominal time (in frame durations, `mskrtd.f90:54`'s `xmc`) at the
/// center of each `IAVPATTERNS` mask, used only for the reported
/// decode timestamp — Stage B has no per-sample burst localization
/// the way Stage A's squared-signal scan does.
const XMC: [f32; 4] = [2.0, 4.5, 2.5, 3.5];

/// Search depth, controlling how hard Stage B tries after Stage A
/// fails. Matches `mskrtd.f90:142-144`'s `ndepth`.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum Depth {
    /// Stage A (short-ping decoder) only.
    Fast,
    /// Stage A, then the first 2 of 4 `IAVPATTERNS`.
    Normal,
    /// Stage A, then all 4 `IAVPATTERNS`.
    #[default]
    Deep,
}

impl Depth {
    fn npat(self) -> usize {
        match self {
            Depth::Fast => 0,
            Depth::Normal => 2,
            Depth::Deep => 4,
        }
    }
}

/// One decoded MSK144 message from a slot scan.
#[derive(Clone, Debug)]
pub struct SlotDecode {
    pub message: String,
    /// SNR estimate (dB), clamped to `[-8, 24]` — WSJT-X's
    /// `10*log10(pmax/pnoise-1)` formula (`mskrtd.f90:185`).
    pub snr_db: i32,
    /// Refined frequency estimate (Hz).
    pub freq_hz: f32,
    /// Decode time (seconds from the start of the slot).
    pub tsec: f32,
}

/// Per-call state threaded across the sliding-window scan: the
/// exponentially-smoothed noise floor and the last accepted message
/// (for dupe suppression). Scoped to one [`decode_slot`] call — see
/// the module doc for why this isn't persisted across calls.
struct ScanState {
    /// `-1.0` = uninitialized (`mskrtd.f90:62`).
    pnoise: f32,
    msglast: String,
    nsnrlast: i32,
}

impl Default for ScanState {
    fn default() -> Self {
        Self {
            pnoise: -1.0,
            msglast: String::new(),
            nsnrlast: -99,
        }
    }
}

/// Decode one `BLOCK_SIZE`-sample (0.597 s) analysis block. Ported
/// from `mskrtd.f90:92-233` (minus the RX-equalizer training and
/// MSK40/SWL paths — see the module doc).
fn decode_block(
    audio_i16: &[i16],
    fc: f32,
    ntol: f32,
    depth: Depth,
    ctx: &DecodeContext,
    state: &mut ScanState,
) -> Option<SlotDecode> {
    assert_eq!(audio_i16.len(), BLOCK_SIZE);

    let d: Vec<f32> = audio_i16.iter().map(|&x| x as f32).collect();
    let rms = (d.iter().map(|&x| x * x).sum::<f32>() / BLOCK_SIZE as f32).sqrt();
    if rms < 1.0 {
        return None; // effectively silent block
    }
    let fac = 1.0 / rms;
    let d_norm: Vec<f32> = d.iter().map(|&x| x * fac).collect();
    let cdat = analytic_signal(&d_norm);

    // Per-frame power, rescaled back to the original signal scale
    // (`mskrtd.f90:110-117`).
    let mut pow = [0.0f32; 8];
    for (i, p) in pow.iter_mut().enumerate() {
        let ib = i * NSPM;
        let ie = ib + NSPM;
        let energy: f32 = cdat[ib..ie].iter().map(|c| c.norm_sqr()).sum();
        *p = energy * rms * rms;
    }
    let pmax = pow.iter().cloned().fold(f32::MIN, f32::max);
    let pavg = pow.iter().sum::<f32>() / 8.0;

    let cbig = &cdat[0..8 * NSPM];

    // Stage A: short-ping decoder.
    if let Some(spd) = short_ping_decode(cbig, fc, ntol, ctx) {
        return finish_decode(spd.message, pmax, state, spd.tdec_sec, spd.fest);
    }

    // Stage B: longer frame-averaging fallback.
    let tframe = NSPM as f32 / 12_000.0;
    for iavg in 0..depth.npat() {
        let navmask = IAVPATTERNS[iavg];
        let navg = navmask.iter().filter(|&&b| b).count().max(1);
        let deltaf = 10.0 / navg as f32;

        let sync_result = msk144_sync(cbig, 8, ntol, deltaf, &navmask, 2, fc);
        if !sync_result.success {
            continue;
        }
        for peak in &sync_result.peaks {
            for dither in 0..3 {
                let ic0 = match dither {
                    0 => peak.shift,
                    1 => peak.shift.saturating_sub(1),
                    _ => (peak.shift + 1).min(NSPM - 1),
                };
                let aligned = rotate_to_shift(&sync_result.frame, ic0);
                let softbits = crate::core::dsp::msk::matched_filter_softbits(
                    aligned.as_slice().try_into().expect("NSPM samples"),
                );
                if let Some(result) = decode_frame(&softbits, ctx) {
                    let tdec_sec = XMC[iavg] * tframe;
                    return finish_decode(result.message, pmax, state, tdec_sec, sync_result.fest);
                }
            }
        }
    }

    // No decode: update the noise floor (`mskrtd.f90:172-179`) --
    // slow to rise, quick to fall.
    if state.pnoise < 0.0 {
        state.pnoise = pavg;
    } else if pavg > state.pnoise {
        state.pnoise = 0.9 * state.pnoise + 0.1 * pavg;
    } else {
        state.pnoise = pavg;
    }
    None
}

/// SNR estimate + dupe-check gate (`mskrtd.f90:182-216`), shared by
/// both Stage A and Stage B decode paths. Note `pnoise` is
/// deliberately **not** updated here — WSJT-X only updates the noise
/// floor from no-decode blocks, to avoid contaminating it with the
/// signal's own power.
fn finish_decode(
    message: String,
    pmax: f32,
    state: &mut ScanState,
    tdec_sec: f32,
    fest: f32,
) -> Option<SlotDecode> {
    let snr0 = if state.pnoise > 0.0 {
        10.0 * (pmax / state.pnoise - 1.0).log10()
    } else {
        0.0
    };
    let nsnr = (snr0.round() as i32).clamp(-8, 24);

    let accept = message != state.msglast || nsnr > state.nsnrlast;
    if !accept {
        return None;
    }
    state.msglast = message.clone();
    state.nsnrlast = nsnr;

    Some(SlotDecode {
        message,
        snr_db: nsnr,
        freq_hz: fest,
        tsec: tdec_sec,
    })
}

/// Scan a whole Rx slot for MSK144 decodes: a sliding `BLOCK_SIZE`
/// (7168-sample, 0.597 s) window at `STEP_SIZE` (3584-sample, 50%
/// overlap) steps, matching `decode_msk144.f90`'s outer loop.
///
/// `audio` is 12 kHz i16 PCM, typically one 15 s (or 30 s) T/R period.
/// `fc` is the nominal center frequency (Hz) to search around, `ntol`
/// the search half-width (Hz), and `depth` how hard to try once the
/// fast short-ping path fails on a given block (see [`Depth`]).
pub fn decode_slot(audio: &[i16], fc: f32, ntol: f32, depth: Depth) -> Vec<SlotDecode> {
    let ctx = DecodeContext::default();
    let mut state = ScanState::default();
    let mut out = Vec::new();

    if audio.len() < BLOCK_SIZE {
        return out;
    }

    let mut position = 0usize;
    while position + BLOCK_SIZE <= audio.len() {
        let block = &audio[position..position + BLOCK_SIZE];
        let tsec = position as f32 / 12_000.0;
        if let Some(mut r) = decode_block(block, fc, ntol, depth, &ctx, &mut state) {
            r.tsec += tsec;
            out.push(r);
        }
        position += STEP_SIZE;
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::FecCodec;
    use crate::core::dsp::msk::build_bitseq;
    use crate::fec::Ldpc128_90;

    /// The `itone` audio-tone sequence msk144sim/genmsk_128_90 actually
    /// transmits is *not* the raw channel bit sequence -- MSK's
    /// continuous-phase property means the instantaneous tone at each
    /// symbol depends on the product of adjacent bipolar rail bits.
    /// Ported from `genmsk_128_90.f90:109-114,118` (same derivation
    /// used in `msk144::spd`'s independent-oracle tests).
    fn build_i4tone(bitseq_natural: &[u8; 144]) -> [u8; 144] {
        let mut bp = [0i8; 144];
        for i in 0..144 {
            bp[i] = 2 * bitseq_natural[i] as i8 - 1;
        }
        let mut i4tone = [0i8; 144];
        for i in 1..=72usize {
            let b_2i_minus_1 = bp[2 * i - 2];
            let b_2i = bp[2 * i - 1];
            let b_wrap = bp[(2 * i) % 144];
            i4tone[2 * i - 2] = (b_2i * b_2i_minus_1 + 1) / 2;
            i4tone[2 * i - 1] = -((b_2i * b_wrap - 1) / 2);
        }
        let mut out = [0u8; 144];
        for i in 0..144 {
            out[i] = (-i4tone[i] + 1) as u8;
        }
        out
    }

    /// Independent WSJT-X-style reference synthesizer
    /// (`msk144sim.f90:52-76`): simple continuous-phase binary FSK,
    /// not this crate's own OQPSK/complex-baseband synth path. See
    /// `msk144::spd`'s test module for why this independence matters.
    fn msk144sim_reference_audio(itone: &[u8], freq_hz: f32) -> Vec<f32> {
        let twopi = 2.0 * core::f32::consts::PI;
        let baud = 2000.0f32;
        let dphi0 = twopi * (freq_hz - 0.25 * baud) / 12_000.0;
        let dphi1 = twopi * (freq_hz + 0.25 * baud) / 12_000.0;
        let mut phi = 0.0f32;
        let mut out = Vec::with_capacity(itone.len() * 6);
        for &tone in itone {
            let dphi = if tone == 0 { dphi0 } else { dphi1 };
            for _ in 0..6 {
                out.push(phi.cos());
                phi = (phi + dphi) % twopi;
            }
        }
        out
    }

    /// Deterministic complex-Gaussian-ish real noise (xorshift32 +
    /// Box-Muller, see `msk144::spd`'s tests for why a tonal fake-noise
    /// generator wouldn't do here), quantized to i16 PCM.
    fn pseudo_gaussian_noise_i16(n: usize, amp: f32, seed: u32) -> Vec<i16> {
        let mut state = seed | 1;
        let mut next_u32 = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        let mut uniform = move || (next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
        (0..n)
            .map(|_| {
                let u1 = uniform();
                let u2 = uniform();
                let r = (-2.0 * u1.ln()).sqrt();
                let theta = 2.0 * core::f32::consts::PI * u2;
                (amp * r * theta.cos()).clamp(i16::MIN as f32, i16::MAX as f32) as i16
            })
            .collect()
    }

    fn build_codeword_bitseq(call1: &str, call2: &str, report: &str) -> [u8; 144] {
        let msg77 = crate::msg::wsjt77::pack77(call1, call2, report).expect("valid message");
        let mut info = [0u8; 90];
        info[..77].copy_from_slice(&msg77);
        let mut bytes = [0u8; 12];
        for (i, &b) in info[..77].iter().enumerate() {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            bytes[byte_idx] |= (b & 1) << bit_pos;
        }
        let crc = crate::fec::ldpc_128_90::crc13(&bytes);
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        build_bitseq(&codeword)
    }

    /// A real message, synthesized via the independent FSK oracle as
    /// a multi-second repeating ping (matching how a real MSK144
    /// transmission continuously repeats the same frame), embedded at
    /// an unknown position inside a realistic 15 s noisy slot buffer:
    /// [`decode_slot`] must find and decode it via i16 PCM audio only,
    /// exercising the RMS normalization, analytic-signal construction,
    /// block segmentation, and Stage A/B routing this module adds on
    /// top of the already-proven sync/frame_decode primitives.
    #[test]
    fn decode_slot_recovers_a_real_message_from_a_15s_buffer() {
        let bitseq = build_codeword_bitseq("K1ABC", "W9XYZ", "EN37");
        let itone = build_i4tone(&bitseq);

        const NREPS: usize = 20; // ~1.44 s ping, well within a 15 s slot
        let itone_repeated: Vec<u8> = itone.iter().copied().cycle().take(144 * NREPS).collect();
        let fc_true = 1500.0f32;
        let audio_f32 = msk144sim_reference_audio(&itone_repeated, fc_true);

        let slot_len = 15 * 12_000;
        let mut audio_i16 = pseudo_gaussian_noise_i16(slot_len, 300.0, 42);
        let true_start = 3 * 12_000; // 3 s into the slot, unknown to the driver
        const AMPLITUDE: f32 = 3000.0;
        for (k, &s) in audio_f32.iter().enumerate() {
            let v = audio_i16[true_start + k] as f32 + AMPLITUDE * s;
            audio_i16[true_start + k] = v.clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        }

        let decodes = decode_slot(&audio_i16, fc_true, 8.0, Depth::Deep);
        assert!(!decodes.is_empty(), "expected at least one decode");
        assert!(
            decodes.iter().any(|d| d.message.contains("K1ABC")
                && d.message.contains("W9XYZ")
                && d.message.contains("EN37")),
            "no decode matched the embedded message: {decodes:?}"
        );
        for d in &decodes {
            assert!((-8..=24).contains(&d.snr_db), "snr_db out of range: {d:?}");
            assert!(
                d.tsec >= 0.0 && d.tsec <= 15.0,
                "tsec out of slot range: {d:?}"
            );
        }
    }

    /// CRC-13 plus the `n3`/`i3` plausibility filter makes a false
    /// decode from pure noise astronomically unlikely; a full 15 s
    /// noise-only slot should decode nothing.
    #[test]
    fn decode_slot_returns_nothing_on_pure_noise() {
        let audio_i16 = pseudo_gaussian_noise_i16(15 * 12_000, 300.0, 7);
        let decodes = decode_slot(&audio_i16, 1500.0, 8.0, Depth::Fast);
        assert!(
            decodes.is_empty(),
            "unexpected decodes from pure noise: {decodes:?}"
        );
    }
}
