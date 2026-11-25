//! Per-candidate fine sync (WSJT-X `ft8b.f90` Stages A / B / C), computed
//! on the 12 kHz audio instead of a downsampled baseband.
//!
//! Ported from `lib/ft8/ft8b.f90:122-174` and `lib/ft8/sync8d.f90`.
//! Upstream mixes each candidate to a 200 Hz baseband `cd0` through a
//! 192 000-point FFT (`ft8_downsample`), then:
//!
//! - **Stage A** — `sync8d` over `i0 +- 10` cd0 samples (+-50 ms in 5 ms)
//!   at the coarse frequency;
//! - **Stage B** — `sync8d` at Stage A's best start with the Costas
//!   waveform tweaked by `ctwk`, `delf` over +-2.5 Hz in 0.5 Hz;
//! - **Stage C** — after re-mixing at `f1 + delfbest`, `sync8d` over
//!   `ibest +- 4`.
//!
//! The embedded planner carries no 192 000-point FFT, and the embedded
//! decode path skipped all three stages (`fine_refine_pass1` is a no-op
//! there since 0.6.3). Its candidates therefore sit where coarse sync
//! put them: on the spectrogram's 3.125 Hz frequency grid and its
//! 40 ms time grid, refined only by coarse sync's own interpolation.
//!
//! **What is different here, and why it is the same quantity.** `sync8d`
//! correlates 32 cd0 samples against one Costas tone. The same number
//! (to the baseband filter's passband ripple) is a single-frequency
//! correlation of 1 920 audio samples at `f0 + tone x 6.25 Hz`. Grouping
//! those samples into 60-sample bins — one bin per cd0 sample, since
//! `NSPS / 32 = 60` — gives every `sync8d` evaluation from one mixing
//! pass per Costas symbol:
//!
//! - Stages A and C move which 32 consecutive bins are summed;
//! - Stage B multiplies the bins by `ft8b.f90`'s own 32-point `ctwk`.
//!
//! One mixing pass covers the 32 bins of a symbol plus the 14 either
//! side that A and C can reach, so the cost is `21 x 60 x 60` = 75 600
//! two-multiply sample steps per candidate, against `41 x 21 x 1 920`
//! for evaluating each of the 41 `sync8d` calls directly. The tweak is
//! held constant across a bin, which is off by at most
//! `2 pi x 2.5 Hz x 30 / 12 000` = 0.04 rad at a bin's edges.
//!
//! **Why it exists — measured**, on the CoreS3 FT8 per-slot pipeline
//! reproduced call for call on the host
//! (`tests/ft8_embedded_pipeline_mirror.rs`, 2026-09-17), with the
//! candidate's coarse position retried once when the refined one fails
//! (the caller's job, see `embedded-shared::dual_core`):
//!
//! ```text
//!                        plateau (201 phases)   acquisition + decode (30 starts)
//!   qso3_busy  shipped        6.35                   5.79  (>= 8: 0 %)
//!              fine sync      8.53                   8.29  (>= 8: 96 %)
//!   qso1       shipped        1.51 *                 3.88
//!              fine sync      1.48                   3.88
//!   qso2       shipped        1.81 *                 4.96 *
//!              fine sync      1.81                   4.96
//!   (* = Stage A only + coarse retry, the previous best on that axis)
//! ```
//!
//! The station it adds on `qso3_busy` is `K1JT EA3AGB`, decoded at 89 %
//! of phases against none: its decodable region sits 1.0-1.5 Hz below
//! the coarse bin, where it decodes across the whole +-30 ms, while at
//! the bin itself it decodes only on a 5 ms comb. Every other station's
//! rate is unchanged and no decode outside the known-real set appeared.
//!
//! **Deliberate divergence from WSJT-X**: none in the search itself —
//! the step sizes, ranges and tie rules (`sync.gt.smax`, `maxloc`: the
//! first maximum wins) are upstream's. The difference is the domain
//! (12 kHz audio rather than `cd0`), which also means no anti-alias
//! filter: a strong neighbour within a few Hz leaks into the
//! correlation the way it does in the embedded Goertzel fill, rather
//! than being shaped by `ft8_downsample`'s window.

use alloc::boxed::Box;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::params::{COSTAS, COSTAS_POS, NSPS};
#[cfg(test)]
use super::types::SAMPLE_RATE_HZ;
use super::types::{AudioSample, Ft8Rate, TONE_SPACING_HZ, TX_START_OFFSET_S};
use crate::engine::sync::SyncCandidate;

/// 12 kHz samples per cd0 sample (`ft8_downsample`'s `NDOWN = 60`); 40 at 8 kHz (`Ft8Rate::bin`). The scratch is
/// sized for the larger.
const BIN: usize = NSPS / 32;
/// cd0 sample rate, Hz (200 at either rate).
#[cfg(test)]
const CD0_RATE_HZ: f32 = SAMPLE_RATE_HZ / BIN as f32;
/// Stage A half-width in cd0 samples (`ft8b.f90`: `i0-10, i0+10`).
const STAGE_A_REACH: i32 = 10;
/// Stage C half-width in cd0 samples (`ft8b.f90`: `-4, 4`).
const STAGE_C_REACH: i32 = 4;
/// Stage B: `ifr = -5..=5`, `delf = ifr * 0.5`.
const STAGE_B_STEPS: i32 = 5;
const STAGE_B_STEP_HZ: f32 = 0.5;
/// How far Stage A then Stage C can move the start, in cd0 samples.
const REACH: i32 = STAGE_A_REACH + STAGE_C_REACH;
/// Bins mixed per Costas symbol: the symbol's own 32 plus `REACH` either
/// side.
const NB: usize = 32 + 2 * REACH as usize;
/// 3 Costas blocks of 7.
const NSYNC: usize = 21;

/// Reusable per-call scratch: ~13.7 KB, so it is heap-allocated once per
/// call rather than put on an embedded task stack.
struct Scratch {
    /// `bins[s][j]`: Costas symbol `s`, bin `j` of its span, phase
    /// referenced to the span's first bin.
    bins: [[Complex<f32>; NB]; NSYNC],
    /// One bin's local oscillator per Costas tone, split for the inner
    /// loop: `lo_re[t][n] - i lo_im[t][n] = e^{-i w_t n}`.
    lo_re: [[f32; BIN]; 8],
    lo_im: [[f32; BIN]; 8],
    /// `ctwk` conjugated, per Stage B step: `e^{-i 2 pi delf m / 200}`.
    tweak: [[Complex<f32>; 32]; 2 * STAGE_B_STEPS as usize + 1],
}

/// Refine each candidate's frequency and DT with `ft8b.f90`'s Stages A,
/// B and C, on 12 kHz audio. Returns one candidate per input, in input
/// order, with `score` unchanged.
///
/// A Costas symbol whose window leaves `audio` contributes nothing, as
/// in `sync8d` — so a prefix of the slot can be passed while the rest
/// is still arriving, at the cost of the late symbols.
pub fn fine_sync_12k<S: AudioSample>(audio: &[S], cands: &[SyncCandidate]) -> Vec<SyncCandidate> {
    fine_sync_at(audio, Ft8Rate::Hz12000, cands)
}

/// [`fine_sync_12k`] on audio at `rate` ([`Ft8Rate`]): the same search on the same 200 Hz grid, its bins `rate / 200`
/// samples wide (60 at 12 kHz, 40 at 8 kHz). Only the sums over a bin's samples and the mixer's phase step are in the
/// rate; the candidates it returns are on the same frequency and time grid.
pub fn fine_sync_at<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    cands: &[SyncCandidate],
) -> Vec<SyncCandidate> {
    if cands.is_empty() {
        return Vec::new();
    }
    let zero = Complex::new(0.0f32, 0.0);
    let mut sc: Box<Scratch> = Box::new(Scratch {
        bins: [[zero; NB]; NSYNC],
        lo_re: [[0.0; BIN]; 8],
        lo_im: [[0.0; BIN]; 8],
        tweak: [[zero; 32]; 2 * STAGE_B_STEPS as usize + 1],
    });
    let cd0_rate = rate.hz() as f32 / rate.bin() as f32;
    for (k, row) in sc.tweak.iter_mut().enumerate() {
        let delf = (k as i32 - STAGE_B_STEPS) as f32 * STAGE_B_STEP_HZ;
        let dphi = core::f32::consts::TAU * delf / cd0_rate;
        for (m, c) in row.iter_mut().enumerate() {
            let ph = dphi * m as f32;
            *c = Complex::new(ph.cos(), -ph.sin());
        }
    }
    cands
        .iter()
        .map(|c| refine_one(audio, rate, c, &mut sc))
        .collect()
}

fn refine_one<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    c: &SyncCandidate,
    sc: &mut Scratch,
) -> SyncCandidate {
    let bin = rate.bin();
    let cd0_rate = rate.hz() as f32 / bin as f32;
    let two_pi_over_fs = core::f32::consts::TAU / rate.hz() as f32;
    // cd0 index of symbol 0: `ft8b.f90`'s `i0 = nint((xdt+0.5)*fs2)`.
    let i0 = ((c.dt_sec + TX_START_OFFSET_S) * cd0_rate).round() as i32;

    // Only the 7 Costas tones are ever mixed.
    let mut rot = [Complex::new(0.0f32, 0.0); 8];
    for &tone in COSTAS.iter() {
        let w = two_pi_over_fs * (c.freq_hz + tone as f32 * TONE_SPACING_HZ);
        for n in 0..bin {
            let ph = w * n as f32;
            sc.lo_re[tone][n] = ph.cos();
            sc.lo_im[tone][n] = ph.sin();
        }
        let ph = w * bin as f32;
        rot[tone] = Complex::new(ph.cos(), -ph.sin());
    }

    let len = audio.len() as i64;
    let mut s = 0;
    for &block in COSTAS_POS.iter() {
        for (k, &tone) in COSTAS.iter().enumerate() {
            let first = i0 - REACH + ((block + k) * 32) as i32;
            // Phase of the bin's first sample relative to the span's.
            let mut phasor = Complex::new(1.0f32, 0.0);
            let (lo_re, lo_im) = (&sc.lo_re[tone], &sc.lo_im[tone]);
            for j in 0..NB {
                let st = (first as i64 + j as i64) * bin as i64;
                sc.bins[s][j] = if st >= 0 && st + bin as i64 <= len {
                    let win = &audio[st as usize..st as usize + bin];
                    let (mut re, mut im) = (0.0f32, 0.0f32);
                    for n in 0..bin {
                        let x = win[n].to_f32();
                        re += x * lo_re[n];
                        im -= x * lo_im[n];
                    }
                    Complex::new(re, im) * phasor
                } else {
                    Complex::new(0.0, 0.0)
                };
                phasor *= rot[tone];
            }
            s += 1;
        }
    }

    // `sync8d` at start `i0 + d`, optionally with tweak row `tw`.
    let bins = &sc.bins;
    let sync = |d: i32, tw: Option<&[Complex<f32>; 32]>| -> f32 {
        let j0 = (d + REACH) as usize;
        let mut p = 0.0f32;
        for sym in bins.iter() {
            let span = &sym[j0..j0 + 32];
            let z = match tw {
                None => span.iter().fold(Complex::new(0.0, 0.0), |a, b| a + b),
                Some(tw) => span
                    .iter()
                    .zip(tw.iter())
                    .fold(Complex::new(0.0, 0.0), |a, (b, t)| a + b * t),
            };
            p += z.norm_sqr();
        }
        p
    };

    // Stage A. Strict `>` from the low end: `if(sync.gt.smax)`.
    let mut da = -STAGE_A_REACH;
    let mut best = f32::MIN;
    for d in -STAGE_A_REACH..=STAGE_A_REACH {
        let p = sync(d, None);
        if p > best {
            best = p;
            da = d;
        }
    }
    // Stage B.
    let mut kb = STAGE_B_STEPS as usize;
    let mut best = f32::MIN;
    for (k, tw) in sc.tweak.iter().enumerate() {
        let p = sync(da, Some(tw));
        if p > best {
            best = p;
            kb = k;
        }
    }
    let delf = (kb as i32 - STAGE_B_STEPS) as f32 * STAGE_B_STEP_HZ;
    // Stage C, at the tweaked frequency. `maxloc` keeps the first
    // maximum, which strict `>` also does.
    let tw = &sc.tweak[kb];
    let mut dc = da;
    let mut best = f32::MIN;
    for d in da - STAGE_C_REACH..=da + STAGE_C_REACH {
        let p = sync(d, Some(tw));
        if p > best {
            best = p;
            dc = d;
        }
    }

    SyncCandidate {
        freq_hz: c.freq_hz + delf,
        dt_sec: (i0 + dc) as f32 / cd0_rate - TX_START_OFFSET_S,
        score: c.score,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// A signal placed off the coarse grid in both axes comes back to
    /// within one search step of where it was placed.
    #[test]
    fn recovers_an_off_grid_signal_through_noise() {
        const F_TRUE: f32 = 1501.2;
        const DT_TRUE: f32 = 0.137;
        let msg: Vec<u8> = (0..77).map(|i| ((i * 7 + 3) % 5 == 0) as u8).collect();
        let itone = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(
            msg.as_slice().try_into().unwrap(),
        );
        let sig =
            crate::engine::tx::synthesize_i16::<crate::ft8::Ft8>(&itone, 12_000, F_TRUE, 2_000);
        let mut audio = vec![0i16; 180_000];
        let start = ((TX_START_OFFSET_S + DT_TRUE) * SAMPLE_RATE_HZ).round() as usize;
        // Deterministic noise, so the fixture measures something.
        let mut state: u32 = 0x1234_5678;
        for (i, a) in audio.iter_mut().enumerate() {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            let noise = (state % 8_001) as i32 - 4_000;
            let s = if i >= start && i - start < sig.len() {
                sig[i - start] as i32
            } else {
                0
            };
            *a = (s + noise).clamp(-32_768, 32_767) as i16;
        }
        let coarse = SyncCandidate {
            freq_hz: 1_500.0,
            dt_sec: 0.10,
            score: 1.0,
        };
        let r = fine_sync_12k(&audio, &[coarse]).remove(0);
        assert!(
            (r.freq_hz - F_TRUE).abs() <= STAGE_B_STEP_HZ,
            "freq {} vs {F_TRUE}",
            r.freq_hz
        );
        assert!(
            (r.dt_sec - DT_TRUE).abs() <= 1.0 / CD0_RATE_HZ,
            "dt {} vs {DT_TRUE}",
            r.dt_sec
        );
        assert_eq!(r.score, 1.0);
    }

    #[test]
    fn empty_input_allocates_nothing_and_returns_nothing() {
        let audio = [0i16; 16];
        assert!(fine_sync_12k(&audio, &[]).is_empty());
    }
}
