//! Protocol-agnostic synchronisation primitives.
//!
//! Coarse sync searches the 2D (freq, lag) plane for candidate frames by
//! correlating per-symbol power spectra against the protocol's sync-block
//! tone patterns. Fine sync refines the timing on the downsampled complex
//! baseband signal.
//!
//! Ported from WSJT-X `sync8.f90` + `sync8d.f90`; generalised so the same
//! code handles FT8 (3 identical Costas-7 blocks) and FT4 (4 different
//! Costas-4 blocks) by iterating over `FrameLayout::SYNC_BLOCKS`.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::{Protocol, SpectrumWindow};
use crate::core::fft::default_planner;

/// One synchronisation candidate.
#[derive(Debug, Clone)]
pub struct SyncCandidate {
    /// Carrier (tone-0) frequency in Hz.
    pub freq_hz: f32,
    /// Time offset relative to the protocol's nominal TX_START_OFFSET_S, in seconds.
    pub dt_sec: f32,
    /// Normalised sync score (larger = better).
    pub score: f32,
}

/// DT median of the top-`top_k` highest-score coarse-sync candidates.
///
/// Used to bootstrap slot alignment when zero confirmed decodes are
/// available (cold start, or a deep-fade slot). Empirically — on
/// reference qso3_busy / WSJT-X 191111 captures — the top-5 candidate
/// DT median lands within ±70 ms of the confirmed-decode DT median,
/// while top-10/20 wash out under false-candidate noise (see
/// `mfsk-core/tests/ft8_coarse_sync_bootstrap.rs`).
///
/// `cands` does not need to be sorted; callers pass the raw output of
/// `decode_block::coarse_sync` or `core::sync::coarse_sync`. Returns
/// `None` if `cands` is empty or `top_k == 0`.
pub fn bootstrap_dt_median(cands: &[SyncCandidate], top_k: usize) -> Option<f32> {
    if cands.is_empty() || top_k == 0 {
        return None;
    }
    let mut sorted: Vec<&SyncCandidate> = cands.iter().collect();
    sorted.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(core::cmp::Ordering::Equal));
    let mut dts: Vec<f32> = sorted.iter().take(top_k).map(|c| c.dt_sec).collect();
    dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let n = dts.len();
    Some(if n % 2 == 1 {
        dts[n / 2]
    } else {
        0.5 * (dts[n / 2 - 1] + dts[n / 2])
    })
}

// ──────────────────────────────────────────────────────────────────────────
// Per-protocol DSP parameter bundle (all derived from P at compile time)
// ──────────────────────────────────────────────────────────────────────────

/// Static-per-protocol parameters used throughout sync. Derived from the
/// `Protocol` trait; inlined by the compiler.
#[derive(Copy, Clone, Debug)]
pub struct SyncDims {
    /// Per-symbol FFT length (= NSPS · NFFT_PER_SYMBOL_FACTOR).
    pub nfft1: usize,
    /// Coarse-sync time-step in samples (= NSPS / NSTEP_PER_SYMBOL).
    pub nstep: usize,
    /// Samples per symbol at 12 kHz.
    pub nsps: usize,
    /// Steps per symbol (= NSTEP_PER_SYMBOL).
    pub nssy: usize,
    /// Frequency oversampling factor (= NFFT_PER_SYMBOL_FACTOR).
    pub nfos: usize,
    /// Slot length in samples at 12 kHz.
    pub nmax: usize,
    /// Time-spectra column count = NMAX / NSTEP - 3.
    pub nhsym: usize,
    /// Positive-frequency bins NFFT1 / 2.
    pub nh1: usize,
    /// Frequency resolution (Hz/bin) = 12_000 / NFFT1.
    pub df: f32,
    /// Time step (s) between coarse-sync columns.
    pub tstep: f32,
    /// Symbol offset (in NSTEP steps) of the nominal frame start.
    /// = round(TX_START_OFFSET_S / tstep).
    pub jstrt: i32,
    /// Max search lag in NSTEP steps (±2.5 s by convention).
    pub jz: i32,
    /// Downsampled samples per symbol (= NSPS / NDOWN).
    pub ds_spb: usize,
    /// Downsampled sample rate (Hz) = 12_000 / NDOWN.
    pub ds_rate: f32,
}

impl SyncDims {
    #[inline]
    pub const fn of<P: Protocol>() -> Self {
        let nsps = P::NSPS as usize;
        let nstep = nsps / P::NSTEP_PER_SYMBOL as usize;
        let nfft1 = nsps * P::NFFT_PER_SYMBOL_FACTOR as usize;
        let nmax = (P::T_SLOT_S * 12_000.0) as usize;
        let ndown = P::NDOWN as usize;
        Self {
            nfft1,
            nstep,
            nsps,
            nssy: P::NSTEP_PER_SYMBOL as usize,
            nfos: P::NFFT_PER_SYMBOL_FACTOR as usize,
            nmax,
            nhsym: nmax / nstep - 3,
            nh1: nfft1 / 2,
            df: 12_000.0 / nfft1 as f32,
            tstep: nstep as f32 / 12_000.0,
            jstrt: (P::TX_START_OFFSET_S / (nstep as f32 / 12_000.0)) as i32,
            jz: (2.5 / (nstep as f32 / 12_000.0)) as i32,
            ds_spb: nsps / ndown,
            ds_rate: 12_000.0 / ndown as f32,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Coarse sync
// ──────────────────────────────────────────────────────────────────────────

/// Flat (n_freq × n_time) spectrogram stored row-major by frequency.
pub struct Spectrogram {
    pub n_freq: usize,
    pub n_time: usize,
    data: Vec<f32>,
}

impl Spectrogram {
    #[inline]
    fn get(&self, freq: usize, time: usize) -> f32 {
        self.data[freq * self.n_time + time]
    }

    /// Mean linear power per FFT bin, averaged across all time slices.
    ///
    /// Returns `Vec<f32>` of length [`Self::n_freq`]. Used by
    /// [`crate::core::baseline::fit_baseline`] to compute the
    /// per-frequency noise floor (WSJT-X `ft4_baseline.f90` /
    /// `baseline.f90` first input). Memory layout is row-major by
    /// frequency, so each output entry is a contiguous reduction.
    pub fn avg_power_per_bin(&self) -> Vec<f32> {
        let inv_t = 1.0 / self.n_time as f32;
        let mut out = vec![0.0f32; self.n_freq];
        for f in 0..self.n_freq {
            let base = f * self.n_time;
            let mut s = 0.0f32;
            for t in 0..self.n_time {
                s += self.data[base + t];
            }
            out[f] = s * inv_t;
        }
        out
    }
}

/// Build the per-sample Nuttall-4 window of length `n`.
/// Matches WSJT-X `nuttal_window.f90`. Coefficients fixed by the
/// CW shape of the window — see `SpectrumWindow::Nuttall4` doc.
fn nuttall_window(n: usize) -> Vec<f32> {
    const A0: f32 = 0.3635819;
    const A1: f32 = 0.4891775;
    const A2: f32 = 0.1365995;
    const A3: f32 = 0.0106411;
    let mut w = vec![0.0f32; n];
    if n < 2 {
        if n == 1 {
            w[0] = 1.0;
        }
        return w;
    }
    let two_pi = 2.0 * PI;
    let denom = (n - 1) as f32;
    for (k, slot) in w.iter_mut().enumerate() {
        let x = k as f32 / denom;
        *slot = A0 - A1 * (two_pi * x).cos() + A2 * (2.0 * two_pi * x).cos()
            - A3 * (3.0 * two_pi * x).cos();
    }
    w
}

/// Compute per-time-step power spectra from raw 12 kHz PCM.
///
/// The per-NSPS-sample chunk is multiplied by `Protocol::SPECTRUM_WINDOW`
/// before the NFFT1-point FFT. FT4 uses [`SpectrumWindow::Nuttall4`] to
/// match WSJT-X `getcandidates4.f90:22` (sidelobe leakage from strong
/// signals would otherwise inflate the per-bin polynomial baseline and
/// mask weak signals); FT8 stays on `Rectangular` (its synth-roundtrip
/// path is calibrated against rectangular).
pub fn compute_spectra<P: Protocol>(audio: &[i16]) -> Spectrogram {
    let d = SyncDims::of::<P>();
    let fac = 1.0f32 / 300.0;
    let mut planner = default_planner();
    let fft = planner.plan_forward(d.nfft1);

    let window: Option<Vec<f32>> = match P::SPECTRUM_WINDOW {
        SpectrumWindow::Rectangular => None,
        SpectrumWindow::Nuttall4 => Some(nuttall_window(d.nsps)),
    };

    let mut data = vec![0.0f32; d.nh1 * d.nhsym];
    let mut buf = vec![Complex::new(0.0f32, 0.0); d.nfft1];

    for j in 0..d.nhsym {
        let ia = j * d.nstep;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < d.nsps {
                let sample = if ia + k < audio.len() {
                    let raw = audio[ia + k] as f32 * fac;
                    match &window {
                        Some(w) => raw * w[k],
                        None => raw,
                    }
                } else {
                    0.0
                };
                Complex::new(sample, 0.0)
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        for i in 0..d.nh1 {
            data[i * d.nhsym + j] = buf[i].norm_sqr();
        }
    }

    Spectrogram {
        n_freq: d.nh1,
        n_time: d.nhsym,
        data,
    }
}

/// Coarse sync: search audio for candidate frames.
///
/// Matches the sync shape of the protocol's `SYNC_BLOCKS`. Returns up to
/// `max_cand` candidates, sorted by score (best first); if `freq_hint` is
/// supplied, nearby candidates are promoted.
///
/// **FT8 callers should not use this function.** As of v0.6 (#48), FT8
/// coarse-sync is owned by [`crate::ft8::decode_block::coarse_sync`],
/// which uses the WSJT-X `sync8.f90`-faithful 16-bin sliding-window
/// allsum noise estimator instead of the same-time-slot non-Costas
/// reference this generic function uses. The generic function stays
/// for FT4 / FST4 / JT9 / Q65 / WSPR / uvpacket where the busy-band
/// recall gap that motivated the FT8 swap (see #40) has not been
/// observed.
pub fn coarse_sync<P: Protocol>(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let d = SyncDims::of::<P>();
    let s = compute_spectra::<P>(audio);
    let ntones = P::NTONES as usize;
    let pattern_len = P::SYNC_MODE.blocks()[0].pattern.len();

    // Leave room for NTONES-1 tones above the candidate bin.
    let ia = (freq_min / d.df).round() as usize;
    let headroom = d.nfos * (ntones - 1) + 1;
    let ib = ((freq_max / d.df).round() as usize).min(d.nh1.saturating_sub(headroom));
    if ib < ia {
        return Vec::new();
    }

    let n_freq = ib - ia + 1;
    let n_lag = (2 * d.jz + 1) as usize;
    let mut sync2d = vec![0.0f32; n_freq * n_lag];
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + d.jz) as usize;

    // Per-block (t_block_k, t0_block_k) accumulators. All-blocks score =
    // Σ t/Σ t0_mean. Trailing-(N-1)-blocks score excludes block 0 (the
    // FT8 heuristic that a late start can still sync on blocks 1..).
    let num_blocks = P::SYNC_MODE.blocks().len();

    for (fi, i) in (ia..=ib).enumerate() {
        for lag in -d.jz..=d.jz {
            // Accumulate per-sync-block correlation power.
            let mut t_blocks = vec![0.0f32; num_blocks];
            let mut t0_blocks = vec![0.0f32; num_blocks];

            for (bk, block) in P::SYNC_MODE.blocks().iter().enumerate() {
                let block_offset = d.nssy as i32 * block.start_symbol as i32;
                for (n, &costas_n) in block.pattern.iter().enumerate() {
                    let m = lag + d.jstrt + block_offset + (d.nssy * n) as i32;
                    let tone_bin = i + d.nfos * costas_n as usize;
                    if m >= 0 && (m as usize) < d.nhsym && tone_bin < d.nh1 {
                        let m = m as usize;
                        t_blocks[bk] += s.get(tone_bin, m);
                        // Reference: sum over all NTONES tones at this time slot.
                        t0_blocks[bk] += (0..ntones)
                            .map(|k| s.get((i + d.nfos * k).min(d.nh1 - 1), m))
                            .sum::<f32>();
                    }
                }
            }

            // All blocks combined.
            let t_all: f32 = t_blocks.iter().sum();
            let t0_all: f32 = t0_blocks.iter().sum();
            // Reference excludes the signal energy: normalise by
            // (t0_total - t_total) / (NTONES - 1).
            // Zero-denominator case: a CLEAN synthetic signal lands
            // entirely on Costas tones (`t0_all == t_all`) so the
            // "non-Costas tone power" is zero. Treat that as a perfect
            // match (the signal is *all* Costas energy) and report
            // `t_all` directly — a large number that beats any
            // noise-floor candidate. Without this, pure-synth
            // round-trip tests get score 0 at the signal bin.
            let t0_ref = (t0_all - t_all) / (ntones as f32 - 1.0);
            let sync_all = if t0_ref > f32::EPSILON {
                t_all / t0_ref
            } else if t_all > 0.0 {
                t_all
            } else {
                0.0
            };

            // Trailing N-1 blocks (drop the first), to tolerate an early-block loss.
            let score = if num_blocks > 1 {
                let t_tail: f32 = t_blocks[1..].iter().sum();
                let t0_tail: f32 = t0_blocks[1..].iter().sum();
                let t0_tail_ref = (t0_tail - t_tail) / (ntones as f32 - 1.0);
                let sync_tail = if t0_tail_ref > f32::EPSILON {
                    t_tail / t0_tail_ref
                } else if t_tail > 0.0 {
                    t_tail
                } else {
                    0.0
                };
                sync_all.max(sync_tail)
            } else {
                sync_all
            };

            sync2d[idx(fi, lag)] = score;
        }
    }

    // Per-frequency peak detection — non-maximum suppression.
    //
    // The previous implementation kept one or two peaks per
    // frequency bin (best in ±MLAG, plus best in ±jz when
    // distinct). That works for slot-based protocols (FT8, FT4,
    // WSPR, JT9/65, Q65) where one transmitter occupies one
    // (freq, slot) cell. It silently drops most frames for
    // chained-frame protocols where many frames sit at the same
    // audio centre, separated only in time.
    //
    // The multi-peak NMS below is a strict superset: for slot-
    // based protocols the second-best lag scores below sync_min
    // after normalisation and is filtered out, recovering the
    // previous behaviour. For chained-frame protocols every frame
    // whose Costas peak survives MLAG-spacing NMS is emitted as
    // its own candidate.
    const MLAG: i32 = 10;

    // First compute the per-bin best score (still needed for the
    // 40-percentile noise-floor normalisation as a fallback).
    let mut red = vec![0.0f32; n_freq];
    for fi in 0..n_freq {
        red[fi] = (-d.jz..=d.jz)
            .map(|lag| sync2d[idx(fi, lag)])
            .fold(0.0f32, f32::max);
    }

    let pct = |xs: &[f32]| {
        let mut sorted = xs.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct_idx = (0.40 * n_freq as f32) as usize;
        sorted[pct_idx.min(n_freq - 1)].max(f32::EPSILON)
    };
    let global_base = pct(&red);

    // Reverted slice 1's per-bin polynomial baseline divisor (issue
    // #18 follow-up): on real WAVs with multiple coexisting signals
    // it inverts the priority — the polyfit baseline tracks the
    // signal-contaminated avg power, raising the divisor ABOVE
    // `global_base` at signal bins (halving real-signal scores) and
    // leaving it at floor in quiet noise regions (where Costas-
    // correlation false alarms from random tones inflate the
    // ranking). Wide-band ranks of the WSJT-X golden signals dropped
    // to 229-2905 / 4000 — well below `max_cand` cutoffs — while
    // spurious peaks at 1234-1250 Hz topped the list at scores
    // 12-18. Plain `global_base` keeps real-signal scores at ~1.0
    // and spurious at ~0.7, so the goldens make the candidate list.
    //
    // The polyfit baseline still has value for **per-symbol LLR
    // normalisation** (slice 2 territory) but that's a separate
    // place from the candidate ranking. Leave the helper
    // `core::baseline::fit_baseline` in place for that future use.
    let sbase: Vec<f32> = vec![global_base; n_freq];

    let mut cands: Vec<SyncCandidate> = Vec::new();
    for fi in 0..n_freq {
        let i = ia + fi;
        let freq_hz = i as f32 * d.df;

        // Per-bin baseline divisor; falls back to the global one
        // computed above when polyfit didn't run.
        let local_base = sbase[fi];

        // Collect every (lag, normalised_score) pair above the
        // threshold within the full ±jz lag range.
        let mut peaks: Vec<(i32, f32)> = (-d.jz..=d.jz)
            .filter_map(|lag| {
                let raw = sync2d[idx(fi, lag)];
                let norm = raw / local_base;
                if norm.is_finite() && norm >= sync_min {
                    Some((lag, norm))
                } else {
                    None
                }
            })
            .collect();
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        // Greedy NMS: each pick suppresses every neighbour within
        // ±MLAG. Two genuine frames at the same audio centre are
        // separated by ≥ frame airtime in lag steps, far more than
        // MLAG, so they survive as distinct candidates.
        let mut picked: Vec<i32> = Vec::new();
        'outer: for (lag, score) in peaks {
            for &pl in &picked {
                if (lag - pl).abs() <= MLAG {
                    continue 'outer;
                }
            }
            picked.push(lag);
            cands.push(SyncCandidate {
                freq_hz,
                dt_sec: (lag as f32 - 0.5) * d.tstep,
                score,
            });
            // Cap per-bin candidates so a noisy bin can't crowd out
            // the rest of the spectrum.
            if picked.len() >= 8 {
                break;
            }
        }
    }
    let _ = pattern_len; // currently unused; kept for future scoring weights

    // De-duplicate: within 4 Hz and 40 ms, keep highest score.
    for i in 1..cands.len() {
        for j in 0..i {
            let fdiff = (cands[i].freq_hz - cands[j].freq_hz).abs();
            let tdiff = (cands[i].dt_sec - cands[j].dt_sec).abs();
            if fdiff < 4.0 && tdiff < 0.04 {
                if cands[i].score >= cands[j].score {
                    cands[j].score = 0.0;
                } else {
                    cands[i].score = 0.0;
                }
            }
        }
    }
    cands.retain(|c| c.score >= sync_min);

    if let Some(fhint) = freq_hint {
        cands.sort_by(|a, b| {
            let a_near = (a.freq_hz - fhint).abs() <= 10.0;
            let b_near = (b.freq_hz - fhint).abs() <= 10.0;
            match (a_near, b_near) {
                (true, false) => core::cmp::Ordering::Less,
                (false, true) => core::cmp::Ordering::Greater,
                _ => b.score.partial_cmp(&a.score).unwrap(),
            }
        });
    } else {
        cands.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    }

    cands.truncate(max_cand);
    cands
}

// ──────────────────────────────────────────────────────────────────────────
// Fine sync (Costas correlation on downsampled complex baseband)
// ──────────────────────────────────────────────────────────────────────────

/// Build complex sinusoidal references (one per Costas tone) for a sync block.
pub fn make_costas_ref(pattern: &[u8], ds_spb: usize) -> Vec<Vec<Complex<f32>>> {
    pattern
        .iter()
        .map(|&tone| {
            let dphi = 2.0 * PI * tone as f32 / ds_spb as f32;
            let mut waves = vec![Complex::new(0.0f32, 0.0); ds_spb];
            let mut phi = 0.0f32;
            for w in waves.iter_mut() {
                *w = Complex::new(phi.cos(), phi.sin());
                phi = (phi + dphi) % (2.0 * PI);
            }
            waves
        })
        .collect()
}

/// Correlate a single Costas block starting at sample `array_start` in `cd0`.
/// `array_start` is signed so callers can pass an `i_start` derived from a
/// candidate with negative `dt_sec` (signal that started before the cd0
/// window). WSJT-X `sync8d.f90:43-45` policy: if any of the `ds_spb` samples
/// would fall outside `cd0`, the block contributes 0 (rather than partially
/// summing).
pub fn score_costas_block(
    cd0: &[Complex<f32>],
    csync: &[Vec<Complex<f32>>],
    ds_spb: usize,
    array_start: i32,
) -> f32 {
    let np2 = cd0.len() as i32;
    csync
        .iter()
        .enumerate()
        .map(|(k, ref_tone)| {
            let start = array_start + (k * ds_spb) as i32;
            if start >= 0 && start + ds_spb as i32 <= np2 {
                let s0 = start as usize;
                cd0[s0..s0 + ds_spb]
                    .iter()
                    .zip(ref_tone.iter())
                    .map(|(&s, &r)| s * r.conj())
                    .sum::<Complex<f32>>()
                    .norm_sqr()
            } else {
                0.0
            }
        })
        .sum()
}

/// Sum of Costas correlation powers across all sync blocks.
pub fn fine_sync_power<P: Protocol>(cd0: &[Complex<f32>], i0: i32) -> f32 {
    fine_sync_power_per_block::<P>(cd0, i0).into_iter().sum()
}

/// Per-block Costas correlation powers for diagnostics and the FT8 double-sync.
pub fn fine_sync_power_per_block<P: Protocol>(cd0: &[Complex<f32>], i0: i32) -> Vec<f32> {
    let d = SyncDims::of::<P>();
    P::SYNC_MODE
        .blocks()
        .iter()
        .map(|block| {
            let csync = make_costas_ref(block.pattern, d.ds_spb);
            let start = i0 + (block.start_symbol as usize * d.ds_spb) as i32;
            score_costas_block(cd0, &csync, d.ds_spb, start)
        })
        .collect()
}

/// Parabolic peak interpolation: returns `(subsample_offset in [-0.5, 0.5], interpolated_peak)`.
pub fn parabolic_peak(y_neg: f32, y_0: f32, y_pos: f32) -> (f32, f32) {
    let denom = y_neg - 2.0 * y_0 + y_pos;
    if denom.abs() < f32::EPSILON {
        return (0.0, y_0);
    }
    let offset = 0.5 * (y_neg - y_pos) / denom;
    let peak = y_0 - 0.25 * (y_neg - y_pos) * offset;
    (offset.clamp(-0.5, 0.5), peak)
}

/// Refine timing by scanning ±`search_steps` downsampled samples, then
/// applying parabolic sub-sample interpolation around the peak for a
/// fractional-sample refinement. The sub-sample shift is used to report a
/// more accurate `dt_sec` but the returned score is the integer peak
/// (interpolating correlation peaks biases small values downward).
pub fn refine_candidate<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    search_steps: i32,
) -> SyncCandidate {
    let d = SyncDims::of::<P>();
    let nominal_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let (best_i0, best_score) = (-search_steps..=search_steps)
        .map(|delta| {
            let i0 = nominal_i0 + delta;
            let score = fine_sync_power::<P>(cd0, i0);
            (i0, score)
        })
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap_or((nominal_i0, 0.0));

    // Parabolic sub-sample refinement around the integer peak.
    let y_neg = fine_sync_power::<P>(cd0, best_i0 - 1);
    let y_pos = fine_sync_power::<P>(cd0, best_i0 + 1);
    let (frac, _) = parabolic_peak(y_neg, best_score, y_pos);

    SyncCandidate {
        freq_hz: candidate.freq_hz,
        dt_sec: (best_i0 as f32 + frac) / d.ds_rate - P::TX_START_OFFSET_S,
        score: best_score,
    }
}

/// Diagnostic result for double-sync refinement.
#[derive(Debug, Clone)]
pub struct FineSyncDetail {
    pub candidate: SyncCandidate,
    /// Per-block Costas correlation powers at the averaged timing.
    pub per_block_scores: Vec<f32>,
    /// Time drift across the first and last sync blocks (seconds).
    /// Near zero for real signals, large for ghosts.
    pub drift_dt_sec: f32,
}

/// Refine a candidate using independent first-block / last-block peak search.
///
/// Generalises the FT8 "double sync" idea to any number of sync blocks: scan
/// the first block and the last block independently, compute a parabolic
/// sub-sample refinement, and report their disagreement as `drift_dt_sec`.
pub fn refine_candidate_double<P: Protocol>(
    cd0: &[Complex<f32>],
    candidate: &SyncCandidate,
    search_steps: i32,
) -> FineSyncDetail {
    let d = SyncDims::of::<P>();
    let blocks = P::SYNC_MODE.blocks();
    let first = &blocks[0];
    let last = &blocks[blocks.len() - 1];
    let csync_first = make_costas_ref(first.pattern, d.ds_spb);
    let csync_last = make_costas_ref(last.pattern, d.ds_spb);

    let nominal_i0 = ((candidate.dt_sec + P::TX_START_OFFSET_S) * d.ds_rate).round() as i32;

    let best_for = |pattern: &[u8], csync: &[Vec<Complex<f32>>], block_start: u32| {
        let _ = pattern;
        let block_off = (block_start as usize * d.ds_spb) as i32;
        let (best_i0, _) = (-search_steps..=search_steps)
            .map(|delta| {
                let i0 = nominal_i0 + delta;
                let off = i0 + block_off;
                (i0, score_costas_block(cd0, csync, d.ds_spb, off))
            })
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((nominal_i0, 0.0));
        // Parabolic sub-sample
        let off_neg = (best_i0 - 1) + block_off;
        let off_0 = best_i0 + block_off;
        let off_pos = (best_i0 + 1) + block_off;
        let (frac, _) = parabolic_peak(
            score_costas_block(cd0, csync, d.ds_spb, off_neg),
            score_costas_block(cd0, csync, d.ds_spb, off_0),
            score_costas_block(cd0, csync, d.ds_spb, off_pos),
        );
        (best_i0, frac)
    };

    let (best_i0_a, frac_a) = best_for(first.pattern, &csync_first, first.start_symbol);
    let (best_i0_c, frac_c) = best_for(last.pattern, &csync_last, last.start_symbol);

    let dt_a = best_i0_a as f32 / d.ds_rate + frac_a / d.ds_rate - P::TX_START_OFFSET_S;
    let dt_c = best_i0_c as f32 / d.ds_rate + frac_c / d.ds_rate - P::TX_START_OFFSET_S;
    let drift_dt_sec = dt_c - dt_a;

    let avg_i0 = ((best_i0_a + best_i0_c) as f32 * 0.5).round() as i32;
    let per_block_scores = fine_sync_power_per_block::<P>(cd0, avg_i0);
    let total: f32 = per_block_scores.iter().sum();

    FineSyncDetail {
        candidate: SyncCandidate {
            freq_hz: candidate.freq_hz,
            dt_sec: (dt_a + dt_c) * 0.5,
            score: total,
        },
        per_block_scores,
        drift_dt_sec,
    }
}
