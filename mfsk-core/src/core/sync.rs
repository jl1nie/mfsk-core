//! Protocol-agnostic synchronisation primitives.
//!
//! Coarse sync searches the 2D (freq, lag) plane for candidate frames by
//! correlating per-symbol power spectra against the protocol's sync-block
//! tone patterns. Fine sync refines the timing on the downsampled complex
//! baseband signal.
//!
//! Ported from WSJT-X `sync8.f90` + `sync8d.f90`; generalised so the same
//! code handles FT8 (3 identical Costas-7 blocks) and FT4 (4 different
//! Costas-4 blocks) by iterating over [`FrameLayout::SYNC_BLOCKS`].

use super::Protocol;
use num_complex::Complex;
use rustfft::FftPlanner;
use std::f32::consts::PI;

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
}

/// Compute per-time-step power spectra from raw 12 kHz PCM.
pub fn compute_spectra<P: Protocol>(audio: &[i16]) -> Spectrogram {
    let d = SyncDims::of::<P>();
    let fac = 1.0f32 / 300.0;
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(d.nfft1);

    let mut data = vec![0.0f32; d.nh1 * d.nhsym];
    let mut buf = vec![Complex::new(0.0f32, 0.0); d.nfft1];

    for j in 0..d.nhsym {
        let ia = j * d.nstep;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < d.nsps {
                let sample = if ia + k < audio.len() {
                    audio[ia + k] as f32 * fac
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

    Spectrogram { n_freq: d.nh1, n_time: d.nhsym, data }
}

/// Coarse sync: search audio for candidate frames.
///
/// Matches the sync shape of the protocol's `SYNC_BLOCKS`. Returns up to
/// `max_cand` candidates, sorted by score (best first); if `freq_hint` is
/// supplied, nearby candidates are promoted.
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
            let t0_ref = (t0_all - t_all) / (ntones as f32 - 1.0);
            let sync_all = if t0_ref > 0.0 { t_all / t0_ref } else { 0.0 };

            // Trailing N-1 blocks (drop the first), to tolerate an early-block loss.
            let score = if num_blocks > 1 {
                let t_tail: f32 = t_blocks[1..].iter().sum();
                let t0_tail: f32 = t0_blocks[1..].iter().sum();
                let t0_tail_ref = (t0_tail - t_tail) / (ntones as f32 - 1.0);
                let sync_tail = if t0_tail_ref > 0.0 { t_tail / t0_tail_ref } else { 0.0 };
                sync_all.max(sync_tail)
            } else {
                sync_all
            };

            sync2d[idx(fi, lag)] = score;
        }
    }

    // Per-frequency peak detection.
    const MLAG: i32 = 10;
    let mut red = vec![0.0f32; n_freq];
    let mut red2 = vec![0.0f32; n_freq];
    let mut jpeak = vec![0i32; n_freq];
    let mut jpeak2 = vec![0i32; n_freq];

    for fi in 0..n_freq {
        let (jp, rv) = (-MLAG..=MLAG)
            .map(|lag| (lag, sync2d[idx(fi, lag)]))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((0, 0.0));
        jpeak[fi] = jp;
        red[fi] = rv;

        let (jp2, rv2) = (-d.jz..=d.jz)
            .map(|lag| (lag, sync2d[idx(fi, lag)]))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((0, 0.0));
        jpeak2[fi] = jp2;
        red2[fi] = rv2;
    }

    let pct = |xs: &[f32]| {
        let mut sorted = xs.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct_idx = (0.40 * n_freq as f32) as usize;
        sorted[pct_idx.min(n_freq - 1)].max(f32::EPSILON)
    };
    let base = pct(&red);
    let base2 = pct(&red2);
    for r in red.iter_mut() { *r /= base; }
    for r in red2.iter_mut() { *r /= base2; }

    let mut cands: Vec<SyncCandidate> = Vec::new();
    let mut order: Vec<usize> = (0..n_freq).collect();
    order.sort_by(|&a, &b| red[b].partial_cmp(&red[a]).unwrap());

    for fi in order {
        if cands.len() >= max_cand * 2 {
            break;
        }
        let i = ia + fi;
        if red[fi] >= sync_min && !red[fi].is_nan() {
            cands.push(SyncCandidate {
                freq_hz: i as f32 * d.df,
                dt_sec: (jpeak[fi] as f32 - 0.5) * d.tstep,
                score: red[fi],
            });
        }
        if jpeak2[fi] != jpeak[fi] && red2[fi] >= sync_min && !red2[fi].is_nan() {
            cands.push(SyncCandidate {
                freq_hz: i as f32 * d.df,
                dt_sec: (jpeak2[fi] as f32 - 0.5) * d.tstep,
                score: red2[fi],
            });
        }
        let _ = pattern_len; // silence unused
    }

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
                (true, false) => std::cmp::Ordering::Less,
                (false, true) => std::cmp::Ordering::Greater,
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
pub fn score_costas_block(
    cd0: &[Complex<f32>],
    csync: &[Vec<Complex<f32>>],
    ds_spb: usize,
    array_start: usize,
) -> f32 {
    let np2 = cd0.len();
    csync
        .iter()
        .enumerate()
        .map(|(k, ref_tone)| {
            let start = array_start + k * ds_spb;
            if start + ds_spb <= np2 {
                cd0[start..start + ds_spb]
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
pub fn fine_sync_power<P: Protocol>(cd0: &[Complex<f32>], i0: usize) -> f32 {
    fine_sync_power_per_block::<P>(cd0, i0).into_iter().sum()
}

/// Per-block Costas correlation powers for diagnostics and the FT8 double-sync.
pub fn fine_sync_power_per_block<P: Protocol>(cd0: &[Complex<f32>], i0: usize) -> Vec<f32> {
    let d = SyncDims::of::<P>();
    P::SYNC_MODE.blocks()
        .iter()
        .map(|block| {
            let csync = make_costas_ref(block.pattern, d.ds_spb);
            let start = i0 + block.start_symbol as usize * d.ds_spb;
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
    let nominal_i0 =
        ((candidate.dt_sec + P::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let (best_i0, best_score) = (-search_steps..=search_steps)
        .map(|delta| {
            let i0 = (nominal_i0 + delta).max(0) as usize;
            let score = fine_sync_power::<P>(cd0, i0);
            (i0, score)
        })
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap_or((0, 0.0));

    // Parabolic sub-sample refinement around the integer peak.
    let frac = if best_i0 >= 1 {
        let y_neg = fine_sync_power::<P>(cd0, best_i0 - 1);
        let y_pos = fine_sync_power::<P>(cd0, best_i0 + 1);
        let (f, _) = parabolic_peak(y_neg, best_score, y_pos);
        f
    } else {
        0.0
    };

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

    let nominal_i0 =
        ((candidate.dt_sec + P::TX_START_OFFSET_S) * d.ds_rate).round() as i32;

    let best_for = |pattern: &[u8], csync: &[Vec<Complex<f32>>], block_start: u32| {
        let _ = pattern;
        let (best_i0, _) = (-search_steps..=search_steps)
            .map(|delta| {
                let i0 = (nominal_i0 + delta).max(0) as usize;
                let off = i0 + block_start as usize * d.ds_spb;
                (i0, score_costas_block(cd0, csync, d.ds_spb, off))
            })
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((nominal_i0.max(0) as usize, 0.0));
        // Parabolic sub-sample
        let frac = if best_i0 > 0 {
            let off_neg = (best_i0 - 1) + block_start as usize * d.ds_spb;
            let off_0 = best_i0 + block_start as usize * d.ds_spb;
            let off_pos = (best_i0 + 1) + block_start as usize * d.ds_spb;
            let (f, _) = parabolic_peak(
                score_costas_block(cd0, csync, d.ds_spb, off_neg),
                score_costas_block(cd0, csync, d.ds_spb, off_0),
                score_costas_block(cd0, csync, d.ds_spb, off_pos),
            );
            f
        } else {
            0.0
        };
        (best_i0, frac)
    };

    let (best_i0_a, frac_a) = best_for(first.pattern, &csync_first, first.start_symbol);
    let (best_i0_c, frac_c) = best_for(last.pattern, &csync_last, last.start_symbol);

    let dt_a = best_i0_a as f32 / d.ds_rate + frac_a / d.ds_rate - P::TX_START_OFFSET_S;
    let dt_c = best_i0_c as f32 / d.ds_rate + frac_c / d.ds_rate - P::TX_START_OFFSET_S;
    let drift_dt_sec = dt_c - dt_a;

    let avg_i0 = ((best_i0_a + best_i0_c) as f32 * 0.5).round() as usize;
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
