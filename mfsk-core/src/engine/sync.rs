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
#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::{Protocol, SpectrumWindow};
use crate::engine::fft::default_planner;

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
/// `decode_block::coarse_sync` or `engine::sync::coarse_sync`. Returns
/// `None` if `cands` is empty or `top_k == 0`.
pub fn bootstrap_dt_median(cands: &[SyncCandidate], top_k: usize) -> Option<f32> {
    if cands.is_empty() || top_k == 0 {
        return None;
    }
    // O(N) top-K partition via `select_nth_unstable_by`, then
    // O(K log K) sort of just the K winners. Saves ~N log N vs full
    // sort; for N≈200 / K=5 / one call per slot the saving is sub-µs,
    // but the cost is identical to the naïve approach so we take it.
    let mut refs: Vec<&SyncCandidate> = cands.iter().collect();
    let k = top_k.min(refs.len());
    if k < refs.len() {
        refs.select_nth_unstable_by(k - 1, |a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(core::cmp::Ordering::Equal)
        });
    }
    let mut dts: Vec<f32> = refs[..k].iter().map(|c| c.dt_sec).collect();
    dts.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
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
///
/// Cropped to `[freq_offset, freq_offset + n_freq)` in absolute bin
/// terms — `compute_spectra` no longer materialises every
/// positive-frequency bin (issue #143, VK3NV: the un-cropped version
/// cost 11.4 MB for FST4-120 alone). `get`/`avg_power_per_bin`'s
/// callers keep using absolute bin indices; the offset subtraction
/// happens once, here.
pub struct Spectrogram {
    pub n_freq: usize,
    pub n_time: usize,
    /// Absolute bin index the cropped `data` starts at (0 = no crop).
    freq_offset: usize,
    data: Vec<f32>,
}

impl Spectrogram {
    #[inline]
    fn get(&self, freq: usize, time: usize) -> f32 {
        debug_assert!(
            freq >= self.freq_offset,
            "Spectrogram::get: freq {freq} below cropped range starting at {}",
            self.freq_offset
        );
        self.data[(freq - self.freq_offset) * self.n_time + time]
    }

    /// Absolute bin index [`Self::avg_power_per_bin`]'s output (and
    /// `get`'s `freq` argument) is offset from — 0 if the spectrogram
    /// wasn't cropped. Callers holding an absolute bin index `i` read
    /// `avg_power_per_bin()[i - freq_offset()]`.
    #[inline]
    pub fn freq_offset(&self) -> usize {
        self.freq_offset
    }

    /// Mean linear power per FFT bin, averaged across all time slices.
    ///
    /// Returns `Vec<f32>` of length [`Self::n_freq`], indexed
    /// **relative to [`Self::freq_offset`]** (not an absolute bin
    /// index — subtract `freq_offset()` from an absolute bin first).
    /// Used by [`crate::engine::baseline::fit_baseline`] to compute the
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
///
/// `pub(crate)`: reused by [`crate::ft4::coarse`] (`getcandidates4.f90`
/// faithful port), which needs it at `NFFT1` length, not just the
/// `Protocol::SPECTRUM_WINDOW`-gated `NSPS` length this module applies
/// internally.
pub(crate) fn nuttall_window(n: usize) -> Vec<f32> {
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
/// Only bins `[bin_lo, bin_hi_incl]` (absolute, inclusive) are kept —
/// this crop is the caller's job to size correctly (`coarse_sync`
/// passes `[ia, ib + headroom]`, matching the range its own
/// correlation/candidate search ever reads; see its doc comment for
/// the `headroom` derivation). Cropping here rather than after the
/// fact is the whole point (issue #143, VK3NV): a full-band
/// spectrogram for e.g. FST4-120 is 11.4 MB even though `coarse_sync`
/// only ever touches a narrow slice of it.
///
/// The per-NSPS-sample chunk is multiplied by `Protocol::SPECTRUM_WINDOW`
/// before the NFFT1-point FFT. FT4 uses [`SpectrumWindow::Nuttall4`] to
/// match WSJT-X `getcandidates4.f90:22` (sidelobe leakage from strong
/// signals would otherwise inflate the per-bin polynomial baseline and
/// mask weak signals); FT8 stays on `Rectangular` (its synth-roundtrip
/// path is calibrated against rectangular).
pub fn compute_spectra<P: Protocol>(
    audio: &[i16],
    bin_lo: usize,
    bin_hi_incl: usize,
) -> Spectrogram {
    let d = SyncDims::of::<P>();
    let fac = 1.0f32 / 300.0;
    let mut planner = default_planner();
    let fft = planner.plan_forward(d.nfft1);

    let window: Option<Vec<f32>> = match P::SPECTRUM_WINDOW {
        SpectrumWindow::Rectangular => None,
        SpectrumWindow::Nuttall4 => Some(nuttall_window(d.nsps)),
    };

    let bin_lo = bin_lo.min(d.nh1.saturating_sub(1));
    let bin_hi_incl = bin_hi_incl.min(d.nh1.saturating_sub(1)).max(bin_lo);
    let n_freq = bin_hi_incl - bin_lo + 1;

    let mut data = vec![0.0f32; n_freq * d.nhsym];
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
        for i in bin_lo..=bin_hi_incl {
            data[(i - bin_lo) * d.nhsym + j] = buf[i].norm_sqr();
        }
    }

    Spectrogram {
        n_freq,
        n_time: d.nhsym,
        freq_offset: bin_lo,
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
/// reference this generic function uses.
///
/// **FT4 mostly doesn't use this function either** (corrected 2026-08-09,
/// issue #143 — this doc comment previously claimed otherwise). FT4's
/// main decode strategies (single-pass, `.sic_rounds()`) route through
/// `engine::ft4_coarse::ft4_coarse_sync` instead, a separate
/// `getcandidates4.f90`-faithful port with its own spectrogram
/// construction — see that module's doc comment for why. FT4's
/// `SniperRequest::ap_hint` path (`msg::pipeline_ap`) still calls this
/// function, unconditionally, for any `P: WsjtApCompatible` — so FT4
/// isn't *entirely* off this path, just off it for the common case.
/// FST4 (all 5 sub-modes) is the one protocol still fully on this path
/// today; JT9/Q65/WSPR/uvpacket each have their own separate coarse-sync
/// implementations (verified via `grep coarse_sync::<` — nothing outside
/// `engine/pipeline.rs` and `msg/pipeline_ap.rs` calls this generic
/// function with a non-FST4/FT4 protocol).
pub fn coarse_sync<P: Protocol>(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let d = SyncDims::of::<P>();
    let ntones = P::NTONES as usize;
    let pattern_len = P::SYNC_MODE.blocks()[0].pattern.len();

    // Leave room for NTONES-1 tones above the candidate bin.
    let ia = (freq_min / d.df).round() as usize;
    let headroom = d.nfos * (ntones - 1) + 1;
    let ib = ((freq_max / d.df).round() as usize).min(d.nh1.saturating_sub(headroom));
    if ib < ia {
        return Vec::new();
    }

    // Crop to exactly the range the correlation loop below and the
    // FST4 stage1 augmentation ever read: candidate bins `ia..=ib`
    // plus `headroom` bins above `ib` for their reference tones.
    // `Spectrogram::get` stays absolute-bin-indexed (subtracts
    // `freq_offset` internally) so nothing below needs to change.
    let s = compute_spectra::<P>(audio, ia, (ib + headroom).min(d.nh1.saturating_sub(1)));

    let n_freq = ib - ia + 1;
    let n_lag = (2 * d.jz + 1) as usize;
    let mut sync2d = vec![0.0f32; n_freq * n_lag];
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + d.jz) as usize;

    // Per-block (t_block_k, t0_block_k) accumulators. All-blocks score =
    // Σ t/Σ t0_mean. Trailing-(N-1)-blocks score excludes block 0 (the
    // FT8 heuristic that a late start can still sync on blocks 1..).
    let num_blocks = P::SYNC_MODE.blocks().len();
    // Upper bound on any protocol's block count (FT8=3, FT4=4, FST4=5 —
    // see each protocol's `SYNC_MODE`/`mod.rs` sync-block table) with
    // headroom. Stack arrays sized to this bound let `t_blocks`/
    // `t0_blocks` below be reused across every (freq-bin, lag) cell
    // instead of heap-allocated per cell — this loop runs `n_freq ×
    // (2·d.jz+1)` times per candidate search (thousands of cells), so a
    // fresh `Vec` per cell was thousands of small allocations, and an
    // opaque allocator call is also an optimization barrier LLVM can't
    // see through.
    const MAX_SYNC_BLOCKS: usize = 8;
    debug_assert!(
        num_blocks <= MAX_SYNC_BLOCKS,
        "protocol has more sync blocks than MAX_SYNC_BLOCKS accounts for"
    );

    // Compute correlation scores for every (freq-bin, lag) cell.
    // Each cell is fully independent, so the outer fi loop is safe to parallelise.
    macro_rules! fill_sync2d_row {
        ($fi:expr, $row:expr) => {{
            let i = ia + $fi;
            let mut t_blocks = [0.0f32; MAX_SYNC_BLOCKS];
            let mut t0_blocks = [0.0f32; MAX_SYNC_BLOCKS];
            for (jlag, lag) in (-d.jz..=d.jz).enumerate() {
                t_blocks[..num_blocks].fill(0.0);
                t0_blocks[..num_blocks].fill(0.0);

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
                let t_all: f32 = t_blocks[..num_blocks].iter().sum();
                let t0_all: f32 = t0_blocks[..num_blocks].iter().sum();
                // Zero-denominator: clean synthetic signal lies entirely on
                // Costas tones (t0_all == t_all).  Report t_all directly so
                // round-trip tests score above noise-floor candidates.
                let t0_ref = (t0_all - t_all) / (ntones as f32 - 1.0);
                let sync_all = if t0_ref > f32::EPSILON {
                    t_all / t0_ref
                } else if t_all > 0.0 {
                    t_all
                } else {
                    0.0
                };

                // Trailing N-1 blocks (drop block 0) tolerate an early-block loss.
                let score = if num_blocks > 1 {
                    let t_tail: f32 = t_blocks[1..num_blocks].iter().sum();
                    let t0_tail: f32 = t0_blocks[1..num_blocks].iter().sum();
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

                $row[jlag] = score;
            }
        }};
    }

    #[cfg(feature = "parallel")]
    sync2d
        .par_chunks_mut(n_lag)
        .enumerate()
        .for_each(|(fi, row)| fill_sync2d_row!(fi, row));

    #[cfg(not(feature = "parallel"))]
    for fi in 0..n_freq {
        let start = fi * n_lag;
        fill_sync2d_row!(fi, sync2d[start..start + n_lag]);
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
    #[cfg(feature = "parallel")]
    red.par_iter_mut().enumerate().for_each(|(fi, r)| {
        *r = (-d.jz..=d.jz)
            .map(|lag| sync2d[idx(fi, lag)])
            .fold(0.0f32, f32::max);
    });
    #[cfg(not(feature = "parallel"))]
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
    // `engine::baseline::fit_baseline` in place for that future use.
    let sbase: Vec<f32> = vec![global_base; n_freq];

    // FST4-specific stage-1 augmentation (issue #146): the Costas grid
    // above only correlates against N_SYNC/N_SYMBOLS of the slot's
    // symbols (25% for every FST4 sub-mode, since all five share frame
    // layout), giving ~3 dB less SNR discrimination than a full-slot
    // detector — measured as a flat ~2.3-3.1 dB AWGN recall gap vs
    // WSJT-X's published thresholds across all five periods, matching
    // sqrt(N_SYMBOLS/N_SYNC) = sqrt(4) = 3.01 dB almost exactly.
    // WSJT-X's own FST4 candidate search (`get_candidates_fst4` in
    // `fst4_decode.f90`) never Costas-correlates for candidate
    // detection at all: it sums power at the NTONES candidate-tone
    // offsets across the *entire* slot (every symbol, not just sync
    // ones) before ever doing a timing search. Mirror that here as an
    // OR-gate alongside the existing Costas-grid threshold — a bin
    // that clears the full-slot non-coherent check gets into the
    // candidate list even when its short-time Costas score alone
    // doesn't clear `sync_min` at any lag. The reported `.score` is
    // still the existing Costas-grid value (whatever it is), so
    // downstream OSD-gating semantics (calibrated against that scale)
    // are unaffected.
    let stage1_norm: Vec<f32> = if P::ID == super::ProtocolId::Fst4 {
        let avg_power = s.avg_power_per_bin();
        // `avg_power` is offset-relative (indexed from `s.freq_offset()`,
        // not an absolute bin) — see `Spectrogram::avg_power_per_bin`'s
        // doc comment. The `.min(d.nh1 - 1)` clamp is still against the
        // *absolute* full-band bound (matches `coarse_sync`'s own
        // `headroom` derivation, which sizes the crop to always cover
        // this read), so the offset subtraction happens last.
        let ccf: Vec<f32> = (0..n_freq)
            .map(|fi| {
                let i = ia + fi;
                (0..ntones)
                    .map(|k| {
                        let abs_bin = (i + d.nfos * k).min(d.nh1 - 1);
                        avg_power[(abs_bin - s.freq_offset()).min(avg_power.len() - 1)]
                    })
                    .sum()
            })
            .collect();
        let stage1_base = pct(&ccf);
        ccf.iter().map(|&c| c / stage1_base).collect()
    } else {
        Vec::new()
    };
    let stage1_pass = |fi: usize| stage1_norm.get(fi).copied().unwrap_or(0.0) >= sync_min;

    // Per-fi candidate extraction: each bin is independent.
    // Extract into a closure so both serial and parallel paths share the logic.
    let fi_cands = |fi: usize| -> Vec<SyncCandidate> {
        let i = ia + fi;
        let freq_hz = i as f32 * d.df;
        let local_base = sbase[fi];
        let bin_stage1_pass = stage1_pass(fi);

        let mut peaks: Vec<(i32, f32)> = (-d.jz..=d.jz)
            .filter_map(|lag| {
                let raw = sync2d[idx(fi, lag)];
                let norm = raw / local_base;
                if norm.is_finite() && (norm >= sync_min || bin_stage1_pass) {
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
        let mut out = Vec::new();
        'outer: for (lag, score) in peaks {
            for &pl in &picked {
                if (lag - pl).abs() <= MLAG {
                    continue 'outer;
                }
            }
            picked.push(lag);
            out.push(SyncCandidate {
                freq_hz,
                dt_sec: (lag as f32 - 0.5) * d.tstep,
                score,
            });
            if picked.len() >= 8 {
                break;
            }
        }
        out
    };

    #[cfg(feature = "parallel")]
    let mut cands: Vec<SyncCandidate> = (0..n_freq)
        .into_par_iter()
        .flat_map_iter(fi_cands)
        .collect();

    #[cfg(not(feature = "parallel"))]
    let mut cands: Vec<SyncCandidate> = (0..n_freq).flat_map(fi_cands).collect();

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
    cands.retain(|c| {
        if c.score >= sync_min {
            return true;
        }
        let fi = ((c.freq_hz / d.df).round() as usize).saturating_sub(ia);
        stage1_pass(fi)
    });

    rank_candidates(cands, freq_hint, max_cand)
}

/// A candidate this far (Hz) from `freq_hint` counts as "at the aim
/// point" for [`rank_candidates`]'s priority group.
pub(crate) const FREQ_HINT_NEAR_HZ: f32 = 10.0;

/// Score-rank candidates for output, honouring an optional aim-point
/// hint, and truncate to `max_cand`.
///
/// Without a hint this is a plain best-score-first sort. With one,
/// candidates within [`FREQ_HINT_NEAR_HZ`] of the hint get priority —
/// but only over the first *half* of the `max_cand` budget. The
/// remaining slots are filled by global score order, and any leftover
/// near-aim candidates trail behind that. Within every group the
/// ordering is by score, best first.
///
/// **Why the half-budget reservation** (issue #257): the previous
/// policy was strict lexicographic "near-aim first, score second",
/// which let arbitrarily *weak* candidates near the aim point evict an
/// arbitrarily *strong* one just outside it. That was not hypothetical.
/// `SniperRequest` runs at `sync_min = 0.8` with `max_cand` clamped to
/// 15, and this function's per-bin NMS emits up to 8 lag peaks per
/// frequency bin — so on FT4 (`df` = 5.21 Hz, three bins inside
/// ±10 Hz) the aim point alone could produce well over 15 candidates,
/// most of them noise-floor lags scoring ~1.0. All 15 slots went to
/// them, and a real signal 16-99 Hz away scoring 12-17 was truncated
/// away before it was ever decoded: a blind annulus between the
/// ±12 Hz that `refine_candidate_position` can pull in from the aim
/// point and the ~±100 Hz beyond which the aim-adjacent bins are clean
/// enough noise to fall under `sync_min` and vanish on their own.
/// Reserving rather than monopolising keeps the hint's actual intent —
/// a weak signal at the aim point should not be ranked out by stronger
/// QRM elsewhere in the search window — without the starvation.
pub(crate) fn rank_candidates(
    mut cands: Vec<SyncCandidate>,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    cands.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    let Some(fhint) = freq_hint else {
        cands.truncate(max_cand);
        return cands;
    };

    // `partition` preserves relative order, so both halves stay
    // score-sorted from the sort above.
    let (near, far): (Vec<SyncCandidate>, Vec<SyncCandidate>) = cands
        .into_iter()
        .partition(|c| (c.freq_hz - fhint).abs() <= FREQ_HINT_NEAR_HZ);

    let reserved = near.len().min(max_cand.div_ceil(2));
    let mut out = Vec::with_capacity(max_cand.min(near.len() + far.len()));
    out.extend_from_slice(&near[..reserved]);
    out.extend(far);
    // Leftover near-aim candidates trail the score-ordered remainder
    // rather than being dropped outright — with no competition (`far`
    // empty, e.g. the target really is at the aim point) this degrades
    // to exactly the old behaviour.
    out.extend_from_slice(&near[reserved..]);
    out.truncate(max_cand);
    out
}

// ──────────────────────────────────────────────────────────────────────────
// Fine sync (Costas correlation on downsampled complex baseband)
// ──────────────────────────────────────────────────────────────────────────

// Small fixed-capacity cross-call cache for `make_costas_ref`'s output,
// keyed by content equality on `(pattern, ds_spb)`. 2 slots is enough
// for every pattern combination that exists in this crate today: FT8/
// FT4 reuse one pattern across all their sync blocks (already cached
// *within* one `fine_sync_power_per_block` call, see that function's
// own doc comment), FST4 alternates exactly two (SYNC_A/SYNC_B) — so 2
// slots never thrash for any protocol actually wired here. Perf review:
// `fine_sync_power_per_block` runs once per candidate from three call
// sites (`ft8/decode.rs`, `engine/pipeline.rs`, `msg/pipeline_ap.rs`),
// and the within-call cache above always started empty — every call
// rebuilt each pattern's trig table from scratch even when the
// previous call used the identical `(pattern, ds_spb)` pair. Making
// the cache persist across calls (same idea as #211's FFT-planner
// cache) skips that recomputation; the clone on a cache hit is a
// memcpy of already-computed values, much cheaper than the sin/cos
// calls it replaces. This also fixes `refine_candidate`'s AP/sniper-
// path loop for free: it calls `fine_sync_power` (→ this function) at
// every one of ~83 offsets (`msg/pipeline_ap.rs`'s `REFINE_STEPS`),
// previously rebuilding the same Costas reference from scratch at each
// one despite it never depending on the loop variable — no separate
// fix needed there once this cache exists.
#[cfg(feature = "std")]
type CostasRefCacheEntry = (&'static [u8], usize, Vec<Vec<Complex<f32>>>);

#[cfg(feature = "std")]
std::thread_local! {
    static COSTAS_REF_CACHE: core::cell::RefCell<Vec<CostasRefCacheEntry>> =
        const { core::cell::RefCell::new(Vec::new()) };
}

/// [`make_costas_ref`] with a small cross-call cache — see
/// `COSTAS_REF_CACHE`'s doc comment. `pattern` must be `'static` (true
/// of every real caller: `Protocol::SYNC_MODE.blocks()` entries are all
/// `const`/`static` table data) so the cache can hold a reference to it
/// past this call's return.
#[cfg(feature = "std")]
fn cached_costas_ref(pattern: &'static [u8], ds_spb: usize) -> Vec<Vec<Complex<f32>>> {
    COSTAS_REF_CACHE.with_borrow_mut(|cache| {
        if let Some((_, _, csync)) = cache.iter().find(|(p, d, _)| *p == pattern && *d == ds_spb) {
            return csync.clone();
        }
        let csync = make_costas_ref(pattern, ds_spb);
        if cache.len() >= 2 {
            cache.remove(0);
        }
        cache.push((pattern, ds_spb, csync.clone()));
        csync
    })
}

/// `no_std` (embedded, `fft-extern`) fallback — `thread_local!` needs
/// `std`. Those builds don't reach this hot path today anyway (every
/// `fine_sync_power_per_block` caller is on the host `fft-rustfft`
/// path), so a plain uncached rebuild is fine here, matching
/// `downsample_cached`'s own `no_std` fallback (`engine/dsp/
/// downsample.rs`).
#[cfg(not(feature = "std"))]
fn cached_costas_ref(pattern: &'static [u8], ds_spb: usize) -> Vec<Vec<Complex<f32>>> {
    make_costas_ref(pattern, ds_spb)
}

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
///
/// Caches `make_costas_ref`'s result across consecutive blocks that
/// share the same (content-equal) `pattern` — FT8's 3 sync blocks all
/// use the identical Costas array, so this avoids rebuilding the same
/// `Vec<Vec<Complex<f32>>>` reference waveform 3x per call for no
/// reason. Content equality (not pointer identity) so it's correct for
/// any `Protocol`, not just ones whose blocks happen to share a
/// `&'static` allocation (issue #182 follow-up — same "don't recompute
/// a value that hasn't changed" pattern as `refine_fine.rs`'s Costas
/// reference table, scoped to this smaller, protocol-generic case).
pub fn fine_sync_power_per_block<P: Protocol>(cd0: &[Complex<f32>], i0: i32) -> Vec<f32> {
    type CachedCsync = (&'static [u8], Vec<Vec<Complex<f32>>>);
    let d = SyncDims::of::<P>();
    let blocks = P::SYNC_MODE.blocks();
    let mut out = Vec::with_capacity(blocks.len());
    let mut last: Option<CachedCsync> = None;
    for block in blocks {
        let csync = match &last {
            Some((p, c)) if *p == block.pattern => c,
            _ => {
                last = Some((block.pattern, cached_costas_ref(block.pattern, d.ds_spb)));
                &last.as_ref().unwrap().1
            }
        };
        let start = i0 + (block.start_symbol as usize * d.ds_spb) as i32;
        out.push(score_costas_block(cd0, csync, d.ds_spb, start));
    }
    out
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
