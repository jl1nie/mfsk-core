// SPDX-License-Identifier: GPL-3.0-only
//! Coarse (frequency × time) sync search for Q65.
//!
//! Q65's distributed sync (22 symbols all on tone 0) is the
//! cleanest correlation target across the WSJT family — every sync
//! symbol carries the full symbol energy on the same frequency bin.
//! The spectrogram build and per-candidate scoring are literally
//! shared with `crate::jt9::search` and `crate::jt65::search` (see
//! [`crate::engine::spectrogram`]), built here at `nsps/8`-symbol time
//! steps (matching WSJT-X's own `NSTEP=8` sync resolution,
//! `lib/qra/q65/q65.f90:3`) instead of their `nsps/4` — only the
//! sync-positions list and the candidate-selection loop below are
//! this protocol's own.

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use num_complex::Complex;

use crate::engine::ModulationParams;
use crate::engine::fft::default_planner;
use crate::engine::source::{PcmSource, Window};
use crate::engine::spectrogram;

use super::sync_pattern::Q65_SYNC_POSITIONS;

/// FFT-bin spectrogram covering the audio buffer at `nsps/8`-symbol
/// time steps. Thin alias — see
/// [`crate::engine::spectrogram::Spectrogram`] for the shared
/// implementation (extracted 2026-08-14, code-sharing audit; was
/// structurally identical to `jt9::search::Spectrogram` /
/// `jt65::search::Spectrogram`, differing only in the time-step
/// divisor — [`NSTEP_PER_SYMBOL`] here vs. their fixed 4).
pub type Spectrogram = spectrogram::Spectrogram;

/// Time step matching WSJT-X's own sync spectrogram resolution
/// (`NSTEP=8`, `lib/qra/q65/q65.f90:3`, "Number of time bins per
/// symbol in s1, s1a, s1b") — an earlier `nsps/2` here under-resolved
/// the sync search 4× relative to `q65_ccf_22`'s own lag search, which
/// was root-caused (verified against real jt9) as the source of a
/// multi-dB AWGN sensitivity gap at `GridDepth::Fast`'s single-shot
/// decode (no further Δt retry to compensate); see
/// `docs/notes/Q65_BENCHMARK.md`.
pub const NSTEP_PER_SYMBOL: usize = 8;

/// Build a spectrogram for Q65 sub-mode `P`. Frequency resolution =
/// `sample_rate / nsps` ≈ tone spacing.
///
/// The previous `Spectrogram::build` Q65-30A convenience wrapper
/// (calling this with `P = Q65a30` hardcoded) had zero callers
/// anywhere in the crate or its tests — dropped rather than carried
/// through this extraction, predating the crate's now-10-wired-
/// sub-mode reality (issue tracking session, 2026-08-14).
pub fn build_spectrogram<P: ModulationParams>(audio: &[f32], sample_rate: u32) -> Spectrogram {
    Spectrogram::build_for::<P>(audio, sample_rate, NSTEP_PER_SYMBOL)
}

pub use crate::engine::search::{DEFAULT_SCORE_THRESHOLD, SearchParams, SyncCandidate};
/// `nsmo`, the number of `smo121` passes `q65_symspec` puts on every
/// symbol spectrum (`q65.f90:96-97,292-295`): `int(0.5 * mode_q65²)`, and
/// none when that is 1 or less. A 0, B 2, C 8, D 32, E 128.
pub(crate) fn nsmo_for<P: ModulationParams>() -> usize {
    let mode_q65 = (P::TONE_SPACING_HZ * P::SYMBOL_DT).round() as usize;
    let nsmo = (0.5 * (mode_q65 * mode_q65) as f32) as usize;
    if nsmo <= 1 { 0 } else { nsmo }
}

/// A copy of `spec` with every time row smoothed [`nsmo_for`] times, the
/// spectra `q65_ccf_22` syncs on (`q65_symspec`). `None` when `nsmo` is 0.
/// `noise_per_bin` is carried over unchanged.
///
/// Only bins `need.0..=need.1` come back smoothed exactly; the rest of
/// each row is whatever the narrowed smoothing left there, and must not be
/// read. `smo121` computes a bin from its neighbours' previous values and
/// leaves a slice's two end bins alone, so smoothing the slice widened by
/// `nsmo` bins each side gives `need` bit for bit what smoothing the whole
/// row does: an end's influence moves one bin inward per pass. The coarse
/// search reads only the sync bins of its window, widened by the drift
/// (`scan_with`). Smoothing the whole 0-6 kHz row was ~20 % of a Q65-120
/// decode on noise, E's `nsmo` being 128 passes (#556).
pub(crate) fn smoothed_for_sync<P: ModulationParams>(
    spec: &Spectrogram,
    need: (usize, usize),
) -> Option<Spectrogram> {
    let nsmo = nsmo_for::<P>();
    if nsmo == 0 || spec.n_freq == 0 {
        return None;
    }
    let lo = need.0.saturating_sub(nsmo);
    let hi = need.1.saturating_add(nsmo).min(spec.n_freq - 1);
    let mut mags_sqr = spec.mags_sqr.clone();
    if lo <= hi {
        for row in mags_sqr.chunks_exact_mut(spec.n_freq) {
            let part = &mut row[lo..=hi];
            for _ in 0..nsmo {
                super::q3::smo121(part);
            }
        }
    }
    Some(Spectrogram {
        mags_sqr,
        n_time: spec.n_time,
        n_freq: spec.n_freq,
        t_step: spec.t_step,
        nsps: spec.nsps,
        df: spec.df,
        noise_per_bin: spec.noise_per_bin,
    })
}

use crate::engine::search::{SearchWindow, best_lag_in_bin};

/// Q65's own coarse-search defaults.
///
/// Default Q65 dial range: 200 Hz .. 3000 Hz inside the
/// SSB passband. Callers can narrow this further.
///
/// The time window is `q65.f90:127-128`'s `lag1=-1.0/dtstep`,
/// `lag2=1.0/dtstep`: -1.0 .. +1.0 s around the nominal start for every
/// sub-mode, which is what WSJT-X's GUI searches unless its EME delay
/// ("Decode at 52 s") is set (`mainwindow.cpp:5270-5271`,
/// `emedelay=0.0`). With it set, `lag2` reaches +5.5 s (`nsps >= 3600`)
/// or +4.0 s (Q65-15) — [`eme_delay_late_sec`], which the Q65 requests'
/// `.eme_delay(true)` applies.
///
/// History: this default was +5.5 s late for every sub-mode from #282
/// until 0.12.0, set from measurements of the `jt9` CLI, which turns the
/// EME delay on for TR 60 s by itself (`jt9_params_init.f90`
/// `apply_per_mode_policy`, `emedelay = 2.5` for Q65-60). That is why
/// Q65-60A reached +5.5 s there and Q65-30A did not — `lag2`'s
/// `nsps >= 3600` covers both; `emedelay` was the difference. Before
/// that it was `time_tolerance_symbols: 5` (±0.75 s on Q65-15).
///
/// `score_threshold` is not read by the Q65 search (#552). Admission is
/// `q65_ccf_22`'s own relative test, see [`coarse_search_drift_on_spec_for`].
pub const fn default_search_params() -> SearchParams {
    SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 1.0,
        time_tolerance_late_sec: 1.0,
        score_threshold: DEFAULT_SCORE_THRESHOLD,
        max_candidates: 8,
    }
}

/// How late the search reaches with WSJT-X's EME delay on
/// (`emedelay > 0`): `q65.f90:129-130`, `lag2=5.5/dtstep` when
/// `nsps >= 3600`, and `lag2=4/dtstep` for Q65-15 (`ntrperiod.eq.15`,
/// `nsps >= 900`). The early edge stays at -1.0 s.
pub fn eme_delay_late_sec<P: ModulationParams + crate::engine::FrameLayout>() -> f32 {
    let mut late = 1.0;
    if P::NSPS >= 3600 {
        late = 5.5;
    }
    if P::T_SLOT_S == 15.0 && P::NSPS >= 900 {
        late = 4.0;
    }
    late
}

/// Score one `(start_row, base_bin)`: sum tone-0 power across the
/// 22 sync positions, divide by `(sum + noise_floor)`.
///
/// Not what the coarse search ranks by since #552: that is
/// `q65_ccf_22`'s mean-subtracted sum, see
/// [`coarse_search_drift_on_spec_for`].
pub fn score_candidate(spec: &Spectrogram, start_row: usize, base_bin: usize) -> f32 {
    let rows_per_symbol = (spec.nsps / spec.t_step).max(1);
    spectrogram::score_candidate(
        spec,
        start_row,
        base_bin,
        &Q65_SYNC_POSITIONS,
        rows_per_symbol,
    )
}

/// Build a spectrogram for Q65 sub-mode `P` and find the top sync
/// candidates inside the search window.
pub fn coarse_search_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    let spec = build_spectrogram::<P>(audio, sample_rate);
    coarse_search_on_spec_for::<P>(&spec, sample_rate, nominal_start_sample, params)
}

/// [`coarse_search_for`] with the Max Drift search — see
/// [`coarse_search_drift_on_spec_for`].
pub fn coarse_search_drift_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    max_drift: u32,
) -> Vec<(SyncCandidate, i32)> {
    let spec = build_spectrogram::<P>(audio, sample_rate);
    coarse_search_drift_on_spec_for::<P>(
        &spec,
        sample_rate,
        nominal_start_sample,
        params,
        max_drift,
    )
}

/// Q65-30A convenience wrapper for [`coarse_search_for`].
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_for::<super::Q65a30>(audio, sample_rate, nominal_start_sample, params)
}

/// Same as [`coarse_search_for`] but accepts a pre-built spectrogram
/// — useful when the same audio is scanned under multiple parameter
/// sets.
pub fn coarse_search_on_spec_for<P: ModulationParams>(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_drift_on_spec_for::<P>(spec, sample_rate, nominal_start_sample, params, 0)
        .into_iter()
        .map(|(c, _)| c)
        .collect()
}

/// What the per-bin sync curve reads of a spectrogram: the power of a cell, and the mean of a column over **all** rows. The
/// whole spectrogram gives both from memory ([`WholeColumns`]); a band of columns built a band at a time gives them for its
/// columns only ([`BandColumns`]), which is all the curve of those bins reads.
trait SyncColumns {
    fn n_time(&self) -> usize;
    fn n_freq(&self) -> usize;
    fn get(&self, row: usize, bin: usize) -> f32;
    fn sync_mean(&self, bin: usize) -> f32;
}

/// `q65_ccf_22`'s curve (`q65.f90:492-494,503-516`) needs, per bin, the 22 sync symbols' power minus `(22.0/jz)*s1avg(i)`,
/// what 22 rows of that bin hold on average over the whole spectrogram. `col` holds the column sums over every row.
fn sync_means(mut col: Vec<f32>, n_time: usize) -> Vec<f32> {
    let k = Q65_SYNC_POSITIONS.len() as f32 / n_time as f32;
    col.iter_mut().for_each(|c| *c *= k);
    col
}

struct WholeColumns<'a> {
    spec: &'a Spectrogram,
    sync_mean: Vec<f32>,
}

impl<'a> WholeColumns<'a> {
    fn new(spec: &'a Spectrogram) -> Self {
        let mut col = vec![0.0_f32; spec.n_freq];
        for row in spec.mags_sqr.chunks_exact(spec.n_freq) {
            for (c, &v) in col.iter_mut().zip(row) {
                *c += v;
            }
        }
        Self {
            spec,
            sync_mean: sync_means(col, spec.n_time),
        }
    }
}

impl SyncColumns for WholeColumns<'_> {
    fn n_time(&self) -> usize {
        self.spec.n_time
    }
    fn n_freq(&self) -> usize {
        self.spec.n_freq
    }
    fn get(&self, row: usize, bin: usize) -> f32 {
        self.spec.get(row, bin)
    }
    fn sync_mean(&self, bin: usize) -> f32 {
        self.sync_mean[bin]
    }
}

/// Bins `lo..lo + width` of a spectrogram of `n_freq` bins, every row of them (row-major, `width` to a row), and the mean of
/// each column over all rows.
struct BandColumns {
    lo: usize,
    width: usize,
    n_time: usize,
    n_freq: usize,
    power: Vec<f32>,
    sync_mean: Vec<f32>,
}

impl SyncColumns for BandColumns {
    fn n_time(&self) -> usize {
        self.n_time
    }
    fn n_freq(&self) -> usize {
        self.n_freq
    }
    fn get(&self, row: usize, bin: usize) -> f32 {
        self.power[row * self.width + (bin - self.lo)]
    }
    fn sync_mean(&self, bin: usize) -> f32 {
        self.sync_mean[bin - self.lo]
    }
}

/// Sync power over the 22 sync symbols with the tone drifting
/// `idrift` bins across the frame — `q65_ccf_22`'s inner sum
/// (`q65.f90:506-516`): symbol `k` (1-based) is read at bin
/// `i + nint(idrift*(k-43)/85.0)`, and a bin off the spectrum drops
/// that term. `idrift = 0` is [`score_candidate`]'s sum.
fn drifted_sync_power<C: SyncColumns>(
    cols: &C,
    start_row: usize,
    base_bin: usize,
    idrift: i32,
    rows_per_symbol: usize,
) -> f32 {
    let mut pwr = 0.0f32;
    for &sym in Q65_SYNC_POSITIONS.iter() {
        let k = sym as f32 + 1.0;
        let off = (idrift as f32 * (k - 43.0) / 85.0).round() as i64;
        let bin = base_bin as i64 + off;
        if bin < 0 || bin as usize >= cols.n_freq() {
            continue;
        }
        pwr += cols.get(start_row + sym as usize * rows_per_symbol, bin as usize);
    }
    pwr
}

/// The best `(row, score, drift)` of frequency bin `fb`: over the lags of `w`, and over a tone drift of `-max_drift..=max_drift`
/// bins across the frame. `None` when the bin leaves no room for the 64 data tones above it, or no lag fits.
fn best_in_bin<C: SyncColumns>(
    cols: &C,
    w: &SearchWindow,
    fb: usize,
    rows_per_symbol: usize,
    bins_per_tone: usize,
    max_drift: i32,
) -> Option<(usize, f32, i32)> {
    // Tone 64 (highest data tone) sits at base_bin + 64 * bins_per_tone for the active sub-mode.
    if fb + 64 * bins_per_tone + 1 > cols.n_freq() {
        return None;
    }
    // Need room for the last data symbol (84) + the 64 data tones above the sync bin.
    let row_fits = |row: usize| row + 84 * rows_per_symbol < cols.n_time();
    let ccf = |row: usize, bin: usize, idrift: i32| {
        drifted_sync_power(cols, row, bin, idrift, rows_per_symbol) - cols.sync_mean(bin)
    };
    if max_drift == 0 {
        best_lag_in_bin(w, fb, row_fits, |row, bin| ccf(row, bin, 0))
            .map(|(row, score)| (row, score, 0))
    } else {
        // `do lag=lag1,lag2; do idrift=-max_drift,max_drift`, keeping the first maximum as `ccft.gt.ccfmax` does.
        let mut best: Option<(usize, f32, i32)> = None;
        for idrift in -max_drift..=max_drift {
            if let Some((row, score)) =
                best_lag_in_bin(w, fb, row_fits, |row, bin| ccf(row, bin, idrift))
                && best.is_none_or(|(_, b, _)| score > b)
            {
                best = Some((row, score, idrift));
            }
        }
        best
    }
}

/// The per-frequency curve's candidate selection (`q65.f90:553-574`): what is left of a coarse search once every bin's best
/// `(row, score, drift)` is known (`curve`, `rows`, `drifts`, from bin `fb_lo`).
#[allow(clippy::too_many_arguments)]
fn select_candidates(
    curve: &[f32],
    rows: &[usize],
    drifts: &[i32],
    fb_lo: usize,
    t_step: usize,
    df: f32,
    bins_per_tone: usize,
    max_candidates: usize,
) -> Vec<(SyncCandidate, i32)> {
    // Noise-adaptive admission threshold — `q65_ccf_22`'s own
    // candidate-selection logic (`lib/qra/q65/q65.f90:553-574`):
    // `ave` = 50th percentile, `base` = 84th percentile of the whole
    // per-frequency score curve (≈ mean+1σ for a roughly-Gaussian
    // noise floor), `rms = base - ave`, admit only candidates with
    // `(score-ave)/rms >= 6.0`.
    let ave = percentile(curve, 50);
    let base = percentile(curve, 84);
    let rms = base - ave;
    let use_adaptive = rms.is_finite() && rms > 1e-6;
    const SNR_ADMIT: f32 = 6.0;

    // Frequency-domain local-max suppression — `i3=i-mode_q65,
    // i4=i+mode_q65; if(ccf2(i).ne.biggest) cycle`
    // (`lib/qra/q65/q65.f90:563-566`) — `mode_q65` there is exactly
    // our `bins_per_tone` (`nBinsPerTone = 1<<submode`, `q65.c:351`).
    //
    // The curve's highest point is admitted whatever its SNR. Upstream
    // always tries the best sync within `nfqso ± ntol` (`q65_dec0`'s
    // `ibest`, `q65.f90:528-534`) before its SNR-gated candidate list.
    // DIVERGENCE: this crate has no Rx frequency in a plain scan, so it takes
    // the best over the whole window instead. Without it, the relative test
    // alone loses weak spread signals whose sync peak sits under 6: Q65-120E
    // at 20 Hz Doppler spread decoded 0/10 at -26 dB where it had 9/10
    // (60-trial cells, #552).
    //
    // This replaces a fixed floor, `score >= params.score_threshold`
    // (0.1), OR'd in beside the relative test. On the old ratio every bin
    // scored about 0.5, so the floor admitted all of them, and every scan
    // decoded `max_candidates` = 8 candidates, noise or not. On a noise-only
    // Q65-60D frame that meant 8 grid decodes and 56 BP runs per scan
    // against 0 now (#552).
    let best_idx = curve
        .iter()
        .enumerate()
        .fold(None, |acc: Option<(usize, f32)>, (i, &v)| match acc {
            Some((_, b)) if b >= v => acc,
            _ => Some((i, v)),
        })
        .map(|(i, _)| i);
    let mut out: Vec<(SyncCandidate, i32)> = Vec::new();
    for (idx, &score) in curve.iter().enumerate() {
        if score <= 0.0 {
            continue;
        }
        let admitted = Some(idx) == best_idx || (use_adaptive && (score - ave) / rms >= SNR_ADMIT);
        if !admitted {
            continue;
        }
        let lo = idx.saturating_sub(bins_per_tone);
        let hi = (idx + bins_per_tone).min(curve.len() - 1);
        let is_local_max = curve[lo..=hi].iter().all(|&other| other <= score);
        if !is_local_max {
            continue;
        }
        out.push((
            SyncCandidate {
                start_sample: rows[idx] * t_step,
                freq_hz: (fb_lo + idx) as f32 * df,
                score,
            },
            drifts[idx],
        ));
    }
    // `rank_and_truncate`'s order, carrying each candidate's drift.
    out.sort_unstable_by(|a, b| {
        b.0.score
            .partial_cmp(&a.0.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    out.truncate(max_candidates);
    out
}

/// [`coarse_search_on_spec_for`] with WSJT-X's **Max Drift** search:
/// every frequency bin keeps the best score over lag *and* over a tone
/// drift of `-max_drift..=max_drift` bins across the frame
/// (`q65_ccf_22`, `do idrift=-max_drift,max_drift`; the GUI's Max Drift,
/// 0..50). Each candidate comes back with the drift, in bins, that
/// scored it. `max_drift = 0` is the plain search.
///
/// Upstream also narrows the window to `nfqso ± ntol` when the drift
/// search is on (`q65.f90:486-489`), because it costs `2*max_drift+1`
/// times the plain search; here the caller's window is the window, so
/// narrow it to match.
pub fn coarse_search_drift_on_spec_for<P: ModulationParams>(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    max_drift: u32,
) -> Vec<(SyncCandidate, i32)> {
    if spec.n_time == 0 {
        return Vec::new();
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let w = SearchWindow::new(spec.t_step, sample_rate, nominal_start_sample, nsps, params);
    let df = w.df;
    let rows_per_symbol = (nsps / spec.t_step.max(1)).max(1);
    // For wider sub-modes (B/C/D/E) the highest data tone sits
    // 64 × bins_per_tone above the sync bin instead of just 64
    // bins; we need that much headroom in the spectrogram before
    // we will accept a candidate base bin.
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round() as usize;

    // Collapse over time first, per frequency bin — mirrors
    // `q65_ccf_22`'s own structure (`lib/qra/q65/q65.f90:506-538`):
    // for each frequency it keeps only the single best-scoring lag
    // (`ccfmax = max over lag,idrift`), then ranks candidates across
    // frequencies from that already-time-collapsed curve. Emitting
    // one `(row, freq)` candidate per cell instead — as an earlier
    // version of this function did — let a finer time step (`t_step`
    // was widened 4× here to match WSJT-X's `NSTEP=8`) flood the
    // `max_candidates`-truncated list with near-duplicate rows all
    // describing the same true peak's neighbourhood, crowding out
    // distinct weaker signals (regression caught by
    // `ionoscatter_6m_120e_decodes_with_fading_metric`, a real-off-air
    // multi-signal recording).
    let fb_lo = w.fmin_bin.max(0) as usize;
    let fb_hi = w.fmax_bin.max(w.fmin_bin) as usize;
    let mut curve: Vec<f32> = vec![0.0; fb_hi.saturating_sub(fb_lo) + 1];
    let mut rows: Vec<usize> = vec![0; curve.len()];
    let mut drifts: Vec<i32> = vec![0; curve.len()];
    let cols = WholeColumns::new(spec);
    for fb in fb_lo..=fb_hi {
        if let Some((row, score, idrift)) = best_in_bin(
            &cols,
            &w,
            fb,
            rows_per_symbol,
            bins_per_tone,
            max_drift as i32,
        ) {
            let idx = fb - fb_lo;
            curve[idx] = score;
            rows[idx] = row;
            drifts[idx] = idrift;
        }
    }
    select_candidates(
        &curve,
        &rows,
        &drifts,
        fb_lo,
        spec.t_step,
        df,
        bins_per_tone,
        params.max_candidates,
    )
}

/// [`coarse_search_drift_on_spec_for`] from a [`PcmSource`] read forward, **without building the spectrogram**: the audio is
/// read in `bands` forward sweeps, and each sweep keeps only the columns (frequency bins) of one band, so what is held is
/// `n_time x (band + 2 * max_drift)` `f32`, not the whole spectrogram.
///
/// The sync curve of a bin reads that bin's column (and, with a drift search, the columns `max_drift` bins either side) over
/// all rows, and nothing of any other bin: the bins are independent until the candidates are chosen from the whole curve. So the
/// curve is made band by band, from the same frames, the same FFT and the same sums in the same order as the whole-spectrogram
/// search, and the candidates are **bit for bit** that search's. For Q65-30A at 12 kHz (793 rows of 1 800 bins, 5.7 MB whole; the
/// columns of 200-3 000 Hz are 2.7 MB) 16 bands hold about 170 KB. The price is `bands` times the FFTs (793 FFTs of `nsps`
/// points a sweep) and `bands` times the reads of the audio (a restart for each, on a [`CountingSource`](crate::engine::source::CountingSource)).
///
/// A frame is `nsps` samples: its FFT buffer (8 bytes a sample) is the floor of the memory, 330 KB at Q65-300A, whatever
/// `bands` is. `bands` is clamped to `1..=` the number of columns. `src` supplies `i16`, read as `i16 / 32768`.
pub fn coarse_search_streamed<P: ModulationParams, A: PcmSource>(
    src: &mut A,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    max_drift: u32,
    bands: usize,
) -> Vec<(SyncCandidate, i32)> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let t_step = (nsps / NSTEP_PER_SYMBOL).max(1);
    // the whole-spectrogram builder gives an empty spectrogram (no candidates) for audio shorter than a frame
    if nsps == 0 || src.len() < nsps {
        return Vec::new();
    }
    let n_time = (src.len() - nsps) / t_step + 1;
    let n_freq = nsps / 2;
    let w = SearchWindow::new(t_step, sample_rate, nominal_start_sample, nsps, params);
    let df = w.df;
    let rows_per_symbol = (nsps / t_step).max(1);
    let bins_per_tone = (P::TONE_SPACING_HZ / df).round() as usize;

    let fb_lo = w.fmin_bin.max(0) as usize;
    let fb_hi = w.fmax_bin.max(w.fmin_bin) as usize;
    let mut curve: Vec<f32> = vec![0.0; fb_hi.saturating_sub(fb_lo) + 1];
    let mut rows: Vec<usize> = vec![0; curve.len()];
    let mut drifts: Vec<i32> = vec![0; curve.len()];

    // the bins that have room for the 64 data tones above them: `best_in_bin` gives the others nothing
    let max_drift_bins = max_drift as usize;
    if let Some(fb_top) = n_freq.checked_sub(64 * bins_per_tone + 1) {
        let last = fb_hi.min(fb_top);
        if fb_lo <= last {
            let n_cols = last - fb_lo + 1;
            let n_bands = bands.clamp(1, n_cols);
            let fft = default_planner().plan_forward(nsps);
            let mut raw = vec![0_i16; nsps];
            let mut buf: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); nsps];
            for band in 0..n_bands {
                let a = fb_lo + band * n_cols / n_bands;
                let z = fb_lo + (band + 1) * n_cols / n_bands - 1;
                // the columns a drift search reads: `max_drift` bins either side of the band's own
                let lo = a.saturating_sub(max_drift_bins);
                let hi = (z + max_drift_bins).min(n_freq - 1);
                let width = hi - lo + 1;

                let mut power = vec![0.0_f32; n_time * width];
                let mut col = vec![0.0_f32; width];
                let mut window = Window::new(src);
                for t in 0..n_time {
                    window.read_into((t * t_step) as i64, &mut raw);
                    for (slot, &v) in buf.iter_mut().zip(&raw) {
                        *slot = Complex::new(f32::from(v) / 32_768.0, 0.0);
                    }
                    fft.process(&mut buf);
                    let row = &mut power[t * width..(t + 1) * width];
                    for ((slot, c), sum) in row.iter_mut().zip(&buf[lo..=hi]).zip(col.iter_mut()) {
                        let p = c.norm_sqr();
                        *slot = p;
                        *sum += p;
                    }
                }
                let cols = BandColumns {
                    lo,
                    width,
                    n_time,
                    n_freq,
                    power,
                    sync_mean: sync_means(col, n_time),
                };
                for fb in a..=z {
                    if let Some((row, score, idrift)) = best_in_bin(
                        &cols,
                        &w,
                        fb,
                        rows_per_symbol,
                        bins_per_tone,
                        max_drift as i32,
                    ) {
                        let idx = fb - fb_lo;
                        curve[idx] = score;
                        rows[idx] = row;
                        drifts[idx] = idrift;
                    }
                }
            }
        }
    }
    select_candidates(
        &curve,
        &rows,
        &drifts,
        fb_lo,
        t_step,
        df,
        bins_per_tone,
        params.max_candidates,
    )
}

/// Nearest-rank percentile — matches WSJT-X's own `pctile`
/// (`lib/pctile.f90`): sort ascending, take element at
/// `round(n * pct/100)` (1-indexed, clamped to `[1, n]`).
pub(super) fn percentile(values: &[f32], pct: u32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f32> = values.to_vec();
    sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let n = sorted.len();
    let j = ((n as f32 * 0.01 * pct as f32).round() as usize)
        .max(1)
        .min(n);
    sorted[j - 1]
}

/// Q65-30A convenience wrapper for [`coarse_search_on_spec_for`].
pub fn coarse_search_on_spec(
    spec: &Spectrogram,
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    coarse_search_on_spec_for::<super::Q65a30>(spec, sample_rate, nominal_start_sample, params)
}

#[cfg(test)]
mod tests {
    use super::super::tx::synthesize_standard;
    use super::*;

    /// The narrowed smoothing equals whole-row smoothing, bit for bit, on
    /// the bins it promises (#556), for E's 128 passes and B's 2, with the
    /// window at a row's edge and in its middle.
    #[test]
    fn narrowed_sync_smoothing_is_exact_where_read() {
        let n_freq = 900;
        let n_time = 3;
        let mut x = 0x1234_5678_u32;
        let mags_sqr: Vec<f32> = (0..n_freq * n_time)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x % 10_000) as f32 / 37.0
            })
            .collect();
        let spec = Spectrogram {
            mags_sqr,
            n_time,
            n_freq,
            t_step: 1,
            nsps: 1,
            df: 1.0,
            noise_per_bin: 1.0,
        };
        fn check<P: ModulationParams>(spec: &Spectrogram, need: (usize, usize)) {
            let full = smoothed_for_sync::<P>(spec, (0, spec.n_freq - 1)).unwrap();
            let part = smoothed_for_sync::<P>(spec, need).unwrap();
            for t in 0..spec.n_time {
                for f in need.0..=need.1 {
                    let i = t * spec.n_freq + f;
                    assert_eq!(
                        full.mags_sqr[i].to_bits(),
                        part.mags_sqr[i].to_bits(),
                        "t {t} f {f}"
                    );
                }
            }
        }
        for need in [(0, 40), (300, 520), (860, 899)] {
            check::<super::super::Q65e120>(&spec, need);
            check::<super::super::Q65b60>(&spec, need);
        }
    }

    #[test]
    fn coarse_search_finds_clean_signal() {
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let cands = coarse_search(&audio, 12_000, 0, &default_search_params());
        assert!(!cands.is_empty(), "search should find a clean signal");
        let best = cands[0];
        // Frequency bin width is 12000/3600 ≈ 3.33 Hz, so ±4 Hz is
        // within one bin tolerance.
        assert!(
            (best.freq_hz - freq).abs() <= 4.0,
            "best freq {} should be near {freq} Hz",
            best.freq_hz
        );
        assert_eq!(best.start_sample, 0, "clean synth starts at sample 0");
        // The curve is `q65_ccf_22`'s mean-subtracted sync sum: positive
        // where the sync rows hold more than the bin's average.
        assert!(
            best.score > 0.0,
            "clean signal should score above its bin's mean"
        );
    }

    /// The streamed coarse search against the whole-spectrogram one.
    mod streamed {
        use super::super::*;
        use crate::engine::source::{CountingSource, SliceSource};
        use crate::q65::{Q65a15, synthesize_standard_for};

        fn gauss_noise(n: usize, seed: u64) -> Vec<f32> {
            let mut s = seed.wrapping_add(1);
            let mut next = || {
                s = s
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                ((s >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
            };
            let mut out = Vec::with_capacity(n);
            while out.len() < n {
                let (u, v) = (next(), next());
                let m = (-2.0 * u.ln()).sqrt();
                out.push(m * (core::f32::consts::TAU * v).cos());
                out.push(m * (core::f32::consts::TAU * v).sin());
            }
            out.truncate(n);
            out
        }

        /// A Q65-15A slot (15 s at 12 kHz) with two signals of different strength and some noise, as i16 and as the f32 a
        /// whole spectrogram is built from (`i16 / 32768`: what the streamed search reads).
        fn slot() -> (Vec<i16>, Vec<f32>) {
            let a =
                synthesize_standard_for::<Q65a15>("K1ABC", "JA1ABC", "PM95", 12_000, 1_500.0, 0.3)
                    .unwrap();
            let b =
                synthesize_standard_for::<Q65a15>("W9XYZ", "JH1ABC", "EN52", 12_000, 1_590.0, 0.15)
                    .unwrap();
            let nz = gauss_noise(a.len(), 5);
            let raw: Vec<i16> = a
                .iter()
                .zip(&b)
                .zip(&nz)
                .map(|((x, y), n)| {
                    ((x + y + n * 0.5) * 0.15 * 32_768.0)
                        .round()
                        .clamp(-32_768.0, 32_767.0) as i16
                })
                .collect();
            let f = raw.iter().map(|&v| f32::from(v) / 32_768.0).collect();
            (raw, f)
        }

        fn params() -> SearchParams {
            SearchParams {
                freq_min_hz: 1_440.0,
                freq_max_hz: 1_620.0,
                time_tolerance_early_sec: 1.0,
                time_tolerance_late_sec: 1.0,
                score_threshold: 0.0,
                max_candidates: 16,
            }
        }

        fn bits(v: &[(SyncCandidate, i32)]) -> Vec<(usize, u32, u32, i32)> {
            v.iter()
                .map(|(c, d)| (c.start_sample, c.freq_hz.to_bits(), c.score.to_bits(), *d))
                .collect()
        }

        /// The streamed search is the whole-spectrogram search, to the bit (start, frequency, score, drift), for any number
        /// of bands (one, several, more than there are columns) and with a drift search or without.
        #[test]
        fn the_streamed_search_is_the_whole_spectrogram_search() {
            let (raw, f) = slot();
            let spec = build_spectrogram::<Q65a15>(&f, 12_000);
            for max_drift in [0u32, 3] {
                let want = coarse_search_drift_on_spec_for::<Q65a15>(
                    &spec,
                    12_000,
                    0,
                    &params(),
                    max_drift,
                );
                assert!(
                    !want.is_empty(),
                    "the search finds the signal (the several-candidate case is the real recordings')"
                );
                for bands in [1usize, 5, 10_000] {
                    let got = coarse_search_streamed::<Q65a15, _>(
                        &mut SliceSource(&raw[..]),
                        12_000,
                        0,
                        &params(),
                        max_drift,
                        bands,
                    );
                    assert_eq!(bits(&want), bits(&got), "drift {max_drift}, {bands} bands");
                }
            }
        }

        /// What a sweep costs: every band reads the audio once, forward (a restart behind the position for each band after
        /// the first, none skipped), and nothing is read twice within a sweep (the frames overlap by 7/8: a cache serves
        /// them).
        #[test]
        fn each_band_is_one_forward_sweep_of_the_audio() {
            let (raw, _) = slot();
            let nsps = 1_800usize;
            let t_step = nsps / NSTEP_PER_SYMBOL;
            let n_time = (raw.len() - nsps) / t_step + 1;
            let last_end = (n_time - 1) * t_step + nsps;
            for bands in [1u64, 4] {
                let mut src = CountingSource::new(SliceSource(&raw[..]));
                let _ = coarse_search_streamed::<Q65a15, _>(
                    &mut src,
                    12_000,
                    0,
                    &params(),
                    0,
                    bands as usize,
                );
                let c = src.cost();
                assert_eq!(c.restarts, bands - 1, "{c:?}");
                assert_eq!(c.samples_skipped, 0, "{c:?}");
                assert!(
                    c.samples_read >= bands * last_end as u64
                        && c.samples_read <= bands * raw.len() as u64,
                    "{bands} bands: {} samples read of {}, a sweep covers the {last_end} the frames reach: {c:?}",
                    c.samples_read,
                    raw.len()
                );
            }
        }

        /// More bands than columns is one column a band, not a sweep (an FFT of every frame) for each band asked for: asking for
        /// 10 000 bands costs the columns' worth of sweeps, about 27 here, and gives the same candidates.
        #[test]
        fn more_bands_than_columns_is_one_sweep_a_column() {
            let (raw, _) = slot();
            let mut src = CountingSource::new(SliceSource(&raw[..]));
            let _ = coarse_search_streamed::<Q65a15, _>(&mut src, 12_000, 0, &params(), 0, 10_000);
            let sweeps = src.cost().restarts + 1;
            assert!(
                (20..=40).contains(&sweeps),
                "{sweeps} sweeps for 180 Hz of 6.7 Hz columns: not about one a column"
            );
        }

        /// Audio shorter than a frame has no spectrogram and no candidates, as for the whole one.
        #[test]
        fn audio_shorter_than_a_frame_gives_nothing() {
            let (raw, _) = slot();
            let short = &raw[..1_799];
            let got = coarse_search_streamed::<Q65a15, _>(
                &mut SliceSource(short),
                12_000,
                0,
                &params(),
                0,
                4,
            );
            assert!(got.is_empty());
        }
    }
}
