//! FT4-specific coarse-candidate stage — faithful port of WSJT-X
//! `getcandidates4.f90` + `ft4_baseline.f90`.
//!
//! Lives in `engine`, not `ft4`, alongside `engine::sync2d::ft4_sync_search`
//! for the same reason that function does: `engine` is compiled regardless
//! of which protocol features are enabled, and `engine::pipeline` needs to
//! call this unconditionally (gated only by a runtime `P::ID == Ft4`
//! check, matching the existing `ft4_sync_search` wiring) — a
//! `crate::ft4::*` reference from `engine` would force the `ft4` feature
//! on for every build.
//!
//! `engine::sync::coarse_sync` (used by every other protocol still on the
//! generic path) is a 2-D (freq × lag) Costas-array correlation search —
//! correct for FT8/FST4/etc, but a structurally different algorithm from
//! what WSJT-X actually does for FT4. `getcandidates4.f90` never searches
//! a lag/Δt dimension at all: it's a pure frequency-domain periodogram —
//! Nuttall-windowed FFT over *overlapping* `NFFT1 = 4×NSPS`-sample
//! segments stepped by one full symbol (`NSTEP=NSPS`, `ft4_params.f90:14`
//! — not the generic function's `NSTEP_PER_SYMBOL`-based fractional
//! step), time-averaged into `savg`, 15-bin boxcar-smoothed into `savsm`,
//! normalised by a 5-term-polynomial baseline (`ft4_baseline.f90`,
//! already a faithful, unit-tested port at
//! [`crate::engine::baseline::fit_baseline`]), and reduced to **one
//! candidate per local-max frequency peak** (parabolic sub-bin
//! interpolation). FT4's actual Δt determination happens entirely later,
//! in the already-faithful [`crate::engine::sync2d::ft4_sync_search`]
//! (an *absolute* full-window coherent search that ignores whatever
//! `dt_sec` a coarse candidate carries).
//!
//! Because of that, the generic function's up-to-8 lag-distinct
//! candidates per frequency bin are functionally redundant downstream for
//! FT4: each independently pays the full `ft4_sync_search` + LLR + BP +
//! OSD cost in `process_candidate_basic` and — for a real signal —
//! converges on the same refined position and decode outcome. Measured
//! on the WSJT-X FT4 golden WAV (`ft4_diag_candidate_cost_split`,
//! `tests/ft4_sweep.rs`): the generic search emits 2000 candidates across
//! only 440 distinct frequencies (4.5× redundancy), and the dominant
//! per-candidate cost by far is `ft4_sync_search` itself (5.09 s summed,
//! vs 1.46 s inferred LLR+BP+OSD, out of ~6.65 s total) — so a faithful
//! one-candidate-per-frequency port should cut wall-clock roughly by that
//! redundancy factor. See `~/.claude/plans/dapper-soaring-nest.md`.
//!
//! Note: `Ft4::SPECTRUM_WINDOW` is `Rectangular` (`ft4/mod.rs`) with a
//! doc comment explaining *why* — Nuttall was tried on the *generic*
//! `coarse_sync` (paired with its crude 40th-percentile floor, not the
//! real polynomial baseline) and reverted for misranking signal bins
//! against sidelobes on the synth-roundtrip tests. That's the
//! mismatched-pairing trap this module avoids: Nuttall windowing here is
//! paired with its *actual* WSJT-X counterpart, `fit_baseline`, not the
//! generic function's unrelated floor estimator.
//!
//! Deviation from WSJT-X, deliberate: `getcandidates4.f90` collects
//! candidates in frequency-scan order and stops once `maxcand` array
//! slots fill (a Fortran fixed-array convenience, not an intentional
//! ranking), then reorders only by nfqso-proximity. This module instead
//! sorts by score (falling back to `freq_hint` proximity, matching
//! `engine::sync::coarse_sync`'s own convention) before truncating to
//! `max_cand` — consistent with how every other caller in this pipeline
//! (dedup, sniper mode) already treats `SyncCandidate::score` as a
//! genuine ranking, not an FFI-array-fill leftover.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::baseline::fit_baseline;
use super::dsp::downsample::with_default_planner;
use super::sync::{SyncCandidate, nuttall_window, parabolic_peak};

/// `NSPS` (samples/symbol at 12 kHz) — `ft4_params.f90:9`.
const NSPS: usize = 576;
/// `NFFT1 = 4×NSPS` — `ft4_params.f90:13`. Four-symbol-wide analysis
/// window (75% overlap at `NSTEP=NSPS`), giving 4× frequency
/// oversampling vs a single-symbol FFT.
const NFFT1: usize = NSPS * 4;
/// `NH1 = NFFT1/2` — positive-frequency bin count.
const NH1: usize = NFFT1 / 2;
/// `NSTEP=NSPS` — full-symbol coarse step (`ft4_params.f90:14`).
/// Deliberately *not* `Ft4::NSTEP_PER_SYMBOL` (that constant belongs to
/// the generic Costas-lag search this module replaces for FT4).
const NSTEP: usize = NSPS;
/// `df = 12000/NFFT1` — `getcandidates4.f90:27`.
///
/// #323 analysis-grid: receiver resolution, not signal geometry —
/// unlike [`F_OFFSET_HZ`] below, which is `-1.5 ×` the tone spacing.
const DF_HZ: f32 = 12_000.0 / NFFT1 as f32;
/// `f_offset = -1.5*12000/NSPS` — `getcandidates4.f90:51`.
const F_OFFSET_HZ: f32 = -1.5 * 12_000.0 / NSPS as f32;
/// WSJT-X hardcodes these regardless of caller `fa`/`fb`
/// (`getcandidates4.f90:45,47`).
const FREQ_HARD_MIN_HZ: f32 = 200.0;
const FREQ_HARD_MAX_HZ: f32 = 4910.0;

/// Time-averaged linear-power periodogram, length `NH1`. Matches
/// `getcandidates4.f90:29-38`: Nuttall-windowed `NFFT1`-point FFT over
/// `NSTEP`-strided, `NFFT1`-wide (overlapping) segments of raw 12 kHz
/// PCM, averaged across all segments.
fn symbol_spectra_avg(audio: &[i16]) -> Vec<f32> {
    let mut b = Ft4SavgBuilder::new(audio.len());
    b.push(audio);
    b.finish()
}

/// `symbol_spectra_avg` fed a slot at a time instead of all at once.
///
/// **Why**: this stage is the FT4 receiver's largest single
/// non-per-candidate cost — 758 ms of a 3 119 ms slot on a CoreS3, 39 %
/// of the 1 960 ms budget, measured after the internal-DRAM scratch fix
/// (`docs/notes/FT4_BENCHMARK.md` §26, §31.2) — and every one of its
/// ~152 transforms depends only on samples that have already arrived.
/// A receiver can complete each row as its audio lands and have `savg`
/// ready at slot end, so the whole stage leaves the post-slot budget
/// rather than being paid out of it. WSPR already ships exactly this
/// shape (`wspr_app`'s `ddc_loop` -> `DDC_READY_IDX` -> `scan_loop`),
/// and [`engine::sync::SpectrogramBuilder`] is the same idea for
/// FST4's 2-D spectrogram.
///
/// It is also the stage that *cannot* be parallelised instead: it is
/// pure FFT, and the esp-dsp backend serialises every transform behind
/// one global twiddle-table lock (§30).
///
/// [`engine::sync::SpectrogramBuilder`]: crate::engine::sync::SpectrogramBuilder
///
/// **The total length is required up front** and is not a convenience.
/// `getcandidates4.f90` averages exactly `(nz - NFFT1) / NSTEP` rows,
/// which is *not* the same as "every row that fits": for a 90 000-sample
/// slot that is 152, while row 152 would still lie entirely inside the
/// audio. A builder that emitted rows greedily would average 153 and
/// produce a different `savg`.
pub struct Ft4SavgBuilder {
    window: Vec<f32>,
    fft: alloc::boxed::Box<dyn crate::engine::fft::Fft>,
    savg: Vec<f32>,
    buf: Vec<Complex<f32>>,
    /// One row's power spectrum, handed to `push_with_rows`'s callback.
    /// Reused across rows: a waterfall consumer turns it into 240
    /// palette indices immediately and keeps nothing.
    row_pow: Vec<f32>,
    /// Samples from `abs_base` onwards that a future row still needs.
    /// Rows overlap by `NFFT1 - NSTEP`, so this stays bounded by
    /// `NFFT1` plus one push, however long the stream runs.
    hist: Vec<i16>,
    /// Absolute stream index of `hist[0]`.
    abs_base: usize,
    /// Next row to emit.
    j: usize,
    /// Total samples pushed so far; `hist` holds `[abs_base,
    /// stream_len)`.
    stream_len: usize,
    nhsym: usize,
    count: usize,
}

impl Ft4SavgBuilder {
    /// `total_samples` is the slot length the finished `savg` must
    /// match — see the type's doc comment for why it is not optional.
    pub fn new(total_samples: usize) -> Self {
        let nhsym = if total_samples > NFFT1 {
            (total_samples - NFFT1) / NSTEP
        } else {
            0
        };
        Self {
            window: nuttall_window(NFFT1),
            fft: with_default_planner(|planner| planner.plan_forward(NFFT1)),
            savg: vec![0.0f32; NH1],
            buf: vec![Complex::new(0.0f32, 0.0); NFFT1],
            row_pow: vec![0.0f32; NH1],
            hist: Vec::with_capacity(NFFT1 + NSTEP),
            abs_base: 0,
            j: 0,
            stream_len: 0,
            nhsym,
            count: 0,
        }
    }

    /// Feed the next contiguous block of 12 kHz PCM, completing every
    /// row it makes available. Block boundaries do not affect the
    /// result — one push of the whole slot and many small pushes give
    /// the same `savg`, which `ft4_savg_builder_matches_whole_slot`
    /// pins bit-for-bit.
    ///
    /// Rows are read **straight out of the caller's block** whenever
    /// nothing is retained, which is every push in the whole-slot case
    /// and most of them in the streaming one; only a row straddling
    /// two blocks needs the retained tail. The first version of this
    /// buffered unconditionally and compacted with `drain` after every
    /// row, which is quadratic when the caller hands over a whole slot
    /// at once — 152 rows each memmoving what was left of 90 000
    /// samples. Measured on a CoreS3: `ft4_coarse_sync` went 758 ms to
    /// 1 906 ms, i.e. the "streaming" rewrite made the non-streaming
    /// path 2.5x worse before anything had been moved off the budget.
    pub fn push(&mut self, audio: &[i16]) {
        self.push_with_rows(audio, &mut |_| {});
    }

    /// [`push`](Self::push), calling `on_row` with each completed row's
    /// power spectrum (`NH1` bins, `DF_HZ` apart, DC first).
    ///
    /// The rows are computed either way — they are what `savg`
    /// averages — so a waterfall built from them costs no transform of
    /// its own. That is the whole reason this hook exists rather than
    /// a second spectrogram: a receiver that is already paying 152
    /// transforms per slot should not pay 152 more to draw them.
    ///
    /// The slice is the builder's internal buffer and is valid only for
    /// the call; a consumer that wants to keep a row must copy it,
    /// which is what turning it into 240 palette indices does anyway.
    pub fn push_with_rows(&mut self, audio: &[i16], on_row: &mut dyn FnMut(&[f32])) {
        let base = self.stream_len;
        self.stream_len += audio.len();

        if self.hist.is_empty() {
            let Self {
                window,
                fft,
                savg,
                buf,
                row_pow,
                j,
                nhsym,
                count,
                ..
            } = self;
            while *j < *nhsym {
                let ia = *j * NSTEP;
                let Some(lo) = ia.checked_sub(base) else {
                    break;
                };
                if lo + NFFT1 > audio.len() {
                    break;
                }
                Self::row(
                    &audio[lo..lo + NFFT1],
                    window,
                    fft.as_ref(),
                    buf,
                    savg,
                    row_pow,
                    on_row,
                );
                *count += 1;
                *j += 1;
            }
            // Retain only what a later row still needs.
            let keep = (self.j * NSTEP).clamp(base, self.stream_len);
            self.hist.extend_from_slice(&audio[keep - base..]);
            self.abs_base = keep;
            return;
        }

        // A row straddles the previous block and this one.
        self.hist.extend_from_slice(audio);
        let Self {
            window,
            fft,
            savg,
            buf,
            row_pow,
            hist,
            abs_base,
            j,
            nhsym,
            count,
            ..
        } = self;
        while *j < *nhsym {
            let ia = *j * NSTEP;
            let Some(lo) = ia.checked_sub(*abs_base) else {
                break;
            };
            if lo + NFFT1 > hist.len() {
                break;
            }
            Self::row(
                &hist[lo..lo + NFFT1],
                window,
                fft.as_ref(),
                buf,
                savg,
                row_pow,
                on_row,
            );
            *count += 1;
            *j += 1;
        }
        let dead = (*j * NSTEP).saturating_sub(*abs_base).min(hist.len());
        if dead > 0 {
            hist.drain(..dead);
            *abs_base += dead;
        }
    }

    /// One windowed transform, accumulated into `savg`.
    ///
    /// Free-standing over the fields it touches so both arms of
    /// [`push`](Self::push) can call it while holding a borrow of the
    /// samples — which live in the caller's block on one arm and in
    /// `hist` on the other.
    fn row(
        src: &[i16],
        window: &[f32],
        fft: &dyn crate::engine::fft::Fft,
        buf: &mut [Complex<f32>],
        savg: &mut [f32],
        row_pow: &mut [f32],
        on_row: &mut dyn FnMut(&[f32]),
    ) {
        let fac = 1.0f32 / 300.0;
        for k in 0..NFFT1 {
            buf[k] = Complex::new(src[k] as f32 * fac * window[k], 0.0);
        }
        fft.process(buf);
        for i in 0..NH1 {
            let p = buf[i].norm_sqr();
            savg[i] += p;
            row_pow[i] = p;
        }
        on_row(row_pow);
    }

    /// The averaged periodogram. Identical to `symbol_spectra_avg`
    /// over the same samples.
    pub fn finish(mut self) -> Vec<f32> {
        if self.count > 0 {
            let inv = 1.0 / self.count as f32;
            for s in self.savg.iter_mut() {
                *s *= inv;
            }
        }
        self.savg
    }
}

/// FT4 coarse-candidate stage: one candidate per frequency-domain local
/// peak, faithful to `getcandidates4.f90`. Signature matches
/// `engine::sync::coarse_sync::<Ft4>` — a drop-in replacement at call
/// sites.
///
/// `dt_sec` on every returned candidate is `0.0` — genuinely unused
/// downstream: `engine::sync2d::ft4_sync_search` (called next in
/// `process_candidate_basic` for `P::ID == Ft4`) searches the absolute
/// time window regardless of the candidate's own `dt_sec`.
///
/// ## `sync_min` is not a loose knob — 1.2 is the floor, not a ceiling
///
/// WSJT-X passes `syncmin = 1.2` (`ft4_decode.f90:195`), and the number
/// means something specific here: `savsm` is divided by the fitted
/// baseline above, so **noise sits at ~1.0 by construction** and any
/// threshold below that admits every peak in the band. Measured
/// (`tests/ft4_candidate_budget.rs`, 2026-08-30): 0.05 and 1.2 return
/// 67.1 vs 1.6 candidates on average over 560 weak sweep files, and 31
/// vs 12 on the WSJT-X golden — with **identical recall** on both. Since
/// every stage downstream is per-candidate, a caller passing 0.05 pays
/// 2.6× to 42× for nothing.
///
/// The knee is close on the other side: 1.4 loses 2 decodes of 237 and
/// 1.7 loses 63, so this is a value to keep at upstream's rather than to
/// tune.
pub fn ft4_coarse_sync(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let savg = symbol_spectra_avg(audio);
    ft4_coarse_sync_from_savg(&savg, freq_min, freq_max, sync_min, freq_hint, max_cand)
}

/// [`ft4_coarse_sync`]'s second half: everything after the
/// periodogram.
///
/// Split out so a receiver that accumulated `savg` during capture with
/// [`Ft4SavgBuilder`] pays only this part after the slot. On a CoreS3
/// the two halves are ~750 ms and ~10 ms, so which side of slot end
/// the transforms fall on decides 39 % of the decode budget
/// (`docs/notes/FT4_BENCHMARK.md` §32).
///
/// `savg` must be `NH1` long — the periodogram over the whole slot,
/// already averaged.
pub fn ft4_coarse_sync_from_savg(
    savg: &[f32],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    assert_eq!(savg.len(), NH1, "savg must be NH1 bins");

    // 15-bin boxcar smooth (`getcandidates4.f90:39-42`: `savsm(i) =
    // sum(savg(i-7:i+7))/15` for 1-indexed `i` in `[8, NH1-7]`; the
    // 0-indexed equivalent valid range is `[7, NH1-8]`).
    let mut savsm = vec![0.0f32; NH1];
    if NH1 > 14 {
        for i in 7..=(NH1 - 8) {
            savsm[i] = savg[i - 7..=i + 7].iter().sum::<f32>() / 15.0;
        }
    }

    // `nfa`/`nfb`: caller's [freq_min, freq_max] intersected with
    // WSJT-X's own hard 200-4910 Hz bounds (`getcandidates4.f90:44-47`).
    // Clamped to [8, NH1-9] so the smoothing/peak-search neighbour
    // indices (`i-1`, `i+1`) are always in the smoothed range above.
    let nfa = ((freq_min / DF_HZ).round().max(0.0) as usize)
        .max((FREQ_HARD_MIN_HZ / DF_HZ).round() as usize)
        .max(8);
    let nfb = ((freq_max / DF_HZ).round().max(0.0) as usize)
        .min((FREQ_HARD_MAX_HZ / DF_HZ).round() as usize)
        .min(NH1.saturating_sub(9));
    if nfb <= nfa {
        return Vec::new();
    }

    // Baseline fit (`ft4_baseline.f90`, ported at
    // `engine::baseline::fit_baseline`) operates on the raw (unsmoothed)
    // `savg`, same as WSJT-X's `call ft4_baseline(savg,nfa,nfb,sbase)`.
    // `fit_baseline` returns dB (its own documented contract); convert
    // back to linear power to match `ft4_baseline.f90:46`'s
    // `sbase(i)=10**(sbase(i)/10.0)` before dividing `savsm` by it
    // (`getcandidates4.f90:50`).
    let sbase_db = fit_baseline(savg, nfa, nfb);
    for (k, i) in (nfa..=nfb).enumerate() {
        let sbase_lin = 10f32.powf(sbase_db[k] / 10.0);
        savsm[i] = if sbase_lin > f32::EPSILON {
            savsm[i] / sbase_lin
        } else {
            0.0
        };
    }

    // Local-max peak detection with parabolic sub-bin interpolation
    // (`getcandidates4.f90:54-68`).
    let mut out: Vec<SyncCandidate> = Vec::new();
    for i in (nfa + 1)..nfb {
        let y0 = savsm[i];
        if y0 < savsm[i - 1] || y0 < savsm[i + 1] || y0 < sync_min {
            continue;
        }
        let (del, speak) = parabolic_peak(savsm[i - 1], y0, savsm[i + 1]);
        let fpeak = (i as f32 + del) * DF_HZ + F_OFFSET_HZ;
        if !(FREQ_HARD_MIN_HZ..=FREQ_HARD_MAX_HZ).contains(&fpeak) {
            continue;
        }
        out.push(SyncCandidate {
            freq_hz: fpeak,
            dt_sec: 0.0,
            score: speak,
        });
    }

    // Score-sort (freq_hint-priority first), then truncate — see the
    // module-doc "Deviation from WSJT-X" note. Shares
    // `engine::sync::rank_candidates` with the generic `coarse_sync`
    // so both paths get the same non-starving hint policy (issue
    // #257); this port emits one candidate per frequency peak rather
    // than up to 8 lag peaks per bin, so it could not hit #257's
    // annulus as hard, but there is no reason for the two to disagree.
    crate::engine::sync::rank_candidates(out, freq_hint, max_cand)
}

#[cfg(test)]
#[cfg(feature = "fft-rustfft")]
mod tests {
    use super::*;
    extern crate std;
    use std::vec::Vec as StdVec;

    fn tone_slot() -> StdVec<i16> {
        (0..90_000)
            .map(|k| {
                let t = k as f32 / 12_000.0;
                let v = (2.0 * core::f32::consts::PI * 1234.0 * t).sin() * 6000.0
                    + (2.0 * core::f32::consts::PI * 2100.0 * t).sin() * 2500.0;
                v as i16
            })
            .collect()
    }

    /// Chunked accumulation must equal the whole-slot one, bit for bit.
    ///
    /// The point of [`Ft4SavgBuilder`] is that a receiver runs it during
    /// capture, where the chunk size is whatever the audio source hands
    /// over — so "block boundaries do not matter" is the property, not
    /// an implementation detail. Covers chunks that do not divide
    /// `NSTEP`, one either side of it, one larger than `NFFT1`, and a
    /// single whole-slot push.
    #[test]
    fn ft4_savg_builder_matches_whole_slot() {
        let audio = tone_slot();
        let want = symbol_spectra_avg(&audio);
        assert_eq!(want.len(), NH1);
        assert!(want.iter().any(|&v| v > 0.0), "reference savg is all zero");

        for &chunk in &[1usize, 7, 512, 576, 1000, 1024, 2304, 3000, 90_000] {
            let mut b = Ft4SavgBuilder::new(audio.len());
            for c in audio.chunks(chunk) {
                b.push(c);
            }
            let got = b.finish();
            assert_eq!(got.len(), want.len(), "chunk {chunk}: length");
            for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
                assert_eq!(g.to_bits(), w.to_bits(), "chunk {chunk}, bin {i}");
            }
        }
    }

    /// The row hook must emit exactly the rows `savg` averages.
    ///
    /// `push_with_rows` exists so a waterfall can be drawn from the
    /// transforms the coarse stage is already doing. If it emitted a
    /// different set — one short at a block boundary, say — the
    /// waterfall would disagree with the decoder about what was on the
    /// band, which is the one thing a waterfall must not do.
    #[test]
    fn ft4_savg_row_hook_emits_every_averaged_row() {
        let audio = tone_slot();
        let expect_rows = (audio.len() - NFFT1) / NSTEP;

        for &chunk in &[1usize, 577, 1024, 90_000] {
            let mut rows = 0usize;
            let mut acc = vec![0.0f32; NH1];
            let mut b = Ft4SavgBuilder::new(audio.len());
            for c in audio.chunks(chunk) {
                b.push_with_rows(c, &mut |row| {
                    assert_eq!(row.len(), NH1, "chunk {chunk}: row width");
                    for (a, &p) in acc.iter_mut().zip(row.iter()) {
                        *a += p;
                    }
                    rows += 1;
                });
            }
            assert_eq!(rows, expect_rows, "chunk {chunk}: row count");

            // Averaging the emitted rows must reproduce `finish`
            // exactly — same values, same order, same divisor.
            let got = b.finish();
            let inv = 1.0 / rows as f32;
            for (i, (g, a)) in got.iter().zip(acc.iter()).enumerate() {
                assert_eq!(g.to_bits(), (a * inv).to_bits(), "chunk {chunk}, bin {i}");
            }
        }
    }

    /// The split entry point must reproduce the all-in-one one.
    #[test]
    fn ft4_coarse_from_savg_matches_the_combined_call() {
        let audio = tone_slot();
        let want = ft4_coarse_sync(&audio, 100.0, 2700.0, 1.2, None, 100);
        let savg = symbol_spectra_avg(&audio);
        let got = ft4_coarse_sync_from_savg(&savg, 100.0, 2700.0, 1.2, None, 100);

        assert!(!want.is_empty(), "no candidates to compare");
        assert_eq!(want.len(), got.len());
        for (a, b) in want.iter().zip(got.iter()) {
            assert_eq!(a.freq_hz.to_bits(), b.freq_hz.to_bits());
            assert_eq!(a.score.to_bits(), b.score.to_bits());
        }
    }
}
