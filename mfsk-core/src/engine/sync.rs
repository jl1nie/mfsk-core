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
/// **Feed this a candidate list from a lag window comparable to the
/// timing error you are estimating** — ±1 s is what the ±70 ms figure
/// above was measured at, via
/// [`coarse_sync_with_lag`](crate::ft8::decode_block::coarse_sync_with_lag).
/// It is *not* valid over a list built at FT8's default (WSJT-X's own
/// ±2.5 s) window: there a strong signal's far-lag ghost can outscore
/// its own true-lag peak and crowd the top ranks, and the top-5 DT
/// median on `qso3_busy.wav` lands 1.9 s away from truth. That is not
/// a defect in this crate's coarse sync — probing real `jt9`'s
/// `sync8` on the same recording shows the identical ghosts at the
/// identical frequencies (e.g. 2534.38 Hz scoring higher at
/// `xdt=+2.38` than at its true `+0.14`); WSJT-X simply never uses a
/// top-K statistic over that list, it calls `ft8b` on every entry.
/// See issue #280.
///
/// `cands` does not need to be sorted. Returns `None` if `cands` is
/// empty or `top_k == 0`.
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
    /// `sample_rate_hz` is the rate `audio` fed to
    /// [`compute_spectra`]/[`coarse_sync`] is actually at — the
    /// "analysis-grid" rate issue #323 classified as needing to vary
    /// for a future DDC front end (#309), separate from
    /// `downsample_cached`'s own rate below.
    ///
    /// `nfft1`/`nstep`/`nsps`/`nmax`/`nhsym`/`nh1`/`df`/`tstep`/`jstrt`/
    /// `jz` (all derived from `sample_rate_hz`) are this rate's fields.
    /// **`ds_spb`/`ds_rate` are not** — those describe
    /// `downsample_cached`'s separate per-candidate baseband pipeline,
    /// which starts from the canonical 12 kHz raw ingest via
    /// `DownsampleCfg::input_rate` regardless of what rate `coarse_sync`
    /// searches at (the two are independent pipeline stages that
    /// happen to coincide at 12 kHz today). Left untouched here —
    /// #323 already routed their three other redundant copies through
    /// `cfg.input_rate` directly; this one stays `P::NSPS`-derived
    /// until a caller actually needs `downsample_cached` itself fed by
    /// something other than the canonical 12 kHz stream.
    #[inline]
    pub fn of<P: Protocol>(sample_rate_hz: f32) -> Self {
        // `SYMBOL_DT = NSPS / 12_000.0` (`fst4_submode!`) is the
        // protocol's physical symbol duration in seconds — rate
        // independent. At `sample_rate_hz == 12_000.0` (every caller
        // today) this recovers `P::NSPS` exactly: `SYMBOL_DT *
        // 12_000.0 == NSPS` round-trips bit-exact in f32 for every
        // FST4/FT4 `NSPS` value in the crate (verified by
        // `sync_dims_of_matches_nsps_at_12khz` below).
        let nsps = (P::SYMBOL_DT * sample_rate_hz).round() as usize;
        let nstep = nsps / P::NSTEP_PER_SYMBOL as usize;
        let nfft1 = nsps * P::NFFT_PER_SYMBOL_FACTOR as usize;
        let nmax = (P::T_SLOT_S * sample_rate_hz) as usize;
        let tstep = nstep as f32 / sample_rate_hz;

        // `downsample_cached`'s rate — see the doc comment above.
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
            df: sample_rate_hz / nfft1 as f32,
            tstep,
            jstrt: (P::TX_START_OFFSET_S / tstep) as i32,
            jz: (2.5 / tstep) as i32,
            ds_spb: P::NSPS as usize / ndown,
            ds_rate: 12_000.0 / ndown as f32,
        }
    }
}

/// Analysis-grid descriptor: the bin↔Hz mapping [`compute_spectra`] and
/// [`coarse_sync`] use, in one place.
///
/// `docs/notes/FST4_DDC_DESIGN.md` §4.1 — VK3NV's `RxAnalysisDescriptor`.
/// Two grids exist:
///
/// - **Real** (`center_hz: 0.0, complex_input: false`): today's 12 kHz
///   real-PCM path. `bin_of` is a plain `hz/df`; `usable_bins` is
///   [`SyncDims::nh1`], the positive-frequency half a real FFT gives.
///   Every existing caller uses this grid, and its numbers are
///   unchanged bit-for-bit from before `RxGrid` existed — the
///   non-regression bar `docs/notes/FST4_DDC_DESIGN.md` §6 item 1 sets.
/// - **Complex** (`complex_input: true`): a DDC-fed complex baseband
///   grid parametrised by `(center_hz, sample_rate_hz)`, covering both
///   the wideband scan and the sniper shape with one mechanism.
///   `compute_spectra` stores this grid's spectrogram **fftshifted**,
///   so `bin_of` stays a monotonic affine map (`(hz-center)/df +
///   nfft1/2`) and [`coarse_sync`]'s correlation loop — which only
///   ever adds an offset to a bin index — needs no change to consume
///   it; see that fftshift-on-store reasoning in this type's own
///   `bin_of`/`usable_bins` doc comments.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RxGrid {
    /// The rate `audio` fed to [`compute_spectra`]/[`coarse_sync`] is
    /// actually at.
    pub sample_rate_hz: f32,
    /// Down-conversion centre frequency, Hz. `0.0` for the real path
    /// (DC-referenced, matching today's 12 kHz real-PCM ingest).
    pub center_hz: f32,
    /// `false`: real PCM, positive-frequency-only spectrum (today's
    /// path). `true`: complex I/Q baseband, full fftshifted spectrum
    /// centred on `center_hz`.
    pub complex_input: bool,
}

impl RxGrid {
    /// The real, DC-referenced grid every caller uses today.
    pub fn real(sample_rate_hz: f32) -> Self {
        Self {
            sample_rate_hz,
            center_hz: 0.0,
            complex_input: false,
        }
    }

    /// A DDC-fed complex baseband grid centred on `center_hz`.
    pub fn complex(sample_rate_hz: f32, center_hz: f32) -> Self {
        Self {
            sample_rate_hz,
            center_hz,
            complex_input: true,
        }
    }

    /// Absolute FFT bin index a frequency `hz` maps to in this grid's
    /// spectrogram, given `d`'s `nfft1`/`df` (from
    /// `SyncDims::of::<P>(self.sample_rate_hz)`).
    ///
    /// Real grid: `hz/df`, unmodified from `coarse_sync`'s pre-`RxGrid`
    /// arithmetic. Complex grid: `(hz-center)/df + nfft1/2` — the
    /// fftshifted bin `compute_spectra`'s complex path actually stores
    /// at, so this and `usable_bins` are the only two call sites
    /// `coarse_sync` needs to change to consume either grid; its
    /// correlation loop body (`abs_bin = i + nfos*k`) stays untouched.
    #[inline]
    pub fn bin_of(&self, d: &SyncDims, hz: f32) -> usize {
        if self.complex_input {
            (((hz - self.center_hz) / d.df) + d.nfft1 as f32 / 2.0).round() as usize
        } else {
            (hz / d.df).round() as usize
        }
    }

    /// Exact inverse of [`Self::bin_of`]: the frequency (Hz) an
    /// absolute FFT bin index represents in this grid. `coarse_sync`
    /// needs this the same way it needs `bin_of` — reporting a
    /// candidate's `freq_hz` back from the bin its correlation peak
    /// landed on is the same bin↔Hz mapping in the other direction,
    /// so it goes through `RxGrid` too rather than the real grid's
    /// `bin*df` shortcut alone.
    #[inline]
    pub fn hz_of(&self, d: &SyncDims, bin: usize) -> f32 {
        if self.complex_input {
            self.center_hz + (bin as f32 - d.nfft1 as f32 / 2.0) * d.df
        } else {
            bin as f32 * d.df
        }
    }

    /// Number of usable bins in this grid's spectrogram — the bound
    /// every `coarse_sync`/`compute_spectra` clamp against `d.nh1`
    /// used before `RxGrid` existed.
    ///
    /// Real grid: `d.nh1` (`nfft1/2`, the positive-frequency half a
    /// real-input FFT gives — anything above is the mirrored negative
    /// half and never valid to read). Complex grid: `d.nfft1` (the
    /// full fftshifted spectrum is meaningful, since a complex FFT's
    /// negative-frequency bins carry real information — content below
    /// `center_hz`).
    #[inline]
    pub fn usable_bins(&self, d: &SyncDims) -> usize {
        if self.complex_input { d.nfft1 } else { d.nh1 }
    }
}

/// The samples [`compute_spectra`]/[`coarse_sync`] analyse: real 12 kHz
/// PCM (today's path) or a DDC-fed complex baseband I/Q pair.
///
/// `Copy`/`Clone` (both variants are just slice references) so callers
/// can pass one value into `coarse_sync`, which forwards it into
/// `compute_spectra` unchanged.
#[derive(Copy, Clone)]
pub enum AudioSource<'a> {
    /// Real PCM, paired with [`RxGrid::real`].
    Real(&'a [i16]),
    /// Complex baseband (I, Q), paired with [`RxGrid::complex`]. Both
    /// slices must be the same length.
    Complex(&'a [f32], &'a [f32]),
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
///
/// `grid`: the rate and (for a complex `audio`) down-conversion centre
/// [`AudioSource`] is actually at — [`RxGrid::real`] for every caller
/// today (issue #323/#309; see [`SyncDims::of`]'s doc). The complex
/// path isn't yet reachable from a real DDC front end (that's
/// `docs/notes/FST4_DDC_DESIGN.md` stage 3); wiring one up later is a
/// call-site change, not a rediscovery of this function's rate/centre
/// assumptions.
///
/// [`AudioSource::Real`] behaves exactly as before `RxGrid`/
/// `AudioSource` existed: bins are read straight off the FFT output,
/// no shift. [`AudioSource::Complex`] stores **fftshifted** — raw FFT
/// bin `k` (standard convention: `k < nfft1/2` positive frequencies,
/// `k >= nfft1/2` negative, wrapped) is written to
/// `(k + nfft1/2) % nfft1`, which is exactly [`RxGrid::bin_of`]'s
/// complex-grid formula inverted. That keeps the stored frequency axis
/// monotonic, which is the whole reason `coarse_sync`'s correlation
/// loop needs no change to read either grid — see `RxGrid`'s own doc
/// comment.
pub fn compute_spectra<P: Protocol>(
    audio: AudioSource,
    bin_lo: usize,
    bin_hi_incl: usize,
    grid: RxGrid,
) -> Spectrogram {
    let d = SyncDims::of::<P>(grid.sample_rate_hz);
    let fac = 1.0f32 / 300.0;
    let mut planner = default_planner();
    let fft = planner.plan_forward(d.nfft1);

    let window: Option<Vec<f32>> = match P::SPECTRUM_WINDOW {
        SpectrumWindow::Rectangular => None,
        SpectrumWindow::Nuttall4 => Some(nuttall_window(d.nsps)),
    };

    let usable = grid.usable_bins(&d);
    let bin_lo = bin_lo.min(usable.saturating_sub(1));
    let bin_hi_incl = bin_hi_incl.min(usable.saturating_sub(1)).max(bin_lo);
    let n_freq = bin_hi_incl - bin_lo + 1;

    let mut data = vec![0.0f32; n_freq * d.nhsym];
    let mut buf = vec![Complex::new(0.0f32, 0.0); d.nfft1];

    for j in 0..d.nhsym {
        let ia = j * d.nstep;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < d.nsps {
                match audio {
                    AudioSource::Real(pcm) => {
                        let sample = if ia + k < pcm.len() {
                            let raw = pcm[ia + k] as f32 * fac;
                            match &window {
                                Some(w) => raw * w[k],
                                None => raw,
                            }
                        } else {
                            0.0
                        };
                        Complex::new(sample, 0.0)
                    }
                    AudioSource::Complex(xi, xq) => {
                        if ia + k < xi.len() {
                            let (mut si, mut sq) = (xi[ia + k], xq[ia + k]);
                            if let Some(w) = &window {
                                si *= w[k];
                                sq *= w[k];
                            }
                            Complex::new(si, sq)
                        } else {
                            Complex::new(0.0, 0.0)
                        }
                    }
                }
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        if grid.complex_input {
            for shifted in bin_lo..=bin_hi_incl {
                let k = (shifted + d.nfft1 / 2) % d.nfft1;
                data[(shifted - bin_lo) * d.nhsym + j] = buf[k].norm_sqr();
            }
        } else {
            for i in bin_lo..=bin_hi_incl {
                data[(i - bin_lo) * d.nhsym + j] = buf[i].norm_sqr();
            }
        }
    }

    Spectrogram {
        n_freq,
        n_time: d.nhsym,
        freq_offset: bin_lo,
        data,
    }
}

/// Incremental, capture-time counterpart to [`compute_spectra`] for the
/// complex (DDC-fed) path.
///
/// [`compute_spectra`] needs the whole slot before it can start, which
/// puts its cost squarely in the post-slot decode budget. But its row
/// loop only ever reads `xi[j·nstep .. j·nstep + nsps]` — row `j` is
/// computable the moment those samples exist, and rows are otherwise
/// independent. A receiver therefore does not have to wait: it can
/// complete each row as audio arrives and have the finished
/// spectrogram ready at slot end, exactly as WSPR already runs its
/// down-converter (`wspr_app`'s `ddc_loop` → `DDC_READY_IDX` →
/// `scan_loop`).
///
/// Measured motivation (issue #307, FST4-60 wideband on CoreS3): the
/// spectrogram is 2842 ms of `coarse_sync`'s 6934 ms, and the DDC ahead
/// of it another 1976 ms. Moving both off the post-slot budget is the
/// difference between a first decode at 10 060 ms and one at 5242 ms,
/// against a 7.2 s guard.
///
/// Output is **bit-identical** to [`compute_spectra`] on the same
/// input — same FFT, same window, same fftshift, same zero-fill past
/// the end of the stream — which
/// `spectrogram_builder_matches_compute_spectra` pins.
///
/// Complex input only: this exists for the DDC path, and the real-PCM
/// path has no streaming caller today.
pub struct SpectrogramBuilder {
    d: SyncDims,
    grid: RxGrid,
    bin_lo: usize,
    bin_hi_incl: usize,
    n_freq: usize,
    window: Option<Vec<f32>>,
    fft: alloc::boxed::Box<dyn crate::engine::fft::Fft>,
    /// Retained tail: samples from `abs_base` onwards that a future row
    /// still needs. Rows overlap by `nsps - nstep`, so this stays
    /// bounded at `nsps` regardless of how long the stream runs.
    hist_i: Vec<f32>,
    hist_q: Vec<f32>,
    /// Absolute stream index of `hist_*[0]`.
    abs_base: usize,
    /// Next row to emit.
    j: usize,
    data: Vec<f32>,
    buf: Vec<Complex<f32>>,
}

impl SpectrogramBuilder {
    pub fn new<P: Protocol>(bin_lo: usize, bin_hi_incl: usize, grid: RxGrid) -> Self {
        let d = SyncDims::of::<P>(grid.sample_rate_hz);
        let window: Option<Vec<f32>> = match P::SPECTRUM_WINDOW {
            SpectrumWindow::Rectangular => None,
            SpectrumWindow::Nuttall4 => Some(nuttall_window(d.nsps)),
        };
        let usable = grid.usable_bins(&d);
        let bin_lo = bin_lo.min(usable.saturating_sub(1));
        let bin_hi_incl = bin_hi_incl.min(usable.saturating_sub(1)).max(bin_lo);
        let n_freq = bin_hi_incl - bin_lo + 1;
        let mut planner = default_planner();
        let fft = planner.plan_forward(d.nfft1);
        let nfft1 = d.nfft1;
        let nhsym = d.nhsym;
        Self {
            d,
            grid,
            bin_lo,
            bin_hi_incl,
            n_freq,
            window,
            fft,
            hist_i: Vec::new(),
            hist_q: Vec::new(),
            abs_base: 0,
            j: 0,
            data: vec![0.0f32; n_freq * nhsym],
            buf: vec![Complex::new(0.0f32, 0.0); nfft1],
        }
    }

    /// Feed the next contiguous block of complex baseband, completing
    /// every row it makes available. Block boundaries are irrelevant to
    /// the result — one `push` of the whole slot and many small pushes
    /// produce the same spectrogram.
    pub fn push(&mut self, xi: &[f32], xq: &[f32]) {
        debug_assert_eq!(xi.len(), xq.len());
        self.hist_i.extend_from_slice(xi);
        self.hist_q.extend_from_slice(xq);
        self.drain_ready(false);
    }

    /// Finish the slot: emit every remaining row, zero-filling past the
    /// end of the stream the way [`compute_spectra`] does.
    pub fn finish(mut self) -> Spectrogram {
        self.drain_ready(true);
        Spectrogram {
            n_freq: self.n_freq,
            n_time: self.d.nhsym,
            freq_offset: self.bin_lo,
            data: self.data,
        }
    }

    fn drain_ready(&mut self, flush: bool) {
        while self.j < self.d.nhsym {
            let ia = self.j * self.d.nstep;
            let need_end = ia + self.d.nsps;
            let have_end = self.abs_base + self.hist_i.len();
            if !flush && have_end < need_end {
                break;
            }
            self.emit_row(ia);
            self.j += 1;
        }
        // Drop history no future row can reach. `saturating_sub` covers
        // the flush case, where `j` has run past the last row.
        if self.j < self.d.nhsym {
            let keep_from = self.j * self.d.nstep;
            let drop = keep_from
                .saturating_sub(self.abs_base)
                .min(self.hist_i.len());
            if drop > 0 {
                self.hist_i.drain(..drop);
                self.hist_q.drain(..drop);
                self.abs_base += drop;
            }
        }
    }

    fn emit_row(&mut self, ia: usize) {
        let d = &self.d;
        for (k, c) in self.buf.iter_mut().enumerate() {
            *c = if k < d.nsps {
                let abs = ia + k;
                // Same zero-fill past end-of-stream `compute_spectra`
                // applies for `ia + k >= xi.len()`.
                if abs >= self.abs_base && abs - self.abs_base < self.hist_i.len() {
                    let idx = abs - self.abs_base;
                    let (mut si, mut sq) = (self.hist_i[idx], self.hist_q[idx]);
                    if let Some(w) = &self.window {
                        si *= w[k];
                        sq *= w[k];
                    }
                    Complex::new(si, sq)
                } else {
                    Complex::new(0.0, 0.0)
                }
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        self.fft.process(&mut self.buf);
        let j = self.j;
        let nhsym = d.nhsym;
        let nfft1 = d.nfft1;
        if self.grid.complex_input {
            for shifted in self.bin_lo..=self.bin_hi_incl {
                let k = (shifted + nfft1 / 2) % nfft1;
                self.data[(shifted - self.bin_lo) * nhsym + j] = self.buf[k].norm_sqr();
            }
        } else {
            for i in self.bin_lo..=self.bin_hi_incl {
                self.data[(i - self.bin_lo) * nhsym + j] = self.buf[i].norm_sqr();
            }
        }
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
///
/// `grid`: the rate and (for a complex `audio`) down-conversion centre
/// [`AudioSource`] is actually at — [`RxGrid::real`] for every caller
/// today (issue #323/#309; see [`SyncDims::of`]'s doc). The complex
/// path isn't yet reachable from a real DDC front end (that's
/// `docs/notes/FST4_DDC_DESIGN.md` stage 3); wiring one up later is a
/// call-site change, not a rediscovery of this function's rate/centre
/// assumptions. `ia`/`ib` and every `nh1`-shaped clamp below go
/// through [`RxGrid::bin_of`]/[`RxGrid::usable_bins`] — the only two
/// places this function's logic depends on which grid it's reading;
/// the correlation loop itself (`abs_bin = i + nfos*k`) is unchanged
/// either way, since [`compute_spectra`]'s complex path already
/// stores its spectrogram on a monotonic bin axis (see that function's
/// own doc comment).
pub fn coarse_sync<P: Protocol>(
    audio: AudioSource,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
    grid: RxGrid,
) -> Vec<SyncCandidate> {
    let Some((bin_lo, bin_hi)) = spectra_crop_for::<P>(freq_min, freq_max, grid) else {
        return Vec::new();
    };
    // Crop to exactly the range the correlation loop and the FST4
    // stage1 augmentation ever read: candidate bins `ia..=ib` plus
    // `headroom` bins above `ib` for their reference tones.
    // `Spectrogram::get` stays absolute-bin-indexed (subtracts
    // `freq_offset` internally) so nothing downstream needs to change.
    let s = compute_spectra::<P>(audio, bin_lo, bin_hi, grid);
    coarse_sync_from_spectra::<P>(&s, freq_min, freq_max, sync_min, freq_hint, max_cand, grid)
}

/// The bin range [`coarse_sync`] crops its spectrogram to, or `None`
/// when the requested band is empty. Exposed so a caller building the
/// spectrogram itself — incrementally, via [`SpectrogramBuilder`] —
/// crops identically instead of re-deriving this and drifting.
pub fn spectra_crop_for<P: Protocol>(
    freq_min: f32,
    freq_max: f32,
    grid: RxGrid,
) -> Option<(usize, usize)> {
    let d = SyncDims::of::<P>(grid.sample_rate_hz);
    let ntones = P::NTONES as usize;
    let usable = grid.usable_bins(&d);
    let ia = grid.bin_of(&d, freq_min);
    let headroom = d.nfos * (ntones - 1) + 1;
    let ib = grid
        .bin_of(&d, freq_max)
        .min(usable.saturating_sub(headroom));
    if ib < ia {
        return None;
    }
    Some((ia, (ib + headroom).min(usable.saturating_sub(1))))
}

/// Upper bound on any protocol's sync-block count (FT8=3, FT4=4,
/// FST4=5 — see each protocol's `SYNC_MODE`/`mod.rs` sync-block table)
/// with headroom. Stack arrays sized to this bound let
/// [`fill_sync2d_row`]'s `t_blocks`/`t0_blocks` be reused across every
/// lag of a row instead of heap-allocated per cell — that loop runs
/// `n_freq × (2·jz+1)` times per candidate search (thousands of cells),
/// so a fresh `Vec` per cell was thousands of small allocations, and an
/// opaque allocator call is also an optimization barrier LLVM can't see
/// through.
const MAX_SYNC_BLOCKS: usize = 8;

/// Shape of the (freq-bin × lag) correlation matrix
/// [`coarse_sync_from_spectra`] fills, plus the per-row parameters
/// [`fill_sync2d_row`] reads.
///
/// Exposed so a caller can fill that matrix itself — one row at a time,
/// on whichever core it likes — and hand the finished matrix to
/// [`coarse_sync_from_sync2d`]. Rows are fully independent (the host
/// already fills them through rayon), which is the whole reason this
/// seam exists: on CoreS3's wideband FST4-60 search the fill is the
/// largest post-slot item while the second core sits idle (#327).
///
/// Build one with [`sync2d_shape`]; the fields are read-only geometry.
#[derive(Copy, Clone, Debug)]
pub struct Sync2dShape {
    /// Protocol/grid dimensions every row read goes through.
    pub d: SyncDims,
    /// Absolute spectrogram bin of row 0.
    pub ia: usize,
    /// Rows — one per candidate bin `ia..=ib`.
    pub n_freq: usize,
    /// Columns per row, lags `-jz..=jz`; also the row stride.
    pub n_lag: usize,
    /// `grid.usable_bins(&d)`, the clamp every tone read applies.
    usable: usize,
}

impl Sync2dShape {
    /// `f32`s in the whole matrix — `n_freq * n_lag`.
    pub fn len(&self) -> usize {
        self.n_freq * self.n_lag
    }

    /// Whether the band produced no candidate bins at all.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// The correlation matrix's shape for a band, or `None` when the band
/// holds no candidate bin.
///
/// Derives `ia`/`ib` exactly as [`coarse_sync_from_spectra`] does —
/// which is the point: a caller filling rows itself must index the same
/// grid the ranking stage will. Note this is the *candidate* bin range,
/// narrower than the spectrogram crop [`spectra_crop_for`] returns by
/// the `headroom` bins the reference tones read above `ib`.
pub fn sync2d_shape<P: Protocol>(
    freq_min: f32,
    freq_max: f32,
    grid: RxGrid,
) -> Option<Sync2dShape> {
    let d = SyncDims::of::<P>(grid.sample_rate_hz);
    let ntones = P::NTONES as usize;
    let usable = grid.usable_bins(&d);

    // Leave room for NTONES-1 tones above the candidate bin.
    let ia = grid.bin_of(&d, freq_min);
    let headroom = d.nfos * (ntones - 1) + 1;
    let ib = grid
        .bin_of(&d, freq_max)
        .min(usable.saturating_sub(headroom));
    if ib < ia {
        return None;
    }

    Some(Sync2dShape {
        d,
        ia,
        n_freq: ib - ia + 1,
        n_lag: (2 * d.jz + 1) as usize,
        usable,
    })
}

/// Fill row `fi` of the correlation matrix: the Costas-grid score at
/// every lag for candidate bin `shape.ia + fi`.
///
/// `row.len()` must be `shape.n_lag`. Every cell is a pure function of
/// the spectrogram, so rows may be filled in any order, concurrently,
/// by any number of threads or cores.
#[inline]
pub fn fill_sync2d_row<P: Protocol>(
    s: &Spectrogram,
    shape: &Sync2dShape,
    fi: usize,
    row: &mut [f32],
) {
    debug_assert_eq!(row.len(), shape.n_lag);
    let d = &shape.d;
    let ntones = P::NTONES as usize;
    let usable = shape.usable;
    let num_blocks = P::SYNC_MODE.blocks().len();
    debug_assert!(
        num_blocks <= MAX_SYNC_BLOCKS,
        "protocol has more sync blocks than MAX_SYNC_BLOCKS accounts for"
    );

    let i = shape.ia + fi;
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
                if m >= 0 && (m as usize) < d.nhsym && tone_bin < usable {
                    let m = m as usize;
                    t_blocks[bk] += s.get(tone_bin, m);
                    // Reference: sum over all NTONES tones at this time slot.
                    t0_blocks[bk] += (0..ntones)
                        .map(|k| s.get((i + d.nfos * k).min(usable - 1), m))
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

        row[jlag] = score;
    }
}

/// [`coarse_sync`]'s second half: everything from a finished
/// spectrogram to a ranked candidate list.
///
/// Split out so a receiver can build the spectrogram **during capture**
/// with [`SpectrogramBuilder`] and pay only this part after the slot
/// ends. On FST4-60 wideband that moves 2842 ms of a 6934 ms
/// `coarse_sync` off the post-slot decode budget (issue #307).
///
/// `s` must have been produced with the same `P`/`grid` and the crop
/// [`spectra_crop_for`] returns; anything else indexes wrongly.
///
/// Itself two stages — [`fill_sync2d_row`] across every row, then
/// [`coarse_sync_from_sync2d`] — split apart for callers that want to
/// spread the fill across cores; see [`Sync2dShape`].
#[allow(clippy::too_many_arguments)]
pub fn coarse_sync_from_spectra<P: Protocol>(
    s: &Spectrogram,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
    grid: RxGrid,
) -> Vec<SyncCandidate> {
    let Some(shape) = sync2d_shape::<P>(freq_min, freq_max, grid) else {
        return Vec::new();
    };

    // Compute correlation scores for every (freq-bin, lag) cell.
    // Each cell is fully independent, so the outer fi loop is safe to parallelise.
    let mut sync2d = alloc::vec![0.0f32; shape.len()];

    #[cfg(feature = "parallel")]
    sync2d
        .par_chunks_mut(shape.n_lag)
        .enumerate()
        .for_each(|(fi, row)| fill_sync2d_row::<P>(s, &shape, fi, row));

    #[cfg(not(feature = "parallel"))]
    for (fi, row) in sync2d.chunks_mut(shape.n_lag).enumerate() {
        fill_sync2d_row::<P>(s, &shape, fi, row);
    }

    coarse_sync_from_sync2d::<P>(s, &sync2d, &shape, sync_min, freq_hint, max_cand, grid)
}

/// [`coarse_sync_from_spectra`]'s ranking half: peak detection,
/// noise-floor normalisation, the FST4 stage-1 OR-gate, dedup and
/// ranking, over an already-filled correlation matrix.
///
/// `sync2d` must be `shape.len()` long and row-major with stride
/// `shape.n_lag` — i.e. exactly what [`fill_sync2d_row`] writes, and
/// `s`/`shape` must be the same pair those rows were filled from.
#[allow(clippy::too_many_arguments)]
pub fn coarse_sync_from_sync2d<P: Protocol>(
    s: &Spectrogram,
    sync2d: &[f32],
    shape: &Sync2dShape,
    sync_min: f32,
    freq_hint: Option<f32>,
    max_cand: usize,
    grid: RxGrid,
) -> Vec<SyncCandidate> {
    debug_assert_eq!(sync2d.len(), shape.len());
    let d = shape.d;
    let ntones = P::NTONES as usize;
    let usable = shape.usable;
    let ia = shape.ia;
    let n_freq = shape.n_freq;
    let n_lag = shape.n_lag;
    let idx = |fi: usize, lag: i32| fi * n_lag + (lag + d.jz) as usize;

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
        // doc comment. The `.min(usable - 1)` clamp is still against the
        // *absolute* full-band bound (matches `coarse_sync`'s own
        // `headroom` derivation, which sizes the crop to always cover
        // this read), so the offset subtraction happens last.
        let ccf: Vec<f32> = (0..n_freq)
            .map(|fi| {
                let i = ia + fi;
                (0..ntones)
                    .map(|k| {
                        let abs_bin = (i + d.nfos * k).min(usable - 1);
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
        let freq_hz = grid.hz_of(&d, i);
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

    // De-duplicate: within 4 Hz and 40 ms, keep highest score.
    //
    // `suppressed` records the decision separately from the score
    // because the score alone cannot carry it past the `retain` below:
    // that gate is an OR (issue #146 — an FST4 candidate under
    // `sync_min` is still kept when the stage-1 normalised score passes),
    // so a loser zeroed here would be re-admitted by the `stage1_pass`
    // arm and go on to occupy a slot after `max_cand` truncation.
    //
    // Measured before this was tracked (issue #312, VK3NV): 3 of the 50
    // slots at the production `max_cand = 50` on the golden's K9KFR
    // target, all three inside `rank_candidates`' reserved
    // near-`freq_hint` group — and across 80 near-threshold sweep trials
    // not one such candidate ever decoded, so honouring the dedup here
    // costs no recall (`fst4_60_diag_dedup_zero_score_recall_effect`,
    // and `fst4_60_diag_wideband_band_recall` for the no-hint path).
    //
    // Deliberately *not* the narrower-looking `score >= sync_min &&
    // stage1_pass(fi)`: that would also drop candidates which clear the
    // score gate but fail stage 1, which is what the #146 OR-gate exists
    // to keep. The defect is only that a rejected duplicate comes back,
    // so only that is fixed.
    let suppressed = dedup_suppress(&mut cands);
    let mut keep = suppressed.iter().map(|s| !s);
    cands.retain(|c| {
        if !keep.next().unwrap_or(true) {
            return false;
        }
        if c.score >= sync_min {
            return true;
        }
        let fi = ((c.freq_hz / d.df).round() as usize).saturating_sub(ia);
        stage1_pass(fi)
    });

    rank_candidates(cands, freq_hint, max_cand)
}

/// Frequency half-window the dedup below treats as "the same signal".
const DEDUP_HZ: f32 = 4.0;
/// Time half-window, likewise.
const DEDUP_SEC: f32 = 0.04;

/// Mark, per candidate, whether a near-duplicate beat it — same signal
/// within [`DEDUP_HZ`] and [`DEDUP_SEC`], loser's score zeroed.
///
/// A sliding window, not the O(n^2) all-pairs loop this used to be.
/// `cands` is frequency-sorted by construction — built `fi`-major over
/// `0..n_freq`, every candidate of one `fi` sharing that bin's
/// `freq_hz`, and `hz_of` monotonic in the bin (rayon's `collect`
/// preserves order, so the parallel path is sorted too) — so the
/// partners of `cands[i]` are the contiguous run ending at `i - 1`
/// whose frequency is within [`DEDUP_HZ`]. Every pair outside that run
/// fails the `fdiff` test and mutates nothing, so skipping them leaves
/// both the visitation order and the result **exactly** as the
/// all-pairs loop had them. That equivalence is load-bearing rather
/// than incidental: the loop mutates scores as it goes, so which pairs
/// are compared in which order decides the outcome
/// (`dedup_suppress_matches_all_pairs` pins it against a brute-force
/// reference).
///
/// The quadratic term was not academic. On the wideband FST4-60
/// monitor search (issue #327, real CoreS3) this ranking half cost
/// 2989 ms against 1130 ms for the correlation fill it ranks — ~15k
/// candidates from 1881 bins x up to 8 lag peaks each, i.e. ~112M pair
/// tests. It is also the whole of the otherwise-unexplained 3.2x
/// per-bin gap between the wideband search (2.147 ms/bin) and the
/// sniper's (0.671 ms/bin), which #307 had provisionally attributed to
/// the spectrogram working set: per-bin *fill* cost is in fact the same
/// for both (0.60 ms/bin), and only this superlinear stage differed.
fn dedup_suppress(cands: &mut [SyncCandidate]) -> Vec<bool> {
    let mut suppressed = alloc::vec![false; cands.len()];
    let mut lo = 0usize;
    for i in 1..cands.len() {
        debug_assert!(
            cands[i].freq_hz >= cands[i - 1].freq_hz,
            "dedup window assumes frequency-sorted candidates"
        );
        while cands[i].freq_hz - cands[lo].freq_hz >= DEDUP_HZ {
            lo += 1;
        }
        for j in lo..i {
            let fdiff = (cands[i].freq_hz - cands[j].freq_hz).abs();
            let tdiff = (cands[i].dt_sec - cands[j].dt_sec).abs();
            if fdiff < DEDUP_HZ && tdiff < DEDUP_SEC {
                if cands[i].score >= cands[j].score {
                    cands[j].score = 0.0;
                    suppressed[j] = true;
                } else {
                    cands[i].score = 0.0;
                    suppressed[i] = true;
                }
            }
        }
    }
    suppressed
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
    // Only `d.ds_spb` is read below — a `SyncDims::of` field that
    // `downsample_cached`'s own rate governs, not the `sample_rate_hz`
    // parameter (see that doc comment), so the argument here is inert.
    let d = SyncDims::of::<P>(12_000.0);
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

/// Refine a coarse-search frequency bin to sub-bin precision via
/// 3-point log-power parabolic ("Jacobsen") interpolation of power
/// around `base_bin`. `power_at(bin)` supplies the (linear-domain,
/// un-normalised) power at a neighbouring bin — callers pass a closure
/// over their own already-computed spectrogram rather than this
/// function owning any spectrogram type, since that type (and how
/// "power at a bin" is defined — which sync positions get summed) is
/// protocol-specific.
///
/// Extracted (2026-08-14, code-sharing audit) from two byte-identical
/// copies that had independently accreted in `jt65::search` and
/// `jt9::search` — the second copy was deliberately written to match
/// the first (see the historical `jt9::search::refine_freq_hz` doc:
/// "reused here with the same technique... and the same non-peak-shaped
/// fallback"), so unifying them changes nothing behaviourally, only
/// where the one copy lives.
///
/// **Not the same estimator as [`parabolic_peak`]**, despite solving a
/// superficially similar problem: `parabolic_peak` fits a parabola in
/// the *linear* power domain and always returns an interpolated offset
/// (the near-zero-denominator guard exists only to avoid a division by
/// zero); this fits in the *log*-power domain and explicitly declines
/// to extrapolate — falling back to the untouched bin center — when
/// the 3-point curve isn't concave-down (`denom >= -1e-9`), i.e. isn't
/// actually peak-shaped. That fallback matters here specifically:
/// coarse-search candidates can be noise-dominated or off the true
/// local max, and extrapolating a non-peak-shaped curve would invent
/// a frequency offset from noise rather than declining to guess.
/// Collapsing the two into one function would be an algorithm change,
/// not a refactor — they stay separate on purpose.
pub fn refine_freq_hz_log_power(
    base_bin: usize,
    bin_count: usize,
    df: f32,
    power_at: impl Fn(usize) -> f32,
) -> f32 {
    if base_bin == 0 || base_bin + 1 >= bin_count {
        return base_bin as f32 * df;
    }
    let y_lo = power_at(base_bin - 1).max(1e-12).ln();
    let y_mid = power_at(base_bin).max(1e-12).ln();
    let y_hi = power_at(base_bin + 1).max(1e-12).ln();
    let denom = y_lo - 2.0 * y_mid + y_hi;
    // `denom < 0` at a genuine local peak (concave-down parabola); a
    // non-negative denom means the 3-point fit isn't peak-shaped
    // (noise-dominated or `base_bin` isn't actually the local max) —
    // don't extrapolate, just keep the coarse bin-center frequency.
    let delta = if denom < -1e-9 {
        (0.5 * (y_lo - y_hi) / denom).clamp(-0.5, 0.5)
    } else {
        0.0
    };
    (base_bin as f32 + delta) * df
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
    // Only `d.ds_rate` is read below — see `fine_sync_power_per_block`'s
    // identical comment.
    let d = SyncDims::of::<P>(12_000.0);
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

#[cfg(all(test, feature = "fst4", feature = "ft4"))]
mod tests {
    use super::{
        AudioSource, DEDUP_HZ, DEDUP_SEC, PI, Protocol, RxGrid, SyncCandidate, SyncDims,
        compute_spectra, dedup_suppress,
    };
    use crate::engine::protocol::ModulationParams;
    use crate::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
    use crate::ft4::Ft4;
    use alloc::vec::Vec;

    /// `SyncDims::of::<P>(12_000.0)`'s `nsps` is derived from
    /// `P::SYMBOL_DT * sample_rate_hz` (issue #309/#323), not read
    /// directly from `P::NSPS` as before this refactor — this is the
    /// "verified to round-trip bit-exact" claim that refactor's commit
    /// makes. `SYMBOL_DT = NSPS / 12_000.0` (`fst4_submode!`), so at
    /// `sample_rate_hz == 12_000.0` (every caller today) the two must
    /// agree exactly, for every real NSPS value in the crate — a
    /// mismatch here would silently shift `nfft1`/`nmax`/`df`/`tstep`
    /// for every existing decode path.
    #[test]
    fn sync_dims_of_matches_nsps_at_12khz() {
        fn check<P: Protocol + ModulationParams>(name: &str) {
            let d = SyncDims::of::<P>(12_000.0);
            assert_eq!(
                d.nsps,
                P::NSPS as usize,
                "{name}: SyncDims::of(12_000.0).nsps != P::NSPS"
            );
        }
        check::<Fst4s15>("Fst4s15");
        check::<Fst4s30>("Fst4s30");
        check::<Fst4s60>("Fst4s60");
        check::<Fst4s120>("Fst4s120");
        check::<Fst4s300>("Fst4s300");
        check::<Ft4>("Ft4");
    }

    /// `RxGrid::real`'s `bin_of`/`usable_bins` must reproduce, bit for
    /// bit, the inline `(hz/df).round() as usize` / `d.nh1` arithmetic
    /// `coarse_sync`/`compute_spectra` used before `RxGrid` existed —
    /// `docs/notes/FST4_DDC_DESIGN.md` §6 item 1's non-regression bar,
    /// pinned the same way `sync_dims_of_matches_nsps_at_12khz` pins
    /// `SyncDims::of`'s own refactor.
    #[test]
    fn rx_grid_real_matches_pre_rxgrid_bin_math() {
        let d = SyncDims::of::<Fst4s60>(12_000.0);
        let grid = RxGrid::real(12_000.0);
        for hz in [0.0f32, 100.0, 1500.0, 2963.34, 3000.0] {
            let want = (hz / d.df).round() as usize;
            assert_eq!(grid.bin_of(&d, hz), want, "hz={hz}");
        }
        assert_eq!(grid.usable_bins(&d), d.nh1);
    }

    /// A complex DDC-fed grid's `compute_spectra` path stores its
    /// spectrogram fftshifted so `RxGrid::bin_of`'s complex formula
    /// predicts where a tone lands — including a tone *below*
    /// `center_hz` (negative baseband frequency), which is the half
    /// the fftshift-on-store scheme exists to keep monotonic (see
    /// `RxGrid`'s and `compute_spectra`'s own doc comments).
    #[test]
    fn rx_grid_complex_bin_of_predicts_compute_spectra_peak() {
        let sample_rate_hz = 12_000.0f32;
        let center_hz = 1500.0f32;
        let d = SyncDims::of::<Fst4s60>(sample_rate_hz);
        let grid = RxGrid::complex(sample_rate_hz, center_hz);

        for offset_hz in [200.0f32, -200.0] {
            let f_signal = center_hz + offset_hz;
            let f_baseband = offset_hz; // e^{j 2*pi*f_baseband*n/Fs}
            let n = d.nmax;
            let w = 2.0 * PI * f_baseband / sample_rate_hz;
            let audio_i: Vec<f32> = (0..n).map(|k| (w * k as f32).cos()).collect();
            let audio_q: Vec<f32> = (0..n).map(|k| (w * k as f32).sin()).collect();

            let s = compute_spectra::<Fst4s60>(
                AudioSource::Complex(&audio_i, &audio_q),
                0,
                d.nfft1 - 1,
                grid,
            );
            let avg = s.avg_power_per_bin();
            let (peak_bin, _) = avg
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .unwrap();

            let want = grid.bin_of(&d, f_signal);
            assert!(
                peak_bin.abs_diff(want) <= 1,
                "offset={offset_hz}: peak_bin={peak_bin}, RxGrid predicted {want}"
            );
        }
    }

    /// The windowed dedup must reproduce the all-pairs loop it
    /// replaced *exactly*, not merely "keep the same winners": the loop
    /// zeroes scores as it goes, so a different visitation order can
    /// flip later comparisons. Brute-force reference, over lists built
    /// to stress the parts that matter — dense ties (`>=` decides
    /// those, and it decides them in favour of the later candidate),
    /// candidates exactly on the 4 Hz boundary, and chains where a
    /// suppressed candidate is itself compared again afterwards.
    #[test]
    fn dedup_suppress_matches_all_pairs() {
        fn reference(cands: &mut [SyncCandidate]) -> Vec<bool> {
            let mut suppressed = alloc::vec![false; cands.len()];
            for i in 1..cands.len() {
                for j in 0..i {
                    let fdiff = (cands[i].freq_hz - cands[j].freq_hz).abs();
                    let tdiff = (cands[i].dt_sec - cands[j].dt_sec).abs();
                    if fdiff < DEDUP_HZ && tdiff < DEDUP_SEC {
                        if cands[i].score >= cands[j].score {
                            cands[j].score = 0.0;
                            suppressed[j] = true;
                        } else {
                            cands[i].score = 0.0;
                            suppressed[i] = true;
                        }
                    }
                }
            }
            suppressed
        }

        // Deterministic pseudo-random lists, frequency-sorted the way
        // `coarse_sync_from_sync2d` builds them: `df`-spaced bins, up
        // to 8 lag peaks per bin.
        let mut state = 0x1234_5678u32;
        let mut next = move || {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            (state >> 16) as f32 / 65_536.0
        };
        for df in [0.5f32, 1.54, 4.0, 13.0] {
            let mut cands: Vec<SyncCandidate> = Vec::new();
            for bin in 0..40 {
                let freq_hz = 100.0 + bin as f32 * df;
                for _ in 0..(1 + (next() * 8.0) as usize) {
                    cands.push(SyncCandidate {
                        freq_hz,
                        // Spread lags across and around the 40 ms
                        // window so both arms of `tdiff` are exercised.
                        dt_sec: (next() * 0.4 - 0.2 + (next() * 4.0).floor() * 0.01),
                        // Coarse quantisation so exact ties are common.
                        score: (next() * 5.0).floor(),
                    });
                }
            }
            let mut want = cands.clone();
            let want_flags = reference(&mut want);
            let mut got = cands.clone();
            let got_flags = dedup_suppress(&mut got);

            assert_eq!(want_flags, got_flags, "df={df}: suppression flags differ");
            for (i, (w, g)) in want.iter().zip(got.iter()).enumerate() {
                assert_eq!(w.score, g.score, "df={df}: candidate {i} score differs");
            }
        }
    }
}
