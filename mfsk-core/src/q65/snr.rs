// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65's real *displayed* SNR — port of `q65_snr` (`q65.f90:744`,
//! issue #255 §5).
//!
//! WSJT-X's Q65 decoder computes two entirely different "SNR"
//! quantities: an `esnodb`-based value inside `q65_dec_q3`/
//! `q65_dec_q012` (the value [`crate::fec::qra::fast_fading::
//! esnodb_fast_fading`] already faithfully ports, currently dead
//! code), and `q65_snr`'s own value — which the caller
//! (`q65_decode.f90:329,441`) always computes *after* decode and
//! passes to the display callback, unconditionally overwriting
//! whatever `esnodb`-based number the decode itself produced. Only
//! `q65_snr`'s value ever reaches a user. This module ports that one.
//!
//! ## The algorithm
//!
//! Unlike FT8/FT4/FST4's SNR formulas (signal power at the one
//! decoded tone vs. a baseline), `q65_snr` builds a **composite,
//! tone-aligned spectrum**: for every one of the 85 symbols (22 sync +
//! 63 data), take that symbol's full power spectrum and shift it so
//! the symbol's *actual transmitted tone* lands on a common reference
//! bin axis, then sum across all 85 symbols. A real signal produces a
//! sharp peak on that composite axis (every symbol's energy stacks
//! coherently at the same reference bin); noise doesn't. The baseline
//! is then a flat mean of two frequency guard bands flanking that
//! peak, and the reported quantity is the integrated excess power
//! (`sig_area`) over the region between them, converted to WSJT-X's
//! 2500 Hz reference bandwidth.
//!
//! Because this stacks the *full* spectrum (not just the tone bins),
//! it needs the raw audio and an FFT per symbol — [`extract_data_energies`]
//! and friends only keep the 64 tone-bin energies per data symbol,
//! not enough to rebuild the composite spectrum. Hence this module's
//! own extraction ([`q65_composite_spectrum`]) rather than reusing
//! that machinery.
//!
//! ## Why not `q65::search::Spectrogram`?
//!
//! WSJT-X's own `q65_snr` reuses the *same* coarse `s1` array its
//! sync search already built (`NSTEP=8` time bins per symbol,
//! `q65.f90:3`) rather than computing anything fresh — an efficiency
//! choice, not a precision requirement (see `q65_symspec`,
//! `q65.f90:265-302` — it also applies `smo121` frequency-domain
//! smoothing and a 2×-time-subsample-then-interpolate trick that
//! `Spectrogram::build_for` doesn't). This module instead computes a
//! fresh FFT at the exact decoded `(start_sample, base_freq_hz)` per
//! symbol, matching [`extract_data_energies`]'s own full-precision
//! convention (already proven correct for the BP decode itself)
//! rather than WSJT-X's `NSTEP=8`-quantised-and-smoothed grid. The
//! precision difference doesn't appear to change the reported dB by a
//! measurable amount in practice: the four real recordings verified
//! below span `mode_q65` = 1, 8, 8, 16 (i.e. WSJT-X's own `nsmo` =
//! 1, 32, 32, 128 for those same signals — `smo121`'s pass count
//! grows with `mode_q65²`) and all four land within 0.6-0.8 dB of
//! `jt9` regardless; if smoothing/interpolation mattered for the
//! final number, the `mode_q65=16` case would be expected to diverge
//! the most, and it doesn't (issue #255 / #256 discussion).
//!
//! ## Verification
//!
//! Checked against a real local `jt9 -3 -d3` build's own displayed
//! SNR on four real off-air Q65 recordings (issue #255's stated
//! verification discipline) — see [`q65_snr_db`]'s doc comment for
//! the numbers.
//!
//! ## Multi-period averaging (`iavg=1,2`)
//!
//! [`super::rx`]'s three `decode_averaged_*`/`decode_fading_with_energies`
//! functions (`super::rx::decode_multi_period_for`'s candidate loop)
//! use [`q65_snr_db_averaged`] instead of [`q65_snr_db`]. WSJT-X's own
//! `iavg` path (`q65_dec0`, `q65.f90:141-142`: `s1 = s1a(:,:,iseq)`)
//! reads its module-level `s1` from an accumulated array rather than a
//! fresh single-slot one, built via the exact same EMA weight
//! (`u = 1.0/min(navg,4)`, `q65_symspec` `q65.f90:298-304`) this
//! crate's own `Spectrogram`-based coarse search already implements
//! for `decode_multi_period_for`'s running-average sync search.
//! [`q65_composite_spectrum_averaged`] applies that identical EMA
//! directly to the *composite* spectrum rather than to the underlying
//! per-symbol energies first — mathematically equivalent, since the
//! "sum 85 symbols' shifted spectra" step the composite spectrum is
//! built from is linear, so averaging its output commutes with
//! averaging its input.

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;
use rustfft::FftPlanner;

use crate::engine::ModulationParams;

use super::sync_pattern::Q65_SYNC_POSITIONS;

/// Build the composite, tone-aligned power spectrum `q65_snr`
/// (`q65.f90:744-766`) sums across all 85 symbols.
///
/// `codeword` is the 63-symbol re-encoded channel codeword (values
/// `0..64`, e.g. from [`crate::fec::qra::Q65Codec::encode`] on the
/// decoded info symbols — the same array [`super::rx`]'s
/// `snr_db_narrow`/`snr_db_wide` already consume).
///
/// Returns `(spec, nsum)`: `spec[i]` is the composite power at
/// absolute FFT bin `ia + i` (`ia = base_bin - 2·nsum`, clamped to
/// `[0, nsps/2)`), and `nsum` is the guard-band width in bins
/// (`q65_snr`'s own `nsum = max(10·mode_q65, round(50/df))`) — both
/// needed by [`q65_snr_from_spectrum`] to find the guard bands and
/// integration region within `spec`.
///
/// Returns `None` if `audio` doesn't span the full 85-symbol frame at
/// `start_sample`, or the guard-band window doesn't fit inside the
/// spectrum (candidate too close to DC or Nyquist).
pub(crate) fn q65_composite_spectrum<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    codeword: &[i32],
) -> Option<(Vec<f32>, i64)> {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    if nsps == 0 || start_sample.checked_add(85 * nsps)? > audio.len() {
        return None;
    }
    let df = sample_rate as f32 / nsps as f32;
    let n_freq = (nsps / 2) as i64;
    let base_bin = (base_freq_hz / df).round() as i64;
    // `mode_q65` (`q65_decode.f90:106`, `2**nsubmode`) — both the
    // per-tone bin shift and the guard-band-width scale factor below.
    // `TONE_SPACING_HZ = baud·mode_q65` and `df = baud` (same `nsps`
    // both constants are derived from), so this ratio recovers
    // `mode_q65` exactly without a separate stored constant.
    let mode_q65 = (P::TONE_SPACING_HZ / df).round().max(1.0) as i64;

    // `q65.f90:775`: `nsum = max(10*mode_q65, nint(50.0/df))`.
    let nsum = (10 * mode_q65).max((50.0 / df).round() as i64).max(1);
    let ia = (base_bin - 2 * nsum).max(0);
    let ib = (base_bin + 2 * nsum).min(n_freq - 1);
    if ib - ia < 2 * nsum {
        return None;
    }
    let width = (ib - ia + 1) as usize;
    let mut spec = vec![0.0f32; width];

    let mut sync_iter = Q65_SYNC_POSITIONS.iter().peekable();
    let mut data_k = 0usize;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nsps);
    let mut scratch = vec![Complex::new(0f32, 0f32); fft.get_inplace_scratch_len()];
    let mut buf: Vec<Complex<f32>> = vec![Complex::new(0f32, 0f32); nsps];

    for sym_idx in 0u32..85 {
        // `itone(j)` (`q65.f90:753-761`): 0 for sync symbols, else the
        // symbol's transmitted tone `codeword(k)+1` (tone 0 is
        // reserved for sync — same "+1" convention
        // `extract_data_energies` uses for its own bin indexing).
        let itone: i64 = if sync_iter.peek().is_some_and(|&&p| p == sym_idx) {
            sync_iter.next();
            0
        } else {
            let t = codeword.get(data_k).copied().unwrap_or(0) as i64 + 1;
            data_k += 1;
            t
        };

        let sym_start = start_sample + sym_idx as usize * nsps;
        for (slot, &s) in buf.iter_mut().zip(&audio[sym_start..sym_start + nsps]) {
            *slot = Complex::new(s, 0.0);
        }
        fft.process_with_scratch(&mut buf, &mut scratch);

        // `spec(i) += s1(i + mode_q65·itone(k), j)` for `i` in
        // `[ia, ib]` — `i`/`ii` are absolute bin indices in WSJT-X's
        // own convention, not offsets from `base_bin`.
        let shift = mode_q65 * itone;
        for (idx, i) in (ia..=ib).enumerate() {
            let ii = i + shift;
            if ii >= 0 && ii < n_freq {
                spec[idx] += buf[ii as usize].norm_sqr();
            }
        }
    }

    Some((spec, nsum))
}

/// Guard-band baseline + integrated-excess-power SNR from a composite
/// spectrum ([`q65_composite_spectrum`]'s output) — `q65_snr`'s own
/// `q65.f90:768-782`:
///
/// ```text
///   sum1 = Σ spec[0..nsum)            ! low guard band
///   sum2 = Σ spec[width-nsum..width)  ! high guard band
///   avg  = (sum1+sum2) / (2·nsum)     ! baseline level (→ 1.0 after normalising)
///   sig_area = Σ (spec[nsum..width-nsum] / avg - 1.0)
///   snr2 = 10·log10(max(1, sig_area)) - 10·log10(2500/df)
/// ```
///
/// Not this crate's own invention: the same shape as
/// [`crate::engine::baseline`]'s percentile+polyfit baseline in
/// *purpose* (estimate the noise floor from the data itself), but a
/// distinct, simpler algorithm — a flat two-guard-band mean, not a
/// polynomial fit — since `q65_snr`'s composite spectrum is already a
/// single sharp peak on a flat floor, not a slowly-varying spectral
/// shape. Do not try to route this through `BaselineParams`.
fn q65_snr_from_spectrum(spec: &[f32], nsum: i64, df: f32) -> Option<f32> {
    let nsum = usize::try_from(nsum).ok()?;
    let width = spec.len();
    if width < 2 * nsum {
        return None;
    }
    let sum1: f32 = spec[..nsum].iter().sum();
    let sum2: f32 = spec[width - nsum..].iter().sum();
    let avg = (sum1 + sum2) / (2.0 * nsum as f32);
    if avg.is_nan() || avg <= 0.0 {
        return None;
    }
    let sig_area: f32 = spec[nsum..width - nsum]
        .iter()
        .map(|&x| x / avg - 1.0)
        .sum();
    Some(10.0 * sig_area.max(1.0).log10() - 10.0 * (2500.0 / df).log10())
}

/// Q65's real displayed SNR (`q65.f90:744-793`). Composes
/// [`q65_composite_spectrum`] + [`q65_snr_from_spectrum`]; falls back
/// to `fallback_db` (callers pass the existing `snr_db_narrow`/
/// `snr_db_wide` adjacent-tone estimate) when the composite spectrum
/// can't be built (candidate too close to a band edge) rather than a
/// hardcoded sentinel, since unlike FST4's `-99.9` WSJT-X convention
/// there is no equivalent "invalid" marker in `q65_snr` itself to
/// port faithfully.
///
/// Verified against a real local `jt9 -3 -d3` build's own displayed
/// SNR on four real off-air Q65 recordings, one per sub-mode, all
/// decoded via [`crate::q65::rx::decode_at_fading_for`]'s fast-fading
/// path (`tests/q65_wsjtx_samples.rs`'s existing golden tests) —
/// landed within ~1 dB on the *first* implementation attempt, no
/// scale-factor archaeology needed (unlike FST4's own port, issue
/// #255 §4 — Q65's `extract_data_energies`/this module's own
/// extraction both FFT the raw audio directly, no shared downsample/
/// RMS-normalisation pipeline in between to introduce a mismatch):
///
/// | sub-mode | file | jt9 | this module | diff |
/// |----------|------|-----|-------------|------|
/// | Q65-60D  | `60D_EME_10GHz/201212_1838.wav` (VK7MO K6QPV) | -15 dB | -14.33 dB | 0.67 dB |
/// | Q65-120D | `120D_Rainscatter_10_GHz/210117_0920.wav` (VK3WE VK7MO) | -16 dB | -16.59 dB | 0.59 dB |
/// | Q65-120E | `120E_Ionoscatter_6m/210130_1442.wav` (KB7IJ N0AN) | -17 dB | -16.39 dB | 0.61 dB |
/// | Q65-300A | `300A_Optical_Scatter/201210_0505.wav` (VK7MO VK7PD) | -34 dB | -33.18 dB | 0.82 dB |
pub(crate) fn q65_snr_db<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    codeword: &[i32],
    fallback_db: f32,
) -> f32 {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps.max(1) as f32;
    match q65_composite_spectrum::<P>(audio, sample_rate, start_sample, base_freq_hz, codeword) {
        Some((spec, nsum)) => q65_snr_from_spectrum(&spec, nsum, df).unwrap_or(fallback_db),
        None => fallback_db,
    }
}

/// EMA-average [`q65_composite_spectrum`] across `audio_slots`,
/// matching WSJT-X's own `s1a` accumulation (`q65.f90:298-304`):
/// `weight = 1.0/min(navg,4)` where `navg` is the 1-based slot index,
/// capped at 4 (the time constant saturates after the 4th slot, so
/// older history decays at a fixed ~25%/slot rate — same formula
/// `q65::search::Spectrogram`-based `decode_multi_period_for` already
/// uses for its own coarse-search EMA).
///
/// Averages the *composite* spectrum directly rather than the
/// underlying per-symbol energies (WSJT-X's own `s1`) — see this
/// module's own doc comment for why that's mathematically equivalent
/// here (the composite-spectrum sum is linear in the per-symbol
/// energies, so EMA-averaging commutes through it).
///
/// `start_sample`/`base_freq_hz`/`codeword` are assumed constant
/// across `audio_slots` (the same candidate position and decoded
/// message recurring slot to slot) — the same assumption
/// `decode_multi_period_for`'s own EMA-on-spectrogram coarse search
/// already makes. A slot whose own [`q65_composite_spectrum`] returns
/// `None` (e.g. too short) or whose geometry (`nsum`/window width)
/// doesn't match the running average is skipped for the composite
/// step but doesn't abort the whole accumulation — best-effort, same
/// spirit as [`super::rx::averaged_data_energies`]'s own per-slot
/// `continue`-on-failure loop.
pub(crate) fn q65_composite_spectrum_averaged<P: ModulationParams>(
    audio_slots: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    codeword: &[i32],
) -> Option<(Vec<f32>, i64)> {
    let mut acc: Option<(Vec<f32>, i64)> = None;
    let mut navg = 0u32;
    for &audio in audio_slots {
        let Some((spec, nsum)) =
            q65_composite_spectrum::<P>(audio, sample_rate, start_sample, base_freq_hz, codeword)
        else {
            continue;
        };
        match &mut acc {
            None => {
                navg = 1;
                acc = Some((spec, nsum));
            }
            Some((acc_spec, acc_nsum)) if acc_spec.len() == spec.len() && *acc_nsum == nsum => {
                navg += 1;
                let weight = 1.0f32 / navg.min(4) as f32;
                let one_minus = 1.0 - weight;
                for (a, s) in acc_spec.iter_mut().zip(&spec) {
                    *a = weight * s + one_minus * *a;
                }
            }
            // Geometry mismatch (shouldn't happen in practice — same
            // sub-mode/sample_rate/base_freq_hz every call) — keep the
            // running average rather than let one odd slot corrupt it.
            Some(_) => {}
        }
    }
    acc
}

/// [`q65_snr_db`]'s multi-period-averaging counterpart — see this
/// module's own doc comment (`## Multi-period averaging`) for the
/// derivation. Used by [`super::rx`]'s `decode_averaged_*`/
/// `decode_fading_with_energies` (`decode_multi_period_for`'s
/// candidate loop).
pub(crate) fn q65_snr_db_averaged<P: ModulationParams>(
    audio_slots: &[&[f32]],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    codeword: &[i32],
    fallback_db: f32,
) -> f32 {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let df = sample_rate as f32 / nsps.max(1) as f32;
    match q65_composite_spectrum_averaged::<P>(
        audio_slots,
        sample_rate,
        start_sample,
        base_freq_hz,
        codeword,
    ) {
        Some((spec, nsum)) => q65_snr_from_spectrum(&spec, nsum, df).unwrap_or(fallback_db),
        None => fallback_db,
    }
}
