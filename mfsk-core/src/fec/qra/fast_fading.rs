//! Fast-fading intrinsic-probability metric for Q65.
//!
//! Direct port of `q65_intrinsics_fastfading` and
//! `q65_esnodb_fastfading` from `lib/qra/q65/q65.c` (Nico Palermo
//! IV3NWV / Joe Taylor K1JT). The plain AWGN
//! [`crate::fec::qra::QraCode::mfsk_bessel_metric`] front end assumes a
//! single sinusoid per tone bin; that loses 5–8 dB on channels with
//! significant Doppler spread (lunar libration at microwave EME, fast
//! aircraft scatter, etc.). The fast-fading metric models the per-tone
//! energy as smeared across a calibrated number of adjacent FFT bins
//! and recovers most of that loss.
//!
//! ## Inputs
//!
//! Both functions consume a **wide energy spectrogram** with shape
//! `nN × nBinsPerSymbol` floats laid out row-major, where:
//!
//! - `nN` is the (punctured) codeword length — 63 for Q65.
//! - `nBinsPerSymbol = nM × (2 + nBinsPerTone)` and `nM = 64`.
//! - `nBinsPerTone = 1 << submode` (1, 2, 4, 8, 16 for sub-modes A‥E)
//!   is the number of FFT bins per tone-spacing — equivalently
//!   `tone_spacing / df` rounded.
//!
//! Per symbol slot the bin layout is `[ nM leading-pad | nM tones ×
//! nBinsPerTone central bins | nM trailing-pad ]`. The pad is
//! intentional space for the spread-weighting window's tails.
//!
//! ## Tuning parameter
//!
//! `b90_ts` is the spread bandwidth × symbol period — the fraction
//! of the symbol's spectral energy that falls inside the central
//! ±B90/2 Hz, expressed in normalised units. Typical values range
//! from ~0.05 (tight, near-AWGN) to ~5 (severe spread).
//!
//! The C reference picks the table index via
//! `hidx = log(B90 / TS_QRA64) / log(1.09) - 0.499`, clamped to
//! `[0, 63]`. We reproduce this exactly so the Rust port consumes the
//! same calibration tables in `super::fading_tables` without
//! reinterpretation.

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fading_tables::{GAUSS_LEN, GAUSS_TAPS, LORENTZ_LEN, LORENTZ_TAPS};
use super::pdmath;
use super::{QraCode, QraCodeType};

/// QRA64 reference symbol period used to scale `B90·Ts` into the
/// pre-computed table index. The fading tables were generated for
/// QRA64 (where `Ts = 0.576 s`); Q65 carries the same tables but
/// rescales `B90` so they remain valid at Q65's symbol period.
const TS_QRA64: f32 = 0.576;

/// Empirical Es/No degradation at maximum Doppler spread for the
/// QRA64 / Q65 channel — 8 dB at `B90 = 240 Hz`. Used to scale the
/// decoder's effective metric Es/No for a given spread bandwidth.
/// Mirrors the constant 240.0 baked into `q65_intrinsics_fastfading`.
const MAX_SPREAD_HZ: f32 = 240.0;

/// Two fading-shape models that ship with the Q65 reference. The
/// Gaussian model fits libration-limited EME and most AWGN-with-
/// jitter scenarios; the Lorentzian model has heavier tails and fits
/// some ionoscatter / meteor-burst signatures.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FadingModel {
    Gaussian,
    Lorentzian,
}

/// Per-frame intermediate state captured by
/// [`intrinsics_fast_fading`] and consumed by [`esnodb_fast_fading`]
/// to estimate the decoded codeword's Es/No.
#[derive(Clone, Debug)]
pub struct FastFadingState {
    /// Effective decoder Es/No metric (linear) used to compute the
    /// weights — the AWGN metric scaled down by the spread-induced
    /// degradation factor.
    pub es_no_metric: f32,
    /// Estimated noise spectral density (with a known overestimate
    /// factor that's accounted for in [`esnodb_fast_fading`]).
    pub noise_var: f32,
    /// FFT bins per tone spacing. 1/2/4/8/16 for sub-modes A‥E.
    pub n_bins_per_tone: usize,
    /// FFT bins per symbol slot (`64 × (2 + n_bins_per_tone)`).
    pub n_bins_per_symbol: usize,
    /// One-sided fading window, length `(L + 1) / 2` where `L` is
    /// the (odd) full window length. Already pre-scaled by
    /// `Es/No / (1 + tap·Es/No) / σ²`.
    pub weights: Vec<f32>,
}

/// Submode index (0..=4 ⇒ A..E).
fn validate_submode(submode: u8) -> u8 {
    assert!(
        submode <= 4,
        "fast-fading submode must be 0..=4 (got {submode})"
    );
    submode
}

fn bins_per_tone(submode: u8) -> usize {
    1usize << validate_submode(submode)
}

/// Map `B90·Ts` to the fading-table index, mirroring
/// `q65_intrinsics_fastfading`'s `hidx` calculation including the
/// `[0, 63]` clamp.
fn table_index(b90_ts: f32) -> usize {
    let b90 = b90_ts / TS_QRA64;
    // The C source uses `(int)(... - 0.499)` which truncates toward
    // zero. For non-negative arguments that's equivalent to the
    // built-in cast (which also truncates toward zero in Rust).
    let raw = (b90.ln() / 1.09_f32.ln() - 0.499).floor() as i32;
    raw.clamp(0, 63) as usize
}

/// Codeword length the fast-fading metric expects on the wide-energy
/// input. For Q65 (`CrcPunctured2`) this is `N - 2 = 63`.
fn punctured_codeword_len(code: &QraCode) -> usize {
    match code.code_type {
        QraCodeType::Normal | QraCodeType::Crc => code.N,
        QraCodeType::CrcPunctured => code.N - 1,
        QraCodeType::CrcPunctured2 => code.N - 2,
    }
}

/// Build per-symbol probability distributions from a wide-energy
/// spectrogram, accounting for tone-energy spread under a Gaussian or
/// Lorentzian fading model.
///
/// Equivalent of `q65_intrinsics_fastfading` from the C reference.
///
/// - `code`: the underlying QRA code (used only for `M`, the alphabet
///   size, and to derive the punctured codeword length).
/// - `intrinsics`: output buffer, length `M × punctured_codeword_len`.
/// - `energies_wide`: input spectrogram, length
///   `punctured_codeword_len × M × (2 + nBinsPerTone)`.
/// - `submode`: 0..=4 corresponding to Q65 sub-modes A..E.
/// - `b90_ts`: spread bandwidth × symbol period, dimensionless.
/// - `model`: Gaussian or Lorentzian fading shape.
/// - `decoder_es_no_metric`: the AWGN-channel decoder Es/No (linear)
///   from `q65_init` — Q65 uses 2.8 dB scaled by `m·R`.
///
/// Returns the [`FastFadingState`] that
/// [`esnodb_fast_fading`] needs to estimate the decoded SNR.
pub fn intrinsics_fast_fading(
    code: &QraCode,
    intrinsics: &mut [f32],
    energies_wide: &[f32],
    submode: u8,
    b90_ts: f32,
    model: FadingModel,
    decoder_es_no_metric: f32,
) -> FastFadingState {
    let big_m = code.M;
    let n_n = punctured_codeword_len(code);
    let bpt = bins_per_tone(submode);
    let bins_per_symbol = big_m * (2 + bpt);
    let total_bins = n_n * bins_per_symbol;

    assert_eq!(
        intrinsics.len(),
        big_m * n_n,
        "intrinsics_fast_fading: intrinsics buffer must be M*N"
    );
    assert_eq!(
        energies_wide.len(),
        total_bins,
        "intrinsics_fast_fading: energies_wide must be N*M*(2+nBinsPerTone)"
    );

    // Pick the calibrated weighting window for the requested spread.
    let hidx = table_index(b90_ts);
    let (hlen, taps) = match model {
        FadingModel::Gaussian => (GAUSS_LEN[hidx], GAUSS_TAPS[hidx]),
        FadingModel::Lorentzian => (LORENTZ_LEN[hidx], LORENTZ_TAPS[hidx]),
    };
    debug_assert_eq!(taps.len(), hlen, "fading table length mismatch");

    // Effective decoder Es/No: the AWGN metric scaled down by the
    // spread-induced degradation factor.
    // EsNoMetric = AWGN * 10^(8*log(B90)/log(240) / 10).
    let b90 = b90_ts / TS_QRA64;
    let degrade_db = 8.0 * b90.ln() / MAX_SPREAD_HZ.ln();
    let es_no_metric = decoder_es_no_metric * 10_f32.powf(degrade_db / 10.0);

    // Average bin energy = noise variance + a known signal-fraction
    // overestimate. The overestimate cancels out in `esnodb_fast_fading`.
    let mut noise_var = energies_wide.iter().sum::<f32>() / total_bins as f32;
    noise_var /= 1.0 + es_no_metric / bins_per_symbol as f32;

    // Pre-scale weights once for the whole codeword.
    let mut weights = vec![0.0_f32; hlen];
    for k in 0..hlen {
        let t = taps[k] * es_no_metric;
        weights[k] = t / (1.0 + t) / noise_var;
    }

    let hhsz = hlen - 1; // count of symmetric (off-centre) taps
    let hlast = 2 * hhsz; // index of the centre tap when traversing the full window

    let mut sym_offset = big_m; // central bin of tone 0 in symbol 0
    let mut ix_offset = 0usize;

    for _ in 0..n_n {
        let first_bin = sym_offset.wrapping_sub(hlen - 1); // first bin of the symbol's window
        let row = &mut intrinsics[ix_offset..ix_offset + big_m];
        let mut max_logp = 0.0_f32;

        for k in 0..big_m {
            let centre = first_bin + k * bpt;
            let mut acc = 0.0_f32;
            // Symmetric off-centre taps.
            for j in 0..hhsz {
                acc += weights[j] * (energies_wide[centre + j] + energies_wide[centre + hlast - j]);
            }
            // Centre tap.
            acc += weights[hhsz] * energies_wide[centre + hhsz];

            row[k] = acc;
            if acc > max_logp {
                max_logp = acc;
            }
        }

        // log → exp normalisation, clamped to avoid f32 overflow.
        let mut sum = 0.0_f32;
        for v in row.iter_mut() {
            let x = (*v - max_logp).clamp(-85.0, 85.0);
            let e = x.exp();
            *v = e;
            sum += e;
        }
        let inv = if sum > 0.0 { 1.0 / sum } else { 0.0 };
        for v in row.iter_mut() {
            *v *= inv;
        }
        // Belt-and-braces: enforce a strict probability vector. In
        // pathological inputs (all energies zero, NaN propagation, …)
        // pdmath::norm is a no-op safety net since we already
        // normalised above, but it also protects against drift.
        pdmath::norm(row);

        sym_offset += bins_per_symbol;
        ix_offset += big_m;
    }

    FastFadingState {
        es_no_metric,
        noise_var,
        n_bins_per_tone: bpt,
        n_bins_per_symbol: bins_per_symbol,
        weights,
    }
}

/// [`intrinsics_fast_fading`], fed **one symbol row at a time**: for a decoder that cannot hold the wide-energy
/// spectrogram (`N x M x (2 + bins per tone)` floats: 48 KB for sub-mode A, 290 KB for E) and so builds it a row at a
/// time, from whatever makes rows (an FFT, a store read forward) in the order of the codeword.
///
/// What is kept is the metric and nothing else: `N x M` `f32` (16 128 B), whatever the sub-mode. A row's logits depend
/// on that row alone (the `M`-bin padding at each end makes its fading window self-contained); only the noise
/// variance depends on all of them (the mean energy of every bin), so the rows are scored with the weights **not yet
/// divided by it**, and [`finish`](Self::finish) applies `1 / noise_var` to the `N x M` logits and normalises, as
/// `intrinsics_fast_fading` does. That is one pass over the rows, with no need to know the mean in advance.
///
/// The probabilities equal [`intrinsics_fast_fading`]'s to float rounding, not to the bit (a sum of products each
/// divided by `noise_var`, against the sum divided once). The returned [`FastFadingState`] (`noise_var`, the weights)
/// is the same to the bit: the energy is summed in the same order.
///
/// The state is what [`esnodb_fast_fading`] takes, and that estimate (SNR reporting, not decoding) reads the wide
/// energies again once the codeword is decoded: a caller that wants it makes the rows a second time.
///
/// ```
/// # use mfsk_core::fec::qra::{FadingModel, FastFadingStream};
/// # use mfsk_core::fec::qra15_65_64::QRA15_65_64_IRR_E23 as CODE;
/// let mut s = FastFadingStream::new(&CODE, 0, 0.05, FadingModel::Gaussian, 1.5);
/// let row = vec![1.0_f32; s.bins_per_symbol()];
/// for _ in 0..s.symbols() {
///     s.push_row(&row);
/// }
/// let mut intrinsics = vec![0.0_f32; s.symbols() * 64];
/// let _state = s.finish(&mut intrinsics);
/// ```
#[derive(Clone, Debug)]
pub struct FastFadingStream {
    big_m: usize,
    n_n: usize,
    bins_per_symbol: usize,
    bpt: usize,
    es_no_metric: f32,
    /// `t / (1 + t)` per tap: the weights before the division by the noise variance.
    raw_weights: Vec<f32>,
    /// The `N x M` logits so far, not yet divided by the noise variance.
    logits: Vec<f32>,
    rows: usize,
    energy_sum: f32,
}

impl FastFadingStream {
    /// Context for one codeword of `code` (Q65's `QRA15_65_64_IRR_E23`) at `submode` (0..=4: A..E), spread `b90_ts`
    /// and `model`; `decoder_es_no_metric` as for [`intrinsics_fast_fading`].
    pub fn new(
        code: &QraCode,
        submode: u8,
        b90_ts: f32,
        model: FadingModel,
        decoder_es_no_metric: f32,
    ) -> Self {
        let big_m = code.M;
        let n_n = punctured_codeword_len(code);
        let bpt = bins_per_tone(submode);
        let bins_per_symbol = big_m * (2 + bpt);
        let hidx = table_index(b90_ts);
        let (hlen, taps) = match model {
            FadingModel::Gaussian => (GAUSS_LEN[hidx], GAUSS_TAPS[hidx]),
            FadingModel::Lorentzian => (LORENTZ_LEN[hidx], LORENTZ_TAPS[hidx]),
        };
        debug_assert_eq!(taps.len(), hlen, "fading table length mismatch");
        let b90 = b90_ts / TS_QRA64;
        let degrade_db = 8.0 * b90.ln() / MAX_SPREAD_HZ.ln();
        let es_no_metric = decoder_es_no_metric * 10_f32.powf(degrade_db / 10.0);
        let raw_weights = taps
            .iter()
            .map(|&tap| {
                let t = tap * es_no_metric;
                t / (1.0 + t)
            })
            .collect();
        Self {
            big_m,
            n_n,
            bins_per_symbol,
            bpt,
            es_no_metric,
            raw_weights,
            logits: vec![0.0; big_m * n_n],
            rows: 0,
            energy_sum: 0.0,
        }
    }

    /// Symbols (rows) of the codeword.
    pub fn symbols(&self) -> usize {
        self.n_n
    }

    /// Length of a row: `M x (2 + bins per tone)`.
    pub fn bins_per_symbol(&self) -> usize {
        self.bins_per_symbol
    }

    /// Rows pushed so far.
    pub fn rows_pushed(&self) -> usize {
        self.rows
    }

    /// The next symbol's wide-energy row (`bins_per_symbol()` long), in codeword order.
    pub fn push_row(&mut self, row: &[f32]) {
        assert_eq!(
            row.len(),
            self.bins_per_symbol,
            "FastFadingStream::push_row: a row is M*(2+nBinsPerTone) bins"
        );
        assert!(
            self.rows < self.n_n,
            "FastFadingStream::push_row: more rows than the codeword has"
        );
        for &e in row {
            self.energy_sum += e;
        }
        let (big_m, bpt) = (self.big_m, self.bpt);
        let hlen = self.raw_weights.len();
        let hhsz = hlen - 1; // off-centre taps on each side
        let hlast = 2 * hhsz; // the full window's last index
        // the window of tone 0, centred on bin `big_m`; `big_m + 1 - hlen` and not `big_m - hlen + 1`: the longest table
        // window is 65 taps, and `64 - 65` overflows (a panic with overflow checks on, the dev profile)
        let first_bin = big_m + 1 - hlen;
        let out = &mut self.logits[self.rows * big_m..(self.rows + 1) * big_m];
        for (k, o) in out.iter_mut().enumerate() {
            let centre = first_bin + k * bpt;
            let mut acc = 0.0_f32;
            for j in 0..hhsz {
                acc += self.raw_weights[j] * (row[centre + j] + row[centre + hlast - j]);
            }
            acc += self.raw_weights[hhsz] * row[centre + hhsz];
            *o = acc;
        }
        self.rows += 1;
    }

    /// Normalise: divide the logits by the noise variance (the mean energy of every bin pushed, as
    /// [`intrinsics_fast_fading`]), and write each symbol's probabilities to `intrinsics` (`N x M`). Panics if
    /// not every row was pushed.
    pub fn finish(self, intrinsics: &mut [f32]) -> FastFadingState {
        assert_eq!(
            self.rows, self.n_n,
            "FastFadingStream::finish: {} of {} rows pushed",
            self.rows, self.n_n
        );
        assert_eq!(
            intrinsics.len(),
            self.big_m * self.n_n,
            "FastFadingStream::finish: intrinsics buffer must be M*N"
        );
        let total_bins = self.n_n * self.bins_per_symbol;
        let mut noise_var = self.energy_sum / total_bins as f32;
        noise_var /= 1.0 + self.es_no_metric / self.bins_per_symbol as f32;
        let inv_var = 1.0 / noise_var;
        for (row, logits) in intrinsics
            .chunks_exact_mut(self.big_m)
            .zip(self.logits.chunks_exact(self.big_m))
        {
            let mut max_logp = 0.0_f32;
            for (v, &l) in row.iter_mut().zip(logits) {
                let acc = l * inv_var;
                *v = acc;
                if acc > max_logp {
                    max_logp = acc;
                }
            }
            let mut sum = 0.0_f32;
            for v in row.iter_mut() {
                let x = (*v - max_logp).clamp(-85.0, 85.0);
                let e = x.exp();
                *v = e;
                sum += e;
            }
            let inv = if sum > 0.0 { 1.0 / sum } else { 0.0 };
            for v in row.iter_mut() {
                *v *= inv;
            }
            pdmath::norm(row);
        }
        let weights = self
            .raw_weights
            .iter()
            .map(|&w| w / noise_var)
            .collect::<Vec<_>>();
        FastFadingState {
            es_no_metric: self.es_no_metric,
            noise_var,
            n_bins_per_tone: self.bpt,
            n_bins_per_symbol: self.bins_per_symbol,
            weights,
        }
    }
}

/// Estimate the decoded codeword's Es/No (in dB) from the same wide
/// spectrogram passed to [`intrinsics_fast_fading`] plus the decoded
/// hard-decision codeword.
///
/// Equivalent of `q65_esnodb_fastfading`. The result is mostly useful
/// for SNR reporting; the decoder itself does not need it.
pub fn esnodb_fast_fading(
    state: &FastFadingState,
    code: &QraCode,
    decoded_codeword: &[i32],
    energies_wide: &[f32],
) -> f32 {
    let big_m = code.M;
    let n_n = punctured_codeword_len(code);
    let bpt = state.n_bins_per_tone;
    let bins_per_symbol = state.n_bins_per_symbol;
    let n_weights = state.weights.len();
    let n_tot_weights = 2 * n_weights - 1;

    assert_eq!(
        decoded_codeword.len(),
        n_n,
        "esnodb_fast_fading: decoded codeword must be of punctured length"
    );
    assert_eq!(
        energies_wide.len(),
        n_n * bins_per_symbol,
        "esnodb_fast_fading: energies_wide length"
    );

    // Sum the energies of the decoded tones over the whole window.
    let mut es_plus_w_no = 0.0_f32;
    let mut sym_offset = big_m; // central bin of tone 0 in symbol 0
    for &y in decoded_codeword {
        let centre = sym_offset + (y as usize) * bpt;
        let first_bin = centre - (n_weights - 1);
        for j in 0..n_tot_weights {
            es_plus_w_no += energies_wide[first_bin + j];
        }
        sym_offset += bins_per_symbol;
    }
    es_plus_w_no /= n_n as f32;

    // Invert the relation that links measured energy to true Es/No,
    // following the C reference's algebra. `u` is the measured energy
    // normalised by the (overestimated) noise variance; clamping it
    // to `nTotWeights + 0.316` floors the reported Es/No at ~-5 dB
    // (matches the WSJT-X convention).
    let u = es_plus_w_no / (state.noise_var * (1.0 + state.es_no_metric / bins_per_symbol as f32));
    let u_floor = n_tot_weights as f32 + 0.316;
    let u = u.max(u_floor);
    let es_no = (u - n_tot_weights as f32) / (1.0 - u / bins_per_symbol as f32);
    10.0 * es_no.log10()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;

    #[test]
    fn table_index_clamps_to_range() {
        // Tiny B90 → hidx clamps to 0.
        assert_eq!(table_index(1e-6), 0);
        // Huge B90 → hidx clamps to 63.
        assert_eq!(table_index(1e6), 63);
    }

    #[test]
    fn table_index_matches_c_reference_pivot_points() {
        // Pivot 1: B90Ts = TS_QRA64 (i.e. B90 = 1) → log(1)/log(1.09) - 0.499 = -0.499
        // → cast to int = 0 (clamped).
        assert_eq!(table_index(TS_QRA64), 0);
        // Pivot 2: a B90Ts that puts log(B90)/log(1.09) at 5 →
        // hidx = 5 - 0.499 → 4.
        let log109 = 1.09_f32.ln();
        let b90_for_5 = (5.0 * log109).exp(); // B90 ≈ 1.539
        assert_eq!(table_index(TS_QRA64 * b90_for_5), 4);
    }

    #[test]
    fn fading_tables_have_nonzero_lengths_and_match() {
        assert_eq!(GAUSS_LEN.len(), 64);
        assert_eq!(LORENTZ_LEN.len(), 64);
        for i in 0..64 {
            assert!(GAUSS_LEN[i] >= 2, "GAUSS_LEN[{i}] suspiciously small");
            assert_eq!(
                GAUSS_TAPS[i].len(),
                GAUSS_LEN[i],
                "GAUSS shape mismatch at {i}"
            );
            assert_eq!(
                LORENTZ_TAPS[i].len(),
                LORENTZ_LEN[i],
                "LORENTZ shape mismatch at {i}"
            );
            for &tap in GAUSS_TAPS[i] {
                assert!(tap.is_finite() && tap >= 0.0, "GAUSS tap negative or NaN");
            }
            for &tap in LORENTZ_TAPS[i] {
                assert!(tap.is_finite() && tap >= 0.0, "LORENTZ tap negative or NaN");
            }
        }
        // Largest table is the last entry, length 65 in both files.
        assert_eq!(GAUSS_LEN[63], 65);
        assert_eq!(LORENTZ_LEN[63], 65);
    }

    /// Build a wide-energy buffer with a single perfect tone per
    /// symbol: ε at every bin except the central bin of tone `c[k]`,
    /// which gets `peak`. Useful for sanity-checking the metric.
    fn synthetic_wide_energies(channel: &[i32], submode: u8, peak: f32, eps: f32) -> Vec<f32> {
        let big_m = 64;
        let bpt = 1usize << submode;
        let bins_per_symbol = big_m * (2 + bpt);
        let n_n = channel.len();
        let mut e = vec![eps; n_n * bins_per_symbol];
        for (k, &sym) in channel.iter().enumerate() {
            let sym_offset = k * bins_per_symbol + big_m;
            let centre = sym_offset + (sym as usize) * bpt;
            e[centre] = peak;
        }
        e
    }

    /// Pseudo-random wide energies (a fixed LCG): every bin different, so no row can be mistaken for another.
    fn noisy_wide_energies(n_bins: usize, seed: u32) -> Vec<f32> {
        let mut x = seed;
        (0..n_bins)
            .map(|_| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                0.05 + (x >> 8) as f32 / (1u32 << 24) as f32
            })
            .collect()
    }

    /// The streaming metric is `intrinsics_fast_fading`: probabilities to float rounding, `noise_var` and the weights to
    /// the bit, for every sub-mode, both models and spreads over the whole of the table.
    #[test]
    fn the_stream_is_the_whole_spectrogram_metric() {
        let code = &QRA15_65_64_IRR_E23;
        let (big_m, n_n) = (64usize, 63usize);
        for submode in 0..=4u8 {
            let bins_per_symbol = big_m * (2 + (1usize << submode));
            let mut energies = noisy_wide_energies(n_n * bins_per_symbol, 7 + submode as u32);
            // a few strong bins, so that the rows are not all alike in their softmax
            for k in 0..n_n {
                let b = k * bins_per_symbol + big_m + (k * 5 % 64) * (1usize << submode);
                energies[b] = 40.0;
            }
            for model in [FadingModel::Gaussian, FadingModel::Lorentzian] {
                for spread in [0.05_f32, 1.0, 3.0, 20.0, 400.0] {
                    let b90_ts = spread * TS_QRA64;
                    let mut want = vec![0.0_f32; big_m * n_n];
                    let ws = intrinsics_fast_fading(
                        code, &mut want, &energies, submode, b90_ts, model, 1.5,
                    );
                    let mut s = FastFadingStream::new(code, submode, b90_ts, model, 1.5);
                    for row in energies.chunks(bins_per_symbol) {
                        s.push_row(row);
                    }
                    let mut got = vec![0.0_f32; big_m * n_n];
                    let gs = s.finish(&mut got);
                    let worst = want
                        .iter()
                        .zip(&got)
                        .map(|(a, b)| (a - b).abs())
                        .fold(0.0_f32, f32::max);
                    assert!(
                        worst < 1e-4,
                        "sub-mode {submode} {model:?} spread {spread}: worst difference {worst}"
                    );
                    assert_eq!(gs.noise_var.to_bits(), ws.noise_var.to_bits());
                    assert_eq!(gs.es_no_metric.to_bits(), ws.es_no_metric.to_bits());
                    assert_eq!(gs.weights.len(), ws.weights.len());
                    for (a, b) in gs.weights.iter().zip(&ws.weights) {
                        assert_eq!(a.to_bits(), b.to_bits());
                    }
                }
            }
        }
    }

    /// A stream that is not given the whole codeword is an error, not a metric made of what there was.
    #[test]
    #[should_panic(expected = "rows pushed")]
    fn finish_before_every_row_is_pushed_panics() {
        let code = &QRA15_65_64_IRR_E23;
        let mut s = FastFadingStream::new(code, 0, 0.05, FadingModel::Gaussian, 1.5);
        let row = vec![1.0_f32; s.bins_per_symbol()];
        s.push_row(&row);
        let mut out = vec![0.0_f32; 64 * 63];
        let _ = s.finish(&mut out);
    }

    #[test]
    #[should_panic(expected = "a row is")]
    fn a_row_of_the_wrong_length_panics() {
        let code = &QRA15_65_64_IRR_E23;
        let mut s = FastFadingStream::new(code, 0, 0.05, FadingModel::Gaussian, 1.5);
        s.push_row(&[1.0; 10]);
    }

    #[test]
    fn intrinsics_concentrate_on_correct_tone_for_clean_input() {
        // With a near-perfect tone per symbol and a tight fading
        // window, each row of the intrinsic should peak strongly on
        // the transmitted symbol.
        let code = &QRA15_65_64_IRR_E23;
        let n_n = 63usize;
        let big_m = 64usize;
        let channel: Vec<i32> = (0..n_n as i32).map(|i| (i * 7 + 3) % 64).collect();
        let energies = synthetic_wide_energies(&channel, 0, 100.0, 0.01);
        let mut intr = vec![0.0_f32; big_m * n_n];

        let _state = intrinsics_fast_fading(
            code,
            &mut intr,
            &energies,
            0,
            0.05,
            FadingModel::Gaussian,
            1.5,
        );

        for k in 0..n_n {
            let row = &intr[big_m * k..big_m * (k + 1)];
            let argmax = row
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .unwrap()
                .0 as i32;
            assert_eq!(argmax, channel[k], "row {k} argmax wrong");
            // Probability vector must sum to ~1.
            let s: f32 = row.iter().sum();
            assert!((s - 1.0).abs() < 1e-3, "row {k} sum = {s}, not normalised");
        }
    }

    #[test]
    fn intrinsics_normalise_each_row_to_unit_sum() {
        // Even on noise-only input every row must be a valid probability vector.
        let code = &QRA15_65_64_IRR_E23;
        let n_n = 63usize;
        let big_m = 64usize;
        let bpt = 4usize; // sub-mode C
        let bins_per_symbol = big_m * (2 + bpt);
        let energies: Vec<f32> = (0..n_n * bins_per_symbol)
            .map(|i| (i % 17) as f32 + 0.1)
            .collect();
        let mut intr = vec![0.0_f32; big_m * n_n];

        intrinsics_fast_fading(
            code,
            &mut intr,
            &energies,
            2,
            0.5,
            FadingModel::Gaussian,
            1.5,
        );

        for k in 0..n_n {
            let row = &intr[big_m * k..big_m * (k + 1)];
            let s: f32 = row.iter().sum();
            assert!((s - 1.0).abs() < 1e-3, "row {k} not normalised: sum = {s}");
            for &p in row {
                assert!(p >= 0.0 && p.is_finite(), "row {k} bad value {p}");
            }
        }
    }

    #[test]
    fn esnodb_returns_finite_value_above_floor() {
        let code = &QRA15_65_64_IRR_E23;
        let n_n = 63usize;
        let big_m = 64usize;
        let channel: Vec<i32> = (0..n_n as i32).map(|i| (i * 11 + 5) % 64).collect();
        let energies = synthetic_wide_energies(&channel, 1, 100.0, 0.05);
        let mut intr = vec![0.0_f32; big_m * n_n];

        let state = intrinsics_fast_fading(
            code,
            &mut intr,
            &energies,
            1,
            0.3,
            FadingModel::Gaussian,
            1.5,
        );
        let snr_db = esnodb_fast_fading(&state, code, &channel, &energies);
        assert!(snr_db.is_finite(), "esnodb returned non-finite: {snr_db}");
        // Floor is roughly -5 dB; clean signal must come out above it.
        assert!(snr_db > -10.0, "esnodb absurdly low: {snr_db}");
    }
}
