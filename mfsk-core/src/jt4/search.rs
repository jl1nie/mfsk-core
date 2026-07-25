//! JT4 sync acquisition on the WSJT-X half-symbol time grid.
//!
//! Every JT4 symbol carries a known sync bit in the tone LSB. The unknown
//! data bit only selects the lower or upper member of the same parity pair,
//! so acquisition can compare the strongest expected-parity tone with the
//! strongest opposite-parity tone without knowing the message. The
//! synchronizer mirrors pinned WSJT-X `ps4`, `xcor4`, and `slope`: rectangular
//! one-symbol FFTs with an equal zero pad, half-symbol stepping, optional
//! triangular frequency smoothing, and a detrended/RMS-normalised sync CCF.

use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Ordering;

use num_complex::Complex;
use rustfft::FftPlanner;

use super::{JT4_BAUD, JT4_SYNC_VECTOR, Jt4Submode, SyncPolarity};

#[derive(Copy, Clone, Debug)]
pub struct SearchParams {
    pub min_frequency_hz: f32,
    pub max_frequency_hz: f32,
    pub time_tolerance_s: f32,
    pub max_candidates: usize,
    pub min_score: f32,
}

impl Default for SearchParams {
    fn default() -> Self {
        Self {
            min_frequency_hz: 100.0,
            max_frequency_hz: 2_700.0,
            // Pinned WSJT-X searches lags -5..59, which accommodates the
            // large EME propagation/clock offsets present in its JT4
            // reference recordings.
            time_tolerance_s: 6.0,
            max_candidates: 12,
            min_score: 1.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SyncCandidate {
    pub start_sample: usize,
    pub base_frequency_hz: f32,
    pub score: f32,
    pub polarity: SyncPolarity,
}

fn candidate_cmp(left: &SyncCandidate, right: &SyncCandidate) -> Ordering {
    right
        .score
        .partial_cmp(&left.score)
        .unwrap_or(Ordering::Equal)
}

fn normalise_ccf(ccf: &mut [f64], peak: usize) -> bool {
    let mut count = 0.0f64;
    let mut sum_x = 0.0f64;
    let mut sum_y = 0.0f64;
    let mut sum_x2 = 0.0f64;
    let mut sum_xy = 0.0f64;
    for (index, &value) in ccf.iter().enumerate() {
        if index.abs_diff(peak) <= 4 {
            continue;
        }
        let x = (index + 1) as f64;
        count += 1.0;
        sum_x += x;
        sum_y += value;
        sum_x2 += x * x;
        sum_xy += x * value;
    }
    let determinant = count * sum_x2 - sum_x * sum_x;
    if count < 3.0 || determinant.abs() <= f64::EPSILON {
        return false;
    }
    let intercept = (sum_x2 * sum_y - sum_x * sum_xy) / determinant;
    let slope = (count * sum_xy - sum_x * sum_y) / determinant;

    let mut square_sum = 0.0f64;
    let mut rms_count = 0usize;
    for (index, value) in ccf.iter_mut().enumerate() {
        *value -= intercept + slope * (index + 1) as f64;
        if index.abs_diff(peak) > 4 {
            square_sum += *value * *value;
            rms_count += 1;
        }
    }
    if rms_count == 0 {
        return false;
    }
    let rms = (square_sum / rms_count as f64).sqrt();
    if !rms.is_finite() || rms <= f64::EPSILON {
        return false;
    }
    for value in ccf {
        *value /= rms;
    }
    true
}

fn smoothed_tone_power(spectrum: &[f32], bin: usize, chip_width: usize) -> Option<f64> {
    if chip_width == 1 {
        return spectrum.get(bin).copied().map(f64::from);
    }
    let half_width = (chip_width / 2).max(1) as isize;
    let mut power = 0.0f64;
    let mut weight_sum = 0.0f64;
    for offset in (-half_width + 1)..half_width {
        let shifted = bin.checked_add_signed(offset)?;
        let weight = (half_width - offset.abs()) as f64 / chip_width as f64;
        power += f64::from(*spectrum.get(shifted)?) * weight;
        weight_sum += weight;
    }
    (weight_sum > 0.0).then_some(power / weight_sum.sqrt())
}

/// Find likely JT4 start/frequency alignments.
pub fn coarse_search(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    submode: Jt4Submode,
    params: &SearchParams,
) -> Vec<SyncCandidate> {
    if sample_rate == 0
        || params.max_candidates == 0
        || params.min_frequency_hz < 0.0
        || params.max_frequency_hz <= params.min_frequency_hz
        || !params.time_tolerance_s.is_finite()
        || params.time_tolerance_s < 0.0
    {
        return Vec::new();
    }

    let samples_per_symbol = sample_rate as f64 / JT4_BAUD as f64;
    let half_symbol = samples_per_symbol / 2.0;
    let tolerance_samples = (params.time_tolerance_s as f64 * sample_rate as f64).round() as usize;
    let first_start = nominal_start_sample.saturating_sub(tolerance_samples);
    let last_start = nominal_start_sample
        .saturating_add(tolerance_samples)
        .min(audio.len().saturating_sub(1));
    if first_start > last_start {
        return Vec::new();
    }
    let first_start_step = (first_start as f64 / half_symbol).ceil() as usize;
    let last_start_step = (last_start as f64 / half_symbol).floor() as usize;
    if first_start_step > last_start_step {
        return Vec::new();
    }
    let start_steps = last_start_step - first_start_step + 1;

    let symbol_samples = samples_per_symbol.round() as usize;
    let nfft = symbol_samples * 2;
    let available_spectrum_steps = if audio.len() < symbol_samples {
        0
    } else {
        ((audio.len() - symbol_samples) as f64 / half_symbol).floor() as usize + 1
    };
    if available_spectrum_steps == 0 {
        return Vec::new();
    }
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nfft);
    let mut scratch = vec![Complex::new(0.0f32, 0.0f32); fft.get_inplace_scratch_len()];
    let mut buffer = vec![Complex::new(0.0f32, 0.0f32); nfft];
    let mut spectra: Vec<Vec<f32>> = Vec::with_capacity(available_spectrum_steps);

    for step in 0..available_spectrum_steps {
        let start = (step as f64 * half_symbol).round() as usize;
        let end = start.saturating_add(symbol_samples);
        if end > audio.len() {
            break;
        }
        buffer.fill(Complex::new(0.0, 0.0));
        for index in 0..symbol_samples {
            // WSJT-X `ps4` uses one rectangular symbol followed by an
            // equally long zero pad.
            buffer[index].re = audio[start + index];
        }
        fft.process_with_scratch(&mut buffer, &mut scratch);
        spectra.push(
            buffer[..nfft / 2]
                .iter()
                .map(|value| value.norm_sqr())
                .collect(),
        );
    }

    let bin_hz = sample_rate as f32 / nfft as f32;
    let spacing_bins = (JT4_BAUD * submode.spacing_multiplier() as f32 / bin_hz).round() as usize;
    let first_bin = (params.min_frequency_hz / bin_hz).ceil() as usize;
    let last_bin = ((params.max_frequency_hz / bin_hz).floor() as usize)
        .min(nfft / 2 - 1)
        .saturating_sub(3 * spacing_bins);
    if first_bin > last_bin || spacing_bins == 0 {
        return Vec::new();
    }

    let maximum_chips = submode.spacing_multiplier() as usize;
    let chip_widths = [1usize, 2, 4, 9, 18, 36, 72];
    let mut measurements = Vec::new();
    let mut strongest_raw_peak = 0.0f64;
    for chip_width in chip_widths
        .into_iter()
        .filter(|width| *width <= maximum_chips)
    {
        for base_bin in first_bin..=last_bin {
            let mut feature = Vec::with_capacity(spectra.len());
            for spectrum in &spectra {
                let even0 = smoothed_tone_power(spectrum, base_bin, chip_width);
                let odd0 = smoothed_tone_power(spectrum, base_bin + spacing_bins, chip_width);
                let even1 = smoothed_tone_power(spectrum, base_bin + 2 * spacing_bins, chip_width);
                let odd1 = smoothed_tone_power(spectrum, base_bin + 3 * spacing_bins, chip_width);
                let Some((even0, odd0, even1, odd1)) = even0
                    .zip(odd0)
                    .zip(even1)
                    .zip(odd1)
                    .map(|(((even0, odd0), even1), odd1)| (even0, odd0, even1, odd1))
                else {
                    feature.clear();
                    break;
                };
                feature.push(odd0.max(odd1) - even0.max(even1));
            }
            if feature.is_empty() {
                continue;
            }

            let mut ccf = Vec::with_capacity(start_steps);
            for relative_start in 0..start_steps {
                let start_step = first_start_step + relative_start;
                let mut correlation = 0.0f64;
                for (symbol, &sync) in JT4_SYNC_VECTOR.iter().enumerate() {
                    let step = start_step + symbol * 2;
                    if let Some(&measurement) = feature.get(step) {
                        correlation += measurement * f64::from(2 * sync as i8 - 1);
                    }
                }
                ccf.push(correlation);
            }
            let Some((peak, &raw_peak)) = ccf.iter().enumerate().max_by(|(_, left), (_, right)| {
                left.abs()
                    .partial_cmp(&right.abs())
                    .unwrap_or(Ordering::Equal)
            }) else {
                continue;
            };
            if !normalise_ccf(&mut ccf, peak) {
                continue;
            }
            strongest_raw_peak = strongest_raw_peak.max(raw_peak.abs());
            measurements.push((
                SyncCandidate {
                    start_sample: ((first_start_step + peak) as f64 * half_symbol).round() as usize,
                    base_frequency_hz: base_bin as f32 * bin_hz,
                    score: ccf[peak].abs() as f32,
                    polarity: if raw_peak >= 0.0 {
                        SyncPolarity::Normal
                    } else {
                        SyncPolarity::Inverted
                    },
                },
                raw_peak.abs(),
            ));
        }
    }
    let mut raw = measurements
        .into_iter()
        .filter_map(|(mut candidate, raw_peak)| {
            // Detrended CCF normalisation is intentionally scale-free. With
            // exact digital silence, round-off-only bins can otherwise acquire
            // a huge score; weight by raw CCF strength to reject those bins.
            candidate.score *= (raw_peak / (strongest_raw_peak + f64::EPSILON)) as f32;
            (candidate.score >= params.min_score).then_some(candidate)
        })
        .collect::<Vec<_>>();
    raw.sort_by(candidate_cmp);

    let time_dedupe = (samples_per_symbol / 2.0).round() as i64;
    let frequency_dedupe = bin_hz * 2.0;
    let mut selected: Vec<SyncCandidate> = Vec::new();
    for candidate in raw {
        if selected.iter().any(|previous| {
            (previous.start_sample as i64 - candidate.start_sample as i64).abs() <= time_dedupe
                && (previous.base_frequency_hz - candidate.base_frequency_hz).abs()
                    <= frequency_dedupe
        }) {
            continue;
        }
        selected.push(candidate);
        if selected.len() == params.max_candidates {
            break;
        }
    }
    selected
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jt4::tx::synthesize_standard;

    #[test]
    fn finds_delayed_clean_signal() {
        let prefix = 9_000usize;
        let signal =
            synthesize_standard("CQ", "K1ABC", "FN42", Jt4Submode::A, 12_000, 1_200.0, 0.3)
                .expect("pack");
        let mut audio = vec![0.0; prefix];
        audio.extend(signal);
        let candidates = coarse_search(
            &audio,
            12_000,
            12_000,
            Jt4Submode::A,
            &SearchParams {
                min_frequency_hz: 1_195.0,
                max_frequency_hz: 1_215.0,
                time_tolerance_s: 1.5,
                ..SearchParams::default()
            },
        );
        assert!(!candidates.is_empty());
        let best = candidates[0];
        assert!(
            (best.start_sample as i64 - prefix as i64).abs() <= 700,
            "start was {}, expected {prefix}; {candidates:?}",
            best.start_sample,
        );
        assert!(
            (best.base_frequency_hz - 1_200.0).abs() <= 2.3,
            "frequency was {}; {candidates:?}",
            best.base_frequency_hz,
        );
    }
}
