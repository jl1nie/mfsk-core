//! 12 kHz audio → 375 Hz complex baseband centered on 1500 Hz.
//!
//! Faithful port of WSJT-X `wsprd.c::readwavfile` (lines 108-187, the
//! `ntrmin == 2` branch). Architecture-equivalent decimation lets the
//! per-symbol demod operate on `NSPS_BASEBAND = 256` complex samples
//! per symbol, matching wsprd's `noncoherent_sequence_detection`
//! geometry. Without this, our pipeline runs at 12 kHz throughout
//! (32× more expensive FFTs) and can't take advantage of wsprd's
//! coherent block detection for low-SNR recall.
//!
//! Output `(idat, qdat)` are `NFFT2 = 46080` samples at `BASEBAND_RATE`
//! Hz, representing the audio mixed by `-1500 Hz` (i.e. the WSPR
//! band, originally 1400-1600 Hz audio, sits as `-100..+100 Hz` in
//! the baseband).

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;

use crate::engine::dsp::downsample::with_default_planner;

/// Baseband sample rate. Matches wsprd `dt = 1.0/375.0` throughout.
pub const BASEBAND_RATE: f32 = 375.0;

/// Output sample count from one decimation. Matches wsprd `nfft2`.
pub const NFFT2: usize = 46080;

/// Large forward-FFT size. Matches wsprd `nfft1 = nfft2 * 32`.
pub const NFFT1: usize = NFFT2 * 32; // 1_474_560

/// Center frequency of the WSPR band that the decimation pulls down
/// to DC. Matches wsprd `i0 = 1500/df` round-trip; standard WSPR
/// dial-relative offset.
pub const CENTER_HZ: f32 = 1500.0;

/// Maximum input samples consumed. Matches wsprd `npoints = 114*12000`
/// for the WSPR-2 / 120 s slot. Excess samples are ignored;
/// shorter recordings get zero-padded up to `NFFT1`.
pub const NPOINTS_MAX: usize = 114 * 12_000;

/// Decimate 12 kHz f32 audio to 375 Hz complex baseband, centered on
/// [`CENTER_HZ`]. Returns `(idat, qdat)` each of length [`NFFT2`].
///
/// Algorithm (exact port of `readwavfile` lines 154-187):
/// 1. Zero-pad / truncate audio to `NFFT1` samples
/// 2. Real-input forward FFT of size `NFFT1`
/// 3. Re-pack `NFFT2` bins centered on `i0 = 1500/df`, with negative
///    half-spectrum at the high end (`fftin[NFFT2-i] = fftout[i0-i]`)
/// 4. Complex inverse FFT of size `NFFT2`
/// 5. Scale by `1/1000` (matches wsprd's `idat[i] = fftout[i].re/1000`)
pub fn decimate_to_baseband(audio: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let mut buf: Vec<Complex<f32>> = Vec::with_capacity(NFFT1);
    let n_in = audio.len().min(NPOINTS_MAX);
    for &s in &audio[..n_in] {
        buf.push(Complex::new(s, 0.0));
    }
    buf.resize(NFFT1, Complex::new(0.0, 0.0));

    with_default_planner(|planner| planner.plan_forward(NFFT1)).process(&mut buf);

    let df = 12_000.0 / NFFT1 as f32;
    let i0 = (CENTER_HZ / df).round() as usize;
    let nh2 = NFFT2 / 2;

    let mut fftin: Vec<Complex<f32>> = vec![Complex::new(0.0, 0.0); NFFT2];
    for i in 0..NFFT2 {
        // wsprd `wsprd.c:172-177`:
        //   j = i0 + i; if i > nh2 then j -= nfft2;
        // The wraparound puts negative-freq half of the WSPR band
        // at the high end of `fftin`, which the inverse FFT then
        // unwraps into a contiguous time-domain complex baseband.
        let j = if i > nh2 {
            i0.wrapping_add(i).wrapping_sub(NFFT2)
        } else {
            i0 + i
        };
        if j < buf.len() {
            fftin[i] = buf[j];
        }
    }

    with_default_planner(|planner| planner.plan_inverse(NFFT2)).process(&mut fftin);

    const NORM: f32 = 1.0 / 1000.0;
    let mut idat = vec![0.0f32; NFFT2];
    let mut qdat = vec![0.0f32; NFFT2];
    for i in 0..NFFT2 {
        idat[i] = fftin[i].re * NORM;
        qdat[i] = fftin[i].im * NORM;
    }
    (idat, qdat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimate_returns_expected_length_and_finite_samples() {
        // Feed a long 1500 Hz cosine; just check shape + finite output.
        let fs = 12_000.0;
        let f = 1500.0;
        let n = NPOINTS_MAX;
        let audio: Vec<f32> = (0..n)
            .map(|k| (2.0 * core::f32::consts::PI * f * k as f32 / fs).cos())
            .collect();
        let (idat, qdat) = decimate_to_baseband(&audio);
        assert_eq!(idat.len(), NFFT2);
        assert_eq!(qdat.len(), NFFT2);
        let max_mag = idat
            .iter()
            .zip(qdat.iter())
            .map(|(&i, &q)| (i * i + q * q).sqrt())
            .fold(0f32, f32::max);
        assert!(
            max_mag.is_finite() && max_mag > 0.0,
            "baseband output should carry the 1500 Hz tone (max mag {})",
            max_mag
        );
    }

    #[test]
    fn decimate_centres_carrier_for_full_length_tone() {
        // 1500 Hz cosine for full 114 s — after mix to DC, the
        // baseband i/q rotates very slowly, so the running mean of
        // (i² + q²) over the active window should be roughly constant
        // (= constant envelope of a steady-tone mixed to DC).
        let fs = 12_000.0;
        let f = 1500.0;
        let n = NPOINTS_MAX;
        let audio: Vec<f32> = (0..n)
            .map(|k| (2.0 * core::f32::consts::PI * f * k as f32 / fs).cos())
            .collect();
        let (idat, qdat) = decimate_to_baseband(&audio);
        // Sample baseband mid-stream: should have non-trivial magnitude.
        let mid = NFFT2 / 2;
        let mag_mid = (idat[mid] * idat[mid] + qdat[mid] * qdat[mid]).sqrt();
        let mag_q = (idat[NFFT2 / 4] * idat[NFFT2 / 4] + qdat[NFFT2 / 4] * qdat[NFFT2 / 4]).sqrt();
        assert!(
            mag_mid > 0.0 && mag_q > 0.0,
            "baseband should not be zero mid-stream; mag_mid={} mag_q={}",
            mag_mid,
            mag_q
        );
    }
}
