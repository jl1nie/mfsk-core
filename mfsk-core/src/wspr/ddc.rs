// SPDX-License-Identifier: GPL-3.0-or-later
//! Streaming digital down-converter: 12 kHz real audio → 375 Hz
//! complex baseband, one chunk at a time.
//!
//! Functional replacement for [`super::baseband::decimate_to_baseband`],
//! which is the whole-slot FFT channelizer `wsprd`'s `readwavfile`
//! uses. That routine is exact and is what every WSPR number in this
//! crate was measured against — and it cannot run on an ESP32-S3:
//!
//! | | |
//! |---|---|
//! | `NFFT1` | 1 474 560 — not a power of two |
//! | its `Complex<f32>` buffer | **11.25 MiB**, on an 8 MB PSRAM part |
//! | `esp-dsp` FFT ceiling | 8 192 |
//!
//! So the embedded WSPR bench is fed a baseband baked on a host, and
//! every timing in `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`
//! is the *downstream* of a stage that does not exist on the device.
//! This module is that stage (issue #260).
//!
//! ## Why streaming is the point
//!
//! The conversion is cheap; what makes it worth doing incrementally is
//! that it runs **while the slot is still being captured**, so it costs
//! no post-slot wall-clock at all, and the 12 kHz slot never has to be
//! stored — 5.2 MiB as `f32`, 2.6 MiB as `i16`, against a decoder that
//! already wants 360 KiB of baseband and a 735 KB `ps`.
//!
//! ## Construction
//!
//! ```text
//! x[n] (12 kHz real)
//!   → × exp(-j2π·1500·n/12000)      period-8 table, no trig per sample
//!   → FIR low-pass, fc = 187.5 Hz   = Fs/(2·32), the decimation Nyquist
//!   → keep every 32nd sample        375 Hz complex
//! ```
//!
//! 1500 Hz is exactly `Fs/8`, so the mixer is an eight-entry lookup —
//! not multiplication-free (the table holds `±1`, `0`, `±√2/2`), but no
//! oscillator state and no `sin`/`cos` per sample.
//!
//! The filter is a single polyphase stage rather than a
//! decimate-by-8-then-4 cascade. A cascade is cheaper in multiplies,
//! but this one costs `NTAPS/32 ≈ 56` mult-adds per input sample per
//! channel — about 150 M for a full slot, well under a second on an
//! LX7 and spread across 120 s of capture — and one stage is markedly
//! easier to show equivalent to the reference, which is where the risk
//! actually is.
//!
//! ## Where it cannot match the reference, by construction
//!
//! Bin selection is a brick wall: the FFT channelizer keeps
//! `1500 ± 187.5 Hz` exactly and zeroes everything else. A FIR has a
//! transition band, so content in the last ~13 Hz below 187.5 Hz is
//! attenuated rather than passed flat, and out-of-band rejection is
//! finite instead of total. That is a real difference, and it is why
//! this ships with a differential test against the reference plus the
//! full recall/precision sweep rather than a bench timing.
//!
//! It is bounded by what the decoder downstream actually reads:
//! `coarse_baseband` works over `WORKING_BINS = 411` bins at
//! `375/512 Hz`, i.e. **±150 Hz**, which sits inside the flat passband.

use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::baseband::{NFFT1, NFFT2, NPOINTS_MAX};

/// Input rate.
pub const AUDIO_RATE_HZ: f32 = 12_000.0;

/// Decimation factor. `12 000 / 32 = 375`, matching
/// [`super::baseband::BASEBAND_RATE`].
pub const DECIM: usize = 32;

/// FIR length, odd for linear phase with an integer group delay.
///
/// Sets the transition width: ~`4/NTAPS` of the input rate, so ~27 Hz
/// here — the response is flat to ~174 Hz and into the stopband by
/// ~201 Hz, against a 187.5 Hz ideal cutoff. Chosen so the flat region
/// comfortably covers the ±150 Hz `coarse_baseband` reads, not to
/// chase the reference's brick wall, which no FIR reaches.
pub const NTAPS: usize = 1793;

/// Group delay in input samples. A linear-phase FIR delays by
/// `(N-1)/2`; the reference channelizer delays by nothing (bin
/// selection preserves the time origin), so this has to be taken back
/// out or every decode's DT shifts by 28 baseband samples.
pub const GROUP_DELAY: usize = (NTAPS - 1) / 2;

/// Scale that puts this module's output on the reference's amplitude
/// scale.
///
/// Derived, not fitted. For a real `A·cos(2π·1500·t)`,
/// `decimate_to_baseband` produces a bin of magnitude `A·NFFT1/2`,
/// its unnormalised inverse FFT turns a single bin into that same
/// constant in the time domain, and it then scales by `1/1000` — so
/// `A·NFFT1/2000`. Mixing and low-passing the same signal leaves
/// `A/2`. The ratio is `NFFT1/1000`.
pub const REFERENCE_GAIN: f32 = NFFT1 as f32 / 1000.0;

/// `exp(-j2π·1500·n/12000) = exp(-jπn/4)`, period 8.
fn mixer_table() -> [(f32, f32); 8] {
    let mut t = [(0.0f32, 0.0f32); 8];
    for (n, e) in t.iter_mut().enumerate() {
        let phi = -core::f32::consts::PI * n as f32 / 4.0;
        *e = (phi.cos(), phi.sin());
    }
    t
}

/// Windowed-sinc low-pass, cutoff `fc` normalised to the sample rate,
/// Blackman window, unit DC gain.
fn design_lowpass(ntaps: usize, fc_norm: f32) -> Vec<f32> {
    let mut h = vec![0.0f32; ntaps];
    let m = (ntaps - 1) as f32;
    let mut sum = 0.0f32;
    for (k, tap) in h.iter_mut().enumerate() {
        let x = k as f32 - m / 2.0;
        let sinc = if x.abs() < 1e-6 {
            2.0 * fc_norm
        } else {
            (2.0 * core::f32::consts::PI * fc_norm * x).sin() / (core::f32::consts::PI * x)
        };
        // Blackman: better stopband than Hamming (~-74 dB vs -53 dB),
        // which is the half that matters here — the passband is
        // oversized on purpose.
        let w = 0.42 - 0.5 * (2.0 * core::f32::consts::PI * k as f32 / m).cos()
            + 0.08 * (4.0 * core::f32::consts::PI * k as f32 / m).cos();
        *tap = sinc * w;
        sum += *tap;
    }
    for tap in h.iter_mut() {
        *tap /= sum;
    }
    h
}

/// Incremental 12 kHz → 375 Hz complex down-converter.
///
/// Feed audio with [`push`](Self::push) as it arrives; complex
/// baseband samples are appended to the caller's buffers. State is one
/// `NTAPS`-sample complex ring buffer (~14 KB) plus two counters —
/// nothing that scales with slot length.
pub struct StreamingDdc<'a> {
    /// Taps in **reverse** order, so the dot product walks history
    /// forwards and both operands are sequential.
    taps_rev: &'a mut [f32],
    mixer: [(f32, f32); 8],
    /// Linear (not circular) history, compacted rarely. A ring costs a
    /// wrap test per tap, which defeats unrolling on the one loop that
    /// matters; see [`Self::dot`].
    hist_i: &'a mut [f32],
    hist_q: &'a mut [f32],
    /// Live samples in `hist_*`.
    hist_len: usize,
    /// Index where the current `NTAPS` window begins, maintained
    /// incrementally rather than as `hist_len - NTAPS`.
    ///
    /// Not a micro-optimisation: the subtraction folds into an
    /// addressing offset of `-NTAPS*4 = -7168`, which the Xtensa
    /// backend of the `esp` Rust fork rejects outright —
    /// `rustc-LLVM ERROR: Cannot select: i32 = Constant<-7168>`. Host
    /// builds are unaffected, so this only shows up when cross-building.
    win_start: usize,
    /// Input samples consumed, for the mixer phase.
    n_in: usize,
    /// Input samples until the next output.
    to_next_out: usize,
}

/// History capacity, and the length [`StreamingDdc::new_in`] requires.
///
/// Compaction copies the live `NTAPS` window back to the front once the
/// buffer fills — every 512 samples here, amortising to ~3.5 floats
/// moved per input sample against ~56 mult-adds. A larger buffer would
/// amortise further; this is sized so the whole working set fits in a
/// scarce internal-DRAM budget (see [`StreamingDdc::new_in`]), which
/// matters more.
pub const HIST_LEN: usize = NTAPS + 512;

/// Working set the caller supplies to [`StreamingDdc::new_in`], owned
/// as `Vec`s.
///
/// The host default. Embedded wants these in internal DRAM instead —
/// see [`StreamingDdc::new_in`].
pub struct DdcBufs {
    taps: alloc::vec::Vec<f32>,
    hist_i: alloc::vec::Vec<f32>,
    hist_q: alloc::vec::Vec<f32>,
}

impl Default for DdcBufs {
    fn default() -> Self {
        Self::new()
    }
}

impl DdcBufs {
    pub fn new() -> Self {
        Self {
            taps: vec![0.0; NTAPS],
            hist_i: vec![0.0; HIST_LEN],
            hist_q: vec![0.0; HIST_LEN],
        }
    }

    /// Borrow the three buffers for [`StreamingDdc::new_in`].
    pub fn as_parts(&mut self) -> (&mut [f32], &mut [f32], &mut [f32]) {
        (&mut self.taps, &mut self.hist_i, &mut self.hist_q)
    }
}

impl<'a> StreamingDdc<'a> {
    /// Build over caller-supplied storage.
    ///
    /// `taps` must be [`NTAPS`] long and `hist_i`/`hist_q` [`HIST_LEN`];
    /// contents are overwritten. **Where the caller puts them is the
    /// point.** Every output reads all three in full — 1 793 taps plus
    /// a 1 793-sample window per channel — and on an ESP32-S3 anything
    /// over `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` lands in PSRAM, which
    /// measured 7.04 us/sample, about 15 cycles per mult-add. Same
    /// answer as FT8's `internal_pool`: keep the bulk wherever, put the
    /// hot working set in internal DRAM. ~25 KB total.
    pub fn new_in(taps: &'a mut [f32], hist_i: &'a mut [f32], hist_q: &'a mut [f32]) -> Self {
        assert_eq!(taps.len(), NTAPS, "taps must be NTAPS long");
        assert_eq!(hist_i.len(), HIST_LEN, "hist_i must be HIST_LEN long");
        assert_eq!(hist_q.len(), HIST_LEN, "hist_q must be HIST_LEN long");
        let designed = design_lowpass(NTAPS, 1.0 / (2.0 * DECIM as f32));
        // Index loop, not `.iter().rev()`: a reverse iterator over a
        // constant-length slice folds into a `-NTAPS*4` addressing
        // offset that the `esp` fork's Xtensa backend cannot select
        // (`rustc-LLVM ERROR: Cannot select: i32 = Constant<-7168>`).
        // Host builds compile it fine, so this only appears when
        // cross-building.
        let last = NTAPS - 1;
        for k in 0..NTAPS {
            taps[k] = designed[last - k];
        }
        // Pre-filled with the zeros the filter would have seen before
        // the stream started.
        hist_i[..NTAPS].fill(0.0);
        hist_q[..NTAPS].fill(0.0);
        Self {
            taps_rev: taps,
            mixer: mixer_table(),
            hist_i,
            hist_q,
            hist_len: NTAPS,
            win_start: 0,
            n_in: 0,
            // The first output is centred on input sample 0, which the
            // filter only sees once `GROUP_DELAY` more samples have
            // arrived.
            to_next_out: GROUP_DELAY + 1,
        }
    }

    /// Consume `audio`, appending any baseband samples it completed.
    ///
    /// Output sample `m` is centred on input sample `32·m`, i.e. the
    /// group delay is already removed — the caller's baseband index is
    /// on the same time base as the reference channelizer's.
    pub fn push(&mut self, audio: &[f32], out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        for &s in audio {
            let (c, sn) = self.mixer[self.n_in % 8];
            self.hist_i[self.hist_len] = s * c;
            self.hist_q[self.hist_len] = s * sn;
            self.hist_len += 1;
            self.win_start += 1;
            self.n_in += 1;

            self.to_next_out -= 1;
            if self.to_next_out == 0 {
                self.to_next_out = DECIM;
                let (i, q) = self.dot();
                out_i.push(i * REFERENCE_GAIN);
                out_q.push(q * REFERENCE_GAIN);
            }

            if self.hist_len == HIST_LEN {
                self.compact();
            }
        }
    }

    fn compact(&mut self) {
        // `win_start` is the same quantity, already maintained without
        // the constant-folded subtraction — see its field comment.
        let keep = self.win_start;
        self.hist_i.copy_within(keep..self.hist_len, 0);
        self.hist_q.copy_within(keep..self.hist_len, 0);
        self.hist_len = NTAPS;
        self.win_start = 0;
    }

    /// Flush the tail: feed `GROUP_DELAY` zeros so the last real input
    /// samples reach the centre of the filter.
    pub fn flush(&mut self, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        let zeros = vec![0.0f32; GROUP_DELAY];
        self.push(&zeros, out_i, out_q);
    }

    /// `Σ h_rev[k]·x[n-NTAPS+k]`, over both channels.
    ///
    /// Four partial sums per channel rather than one. The FIR is an
    /// accumulation chain and Xtensa's `fadd` has ~3-4 cycle latency
    /// against 1-cycle throughput, so a single accumulator spends most
    /// of its cycles waiting on itself; independent partials fill the
    /// issue slots. Same reason the embedded Goertzel runs
    /// sample-outer / tone-inner (`ft8::decode_block::
    /// fill_symbol_spectra_goertzel`, 2.46 s → 1.47 s) — there the
    /// independent chains are the eight tones, here they are the four
    /// partials.
    ///
    /// The window is contiguous and the taps pre-reversed, so both
    /// operands walk forwards and the loop has no wrap test to defeat
    /// unrolling.
    fn dot(&self) -> (f32, f32) {
        let a = self.win_start;
        let hi = &self.hist_i[a..a + NTAPS];
        let hq = &self.hist_q[a..a + NTAPS];
        let h = &self.taps_rev[..];

        let mut ai = [0.0f32; 4];
        let mut aq = [0.0f32; 4];
        let chunks = NTAPS / 4;
        for c in 0..chunks {
            let b = c * 4;
            for l in 0..4 {
                ai[l] += h[b + l] * hi[b + l];
                aq[l] += h[b + l] * hq[b + l];
            }
        }
        let mut si = ai[0] + ai[1] + ai[2] + ai[3];
        let mut sq = aq[0] + aq[1] + aq[2] + aq[3];
        // NTAPS is odd, so a remainder of up to 3 is left over.
        for k in chunks * 4..NTAPS {
            si += h[k] * hi[k];
            sq += h[k] * hq[k];
        }
        (si, sq)
    }
}

/// Block-mode convenience with [`decimate_to_baseband`]'s signature,
/// for A/B testing and for host callers that already hold a whole slot.
///
/// [`decimate_to_baseband`]: super::baseband::decimate_to_baseband
pub fn ddc_to_baseband(audio: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let n_in = audio.len().min(NPOINTS_MAX);
    let mut bufs = DdcBufs::new();
    let (t, hi, hq) = bufs.as_parts();
    let mut ddc = StreamingDdc::new_in(t, hi, hq);
    let mut idat = Vec::with_capacity(NFFT2);
    let mut qdat = Vec::with_capacity(NFFT2);
    ddc.push(&audio[..n_in], &mut idat, &mut qdat);
    ddc.flush(&mut idat, &mut qdat);
    // The reference always returns `NFFT2` samples because it
    // zero-pads to `NFFT1`; match that so the two are drop-in
    // substitutable.
    idat.resize(NFFT2, 0.0);
    qdat.resize(NFFT2, 0.0);
    idat.truncate(NFFT2);
    qdat.truncate(NFFT2);
    (idat, qdat)
}

#[cfg(test)]
mod tests {
    use super::super::baseband::{BASEBAND_RATE, CENTER_HZ, decimate_to_baseband};
    use super::*;

    /// Phase accumulated in `f64`, deliberately.
    ///
    /// `cos(2π·f·k/Fs)` evaluated in `f32` at `k ≈ 5·10^5` has an
    /// argument of ~5·10^5 radians, where `f32` carries ~0.03 rad of
    /// error — enough phase noise to spread a "pure" tone across the
    /// whole spectrum and put a floor of about −54 dB under any
    /// rejection measurement made with it. The filter designed here is
    /// −129 dB at the frequency the stopband test uses, so the fixture
    /// would have been the thing under test.
    fn tone(freq_hz: f32, amp: f32, n: usize) -> Vec<f32> {
        let w = 2.0 * core::f64::consts::PI * freq_hz as f64 / AUDIO_RATE_HZ as f64;
        (0..n)
            .map(|k| (amp as f64 * (w * k as f64).cos()) as f32)
            .collect()
    }

    #[test]
    fn output_shape_matches_the_reference() {
        let audio = tone(CENTER_HZ, 0.5, 240_000);
        let (i, q) = ddc_to_baseband(&audio);
        assert_eq!(i.len(), NFFT2);
        assert_eq!(q.len(), NFFT2);
        assert!(i.iter().all(|v| v.is_finite()));
        assert!(q.iter().all(|v| v.is_finite()));
    }

    /// A tone at the band centre lands at DC, and on the reference's
    /// amplitude scale — the point of `REFERENCE_GAIN` being derived
    /// rather than tuned.
    #[test]
    fn centre_tone_lands_at_dc_with_reference_amplitude() {
        let amp = 0.5f32;
        let n = 480_000; // 40 s, long enough to be past both transients
        let audio = tone(CENTER_HZ, amp, n);
        let (i, q) = ddc_to_baseband(&audio);

        // 480 000 input samples decimate to 15 000 outputs, and the
        // filter settles within `GROUP_DELAY/DECIM` = 28 of them.
        let mid = 7_000;
        let mag = (i[mid] * i[mid] + q[mid] * q[mid]).sqrt();
        let expect = amp / 2.0 * REFERENCE_GAIN;
        assert!(
            (mag - expect).abs() / expect < 0.02,
            "|y| = {mag}, expected ~{expect}"
        );
    }

    /// An offset tone becomes a complex exponential at the offset, with
    /// the sign convention the decoder expects (positive audio offset →
    /// positive baseband frequency).
    #[test]
    fn offset_tone_becomes_a_rotating_phasor_at_the_offset() {
        let offset = 40.0f32;
        let n = 480_000;
        let audio = tone(CENTER_HZ + offset, 0.5, n);
        let (i, q) = ddc_to_baseband(&audio);

        // Phase advance per baseband sample over a settled span.
        let a = 5_000usize;
        let b = a + 1000;
        let ph = |k: usize| q[k].atan2(i[k]);
        let mut d = ph(b) - ph(a);
        // Unwrap into (-pi, pi] per-sample by using the known span.
        let expect_total = 2.0 * core::f32::consts::PI * offset * (b - a) as f32 / BASEBAND_RATE;
        while d - expect_total > core::f32::consts::PI {
            d -= 2.0 * core::f32::consts::PI;
        }
        while expect_total - d > core::f32::consts::PI {
            d += 2.0 * core::f32::consts::PI;
        }
        let measured_hz = d / (2.0 * core::f32::consts::PI) * BASEBAND_RATE / (b - a) as f32;
        assert!(
            (measured_hz - offset).abs() < 0.5,
            "measured {measured_hz} Hz, expected {offset}"
        );
    }

    /// Out-of-band audio must not fold into the baseband. The reference
    /// zeroes it outright; this asserts the FIR's stopband is deep
    /// enough that what leaks through is far below anything the decoder
    /// would call a signal.
    #[test]
    fn out_of_band_tone_is_rejected() {
        let n = 480_000;
        let in_band = ddc_to_baseband(&tone(CENTER_HZ, 0.5, n));
        // 1500 + 400 Hz: outside the +-187.5 Hz keep-band, and placed
        // where decimation would alias it to -350 + 375 = +25 Hz if the
        // filter did nothing.
        let out_band = ddc_to_baseband(&tone(CENTER_HZ + 400.0, 0.5, n));

        let rms = |(i, q): &(Vec<f32>, Vec<f32>)| -> f32 {
            let span = 5_000..10_000;
            let s: f32 =
                span.clone().map(|k| i[k] * i[k] + q[k] * q[k]).sum::<f32>() / span.len() as f32;
            s.sqrt()
        };
        let db = 20.0 * (rms(&out_band) / rms(&in_band)).log10();
        assert!(db < -60.0, "out-of-band rejection only {db} dB");
    }

    /// The one that matters: against the reference channelizer on a
    /// signal shaped like the band it will actually see — several tones
    /// inside the keep-band plus one well outside it.
    ///
    /// Compared on correlation rather than sample-wise error, because
    /// the two are *not* the same filter: a brick wall versus a FIR
    /// with a transition band. What has to hold is that the decoder
    /// sees the same waveform, not that the bytes match.
    #[test]
    fn tracks_the_reference_channelizer_on_a_multi_tone() {
        let n = NPOINTS_MAX;
        let mut audio = vec![0.0f32; n];
        for (f, a) in [
            (CENTER_HZ - 120.0, 0.30),
            (CENTER_HZ - 20.0, 0.20),
            (CENTER_HZ + 75.0, 0.25),
            (CENTER_HZ + 600.0, 0.40), // out of band, must not appear
        ] {
            // `f64` phase, for the reason `tone` documents.
            let w = 2.0 * core::f64::consts::PI * f as f64 / AUDIO_RATE_HZ as f64;
            for (k, s) in audio.iter_mut().enumerate() {
                *s += (a * (w * k as f64).cos()) as f32;
            }
        }

        let (ri, rq) = decimate_to_baseband(&audio);
        let (di, dq) = ddc_to_baseband(&audio);

        // Skip the filter's settling region at both ends.
        let span = 2_000..40_000;
        let (mut num_re, mut num_im, mut pr, mut pd) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for k in span {
            let (r, d) = ((ri[k], rq[k]), (di[k], dq[k]));
            // <ref, ddc*>
            num_re += (r.0 * d.0 + r.1 * d.1) as f64;
            num_im += (r.1 * d.0 - r.0 * d.1) as f64;
            pr += (r.0 * r.0 + r.1 * r.1) as f64;
            pd += (d.0 * d.0 + d.1 * d.1) as f64;
        }
        let coh = (num_re * num_re + num_im * num_im).sqrt() / (pr * pd).sqrt();
        assert!(coh > 0.99, "coherence with the reference only {coh}");

        // And the amplitude scale agrees, which is what `REFERENCE_GAIN`
        // is for.
        let ratio = (pd / pr).sqrt();
        assert!(
            (ratio - 1.0).abs() < 0.05,
            "amplitude ratio ddc/reference = {ratio}"
        );
    }
}
