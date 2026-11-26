//! Resampler: arbitrary input rate → 12 000 Hz.
//!
//! Used at the decode entry point so the rest of the pipeline can
//! assume a fixed 12 000 Hz sample rate. Above 12 kHz the input first
//! goes through an anti-alias low-pass at its own rate, then linear
//! interpolation picks the 12 kHz samples; at or below 12 kHz there is
//! nothing to fold and only the interpolation runs.
//!
//! The low-pass is WSJT-X's own at 48 kHz: `Detector.cpp` runs the sound
//! card's 48 kHz through `fil4_state` (`lib/fil4.f90`, v3.2.0-rc1) — 49
//! taps, pass to 4500 Hz, stop from 6000 Hz, 40 dB — before decimating by
//! 4. Other rates get a Kaiser low-pass to the same specification. Before
//! #576 there was no filter at all: 48 kHz → 12 kHz was a plain
//! decimation by 4 that folded 6–24 kHz into the band, and on full-band
//! white noise FT8's 50 % crossing sat at -15.3 dB against -21.1 dB with
//! `fil4` (-21.0 dB for the same signal and noise generated at 12 kHz).

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

/// `fil4`'s taps (`lib/fil4.f90`, WSJT-X v3.2.0-rc1), designed with ScopeFIR
/// for 48 kHz: 49 taps, fc 4500 Hz, stop 6000 Hz, 1 dB ripple, 40 dB.
/// Computed from these taps: +0.5 dB at DC (the gain sums to 1.056, kept
/// as upstream has it), -0.5 dB at 3–4.5 kHz, <= -40 dB from 6 kHz up.
/// Spelled as in the Fortran, hence the precision allow.
#[allow(clippy::excessive_precision)]
const FIL4: [f32; 49] = [
    0.000861074040,
    0.010051920210,
    0.010161983649,
    0.011363155076,
    0.008706594219,
    0.002613872664,
    -0.005202883094,
    -0.011720748164,
    -0.013752163325,
    -0.009431602741,
    0.000539063909,
    0.012636767098,
    0.021494659597,
    0.021951235065,
    0.011564169382,
    -0.007656470131,
    -0.028965787341,
    -0.042637874109,
    -0.039203309748,
    -0.013153301537,
    0.034320769178,
    0.094717832646,
    0.154224604789,
    0.197758325022,
    0.213715139513,
    0.197758325022,
    0.154224604789,
    0.094717832646,
    0.034320769178,
    -0.013153301537,
    -0.039203309748,
    -0.042637874109,
    -0.028965787341,
    -0.007656470131,
    0.011564169382,
    0.021951235065,
    0.021494659597,
    0.012636767098,
    0.000539063909,
    -0.009431602741,
    -0.013752163325,
    -0.011720748164,
    -0.005202883094,
    0.002613872664,
    0.008706594219,
    0.011363155076,
    0.010161983649,
    0.010051920210,
    0.000861074040,
];

/// The anti-alias low-pass applied at `src_rate` before going down to `target` Hz (12 000 or 8 000): `fil4` at 48 kHz to 12 kHz, a Kaiser low-pass to the target's
/// specification at any other rate above the target, and `None` at or below it, where nothing can fold. The specification, as `fil4`'s (40 dB):
/// **12 kHz**: pass 4 500 Hz, stop 6 000 Hz (the target's Nyquist); **8 kHz**: pass 3 200 Hz, stop 4 000 Hz (the target's Nyquist; FT8's band ends at 3 050 Hz).
/// Causal, like `fil4_state`: the output lags by `(taps - 1) / 2` source samples (0.5 ms at 48 kHz to 12 kHz, as in WSJT-X).
fn antialias_taps(src_rate: u32, target: u32) -> Option<Vec<f32>> {
    if src_rate <= target {
        return None;
    }
    if src_rate == 48_000 && target == 12_000 {
        return Some(FIL4.to_vec());
    }
    let fs = src_rate as f64;
    let (pass, stop) = match target {
        8_000 => (3_200.0, 4_000.0),
        _ => (4_500.0, 6_000.0),
    };
    let (ntaps, beta) = super::fir_decimate::kaiser_order(40.0, (stop - pass) / fs);
    Some(super::fir_decimate::design_lowpass_kaiser(
        ntaps,
        (pass + stop) / 2.0 / fs,
        beta,
    ))
}

/// Linear interpolation of `len` source samples at `src_rate` onto the
/// `target` grid, reading source sample `n` through `x(n)`. The right-hand
/// sample is read only when the output falls between two inputs, so an
/// integer ratio (48 kHz) reads one filtered sample per output.
fn interp_to(target: u32, len: usize, src_rate: u32, x: impl Fn(usize) -> f64) -> Vec<f64> {
    let ratio = target as f64 / src_rate as f64;
    let out_len = (len as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        if idx + 1 < len {
            let a = x(idx);
            out.push(if frac == 0.0 {
                a
            } else {
                a + (x(idx + 1) - a) * frac
            });
        } else if idx < len {
            out.push(x(idx));
        }
    }
    out
}

/// `interp_to` over `raw`, through the anti-alias low-pass when
/// `src_rate` needs one. The filter is evaluated only at the source
/// samples the interpolation reads.
fn to_rate(target: u32, len: usize, src_rate: u32, raw: impl Fn(usize) -> f64) -> Vec<f64> {
    match antialias_taps(src_rate, target) {
        None => interp_to(target, len, src_rate, raw),
        Some(h) => interp_to(target, len, src_rate, |n| {
            h.iter()
                .take(n + 1)
                .enumerate()
                .map(|(k, &w)| w as f64 * raw(n - k))
                .sum()
        }),
    }
}

/// Resample `samples` from `src_rate` Hz to 12 000 Hz: the anti-alias
/// low-pass above 12 kHz (`fil4` at 48 kHz), then linear interpolation.
///
/// Returns the resampled buffer.  If `src_rate` is already 12 000, the
/// input is returned as-is (zero-copy via `Cow` semantics at the call site).
pub fn resample_to_12k(samples: &[i16], src_rate: u32) -> Vec<i16> {
    to_rate(12_000, samples.len(), src_rate, |n| samples[n] as f64)
        .into_iter()
        .map(|v| v.round().clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

/// f32 → 12 000 Hz i16 in a single pass (anti-alias low-pass, linear
/// interpolation, scaling).
///
/// Used by the WASM live-capture path so the JS side can hand a Float32Array
/// straight from the AudioWorklet without an intermediate i16 conversion loop.
///
/// **Normalization:** before resampling the input is peak-normalised to
/// `TARGET_PEAK` (0.8 full-scale).  This ensures the full i16 dynamic range
/// is used regardless of the hardware input level — a common problem with USB
/// radio audio adapters whose Windows volume setting may be very low.
/// Signal-to-noise ratio is preserved because signal and noise are scaled
/// equally.  Buffers whose peak is below `SILENCE_FLOOR` are treated as
/// silence and left at 0.
///
/// If `src_rate == 12000`, this still allocates and converts (no zero-copy)
/// because the output is i16 and the input is f32.
pub fn resample_f32_to_12k(samples: &[f32], src_rate: u32) -> Vec<i16> {
    const TARGET_PEAK: f64 = 0.8;
    const SILENCE_FLOOR: f64 = 1e-6;

    // Find peak amplitude
    let peak = samples.iter().fold(0.0f64, |m, &s| m.max((s as f64).abs()));
    let scale = if peak > SILENCE_FLOOR {
        TARGET_PEAK / peak
    } else {
        1.0
    };

    to_rate(12_000, samples.len(), src_rate, |n| samples[n] as f64)
        .into_iter()
        .map(|v| (v * scale * 32767.0).clamp(-32768.0, 32767.0).round() as i16)
        .collect()
}

/// f32 → 12 000 Hz f32, anti-aliased as above, **no normalisation**.
///
/// Preserves absolute amplitude — use this from decoders whose LLR
/// scaling depends on the raw signal/noise ratio (WSPR's noncoherent
/// 4-FSK LLR, for instance). If `src_rate == 12000`, the input is
/// copied verbatim; otherwise standard linear resampling applies.
pub fn resample_f32_to_12k_f32(samples: &[f32], src_rate: u32) -> Vec<f32> {
    if src_rate == 12_000 {
        return samples.to_vec();
    }
    to_rate(12_000, samples.len(), src_rate, |n| samples[n] as f64)
        .into_iter()
        .map(|v| v as f32)
        .collect()
}

/// i16 → 12 000 Hz f32. Thin wrapper: resample as i16, convert to f32
/// in [-1, 1]. Used at WSPR WAV entry points where the incoming PCM
/// is `Int16Array` but the decoder wants `f32`.
pub fn resample_i16_to_12k_f32(samples: &[i16], src_rate: u32) -> Vec<f32> {
    if src_rate == 12_000 {
        return samples.iter().map(|&s| s as f32 / 32768.0).collect();
    }
    resample_to_12k(samples, src_rate)
        .into_iter()
        .map(|s| s as f32 / 32768.0)
        .collect()
}

/// Stateful chunk-based linear resampler `src_rate → target_hz` (12 000 or 8 000 Hz); [`LinearResamplerI16To12k`] and [`LinearResamplerI16To8k`] are its two forms.
///
/// The batch [`resample_to_12k`] family allocates a fresh `Vec` per
/// call and assumes the entire input is in hand. Streaming receivers
/// (I2S DMA on ESP32 / RP2350 / Cortex-M, or sound-card capture on
/// host) instead push small chunks as they arrive and need the
/// resampler to carry interpolation state across calls so the chunk
/// boundary doesn't introduce a discontinuity.
///
/// `LinearResamplerI16To12k` is that streaming variant: the same
/// anti-alias low-pass as the batch path (`fil4` at 48 kHz, #576) in Q15
/// integer taps with the filter history carried across calls, then the
/// same linear interpolation, with a fixed-point `phase_q32` (Q32
/// fractional source position) and a `last_in` carry-over sample. Output
/// is written into a caller-provided buffer — no per-call heap
/// allocation. Pure scalar i64 arithmetic; runs on FPU-less MCUs. The
/// filter is evaluated only where an output needs it — once per output at
/// 48 kHz, 49 MACs, about 0.6 M MAC/s — and the history is a ring, so an
/// input sample costs one store.
///
/// **Paired with** `MfskFt8Stream` in `mfsk-ffi-ft8`, which held one
/// of these plus a 12 kHz ring buffer for the FT8 decode entry. That
/// crate was retired in 0.11.0; `mfsk-ffi`'s `mfsk_stream_*` is the
/// generalised successor, and `embedded-shared`'s own pipeline is what
/// drives this type on the boards.
pub struct LinearResamplerI16 {
    src_rate: u32,
    /// The output rate, Hz (12 000 or 8 000).
    target_hz: u32,
    /// Q32 source-sample step per output sample.
    /// `step_q32 = (src_rate << 32) / 12_000`.
    step_q32: u64,
    /// Q32 fractional source position relative to `last_in`.
    /// Invariant: maintained `< 2^32` whenever output is being produced
    /// (loop drains integer parts by consuming source samples first).
    phase_q32: u64,
    /// Last input sample absorbed; used as the left endpoint of the
    /// next interpolation pair.
    last_in: i16,
    /// `false` until the very first sample has been absorbed into
    /// `last_in`. The first `process()` call consumes one src sample
    /// to prime; from then on the resampler can produce one output
    /// per `step_q32` worth of phase.
    primed: bool,
    /// The anti-alias taps in Q15, empty at or below 12 kHz.
    taps_q15: Vec<i32>,
    /// The last `taps_q15.len()` raw input samples, a ring: the newest is
    /// at `hist_pos - 1`. `last_in` is the newest one when filtering.
    hist: Vec<i16>,
    hist_pos: usize,
}

impl core::fmt::Debug for LinearResamplerI16 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LinearResamplerI16")
            .field("src_rate", &self.src_rate)
            .field("target_hz", &self.target_hz)
            .field("primed", &self.primed)
            .field("n_taps", &self.taps_q15.len())
            .finish_non_exhaustive()
    }
}

impl LinearResamplerI16 {
    /// Construct a resampler for `src_rate_hz` → `target_hz` (12 000 or 8 000: the anti-alias specification of each is in `antialias_taps`).
    /// Panics if `src_rate_hz == 0` or `target_hz == 0`.
    pub fn new(src_rate_hz: u32, target_hz: u32) -> Self {
        assert!(src_rate_hz > 0, "src_rate_hz must be > 0");
        assert!(target_hz > 0, "target_hz must be > 0");
        let step_q32 = ((src_rate_hz as u64) << 32) / target_hz as u64;
        let taps_q15: Vec<i32> = antialias_taps(src_rate_hz, target_hz)
            .unwrap_or_default()
            .iter()
            .map(|&h| (h as f64 * 32_768.0).round() as i32)
            .collect();
        let hist = alloc::vec![0i16; taps_q15.len()];
        Self {
            src_rate: src_rate_hz,
            target_hz,
            step_q32,
            phase_q32: 0,
            last_in: 0,
            primed: false,
            taps_q15,
            hist,
            hist_pos: 0,
        }
    }

    /// Take `x` as the newest input.
    fn absorb(&mut self, x: i16) {
        self.last_in = x;
        if !self.hist.is_empty() {
            self.hist[self.hist_pos] = x;
            self.hist_pos = (self.hist_pos + 1) % self.hist.len();
        }
    }

    /// The low-pass output at the newest input, or with `next` appended
    /// first (without taking it). Without a filter, the raw sample.
    fn filtered(&self, next: Option<i16>) -> i16 {
        let n = self.hist.len();
        if n == 0 {
            return next.unwrap_or(self.last_in);
        }
        // `k` steps back from the sample being filtered: with `next`, step 0
        // is `next` and step k is the ring's (k-1)-th newest.
        let back = |k: usize| -> i16 {
            match (next, k) {
                (Some(x), 0) => x,
                (Some(_), k) => self.hist[(self.hist_pos + n - k) % n],
                (None, k) => self.hist[(self.hist_pos + n - 1 - k) % n],
            }
        };
        let acc: i64 = self
            .taps_q15
            .iter()
            .enumerate()
            .map(|(k, &h)| h as i64 * back(k) as i64)
            .sum();
        ((acc + (1 << 14)) >> 15).clamp(i16::MIN as i64, i16::MAX as i64) as i16
    }

    /// Source rate this resampler was constructed with.
    pub fn src_rate(&self) -> u32 {
        self.src_rate
    }

    /// Consume up to `src.len()` input samples and emit up to
    /// `dst.len()` output samples at 12 kHz.
    ///
    /// Returns `(consumed, produced)`: the number of source samples
    /// drained from `src` and the number of output samples written
    /// into `dst[..produced]`. Either limit can be the binding one;
    /// the caller drives the loop by feeding more `src` chunks until
    /// `produced` reaches the desired count.
    ///
    /// Linear interpolation over the (`last_in`, next-src) pair —
    /// rounded half-up via `(diff * frac + (1 << 31)) >> 32`.
    pub fn process(&mut self, src: &[i16], dst: &mut [i16]) -> (usize, usize) {
        let mut src = src;
        let mut consumed = 0usize;
        let mut produced = 0usize;

        // Prime the carry sample on the very first call.
        if !self.primed {
            if src.is_empty() {
                return (0, 0);
            }
            self.absorb(src[0]);
            src = &src[1..];
            consumed += 1;
            self.primed = true;
            // phase_q32 starts at 0 → first emitted output equals
            // `last_in` (matches the batch resampler, whose first
            // output is `samples[0]`).
        }

        while produced < dst.len() {
            // Drain whole-source-sample integer parts of phase by
            // shifting the (last_in, src[0]) pair forward.
            while self.phase_q32 >= 1u64 << 32 {
                if src.is_empty() {
                    return (consumed, produced);
                }
                self.absorb(src[0]);
                src = &src[1..];
                consumed += 1;
                self.phase_q32 -= 1u64 << 32;
            }

            // Emit one output. If phase is exactly 0 the answer is
            // `last_in` itself (no interpolation needed, no src lookup
            // required — important for the tail of a finite stream).
            let out = if self.phase_q32 == 0 {
                self.filtered(None)
            } else {
                // Need src[0] as the right endpoint.
                if src.is_empty() {
                    return (consumed, produced);
                }
                let a = self.filtered(None) as i64;
                let b = self.filtered(Some(src[0])) as i64;
                let frac = self.phase_q32 as i64; // < 2^32
                // (b - a) ∈ [-65535, 65535]; * frac ∈ [-2^48, 2^48]; fits i64.
                let interp = a + (((b - a) * frac + (1 << 31)) >> 32);
                interp as i16
            };
            dst[produced] = out;
            produced += 1;
            self.phase_q32 += self.step_q32;
        }

        (consumed, produced)
    }

    /// Maximum number of output samples that *could* be produced
    /// from `src_len` source samples (worst case, ignoring fractional
    /// phase state). Used by the streaming wrapper to size scratch
    /// buffers.
    pub fn max_output_for(&self, src_len: usize) -> usize {
        // out ≈ src_len * target / src_rate, plus 1 for boundary
        // rounding.
        ((src_len as u64) * self.target_hz as u64 / self.src_rate as u64) as usize + 1
    }
}

/// `src_rate → 12 000 Hz`: [`LinearResamplerI16`] at the target every mode but FT8's 8 kHz flow uses (`fil4` at 48 kHz).
#[derive(Debug)]
pub struct LinearResamplerI16To12k(LinearResamplerI16);

impl LinearResamplerI16To12k {
    /// Construct a resampler for `src_rate_hz` → 12 000 Hz. Panics if `src_rate_hz == 0`.
    pub fn new(src_rate_hz: u32) -> Self {
        Self(LinearResamplerI16::new(src_rate_hz, 12_000))
    }

    /// Source rate this resampler was constructed with.
    pub fn src_rate(&self) -> u32 {
        self.0.src_rate()
    }

    /// See [`LinearResamplerI16::process`].
    pub fn process(&mut self, src: &[i16], dst: &mut [i16]) -> (usize, usize) {
        self.0.process(src, dst)
    }

    /// See [`LinearResamplerI16::max_output_for`].
    pub fn max_output_for(&self, src_len: usize) -> usize {
        self.0.max_output_for(src_len)
    }
}

/// `src_rate → 8 000 Hz`: [`LinearResamplerI16`] at the rate of the FT8 flow on the half-dB cells (`Ft8Rate::Hz8000`). At 48 kHz a Kaiser low-pass (pass 3 200 Hz,
/// stop 4 000 Hz, 40 dB; about 135 taps, evaluated once per output: about 1.1 M MAC/s) then the 6:1 linear interpolation.
#[derive(Debug)]
pub struct LinearResamplerI16To8k(LinearResamplerI16);

impl LinearResamplerI16To8k {
    /// Construct a resampler for `src_rate_hz` → 8 000 Hz. Panics if `src_rate_hz == 0`.
    pub fn new(src_rate_hz: u32) -> Self {
        Self(LinearResamplerI16::new(src_rate_hz, 8_000))
    }

    /// Source rate this resampler was constructed with.
    pub fn src_rate(&self) -> u32 {
        self.0.src_rate()
    }

    /// See [`LinearResamplerI16::process`].
    pub fn process(&mut self, src: &[i16], dst: &mut [i16]) -> (usize, usize) {
        self.0.process(src, dst)
    }

    /// See [`LinearResamplerI16::max_output_for`].
    pub fn max_output_for(&self, src_len: usize) -> usize {
        self.0.max_output_for(src_len)
    }
}

/// Resample `samples` from `src_rate` Hz to 8 000 Hz: the anti-alias low-pass above 8 kHz (pass 3 200 Hz, stop 4 000 Hz, 40 dB), then linear interpolation.
pub fn resample_to_8k(samples: &[i16], src_rate: u32) -> Vec<i16> {
    to_rate(8_000, samples.len(), src_rate, |n| samples[n] as f64)
        .into_iter()
        .map(|v| v.round().clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passthrough_at_12k() {
        let input: Vec<i16> = (0..100).collect();
        let out = resample_to_12k(&input, 12000);
        assert_eq!(out.len(), 100);
        assert_eq!(out, input);
    }

    #[test]
    fn downsample_from_48k() {
        // 48000 → 12000 = factor 4
        let input: Vec<i16> = (0..4800).map(|i| (i % 100) as i16).collect();
        let out = resample_to_12k(&input, 48000);
        assert_eq!(out.len(), 1200);
    }

    #[test]
    fn downsample_from_44100() {
        // 44100 → 12000: non-integer ratio
        let input: Vec<i16> = vec![0i16; 44100];
        let out = resample_to_12k(&input, 44100);
        // Should be close to 12000 samples for 1 second
        assert!((out.len() as i32 - 12000).abs() <= 1);
    }

    // ── Streaming resampler tests ────────────────────────────────────

    #[test]
    fn streaming_passthrough_at_12k() {
        let input: Vec<i16> = (0..100).collect();
        let mut r = LinearResamplerI16To12k::new(12_000);
        let mut out = vec![0i16; 100];
        let (cons, prod) = r.process(&input, &mut out);
        assert_eq!(prod, 100);
        // Pass-through should emit the input verbatim (modulo boundary
        // tail). At 12k → 12k, step_q32 = 2^32 so phase is always 0
        // when emitting and every output equals last_in = src[i].
        assert_eq!(&out[..prod], &input[..prod]);
        // Consumed 1 prime + (prod - 1) post-prime drains = prod source samples.
        assert_eq!(cons, prod);
    }

    #[test]
    fn streaming_downsample_48k_to_12k() {
        // 48k → 12k = factor 4. step_q32 = 4 * 2^32: one output per four
        // inputs, each the fil4 output at that input.
        let input = vec![1_000i16; 400];
        let mut r = LinearResamplerI16To12k::new(48_000);
        let mut out = vec![0i16; 100];
        let (cons, prod) = r.process(&input, &mut out);
        assert_eq!(prod, 100);
        assert_eq!(cons, 397); // 1 prime + 99 post-prime × 4 = 397
        // Once the 49-tap history is full, a constant comes out at fil4's
        // DC gain (1.056).
        for &y in &out[13..] {
            assert!((1_053..=1_059).contains(&y), "{y}");
        }
    }

    /// Output power of a tone at `f_hz` through the resampler, relative to
    /// the input's, in dB — measured after the filter has settled.
    fn tone_gain_db(src_rate: u32, f_hz: f64, streaming: bool) -> f64 {
        let n = src_rate as usize; // one second
        let amp = 10_000.0;
        let x: Vec<i16> = (0..n)
            .map(|i| {
                (amp * (2.0 * core::f64::consts::PI * f_hz * i as f64 / src_rate as f64).sin())
                    as i16
            })
            .collect();
        let y = if streaming {
            let mut r = LinearResamplerI16To12k::new(src_rate);
            let mut out = vec![0i16; r.max_output_for(n)];
            let (_c, p) = r.process(&x, &mut out);
            out.truncate(p);
            out
        } else {
            resample_to_12k(&x, src_rate)
        };
        let tail = &y[1_000..y.len() - 10];
        let p_out = tail.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / tail.len() as f64;
        10.0 * (p_out / (amp * amp / 2.0)).log10()
    }

    /// #576: above 12 kHz the input must not fold into the band. A tone at
    /// 9 kHz lands on 3 kHz if it is decimated without a filter, which is
    /// what both paths did; it must now come out at least 30 dB down, and a
    /// tone in the band must keep its level.
    #[test]
    fn rates_above_12k_reject_what_would_alias() {
        for &(rate, streaming) in &[
            (48_000, false),
            (48_000, true),
            (44_100, false),
            (44_100, true),
            (96_000, false),
            (24_000, false),
        ] {
            let alias = tone_gain_db(rate, 9_000.0, streaming);
            let band = tone_gain_db(rate, 1_500.0, streaming);
            assert!(
                alias < -30.0,
                "{rate} Hz (streaming {streaming}): 9 kHz at {alias:.1} dB"
            );
            assert!(
                band.abs() < 1.0,
                "{rate} Hz (streaming {streaming}): 1.5 kHz at {band:.2} dB"
            );
        }
    }

    #[test]
    fn streaming_chunked_matches_single_call() {
        // Splitting the input into chunks must not change the output —
        // this is the whole point of carrying state.
        let input: Vec<i16> = (0..4410).map(|i| (i % 200) as i16).collect();

        let mut r1 = LinearResamplerI16To12k::new(44_100);
        let mut single = vec![0i16; 1500];
        let (_c1, p1) = r1.process(&input, &mut single);

        let mut r2 = LinearResamplerI16To12k::new(44_100);
        let mut chunked = vec![0i16; 1500];
        let mut produced = 0;
        let mut src_pos = 0;
        while src_pos < input.len() && produced < chunked.len() {
            let chunk_end = (src_pos + 137).min(input.len()); // odd chunk size
            let (c, p) = r2.process(&input[src_pos..chunk_end], &mut chunked[produced..]);
            src_pos += c;
            produced += p;
            if c == 0 && p == 0 {
                break; // would be infinite loop otherwise
            }
        }

        assert_eq!(produced, p1);
        assert_eq!(&chunked[..produced], &single[..p1]);
    }

    #[test]
    fn streaming_upsample_6k_to_12k() {
        // 6k → 12k = factor 0.5. step_q32 = 2^31.
        // Outputs alternate: src[i], midpoint(src[i], src[i+1]), src[i+1], midpoint, …
        let input: Vec<i16> = vec![0, 100, 200, 300, 400, 500];
        let mut r = LinearResamplerI16To12k::new(6_000);
        let mut out = vec![0i16; 11];
        let (_cons, prod) = r.process(&input, &mut out);
        // First output = src[0] = 0.
        assert_eq!(out[0], 0);
        // Second output = midpoint(0, 100) = 50.
        assert_eq!(out[1], 50);
        // Third output = src[1] = 100.
        assert_eq!(out[2], 100);
        // Fourth output = midpoint(100, 200) = 150.
        assert_eq!(out[3], 150);
        assert!(prod >= 10);
    }

    // Integration tests that depend on the FT8 decode pipeline live in
    // `mfsk-core/tests/ft8_resample.rs`.

    // ── The 8 kHz target (step 0c of the CoreS3 plan) ────────────────

    fn synth(n: usize, seed: u64) -> Vec<i16> {
        let mut st = seed;
        (0..n)
            .map(|i| {
                st = st
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let noise = ((st >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 6000.0;
                let tone = 8000.0 * (core::f32::consts::TAU * 1500.0 * i as f32 / 48000.0).sin();
                (noise + tone) as i16
            })
            .collect()
    }

    fn fnv(v: &[i16]) -> u64 {
        let mut h = 0xcbf29ce484222325u64;
        for &x in v {
            for b in x.to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x100000001b3);
            }
        }
        h
    }

    /// The 12 kHz outputs are what they were before the target became a parameter: the batch and the streaming resampler at 48 000, 44 100, 24 000 and 16 000 Hz
    /// give the samples whose FNV hashes were taken from the code as it was (a scratch build of the commit before). **Fails if** the refactor changes one sample
    /// or one output count of the 12 kHz path (the filter taps, the interpolation, the phase step, the ring).
    #[test]
    fn the_12k_outputs_are_what_they_were() {
        const GOLDEN: [(u32, usize, u64, usize, u64, usize); 4] = [
            (
                48_000,
                96_000,
                244931166423935021,
                24000,
                10208247876593079070,
                24000,
            ),
            (
                44_100,
                88_200,
                1939685733906210040,
                24001,
                12972434343827817668,
                24000,
            ),
            (
                24_000,
                48_000,
                17916385077328962935,
                24000,
                18275154338166326010,
                24000,
            ),
            (
                16_000,
                32_000,
                13285267916852385764,
                24000,
                466958958094099648,
                24000,
            ),
        ];
        for (rate, n, batch, batch_len, stream, stream_len) in GOLDEN {
            let x = synth(n, 7);
            let y = resample_to_12k(&x, rate);
            assert_eq!((fnv(&y), y.len()), (batch, batch_len), "{rate} Hz batch");
            let mut r = LinearResamplerI16To12k::new(rate);
            let mut out = vec![0i16; y.len() + 8];
            let mut got = Vec::new();
            for c in x.chunks(777) {
                let (_, p) = r.process(c, &mut out);
                got.extend_from_slice(&out[..p]);
            }
            assert_eq!(
                (fnv(&got), got.len()),
                (stream, stream_len),
                "{rate} Hz streaming"
            );
        }
    }

    /// `|H(f)|` of the taps at `f` Hz for a signal at `fs`.
    fn gain(h: &[f32], f: f64, fs: f64) -> f64 {
        let w = core::f64::consts::TAU * f / fs;
        let (mut re, mut im) = (0.0, 0.0);
        for (k, &c) in h.iter().enumerate() {
            re += c as f64 * (w * k as f64).cos();
            im -= c as f64 * (w * k as f64).sin();
        }
        (re * re + im * im).sqrt()
    }

    /// The 8 kHz anti-alias at 48 kHz: flat to 3 100 Hz within 1 dB of its gain at DC, and at least 40 dB down from the target's Nyquist (4 000 Hz) up to 24 kHz.
    /// **Fails if** the pass or stop edge, or the attenuation, of the design is off (the 12 kHz `fil4` would let 4 000-6 000 Hz through and fold it onto 2 000-4 000 Hz).
    #[test]
    fn the_8k_antialias_passes_the_band_and_stops_the_alias() {
        let h = antialias_taps(48_000, 8_000).expect("48 kHz needs a filter for 8 kHz");
        let dc = gain(&h, 0.0, 48_000.0);
        for f in (0..=3_100).step_by(50) {
            let db = 20.0 * (gain(&h, f as f64, 48_000.0) / dc).log10();
            assert!(db.abs() <= 1.0, "{f} Hz: {db:.2} dB against DC");
        }
        for f in (4_000..=24_000).step_by(100) {
            let db = 20.0 * (gain(&h, f as f64, 48_000.0) / dc).log10();
            assert!(db <= -40.0 + 0.5, "{f} Hz: {db:.1} dB against DC");
        }
        assert!(h.len() < 200, "{} taps", h.len());
    }

    /// White noise at 48 kHz brought to 8 kHz and to 12 kHz keeps, in 200-3 000 Hz, the power its own filter's passband predicts: no noise folds into the band (the
    /// in-band power over the in-band mean of `|H|^2`, against the input's, within 0.3 dB, for both targets). **Fails if** the stop band is too shallow or starts too
    /// high (a filter built for 12 kHz used for 8 kHz leaves 4 000-5 500 Hz noise to fold onto 2 500-4 000 Hz).
    #[test]
    fn white_noise_does_not_fold_into_the_band() {
        let n = 48_000 * 12;
        let mut st = 99u64;
        let noise: Vec<i16> = (0..n)
            .map(|_| {
                let mut g = 0.0f32;
                for _ in 0..12 {
                    st = st
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    g += ((st >> 40) as f32) / (1u64 << 24) as f32;
                }
                ((g - 6.0) * 2000.0) as i16
            })
            .collect();
        // band power of `y` at `fs` over 200-3 000 Hz by Welch segments of 1 024 with a Hann window
        let band_power = |y: &[i16], fs: f64| -> f64 {
            let seg = 1024usize;
            let win: Vec<f64> = (0..seg)
                .map(|i| 0.5 - 0.5 * (core::f64::consts::TAU * i as f64 / seg as f64).cos())
                .collect();
            let wsum: f64 = win.iter().map(|w| w * w).sum();
            let bins: Vec<usize> = ((200.0 * seg as f64 / fs) as usize
                ..=(3000.0 * seg as f64 / fs) as usize)
                .collect();
            let mut total = 0.0;
            let mut count = 0usize;
            for chunk in y.chunks_exact(seg).take(40) {
                for &b in &bins {
                    let (mut re, mut im) = (0.0, 0.0);
                    for (i, (&v, w)) in chunk.iter().zip(&win).enumerate() {
                        let a = core::f64::consts::TAU * (b * i) as f64 / seg as f64;
                        re += v as f64 * w * a.cos();
                        im -= v as f64 * w * a.sin();
                    }
                    total += (re * re + im * im) / wsum;
                    count += 1;
                }
            }
            total / count as f64
        };
        let p_in = band_power(&noise, 48_000.0);
        for (target, out) in [
            (8_000u32, resample_to_8k(&noise, 48_000)),
            (12_000, resample_to_12k(&noise, 48_000)),
        ] {
            let h = antialias_taps(48_000, target).unwrap();
            let mean_h2: f64 = (200..=3000)
                .step_by(20)
                .map(|f| gain(&h, f as f64, 48_000.0).powi(2))
                .sum::<f64>()
                / ((3000 - 200) / 20 + 1) as f64;
            let p_out = band_power(&out, target as f64);
            // the estimator is the power per sample: white noise of variance v at 48 kHz is the density v / 48 000 a Hz, and at `target` v x target / 48 000
            let ratio_db = 10.0 * (p_out / (p_in * mean_h2 * target as f64 / 48_000.0)).log10();
            assert!(
                ratio_db.abs() <= 0.3,
                "{target} Hz: {ratio_db:.2} dB against the filter's prediction"
            );
        }
    }

    /// The 8 kHz streaming resampler gives the same samples for any chunking (1, 777 and 4 096 samples: bit for bit) and the batch function's within 2 LSB.
    /// **Fails if** the ring or the phase carries wrongly across a chunk boundary, or the Q15 taps and the batch's differ.
    #[test]
    fn the_8k_streaming_resampler_is_chunking_independent() {
        let x = synth(48_000 * 2, 5);
        let batch = resample_to_8k(&x, 48_000);
        let run = |chunk: usize| {
            let mut r = LinearResamplerI16To8k::new(48_000);
            let mut out = vec![0i16; batch.len() + 8];
            let mut got = Vec::new();
            for c in x.chunks(chunk) {
                let (_, p) = r.process(c, &mut out);
                got.extend_from_slice(&out[..p]);
            }
            got
        };
        let (a, b, c) = (run(1), run(777), run(4096));
        assert_eq!(a, b);
        assert_eq!(b, c);
        assert_eq!(a.len(), batch.len());
        let worst = a
            .iter()
            .zip(&batch)
            .map(|(p, q)| (*p as i32 - *q as i32).abs())
            .max()
            .unwrap();
        assert!(worst <= 2, "{worst} LSB against the batch function");
    }

    /// The output size bound follows the target: 8 000 and 12 000 Hz outputs of a second of 48 kHz. **Fails if** `max_output_for` uses a fixed 12 kHz.
    #[test]
    fn the_output_bound_follows_the_target() {
        assert_eq!(
            LinearResamplerI16To8k::new(48_000).max_output_for(48_000),
            8_001
        );
        assert_eq!(
            LinearResamplerI16To12k::new(48_000).max_output_for(48_000),
            12_001
        );
    }
}
