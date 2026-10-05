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

const TARGET_RATE: f64 = 12_000.0;

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

/// The anti-alias low-pass applied at `src_rate` before going down to
/// 12 kHz: `fil4` at 48 kHz, a Kaiser low-pass to `fil4`'s specification
/// (pass 4500 Hz, stop 6000 Hz, 40 dB) at any other rate above 12 kHz, and
/// `None` at or below 12 kHz, where nothing can fold. Causal, like
/// `fil4_state`: the output lags by `(taps - 1) / 2` source samples
/// (0.5 ms at 48 kHz, as in WSJT-X).
fn antialias_taps(src_rate: u32) -> Option<Vec<f32>> {
    if src_rate <= 12_000 {
        return None;
    }
    if src_rate == 48_000 {
        return Some(FIL4.to_vec());
    }
    let fs = src_rate as f64;
    let (ntaps, beta) = super::fir_decimate::kaiser_order(40.0, 1_500.0 / fs);
    Some(super::fir_decimate::design_lowpass_kaiser(
        ntaps,
        5_250.0 / fs,
        beta,
    ))
}

/// Linear interpolation of `len` source samples at `src_rate` onto the
/// 12 kHz grid, reading source sample `n` through `x(n)`. The right-hand
/// sample is read only when the output falls between two inputs, so an
/// integer ratio (48 kHz) reads one filtered sample per output.
fn interp_to_12k(len: usize, src_rate: u32, x: impl Fn(usize) -> f64) -> Vec<f64> {
    let ratio = TARGET_RATE / src_rate as f64;
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

/// `interp_to_12k` over `raw`, through the anti-alias low-pass when
/// `src_rate` needs one. The filter is evaluated only at the source
/// samples the interpolation reads.
fn to_12k(len: usize, src_rate: u32, raw: impl Fn(usize) -> f64) -> Vec<f64> {
    match antialias_taps(src_rate) {
        None => interp_to_12k(len, src_rate, raw),
        Some(h) => interp_to_12k(len, src_rate, |n| {
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
    to_12k(samples.len(), src_rate, |n| samples[n] as f64)
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

    to_12k(samples.len(), src_rate, |n| samples[n] as f64)
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
    to_12k(samples.len(), src_rate, |n| samples[n] as f64)
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

/// Stateful chunk-based linear resampler `src_rate → 12 000 Hz`.
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
pub struct LinearResamplerI16To12k {
    src_rate: u32,
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

impl LinearResamplerI16To12k {
    /// Construct a resampler for `src_rate_hz` → 12 000 Hz.
    /// Panics if `src_rate_hz == 0`.
    pub fn new(src_rate_hz: u32) -> Self {
        assert!(src_rate_hz > 0, "src_rate_hz must be > 0");
        let step_q32 = ((src_rate_hz as u64) << 32) / 12_000;
        let taps_q15: Vec<i32> = antialias_taps(src_rate_hz)
            .unwrap_or_default()
            .iter()
            .map(|&h| (h as f64 * 32_768.0).round() as i32)
            .collect();
        let hist = alloc::vec![0i16; taps_q15.len()];
        Self {
            src_rate: src_rate_hz,
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
        // out ≈ src_len * 12000 / src_rate, plus 1 for boundary
        // rounding.
        ((src_len as u64) * 12_000 / self.src_rate as u64) as usize + 1
    }
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

    // Integration tests that depend on ft8-core's decode pipeline live in
    // `ft8-core/tests/resample_ft8.rs` (moved there alongside this module's
    // migration to mfsk-core).
}
