//! Linear-interpolation resampler: arbitrary input rate → 12 000 Hz.
//!
//! Used at the decode entry point so the rest of the pipeline can
//! assume a fixed 12 000 Hz sample rate.

const TARGET_RATE: f64 = 12_000.0;

/// Resample `samples` from `src_rate` Hz to 12 000 Hz using linear interpolation.
///
/// Returns the resampled buffer.  If `src_rate` is already 12 000, the
/// input is returned as-is (zero-copy via `Cow` semantics at the call site).
pub fn resample_to_12k(samples: &[i16], src_rate: u32) -> Vec<i16> {
    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);

    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;

        if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            let v = a + (b - a) * frac;
            out.push(v.round() as i16);
        } else if idx < samples.len() {
            out.push(samples[idx]);
        }
    }

    out
}

/// f32 → 12 000 Hz i16 in a single pass (linear interpolation + scaling).
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

    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);

    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;

        let v = if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            a + (b - a) * frac
        } else if idx < samples.len() {
            samples[idx] as f64
        } else {
            continue;
        };

        let scaled = (v * scale * 32767.0).clamp(-32768.0, 32767.0);
        out.push(scaled.round() as i16);
    }

    out
}

/// f32 → 12 000 Hz f32, linear interpolation, **no normalisation**.
///
/// Preserves absolute amplitude — use this from decoders whose LLR
/// scaling depends on the raw signal/noise ratio (WSPR's noncoherent
/// 4-FSK LLR, for instance). If `src_rate == 12000`, the input is
/// copied verbatim; otherwise standard linear resampling applies.
pub fn resample_f32_to_12k_f32(samples: &[f32], src_rate: u32) -> Vec<f32> {
    if src_rate == 12_000 {
        return samples.to_vec();
    }
    let ratio = TARGET_RATE / src_rate as f64;
    let out_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        let v = if idx + 1 < samples.len() {
            let a = samples[idx] as f64;
            let b = samples[idx + 1] as f64;
            a + (b - a) * frac
        } else if idx < samples.len() {
            samples[idx] as f64
        } else {
            continue;
        };
        out.push(v as f32);
    }
    out
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

    // Integration tests that depend on ft8-core's decode pipeline live in
    // `ft8-core/tests/resample_ft8.rs` (moved there alongside this module's
    // migration to mfsk-core).
}
