//! Issue #567: the LPF subtract on a slot buffer shorter than the frame.
//!
//! A receiver that attaches mid-slot, or a sound card that drops a block,
//! hands the decoder less than one frame of audio. `subtractft8.f90` never
//! sees that case as a hazard: its FFT is `NFFT = NMAX = 15*12000`, always
//! longer than `NFRAME = 1920*79` (v3.2.0-rc1, lines 10-11), and a short
//! `dd` is just zero-filled. `apply_at_offset` sized the FFT from the
//! buffer instead, so on a short buffer it indexed past it and panicked.
//!
//! Three properties, each catching a different wrong fix:
//!
//! - `short_buffers_do_not_panic`: no geometry panics — negative, zero
//!   and positive `dt`, with and without end correction (the end
//!   correction panics at any `dt` on a short buffer), FT8 and FT4.
//! - `late_attach_subtracts_the_whole_overlap`: the subtract removes the
//!   signal everywhere the buffer holds it. Clamping the loop range to the
//!   buffer stops the panic but leaves the last `|signed_start|` samples
//!   untouched; this geometry puts 2.6 s of signal there.
//! - `full_slot_output_is_pinned`: a full-length slot comes out as it did
//!   before the fix, which is the decode path every golden test runs —
//!   within the last bit a platform's FFT kernel and libm leave (#579).

use mfsk_core::engine::dsp::subtract::{
    GfskParams, SubtractCfg, subtract_tones_lpf, subtract_tones_lpf_refine_dt,
};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;

#[allow(dead_code)]
mod common;

const FS: f32 = 12_000.0;
/// FT8 slot as the decoder normally receives it: 15 s at 12 kHz (`NMAX`).
const FT8_SLOT: usize = 180_000;
/// FT8 frame: 79 symbols x 1920 samples (`NFRAME`).
const FT8_FRAME: usize = 79 * 1920;
/// `lpf_half` the FT8 subtract uses (`NFILT = 4000`).
const FT8_LPF_HALF: usize = 2000;

/// As `ft8::subtract`'s private `FT8_CFG`.
const FT8_CFG: SubtractCfg = SubtractCfg {
    sample_rate: FS,
    tone_spacing_hz: 6.25,
    samples_per_symbol: 1920,
    base_offset_s: 0.5,
    gfsk: Some(GfskParams {
        bt: 2.0,
        hmod: 1.0,
        ramp_samples: 1920 / 8,
    }),
};

/// FT4's geometry: 103 symbols x 576 samples in a 7.5 s slot.
const FT4_CFG: SubtractCfg = SubtractCfg {
    sample_rate: FS,
    tone_spacing_hz: 12_000.0 / 576.0,
    samples_per_symbol: 576,
    base_offset_s: 0.5,
    gfsk: Some(GfskParams {
        bt: 1.0,
        hmod: 1.0,
        ramp_samples: 576 / 8,
    }),
};

/// A fixed, non-trivial FT8 tone sequence (Costas arrays included).
fn ft8_tones() -> Vec<u8> {
    let mut bits = [0u8; 77];
    let mut x: u32 = 0x5678_1234;
    for b in bits.iter_mut() {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *b = (x & 1) as u8;
    }
    message_to_tones::<Ft8>(&bits)
}

/// Runs `f` and reports whether it panicked. Each panic still prints its
/// message: the panic hook is process-wide, and silencing it here would
/// also hide the other tests' assertion messages while they run in
/// parallel.
fn no_panic(f: impl FnOnce() + std::panic::UnwindSafe) -> bool {
    std::panic::catch_unwind(f).is_ok()
}

#[test]
fn short_buffers_do_not_panic() {
    let ft8 = ft8_tones();
    let ft4: Vec<u8> = (0..103).map(|k| (k % 4) as u8).collect();
    let ft4_frame = 103 * 576;

    // Lengths around the frame and well below it; 130 284 is #567's own.
    let ft8_lens = [
        1,
        1_919,
        FT8_FRAME / 2,
        130_284,
        FT8_FRAME - 1,
        FT8_FRAME,
        FT8_FRAME + 1,
        FT8_SLOT,
    ];
    let ft4_lens = [1, ft4_frame / 2, ft4_frame - 1, ft4_frame, 90_000];
    let dts = [-2.5f32, -1.0, -0.5, 0.0, 0.5, 2.5];

    let mut failures = Vec::new();
    for (name, cfg, tones, lens) in [
        ("ft8", FT8_CFG, &ft8, &ft8_lens[..]),
        ("ft4", FT4_CFG, &ft4, &ft4_lens[..]),
    ] {
        for &len in lens {
            for &dt in &dts {
                for endc in [true, false] {
                    let t = tones.clone();
                    let ok = no_panic(move || {
                        let mut audio = vec![100i16; len];
                        subtract_tones_lpf(&mut audio, &t, 1000.0, dt, &cfg, 200, endc);
                    });
                    if !ok {
                        failures.push(format!("{name} lpf len={len} dt={dt} endcorr={endc}"));
                    }
                }
                let t = tones.clone();
                let ok = no_panic(move || {
                    let mut audio = vec![100i16; len];
                    subtract_tones_lpf_refine_dt(&mut audio, &t, 1000.0, dt, &cfg, 200, true);
                });
                if !ok {
                    failures.push(format!("{name} refine_dt len={len} dt={dt}"));
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} of the cases panicked:\n  {}",
        failures.len(),
        failures.join("\n  ")
    );
}

/// Energy of `a[r]` in dB relative to `b[r]`.
fn ratio_db(a: &[i16], b: &[i16], r: std::ops::Range<usize>) -> f64 {
    let e = |x: &[i16]| {
        x[r.clone()]
            .iter()
            .map(|&s| (s as f64).powi(2))
            .sum::<f64>()
    };
    10.0 * (e(a) / e(b)).log10()
}

#[test]
fn late_attach_subtracts_the_whole_overlap() {
    let tones = ft8_tones();
    let wave = synthesize_i16::<Ft8>(&tones, FS as u32, 1500.0, 3000);
    assert_eq!(wave.len(), FT8_FRAME);

    // The full slot: the frame at dt = 0, i.e. starting at 0.5 s.
    let dt_full = 0.0f32;
    let start_full = ((FT8_CFG.base_offset_s + dt_full) * FS).round() as usize;
    let mut full = vec![0i16; FT8_SLOT];
    full[start_full..start_full + FT8_FRAME].copy_from_slice(&wave);

    // The receiver attached 5 s late: the buffer is the slot's last 10 s,
    // and the frame now starts 54 000 samples before it.
    let late = 60_000usize;
    let short_orig = full[late..].to_vec();
    let dt_short = dt_full - late as f32 / FS;
    let signed_start = start_full as i64 - late as i64; // -54 000
    let in_short = 0..(signed_start + FT8_FRAME as i64) as usize; // 0..97 680
    // The samples a fix that clamps `i` to the buffer would never touch:
    // `j >= len + signed_start`, and the frame covers them up to 97 680.
    let tail = (short_orig.len() as i64 + signed_start) as usize..in_short.end; // 66 000..97 680
    assert!(tail.len() > 30_000, "geometry no longer exercises the tail");

    let mut full_sub = full.clone();
    subtract_tones_lpf(
        &mut full_sub,
        &tones,
        1500.0,
        dt_full,
        &FT8_CFG,
        FT8_LPF_HALF,
        true,
    );
    let mut short_sub = short_orig.clone();
    subtract_tones_lpf(
        &mut short_sub,
        &tones,
        1500.0,
        dt_short,
        &FT8_CFG,
        FT8_LPF_HALF,
        true,
    );

    // The same samples, seen through the full slot: what a correct
    // subtract achieves on them.
    let full_view = |r: &std::ops::Range<usize>| (r.start + late)..(r.end + late);
    let ref_tail = ratio_db(&full_sub, &full, full_view(&tail));
    let got_tail = ratio_db(&short_sub, &short_orig, tail.clone());
    let got_all = ratio_db(&short_sub, &short_orig, in_short.clone());

    eprintln!(
        "tail {} samples: {got_tail:.1} dB (full slot {ref_tail:.1} dB); whole overlap {got_all:.1} dB",
        tail.len()
    );
    assert!(
        ref_tail < -30.0,
        "the full-slot subtract itself only reaches {ref_tail:.1} dB on the tail"
    );
    assert!(
        got_tail < ref_tail + 6.0,
        "the last {} samples the frame covers were not subtracted: \
         {got_tail:.1} dB residual, against {ref_tail:.1} dB on a full slot",
        tail.len()
    );
    assert!(
        got_all < -20.0,
        "short-buffer subtract left {got_all:.1} dB overall"
    );
}

/// The full-slot subtract's output, `i16` little-endian, recorded on Apple M5
/// from `main` and from the commit before #567's fix (148a9e85), which agree
/// bit for bit there.
const PINNED_FULL_SLOT: &str = asset_path!("ft8_subtract_full_slot.bin");

/// How far a platform may sit from the pin (#579's rule: a few times the
/// largest gap measured). The output is `i16` after f32 work whose last bit
/// depends on rustfft's kernel (AVX2 on x86_64, NEON on aarch64, scalar
/// elsewhere) and on the libm, and a last bit on the wrong side of a
/// rounding edge moves a sample by 1. Measured on Apple M5, NEON against
/// rustfft's scalar kernel: 27 of the 180 000 samples differ, each by 1.
/// Shortening the LPF by one tap (`lpf_half` 1999), the smallest real change
/// tried, moves 263, also by 1 — so the count is the check, and the bound sits
/// between the two. The pin used to be a hash of every sample, written on
/// x86_64 Linux, which a single such sample breaks: it failed on Apple M5.
const MAX_SAMPLE_DIFF: i32 = 2;
const MAX_SAMPLES_DIFFERING: usize = 100;

#[test]
fn full_slot_output_is_pinned() {
    let Ok(bytes) = std::fs::read(PINNED_FULL_SLOT) else {
        common::skip_or_fail("ft8_subtract_full_slot.bin");
        return;
    };
    let pinned: Vec<i16> = bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&b| i16::from_le_bytes(b))
        .collect();
    let tones = ft8_tones();
    let wave = synthesize_i16::<Ft8>(&tones, FS as u32, 1500.0, 3000);
    // dt = -0.3: the frame starts 0.2 s into the slot and ends inside it.
    let start = ((FT8_CFG.base_offset_s - 0.3) * FS).round() as usize;
    let mut audio = vec![0i16; FT8_SLOT];
    audio[start..start + FT8_FRAME].copy_from_slice(&wave);
    // A little deterministic noise so the pin depends on the LPF, not just
    // on an exact cancellation.
    let mut x: u32 = 1;
    for s in audio.iter_mut() {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *s = s.saturating_add(((x >> 24) as i16) - 128);
    }
    subtract_tones_lpf(
        &mut audio,
        &tones,
        1500.0,
        -0.3,
        &FT8_CFG,
        FT8_LPF_HALF,
        true,
    );
    assert_eq!(audio.len(), pinned.len());
    let diffs: Vec<(usize, i32)> = audio
        .iter()
        .zip(&pinned)
        .enumerate()
        .map(|(i, (&g, &w))| (i, (g as i32 - w as i32).abs()))
        .filter(|&(_, d)| d != 0)
        .collect();
    let worst = diffs.iter().map(|&(_, d)| d).max().unwrap_or(0);
    eprintln!(
        "{} samples differ from the pin, by at most {worst}",
        diffs.len()
    );
    assert!(
        worst <= MAX_SAMPLE_DIFF,
        "full-slot subtract output changed: a sample moved by {worst} (first at {:?})",
        diffs.iter().find(|&&(_, d)| d == worst).map(|&(i, _)| i)
    );
    assert!(
        diffs.len() <= MAX_SAMPLES_DIFFERING,
        "full-slot subtract output changed: {} samples differ from the pin",
        diffs.len()
    );
}
