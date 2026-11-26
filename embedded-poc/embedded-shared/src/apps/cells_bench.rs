//! FT8 half-dB cell producer bench (CoreS3 / ESP32-S3 / LX7): the S3 plan's step 1
//! (`docs/notes/S3_CELLS_IMPLEMENTATION_PLAN.md` sections 4.2, 4.3, 9).
//!
//! What the host cannot say: the 2 560-point esp-dsp transform (`MixedRadix2560Sc16Fft`) against the library's software
//! `Plan2560Sc16` **on the same frames** (compared here, not assumed), the time of one row pair of the producer (Hann pack,
//! 2 560-point FFT, two-for-one demux, `half_db`, written with `HalfDbCells::row_mut` / `mark_filled`) against the
//! 160 ms a pair of rows covers (80 ms a row), and the memory the store and the staging take.
//!
//! The producer here is the plan's 4.3 written out (Hann over two symbols in Q15, `FP` gain 3/2 at 8 kHz, demux as
//! `spectrogram::spectrogram_cells`, `half_db` as `spectrogram::half_db`); it is a **copy** of the library's private
//! demux, so the cells it makes are not asserted equal to the library's, only esp-dsp against the software plan
//! through the *same* producer. The input is deterministic noise with a few tones (no recording: the comparison does
//! not depend on the signal). One core, one thread, no streaming, no second core: it measures the cost of a row pair,
//! not the slot's grid.
//!
//! Prints one `cells-bench:` line per fact; nothing is allocated per frame.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use mfsk_core::engine::dsp::fft_mixed_2560_sc16::Plan2560Sc16;
use mfsk_core::engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16;
use mfsk_core::engine::fft::{Fft16, default_planner_16};
use mfsk_core::ft8::decode_block::{Ft8Rate, HalfDbCells};
use num_complex::Complex;

const NSPS: usize = 1280;
const NFFT: usize = 2560;
const NSTEP: usize = 640;
const N_TIME: usize = 184;
const N_SAMPLES: usize = 120_000;
/// 200 Hz and 3 000 Hz in 3.125 Hz bins; the store's row is `HI - LO` = 936 bins.
const LO: usize = 64;
const HI: usize = 1000;
/// The pre-scale of the bench's frames (a real slot picks it from the peak: `gain_for_peak`).
const SHIFT: u32 = 3;

const HALF_DB_TABLE: [u8; 16] = [1, 3, 5, 7, 9, 10, 12, 13, 15, 16, 18, 19, 20, 21, 22, 24];

#[inline(always)]
fn half_db(p: u32) -> u8 {
    if p == 0 {
        return 0;
    }
    let l = 31 - p.leading_zeros();
    let m = if l >= 4 {
        (p >> (l - 4)) & 15
    } else {
        (p << (4 - l)) & 15
    } as usize;
    ((((l * 12332) >> 9) + HALF_DB_TABLE[m] as u32 + 2) >> 2).min(255) as u8
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn hann_q15() -> Vec<i16> {
    (0..NFFT)
        .map(|k| {
            let k = k.min(NFFT - k);
            let s = (core::f32::consts::PI * k as f32 / NFFT as f32).sin();
            (s * s * 32767.0 + 0.5) as i16
        })
        .collect()
}

/// Deterministic noise plus tones at 8 kHz, 15 s.
fn make_audio() -> Vec<i16> {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    (0..N_SAMPLES)
        .map(|n| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let noise = ((s >> 40) as i32 & 0x3FF) - 512;
            let mut v = noise as f32;
            for (f, a) in [(700.0f32, 600.0f32), (1500.0, 400.0), (2300.0, 250.0)] {
                v += a * (core::f32::consts::TAU * f * n as f32 / 8000.0).cos();
            }
            v as i16
        })
        .collect()
}

#[inline]
fn sample(audio: &[i16], i: i64) -> i16 {
    if i < 0 || i as usize >= audio.len() {
        0
    } else {
        audio[i as usize]
    }
}

/// One pair of rows: pack the two Hann frames as re / im, transform, demux into the two rows.
fn produce_pair(
    fft: &dyn Fft16,
    audio: &[i16],
    win: &[i16],
    jj: usize,
    buf: &mut [Complex<i16>],
    row_a: &mut [u8],
    row_b: &mut [u8],
) {
    let lead = (NSPS / 2) as i64;
    let (ia, ib) = (((2 * jj) * NSTEP) as i64 - lead, ((2 * jj + 1) * NSTEP) as i64 - lead);
    let scale = |raw: i16, k: usize| -> i16 {
        let v = (raw as i32) << SHIFT;
        let v = ((v as i64 * win[k] as i64) >> 15) as i32;
        (v * 3 / 2).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    };
    for (k, c) in buf.iter_mut().enumerate() {
        *c = Complex::new(
            scale(sample(audio, ia + k as i64), k),
            scale(sample(audio, ib + k as i64), k),
        );
    }
    fft.process(buf);
    for k in LO..HI {
        let yk = buf[k];
        let yn = buf[NFFT - k];
        let (ar, ai) = (
            (yk.re as i32 + yn.re as i32) >> 1,
            (yk.im as i32 - yn.im as i32) >> 1,
        );
        let (br, bi) = (
            (yk.im as i32 + yn.im as i32) >> 1,
            (yn.re as i32 - yk.re as i32) >> 1,
        );
        row_a[k - LO] = half_db((ar * ar) as u32 + (ai * ai) as u32);
        row_b[k - LO] = half_db((br * br) as u32 + (bi * bi) as u32);
    }
}

fn free_internal() -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_free_size((1 << 11) | (1 << 2)) }
}
fn free_psram() -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_free_size(1 << 10) }
}

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("cells-bench: {target}, FT8 8 kHz half-dB cell producer (S3 plan step 1)");
    log::info!(
        "cells-bench: free internal {} B, PSRAM {} B at start",
        free_internal(),
        free_psram()
    );
    let audio = make_audio();
    let win = hann_q15();
    let mut planner = default_planner_16();
    let esp = planner.plan_forward(NFFT);
    log::info!("cells-bench: plan_forward({NFFT}) ok (esp-dsp)");
    let soft = Plan2560Sc16::new();

    // 1. The transform: esp-dsp against the software plan, on the packed frames of 8 row pairs.
    let mut buf = vec![Complex::new(0i16, 0); NFFT];
    let mut buf2 = buf.clone();
    let (mut maxd, mut sumd, mut nd) = (0i32, 0i64, 0i64);
    // |esp - soft| by the size of the software value (max of |re|, |im|): <8, <64, <512, more
    let (mut bn, mut bs, mut bm) = ([0i64; 4], [0i64; 4], [0i32; 4]);
    let (mut t_esp, mut t_soft) = (0i64, 0i64);
    let (mut ra, mut rb) = (vec![0u8; HI - LO], vec![0u8; HI - LO]);
    for jj in 0..8 {
        // pack as the producer does, by running it once (the frame is left in `buf` before the FFT on a copy)
        let lead = (NSPS / 2) as i64;
        let (ia, ib) = (((2 * jj) * NSTEP) as i64 - lead, ((2 * jj + 1) * NSTEP) as i64 - lead);
        for k in 0..NFFT {
            let f = |raw: i16| -> i16 {
                let v = ((raw as i32) << SHIFT) as i64 * win[k] as i64 >> 15;
                (v as i32 * 3 / 2).clamp(i16::MIN as i32, i16::MAX as i32) as i16
            };
            buf[k] = Complex::new(f(sample(&audio, ia + k as i64)), f(sample(&audio, ib + k as i64)));
        }
        buf2.copy_from_slice(&buf);
        let t0 = now_us();
        esp.process(&mut buf);
        let t1 = now_us();
        soft.process(&mut buf2);
        let t2 = now_us();
        t_esp += t1 - t0;
        t_soft += t2 - t1;
        for k in 0..NFFT {
            for d in [
                (buf[k].re as i32 - buf2[k].re as i32).abs(),
                (buf[k].im as i32 - buf2[k].im as i32).abs(),
            ] {
                maxd = maxd.max(d);
                sumd += d as i64;
                nd += 1;
            }
            let mag = (buf2[k].re as i32).abs().max((buf2[k].im as i32).abs());
            let b = if mag < 8 { 0 } else if mag < 64 { 1 } else if mag < 512 { 2 } else { 3 };
            let d = (buf[k].re as i32 - buf2[k].re as i32).abs().max((buf[k].im as i32 - buf2[k].im as i32).abs());
            bn[b] += 1;
            bs[b] += d as i64;
            bm[b] = bm[b].max(d);
        }
    }
    for (i, name) in ["<8", "<64", "<512", ">=512"].iter().enumerate() {
        log::info!(
            "cells-bench: 2560 diff where |soft| {name}: {} bins, mean {:.2} LSB, max {}",
            bn[i],
            bs[i] as f32 / bn[i].max(1) as f32,
            bm[i]
        );
    }
    // Control: the same comparison at 3840 (the shipped esp-dsp plan against `Plan3840Sc16`), on the first 3840
    // samples of the same frames. If this differs as much, the gap is the esp-dsp kernel against the software
    // radix-2 stage, not the 2560 wrapper.
    {
        let esp3840 = planner.plan_forward(3840);
        let soft3840 = Plan3840Sc16::new();
        let (mut m, mut sm, mut n) = (0i32, 0i64, 0i64);
        let mut a = vec![Complex::new(0i16, 0); 3840];
        let mut b = a.clone();
        for jj in 0..8usize {
            for k in 0..3840 {
                let v = (sample(&audio, (jj * 3840 + k) as i64) as i32) << SHIFT;
                a[k] = Complex::new(v as i16, sample(&audio, (jj * 3840 + k + 7) as i64) << SHIFT);
            }
            b.copy_from_slice(&a);
            esp3840.process(&mut a);
            soft3840.process(&mut b);
            for k in 0..3840 {
                for d in [(a[k].re as i32 - b[k].re as i32).abs(), (a[k].im as i32 - b[k].im as i32).abs()] {
                    m = m.max(d);
                    sm += d as i64;
                    n += 1;
                }
            }
        }
        log::info!(
            "cells-bench: control 3840 esp-dsp vs Plan3840Sc16: max {m} LSB, mean {:.3} LSB",
            sm as f32 / n as f32
        );
    }
    log::info!(
        "cells-bench: fft esp-dsp {} us, software {} us (mean of 8), |esp - soft| max {} LSB, mean {:.3} LSB",
        t_esp / 8,
        t_soft / 8,
        maxd,
        sumd as f32 / nd as f32
    );
    let _ = (&mut ra, &mut rb);

    // 2. The producer into a `HalfDbCells`, row pair by row pair, esp-dsp; and the same through the software plan.
    struct Soft<'a>(&'a Plan2560Sc16);
    impl Fft16 for Soft<'_> {
        fn process(&self, b: &mut [Complex<i16>]) {
            self.0.process(b)
        }
        fn len(&self) -> usize {
            NFFT
        }
    }
    let mut store = Box::new(HalfDbCells::new(Ft8Rate::Hz8000, LO, HI - LO, N_TIME));
    let mut refstore = Box::new(HalfDbCells::new(Ft8Rate::Hz8000, LO, HI - LO, N_TIME));
    log::info!(
        "cells-bench: store {} B x2; free internal {} B, PSRAM {} B",
        store.size_bytes(),
        free_internal(),
        free_psram()
    );
    let mut stage = |fft: &dyn Fft16, st: &mut HalfDbCells, tag: &str| {
        let (mut tmax, mut tsum) = (0i64, 0i64);
        for jj in 0..N_TIME / 2 {
            let t0 = now_us();
            // two rows of the store, one after the other: `row_mut` borrows the store once at a time
            let mut a = vec_row();
            let mut b = vec_row();
            produce_pair(fft, &audio, &win, jj, &mut buf, &mut a, &mut b);
            st.row_mut(2 * jj).copy_from_slice(&a);
            st.row_mut(2 * jj + 1).copy_from_slice(&b);
            st.mark_filled(2 * jj + 2);
            let dt = now_us() - t0;
            tmax = tmax.max(dt);
            tsum += dt;
        }
        log::info!(
            "cells-bench: {tag}: 92 row pairs, mean {} us, max {} us a pair (a pair covers 160 000 us), filled {}",
            tsum / 92,
            tmax,
            st.filled_rows()
        );
    };
    stage(&*esp, &mut store, "esp-dsp");
    stage(&Soft(&soft), &mut refstore, "software");

    // 3. The cells: esp-dsp against software, row for row.
    let (mut ndiff, mut maxc, mut ncell) = (0usize, 0i32, 0usize);
    for row in 0..N_TIME {
        for bin in LO..HI {
            let (x, y) = (store.cell(row, bin).unwrap(), refstore.cell(row, bin).unwrap());
            ncell += 1;
            if x != y {
                ndiff += 1;
                maxc = maxc.max((x as i32 - y as i32).abs());
            }
        }
    }
    log::info!(
        "cells-bench: cells esp-dsp vs software: {ndiff} of {ncell} differ, max {maxc} half-dB units"
    );
    drop((store, refstore));

    // 4. Step 2, one core: a 12 kHz recording through `resample_to_8k` and the library's whole cells flow
    // (`decode_block_cells`: search over the cells, 15 candidates, the staircase without OSD, as the board ships), the
    // esp-dsp 2560 plan underneath (`default_planner_16`). The same call on the host is `tests/cells_flow_host_reference.rs`;
    // the texts must be the same set. On a thread of its own: the main task's stack is 16 KB.
    let h = std::thread::Builder::new()
        .name("cells_wav".into())
        .stack_size(48 * 1024)
        .spawn(move || wav_replay(wavs))
        .expect("cells-bench: could not spawn the replay thread");
    let _ = h.join();
    log::info!(
        "cells-bench: end; free internal {} B, PSRAM {} B",
        free_internal(),
        free_psram()
    );
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn vec_row() -> Vec<u8> {
    vec![0u8; HI - LO]
}

fn wav_replay(wavs: &'static [(&'static str, &'static [u8])]) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::ft8::decode::DecodeDepth;
    use mfsk_core::ft8::decode_block::{GatedSearch, decode_block_cells};
    use mfsk_core::msg::wsjt77::unpack77;

    for (label, wav) in wavs {
        let payload = &wav[44..];
        let slot12: Vec<i16> = payload
            .chunks_exact(2)
            .take(180_000)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        let t0 = now_us();
        let a8 = resample_to_8k(&slot12, 12_000);
        let t1 = now_us();
        let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
        for max_cand in [15usize, 30] {
            let mut texts: Vec<alloc::string::String> = Vec::new();
            let (mut t_run, mut n) = (0i64, 0usize);
            // cold, then warm: read the warm one
            for pass in 0..2 {
                let t2 = now_us();
                let out = decode_block_cells(&a8, Ft8Rate::Hz8000, &search, max_cand, DecodeDepth::EMBEDDED)
                    .expect("Hz8000 cells flow");
                t_run = now_us() - t2;
                if pass == 1 {
                    n = out.len();
                    texts = out.iter().filter_map(|d| unpack77(d.message77())).collect();
                    texts.sort();
                }
            }
            log::info!(
                "cells-bench: wav {label}: max_cand {max_cand}: decode_block_cells {t_run} us, {n} decodes (resample {} us)",
                t1 - t0
            );
            for t in &texts {
                log::info!("cells-bench: wav {label}: {max_cand}| {t}");
            }
        }
        log::info!(
            "cells-bench: wav {label}: free internal {} B, PSRAM {} B",
            free_internal(),
            free_psram()
        );
    }
}
