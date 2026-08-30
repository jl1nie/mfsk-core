//! Synthetic-WAV compute benchmark — runs `decode_block` on each of a
//! set of baked QSO recordings and logs per-stage timings + recovered
//! messages. Identical compute pipeline to `rx_wavsim`, minus the
//! streaming wav_sim/stage1_inc machinery.
//!
//! Per-target binaries (m5stack-core2 / m5stack-s3) supply the WAV
//! list and a target name string for the boot log.

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;

use mfsk_core::ft8::decode_block::compute_spectrogram;
use mfsk_core::msg::wsjt77::unpack77;

use crate::{dual_core, esp_dsp_fft};

/// Skip the OSD fallback (Bp staircase still runs all 4 LLR variants).
const OSD_ENABLED: bool = false;

/// Pass 1 candidate cap before Pass 2 `sync_quality_block0` re-rank.
const PASS1_LIMIT: usize = 30;

const SLOT_LEN: usize = 180_000;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn load_wav_slot(wav: &[u8]) -> Box<[i16]> {
    const HEADER: usize = 44;
    let payload = &wav[HEADER..];
    let n = payload.len() / 2;
    let mut slot: Vec<i16> = alloc::vec![0i16; SLOT_LEN];
    let copy_n = n.min(SLOT_LEN);
    for i in 0..copy_n {
        slot[i] = i16::from_le_bytes([payload[i * 2], payload[i * 2 + 1]]);
    }
    slot.into_boxed_slice()
}

/// Run the compute bench. `target_name` is purely for the boot log
/// Installs the `EspLogger` exactly once.
///
/// Must be used instead of `EspLogger::initialize_default` — a second
/// install `abort()`s the process, which is what a bin that wanted to
/// log something *before* calling [`run`] used to cause: boot loop,
/// panic at `esp-idf-svc/src/log.rs`, and nothing to say the two calls
/// were the problem. Same guard, same reason, as `ft4_bench`'s.
pub fn init_logger_once() {
    static READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if READY
        .compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        )
        .is_ok()
    {
        esp_idf_svc::log::EspLogger::initialize_default();
    }
}

/// (e.g. "m5stack-core2", "m5stack-s3"). `qso_wavs` is a
/// `&[(label, wav_bytes)]` slice the bench iterates over.
pub fn run(target_name: &str, qso_wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    log::info!("mfsk-core-{target_name} PoC starting");
    log::info!("mfsk-core version: {}", mfsk_core::VERSION);

    // Free heap (DRAM vs PSRAM) so we know the budget.
    const MALLOC_CAP_INTERNAL: u32 = 1 << 11;
    const MALLOC_CAP_8BIT: u32 = 1 << 2;
    const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
    unsafe {
        log::info!(
            "free heap: internal = {} KB (largest contig {} KB), PSRAM = {} KB",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            ) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
        );
    }

    log::info!(
        "decode_block: NFFT_SPEC={}, NMS α=0.75, Bp, parabolic-dt, Costas-first",
        mfsk_core::ft8::decode_block::NFFT_SPEC,
    );
    log::info!("baked WAVs: {}", qso_wavs.len());

    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);
    log::info!("FFT twiddle tables pre-warmed");

    dual_core::init();

    // max_cand sweep — host PASS1×max_cand sweep at PASS1=75 showed
    // identical 15/22 truth recall for max_cand ∈ {15, 20, 30}; only
    // the time changes (stage 3 work is per-cand). 15 is the floor;
    // 30 is the previous default (kept for direct A/B).
    const MAX_CAND_SWEEP: &[usize] = &[15, 20, 30];
    const DT_GRID_SWEEP: &[u8] = &[0];
    const DF_GRID_SWEEP: &[u8] = &[0];
    // q>12 (instead of q>6): trades weak-signal recall (~-1 truth)
    // for ~30-40% faster stage 3 and zero phantom CRC-passing decodes.
    const Q_THRESH_SWEEP: &[u32] = &[12];

    let slots: Vec<(&str, Box<[i16]>)> = qso_wavs
        .iter()
        .map(|(label, wav)| (*label, load_wav_slot(wav)))
        .collect();

    for &q_thresh in Q_THRESH_SWEEP {
        for &max_cand in MAX_CAND_SWEEP {
            for &dt_grid in DT_GRID_SWEEP {
                for &df_grid in DF_GRID_SWEEP {
                    log::info!("\n════════════════════════════════════════════");
                    log::info!(
                        "RUN: max_cand={max_cand}  dt_grid={dt_grid}  df_grid={df_grid}  q>{q_thresh}"
                    );
                    log::info!("════════════════════════════════════════════");
                    for (label, slot) in &slots {
                        log::info!("\nWAV: {label}");
                        decode_one(slot, max_cand, dt_grid, df_grid, q_thresh);
                    }
                }
            }
        }
    }

    log::info!("\n════════════════════════════════════════════");
    log::info!("FFI smoke: mfsk_ft8_decode_i16 (C ABI) on each WAV");
    log::info!("════════════════════════════════════════════");
    for (label, slot) in &slots {
        log::info!("\nWAV: {label} (via FFI)");
        ffi_smoke_one(slot);
    }

    log::info!("\n=== Sweep complete. Idling. ===");
    loop {
        unsafe {
            esp_idf_svc::sys::vTaskDelay(u32::MAX);
        }
    }
}

fn ffi_smoke_one(slot: &[i16]) {
    use mfsk_ft8::{
        mfsk_ft8_decode_i16, mfsk_ft8_options_free, mfsk_ft8_options_new,
        mfsk_ft8_result_list_free, MfskDecodeDepth, MfskResultList,
    };
    let mut results = MfskResultList {
        items: core::ptr::null_mut(),
        len: 0,
        _capacity: 0,
    };
    // issue #205: the five positional tuning knobs are now behind an
    // MfskDecodeOptions handle.
    let options = mfsk_ft8_options_new(100.0, 3_000.0, 1.0, 30, MfskDecodeDepth::BpAll);
    let t0 = now_us();
    let st = unsafe { mfsk_ft8_decode_i16(slot.as_ptr(), slot.len(), options, &mut results) };
    unsafe { mfsk_ft8_options_free(options) };
    let t1 = now_us();
    log::info!(
        "  ffi status={:?}  {:>3} result(s)  {:>8} us",
        st,
        results.len,
        t1 - t0,
    );
    if !results.items.is_null() {
        let items = unsafe { core::slice::from_raw_parts(results.items, results.len) };
        for (i, r) in items.iter().enumerate() {
            let bytes: &[u8] =
                unsafe { core::slice::from_raw_parts(r.text.as_ptr() as *const u8, r.text.len()) };
            let n = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
            let text = core::str::from_utf8(&bytes[..n]).unwrap_or("<bad utf8>");
            log::info!(
                "    [{i}]  {:>4.0} Hz  SNR={:>5.1} dB  e={}  '{}'",
                r.freq_hz,
                r.snr_db,
                r.hard_errors,
                text,
            );
        }
    }
    unsafe { mfsk_ft8_result_list_free(&mut results) };
}

/// Belief propagation in each scalar, same LLRs, same binary.
///
/// The other half of the `fixed-point` question. Stage 3 is 34 % of an
/// FT8 slot on this board (`docs/notes/FT4_BENCHMARK.md` §35.1) and
/// `fixed-point` puts the LLR ladder and the NMS check node in Q11i16
/// — but `LlrScalar` is a trait, `f32` implements it, and
/// `bp_decode_nms_with_scratch` and `BpScratch` are generic over it, so
/// unlike the spectrogram both versions can simply be instantiated.
///
/// FT4 runs its LLR/BP in f32 on this same board inside 479 ms, which
/// is why "the quantisation is not needed here" is a live question
/// rather than a rhetorical one.
///
/// The LLRs are synthetic — a scaled ramp, not a decode — because what
/// is being compared is the arithmetic, and both scalars see exactly
/// the same input. The iteration count is pinned so neither can win by
/// converging sooner: with `verify = None` and no valid codeword, both
/// run the full `max_iter`.
fn bp_scalar_ab() {
    use mfsk_core::engine::scalar::LlrScalar;
    use mfsk_core::fec::ldpc::bp::{BpScratch, bp_decode_nms_with_scratch};
    use mfsk_core::fec::ldpc::{LDPC_N, Ldpc174_91Params};

    const ITERS: usize = 200;
    const MAX_ITER: u32 = 30;
    const ALPHA: f32 = 0.75;

    // Same numeric values into both, converted through the trait.
    let mut src = [0.0f32; LDPC_N];
    for (i, v) in src.iter_mut().enumerate() {
        *v = ((i % 17) as f32 - 8.0) * 0.35;
    }

    fn run<T: LlrScalar>(src: &[f32; LDPC_N], label: &str) -> i64 {
        let mut llr = [T::default(); LDPC_N];
        for (d, &s) in llr.iter_mut().zip(src.iter()) {
            *d = T::from_f32(s);
        }
        let mut scratch: BpScratch<Ldpc174_91Params, T> = BpScratch::new();
        let t0 = now_us();
        let mut hits = 0usize;
        for _ in 0..ITERS {
            if bp_decode_nms_with_scratch::<T>(&mut scratch, &llr, None, MAX_ITER, None, ALPHA)
                .is_some()
            {
                hits += 1;
            }
        }
        let us = now_us() - t0;
        log::info!(
            "  BP {label:<8} {:>8} us / {ITERS} runs = {:>6} us/run  ({hits} converged)",
            us,
            us / ITERS as i64,
        );
        us
    }

    log::info!("  BP scalar A/B — {LDPC_N} LLRs, NMS a={ALPHA}, max_iter={MAX_ITER}");
    // Named directly rather than through `process_candidates`'s
    // private `LlrT` alias — which is what the shipped path resolves
    // to under `fixed-point`, and the type this probe is about.
    let fixed = run::<mfsk_core::engine::scalar::Q11i16>(&src, "Q11i16");
    let float = run::<f32>(&src, "f32");
    log::info!(
        "  BP scalar A/B — Q11i16 is {}.{:02}x f32",
        float / fixed.max(1),
        (float * 100 / fixed.max(1)) % 100,
    );
}

fn decode_one(slot: &[i16], max_cand: usize, _dt_grid: u8, _df_grid: u8, _q_thresh: u32) {
    bp_scalar_ab();

    let t0 = now_us();
    let spec = compute_spectrogram(slot, 3_000.0);
    let t1 = now_us();
    log::info!(
        "  stage 1 (spec):       {:>8} us  ({}× FFT, {} freq bins)",
        t1 - t0,
        spec.n_time,
        spec.n_freq,
    );

    // The same build, the other cell type. `fixed-point` exists to
    // halve this stage's PSRAM bandwidth and this stage is 51 % of an
    // FT8 slot, but the two builders are `#[cfg]` alternatives so no
    // binary ever held both — and the feature cannot be turned off to
    // compare, because `stage1_inc` needs the `Fft16` planner that
    // only exists under it.
    //
    // Not a clean u16-vs-f32 contrast: the f32 path windows
    // rectangular and the fixed-point one Hann, and the latter also
    // scans the slot to pick its shift. It is what each path costs.
    let t_f32 = now_us();
    let (nf, nt, data) =
        mfsk_core::ft8::decode_block::compute_spectrogram_f32_timed(slot, 3_000.0);
    let t_f32_end = now_us();
    log::info!(
        "  stage 1 f32 (A/B):    {:>8} us  ({nt}× FFT, {nf} freq bins, {} KB vs {} KB)          → fixed-point is {}.{:02}x",
        t_f32_end - t_f32,
        nf * nt * 4 / 1024,
        spec.n_freq * spec.n_time * core::mem::size_of_val(&spec.data[0]) / 1024,
        (t_f32_end - t_f32) / (t1 - t0).max(1),
        ((t_f32_end - t_f32) * 100 / (t1 - t0).max(1)) % 100,
    );
    drop(data);

    let t2 = now_us();
    let pass1 = dual_core::coarse_sync_split(&spec, 100.0, 3_000.0, 1.0, PASS1_LIMIT);
    let t3 = now_us();
    log::info!(
        "  stage 2 (sync):       {:>8} us  ({} cand)",
        t3 - t2,
        pass1.len(),
    );
    drop(spec);

    let t_pass2 = now_us();
    let pass2 = dual_core::pass2_split(slot, pass1, max_cand);
    let t_pass2_end = now_us();
    log::info!(
        "  pass 2 (re-rank):     {:>8} us  → top {} by sync_quality_block0",
        t_pass2_end - t_pass2,
        pass2.len(),
    );

    let depth = if OSD_ENABLED {
        mfsk_core::ft8::decode::DecodeDepth::FULL
    } else {
        mfsk_core::ft8::decode::DecodeDepth::BP_ONLY
    };
    let t4 = now_us();
    let results = dual_core::stage3_split(
        slot,
        pass2,
        depth,
        mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH,
        mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
    );
    let t5 = now_us();
    log::info!(
        "  stage 3 (refine+BP):  {:>8} us  ({} result(s))",
        t5 - t4,
        results.len(),
    );
    log::info!(
        "  ─── total decode:    {:>8} us = {:.3} s",
        t5 - t0,
        (t5 - t0) as f64 / 1e6,
    );
    for (i, r) in results.iter().enumerate() {
        if let Some(text) = unpack77(r.message77()) {
            log::info!(
                "    [{i}]  {:>4.0} Hz  SNR={:>5.1} dB  e={}  '{}'",
                r.freq_hz,
                r.snr_db,
                r.hard_errors,
                text
            );
        }
    }
}
