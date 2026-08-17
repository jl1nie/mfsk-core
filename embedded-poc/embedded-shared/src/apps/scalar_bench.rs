//! `LlrScalar` microbenchmark for ESP32-S3 (LX7) — issue #198 revisit,
//! via issue #306.
//!
//! ## Why this exists
//!
//! Issue #198 proposed genericizing `FecCodec::decode_soft` so BP
//! could run in fixed-point (`Q11i16`) through the shared FT4/FST4/
//! MSK144 pipeline — unlocking `bp_decode_generic_nms::<P, T: LlrScalar>`,
//! which already exists and already backs FT8's own bespoke fixed-point
//! path. The author explicitly deprioritized it: host benchmarks (x86
//! Ryzen, Apple Silicon) showed *no measurable difference* between
//! fixed-point and `f32` BP, because fixed-point doesn't reduce FLOP
//! count — it's the same arithmetic under a different representation,
//! and whether it's faster depends entirely on the target's relative
//! int/float performance. No benchmark for a real weak-FPU embedded
//! target existed anywhere in the codebase's history; the issue was
//! closed on that basis (sealing `FecCodec` as the minimum bar, full
//! genericization left as a stretch goal pending "a concrete weak-FPU
//! deployment target").
//!
//! FST4's issue #306 investigation found the candidate loop's dominant
//! cost is LLR/OSD, not BP — but fixed-point came up again as a
//! possible different-class lever once the small, disassembly-led
//! fixes (issue #306's five rounds) started showing diminishing
//! returns (~7.7× over budget after all five). Before spending a
//! session on the `FecCodec` genericization issue #198 scoped (real
//! numerical work: `BpScratch` ownership, `LlrT` wiring, recall-floor
//! verification across every protocol), issue #198's own closing
//! rationale says to check the premise first: **is `Q11i16` arithmetic
//! actually faster than `f32` on this specific silicon?**
//!
//! CoreS3 is ESP32-S3 / Xtensa LX7, which — unlike issue #198's
//! hypothetical weak-FPU ARM SBC — has a real single-precision FPU
//! (this session's own disassembly work found native `add.s`/`ule.s`/
//! `wfr` instructions throughout; the one costly float op found,
//! `f32::max`, was missing a *specific* instruction, not missing FPU
//! hardware generally). So the premise isn't obviously true here, and
//! this bench answers it directly rather than assuming either way.
//!
//! ## What it measures
//!
//! Not a generic "int vs float" strawman — the exact primitive
//! operations `bp_decode_generic_nms`'s two hottest per-element loops
//! (the min-sum check-node kernel, `fec/ldpc/bp.rs` around line 1234,
//! and the α-scaled check-to-variable update around line 1262) actually
//! call on every element, via the same `LlrScalar` trait both the real
//! BP kernel and this bench use: `is_negative`, `abs_sat`, `lt_total`
//! (×2, worst case), `mul_alpha`, `neg_sat`, `to_wide`, `wide_add`,
//! `wide_sub`, `from_wide_sat` — one of each, per bench iteration, for
//! `f32`, `Q11i16` (BP's actual fixed-point candidate) and `Q3i8`
//! (the narrower alternative already defined in `engine::scalar`).
//!
//! `core::hint::black_box` around the loop-carried accumulator and
//! input index keeps LLVM from folding the whole loop to a constant —
//! release + LTO would otherwise see straight through a fixed input.

use mfsk_core::engine::scalar::{LlrScalar, Q3i8, Q11i16};

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Install the ESP-IDF logger, at most once per boot — same guard
/// `fst4_bench`/`wspr_bench` use.
pub fn init_logger_once() {
    static LOGGER_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if LOGGER_READY
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

/// One synthetic "sweep" of representative `LlrScalar` ops, run `n`
/// times over a small rotating set of values (`core::hint::black_box`
/// keeps the index and the running accumulator from being constant-
/// folded). Mirrors `bp_decode_generic_nms`'s per-element op mix — see
/// the module doc for exactly which loops this is modelled on. Returns
/// `(elapsed_us, accumulator)`; the accumulator is logged by the
/// caller so the whole computation can't be dead-code-eliminated.
fn bench_ops<T: LlrScalar>(n: u32) -> (i64, i32) {
    // 8 representative values spanning sign and magnitude — enough
    // variety that min1/min2 tracking and the sign-xor path both take
    // real branches, not a single predictable one.
    let seeds: [f32; 8] = [3.5, -1.2, 0.0, 8.0, -8.0, 0.25, -0.25, 5.75];
    let vals: [T; 8] = seeds.map(T::from_f32);

    let t0 = now_us();
    let mut min1 = T::POS_INF_LIKE;
    let mut min2 = T::POS_INF_LIKE;
    let mut sign_xor = false;
    let mut acc = T::wide_zero();
    let mut idx: u32 = 0;
    for _ in 0..n {
        idx = core::hint::black_box(idx);
        let v = vals[(idx & 7) as usize];
        idx = idx.wrapping_add(1);

        if v.is_negative() {
            sign_xor = !sign_xor;
        }
        let av = v.abs_sat();
        if av.lt_total(min1) {
            min2 = min1;
            min1 = av;
        } else if av.lt_total(min2) {
            min2 = av;
        }

        let scaled = v.mul_alpha(0.75);
        let scaled = if sign_xor { scaled.neg_sat() } else { scaled };

        acc = T::wide_add(acc, scaled.to_wide());
        acc = T::wide_sub(acc, T::wide_zero());
        let _ = T::from_wide_sat(acc);
    }
    let elapsed = now_us() - t0;

    // Fold everything observable into one i32 so every code path above
    // is provably load-bearing for the return value, regardless of
    // T::Wide's concrete type.
    let acc_bits = if T::wide_is_positive(acc) { 1 } else { 0 }
        + if min1.lt_total(min2) { 2 } else { 0 }
        + if sign_xor { 4 } else { 0 };
    (elapsed, acc_bits + core::hint::black_box(idx) as i32)
}

pub fn run_bench() {
    const N: u32 = 20_000_000;

    log::info!("scalar_bench: {N} iterations per scalar type, issue #198/#306");

    let (t_f32, acc_f32) = bench_ops::<f32>(N);
    let (t_q11, acc_q11) = bench_ops::<Q11i16>(N);
    let (t_q3, acc_q3) = bench_ops::<Q3i8>(N);

    log::info!(
        "scalar_bench: f32    {} ms  ({:.1} ns/iter)  [acc={acc_f32}]",
        t_f32 / 1000,
        t_f32 as f64 * 1000.0 / N as f64,
    );
    log::info!(
        "scalar_bench: Q11i16 {} ms  ({:.1} ns/iter)  [acc={acc_q11}]",
        t_q11 / 1000,
        t_q11 as f64 * 1000.0 / N as f64,
    );
    log::info!(
        "scalar_bench: Q3i8   {} ms  ({:.1} ns/iter)  [acc={acc_q3}]",
        t_q3 / 1000,
        t_q3 as f64 * 1000.0 / N as f64,
    );
    log::info!(
        "scalar_bench: f32/Q11i16 ratio = {:.3}x  (>1 means fixed-point is faster)",
        t_f32 as f64 / t_q11 as f64,
    );
    log::info!(
        "scalar_bench: f32/Q3i8 ratio   = {:.3}x  (>1 means fixed-point is faster)",
        t_f32 as f64 / t_q3 as f64,
    );
    log::info!("=== scalar_bench complete ===");
}

pub const BENCH_STACK: u32 = 16 * 1024;

extern "C" fn bench_task(_arg: *mut core::ffi::c_void) {
    run_bench();
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// Spawns [`bench_task`] pinned to core 0, then idles forever — same
/// shape as `fst4_bench::run`, minus the baked-asset argument this
/// bench doesn't need (synthetic input only).
pub fn run() -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"scalar_bench".as_ptr(),
            BENCH_STACK,
            core::ptr::null_mut(),
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create scalar_bench task ({BENCH_STACK} B stack)");
    }

    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
