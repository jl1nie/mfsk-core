//! esp-dsp dot-product micro-bench (M5Stack CoreS3 / ESP32-S3 / LX7).
//!
//! Decides whether esp-dsp PIE is worth the layout work that
//! `PolyphaseResampler::dot` and `fst4_sync_search` would need, *before*
//! doing any of it. Issue #307.
//!
//! Both kernels are contiguous real dot products at their core, so
//! `dsps_dotprod_f32_aes3` (4-wide f32 PIE) is the candidate
//! replacement. But that kernel's own asm checks `len % 4 == 0` **and**
//! 16-byte alignment on *both* pointers, and branches to the scalar
//! `ae32` body if either fails — so the question is not "is PIE
//! faster" but "is PIE faster by enough to pay for guaranteeing
//! alignment", which neither call site gets for free:
//!
//! - `PolyphaseResampler::dot`'s window start advances by a Bresenham
//!   carry (7 or 8 for the sniper refine's L=9/M=64), so the history
//!   pointer lands on an arbitrary 4-byte offset.
//! - `fst4_sync_search`'s window start is `i0 + block_offset`, stepping
//!   by 4 in the coarse pass and by 1 in the fine pass.
//!
//! So this measures four things at the depths those call sites actually
//! use (107 = sniper refine, 422 = wideband refine, 288 =
//! `fst4_sync_search`'s Costas block), against the plain Rust loop the
//! crate ships today:
//!
//! 1. Rust scalar (what `PolyphaseResampler::dot` does now)
//! 2. `dsps_dotprod_f32_ansi`  — portable C reference
//! 3. `dsps_dotprod_f32_ae32`  — Xtensa scalar asm, no alignment need
//! 4. `dsps_dotprod_f32_aes3`  — LX7 PIE, aligned (fast path)
//! 5. `dsps_dotprod_f32_aes3`  — LX7 PIE, deliberately misaligned
//!    (confirms the silent scalar fallback rather than assuming it)
//!
//! Build: `cargo build --release --bin dotprod-bench`.

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

unsafe extern "C" {
    fn dsps_dotprod_f32_ansi(src1: *const f32, src2: *const f32, dest: *mut f32, len: i32) -> i32;
    fn dsps_dotprod_f32_ae32(src1: *const f32, src2: *const f32, dest: *mut f32, len: i32) -> i32;
    fn dsps_dotprod_f32_aes3(src1: *const f32, src2: *const f32, dest: *mut f32, len: i32) -> i32;
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// 16-byte-aligned internal-DRAM f32 buffer. `heap_caps_malloc` returns
/// at least 8-byte alignment; over-allocate and round up so the base is
/// genuinely 16-byte aligned rather than assumed to be.
struct Aligned {
    raw: *mut u8,
    ptr: *mut f32,
    len: usize,
}

impl Aligned {
    fn new(len: usize) -> Self {
        let bytes = len * 4 + 16;
        let raw = unsafe {
            esp_idf_svc::sys::heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
                as *mut u8
        };
        assert!(!raw.is_null(), "internal alloc failed");
        let off = (16 - (raw as usize % 16)) % 16;
        let ptr = unsafe { raw.add(off) } as *mut f32;
        assert_eq!(ptr as usize % 16, 0);
        Self { raw, ptr, len }
    }
    fn fill(&self, seed: f32) {
        for i in 0..self.len {
            // Deterministic, non-trivial, and bounded so the sums stay
            // comparable across variants.
            unsafe { *self.ptr.add(i) = ((i as f32 * 0.37 + seed).sin()) * 0.5 };
        }
    }
    fn as_slice(&self) -> &[f32] {
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }
}

impl Drop for Aligned {
    fn drop(&mut self) {
        unsafe { esp_idf_svc::sys::heap_caps_free(self.raw as *mut core::ffi::c_void) };
    }
}

/// The kernel `PolyphaseResampler::dot` ships today — one dependent
/// accumulator chain, which is also what `fst4_sync_search`'s inner
/// loop reduces to per channel.
fn rust_dot(a: &[f32], b: &[f32]) -> f32 {
    let mut s = 0.0f32;
    for k in 0..a.len() {
        s += a[k] * b[k];
    }
    s
}

fn bench_depth(depth: usize, iters: u32) {
    let x = Aligned::new(depth + 4);
    let h = Aligned::new(depth + 4);
    x.fill(0.11);
    h.fill(0.73);

    let xs = &x.as_slice()[..depth];
    let hs = &h.as_slice()[..depth];

    let mut acc = 0.0f32;
    let t0 = now_us();
    for _ in 0..iters {
        acc += rust_dot(xs, hs);
    }
    let t_rust = now_us() - t0;

    let mut out = 0.0f32;
    let t1 = now_us();
    for _ in 0..iters {
        unsafe { dsps_dotprod_f32_ansi(x.ptr, h.ptr, &mut out, depth as i32) };
        acc += out;
    }
    let t_ansi = now_us() - t1;

    let t2 = now_us();
    for _ in 0..iters {
        unsafe { dsps_dotprod_f32_ae32(x.ptr, h.ptr, &mut out, depth as i32) };
        acc += out;
    }
    let t_ae32 = now_us() - t2;

    // Aligned fast path: base is 16-byte aligned; `depth4` is rounded
    // up to a multiple of 4 so `len % 4 == 0` holds too. Rounding up
    // (rather than down) is what a real caller would do — the tap table
    // is zero-padded, so the extra taps contribute nothing.
    let depth4 = depth.div_ceil(4) * 4;
    let t3 = now_us();
    for _ in 0..iters {
        unsafe { dsps_dotprod_f32_aes3(x.ptr, h.ptr, &mut out, depth4 as i32) };
        acc += out;
    }
    let t_aes3 = now_us() - t3;

    // Misaligned: offset one float (4 bytes) so `(src1|src2) & 15 != 0`
    // and the asm takes its scalar fallback branch. Measuring this
    // rather than trusting the disassembly.
    let xm = unsafe { x.ptr.add(1) };
    let hm = unsafe { h.ptr.add(1) };
    let t4 = now_us();
    for _ in 0..iters {
        unsafe { dsps_dotprod_f32_aes3(xm, hm, &mut out, depth4 as i32) };
        acc += out;
    }
    let t_aes3_mis = now_us() - t4;

    let per = |t: i64| (t as f64 * 1000.0) / iters as f64; // ns/call
    log::info!(
        "depth {:4} x{} — rust {:7.0} ns | ansi {:7.0} | ae32 {:7.0} | \
         aes3(aligned) {:7.0} | aes3(misaligned) {:7.0} ns",
        depth,
        iters,
        per(t_rust),
        per(t_ansi),
        per(t_ae32),
        per(t_aes3),
        per(t_aes3_mis),
    );
    log::info!(
        "            speedup vs rust — ansi {:.2}x | ae32 {:.2}x | aes3 {:.2}x | aes3-mis {:.2}x",
        t_rust as f64 / t_ansi as f64,
        t_rust as f64 / t_ae32 as f64,
        t_rust as f64 / t_aes3 as f64,
        t_rust as f64 / t_aes3_mis as f64,
    );
    // Keep `acc` observable so nothing above is optimised away.
    log::info!("            (checksum {acc:.3})");
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== dotprod_bench: esp-dsp PIE vs scalar (issue #307) ===");
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // 107 = sniper refine `max_depth`; 288 = fst4_sync_search Costas
    // block; 422 = wideband refine `max_depth`.
    bench_depth(107, 20_000);
    bench_depth(288, 20_000);
    bench_depth(422, 20_000);

    log::info!("=== dotprod_bench complete ===");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
