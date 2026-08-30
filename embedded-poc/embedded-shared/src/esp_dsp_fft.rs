//! `mfsk_core::engine::fft` backend bridged to Espressif `esp-dsp`.
//!
//! `esp-dsp` ships hand-written Xtensa assembly for the FFT (1.8-3×
//! the C reference on ESP32-S3). We expose it as an
//! [`mfsk_core::engine::fft::FftPlanner`] so `mfsk-core`'s decode
//! pipeline can use it without knowing it's there.
//!
//! ## Sizes
//!
//! `dsps_fft2r_fc32_ae32` is a radix-2 FFT and supports any
//! power-of-2 length up to `CONFIG_DSP_MAX_FFT_SIZE` — this repo's
//! `sdkconfig.defaults` sets `CONFIG_DSP_MAX_FFT_SIZE_8192`, so 8192
//! is the actual ceiling on every board here today. (An earlier
//! version of this comment claimed 32768 via a
//! `CONFIG_DSP_TABLE_SIZE_4096_TO_32768` symbol that doesn't exist in
//! the pinned `espressif/esp-dsp` 1.8.2 Kconfig — that symbol was
//! never real for this version, and the claim went unnoticed because
//! the one-shot-init bug `ensure_fc32_table` fixes, see
//! [`FC32_TABLE_LEN`], made `dsps_fft2r_init_fc32` short-circuit
//! before it ever reached its own `table_size > CONFIG_DSP_MAX_FFT_SIZE`
//! bounds check whenever a smaller size like `coarse_baseband`'s 512
//! had already been requested first — so a genuine call for 32768 was
//! never actually *tried* on real hardware until that fix went in.
//! Raise `CONFIG_DSP_MAX_FFT_SIZE_32768` in `sdkconfig.defaults`
//! first if a future caller genuinely needs a plan past 8192.)
//! Plans for non-power-of-2 sizes panic — `mfsk-core`'s wide-band FFT
//! cache (192 000 for FT8 / 92 160 for FT4) is unsupported, but the
//! narrow-band sniper / WSPR aligned paths fit comfortably under the
//! 8192-point cap.
//!
//! ## Memory
//!
//! `dsps_fft2r_init_fc32` allocates a twiddle table the first time;
//! subsequent plans reuse it. We initialise on the first
//! `plan_forward` / `plan_inverse` call.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicUsize, Ordering};

use mfsk_core::engine::fft::{Fft, Fft16, FftPlanner, FftPlanner16};
use num_complex::{Complex, Complex32};
// `f32::cos`/`sin` in `DirectDft::new` below resolve without this in
// every feature combination this crate currently builds under (Cargo
// feature unification means some other crate in the same build graph
// — e.g. a sibling's `mfsk-core/std` — ends up making them available
// here too, even though this crate is itself `#![no_std]`). Kept
// explicit anyway, same as `mfsk-core`'s own identical dependency
// line, so a future build that genuinely has no `std` anywhere in its
// graph doesn't silently lose trig — `#[allow]` rather than deleting
// an import whose necessity depends on what else happens to be linked.
#[allow(unused_imports)]
use num_traits::Float;

/// Four `Complex32` under a 16-byte alignment guarantee.
///
/// The LX7 PIE kernels (`_aes3_`) move float **pairs** with
/// `ee.ldf.64.ip` / `ee.stf.64.ip`, which require an 8-byte-aligned
/// address. `Complex32` is `repr(C)` over two `f32`, so its alignment
/// is 4 — a `Vec<Complex32>` is free to land on a 4-mod-8 boundary,
/// and when it does, the kernel's very first `ee.stf.64.ip` writes
/// four bytes *below* the allocation, directly into the preceding TLSF
/// block header. The failure surfaces much later as `StoreProhibited`
/// inside an unrelated `tlsf_free`, with the header holding what are
/// recognisably float samples.
///
/// Found 2026-08-13 by the WSPR candidate-loop bench (issue #260),
/// whose `coarse_baseband` plans a 512-point FFT over a plain
/// `vec![Complex::new(0.0, 0.0); 512]`. The FT8 path never tripped it
/// because its only PIE FFT is the 256-point inner stage of
/// `MixedRadix3840Fft`, working on rows *inside* a 3840-element
/// buffer: a 4-byte underrun there lands in the same allocation and is
/// invisible. That makes this a latent bug in the shipped FT8 path
/// too, one allocator decision away from the same corruption — the
/// staging below covers both.
#[repr(align(16))]
#[derive(Clone, Copy)]
struct Align16Quad(
    // Read only through the reinterpreting slice in
    // `AlignedStaging::as_slice` — the field exists to size and align
    // the backing store.
    #[allow(dead_code)] [Complex32; 4],
);

impl Align16Quad {
    const ZERO: Self = Self([Complex32::new(0.0, 0.0); 4]);
}

/// Lazily-grown 16-byte-aligned staging for one FFT.
///
/// Only used when the caller's buffer is not already 16-byte aligned,
/// so the aligned-input case keeps the in-place zero-copy path and the
/// timings this backend is measured with stay honest.
struct AlignedStaging {
    quads: Vec<Align16Quad>,
}

impl AlignedStaging {
    const fn new() -> Self {
        Self { quads: Vec::new() }
    }

    /// `len` must be a multiple of 4 — true for every power-of-2 ≥ 4,
    /// which is all this backend plans.
    fn as_slice(&mut self, len: usize) -> &mut [Complex32] {
        debug_assert_eq!(len % 4, 0);
        if self.quads.len() * 4 < len {
            self.quads.resize(len / 4, Align16Quad::ZERO);
        }
        // SAFETY: `Align16Quad` is `repr(align(16))` over
        // `[Complex32; 4]`, so the backing store is exactly `len`
        // contiguous `Complex32` with 16-byte alignment.
        unsafe { core::slice::from_raw_parts_mut(self.quads.as_mut_ptr() as *mut Complex32, len) }
    }
}

/// Is `buf` safe to hand a PIE kernel directly?
fn pie_aligned(buf: &[Complex32]) -> bool {
    (buf.as_ptr() as usize) % 16 == 0
}

/// How often the PIE path got a caller buffer it could use in place,
/// versus how often it paid [`AlignedStaging`]'s copy-in/copy-out.
///
/// Diagnostic for issue #260's open question: the FFT-based LPF in
/// `wspr::subtract` sped the subtract step up only 3-4 %, far less
/// than its flop count predicted, and an unaligned `Vec<Complex32>`
/// falling back to staging on every call is the leading suspect. That
/// is a *hypothesis about the allocator* — whether a 64 KB
/// `Vec<Complex32>` actually lands off a 16-byte boundary is not
/// something host code can answer — so it gets measured before
/// anything is rewritten to chase it.
///
/// `*_LEN_MASK` is an OR of the observed lengths, not a count. Every
/// length this backend plans is a power of two, so the mask reads back
/// as an exact set of lengths without needing a per-length table.
///
/// **The three mixed-radix kernels report their inner rows too**
/// (added 2026-08-30). Until then only the plain radix-2 path did, so
/// this report was blind to `fft_mixed_2304` / `_3840` / `_5120` —
/// i.e. to every transform FT4 and FT8 actually run on this board, and
/// to the one whose 1 288 ms coarse stage prompted the question
/// (`docs/notes/FT4_BENCHMARK.md` §25). Their inner lengths collide in
/// the mask (256 for 2304 and 3840, 1024 for 5120, both also legal
/// standalone plans), so attribution comes from *when* the counters
/// move rather than from the mask: read the report either side of a
/// stage that runs only one kernel.
///
/// Two relaxed atomics per call, and only under `aes3` — negligible
/// against even the 256-point kernel they sit in front of.
#[cfg(feature = "aes3")]
static PIE_ALIGNED_CALLS: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "aes3")]
static PIE_ALIGNED_LEN_MASK: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "aes3")]
static PIE_STAGED_CALLS: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "aes3")]
static PIE_STAGED_LEN_MASK: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "aes3")]
fn record_pie_path(len: usize, aligned: bool) {
    let (calls, mask) = if aligned {
        (&PIE_ALIGNED_CALLS, &PIE_ALIGNED_LEN_MASK)
    } else {
        (&PIE_STAGED_CALLS, &PIE_STAGED_LEN_MASK)
    };
    calls.fetch_add(1, Ordering::Relaxed);
    mask.fetch_or(len, Ordering::Relaxed);
}

/// Microseconds attributed to the three parts of a mixed-radix
/// transform, so a slow one can be blamed on the right layer.
///
/// `docs/notes/FT4_BENCHMARK.md` §25.4 established that
/// `ft4_coarse_sync`'s 1 288 ms is not the PIE staging copy — all
/// 1 368 of its inner rows already run in place — which leaves "the
/// wrapper" as the answer by elimination. Elimination is not a
/// measurement, and "the wrapper" is three different things: the
/// esp-dsp kernel itself, the staging copies when they do happen, and
/// the Cooley-Tukey combine (twiddles, gather/scatter) in
/// `mfsk_core::engine::dsp::fft_mixed_*`. These split them:
///
/// - [`PIE_PROCESS_US`] — whole `Fft::process` call
/// - [`PIE_KERNEL_US`] — inside `dsps_fft2r_fc32_*` + `bit_rev`
/// - [`PIE_STAGING_US`] — the copy-in/copy-out, when taken
///
/// so combine = process − kernel − staging, and anything the caller
/// spends *outside* `process` (for the coarse stage: the Nuttall
/// window and the magnitude accumulation) is its own wall-clock minus
/// process.
///
/// Cost of the probe itself: two `esp_timer_get_time` reads per inner
/// row and two per transform. At the coarse stage's 1 368 rows that is
/// under a millisecond against 1 288 — checked against the
/// "diagnostic probe must not perturb" list in `embedded-poc/CLAUDE.md`:
/// no allocation, no locking, no new `static` mutex, nothing on the
/// stack beyond a couple of `i64`.
#[cfg(feature = "aes3")]
static PIE_PROCESS_US: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "aes3")]
static PIE_KERNEL_US: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "aes3")]
static PIE_STAGING_US: AtomicUsize = AtomicUsize::new(0);

/// Monotonic microseconds, or 0 when the probe is compiled out.
#[inline(always)]
fn probe_us() -> i64 {
    #[cfg(feature = "aes3")]
    {
        unsafe { esp_idf_svc::sys::esp_timer_get_time() }
    }
    #[cfg(not(feature = "aes3"))]
    {
        0
    }
}

#[inline(always)]
#[allow(unused_variables)]
fn add_kernel_us(us: i64) {
    #[cfg(feature = "aes3")]
    PIE_KERNEL_US.fetch_add(us.max(0) as usize, Ordering::Relaxed);
}

#[inline(always)]
#[allow(unused_variables)]
fn add_staging_us(us: i64) {
    #[cfg(feature = "aes3")]
    PIE_STAGING_US.fetch_add(us.max(0) as usize, Ordering::Relaxed);
}

#[inline(always)]
#[allow(unused_variables)]
fn add_process_us(us: i64) {
    #[cfg(feature = "aes3")]
    PIE_PROCESS_US.fetch_add(us.max(0) as usize, Ordering::Relaxed);
}

/// `(process_us, kernel_us, staging_us)` — see [`PIE_PROCESS_US`].
/// Mixed-radix kernels only; the plain radix-2 path does not time
/// itself. All zero when `aes3` is off.
pub fn pie_timing_report() -> (usize, usize, usize) {
    #[cfg(feature = "aes3")]
    {
        (
            PIE_PROCESS_US.load(Ordering::Relaxed),
            PIE_KERNEL_US.load(Ordering::Relaxed),
            PIE_STAGING_US.load(Ordering::Relaxed),
        )
    }
    #[cfg(not(feature = "aes3"))]
    {
        (0, 0, 0)
    }
}

/// `(aligned_calls, aligned_len_mask, staged_calls, staged_len_mask)`
/// — see [`record_pie_path`]. All zero when `aes3` is off, where the
/// non-PIE kernel has no alignment requirement to begin with.
pub fn pie_alignment_report() -> (usize, usize, usize, usize) {
    #[cfg(feature = "aes3")]
    {
        (
            PIE_ALIGNED_CALLS.load(Ordering::Relaxed),
            PIE_ALIGNED_LEN_MASK.load(Ordering::Relaxed),
            PIE_STAGED_CALLS.load(Ordering::Relaxed),
            PIE_STAGED_LEN_MASK.load(Ordering::Relaxed),
        )
    }
    #[cfg(not(feature = "aes3"))]
    {
        (0, 0, 0, 0)
    }
}

/// Factory called by `mfsk_core::engine::fft::default_planner()` when
/// the crate is built with `fft-extern` (and no built-in backend like
/// `fft-rustfft`). Symbol name + signature are the link-time contract;
/// see `mfsk_core::engine::fft::default_planner` for the spec.
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner> {
    Box::new(EspDspPlanner::new())
}

/// Initialise both twiddle tables (f32 + i16) for the given size up
/// front. Call once from `main()` before spawning any task. After this
/// the global tables are read-only and concurrent FFT calls from
/// pinned-to-core workers cannot race in `dsps_fft2r_init_*`.
///
/// `len` must be a power of two; pass the largest size the pipeline
/// will use (NFFT_SPEC = 4096 for FT8 decode_block).
pub fn prewarm(len: usize) {
    // The mixed-radix 3840 path (FT8 NFFT_SPEC) uses a 256-pt esp-dsp
    // sub-kernel; non-power-of-two sizes are routed through it. Map
    // them to the inner radix-2 size before initialising the table.
    let radix2_len = if len.is_power_of_two() {
        len
    } else if len == 3840 {
        256
    } else {
        panic!("prewarm: unsupported len {len}");
    };
    // Route through the tracked helpers (not the raw FFI calls) so
    // `FC32_TABLE_LEN` / `SC16_TABLE_LEN` agree with what's actually
    // initialised — see `ensure_fc32_table`'s doc comment. A caller
    // that later requests a *different* size still gets a real
    // regenerate instead of a silent no-op.
    ensure_fc32_table(radix2_len);
    ensure_sc16_table(radix2_len);
}

/// i16 sibling factory for the `fixed-point` build path. Wraps
/// `dsps_fft2r_sc16_ae32_` from the same managed component.
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner_16() -> Box<dyn FftPlanner16> {
    Box::new(EspDspPlanner16::new())
}

// Manual FFI declarations against `esp-dsp` (the IDF managed component
// pulled by `idf_component.yml`). esp-idf-sys's auto-bindgen doesn't
// cover esp-dsp's headers by default, so we declare just the four
// symbols we need. Signatures match
// `components/dsp/modules/fft/float/dsps_fft2r_fc32_*.{h,c}` in the
// upstream esp-dsp 1.4 source.
const ESP_OK: i32 = 0;

/// Decode an `esp-dsp` error code for panic messages. Values from
/// `modules/common/include/dsp_err_codes.h` in the vendored source
/// (`ESP_ERR_DSP_BASE = 0x70000`); not exposed as an FFI symbol, so
/// hand-transcribed here rather than declared `extern`.
fn describe_esp_dsp_err(code: i32) -> &'static str {
    match code {
        0x70001 => "ESP_ERR_DSP_INVALID_LENGTH (not a power of 2?)",
        0x70002 => "ESP_ERR_DSP_INVALID_PARAM",
        0x70003 => {
            "ESP_ERR_DSP_PARAM_OUTOFRANGE (len > CONFIG_DSP_MAX_FFT_SIZE — \
                     raise the CONFIG_DSP_MAX_FFT_SIZE_* choice in sdkconfig.defaults)"
        }
        0x70004 => "ESP_ERR_DSP_UNINITIALIZED",
        0x70005 => "ESP_ERR_DSP_REINITIALIZED",
        0x70006 => "ESP_ERR_DSP_ARRAY_NOT_ALIGNED",
        _ => "unknown esp-dsp error code",
    }
}

unsafe extern "C" {
    /// Pre-compute the twiddle table for radix-2 FFTs up to `table_size`
    /// points. Pass a NULL buffer to let the lib `malloc` its own table.
    fn dsps_fft2r_init_fc32(fft_table_buff: *mut f32, table_size: i32) -> i32;

    /// Forward radix-2 FFT, in place. `data` is interleaved
    /// `{re, im, ...}` with `2 * N` floats; `N` must be a power of
    /// 2 ≤ the initialised table size; `w` is the global twiddle
    /// table populated by `dsps_fft2r_init_fc32`.
    ///
    /// **Three-argument signature.** From `dsps_fft2r.h`:
    /// ```c
    /// esp_err_t dsps_fft2r_fc32_ae32_(float *data, int N, float *w);
    /// #define dsps_fft2r_fc32_ae32(data, N) \
    ///     dsps_fft2r_fc32_ae32_(data, N, dsps_fft_w_table_fc32)
    /// ```
    /// An earlier version of this file declared a 2-arg form (the
    /// algorithm pseudocode at the top of `dsps_fft2r_fc32_ae32_.S`
    /// shows a 2-arg signature, but that's the C-equivalent
    /// description, not the linkage). Calling with 2 args leaves
    /// register `a4` (the `w` pointer) uninitialised — the asm
    /// then reads from a garbage address inside the inner butterfly
    /// loop and crashes with `LoadProhibited` on real hardware.
    /// LX6 baseline radix-2 fc32 FFT. Active when `aes3` feature is off.
    #[cfg(not(feature = "aes3"))]
    fn dsps_fft2r_fc32_ae32_(data: *mut f32, N: i32, w: *const f32) -> i32;

    /// LX7 PIE radix-2 fc32 FFT — ~2× throughput on ESP32-S3.
    #[cfg(feature = "aes3")]
    fn dsps_fft2r_fc32_aes3_(data: *mut f32, N: i32, w: *const f32) -> i32;

    /// Bit-reverse the radix-2 output into natural order. Call after
    /// `dsps_fft2r_fc32_ae32_` / `_aes3_`.
    fn dsps_bit_rev_fc32_ansi(data: *mut f32, N: i32) -> i32;

    /// Twiddle-factor table allocated and populated by
    /// `dsps_fft2r_init_fc32`. Pointer becomes valid after init.
    static dsps_fft_w_table_fc32: *const f32;

    /// Frees the table and clears `dsps_fft2r_initialized`, so the
    /// *next* `dsps_fft2r_init_fc32` call actually regenerates
    /// instead of no-opping. See [`ensure_fc32_table`].
    fn dsps_fft2r_deinit_fc32();

    // ── i16 (sc16) variants ──────────────────────────────────
    fn dsps_fft2r_init_sc16(fft_table_buff: *mut i16, table_size: i32) -> i32;
    /// sc16 sibling of `dsps_fft2r_deinit_fc32`.
    fn dsps_fft2r_deinit_sc16();
    fn dsps_fft2r_sc16_ae32_(data: *mut i16, N: i32, w: *const i16) -> i32;
    /// LX7 PIE sc16 FFT. Requires 16-byte aligned input — caller must use
    /// an aligned buffer (see `Aligned16Rows` in `MixedRadix3840Sc16Fft`).
    #[cfg(feature = "aes3")]
    fn dsps_fft2r_sc16_aes3_(data: *mut i16, N: i32, w: *const i16) -> i32;
    fn dsps_bit_rev_sc16_ansi(data: *mut i16, N: i32) -> i32;
    static dsps_fft_w_table_sc16: *const i16;
}

/// Exact length the process-global esp-dsp fc32 twiddle table is
/// *actually* built for right now. `0` = never initialised.
///
/// This has to live here, at module (process) scope, rather than on
/// [`EspDspPlanner`] — `dsps_fft2r_init_fc32` guards its work with a
/// **C file-scope static** (`dsps_fft2r_initialized` in
/// `dsps_fft2r_fc32_ansi.c`), not anything keyed by the `table_size`
/// argument:
/// ```c
/// esp_err_t dsps_fft2r_init_fc32(float *fft_table_buff, int table_size) {
///     if (dsps_fft2r_initialized != 0) { return result; }  // <- no-op regardless of table_size
///     ...
///     dsps_fft_w_table_size = table_size;
///     result = dsps_gen_w_r2_fc32(dsps_fft_w_table_fc32, dsps_fft_w_table_size);
///     dsps_fft2r_initialized = 1;
/// }
/// ```
/// So *whichever* size is requested **first, from anywhere in the
/// process**, wins forever — every later call for a *different* size
/// silently returns `ESP_OK` without touching the table, and the
/// kernel (`dsps_fft2r_fc32_ae32_`/`_aes3_`) then runs against a
/// twiddle table sized (and allocated!) for the wrong N. For a
/// smaller N this reads nonsense angles (the front slice of a bigger
/// table is not a smaller table); for a bigger N it reads **out of
/// bounds of the original, too-small allocation**.
///
/// `EspDspPlanner::ensure_table` used to track only its own
/// `initialised_max` and assume `dsps_fft2r_init_fc32` would "grow"
/// the table on a bigger request — a reasonable-sounding API but not
/// what the C side does, and doubly misleading because a *fresh*
/// `EspDspPlanner` (the common case — see `downsample.rs`'s
/// `with_default_planner` on `no_std`, which constructs one per
/// call) starts remembering nothing, so it can't even self-consistently
/// detect "the process-wide table is already the wrong size for me".
/// Found 2026-08-14 (issue #260): the WSPR overlap-save LPF experiment
/// planned a 32768-pt FFT into the same process as `coarse_baseband`'s
/// existing 512-pt plans; the Rust wrapper only ever saw the 512-pt
/// table it *thought* had grown, while the kernel actually walked off
/// the end of the original 512-entry allocation. Three independent,
/// real fixes to the LPF code itself (kernel-placement off-by-one,
/// the 32768-pt hardware ceiling, forward/inverse normalisation) left
/// the on-device symptom byte-for-byte unchanged, because none of
/// them touched this.
///
/// The fix: track the *actually-initialised* size at module (i.e.
/// process) scope, matching where the real C-side state lives, and
/// force a real regenerate — `dsps_fft2r_deinit_fc32()` then
/// `dsps_fft2r_init_fc32()` — whenever the requested size differs
/// from it, not just when it's bigger.
///
/// **Caveat**: `dsps_fft2r_initialized` and friends are plain C
/// globals with no locking. If two cores ever plan *different* fc32
/// sizes concurrently (they don't today — `wspr_dual_core`'s workers
/// share one post-subtract candidate loop with no FFT in it, and FT8
/// decode is single-core through the spectrogram stage), the
/// deinit/reinit dance here would race exactly like the original bug
/// did. Not fixed here; flagging for whoever adds the next
/// concurrent FFT caller.
static FC32_TABLE_LEN: AtomicUsize = AtomicUsize::new(0);

/// Serialises the twiddle table against the transforms that read it.
///
/// [`FC32_TABLE_LEN`]'s doc comment flagged this hazard and left it
/// unfixed, on the correct observation that nothing then planned two
/// fc32 sizes concurrently. The FST4 receiver app does: its capture
/// task builds the coarse spectrogram at 2048 points on one core while
/// the decode runs `fst4_ddc_snr_db`'s 512-point Welch periodogram on
/// the other. The failure is not subtle once you look for it, and not
/// random either — 111 of 1888 spectrogram bins came back non-finite,
/// the *same* count every slot, because a 2048-point kernel reading a
/// 512-entry table walks off the same end of the same allocation every
/// time. Slot 0 was always clean: nothing had planned the second size
/// yet.
///
/// A plain spin lock rather than a FreeRTOS mutex: this guards a few
/// hundred microseconds of arithmetic, is never taken from an ISR, and
/// must work identically on both cores before any scheduler object is
/// necessarily available.
static FC32_LOCK: AtomicUsize = AtomicUsize::new(0);

struct Fc32Guard;

impl Fc32Guard {
    fn acquire() -> Self {
        while FC32_LOCK
            .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
        Self
    }
}

impl Drop for Fc32Guard {
    fn drop(&mut self) {
        FC32_LOCK.store(0, Ordering::Release);
    }
}

/// Force the process-global fc32 twiddle table to be exactly `len`
/// entries, regenerating it if it currently isn't. See
/// [`FC32_TABLE_LEN`] for why this can't just be "call init if we
/// haven't seen this size before" on a per-planner-instance basis.
///
/// **Caller must hold [`Fc32Guard`]** and keep holding it until the
/// transform that reads the table has finished — resizing it is only
/// half the invariant.
fn ensure_fc32_table(len: usize) {
    let _guard = Fc32Guard::acquire();
    ensure_fc32_table_locked(len);
}

/// [`ensure_fc32_table`]'s body, for callers already holding
/// [`Fc32Guard`] because they are about to run a transform against the
/// table and must not let it change underneath them.
fn ensure_fc32_table_locked(len: usize) {
    if FC32_TABLE_LEN.load(Ordering::Relaxed) == len {
        return;
    }
    // SAFETY: `dsps_fft2r_deinit_fc32` is safe to call even when
    // nothing was ever initialised (it checks its own "allocated"
    // flag first) and NULL asks the lib to alloc+size its own table
    // for exactly `len`, a power of 2 ≤ `CONFIG_DSP_MAX_FFT_SIZE`
    // (`CONFIG_DSP_TABLE_SIZE_*` in sdkconfig.defaults).
    unsafe {
        dsps_fft2r_deinit_fc32();
        let r = dsps_fft2r_init_fc32(core::ptr::null_mut(), len as i32);
        assert_eq!(
            r,
            ESP_OK,
            "dsps_fft2r_init_fc32({len}) returned {r} = {}",
            describe_esp_dsp_err(r)
        );
    }
    FC32_TABLE_LEN.store(len, Ordering::Relaxed);
}

/// `esp-dsp` FFT planner. Construct once per session, share across
/// all decode invocations so the twiddle table inits exactly once —
/// or construct fresh per call (`no_std`'s `with_default_planner`
/// does); either way is safe, since the table itself is tracked at
/// process scope by [`FC32_TABLE_LEN`], not on this struct. See
/// [`FC32_TABLE_LEN`] for why a per-instance cache alone is not
/// enough.
pub struct EspDspPlanner;

impl EspDspPlanner {
    pub fn new() -> Self {
        Self
    }

    fn ensure_table(&mut self, len: usize) {
        ensure_fc32_table(len);
    }
}

impl Default for EspDspPlanner {
    fn default() -> Self {
        Self::new()
    }
}

/// Naive O(N²) forward DFT — the textbook definition,
/// `X[k] = Σₙ x[n]·e^{-2πikn/N}`, unnormalised (matches `rustfft`'s
/// own forward-transform convention, which is what every existing
/// caller of this planner was already verified against on host). Not
/// an approximation or a novel algorithm — every FFT computes exactly
/// this sum by a faster route, so implementing the sum directly
/// carries no algorithm-specific correctness risk the way a new
/// fast-transform derivation would.
///
/// Exists for lengths esp-dsp's radix-2 kernel can't serve
/// (non-power-of-2, and not `3840`'s hand-rolled mixed-radix case)
/// but that are cheap enough not to need a real fast algorithm at
/// all — see [`DIRECT_DFT_MAX_LEN`]. Twiddle factors are the whole
/// `N×N` matrix, precomputed once at plan time; `process` is then a
/// plain matrix-vector product.
///
/// First use: `engine::llr::symbol_spectra`'s per-symbol FFT
/// (`ds_spb = NSPS/NDOWN`, 36-42 across FST4's 5 submodes — issue
/// #306/#307), planned once per candidate and `.process()`-ed once
/// per symbol (162 times for FST4-60A). O(42²) = 1 764 complex
/// multiply-adds per call is negligible next to the LLR/BP/OSD work
/// each one feeds.
struct DirectDft {
    len: usize,
    /// Row-major `len × len`: `twiddles[k*len + n] = e^{-2πikn/len}`.
    twiddles: Box<[Complex32]>,
}

impl DirectDft {
    fn new(len: usize) -> Self {
        let mut twiddles = Vec::with_capacity(len * len);
        for k in 0..len {
            for n in 0..len {
                let theta = -2.0 * core::f32::consts::PI * (k * n) as f32 / len as f32;
                twiddles.push(Complex32::new(theta.cos(), theta.sin()));
            }
        }
        Self {
            len,
            twiddles: twiddles.into_boxed_slice(),
        }
    }
}

impl Fft for DirectDft {
    fn process(&self, buf: &mut [Complex32]) {
        assert_eq!(buf.len(), self.len, "DirectDft input length mismatch");
        let mut out = alloc::vec![Complex32::new(0.0, 0.0); self.len];
        for (k, out_k) in out.iter_mut().enumerate() {
            let row = &self.twiddles[k * self.len..(k + 1) * self.len];
            let mut acc = Complex32::new(0.0, 0.0);
            for (n, &x) in buf.iter().enumerate() {
                acc += x * row[n];
            }
            *out_k = acc;
        }
        buf.copy_from_slice(&out);
    }

    fn len(&self) -> usize {
        self.len
    }
}

/// Above this length, [`DirectDft`]'s O(N²) cost stops being clearly
/// negligible and a real fast-transform algorithm (issue #307's
/// generic non-power-of-2 kernel — Bluestein/chirp-Z looks like the
/// right shape) is worth having instead. 64 is comfortable headroom
/// over the 36-42 range FST4's 5 submodes actually need
/// (`symbol_spectra`'s `ds_spb = NSPS/NDOWN`) without reaching into
/// territory where O(N²) would compete with the FFT it's standing in
/// for.
const DIRECT_DFT_MAX_LEN: usize = 64;

impl FftPlanner for EspDspPlanner {
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
        // 3840 = 256 × 15 mixed-radix path: WSJT-X-faithful FT8 spectrogram
        // FFT length. The inner 256-pt FFT uses esp-dsp's radix-2 asm kernel;
        // the 15-pt PFA factor is hand-rolled in mfsk-core
        // (`core::dsp::fft_15`).
        if len == 3840 {
            self.ensure_table(256);
            return Box::new(MixedRadix3840Fft::new());
        }
        // 5120 = 1024 x 5 mixed-radix path: `FT4_DOWNSAMPLE.fft2_size`,
        // i.e. the inverse transform `downsample_cached` runs once per
        // FT4 candidate. Wired in both directions even though only the
        // inverse has a caller today -- the wrapper is direction-agnostic
        // and a forward-only plan would be a trap for the next reader.
        if len == mfsk_core::engine::dsp::fft_mixed_5120::N {
            self.ensure_table(1024);
            return Box::new(MixedRadix5120Fft::new(true));
        }
        // 2304 = 256 x 9 mixed-radix path: `ft4_coarse::NFFT1`, the
        // windowed periodogram `ft4_coarse_sync` runs once per symbol
        // step (~152 per slot). Forward is the direction FT4 needs;
        // the inverse arm below exists for the same reason 5120's
        // forward one does.
        if len == mfsk_core::engine::dsp::fft_mixed_2304::N {
            self.ensure_table(256);
            return Box::new(MixedRadix2304Fft::new(true));
        }
        if !len.is_power_of_two() && len <= DIRECT_DFT_MAX_LEN {
            return Box::new(DirectDft::new(len));
        }
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft::new(len, true))
    }

    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
        if len == 3840 {
            unimplemented!(
                "inverse 3840-pt FFT not wired (current FT8 spectrogram path is forward only)"
            );
        }
        // See `plan_forward`'s 5120 arm. This is the direction FT4
        // actually needs: `downsample_cached`'s
        // `plan_inverse(cfg.fft2_size)`.
        if len == mfsk_core::engine::dsp::fft_mixed_5120::N {
            self.ensure_table(1024);
            return Box::new(MixedRadix5120Fft::new(false));
        }
        // See `plan_forward`'s 2304 arm.
        if len == mfsk_core::engine::dsp::fft_mixed_2304::N {
            self.ensure_table(256);
            return Box::new(MixedRadix2304Fft::new(false));
        }
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft::new(len, false))
    }
}

/// 3840-pt forward FFT via Cooley-Tukey 256 × 15 mixed-radix.
/// 256-pt: esp-dsp `dsps_fft2r_fc32_ae32_` (asm). 15-pt: see
/// [`mfsk_core::engine::dsp::fft_15`]. Inter-stage twiddles cached.
struct MixedRadix3840Fft {
    twiddles: Box<[Complex32; mfsk_core::engine::dsp::fft_mixed_3840::N]>,
    /// 256-`Complex32` staging for the PIE kernel — see
    /// [`Align16Quad`]. Rows sit at a multiple of 256 `Complex32` from
    /// the caller's base pointer, so they inherit its alignment: if the
    /// caller's 3840-element buffer lands 4-mod-8, *every* row does.
    staging: core::cell::UnsafeCell<AlignedStaging>,
}

impl MixedRadix3840Fft {
    fn new() -> Self {
        Self {
            twiddles: mfsk_core::engine::dsp::fft_mixed_3840::build_twiddles(),
            staging: core::cell::UnsafeCell::new(AlignedStaging::new()),
        }
    }
}

impl Fft for MixedRadix3840Fft {
    fn process(&self, buf: &mut [Complex32]) {
        const N: usize = mfsk_core::engine::dsp::fft_mixed_3840::N;
        assert_eq!(buf.len(), N, "3840 FFT input length mismatch");
        let buf_arr: &mut [Complex32; N] = buf.try_into().expect("buf.len() == N already asserted");

        // Same table discipline as `EspDspFft::process` — see its
        // comment. Held across every inner 256-pt transform below, not
        // re-acquired per call, since they are one logical FFT.
        let _guard = Fc32Guard::acquire();
        ensure_fc32_table_locked(256);

        // Inner 256-pt forward FFT via esp-dsp asm path. Mirrors
        // `EspDspFft::process` but specialised to len=256.
        let run_256 = |slice: &mut [Complex32]| {
            let ptr = slice.as_mut_ptr() as *mut f32;
            unsafe {
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_fc32_ae32_(ptr, 256, dsps_fft_w_table_fc32);
                #[cfg(feature = "aes3")]
                dsps_fft2r_fc32_aes3_(ptr, 256, dsps_fft_w_table_fc32);
                dsps_bit_rev_fc32_ansi(ptr, 256);
            }
        };
        let mut esp_dsp_256 = |row: &mut [Complex32; 256]| {
            let staged = cfg!(feature = "aes3") && !pie_aligned(row);
            #[cfg(feature = "aes3")]
            record_pie_path(256, !staged);
            if staged {
                // SAFETY: `dyn Fft` carries no `Sync` bound and a
                // planned instance is owned by a single caller.
                let staging = unsafe { &mut *self.staging.get() };
                let work = staging.as_slice(256);
                let t0 = probe_us();
                work.copy_from_slice(row);
                let t1 = probe_us();
                run_256(work);
                let t2 = probe_us();
                row.copy_from_slice(work);
                let t3 = probe_us();
                add_kernel_us(t2 - t1);
                add_staging_us((t1 - t0) + (t3 - t2));
            } else {
                let t0 = probe_us();
                run_256(row);
                add_kernel_us(probe_us() - t0);
            }
        };

        let t_proc = probe_us();
        mfsk_core::engine::dsp::fft_mixed_3840::fft_3840_with(
            buf_arr,
            &mut esp_dsp_256,
            &self.twiddles,
        );
        add_process_us(probe_us() - t_proc);
    }

    fn len(&self) -> usize {
        mfsk_core::engine::dsp::fft_mixed_3840::N
    }
}

/// 2304-pt FFT via Cooley-Tukey 256 x 9 mixed-radix, either direction.
/// 256-pt: esp-dsp `dsps_fft2r_fc32_ae32_`/`_aes3_` (asm). 9-pt:
/// [`mfsk_core::engine::dsp::fft_mixed_2304::fft_9`]. Inter-stage
/// twiddles cached.
///
/// Exists because `engine::ft4_coarse`'s `NFFT1 = 4*NSPS = 2304`
/// (= 2^8 * 3^2) is not a power of two, so FT4's coarse-candidate
/// periodogram -- ~152 transforms per slot -- has no radix-2 kernel.
/// This is the last of the three FT4 lengths: `fft_mixed_5120` covers
/// the per-candidate inverse, `mfsk_core::ft4::ddc` removes the
/// 92 160-point slot transform outright, and this one lets the board
/// find its own candidates instead of reading a list baked on a host
/// (`embedded-poc/assets/ft4_golden_candidates.bin`).
struct MixedRadix2304Fft {
    forward: bool,
    twiddles: Box<[Complex32; mfsk_core::engine::dsp::fft_mixed_2304::N]>,
    /// 256-`Complex32` staging for the PIE kernel -- same inheritance
    /// argument as [`MixedRadix3840Fft`]'s, and the same inner length.
    staging: core::cell::UnsafeCell<AlignedStaging>,
}

impl MixedRadix2304Fft {
    fn new(forward: bool) -> Self {
        Self {
            forward,
            twiddles: mfsk_core::engine::dsp::fft_mixed_2304::build_twiddles(),
            staging: core::cell::UnsafeCell::new(AlignedStaging::new()),
        }
    }
}

impl Fft for MixedRadix2304Fft {
    fn process(&self, buf: &mut [Complex32]) {
        const N: usize = mfsk_core::engine::dsp::fft_mixed_2304::N;
        assert_eq!(buf.len(), N, "2304 FFT input length mismatch");
        let buf_arr: &mut [Complex32; N] = buf.try_into().expect("buf.len() == N already asserted");

        // One guard across all nine inner 256-pt transforms -- they are
        // one logical FFT over a process-global twiddle table, same
        // discipline as `MixedRadix3840Fft::process`.
        let _guard = Fc32Guard::acquire();
        ensure_fc32_table_locked(256);

        let run_256 = |slice: &mut [Complex32]| {
            let ptr = slice.as_mut_ptr() as *mut f32;
            unsafe {
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_fc32_ae32_(ptr, 256, dsps_fft_w_table_fc32);
                #[cfg(feature = "aes3")]
                dsps_fft2r_fc32_aes3_(ptr, 256, dsps_fft_w_table_fc32);
                dsps_bit_rev_fc32_ansi(ptr, 256);
            }
        };
        let mut esp_dsp_256 = |row: &mut [Complex32; 256]| {
            let staged = cfg!(feature = "aes3") && !pie_aligned(row);
            #[cfg(feature = "aes3")]
            record_pie_path(256, !staged);
            if staged {
                // SAFETY: `dyn Fft` carries no `Sync` bound and a
                // planned instance is owned by a single caller.
                let staging = unsafe { &mut *self.staging.get() };
                let work = staging.as_slice(256);
                let t0 = probe_us();
                work.copy_from_slice(row);
                let t1 = probe_us();
                run_256(work);
                let t2 = probe_us();
                row.copy_from_slice(work);
                let t3 = probe_us();
                add_kernel_us(t2 - t1);
                add_staging_us((t1 - t0) + (t3 - t2));
            } else {
                let t0 = probe_us();
                run_256(row);
                add_kernel_us(probe_us() - t0);
            }
        };

        let t_proc = probe_us();
        if self.forward {
            mfsk_core::engine::dsp::fft_mixed_2304::fft_2304_with(
                buf_arr,
                &mut esp_dsp_256,
                &self.twiddles,
            );
        } else {
            mfsk_core::engine::dsp::fft_mixed_2304::ifft_2304_with(
                buf_arr,
                &mut esp_dsp_256,
                &self.twiddles,
            );
        }
        add_process_us(probe_us() - t_proc);
    }

    fn len(&self) -> usize {
        mfsk_core::engine::dsp::fft_mixed_2304::N
    }
}

/// 5120-pt FFT via Cooley-Tukey 1024 x 5 mixed-radix, either
/// direction. 1024-pt: esp-dsp `dsps_fft2r_fc32_ae32_`/`_aes3_` (asm).
/// 5-pt: [`mfsk_core::engine::dsp::fft_15::fft_5`]. Inter-stage
/// twiddles cached.
///
/// Exists because `FT4_DOWNSAMPLE.fft2_size = 5120` is not a power of
/// two, so `downsample_cached`'s per-candidate inverse transform has no
/// radix-2 kernel -- the FT4 half of the wall this module's header
/// comment describes. The wideband `fft1_size = 92_160` half stays
/// unsolved here and is supplied pre-baked through `decode_frame`'s
/// `precomputed_fft` seam, the same way FST4 does it (issue #306).
struct MixedRadix5120Fft {
    forward: bool,
    twiddles: Box<[Complex32; mfsk_core::engine::dsp::fft_mixed_5120::N]>,
    /// 1024-`Complex32` staging for the PIE kernel -- see
    /// [`Align16Quad`]. Same inheritance argument as
    /// [`MixedRadix3840Fft`]: rows sit at a multiple of 1024
    /// `Complex32` from the caller's base pointer, so if the caller's
    /// buffer lands 4-mod-8 every row does.
    staging: core::cell::UnsafeCell<AlignedStaging>,
}

impl MixedRadix5120Fft {
    fn new(forward: bool) -> Self {
        Self {
            forward,
            twiddles: mfsk_core::engine::dsp::fft_mixed_5120::build_twiddles(),
            staging: core::cell::UnsafeCell::new(AlignedStaging::new()),
        }
    }
}

impl Fft for MixedRadix5120Fft {
    fn process(&self, buf: &mut [Complex32]) {
        const N: usize = mfsk_core::engine::dsp::fft_mixed_5120::N;
        assert_eq!(buf.len(), N, "5120 FFT input length mismatch");
        let buf_arr: &mut [Complex32; N] = buf.try_into().expect("buf.len() == N already asserted");

        // Same table discipline as `MixedRadix3840Fft::process`: one
        // guard held across all five inner 1024-pt transforms, since
        // they are one logical FFT over a process-global twiddle table.
        let _guard = Fc32Guard::acquire();
        ensure_fc32_table_locked(1024);

        let run_1024 = |slice: &mut [Complex32]| {
            let ptr = slice.as_mut_ptr() as *mut f32;
            unsafe {
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_fc32_ae32_(ptr, 1024, dsps_fft_w_table_fc32);
                #[cfg(feature = "aes3")]
                dsps_fft2r_fc32_aes3_(ptr, 1024, dsps_fft_w_table_fc32);
                dsps_bit_rev_fc32_ansi(ptr, 1024);
            }
        };
        let mut esp_dsp_1024 = |row: &mut [Complex32; 1024]| {
            let staged = cfg!(feature = "aes3") && !pie_aligned(row);
            #[cfg(feature = "aes3")]
            record_pie_path(1024, !staged);
            if staged {
                // SAFETY: `dyn Fft` carries no `Sync` bound and a
                // planned instance is owned by a single caller.
                let staging = unsafe { &mut *self.staging.get() };
                let work = staging.as_slice(1024);
                let t0 = probe_us();
                work.copy_from_slice(row);
                let t1 = probe_us();
                run_1024(work);
                let t2 = probe_us();
                row.copy_from_slice(work);
                let t3 = probe_us();
                add_kernel_us(t2 - t1);
                add_staging_us((t1 - t0) + (t3 - t2));
            } else {
                let t0 = probe_us();
                run_1024(row);
                add_kernel_us(probe_us() - t0);
            }
        };

        // The inner kernel is forward in both cases --
        // `ifft_5120_with` conjugates around the whole transform.
        let t_proc = probe_us();
        if self.forward {
            mfsk_core::engine::dsp::fft_mixed_5120::fft_5120_with(
                buf_arr,
                &mut esp_dsp_1024,
                &self.twiddles,
            );
        } else {
            mfsk_core::engine::dsp::fft_mixed_5120::ifft_5120_with(
                buf_arr,
                &mut esp_dsp_1024,
                &self.twiddles,
            );
        }
        add_process_us(probe_us() - t_proc);
    }

    fn len(&self) -> usize {
        mfsk_core::engine::dsp::fft_mixed_5120::N
    }
}


struct EspDspFft {
    len: usize,
    forward: bool,
    /// See [`Align16Quad`]. `UnsafeCell` because [`Fft::process`] takes
    /// `&self`; a planned `Box<dyn Fft>` is owned by one caller and
    /// `dyn Fft` carries no `Sync` bound, so there is no sharing to
    /// race against.
    staging: core::cell::UnsafeCell<AlignedStaging>,
}

impl EspDspFft {
    fn new(len: usize, forward: bool) -> Self {
        Self {
            len,
            forward,
            staging: core::cell::UnsafeCell::new(AlignedStaging::new()),
        }
    }

    /// Forward radix-2 kernel + bit-reverse, in place on `work`.
    /// `work` must be 16-byte aligned when the PIE path is compiled in.
    fn kernel(&self, work: &mut [Complex32]) {
        debug_assert!(!cfg!(feature = "aes3") || pie_aligned(work));
        let ptr = work.as_mut_ptr() as *mut f32;
        // SAFETY: ptr points to 2*N contiguous f32 (Complex32 layout).
        // `dsps_fft_w_table_fc32` is valid because `ensure_table` has
        // already called `dsps_fft2r_init_fc32` which populates it.
        unsafe {
            #[cfg(not(feature = "aes3"))]
            dsps_fft2r_fc32_ae32_(ptr, self.len as i32, dsps_fft_w_table_fc32);
            #[cfg(feature = "aes3")]
            dsps_fft2r_fc32_aes3_(ptr, self.len as i32, dsps_fft_w_table_fc32);
            // Bit-reverse the in-place output to get natural order.
            dsps_bit_rev_fc32_ansi(ptr, self.len as i32);
        }
    }
}

impl Fft for EspDspFft {
    fn process(&self, buf: &mut [Complex32]) {
        assert_eq!(buf.len(), self.len, "FFT input length mismatch");
        // Held across the kernel, not just across the resize: the
        // table this transform reads is process-global, and another
        // core planning a different size mid-transform is exactly the
        // corruption `FC32_LOCK`'s doc comment describes. Re-checking
        // the size here rather than trusting `plan_forward`'s call is
        // the other half — a `Box<dyn Fft>` outlives any number of
        // other plans.
        let _guard = Fc32Guard::acquire();
        ensure_fc32_table_locked(self.len);
        // esp-dsp expects an interleaved {re, im, re, im, ...} f32
        // array of length 2*N. Complex32 is repr(C) with this exact
        // layout, so we can cast in place — subject to alignment, see
        // `Align16Quad`.
        if !self.forward {
            // Emulate inverse via conjugate-flip (esp-dsp has no
            // inverse-mode FFT for the radix-2 routine).
            for c in buf.iter_mut() {
                c.im = -c.im;
            }
        }
        if cfg!(feature = "aes3") && !pie_aligned(buf) {
            #[cfg(feature = "aes3")]
            record_pie_path(self.len, false);
            // SAFETY: see the field comment on `staging`.
            let staging = unsafe { &mut *self.staging.get() };
            let work = staging.as_slice(self.len);
            work.copy_from_slice(buf);
            self.kernel(work);
            buf.copy_from_slice(work);
        } else {
            #[cfg(feature = "aes3")]
            record_pie_path(self.len, true);
            self.kernel(buf);
        }
        if !self.forward {
            let scale = 1.0 / self.len as f32;
            for c in buf.iter_mut() {
                c.re *= scale;
                c.im = -c.im * scale;
            }
        }
    }

    fn len(&self) -> usize {
        self.len
    }
}

// ── i16 / sc16 planner ──────────────────────────────────────────────────

/// sc16 sibling of [`FC32_TABLE_LEN`] / [`ensure_fc32_table`] — same
/// one-shot-gate structure on the C side (`dsps_fft2r_sc16_initialized`
/// in `dsps_fft2r_sc16_ansi.c`), same fix (deinit+reinit on any size
/// change, tracked at process scope, not per-`EspDspPlanner16`).
///
/// **Does not track what size the table is actually generated for.**
/// Unlike fc32, `dsps_fft2r_init_sc16`'s `fft_table_buff == NULL`
/// branch ignores its own `table_size` argument for generation and
/// always builds `CONFIG_DSP_MAX_FFT_SIZE` entries:
/// ```c
/// } else {
///     if (!dsps_fft2r_sc16_mem_allocated) {
///         dsps_fft_w_table_sc16 = memalign(16, CONFIG_DSP_MAX_FFT_SIZE * sizeof(int16_t));
///     }
///     dsps_fft_w_table_sc16_size = CONFIG_DSP_MAX_FFT_SIZE;  // <- not `table_size`
///     ...
/// }
/// result = dsps_gen_w_r2_sc16(dsps_fft_w_table_sc16, dsps_fft_w_table_sc16_size);
/// ```
/// (confirmed against the actual pinned component,
/// `espressif/esp-dsp` 1.8.2, `CONFIG_DSP_MAX_FFT_SIZE=8192` per this
/// project's `sdkconfig.defaults`). Read at face value, every sc16
/// FFT shorter than 8192 — including the 256-pt inner stage of
/// `MixedRadix3840Sc16Fft`, on the critical path of every FT8
/// decode, since `fixed-point` is on by default for every app crate
/// — would run against twiddle "factors" that are actually a tiny
/// low-angle slice of an 8192-point table, not a valid 256-point
/// table.
///
/// That contradicts this project's own repeatedly-verified real-
/// hardware FT8 recall under `fixed-point` (7/7, multiple sessions),
/// so something in this reading is very likely incomplete — a
/// platform-specific override this survey didn't find, or a detail
/// of how the asm kernel actually walks the table. It is being left
/// **unfixed and unactioned** rather than "corrected" against a
/// mechanism that provably works on real silicon today; the fc32 fix
/// above is applied to sc16 only for the *identical, independently-
/// confirmed* one-shot-gate structure, which is a no-op for every
/// current sc16 caller (all of them request exactly one size, 256,
/// for the process's lifetime). Whoever revisits this should start
/// with a device round-trip dump of `dsps_fft_w_table_sc16` itself
/// rather than more source-reading.
static SC16_TABLE_LEN: AtomicUsize = AtomicUsize::new(0);

fn ensure_sc16_table(len: usize) {
    if SC16_TABLE_LEN.load(Ordering::Relaxed) == len {
        return;
    }
    // SAFETY: see `ensure_fc32_table` — same contract, sc16 sibling.
    unsafe {
        dsps_fft2r_deinit_sc16();
        let r = dsps_fft2r_init_sc16(core::ptr::null_mut(), len as i32);
        assert_eq!(
            r,
            ESP_OK,
            "dsps_fft2r_init_sc16({len}) returned {r} = {}",
            describe_esp_dsp_err(r)
        );
    }
    SC16_TABLE_LEN.store(len, Ordering::Relaxed);
}

pub struct EspDspPlanner16;

impl EspDspPlanner16 {
    pub fn new() -> Self {
        Self
    }

    fn ensure_table(&mut self, len: usize) {
        ensure_sc16_table(len);
    }
}

impl Default for EspDspPlanner16 {
    fn default() -> Self {
        Self::new()
    }
}

impl FftPlanner16 for EspDspPlanner16 {
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16> {
        // 3840 = 256 × 15 mixed-radix (FT8 NFFT_SPEC). The 256-pt sc16
        // FFT runs on esp-dsp asm; the 15-pt PFA + twiddle multiply
        // happens in f32 (one-shot scratch alloc in `process`).
        if len == 3840 {
            self.ensure_table(256);
            return Box::new(MixedRadix3840Sc16Fft::new());
        }
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp i16 FFT: unsupported length {len} (need power-of-2 ≥ 4 or 3840)"
        );
        self.ensure_table(len);
        Box::new(EspDspFft16 { len, forward: true })
    }

    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft16> {
        if len == 3840 {
            unimplemented!("inverse 3840-pt sc16 FFT not wired (FT8 spectrogram is forward-only)");
        }
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp i16 FFT: unsupported length {len} (need power-of-2 ≥ 4)"
        );
        self.ensure_table(len);
        Box::new(EspDspFft16 {
            len,
            forward: false,
        })
    }
}

/// 3840-pt sc16 forward FFT via Cooley-Tukey 256 × 15 mixed-radix.
///
/// The 256-pt sub-kernel uses esp-dsp's sc16 asm path. The 15-pt PFA
/// runs in f32 because Q15 i16 cos/sin twiddles at 5-pt Winograd
/// granularity lose ~3 effective bits per stage, which collapses
/// realistic FT8 SNR margins. The f32 detour costs ~1 ms over the
/// 184-frame slot — invisible vs the ~3 ms/FFT mixed-radix budget.
struct MixedRadix3840Sc16Fft {
    twiddles: alloc::boxed::Box<[Complex32; mfsk_core::engine::dsp::fft_mixed_3840::N]>,
}

impl MixedRadix3840Sc16Fft {
    fn new() -> Self {
        Self {
            twiddles: mfsk_core::engine::dsp::fft_mixed_3840::build_twiddles(),
        }
    }
}

impl Fft16 for MixedRadix3840Sc16Fft {
    fn process(&self, buf: &mut [Complex<i16>]) {
        const N: usize = mfsk_core::engine::dsp::fft_mixed_3840::N;
        const N1: usize = 256;
        const N2: usize = 15;
        assert_eq!(buf.len(), N, "3840 sc16 FFT input length mismatch");

        // ── Step 1+2: reshape to 15 rows × 256 cols (i16) and run a
        //              256-pt sc16 esp-dsp FFT on each row.
        //
        // dsps_fft2r_sc16_aes3_ (PIE) requires 16-byte aligned input.
        // alloc::vec! only guarantees align_of::<Complex<i16>>() = 2 bytes,
        // so we use alloc_zeroed with a repr(align(16)) struct and
        // Box::from_raw — Box drop then calls dealloc with the correct layout.
        #[repr(C, align(16))]
        struct AlignedRows([Complex<i16>; N]);
        let mut rows_box: alloc::boxed::Box<AlignedRows> = unsafe {
            let layout = alloc::alloc::Layout::new::<AlignedRows>();
            let ptr = alloc::alloc::alloc_zeroed(layout) as *mut AlignedRows;
            if ptr.is_null() {
                alloc::alloc::handle_alloc_error(layout);
            }
            alloc::boxed::Box::from_raw(ptr)
        };
        let rows: &mut [Complex<i16>] = &mut rows_box.0;
        for n1 in 0..N1 {
            for n2 in 0..N2 {
                rows[n2 * N1 + n1] = buf[15 * n1 + n2];
            }
        }
        for n2 in 0..N2 {
            let row_ptr = rows[n2 * N1..(n2 + 1) * N1].as_mut_ptr() as *mut i16;
            unsafe {
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_sc16_ae32_(row_ptr, N1 as i32, dsps_fft_w_table_sc16);
                #[cfg(feature = "aes3")]
                dsps_fft2r_sc16_aes3_(row_ptr, N1 as i32, dsps_fft_w_table_sc16);
                dsps_bit_rev_sc16_ansi(row_ptr, N1 as i32);
            }
        }

        // ── Step 3+4: convert to f32, twiddle, run 15-pt PFA per column.
        let mut m: alloc::vec::Vec<Complex32> = alloc::vec![Complex32::new(0.0, 0.0); N];
        for (i, c) in rows.iter().enumerate() {
            m[i] = Complex32::new(c.re as f32, c.im as f32) * self.twiddles[i];
        }
        let mut col = [Complex32::new(0.0, 0.0); N2];
        for k1 in 0..N1 {
            for k2 in 0..N2 {
                col[k2] = m[k2 * N1 + k1];
            }
            mfsk_core::engine::dsp::fft_15::fft_15(&mut col);
            for k2 in 0..N2 {
                m[k2 * N1 + k1] = col[k2];
            }
        }

        // ── Step 5: clamp to i16 and write back in natural index order.
        for k2 in 0..N2 {
            for k1 in 0..N1 {
                let c = m[k2 * N1 + k1];
                let re = c.re.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
                let im = c.im.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
                buf[N1 * k2 + k1] = Complex::new(re, im);
            }
        }
    }

    fn len(&self) -> usize {
        mfsk_core::engine::dsp::fft_mixed_3840::N
    }
}

struct EspDspFft16 {
    len: usize,
    forward: bool,
}

impl Fft16 for EspDspFft16 {
    fn process(&self, buf: &mut [Complex<i16>]) {
        assert_eq!(buf.len(), self.len, "FFT input length mismatch");
        let ptr = buf.as_mut_ptr() as *mut i16;
        if !self.forward {
            for c in buf.iter_mut() {
                c.im = -c.im;
            }
        }
        // SAFETY: 2*N contiguous i16; `dsps_fft_w_table_sc16` valid post-init.
        unsafe {
            dsps_fft2r_sc16_ae32_(ptr, self.len as i32, dsps_fft_w_table_sc16);
            dsps_bit_rev_sc16_ansi(ptr, self.len as i32);
        }
        if !self.forward {
            // No /N scale on the i16 path — sc16 keeps stage-by-stage
            // auto-scaling instead. The host stub does similar.
            for c in buf.iter_mut() {
                c.im = -c.im;
            }
        }
    }

    fn len(&self) -> usize {
        self.len
    }
}
