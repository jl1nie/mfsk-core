//! `mfsk_core::core::fft` backend bridged to Espressif `esp-dsp`.
//!
//! `esp-dsp` ships hand-written Xtensa assembly for the FFT (1.8-3×
//! the C reference on ESP32-S3). We expose it as an
//! [`mfsk_core::core::fft::FftPlanner`] so `mfsk-core`'s decode
//! pipeline can use it without knowing it's there.
//!
//! ## Sizes
//!
//! `dsps_fft2r_fc32_ae32` is a radix-2 FFT and supports any
//! power-of-2 length up to 4096 in the default build, or 32768 if
//! `CONFIG_DSP_TABLE_SIZE_4096_TO_32768` is enabled (we set it via
//! `sdkconfig.defaults`). Plans for non-power-of-2 sizes panic —
//! `mfsk-core`'s wide-band FFT cache (192 000 for FT8 / 92 160 for
//! FT4) is unsupported, but the narrow-band sniper / WSPR aligned
//! paths fit comfortably under the 8192-point cap.
//!
//! ## Memory
//!
//! `dsps_fft2r_init_fc32` allocates a twiddle table the first time;
//! subsequent plans reuse it. We initialise on the first
//! `plan_forward` / `plan_inverse` call.

use alloc::boxed::Box;

use mfsk_core::core::fft::{Fft, Fft16, FftPlanner, FftPlanner16};
use num_complex::{Complex, Complex32};

/// Factory called by `mfsk_core::core::fft::default_planner()` when
/// the crate is built with `fft-extern` (and no built-in backend like
/// `fft-rustfft`). Symbol name + signature are the link-time contract;
/// see `mfsk_core::core::fft::default_planner` for the spec.
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
    unsafe {
        assert_eq!(
            dsps_fft2r_init_fc32(core::ptr::null_mut(), radix2_len as i32),
            ESP_OK
        );
        assert_eq!(
            dsps_fft2r_init_sc16(core::ptr::null_mut(), radix2_len as i32),
            ESP_OK
        );
    }
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

    // ── i16 (sc16) variants ──────────────────────────────────
    fn dsps_fft2r_init_sc16(fft_table_buff: *mut i16, table_size: i32) -> i32;
    fn dsps_fft2r_sc16_ae32_(data: *mut i16, N: i32, w: *const i16) -> i32;
    /// LX7 PIE sc16 FFT. Requires 16-byte aligned input — caller must use
    /// an aligned buffer (see `Aligned16Rows` in `MixedRadix3840Sc16Fft`).
    #[cfg(feature = "aes3")]
    fn dsps_fft2r_sc16_aes3_(data: *mut i16, N: i32, w: *const i16) -> i32;
    fn dsps_bit_rev_sc16_ansi(data: *mut i16, N: i32) -> i32;
    static dsps_fft_w_table_sc16: *const i16;

    /// `esp_err_t dsps_dotprod_s16_ae32(const int16_t *src1,
    ///                                  const int16_t *src2,
    ///                                  int16_t *dest, int len,
    ///                                  int8_t shift);`
    ///
    /// AE32 asm Q15 dot product: 1 cycle/sample on LX6 (vs ~14
    /// cycles/sample for the equivalent Rust scalar loop). Output
    /// is i16 — `*dest = acc >> (15 - shift)` where `acc` is the
    /// i64-accumulated `Σ src1[i] * src2[i]`. Choose `shift` to
    /// keep the i16 range from overflowing for typical input
    /// magnitudes; `mfsk_core_dot_q15_i32` below picks `-11` for
    /// the auto-gain'd FT8 levels (post-scale audio peak ≤ 29000).
    fn dsps_dotprod_s16_ae32(
        src1: *const i16,
        src2: *const i16,
        dest: *mut i16,
        len: i32,
        shift: i8,
    ) -> i32;
}

/// Q15 dot product for `mfsk_core::core::dotprod` — bridges to
/// esp-dsp's AE32 asm. Trades ~9 bits of low-end precision for a
/// ~10× cycle-count win on Xtensa LX6.
///
/// **Shift tuning**: with `shift = -11` esp-dsp computes
/// `out = acc >> 26`. For NSPS=1920 i16 audio (peak ≤ 29000) ×
/// Q15 basis (peak 32767), max `acc ≈ 29000 × 960 × 32767 ≈ 9 × 10¹¹`
/// → `>>26 ≈ 13600`, fits i16. Weak signals at -22 dB SNR still
/// land at ~80 (above zero). Multiplying the i16 result by `1 << 11`
/// recovers the Q15-normalised i32 the trait contract expects (with
/// the bottom 11 bits zero-padded — equivalent to losing 11 bits of
/// precision off the bottom of the dynamic range).
///
/// Runtime-tunable via [`set_dot_shift`] — `main.rs` sweeps `-9 / -11
/// / -13` to find where precision starts to bite vs where i16
/// saturation steals strong-signal magnitude.
pub static ESP_DSP_DOT_SHIFT: core::sync::atomic::AtomicI8 = core::sync::atomic::AtomicI8::new(-11);

/// Atomically swap the dot-product shift parameter. Returns the
/// previous value. Used by `main.rs` for the shift sweep.
pub fn set_dot_shift(new_shift: i8) -> i8 {
    ESP_DSP_DOT_SHIFT.swap(new_shift, core::sync::atomic::Ordering::SeqCst)
}

#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_dot_q15_i32(a: *const i16, b: *const i16, n: usize) -> i32 {
    let shift = ESP_DSP_DOT_SHIFT.load(core::sync::atomic::Ordering::Relaxed);
    let mut out_i16: i16 = 0;
    // SAFETY: `a` and `b` point to `n` valid i16 elements (callers
    // pass `slice.as_ptr()`, `slice.len()`); `out_i16` is a stack
    // local. esp-dsp doesn't touch the input pointers beyond
    // `n` reads each.
    let r = unsafe { dsps_dotprod_s16_ae32(a, b, &mut out_i16 as *mut i16, n as i32, shift) };
    debug_assert_eq!(r, ESP_OK);
    // Restore Q15-normalised i32 magnitude. With shift = X the
    // i16 output represents `acc >> (15 - X)`. Left-shift by
    // `(15 - X) - 15 = -X` puts it back into `acc >> 15` range.
    // For X = -11: shift back by 11 (zero-pad bottom 11 bits).
    let restore = (-(shift as i32)) as u32;
    (out_i16 as i32) << restore
}

/// `esp-dsp` FFT planner. Construct once per session, share across
/// all decode invocations so the twiddle table inits exactly once.
pub struct EspDspPlanner {
    /// Largest table size already initialised. `0` = uninitialised.
    /// The init API is one-shot per max size; we re-call when a
    /// larger plan is requested (the lib handles it gracefully).
    initialised_max: usize,
}

impl EspDspPlanner {
    pub fn new() -> Self {
        Self { initialised_max: 0 }
    }

    fn ensure_table(&mut self, len: usize) {
        if len <= self.initialised_max {
            return;
        }
        // SAFETY: NULL buffer asks the lib to alloc its own table.
        // `len` must be a power of 2 ≤ `CONFIG_DSP_TABLE_SIZE_*`
        // (we set 4096-32768 in sdkconfig.defaults).
        unsafe {
            let r = dsps_fft2r_init_fc32(core::ptr::null_mut(), len as i32);
            assert_eq!(r, ESP_OK, "dsps_fft2r_init_fc32({len}) returned {r}");
        }
        self.initialised_max = len;
    }
}

impl Default for EspDspPlanner {
    fn default() -> Self {
        Self::new()
    }
}

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
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft { len, forward: true })
    }

    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
        if len == 3840 {
            unimplemented!(
                "inverse 3840-pt FFT not wired (current FT8 spectrogram path is forward only)"
            );
        }
        assert!(
            len.is_power_of_two() && len >= 4,
            "esp-dsp FFT requires power-of-2 length ≥ 4 (got {len})"
        );
        self.ensure_table(len);
        Box::new(EspDspFft {
            len,
            forward: false,
        })
    }
}

/// 3840-pt forward FFT via Cooley-Tukey 256 × 15 mixed-radix.
/// 256-pt: esp-dsp `dsps_fft2r_fc32_ae32_` (asm). 15-pt: see
/// [`mfsk_core::core::dsp::fft_15`]. Inter-stage twiddles cached.
struct MixedRadix3840Fft {
    twiddles: Box<[Complex32; mfsk_core::core::dsp::fft_mixed_3840::N]>,
}

impl MixedRadix3840Fft {
    fn new() -> Self {
        Self {
            twiddles: mfsk_core::core::dsp::fft_mixed_3840::build_twiddles(),
        }
    }
}

impl Fft for MixedRadix3840Fft {
    fn process(&self, buf: &mut [Complex32]) {
        const N: usize = mfsk_core::core::dsp::fft_mixed_3840::N;
        assert_eq!(buf.len(), N, "3840 FFT input length mismatch");
        let buf_arr: &mut [Complex32; N] = buf.try_into().expect("buf.len() == N already asserted");

        // Inner 256-pt forward FFT via esp-dsp asm path. Mirrors
        // `EspDspFft::process` but specialised to len=256.
        let mut esp_dsp_256 = |row: &mut [Complex32; 256]| {
            let ptr = row.as_mut_ptr() as *mut f32;
            unsafe {
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_fc32_ae32_(ptr, 256, dsps_fft_w_table_fc32);
                #[cfg(feature = "aes3")]
                dsps_fft2r_fc32_aes3_(ptr, 256, dsps_fft_w_table_fc32);
                dsps_bit_rev_fc32_ansi(ptr, 256);
            }
        };

        mfsk_core::core::dsp::fft_mixed_3840::fft_3840_with(
            buf_arr,
            &mut esp_dsp_256,
            &self.twiddles,
        );
    }

    fn len(&self) -> usize {
        mfsk_core::core::dsp::fft_mixed_3840::N
    }
}

struct EspDspFft {
    len: usize,
    forward: bool,
}

impl Fft for EspDspFft {
    fn process(&self, buf: &mut [Complex32]) {
        assert_eq!(buf.len(), self.len, "FFT input length mismatch");
        // esp-dsp expects an interleaved {re, im, re, im, ...} f32
        // array of length 2*N. Complex32 is repr(C) with this exact
        // layout, so we can cast in place.
        let ptr = buf.as_mut_ptr() as *mut f32;
        if !self.forward {
            // Emulate inverse via conjugate-flip (esp-dsp has no
            // inverse-mode FFT for the radix-2 routine).
            for c in buf.iter_mut() {
                c.im = -c.im;
            }
        }
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

pub struct EspDspPlanner16 {
    initialised_max: usize,
}

impl EspDspPlanner16 {
    pub fn new() -> Self {
        Self { initialised_max: 0 }
    }

    fn ensure_table(&mut self, len: usize) {
        if len <= self.initialised_max {
            return;
        }
        unsafe {
            let r = dsps_fft2r_init_sc16(core::ptr::null_mut(), len as i32);
            assert_eq!(r, ESP_OK, "dsps_fft2r_init_sc16({len}) returned {r}");
        }
        self.initialised_max = len;
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
    twiddles: alloc::boxed::Box<[Complex32; mfsk_core::core::dsp::fft_mixed_3840::N]>,
}

impl MixedRadix3840Sc16Fft {
    fn new() -> Self {
        Self {
            twiddles: mfsk_core::core::dsp::fft_mixed_3840::build_twiddles(),
        }
    }
}

impl Fft16 for MixedRadix3840Sc16Fft {
    fn process(&self, buf: &mut [Complex<i16>]) {
        const N: usize = mfsk_core::core::dsp::fft_mixed_3840::N;
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
            mfsk_core::core::dsp::fft_15::fft_15(&mut col);
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
        mfsk_core::core::dsp::fft_mixed_3840::N
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
