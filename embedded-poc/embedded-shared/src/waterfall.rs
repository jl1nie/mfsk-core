// SPDX-License-Identifier: GPL-3.0-only
//! Waterfall rows from raw 12 kHz audio — one builder for every mode.
//!
//! The panel's waterfall is the same surface whatever the receiver
//! (`ui::state::UI`, drawn by the board's log panel), and what it shows
//! is a property of the *audio*, not of the decoder. Its rows used to
//! come from each decoder's own spectra instead — FT8's stage-1 pairs,
//! FT4's coarse periodogram — so every mode needed its own feed, and a
//! receiver without one (WSPR, FST4) showed an empty waterfall on the
//! shared panel. This builds them from the audio itself, so one feed
//! serves every mode and a new mode gets a waterfall without writing
//! one.
//!
//! **All of it on esp-dsp.** A row is a real transform, run the way
//! esp-dsp's own `fft4real` example does: the samples, windowed by
//! `dsps_mul_f32`, are read as half as many *complex* points, transformed,
//! and
//! `dsps_cplx2real_fc32` unfolds that into the real input's spectrum — a
//! transform of half the points, where a plain complex FFT of real
//! samples would compute a mirror image and throw half of it away. The
//! power spectrum is two more esp-dsp passes (`dsps_mul_f32`,
//! `dsps_add_f32`). What is left in Rust is the column mapping, which is
//! branches and integer arithmetic.
//!
//! **Two sizes, two kernels, one unfold table.** The slotted modes run
//! 2 048 real points ([`WF_NFFT_FINE`]) as 1 024 complex — a power of 4 —
//! on the radix-4 PIE kernel. JTTY runs 1 024 ([`WF_NFFT_COLUMN`]) as 512
//! complex, which esp-dsp's radix-4 kernel refuses
//! (`dsps_fft4r_fc32_ansi.c`: `log2N` must be even), so that transform is
//! the radix-2 PIE kernel on `esp_dsp_fft`'s read-only 512-point table.
//! `dsps_cplx2real_fc32` has no such limit — it strides the radix-4
//! twiddle table by `table_size / N` — so both sizes unfold with that one
//! table.
//!
//! No threading and no board state here: push audio, receive rows.
//! Where the audio comes from and who owns the builder is the board's.

extern crate alloc;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};

use mfsk_core::engine::fft::AlignedComplexBuf;

use crate::pipeline::WF_ROW_LEN;

/// Transform length, in real samples, for the slotted modes (FT8, FT4,
/// FST4, WSPR): 5.86 Hz bins against the panel's 11.7 Hz columns, so
/// every column sees two whole bins, over a 171 ms window — a row sees
/// all of each 167 ms of audio at `WF_HOP`.
pub const WF_NFFT_FINE: usize = 2_048;

/// Transform length for a receiver whose decode leaves the panel's core
/// no slack (JTTY): 11.7 Hz bins, the panel's own column width, over an
/// 85 ms window.
///
/// At 2 048 points the transform was the panel's largest CPU cost —
/// ~35 ms of every second of the core it shares with JTTY's `Back`,
/// against ~5 ms for building the drawn rows (jtty-demo, 2026-09-28; the
/// rest of the redraw is SPI DMA the core is free through) — and half its
/// bins were max-pooled away by the 11.7 Hz columns anyway. Measured there
/// at 1 024: 1.0-1.2 ms a row against 3.9-4.1, and one palette step
/// (~2 dB) less contrast between a station and its neighbourhood (5-7
/// against 3-4, where 2 048 gives 6-8), as the wider bin predicts. A row
/// then sees only 85 ms of each 167 ms, which is why the slotted modes,
/// with slack to spare, stay on [`WF_NFFT_FINE`].
pub const WF_NFFT_COLUMN: usize = 1_024;

/// The radix-4 twiddle table is process-global and built once, for the
/// largest transform either size runs as ([`WF_NFFT_FINE`] / 2 complex
/// points). The 1 024-point transform's unfold reads every fourth entry
/// of it rather than a table of its own (`dsps_cplx2real_fc32` strides by
/// `table_size / N`).
const FFT4R_TABLE_POINTS: usize = WF_NFFT_FINE / 2;

/// Samples between rows: 6 rows a second, one per panel frame
/// (`display.rs`'s `FRAME_US`), so each redraw moves the waterfall by
/// exactly one row and it scrolls smoothly. 100 rows are ~17 s of band.
///
/// It was 12, FT8's old stage-1 cadence. That put two to three new rows
/// in every redraw once the panel had to be held to a frame rate — and
/// it has to be: running above FT4's decode so the screen never stops,
/// the panel at 10 frames/s used 16-19 % of core 0, which the decode
/// paid for in full (loop ends 1 005-1 189 ms, one of eleven decodes
/// late in half the slots; 2026-09-21).
pub const WF_HOP: usize = 2_000;

/// Frequency span a row covers, shared with the panel's axis
/// (`ui::waterfall`).
pub const WF_FREQ_LO_HZ: f32 = 200.0;
/// Up to 3 000 Hz: the top of the band the FT8 decoder searches
/// (`stage1_inc`'s `ALLSUM_FREQ_MAX`). FT8's own rows used to stop at
/// 2 700, so stations between 2 700 and 3 000 Hz decoded without ever
/// being drawn. 240 columns over 2 800 Hz are 11.7 Hz each: two bins at
/// [`WF_NFFT_FINE`], one at [`WF_NFFT_COLUMN`].
pub const WF_FREQ_HI_HZ: f32 = 3_000.0;

const SAMPLE_RATE_HZ: f32 = 12_000.0;

unsafe extern "C" {
    /// Radix-4 twiddle table, allocated internally for up to
    /// `max_fft_size` complex points when `fft_table_buff` is null.
    /// Process-global; nothing else in this tree uses radix-4.
    fn dsps_fft4r_init_fc32(fft_table_buff: *mut f32, max_fft_size: i32) -> i32;
    #[cfg(feature = "aes3")]
    fn dsps_fft4r_fc32_aes3_(data: *mut f32, n: i32, table: *mut f32, table_size: i32) -> i32;
    #[cfg(not(feature = "aes3"))]
    fn dsps_fft4r_fc32_ae32_(data: *mut f32, n: i32, table: *mut f32, table_size: i32) -> i32;
    fn dsps_bit_rev4r_fc32_ae32(data: *mut f32, n: i32) -> i32;
    #[cfg(feature = "aes3")]
    fn dsps_fft2r_fc32_aes3_(data: *mut f32, n: i32, w: *const f32) -> i32;
    #[cfg(not(feature = "aes3"))]
    fn dsps_fft2r_fc32_ae32_(data: *mut f32, n: i32, w: *const f32) -> i32;
    fn dsps_bit_rev_fc32_ansi(data: *mut f32, n: i32) -> i32;
    fn dsps_cplx2real_fc32_ae32_(data: *mut f32, n: i32, table: *mut f32, table_size: i32) -> i32;
    fn dsps_mul_f32_ae32(
        a: *const f32,
        b: *const f32,
        out: *mut f32,
        len: i32,
        step_a: i32,
        step_b: i32,
        step_out: i32,
    ) -> i32;
    fn dsps_add_f32_ae32(
        a: *const f32,
        b: *const f32,
        out: *mut f32,
        len: i32,
        step_a: i32,
        step_b: i32,
        step_out: i32,
    ) -> i32;
    /// What `dsps_fft4r_init_fc32` fills — the C API passes these to
    /// the kernels through macros, which a binding cannot.
    static mut dsps_fft4r_w_table_fc32: *mut f32;
    static mut dsps_fft4r_w_table_size: i32;
}

static FFT4R_READY: AtomicBool = AtomicBool::new(false);

/// What the rows cost, accumulated since the last
/// [`WfRowBuilder::take_timing`] — so the UI transform's speed is a
/// number the board reports, not an estimate.
#[derive(Clone, Copy, Debug, Default)]
pub struct WfTiming {
    pub rows: u32,
    /// Window, FFT, real unfold and power spectrum — the esp-dsp part.
    pub fft_us: i64,
    /// Spectrum to palette columns (`wf_row`).
    pub map_us: i64,
    /// Slowest single row, both parts.
    pub max_row_us: i64,
    /// Fastest single esp-dsp part. These are wall-clock times on
    /// whatever task builds the rows — on the CoreS3 the panel, the
    /// lowest priority on its core — so the averages carry every
    /// preemption; the minimum is the closest reading of the transform
    /// itself.
    pub min_fft_us: i64,
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Builds waterfall rows from a 12 kHz stream, whatever block sizes it
/// arrives in.
pub struct WfRowBuilder {
    /// Transform length in real samples: [`WF_NFFT_FINE`] or
    /// [`WF_NFFT_COLUMN`].
    n: usize,
    /// 16-byte aligned for the PIE kernel: `n / 2` complex points, i.e.
    /// `n` floats.
    buf: AlignedComplexBuf,
    window: Vec<f32>,
    /// The last `n` samples, circular: `head` is the oldest once `filled`
    /// reaches `n`.
    hist: Vec<f32>,
    head: usize,
    filled: usize,
    /// Samples pushed since the last row.
    since_row: usize,
    /// `re², im²` per bin, then the power spectrum up to
    /// [`WF_FREQ_HI_HZ`].
    sq: Vec<f32>,
    spec: Vec<f32>,
    /// Samples pushed in all — what a caller locates a row by.
    total: u64,
    timing: WfTiming,
}

impl Default for WfRowBuilder {
    fn default() -> Self {
        Self::new(WF_NFFT_FINE)
    }
}

impl WfRowBuilder {
    /// A builder running `nfft`-point transforms: [`WF_NFFT_FINE`] or
    /// [`WF_NFFT_COLUMN`].
    ///
    /// # Panics
    /// On any other length.
    pub fn new(nfft: usize) -> Self {
        assert!(
            nfft == WF_NFFT_FINE || nfft == WF_NFFT_COLUMN,
            "waterfall: {nfft}-point rows are not supported"
        );
        if !FFT4R_READY.swap(true, Ordering::AcqRel) {
            // SAFETY: null buffer asks esp-dsp to allocate the table.
            let r =
                unsafe { dsps_fft4r_init_fc32(core::ptr::null_mut(), FFT4R_TABLE_POINTS as i32) };
            if r != 0 {
                log::error!("waterfall: dsps_fft4r_init_fc32 failed ({r:#x})");
            }
        }
        let n = nfft;
        let half = n / 2;
        if half != FFT4R_TABLE_POINTS {
            // The radix-2 table this size's transform runs on, installed
            // here rather than on the first row: it is allocated from
            // internal DRAM once, and the first row is drawn after WiFi has
            // taken its share.
            let _ = crate::esp_dsp_fft::fc32_table(half);
        }
        // Hann: the sidelobes of a strong signal would otherwise paint
        // across columns and bury the weak ones beside it.
        let window = (0..n)
            .map(|k| {
                let x = core::f32::consts::PI * k as f32 / n as f32;
                let s = x.sin();
                s * s
            })
            .collect();
        let bins = ((WF_FREQ_HI_HZ / (SAMPLE_RATE_HZ / n as f32)) as usize + 2).min(half);
        Self {
            n,
            buf: AlignedComplexBuf::zeroed(half),
            window,
            hist: alloc::vec![0.0; n],
            head: 0,
            filled: 0,
            since_row: 0,
            sq: alloc::vec![0.0; 2 * bins],
            spec: alloc::vec![0.0; bins],
            total: 0,
            timing: WfTiming::default(),
        }
    }

    /// Feed the next samples. `on_row` gets each row as it completes,
    /// with the stream position (samples pushed in all) at its end.
    pub fn push(&mut self, samples: &[i16], on_row: &mut dyn FnMut([u8; WF_ROW_LEN], u64)) {
        for &s in samples {
            // Overwrite the oldest: a circular history costs one store
            // a sample, where shifting a linear one costs `n`.
            self.hist[self.head] = s as f32;
            self.head = (self.head + 1) % self.n;
            self.filled = (self.filled + 1).min(self.n);
            self.total += 1;
            self.since_row += 1;
            if self.since_row >= WF_HOP && self.filled == self.n {
                self.since_row = 0;
                on_row(self.row(), self.total);
            }
        }
    }

    /// The cost of the rows built since the last call, and reset.
    pub fn take_timing(&mut self) -> WfTiming {
        core::mem::take(&mut self.timing)
    }

    fn row(&mut self) -> [u8; WF_ROW_LEN] {
        let t0 = now_us();
        // SAFETY: `Complex32` is `repr(C)` over two `f32`, so the buffer
        // is `n` contiguous floats — the real samples, read as `n / 2`
        // interleaved complex points.
        let data = self.buf.as_mut_slice().as_mut_ptr() as *mut f32;
        let half = self.n / 2;
        // Oldest first: `head` onwards, then the start up to it — one
        // windowed multiply per segment.
        let older = self.n - self.head;
        let bins = self.spec.len();
        // SAFETY: every pointer spans the length passed with it; the
        // twiddle tables are the radix-4 one `dsps_fft4r_init_fc32` built
        // for `FFT4R_TABLE_POINTS` points and, at 512, `esp_dsp_fft`'s
        // radix-2 table for exactly `half`, both read only.
        unsafe {
            dsps_mul_f32_ae32(
                self.hist.as_ptr().add(self.head),
                self.window.as_ptr(),
                data,
                older as i32,
                1,
                1,
                1,
            );
            if self.head > 0 {
                dsps_mul_f32_ae32(
                    self.hist.as_ptr(),
                    self.window.as_ptr().add(older),
                    data.add(older),
                    self.head as i32,
                    1,
                    1,
                    1,
                );
            }
            let table = dsps_fft4r_w_table_fc32;
            let table_size = dsps_fft4r_w_table_size;
            if half == FFT4R_TABLE_POINTS {
                #[cfg(feature = "aes3")]
                dsps_fft4r_fc32_aes3_(data, half as i32, table, table_size);
                #[cfg(not(feature = "aes3"))]
                dsps_fft4r_fc32_ae32_(data, half as i32, table, table_size);
                dsps_bit_rev4r_fc32_ae32(data, half as i32);
            } else {
                let w2 = crate::esp_dsp_fft::fc32_table(half);
                #[cfg(feature = "aes3")]
                dsps_fft2r_fc32_aes3_(data, half as i32, w2);
                #[cfg(not(feature = "aes3"))]
                dsps_fft2r_fc32_ae32_(data, half as i32, w2);
                dsps_bit_rev_fc32_ansi(data, half as i32);
            }
            dsps_cplx2real_fc32_ae32_(data, half as i32, table, table_size);
            // |X|² = re² + im², for the bins the row reads.
            dsps_mul_f32_ae32(data, data, self.sq.as_mut_ptr(), (2 * bins) as i32, 1, 1, 1);
            dsps_add_f32_ae32(
                self.sq.as_ptr(),
                self.sq.as_ptr().add(1),
                self.spec.as_mut_ptr(),
                bins as i32,
                2,
                2,
                1,
            );
        }
        let t1 = now_us();
        let row = wf_row(&self.spec, SAMPLE_RATE_HZ / self.n as f32);
        let t2 = now_us();
        let t = &mut self.timing;
        t.rows += 1;
        t.fft_us += t1 - t0;
        t.map_us += t2 - t1;
        t.max_row_us = t.max_row_us.max(t2 - t0);
        t.min_fft_us = if t.rows == 1 { t1 - t0 } else { t.min_fft_us.min(t1 - t0) };
        row
    }
}

/// Turn one power spectrum, bins `df_hz` apart from DC, into
/// [`WF_ROW_LEN`] palette indices 0..15 over
/// [`WF_FREQ_LO_HZ`]..[`WF_FREQ_HI_HZ`].
///
/// Moved here from `apps::ft4_rx`, where it mapped FT4's coarse
/// periodogram rows; the choices below were measured there on a CoreS3.
///
/// **Integer log2, not `log10`.** The first version took
/// `10 * log10(p)` per column and measured **623 µs per row** —
/// `f32::log10` is ~620 cycles on this core and there are 240 of them.
/// The level comes from the exponent instead, the way FT8's own
/// `decimate_pair_to_wf` did it: scale by the row's mean, take the MSB,
/// keep one fractional bit for half-octave resolution (~1.5 dB, below
/// what 16 palette steps can show).
///
/// **Per-column maximum, not mean.** A column is wider than a tone, so
/// averaging would halve every signal against its neighbouring noise
/// while leaving the floor alone.
///
/// **Relative to the row's own mean power.** An absolute scale needs a
/// calibrated input level, which a receiver taking whatever a radio's
/// USB audio hands it does not have; this shows the band the way an
/// operator reads one, against its own noise.
pub fn wf_row(spectrum: &[f32], df_hz: f32) -> [u8; WF_ROW_LEN] {
    /// `1.0` in the fixed-point ratio below, i.e. `2^SCALE_LOG2`.
    const SCALE_LOG2: u32 = 10;
    /// Half-octaves the palette spans.
    const HALF_OCTAVES: u32 = 20;

    let mut out = [0u8; WF_ROW_LEN];
    if spectrum.is_empty() {
        return out;
    }
    let mean: f32 = spectrum.iter().sum::<f32>() / spectrum.len() as f32;
    // A silent input has no scale; leave the row black.
    if !(mean > 0.0) {
        return out;
    }
    let inv = (1u32 << SCALE_LOG2) as f32 / mean;

    // Column -> bin range, walked rather than divided per column.
    let bin_step = (WF_FREQ_HI_HZ - WF_FREQ_LO_HZ) / (WF_ROW_LEN as f32) / df_hz;
    let mut edge = WF_FREQ_LO_HZ / df_hz;
    for cell in out.iter_mut() {
        let lo = edge as usize;
        edge += bin_step;
        let hi = ((edge as usize) + 1).min(spectrum.len());
        if hi <= lo {
            continue;
        }
        let mut peak = 0.0f32;
        for &p in &spectrum[lo..hi] {
            if p > peak {
                peak = p;
            }
        }
        let scaled = peak * inv;
        if !(scaled >= 1.0) {
            continue;
        }
        let q = scaled as u32;
        let e = 31 - q.leading_zeros();
        let frac = if e > 0 { (q >> (e - 1)) & 1 } else { 0 };
        let half_oct = (e * 2 + frac).saturating_sub(SCALE_LOG2 * 2);
        *cell = ((half_oct * 15) / HALF_OCTAVES).min(15) as u8;
    }
    out
}
