//! Internal-DRAM scratch buffers for Stage 3's per-candidate
//! `cs Box` staging (Phase B, issue #15).
//!
//! Each candidate's symbol-spectra block (`[[Cmplx<f32>; 8]; 79]`,
//! ~5 KB) lives in PSRAM as an entry of the Pass 2 result Vec; BP /
//! `compute_llr_*` access it many times per candidate, paying PSRAM
//! latency on every read. Copying it once into an internal-DRAM
//! scratch (`.bss`) before the BP loop lets the hot path run at
//! internal-SRAM speed (~5–10× faster).
//!
//! Two scratches because Pass 2 / Stage 3 split candidates across
//! both cores via `dual_core` — main core uses [`CS_SCRATCH_MAIN`],
//! APP_CPU worker uses [`CS_SCRATCH_WORKER`], 5 KB each, 10 KB total
//! out of the ~111 KB free internal DRAM after BASIS pins.

use mfsk_core::engine::scalar::Cmplx;

/// 79 symbols × 8 tones = `[[Cmplx<f32>; 8]; 79]` = 5,056 bytes.
pub const CS_LEN_SYMBOLS: usize = 79;
pub const CS_LEN_TONES: usize = 8;

/// Main-core (PRO_CPU) cs Box staging. `.bss` placement keeps it
/// in internal DRAM. Initialised to zero by the loader; each Stage 3
/// iter overwrites it via `*scratch = *cs_box` before use.
///
/// SAFETY: only the PRO_CPU side of `dual_core::stage3_split`
/// reads/writes this. The APP_CPU worker uses [`CS_SCRATCH_WORKER`].
pub static mut CS_SCRATCH_MAIN: [[Cmplx<f32>; CS_LEN_TONES]; CS_LEN_SYMBOLS] =
    [[Cmplx { re: 0.0, im: 0.0 }; CS_LEN_TONES]; CS_LEN_SYMBOLS];

/// APP_CPU (worker) cs Box staging.
///
/// SAFETY: only the APP_CPU worker (`dual_core::worker_main`)
/// reads/writes this.
pub static mut CS_SCRATCH_WORKER: [[Cmplx<f32>; CS_LEN_TONES]; CS_LEN_SYMBOLS] =
    [[Cmplx { re: 0.0, im: 0.0 }; CS_LEN_TONES]; CS_LEN_SYMBOLS];
