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

/// One candidate's symbol-spectra block.
pub type CsScratch = [[Cmplx<f32>; CS_LEN_TONES]; CS_LEN_SYMBOLS];

/// 5,056 B.
pub const CS_SCRATCH_BYTES: usize = core::mem::size_of::<CsScratch>();

/// Both staging buffers, side by side.
const CS_TOTAL_BYTES: usize = 2 * CS_SCRATCH_BYTES;

/// Take the FT8 staging pair at boot, before WiFi.
///
/// Must run before anything fragments internal DRAM — see
/// [`crate::worker_arena`].
pub fn reserve_arena() -> bool {
    crate::worker_arena::reserve(crate::worker_arena::Owner::Ft8Scratch, CS_TOTAL_BYTES)
}

/// Main-core (PRO_CPU) cs Box staging.
///
/// Carved from [`crate::worker_arena`] rather than reserved here: the
/// same internal-DRAM block backs WSPR's and FST4's worker stacks, and
/// only one of the three modes runs in a given boot. It is still a
/// link-time reservation — see that module for why the heap is not an
/// option once WiFi is up.
///
/// Zero-initialised like the `.bss` statics this replaces, and
/// overwritten by `*scratch = *cs_box` before every use anyway.
///
/// # Safety
///
/// Only the PRO_CPU side of `dual_core::stage3_split` may touch this;
/// the APP_CPU worker uses [`cs_scratch_worker`]. The two are distinct,
/// non-overlapping halves of the same claim.
pub unsafe fn cs_scratch_main() -> &'static mut CsScratch {
    let base = crate::worker_arena::claim(crate::worker_arena::Owner::Ft8Scratch, CS_TOTAL_BYTES)
        .expect("internal_pool: cs scratch arena unavailable");
    // SAFETY: the arena is 16-byte aligned (`Cmplx<f32>` needs 4) and
    // the claim covers both halves.
    unsafe { &mut *(base as *mut CsScratch) }
}

/// APP_CPU (worker) cs Box staging — the second half of the same claim.
///
/// # Safety
///
/// Only the APP_CPU worker (`dual_core::worker_main`) may touch this.
pub unsafe fn cs_scratch_worker() -> &'static mut CsScratch {
    let base = crate::worker_arena::claim(crate::worker_arena::Owner::Ft8Scratch, CS_TOTAL_BYTES)
        .expect("internal_pool: cs scratch arena unavailable");
    // SAFETY: as above, offset past the main half.
    unsafe { &mut *(base.add(CS_SCRATCH_BYTES) as *mut CsScratch) }
}
