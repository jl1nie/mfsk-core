//! One internal-DRAM block for whichever mode's decode workers are
//! running, taken at boot before anything else has fragmented the heap.
//!
//! Three consumers each need a large internal-DRAM scratch, and only
//! one of them is ever live in a given boot:
//!
//! | consumer | bytes | used by |
//! |---|---|---|
//! | [`Owner::WsprWorker`] | 81 920 | WSPR's `wspr_dual_core` worker stack |
//! | [`Owner::Fst4Worker`] | 24 576 | FST4's `fst4_dual_core` worker stack |
//! | [`Owner::Ft8Scratch`] | 10 112 | FT8's two `cs Box` staging buffers |
//!
//! ## Why not `.bss`, which is where these used to live
//!
//! A `.bss` reservation is paid whether or not the mode runs, and the
//! linker cannot drop it once anything references it. Measured
//! 2026-08-23: making the three share one 81 920 B `.bss` union took
//! the FT8 controller's static footprint from 79 102 B to 150 910 B —
//! it now carried WSPR's 80 KB to use 10 KB of it. That is 72 KB out
//! of a build that had 73 KB of heap left with WiFi, the USB host and
//! audio all running (#163). A union in `.bss` is the wrong shape: it
//! sizes every binary for the greediest mode.
//!
//! ## Why not the heap, which is where they lived before *that*
//!
//! `wspr_dual_core`'s worker stack was a heap allocation guarded by a
//! live `heap_caps_get_largest_free_block` check, taken lazily at the
//! start of each pass and falling back to single-core when the block
//! was not there. With WiFi associated, pass 0's worker got its stack
//! and pass 1's did not — costing that pass 17 583 -> 28 246 ms and
//! the scan 11 s, with nothing in the log to say why (issue #260).
//! The 2026-08-23 session measured the same wall from the other side:
//! with WiFi up, the largest free internal block is 31 744 B, so an
//! 80 KB allocation is not unlikely, it is impossible.
//!
//! ## What actually works
//!
//! Both of those are true at once, and neither is the whole picture:
//! the heap is only that tight *after* WiFi and the USB host have
//! taken their share. At boot it is not. Measured on the same board,
//! same session, before any of it starts:
//!
//! ```text
//! [mem] pre-thread-spawn free_internal=247471 largest=155648
//! ```
//!
//! So [`reserve`] takes the block once, from `main`, after the NVS
//! `boot_mode` says which mode this boot is and before WiFi is
//! started. Only the running mode's block is ever allocated, so a
//! single-mode binary pays exactly what it uses and a build carrying
//! all three pays for the one it picked — which is the property a
//! `.bss` union cannot have.
//!
//! ## Exclusivity
//!
//! [`claim`] returns the same pointer to every request from the owner
//! that reserved it, and refuses a different one. Repeat claims are
//! expected — WSPR's pass-0/1 and pass-2 workers share the buffer, and
//! FT8 asks per slot — so only a *different* owner is an error, and
//! that means two modes were started in one boot.

use core::sync::atomic::{AtomicPtr, AtomicU8, AtomicUsize, Ordering};

/// Who is asking. The discriminants are what [`OWNER`] stores, so 0
/// stays free to mean "nothing reserved yet".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Owner {
    Ft8Scratch = 1,
    Fst4Worker = 2,
    WsprWorker = 3,
}

static OWNER: AtomicU8 = AtomicU8::new(0);
static BASE: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());
static BYTES: AtomicUsize = AtomicUsize::new(0);

/// Take the block for `owner`, once, at boot.
///
/// Call from `main` after the boot mode is known and **before WiFi or
/// the USB host start** — that ordering is the whole point; see the
/// module docs. Returns `false` if the allocation failed or a
/// different owner already reserved.
///
/// 16-byte aligned, which `heap_caps_malloc` gives on this target, so
/// the Xtensa port never has to shave a task's stack base and
/// PIE-aligned buffers at the bottom of a frame keep their alignment.
pub fn reserve(owner: Owner, bytes: usize) -> bool {
    let tag = owner as u8;
    match OWNER.compare_exchange(0, tag, Ordering::AcqRel, Ordering::Acquire) {
        Ok(_) => {}
        Err(held) if held == tag => return !BASE.load(Ordering::Acquire).is_null(),
        Err(held) => {
            log::error!("worker_arena: {owner:?} cannot reserve — owner {held} already has it");
            return false;
        }
    }

    // SAFETY: a plain sized allocation; the caps ask for internal
    // DRAM, which is the point — PSRAM would cost the decode workers
    // their speed.
    let p = unsafe {
        esp_idf_svc::sys::heap_caps_malloc(
            bytes,
            esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT,
        )
    } as *mut u8;

    if p.is_null() {
        // SAFETY: read-only query.
        let largest = unsafe {
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT,
            )
        };
        log::error!(
            "worker_arena: {owner:?} could not reserve {bytes} B of internal DRAM \
             (largest free {largest} B) — reserve() must run before WiFi starts"
        );
        OWNER.store(0, Ordering::Release);
        return false;
    }

    BASE.store(p, Ordering::Release);
    BYTES.store(bytes, Ordering::Release);
    log::info!("worker_arena: {owner:?} reserved {bytes} B of internal DRAM at {p:p}");
    true
}

/// The reserved block, if `owner` reserved it and it covers `bytes`.
///
/// Carving it up is the caller's job, and so is keeping the pieces
/// non-overlapping.
pub fn claim(owner: Owner, bytes: usize) -> Option<*mut u8> {
    let held = OWNER.load(Ordering::Acquire);
    if held != owner as u8 {
        log::error!(
            "worker_arena: {owner:?} claimed but owner is {held} — was reserve() called at boot?"
        );
        return None;
    }
    let have = BYTES.load(Ordering::Acquire);
    if bytes > have {
        log::error!("worker_arena: {owner:?} wants {bytes} B but only {have} B was reserved");
        return None;
    }
    let base = BASE.load(Ordering::Acquire);
    if base.is_null() {
        return None;
    }
    Some(base)
}

/// `(owner, bytes)` for a status line; `None` if nothing is reserved.
pub fn reserved() -> Option<(u8, usize)> {
    match OWNER.load(Ordering::Acquire) {
        0 => None,
        n => Some((n, BYTES.load(Ordering::Acquire))),
    }
}
