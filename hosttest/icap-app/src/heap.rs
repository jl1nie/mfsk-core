// SPDX-License-Identifier: GPL-3.0-only
//! A counting global allocator: live bytes and their high-water mark.

use std::alloc::{GlobalAlloc, Layout, System};
use std::backtrace::Backtrace;
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
use std::sync::{Arc, Mutex};

static CUR: AtomicUsize = AtomicUsize::new(0);
/// Allocations (`alloc` and a growing `realloc`) since the process started: [`alloc_count`].
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
static ALLOC_MAX: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

/// Allocations of at least this many bytes are logged (size only), the first 256 since [`reset_peak`].
pub const BIG_MIN: usize = 64 * 1024;
const BIG_MAX: usize = 256;
static BIG_N: AtomicUsize = AtomicUsize::new(0);
static BIG: [AtomicUsize; BIG_MAX] = [const { AtomicUsize::new(0) }; BIG_MAX];

fn log_big(n: usize) {
    if n >= BIG_MIN {
        let i = BIG_N.fetch_add(1, Relaxed);
        if i < BIG_MAX {
            BIG[i].store(n, Relaxed);
        }
    }
}

pub struct Meter;

fn add(n: usize) {
    let c = CUR.fetch_add(n, Relaxed) + n;
    PEAK.fetch_max(c, Relaxed);
}

unsafe impl GlobalAlloc for Meter {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Relaxed);
        ALLOC_BYTES.fetch_add(l.size(), Relaxed);
        ALLOC_MAX.fetch_max(l.size(), Relaxed);
        add(l.size());
        log_big(l.size());
        let p = unsafe { System.alloc(l) };
        diag_note(p as usize, l.size());
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        CUR.fetch_sub(l.size(), Relaxed);
        diag_forget(p as usize);
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Relaxed);
        ALLOC_BYTES.fetch_add(n, Relaxed);
        ALLOC_MAX.fetch_max(n, Relaxed);
        if n > l.size() {
            add(n - l.size());
            log_big(n);
        } else {
            CUR.fetch_sub(l.size() - n, Relaxed);
        }
        diag_forget(p as usize);
        let q = unsafe { System.realloc(p, l, n) };
        diag_note(q as usize, n);
        q
    }
}

// ---- diagnostic: who holds the heap at its peak --------------------------------------------------------
//
// Off unless [`diag_start`] is called. Every allocation of at least `DIAG_MIN` bytes is recorded with where
// it was allocated (the innermost mfsk-core / icap-app frames); each time the recorded live total passes its
// previous peak the recorded set is copied. The totals are those of the recorded set, not of the meter, which
// the diagnostic's own allocations would inflate. [`diag_report`] names what held the memory at the peak, and at the
// highest point **after coarse sync** (once nothing allocated in `coarse_sync.rs` / `spectrogram.rs` is
// live any more), which is the peak of everything that follows.

const DIAG_MIN: usize = 16;
static DIAG_ON: AtomicBool = AtomicBool::new(false);

thread_local! {
    static IN_DIAG: Cell<bool> = const { Cell::new(false) };
}

struct Entry {
    ptr: usize,
    size: usize,
    at: Arc<str>,
    coarse: bool,
}

#[derive(Default)]
struct Snap {
    held: Vec<(usize, Arc<str>)>,
    total: usize,
    tracked: usize,
}

struct Diag {
    live: Vec<Entry>,
    live_bytes: usize,
    coarse_seen: bool,
    late: bool,
    all: Snap,
    after_coarse: Snap,
}

static DIAG: Mutex<Diag> = Mutex::new(Diag {
    live: Vec::new(),
    live_bytes: 0,
    coarse_seen: false,
    late: false,
    all: Snap {
        held: Vec::new(),
        total: 0,
        tracked: 0,
    },
    after_coarse: Snap {
        held: Vec::new(),
        total: 0,
        tracked: 0,
    },
});

fn snapshot(live: &[Entry], bytes: usize, into: &mut Snap) {
    into.held = live.iter().map(|e| (e.size, e.at.clone())).collect();
    into.total = bytes;
    into.tracked = bytes;
}

fn diag_note(ptr: usize, size: usize) {
    if size < DIAG_MIN || !DIAG_ON.load(Relaxed) || ptr == 0 {
        return;
    }
    IN_DIAG.with(|f| {
        if f.get() {
            return;
        }
        f.set(true);
        let at: Arc<str> = frames(&Backtrace::force_capture().to_string()).into();
        let coarse = at.contains("coarse_sync.rs") || at.contains("spectrogram.rs");
        if let Ok(mut d) = DIAG.lock() {
            d.coarse_seen |= coarse;
            d.live.push(Entry {
                ptr,
                size,
                at,
                coarse,
            });
            d.live_bytes += size;
            let cur = d.live_bytes;
            if cur > d.all.total {
                let Diag { live, all, .. } = &mut *d;
                snapshot(live, cur, all);
            }
            if d.late && cur > d.after_coarse.total {
                let Diag {
                    live, after_coarse, ..
                } = &mut *d;
                snapshot(live, cur, after_coarse);
            }
        }
        f.set(false);
    });
}

fn diag_forget(ptr: usize) {
    if !DIAG_ON.load(Relaxed) {
        return;
    }
    IN_DIAG.with(|f| {
        if f.get() {
            return;
        }
        f.set(true);
        if let Ok(mut d) = DIAG.lock()
            && let Some(i) = d.live.iter().rposition(|e| e.ptr == ptr)
        {
            let e = d.live.swap_remove(i);
            d.live_bytes -= e.size;
            if d.coarse_seen && !d.late && !d.live.iter().any(|e| e.coarse) {
                d.late = true;
                d.after_coarse.total = d.live_bytes;
            }
        }
        f.set(false);
    });
}

/// Start recording allocations of 1 KiB or more (slow: a backtrace each). Call before the decode.
pub fn diag_start() {
    if let Ok(mut d) = DIAG.lock() {
        d.live.clear();
        d.live_bytes = 0;
        d.coarse_seen = false;
        d.late = false;
        d.all = Snap::default();
        d.after_coarse = Snap::default();
    }
    DIAG_ON.store(true, Relaxed);
}

/// One snapshot grouped by where it was allocated: `(bytes, count, where)` largest first.
pub struct Held {
    pub groups: Vec<(usize, usize, String)>,
    pub total: usize,
}

fn group(s: &Snap) -> Held {
    let mut groups: Vec<(usize, usize, String)> = Vec::new();
    for (size, at) in &s.held {
        match groups.iter_mut().find(|g| g.2 == **at) {
            Some(g) => {
                g.0 += size;
                g.1 += 1;
            }
            None => groups.push((*size, 1, at.to_string())),
        }
    }
    groups.sort_by_key(|g| core::cmp::Reverse(g.0));
    Held {
        groups,
        total: s.total,
    }
}

/// What held the heap at its highest point since [`diag_start`], and at its highest point after coarse sync.
pub fn diag_report() -> (Held, Held) {
    DIAG_ON.store(false, Relaxed);
    let d = DIAG.lock().unwrap();
    (group(&d.all), group(&d.after_coarse))
}

/// The innermost frames of a rendered backtrace that are in mfsk-core or this crate's sources, as
/// `file:line function`. The frames are `N: symbol` lines, each followed by `at path:line`.
fn frames(bt: &str) -> String {
    let mut out: Vec<String> = Vec::new();
    let mut sym = "";
    for l in bt.lines() {
        let l = l.trim();
        if let Some(at) = l.strip_prefix("at ") {
            let ours = (at.contains("mfsk-core/src/") || at.contains("icap-app/src/"))
                && !at.contains("heap.rs");
            if ours {
                let file = at.rsplit('/').next().unwrap_or(at);
                let f = format!("{file} {}", sym.split('<').next().unwrap_or(sym));
                if out.last() != Some(&f) {
                    out.push(f);
                }
                if out.len() == 3 {
                    break;
                }
            }
        } else if let Some((_, s)) = l.split_once(": ") {
            sym = s;
        }
    }
    if out.is_empty() {
        "(no crate frame)".to_string()
    } else {
        out.join(" <- ")
    }
}

/// Live heap bytes now.
pub fn current() -> usize {
    CUR.load(Relaxed)
}

/// Restart the high-water mark from the live bytes; returns them.
pub fn reset_peak() -> usize {
    BIG_N.store(0, Relaxed);
    let c = CUR.load(Relaxed);
    PEAK.store(c, Relaxed);
    c
}

/// High-water mark since [`reset_peak`].
pub fn peak() -> usize {
    PEAK.load(Relaxed)
}

/// How many allocations (`alloc`, `realloc`) the process has made: the difference across a call is the number of heap allocations it made.
pub fn alloc_count() -> usize {
    ALLOCS.load(Relaxed)
}

/// The largest single allocation since [`reset_alloc_max`].
pub fn alloc_max() -> usize {
    ALLOC_MAX.load(Relaxed)
}

/// Forget the largest allocation so far.
pub fn reset_alloc_max() {
    ALLOC_MAX.store(0, Relaxed);
}

/// The bytes those allocations asked for (a `realloc` counts its new size).
pub fn alloc_bytes() -> usize {
    ALLOC_BYTES.load(Relaxed)
}

/// The large allocations (>= [`BIG_MIN`]) made since [`reset_peak`], largest first, one entry per
/// distinct size with how many times that size was allocated.
pub fn big_allocs() -> Vec<(usize, usize)> {
    let n = BIG_N.load(Relaxed).min(BIG_MAX);
    let mut v: Vec<usize> = (0..n).map(|i| BIG[i].load(Relaxed)).collect();
    v.sort_unstable_by(|a, b| b.cmp(a));
    let mut out: Vec<(usize, usize)> = Vec::new();
    for s in v {
        match out.last_mut() {
            Some(l) if l.0 == s => l.1 += 1,
            _ => out.push((s, 1)),
        }
    }
    out
}
