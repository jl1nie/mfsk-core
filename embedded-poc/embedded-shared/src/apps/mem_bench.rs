//! What two cores of the CoreS3 (ESP32-S3, LX7) share that slows each of them: the same loop on core 0 alone and on both cores at once, over different memories.
//! No FT8 in it. (`ft8-two-core-bench` found an OSD attempt 38 ms on one core and about 55 ms on each of two running together, the BP of the round 1.35 times slower: design note
//! section 30. The OSD memory's place, internal DRAM or PSRAM, explained 5 % of it.) Every arm is a chain of dependent reads (the next index comes from the value just read, as a table walk does)
//! over a table of its own per core unless it says shared, timed on core 0 while core 1 is idle, then on both with a common start (each core's own time is read; the second run of each is kept):
//!
//! | arm | what the loop reads |
//! |---|---|
//! | `alu` | nothing: eight independent f32 chains (the FPU), no memory |
//! | `int-sep` | 8 KB tables in internal DRAM, each core's in a **different 32 KB bank** (the S3's internal SRAM is in 32 KB banks; two masters on one bank are arbitrated) |
//! | `int-same` | 8 KB tables in internal DRAM, both in **one** 32 KB bank |
//! | `int-shared` | one 8 KB table in internal DRAM, both cores read it |
//! | `flash-8k`, `flash-128k` | one `static` table (flash `.rodata`, through the cache: where `NMS_EDGES` and the OSD's `COLS` are) of 8 KB (fits the data cache) and of 128 KB (does not) |
//! | `psram-32k`, `psram-512k` | a table per core in PSRAM, 32 KB and 512 KB |
//!
//! Logged per arm: ns a read (core 0 alone; core 0 and core 1 together) and the slowdown of core 0 (together over alone) and of core 1 (together over its own alone, measured the same way
//! with core 0 idle). The addresses of the tables are logged (internal DRAM 0x3FC8..., PSRAM 0x3C..., flash 0x3C... below the PSRAM window: the log says which each is).
//!
//! **Predictions, written before the first run.** The observation to explain: x1.35-1.45 on the BP / OSD code of the cells flow when both cores run. Four readings, and what each says:
//! **(flash cache)** `flash-8k` and `flash-128k` slow by **x1.25 or more**, the rest x1.10 or less; **(bank)** `int-same` slows by x1.25 or more and `int-sep` / `int-shared` by x1.10 or less;
//! **(PSRAM / cache)** `psram-32k` / `psram-512k` slow by x1.25 or more; **(none of the memories)** every arm x1.10 or less, `alu` included: the slowdown is not in the loads (the tick and
//! other interrupts on the second core, a lock, the code's fetch from the shared instruction cache, a frequency change when both run). The expected values with no sharing: **x1.00 +-0.03**. If two
//! readings hold at once, the arms that slow tell which. Not a pass / fail test: the numbers are the result.

extern crate alloc;
extern crate std;

use alloc::vec::Vec;
use core::hint::black_box;
use core::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};

const INTERNAL: u32 = (1 << 11) | (1 << 2);

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

const fn lcg_table<const N: usize>() -> [u32; N] {
    let mut t = [0u32; N];
    let mut x = 12345u32;
    let mut i = 0;
    while i < N {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
        t[i] = x;
        i += 1;
    }
    t
}
/// Flash `.rodata` (not `DRAM_ATTR`): read through the cache.
static FLASH_8K: [u32; 2048] = lcg_table::<2048>();
static FLASH_128K: [u32; 32768] = lcg_table::<32768>();

/// A walk of `n` dependent reads over `table` (a power of two long): the next index is made from the value read.
#[inline(never)]
fn walk(table: &[u32], n: usize, seed: u32) -> u32 {
    let mask = table.len() - 1;
    let (mut x, mut acc) = (seed, 0u32);
    for _ in 0..n {
        let v = unsafe { *table.get_unchecked((x as usize) & mask) };
        acc = acc.wrapping_add(v);
        x = x.wrapping_mul(1664525).wrapping_add(1013904223) ^ v;
    }
    black_box(acc)
}

/// Eight independent f32 chains: the FPU and nothing else.
#[inline(never)]
fn alu(n: usize) -> u32 {
    let mut a = [1.0f32, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7];
    for _ in 0..n {
        for v in a.iter_mut() {
            *v = *v * 1.000_000_1 + 0.5;
            if *v > 1e6 {
                *v = 1.0;
            }
        }
    }
    black_box(a.iter().sum::<f32>()) as u32
}

#[derive(Clone, Copy)]
struct Tab(*const u32, usize);
unsafe impl Send for Tab {}
impl Tab {
    fn slice(&self) -> &'static [u32] {
        // SAFETY: the tables outlive the bench (it never returns) and are only read
        unsafe { core::slice::from_raw_parts(self.0, self.1) }
    }
}

#[derive(Clone, Copy)]
enum Kernel {
    Alu(usize),
    Walk(Tab, usize),
}
fn run_kernel(k: Kernel, seed: u32) -> u32 {
    match k {
        Kernel::Alu(n) => alu(n),
        Kernel::Walk(t, n) => walk(t.slice(), n, seed),
    }
}

static START: AtomicU32 = AtomicU32::new(0);

fn spawn_pinned(name: &'static core::ffi::CStr, core: esp_idf_svc::hal::cpu::Core, stack: usize, f: impl FnOnce() + Send + 'static) -> std::thread::JoinHandle<()> {
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    let d = ThreadSpawnConfiguration::default();
    ThreadSpawnConfiguration { name: Some(name), stack_size: stack, pin_to_core: Some(core), ..d }
        .set()
        .expect("mem-bench: thread config");
    let h = std::thread::Builder::new().stack_size(stack).spawn(f).expect("mem-bench: spawn");
    let _ = ThreadSpawnConfiguration::default().set();
    h
}

pub fn run() -> ! {
    log::info!("mem-bench: what two cores share (predictions in the module doc)");
    let h = spawn_pinned(c"mem_bench", esp_idf_svc::hal::cpu::Core::Core0, 16 * 1024, body);
    let _ = h.join();
    log::info!("mem-bench: done");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn body() {
    // core 1: waits for a kernel, spins on START (a common start), runs it, replies its microseconds
    let (tx, rx): (Sender<(Kernel, u32, u32)>, Receiver<(Kernel, u32, u32)>) = channel();
    let (done_tx, done_rx): (Sender<(i64, u32)>, Receiver<(i64, u32)>) = channel();
    let _c1 = spawn_pinned(c"mem_c1", esp_idf_svc::hal::cpu::Core::Core1, 16 * 1024, move || {
        while let Ok((k, seed, gen)) = rx.recv() {
            while START.load(Ordering::Acquire) != gen {
                core::hint::spin_loop();
            }
            let t0 = now_us();
            let r = run_kernel(k, seed);
            done_tx.send((now_us() - t0, r)).unwrap();
        }
    });

    // internal DRAM: 96 KB aligned to 32 KB (three banks): the tables go in the same bank or in two banks
    let blk = unsafe { esp_idf_svc::sys::heap_caps_aligned_alloc(32768, 98304, INTERNAL) } as *mut u32;
    assert!(!blk.is_null(), "mem-bench: 96 KB of internal DRAM");
    let words = |bytes: usize| bytes / 4;
    let fill = |p: *mut u32, n: usize, salt: u32| {
        let mut x = salt;
        for i in 0..n {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            unsafe { *p.add(i) = x };
        }
    };
    for i in 0..words(98304) {
        unsafe { *blk.add(i) = (i as u32).wrapping_mul(2654435761) };
    }
    fill(blk, words(8192), 1);
    fill(unsafe { blk.add(words(8192)) }, words(8192), 2); // same bank as the first: offset 8 KB
    fill(unsafe { blk.add(words(32768)) }, words(8192), 3); // the next bank
    let t8 = |off: usize| Tab(unsafe { blk.add(words(off)) }, words(8192));
    // PSRAM tables: a vector each, above the internal limit
    let mk = |n: usize, salt: u32| -> Tab {
        let mut v: Vec<u32> = alloc::vec![0; n];
        let mut x = salt;
        for e in v.iter_mut() {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            *e = x;
        }
        let t = Tab(v.as_ptr(), n);
        core::mem::forget(v);
        t
    };
    let (p32a, p32b, p512a, p512b) = (mk(8192, 5), mk(8192, 6), mk(131072, 7), mk(131072, 8));
    log::info!(
        "mem-bench: internal block {:p} (bank {}), tables at +0 / +8 KB / +32 KB; flash 8k {:p}, 128k {:p}; psram {:p} {:p} {:p} {:p}",
        blk,
        (blk as usize >> 15) & 0xFF,
        FLASH_8K.as_ptr(),
        FLASH_128K.as_ptr(),
        p32a.0,
        p32b.0,
        p512a.0,
        p512b.0
    );
    let (f8, f128) = (Tab(FLASH_8K.as_ptr(), FLASH_8K.len()), Tab(FLASH_128K.as_ptr(), FLASH_128K.len()));

    // (arm, kernel of core 0, kernel of core 1, reads)
    let arms: [(&str, Kernel, Kernel); 9] = [
        ("alu       ", Kernel::Alu(2_000_000), Kernel::Alu(2_000_000)),
        ("int-sep   ", Kernel::Walk(t8(0), 6_000_000), Kernel::Walk(t8(32768), 6_000_000)),
        ("int-same  ", Kernel::Walk(t8(0), 6_000_000), Kernel::Walk(t8(8192), 6_000_000)),
        ("int-shared", Kernel::Walk(t8(0), 6_000_000), Kernel::Walk(t8(0), 6_000_000)),
        ("flash-8k  ", Kernel::Walk(f8, 6_000_000), Kernel::Walk(f8, 6_000_000)),
        ("flash-128k", Kernel::Walk(f128, 2_000_000), Kernel::Walk(f128, 2_000_000)),
        ("psram-32k ", Kernel::Walk(p32a, 3_000_000), Kernel::Walk(p32b, 3_000_000)),
        ("psram-512k", Kernel::Walk(p512a, 600_000), Kernel::Walk(p512b, 600_000)),
        ("psram-shr ", Kernel::Walk(p32a, 3_000_000), Kernel::Walk(p32a, 3_000_000)),
    ];
    let mut gen = 0u32;
    let mut sink = 0u32;
    for (name, k0, k1) in arms {
        let ops = match k0 {
            Kernel::Alu(n) => n * 8,
            Kernel::Walk(_, n) => n,
        } as f64;
        // core 0 alone, core 1 alone (the other idle), then both; the second run of each is kept
        let (mut a0, mut a1, mut b0, mut b1) = (0i64, 0i64, 0i64, 0i64);
        for _ in 0..2 {
            let t0 = now_us();
            sink ^= run_kernel(k0, 11);
            a0 = now_us() - t0;
            gen += 1;
            tx.send((k1, 22, gen)).unwrap();
            START.store(gen, Ordering::Release);
            let (t, r) = done_rx.recv().unwrap();
            a1 = t;
            sink ^= r;
            gen += 1;
            tx.send((k1, 22, gen)).unwrap();
            // let core 1 reach its spin, then start both
            unsafe { esp_idf_svc::sys::vTaskDelay(2) };
            START.store(gen, Ordering::Release);
            let t0 = now_us();
            sink ^= run_kernel(k0, 11);
            b0 = now_us() - t0;
            let (t, r) = done_rx.recv().unwrap();
            b1 = t;
            sink ^= r;
        }
        log::info!(
            "mem-bench: {name} core 0: {:.2} ns alone, {:.2} ns with core 1 (x{:.2}); core 1: {:.2} ns alone, {:.2} ns with core 0 (x{:.2})",
            a0 as f64 * 1000.0 / ops,
            b0 as f64 * 1000.0 / ops,
            b0 as f64 / a0 as f64,
            a1 as f64 * 1000.0 / ops,
            b1 as f64 * 1000.0 / ops,
            b1 as f64 / a1 as f64
        );
    }
    log::info!("mem-bench: sink {sink}");
}
