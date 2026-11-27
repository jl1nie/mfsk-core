//! What two cores of the CoreS3 (ESP32-S3, LX7) share that slows each of them: the same loop on core 0 alone and on both cores at once, over different memories and with the code in
//! different memories. No FT8 in it. (`ft8-two-core-bench` found an OSD attempt 38 ms on one core and about 55 ms on each of two running together, the BP of the round 1.35 times slower:
//! design note section 30. The OSD memory's place, internal DRAM or PSRAM, explained 5 % of it.) The loop of a memory arm **chases a cyclic permutation** (`i = table[i]`, one cycle through every
//! entry: Sattolo), so the working set is the whole table and each step waits for its load; a table of its own per core unless it says shared. Timed on core 0 while core 1 is idle, on core 1 the
//! same way, then on both with a common start (the second run of each is kept):
//!
//! | arm | what the loop reads / where its code is |
//! |---|---|
//! | `alu`, `mul` | no memory: eight independent f32 chains; a dependent integer multiply-add chain (the code in flash, fetched through the instruction cache) |
//! | `alu-iram`, `mul-iram` | the same loops with their code in **IRAM** (`.iram1`: no cache, no flash) |
//! | `int-sep` | 8 KB tables in internal DRAM, each core's in a **different 32 KB bank** (the S3's internal SRAM is in 32 KB banks; two masters on one bank are arbitrated) |
//! | `int-same` | 8 KB tables in internal DRAM, both in **one** 32 KB bank |
//! | `int-shared` | one 8 KB table in internal DRAM, both cores read it |
//! | `flash-8k`, `flash-128k` | one `static` table (flash `.rodata`, through the data cache: where `NMS_EDGES` and the OSD's `COLS` are) of 8 KB (fits the cache) and of 128 KB (does not) |
//! | `psram-32k`, `psram-512k`, `psram-shr` | a table per core in PSRAM, 32 KB and 512 KB; one 32 KB table shared |
//!
//! The shared-table arms start the second core half a cycle on (the first version of this bench walked them in step). The CPU frequency is read back and logged.
//!
//! **The first run (2026-11-27) is void for the memory arms**: its walk made the index from the low bits of a state that depends only on those bits, so it cycled through a few hundred entries in
//! every table and every arm ran from the cache at the same 54.2 ns a step. What it did show is real: the two **compute-only** loops, `alu` (x1.15) and the multiply chain of the walk (x1.23), ran
//! slower on each core while the other core ran the same loop, with no data memory on the critical path. That points at the code's fetch, or at the core itself, before any data memory.
//!
//! **Predictions for this run, written before it.** The values with nothing shared: **x1.00 +-0.03**. Readings, and what each says: **(code fetch)** `alu` and `mul` (code in flash) slow
//! by **x1.10-1.25** as before and `alu-iram` / `mul-iram` by **x1.03 or less**; **(core / clock / interrupts)** the IRAM variants slow as much as the flash ones (x1.10-1.25), and then every arm
//! carries that and the memory arms say little; **(flash cache)** `flash-8k` / `flash-128k` slow by more than x1.25 and `int-*` by x1.10 or less; **(bank)** `int-same` x1.25 or more while `int-sep`
//! / `int-shared` x1.10 or less; **(PSRAM)** `psram-*` x1.25 or more. A memory arm's step costs: internal DRAM 3-6 ns, flash-8k about the same (a cache hit), flash-128k and psram-512k 40-150 ns (misses).
//! Not a pass / fail test: the numbers are the result.

extern crate alloc;
extern crate std;

use alloc::vec::Vec;
use core::hint::black_box;
use core::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};

const INTERNAL: u32 = (1 << 11) | (1 << 2);

/// The core's cycle counter (`CCOUNT`, through the HAL: inline assembly is not stable on Xtensa).
#[inline(always)]
fn cc() -> u32 {
    unsafe { esp_idf_svc::sys::xthal_get_ccount() }
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// A cyclic permutation of `0..N` (Sattolo): following `i = t[i]` from any entry visits all `N` before it returns.
const fn perm_table<const N: usize>() -> [u32; N] {
    let mut t = [0u32; N];
    let mut i = 0;
    while i < N {
        t[i] = i as u32;
        i += 1;
    }
    let mut x = 12345u32;
    let mut k = N - 1;
    while k >= 1 {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
        let j = ((x >> 8) as usize) % k;
        let tmp = t[k];
        t[k] = t[j];
        t[j] = tmp;
        k -= 1;
    }
    t
}

/// The same at run time, over `n` words at `p`.
fn fill_perm(p: *mut u32, n: usize, seed: u32) {
    let mut x = seed;
    unsafe {
        for i in 0..n {
            *p.add(i) = i as u32;
        }
        for k in (1..n).rev() {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let j = ((x >> 8) as usize) % k;
            let t = *p.add(k);
            *p.add(k) = *p.add(j);
            *p.add(j) = t;
        }
    }
}
/// Flash `.rodata` (not `DRAM_ATTR`): read through the cache.
static FLASH_8K: [u32; 2048] = perm_table::<2048>();
static FLASH_128K: [u32; 32768] = perm_table::<32768>();

/// `n` steps of `i = table[i]` over a cyclic permutation, from entry `start`.
#[inline(never)]
fn chase(table: &[u32], n: usize, start: usize) -> u32 {
    let (mut i, mut acc) = (start, 0u32);
    for _ in 0..n {
        i = unsafe { *table.get_unchecked(i) } as usize;
        acc ^= i as u32;
    }
    black_box(acc)
}

/// A dependent multiply-add chain: the integer pipeline and nothing else.
#[inline(never)]
fn mul(n: usize) -> u32 {
    let mut x = 12345u32;
    for _ in 0..n {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
    }
    black_box(x)
}

/// Eight independent f32 chains: the FPU and nothing else.
#[inline(never)]
fn alu(n: usize) -> u32 {
    alu_body(n)
}

/// `alu` with its code in IRAM.
#[inline(never)]
#[unsafe(link_section = ".iram1.mem_bench_alu")]
fn alu_iram(n: usize) -> u32 {
    alu_body(n)
}

/// `mul` with its code in IRAM.
#[inline(never)]
#[unsafe(link_section = ".iram1.mem_bench_mul")]
fn mul_iram(n: usize) -> u32 {
    let mut x = 12345u32;
    for _ in 0..n {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
    }
    black_box(x)
}

#[inline(always)]
fn alu_body(n: usize) -> u32 {
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
    AluIram(usize),
    Mul(usize),
    MulIram(usize),
    Walk(Tab, usize, usize),
}
fn run_kernel(k: Kernel, _seed: u32) -> u32 {
    match k {
        Kernel::Alu(n) => alu(n),
        Kernel::AluIram(n) => alu_iram(n),
        Kernel::Mul(n) => mul(n),
        Kernel::MulIram(n) => mul_iram(n),
        Kernel::Walk(t, n, start) => chase(t.slice(), n, start),
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
    let (done_tx, done_rx): (Sender<(i64, u32, u32)>, Receiver<(i64, u32, u32)>) = channel();
    let _c1 = spawn_pinned(c"mem_c1", esp_idf_svc::hal::cpu::Core::Core1, 16 * 1024, move || {
        while let Ok((k, seed, gen)) = rx.recv() {
            while START.load(Ordering::Acquire) != gen {
                core::hint::spin_loop();
            }
            let (t0, c0) = (now_us(), cc());
            let r = run_kernel(k, seed);
            let (cyc, us) = (cc().wrapping_sub(c0), now_us() - t0);
            done_tx.send((us, cyc, r)).unwrap();
        }
    });

    // internal DRAM: 96 KB aligned to 32 KB (three banks): the tables go in the same bank or in two banks
    let blk = unsafe { esp_idf_svc::sys::heap_caps_aligned_alloc(32768, 98304, INTERNAL) } as *mut u32;
    assert!(!blk.is_null(), "mem-bench: 96 KB of internal DRAM");
    let words = |bytes: usize| bytes / 4;
    // each 8 KB table a cyclic permutation of its own 2048 entries; at +0 and +8 KB (one bank) and +32 KB (the next)
    fill_perm(blk, words(8192), 1);
    fill_perm(unsafe { blk.add(words(8192)) }, words(8192), 2);
    fill_perm(unsafe { blk.add(words(32768)) }, words(8192), 3);
    let t8 = |off: usize| Tab(unsafe { blk.add(words(off)) }, words(8192));
    // PSRAM tables: a vector each, above the internal limit
    let mk = |n: usize, salt: u32| -> Tab {
        let mut v: Vec<u32> = alloc::vec![0; n];
        fill_perm(v.as_mut_ptr(), n, salt);
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
    // a table both cores read starts the second core half a cycle on, so that they do not walk the same lines in step (one core's miss would fill the other's hit)
    let half = |t: Tab| -> usize {
        let s = t.slice();
        let mut i = 0usize;
        for _ in 0..s.len() / 2 {
            i = s[i] as usize;
        }
        i
    };
    let arms: [(&str, Kernel, Kernel); 12] = [
        ("alu       ", Kernel::Alu(2_000_000), Kernel::Alu(2_000_000)),
        ("alu-iram  ", Kernel::AluIram(2_000_000), Kernel::AluIram(2_000_000)),
        ("mul       ", Kernel::Mul(20_000_000), Kernel::Mul(20_000_000)),
        ("mul-iram  ", Kernel::MulIram(20_000_000), Kernel::MulIram(20_000_000)),
        ("int-sep   ", Kernel::Walk(t8(0), 6_000_000, 0), Kernel::Walk(t8(32768), 6_000_000, 0)),
        ("int-same  ", Kernel::Walk(t8(0), 6_000_000, 0), Kernel::Walk(t8(8192), 6_000_000, 0)),
        ("int-shared", Kernel::Walk(t8(0), 6_000_000, 0), Kernel::Walk(t8(0), 6_000_000, half(t8(0)))),
        ("flash-8k  ", Kernel::Walk(f8, 6_000_000, 0), Kernel::Walk(f8, 6_000_000, half(f8))),
        ("flash-128k", Kernel::Walk(f128, 2_000_000, 0), Kernel::Walk(f128, 2_000_000, half(f128))),
        ("psram-32k ", Kernel::Walk(p32a, 3_000_000, 0), Kernel::Walk(p32b, 3_000_000, 0)),
        ("psram-512k", Kernel::Walk(p512a, 600_000, 0), Kernel::Walk(p512b, 600_000, 0)),
        ("psram-shr ", Kernel::Walk(p32a, 3_000_000, 0), Kernel::Walk(p32a, 3_000_000, half(p32a))),
    ];
    log::info!("mem-bench: the data cache is 32 KB, 8 ways, 32 B lines, shared by the two cores (sdkconfig); CPU {} MHz (`esp_rom_get_cpu_ticks_per_us`); the cycle counter is read around every run", unsafe { esp_idf_svc::sys::esp_rom_get_cpu_ticks_per_us() });
    let mut gen = 0u32;
    let mut sink = 0u32;
    for (name, k0, k1) in arms {
        let ops = match k0 {
            Kernel::Alu(n) | Kernel::AluIram(n) => n * 8,
            Kernel::Mul(n) | Kernel::MulIram(n) => n,
            Kernel::Walk(_, n, _) => n,
        } as f64;
        // core 0 alone, core 1 alone (the other idle), then both; the second run of each is kept
        let (mut a0, mut a1, mut b0, mut b1) = (0i64, 0i64, 0i64, 0i64);
        let (mut ca0, mut ca1, mut cb0, mut cb1) = (0u32, 0u32, 0u32, 0u32);
        for _ in 0..2 {
            let (t0, c0) = (now_us(), cc());
            sink ^= run_kernel(k0, 11);
            ca0 = cc().wrapping_sub(c0);
            a0 = now_us() - t0;
            gen += 1;
            tx.send((k1, 22, gen)).unwrap();
            START.store(gen, Ordering::Release);
            let (t, c, r) = done_rx.recv().unwrap();
            (a1, ca1) = (t, c);
            sink ^= r;
            gen += 1;
            tx.send((k1, 22, gen)).unwrap();
            // let core 1 reach its spin, then start both
            unsafe { esp_idf_svc::sys::vTaskDelay(2) };
            START.store(gen, Ordering::Release);
            let (t0, c0) = (now_us(), cc());
            sink ^= run_kernel(k0, 11);
            cb0 = cc().wrapping_sub(c0);
            b0 = now_us() - t0;
            let (t, c, r) = done_rx.recv().unwrap();
            (b1, cb1) = (t, c);
            sink ^= r;
        }
        log::info!(
            "mem-bench: {name} core 0: {:.2} ns alone, {:.2} ns with core 1 (x{:.2}); core 1: {:.2} ns alone, {:.2} ns with core 0 (x{:.2}); core 0 cycles a step {:.2} alone, {:.2} with ({:.0} / {:.0} MHz)",
            a0 as f64 * 1000.0 / ops,
            b0 as f64 * 1000.0 / ops,
            b0 as f64 / a0 as f64,
            a1 as f64 * 1000.0 / ops,
            b1 as f64 * 1000.0 / ops,
            b1 as f64 / a1 as f64,
            ca0 as f64 / ops,
            cb0 as f64 / ops,
            ca0 as f64 / a0 as f64,
            cb0 as f64 / b0 as f64
        );
        let _ = (ca1, cb1);
    }
    log::info!("mem-bench: sink {sink}");
}
