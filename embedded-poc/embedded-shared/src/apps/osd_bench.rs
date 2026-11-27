//! FT8's OSD on the CoreS3: `osd174_91` (the host's port, `u64` rows, a `[u128; 256]` and `Vec`s a call) against `osd174_91_into` (the embedded one: `u32` rows,
//! compile-time columns, the caller's memory, a distance that stops early), on the same 44 LLR vectors, one core, no WiFi / USB host.
//!
//! The inputs (`assets/osd_micro_in.bin`) are the `ndeep = 2` vectors of `mfsk-core/tests/fixtures/osd_upstream` (LLRs that reached OSD in real FT8 decodes): 6 that
//! upstream decodes and 38 that it does not (the cells flow's OSD fails about nine times in ten). Each of the two runs over all 44, **twice; the second is read**, the
//! embedded one first so the stack high-water mark after it is its own. Logged: microseconds a call for each and in all, how many vectors decode, **whether the two
//! agree on every one** (codeword and hard errors), the stack free after each, the internal heap minimum.
//!
//! **Predictions, written before the first run.** Host callgrind (`scripts/s3-stage-instr.sh` with OSD on, qso3_busy / qso1 / qso2): `osd174_91` is 0.72 M instructions a
//! call, `osd174_91_into` 0.76-0.77 M (+7 % on x86-64, where a `u32` row is two operations for the `u64`'s one: the host cannot show the gain this port is for). The board's
//! BP pass is 37 600 host instructions and 0.41 ms: 10.9 ns an instruction.
//!
//! | | prediction | why |
//! |---|---|---|
//! | `osd174_91`, a call | **6-10 ms** (7.8 at the BP ratio) | 0.72 M instructions at 10.9 ns; the `u64` shifts and the `u128` assembly are on top if the LX7 pays for them |
//! | `osd174_91_into`, a call | **4-8.5 ms**, **0.55-1.05 times the old** | 0.76 M instructions at the same ratio (8.3 ms) if the `u32` words buy nothing; fewer instructions on this core if the `u64` ops were costing |
//! | agreement | **44 of 44** | `tests/osd_embedded.rs` on the host: 22 000 inputs, equal |
//! | decodes | 6 of 44 (both) | the fixture's labels |
//! | stack, embedded | **under 1.5 KB** used by the call (the `Osd174Scratch`, 4.5 KB, is on the heap) | |
//! | stack, old | 9-12 KB used (`osd_npre`'s frame, 10.3 KB, in the gate bench's ELF) | |
//!
//! **The generator built from 32 x 32 transposes (feature `osd-gen-transpose`), predicted before its run.** The scatter it replaces is about 8 000 trips of a data-dependent `trailing_zeros` loop a call (174 columns x
//! about 45 set bits), each a read-modify-write: say 10-14 cycles a trip and a branch that mispredicts, 80-110 k cycles, 0.35-0.45 ms of the 5.60 ms; the 18 transposes are 18 x 80 pair steps of constant
//! trip counts, about 10 k cycles. **Predicted: `osd174_91_into` 5.60 -> 5.15-5.35 ms a call (-5 to -8 %), 44 of 44 equal, 6 of 44 decoded.** Stop rule: more than 25 % off, or one vector differing.
//!
//! **Stop rule:** if either call is outside its range by more than 25 %, or the two disagree on one vector, stop and explain it on the host before the next flash.

extern crate alloc;
extern crate std;

use alloc::vec::Vec;
use std::time::Instant;

use mfsk_core::fec::ldpc::LDPC_N;
use mfsk_core::fec::ldpc::osd_emb::{Osd174Scratch, osd174_91_into};
use mfsk_core::fec::ldpc::osd_npre::{NDEEP2_174_91, osd174_91};

const INTERNAL: u32 = esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT;

fn stack_free() -> usize {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize }
}

fn min_free_internal() -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_minimum_free_size(INTERNAL) }
}

fn load(bytes: &[u8]) -> Vec<[f32; LDPC_N]> {
    let n = u32::from_le_bytes(bytes[0..4].try_into().unwrap()) as usize;
    assert_eq!(bytes.len(), 4 + n * 4 * LDPC_N, "osd_micro_in.bin: size");
    (0..n)
        .map(|i| {
            let mut v = [0f32; LDPC_N];
            for (x, c) in v.iter_mut().zip(bytes[4 + i * 4 * LDPC_N..].as_chunks::<4>().0) {
                *x = f32::from_le_bytes(*c);
            }
            v
        })
        .collect()
}

pub fn run(target: &str, inputs: &'static [u8]) -> ! {
    log::info!("osd-bench: {target}, osd174_91 against osd174_91_into, one core (predictions in the module doc)");
    let h = std::thread::Builder::new()
        .name("osd_bench".into())
        .stack_size(48 * 1024)
        .spawn(move || body(inputs))
        .expect("osd-bench: could not spawn the bench thread");
    let _ = h.join();
    log::info!("osd-bench: done");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn body(inputs: &[u8]) {
    let llrs = load(inputs);
    log::info!("osd-bench: {} vectors; boot stack free {} B, min free internal {} B", llrs.len(), stack_free(), min_free_internal());
    // The embedded one's memory is made once, as the decoder would: on the heap.
    let mut sc = alloc::boxed::Box::new(Osd174Scratch::new());
    let mut hits: Vec<Option<(u32, [u8; LDPC_N])>> = Vec::new();
    let mut us_new = 0u64;
    for round in 0..2 {
        hits.clear();
        us_new = 0;
        for (i, llr) in llrs.iter().enumerate() {
            let t = Instant::now();
            let r = osd174_91_into(&mut sc, llr, None);
            let us = t.elapsed().as_micros() as u64;
            us_new += us;
            hits.push(r.map(|h| (h.hard_errors, *sc.codeword())));
            if round == 1 {
                log::info!("osd-bench: new #{i}: {us} us, {}", if r.is_some() { "decoded" } else { "no" });
            }
        }
    }
    let stack_after_new = stack_free();
    log::info!("osd-bench: new stack free {stack_after_new} B (boot figure above)");
    let (mut us_old, mut agree, mut decoded) = (0u64, 0usize, 0usize);
    for round in 0..2 {
        us_old = 0;
        agree = 0;
        decoded = 0;
        for (i, llr) in llrs.iter().enumerate() {
            let t = Instant::now();
            let r = osd174_91(llr, NDEEP2_174_91, None);
            let us = t.elapsed().as_micros() as u64;
            us_old += us;
            decoded += r.is_some() as usize;
            let same = match (&r, &hits[i]) {
                (None, None) => true,
                (Some(o), Some((he, cw))) => o.hard_errors == *he && o.codeword[..] == cw[..],
                _ => false,
            };
            agree += same as usize;
            if round == 1 {
                log::info!("osd-bench: old #{i}: {us} us, {}{}", if r.is_some() { "decoded" } else { "no" }, if same { "" } else { "  DISAGREES" });
            }
        }
    }
    let n = llrs.len() as u64;
    log::info!("osd-bench: old stack free {} B", stack_free());
    log::info!(
        "osd-bench: old {} us ({} us a call), new {} us ({} us a call), new/old {:.3}; agree {agree} of {}, decoded {decoded} of {}; min free internal {} B",
        us_old,
        us_old / n,
        us_new,
        us_new / n,
        us_new as f32 / us_old as f32,
        llrs.len(),
        llrs.len(),
        min_free_internal()
    );
}
