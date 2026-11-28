//! The cells flow's QSO-context AP rung on the CoreS3: what `CellsWorker::with_ap_context` and `decode_partner` cost on the LX7, and whether they decode what the host says they do
//! (`docs/notes/PI3_JT9_GAP.md` sections 22-24, `mfsk-core/tests/cells_ap.rs`, `hosttest/icap-app/tests/icap_ap.rs`). One core, no WiFi / USB host, the audio made on the board.
//!
//! The slots are AWGN with a synthetic reply to the operator (`K1JT HA0DU RR73` for the Rogers stage, `K1JT HA0DU -15` for Replying) at 1 500 Hz, 0.5 s in (the generator of the host tests);
//! the search runs once for a slot and the rounds on its cells, so the numbers compare like with like. Per slot:
//!
//! | what | timed |
//! |---|---|
//! | `round` (no context) | `decode_round_osd`, 30 candidates, the capped OSD (K = 6): the window path as it is |
//! | `round` (context) | the same with `set_ap_context`: the rung runs inside it on the candidates near the partner |
//! | `partner` | `decode_partner` (the partner's own candidates, up to 4, the AP rung each) |
//!
//! Per row: hits (the reply decoded) with and without the context, BP calls and passes (`BpStats`), microseconds. Then the worst case, a reply too weak to decode (-27 dB: every pass of the
//! stage fails on every partner candidate), the same on `qso3_busy` with the reply added, noise-only slots with the context (phantoms), and the thread's stack high-water mark and the heap
//! minima. A line `ap-bench: check ...` says whether the context decodes more than no context and the noise gives nothing; the run ends with `ap-bench: done ok` or `done FAIL`.
//!
//! **Predictions, written before the first run** (host: the cells window with the context costs +2-5 ms a slot, `PI3_JT9_GAP.md` section 24; the board's window-path jobs are about 120 times
//! the host's, section 21; a BP pass is 0.41-0.7 ms, an `osd174_91_into` 5.1-7.3 ms, one deferred OSD attempt 33-38 ms, `stage_bench`):
//! - **`partner`, a reply that decodes (-20 dB)**: it ends at the first accepted pass: **20-150 ms**.
//! - **`partner`, the worst case (-27 dB), Rogers (passes 9, 10, 11, 8; two LLR variants; BP then OSD at 1 and 2 iterations each)**: per candidate 8 masked BP calls of up to 30 iterations
//!   (a BP call that does not converge runs its iterations: 30 x 0.41 = 12 ms) and 16 OSD runs (5-7 ms each, plus the two zsum BPs): **about 100 + 100 = 200 ms a candidate, four candidates
//!   0.4-0.9 s**; Replying (passes 8, 5) half of that: **0.2-0.45 s**.
//! - **`round` with the context against without**: +0 on a slot whose partner decodes in the round, +0.1-0.4 s where it does not (the rung runs on the round's candidate near the partner).
//! - **The window path's slack** (list out 380-390 ms after the end, deadline 500 ms: about 120 ms) **does not hold the worst case**; the rung belongs on core 1 beside the window, or in the late path.
//! - **Hits** (host, 12 seeds, board's build): `RR73` at -20 dB 12 / 12 with the context, 0 without; -22 dB about 40 %; the report at -20 dB about 80 %, at -22 dB 10 %. **Stack**: the rung's
//!   frame +0.5-2 KB over the stage bench's 10.8 KB free (the 48 KB bench thread: the figure to read is the difference to a run of the same arm without a context).
//! **Stop rule:** if a figure is more than 25 % off its prediction, stop and explain it on the host before the next flash.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    ApContext, ApStage, CellTables, CellsOptions, CellsWorker, Found, Ft8BpScratch, Ft8Rate,
    GatedSearch, coarse_sync_gated_cells,
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

const INTERNAL: u32 = (1 << 11) | (1 << 2);
const PSRAM: u32 = (1 << 10) | (1 << 2);
const MYCALL: &str = "K1JT";
const PARTNER: &str = "HA0DU";
const F0: f32 = 1_500.0;
/// Seeds a cell of the hit table; the worst-case and the noise runs use fewer.
const SEEDS: u64 = 5;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
fn min_free(caps: u32) -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_minimum_free_size(caps) }
}
fn stack_free() -> usize {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize }
}

pub fn run(target: &str, busy_wav: &'static [u8]) -> ! {
    log::info!("ap-bench: {target}, the cells flow's QSO-context AP rung, one core (predictions in the module doc)");
    let h = std::thread::Builder::new()
        .name("ap_bench".into())
        .stack_size(48 * 1024)
        .spawn(move || body(busy_wav))
        .expect("ap-bench: could not spawn the bench thread");
    let _ = h.join();
    log::info!("ap-bench: end");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// xorshift64 and an Irwin-Hall Gaussian (the sum of twelve uniforms less six: unit variance, no `ln` / `cos`: the LX7 has no double-precision FPU and a Box-Muller sample costs about as
/// much as a candidate's BP pass, 180 000 of them a slot).
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn gauss(&mut self) -> f32 {
        let mut acc = 0f32;
        for _ in 0..6 {
            let x = self.next();
            acc += (x & 0xFFFF_FFFF) as f32 * (1.0 / 4_294_967_296.0) + (x >> 32) as f32 * (1.0 / 4_294_967_296.0);
        }
        acc - 6.0
    }
}

/// An AWGN slot with `clean` (a reply; `None`: noise only) at `snr_db` in 2 500 Hz, 0.5 s in. With a `background` (a recording) there is no noise added and the reply goes in at the amplitude
/// `600 x 10^((snr_db + 20) / 20)`: **a nominal level** (the recording's own noise floor is not known here), not a calibrated SNR; the busy rows say so.
fn slot(clean: Option<&[i16]>, snr_db: f32, seed: u64, background: Option<&[i16]>) -> Vec<i16> {
    let amp = 6_000.0f32;
    let mut audio = vec![0f32; 180_000];
    if let Some(bg) = background {
        for (a, &b) in audio.iter_mut().zip(bg.iter()) {
            *a = f32::from(b);
        }
        if let Some(c) = clean {
            let gain = 600.0 / 6_000.0 * 10f32.powf((snr_db + 20.0) / 20.0);
            for (i, &v) in c.iter().enumerate() {
                audio[6_000 + i] += f32::from(v) * gain;
            }
        }
    } else {
        if let Some(c) = clean {
            for (i, &v) in c.iter().enumerate() {
                audio[6_000 + i] = f32::from(v);
            }
        }
        let sigma = ((amp * amp / 2.0) / (10f32.powf(snr_db / 10.0) * 2500.0 / 6000.0)).sqrt();
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
        for (i, v) in audio.iter_mut().enumerate() {
            *v += sigma * rng.gauss();
            if i % 20_000 == 0 {
                // let the idle task feed the watchdog
                unsafe { esp_idf_svc::sys::vTaskDelay(1) };
            }
        }
    }
    audio.iter().map(|v| v.round().clamp(-32768.0, 32767.0) as i16).collect()
}

fn reply(field: &str) -> Vec<i16> {
    let bits = pack77(MYCALL, PARTNER, field).expect("pack");
    synthesize_i16::<Ft8>(&message_to_tones::<Ft8>(&bits), 12_000, F0, 6_000)
}

fn texts(found: &[Found]) -> Vec<String> {
    found.iter().filter_map(|f| unpack77(f.res.message77())).collect()
}

fn body(busy_wav: &'static [u8]) {
    log::info!("ap-bench: boot min free internal {} B, PSRAM {} B; stack free {} B", min_free(INTERNAL), min_free(PSRAM), stack_free());
    let rate = Ft8Rate::Hz8000;
    let opts = CellsOptions::p0_window();
    let depth: DecodeDepth = opts.depth();
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let tables = CellTables::new();
    // SAFETY: this thread is the only user of the main staging buffer here.
    let cs = unsafe { crate::internal_pool::cs_scratch_main() };
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(cs, &mut bp, &tables).with_options(&opts);
    worker.reserve_osd_work();
    let stack_before = stack_free();

    let mut ok = true;
    // (label, field of the reply, the stage that expects it)
    let scenarios = [("Rogers RR73", "RR73", ApStage::Rogers), ("Replying -15", "-15", ApStage::Replying)];

    // one slot: search once, the round without and with the context, then the partner's candidates on their own (timed). Returns (hit without, hit with, hit by partner).
    let one = |worker: &mut CellsWorker<'_>, audio12: &[i16], msg: &str, stage: ApStage, label: &str, snr: f32, seed: u64| -> (bool, bool, bool, i64) {
        let a8 = resample_to_8k(audio12, 12_000);
        let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).expect("8 kHz");
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0 + 1.0, 0.02);
        let _ = worker.take_bp_stats();
        // without the context
        worker.set_ap_context(None);
        let mut k0: Vec<Found> = Vec::new();
        let t0 = now_us();
        worker.decode_round_osd(&cells, &cands, opts.max_cand, depth, 0.25, &mut k0, 6);
        let us0 = now_us() - t0;
        let st0 = worker.take_bp_stats();
        // the stack's high-water mark after the round without the context (it only falls: the difference to the one below is what the rung's frames add)
        let stack0 = stack_free();
        // with the context: the round (the rung runs inside it on the candidates near the partner)
        worker.set_ap_context(ctx);
        let mut k1: Vec<Found> = Vec::new();
        let t1 = now_us();
        worker.decode_round_osd(&cells, &cands, opts.max_cand, depth, 0.25, &mut k1, 6);
        let us1 = now_us() - t1;
        let st1 = worker.take_bp_stats();
        // the partner's own candidates, after the round
        let t2 = now_us();
        worker.decode_partner(&cells, depth, 0.25, &mut k1);
        let us2 = now_us() - t2;
        let st2 = worker.take_bp_stats();
        let stack1 = stack_free();
        let (h0, h1) = (texts(&k0).iter().any(|m| m == msg), texts(&k1).iter().any(|m| m == msg));
        log::info!(
            "ap-bench: {label} {snr:.0} dB seed {seed}: without {} ({us0} us, {} BP calls / {} passes) | with: round {us1} us ({} calls / {} passes), partner {us2} us ({} calls / {} passes), reply {} | stack free {stack0} B without, {stack1} B after the context",
            if h0 { "HIT" } else { "-" },
            st0.calls,
            st0.passes,
            st1.calls,
            st1.passes,
            st2.calls,
            st2.passes,
            if h1 { "HIT" } else { "-" }
        );
        (h0, h1, h1, us1 + us2 - us0)
    };

    // 1. the hit table: AWGN, -20 and -22 dB
    let (mut hits_without, mut hits_with) = (0u32, 0u32);
    let mut extra_us: Vec<i64> = Vec::new();
    for (label, field, stage) in scenarios {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let clean = reply(field);
        for snr in [-20.0f32, -22.0] {
            let (mut w0, mut w1) = (0, 0);
            for seed in 1..=SEEDS {
                let (h0, h1, _, extra) = one(&mut worker, &slot(Some(&clean), snr, seed, None), &msg, stage, label, snr, seed);
                w0 += u32::from(h0);
                w1 += u32::from(h1);
                extra_us.push(extra);
            }
            log::info!("ap-bench: {label} {snr:.0} dB: hits of {SEEDS} without the context {w0}, with it {w1}");
            if snr == -20.0 {
                hits_without += w0;
                hits_with += w1;
            }
        }
    }
    // 2. the worst case: a reply too weak to decode, every pass fails on every partner candidate
    for (label, field, stage) in scenarios {
        let msg = format!("{MYCALL} {PARTNER} {field}");
        let clean = reply(field);
        for seed in 1..=3u64 {
            let _ = one(&mut worker, &slot(Some(&clean), -27.0, seed, None), &msg, stage, &format!("worst {label}"), -27.0, seed);
        }
    }
    // 3. a busy background (the recording; the reply added at a level the host's readings put near -20 dB: a row of the busy band, not a calibrated crossing)
    {
        let busy: Vec<i16> = busy_wav[44..]
            .chunks_exact(2)
            .take(180_000)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        let clean = reply("RR73");
        let msg = format!("{MYCALL} {PARTNER} RR73");
        for seed in 1..=2u64 {
            let _ = one(&mut worker, &slot(Some(&clean), -20.0, seed, Some(&busy)), &msg, ApStage::Rogers, "busy Rogers RR73 (nominal level)", -20.0, seed);
        }
    }
    // 4. noise only, the context set: no message with the locked calls
    let mut phantoms = 0u32;
    for stage in [ApStage::Rogers, ApStage::Replying] {
        let ctx = ApContext::new(MYCALL, PARTNER, stage, F0, 0.0);
        for seed in 1..=4u64 {
            let audio = slot(None, -20.0, 70_000 + seed, None);
            let a8 = resample_to_8k(&audio, 12_000);
            let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).expect("8 kHz");
            worker.set_ap_context(ctx);
            let mut known: Vec<Found> = Vec::new();
            worker.decode_round_osd(&cells, &cands, opts.max_cand, depth, 0.25, &mut known, 6);
            worker.decode_partner(&cells, depth, 0.25, &mut known);
            for t in texts(&known) {
                if t.contains(MYCALL) || t.contains(PARTNER) {
                    phantoms += 1;
                    log::warn!("ap-bench: PHANTOM {stage:?} seed {seed}: {t}");
                }
            }
        }
    }
    worker.set_ap_context(None);

    extra_us.sort_unstable();
    let med = extra_us.get(extra_us.len() / 2).copied().unwrap_or(0);
    log::info!(
        "ap-bench: the context adds (round + partner - round without) median {med} us over {} slots (max {} us)",
        extra_us.len(),
        extra_us.last().copied().unwrap_or(0)
    );
    log::info!(
        "ap-bench: stack free {} B now, {} B before the first run; min free internal {} B, PSRAM {} B",
        stack_free(),
        stack_before,
        min_free(INTERNAL),
        min_free(PSRAM)
    );
    let check_hits = hits_with > hits_without;
    let check_noise = phantoms == 0;
    log::info!("ap-bench: check hits at -20 dB {hits_with} with against {hits_without} without: {}; noise phantoms {phantoms}: {}", if check_hits { "ok" } else { "FAIL" }, if check_noise { "ok" } else { "FAIL" });
    ok &= check_hits && check_noise;
    log::info!("ap-bench: done {}", if ok { "ok" } else { "FAIL" });
}
