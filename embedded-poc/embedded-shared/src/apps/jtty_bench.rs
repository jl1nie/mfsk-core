//! JTTY on the ESP32-S3 (LX7): what a receive window costs — issue #499, E0.
//!
//! `docs/notes/JTTY_UPSTREAM.md` and #499 estimate, from esp-dsp rates and the
//! repo's scalar-f32 experience, that the host receiver's per-window work does not
//! fit the 0.472 s window budget: 237 columns of one 8192-point complex FFT (the
//! sync surface) come to about 0.5 s alone, and the ladder to 0.15–0.3 s more.
//! Those are estimates. This bench replaces them with numbers, in four parts:
//!
//! 1. **The FFT**, `default_planner()` (the esp-dsp backend), sizes 256 … 8192, its
//!    buffer in internal DRAM and in PSRAM. `ft4_bench` measured 1.12× for moving a
//!    hot buffer into internal DRAM where 5–10× had been projected, so the PSRAM
//!    penalty is measured, not assumed.
//! 2. **One sync-surface column**, as `Receiver::sync_surface` does it: multiply the
//!    13-symbol sync waveform (2496 samples) into the window, zero-pad to 8192,
//!    FFT, power over the band, 1-2-3-2-1 smoothing — times 237 for a surface. Then
//!    the alternative #499 proposes: the same after decimating the 6 kHz analytic
//!    signal by 16 (156-sample waveform, 256-point FFT).
//! 3. **The ladder**, `Ladder::decode`, on synthetic correlations of a real frame
//!    at several SNRs and on noise alone (the case that costs most: all four rungs
//!    run and none accepts), and what `Ladder::new` costs in time and memory.
//! 4. **Memory**: internal and PSRAM free before and after each thing built.
//!
//! No radio, no baked asset; synthetic input only. Build with
//! `cargo build --release --bin jtty-bench --features jtty` in
//! `m5stack-cores3-app`.

use mfsk_core::engine::fft::default_planner;
use mfsk_core::jtty::ladder::Ladder;
use mfsk_core::jtty::pack::{self, ExchangeProfile};
use mfsk_core::jtty::trellis::Correlations;
use mfsk_core::jtty::{INFO_BITS, tx};
use num_complex::Complex32;
#[allow(unused_imports)]
use num_traits::Float;

const MALLOC_CAP_INTERNAL: u32 = esp_idf_svc::sys::MALLOC_CAP_INTERNAL;
const MALLOC_CAP_SPIRAM: u32 = esp_idf_svc::sys::MALLOC_CAP_SPIRAM;
const MALLOC_CAP_8BIT: u32 = esp_idf_svc::sys::MALLOC_CAP_8BIT;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Install the ESP-IDF logger, at most once per boot — the same guard the other
/// benches use.
pub fn init_logger_once() {
    static LOGGER_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if LOGGER_READY
        .compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        )
        .is_ok()
    {
        esp_idf_svc::log::EspLogger::initialize_default();
    }
}

fn log_heap(tag: &str) {
    unsafe {
        log::info!(
            "[mem] {tag}: internal {} KB (largest contig {} KB), PSRAM {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
                / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
        );
    }
}

/// Where a buffer lives.
#[derive(Clone, Copy)]
enum Place {
    Internal,
    Psram,
}

impl Place {
    fn name(self) -> &'static str {
        match self {
            Place::Internal => "internal",
            Place::Psram => "PSRAM",
        }
    }
    fn caps(self) -> u32 {
        match self {
            Place::Internal => MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
            Place::Psram => MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        }
    }
}

/// A heap block of `Complex32`, 16-byte aligned (the PIE FFT path needs it), in one
/// place; freed on drop, so one bench's buffers do not starve the next.
struct Buf {
    p: *mut Complex32,
    len: usize,
}

impl Buf {
    #[inline(never)]
    fn new(len: usize, place: Place) -> Option<Self> {
        let bytes = len * core::mem::size_of::<Complex32>();
        // SAFETY: null or a 16-byte-aligned block of `bytes`.
        let p = unsafe { esp_idf_svc::sys::heap_caps_aligned_alloc(16, bytes, place.caps()) }
            as *mut Complex32;
        (!p.is_null()).then_some(Self { p, len })
    }
}

impl core::ops::Deref for Buf {
    type Target = [Complex32];
    fn deref(&self) -> &[Complex32] {
        // SAFETY: `p` is a live block of `len` Complex32 (repr(C) over two f32).
        unsafe { core::slice::from_raw_parts(self.p, self.len) }
    }
}

impl core::ops::DerefMut for Buf {
    fn deref_mut(&mut self) -> &mut [Complex32] {
        // SAFETY: as above, and `&mut self` is unique.
        unsafe { core::slice::from_raw_parts_mut(self.p, self.len) }
    }
}

impl Drop for Buf {
    fn drop(&mut self) {
        // SAFETY: allocated by `heap_caps_aligned_alloc`, freed once.
        unsafe { esp_idf_svc::sys::heap_caps_free(self.p as *mut core::ffi::c_void) }
    }
}

/// Let the idle task run: a multi-second compute loop on core 0 otherwise trips the
/// task watchdog. Outside every timed region.
fn yield_now() {
    unsafe { esp_idf_svc::sys::vTaskDelay(1) };
}

/// A small deterministic generator, so every run sees the same numbers.
struct Lcg(u64);
impl Lcg {
    fn uniform(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }
    fn gauss(&mut self) -> f32 {
        let (a, b) = (self.uniform().max(1e-7), self.uniform());
        (-2.0 * a.ln()).sqrt() * (core::f32::consts::TAU * b).cos()
    }
}

fn fill_noise(buf: &mut [Complex32], rng: &mut Lcg) {
    for z in buf.iter_mut() {
        *z = Complex32::new(rng.gauss(), rng.gauss());
    }
}

/// Part 1: one forward FFT of `n` points, in `place`, timed over `reps` runs of
/// `process` alone (the buffer is refilled outside the timing: repeated transforms of
/// the same data would grow without bound).
#[inline(never)]
fn bench_fft(n: usize, place: Place, reps: u32) {
    let Some(mut buf) = Buf::new(n, place) else {
        log::warn!("fft {n:>5}  {:<8}: cannot allocate {} KB", place.name(), n * 8 / 1024);
        return;
    };
    let mut planner = default_planner();
    let t0 = now_us();
    let fft = planner.plan_forward(n);
    let t_plan = now_us() - t0;
    let mut rng = Lcg(n as u64);
    let mut total = 0i64;
    for _ in 0..reps {
        fill_noise(&mut buf, &mut rng);
        let t = now_us();
        fft.process(&mut buf);
        total += now_us() - t;
        yield_now();
    }
    let per = total as f64 / f64::from(reps);
    log::info!(
        "fft {n:>5}  {:<8}: {per:>9.1} us per transform ({reps} runs), plan {t_plan} us",
        place.name()
    );
    core::hint::black_box(&buf[0]);
}

/// Part 2: one column of the sync surface, all the steps `Receiver::sync_surface`
/// does, with the window, the sync waveform and the FFT buffer in `place`.
/// `wave_len` samples of sync waveform, `nfft`-point FFT, `band` bins of power kept
/// (plus 4 for the smoothing).
// `inline(never)` and `black_box` on the sizes: with the sizes folded in, the Xtensa LLVM
// backend fails to select an `i32` constant here (`Cannot select: i32 =
// Constant<23040>`, 2880 * 8 bytes), the same class of codegen fault the crate's release
// profile comment records for the f32-select case.
#[inline(never)]
fn bench_column(label: &str, wave_len: usize, nfft: usize, band: usize, place: Place, reps: u32) {
    let (Some(mut buf), Some(mut sync), Some(mut win)) = (
        Buf::new(nfft, place),
        Buf::new(wave_len, place),
        Buf::new(core::hint::black_box(wave_len) + 240 * 12, place),
    ) else {
        log::warn!("column [{label}] {}: cannot allocate", place.name());
        return;
    };
    let mut rng = Lcg(wave_len as u64);
    fill_noise(&mut sync, &mut rng);
    fill_noise(&mut win, &mut rng);
    let mut planner = default_planner();
    let fft = planner.plan_forward(nfft);
    let mut power = alloc::vec![0f32; band + 4];
    let mut smoothed = alloc::vec![0f32; band];
    let mut total = 0i64;
    let mut sink = 0f32;
    for r in 0..reps {
        let i0 = (r as usize % 200) * 12;
        let t = now_us();
        for ((b, s), x) in buf.iter_mut().zip(sync.iter()).zip(&win[i0..]) {
            *b = s.conj() * *x;
        }
        buf[wave_len..].fill(Complex32::new(0.0, 0.0));
        fft.process(&mut buf);
        for (p, z) in power.iter_mut().zip(&buf[10..10 + band + 4]) {
            *p = z.norm_sqr();
        }
        for (o, w) in smoothed.iter_mut().zip(power.windows(5)) {
            *o = w[0] + 2.0 * w[1] + 3.0 * w[2] + 2.0 * w[3] + w[4];
        }
        total += now_us() - t;
        sink += smoothed[0];
        yield_now();
    }
    core::hint::black_box(sink);
    let per = total as f64 / f64::from(reps);
    log::info!(
        "column [{label}] {:<8}: {per:>8.1} us per column, x237 = {:>7.1} ms per surface",
        place.name(),
        per * 237.0 / 1000.0
    );
}

/// Synthetic correlations for a real frame (the oracle's model at zero drift): every
/// symbol is the sum of two half-symbol correlations, the true tone `A/2 + noise`
/// in each, the other three tones noise alone, `A = 10^(snr/20)`; `zhalf` is
/// `sqrt(|h1|² + |h2|²)`. `snr_db = None` is noise alone.
fn synth_correlations(tones: &[u8], snr_db: Option<f32>, rng: &mut Lcg) -> (Correlations, Correlations) {
    let amp = snr_db.map_or(0.0, |s| 10f32.powf(s / 20.0));
    let mut zsym = [[Complex32::new(0.0, 0.0); 4]; INFO_BITS];
    let mut zhalf = zsym;
    for k in 0..INFO_BITS {
        for t in 0..4 {
            let signal = if snr_db.is_some() && t == usize::from(tones[k]) {
                amp / 2.0
            } else {
                0.0
            };
            let h1 = Complex32::new(signal + 0.5 * rng.gauss(), 0.5 * rng.gauss());
            let h2 = Complex32::new(signal + 0.5 * rng.gauss(), 0.5 * rng.gauss());
            zsym[k][t] = h1 + h2;
            zhalf[k][t] = Complex32::new((h1.norm_sqr() + h2.norm_sqr()).sqrt(), 0.0);
        }
    }
    (zsym, zhalf)
}

/// Part 3: the ladder with its trellis metrics in `f64` (as upstream) and in `f32`
/// (`Ladder::with_f32_metrics`, #499 E1b) on the same synthetic frames.
fn bench_ladder() {
    log_heap("before Ladder::new");
    let t = now_us();
    let ladder = Ladder::new();
    log::info!("Ladder::new: {} us", now_us() - t);
    log_heap("after Ladder::new");
    bench_ladder_with("f64", &ladder);
    let t = now_us();
    let ladder32 = Ladder::new().with_f32_metrics();
    log::info!("Ladder::new (f32 metrics): {} us", now_us() - t);
    bench_ladder_with("f32", &ladder32);
}

fn bench_ladder_with(name: &str, ladder: &Ladder) {
    log::info!("--- ladder, {name} metrics ---");

    // one real frame: "CQ K1ABC CQ" through CRC-12 and the tail-biting code
    let atoms = pack::pack("CQ K1ABC CQ", ExchangeProfile::Unknown).expect("packs");
    let payloads = tx::payloads(&atoms).expect("encodes");
    let data = tx::frame_tones(&payloads[0]);
    let tones = &data[mfsk_core::jtty::SYNC_SYMBOLS..];

    let mut rng = Lcg(0x477);
    for (label, snr) in [
        ("+12 dB", Some(12.0)),
        ("+6 dB", Some(6.0)),
        ("+3 dB", Some(3.0)),
        ("0 dB", Some(0.0)),
        ("-3 dB", Some(-3.0)),
        ("noise only", None),
    ] {
        const RUNS: u32 = 4;
        // the same frames for both precisions: reseed per condition
        rng = Lcg(0x477 ^ (snr.map_or(0, |s| s as i64 as u64 + 100)));
        let mut total = 0i64;
        let mut worst = 0i64;
        let mut outcome = alloc::string::String::new();
        for _ in 0..RUNS {
            let (zsym, zhalf) = synth_correlations(tones, snr, &mut rng);
            let t = now_us();
            let r = ladder.decode(&zsym, &zhalf);
            let dt = now_us() - t;
            yield_now();
            total += dt;
            worst = worst.max(dt);
            outcome.push_str(&match r {
                Some(a) => alloc::format!(" rung{}{}", a.rung, if a.payload == payloads[0] { "" } else { "!" }),
                None => alloc::string::String::from(" none"),
            });
        }
        log::info!(
            "ladder[{name}] {label:<10}: {:>8.1} ms mean, {:>8.1} ms worst over {RUNS}  [{}]",
            total as f64 / f64::from(RUNS) / 1000.0,
            worst as f64 / 1000.0,
            outcome.trim_start()
        );
    }
}

/// Part 3b: does *where the trellis lives* matter? `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` is 2048,
/// so the two 32 KB survivor arrays and the larger plan tables of a decode land in PSRAM, past
/// the 32 KB data cache — the 14x of the 8192-point FFT. `heap_caps_malloc_extmem_enable`
/// moves that threshold at run time; the L=1 plan and its working set (about 160 KB) then fit
/// internal DRAM, and the same decode is timed in both places, `f64` and `f32` metrics.
fn bench_trellis_placement() {
    use mfsk_core::jtty::trellis::Plan;
    let atoms = pack::pack("CQ K1ABC CQ", ExchangeProfile::Unknown).expect("packs");
    let payloads = tx::payloads(&atoms).expect("encodes");
    let data = tx::frame_tones(&payloads[0]);
    let tones = &data[mfsk_core::jtty::SYNC_SYMBOLS..];
    for (place, limit) in [("default (> 2 KB in PSRAM)", 2048usize), ("threshold 256 KB (internal)", 256 * 1024)] {
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(limit) };
        log::info!("--- trellis, L=1 plan, allocations up to {limit} B prefer internal DRAM: {place} ---");
        log_heap("before Plan::new(1)");
        let plan = Plan::new(1);
        log_heap("after Plan::new(1)");
        for (label, snr) in [("+12 dB", Some(12.0)), ("noise only", None)] {
            let mut rng = Lcg(0x1177 ^ snr.map_or(0, |s| s as i64 as u64 + 100));
            let (zsym, _) = synth_correlations(tones, snr, &mut rng);
            for (name, f32_metrics) in [("f64", false), ("f32", true)] {
                const RUNS: u32 = 3;
                let mut total = 0i64;
                for _ in 0..RUNS {
                    let t = now_us();
                    let r = if f32_metrics {
                        plan.decode_f32(&zsym, true)
                    } else {
                        plan.decode(&zsym, true)
                    };
                    total += now_us() - t;
                    core::hint::black_box(r.pool);
                    yield_now();
                }
                log::info!(
                    "trellis L1 [{name}] {label:<10}: {:>8.1} ms per decode ({place})",
                    total as f64 / f64::from(RUNS) / 1000.0
                );
            }
        }
        drop(plan);
    }
    unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(2048) };
}

/// Part 4: the DSP around the trellis, each at the size the receiver runs it: the unit costs
/// the per-window model (`docs/notes/JTTY_EMBEDDED_BUDGET.md`) is built from. Buffers in PSRAM
/// (a 14 160-sample window is 113 KB, so that is where the receiver has it) and, for the
/// small ones, in internal DRAM.
#[inline(never)]
fn bench_dsp() {
    use mfsk_core::jtty::correlate::{ToneRefs, correlate_payload};
    use mfsk_core::jtty::dsp::{shift_frequency, sync_wave};
    use mfsk_core::jtty::rx::{NCHUNK, peakup};
    use mfsk_core::jtty::subtract::subtract_frame;

    const WIN: usize = NCHUNK / 2; // 14 160 complex samples at 6 kHz
    let mut rng = Lcg(0xD59);
    let (Some(mut src), Some(mut dst)) = (Buf::new(WIN, Place::Psram), Buf::new(WIN, Place::Psram)) else {
        log::warn!("dsp: cannot allocate the window");
        return;
    };
    fill_noise(&mut src, &mut rng);
    let csync = sync_wave();
    let refs = ToneRefs::new(192);

    // a whole-window mix to DC, as `process` does for every candidate (plus its allocation)
    let mut total = 0i64;
    for _ in 0..5 {
        let t = now_us();
        shift_frequency(&src, &mut dst, 6000.0, -1500.3);
        total += now_us() - t;
        core::hint::black_box(&dst[0]);
        yield_now();
    }
    log::info!("dsp: shift_frequency, {WIN} samples, PSRAM to PSRAM: {:.2} ms", total as f64 / 5000.0);
    if let (Some(mut a), Some(mut b)) = (Buf::new(WIN, Place::Internal), Buf::new(WIN, Place::Internal)) {
        a.copy_from_slice(&src);
        let mut total = 0i64;
        for _ in 0..5 {
            let t = now_us();
            shift_frequency(&a, &mut b, 6000.0, -1500.3);
            total += now_us() - t;
            core::hint::black_box(&b[0]);
            yield_now();
        }
        log::info!("dsp: shift_frequency, {WIN} samples, internal to internal: {:.2} ms", total as f64 / 5000.0);
    }

    // peak-up: 11 frequency steps, 13 hops, over the sync waveform
    let mut total = 0i64;
    let mut sink = 0f32;
    for i in 0..3 {
        let t = now_us();
        let (x, f, s) = peakup(&src, &csync, 0.9 + 0.001 * i as f32, 1500.0);
        total += now_us() - t;
        sink += x + f + s;
        yield_now();
    }
    core::hint::black_box(sink);
    log::info!("dsp: peakup: {:.2} ms", total as f64 / 3000.0);

    // the candidate path since #499: four rotated references (768 samples) and the 13-symbol gate
    // on the unshifted window, in place of the whole-window mix above
    let mut total = 0i64;
    let mut sink = 0f32;
    for _ in 0..10 {
        let t = now_us();
        let rot = refs.rotated(-1500.3, 6000.0);
        for j in 0..13 {
            for k in 0..4 {
                sink += rot.power(k, &src[4000 + j * 192..4000 + (j + 1) * 192]);
            }
        }
        total += now_us() - t;
        yield_now();
    }
    core::hint::black_box(sink);
    log::info!("dsp: rotated references + 13-symbol gate: {:.2} ms", total as f64 / 10000.0);
    let rot = refs.rotated(-1500.3, 6000.0);
    let mut total = 0i64;
    for _ in 0..5 {
        let t = now_us();
        let (zs, zh) = rot.correlate_payload(&src, 4000);
        total += now_us() - t;
        core::hint::black_box((zs[0][0], zh[0][0]));
        yield_now();
    }
    log::info!("dsp: rotated correlate_payload: {:.2} ms", total as f64 / 5000.0);

    // payload correlation: 46 symbols x 4 tones x (full + two halves) x 192 samples
    let mut total = 0i64;
    for _ in 0..5 {
        let t = now_us();
        let (zs, zh) = correlate_payload(&refs, &src, 4000);
        total += now_us() - t;
        core::hint::black_box((zs[0][0], zh[0][0]));
        yield_now();
    }
    log::info!("dsp: correlate_payload: {:.2} ms", total as f64 / 5000.0);

    // subtracting one decoded frame: 11 328 samples, `f64` prefix sums and `sin`/`cos` per sample
    let atoms = pack::pack("CQ K1ABC CQ", ExchangeProfile::Unknown).expect("packs");
    let payloads = tx::payloads(&atoms).expect("encodes");
    let tones = tx::frame_tones(&payloads[0]);
    let mut total = 0i64;
    for _ in 0..3 {
        let t = now_us();
        subtract_frame(&mut src, &tones, 1500.0, 0.5);
        total += now_us() - t;
        yield_now();
    }
    log::info!("dsp: subtract_frame: {:.2} ms", total as f64 / 3000.0);
}

fn stack_headroom() -> u32 {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

/// Part 5: what a window costs after its analytic signal, with the default search and with
/// `Params::embedded()` (channel 0 only, decimated 512-point surface, raw-then-refined
/// candidates), on noise alone and with one frame in it. The analytic stage is not in these
/// numbers: its 32 768-point transform does not run here.
fn bench_search() {
    use mfsk_core::jtty::rx::{NCHUNK, Params, Receiver};
    use mfsk_core::jtty::tx::Synth;

    const WIN: usize = NCHUNK / 2;
    let rx = Receiver::new().with_f32_metrics();
    log_heap("search: receiver built");
    let mut rng = Lcg(0x5EA);
    let atoms = pack::pack("CQ K1ABC CQ", ExchangeProfile::Unknown).expect("packs");
    let payloads = tx::payloads(&atoms).expect("encodes");
    let tones = tx::frame_tones(&payloads[0]);
    let mut noise = alloc::vec![Complex32::new(0.0, 0.0); WIN];
    fill_noise(&mut noise, &mut rng);
    let mut frame = alloc::vec![Complex32::new(0.0, 0.0); WIN];
    // 0.35 against unit-variance noise per component: about 12 dB in the symbol bandwidth
    let mut synth = Synth::<f32>::at(&tones, 1500.0, 0.35, 192, 6000.0);
    let start = 1800; // 0.3 s
    let n = synth.fill_complex(&mut frame[start..(start + 59 * 192).min(WIN)]);
    log::info!("search: frame of {n} samples at 1500 Hz, start {start}");
    for (a, b) in frame.iter_mut().zip(&noise) {
        *a += *b;
    }

    let default = Params::default();
    let embedded = Params::default().embedded();
    let embedded_nosub = Params { subtract: false, ..embedded };
    let variants: [(&str, &Params); 3] = [
        ("default          ", &default),
        ("embedded         ", &embedded),
        ("embedded, no sub ", &embedded_nosub),
    ];
    for (name, p) in variants {
        let reps = if p.ch0_only { 3 } else { 1 };
        let mut total = 0i64;
        let mut sink = 0f32;
        for _ in 0..reps {
            let t = now_us();
            sink += rx.bench_surface(&noise, p);
            total += now_us() - t;
            yield_now();
        }
        core::hint::black_box(sink);
        log::info!("search: {name} sync surface: {:.1} ms", total as f64 / (1000.0 * reps as f64));
    }
    log_heap("search: after surfaces");
    for (label, win) in [("noise ", &noise), ("frame ", &frame)] {
        for (name, p) in variants {
            // the default search takes 20-45 s a window here: once, and not with a frame in it
            let slow = !p.ch0_only;
            if slow && label == "frame " {
                continue;
            }
            let mut times = [0f64; 3];
            let mut found = 0;
            for t in times.iter_mut().take(if slow { 1 } else { 3 }) {
                let c0 = win.to_vec();
                let t0 = now_us();
                found = rx.bench_window(c0, p);
                *t = (now_us() - t0) as f64 / 1000.0;
                yield_now();
            }
            log::info!(
                "search: {label}window, {name} {:.0} / {:.0} / {:.0} ms (0 = not run), {found} frame(s)",
                times[0],
                times[1],
                times[2]
            );
        }
        log_heap("search: after windows");
    }
}

fn run_bench() {
    log::info!("=== jtty-bench: what a JTTY receive window costs on the LX7 (#499, E0) ===");
    log::info!("window budget: 472 ms (a quarter frame); host: 11.7 ms per window, one thread");
    log_heap("start");

    // The esp-dsp twiddle tables: 8192 is `CONFIG_DSP_MAX_FFT_SIZE`'s ceiling here.
    crate::esp_dsp_fft::prewarm(8192);

    log::info!("--- 1. FFT, complex f32, esp-dsp via default_planner ---");
    for n in [256usize, 512, 1024, 2048, 4096, 8192] {
        let reps = if n >= 4096 { 10 } else { 50 };
        for place in [Place::Internal, Place::Psram] {
            bench_fft(n, place, reps);
        }
    }
    log_heap("after FFT");

    log::info!("--- 2. one sync-surface column (multiply, zero-pad, FFT, power, smoothing) ---");
    log::info!("reference: 2496-sample sync waveform, 8192-point FFT, 141-bin band (channel 0, +-50 Hz)");
    for place in [Place::Internal, Place::Psram] {
        bench_column("8192-pt, ch0 band", 2496, 8192, 141, place, 40);
    }
    log::info!("channel 1/2 are +-150 Hz: 411 bins");
    bench_column("8192-pt, ch1/2 band", 2496, 8192, 411, Place::Psram, 40);
    log::info!("alternative (#499): decimate by 16 -> 375 Hz, 156-sample waveform, 256-point FFT");
    for place in [Place::Internal, Place::Psram] {
        // +-50 Hz at 1.46 Hz a bin is 68 bins; +-150 Hz is 205
        bench_column("256-pt, ch0 band", 156, 256, 68, place, 400);
        bench_column("256-pt, ch1/2 band", 156, 256, 205, place, 400);
    }
    log_heap("after columns");

    log::info!("--- 4. the DSP around the trellis ---");
    bench_dsp();
    log::info!("--- 5. a window after its analytic signal ---");
    bench_search();
    log::info!("--- 3. the ladder ---");
    bench_ladder();
    log::info!("--- 3b. trellis placement ---");
    bench_trellis_placement();
    log_heap("end");
    log::info!("stack headroom: {} B", stack_headroom());
    log::info!("=== jtty-bench done ===");
}

extern "C" fn bench_task(_arg: *mut core::ffi::c_void) {
    run_bench();
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// Stack of the bench task: the ladder's plans and the trellis scratch live on the
/// heap, but the correlations (2 x 46 x 4 complex) and the formatting are on the stack.
pub const BENCH_STACK: u32 = 32 * 1024;

/// Spawn [`bench_task`] pinned to core 0, then idle forever.
pub fn run() -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"jtty_bench".as_ptr(),
            BENCH_STACK,
            core::ptr::null_mut(),
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create jtty_bench task ({BENCH_STACK} B stack)");
    }
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
