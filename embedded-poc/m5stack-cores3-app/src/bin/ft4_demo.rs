//! FT4 receiver, WAV demo — the desk equivalent of a radio on 40 m.
//!
//! Replays the vendored WSJT-X golden at 12 kHz in real time and runs
//! the shipping FT4 slot pipeline over it: the coarse periodogram
//! accumulated **during** capture, then DDC / Δt search / decode after
//! the slot closes. The FT8 controller has had `wav_sim` for this since
//! the beginning; FT4 had nothing, which is why every FT4 number in
//! `docs/notes/FT4_BENCHMARK.md` up to §33 came from a bench feeding
//! whole buffers rather than a receiver.
//!
//! **This is a separate bin rather than a boot mode** so it can be
//! flashed and re-flashed freely: it brings up no USB host, so the
//! serial console stays attached. `apps::ft4::run` is the boot-mode
//! entry point that takes a radio.
//!
//! What the timing means: the feeder paces itself to 12 kHz, so
//! `slot` lines arrive 7.5 s apart and `decode` is real post-slot
//! latency against the 1 960 ms budget — the same quantity `ft4-bench`
//! reports, but measured end to end by a receiver.
//!
//! Build: `cargo build --release --features ft4 --bin ft4-demo`.

use embedded_shared::apps::ft4_rx as ft4;

/// The same slot `ft4-bench` uses: WSJT-X's `FT4/000000_000002.wav`,
/// baked to raw `i16` by `ft4_bake_golden_precomputed`. 19 signals, of
/// which 14 are inside the 100-2700 Hz search band and 11 decode in a
/// single pass at `DecodeDepth::EMBEDDED`.
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/ft4_golden_audio.bin");

/// One UAC read's worth of resampled audio — `uac::reader_thread`'s
/// `dst_scratch` is sized for ~256 samples at 48 k → 12 k. Feeding the
/// same size means the demo exercises the block cadence the radio path
/// will, rather than a friendlier one.
const BLOCK: usize = 256;

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== mfsk-core-m5stack-cores3-app ft4-demo ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop is compute-bound for hundreds of milliseconds
    // with no yield point; IDLE starves and the watchdog fires. Same
    // call and same reason as every bench and receiver here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("ft4-demo: task watchdog deinit -> {r}");

    // Before anything plans a transform — the coarse stage's 2304-point
    // workspace is 41 % faster in internal DRAM, and this is the only
    // moment the block is available. See `esp_dsp_fft`'s own docs.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft4-demo: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );

    let audio: Vec<i16> = GOLDEN_AUDIO
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    assert_eq!(
        audio.len(),
        ft4::SLOT_SAMPLES,
        "the baked golden must be exactly one FT4 slot"
    );
    log::info!(
        "ft4-demo: replaying {} samples ({}.{} s) in {BLOCK}-sample blocks, forever",
        audio.len(),
        audio.len() / 12_000,
        (audio.len() % 12_000) * 10 / 12_000,
    );

    let mut accum = ft4::SlotAccum::new();
    let mut slot_no: u32 = 0;
    // Absolute pacing: sleeping `BLOCK/12` ms per block would drift by
    // whatever each block's work cost. Anchor to the start instead so
    // the feed stays at 12 kHz however long a block takes.
    let t_start = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let mut fed: u64 = 0;

    loop {
        for chunk in audio.chunks(BLOCK) {
            if let Some(slot) = accum.push(chunk) {
                slot_no += 1;
                let (decodes, cands, us) = ft4::decode_slot(&slot);
                log::info!(
                    "ft4-demo: slot {slot_no} — {cands} candidates, {} decodes in {} ms \
                     (budget 1960 ms -> {})",
                    decodes.len(),
                    us / 1000,
                    if us / 1000 <= 1_960 { "FITS" } else { "OVER" },
                );
                for d in &decodes {
                    log::info!(
                        "    {:>6.1} Hz  {:>+5.2} s  {:>3.0} dB  {}",
                        d.freq_hz,
                        d.dt_sec,
                        d.snr_db,
                        d.msg,
                    );
                }
            }
            fed += chunk.len() as u64;

            // Sleep only if the pipeline is ahead of real time; if it
            // is behind, the next block goes straight out and the log
            // above will show the overrun rather than this hiding it.
            let due_us = (fed * 1_000_000 / 12_000) as i64;
            let now = unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_start;
            if due_us > now {
                unsafe {
                    esp_idf_svc::sys::vTaskDelay(
                        (((due_us - now) as u32) / 1_000).max(1)
                            / (1_000 / esp_idf_svc::sys::configTICK_RATE_HZ).max(1),
                    )
                };
            }
        }
    }
}
