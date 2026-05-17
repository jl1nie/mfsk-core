//! Phase 1.6-Stick: FT8 TX synthesis via ES8311 DAC + acoustic VOX.
//!
//! Pipeline:
//!   text (e.g. "CQ JL1NIE PM95")
//!   → `pack77` (msg/wsjt77.rs — 77-bit FT8 message field)
//!   → `message_to_tones` (LDPC encode + Gray + 7 sync tones = 79 symbols)
//!   → `tones_to_i16_into` (GFSK shaping, 12 kHz mono i16, 151_680 samples)
//!   → 4× zero-order-hold upsample → 48 kHz stereo i16 chunks
//!   → I2S TX → ES8311 DAC → built-in speaker
//!
//! TX bracket via CI-V:
//!   1. `civ::set_ptt(true)`  — radio key TX
//!   2. wait ~100 ms (radio settle)
//!   3. play 151_680 × 12.5 / 12_000 ≈ 12.64 s of tones
//!   4. `civ::set_ptt(false)` — radio drop TX
//!
//! Hardware note: M5StickS3 has no UAC OUT (silicon-blocked, see
//! memory `project_m5stick_s3_no_usb_host`), so audio crosses the
//! room via the internal speaker → IC-705 microphone. Distance and
//! orientation matter (~10 cm with mic-facing speaker is fine for
//! FT8's -21 dB SNR floor). Position the Stick near the radio
//! before triggering TX; ambient noise is fine.
//!
//! Phase 1.6 is verified standalone via `BootMode::TxTest` (BLE +
//! TX only, no decode pipeline). Phase 1.7 hooks this into the QSO
//! FSM so auto-CQ / report exchange actually transmits.

use anyhow::{anyhow, Result};
use esp_idf_hal::{
    delay::TickType,
    i2s::{I2sDriver, I2sTx},
};
use mfsk_core::ft8::message::pack77;
use mfsk_core::ft8::params::{NN, NSPS};
use mfsk_core::ft8::wave_gen::tones_to_i16_into;

/// FT8 12 kHz mono sample rate (canonical).
const TX_SAMPLE_RATE_HZ: u32 = 12_000;
/// 79 symbols × 1920 samples/symbol = full 12.6 s FT8 frame.
const TX_SAMPLES_12K: usize = NN * NSPS;
/// Default TX-audio amplitude into the ES8311 DAC. 20000/32767 ≈
/// -4 dBFS — leaves headroom for the GFSK envelope while keeping the
/// acoustic level above ambient. Tune up if IC-705 ALC isn't getting
/// enough drive; down if it clips.
const TX_AMPLITUDE: i16 = 20_000;

/// Synthesise an FT8 frame for a CQ message.
/// Returned Vec is 12 kHz mono i16, 151_680 samples (~300 KB — sits
/// in PSRAM via the global allocator, fine on S3 with 8 MB).
pub fn synthesize_cq(callsign: &str, grid: &str, f0_hz: f32) -> Result<Vec<i16>> {
    let msg77 = pack77("CQ", callsign, grid)
        .ok_or_else(|| anyhow!("pack77 failed for CQ {callsign} {grid}"))?;
    synthesize_message77(&msg77, f0_hz)
}

/// Synthesise an arbitrary 77-bit message at base frequency `f0_hz`.
/// Used by the QSO FSM (Phase 1.7) once it produces non-CQ messages.
pub fn synthesize_message77(msg77: &[u8; 77], f0_hz: f32) -> Result<Vec<i16>> {
    let tones = mfsk_core::ft8::wave_gen::message_to_tones(msg77);
    let mut samples = vec![0i16; TX_SAMPLES_12K];
    tones_to_i16_into(&mut samples, &tones, f0_hz, TX_AMPLITUDE);
    Ok(samples)
}

/// Stream a 12 kHz mono sample buffer to I2S TX as 48 kHz stereo
/// (4× zero-order-hold L=R). Blocks until playback completes;
/// ~12.64 s for a full FT8 frame.
///
/// Chunk size = 20 ms of input (240 samples 12k mono) = 80 ms of
/// output (3840 samples 48k stereo = 15 360 bytes) — matches the
/// existing playback path in `audio::audio_thread` for buffer sizing
/// rationale (DMA underrun headroom vs ringbuf overhead).
pub fn play(i2s: &mut I2sDriver<'static, I2sTx>, samples_12k: &[i16]) -> Result<()> {
    const IN_CHUNK: usize = 240; // 20 ms @ 12 kHz
    // 4× ZOH × stereo (L=R) × i16 = 16 bytes per input sample
    const OUT_BYTES_PER_IN: usize = 16;
    let mut out = vec![0u8; IN_CHUNK * OUT_BYTES_PER_IN];
    for chunk in samples_12k.chunks(IN_CHUNK) {
        let mut o = 0;
        for &s in chunk {
            let b = s.to_le_bytes();
            // 4 stereo frames per input sample: L b[0..2], R b[0..2]
            for _ in 0..4 {
                out[o] = b[0];
                out[o + 1] = b[1];
                out[o + 2] = b[0];
                out[o + 3] = b[1];
                o += 4;
            }
        }
        let nbytes = chunk.len() * OUT_BYTES_PER_IN;
        i2s.write_all(&out[..nbytes], TickType::new_millis(500).ticks())
            .map_err(|e| anyhow!("I2S TX write failed: {e:?}"))?;
    }
    Ok(())
}

/// Bring-up test loop: every 30 s, do a full TX cycle of "CQ JL1NIE
/// PM95" at 1500 Hz with CI-V PTT bracketing. Verifies the full
/// chain (synth + I2S + speaker → IC-705 mic → radio TX → on-air).
///
/// To verify: have another FT8 RX (WSJT-X, JTDX, another mfsk-core
/// instance) tuned to the same band/freq; "CQ JL1NIE PM95" should
/// appear in their decoded list.
pub fn start_test_loop(i2s: I2sDriver<'static, I2sTx>) -> Result<()> {
    std::thread::Builder::new()
        .stack_size(8 * 1024)
        .name("tx_test".into())
        .spawn(move || tx_test_thread(i2s))
        .map_err(|e| anyhow!("tx_test thread spawn failed: {e}"))?;
    log::info!("tx: test loop spawned (CQ JL1NIE PM95 @ 1500 Hz, every 30 s)");
    Ok(())
}

fn tx_test_thread(mut i2s: I2sDriver<'static, I2sTx>) {
    use esp_idf_svc::hal::delay::FreeRtos;
    // Boost priority above stage1_inc / dual_core so the I2S DMA
    // refill doesn't get starved by other work — same rationale as
    // `audio::audio_thread`. Priority 8 = between dsp_worker (5) and
    // watchdog (idle 0).
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }
    if let Err(e) = i2s.tx_enable() {
        log::error!("tx: i2s tx_enable failed: {e:?}");
        return;
    }
    // Boot grace: BLE needs ~10 s to connect + pair to IC-705 before
    // the first PTT command can land. Without this, first TX cycle
    // keys the speaker but the radio is still in RX (no on-air).
    log::info!("tx: bring-up grace 10 s (waiting for BLE pair)");
    FreeRtos::delay_ms(10_000);

    loop {
        log::info!("tx: synthesizing CQ JL1NIE PM95 at 1500 Hz");
        let samples = match synthesize_cq("JL1NIE", "PM95", 1500.0) {
            Ok(s) => s,
            Err(e) => {
                log::error!("tx synth failed: {e:#}");
                FreeRtos::delay_ms(30_000);
                continue;
            }
        };

        log::info!("tx: PTT-on, settling 100 ms");
        crate::civ::set_ptt(true);
        FreeRtos::delay_ms(100);

        let t_start = std::time::Instant::now();
        log::info!("tx: playing {} samples (~12.6 s)", samples.len());
        if let Err(e) = play(&mut i2s, &samples) {
            log::error!("tx: play failed: {e:#}");
        }
        log::info!("tx: play complete in {:?}, PTT-off", t_start.elapsed());
        crate::civ::set_ptt(false);

        // 30 s between TX cycles — long enough that an FT8 slot
        // boundary (every 15 s) falls inside the window and a
        // potential listener has time to decode + log + react.
        FreeRtos::delay_ms(30_000);
    }
}
