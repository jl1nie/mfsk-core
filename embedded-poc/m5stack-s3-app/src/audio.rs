//! ES8311 audio codec init + I2S TX playback (Phase 3 demo).
//!
//! M5StickS3 carries an ES8311 low-power codec on the shared I2C
//! bus (addr 0x18) and a PDM-driven internal speaker. For Phase 3 we
//! play back the same `qso3_busy.wav` the decoder is consuming so
//! the device feels alive — useful as both a sanity check on the
//! audio path and a demo while QSO-receive logic is wired in later
//! phases.
//!
//! Pinout (per `reference_m5stick_s3_pinout.md`):
//!   ES8311 MCLK = GPIO18      ES8311 BCLK  = GPIO17
//!   ES8311 LRCK = GPIO15      ES8311 DIN   = GPIO16  (S3 → codec)
//!
//! ES8311 init register sequence is a stripped-down port of the
//! M5Unified Arduino reference — minimum subset to get DAC →
//! speaker sounding correct at 12 kHz / 16-bit / mono. Full register
//! map at <https://docs.m5stack.com> ES8311 datasheet.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use anyhow::{Context, Result};
use embedded_shared::pipeline::{CHUNK_LEN, ChunkMsg, send_box, try_recv_box};
use esp_idf_hal::{
    delay::TickType,
    i2c::I2cDriver,
    i2s::{I2sBiDir, I2sDriver, I2sRx, I2sTx},
};
use esp_idf_svc::sys;
use mfsk_core::core::dsp::resample::LinearResamplerI16To12k;

/// Audio playback gate. `true` (default) = stream WAV samples,
/// `false` = emit silence. The decode pipeline flips this off
/// around `pass2_split`+`stage3_split` because the BP stage
/// sequesters both LX7 cores hard enough that the I2S DMA buffer
/// underruns and the speaker emits buzz/clicks (user reported as
/// "ぶつぶつ"). Silence avoids the audible glitch without disabling
/// the I2S channel itself (which would re-introduce a transient
/// pop on enable/disable).
pub static AUDIO_GATE: AtomicBool = AtomicBool::new(true);

const ES8311_ADDR: u8 = 0x18;
/// PMIC (M5PM1) at I2C 0x6E. GPIO3 (bit 3 of reg 0x11) drives the
/// onboard speaker amplifier's enable line on M5StickS3 — high to
/// unmute, low to mute. Identical to the `py32pmic_i2c_addr` constant
/// in the M5Unified board source.
const PMIC_ADDR: u8 = 0x6E;
const I2C_TIMEOUT: u32 = 100;

/// Verbatim port of M5Unified's `_speaker_enabled_cb_sticks3`
/// register sequence. Programs the codec into "MCLK=BCLK" mode so
/// the I2S master can drive both clocks off the same pin pair, then
/// powers up the analog stage and the headphone-drive amplifier and
/// unmutes the DAC. Caller is responsible for asserting the PA
/// enable on PMIC GPIO3 (`pa_enable`) right after this returns.
pub fn init_es8311(i2c: &mut I2cDriver) -> Result<()> {
    i2c.write(ES8311_ADDR, &[0x00], I2C_TIMEOUT)
        .context("ES8311 not found at I2C 0x18")?;

    // Configure PMIC GPIO3 as a push-pull output, idle low.
    // Mirrors the StickS3 PA-control init in M5Unified.cpp:
    //   reg 0x16 bit3 = 0  → GPIO3 = GPIO function (not alt)
    //   reg 0x10 bit3 = 1  → GPIO3 = output
    //   reg 0x13 bit3 = 0  → push-pull
    //   reg 0x11 bit3 = 0  → output low (PA off until we play)
    pmic_bit_off(i2c, 0x16, 1 << 3)?;
    pmic_bit_on(i2c, 0x10, 1 << 3)?;
    pmic_bit_off(i2c, 0x13, 1 << 3)?;
    pmic_bit_off(i2c, 0x11, 1 << 3)?;

    // ES8311 minimum init for playback (8 registers — same set the
    // M5Unified `_speaker_enabled_cb_sticks3` writes when it enables
    // the codec at runtime).
    let seq: &[(u8, u8)] = &[
        (0x00, 0x80), // RESET + CSM power on
        (0x01, 0xB5), // CLOCK_MANAGER: MCLK source = BCLK
        (0x02, 0x18), // CLOCK_MANAGER: MULT_PRE = 3
        (0x0D, 0x01), // SYSTEM: power up analog
        (0x12, 0x00), // SYSTEM: power up DAC
        (0x13, 0x10), // SYSTEM: enable output to HP drive
        // DAC volume — ES8311 reg 0x32. M5Unified board init uses
        // 0xBF and labels it "0 dB"; the M5PaperColor board's
        // `+16 dB` value 0xCF gives a 1 dB/step calibration.
        // 0xB5 = 0xBF − 10 = ~ -10 dB ≈ 1/3 linear of M5Unified's
        // 0 dB reference (user's preferred listening level on the
        // tiny built-in speaker after the qso3 WAV bring-up).
        (0x32, 0xA9), // DAC volume: ~ -22 dB (= 0xB5 −12 dB, two halvings)
        (0x37, 0x08), // DAC: bypass equalizer
    ];
    for &(reg, val) in seq {
        i2c.write(ES8311_ADDR, &[reg, val], I2C_TIMEOUT)
            .with_context(|| format!("ES8311 reg 0x{reg:02X} write failed"))?;
    }

    log::info!("ES8311 init OK (M5Unified-port; PA disabled at PMIC GPIO3)");
    Ok(())
}

/// Drive the speaker amplifier's enable pin (PMIC GPIO3 high). Call
/// this once just before starting playback; pair with [`pa_disable`]
/// when the audio thread shuts down.
pub fn pa_enable(i2c: &mut I2cDriver) -> Result<()> {
    pmic_bit_on(i2c, 0x11, 1 << 3)
}

#[allow(dead_code)]
pub fn pa_disable(i2c: &mut I2cDriver) -> Result<()> {
    pmic_bit_off(i2c, 0x11, 1 << 3)
}

fn pmic_bit_on(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<()> {
    let mut buf = [0u8; 1];
    i2c.write_read(PMIC_ADDR, &[reg], &mut buf, I2C_TIMEOUT)
        .with_context(|| format!("PMIC read 0x{reg:02X}"))?;
    let v = buf[0] | mask;
    i2c.write(PMIC_ADDR, &[reg, v], I2C_TIMEOUT)
        .with_context(|| format!("PMIC write 0x{reg:02X}"))
}

fn pmic_bit_off(i2c: &mut I2cDriver, reg: u8, mask: u8) -> Result<()> {
    let mut buf = [0u8; 1];
    i2c.write_read(PMIC_ADDR, &[reg], &mut buf, I2C_TIMEOUT)
        .with_context(|| format!("PMIC read 0x{reg:02X}"))?;
    let v = buf[0] & !mask;
    i2c.write(PMIC_ADDR, &[reg, v], I2C_TIMEOUT)
        .with_context(|| format!("PMIC write 0x{reg:02X}"))
}

/// Loop the supplied 12 kHz mono 16-bit WAV out the I2S TX driver,
/// upsampling 4× to 48 kHz stereo on the fly (zero-order hold).
/// The codec is in `MCLK=BCLK` mode so the actual sample rate is
/// whatever I2S generates — caller has set the I2S driver to 48 kHz.
///
/// 4× ZOH is audibly fine for a band-monitor demo (the FT8 audio
/// already sits below 3 kHz, so the 6 kHz Nyquist of the source
/// doesn't fold any spectral content into a problematic region).
/// Switch to a polyphase filter later if a follow-up needs better
/// transient response.
///
/// Digital attenuation of -18 dBFS (shift right by 3) sits the
/// playback at the same listening level the user OK'd in the sine
/// test (ES8311 DAC at 0x80 = -32 dB analog).
pub fn audio_thread(mut i2s: I2sDriver<'static, I2sTx>, wav: &'static [u8]) -> ! {
    // Bump our FreeRTOS priority above the dual_core worker (= 5)
    // and stage1_inc (= 3) so stage 3 BP can't starve the I2S DMA
    // refill — that starvation drains the DMA buffer to underrun
    // and the codec emits a click when audio resumes. Priority 8
    // lands between the dual_core worker and the watchdog.
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }

    i2s.tx_enable().expect("I2S tx_enable");
    log::info!("audio: streaming WAV (12 kHz mono → 48 kHz stereo, 4× ZOH, prio 8)");

    // Skip the 44-byte RIFF/fmt/data header. qso3_busy.wav is
    // canonical 12 kHz mono i16 LE.
    let pcm = if wav.len() > 44 { &wav[44..] } else { wav };

    // Output chunk = 80 ms of 48 kHz stereo i16 = 48000 × 0.08 × 4 bytes.
    // Input  chunk = 80 ms of 12 kHz mono i16 = 12000 × 0.08 × 2 = 1920 B
    //                                             = 960 samples.
    const IN_SAMPLES: usize = 960;
    const OUT_BYTES: usize = IN_SAMPLES * 4 /*upsample*/ * 4 /*stereo i16*/;
    let mut out = vec![0u8; OUT_BYTES];

    // Per-input-sample envelope, ramped towards the gate's target
    // (1.0 = play, 0.0 = mute). 600-sample ramp at the 12 kHz input
    // rate ≈ 50 ms — long enough that the I2S DMA buffer (~80 ms)
    // can drain through the ramped tail without a step discontinuity
    // even when the audio thread loses CPU to stage 3 BP for a
    // moment. (5 ms was too short: the DMA queue carries enough
    // pre-mute audio that the speaker still saw a step.)
    let mut env: f32 = 1.0;
    const ENV_STEP: f32 = 1.0 / 600.0;

    // Loop-boundary fade window — # of input samples over which we
    // ramp at the start and end of each WAV cycle. 600 samples at
    // 12 kHz = 50 ms each side. The FT8 slot has ~0.5 s of silence
    // around the tone block in the source WAV, so this fade fits
    // entirely inside the natural quiet zone and the user doesn't
    // hear a level dip on the signal itself — only the discontinuity
    // at byte_pos wrap-around is smoothed away.
    const LOOP_FADE_SAMPLES: usize = 600;
    let total_samples = pcm.len() / 2;

    let mut byte_pos = 0usize;
    loop {
        let target_env: f32 = if AUDIO_GATE.load(Ordering::Acquire) {
            1.0
        } else {
            0.0
        };
        let mut o = 0usize;
        for _ in 0..IN_SAMPLES {
            if byte_pos + 2 > pcm.len() {
                byte_pos = 0; // loop the WAV
            }
            let s = i16::from_le_bytes([pcm[byte_pos], pcm[byte_pos + 1]]);
            let sample_idx = byte_pos / 2;
            byte_pos += 2;

            // Gate envelope — walks toward the AUDIO_GATE target one
            // step per input sample (= 1/12 kHz tick).
            if env < target_env {
                env = (env + ENV_STEP).min(target_env);
            } else if env > target_env {
                env = (env - ENV_STEP).max(target_env);
            }

            // Loop-boundary envelope — 1.0 in the middle of the
            // WAV, ramping linearly to 0 in the LOOP_FADE_SAMPLES
            // closest to either end. Multiplied with the gate
            // envelope so loop discontinuity and gate transitions
            // are both smoothed.
            let dist_to_end = total_samples.saturating_sub(sample_idx);
            let loop_env = if sample_idx < LOOP_FADE_SAMPLES {
                sample_idx as f32 / LOOP_FADE_SAMPLES as f32
            } else if dist_to_end <= LOOP_FADE_SAMPLES {
                dist_to_end as f32 / LOOP_FADE_SAMPLES as f32
            } else {
                1.0
            };

            // -18 dBFS digital attenuation × gate × loop envelope.
            let attenuated = ((s >> 3) as f32 * env * loop_env) as i16;
            let attn = attenuated.to_le_bytes();
            for _ in 0..4 {
                out[o] = attn[0];
                out[o + 1] = attn[1];
                out[o + 2] = attn[0];
                out[o + 3] = attn[1];
                o += 4;
            }
        }
        if let Err(e) = i2s.write_all(&out, TickType::new_millis(500).ticks()) {
            log::warn!("audio: i2s write err {e:?}");
        }
    }
}

// ════════════════════════════════════════════════════════════════════
// Phase 1.5-Stick: ES8311 ADC mode + I2S RX acoustic capture
// ════════════════════════════════════════════════════════════════════
//
// IC-705 SPEAKER OUT を M5StickS3 内蔵 MEMS mic → ES8311 ADC → I2S RX
// で拾い、decode_pipeline の chunk queue に流す。BootMode::Acoustic で
// 起動した時のみ有効。UAC path (uac.rs) と shape を揃えてあるので、
// 将来 CoreS3 経由で UAC が動いたあとも acoustic path は demo として
// 残せる。
//
// Sample rate: I2S を 48 kHz stereo で回し、L channel 抽出 → 48k→12k
// linear resample (UAC と同じ pattern, LinearResamplerI16To12k)。
// ES8311 は MCLK=BCLK mode (reg 0x01=0xB5) なので codec は I2S master
// が生成する rate に追従する。stereo にしているのは codec が単一 mic
// なので L=R 同値だが、I2sStdSlotConfig::philips_slot_default
// (DataBitWidth::Bits16, SlotMode::Stereo) が既存 TX path と全く同じで、
// codec 側 init 差分を最小化できるため。

/// Chunk queue handle the capture thread pushes 12 kHz mono samples
/// into. Set by [`set_chunk_q`] from the `decode_pipeline`'s
/// source-spawn closure (`run_with_source(|q| audio::set_chunk_q(q))`).
/// `0` = not yet wired (capture loops + drops samples until pipeline
/// reaches the source-spawn step).
static CAPTURE_CHUNK_Q_ADDR: AtomicUsize = AtomicUsize::new(0);

/// Stats for the 1 Hz throughput log.
static CAPTURE_BYTES: AtomicU32 = AtomicU32::new(0);
static CAPTURE_PACKETS: AtomicU32 = AtomicU32::new(0);
static CAPTURE_ERRORS: AtomicU32 = AtomicU32::new(0);

/// 48 kHz stereo i16 = 192_000 B/s. Buffer 4 KB = ~21 ms; matches the
/// UAC reader's choice for the same reason (disconnect latency budget
/// vs ringbuf overhead).
const CAPTURE_BUFFER_BYTES: usize = 4096;

/// I2S RX timeout — 100 ms ≈ half an FT8 symbol. ES8311 always streams
/// once started, so a timeout means the DMA pipeline is stuck (e.g.
/// codec init failure). Logged but not fatal; the loop retries.
const CAPTURE_READ_TIMEOUT_MS: u32 = 100;

/// `SlotEnd` cadence in 12 kHz mono samples. Same as `wav_sim`'s
/// `SLOT_SAMPLES` and `uac::SLOT_SAMPLES_12K` — 180_000 = 15 s @ 12 kHz.
/// Wall-clock alignment isn't done here; decode DT reads as offset from
/// the midpoint of whatever 15 s window the capture happened to start
/// in (same as the UAC path).
const CAPTURE_SLOT_SAMPLES_12K: usize = 180_000;

/// Phase 1.7.3-Stick manual sync bootstrap (2026-05-17). User
/// listens to the radio audio and taps BtnA when they hear the FT8
/// slot start; that resets capture_thread's slot timing so the
/// next SlotEnd fires exactly 15 s later, aligned with the band.
/// From there the existing DT median (Phase 1.5 self-sync) handles
/// fine refinement. Cheap operator-in-the-loop sync, no algorithmic
/// changes to coarse_sync required.
pub static SYNC_RESET_PENDING: AtomicBool = AtomicBool::new(false);

/// Monotonically incremented every time `reset_slot_boundary` accepts
/// a BtnA press. Phase 1.7.4-Stick decode_pipeline reads this to detect
/// "the user pressed BtnA — drop my high-water-mark DT lock" without
/// taking a second atomic flag. Wraps after 4G presses (fine for any
/// realistic operator session).
pub static SYNC_RESET_GEN: AtomicU32 = AtomicU32::new(0);

/// `reset_slot_boundary` debounce — milliseconds since boot of the
/// last successful sync-mark trigger. Subsequent presses within
/// `SYNC_MARK_DEBOUNCE_MS` are silently dropped so the user can't
/// inadvertently reset the slot every iteration of the polling
/// loop (~100 ms cadence) which would prevent a full slot from
/// accumulating. Stored as ms-since-boot to fit in u32 (xtensa has
/// no 64-bit atomic intrinsics).
static LAST_SYNC_MARK_MS: AtomicU32 = AtomicU32::new(0);
/// Hardware-bounce protection only — was 2 s but that suppressed
/// legitimate re-sync attempts the operator made just after a slot
/// boundary, with the next accepted press landing ~2 s late. 200 ms
/// is well above any KEY1 GPIO bouncing window (< 1 ms typical) and
/// well below the FT8 slot period (15 s) so legitimate sync attempts
/// are never rejected.
const SYNC_MARK_DEBOUNCE_MS: u32 = 200;

/// Trigger a manual slot-boundary reset from the display loop /
/// menu path. Capture thread sees this on its next iteration and
/// restarts slot accumulation. Returns `true` if the mark was
/// accepted (state changed) or `false` if it was suppressed by the
/// debounce window. The caller uses the return value to drive
/// instant UI feedback ("SYNC SET, wait for next slot").
pub fn reset_slot_boundary() -> bool {
    // ms-since-boot — esp_timer_get_time returns us; truncate to ms
    // then to u32 (wraps after 49 days, fine for this purpose).
    let now_ms = (unsafe { sys::esp_timer_get_time() } / 1000) as u32;
    let last = LAST_SYNC_MARK_MS.load(Ordering::Acquire);
    if last != 0 && now_ms.wrapping_sub(last) < SYNC_MARK_DEBOUNCE_MS {
        log::info!(
            "audio: sync mark suppressed (debounce {} ms < {} ms)",
            now_ms.wrapping_sub(last),
            SYNC_MARK_DEBOUNCE_MS
        );
        return false;
    }
    LAST_SYNC_MARK_MS.store(now_ms, Ordering::Release);
    SYNC_RESET_PENDING.store(true, Ordering::Release);
    SYNC_RESET_GEN.fetch_add(1, Ordering::AcqRel);
    true
}

/// Wire the chunk queue handle. Called from
/// `decode_pipeline::run_with_source`'s source-spawn closure in the
/// pipeline thread, before the decode loop blocks on `recv_box`.
pub fn set_chunk_q(q: sys::QueueHandle_t) {
    CAPTURE_CHUNK_Q_ADDR.store(q as usize, Ordering::Release);
    log::info!("audio capture: chunk_q wired (addr={:#x})", q as usize);
}

/// Standalone ES8311 ADC mic-mode init. Verbatim port of M5Unified's
/// `_microphone_enabled_cb_sticks3` for board_M5StickS3 (sequence at
/// `M5Unified.cpp` master branch, 2026-05-17 fetch). **Do NOT call
/// [`init_es8311`] first** — this sequence starts with a soft RESET
/// + CSM power-on and configures the codec purely for mic capture
/// (skips the DAC-only registers 0x12 / 0x13 that the speaker init
/// writes; reg 0x01=0xBA differs from speaker's 0xB5 to engage the
/// ADC clock path).
///
/// Register meanings:
/// - 0x00 0x80: RESET + CSM POWER ON
/// - 0x01 0xBA: CLOCK_MANAGER, MCLK = BCLK (mic-mode variant)
/// - 0x02 0x18: CLOCK_MANAGER, MULT_PRE = 3
/// - 0x0D 0x01: SYSTEM, power up analog
/// - 0x0E 0x02: SYSTEM, enable analog PGA + ADC modulator
/// - 0x14 0x10: ADC_REG14, Mic1p-Mic1n differential, PGA gain MIN
/// - 0x17 0xFF: ADC_REG17, ADC volume MAXGAIN
/// - 0x1C 0x6A: ADC_REG1C, EQ bypass + DC offset cancel
///
/// If captured signal amplitude is too low, the next knob is the
/// PGA gain in reg 0x14 (raise the low nibble 0x10 → 0x1F for higher
/// gain). If clipping, lower ADC volume reg 0x17.
pub fn init_es8311_capture(i2c: &mut I2cDriver) -> Result<()> {
    use esp_idf_svc::hal::delay::FreeRtos;
    // M5StickS3 PA enable line off (codec speaker stays silent during
    // mic capture; otherwise the speaker amp would howl any leaked
    // signal back to the mic).
    let _ = pmic_bit_off(i2c, 0x11, 1 << 3);

    // **Disable → settle → enable** init pattern. ES8311 is a
    // standalone IC on the I2C bus — S3 reset doesn't reset the
    // codec, so the previous boot's register state persists across
    // reboot. User-observed 2026-05-17: "一度リブートすると二度と
    // 同期しない" = first boot's mic-mode init works, second boot the
    // codec is in some weird half-configured state and the writes
    // don't all take. M5Unified's mic enable_cb pairs with a
    // disable_cb that powers down the analog stage first; by always
    // running disable→enable on init we get a clean starting state
    // regardless of what the previous run left behind.
    let disable_seq: &[(u8, u8)] = &[
        (0x0D, 0xFC), // SYSTEM: power down analog
        (0x0E, 0x6A), // SYSTEM
        (0x00, 0x00), // RESET / CSM POWER DOWN
    ];
    for &(reg, val) in disable_seq {
        let _ = i2c.write(ES8311_ADDR, &[reg, val], I2C_TIMEOUT);
    }
    FreeRtos::delay_ms(20); // analog settle after power-down

    let mic_seq: &[(u8, u8)] = &[
        (0x00, 0x80),
        (0x01, 0xBA),
        (0x02, 0x18),
        (0x0D, 0x01),
        (0x0E, 0x02),
        (0x14, 0x10),
        (0x17, 0xFF),
        (0x1C, 0x6A),
    ];
    // 10 ms after RESET (write[0]) before the rest of the sequence so
    // the CSM is fully up — observed empirical, datasheet recommends
    // ≥ 5 ms after CSM_RESET deassertion.
    i2c.write(ES8311_ADDR, &[mic_seq[0].0, mic_seq[0].1], I2C_TIMEOUT)
        .with_context(|| format!("ES8311 mic reg 0x{:02X} write failed", mic_seq[0].0))?;
    FreeRtos::delay_ms(10);
    for &(reg, val) in &mic_seq[1..] {
        i2c.write(ES8311_ADDR, &[reg, val], I2C_TIMEOUT)
            .with_context(|| format!("ES8311 mic reg 0x{reg:02X} write failed"))?;
    }
    log::info!(
        "ES8311 ADC mic-mode init OK (M5Unified-port: disable→settle→enable, MCLK=BCLK 0xBA, Mic1 diff, vol MAX, EQ bypass)"
    );
    Ok(())
}

/// Capture thread body. Pumps `I2sDriver<I2sRx>` at the configured
/// I2S rate (callers must construct with the same 48 kHz stereo
/// settings used for TX so the codec's MCLK=BCLK clock chain matches),
/// extracts left channel, resamples 48 k → 12 k mono via
/// `LinearResamplerI16To12k`, and pushes `CHUNK_LEN`-sized chunks into
/// the decode pipeline's chunk queue (with `SlotEnd` every
/// `CAPTURE_SLOT_SAMPLES_12K` samples).
///
/// Counts I2S RX throughput in `CAPTURE_BYTES` / `CAPTURE_PACKETS` /
/// `CAPTURE_ERRORS` and logs a 1 Hz status line. Same shape as
/// `uac::reader_thread` so the diagnostic flow is identical between
/// the acoustic and UAC paths.
pub fn capture_thread(mut i2s: I2sDriver<'static, I2sRx>) -> ! {
    use esp_idf_svc::hal::delay::FreeRtos;
    // Bump priority above the dual_core worker (= 5) and stage1_inc
    // (= 3) so stage 3 BP can't starve the I2S DMA refill. Same
    // rationale as `audio_thread` (priority 8 — between dual_core
    // worker and watchdog).
    unsafe {
        sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }

    // Phase 1.7.3-Stick fix: wait for the decode_pipeline thread to
    // finish BASIS alloc + stage1_inc spawn + chunk_q registration
    // BEFORE we enable I2S RX. Otherwise the ~1 s of audio captured
    // during pipeline init was being silently dropped (CHUNK_Q_ADDR
    // == 0 → samples not pushed), which gave the user-visible "first
    // slot of WF is dark" + first slot decode misalignment.
    log::info!("audio capture: waiting for chunk_q registration before I2S RX enable");
    let wait_start = std::time::Instant::now();
    while CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire) == 0 {
        FreeRtos::delay_ms(10);
        if wait_start.elapsed().as_secs() >= 10 {
            log::warn!(
                "audio capture: chunk_q still unset after 10 s — enabling I2S anyway"
            );
            break;
        }
    }
    log::info!(
        "audio capture: chunk_q wait done ({} ms), enabling I2S RX",
        wait_start.elapsed().as_millis()
    );

    i2s.rx_enable().expect("I2S rx_enable");
    log::info!(
        "audio capture: streaming I2S RX (48 kHz stereo → L extract → 12 kHz mono, prio 8)"
    );

    // Move big buffers to heap so the thread stack stays under 6 KB
    // (internal DRAM is tight after BASIS + stage1_inc + dual_core —
    // 8 KB stack spawn OOMs during bring-up).
    let mut buf: Box<[u8; CAPTURE_BUFFER_BYTES]> = Box::new([0u8; CAPTURE_BUFFER_BYTES]);

    // ES8311 ADC + PGA settling. The codec takes ~250 ms after
    // rx_enable to produce stable samples — first slot's worth of
    // audio is a noisy AGC ramp + transient PGA artefacts which
    // makes the WF's first 1-2 slots look visibly wrong (user-observed
    // 2026-05-17: "1st/2nd の WF だけおかしい"). Drain reads into the
    // heap-allocated buf so we don't stack-overflow (a 4 KB local
    // array tipped the thread over its 6 KB stack limit — 2026-05-17).
    log::info!("audio capture: codec settling drain (~250 ms)");
    let settle_start = std::time::Instant::now();
    while settle_start.elapsed().as_millis() < 250 {
        let _ = i2s.read(&mut buf[..], TickType::new_millis(50).ticks());
    }
    log::info!("audio capture: settling done, starting normal capture");
    let mut resampler = LinearResamplerI16To12k::new(48_000);
    let mut left_scratch: Box<[i16; CAPTURE_BUFFER_BYTES / 4]> =
        Box::new([0i16; CAPTURE_BUFFER_BYTES / 4]);
    let mut dst_scratch: Box<[i16; 512]> = Box::new([0i16; 512]);
    // Per-chunk accumulator. Flushed at CHUNK_LEN (1200 samples =
    // 100 ms @ 12 kHz, matches wav_sim and UAC).
    let mut chunk: Vec<i16> = Vec::with_capacity(CHUNK_LEN);
    let mut slot_samples: usize = 0;
    // Mutable slot target — adjusted by one-shot bootstrap shift via
    // `time_sync::take_bootstrap_slot_shift_12k` at each SlotEnd so
    // the next slot aligns with the band's actual TX cadence.
    let mut slot_end_target: usize = CAPTURE_SLOT_SAMPLES_12K;
    let mut wav_idx: usize = 0;
    let mut last_log = std::time::Instant::now();
    let mut last_bytes: u32 = 0;

    loop {
        // Phase 1.7.4-Stick coordinated reset protocol (2026-05-17).
        // BtnA pressed — atomically realign capture+stage1_inc+
        // shift-atomic so the next slot decoded is clean (no
        // pre-/post-BtnA audio bleed, no stale auto-sync shift).
        if SYNC_RESET_PENDING.swap(false, Ordering::AcqRel) {
            let chunk_q_addr = CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire);
            let mut drained = 0usize;
            if chunk_q_addr != 0 {
                let chunk_q = chunk_q_addr as sys::QueueHandle_t;
                // 1. Drop any stale pre-BtnA Samples still in flight
                //    (chunk_q depth 4 = up to ~400 ms backlog).
                while let Some(_stale) = try_recv_box::<ChunkMsg>(chunk_q) {
                    drained += 1;
                }
                // 2. Clear stale auto-sync shift posted by
                //    decode_pipeline for the now-discarded slot
                //    transition.
                let _ = mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                // 3. Signal stage1_inc to discard its in-progress
                //    spec — otherwise the half-built accumulator
                //    would merge with post-reset audio.
                send_box(chunk_q, Box::new(ChunkMsg::SlotResetMark));
            }
            log::info!(
                "audio capture: SYNC_RESET applied (drained {} chunks, was {} samples into slot)",
                drained, slot_samples
            );
            chunk.clear();
            slot_samples = 0;
            slot_end_target = CAPTURE_SLOT_SAMPLES_12K;
            resampler = LinearResamplerI16To12k::new(48_000);
        }

        let bytes_read = match i2s.read(&mut buf[..], TickType::new_millis(CAPTURE_READ_TIMEOUT_MS.into()).ticks()) {
            Ok(n) => n,
            Err(e) => {
                CAPTURE_ERRORS.fetch_add(1, Ordering::Relaxed);
                log::warn!("audio capture: i2s read err {e:?}");
                continue;
            }
        };
        if bytes_read == 0 {
            // Timeout-with-no-data; the DMA pipeline is normally
            // never idle once the codec is streaming, so 0 likely
            // means codec init didn't take. Keep looping; the 1 Hz
            // log will show bytes=0 and flag the issue.
            continue;
        }
        CAPTURE_BYTES.fetch_add(bytes_read as u32, Ordering::Relaxed);
        CAPTURE_PACKETS.fetch_add(1, Ordering::Relaxed);

        // Decode interleaved stereo i16 → take left channel only.
        // bytes_read should always be a multiple of 4 (stereo i16)
        // given the philips_slot_default(Bits16, Stereo) config.
        let stereo_samples = (bytes_read) / 4;
        debug_assert!(stereo_samples <= left_scratch.len());
        for i in 0..stereo_samples {
            let off = i * 4;
            left_scratch[i] = i16::from_le_bytes([buf[off], buf[off + 1]]);
        }

        // Wait for the decode_pipeline to populate the chunk queue.
        // Until then drop samples — bounded race window (~200 ms
        // pipeline init), same as the UAC path.
        let chunk_q_addr = CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire);
        if chunk_q_addr == 0 {
            continue;
        }
        let chunk_q = chunk_q_addr as sys::QueueHandle_t;

        // Feed the resampler in a loop until input drained.
        let mut src_offset = 0usize;
        while src_offset < stereo_samples {
            let (consumed, produced) = resampler.process(
                &left_scratch[src_offset..stereo_samples],
                &mut dst_scratch[..],
            );
            for &s in &dst_scratch[..produced] {
                chunk.push(s);
                if chunk.len() >= CHUNK_LEN {
                    let to_send = core::mem::replace(
                        &mut chunk,
                        Vec::with_capacity(CHUNK_LEN),
                    );
                    send_box(chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                    slot_samples += CHUNK_LEN;
                    if slot_samples >= slot_end_target {
                        send_box(
                            chunk_q,
                            Box::new(ChunkMsg::SlotEnd {
                                wav_idx,
                                total_samples: slot_samples,
                            }),
                        );
                        // Consume the latest bootstrap shift hint
                        // from decode_pipeline (pass1 candidate DT
                        // median). Apply once as a one-shot adjustment
                        // to the next slot length; the producer side
                        // posts 0 in steady state so this is idempotent.
                        let shift =
                            mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                        // shift is signed, capped at ±2400 samples
                        // (±0.2 s) by decode_pipeline so the slot
                        // length always lands in [177_600, 182_400],
                        // i.e. stage1_inc still completes pair 91
                        // (177_600-sample requirement) and truncates
                        // safely on the lengthen side.
                        let next_target = (CAPTURE_SLOT_SAMPLES_12K as i32 + shift)
                            .clamp(60_000, 200_000)
                            as usize;
                        if shift != 0 {
                            log::info!(
                                "audio capture: applying slot shift {:+} samples (next slot = {} samples)",
                                shift, next_target
                            );
                        }
                        slot_end_target = next_target;
                        wav_idx = wav_idx.wrapping_add(1);
                        slot_samples = 0;
                    }
                }
            }
            if consumed == 0 && produced == 0 {
                break;
            }
            src_offset += consumed;
        }

        // 1 Hz throughput log. Expected ~192_000 B/s for fully
        // streaming codec. < 50 kB/s suggests codec init didn't
        // engage the ADC path; 0 B/s means the I2S RX channel
        // itself isn't running.
        let now = std::time::Instant::now();
        if now.duration_since(last_log).as_secs() >= 1 {
            let bytes = CAPTURE_BYTES.load(Ordering::Relaxed);
            let packets = CAPTURE_PACKETS.load(Ordering::Relaxed);
            let errors = CAPTURE_ERRORS.load(Ordering::Relaxed);
            let bps = bytes.wrapping_sub(last_bytes);
            log::info!(
                "audio capture tick: {bps} B/s (total {bytes} B / {packets} pkt / {errors} err / slot={slot_samples}/12k)"
            );
            last_log = now;
            last_bytes = bytes;
        }
    }
}

// ════════════════════════════════════════════════════════════════════
// Phase 1.7.1-Stick: combined RX + TX I/O for BootMode::Qso
// ════════════════════════════════════════════════════════════════════
//
// Single-thread owner of `I2sDriver<I2sBiDir>` (single i2s0 controller
// with both RX and TX channels enabled — ES8311 codec supports
// simultaneous ADC + DAC; pins BCLK 17 / MCLK 18 / LRCK 15 shared,
// codec DIN 16 = RX side, codec DOUT 14 = TX side).
//
// Loop priority: TX first, RX second.
// - When the QSO FSM has a pending TxIntent and auto_cq permits, we
//   PTT-on + synth + write (~12.6 s blocking). RX DMA keeps streaming
//   into the IDF ringbuf during TX; we don't read it, so the ringbuf
//   overflows and oldest data is silently dropped — that's fine since
//   the audio we'd capture during our own TX is leakage from the
//   speaker into the mic (unwanted feedback anyway). After TX, the
//   first few RX reads catch the post-TX moment.
// - Otherwise we do one RX read iteration (4 KB / 21 ms) and push the
//   resampled L-channel to the decode pipeline's chunk queue. The
//   self-sync bootstrap (Phase 1.5) re-aligns any slot drift caused
//   by the TX gap.
//
// AUDIO_GATE is used as the "don't push samples to decode pipeline"
// flag during TX: capture_thread + capture_tx_thread both honour it
// (the read still happens to drain the ringbuf, but the chunk push
// is suppressed). tx_scheduler / Phase 1.6 also flip the gate around
// each TX bracket; doing it again here is idempotent.

pub fn capture_tx_thread(
    mut i2s: I2sDriver<'static, I2sBiDir>,
    qso: std::sync::Arc<std::sync::Mutex<mfsk_app_shared::qso::QsoManager>>,
) -> ! {
    use esp_idf_svc::hal::delay::FreeRtos;
    unsafe {
        sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }
    // Phase 1.7.3-Stick fix: same chunk_q wait pattern as
    // capture_thread (Acoustic). Prevents the pipeline-init race
    // that was silently dropping the first ~1 s of audio.
    log::info!("audio capture+tx: waiting for chunk_q registration before I2S bidir enable");
    let wait_start = std::time::Instant::now();
    while CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire) == 0 {
        FreeRtos::delay_ms(10);
        if wait_start.elapsed().as_secs() >= 10 {
            log::warn!(
                "audio capture+tx: chunk_q still unset after 10 s — enabling I2S anyway"
            );
            break;
        }
    }
    log::info!(
        "audio capture+tx: chunk_q wait done ({} ms), enabling I2S",
        wait_start.elapsed().as_millis()
    );
    i2s.rx_enable().expect("I2S rx_enable");
    i2s.tx_enable().expect("I2S tx_enable");
    log::info!("audio capture+tx: bidir I2S up (RX 48k stereo + TX 48k stereo on i2s0)");

    let mut buf: Box<[u8; CAPTURE_BUFFER_BYTES]> = Box::new([0u8; CAPTURE_BUFFER_BYTES]);
    // Codec settling drain (same rationale as capture_thread): ~250 ms
    // of ES8311 ADC AGC / PGA transients dropped before normal capture
    // resumes. Reuses heap-allocated `buf` to avoid the 4 KB stack
    // overflow that a local settle_buf caused.
    log::info!("audio capture+tx: codec settling drain (~250 ms)");
    let settle_start = std::time::Instant::now();
    while settle_start.elapsed().as_millis() < 250 {
        let _ = i2s.read(&mut buf[..], TickType::new_millis(50).ticks());
    }
    log::info!("audio capture+tx: settling done, starting normal capture");

    let mut resampler = LinearResamplerI16To12k::new(48_000);
    let mut left_scratch: Box<[i16; CAPTURE_BUFFER_BYTES / 4]> =
        Box::new([0i16; CAPTURE_BUFFER_BYTES / 4]);
    let mut dst_scratch: Box<[i16; 512]> = Box::new([0i16; 512]);
    let mut chunk: Vec<i16> = Vec::with_capacity(CHUNK_LEN);
    let mut slot_samples: usize = 0;
    let mut slot_end_target: usize = CAPTURE_SLOT_SAMPLES_12K;
    let mut wav_idx: usize = 0;
    let mut last_log = std::time::Instant::now();
    let mut last_bytes: u32 = 0;
    // BLE pair grace — civ::start spawns its own ble_thread which takes
    // 5-15 s to scan + pair to IC-705; without this delay our first
    // call_cq could fire before set_ptt has a working CI-V link.
    let bring_up_grace = std::time::Instant::now() + std::time::Duration::from_secs(10);

    // TX-side scratch (48k stereo i16): 240 12k mono samples × 4 ZOH
    // × stereo × 2 bytes = 3 840 bytes. Reused per write_all call.
    const TX_IN_CHUNK: usize = 240;
    const TX_OUT_BYTES_PER_IN: usize = 16;
    let mut tx_out: Box<[u8; TX_IN_CHUNK * TX_OUT_BYTES_PER_IN]> =
        Box::new([0u8; TX_IN_CHUNK * TX_OUT_BYTES_PER_IN]);

    loop {
        // Phase 1.7.4-Stick coordinated reset (mirrors capture_thread).
        // Drain pre-BtnA chunks + clear stale shift + signal stage1_inc
        // before resetting local capture state.
        if SYNC_RESET_PENDING.swap(false, Ordering::AcqRel) {
            let chunk_q_addr = CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire);
            let mut drained = 0usize;
            if chunk_q_addr != 0 {
                let chunk_q = chunk_q_addr as sys::QueueHandle_t;
                while let Some(_stale) = try_recv_box::<ChunkMsg>(chunk_q) {
                    drained += 1;
                }
                let _ = mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                send_box(chunk_q, Box::new(ChunkMsg::SlotResetMark));
            }
            log::info!(
                "audio capture+tx: SYNC_RESET applied (drained {} chunks, was {} samples in)",
                drained, slot_samples
            );
            chunk.clear();
            slot_samples = 0;
            slot_end_target = CAPTURE_SLOT_SAMPLES_12K;
            resampler = LinearResamplerI16To12k::new(48_000);
        }

        // ── TX priority check ─────────────────────────────────────
        let intent_opt = if std::time::Instant::now() >= bring_up_grace {
            tx_scheduler_pick_intent(&qso)
        } else {
            None
        };
        if let Some(intent) = intent_opt {
            do_tx_cycle(&mut i2s, &qso, &intent, &mut tx_out[..]);
            // Reset capture state — the gap in RX during ~12.6 s TX
            // makes any in-flight resample / slot accumulator stale.
            // Drop partial chunk; next slot will start fresh, self-sync
            // (Phase 1.5 bootstrap_slot_shift_12k) realigns within 1-2
            // slots from the FSM's CQ/Reply spacing.
            chunk.clear();
            slot_samples = 0;
            slot_end_target = CAPTURE_SLOT_SAMPLES_12K;
            resampler = LinearResamplerI16To12k::new(48_000);
            continue;
        }

        // ── RX iteration (same shape as capture_thread) ───────────
        let bytes_read = match i2s.read(&mut buf[..], TickType::new_millis(CAPTURE_READ_TIMEOUT_MS.into()).ticks()) {
            Ok(n) => n,
            Err(e) => {
                CAPTURE_ERRORS.fetch_add(1, Ordering::Relaxed);
                log::warn!("audio capture+tx: i2s read err {e:?}");
                continue;
            }
        };
        if bytes_read == 0 {
            continue;
        }
        // AUDIO_GATE: when false (e.g. just before/during TX), drain
        // the read but don't push to the decode pipeline.
        if !AUDIO_GATE.load(Ordering::Acquire) {
            continue;
        }
        CAPTURE_BYTES.fetch_add(bytes_read as u32, Ordering::Relaxed);
        CAPTURE_PACKETS.fetch_add(1, Ordering::Relaxed);

        let stereo_samples = bytes_read / 4;
        debug_assert!(stereo_samples <= left_scratch.len());
        for i in 0..stereo_samples {
            let off = i * 4;
            left_scratch[i] = i16::from_le_bytes([buf[off], buf[off + 1]]);
        }

        let chunk_q_addr = CAPTURE_CHUNK_Q_ADDR.load(Ordering::Acquire);
        if chunk_q_addr == 0 {
            continue;
        }
        let chunk_q = chunk_q_addr as sys::QueueHandle_t;

        let mut src_offset = 0usize;
        while src_offset < stereo_samples {
            let (consumed, produced) = resampler.process(
                &left_scratch[src_offset..stereo_samples],
                &mut dst_scratch[..],
            );
            for &s in &dst_scratch[..produced] {
                chunk.push(s);
                if chunk.len() >= CHUNK_LEN {
                    let to_send = core::mem::replace(&mut chunk, Vec::with_capacity(CHUNK_LEN));
                    send_box(chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                    slot_samples += CHUNK_LEN;
                    if slot_samples >= slot_end_target {
                        send_box(
                            chunk_q,
                            Box::new(ChunkMsg::SlotEnd {
                                wav_idx,
                                total_samples: slot_samples,
                            }),
                        );
                        let shift =
                            mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                        let next_target = (CAPTURE_SLOT_SAMPLES_12K as i32 + shift)
                            .max(60_000) as usize;
                        slot_end_target = next_target;
                        wav_idx = wav_idx.wrapping_add(1);
                        slot_samples = 0;
                    }
                }
            }
            if consumed == 0 && produced == 0 {
                break;
            }
            src_offset += consumed;
        }

        let now = std::time::Instant::now();
        if now.duration_since(last_log).as_secs() >= 1 {
            let bytes = CAPTURE_BYTES.load(Ordering::Relaxed);
            let bps = bytes.wrapping_sub(last_bytes);
            log::info!("audio capture+tx tick: {bps} B/s rx (slot={slot_samples}/12k)");
            last_log = now;
            last_bytes = bytes;
        }
    }
}

/// Pull the next TX intent if one is pending. Mirrors the same logic
/// as `tx_scheduler::scheduler_thread` so the auto-CQ behaviour is
/// identical between BootMode::TxTest (RX-less scheduler) and
/// BootMode::Qso (combined thread).
fn tx_scheduler_pick_intent(
    qso: &std::sync::Arc<std::sync::Mutex<mfsk_app_shared::qso::QsoManager>>,
) -> Option<mfsk_app_shared::qso::TxIntent> {
    let q = qso.lock().ok()?;
    let auto_cq = q.auto_cq_enabled;
    let intent = q.next_tx();
    if intent.is_some() {
        return intent;
    }
    if auto_cq && q.state == mfsk_app_shared::qso::QsoState::Idle {
        drop(q);
        let mut q = qso.lock().ok()?;
        return Some(q.call_cq(None));
    }
    None
}

/// One full TX cycle: PTT bracket + synth + write to the bidir I2S TX
/// side. Blocks ~12.6 s. After this returns, the caller should reset
/// its capture / resampler state (RX gap during TX leaves stale data).
fn do_tx_cycle(
    i2s: &mut I2sDriver<'static, I2sBiDir>,
    qso: &std::sync::Arc<std::sync::Mutex<mfsk_app_shared::qso::QsoManager>>,
    intent: &mfsk_app_shared::qso::TxIntent,
    tx_out: &mut [u8],
) {
    use esp_idf_svc::hal::delay::FreeRtos;
    use mfsk_core::ft8::message::pack77;

    let df_hz = mfsk_app_shared::ui::state::UI
        .lock()
        .ok()
        .map(|u| u.current_df_hz)
        .unwrap_or(0);
    let df_hz = if df_hz == 0 { 1500u16 } else { df_hz };

    let formatted = intent.formatted();
    log::info!(
        "audio TX: firing → '{}' @ {df_hz} Hz",
        formatted.as_str()
    );

    let msg77 = match pack77(intent.call1.as_str(), intent.call2.as_str(), intent.report.as_str()) {
        Some(m) => m,
        None => {
            log::warn!("audio TX: pack77 failed, skipping slot");
            return;
        }
    };

    // Mute decode pipeline ingestion + key TX.
    AUDIO_GATE.store(false, Ordering::Release);
    crate::civ::set_ptt(true);
    FreeRtos::delay_ms(100);

    let samples = match crate::tx::synthesize_message77(&msg77, df_hz as f32) {
        Ok(s) => s,
        Err(e) => {
            log::error!("audio TX: synth failed: {e:#}");
            crate::civ::set_ptt(false);
            AUDIO_GATE.store(true, Ordering::Release);
            return;
        }
    };

    // Stream 12 kHz mono → 48 kHz stereo 4× ZOH chunks via I2S TX.
    const IN_CHUNK: usize = 240;
    const OUT_PER_IN: usize = 16;
    let t_start = std::time::Instant::now();
    for chunk in samples.chunks(IN_CHUNK) {
        let mut o = 0;
        for &s in chunk {
            let b = s.to_le_bytes();
            for _ in 0..4 {
                tx_out[o] = b[0];
                tx_out[o + 1] = b[1];
                tx_out[o + 2] = b[0];
                tx_out[o + 3] = b[1];
                o += 4;
            }
        }
        let nbytes = chunk.len() * OUT_PER_IN;
        if let Err(e) = i2s.write_all(&tx_out[..nbytes], TickType::new_millis(500).ticks()) {
            log::warn!("audio TX: i2s write err {e:?}");
            break;
        }
    }
    log::info!(
        "audio TX: complete in {:?} ({} samples)",
        t_start.elapsed(),
        samples.len()
    );

    crate::civ::set_ptt(false);
    AUDIO_GATE.store(true, Ordering::Release);

    // Advance FSM (retry counter / Idle reset).
    if let Ok(mut q) = qso.lock() {
        let _ = q.on_period_end();
    }
}
