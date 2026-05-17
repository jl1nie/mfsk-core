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
use embedded_shared::pipeline::{CHUNK_LEN, ChunkMsg, send_box};
use esp_idf_hal::{
    delay::TickType,
    i2c::I2cDriver,
    i2s::{I2sDriver, I2sRx, I2sTx},
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

/// Wire the chunk queue handle. Called from
/// `decode_pipeline::run_with_source`'s source-spawn closure in the
/// pipeline thread, before the decode loop blocks on `recv_box`.
pub fn set_chunk_q(q: sys::QueueHandle_t) {
    CAPTURE_CHUNK_Q_ADDR.store(q as usize, Ordering::Release);
    log::info!("audio capture: chunk_q wired (addr={:#x})", q as usize);
}

/// Extend the existing ES8311 DAC init with ADC mic-mode registers.
/// Caller must have run [`init_es8311`] first (powers up the CSM and
/// configures MCLK=BCLK). This function adds:
///
/// - reg 0x14 PGA: MIC1 single-ended at +12 dB
/// - reg 0x15 ADC DPF + noise gate: off
/// - reg 0x16 ADC HPF time constant: default
/// - reg 0x17 ADC volume: 0 dB (0xBF)
/// - reg 0x09 ADC OSR: 32 (matches DAC OSR)
/// - reg 0x0A ADC HPF: bypass (we want raw signal, FT8 has no DC offset
///   issue at 1500 Hz)
/// - reg 0x0E ADC powerup
///
/// Values are taken from the espressif `esp_codec_dev` reference
/// driver's mic-init path; tested working on Espressif's own ES8311
/// dev kits. Untested on M5StickS3 — first-flash output will say
/// whether the mic actually produces non-zero samples, and gain may
/// need bumping (reg 0x14) if speech-volume audio comes through at
/// near-zero amplitude.
pub fn init_es8311_capture(i2c: &mut I2cDriver) -> Result<()> {
    let mic_seq: &[(u8, u8)] = &[
        (0x14, 0x1A), // PGA: MIC1 single-ended, +12 dB (bits 4:0 = 0b11010 = +18 dB? see datasheet)
        (0x15, 0x40), // ADC DPF off, noise gate off
        (0x16, 0x00), // ADC HPF time const = 0
        (0x17, 0xBF), // ADC volume = 0 dB (0xBF nominal)
        (0x09, 0x00), // ADC OSR low byte
        (0x0A, 0x00), // ADC HPF disabled (FT8 doesn't need HPF)
        (0x0E, 0x02), // ADC powerup: enable ADC analog + digital
        (0x44, 0x08), // DAC->ADC selector: keep DAC out separate
    ];
    for &(reg, val) in mic_seq {
        i2c.write(ES8311_ADDR, &[reg, val], I2C_TIMEOUT)
            .with_context(|| format!("ES8311 mic reg 0x{reg:02X} write failed"))?;
    }
    log::info!("ES8311 ADC mic-mode init OK (PGA +12dB, ADC vol 0dB, HPF bypass)");
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
    // Bump priority above the dual_core worker (= 5) and stage1_inc
    // (= 3) so stage 3 BP can't starve the I2S DMA refill. Same
    // rationale as `audio_thread` (priority 8 — between dual_core
    // worker and watchdog).
    unsafe {
        sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }

    i2s.rx_enable().expect("I2S rx_enable");
    log::info!(
        "audio capture: streaming I2S RX (48 kHz stereo → L extract → 12 kHz mono, prio 8)"
    );

    let mut buf = [0u8; CAPTURE_BUFFER_BYTES];
    let mut resampler = LinearResamplerI16To12k::new(48_000);
    // L-channel scratch — max bytes_read / 4 mono samples per read.
    let mut left_scratch = [0i16; CAPTURE_BUFFER_BYTES / 4];
    // Resampled output staging. 48k→12k = 4:1, so ~256 samples per
    // 4 KB read. Doubled for resampler rounding headroom.
    let mut dst_scratch = [0i16; 512];
    // Per-chunk accumulator. Flushed at CHUNK_LEN (1200 samples =
    // 100 ms @ 12 kHz, matches wav_sim and UAC).
    let mut chunk: Vec<i16> = Vec::with_capacity(CHUNK_LEN);
    let mut slot_samples: usize = 0;
    let mut wav_idx: usize = 0;
    let mut last_log = std::time::Instant::now();
    let mut last_bytes: u32 = 0;

    loop {
        let bytes_read = match i2s.read(&mut buf, TickType::new_millis(CAPTURE_READ_TIMEOUT_MS.into()).ticks()) {
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
                &mut dst_scratch,
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
                    if slot_samples >= CAPTURE_SLOT_SAMPLES_12K {
                        send_box(
                            chunk_q,
                            Box::new(ChunkMsg::SlotEnd {
                                wav_idx,
                                total_samples: slot_samples,
                            }),
                        );
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
