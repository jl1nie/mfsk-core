// SPDX-License-Identifier: GPL-3.0-or-later
//! Speaker playback for the CoreS3 — I2S TX into the AW88298.
//!
//! Exists so `ft4-demo` can play the slot it is decoding, the way
//! `m5stack-s3-app`'s `audio.rs` does for FT8 on the StickS3. Hearing
//! the audio is how you tell "the decoder found nothing" apart from
//! "there was nothing to find", which on a replayed WAV is the whole
//! question a desk demo answers.
//!
//! ## Incomplete on purpose: [`AW88298_INIT`] is empty
//!
//! Three of the four things this needs are known and are here:
//!
//! | piece | where it comes from |
//! |---|---|
//! | 1.8 V rail | AXP2101 ALDO1, already raised by [`crate::pmic`] |
//! | amp enable | AW9523B port0 bit 2 ([`board::AW9523_P0_SPK_EN`]) |
//! | I2S TX | [`init_i2s_tx`] below |
//! | **codec registers** | **not in this tree** |
//!
//! The AW88298's register map is in neither this repository nor the
//! vendored esp-idf components, and M5Unified — where the rest of this
//! board's magic numbers were cross-checked from (see `board.rs`'s
//! pin-table corrections) — is not available here either. Writing an
//! amplifier's control registers from memory is not the same class of
//! guess as a pin name: a wrong `SYSCTRL` or gain setting drives a
//! speaker with a configuration nobody chose.
//!
//! So the table is empty, and [`speaker_available`] is false while it
//! is. Nothing raises `SPK_EN` and nothing writes I2S until it is
//! filled — an unconfigured amplifier that has been *enabled* is
//! exactly the state worth avoiding. Fill the table and playback
//! starts working with no other change.
//!
//! [`board::AW9523_P0_SPK_EN`]: crate::board::AW9523_P0_SPK_EN

use anyhow::Result;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::i2s::{I2sDriver, I2sTx};

/// AW88298 control registers, `(reg, value)` in write order.
///
/// **Empty: the sequence is unverified and unavailable.** See the
/// module docs. What is known:
///
/// - I2C address `0x36` ([`crate::board::AW88298_I2C_ADDR`])
/// - registers are 16-bit, MSB first
/// - it wants its 1.8 V rail (ALDO1) up first, which `pmic::init` does
/// - M5Unified's `_speaker_enabled_cb_cores3` gates the amp with
///   AW9523B port0 bit 2, which is what [`enable_amp`] writes
///
/// What is not known, and must not be guessed: reset, `SYSCTRL`,
/// `I2SCTRL`, sample-rate/format selection, boost and volume.
const AW88298_INIT: &[(u8, u16)] = &[];

/// Whether playback can be brought up at all.
///
/// False until [`AW88298_INIT`] is filled. Callers should log and carry
/// on rather than treating it as an error — a receiver that cannot play
/// audio is still a receiver, and this is exactly the state the board
/// has been in all along.
pub fn speaker_available() -> bool {
    !AW88298_INIT.is_empty()
}

/// Write the codec's init sequence and raise the amp enable.
///
/// Refuses when [`AW88298_INIT`] is empty rather than enabling an
/// amplifier nobody has configured.
pub fn enable_amp(i2c: &mut I2cDriver) -> Result<()> {
    if !speaker_available() {
        anyhow::bail!("AW88298 init sequence is empty — playback not configured");
    }
    for &(reg, val) in AW88298_INIT {
        let buf = [reg, (val >> 8) as u8, (val & 0xff) as u8];
        i2c.write(crate::board::AW88298_I2C_ADDR, &buf, 100)?;
    }
    // Port-0 read-modify-write, the same shape `pmic.rs` uses for the
    // USB host rails: the other bits on this port are the touch
    // enable, the external 5 V rail and the USB OTG gate, and clearing
    // any of them here would be a very confusing bug elsewhere.
    let mut p0 = [0u8; 1];
    i2c.write_read(crate::board::AW9523B_I2C_ADDR, &[0x02], &mut p0, 100)?;
    let next = p0[0] | crate::board::AW9523_P0_SPK_EN;
    i2c.write(crate::board::AW9523B_I2C_ADDR, &[0x02, next], 100)?;
    log::info!("audio_out: AW88298 configured, SPK_EN raised (port0 0x{p0:02x?} -> 0x{next:02x})");
    Ok(())
}

/// Drop the amp enable. Leaves the codec registers alone.
pub fn disable_amp(i2c: &mut I2cDriver) -> Result<()> {
    let mut p0 = [0u8; 1];
    i2c.write_read(crate::board::AW9523B_I2C_ADDR, &[0x02], &mut p0, 100)?;
    let next = p0[0] & !crate::board::AW9523_P0_SPK_EN;
    i2c.write(crate::board::AW9523B_I2C_ADDR, &[0x02, next], 100)?;
    Ok(())
}

/// Output rate. The codec runs at 48 kHz; the decode path is 12 kHz, so
/// [`play_block`] interpolates 4:1 rather than the codec being asked
/// for an unusual rate.
pub const OUT_RATE_HZ: u32 = 48_000;

/// I2S pins, **from M5Stack's published CoreS3 pinout, not verified on
/// this board**.
///
/// `board.rs` carries two corrections to pin tables that were wrong for
/// months (LCD_RST/TP_RST swapped, and SPK_EN read as P0_7 when the bit
/// M5Unified toggles is 2), so a pin table here is a claim to check
/// rather than a fact to rely on. Unlike the codec registers these are
/// safe to get wrong — a misrouted I2S line is silent, not damaging —
/// so they are stated and used, with this note attached.
pub const PIN_BCLK: u32 = 34;
/// See [`PIN_BCLK`].
pub const PIN_LRCK: u32 = 33;
/// See [`PIN_BCLK`]. CoreS3 → AW88298.
pub const PIN_DOUT: u32 = 13;
/// See [`PIN_BCLK`]. The AW88298 is clocked from BCLK; no MCLK is
/// routed to it on this board as far as the published pinout shows.
pub const PIN_MCLK: Option<u32> = None;

/// Bring up I2S TX on `i2s0` at [`OUT_RATE_HZ`], 16-bit stereo.
///
/// Philips slot format, the same `StdConfig` shape `m5stack-s3-app`
/// uses for its ES8311 playback — the codec differs, the transport does
/// not.
///
/// Refuses when [`speaker_available`] is false: without the codec's
/// registers there is nothing on the other end of the wire, and
/// bringing the peripheral up would only make it look configured.
pub fn init_i2s_tx(
    i2s0: esp_idf_hal::i2s::I2S0<'static>,
    // BCLK and WS are bidirectional in the HAL's signature (a slave
    // configuration drives them the other way), so they need
    // `InputPin + OutputPin` even though this is a TX-only master.
    bclk: impl esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin + 'static,
    dout: impl esp_idf_hal::gpio::OutputPin + 'static,
    lrck: impl esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin + 'static,
) -> Result<I2sDriver<'static, I2sTx>> {
    if !speaker_available() {
        anyhow::bail!("AW88298 init sequence is empty — not bringing up I2S TX");
    }
    use esp_idf_hal::i2s::config::{
        ClockSource, Config as I2sConfig, DataBitWidth, MclkMultiple, SlotMode, StdClkConfig,
        StdConfig, StdGpioConfig, StdSlotConfig,
    };
    let cfg = StdConfig::new(
        I2sConfig::default(),
        StdClkConfig::new(OUT_RATE_HZ, ClockSource::Pll160M, MclkMultiple::M256),
        StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
        StdGpioConfig::default(),
    );
    // No MCLK: see `PIN_MCLK`.
    Ok(I2sDriver::new_std_tx(
        i2s0,
        &cfg,
        bclk,
        dout,
        None::<esp_idf_hal::gpio::AnyIOPin>,
        lrck,
    )?)
}

/// 12 kHz mono `i16` → 48 kHz stereo, linearly interpolated, gain
/// applied, written to `i2s`.
///
/// Linear rather than zero-order hold: a 4× hold puts images at 12 and
/// 24 kHz that a small speaker makes audible as a buzz, and the
/// interpolation is three multiply-adds per output sample against an
/// I2S write that dominates either way.
///
/// `gain` is linear, `0.0..=1.0`. The caller ramps it — starting an
/// amplifier at full scale is how a demo becomes unpleasant.
pub fn play_block(
    i2s: &mut I2sDriver<'static, I2sTx>,
    mono_12k: &[i16],
    tail: &mut i16,
    gain: f32,
) -> Result<()> {
    let mut out: Vec<i16> = Vec::with_capacity(mono_12k.len() * 8);
    let mut prev = *tail;
    for &s in mono_12k {
        for k in 0..4 {
            let a = prev as f32;
            let b = s as f32;
            let v = (a + (b - a) * (k as f32 / 4.0)) * gain;
            let v = v.clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            // Stereo: the AW88298 drives one speaker, but the slot is
            // a stereo slot and a mono write would play at half rate.
            out.push(v);
            out.push(v);
        }
        prev = s;
    }
    *tail = prev;
    let bytes: &[u8] = unsafe {
        core::slice::from_raw_parts(out.as_ptr() as *const u8, core::mem::size_of_val(&out[..]))
    };
    i2s.write_all(bytes, esp_idf_hal::delay::TickType::new_millis(500).ticks())?;
    Ok(())
}
