//! FT6336U capacitive touch on CoreS3 (Phase 6-Core, first bring-up).
//!
//! The panel shares I2C0 with the AXP2101 and the AW9523B, at address
//! `board::TOUCH_I2C_ADDR` (0x38). `pmic::init` already pulses TP_RST
//! (AW9523B P1_0) low and releases it, so the controller is out of
//! reset by the time anything here runs — nothing else was needed to
//! make it answer.
//!
//! ## This is a probe, not a driver yet
//!
//! It reads coordinates and says what it saw. No gestures, no widget
//! hit-testing, no debounce beyond "report only on change". The point
//! of the first flash is to answer three questions that cannot be
//! answered from a datasheet: does the part respond at all, what
//! coordinate convention does this panel report in (the controller's
//! axes and the LCD's do not have to agree, and M5's own boards differ
//! between models), and — the one that matters here — does polling it
//! destabilise anything.
//!
//! ## Why that last question is not paranoia
//!
//! Reading this panel means an I2C transaction from the render loop,
//! and that exact pattern took the board down once already: polling
//! `pmic::vbus_mv` from `display::run_log_panel` aborted inside
//! `i2c_cmd_link_delete` about a second after audio started, because
//! the esp-idf I2C driver allocates a command link per transaction and
//! internal DRAM had run out (#163). It is the reason `_pmic_i2c` has
//! been held and deliberately unused since.
//!
//! Conditions are better now — that session ended with ~22 KB of
//! internal DRAM free, and today's build has 51-78 KB depending on
//! mode — but "better" is not "measured". Hence the low poll rate, the
//! change-only logging, and the free-memory readout alongside the
//! first reads.

use esp_idf_hal::i2c::I2cDriver;

use crate::board::TOUCH_I2C_ADDR;

/// Registers, named as LovyanGFX's `Touch_FT5x06.cpp` names them.
const FT5X06_CIPHER_REG: u8 = 0xA3;
const FT5X06_INTMODE_REG: u8 = 0xA4;
/// Number of points currently down, in the low nibble.
const REG_TD_STATUS: u8 = 0x02;

/// One contact, in screen coordinates.
///
/// M5GFX configures this panel as `x_min=0, x_max=319, y_min=0,
/// y_max=239` and measurement agrees — a touch at the bottom-right
/// corner reads (319, 239). No rotation or scaling is needed.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Contact {
    pub points: u8,
    pub x: u16,
    pub y: u16,
}

fn write_reg(i2c: &mut I2cDriver<'_>, reg: u8, val: u8) -> bool {
    i2c.write(TOUCH_I2C_ADDR, &[reg, val], 100).is_ok()
}

/// Bring the controller up the way `Touch_FT5x06::_check_init` does.
///
/// ```c
/// _inited = _write_reg(0x00, 0x00)
///       && _read_reg(FT5x06_CIPHER_REG, tmp, 6)
///       && _write_reg(FT5x06_INTMODE_REG, 0x00) // INT Polling mode
///       && tmp[5];
/// ```
///
/// Two things this settles that a first attempt here got wrong: the
/// six-byte block from 0xA3 puts the vendor id at `tmp[5]` (0xA8), and
/// the only test LovyanGFX applies to it is **non-zero** — the
/// comparison against 0x11 lives in M5GFX's panel-variant detection,
/// which logs a warning and carries on when it fails. This board reads
/// 0x01 there, which is therefore fine.
///
/// The INT-polling write is the part that was missing entirely.
pub fn init(i2c: &mut I2cDriver<'_>) -> bool {
    if !write_reg(i2c, 0x00, 0x00) {
        log::warn!("touch: no answer at 0x{TOUCH_I2C_ADDR:02x}");
        return false;
    }
    let mut tmp = [0u8; 6];
    if i2c
        .write_read(TOUCH_I2C_ADDR, &[FT5X06_CIPHER_REG], &mut tmp, 100)
        .is_err()
    {
        log::warn!("touch: version block read failed");
        return false;
    }
    if !write_reg(i2c, FT5X06_INTMODE_REG, 0x00) {
        log::warn!("touch: INT polling mode write failed");
        return false;
    }
    if tmp[5] == 0 {
        log::warn!("touch: vendor id reads 0 — treating as absent, as LovyanGFX does");
        return false;
    }
    log::info!(
        "touch: FT5x06 at 0x{TOUCH_I2C_ADDR:02x} — CIPHER:0x{:02x} FIRMID:0x{:02x} VENDID:0x{:02x}",
        tmp[0],
        tmp[3],
        tmp[5]
    );
    true
}

/// One read of the contact block: status byte, then `points * 6 - 2`
/// more. Returns the raw bytes and how many were read.
fn read_raw(i2c: &mut I2cDriver<'_>, out: &mut [u8; 30]) -> usize {
    let mut status = [0u8; 1];
    if i2c
        .write_read(TOUCH_I2C_ADDR, &[REG_TD_STATUS], &mut status, 100)
        .is_err()
    {
        return 0;
    }
    out[0] = status[0];
    let points = (status[0] & 0x0F) as usize;
    if points == 0 {
        return 1;
    }
    let want = points * 6 - 2;
    if want > out.len() - 1 {
        return 1;
    }
    // The rest of the block follows the status byte, so read it from
    // 0x03 rather than reopening at 0x02.
    if i2c
        .write_read(TOUCH_I2C_ADDR, &[0x03], &mut out[1..1 + want], 100)
        .is_err()
    {
        return 0;
    }
    points * 6 - 1
}

/// Current contact.
///
/// Read twice and compare, up to five times, because LovyanGFX does
/// and says why: 「読出し中に値が変わる事があるので、連続読出しして
/// 前回と同値でなければリトライする」. A single read can catch the
/// controller mid-update and return a coordinate that was never
/// touched.
pub fn read(i2c: &mut I2cDriver<'_>) -> Option<Contact> {
    let mut a = [0u8; 30];
    let mut b = [0u8; 30];
    let mut len_a = read_raw(i2c, &mut a);
    if len_a == 0 {
        return None;
    }
    if len_a == 1 {
        return Some(Contact::default());
    }
    for _ in 0..5 {
        let len_b = read_raw(i2c, &mut b);
        if len_b == len_a && a[..len_a] == b[..len_b] {
            break;
        }
        a = b;
        len_a = len_b;
        if len_a == 0 {
            return None;
        }
        if len_a == 1 {
            return Some(Contact::default());
        }
    }
    let points = a[0] & 0x0F;
    if points == 0 {
        return Some(Contact::default());
    }
    // `data[1]`/`data[2]` are XH/XL and `data[3]`/`data[4]` YH/YL,
    // counting from the status byte — the same indices LovyanGFX uses.
    Some(Contact {
        points,
        x: (((a[1] & 0x0F) as u16) << 8) | a[2] as u16,
        y: (((a[3] & 0x0F) as u16) << 8) | a[4] as u16,
    })
}
