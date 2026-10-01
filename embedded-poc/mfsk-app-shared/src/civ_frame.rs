//! Icom CI-V framing — building the frames the controller sends and
//! reading the ones the radio sends back, independent of the wire.
//!
//! The M5StickS3 reaches its IC-705 over BLE (`m5stack-s3-app`'s
//! `civ.rs`), the CoreS3 over the USB CDC port the radio exposes next
//! to its audio interface. Both carry the same bytes, so the bytes live
//! here where the host can test them.
//!
//! A frame is `FE FE <to> <from> <cmd> [<data>…] FD`. Opcodes and their
//! layouts follow hamlib's Icom backend (`rigs/icom/icom_defs.h`,
//! `icom.c`), which is what WSJT-X drives an IC-705 through:
//!
//! | op | direction | data |
//! |---|---|---|
//! | `00` | radio → all (`to = 00`) | transceive: the dial moved, 5-byte BCD |
//! | `01` | radio → all | transceive: the mode changed, `<mode> <filter>` |
//! | `03` | ctrl → radio, and its reply | read frequency; reply is 5-byte BCD |
//! | `04` | ctrl → radio, and its reply | read mode; reply `<mode> <filter>` |
//! | `05` | ctrl → radio | set frequency, 5-byte BCD |
//! | `1C 00` | ctrl → radio | PTT, `00`/`01` |
//! | `26` | ctrl → radio, and its reply | selected-VFO mode *with* the data flag: `<vfo> <mode> <data> <filter>` |
//! | `FB` / `FA` | radio → ctrl | OK / NG |
//!
//! **USB-DATA is `26 00 01 01 01`, not `06 01 01 02`.** The latter is
//! what the StickS3's BLE path shipped with; `06` takes a mode and an
//! optional filter, so the trailing byte made it a malformed frame the
//! radio answers with NG. hamlib sets PKTUSB on the IC-705
//! (`x25x26_always`, `data_mode_supported`) through `26`.
//!
//! **A controller can see its own frames.** CI-V is a bus; hamlib marks
//! the IC-705's USB port `serial_USB_echo_check` because an echo may or
//! may not come back. [`Reader`] therefore keeps only frames *from* the
//! radio, addressed to the controller or broadcast.

use heapless::Vec;

pub const PRE: u8 = 0xFE;
pub const POST: u8 = 0xFD;
/// The controller's address — hamlib's default, and what the IC-705
/// expects unless its menu was changed.
pub const CTRL_ADDR: u8 = 0xE0;
/// IC-705 factory default. **Not** 0x94, which is the IC-7300's.
pub const IC705_ADDR: u8 = 0xA4;
/// Transceive frames are addressed to everyone.
pub const BROADCAST: u8 = 0x00;

pub const OK: u8 = 0xFB;
pub const NG: u8 = 0xFA;

/// Longest frame this module builds or accepts. The IC-705's GPS reply
/// (`23 00`, 27 data bytes) is the longest thing either end sends here.
pub const MAX_FRAME: usize = 40;

pub type Frame = Vec<u8, MAX_FRAME>;

/// Icom mode byte for USB (`icom_defs.h` `S_USB`).
pub const MODE_USB: u8 = 0x01;

/// Encode `hz` as the 5-byte little-endian BCD of ops `00`/`03`/`05`:
/// byte 0 holds the 10 Hz and 1 Hz digits, byte 4 the 1 GHz and
/// 100 MHz ones.
pub fn freq_to_bcd_le(hz: u32) -> [u8; 5] {
    let mut bcd = [0u8; 5];
    let mut f = hz;
    for byte in &mut bcd {
        let lo = (f % 10) as u8;
        f /= 10;
        let hi = (f % 10) as u8;
        f /= 10;
        *byte = (hi << 4) | lo;
    }
    bcd
}

/// The inverse of [`freq_to_bcd_le`]. `None` if a nibble is not a
/// decimal digit, which is what a torn or misaligned frame looks like.
pub fn bcd_le_to_freq(bcd: &[u8]) -> Option<u32> {
    let mut hz: u64 = 0;
    for &b in bcd.iter().rev() {
        let (hi, lo) = (b >> 4, b & 0x0F);
        if hi > 9 || lo > 9 {
            return None;
        }
        hz = hz * 100 + (hi as u64) * 10 + lo as u64;
    }
    u32::try_from(hz).ok()
}

/// `FE FE <to> E0 <body…> FD`.
pub fn build(to: u8, body: &[u8]) -> Frame {
    let mut f = Frame::new();
    // Every builder below passes a body well under MAX_FRAME - 5.
    let _ = f.extend_from_slice(&[PRE, PRE, to, CTRL_ADDR]);
    let _ = f.extend_from_slice(body);
    let _ = f.push(POST);
    f
}

pub fn read_freq(to: u8) -> Frame {
    build(to, &[0x03])
}

pub fn read_mode(to: u8) -> Frame {
    // `26 00`: the selected VFO's mode, data flag included.
    build(to, &[0x26, 0x00])
}

pub fn set_freq(to: u8, hz: u32) -> Frame {
    let bcd = freq_to_bcd_le(hz);
    let mut body = [0x05, 0, 0, 0, 0, 0];
    body[1..].copy_from_slice(&bcd);
    build(to, &body)
}

/// USB with the data flag on, filter 1 — what WSJT-X asks hamlib for
/// ("Data/Pkt" mode) when it drives an IC-705.
pub fn set_mode_usb_data(to: u8) -> Frame {
    build(to, &[0x26, 0x00, MODE_USB, 0x01, 0x01])
}

pub fn ptt(to: u8, on: bool) -> Frame {
    build(to, &[0x1C, 0x00, on as u8])
}

/// `1C 00` with no data: ask the radio whether it is transmitting. The
/// reply parses as [`Event::Ptt`] — the readback that says a `ptt`
/// command took effect, where the `FB` after it only says it was
/// understood.
pub fn read_ptt(to: u8) -> Frame {
    build(to, &[0x1C, 0x00])
}

pub fn gps_query(to: u8) -> Frame {
    build(to, &[0x23, 0x00])
}

/// What a frame from the radio said, as far as this controller cares.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The dial frequency — a reply to `03` or a transceive `00`.
    Freq(u32),
    /// The operating mode — a reply to `04`/`26` or a transceive `01`.
    /// `data` is `None` where the frame does not carry the flag
    /// (`01`/`04` are the legacy layout).
    Mode {
        mode: u8,
        data: Option<bool>,
        filter: u8,
    },
    /// PTT state, echoed by the radio in reply to a `1C 00` read.
    Ptt(bool),
    Ok,
    Ng,
    /// A well-formed frame from the radio that none of the above covers
    /// (e.g. the GPS reply). The opcode, for logging.
    Other(u8),
}

/// Interpret one complete frame. `None` for a frame not from `radio`,
/// not addressed to the controller or to everyone, or malformed.
pub fn parse(frame: &[u8], radio: u8) -> Option<Event> {
    let n = frame.len();
    if n < 6 || frame[0] != PRE || frame[1] != PRE || frame[n - 1] != POST {
        return None;
    }
    let (to, from) = (frame[2], frame[3]);
    if from != radio || (to != CTRL_ADDR && to != BROADCAST) {
        return None;
    }
    let cmd = frame[4];
    let data = &frame[5..n - 1];
    Some(match cmd {
        0x00 | 0x03 => Event::Freq(bcd_le_to_freq(data.get(..5)?)?),
        0x01 | 0x04 => Event::Mode {
            mode: *data.first()?,
            data: None,
            filter: data.get(1).copied().unwrap_or(0),
        },
        0x26 => Event::Mode {
            mode: *data.get(1)?,
            data: Some(*data.get(2)? != 0),
            filter: data.get(3).copied().unwrap_or(0),
        },
        0x1C if data.first() == Some(&0x00) && data.len() == 2 => Event::Ptt(data[1] != 0),
        OK => Event::Ok,
        NG => Event::Ng,
        other => Event::Other(other),
    })
}

/// Reassembles frames from a byte stream. A USB bulk transfer or a BLE
/// notification is not guaranteed to carry exactly one frame, so bytes
/// go in as they arrive and whole frames come out.
pub struct Reader {
    buf: Frame,
}

impl Default for Reader {
    fn default() -> Self {
        Self::new()
    }
}

impl Reader {
    pub const fn new() -> Self {
        Self { buf: Vec::new() }
    }

    /// Feed one byte; returns a complete `FE FE … FD` frame when this
    /// byte finishes one. Anything before a preamble is dropped, and a
    /// frame longer than [`MAX_FRAME`] is discarded rather than wrapped.
    pub fn push(&mut self, b: u8) -> Option<Frame> {
        if self.buf.is_empty() {
            if b == PRE {
                let _ = self.buf.push(b);
            }
            return None;
        }
        if self.buf.len() == 1 {
            if b == PRE {
                let _ = self.buf.push(b);
            } else {
                self.buf.clear();
            }
            return None;
        }
        // A third FE is a repeated preamble (Icom radios may send
        // several); stay aligned on the last two.
        if self.buf.len() == 2 && b == PRE {
            return None;
        }
        if self.buf.push(b).is_err() {
            self.buf.clear();
            return None;
        }
        if b == POST {
            let out = self.buf.clone();
            self.buf.clear();
            return Some(out);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const R: u8 = IC705_ADDR;

    fn from_radio(to: u8, body: &[u8]) -> Frame {
        let mut f = Frame::new();
        f.extend_from_slice(&[PRE, PRE, to, R]).unwrap();
        f.extend_from_slice(body).unwrap();
        f.push(POST).unwrap();
        f
    }

    #[test]
    fn bcd_known_values() {
        assert_eq!(freq_to_bcd_le(14_074_000), [0x00, 0x40, 0x07, 0x14, 0x00]);
        assert_eq!(freq_to_bcd_le(7_074_000), [0x00, 0x40, 0x07, 0x07, 0x00]);
        assert_eq!(freq_to_bcd_le(144_174_000), [0x00, 0x40, 0x17, 0x44, 0x01]);
    }

    #[test]
    fn bcd_round_trips_every_ft8_dial() {
        for hz in [
            1_840_000,
            3_573_000,
            7_074_000,
            10_136_000,
            14_074_000,
            18_100_000,
            21_074_000,
            24_915_000,
            28_074_000,
            50_313_000,
            144_174_000,
            430_000_000,
            123_456_789,
        ] {
            assert_eq!(bcd_le_to_freq(&freq_to_bcd_le(hz)), Some(hz), "{hz}");
        }
    }

    #[test]
    fn bcd_rejects_non_decimal_nibbles() {
        assert_eq!(bcd_le_to_freq(&[0x0A, 0, 0, 0, 0]), None);
        assert_eq!(bcd_le_to_freq(&[0, 0, 0, 0, 0xF0]), None);
    }

    #[test]
    fn builders_match_hamlib_layouts() {
        assert_eq!(
            read_freq(R).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x03, 0xFD]
        );
        assert_eq!(
            set_freq(R, 14_074_000).as_slice(),
            &[
                0xFE, 0xFE, 0xA4, 0xE0, 0x05, 0x00, 0x40, 0x07, 0x14, 0x00, 0xFD
            ]
        );
        assert_eq!(
            set_mode_usb_data(R).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x26, 0x00, 0x01, 0x01, 0x01, 0xFD]
        );
        assert_eq!(
            ptt(R, true).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x1C, 0x00, 0x01, 0xFD]
        );
        assert_eq!(
            ptt(R, false).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x1C, 0x00, 0x00, 0xFD]
        );
        assert_eq!(
            read_ptt(R).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x1C, 0x00, 0xFD]
        );
        assert_eq!(
            read_mode(R).as_slice(),
            &[0xFE, 0xFE, 0xA4, 0xE0, 0x26, 0x00, 0xFD]
        );
    }

    #[test]
    fn transceive_and_reply_frequencies() {
        let bcd = freq_to_bcd_le(7_074_000);
        let mut body = [0u8; 6];
        body[1..].copy_from_slice(&bcd);
        body[0] = 0x00;
        assert_eq!(
            parse(&from_radio(BROADCAST, &body), R),
            Some(Event::Freq(7_074_000))
        );
        body[0] = 0x03;
        assert_eq!(
            parse(&from_radio(CTRL_ADDR, &body), R),
            Some(Event::Freq(7_074_000))
        );
    }

    #[test]
    fn modes_in_both_layouts() {
        assert_eq!(
            parse(&from_radio(CTRL_ADDR, &[0x26, 0x00, 0x01, 0x01, 0x01]), R),
            Some(Event::Mode {
                mode: MODE_USB,
                data: Some(true),
                filter: 1
            })
        );
        assert_eq!(
            parse(&from_radio(BROADCAST, &[0x01, 0x01, 0x02]), R),
            Some(Event::Mode {
                mode: MODE_USB,
                data: None,
                filter: 2
            })
        );
    }

    #[test]
    fn ok_ng_and_ptt() {
        assert_eq!(parse(&from_radio(CTRL_ADDR, &[OK]), R), Some(Event::Ok));
        assert_eq!(parse(&from_radio(CTRL_ADDR, &[NG]), R), Some(Event::Ng));
        assert_eq!(
            parse(&from_radio(CTRL_ADDR, &[0x1C, 0x00, 0x01]), R),
            Some(Event::Ptt(true))
        );
    }

    /// Our own frame coming back must not read as the radio's answer —
    /// a `05` echo would otherwise be taken for a frequency report.
    #[test]
    fn own_echo_and_other_radios_are_ignored() {
        assert_eq!(parse(&set_freq(R, 14_074_000), R), None);
        let mut other = from_radio(CTRL_ADDR, &[OK]);
        other[3] = 0x94; // an IC-7300 on the same bus
        assert_eq!(parse(&other, R), None);
        let mut to_someone_else = from_radio(0x70, &[OK]);
        to_someone_else[2] = 0x70;
        assert_eq!(parse(&to_someone_else, R), None);
    }

    #[test]
    fn torn_frames_are_rejected() {
        assert_eq!(
            parse(&[0xFE, 0xFE, 0xE0, 0xA4, 0x03, 0x00, 0x40, 0xFD], R),
            None
        );
        assert_eq!(parse(&[0xFE, 0xFE, 0xE0, 0xA4, 0xFB], R), None);
    }

    #[test]
    fn reader_splits_a_stream_and_resyncs() {
        let mut stream: std::vec::Vec<u8> = vec![0x12, 0xFD, 0xFE]; // garbage, then a lone FE
        stream.extend_from_slice(&[0x33]); // FE followed by non-FE: drop
        stream.extend_from_slice(&[0xFE, 0xFE, 0xFE]); // repeated preamble
        stream.extend_from_slice(&[0x00, 0xA4, 0x00, 0x00, 0x40, 0x07, 0x14, 0x00, 0xFD]);
        stream.extend_from_slice(&from_radio(CTRL_ADDR, &[OK]));
        let mut r = Reader::new();
        let frames: std::vec::Vec<Frame> = stream.iter().filter_map(|&b| r.push(b)).collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(parse(&frames[0], R), Some(Event::Freq(14_074_000)));
        assert_eq!(parse(&frames[1], R), Some(Event::Ok));
    }

    #[test]
    fn reader_discards_an_overlong_frame() {
        let mut r = Reader::new();
        let mut got = 0;
        for b in [PRE, PRE]
            .into_iter()
            .chain(core::iter::repeat_n(0x11, MAX_FRAME + 4))
        {
            got += r.push(b).is_some() as u32;
        }
        assert_eq!(got, 0);
        for &b in from_radio(CTRL_ADDR, &[OK]).iter() {
            if let Some(f) = r.push(b) {
                assert_eq!(parse(&f, R), Some(Event::Ok));
                got += 1;
            }
        }
        assert_eq!(got, 1);
    }
}
