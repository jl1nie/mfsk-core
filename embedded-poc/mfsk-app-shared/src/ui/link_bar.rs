//! Link bar — the bottom 14 px of the panel, in every mode.
//!
//! Two facts that decide whether the receiver can work at all, and
//! neither of which any screen used to carry: is there a radio on the
//! USB port, and is there a network.
//!
//! **Why it is its own bar rather than a field in each header.** The
//! FT8 controller showed USB state in a ten-line diagnostic panel and
//! the other two receivers showed none at all — so "the board is not
//! hearing the radio" looked identical in every mode to "the board is
//! working". Retiring that panel (it is bring-up instrumentation and
//! it covered the decodes) removed the last window, which is how a
//! bench session ended up asking whether VBUS had been set at all with
//! no way to answer from the screen.
//!
//! Each receiver's own header is already full at 40 characters, and
//! they disagree about what belongs in one. The bottom row is the one
//! piece of geometry all three can spare and all three can share, so
//! the same fields sit in the same place whichever mode booted.

use core::fmt::Write as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

pub const HEIGHT: u32 = 14;

/// What the USB port is doing.
///
/// `Peripheral` is not a failure: one USB-C connector cannot both take
/// power in and hand it out, so a board plugged into a PC charges and
/// stays flashable instead of running as a host. That is the state the
/// board is in on the bench, and the state that looked like a broken
/// UAC stack because nothing said so.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UsbLink {
    /// Host driver not installed *by choice* — external power is
    /// present, so the board charges and stays flashable.
    Peripheral,
    /// Host mode was chosen — VBUS was enabled — and the host driver is
    /// not up. Install pending, or it failed.
    ///
    /// This used to render as `Peripheral`, because the role was
    /// derived from "is the host installed" rather than from what the
    /// firmware decided. A board on battery trying to power a radio
    /// therefore displayed `P chg`: peripheral, charging. Both halves
    /// false, and it hid the actual fault.
    NoHost,
    /// Host installed, VBUS up, nothing enumerated yet.
    Waiting,
    /// A device is enumerated and audio is flowing.
    Streaming,
    /// The host stack reported an error.
    Error,
}

impl UsbLink {
    /// Host or peripheral, in one character. The word cost six and
    /// said the least interesting thing on the line — what the port is
    /// *doing* is the state below, and the volts are what say whether
    /// it can.
    fn role(self) -> &'static str {
        match self {
            UsbLink::Peripheral => "P",
            _ => "H",
        }
    }

    fn is_fault(self) -> bool {
        matches!(self, UsbLink::NoHost | UsbLink::Error)
    }

    /// One six-character state name, same case, same width, for every
    /// state.
    ///
    /// They used to be `chg`, `no dev`, `NOHOST`, `STREAM`, `ERROR ` —
    /// three casings and two naming styles across five values on one
    /// line, which reads as unrelated fields rather than one state
    /// machine. Fixed width also keeps everything to the right of it
    /// from shifting as the state changes.
    fn label(self) -> &'static str {
        match self {
            // Not "IDLE": the board is doing something useful, and the
            // battery volts two fields along are the reason to care.
            UsbLink::Peripheral => "CHARGE",
            UsbLink::NoHost => "NOHOST",
            UsbLink::Waiting => "NODEV ",
            UsbLink::Streaming => "STREAM",
            UsbLink::Error => "ERROR ",
        }
    }

    fn color(self) -> Rgb565 {
        // Same three roles `decoded_list` already uses on this panel:
        // GREEN for live, CSS_ORANGE for actionable, WHITE for inert.
        match self {
            UsbLink::Streaming => Rgb565::GREEN,
            UsbLink::Error | UsbLink::NoHost => Rgb565::CSS_ORANGE,
            _ => Rgb565::WHITE,
        }
    }
}

/// Everything the bar draws. Cheap to build once per frame.
#[derive(Clone, Copy, Debug)]
pub struct LinkInfo {
    pub usb: UsbLink,
    /// Devices the host stack currently has open. Zero with
    /// [`UsbLink::Waiting`] is the "radio not seen" case.
    pub devices: u8,
    /// `None` when the station is not associated.
    pub wifi_rssi: Option<i8>,
    /// Raw AW9523B enable bits, so "is VBUS actually set" is answerable
    /// from the screen rather than from a serial console that host mode
    /// takes away. `None` on boards that never tried.
    ///
    /// **All three**, in the order `BOOST_EN`, `USB_OTG_EN`,
    /// `BUS_OUT_EN`. #163 established that the IC-705 sees VBUS only
    /// with every one of them high, and `BUS_OUT_EN` is the one that
    /// sounds unrelated — showing the first two alone reads as "VBUS is
    /// set" while the load-bearing bit is unaccounted for.
    pub vbus: Option<(bool, bool, bool)>,
    /// False once the I/O expander those bits live in has stopped
    /// answering. The bits are then a memory, not a measurement, and
    /// the bar says so rather than repeating them.
    pub expander_ok: bool,
    /// Cell voltage in millivolts. The 5 V boost that feeds the radio
    /// comes out of this, so it is the number that explains a port
    /// which is enabled and still delivering nothing.
    pub battery_mv: u16,
    /// VBUS at the connector, in millivolts.
    ///
    /// Trustworthy only while something else is supplying the port. The
    /// AXP2101's VBUS ADC cannot see this board's own boost output — it
    /// rails to full scale in host mode even with VBUS definitely
    /// present (#163, measured) — so anything implausible renders as
    /// `----` rather than as a number that would be read as a
    /// measurement.
    pub vbus_mv: u16,
    /// Whether the system clock is set — i.e. whether NTP (or an RTC)
    /// has run.
    ///
    /// Every receiver here anchors its slot grid to UTC and none of
    /// them can decode without it: FT8 tolerates ±2.5 s against a 15 s
    /// grid, so an unset clock means a free-running phase and no
    /// decodes at all, with a waterfall full of signal. That failure
    /// looks exactly like a broken decoder from every other indicator
    /// on the screen, which is how it cost an evening.
    pub clock_set: bool,
}

/// Draw the bar at `origin_y`, spanning `width`.
pub fn render<D>(
    display: &mut D,
    info: &LinkInfo,
    width: u32,
    origin_y: i32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let bg = Rgb565::new(0, 8, 0);
    let style = |fg: Rgb565| {
        MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(fg)
            .background_color(bg)
            .build()
    };

    Rectangle::new(Point::new(0, origin_y), Size::new(width, HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(bg))
        .draw(display)?;

    // "USB " then the state in its own colour, so the one field worth
    // glancing at reads without parsing the line.
    let y = origin_y + 2;
    let mut head: String<8> = String::new();
    let _ = write!(&mut head, "USB {} ", info.usb.role());
    Text::with_baseline(
        head.as_str(),
        Point::new(2, y),
        style(Rgb565::CSS_GRAY),
        Baseline::Top,
    )
    .draw(display)?;
    Text::with_baseline(
        info.usb.label(),
        Point::new(2 + 6 * 6, y),
        style(info.usb.color()),
        Baseline::Top,
    )
    .draw(display)?;

    let mut rest: String<40> = String::new();
    let _ = write!(&mut rest, " d{}", info.devices);
    match info.vbus {
        _ if !info.expander_ok => {
            // Not "unknown" — a fault. The expander answered once and
            // has stopped, which on this board means its rail went
            // down under the boost's load.
            let _ = rest.push_str(" V!!!");
        }
        // The boost converter, the switch that gates it onto the
        // connector, and the external rail — all three high or nothing
        // enumerates.
        Some((boost, otg, bus)) => {
            let _ = write!(
                &mut rest,
                " V{}{}{}",
                u8::from(boost),
                u8::from(otg),
                u8::from(bus)
            );
        }
        None => {
            let _ = rest.push_str(" V---");
        }
    }
    // Anything above ~6 V is the ADC railed, not a reading.
    match info.vbus_mv {
        1..=6000 => {
            let _ = write!(
                &mut rest,
                " v{}.{:02}",
                info.vbus_mv / 1000,
                (info.vbus_mv % 1000) / 10
            );
        }
        _ => {
            let _ = rest.push_str(" v----");
        }
    }
    if info.battery_mv > 0 {
        let _ = write!(
            &mut rest,
            " b{}.{:02}",
            info.battery_mv / 1000,
            (info.battery_mv % 1000) / 10
        );
    }
    // ` W-52`, not `  WiFi -52dBm`. With the volts on the line there
    // is no room for the unit, and a two-digit negative number after a
    // W is unambiguous.
    let _ = rest.push_str(" W");
    match info.wifi_rssi {
        Some(dbm) => {
            let _ = write!(&mut rest, "{dbm}");
        }
        None => {
            let _ = rest.push_str("down");
        }
    }
    // One character, because there is room for one: `T` when the clock
    // is set, `-` when it is not. The receivers' own headers carry the
    // time itself; this says whether that time means anything.
    let _ = rest.push_str(if info.clock_set { " T" } else { " -" });
    let wifi_fg = if info.wifi_rssi.is_some() {
        Rgb565::WHITE
    } else {
        Rgb565::CSS_GRAY
    };
    Text::with_baseline(
        rest.as_str(),
        Point::new(2 + 12 * 6, y),
        style(wifi_fg),
        Baseline::Top,
    )
    .draw(display)?;
    Ok(())
}
