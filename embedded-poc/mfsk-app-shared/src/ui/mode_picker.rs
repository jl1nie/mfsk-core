//! Mode picker — the way back out of whichever receiver is running.
//!
//! The CoreS3 app is one binary carrying three receivers, chosen at
//! boot from the NVS `boot_mode`. That only helps if the choice can be
//! changed from the running app: the board has no buttons, and in UAC
//! host mode it has no serial console either, because the USB host
//! driver takes the port a flasher would use.
//!
//! **Why this is shared rather than per-app.** The first version lived
//! in the FT8 controller's render loop alone, which made switching a
//! one-way trip — WSPR and FST4 draw their own screens and had no way
//! back, so choosing one meant re-flashing or erasing NVS to undo it.
//! That is the situation the single binary exists to end. Every
//! receiver draws this.
//!
//! ## Select, then confirm — on two different controls
//!
//! Tapping a mode selects it. Committing is a separate bar underneath
//! that names what it will do ("SWITCH TO WSPR"). Two controls, not
//! two taps on one.
//!
//! The first version asked for a second tap on the same button and
//! replaced its label with "tap again" to say so. That deleted the one
//! piece of information the confirmation existed to confirm — which
//! mode had been chosen — and, because the screen otherwise looked
//! unchanged, was indistinguishable from a tap that had done nothing.
//! Reported from the bench as the menu simply not working.
//!
//! Confirmation is still required: a stray touch during a capture
//! session must not reboot the receiver mid-slot. It just does not
//! cost the label to ask for it.
//!
//! ## Held open, not always on screen
//!
//! WSPR's spot lists use 312 of 320 vertical pixels and FST4's screen
//! is as full; there is no column to give a permanent widget. So it is
//! an overlay: hold a finger anywhere for [`OPEN_MS`] and it appears,
//! tap a mode to arm, tap it again to commit, tap outside to dismiss.
//! Nothing is reserved when it is closed.
//!
//! On dismissal the app has to repaint whatever the overlay covered —
//! [`ModePicker::take_just_closed`] says when.

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

use crate::boot_mode::BootMode;

/// The receivers this binary can boot into, in draw order.
pub const MODES: [(BootMode, &str); 4] = [
    (BootMode::Uac, "FT8 / UAC"),
    (BootMode::Wspr, "WSPR"),
    (BootMode::Fst4, "FST4"),
    (BootMode::Decode, "DECODE (wav)"),
];

pub const WIDTH: u32 = 208;
/// Drawn height of one button.
///
/// 40 px, not the 24 it started at. At 24 with a 26 px pitch the four
/// buttons spanned 104 px and a fingertip covered more than one — the
/// bench report was that hitting FT8 or WSPR was hard and taps kept
/// landing on a neighbour, which reads as "tap again" appearing on the
/// wrong row. A finger contact is several millimetres across; the
/// panel is 240x320 and can afford this.
pub const BUTTON_H: u32 = 40;
/// Distance between button tops. The gap is drawn but not dead — the
/// hit test below covers it, so nothing lands between two buttons.
pub const PITCH: i32 = 48;

/// How long a selection stands before it lapses.
pub const ARM_MS: u64 = 8_000;

/// How long a finger must stay down to summon the picker. Long enough
/// not to fire while someone is using the screen for something else.
pub const OPEN_MS: u64 = 800;

/// Height of the commit bar under the list.
pub const COMMIT_H: u32 = 40;

/// Total height: the mode rows plus the commit bar.
pub const fn height() -> u32 {
    (MODES.len() as i32 * PITCH) as u32 + COMMIT_H
}

/// Top of the commit bar, relative to the widget origin.
const fn commit_top() -> i32 {
    MODES.len() as i32 * PITCH
}

/// What a touch landed on.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Target {
    Mode(usize),
    Commit,
}

/// Bands are contiguous — a press in the drawn gap between two rows
/// goes to the nearer one rather than nowhere. Taps were landing on
/// those edges and doing nothing, which reads as a dead panel rather
/// than a near miss.
fn hit(origin: Point, x: u16, y: u16) -> Option<Target> {
    let (x, y) = (x as i32, y as i32);
    if x < origin.x || x >= origin.x + WIDTH as i32 {
        return None;
    }
    let dy = y - origin.y;
    if dy < 0 || dy >= height() as i32 {
        return None;
    }
    if dy >= commit_top() {
        Some(Target::Commit)
    } else {
        Some(Target::Mode((dy / PITCH) as usize))
    }
}

/// Draw + two-tap state. Owns the commit policy so it cannot drift
/// between the three receivers.
pub struct ModePicker {
    origin: Point,
    open: bool,
    just_closed: bool,
    /// When the current press began, for the hold-to-open gesture.
    press_since: Option<std::time::Instant>,
    armed: Option<(usize, std::time::Instant)>,
    drawn: Option<usize>,
    needs_draw: bool,
}

impl ModePicker {
    pub fn new(origin: Point) -> Self {
        Self {
            origin,
            open: false,
            just_closed: false,
            press_since: None,
            armed: None,
            drawn: None,
            needs_draw: false,
        }
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    /// True once after the overlay closes, so the caller can repaint
    /// what it covered. Clears on read.
    pub fn take_just_closed(&mut self) -> bool {
        core::mem::take(&mut self.just_closed)
    }

    /// Feed the touch state once per render frame. `Some(mode)` when
    /// the commit bar is pressed with a selection standing — the
    /// caller writes NVS and restarts.
    ///
    /// `pressed` is the current contact state; `x`/`y` are only read
    /// while it is true.
    pub fn update(&mut self, pressed: bool, x: u16, y: u16) -> Option<BootMode> {
        let now = std::time::Instant::now();
        let was_pressed = self.press_since.is_some();

        if !pressed {
            self.press_since = None;
        } else if !was_pressed {
            self.press_since = Some(now);
            if self.open {
                match hit(self.origin, x, y) {
                    Some(Target::Mode(idx)) => {
                        // Selecting only selects. The label stays
                        // readable; the commit bar below says what
                        // pressing it will do.
                        self.armed = Some((idx, now));
                        self.needs_draw = true;
                    }
                    Some(Target::Commit) => {
                        if let Some((idx, _)) = self.armed {
                            return Some(MODES[idx].0);
                        }
                    }
                    None => self.close(),
                }
            }
        } else if !self.open {
            if let Some(since) = self.press_since {
                if now.duration_since(since).as_millis() as u64 >= OPEN_MS {
                    self.open = true;
                    self.armed = None;
                    self.needs_draw = true;
                }
            }
        }

        // A selection left alone lapses, so a half-finished choice does
        // not sit there waiting to be committed by the next stray tap.
        if let Some((_, at)) = self.armed {
            if at.elapsed().as_millis() as u64 > ARM_MS {
                self.armed = None;
                self.needs_draw = true;
            }
        }
        None
    }

    fn close(&mut self) {
        self.open = false;
        self.armed = None;
        self.drawn = None;
        self.needs_draw = false;
        self.just_closed = true;
    }

    /// Draw, but only while open and only when something changed.
    pub fn render<D>(&mut self, display: &mut D, current: BootMode) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        if !self.open {
            return Ok(());
        }
        let armed_idx = self.armed.map(|(i, _)| i);
        if !self.needs_draw && self.drawn == armed_idx {
            return Ok(());
        }
        let text = |fg: Rgb565, bg: Rgb565| {
            MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(fg)
                .background_color(bg)
                .build()
        };

        for (i, (target, label)) in MODES.iter().enumerate() {
            let top = self.origin.y + i as i32 * PITCH;
            let selected = armed_idx == Some(i);
            // Green from `decoded_list`'s palette — already known to
            // render as green on this panel.
            let (bg, fg) = if selected {
                (Rgb565::GREEN, Rgb565::BLACK)
            } else {
                (Rgb565::BLACK, Rgb565::WHITE)
            };
            Rectangle::new(Point::new(self.origin.x, top), Size::new(WIDTH, BUTTON_H))
                .into_styled(PrimitiveStyle::with_fill(bg))
                .draw(display)?;
            // The name always stays. It is the thing a confirmation
            // step exists to let you check.
            Text::with_baseline(
                label,
                Point::new(self.origin.x + 10, top + (BUTTON_H as i32 - 10) / 2),
                text(fg, bg),
                Baseline::Top,
            )
            .draw(display)?;
            // Where this boot already is.
            if *target == current {
                Text::with_baseline(
                    "*",
                    Point::new(self.origin.x + WIDTH as i32 - 14, top + (BUTTON_H as i32 - 10) / 2),
                    text(fg, bg),
                    Baseline::Top,
                )
                .draw(display)?;
            }
        }

        // The commit bar names what it will do, so nothing depends on
        // remembering which row was tapped.
        let ctop = self.origin.y + commit_top();
        let (cbg, cfg) = match armed_idx {
            Some(_) => (Rgb565::new(0, 24, 0), Rgb565::WHITE),
            None => (Rgb565::BLACK, Rgb565::CSS_GRAY),
        };
        Rectangle::new(Point::new(self.origin.x, ctop), Size::new(WIDTH, COMMIT_H))
            .into_styled(PrimitiveStyle::with_fill(cbg))
            .draw(display)?;
        let mut line: heapless::String<32> = heapless::String::new();
        match armed_idx {
            Some(i) => {
                let _ = line.push_str("SWITCH TO ");
                let _ = line.push_str(MODES[i].1);
            }
            None => {
                let _ = line.push_str("pick a mode above");
            }
        }
        Text::with_baseline(
            line.as_str(),
            Point::new(self.origin.x + 10, ctop + (COMMIT_H as i32 - 10) / 2),
            text(cfg, cbg),
            Baseline::Top,
        )
        .draw(display)?;

        self.drawn = armed_idx;
        self.needs_draw = false;
        Ok(())
    }
}
