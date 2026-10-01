//! Mode picker — the way back out of whichever receiver is running.
//!
//! The CoreS3 app is one binary carrying its receivers (FT8, FT4, JTTY;
//! FST4 too in an image built with that feature), chosen at
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
//! tap a row to arm, press the bar to commit, tap outside to dismiss.
//! Nothing is reserved when it is closed.
//!
//! The hold is acknowledged the instant it starts, with a border round
//! the panel ([`HOLD_BORDER_W`]) — without that, a press that fell
//! short of the threshold looked exactly like a panel that had not
//! noticed the finger at all.
//!
//! ## Two levels, because there are two kinds of setting
//!
//! The root names the two: **MODE** (which receiver boots) and
//! **CONFIG** (the two settings that are not a receiver — where the
//! slot grid's phase comes from, and whether WiFi comes up at all).
//! A tap on either opens that page; the commit bar then works exactly
//! as it did, on whichever kind of thing the page holds. Both commits
//! restart the board, so both are worth a confirmation, and neither is
//! reachable by a single stray touch.
//!
//! **FREQ** is the exception on both counts: it lists the running
//! receiver's dial presets, and its commit goes to the rig over CAT
//! without a restart — a wrong dial costs one more tap, not a reboot.
//!
//! The pages share one geometry: rows are drawn from the top and the
//! commit bar stays where it was, so the widget does not move under
//! the finger when the page changes. Unused rows are painted out.
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
use crate::freq_presets::{self, FreqPreset};
use crate::grid_src::GridSource;
use crate::wifi_pref::WifiPref;

/// Which page the overlay is showing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Page {
    Root,
    Mode,
    Config,
    /// The running receiver's dial presets, paged ([`freq_presets::page`]).
    Freq,
    Demo,
}

/// The root's rows, in draw order.
///
/// FREQ is here rather than under CONFIG: CONFIG already fills all four
/// rows, and a dial change is the one setting an operator makes in the
/// field, so it earns the shorter path.
const ROOT: [(Page, &str); 4] = [
    (Page::Mode, "MODE"),
    (Page::Config, "CONFIG"),
    (Page::Freq, "FREQ"),
    (Page::Demo, "DEMO"),
];

/// The FREQ page's paging row.
const NEXT_LABEL: &str = "NEXT >";

/// What [`ModePicker::update`] hands back when the commit bar fires.
/// All but `Freq` restart the board; the caller writes the one it is
/// given.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Commit {
    Mode(BootMode),
    Grid(GridSource),
    Wifi(WifiPref),
    /// A dial in Hz, for the rig over CAT. No restart: the overlay
    /// closes and the receiver carries on.
    Freq(u32),
}

/// One row on the CONFIG page.
///
/// The page carries two settings that have nothing to do with each
/// other — where the slot grid's phase comes from (`grid_src`) and
/// whether the radio associates (`wifi_pref`) — so a row names a
/// *value*, not a setting. Tapping `WIFI: OFF` arms that value and the
/// commit bar applies it, exactly as `TIME: AIR DT` does, and the `*`
/// marks the standing value of **each** setting rather than one row.
///
/// Keeping them on one page rather than giving WiFi its own was the
/// cheaper of the two: a third level would put two taps between the
/// operator and a setting that is changed at the moment the board is
/// somewhere awkward, and four rows is exactly what the widget already
/// draws ([`ROWS`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigRow {
    Grid(GridSource),
    Wifi(WifiPref),
}

impl ConfigRow {
    fn label(self) -> &'static str {
        match self {
            ConfigRow::Grid(src) => src.label(),
            ConfigRow::Wifi(pref) => pref.label(),
        }
    }
}

/// The CONFIG page's rows, in draw order: the grid sources, then the
/// WiFi choices.
pub const CONFIG_ROWS: [ConfigRow; 4] = [
    ConfigRow::Grid(GridSource::Ntp),
    ConfigRow::Grid(GridSource::AirDt),
    ConfigRow::Wifi(WifiPref::On),
    ConfigRow::Wifi(WifiPref::Off),
];

/// The receivers this binary can boot into, in draw order.
///
/// Named by mode alone. They used to read "FT8 / UAC", which said
/// nothing a reader could use: every one of these takes its audio from
/// the radio over USB, so "UAC" is the board, not the choice.
///
/// FT8, FT4 and JTTY (2026-10-01): the three modes the app is being built
/// around — automatic QSOs on FT8 and FT4, template messages on JTTY.
/// FST4 left the list then; its receiver stays behind the `fst4` feature,
/// and a board whose NVS still says `fst4` boots it if the image has it
/// (or falls back to FT8 and logs why), so [`mode_name`] still names it.
pub const MODES: [(BootMode, &str); 3] = [
    (BootMode::Uac, "FT8"),
    (BootMode::Ft4, "FT4"),
    (BootMode::Jtty, "JTTY"),
];

/// The name the screen shows for a boot mode — the picker's own label
/// for a receiver, and "FT8" for the WAV replay, which runs the FT8
/// pipeline. `None` for modes that are not receivers.
pub fn mode_name(mode: BootMode) -> Option<&'static str> {
    match mode {
        BootMode::Decode => Some("FT8"),
        // Off the picker since 2026-10-01, still a receiver an image can boot.
        BootMode::Fst4 => Some("FST4"),
        m => MODES.iter().find(|(b, _)| *b == m).map(|(_, name)| *name),
    }
}

/// Not a receiver — a fixed recording, decoded on a loop.
///
/// It lived among the modes as "DECODE (wav)", where the one thing it
/// needed to say (that no radio is involved) was the part in brackets.
/// On its own page the page says it.
pub const DEMOS: [(BootMode, &str); 1] = [(BootMode::Decode, "WAV REPLAY")];

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
pub const OPEN_MS: u64 = 300;

/// Width of the hold indicator's border, in pixels.
///
/// **A hold with no feedback is indistinguishable from a dead panel.**
/// Opening the menu is a press-and-hold because neither the WSPR nor
/// the FST4 screen has a pixel to spare for a permanent button — but
/// nothing on screen said so, or said that the hold was registering, so
/// a press that fell short looked exactly like a press that was never
/// seen.
///
/// So the whole screen takes a two-pixel border the moment a finger is
/// seen: one draw, no animation, gone on release. A progress bar was
/// tried first and is the wrong instrument — it draws the eye to one
/// place, animates a wait nobody asked to watch, and at 300 ms is over
/// before it reads. What the operator needs to know is binary: the
/// panel felt that.
pub const HOLD_BORDER_W: u32 = 2;

/// Height of the commit bar under the list.
pub const COMMIT_H: u32 = 40;

/// Total height: the mode rows plus the commit bar.
/// Rows the widget is sized for — the widest page. A fixed four, not any
/// one page's length: it was `MODES.len()`, and taking FST4 off the MODE
/// page would then have shrunk the widget under the root and CONFIG, which
/// have four. Compile-checked against every page so adding a row to any of
/// them cannot quietly leave it undrawable.
pub const ROWS: usize = 4;
const _: () = assert!(ROWS >= MODES.len(), "MODE has more rows than the widget draws");
const _: () = assert!(ROWS >= ROOT.len(), "the root has more rows than the widget draws");
const _: () = assert!(ROWS >= CONFIG_ROWS.len(), "CONFIG has more rows than the widget draws");
const _: () = assert!(ROWS >= DEMOS.len(), "DEMO has more rows than the widget draws");

pub const fn height() -> u32 {
    (ROWS as i32 * PITCH) as u32 + COMMIT_H
}

/// Top of the commit bar, relative to the widget origin.
const fn commit_top() -> i32 {
    ROWS as i32 * PITCH
}

/// What a touch landed on.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Target {
    Mode(usize),
    Commit,
    /// A band inside the widget that this page does not use.
    ///
    /// The widget is sized for the longest page, so the root and
    /// CONFIG leave blank rows under their last one. They look like
    /// part of the widget because they are, and a press there used to
    /// fall through to "outside" — which *dismisses*. So aiming at the
    /// lower half of an open menu closed it, which is indistinguishable
    /// from the panel ignoring the touch. Dead, not outside.
    Dead,
}

/// Bands are contiguous — a press in the drawn gap between two rows
/// goes to the nearer one rather than nowhere. Taps were landing on
/// those edges and doing nothing, which reads as a dead panel rather
/// than a near miss.
/// Slop below the commit bar, and to either side.
///
/// A press just past the bottom edge used to land outside the widget,
/// and outside means *dismiss* — so the worst possible response to
/// aiming a few pixels low was the menu vanishing. Measured from the
/// bench: presses at y=280 and y=282 against a bar ending at y=276,
/// both dismissals, between two that landed. A fingertip is several
/// millimetres across and the contact point is not where the operator
/// thinks it is.
///
/// Dismissal still works — it just needs a press that is clearly
/// elsewhere rather than one that is nearly right.
const SLOP: i32 = 14;

fn hit(origin: Point, x: u16, y: u16, rows: usize) -> Option<Target> {
    let (x, y) = (x as i32, y as i32);
    if x < origin.x - SLOP || x >= origin.x + WIDTH as i32 + SLOP {
        return None;
    }
    let dy = y - origin.y;
    if dy < 0 || dy >= height() as i32 + SLOP {
        return None;
    }
    if dy >= commit_top() {
        return Some(Target::Commit);
    }
    let idx = (dy / PITCH) as usize;
    // A page with fewer rows than the widget has bands: the empty ones
    // are painted out, and a press there is a press on nothing — not
    // on the last row (which is how a mis-hit turns into a commit), and
    // not outside either (which would dismiss).
    if idx < rows {
        Some(Target::Mode(idx))
    } else {
        Some(Target::Dead)
    }
}

/// Draw + two-tap state. Owns the commit policy so it cannot drift
/// between the three receivers.
pub struct ModePicker {
    origin: Point,
    /// The panel, for the hold border. The widget draws nothing else
    /// outside its own footprint.
    screen: Size,
    page: Page,
    open: bool,
    just_closed: bool,
    /// When the current press began, for the hold-to-open gesture.
    press_since: Option<std::time::Instant>,
    armed: Option<(usize, std::time::Instant)>,
    /// What the finger is currently on, for the pressed highlight.
    ///
    /// Without this the widget repaints only when the *selection*
    /// changes, so pressing the commit bar changed nothing on screen —
    /// and the press that mattered most was the one with no feedback at
    /// all. A control that does not acknowledge being touched is
    /// indistinguishable from a control that is not there, which is how
    /// this one was reported from the bench.
    pressed: Option<Target>,
    /// The hold indicator is on screen, so leaving it needs an erase
    /// and a repaint of what it covered.
    hold_drawn: bool,
    /// The overlay opened while the border was up: the next render
    /// clears it, since the overlay's own footprint does not.
    erase_hold: bool,
    /// Set once the commit fires. The caller is on its way to a reboot;
    /// until it arrives, the bar says so rather than sitting there
    /// looking unpressed.
    committing: bool,
    drawn: Option<(Page, Option<usize>, usize)>,
    needs_draw: bool,
    /// The running receiver's dial presets ([`Self::set_freq_presets`]).
    presets: &'static [FreqPreset],
    /// Which page of them FREQ shows.
    freq_page: usize,
    /// The rig's dial as last rendered, so opening FREQ lands on the
    /// page that holds it.
    rig_hz: Option<u32>,
}

impl ModePicker {
    pub fn new(origin: Point, screen: Size) -> Self {
        Self {
            origin,
            screen,
            page: Page::Root,
            open: false,
            just_closed: false,
            press_since: None,
            armed: None,
            pressed: None,
            hold_drawn: false,
            erase_hold: false,
            committing: false,
            drawn: None,
            needs_draw: false,
            presets: &[],
            freq_page: 0,
            rig_hz: None,
        }
    }

    /// The table FREQ offers — the running receiver's
    /// ([`freq_presets::for_mode`]).
    pub fn set_freq_presets(&mut self, presets: &'static [FreqPreset]) {
        self.presets = presets;
        self.freq_page = 0;
    }

    fn freq_view(&self) -> freq_presets::Page {
        freq_presets::page(self.presets.len(), ROWS, self.freq_page)
    }

    /// The preset behind row `i` of the FREQ page, `None` for NEXT.
    fn freq_row(&self, i: usize) -> Option<&'static FreqPreset> {
        let v = self.freq_view();
        (i < v.count).then(|| &self.presets[v.start + i])
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
    /// Rows the current page holds.
    fn rows(&self) -> usize {
        match self.page {
            Page::Root => ROOT.len(),
            Page::Mode => MODES.len(),
            Page::Config => CONFIG_ROWS.len(),
            Page::Freq => {
                let v = self.freq_view();
                v.count + usize::from(v.has_next)
            }
            Page::Demo => DEMOS.len(),
        }
    }

    /// The label of row `i` on the current page.
    fn row_label(&self, i: usize) -> &'static str {
        match self.page {
            Page::Root => ROOT[i].1,
            Page::Mode => MODES[i].1,
            Page::Config => CONFIG_ROWS[i].label(),
            Page::Freq => self.freq_row(i).map_or(NEXT_LABEL, |p| p.label),
            Page::Demo => DEMOS[i].1,
        }
    }

    pub fn update(&mut self, pressed: bool, x: u16, y: u16) -> Option<Commit> {
        let now = std::time::Instant::now();
        let was_pressed = self.press_since.is_some();

        if !pressed {
            self.press_since = None;
            if self.pressed.take().is_some() {
                self.needs_draw = true;
            }
        } else if !was_pressed {
            self.press_since = Some(now);
            if self.open {
                let t = hit(self.origin, x, y, self.rows());
                // One line per press, only while the overlay is up, so
                // it is bounded by how fast a finger can tap. Every
                // remaining way this widget can fail to commit is
                // distinguishable from this line alone: whether the
                // press arrived, where it landed, and whether a
                // selection was standing when it did. Three flashes
                // were spent guessing between those instead.
                log::info!(
                    "picker: press ({x}, {y}) origin=({}, {}) page={:?} -> {t:?}, armed={:?}",
                    self.origin.x,
                    self.origin.y,
                    self.page,
                    self.armed.map(|(i, _)| self.row_label(i)),
                );
                if self.pressed != t {
                    self.pressed = t;
                    self.needs_draw = true;
                }
                match t {
                    Some(Target::Mode(idx)) => {
                        if self.page == Page::Root {
                            // The root only navigates: there is
                            // nothing to confirm about opening a page,
                            // and asking would put two taps between
                            // the operator and every setting.
                            self.page = ROOT[idx].0;
                            self.armed = None;
                            self.needs_draw = true;
                            if self.page == Page::Freq {
                                // Open on the page that holds the
                                // rig's dial, if it is a preset.
                                let per = ROWS - 1;
                                self.freq_page = self
                                    .rig_hz
                                    .and_then(|hz| self.presets.iter().position(|p| p.hz == hz))
                                    .filter(|_| self.presets.len() > ROWS)
                                    .map_or(0, |i| i / per);
                            }
                        } else if self.page == Page::Freq && self.freq_row(idx).is_none() {
                            // NEXT pages; it is not a selection.
                            self.freq_page += 1;
                            self.armed = None;
                            self.needs_draw = true;
                        } else {
                            // Selecting only selects. The label stays
                            // readable; the commit bar below says what
                            // pressing it will do.
                            self.armed = Some((idx, now));
                            self.needs_draw = true;
                        }
                    }
                    Some(Target::Commit) => {
                        if self.page == Page::Root {
                            // Nothing on the root can be committed;
                            // the bar says so.
                            log::info!("picker: commit pressed on the root page");
                        } else if let (Page::Freq, Some((idx, _))) = (self.page, self.armed) {
                            // Nothing to wait for: the dial goes out
                            // over CAT and the overlay gets out of the
                            // way of the waterfall it is about to move.
                            let hz = self.freq_row(idx).map(|p| p.hz);
                            self.close();
                            if let Some(hz) = hz {
                                return Some(Commit::Freq(hz));
                            }
                        } else if let Some((idx, _)) = self.armed {
                            self.committing = true;
                            self.needs_draw = true;
                            return Some(match self.page {
                                Page::Mode => Commit::Mode(MODES[idx].0),
                                Page::Demo => Commit::Mode(DEMOS[idx].0),
                                Page::Config => match CONFIG_ROWS[idx] {
                                    ConfigRow::Grid(src) => Commit::Grid(src),
                                    ConfigRow::Wifi(pref) => Commit::Wifi(pref),
                                },
                                Page::Root | Page::Freq => unreachable!("handled above"),
                            });
                        } else {
                            // Pressing commit with nothing selected is
                            // a real state (the bar says "pick a mode
                            // above"), not a fault — but it is also
                            // indistinguishable on screen from a press
                            // that was never seen, so say which it was.
                            log::info!("picker: commit pressed with no selection standing");
                        }
                    }
                    Some(Target::Dead) => {
                        log::info!("picker: press on an unused row — ignored");
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
                    // The border is at the panel's edges, which the
                    // overlay does not cover — `render_hold` is not
                    // called once `open` is set, so erase it here.
                    // `close()` repaints the screen anyway.
                    self.erase_hold = self.hold_drawn;
                    self.hold_drawn = false;
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
        // Next open starts at the root rather than wherever the last
        // one was left — an overlay that reappears on a page nobody
        // chose is how a stray tap lands on a setting.
        self.page = Page::Root;
        self.armed = None;
        self.pressed = None;
        self.committing = false;
        self.drawn = None;
        self.needs_draw = false;
        self.just_closed = true;
    }

    /// Draw, but only while open and only when something changed.
    /// `current` / `current_grid` / `current_wifi` are what this boot
    /// is running, marked with a `*` on their pages so the operator can
    /// see the setting before changing it. CONFIG carries two settings,
    /// so it marks two rows.
    pub fn render<D>(
        &mut self,
        display: &mut D,
        current: BootMode,
        current_grid: GridSource,
        current_wifi: WifiPref,
        rig_hz: Option<u32>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.rig_hz = rig_hz;
        if !self.open {
            return self.render_hold(display);
        }
        if core::mem::take(&mut self.erase_hold) {
            let (w, h) = (self.screen.width, self.screen.height);
            let b = HOLD_BORDER_W;
            for (p, size) in [
                (Point::new(0, 0), Size::new(w, b)),
                (Point::new(0, (h - b) as i32), Size::new(w, b)),
                (Point::new(0, 0), Size::new(b, h)),
                (Point::new((w - b) as i32, 0), Size::new(b, h)),
            ] {
                Rectangle::new(p, size)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
        let armed_idx = self.armed.map(|(i, _)| i);
        // `needs_draw` is what carries a pressed/released edge here —
        // neither changes `armed_idx`, so gating on the selection alone
        // is exactly what made a press invisible. The page is in the
        // same key because navigating changes neither.
        if !self.needs_draw && self.drawn == Some((self.page, armed_idx, self.freq_page)) {
            return Ok(());
        }
        let text = |fg: Rgb565, bg: Rgb565| {
            MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(fg)
                .background_color(bg)
                .build()
        };

        // Every page draws into the same bands; the widest is what the
        // widget was sized for.
        for i in 0..ROWS {
            let top = self.origin.y + i as i32 * PITCH;
            // Rows past this page's end are painted out, so a shorter
            // page does not leave the previous one's labels standing.
            if i >= self.rows() {
                Rectangle::new(Point::new(self.origin.x, top), Size::new(WIDTH, BUTTON_H))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
                continue;
            }
            let label = self.row_label(i);
            // The row that names what this boot is already running, or
            // on the root, the page that leads to it.
            let is_current = match self.page {
                Page::Root => false,
                Page::Mode => MODES[i].0 == current,
                Page::Demo => DEMOS[i].0 == current,
                Page::Config => match CONFIG_ROWS[i] {
                    ConfigRow::Grid(src) => src == current_grid,
                    ConfigRow::Wifi(pref) => pref == current_wifi,
                },
                Page::Freq => {
                    rig_hz.is_some() && self.freq_row(i).map(|p| p.hz) == rig_hz
                }
            };
            let selected = armed_idx == Some(i);
            // Palette borrowed whole from `decoded_list`, which is
            // already proven on this panel: GREEN for the current
            // slot, CSS_ORANGE for the actionable row. Here GREEN is
            // the standing selection and CSS_ORANGE is the finger.
            let (bg, fg) = if self.pressed == Some(Target::Mode(i)) {
                (Rgb565::CSS_ORANGE, Rgb565::BLACK)
            } else if selected {
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
            if is_current {
                Text::with_baseline(
                    "*",
                    Point::new(
                        self.origin.x + WIDTH as i32 - 14,
                        top + (BUTTON_H as i32 - 10) / 2,
                    ),
                    text(fg, bg),
                    Baseline::Top,
                )
                .draw(display)?;
            }
        }

        // The commit bar names what it will do, so nothing depends on
        // remembering which row was tapped.
        let ctop = self.origin.y + commit_top();
        let (cbg, cfg) = if self.committing {
            (Rgb565::GREEN, Rgb565::BLACK)
        } else if self.pressed == Some(Target::Commit) {
            (Rgb565::CSS_ORANGE, Rgb565::BLACK)
        } else {
            match armed_idx {
                Some(_) => (Rgb565::new(0, 24, 0), Rgb565::WHITE),
                None => (Rgb565::BLACK, Rgb565::CSS_GRAY),
            }
        };
        Rectangle::new(Point::new(self.origin.x, ctop), Size::new(WIDTH, COMMIT_H))
            .into_styled(PrimitiveStyle::with_fill(cbg))
            .draw(display)?;
        let mut line: heapless::String<32> = heapless::String::new();
        match (self.page, armed_idx) {
            (Page::Root, _) => {
                let _ = line.push_str("pick a page above");
            }
            (_, Some(i)) => {
                let _ = line.push_str(if self.committing {
                    "APPLYING "
                } else {
                    "APPLY "
                });
                let _ = line.push_str(self.row_label(i));
            }
            (Page::Mode, None) | (Page::Demo, None) => {
                let _ = line.push_str("pick a mode above");
            }
            (Page::Config, None) => {
                let _ = line.push_str("pick a setting above");
            }
            (Page::Freq, None) => {
                use core::fmt::Write as _;
                let v = self.freq_view();
                let _ = if self.presets.is_empty() {
                    write!(&mut line, "no presets for this mode")
                } else if v.of > 1 {
                    write!(&mut line, "pick a dial  {}/{}", v.number, v.of)
                } else {
                    write!(&mut line, "pick a dial above")
                };
            }
        }
        Text::with_baseline(
            line.as_str(),
            Point::new(self.origin.x + 10, ctop + (COMMIT_H as i32 - 10) / 2),
            text(cfg, cbg),
            Baseline::Top,
        )
        .draw(display)?;

        self.drawn = Some((self.page, armed_idx, self.freq_page));
        self.needs_draw = false;
        Ok(())
    }

    /// The hold indicator, drawn while the overlay is still closed: a
    /// border round the whole panel for as long as a finger is down.
    ///
    /// Drawn once when the press begins and erased once when it ends —
    /// the intermediate frames have nothing to say. On release the
    /// erase requests a repaint through [`Self::take_just_closed`], the
    /// same path the overlay itself uses, because the border sat on
    /// whatever the screen was showing.
    fn render_hold<D>(&mut self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let pressed = self.press_since.is_some();
        if pressed == self.hold_drawn {
            return Ok(());
        }
        let colour = if pressed {
            Rgb565::CSS_ORANGE
        } else {
            Rgb565::BLACK
        };
        let (w, h) = (self.screen.width, self.screen.height);
        let b = HOLD_BORDER_W;
        // Four bars rather than a stroked rectangle: the stroke would
        // be centred on the edge and half of it would fall off the
        // panel, which on this driver is a clipped write per frame.
        for (p, size) in [
            (Point::new(0, 0), Size::new(w, b)),
            (Point::new(0, (h - b) as i32), Size::new(w, b)),
            (Point::new(0, 0), Size::new(b, h)),
            (Point::new((w - b) as i32, 0), Size::new(b, h)),
        ] {
            Rectangle::new(p, size)
                .into_styled(PrimitiveStyle::with_fill(colour))
                .draw(display)?;
        }
        self.hold_drawn = pressed;
        if !pressed {
            // Whatever the border sat on has to come back.
            self.just_closed = true;
        }
        Ok(())
    }
}
