//! Status bar — 14 px tall, drawn at y ∈ [0, 14). Fixed-position
//! header showing rig freq / mode / UTC / heap. All fields optional
//! and rendered as `--` while their source (CI-V, time_sync, PMIC)
//! is offline; the bar is therefore safe to show during boot.
//!
//! Layout (FONT_6X10 = 6 × 10 px, 22 chars at 135 px width):
//!
//! ```text
//! 7074 USB  19:30:45  235k
//! ^^^^      ^^^^^^^^  ^^^^^   freq kHz / UTC HH:MM:SS / heap KB
//!     ^^^   mode (3 chars)
//! ```

use core::fmt::Write as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

use crate::ui::state::StatusInfo;

pub const ORIGIN_Y: i32 = 0;
pub const HEIGHT: u32 = 14;

pub fn render<D>(display: &mut D, status: &StatusInfo) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let bg = Rgb565::new(0, 8, 0); // very dark green for the bar
    let fg = Rgb565::WHITE;

    // **No leading full-rectangle wipe.** The MonoTextStyle below
    // carries `background_color`, so each glyph cell repaints both
    // foreground and background atomically — a separate wipe would
    // briefly clear the bar before the text lands and produces the
    // 100 ms-cadence flicker the user sees. The 22-char fixed-width
    // line covers 132 px of the 135 px panel; we paint the trailing
    // 3 px below as a single tiny rectangle (negligible flicker).

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(fg)
        .background_color(bg)
        .build();

    let mut s: String<32> = String::new();
    // Freq: kHz, 4-5 digits ("7074", "14074", "144174").
    match status.rig_freq_hz {
        Some(hz) => {
            let _ = write!(&mut s, "{:>5}", hz / 1000);
        }
        None => {
            let _ = s.push_str(" ----");
        }
    }
    // Mode: 3 chars.
    let mode: &str = status.rig_mode.as_deref().unwrap_or("---");
    let _ = write!(&mut s, " {mode:<3}");
    // UTC HH:MM:SS or "--:--:--" (8 chars).
    match status.utc_sod {
        Some(sod) => {
            let h = sod / 3600;
            let m = (sod / 60) % 60;
            let sec = sod % 60;
            let _ = write!(&mut s, " {h:02}:{m:02}:{sec:02}");
        }
        None => {
            let _ = s.push_str(" --:--:--");
        }
    }
    // Heap KB (4 chars).
    let _ = write!(&mut s, " {:>4}", status.free_heap_kb);

    // Drop chars beyond the 22-char width if formatting overflowed.
    let visible = s.as_str();
    let visible = &visible[..visible.len().min(22)];

    Text::with_baseline(visible, Point::new(1, 2), style, Baseline::Top)
        .draw(display)?;
    // Tail-paint the 3 px right margin so the bar background covers
    // the full panel width (132..135).
    Rectangle::new(Point::new(132, ORIGIN_Y), Size::new(3, HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(bg))
        .draw(display)?;
    Ok(())
}
