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

pub fn render<D>(display: &mut D, status: &StatusInfo, width: u32) -> Result<(), D::Error>
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

    // 6 px per glyph at FONT_6X10, 1 px of left margin.
    const CHAR_W: u32 = 6;
    let cols = ((width.saturating_sub(1)) / CHAR_W) as usize;

    let mut s: String<32> = String::new();
    // The mode name, where the panel can afford it.
    //
    // The 22-char core below was sized for the M5StickS3's 135 px, and
    // on that panel there is no room. The CoreS3 and the Core2 have
    // 240 and 320, and on the CoreS3 the omission actually mattered:
    // one binary boots into one of four receivers, WSPR and FST4 name
    // themselves on their own status lines, and the FT8 screen did not.
    let named = cols >= 26;
    if named {
        let _ = s.push_str("FT8 ");
    }
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

    // Drop chars the panel cannot show, if formatting overflowed.
    let budget = if named { 26 } else { 22 };
    let visible = s.as_str();
    let visible = &visible[..visible.len().min(budget)];

    Text::with_baseline(visible, Point::new(1, 2), style, Baseline::Top).draw(display)?;
    // Tail-paint whatever the line does not cover, so the bar
    // background reaches the right edge on any panel width.
    let text_px = budget as u32 * CHAR_W + 1;
    if width > text_px {
        Rectangle::new(
            Point::new(text_px as i32, ORIGIN_Y),
            Size::new(width - text_px, HEIGHT),
        )
        .into_styled(PrimitiveStyle::with_fill(bg))
        .draw(display)?;
    }
    Ok(())
}
