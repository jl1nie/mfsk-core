//! Waterfall renderer — region y ∈ [14, 114), 135 × 100 px.
//!
//! Newest slot at the bottom (y=113), oldest at the top (y=14). Each
//! row is a pre-baked `WfLine` of 135 palette indices (0..15), so the
//! render pass is a flat colour-lookup + raster — no per-call FFT or
//! log work. Decode-pipeline pre-computes the row once per slot
//! (~once / 15 s) in `decode_pipeline::build_wf_row`.
//!
//! Repaint cost: 135 × 100 = 13 500 px × 16 bit = 27 KB pushed in
//! **one** `fill_contiguous` call so the SPI driver issues a single
//! `CASET`/`RASET`/`RAMWR` command + DC pulse, not 100. At 40 MHz
//! that's ~5.4 ms transfer + ~1 ms DMA / overhead = ~7-10 ms full
//! repaint, fired once per slot (~15 s). Effectively free.
//!
//! HW vertical scroll via ST7789 `VSCRDEF` / `VSCRSAD` is *also*
//! possible (TFA=14 status bar, VSA=100 waterfall, BFA=126 below)
//! and would cut the SPI cost to one row (~270 bytes = 70 µs) plus
//! the scroll-pointer write. Deferred — at one repaint per 15 s the
//! coalesced full-region path is already inside the noise floor of
//! the post-SlotEnd budget. Re-evaluate when streaming live UAC at
//! finer cadence (Phase 1 follow-up).
//!
//! Palette is a coarse NanoVNA-style 16-step gradient (black → blue
//! → cyan → green → yellow → orange → red → white).

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
};

use crate::ui::state::{WfLine, WF_DEPTH};

pub const ORIGIN_Y: i32 = 14;
pub const HEIGHT: u32 = 100;
pub const WIDTH: u32 = 135;
pub const WF_FREQ_LO_HZ: f32 = 200.0;
pub const WF_FREQ_HI_HZ: f32 = 2700.0;

/// 16-step palette. Indices 0..15 map magnitude bands; 0 = silence
/// (black), 15 = peak (white). RGB565 encoded inline.
const PALETTE: [Rgb565; 16] = [
    Rgb565::new(0, 0, 0),     //  0  black
    Rgb565::new(0, 0, 6),     //  1  near-black blue
    Rgb565::new(0, 0, 12),    //  2  dim blue
    Rgb565::new(0, 4, 18),    //  3  blue
    Rgb565::new(0, 12, 24),   //  4  cyan-blue
    Rgb565::new(0, 24, 24),   //  5  cyan
    Rgb565::new(0, 36, 16),   //  6  teal-green
    Rgb565::new(0, 48, 0),    //  7  green
    Rgb565::new(8, 56, 0),    //  8  yellow-green
    Rgb565::new(16, 60, 0),   //  9  lime
    Rgb565::new(24, 60, 0),   // 10  yellow-lime
    Rgb565::new(31, 56, 0),   // 11  yellow
    Rgb565::new(31, 40, 0),   // 12  orange-yellow
    Rgb565::new(31, 24, 0),   // 13  orange
    Rgb565::new(31, 8, 0),    // 14  red
    Rgb565::new(31, 31, 31),  // 15  white (peak)
];

/// Repaint the waterfall region from `lines` (oldest first, newest
/// last). Lines beyond `WF_DEPTH` are ignored. Top rows are filled
/// with palette[0] (black) when fewer than `WF_DEPTH` slots have
/// arrived. Caller gates by `UiState::dirty_seq` — single
/// `fill_contiguous` over the whole 135 × 100 region so the SPI
/// driver issues one CASET/RASET/RAMWR.
pub fn render<D>(display: &mut D, lines: &[&WfLine]) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let n = lines.len().min(WF_DEPTH);
    let blank_rows = WF_DEPTH - n;
    let take = &lines[lines.len() - n..];

    // Stream pixels top-to-bottom, left-to-right. The first
    // `blank_rows × WIDTH` pixels are palette[0]; the rest are
    // unpacked from the supplied lines.
    let rect = Rectangle::new(
        Point::new(0, ORIGIN_Y),
        Size::new(WIDTH, HEIGHT),
    );
    let pixels = (0..HEIGHT as usize).flat_map(|row| {
        let row_pixels: &[u8] = if row < blank_rows {
            &[][..]
        } else {
            &take[row - blank_rows][..]
        };
        // For a blank row return a 135-long zero stream; otherwise
        // map each palette index to its RGB565 colour.
        let blank = row < blank_rows;
        (0..WIDTH as usize).map(move |col| {
            let idx = if blank {
                0
            } else {
                row_pixels[col] & 0x0F
            };
            PALETTE[idx as usize]
        })
    });
    display.fill_contiguous(&rect, pixels)
}
