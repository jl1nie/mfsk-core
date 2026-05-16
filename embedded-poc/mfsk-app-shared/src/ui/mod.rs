//! UI rendering layer (Phase 3).
//!
//! ターゲット: ST7789v2 135x240 (M5StickS3 内蔵 LCD)。
//! ドライバ: `mipidsi` + `display-interface-spi` + DMA。
//!
//! レイアウト (135 width x 240 height):
//!   y=0..14    status bar    (freq / mode / UTC / battery)
//!   y=14..114  waterfall     (100 px 高、2 px/sec、~19 Hz/bin)
//!   y=114..226 decoded list  (7 行 x 16 px)
//!   y=226..240 TX message    (1 行 14 px)
//!
//! Partial update + dirty-rect が前提 (全画面 redraw は ~800 ms 掛かる)。
//!
//! Phase 0: モジュール宣言のみ。

pub mod decoded_list;
pub mod menu;
pub mod state;
pub mod status_bar;
pub mod waterfall;
