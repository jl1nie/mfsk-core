//! Board-agnostic logic for the mfsk FT8 controller apps.
//!
//! See `Cargo.toml` for the shared/board boundary rationale. Each module
//! below is moved verbatim from the original `m5stack-s3-app` crate
//! (see `git log --follow` on each file for pre-extraction history).

#![allow(dead_code)]

pub mod activator;
pub mod adif;
pub mod all_txt;
pub mod boot_mode;
pub mod grid_src;
pub mod capture_window;
pub mod civ_frame;
pub mod civil_time;
pub mod flash_log;
pub mod freq_presets;
pub mod grid_fix;
pub mod grid_state;
pub mod http_config;
pub mod jtty_tx;
pub mod log_sink;
pub mod ntp;
pub mod parity;
pub mod qso;
pub mod settings;
pub mod snr_norm;
pub mod time_sync;
pub mod tx_picker;
pub mod udp_log;
pub mod ui;
pub mod wifi;
pub mod wifi_pref;
pub mod wspr_bands;
pub mod wsprnet;
