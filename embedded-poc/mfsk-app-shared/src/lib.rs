//! Board-agnostic logic for the mfsk FT8 controller apps.
//!
//! See `Cargo.toml` for the shared/board boundary rationale. Each module
//! below is moved verbatim from the original `m5stack-s3-app` crate
//! (see `git log --follow` on each file for pre-extraction history).

#![allow(dead_code)]

pub mod adif;
pub mod boot_mode;
pub mod flash_log;
pub mod http_config;
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
pub mod wspr_bands;
pub mod wsprnet;
