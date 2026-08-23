//! The receivers this binary can boot into.
//!
//! Each is a mode of one image rather than its own binary: changing
//! mode used to mean re-flashing, and on CoreS3 that means unplugging
//! the radio, because `usb_host_install` takes the port the flasher
//! would use. `main` reads the NVS `boot_mode` and calls one of these.
//!
//! They own their own tasks, screens and stacks; what they share —
//! `board`, `pmic`, `uac`, the log fanout and the `cfg.toml` env
//! constants — lives at the crate root, one copy for all of them.

pub mod fst4;
pub mod wspr;
