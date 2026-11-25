// SPDX-License-Identifier: GPL-3.0-only
//! Host-side ICAP profile for mfsk-core's embedded FT8 path.
//!
//! ICAP (`../icap-sdk`, custom IC-705 / IC-9700 firmware) gives an app 8 kHz mono audio, a Cortex-A9 and
//! about 320 KiB of RAM (80 KiB app window, 240 KiB data region; `apps/q65/README.md` there). This crate
//! does **not** run on the radio: it runs the embedded decode path (`decode_block`, the
//! `not(fft-rustfft)` body) on a host under the same input rate and a heap meter, so that recall,
//! phantoms, peak heap and time can be measured without hardware, against the same 22 real recordings and
//! `jt9` reference that tier B uses (`mfsk-core/tests/ft8_real_recordings_vs_jt9.rs`).
//!
//! Run it with `cargo run --release -p icap-app-hosttest --bin icap-profile`. Run it **by package**: a
//! workspace-wide build unifies `fft-rustfft` into mfsk-core, which switches `decode_block` to the host
//! driver; [`fft_shim::factory_calls`] lets a run notice that (the test asserts it).

pub mod fft_shim;
pub mod heap;
pub mod profile;
pub mod resample;
pub mod wav;

#[global_allocator]
static ALLOC: heap::Meter = heap::Meter;
