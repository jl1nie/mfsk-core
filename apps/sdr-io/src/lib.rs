// SPDX-License-Identifier: GPL-3.0-only
//! IQ input for the sample apps, apart from any decoder: the SpyServer and `rtl_tcp` clients
//! ([`spyserver`], [`rtltcp`]), the NTP-corrected UTC they stamp messages with ([`clock`]), and
//! the estimate that ties a stream's sample count to that UTC ([`anchor`]).
//!
//! **std only** (sockets, threads, the system clock), and nothing of `mfsk-core` but
//! [`IqSampleFormat`](mfsk_core::iq::IqSampleFormat): no protocol, FFT or storage. It was split
//! out of `skimmer-core`, which re-exports these modules at their old paths, so that a light
//! receiver (a WSPR-only daemon) can use the clients without the skimmer's SQLite store, PSK
//! Reporter uploader and every mode's decoder.

pub mod anchor;
pub mod clock;
pub mod rtltcp;
pub mod spyserver;

pub use clock::now_ns;
