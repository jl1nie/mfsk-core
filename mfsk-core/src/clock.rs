//! The one clock the stage timers use (`wspr::instrument`, `msk144` trace
//! counters).
//!
//! `std` does not imply a clock: on `wasm32-unknown-unknown` std's
//! `Instant::now()` is the "unsupported" backend and **panics** (`time not
//! implemented on this platform`, issue #583) — which took `Decoder<Wspr>`
//! and the MSK144 decoder down on any input in a browser build, since
//! `fft-rustfft` requires `std`. Those timers only feed benchmark counters, so
//! there the tick reads zero, as `wspr::instrument` already documents for
//! builds without a clock.
//!
//! Every `Instant::now()` that is not behind a trace flag or in a test goes
//! through [`Tick`]; `grep -rn 'Instant::now' src/` should show nothing else
//! on a decode path.

#[cfg(not(all(target_arch = "wasm32", target_os = "unknown")))]
#[derive(Clone, Copy)]
pub(crate) struct Tick(std::time::Instant);

#[cfg(not(all(target_arch = "wasm32", target_os = "unknown")))]
impl Tick {
    #[inline]
    pub(crate) fn now() -> Self {
        Self(std::time::Instant::now())
    }
    #[inline]
    pub(crate) fn elapsed_micros(&self) -> u64 {
        self.0.elapsed().as_micros() as u64
    }
    #[inline]
    pub(crate) fn elapsed_nanos(&self) -> u64 {
        self.0.elapsed().as_nanos() as u64
    }
}

/// `wasm32-unknown-unknown`: no clock, so no reading.
#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
#[derive(Clone, Copy)]
pub(crate) struct Tick;

#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
impl Tick {
    #[inline]
    pub(crate) fn now() -> Self {
        Self
    }
    #[inline]
    pub(crate) fn elapsed_micros(&self) -> u64 {
        0
    }
    #[inline]
    pub(crate) fn elapsed_nanos(&self) -> u64 {
        0
    }
}
