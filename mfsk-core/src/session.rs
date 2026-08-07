//! Streaming audio ingestion — assembling soundcard samples into decode slots.
//!
//! Where [`crate::msg::decoded`] and the `.on_result` streaming callbacks
//! (see `docs/reference/STREAMING.md`) handle results flowing *out* of a
//! decode incrementally, this module handles audio flowing *in*
//! incrementally. It is the layer that sits **above** the decode entry
//! points and **below** a live UI: it takes the native-rate samples a
//! soundcard / UAC reader / I2S capture produces, resamples them to the
//! fixed 12 kHz the pipeline expects, windows them into fixed-length
//! slots, and hands each completed slot back to the caller to decode.
//!
//! It deliberately stops at *assembly*. It does **not** run the decode
//! itself — decode configuration (frequency range, depth, SIC, AP, hash
//! table) is rich and protocol-specific, and belongs to the caller's
//! [`DecodeRequest`](crate::msg::decode_request) / `decode_scan_*` call.
//! A completed [`Slot`] owns its samples, so the caller is free to move
//! it onto a worker thread (`std::thread::spawn`, Tokio `spawn_blocking`,
//! an embedded decode task) without fighting a borrow — keeping the heavy
//! decode off whatever thread is feeding audio in.
//!
//! # Slot timing is sample-counted, not wall-clock
//!
//! Slot boundaries are tracked purely by **accumulated 12 kHz sample
//! count** (`T_SLOT_S × 12000`), never by reading a clock. This is what
//! keeps the type `no_std`-friendly and executor-free: there is no
//! `std::time`, no async runtime, no timer. Aligning the slot grid to a
//! real time source (UTC `:00/:15/:30/:45` on a host, GPS PPS or an
//! operator button on embedded) is the caller's job, done by calling
//! [`SlotAssembler::mark_slot_start`] at the instant a slot should begin.
//! This mirrors the ad-hoc `reset_slot_boundary` logic the M5Stack apps
//! already carry (`embedded-poc/m5stack-s3-app/src/audio.rs`), generalised.
//!
//! # Example
//!
//! ```no_run
//! # #[cfg(feature = "ft8")] {
//! use mfsk_core::ft8::Ft8;
//! use mfsk_core::session::SlotAssembler;
//!
//! // Soundcard delivers 48 kHz mono i16; FT8 slots are 15 s.
//! let mut asm = SlotAssembler::for_protocol::<Ft8>(48_000);
//!
//! # let audio_chunk: &[i16] = &[];
//! // In the capture callback: feed native-rate samples as they arrive.
//! if let Some(slot) = asm.push(audio_chunk) {
//!     // A full 15 s slot is ready. Move it onto a worker to decode;
//!     // `slot.nominal_start_sample` feeds straight into the WSPR/JT
//!     // family `to_decoded(sample_rate, nominal_start_sample)`.
//!     let samples = slot.samples; // owned Vec<i16>, exactly one slot long
//!     // std::thread::spawn(move || decode(&samples)); ...
//!     # let _ = samples;
//! }
//! # }
//! ```

use alloc::vec::Vec;

use crate::engine::dsp::LinearResamplerI16To12k;
use crate::engine::protocol::FrameLayout;

/// Fixed sample rate every decode entry point operates at.
pub const SAMPLE_RATE_HZ: u32 = 12_000;

/// One completed decode slot: exactly [`SlotAssembler::slot_samples`]
/// samples of 12 kHz mono audio, owned so it can be moved to a worker.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Slot {
    /// 12 kHz mono samples, length always equal to the assembler's
    /// `slot_samples`. Pass by reference to a decode entry point.
    pub samples: Vec<i16>,
    /// Monotonic slot counter, starting at 0. Increments once per
    /// completed slot; unaffected by [`SlotAssembler::mark_slot_start`].
    pub index: u64,
    /// Sample offset within `samples` of the mode's `dt = 0` anchor —
    /// `round(TX_START_OFFSET_S × 12000)`, e.g. 6000 (0.5 s) for FT8/FT4.
    /// Feed directly as the `nominal_start_sample` argument of the
    /// WSPR / JT65 / JT9 `to_decoded(sample_rate, nominal_start_sample)`
    /// conversions. `0` when built via [`SlotAssembler::new`] without a
    /// protocol.
    pub nominal_start_sample: usize,
}

/// Accumulates streaming native-rate audio into fixed-length 12 kHz slots.
///
/// Feed samples with [`push`](Self::push); it returns a [`Slot`] the moment
/// enough audio has accumulated to fill one. Realtime capture chunks are
/// far shorter than a slot, so a given `push` returns `Some` only on the
/// chunk that crosses a boundary and `None` otherwise. If a single push
/// carries more than one slot's worth (bulk / offline feeding), the first
/// completed slot is returned and the rest are drained with
/// [`poll`](Self::poll).
pub struct SlotAssembler {
    /// `None` when the input is already 12 kHz (pure pass-through).
    resampler: Option<LinearResamplerI16To12k>,
    /// Staging buffer for resampler output, reused across pushes.
    scratch: Vec<i16>,
    /// 12 kHz accumulation. Holds `< slot_samples` between completed
    /// slots (plus any not-yet-drained overflow from a large push).
    buf: Vec<i16>,
    slot_samples: usize,
    nominal_start_sample: usize,
    next_index: u64,
}

impl SlotAssembler {
    /// Build an assembler that emits slots of `slot_samples` 12 kHz
    /// samples, resampling from `input_rate_hz` on the way in.
    ///
    /// `input_rate_hz == 12000` is a zero-cost pass-through (no resampler
    /// is constructed). [`Slot::nominal_start_sample`] is `0`; set it with
    /// [`set_nominal_start_sample`](Self::set_nominal_start_sample) or use
    /// [`for_protocol`](Self::for_protocol) to derive everything from a mode.
    ///
    /// # Panics
    /// Panics if `slot_samples == 0` or `input_rate_hz == 0`.
    pub fn new(slot_samples: usize, input_rate_hz: u32) -> Self {
        assert!(slot_samples > 0, "slot_samples must be > 0");
        assert!(input_rate_hz > 0, "input_rate_hz must be > 0");
        let resampler =
            (input_rate_hz != SAMPLE_RATE_HZ).then(|| LinearResamplerI16To12k::new(input_rate_hz));
        Self {
            resampler,
            scratch: Vec::new(),
            buf: Vec::with_capacity(slot_samples),
            slot_samples,
            nominal_start_sample: 0,
            next_index: 0,
        }
    }

    /// Build an assembler sized for protocol `P`, resampling from
    /// `input_rate_hz`.
    ///
    /// Slot length is `round(P::T_SLOT_S × 12000)` (180 000 for FT8,
    /// 90 000 for FT4, …) and [`Slot::nominal_start_sample`] is
    /// `round(P::TX_START_OFFSET_S × 12000)`.
    pub fn for_protocol<P: FrameLayout>(input_rate_hz: u32) -> Self {
        let slot_samples = round_to_samples(P::T_SLOT_S);
        let mut asm = Self::new(slot_samples, input_rate_hz);
        asm.nominal_start_sample = round_to_samples(P::TX_START_OFFSET_S);
        asm
    }

    /// Override the `dt = 0` anchor stamped onto each emitted [`Slot`].
    pub fn set_nominal_start_sample(&mut self, nominal_start_sample: usize) {
        self.nominal_start_sample = nominal_start_sample;
    }

    /// Feed native-rate samples. Returns the first slot completed by this
    /// call, if any; drain further completed slots (rare — only when one
    /// push carried more than a slot's worth) with [`poll`](Self::poll).
    pub fn push(&mut self, native: &[i16]) -> Option<Slot> {
        self.ingest(native);
        self.poll()
    }

    /// Take one already-accumulated slot without feeding new samples.
    /// Returns `None` until at least `slot_samples` have accumulated.
    pub fn poll(&mut self) -> Option<Slot> {
        if self.buf.len() < self.slot_samples {
            return None;
        }
        // Split the leading slot out as an owned buffer; the tail (usually
        // empty) stays in `buf` to seed the next slot.
        let samples: Vec<i16> = self.buf.drain(..self.slot_samples).collect();
        let index = self.next_index;
        self.next_index += 1;
        Some(Slot {
            samples,
            index,
            nominal_start_sample: self.nominal_start_sample,
        })
    }

    /// Realign the slot grid: discard the partial audio accumulated so far
    /// so the *next* slot boundary falls one slot length from now.
    ///
    /// This is the hook a caller drives from its own time source to lock
    /// the slot grid onto the band (UTC quarter-minutes on a host, an
    /// operator sync-tap or GPS PPS on embedded). Resampler continuity and
    /// the [`Slot::index`] counter are preserved — only the current
    /// in-progress slot is dropped.
    pub fn mark_slot_start(&mut self) {
        self.buf.clear();
    }

    /// Samples per completed slot (12 kHz).
    pub fn slot_samples(&self) -> usize {
        self.slot_samples
    }

    /// 12 kHz samples accumulated toward the next slot.
    pub fn buffered(&self) -> usize {
        self.buf.len()
    }

    /// Input sample rate this assembler resamples from (`12000` = pass-through).
    pub fn input_rate(&self) -> u32 {
        self.resampler
            .as_ref()
            .map_or(SAMPLE_RATE_HZ, |r| r.src_rate())
    }

    /// Append `native` to `buf`, resampling to 12 kHz first if needed.
    fn ingest(&mut self, native: &[i16]) {
        let Some(rs) = self.resampler.as_mut() else {
            self.buf.extend_from_slice(native);
            return;
        };
        let mut src = native;
        while !src.is_empty() {
            let cap = rs.max_output_for(src.len()).max(1);
            if self.scratch.len() < cap {
                self.scratch.resize(cap, 0);
            }
            let (consumed, produced) = rs.process(src, &mut self.scratch[..cap]);
            self.buf.extend_from_slice(&self.scratch[..produced]);
            if consumed == 0 {
                // No forward progress possible (only happens on empty src,
                // already excluded) — guard against a spin regardless.
                break;
            }
            src = &src[consumed..];
        }
    }
}

/// `round(seconds × 12000)` as a sample count. All WSJT modes have
/// integer-valued products, so the `+ 0.5` truncation is exact rounding.
fn round_to_samples(seconds: f32) -> usize {
    (seconds * SAMPLE_RATE_HZ as f32 + 0.5) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passthrough_emits_exactly_one_slot_at_the_boundary() {
        let mut asm = SlotAssembler::new(100, SAMPLE_RATE_HZ);
        assert_eq!(asm.input_rate(), SAMPLE_RATE_HZ);

        // One short of a slot: nothing yet.
        let ramp: Vec<i16> = (0..99).collect();
        assert!(asm.push(&ramp).is_none());
        assert_eq!(asm.buffered(), 99);

        // The sample that crosses the boundary yields the slot.
        let slot = asm.push(&[99]).expect("slot completes at 100 samples");
        assert_eq!(slot.index, 0);
        assert_eq!(slot.samples.len(), 100);
        assert_eq!(slot.samples, (0..100).collect::<Vec<i16>>());
        assert_eq!(asm.buffered(), 0);
    }

    #[test]
    fn slot_index_increments_across_slots() {
        let mut asm = SlotAssembler::new(10, SAMPLE_RATE_HZ);
        let chunk: Vec<i16> = (0..10).collect();
        assert_eq!(asm.push(&chunk).unwrap().index, 0);
        assert_eq!(asm.push(&chunk).unwrap().index, 1);
        assert_eq!(asm.push(&chunk).unwrap().index, 2);
    }

    #[test]
    fn large_push_drains_extra_slots_via_poll() {
        let mut asm = SlotAssembler::new(10, SAMPLE_RATE_HZ);
        // 25 samples = two full slots + 5 leftover.
        let bulk: Vec<i16> = (0..25).collect();
        let first = asm.push(&bulk).expect("first slot from push");
        assert_eq!(first.index, 0);
        assert_eq!(first.samples, (0..10).collect::<Vec<i16>>());

        let second = asm.poll().expect("second slot from poll");
        assert_eq!(second.index, 1);
        assert_eq!(second.samples, (10..20).collect::<Vec<i16>>());

        assert!(asm.poll().is_none());
        assert_eq!(asm.buffered(), 5);
    }

    #[test]
    fn mark_slot_start_discards_partial_but_keeps_index() {
        let mut asm = SlotAssembler::new(10, SAMPLE_RATE_HZ);
        asm.push(&[0, 1, 2, 3, 4]);
        assert_eq!(asm.buffered(), 5);

        asm.mark_slot_start();
        assert_eq!(asm.buffered(), 0);

        // Next full slot after the realignment still starts at index 0.
        let slot = asm.push(&(0..10).collect::<Vec<i16>>()).unwrap();
        assert_eq!(slot.index, 0);
    }

    #[test]
    fn resampling_halves_a_24k_input() {
        let mut asm = SlotAssembler::new(100, 24_000);
        assert_eq!(asm.input_rate(), 24_000);
        // 240 input samples @ 24k -> ~120 @ 12k -> one 100-sample slot,
        // ~20 buffered. Exact counts depend on the resampler's priming.
        let input: Vec<i16> = (0..240).map(|i| (i % 50) as i16).collect();
        let slot = asm.push(&input).expect("24k -> 12k yields a slot");
        assert_eq!(slot.samples.len(), 100);
        assert!(asm.buffered() < 100);
    }

    #[test]
    fn split_input_across_many_small_chunks() {
        let mut asm = SlotAssembler::new(64, SAMPLE_RATE_HZ);
        let mut completed = 0;
        for i in 0..64i16 {
            if asm.push(&[i]).is_some() {
                completed += 1;
            }
        }
        assert_eq!(completed, 1);
    }

    #[cfg(feature = "ft8")]
    #[test]
    fn for_protocol_ft8_sizes_slot_and_anchor() {
        use crate::ft8::Ft8;
        let asm = SlotAssembler::for_protocol::<Ft8>(48_000);
        assert_eq!(asm.slot_samples(), 180_000); // 15 s @ 12 kHz
        assert_eq!(asm.input_rate(), 48_000);
        // TX_START_OFFSET_S = 0.5 -> 6000 samples, stamped onto slots.
        let mut asm = asm;
        let slot = asm
            .push(&vec![0i16; 48_000 * 15])
            .expect("15 s of 48 kHz audio fills one FT8 slot");
        assert_eq!(slot.nominal_start_sample, 6_000);
    }
}
