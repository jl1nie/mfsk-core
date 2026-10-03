//! `DecodeRequest`/`SniperRequest` builder for JT9 (issue #403).
//!
//! Replaces the six `jt9::decode_*` free functions — `decode_scan`,
//! `decode_scan_default`, `decode_scan_with_depth`,
//! `decode_scan_streaming`, `decode_scan_streaming_with_depth` and
//! `decode_at` — whose names multiplied with every axis: adding
//! [`Jt9Depth`] alone had to add both `_with_depth` and
//! `_streaming_with_depth`. Same shape as
//! [`crate::q65::decode_request`], and for the same reason it is a
//! per-mode type rather than `crate::msg::decode_request`: JT9 decodes
//! `&[f32]` PCM, where the generic builder is fixed to `&[i16]`.

// Since 0.13 the wide-band request is the engine's: `crate::decoder::Decoder` is
// the decode API and this is reached only through `internal-testing`, so
// its builders are dead in a build without it.
#![cfg_attr(not(any(test, feature = "internal-testing")), allow(dead_code))]

use alloc::vec::Vec;

use crate::msg::Jt72Message;

use super::Jt9Result;
use super::decode::Jt9Depth;
use super::search::{SearchParams, default_search_params};

/// Wide-band JT9 decode request: coarse (freq, time) search over
/// `params`, then the WSJT-X-faithful `softsym` pipeline (`downsam9` +
/// `peakdt9` + `symspec2` + Fano) on each candidate in score order,
/// collapsing duplicates that decode to the same message within ±4 Hz /
/// ±1 symbol.
///
/// ```no_run
/// use mfsk_core::jt9::DecodeRequest;
///
/// # let audio: Vec<f32> = vec![];
/// // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
/// for r in DecodeRequest::new(&audio, 12_000).decode() {
///     println!("{:+7.1} Hz  dt={:+.1}  {}", r.freq_hz, r.dt_sec, r.message);
/// }
/// ```
pub struct DecodeRequest<'a> {
    audio: &'a [f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: SearchParams,
    depth: Jt9Depth,
    narrow: Option<(f32, f32)>,
    on_result: Option<&'a (dyn Fn(&Jt9Result) + Sync)>,
}

impl<'a> DecodeRequest<'a> {
    /// Scan the whole buffer with [`default_search_params`], nominal
    /// start at sample 0, [`Jt9Depth::default`].
    pub fn new(audio: &'a [f32], sample_rate: u32) -> Self {
        Self {
            audio,
            sample_rate,
            nominal_start_sample: 0,
            params: default_search_params(),
            depth: Jt9Depth::default(),
            narrow: None,
            on_result: None,
        }
    }

    /// Single-target request at a known alignment. Same convention as
    /// [`crate::q65::DecodeRequest::sniper`].
    pub fn sniper(
        audio: &'a [f32],
        sample_rate: u32,
        start_sample: usize,
        base_freq_hz: f32,
    ) -> SniperRequest<'a> {
        SniperRequest::new(audio, sample_rate, start_sample, base_freq_hz)
    }

    /// Sample index where the slot nominally starts. [`Jt9Result::dt_sec`]
    /// is measured from here (#397). Default 0.
    pub fn nominal_start(mut self, sample: usize) -> Self {
        self.nominal_start_sample = sample;
        self
    }

    /// Search window and candidate cap. Default [`default_search_params`].
    pub fn params(mut self, params: SearchParams) -> Self {
        self.params = params;
        self
    }

    /// Fano cycle budget per candidate — WSJT-X's `-d` / "Decode Again"
    /// tiers. See [`Jt9Depth`] for the measured tradeoff; the default
    /// is `Fast`.
    pub fn depth(mut self, depth: Jt9Depth) -> Self {
        self.depth = depth;
        self
    }

    /// Fire `cb` once per candidate as it's accepted, *in addition to*
    /// (not instead of) `decode()`'s returned `Vec` — same shape as
    /// `crate::msg::decode_request::DecodeRequest::on_result`.
    ///
    /// **Delivery order/dedup contract**: the candidate loop is
    /// sequential with no early exit and no parallelism — `cb` fires
    /// exactly once per result that ends up in the returned `Vec`, in
    /// the same order. Candidates are tried in coarse-score-descending
    /// order, so `cb` tends to see stronger signals first; that is a
    /// correlation, not a guarantee.
    /// WSJT-X's Rx frequency and tolerance (`nfqso`, `ntol`): candidates
    /// within `tol_hz` of `rx_hz` that the wide scan's sync gates reject are
    /// tried again with the looser ones and the deepest Fano limit, as
    /// `jt9_decode.f90`'s `nqd = 1` pass does. Off by default.
    pub fn narrow(mut self, rx_hz: f32, tol_hz: f32) -> Self {
        self.narrow = Some((rx_hz, tol_hz));
        self
    }

    pub fn on_result(mut self, cb: &'a (dyn Fn(&Jt9Result) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    pub fn decode(&self) -> Vec<Jt9Result> {
        super::decode_scan_inner(
            self.audio,
            self.sample_rate,
            self.nominal_start_sample,
            &self.params,
            self.depth,
            self.narrow,
            self.on_result,
        )
    }
}

/// Single-target JT9 decode at a known `(start_sample, base_freq_hz)`.
/// Construct with `DecodeRequest::sniper` or [`SniperRequest::new`].
///
/// This is the plain aligned demodulator plus Fano at the default cycle
/// budget, with no sync search, AFC or SNR estimate — which is why it
/// returns the message alone rather than a [`Jt9Result`]. It is what
/// `mfsk-ffi`'s `mfsk_jt9_decode_at` wraps.
pub struct SniperRequest<'a> {
    audio: &'a [f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
}

impl<'a> SniperRequest<'a> {
    pub fn new(audio: &'a [f32], sample_rate: u32, start_sample: usize, base_freq_hz: f32) -> Self {
        Self {
            audio,
            sample_rate,
            start_sample,
            base_freq_hz,
        }
    }

    /// `None` when Fano does not converge or the payload does not unpack.
    pub fn decode(&self) -> Option<Jt72Message> {
        super::decode_at_inner(
            self.audio,
            self.sample_rate,
            self.start_sample,
            self.base_freq_hz,
        )
    }
}
