//! `DecodeRequest`/`SniperRequest` builder for JT65 (issue #403).
//!
//! Replaces the nine `jt65::decode_*` free functions — `decode_scan`,
//! `decode_scan_default`, `decode_scan_streaming`, `decode_scan_chase`,
//! `decode_scan_chase_default`, `decode_scan_chase_streaming`,
//! `decode_at`, `decode_at_with_erasures` and `decode_at_with_chase` —
//! which were two axes (chase or not, streaming or not) spelled out as a
//! cross product. Same shape as [`crate::jt9::decode_request`] and
//! [`crate::q65::decode_request`], and per-mode for the same reason: JT65
//! decodes `&[f32]` PCM, where `crate::msg::decode_request` takes
//! `&[i16]`.

use alloc::vec::Vec;

use crate::msg::Jt72Message;

use super::Jt65Result;
use super::chase::ChaseParams;
use super::search::{SearchParams, default_search_params};

/// Wide-band JT65 decode request: coarse (freq, time) search over
/// `params`, then Reed-Solomon on each candidate in score order,
/// collapsing duplicates (same message ±2 Hz / ±1 symbol).
///
/// Each candidate is decoded hard-decision by default, or with WSJT-X's
/// `ftrsdap` stochastic Chase search once [`Self::chase`] is set —
/// slower, and 1 dB deeper at the 50% crossing on the AWGN sweep
/// (−23.5 against −22.5 dB, `docs/notes/sweep-baseline.json`).
///
/// ```no_run
/// use mfsk_core::jt65::{ChaseParams, DecodeRequest};
///
/// # let audio: Vec<f32> = vec![];
/// // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
/// for r in DecodeRequest::new(&audio, 12_000)
///     .chase(ChaseParams::default())
///     .decode()
/// {
///     println!("{:+7.1} Hz  dt={:+.1}  {}", r.freq_hz, r.dt_sec, r.message);
/// }
/// ```
pub struct DecodeRequest<'a> {
    audio: &'a [f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: SearchParams,
    chase: Option<ChaseParams>,
    on_result: Option<&'a (dyn Fn(&Jt65Result) + Sync)>,
}

impl<'a> DecodeRequest<'a> {
    /// Scan the whole buffer with [`default_search_params`], nominal
    /// start at sample 0, hard-decision RS.
    pub fn new(audio: &'a [f32], sample_rate: u32) -> Self {
        Self {
            audio,
            sample_rate,
            nominal_start_sample: 0,
            params: default_search_params(),
            chase: None,
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

    /// Sample index where the slot nominally starts.
    /// [`Jt65Result::dt_sec`] is measured from here (#397). Default 0.
    pub fn nominal_start(mut self, sample: usize) -> Self {
        self.nominal_start_sample = sample;
        self
    }

    /// Search window and candidate cap. Default [`default_search_params`].
    pub fn params(mut self, params: SearchParams) -> Self {
        self.params = params;
        self
    }

    /// Decode each candidate with the stochastic Chase search
    /// ([`super::chase`]) instead of plain hard-decision RS.
    pub fn chase(mut self, params: ChaseParams) -> Self {
        self.chase = Some(params);
        self
    }

    /// Fire `cb` once per candidate as it's accepted, *in addition to*
    /// (not instead of) `decode()`'s returned `Vec` — same shape as
    /// `crate::msg::decode_request::DecodeRequest::on_result`.
    ///
    /// **Delivery order/dedup contract**: the candidate loop is
    /// sequential with no early exit and no parallelism — `cb` fires
    /// exactly once per result that ends up in the returned `Vec`, in
    /// the same order.
    pub fn on_result(mut self, cb: &'a (dyn Fn(&Jt65Result) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    pub fn decode(&self) -> Vec<Jt65Result> {
        super::decode_scan_inner(
            self.audio,
            self.sample_rate,
            self.nominal_start_sample,
            &self.params,
            self.chase.as_ref(),
            self.on_result,
        )
    }
}

/// How [`SniperRequest`] runs Reed-Solomon. The last of
/// [`SniperRequest::erasures`] / [`SniperRequest::chase`] called wins.
enum RsStrategy<'a> {
    Hard,
    Erasures(&'a [usize]),
    Chase(ChaseParams),
}

/// Single-target JT65 decode at a known `(start_sample, base_freq_hz)`.
/// Construct with [`DecodeRequest::sniper`] or [`SniperRequest::new`].
///
/// Returns the message alone, as the point decodes always have; it is
/// what `mfsk-ffi`'s `mfsk_jt65_decode_at` wraps.
pub struct SniperRequest<'a> {
    audio: &'a [f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    strategy: RsStrategy<'a>,
}

impl<'a> SniperRequest<'a> {
    pub fn new(audio: &'a [f32], sample_rate: u32, start_sample: usize, base_freq_hz: f32) -> Self {
        Self {
            audio,
            sample_rate,
            start_sample,
            base_freq_hz,
            strategy: RsStrategy::Hard,
        }
    }

    /// Flag the least-confident symbols as RS erasures, trying each
    /// count in `attempts` in order until one unpacks into a valid
    /// message. Each erasure buys one more correctable symbol
    /// (`2·errors + erasures ≤ 51`); `&[0, 8, 16, 24, 32]` is a
    /// reasonable ladder. Counts above 51 are clamped.
    pub fn erasures(mut self, attempts: &'a [usize]) -> Self {
        self.strategy = RsStrategy::Erasures(attempts);
        self
    }

    /// WSJT-X's `ftrsdap` stochastic Chase search — randomized erasure
    /// trials with its own acceptance gate. See [`super::chase`].
    pub fn chase(mut self, params: ChaseParams) -> Self {
        self.strategy = RsStrategy::Chase(params);
        self
    }

    /// `None` when RS does not converge, the acceptance gate rejects
    /// it, or the payload does not unpack.
    pub fn decode(&self) -> Option<Jt72Message> {
        let (audio, rate, start, freq) = (
            self.audio,
            self.sample_rate,
            self.start_sample,
            self.base_freq_hz,
        );
        match &self.strategy {
            RsStrategy::Hard => super::decode_at_with_snr(audio, rate, start, freq).map(|(m, _)| m),
            RsStrategy::Erasures(attempts) => {
                super::decode_at_with_erasures(audio, rate, start, freq, attempts)
            }
            RsStrategy::Chase(params) => {
                super::chase::decode_at_with_chase(audio, rate, start, freq, params).map(|(m, _)| m)
            }
        }
    }
}
