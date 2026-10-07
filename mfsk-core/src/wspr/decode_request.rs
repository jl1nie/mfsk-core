//! `DecodeRequest`/`SniperRequest` builder for WSPR (issue #403).
//!
//! Replaces the public `wspr::decode::decode_*` free functions. Scan:
//! `decode_scan`, `decode_scan_default`, `decode_scan_streaming`,
//! `decode_scan_with_table`. Point decode: `decode_at`,
//! `decode_at_with_drift`, `decode_at_baseband`,
//! `decode_at_baseband_nblocks`, `decode_at_baseband_nblocks_gated` and
//! `decode_at_baseband_nblocks_gated_drift` — the last of those being
//! three axes stacked into one name, each wrapper passing one more
//! argument down to the next. The deprecated `decode_scan_subtract*`
//! pair left the public API in the same change; see
//! `wspr::decode::decode_scan_subtract` (`internal-testing`).
//!
//! Same shape as [`crate::jt9::decode_request`] /
//! [`crate::jt65::decode_request`], per-mode for the same reason: WSPR
//! decodes `&[f32]` PCM, where `crate::msg::decode_request` takes
//! `&[i16]`.
//!
//! What stays public beside this: the embedded pass-2 stages
//! (`decode::rank_pass2_candidates` / `deep_decode_pass2_candidate`,
//! feature `wspr-pass2-topn`) and [`WsprCallsignTable`]. Those are the
//! pieces `embedded-shared` composes its own dual-core pipeline from —
//! pipeline stages, not a cross product of conveniences.

// Since 0.13 the wide-band request is the engine's: `crate::decoder::Decoder` is
// the decode API and this is reached only through `internal-testing`, so
// its builders are dead in a build without it.
#![cfg_attr(not(any(test, feature = "internal-testing")), allow(dead_code))]

use alloc::vec::Vec;

use super::decode::{WsprCallsignTable, WsprResult};
use super::search::{SearchParams, default_search_params};
use crate::engine::pipeline::BudgetCheck;

/// Wide-band WSPR decode request: wsprd's coarse search on the 375 Hz
/// baseband, then its three decode passes (Fano, then the Fano + OSD
/// ladder on the `minsync2` survivors).
///
/// ```no_run
/// use mfsk_core::wspr::DecodeRequest;
///
/// # let audio: Vec<f32> = vec![];
/// // `audio` is ~1.44M f32 samples at 12 kHz (120 s slot).
/// for r in DecodeRequest::new(&audio, 12_000).decode() {
///     println!("{:+7.1} Hz  dt={:+.1}  {}", r.freq_hz, r.dt_sec, r.message);
/// }
/// ```
pub struct DecodeRequest<'a> {
    audio: &'a [f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: SearchParams,
    table: Option<&'a mut WsprCallsignTable>,
    on_result: Option<&'a (dyn Fn(&WsprResult) + Sync)>,
    depth: super::decode::ScanDepth,
    budget: Option<BudgetCheck<'a>>,
}

impl<'a> DecodeRequest<'a> {
    /// Scan the whole buffer with [`default_search_params`], nominal
    /// start at sample 0, no cross-slot table.
    pub fn new(audio: &'a [f32], sample_rate: u32) -> Self {
        Self {
            audio,
            sample_rate,
            nominal_start_sample: 0,
            params: default_search_params(),
            table: None,
            on_result: None,
            depth: super::decode::ScanDepth::DEFAULT,
            budget: None,
        }
    }

    /// Single-target request at a known alignment on 12 kHz audio. Same
    /// convention as [`crate::q65::DecodeRequest::sniper`]; see
    /// [`SniperRequest::baseband`] for the pre-decimated form.
    pub fn sniper(
        audio: &'a [f32],
        sample_rate: u32,
        start_sample: usize,
        freq_hz: f32,
    ) -> SniperRequest<'a> {
        SniperRequest::new(audio, sample_rate, start_sample, freq_hz)
    }

    /// Sample index where the slot nominally starts. Default 0.
    /// What the scan does per `wsprd`'s arguments; the default is its own.
    pub(crate) fn scan_depth(mut self, depth: super::decode::ScanDepth) -> Self {
        self.depth = depth;
        self
    }

    pub fn nominal_start(mut self, sample: usize) -> Self {
        self.nominal_start_sample = sample;
        self
    }

    /// Search window and candidate cap. Default [`default_search_params`].
    pub fn params(mut self, params: SearchParams) -> Self {
        self.params = params;
        self
    }

    /// A caller-owned [`WsprCallsignTable`] that persists **across
    /// slots**. The scan reads it to gate OSD and records this slot's
    /// decodes into it.
    ///
    /// This is what makes OSD worth having. Within a single slot the
    /// table can only be populated by that slot's own pass-1 Fano
    /// decodes, so a station too weak for Fano anywhere in the file is
    /// simply lost — on `150426_0918.wav` that costs W3BI at -25 dB,
    /// which real `wsprd` does report. `wsprd` gets it because its
    /// `hashtab` outlives the file: it is carried across decode passes
    /// *and* persisted to `hashtable.txt` between invocations, so a
    /// station confirmed once stays confirmable.
    ///
    /// A WSPR receiver sees the same beacons every 2 minutes for hours.
    /// Feed the same table back in each slot and OSD recovers those
    /// stations in slots where Fano cannot reach them — with the phantom
    /// risk still closed, because OSD can only ever re-find a callsign
    /// some Fano decode already established.
    ///
    /// The table grows by one entry per distinct station heard; a busy
    /// band is a few hundred entries over a session, so callers can hold
    /// it for the whole session without managing its size.
    pub fn table(mut self, table: &'a mut WsprCallsignTable) -> Self {
        self.table = Some(table);
        self
    }

    /// Fire `cb` once per candidate as it's accepted, *in addition to*
    /// (not instead of) `decode()`'s returned `Vec` — same shape as
    /// `crate::msg::decode_request::DecodeRequest::on_result`.
    ///
    /// **Delivery order/dedup contract — weaker than JT9's or JT65's**:
    /// both pass 1 and pass 2's per-candidate decode step run under
    /// `rayon::par_iter()` (feature `parallel`), so this carries the
    /// completion-order/possible-duplicate caveat documented on
    /// `crate::msg::decode_request::DecodeRequest::on_result`'s
    /// parallel single-pass strategy: `cb` fires from whichever thread
    /// decoded that candidate, in completion order, *before* the
    /// dedup-then-push step that decides what lands in the returned
    /// `Vec` — a same-message duplicate found by two different
    /// candidates could fire `cb` twice even though only one survives
    /// into the batch result. `cb` must be `Sync` for this reason.
    pub fn on_result(mut self, cb: &'a (dyn Fn(&WsprResult) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// `&mut self` because a [`Self::table`] is written back to. Chains
    /// on a temporary like any other builder:
    /// `DecodeRequest::new(&a, 12_000).table(&mut t).decode()`.
    pub fn decode(&mut self) -> Vec<WsprResult> {
        self.decode_reported().0
    }

    /// Stop the scan when `check` returns `false`: it is polled once per
    /// candidate before the candidate is decoded, in every pass, and between a
    /// candidate's ladder positions. A candidate already running finishes (bar
    /// that poll), the coarse search and the per-pass refinement are not cut,
    /// and the passes after a spent budget are skipped.
    pub fn budget(mut self, check: BudgetCheck<'a>) -> Self {
        self.budget = Some(check);
        self
    }

    /// [`Self::decode`], and whether the budget ([`Self::budget`]) said stop at
    /// least once. `false` without a budget.
    pub fn decode_reported(&mut self) -> (Vec<WsprResult>, bool) {
        super::decode::decode_scan_inner(
            self.audio,
            self.sample_rate,
            self.nominal_start_sample,
            &self.params,
            self.on_result,
            self.table.as_deref_mut(),
            self.depth,
            self.budget,
        )
    }
}

/// What [`SniperRequest`] decodes from.
enum Source<'a> {
    /// 12 kHz audio, decimated per call.
    Audio(&'a [f32]),
    /// A 375 Hz baseband the caller already decimated — see
    /// [`SniperRequest::baseband`].
    Baseband { idat: &'a [f32], qdat: &'a [f32] },
}

/// Single-target WSPR decode at a known `(start_sample, freq_hz)`:
/// wsprd's refine cascade, the `minsync2` gate, then Fano and (when
/// [`Self::confirmed`] is given) OSD. Construct with
/// `DecodeRequest::sniper` for 12 kHz audio or
/// [`SniperRequest::baseband`] for a pre-decimated one.
///
/// `freq_hz` is the tone-0 frequency in audio Hz — this crate's coarse
/// search and synthesiser convention; the decoder converts it to
/// wsprd's tone-centre internally. `start_sample` is the audio-rate
/// sample where symbol 0 starts.
///
/// [`WsprResult::snr_db`] is `0.0` here: the SNR comes from a coarse
/// candidate, and a point decode has none.
pub struct SniperRequest<'a> {
    source: Source<'a>,
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &'a [usize],
    confirmed: Option<&'a WsprCallsignTable>,
    refine_drift: bool,
}

impl<'a> SniperRequest<'a> {
    /// From 12 kHz audio. The audio is decimated with the reference
    /// whole-slot FFT channelizer on every [`Self::decode`], regardless
    /// of the `wspr-ddc*` features; decode many candidates from one
    /// buffer through [`Self::baseband`] instead.
    pub fn new(audio: &'a [f32], sample_rate: u32, start_sample: usize, freq_hz: f32) -> Self {
        Self::with_source(Source::Audio(audio), sample_rate, start_sample, freq_hz)
    }

    /// From a 375 Hz baseband already produced by
    /// [`super::baseband::decimate_to_baseband`] (or one of the
    /// `wspr::ddc` down-converters). Skips the O(NFFT1) decimation, which
    /// is what the scan and the embedded pipelines need when many
    /// candidates share one slot.
    pub fn baseband(
        idat: &'a [f32],
        qdat: &'a [f32],
        sample_rate: u32,
        start_sample: usize,
        freq_hz: f32,
    ) -> Self {
        Self::with_source(
            Source::Baseband { idat, qdat },
            sample_rate,
            start_sample,
            freq_hz,
        )
    }

    fn with_source(
        source: Source<'a>,
        sample_rate: u32,
        start_sample: usize,
        freq_hz: f32,
    ) -> Self {
        Self {
            source,
            sample_rate,
            start_sample,
            freq_hz,
            drift_hz: 0.0,
            nblocks: &[1],
            confirmed: None,
            refine_drift: true,
        }
    }

    /// Linear drift estimate, in Hz across the 110.6 s frame — wsprd's
    /// `drift1`. Default 0. The refine cascade searches ±0.5 Hz around
    /// it while [`Self::refine_drift`] is on.
    pub fn drift(mut self, drift_hz: f32) -> Self {
        self.drift_hz = drift_hz;
        self
    }

    /// Coherent block lengths to try, in order (wsprd's `nblocksize`
    /// ladder). Default `&[1]`; the scan's final pass uses
    /// `&[1, 2, 3, 0]`, where `0` is this crate's spelling of wsprd's
    /// fourth rung (`ib == 4`: block size 1 with the alternate bit
    /// metric). The hot loop scales with the slice length.
    pub fn nblocks(mut self, nblocks: &'a [usize]) -> Self {
        self.nblocks = nblocks;
        self
    }

    /// Callsigns an earlier Fano decode already established. An OSD
    /// result is accepted only if its callsign is in this table —
    /// wsprd's `hashtab` check (`wsprd.c:1396`). Without it every OSD
    /// result is rejected: OSD synthesises a valid codeword for any
    /// input, so ungated it is a phantom generator. See
    /// [`WsprCallsignTable`] for the measurement behind this.
    pub fn confirmed(mut self, table: &'a WsprCallsignTable) -> Self {
        self.confirmed = Some(table);
        self
    }

    /// wsprd's per-pass drift switch (`wsprd.c:1236`, `if (ipass < 2)`).
    /// On (the default), the cascade tries `drift ± 0.5` and keeps the
    /// better sync, and `minsync2` is 0.12. Off, as on wsprd's final
    /// pass, drift is held fixed and `minsync2` drops to 0.10 — trading
    /// drift tracking for a lower-variance frequency estimate on weak
    /// signals.
    pub fn refine_drift(mut self, on: bool) -> Self {
        self.refine_drift = on;
        self
    }

    /// `None` when the refined sync misses `minsync2`, Fano and OSD
    /// both fail, or the payload does not unpack.
    pub fn decode(&self) -> Option<WsprResult> {
        let decimated;
        let (idat, qdat): (&[f32], &[f32]) = match self.source {
            Source::Audio(audio) => {
                decimated = super::baseband::decimate_to_baseband(audio);
                (&decimated.0, &decimated.1)
            }
            Source::Baseband { idat, qdat } => (idat, qdat),
        };
        super::decode::decode_at_baseband_inner(
            idat,
            qdat,
            self.sample_rate,
            self.start_sample,
            self.freq_hz,
            self.drift_hz,
            self.nblocks,
            self.confirmed,
            self.refine_drift,
            super::decode::Ladder::DEFAULT,
        )
    }
}
