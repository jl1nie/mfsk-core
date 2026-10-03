// SPDX-License-Identifier: GPL-3.0-or-later
//! [`Decodable`] for the 77-bit frame family: FT8, FT4 and the five FST4
//! T/R periods.
//!
//! [`Depth`] is mapped to search settings per mode after WSJT-X v3.2.0-rc1:
//!
//! | mode | Fast (1) | Normal (2) | Deep (3) | upstream |
//! |---|---|---|---|---|
//! | FT8 syncmin | 2.1 | 2.1 | 1.3 | `ft8_decode.f90:180-181` |
//! | FT8 maxcand | 1000 | 1000 | 1000 | `ft8_decode.f90:46,198` |
//! | FT8 passes | 2 | 3, with the 41/47 checkpoints | same | `ft8_decode.f90:101-107,175-177` |
//! | FT8 OSD | off (`maxosd = -1`) | on | on | `ft8b.f90:430-437` |
//! | FT8 nsync floor | 8 | 8 | 6 / 7 | `ft8b.f90:178-180` |
//! | FT4 syncmin / MAXCAND | 1.18 / 200 | same | same | `ft4_decode.f90:31,192` |
//! | FT4 subtraction passes | 1 | 3 | 3 | `ft4_decode.f90:193-203` |
//! | FT4 OSD | off | off | on | `ft4_decode.f90:196-202` |
//! | FST4 minsync / candidates | 1.20 (1.15 at 15 s) / 200 | same | same | `fst4_decode.f90:53,308-309` |
//!
//! The library's own settings ([`Tuning`]) override these only when set.
//!
//! Measured against real `jt9 -8 -dN` on `qso3_busy.wav` (when this mapping
//! was the `wsjtx_depth` constructor): 19 / 22 decodes for jt9 `-d2` / `-d3`
//! against 22 / 22 here. On the busy-band corpus a sync of 1.3 keeps
//! `SicEarly`'s recall while cutting its unexpected decodes from 22 (at 0.8)
//! to 6; 2.1 costs 3-4 points of recall and gains nothing more
//! (`docs/notes/BENCHMARKS.md`, "The busy-band corpus"). The crate's OSD is
//! upstream's `maxosd > 0` branch at every depth that has OSD.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::{
    Audio, Decodable, DecodeParams, Depth, F32_TO_I16_RMS, OnRow, Row, SlotInput, SlotResult,
};
use crate::engine::equalize::EqMode;
use crate::engine::pipeline::{DecodeResult, DecodeStrictness};
use crate::engine::protocol::Protocol;
use crate::msg::ApHint;
use crate::msg::decode_request::{
    DecodeOutcome, DecodeRequest, FrameDecodable, SupportsMessageFilter,
};
use crate::msg::decoded::Decoded;
use crate::msg::hash_table::CallsignHashTable;
use crate::msg::wsjt77::{Wsjt77Fields, unpack77_learn, unpack77_with_hash};

/// Which messages a decode delivers beyond (or instead of) the codec's own
/// verdict. `fn` pointers, so the extras stay `Clone` and `'static`; each
/// variant runs its own monomorphised copy of the engine, and `Default`
/// costs nothing.
#[derive(Clone, Copy, Debug, Default)]
pub enum MessageFilter {
    /// The codec's verdict, with the protocol's own default filter.
    #[default]
    Default,
    /// The codec's verdict alone.
    Codec,
    /// The codec's verdict, plus anything `f` accepts.
    AlsoAccept(fn(&Wsjt77Fields) -> bool),
    /// Only what `f` accepts.
    Only(fn(&Wsjt77Fields) -> bool),
}

/// The library's search settings, overriding what [`Depth`] sets. `None`
/// keeps the depth's value. Used where WSJT-X's depths do not fit: an
/// embedded budget (few candidates, one pass), or a measurement that pins
/// one knob.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tuning<S> {
    pub sync_min: Option<f32>,
    pub max_cand: Option<usize>,
    pub osd: Option<bool>,
    pub strictness: Option<DecodeStrictness>,
    pub strategy: Option<S>,
}

impl<S> Default for Tuning<S> {
    fn default() -> Self {
        Self {
            sync_min: None,
            max_cand: None,
            osd: None,
            strictness: None,
            strategy: None,
        }
    }
}

/// FT8's search strategies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ft8Strategy {
    /// One pass, no subtraction.
    SinglePass,
    /// `n` flat passes, subtracting each pass's decodes (`jt9 -d1` is 2).
    SicRounds(usize),
    /// WSJT-X's checkpointed passes at 41/47/50 symbols, run on the whole
    /// period (`jt9 -d2/-d3`).
    SicEarly,
}

/// FT4's search strategies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ft4Strategy {
    SinglePass,
    /// `n` passes, subtracting each pass's decodes (`nsp`).
    SicRounds(usize),
}

/// FST4 has one strategy; `fst4_decode.f90` has no subtraction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fst4Strategy {
    SinglePass,
}

/// FT8's roofing-filter mode: the audio is already narrowed by the radio's
/// analogue filter around [`DecodeParams::rx_freq_hz`], and the search is
/// confined to `± search_hz` of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sniper {
    pub search_hz: f32,
}

impl Default for Sniper {
    fn default() -> Self {
        Self { search_hz: 250.0 }
    }
}

/// FT8's library options.
#[derive(Clone, Debug, Default)]
pub struct Ft8Extras {
    pub tuning: Tuning<Ft8Strategy>,
    /// A free-form AP hint beside upstream's QSO-context AP.
    pub ap_hint: Option<ApHint>,
    pub eq: EqMode,
    pub filter: MessageFilter,
    /// Roofing-filter mode; needs `rx_freq_hz`.
    pub sniper: Option<Sniper>,
    /// WSJT-X's a7 list decoder (`ft8_a7.f90`), fed by the decoder's own
    /// decodes of period `n - 2`.
    pub a7: bool,
}

/// FT4's library options.
#[derive(Clone, Debug, Default)]
pub struct Ft4Extras {
    pub tuning: Tuning<Ft4Strategy>,
    pub ap_hint: Option<ApHint>,
    pub eq: EqMode,
    pub filter: MessageFilter,
}

/// FST4's library options.
#[derive(Clone, Debug, Default)]
pub struct Fst4Extras {
    pub tuning: Tuning<Fst4Strategy>,
    pub ap_hint: Option<ApHint>,
    pub eq: EqMode,
    pub filter: MessageFilter,
    /// WSJT-X's NB setting for FST4 (`nexp_decode / 256 - 3`).
    pub noise_blanker: Option<crate::msg::decode_request::NoiseBlanker>,
}

/// What a frame-family decoder keeps across periods: the callsign hash
/// table (`packjt77`), and for FT8 with a7 on, the decodes of the last two
/// periods.
#[derive(Default)]
pub struct FrameState {
    table: CallsignHashTable,
    recent: Vec<(i64, Vec<DecodeResult>)>,
}

impl FrameState {
    pub fn hash_table(&self) -> &CallsignHashTable {
        &self.table
    }
}

/// The search a depth sets, before [`Tuning`].
struct Search<S> {
    sync_min: f32,
    max_cand: usize,
    osd: bool,
    strategy: S,
    /// FT8's `ndepth <= 2` nsync floor.
    #[cfg_attr(not(feature = "ft8"), allow(dead_code))]
    low_depth: bool,
}

impl<S: Copy> Search<S> {
    fn tuned(mut self, t: &Tuning<S>) -> (Self, DecodeStrictness) {
        if let Some(v) = t.sync_min {
            self.sync_min = v;
        }
        if let Some(v) = t.max_cand {
            self.max_cand = v;
        }
        if let Some(v) = t.osd {
            self.osd = v;
        }
        if let Some(v) = t.strategy {
            self.strategy = v;
        }
        (self, t.strictness.unwrap_or_default())
    }
}

/// The 16-bit audio the frame engines take. `F32` is scaled to
/// [`F32_TO_I16_RMS`]; silence or NaN gives `None`.
fn pcm<'a>(audio: Audio<'a>, owned: &'a mut Vec<i16>) -> Option<&'a [i16]> {
    match audio {
        Audio::I16(a) => Some(a),
        Audio::F32(a) => {
            let g = f32_gain(a)?;
            // Two steps, as the IQ path has always scaled: to the `f32`
            // level the other engines take, then to 16 bits.
            owned.clear();
            owned.extend(
                a.iter()
                    .map(|&v| ((v * g) * 32_768.0).round().clamp(-32_768.0, 32_767.0) as i16),
            );
            Some(owned)
        }
    }
}

/// The gain taking `a` to an RMS of `F32_TO_I16_RMS / 32768`, or `None`
/// for silence or NaN.
pub(crate) fn f32_gain(a: &[f32]) -> Option<f32> {
    let n = a.len() as f32;
    let rms = (a.iter().map(|v| v * v).sum::<f32>() / n).sqrt();
    if rms.is_nan() || rms <= 0.0 {
        return None;
    }
    let g = F32_TO_I16_RMS / 32_768.0 / rms;
    // Audio already at the level (the IQ receiver hands slots over scaled to
    // it) is converted as it is, not regained by a rounding error that would
    // move an `i16` sample by one now and then.
    Some(if (g - 1.0).abs() < 1e-3 { 1.0 } else { g })
}

/// The interim AP hint from the parameter block: with AP on and no QSO
/// context, upstream's only hypothesis is CQ (`iaptype = 1`). The full
/// QSO-context tables replace this.
fn block_ap(params: &DecodeParams, allowed: bool) -> Option<ApHint> {
    (allowed && params.ap != super::ApMode::Off).then(|| ApHint::new().with_call1("CQ"))
}

fn run<P: SupportsMessageFilter>(
    req: DecodeRequest<'_, P>,
    filter: MessageFilter,
) -> DecodeOutcome<P> {
    match filter {
        MessageFilter::Default => req.decode(),
        MessageFilter::Codec => req.codec_filter().decode(),
        MessageFilter::AlsoAccept(f) => req.also_accept(f).decode(),
        MessageFilter::Only(f) => req.message_filter(f).decode(),
    }
}

/// The wide-band request every frame mode starts from.
fn base_request<'a, P>(
    pcm: &'a [i16],
    params: &DecodeParams,
    sync_min: f32,
    max_cand: usize,
    osd: bool,
    strictness: DecodeStrictness,
    eq: EqMode,
    slot: &SlotInput<'a>,
    on_result: Option<&'a (dyn Fn(&DecodeResult) + Sync)>,
) -> DecodeRequest<'a, P>
where
    P: FrameDecodable<DecodeResult = DecodeResult>,
{
    let mut req =
        DecodeRequest::<P>::new(pcm, params.band_hz.0, params.band_hz.1, sync_min, max_cand)
            .osd(osd)
            .strictness(strictness)
            .eq_mode(eq);
    if let Some(f) = params.rx_freq_hz {
        req = req.freq_hint(f);
    }
    if let Some(f) = params.tx_freq_hz {
        req = req.tx_freq(f);
    }
    if let Some(cb) = on_result {
        req = req.on_result(cb);
    }
    if let Some(b) = slot.budget {
        req = req.budget(b);
    }
    req
}

/// Turn the engine's results into rows: resolve each against the table,
/// then learn from it, in decode order (`unpack77_learn`).
fn rows<P: Protocol>(
    results: Vec<DecodeResult>,
    table: &mut CallsignHashTable,
) -> Vec<Row<DecodeResult>> {
    results
        .into_iter()
        .filter_map(|r| {
            let text = unpack77_learn(r.message77(), table)?;
            Some(Row {
                decoded: Decoded {
                    text,
                    freq_hz: r.freq_hz,
                    dt_sec: r.dt_sec,
                    snr_db: r.snr_db,
                    protocol: P::ID,
                },
                native: r,
            })
        })
        .collect()
}

/// Run one frame-family decode: convert the audio, wrap `on_row` so it
/// sees resolved rows, call `go`, then turn the results into rows.
fn frame_decode<P, F>(
    state: &mut FrameState,
    slot: &SlotInput<'_>,
    on_row: Option<OnRow<'_, DecodeResult>>,
    go: F,
) -> (SlotResult<DecodeResult>, Vec<DecodeResult>)
where
    P: Protocol,
    F: for<'b> FnOnce(
        &'b [i16],
        Option<&'b (dyn Fn(&DecodeResult) + Sync)>,
        &'b [DecodeResult],
    ) -> DecodeOutcome<P>,
    P: FrameDecodable<DecodeResult = DecodeResult>,
{
    let mut owned = Vec::new();
    let Some(audio) = pcm(slot.audio, &mut owned) else {
        return (
            SlotResult {
                rows: Vec::new(),
                budget: Default::default(),
            },
            Vec::new(),
        );
    };
    let previous: &[DecodeResult] = match slot.period {
        Some(n) => state
            .recent
            .iter()
            .find(|(p, _)| *p == n - 2)
            .map(|(_, r)| r.as_slice())
            .unwrap_or(&[]),
        None => &[],
    };
    let table = &state.table;
    let wrapped = on_row.map(|cb| {
        move |r: &DecodeResult| {
            if let Some(text) = unpack77_with_hash(r.message77(), table) {
                cb(&Row {
                    decoded: Decoded {
                        text,
                        freq_hz: r.freq_hz,
                        dt_sec: r.dt_sec,
                        snr_db: r.snr_db,
                        protocol: P::ID,
                    },
                    native: r.clone(),
                });
            }
        }
    });
    let out = go(
        audio,
        wrapped
            .as_ref()
            .map(|f| f as &(dyn Fn(&DecodeResult) + Sync)),
        previous,
    );
    let results = out.results.clone();
    (
        SlotResult {
            rows: rows::<P>(out.results, &mut state.table),
            budget: out.budget,
        },
        results,
    )
}

/// Keep `results` as period `n`'s, holding only the two latest periods.
#[cfg(feature = "ft8")]
fn remember(state: &mut FrameState, period: Option<i64>, results: Vec<DecodeResult>) {
    let Some(n) = period else { return };
    state.recent.retain(|(p, _)| *p > n - 2 && *p != n);
    state.recent.push((n, results));
}

#[cfg(feature = "ft8")]
fn ft8_search(depth: Depth) -> Search<Ft8Strategy> {
    // `ft8_decode.f90:175-181`, `ft8b.f90:178-180,430-437`.
    match depth {
        Depth::Fast => Search {
            sync_min: 2.1,
            max_cand: 1000,
            osd: false,
            strategy: Ft8Strategy::SicRounds(2),
            low_depth: true,
        },
        Depth::Normal => Search {
            sync_min: 2.1,
            max_cand: 1000,
            osd: true,
            strategy: Ft8Strategy::SicEarly,
            low_depth: true,
        },
        Depth::Deep => Search {
            sync_min: 1.3,
            max_cand: 1000,
            osd: true,
            strategy: Ft8Strategy::SicEarly,
            low_depth: false,
        },
    }
}

#[cfg(feature = "ft8")]
impl Decodable for crate::Ft8 {
    const MODE: crate::Mode = crate::Mode::Ft8;
    type State = FrameState;
    type Extras = Ft8Extras;
    type Row = DecodeResult;

    fn __decode(
        params: &DecodeParams,
        extras: &Ft8Extras,
        state: &mut FrameState,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, DecodeResult>>,
    ) -> SlotResult<DecodeResult> {
        let (s, strictness) = ft8_search(params.depth).tuned(&extras.tuning);
        let ap = extras.ap_hint.clone().or_else(|| block_ap(params, true));
        let (out, results) =
            frame_decode::<crate::Ft8, _>(state, slot, on_row, |pcm, cb, previous| {
                if let (Some(sn), Some(target)) = (extras.sniper, params.rx_freq_hz) {
                    let mut req = DecodeRequest::<crate::Ft8>::sniper(pcm, target, s.max_cand)
                        .search_hz(sn.search_hz)
                        .sync_min(s.sync_min)
                        .osd(s.osd)
                        .strictness(strictness)
                        .eq_mode(extras.eq);
                    if let Some(ap) = ap.as_ref() {
                        req = req.ap_hint(ap);
                    }
                    if let Some(cb) = cb {
                        req = req.on_result(cb);
                    }
                    if let Some(b) = slot.budget {
                        req = req.budget(b);
                    }
                    return match extras.filter {
                        MessageFilter::Default => req.decode(),
                        MessageFilter::Codec => req.codec_filter().decode(),
                        MessageFilter::AlsoAccept(f) => req.also_accept(f).decode(),
                        MessageFilter::Only(f) => req.message_filter(f).decode(),
                    };
                }
                let mut req = base_request::<crate::Ft8>(
                    pcm, params, s.sync_min, s.max_cand, s.osd, strictness, extras.eq, slot, cb,
                )
                .contest(params.contest != super::Contest::None)
                .eme_delay(params.eme_delay);
                req.wsjtx_low_depth = s.low_depth;
                req = match s.strategy {
                    Ft8Strategy::SinglePass => req.single_pass(),
                    Ft8Strategy::SicRounds(n) => req.sic_rounds(n),
                    Ft8Strategy::SicEarly => req.sic_early(),
                };
                if let Some(ap) = ap.as_ref() {
                    req = req.ap_hint(ap);
                }
                if extras.a7 {
                    req = req.previous_cycle(previous);
                }
                run(req, extras.filter)
            });
        if extras.a7 {
            remember(state, slot.period, results);
        }
        out
    }
}

#[cfg(feature = "ft4")]
fn ft4_search(depth: Depth) -> Search<Ft4Strategy> {
    // `ft4_decode.f90:31,192-203`.
    let (strategy, osd) = match depth {
        Depth::Fast => (Ft4Strategy::SinglePass, false),
        Depth::Normal => (Ft4Strategy::SicRounds(3), false),
        Depth::Deep => (Ft4Strategy::SicRounds(3), true),
    };
    Search {
        sync_min: 1.18,
        max_cand: 200,
        osd,
        strategy,
        low_depth: false,
    }
}

#[cfg(feature = "ft4")]
impl Decodable for crate::Ft4 {
    const MODE: crate::Mode = crate::Mode::Ft4;
    type State = FrameState;
    type Extras = Ft4Extras;
    type Row = DecodeResult;

    fn __decode(
        params: &DecodeParams,
        extras: &Ft4Extras,
        state: &mut FrameState,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, DecodeResult>>,
    ) -> SlotResult<DecodeResult> {
        let (s, strictness) = ft4_search(params.depth).tuned(&extras.tuning);
        // `ft4_decode.f90:321-324`: no AP at depth 1.
        let ap = extras
            .ap_hint
            .clone()
            .or_else(|| block_ap(params, params.depth != Depth::Fast));
        frame_decode::<crate::Ft4, _>(state, slot, on_row, |pcm, cb, _| {
            let mut req = base_request::<crate::Ft4>(
                pcm, params, s.sync_min, s.max_cand, s.osd, strictness, extras.eq, slot, cb,
            );
            req = match s.strategy {
                Ft4Strategy::SinglePass => req.single_pass(),
                Ft4Strategy::SicRounds(n) => req.sic_rounds(n),
            };
            if let Some(ap) = ap.as_ref() {
                req = req.ap_hint(ap);
            }
            run(req, extras.filter)
        })
        .0
    }
}

#[cfg(feature = "fst4")]
macro_rules! fst4_decodable {
    ($ty:ty, $mode:ident, $minsync:expr) => {
        impl Decodable for $ty {
            const MODE: crate::Mode = crate::Mode::$mode;
            type State = FrameState;
            type Extras = Fst4Extras;
            type Row = DecodeResult;

            fn __decode(
                params: &DecodeParams,
                extras: &Fst4Extras,
                state: &mut FrameState,
                slot: &SlotInput<'_>,
                on_row: Option<OnRow<'_, DecodeResult>>,
            ) -> SlotResult<DecodeResult> {
                // `fst4_decode.f90:53,308-309,421-423`. Depth's jitter split
                // from OSD is not yet expressible: every depth runs OSD with
                // jitter.
                let search = Search {
                    sync_min: $minsync,
                    max_cand: 200,
                    osd: true,
                    strategy: Fst4Strategy::SinglePass,
                    low_depth: false,
                };
                let (s, strictness) = search.tuned(&extras.tuning);
                let ap = extras
                    .ap_hint
                    .clone()
                    .or_else(|| block_ap(params, params.depth != Depth::Fast));
                frame_decode::<$ty, _>(state, slot, on_row, |pcm, cb, _| {
                    let mut req = base_request::<$ty>(
                        pcm, params, s.sync_min, s.max_cand, s.osd, strictness, extras.eq, slot, cb,
                    );
                    if let Some(nb) = extras.noise_blanker {
                        req = req.noise_blanker(nb);
                    }
                    if let Some(ap) = ap.as_ref() {
                        req = req.ap_hint(ap);
                    }
                    run(req, extras.filter)
                })
                .0
            }
        }
    };
}

#[cfg(feature = "fst4")]
fst4_decodable!(crate::fst4::Fst4s15, Fst4S15, 1.15);
#[cfg(feature = "fst4")]
fst4_decodable!(crate::fst4::Fst4s30, Fst4S30, 1.20);
#[cfg(feature = "fst4")]
fst4_decodable!(crate::fst4::Fst4s60, Fst4S60, 1.20);
#[cfg(feature = "fst4")]
fst4_decodable!(crate::fst4::Fst4s120, Fst4S120, 1.20);
#[cfg(feature = "fst4")]
fst4_decodable!(crate::fst4::Fst4s300, Fst4S300, 1.20);
