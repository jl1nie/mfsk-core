// SPDX-License-Identifier: GPL-3.0-only
//! [`Decodable`] for the ten Q65 sub-modes.
//!
//! Across periods a Q65 decoder keeps what `q65_decode.f90` keeps: the
//! running average of the symbol spectra (`s1a`, `navg`; turned on by
//! [`DecodeParams::averaging`], WSJT-X's `ndepth & 16`), and the callsign
//! hash table. Averaging needs consecutive periods: a gap in
//! [`SlotInput::period`], or a period of `None`, restarts it.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{
    Audio, Decodable, DecodeParams, OnRow, Row, RowDetail, SearchTuning, SlotInput, SlotResult,
};
use crate::fec::qra::FadingModel;
use crate::msg::ApHint;
use crate::msg::hash_table::CallsignHashTable;
use crate::q65::decode_request::DEFAULT_FTOL_HZ;
use crate::q65::rx::MultiPeriod;
use crate::q65::{DecodeRequest, Q65Result, Q65SubMode};

/// The periods whose audio an averaging decoder keeps. The spectra average
/// saturates at `min(navg, 4)` (`q65.f90:300-304`), so a period older than
/// this has a weight of `0.75^8` (10 %) or less in it.
pub const MAX_AVERAGED_PERIODS: usize = 8;

/// Q65's library options.
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct Q65Extras {
    pub search: SearchTuning,
    /// A free-form AP hint beside upstream's QSO-context AP.
    pub ap_hint: Option<ApHint>,
    /// Candidate messages for the AP-list decode (`q65_ap`), 63 symbols each,
    /// in place of the list the decoder builds from the station and QSO
    /// (`q65_decode.f90:195-210`).
    pub ap_list: Vec<[i32; 63]>,
    /// The contest callers heard (`q65_hist2`): with `Contest::GridExchange`
    /// they join the list (`ncontest = 1`).
    pub callers: Option<crate::q65::Q65Callers>,
    /// Q65 pileup mode (`nexp_decode` bit 128).
    pub pileup: bool,
    /// Frequency drift search, in bins (`max_drift`).
    pub max_drift: u32,
    /// The fast-fading metric: model and `b90` in Hz.
    pub fading: Option<(FadingModel, f32)>,
}

/// What a Q65 decoder carries from period to period.
pub struct Q65State {
    table: Arc<CallsignHashTable>,
    avg: MultiPeriod,
    last_period: Option<i64>,
}

impl Default for Q65State {
    fn default() -> Self {
        Self {
            table: Arc::new(CallsignHashTable::new()),
            avg: MultiPeriod::default(),
            last_period: None,
        }
    }
}

impl Q65State {
    pub fn hash_table(&self) -> &CallsignHashTable {
        &self.table
    }
}

/// `q65_decode.f90:183-188` and `q65_loops.f90:27-40`: `ndepth & 3` 1 / 2 / 3.
fn grid_depth(d: super::Depth) -> crate::q65::rx::GridDepth {
    use crate::q65::rx::GridDepth;
    match d {
        super::Depth::Fast => GridDepth::Fast,
        super::Depth::Normal => GridDepth::Normal,
        super::Depth::Deep => GridDepth::Deep,
    }
}

fn row(r: Q65Result) -> Row<Q65Result> {
    Row {
        decoded: r.to_decoded(),
        detail: RowDetail {
            copied_last_tx: r.copied_last_tx,
            info: r.bits77.to_vec(),
            ..RowDetail::default()
        },
        native: r,
    }
}

fn decode<P: Q65SubMode>(
    params: &DecodeParams,
    extras: &Q65Extras,
    state: &mut Q65State,
    slot: &SlotInput<'_>,
    on_row: Option<OnRow<'_, Q65Result>>,
) -> SlotResult<Q65Result> {
    use crate::engine::FrameLayout;

    let owned: Vec<f32>;
    let audio: &[f32] = match slot.audio {
        Audio::F32(a) => a,
        Audio::I16(a) => {
            owned = a.iter().map(|&v| f32::from(v) / 32_768.0).collect();
            &owned
        }
    };
    let nominal = (<P as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let search = extras
        .search
        .apply(crate::q65::search::default_search_params(), params);
    let ftol = params.tol_hz.unwrap_or(DEFAULT_FTOL_HZ);
    let cb = on_row.map(|f| move |r: &Q65Result| f(&row(r.clone())));
    // The full-AP list: the caller's own, else upstream's from MyCall and
    // DxCall (`q65_decode.f90:195-210`, only where it is looked for: at the
    // Rx frequency, or in a contest whose callers are known).
    let built;
    let ap_list: Option<&[[i32; 63]]> = if !extras.ap_list.is_empty() {
        Some(extras.ap_list.as_slice())
    } else if params.ap != super::ApMode::Off
        && (params.rx_freq_hz.is_some() || params.contest == super::Contest::GridExchange)
        && !params.station.call.is_empty()
    {
        let (me, him, grid) = (
            params.station.call.as_str(),
            params.qso.his_call.as_str(),
            params.qso.his_grid.as_str(),
        );
        built = match (&extras.callers, params.contest) {
            (Some(c), super::Contest::GridExchange) => {
                crate::q65::contest_codewords(me, him, grid, c)
            }
            _ => crate::q65::standard_qso_codewords(me, him, grid),
        };
        (!built.is_empty()).then_some(built.as_slice())
    } else {
        None
    };

    let results: Vec<Q65Result> = match (params.averaging, slot.period) {
        (true, Some(n)) => {
            // Consecutive periods only: anything else restarts the average.
            if state.last_period.is_none_or(|p| p + 1 != n) {
                state.avg = MultiPeriod::default();
            }
            state.last_period = Some(n);
            let mut search = search;
            if params.eme_delay {
                search.time_tolerance_late_sec = search
                    .time_tolerance_late_sec
                    .max(crate::q65::search::eme_delay_late_sec::<P>());
            }
            let q3 = ap_list
                .and(params.rx_freq_hz)
                .map(|rx| crate::q65::q3::Q3Params {
                    rx_freq_hz: rx,
                    ftol_hz: ftol,
                    slot_start: nominal as i64
                        - (<P as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as i64,
                    late_sec: search.time_tolerance_late_sec,
                    drift_hz: 0.0,
                });
            let ctx = crate::q65::decode_request::ctx_from_hash_table(Some(&state.table));
            let mut d = state
                .avg
                .step::<P>(audio, 12_000, nominal, &search, ap_list, q3, &ctx);
            state.avg.forget_beyond(MAX_AVERAGED_PERIODS);
            if let Some(d) = d.as_mut() {
                d.dt_sec = (d.start_sample as f32 - nominal as f32) / 12_000.0;
                if let Some(cb) = cb.as_ref() {
                    cb(d);
                }
            }
            d.into_iter().collect()
        }
        _ => {
            state.avg = MultiPeriod::default();
            state.last_period = None;
            let mut req = DecodeRequest::<P>::new(audio, 12_000, nominal, search)
                .eme_delay(params.eme_delay)
                .ftol(ftol)
                .depth(grid_depth(params.depth))
                .pileup(extras.pileup)
                .max_drift(extras.max_drift)
                .hash_table(Arc::clone(&state.table));
            if let Some(f) = params.rx_freq_hz {
                req = req.rx_freq(f);
            }
            if let Some(h) = extras.ap_hint.as_ref() {
                req = req.ap_hint(h);
            }
            if let Some(l) = ap_list {
                req = req.ap_list(l);
            }
            if let Some((m, b90)) = extras.fading {
                req = req.fading(m, b90);
            }
            if let Some(cb) = cb.as_ref() {
                req = req.on_result(cb);
            }
            req.decode()
        }
    };
    let rows: Vec<Row<Q65Result>> = results.into_iter().map(row).collect();
    // Learn after the period, in decode order, as `unpack77_learn` does.
    for r in &rows {
        Arc::make_mut(&mut state.table).learn(&r.decoded.text);
    }
    SlotResult {
        rows,
        budget: Default::default(),
    }
}

macro_rules! q65_decodable {
    ($($ty:ty => $mode:ident),* $(,)?) => {$(
        impl Decodable for $ty {
            const MODE: crate::Mode = crate::Mode::$mode;
            type State = Q65State;
            type Extras = Q65Extras;
            type Row = Q65Result;

            fn __learn(state: &mut Q65State, call: &str) -> bool {
                Arc::make_mut(&mut state.table).insert(call);
                true
            }

            fn __unpack77(state: &Q65State, msg77: &[u8]) -> Option<alloc::string::String> {
                crate::msg::wsjt77::unpack77_with_hash(msg77, &state.table)
            }

            fn __decode(
                params: &DecodeParams,
                extras: &Q65Extras,
                state: &mut Q65State,
                slot: &SlotInput<'_>,
                on_row: Option<OnRow<'_, Q65Result>>,
            ) -> SlotResult<Q65Result> {
                decode::<$ty>(params, extras, state, slot, on_row)
            }
        }
    )*};
}

q65_decodable! {
    crate::q65::Q65a15 => Q65A15,
    crate::q65::Q65a30 => Q65A30,
    crate::q65::Q65a60 => Q65A60,
    crate::q65::Q65b60 => Q65B60,
    crate::q65::Q65c60 => Q65C60,
    crate::q65::Q65d60 => Q65D60,
    crate::q65::Q65e60 => Q65E60,
    crate::q65::Q65d120 => Q65D120,
    crate::q65::Q65e120 => Q65E120,
    crate::q65::Q65a300 => Q65A300,
}
