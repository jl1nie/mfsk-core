// SPDX-License-Identifier: GPL-3.0-only
//! FST4W as a [`Decoder`]: `lib/fst4_decode.f90` with `iwspr=1`, WSJT-X
//! `v3.3.0-beta1`. The ladder itself is [`crate::fst4w::decode`]; this file is
//! the parameter block, the rows and the cross-period state.
//!
//! What a field of the block means here:
//!
//! - `rx_freq_hz` / `tol_hz` (`nfqso`, `ntol`) are the search window,
//!   `nfqso ± ntol` (`fst4_decode.f90:517-521`); without them the block's band
//!   gives the same window (its centre and half width).
//! - `depth` is `ndepth`: Deep runs Keff 50 and the `i0 ± 1` retry, Normal the
//!   retry only, Fast neither (`:485-499`).
//! - AP, the station and the QSO context are not read: FST4W has no a-priori
//!   decoding (`:674-677`).
//!
//! The state is the callsign hash table and the Keff-50 known-call list
//! ([`Decoder::wcalls`], [`Decoder::set_wcalls`]), which a host keeps across
//! runs as WSJT-X keeps `fst4w_calls.txt`.

use alloc::string::String;
use alloc::vec::Vec;

use super::{
    Decodable, DecodeParams, Decoder, OnRow, Row, RowDetail, SlotInput, SlotResult, frame,
};
use crate::engine::ProtocolId;
use crate::fst4w::decode::{Fst4wPeriod, Fst4wResult, Fst4wState, SlotSettings, TooManyCalls};
use crate::fst4w::{Fst4w120, Fst4w300, Fst4w900, Fst4w1800, Fst4wMessage};
use crate::msg::Decoded;
use crate::msg::wsjt77::unpack77;

/// Options the library adds beyond upstream. None yet for FST4W.
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct Fst4wExtras {}

fn row_of(r: &Fst4wResult) -> Row<Fst4wResult> {
    let plain = Fst4wMessage::payload_to_77(&r.info[..50]).and_then(|b| unpack77(&b));
    Row {
        decoded: Decoded::new(
            r.text.clone(),
            r.freq_hz,
            r.dt_sec,
            r.snr_db,
            ProtocolId::Fst4w,
        )
        .with_hash22(r.hash22),
        detail: RowDetail {
            sync_score: Some(r.sync_score),
            sync_cv: Some(r.sync_cv),
            hard_errors: Some(r.hard_errors),
            pass: r.variant,
            info: r.info.to_vec(),
            hash_resolved: plain.as_deref() != Some(r.text.as_str()),
            copied_last_tx: false,
            delivery: None,
            stage: None,
        },
        native: r.clone(),
    }
}

fn settings(params: &DecodeParams) -> SlotSettings {
    let (lo, hi) = params.band_hz;
    SlotSettings {
        nfqso: params.rx_freq_hz.unwrap_or((lo + hi) / 2.0),
        ntol: params.tol_hz.unwrap_or((hi - lo) / 2.0),
        ndepth: params.depth.ndepth(),
    }
}

macro_rules! fst4w_decodable {
    ($ty:ty, $mode:ident) => {
        impl Decodable for $ty {
            const MODE: crate::Mode = crate::Mode::$mode;
            type State = Fst4wState;
            type Extras = Fst4wExtras;
            type Row = Fst4wResult;

            fn __learn(state: &mut Fst4wState, call: &str) -> bool {
                state.table.insert(call);
                true
            }

            fn __unpack77(state: &Fst4wState, msg77: &[u8]) -> Option<String> {
                crate::msg::wsjt77::unpack77_with_hash(msg77, &state.table)
            }

            fn __decode(
                params: &DecodeParams,
                _extras: &Fst4wExtras,
                state: &mut Fst4wState,
                slot: &SlotInput<'_>,
                on_row: Option<OnRow<'_, Fst4wResult>>,
            ) -> SlotResult<Fst4wResult> {
                decode_period::<$ty>(params, state, slot, on_row)
            }
        }
    };
}

fst4w_decodable!(Fst4w120, Fst4W120);
fst4w_decodable!(Fst4w300, Fst4W300);
fst4w_decodable!(Fst4w900, Fst4W900);
fst4w_decodable!(Fst4w1800, Fst4W1800);

fn decode_period<P: Fst4wPeriod>(
    params: &DecodeParams,
    state: &mut Fst4wState,
    slot: &SlotInput<'_>,
    on_row: Option<OnRow<'_, Fst4wResult>>,
) -> SlotResult<Fst4wResult> {
    let mut owned = Vec::new();
    let Some(audio) = frame::pcm(slot.audio, &mut owned, None) else {
        return SlotResult {
            rows: Vec::new(),
            budget: Default::default(),
        };
    };
    let wrapped = on_row.map(|cb| move |r: &Fst4wResult| cb(&row_of(r)));
    let (results, budget) = crate::fst4w::decode::decode_slot::<P>(
        audio,
        &settings(params),
        state,
        slot.budget,
        wrapped
            .as_ref()
            .map(|f| f as &(dyn Fn(&Fst4wResult) + Sync)),
    );
    SlotResult {
        rows: results.iter().map(row_of).collect(),
        budget,
    }
}

impl<P: Decodable<State = Fst4wState>> Decoder<P> {
    /// The Keff-50 known-call list, oldest first (`get_known_calls`): the
    /// `CALL GRID` of every type-1 message a Keff-66 decode has found, which
    /// is what lets a Keff-50 word (no CRC) through. A host saves it between
    /// runs as WSJT-X saves `fst4w_calls.txt`.
    pub fn wcalls(&self) -> &[String] {
        self.state.wcalls()
    }

    /// Replace the list (`set_known_calls`). At most 100 entries of 20
    /// characters; a blank entry vouches for nothing.
    pub fn set_wcalls(&mut self, calls: &[String]) -> Result<(), TooManyCalls> {
        self.state.set_wcalls(calls)
    }
}
