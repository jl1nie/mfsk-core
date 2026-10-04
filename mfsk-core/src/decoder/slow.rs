// SPDX-License-Identifier: GPL-3.0-only
//! [`Decodable`] for the modes whose engines work on `f32` audio and whose
//! messages are not the 77-bit frame: WSPR, JT9 and JT65 (Q65 has its own
//! module). Also the [`SearchTuning`] they share.
//!
//! What each keeps across periods is what its upstream decoder keeps:
//! WSPR's callsign table (`wsprd`'s `hashtable`, which is what lets OSD
//! confirm a station Fano has already heard), and nothing for JT9 and
//! JT65 outside averaging (`jt65::averaging`, `avg65`). Their
//! 72-bit messages carry no hashed calls.
//!
//! [`Depth`] reaches each engine as far as the engine exposes it today; the
//! rest of upstream's per-depth behaviour is ported mode by mode and
//! recorded in that mode's module documentation as it lands.

#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
use alloc::vec::Vec;

#[cfg(any(feature = "jt9", feature = "wspr", feature = "jt65"))]
use super::Depth;
use super::{Audio, DecodeParams, SearchTuning};
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
use super::{Decodable, OnRow, Row, RowDetail, SlotInput, SlotResult};

#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
/// `f32` audio at the level the engines take (full scale is 1.0): an `i16`
/// period is divided by 32768, an `f32` one is used as is.
fn f32_audio<'a>(audio: Audio<'a>, owned: &'a mut Vec<f32>) -> &'a [f32] {
    match audio {
        Audio::F32(a) => a,
        Audio::I16(a) => {
            owned.clear();
            owned.extend(a.iter().map(|&v| f32::from(v) / 32_768.0));
            owned
        }
    }
}

/// Samples from the period's start to the frame's `dt = 0`.
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
fn nominal_start(mode: crate::Mode) -> usize {
    #[cfg(not(feature = "std"))]
    #[allow(unused_imports)]
    use num_traits::Float;
    (mode.meta().tx_start_offset_s * 12_000.0).round() as usize
}

// ── WSPR ─────────────────────────────────────────────────────────────────

#[cfg(feature = "wspr")]
mod wspr_impl {
    use super::*;
    use crate::wspr::{DecodeRequest, Wspr, WsprCallsignTable, WsprResult};

    /// WSPR's library options.
    #[derive(Clone, Debug, Default)]
    pub struct WsprExtras {
        pub search: SearchTuning,
        /// Fano's cycle budget per bit (`wsprd -C`), over the depth's:
        /// 10000 is `wsprd`'s own default, which the GUI's Normal and Deep
        /// lower to 500 for speed. Measured: on the WSJT-X golden 500 loses
        /// G8VDQ (-23 dB), which 10000 decodes.
        pub max_cycles_per_bit: Option<u64>,
    }

    /// WSPR's cross-period state: wsprd's callsign table.
    #[derive(Default)]
    pub struct WsprState {
        table: WsprCallsignTable,
    }

    impl WsprState {
        pub fn table(&self) -> &WsprCallsignTable {
            &self.table
        }
    }

    /// The arguments the WSJT-X GUI gives `wsprd` per decoding depth
    /// (`widgets/mainwindow.cpp:2824-2826`, `wsprd.c:819-900`): Fast
    /// `-qB`, Normal `-C 500 -o 4`, Deep `-C 500 -o 4 -d`. OSD (`-o`) is the
    /// final pass's, gated on the decoder's callsign table, as in the
    /// crate's scan; Fast has two passes and so none.
    fn scan_depth(depth: Depth) -> crate::wspr::decode::ScanDepth {
        use crate::wspr::decode::{Ladder, ScanDepth};
        match depth {
            Depth::Fast => ScanDepth {
                passes: 2,
                ladder: Ladder {
                    jitter: false,
                    ..Ladder::DEFAULT
                },
                more_candidates: false,
            },
            Depth::Normal | Depth::Deep => ScanDepth {
                passes: 3,
                ladder: Ladder {
                    jitter: true,
                    max_cycles_per_bit: 500,
                },
                more_candidates: depth == Depth::Deep,
            },
        }
    }

    impl Decodable for Wspr {
        const MODE: crate::Mode = crate::Mode::Wspr;
        type State = WsprState;
        type Extras = WsprExtras;
        type Row = WsprResult;

        fn __decode(
            params: &DecodeParams,
            extras: &WsprExtras,
            state: &mut WsprState,
            slot: &SlotInput<'_>,
            on_row: Option<OnRow<'_, WsprResult>>,
        ) -> SlotResult<WsprResult> {
            let mut owned = Vec::new();
            let audio = f32_audio(slot.audio, &mut owned);
            let search = extras
                .search
                .apply(crate::wspr::search::default_search_params(), params);
            let cb = on_row.map(|f| {
                move |r: &WsprResult| {
                    f(&Row {
                        decoded: r.to_decoded(),
                        detail: RowDetail::default(),
                        native: r.clone(),
                    })
                }
            });
            let mut req = DecodeRequest::new(audio, 12_000)
                .nominal_start(nominal_start(crate::Mode::Wspr))
                .params(search)
                .scan_depth({
                    let mut d = scan_depth(params.depth);
                    if let Some(c) = extras.max_cycles_per_bit {
                        d.ladder.max_cycles_per_bit = c;
                    }
                    d
                })
                .table(&mut state.table);
            if let Some(cb) = cb.as_ref() {
                req = req.on_result(cb);
            }
            let rows = req
                .decode()
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: RowDetail::default(),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: Default::default(),
            }
        }
    }
}

#[cfg(feature = "wspr")]
pub use wspr_impl::{WsprExtras, WsprState};

// ── JT9 ──────────────────────────────────────────────────────────────────

#[cfg(feature = "jt9")]
mod jt9_impl {
    use super::*;
    use crate::jt9::{DecodeRequest, Jt9, Jt9Depth, Jt9Result};

    /// JT9's library options.
    #[derive(Clone, Debug, Default)]
    pub struct Jt9Extras {
        pub search: SearchTuning,
    }

    /// `jt9_decode.f90:83-100`: the Fano `limit` 5000 / 10000 / 30000 for
    /// `ndepth` 1 / 2 / 3.
    fn limit(depth: Depth) -> Jt9Depth {
        match depth {
            Depth::Fast => Jt9Depth::Fast,
            Depth::Normal => Jt9Depth::Normal,
            Depth::Deep => Jt9Depth::Deep,
        }
    }

    impl Decodable for Jt9 {
        const MODE: crate::Mode = crate::Mode::Jt9;
        type State = ();
        type Extras = Jt9Extras;
        type Row = Jt9Result;

        fn __decode(
            params: &DecodeParams,
            extras: &Jt9Extras,
            _state: &mut (),
            slot: &SlotInput<'_>,
            on_row: Option<OnRow<'_, Jt9Result>>,
        ) -> SlotResult<Jt9Result> {
            let mut owned = Vec::new();
            let audio = f32_audio(slot.audio, &mut owned);
            let search = extras
                .search
                .apply(crate::jt9::search::default_search_params(), params);
            let cb = on_row.map(|f| {
                move |r: &Jt9Result| {
                    f(&Row {
                        decoded: r.to_decoded(),
                        detail: RowDetail::default(),
                        native: r.clone(),
                    })
                }
            });
            let mut req = DecodeRequest::new(audio, 12_000)
                .nominal_start(nominal_start(crate::Mode::Jt9))
                .params(search)
                .depth(limit(params.depth));
            if let Some(rx) = params.rx_freq_hz {
                // The GUI's ntol (`sbFtol`) defaults to 50 Hz.
                req = req.narrow(rx, params.tol_hz.unwrap_or(50.0));
            }
            if let Some(cb) = cb.as_ref() {
                req = req.on_result(cb);
            }
            let rows = req
                .decode()
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: RowDetail::default(),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: Default::default(),
            }
        }
    }
}

#[cfg(feature = "jt9")]
pub use jt9_impl::Jt9Extras;

// ── JT65 ─────────────────────────────────────────────────────────────────

#[cfg(feature = "jt65")]
mod jt65_impl {
    use super::*;
    use crate::jt65::averaging::Averager;
    use crate::jt65::{ChaseParams, DecodeRequest, Jt65, Jt65Result};

    /// JT65's library options.
    #[derive(Clone, Debug, Default)]
    pub struct Jt65Extras {
        pub search: SearchTuning,
        /// Replace the Chase decoder's settings; by default its trial count
        /// is the depth's `nvec` and the rest are `ftrsdap.c`'s.
        pub chase: Option<ChaseParams>,
    }

    /// `jt65_decode.f90:110-119` (non-VHF): `ndepth` 1 is 2 passes and
    /// `nvec = 100`, 2 is 2 passes and 1000, 3 is 4 passes and 1000.
    fn passes_and_nvec(depth: Depth) -> (u8, usize) {
        match depth {
            Depth::Fast => (2, 100),
            Depth::Normal => (2, 1000),
            Depth::Deep => (4, 1000),
        }
    }

    impl Decodable for Jt65 {
        const MODE: crate::Mode = crate::Mode::Jt65;
        type State = Averager;
        type Extras = Jt65Extras;
        type Row = Jt65Result;

        fn __decode(
            params: &DecodeParams,
            extras: &Jt65Extras,
            state: &mut Averager,
            slot: &SlotInput<'_>,
            on_row: Option<OnRow<'_, Jt65Result>>,
        ) -> SlotResult<Jt65Result> {
            let mut owned = Vec::new();
            let audio = f32_audio(slot.audio, &mut owned);
            // The crate's own candidate cap (8) bounds each pass. Upstream's
            // 50 (`jt65_decode.f90:176`) is for its `sync65` list, whose
            // `thresh0` leaves few noise candidates; with this crate's
            // coarse search, 50 candidates under the Chase decoder produced
            // 8 false decodes on the `jt65sim` golden, 0 with 8.
            let search = extras
                .search
                .apply(crate::jt65::search::default_search_params(), params);
            let (npass, nvec) = passes_and_nvec(params.depth);
            let chase = extras.chase.clone().unwrap_or_else(|| ChaseParams {
                max_trials: nvec,
                ..ChaseParams::default()
            });
            let cb = on_row.map(|f| {
                move |r: &Jt65Result| {
                    f(&Row {
                        decoded: r.to_decoded(),
                        detail: RowDetail::default(),
                        native: r.clone(),
                    })
                }
            });
            let mut req = DecodeRequest::new(audio, 12_000)
                .nominal_start(nominal_start(crate::Mode::Jt65))
                .params(search)
                .chase(chase)
                .passes(npass);
            // `ndepth & 16`: needs the period index to know which saved periods
            // share the parity and which are the same.
            if params.averaging
                && let Some(period) = slot.period
            {
                req = req.average(state, period, params.tol_hz.unwrap_or(50.0));
            }
            if let Some(cb) = cb.as_ref() {
                req = req.on_result(cb);
            }
            let rows = req
                .decode()
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: RowDetail::default(),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: Default::default(),
            }
        }
    }
}

#[cfg(feature = "jt65")]
pub use jt65_impl::Jt65Extras;
