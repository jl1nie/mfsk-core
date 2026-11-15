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
/// A row detail carrying only the identity key (`RowDetail::info`, #592):
/// these modes have no sync score or hard-error count to report.
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
fn info_detail(info: Vec<u8>) -> RowDetail {
    RowDetail {
        info,
        ..RowDetail::default()
    }
}

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

    /// Which OSD a WSPR decode runs (`wsprd -o`).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[non_exhaustive]
    pub enum WsprOsd {
        /// This crate's own OSD, the [`Depth`] mapping's: the final pass only, an order-2
        /// sweep over the float soft symbols, gated on the callsign table. **Not `wsprd`'s**:
        /// measured against `wsprd`, it matches none of `-o 0..5`.
        Crate,
        /// `-o -1`, `wsprd`'s default: no OSD.
        Off,
        /// `-o n`: `osdwspr` at depth `n` (`wsprd.c:826,1422-1469`) on the quantised soft
        /// symbols at every rung of every pass, gated on the callsign table. `n` above 5 is
        /// 5, as `osdwspr.f90` clamps it.
        Wsprd(u8),
    }

    /// WSPR's library options.
    ///
    /// **Precedence.** The decode is what [`Depth`] maps to (the arguments the WSJT-X GUI gives
    /// `wsprd`: Fast `-qB`, Normal `-C 500 -o 4`, Deep `-C 500 -o 4 -d`, see `scan_depth`)
    /// unless a field below is `Some`, which replaces that one setting and leaves the rest of
    /// the mapping alone. [`WsprExtras::wsprd`] sets every field to `wsprd`'s own default, so
    /// the depth no longer matters: the decode is what bare `wsprd` does, and each builder
    /// method then changes one option as its command-line flag does. Each field names the
    /// `wsprd.c` line it mirrors.
    #[derive(Clone, Debug, Default)]
    #[non_exhaustive]
    pub struct WsprExtras {
        pub search: SearchTuning,
        /// Fano's cycle budget per bit (`wsprd -C`), over the depth's:
        /// 10000 is `wsprd`'s own default, which the GUI's Normal and Deep
        /// lower to 500 for speed. Measured: on the WSJT-X golden 500 loses
        /// G8VDQ (-23 dB), which 10000 decodes.
        pub max_cycles_per_bit: Option<u64>,
        /// `npasses` (`wsprd.c:825,864-866,899-900`): 3 by default, `-B` is 2 (no final pass with
        /// the coherent-block ladder), `-s` is 1. Pass 1 is skipped when pass 0 found
        /// nothing, only with `npasses > 2` (`wsprd.c:1065`).
        pub passes: Option<u8>,
        /// `subtraction` (`wsprd.c:824`): remove each decode from the data before the next
        /// candidate. `-s` turns it off (and sets `passes` to 1).
        pub subtraction: Option<bool>,
        /// `!quickmode` (`wsprd.c:766,1475`): retry a candidate's demodulation at the 17 DT
        /// offsets `shift + 8k`. `-q` is `false`.
        pub jitter: Option<bool>,
        /// `more_candidates` (`wsprd.c:873-875`, `-d`): every third spectrum bin above the SNR
        /// floor is a candidate too.
        pub more_candidates: Option<bool>,
        /// `ndepth` (`wsprd -o n`, `wsprd.c:826,1422`), see [`WsprOsd`]. `None` keeps the
        /// depth's ([`WsprOsd::Crate`]). The gate (`wsprd.c:1462-1468`) admits a Type 1 call
        /// and locator, or a Type 2 call, that an earlier decode stored.
        pub osd: Option<WsprOsd>,
        /// `fmin` / `fmax` (`wsprd.c:781`, `-w` is `(-150, 150)`): the audio-frequency window
        /// around 1500 Hz that candidates are taken from. It also switches candidate
        /// selection to `wsprd`'s, which changes what `search.max_candidates` means: the first
        /// N local maxima **in frequency order** (`wsprd.c:1154`), `-d`'s extra bins after
        /// them, and only then the window; the default is the strongest N of the whole
        /// baseband. `None` keeps this crate's.
        pub freq_span_hz: Option<(f32, f32)>,
        /// `dialfreq_error` (`-e`, `wsprd.c:1172-1176`, Hz): added to the window once per pass
        /// the loop runs, as `wsprd` does (the window is `-110 + e`, `-110 + 2e`, ...). Because
        /// `wsprd` always has a window, `Some` here without `freq_span_hz` means `wsprd`'s
        /// +/-110 Hz one. The printed frequency's own correction, `dialfreq - e * 1e-6`, is
        /// the caller's, in the dial it gives `mfsk_core::wspr::wsprd_fmt`.
        pub dial_error_hz: Option<f64>,
        /// The subtraction replica and `unpk_` limits of `wsprd` (`wsprd.c:1497-1503`).
        /// `wsprd` rebuilds the signal it subtracts from the **printed text**: a Type 3 call
        /// that is not in the table prints `<...>`, so it subtracts the codeword of
        /// `nhash("...")`, the wrong one, counts `nhardmin` against it, and ends the pass's
        /// candidate loop where the rebuild refuses a text; a hashed call of 11 or 12
        /// characters does not fit `unpk_`'s 13-byte buffer and is not printed
        /// (`wsprd_utils.c:392-393`). `Some(true)` reproduces all of that,
        /// [`WsprExtras::wsprd`] sets it. `None` / `Some(false)` subtract the decoded
        /// codeword, which is right and **a deliberate divergence from `wsprd`**.
        pub replica_from_text: Option<bool>,
        /// `wsprd`'s acceptance rules, where the default keeps this crate's: the RMS gate on
        /// the quantised soft symbols (`wsprd.c:1400-1408`; the crate's reads the floats) and
        /// the duplicate rule, the same call text within 4 Hz with no time test, at most 50
        /// decodes (`wsprd.c:1506-1516`; the crate's is the same message within 5 Hz and a
        /// symbol). [`WsprExtras::wsprd`] sets it.
        pub wsprd_gates: Option<bool>,
        /// `bias` (`wsprd -z`, `wsprd.c:830,944-945`): the Fano metric table's bias,
        /// 0.45 by default.
        pub fano_bias: Option<f32>,
    }

    impl WsprExtras {
        /// Bare `wsprd`'s defaults, every option explicit: three passes with subtraction, the
        /// DT jitter, no extra candidates, no OSD (`-o -1`), candidates within +/-110 Hz of
        /// 1500 Hz and no dial error, 10000 Fano cycles per bit, bias 0.45, and the 200
        /// candidates of `wsprd.c:1154`.
        pub fn wsprd() -> Self {
            let mut e = Self::default();
            e.search.max_candidates = Some(200);
            e.max_cycles_per_bit = Some(10_000);
            e.passes = Some(3);
            e.subtraction = Some(true);
            e.jitter = Some(true);
            e.more_candidates = Some(false);
            e.osd = Some(WsprOsd::Off);
            e.freq_span_hz = Some((-110.0, 110.0));
            e.dial_error_hz = Some(0.0);
            e.replica_from_text = Some(true);
            e.wsprd_gates = Some(true);
            e.fano_bias = Some(0.45);
            e
        }

        /// `-C n`.
        pub fn with_max_cycles_per_bit(mut self, n: u64) -> Self {
            self.max_cycles_per_bit = Some(n);
            self
        }
        /// `-B` is `with_passes(2)`; `-s` is `with_passes(1).with_subtraction(false)`.
        pub fn with_passes(mut self, n: u8) -> Self {
            self.passes = Some(n);
            self
        }
        /// See [`Self::subtraction`].
        pub fn with_subtraction(mut self, on: bool) -> Self {
            self.subtraction = Some(on);
            self
        }
        /// `-q` is `with_jitter(false)`.
        pub fn with_jitter(mut self, on: bool) -> Self {
            self.jitter = Some(on);
            self
        }
        /// `-d`.
        pub fn with_more_candidates(mut self, on: bool) -> Self {
            self.more_candidates = Some(on);
            self
        }
        /// `-o n`; a negative `n` is none (`-o -1`).
        pub fn with_osd_depth(mut self, n: i8) -> Self {
            self.osd = Some(if n < 0 {
                WsprOsd::Off
            } else {
                WsprOsd::Wsprd(n as u8)
            });
            self
        }
        /// See [`Self::osd`].
        pub fn with_osd(mut self, osd: WsprOsd) -> Self {
            self.osd = Some(osd);
            self
        }
        /// See [`Self::replica_from_text`].
        pub fn with_replica_from_text(mut self, on: bool) -> Self {
            self.replica_from_text = Some(on);
            self
        }
        /// See [`Self::wsprd_gates`].
        pub fn with_wsprd_gates(mut self, on: bool) -> Self {
            self.wsprd_gates = Some(on);
            self
        }
        /// `fmin`, `fmax`; `-w` is `with_freq_span(-150.0, 150.0)`.
        pub fn with_freq_span(mut self, fmin: f32, fmax: f32) -> Self {
            self.freq_span_hz = Some((fmin, fmax));
            self
        }
        /// `-e x`.
        pub fn with_dial_error_hz(mut self, hz: f64) -> Self {
            self.dial_error_hz = Some(hz);
            self
        }
        /// `-z x`.
        pub fn with_fano_bias(mut self, bias: f32) -> Self {
            self.fano_bias = Some(bias);
            self
        }
    }

    /// WSPR's cross-period state: wsprd's callsign table.
    #[derive(Default)]
    pub struct WsprState {
        table: WsprCallsignTable,
    }

    impl core::fmt::Debug for WsprState {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("WsprState").finish_non_exhaustive()
        }
    }

    impl WsprState {
        pub fn table(&self) -> &WsprCallsignTable {
            &self.table
        }

        /// The table to load before a decode and save after (`wsprd`'s `hashtable.txt`,
        /// [`WsprCallsignTable::from_hashtable_txt`] / [`WsprCallsignTable::to_hashtable_txt`]).
        /// `wsprd -H` is a caller that does neither: the table still serves the one run, as
        /// `wsprd`'s does.
        pub fn table_mut(&mut self) -> &mut WsprCallsignTable {
            &mut self.table
        }
    }

    /// The arguments the WSJT-X GUI gives `wsprd` per decoding depth
    /// (`widgets/mainwindow.cpp:2824-2826`, `wsprd.c:819-900`): Fast
    /// `-qB`, Normal `-C 500 -o 4`, Deep `-C 500 -o 4 -d`, **except the OSD**: `wsprd -o 4`
    /// runs `osdwspr` depth 4 on the quantised symbols at every rung of every pass, where this
    /// mapping runs the crate's own order-2 sweep (`OsdSel::Legacy`) on the final pass only, which
    /// matches none of `wsprd -o 0..5` (`docs/notes/WSPRD_PARITY.md`). A deliberate divergence,
    /// CLAUDE.md §2.1; `WsprExtras::osd = Some(WsprOsd::Wsprd(4))` is `wsprd`'s. Fast has two
    /// passes and so no OSD.
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
                ..ScanDepth::DEFAULT
            },
            Depth::Normal | Depth::Deep => ScanDepth {
                passes: 3,
                ladder: Ladder {
                    jitter: true,
                    max_cycles_per_bit: 500,
                    ..Ladder::DEFAULT
                },
                more_candidates: depth == Depth::Deep,
                ..ScanDepth::DEFAULT
            },
        }
    }

    /// The depth's scan with `extras`' explicit settings laid over it.
    fn apply_extras(
        mut d: crate::wspr::decode::ScanDepth,
        extras: &WsprExtras,
    ) -> crate::wspr::decode::ScanDepth {
        use crate::wspr::decode::OsdSel;
        if let Some(c) = extras.max_cycles_per_bit {
            d.ladder.max_cycles_per_bit = c;
        }
        if let Some(n) = extras.passes {
            d.passes = n;
        }
        if let Some(on) = extras.subtraction {
            d.ladder.subtraction = on;
        }
        if let Some(on) = extras.jitter {
            d.ladder.jitter = on;
        }
        if let Some(on) = extras.more_candidates {
            d.more_candidates = on;
        }
        if let Some(o) = extras.osd {
            d.ladder.osd = match o {
                WsprOsd::Crate => OsdSel::Legacy,
                WsprOsd::Off => OsdSel::Off,
                WsprOsd::Wsprd(n) => OsdSel::Wsprd(n.min(5)),
            };
        }
        if let Some(w) = extras.freq_span_hz {
            d.freq_window = Some(w);
        }
        if let Some(e) = extras.dial_error_hz {
            d.dial_error_hz = e;
            // `wsprd` always has a window for `-e` to move.
            d.freq_window.get_or_insert((-110.0, 110.0));
        }
        if let Some(on) = extras.replica_from_text {
            d.ladder.replica_from_text = on;
        }
        if let Some(on) = extras.wsprd_gates {
            d.ladder.wsprd_gates = on;
        }
        if let Some(b) = extras.fano_bias {
            d.ladder.bias = b;
        }
        d
    }

    impl Decodable for Wspr {
        const MODE: crate::Mode = crate::Mode::Wspr;
        type State = WsprState;
        type Extras = WsprExtras;
        type Row = WsprResult;

        /// Its candidate loops are the parallel contract (`STREAMING.md` §3b).
        fn __delivery_is_exact(_params: &DecodeParams, _extras: &WsprExtras) -> bool {
            false
        }

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
                        detail: info_detail(r.info_bits.to_vec()),
                        native: r.clone(),
                    })
                }
            });
            let mut req = DecodeRequest::new(audio, 12_000)
                .nominal_start(nominal_start(crate::Mode::Wspr))
                .params(search)
                .scan_depth(apply_extras(scan_depth(params.depth), extras))
                .table(&mut state.table);
            if let Some(cb) = cb.as_ref() {
                req = req.on_result(cb);
            }
            if let Some(b) = slot.budget {
                req = req.budget(b);
            }
            let (found, exhausted, candidates) = req.decode_reported();
            let rows = found
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: info_detail(r.info_bits.to_vec()),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: crate::decoder::BudgetReport {
                    exhausted,
                    ..Default::default()
                },
                candidates,
            }
        }
    }
}

#[cfg(feature = "wspr")]
pub use wspr_impl::{WsprExtras, WsprOsd, WsprState};

// ── JT9 ──────────────────────────────────────────────────────────────────

#[cfg(feature = "jt9")]
mod jt9_impl {
    use super::*;
    use crate::jt9::{DecodeRequest, Jt9, Jt9Depth, Jt9Result};

    /// JT9's library options. Of `search`, JT9 reads the band only: its scan is `jt9`'s (`sync9`'s
    /// fixed lag range and `ccflim` gates), so the time tolerances, `score_threshold` and
    /// `max_candidates` are not read (#726).
    #[derive(Clone, Debug, Default)]
    #[non_exhaustive]
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
                        detail: info_detail(r.message.bits72().unwrap_or_default()),
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
            if let Some(b) = slot.budget {
                req = req.budget(b);
            }
            let (found, exhausted, candidates) = req.decode_reported();
            let rows = found
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: info_detail(r.message.bits72().unwrap_or_default()),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: crate::decoder::BudgetReport {
                    exhausted,
                    ..Default::default()
                },
                candidates,
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
    #[non_exhaustive]
    pub struct Jt65Extras {
        pub search: SearchTuning,
        /// Replace the Chase decoder's settings; by default its trial count
        /// is the depth's `nvec` and the rest are `ftrsdap.c`'s.
        pub chase: Option<ChaseParams>,
        /// The known calls the deep search (`DecodeParams::deep_search`,
        /// `ndepth & 32`) matches against, as `(call, grid)`: WSJT-X's
        /// `CALL3.TXT`, which beta1 replaced by a caller-supplied list. The
        /// station and DX station come from `DecodeParams`. Empty with
        /// `deep_search` on still tries the DX station's messages.
        pub known_calls: Vec<(alloc::string::String, alloc::string::String)>,
        /// `naggressive`, 0..=10: the deep search accepts a margin of
        /// `2.0 - 0.1 * this` (`extract.f90:186`). The Chase gate keeps its
        /// own setting in [`Self::chase`].
        pub aggressiveness: u8,
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
                        detail: info_detail(r.message.bits72().unwrap_or_default()),
                        native: r.clone(),
                    })
                }
            });
            let bank = params.deep_search.then(|| {
                crate::jt65::HintBank::new(&crate::jt65::HintSpec {
                    mycall: params.station.call.clone(),
                    hiscall: params.qso.his_call.clone(),
                    hisgrid: params.qso.his_grid.clone(),
                    calls: extras.known_calls.clone(),
                    aggressiveness: extras.aggressiveness,
                })
            });
            let mut req = DecodeRequest::new(audio, 12_000)
                .nominal_start(nominal_start(crate::Mode::Jt65))
                .params(search)
                .chase(chase)
                .passes(npass);
            if let Some(b) = bank.as_ref() {
                req = req.hint(b);
            }
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
            if let Some(b) = slot.budget {
                req = req.budget(b);
            }
            let (found, exhausted, candidates) = req.decode_reported();
            let rows = found
                .into_iter()
                .map(|r| Row {
                    decoded: r.to_decoded(),
                    detail: info_detail(r.message.bits72().unwrap_or_default()),
                    native: r,
                })
                .collect();
            SlotResult {
                rows,
                budget: crate::decoder::BudgetReport {
                    exhausted,
                    ..Default::default()
                },
                candidates,
            }
        }
    }
}

#[cfg(feature = "jt65")]
pub use jt65_impl::Jt65Extras;
