// SPDX-License-Identifier: GPL-3.0-only
//! The per-period parameter block, after WSJT-X's `params` common block
//! (`lib/jt9com.f90:9-49`), which the GUI fills before every decode and
//! `jt9 -s` reads.
//!
//! Each mode reads the fields its upstream decoder reads and ignores the
//! rest, as `jt9` does; [`Depth`] decides every search setting the way
//! `ndepth` does. Which fields a mode can read is
//! [`params_read`](super::params_read) (#698), with the decoder lines each
//! cell comes from.

use alloc::string::String;

/// WSJT-X's decoding depth, `ndepth & 7` (the GUI's Decode → Fast /
/// Normal / Deep). Per mode it sets sync thresholds, candidate counts,
/// subtraction passes, OSD and BP effort, and whether AP runs, exactly as
/// the mode's upstream decoder does for `ndepth` 1 / 2 / 3.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Depth {
    /// `ndepth = 1`.
    Fast,
    /// `ndepth = 2`.
    Normal,
    /// `ndepth = 3`, the GUI default (`"NDepth"`, default 3,
    /// `widgets/mainwindow_settings.cpp:513` in v3.2.0-rc1).
    #[default]
    Deep,
}

impl Depth {
    /// The upstream `ndepth & 7` value.
    pub fn ndepth(self) -> u32 {
        match self {
            Depth::Fast => 1,
            Depth::Normal => 2,
            Depth::Deep => 3,
        }
    }
}

/// The operating activity, WSJT-X's "Special operating activity"
/// (`Configuration::SpecialOperatingActivity`), as the decoder sees it in
/// `ncontest = iand(nexp_decode, 7)` (`lib/decoder.f90:103`).
///
/// The GUI folds WW Digi, ARRL Digi and Q65 pileup into the NA VHF code,
/// because all four exchange a 4-character grid
/// (`widgets/mainwindow.cpp:3509-3512`); they are one variant here for the
/// same reason.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Contest {
    /// No activity, `ncontest = 0`.
    #[default]
    None,
    /// NA VHF, WW Digi, ARRL Digi or Q65 pileup: `ncontest = 1`.
    GridExchange,
    /// EU VHF: `ncontest = 2`.
    EuVhf,
    /// ARRL Field Day: `ncontest = 3`.
    FieldDay,
    /// ARRL RTTY Roundup: `ncontest = 4`.
    RttyRoundup,
    /// FT8 DXpedition, Fox side: `ncontest = 6`.
    Fox,
    /// FT8 DXpedition, Hound side: `ncontest = 7`.
    Hound,
}

impl Contest {
    /// The upstream `ncontest` value.
    pub fn ncontest(self) -> u32 {
        match self {
            Contest::None => 0,
            Contest::GridExchange => 1,
            Contest::EuVhf => 2,
            Contest::FieldDay => 3,
            Contest::RttyRoundup => 4,
            Contest::Fox => 6,
            Contest::Hound => 7,
        }
    }
}

/// Which a-priori hypotheses upstream's own QSO-context tables may try
/// (`lft8apon`, `lapcqonly`).
///
/// The default differs per mode, as in the GUI: FT8's and JT65's "Enable AP"
/// boxes start unchecked (`mainwindow_settings.cpp:604-605`); the other
/// modes have no box and run AP whenever the QSO context allows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ApMode {
    /// No AP (`lft8apon = .false.`).
    Off,
    /// Only the CQ hypotheses (`lapcqonly = .true.`).
    CqOnly,
    /// Every hypothesis the QSO context allows ("Enable AP" checked).
    Full,
}

/// The operator: `mycall`, `mygrid`. Empty strings mean unknown, as in the
/// params block.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct Station {
    pub call: String,
    pub grid: String,
}

impl Station {
    /// The operator's callsign and grid. `""` for either means unknown,
    /// as in the params block.
    ///
    /// `Station` is `#[non_exhaustive]` (issue #573), so this — or
    /// [`Default`] plus field assignment — is how a caller outside the
    /// crate builds one.
    pub fn new(call: &str, grid: &str) -> Self {
        Self {
            call: call.into(),
            grid: grid.into(),
        }
    }
}

/// Where the QSO stands: `hiscall`, `hisgrid`, `nQSOProgress`. Empty
/// strings mean no QSO in progress.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct QsoContext {
    pub his_call: String,
    pub his_grid: String,
    /// `nQSOProgress`: 0 CALLING, 1 REPLYING, 2 REPORT, 3 ROGER_REPORT,
    /// 4 ROGERS, 5 SIGNOFF (`widgets/mainwindow.h`).
    pub progress: QsoProgress,
}

impl QsoContext {
    /// Who is being worked, and where the QSO stands. `""` for either
    /// callsign field means no QSO in progress.
    ///
    /// `QsoContext` is `#[non_exhaustive]` (issue #573), so this — or
    /// [`Default`] plus field assignment — is how a caller outside the
    /// crate builds one. The argument order is
    /// [`DecodeParams::qso`]'s.
    pub fn new(his_call: &str, his_grid: &str, progress: QsoProgress) -> Self {
        Self {
            his_call: his_call.into(),
            his_grid: his_grid.into(),
            progress,
        }
    }
}

/// `nQSOProgress` (`MainWindow::QSOProgress`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum QsoProgress {
    #[default]
    Calling,
    Replying,
    Report,
    RogerReport,
    Rogers,
    Signoff,
}

impl QsoProgress {
    /// The upstream `nQSOProgress` value.
    pub fn n(self) -> u32 {
        self as u32
    }
}

/// One field of [`DecodeParams`], as [`params_read`](super::params_read)
/// names it. The values are the bits of [`ParamFields`] and of the C ABI's
/// `MFSK_FIELD_*`, so they are fixed: a new field takes the next bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[repr(u32)]
pub enum ParamField {
    /// [`DecodeParams::band_hz`].
    BandHz = 1 << 0,
    /// [`DecodeParams::rx_freq_hz`].
    RxFreqHz = 1 << 1,
    /// [`DecodeParams::tol_hz`].
    TolHz = 1 << 2,
    /// [`DecodeParams::tx_freq_hz`].
    TxFreqHz = 1 << 3,
    /// [`DecodeParams::depth`].
    Depth = 1 << 4,
    /// [`DecodeParams::averaging`].
    Averaging = 1 << 5,
    /// [`DecodeParams::deep_search`].
    DeepSearch = 1 << 6,
    /// [`Station::call`] of [`DecodeParams::station`].
    StationCall = 1 << 7,
    /// [`Station::grid`] of [`DecodeParams::station`].
    StationGrid = 1 << 8,
    /// [`QsoContext::his_call`] of [`DecodeParams::qso`].
    QsoHisCall = 1 << 9,
    /// [`QsoContext::his_grid`] of [`DecodeParams::qso`].
    QsoHisGrid = 1 << 10,
    /// [`QsoContext::progress`] of [`DecodeParams::qso`].
    QsoProgress = 1 << 11,
    /// [`DecodeParams::ap`].
    Ap = 1 << 12,
    /// [`DecodeParams::contest`].
    Contest = 1 << 13,
    /// [`DecodeParams::eme_delay`].
    EmeDelay = 1 << 14,
}

impl ParamField {
    /// Every field, in bit order.
    pub const ALL: &'static [ParamField] = &[
        ParamField::BandHz,
        ParamField::RxFreqHz,
        ParamField::TolHz,
        ParamField::TxFreqHz,
        ParamField::Depth,
        ParamField::Averaging,
        ParamField::DeepSearch,
        ParamField::StationCall,
        ParamField::StationGrid,
        ParamField::QsoHisCall,
        ParamField::QsoHisGrid,
        ParamField::QsoProgress,
        ParamField::Ap,
        ParamField::Contest,
        ParamField::EmeDelay,
    ];

    /// The field's bit.
    pub const fn bit(self) -> u32 {
        self as u32
    }
}

/// A set of [`ParamField`]s: what [`params_read`](super::params_read) returns.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ParamFields(u32);

impl ParamFields {
    /// No field.
    pub const EMPTY: ParamFields = ParamFields(0);

    /// The set of `fields`.
    pub const fn of(fields: &[ParamField]) -> Self {
        let mut bits = 0;
        let mut i = 0;
        while i < fields.len() {
            bits |= fields[i].bit();
            i += 1;
        }
        ParamFields(bits)
    }

    /// The set with `fields` added.
    pub const fn with(self, fields: &[ParamField]) -> Self {
        ParamFields(self.0 | ParamFields::of(fields).0)
    }

    /// Whether `field` is in the set.
    pub const fn contains(self, field: ParamField) -> bool {
        self.0 & field.bit() != 0
    }

    /// The bits, as the C ABI's `mfsk_mode_fields_read` returns them.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// The fields in the set, in bit order.
    pub fn iter(self) -> impl Iterator<Item = ParamField> {
        ParamField::ALL
            .iter()
            .copied()
            .filter(move |&f| self.contains(f))
    }
}

/// The per-period parameter block (`params`, `lib/jt9com.f90:9-49`).
///
/// One per decoder, changed between periods through
/// `Decoder::params_mut`, as the GUI rewrites the block before each
/// period. Fields a mode's upstream decoder does not read are ignored by
/// that mode; [`params_read`](super::params_read) says which those are.
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq)]
pub struct DecodeParams {
    /// Audio band searched, Hz: `nfa`, `nfb`.
    pub band_hz: (f32, f32),
    /// The Rx frequency, Hz (`nfqso`): where AP hypotheses that name a
    /// QSO partner are tried, and the centre of single-signal modes'
    /// search.
    pub rx_freq_hz: Option<f32>,
    /// Frequency tolerance around `rx_freq_hz`, Hz (`ntol`).
    pub tol_hz: Option<f32>,
    /// The Tx frequency, Hz (`nftx`).
    pub tx_freq_hz: Option<f32>,
    /// Decoding depth (`ndepth & 7`).
    pub depth: Depth,
    /// Average over periods (`ndepth & 16`; JT65, Q65).
    pub averaging: bool,
    /// JT65 deep search (`ndepth & 32`).
    pub deep_search: bool,
    pub station: Station,
    pub qso: QsoContext,
    pub ap: ApMode,
    pub contest: Contest,
    /// "Decode at 52 s" / EME delay (`emedelay`).
    pub eme_delay: bool,
}

impl DecodeParams {
    /// The bare block over `band_hz`: `Depth::Deep`, AP `Full`, no station or
    /// QSO, no activity, no Rx/Tx frequency. **Not a mode's defaults**: FT8's and
    /// JT65's GUI start with AP off, and every mode has its own band. To change only
    /// the band, start from [`default_params`](super::default_params)`(mode).band(lo, hi)`,
    /// or `params_mut()` of a [`Decoder::with_defaults`](super::Decoder::with_defaults) (#711).
    pub fn for_band(band_hz: (f32, f32)) -> Self {
        Self {
            band_hz,
            rx_freq_hz: None,
            tol_hz: None,
            tx_freq_hz: None,
            depth: Depth::Deep,
            averaging: false,
            deep_search: false,
            station: Station::default(),
            qso: QsoContext::default(),
            ap: ApMode::Full,
            contest: Contest::None,
            eme_delay: false,
        }
    }

    /// The block a decoder hands its mode (#717): every value WSJT-X's GUI could
    /// send passes through bit-identically, and the ones it could not (its `nfa`,
    /// `nfb`, `nfqso`, `ntol` are integers) are made harmless. A band edge is
    /// clamped into the 12 kHz baseband, `[0, 6000]` Hz, a NaN edge becoming that
    /// side's end; a non-finite `rx_freq_hz` / `tx_freq_hz`, or a non-finite or
    /// negative `tol_hz`, becomes unset. Measured before this: an infinite band top
    /// panicked FT8, FST4 and Q65 and did not return from JT9 and JT65, a NaN Rx
    /// frequency or tolerance lost the FST4W decode. An inverted or empty band is
    /// left as it is (it decodes nothing and crashes nothing).
    pub(crate) fn normalised(&self) -> Self {
        const TOP_HZ: f32 = 6_000.0;
        let edge = |hz: f32, if_nan: f32| {
            if hz.is_nan() {
                if_nan
            } else {
                hz.clamp(0.0, TOP_HZ)
            }
        };
        let finite = |hz: Option<f32>| hz.filter(|f| f.is_finite());
        let mut p = self.clone();
        p.band_hz = (edge(self.band_hz.0, 0.0), edge(self.band_hz.1, TOP_HZ));
        p.rx_freq_hz = finite(self.rx_freq_hz);
        p.tx_freq_hz = finite(self.tx_freq_hz);
        p.tol_hz = finite(self.tol_hz).filter(|&t| t >= 0.0);
        p
    }

    pub fn band(mut self, lo_hz: f32, hi_hz: f32) -> Self {
        self.band_hz = (lo_hz, hi_hz);
        self
    }

    pub fn rx_freq(mut self, hz: f32) -> Self {
        self.rx_freq_hz = Some(hz);
        self
    }

    pub fn tol(mut self, hz: f32) -> Self {
        self.tol_hz = Some(hz);
        self
    }

    pub fn tx_freq(mut self, hz: f32) -> Self {
        self.tx_freq_hz = Some(hz);
        self
    }

    pub fn depth(mut self, depth: Depth) -> Self {
        self.depth = depth;
        self
    }

    pub fn averaging(mut self, on: bool) -> Self {
        self.averaging = on;
        self
    }

    pub fn deep_search(mut self, on: bool) -> Self {
        self.deep_search = on;
        self
    }

    pub fn station(mut self, call: &str, grid: &str) -> Self {
        self.station = Station::new(call, grid);
        self
    }

    pub fn qso(mut self, his_call: &str, his_grid: &str, progress: QsoProgress) -> Self {
        self.qso = QsoContext::new(his_call, his_grid, progress);
        self
    }

    pub fn ap(mut self, ap: ApMode) -> Self {
        self.ap = ap;
        self
    }

    pub fn contest(mut self, contest: Contest) -> Self {
        self.contest = contest;
        self
    }

    pub fn eme_delay(mut self, on: bool) -> Self {
        self.eme_delay = on;
        self
    }
}

#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65", feature = "q65"))]
/// Search settings the library overrides on top of the parameter block's
/// band. `None` keeps the mode's default.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct SearchTuning {
    pub time_tolerance_early_sec: Option<f32>,
    pub time_tolerance_late_sec: Option<f32>,
    pub score_threshold: Option<f32>,
    pub max_candidates: Option<usize>,
}

#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65", feature = "q65"))]
impl SearchTuning {
    /// `base` with the block's band and these overrides.
    pub(crate) fn apply(
        &self,
        mut base: crate::engine::search::SearchParams,
        params: &DecodeParams,
    ) -> crate::engine::search::SearchParams {
        base.freq_min_hz = params.band_hz.0;
        base.freq_max_hz = params.band_hz.1;
        if let Some(v) = self.time_tolerance_early_sec {
            base.time_tolerance_early_sec = v;
        }
        if let Some(v) = self.time_tolerance_late_sec {
            base.time_tolerance_late_sec = v;
        }
        if let Some(v) = self.score_threshold {
            base.score_threshold = v;
        }
        if let Some(v) = self.max_candidates {
            base.max_candidates = v;
        }
        base
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What the GUI can send passes through bit-identically (#717).
    #[test]
    fn normalised_leaves_a_valid_block_alone() {
        let p = DecodeParams::for_band((200.0, 4_000.0))
            .rx_freq(1_500.0)
            .tol(0.0)
            .tx_freq(1_200.0);
        assert_eq!(p.normalised(), p);
        let edges = DecodeParams::for_band((0.0, 6_000.0));
        assert_eq!(edges.normalised(), edges);
    }

    #[test]
    fn normalised_makes_the_rest_harmless() {
        let p = DecodeParams::for_band((f32::NAN, f32::INFINITY))
            .rx_freq(f32::NAN)
            .tol(-1.0)
            .tx_freq(f32::NEG_INFINITY)
            .normalised();
        assert_eq!(p.band_hz, (0.0, 6_000.0));
        assert_eq!((p.rx_freq_hz, p.tol_hz, p.tx_freq_hz), (None, None, None));
        let q = DecodeParams::for_band((-500.0, 9_000.0))
            .tol(f32::INFINITY)
            .normalised();
        assert_eq!((q.band_hz, q.tol_hz), ((0.0, 6_000.0), None));
        // An inverted band stays inverted: it decodes nothing and crashes nothing.
        let r = DecodeParams::for_band((3_000.0, 200.0));
        assert_eq!(r.normalised().band_hz, (3_000.0, 200.0));
    }
}
