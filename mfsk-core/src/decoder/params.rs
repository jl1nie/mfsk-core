// SPDX-License-Identifier: GPL-3.0-or-later
//! The per-period parameter block, after WSJT-X's `params` common block
//! (`lib/jt9com.f90:9-49`), which the GUI fills before every decode and
//! `jt9 -s` reads.
//!
//! Each mode reads the fields its upstream decoder reads and ignores the
//! rest, as `jt9` does; [`Depth`] decides every search setting the way
//! `ndepth` does. What a mode makes of each field is documented on the
//! mode's `Decodable` impl, with the upstream lines it follows.

use alloc::string::String;

/// WSJT-X's decoding depth, `ndepth & 7` (the GUI's Decode → Fast /
/// Normal / Deep). Per mode it sets sync thresholds, candidate counts,
/// subtraction passes, OSD and BP effort, and whether AP runs, exactly as
/// the mode's upstream decoder does for `ndepth` 1 / 2 / 3.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
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
pub struct Station {
    pub call: String,
    pub grid: String,
}

/// Where the QSO stands: `hiscall`, `hisgrid`, `nQSOProgress`. Empty
/// strings mean no QSO in progress.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct QsoContext {
    pub his_call: String,
    pub his_grid: String,
    /// `nQSOProgress`: 0 CALLING, 1 REPLYING, 2 REPORT, 3 ROGER_REPORT,
    /// 4 ROGERS, 5 SIGNOFF (`widgets/mainwindow.h`).
    pub progress: QsoProgress,
}

/// `nQSOProgress` (`MainWindow::QSOProgress`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
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

/// The per-period parameter block (`params`, `lib/jt9com.f90:9-49`).
///
/// One per decoder, changed between periods through
/// `Decoder::params_mut`, as the GUI rewrites the block before each
/// period. Fields a mode's upstream decoder does not read are ignored by
/// that mode.
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
    /// A block over `band_hz` with the GUI's defaults: `Depth::Deep`, AP
    /// `Full`, no station or QSO, no activity, no Rx/Tx frequency.
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
        self.station = Station {
            call: call.into(),
            grid: grid.into(),
        };
        self
    }

    pub fn qso(mut self, his_call: &str, his_grid: &str, progress: QsoProgress) -> Self {
        self.qso = QsoContext {
            his_call: his_call.into(),
            his_grid: his_grid.into(),
            progress,
        };
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
