//! The decoder handle: one persistent decoder per mode, driven once per
//! period, after WSJT-X's own (`jt9 -s`). Rust-side it is
//! `mfsk_core::decoder::AnyDecoder`; this is its C face.
//!
//! - `mfsk_decoder_open(mode, params, extras)` makes the decoder.
//!   [`MfskParams`] is the per-period parameter block (`lib/jt9com.f90`),
//!   [`MfskExtras`] the library's options beyond it. Both are size-versioned
//!   structs of plain integers and floats.
//! - `mfsk_decoder_decode_*` decodes one period and returns the rows. A
//!   callback set with `mfsk_decoder_set_on_decode` is called for each row
//!   **as it is found**, not after the decode.
//! - `period` is the period's index on the UTC grid. A decoder keeps what
//!   upstream keeps across periods (the callsign hash table, FT8's a7 list,
//!   Q65's and JT65's averages, WSPR's call table); the ones that need
//!   consecutive periods use them only when the index is known.

use super::*;

use mfsk_core::decoder::{AnyDecoder, AnyExtras, ApMode, Contest, QsoProgress};
use mfsk_core::decoder::{Audio, DecodeParams, Depth, RowDetail, SlotInput, default_params};
use mfsk_core::msg::{ApHint, Decoded};

pub use mfsk_ffi_abi::{MfskDecoder, MfskExtras, MfskParams};

// ── Constants (defined here, not in the ABI crate: cbindgen emits a
// constant only from the crate it generates the header from) ───────────────

/// [`MfskParams::depth`]: `ndepth & 7` of 1.
pub const MFSK_DEPTH_FAST: u32 = 1;
/// `ndepth` 2.
pub const MFSK_DEPTH_NORMAL: u32 = 2;
/// `ndepth` 3, the GUI's default and the value 0 means.
pub const MFSK_DEPTH_DEEP: u32 = 3;

/// [`MfskParams::flags`]: average over periods (`ndepth & 16`; JT65, Q65).
pub const MFSK_PARAM_AVERAGING: u32 = 1 << 0;
/// JT65 deep search (`ndepth & 32`).
pub const MFSK_PARAM_DEEP_SEARCH: u32 = 1 << 1;
/// EME delay (`emedelay`).
pub const MFSK_PARAM_EME_DELAY: u32 = 1 << 2;

/// [`MfskParams::ap_mode`]: no AP (`lft8apon = .false.`).
pub const MFSK_AP_OFF: u32 = 0;
/// Only the CQ hypothesis (`lapcqonly`).
pub const MFSK_AP_CQ_ONLY: u32 = 1;
/// Every hypothesis the QSO context allows.
pub const MFSK_AP_FULL: u32 = 2;

/// [`MfskParams::contest`]: no special activity.
pub const MFSK_CONTEST_NONE: u32 = 0;
/// NA VHF, WW Digi, ARRL Digi, Q65 pileup: a 4-character grid exchange.
pub const MFSK_CONTEST_GRID_EXCHANGE: u32 = 1;
/// EU VHF.
pub const MFSK_CONTEST_EU_VHF: u32 = 2;
/// ARRL Field Day.
pub const MFSK_CONTEST_FIELD_DAY: u32 = 3;
/// ARRL RTTY Roundup.
pub const MFSK_CONTEST_RTTY_ROUNDUP: u32 = 4;
/// FT8 DXpedition, Fox.
pub const MFSK_CONTEST_FOX: u32 = 6;
/// FT8 DXpedition, Hound.
pub const MFSK_CONTEST_HOUND: u32 = 7;

/// [`MfskParams::qso_progress`]: calling CQ.
pub const MFSK_QSO_CALLING: u32 = 0;
/// Replying.
pub const MFSK_QSO_REPLYING: u32 = 1;
/// Sending a report.
pub const MFSK_QSO_REPORT: u32 = 2;
/// Sending roger and a report.
pub const MFSK_QSO_ROGER_REPORT: u32 = 3;
/// Sending rogers.
pub const MFSK_QSO_ROGERS: u32 = 4;
/// Signing off.
pub const MFSK_QSO_SIGNOFF: u32 = 5;

/// [`MfskExtras::strategy`]: the depth's own.
pub const MFSK_STRATEGY_DEFAULT: u32 = 0;
/// One pass, no subtraction.
pub const MFSK_STRATEGY_SINGLE_PASS: u32 = 1;
/// `sic_rounds` rounds of subtraction.
pub const MFSK_STRATEGY_SIC_ROUNDS: u32 = 2;
/// FT8's checkpointed passes.
pub const MFSK_STRATEGY_SIC_EARLY: u32 = 3;

/// `period` for a decode whose slot index is not known; a decoder then
/// leaves its period-to-period state (a7, averaging) alone.
pub const MFSK_PERIOD_NONE: i64 = i64::MIN;

// ── Callback and budget types ─────────────────────────────────────────────

/// Polled during a decode to ask whether to keep going. Returning `false`
/// stops the search and returns what has been found so far.
///
/// **The library reads no clock.** `std::time::Instant::now` is
/// unimplemented on `wasm32-unknown-unknown` and absent on `no_std`, so the
/// deadline is the caller's: host `Instant`, browser `performance.now()`,
/// embedded `esp_timer_get_time`.
pub type MfskBudgetCheck = Option<unsafe extern "C" fn(user_data: *mut c_void) -> bool>;

/// What a budgeted decode left undone. Size-versioned. All-zero means no
/// budget was set, or it was never reached.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct MfskBudgetReport {
    /// `sizeof(MfskBudgetReport)` as the caller understands it.
    pub size: u32,
    /// The predicate returned `false` at least once: work was left undone.
    pub exhausted: bool,
    /// Units of work declined (a candidate, or a SIC round).
    pub candidates_skipped: u32,
    /// Units of work actually run, counted the same way.
    pub stages_run: u32,
    /// Costas sync quality (0..=`n_sync`) of the best skipped candidate, or
    /// -1 when not applicable. FT8 only.
    pub cut_at_sync: i32,
    /// Sync score of the best skipped candidate, or NaN when there was none.
    pub cut_at_score: f32,
    /// Rows subtracted from the residual before a later search saw it: FT8
    /// `MFSK_STRATEGY_SIC_EARLY`'s checkpoint-B and -C loops, which poll the
    /// budget before each row (#589). Fewer than the rows returned, with
    /// `exhausted` set, means the cut came while cleaning up rather than
    /// while searching. 0 on every other strategy and mode. Appended; a
    /// caller built against the shorter struct does not see it.
    pub rows_subtracted: u32,
}

/// Called once per decode, as it is found. The row pointer is valid only
/// for the duration of the call.
pub type MfskDecodeCallback =
    Option<unsafe extern "C" fn(row: *const MfskDecode, user_data: *mut c_void)>;

fn budget_report(r: &mfsk_core::decoder::BudgetReport) -> MfskBudgetReport {
    MfskBudgetReport {
        size: core::mem::size_of::<MfskBudgetReport>() as u32,
        exhausted: r.exhausted,
        candidates_skipped: r.candidates_skipped,
        stages_run: r.stages_run,
        cut_at_sync: r.cut_at_sync.map(|v| v as i32).unwrap_or(-1),
        cut_at_score: r.cut_at_score.unwrap_or(f32::NAN),
        rows_subtracted: r.rows_subtracted,
    }
}

// ── The handle ────────────────────────────────────────────────────────────

pub(crate) struct FfiDecoder {
    mode: MfskMode,
    any: AnyDecoder,
    /// The last decode's rows, for `copy_info`.
    last: Vec<(Decoded, RowDetail)>,
    /// `(period, samples)` of a prefix call refused for a short output
    /// buffer. Its stage has run and will not run again, so the retry the
    /// ABI asks for (same call, a bigger buffer) is answered from `last`.
    short_prefix: Option<(i64, usize)>,
    on_decode: MfskDecodeCallback,
    on_decode_user: SyncUserData,
    budget: MfskBudgetCheck,
    budget_user: SyncUserData,
    last_budget: MfskBudgetReport,
    /// The contest callers (`q65_hist2`), kept here because
    /// `mfsk_decoder_set_extras` replaces the whole extras block.
    q65_callers: Option<mfsk_core::q65::Q65Callers>,
    /// Per-handle error slot: the process-global `thread_local!` is wrong for
    /// a coroutine or `async` caller, which hops threads between checking a
    /// status and reading the message.
    error: Option<CString>,
}

impl FfiDecoder {
    /// Decode one slot of 12 kHz audio at `period`, rows left on the handle.
    pub(crate) fn decode_slot(&mut self, audio: &[f32], period: i64) {
        let _ = run(self, Audio::F32(audio), period, false);
    }

    pub(crate) fn rows(&self) -> impl Iterator<Item = (&Decoded, &RowDetail)> {
        self.last.iter().map(|(a, b)| (a, b))
    }

    pub(crate) fn any_unpack77(&self, msg77: &[u8]) -> Option<String> {
        self.any.unpack77(msg77)
    }

    fn fail(&mut self, status: MfskStatus, msg: impl Into<String>) -> MfskStatus {
        let m = msg.into();
        set_error(m.clone());
        self.error = CString::new(m).ok();
        status
    }
}

fn handle(dec: *mut MfskDecoder) -> Option<&'static mut FfiDecoder> {
    unsafe { (dec as *mut FfiDecoder).as_mut() }
}

fn handle_ref(dec: *const MfskDecoder) -> Option<&'static FfiDecoder> {
    unsafe { (dec as *const FfiDecoder).as_ref() }
}

/// The library's [`mfsk_core::Mode`] for a C mode, or `None` for one a
/// decoder does not carry (MSK144, JTTY, uvpacket) or a build without it.
fn core_mode_of(m: MfskMode) -> Option<mfsk_core::Mode> {
    iq_mode_of(m)
}

// ── Parameters ────────────────────────────────────────────────────────────

fn params_from_core(p: &DecodeParams) -> MfskParams {
    let mut out = MfskParams {
        size: core::mem::size_of::<MfskParams>() as u32,
        depth: p.depth.ndepth(),
        flags: (u32::from(p.averaging) * MFSK_PARAM_AVERAGING)
            | (u32::from(p.deep_search) * MFSK_PARAM_DEEP_SEARCH)
            | (u32::from(p.eme_delay) * MFSK_PARAM_EME_DELAY),
        ap_mode: match p.ap {
            ApMode::Off => MFSK_AP_OFF,
            ApMode::CqOnly => MFSK_AP_CQ_ONLY,
            ApMode::Full => MFSK_AP_FULL,
        },
        contest: p.contest.ncontest(),
        qso_progress: p.qso.progress.n(),
        band_lo_hz: p.band_hz.0,
        band_hi_hz: p.band_hz.1,
        rx_freq_hz: p.rx_freq_hz.unwrap_or(f32::NAN),
        tol_hz: p.tol_hz.unwrap_or(f32::NAN),
        tx_freq_hz: p.tx_freq_hz.unwrap_or(f32::NAN),
        mycall: [0; 16],
        mygrid: [0; 8],
        hiscall: [0; 16],
        hisgrid: [0; 8],
    };
    write_field(&mut out.mycall, &p.station.call);
    write_field(&mut out.mygrid, &p.station.grid);
    write_field(&mut out.hiscall, &p.qso.his_call);
    write_field(&mut out.hisgrid, &p.qso.his_grid);
    out
}

/// Read a caller's size-versioned struct of plain data over `base`: only the
/// prefix they declared is taken (0 or too large means the whole struct), so
/// a caller built against an older header leaves the rest at `base`'s.
///
/// # Safety
/// `src` must point to at least `declared` readable bytes, `declared` being
/// the `u32` at its start, and `T` must be `#[repr(C)]` with `size: u32`
/// first and no field with an invalid bit pattern (plain integers, floats,
/// arrays of them).
unsafe fn read_prefixed<T: Copy>(src: *const T, mut base: T) -> T {
    let full = core::mem::size_of::<T>();
    let declared = unsafe { core::ptr::read_unaligned(src as *const u32) } as usize;
    let n = if declared == 0 || declared > full {
        full
    } else {
        declared
    };
    unsafe { core::ptr::copy_nonoverlapping(src as *const u8, &mut base as *mut T as *mut u8, n) };
    base
}

fn params_to_core(mode: MfskMode, p: &MfskParams) -> Result<DecodeParams, String> {
    let name = mode_index(mode).map(mode_name_str).unwrap_or("?");
    let depth = match p.depth {
        0 | MFSK_DEPTH_DEEP => Depth::Deep,
        MFSK_DEPTH_NORMAL => Depth::Normal,
        MFSK_DEPTH_FAST => Depth::Fast,
        d => return Err(format!("{name}: depth {d} is not MFSK_DEPTH_*")),
    };
    let ap = match p.ap_mode {
        MFSK_AP_OFF => ApMode::Off,
        MFSK_AP_CQ_ONLY => ApMode::CqOnly,
        MFSK_AP_FULL => ApMode::Full,
        a => return Err(format!("{name}: ap_mode {a} is not MFSK_AP_*")),
    };
    let contest = match p.contest {
        MFSK_CONTEST_NONE => Contest::None,
        MFSK_CONTEST_GRID_EXCHANGE => Contest::GridExchange,
        MFSK_CONTEST_EU_VHF => Contest::EuVhf,
        MFSK_CONTEST_FIELD_DAY => Contest::FieldDay,
        MFSK_CONTEST_RTTY_ROUNDUP => Contest::RttyRoundup,
        MFSK_CONTEST_FOX => Contest::Fox,
        MFSK_CONTEST_HOUND => Contest::Hound,
        c => return Err(format!("{name}: contest {c} is not MFSK_CONTEST_*")),
    };
    let progress = match p.qso_progress {
        MFSK_QSO_CALLING => QsoProgress::Calling,
        MFSK_QSO_REPLYING => QsoProgress::Replying,
        MFSK_QSO_REPORT => QsoProgress::Report,
        MFSK_QSO_ROGER_REPORT => QsoProgress::RogerReport,
        MFSK_QSO_ROGERS => QsoProgress::Rogers,
        MFSK_QSO_SIGNOFF => QsoProgress::Signoff,
        n => return Err(format!("{name}: qso_progress {n} is not MFSK_QSO_*")),
    };
    // Spelled out rather than `!(hi > lo)`: a NaN edge is a band that can
    // never match anything, from a caller that did not call init.
    if !p.band_lo_hz.is_finite() || !p.band_hi_hz.is_finite() || p.band_hi_hz <= p.band_lo_hz {
        return Err(format!(
            "{name}: band [{}, {}] is not a usable range — did you call mfsk_params_init?",
            p.band_lo_hz, p.band_hi_hz
        ));
    }
    let mut d = DecodeParams::for_band((p.band_lo_hz, p.band_hi_hz))
        .depth(depth)
        .averaging(p.flags & MFSK_PARAM_AVERAGING != 0)
        .deep_search(p.flags & MFSK_PARAM_DEEP_SEARCH != 0)
        .eme_delay(p.flags & MFSK_PARAM_EME_DELAY != 0)
        .ap(ap)
        .contest(contest)
        .station(cstr_field(&p.mycall), cstr_field(&p.mygrid))
        .qso(cstr_field(&p.hiscall), cstr_field(&p.hisgrid), progress);
    if p.rx_freq_hz.is_finite() {
        d = d.rx_freq(p.rx_freq_hz);
    }
    if p.tol_hz.is_finite() {
        d = d.tol(p.tol_hz);
    }
    if p.tx_freq_hz.is_finite() {
        d = d.tx_freq(p.tx_freq_hz);
    }
    Ok(d)
}

/// Fill `out` with `mode`'s defaults: the GUI's (Deep; AP off for FT8 and
/// JT65), the mode's band, and nothing else set.
///
/// Always call this before touching a [`MfskParams`]; zeroing it by hand is
/// not equivalent (a zero band decodes nothing, and 0 Hz is a frequency, not
/// "unset").
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_params_init(mode: u32, out: *mut MfskParams) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_params_init: out is NULL");
        return MfskStatus::InvalidArg;
    }
    let Some(m) = mode_of(mode) else {
        set_error("mfsk_params_init: not a mode this library knows");
        return MfskStatus::InvalidArg;
    };
    let Some(cm) = core_mode_of(m) else {
        set_error("mfsk_params_init: no decoder for this mode in this build");
        return MfskStatus::UnknownProtocol;
    };
    let p = params_from_core(&default_params(cm));
    unsafe { write_size_versioned(out, &p) };
    MfskStatus::Ok
}

fn extras_unset() -> MfskExtras {
    MfskExtras {
        size: core::mem::size_of::<MfskExtras>() as u32,
        sync_min: f32::NAN,
        max_cand: 0,
        osd: -1,
        strictness: -1,
        strategy: MFSK_STRATEGY_DEFAULT,
        sic_rounds: 0,
        eq_mode: 0,
        message_filter: 0,
        a7: 0,
        sniper_hz: 0.0,
        has_ap_hint: 0,
        ap_call1: [0; 16],
        ap_call2: [0; 16],
        ap_grid: [0; 16],
        ap_report: [0; 16],
        nb_percent: 0,
        nb_sweep_step: 0,
        nb_ftol_hz: 0.0,
        t_early_s: f32::NAN,
        t_late_s: f32::NAN,
        score_threshold: f32::NAN,
        max_cycles_per_bit: 0,
        chase_trials: 0,
        pileup: 0,
        max_drift: 0,
        fading_b90_ts: f32::NAN,
        fading_model: 0,
    }
}

/// Fill `out` with "unset" for every option. A zeroed struct is **not** the
/// same: 0 `strictness` is Strict, 0 `osd` is off, 0 Hz is a frequency.
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_extras_init(out: *mut MfskExtras) -> MfskStatus {
    if out.is_null() {
        set_error("mfsk_extras_init: out is NULL");
        return MfskStatus::InvalidArg;
    }
    unsafe { write_size_versioned(out, &extras_unset()) };
    MfskStatus::Ok
}

// ── Extras ────────────────────────────────────────────────────────────────

/// Why an option cannot be applied: refused, naming it, never dropped.
fn unsupported(mode: MfskMode, what: &str) -> (MfskStatus, String) {
    let name = mode_index(mode).map(mode_name_str).unwrap_or("?");
    (MfskStatus::Unsupported, format!("{name} has no {what}"))
}

fn invalid(msg: String) -> (MfskStatus, String) {
    (MfskStatus::InvalidArg, msg)
}

fn strictness_of(v: i32) -> Result<Option<mfsk_core::engine::pipeline::DecodeStrictness>, String> {
    use mfsk_core::engine::pipeline::DecodeStrictness as S;
    Ok(match v {
        -1 => None,
        0 => Some(S::Strict),
        1 => Some(S::Normal),
        2 => Some(S::Deep),
        n => return Err(format!("strictness {n} is not -1..=2")),
    })
}

fn hint_of(e: &MfskExtras) -> Option<ApHint> {
    if e.has_ap_hint == 0 {
        return None;
    }
    let mut h = ApHint::new();
    for (field, set) in [
        (&e.ap_call1, 1),
        (&e.ap_call2, 2),
        (&e.ap_grid, 3),
        (&e.ap_report, 4),
    ] {
        let v = cstr_field(field);
        if v.is_empty() {
            continue;
        }
        h = match set {
            1 => h.with_call1(v),
            2 => h.with_call2(v),
            3 => h.with_grid(v),
            _ => h.with_report(v),
        };
    }
    h.has_info().then_some(h)
}

fn search_tuning(e: &MfskExtras) -> mfsk_core::decoder::SearchTuning {
    let f = |v: f32| v.is_finite().then_some(v);
    // `SearchTuning` is `#[non_exhaustive]` (issue #573), so it is built
    // from `Default` and assigned field by field rather than by literal.
    let mut t = mfsk_core::decoder::SearchTuning::default();
    t.time_tolerance_early_sec = f(e.t_early_s);
    t.time_tolerance_late_sec = f(e.t_late_s);
    t.score_threshold = f(e.score_threshold);
    t.max_candidates = (e.max_cand > 0).then_some(e.max_cand as usize);
    t
}

/// Apply `e` to `any`, replacing its extras block. An option the mode does
/// not have is `Unsupported`; a value out of range, `InvalidArg`.
fn apply_extras(
    mode: MfskMode,
    any: &mut AnyDecoder,
    e: &MfskExtras,
    q65_callers: &Option<mfsk_core::q65::Q65Callers>,
) -> Result<(), (MfskStatus, String)> {
    use mfsk_core::decoder::{Fst4Strategy, Ft4Strategy, Ft8Strategy, MessageFilter, Sniper};
    use mfsk_core::engine::equalize::EqMode;

    let eq = match e.eq_mode {
        0 => EqMode::Off,
        1 => EqMode::Local,
        n => return Err(invalid(format!("eq_mode {n} is not 0 or 1"))),
    };
    let filter = match e.message_filter {
        0 => MessageFilter::Default,
        1 => MessageFilter::Codec,
        n => return Err(invalid(format!("message_filter {n} is not 0 or 1"))),
    };
    let strictness = strictness_of(e.strictness).map_err(invalid)?;
    let osd = match e.osd {
        -1 => None,
        0 => Some(false),
        1 => Some(true),
        n => return Err(invalid(format!("osd {n} is not -1, 0 or 1"))),
    };
    if !matches!(e.strategy, 0..=3) {
        return Err(invalid(format!(
            "strategy {} is not MFSK_STRATEGY_*",
            e.strategy
        )));
    }
    let hint = hint_of(e);
    let sync_min = e.sync_min.is_finite().then_some(e.sync_min);
    let max_cand = (e.max_cand > 0).then_some(e.max_cand as usize);
    let search_set = [e.t_early_s, e.t_late_s, e.score_threshold]
        .iter()
        .any(|v| v.is_finite());
    let frame_only = osd.is_some()
        || strictness.is_some()
        || e.strategy != MFSK_STRATEGY_DEFAULT
        || e.eq_mode != 0
        || e.message_filter != 0
        || sync_min.is_some();
    let nb_set = e.nb_percent != 0 || e.nb_sweep_step != 0;
    let q65_set = e.pileup != 0 || e.max_drift != 0 || e.fading_b90_ts.is_finite();

    // Options no mode but a few have, checked once.
    let refuse = |cond: bool, what: &str| -> Result<(), (MfskStatus, String)> {
        if cond {
            Err(unsupported(mode, what))
        } else {
            Ok(())
        }
    };
    match any.extras_mut() {
        #[cfg(feature = "protocols")]
        AnyExtras::Ft8(x) => {
            refuse(
                search_set,
                "search window (t_early_s, t_late_s, score_threshold)",
            )?;
            refuse(nb_set, "noise blanker")?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            *x = Default::default();
            x.tuning.sync_min = sync_min;
            x.tuning.max_cand = max_cand;
            x.tuning.osd = osd;
            x.tuning.strictness = strictness;
            x.tuning.strategy = match e.strategy {
                MFSK_STRATEGY_SINGLE_PASS => Some(Ft8Strategy::SinglePass),
                MFSK_STRATEGY_SIC_ROUNDS => Some(Ft8Strategy::SicRounds(e.sic_rounds as usize)),
                MFSK_STRATEGY_SIC_EARLY => Some(Ft8Strategy::SicEarly),
                _ => None,
            };
            x.eq = eq;
            x.filter = filter;
            x.a7 = e.a7 != 0;
            x.ap_hint = hint;
            x.sniper = (e.sniper_hz > 0.0).then(|| Sniper::new(e.sniper_hz));
        }
        #[cfg(feature = "protocols")]
        AnyExtras::Ft4(x) => {
            refuse(
                search_set,
                "search window (t_early_s, t_late_s, score_threshold)",
            )?;
            refuse(nb_set, "noise blanker")?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            refuse(e.a7 != 0, "a7 list decoder")?;
            refuse(e.sniper_hz > 0.0, "roofing-filter (sniper) search")?;
            refuse(e.strategy == MFSK_STRATEGY_SIC_EARLY, "checkpointed passes")?;
            *x = Default::default();
            x.tuning.sync_min = sync_min;
            x.tuning.max_cand = max_cand;
            x.tuning.osd = osd;
            x.tuning.strictness = strictness;
            x.tuning.strategy = match e.strategy {
                MFSK_STRATEGY_SINGLE_PASS => Some(Ft4Strategy::SinglePass),
                MFSK_STRATEGY_SIC_ROUNDS => Some(Ft4Strategy::SicRounds(e.sic_rounds as usize)),
                _ => None,
            };
            x.eq = eq;
            x.filter = filter;
            x.ap_hint = hint;
        }
        #[cfg(feature = "protocols")]
        AnyExtras::Fst4(x) => {
            refuse(
                search_set,
                "search window (t_early_s, t_late_s, score_threshold)",
            )?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            refuse(e.a7 != 0, "a7 list decoder")?;
            refuse(e.sniper_hz > 0.0, "roofing-filter (sniper) search")?;
            refuse(
                matches!(
                    e.strategy,
                    MFSK_STRATEGY_SIC_ROUNDS | MFSK_STRATEGY_SIC_EARLY
                ),
                "signal subtraction (upstream's fst4_decode has none)",
            )?;
            let nb = if e.nb_sweep_step != 0 {
                if !matches!(e.nb_sweep_step, 1 | 2 | 5) {
                    return Err(invalid("nb_sweep_step must be 1, 2 or 5".into()));
                }
                if !(e.nb_ftol_hz.is_finite() && e.nb_ftol_hz > 0.0) {
                    return Err(invalid(
                        "nb_ftol_hz must be positive with nb_sweep_step".into(),
                    ));
                }
                Some(mfsk_core::decoder::NoiseBlanker::Sweep {
                    step: e.nb_sweep_step as u8,
                    ftol_hz: e.nb_ftol_hz,
                })
            } else if e.nb_percent != 0 {
                if e.nb_percent > 25 {
                    return Err(invalid("nb_percent is 0..=25".into()));
                }
                Some(mfsk_core::decoder::NoiseBlanker::Percent(
                    e.nb_percent as u8,
                ))
            } else {
                None
            };
            *x = Default::default();
            x.tuning.sync_min = sync_min;
            x.tuning.max_cand = max_cand;
            x.tuning.osd = osd;
            x.tuning.strictness = strictness;
            x.tuning.strategy =
                (e.strategy == MFSK_STRATEGY_SINGLE_PASS).then_some(Fst4Strategy::SinglePass);
            x.eq = eq;
            x.filter = filter;
            x.ap_hint = hint;
            x.noise_blanker = nb;
        }
        #[cfg(feature = "legacy")]
        AnyExtras::Wspr(x) => {
            refuse(frame_only, "FT8/FT4/FST4 search option")?;
            refuse(nb_set, "noise blanker")?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            refuse(
                e.a7 != 0 || e.sniper_hz > 0.0 || hint.is_some(),
                "a-priori hint",
            )?;
            *x = Default::default();
            x.search = search_tuning(e);
            x.max_cycles_per_bit =
                (e.max_cycles_per_bit > 0).then_some(u64::from(e.max_cycles_per_bit));
        }
        #[cfg(feature = "legacy")]
        AnyExtras::Jt9(x) => {
            refuse(frame_only, "FT8/FT4/FST4 search option")?;
            refuse(nb_set, "noise blanker")?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            refuse(
                e.a7 != 0 || e.sniper_hz > 0.0 || hint.is_some(),
                "a-priori hint",
            )?;
            *x = Default::default();
            x.search = search_tuning(e);
        }
        #[cfg(feature = "legacy")]
        AnyExtras::Jt65(x) => {
            refuse(frame_only, "FT8/FT4/FST4 search option")?;
            refuse(nb_set, "noise blanker")?;
            refuse(q65_set, "Q65 option")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(
                e.a7 != 0 || e.sniper_hz > 0.0 || hint.is_some(),
                "a-priori hint",
            )?;
            *x = Default::default();
            x.search = search_tuning(e);
            x.chase = (e.chase_trials > 0).then(|| mfsk_core::jt65::ChaseParams {
                max_trials: e.chase_trials as usize,
                ..mfsk_core::jt65::ChaseParams::default()
            });
        }
        #[cfg(feature = "legacy")]
        AnyExtras::Q65(x) => {
            refuse(frame_only, "FT8/FT4/FST4 search option")?;
            refuse(nb_set, "noise blanker")?;
            refuse(e.max_cycles_per_bit != 0, "Fano cycle budget")?;
            refuse(e.chase_trials != 0, "Chase decoder")?;
            refuse(e.a7 != 0 || e.sniper_hz > 0.0, "a7 / sniper search")?;
            if e.max_drift > 50 {
                return Err(invalid("max_drift is 0..=50 bins".into()));
            }
            if e.pileup != 0 && hint.is_none() {
                return Err(unsupported(mode, "Pileup without an AP hint to match"));
            }
            *x = Default::default();
            x.search = search_tuning(e);
            x.ap_hint = hint;
            x.pileup = e.pileup != 0;
            x.max_drift = e.max_drift;
            x.callers = q65_callers.clone();
            if e.fading_b90_ts.is_finite() {
                let model = match e.fading_model {
                    0 => mfsk_core::fec::qra::FadingModel::Gaussian,
                    1 => mfsk_core::fec::qra::FadingModel::Lorentzian,
                    n => return Err(invalid(format!("fading_model {n} is not 0 or 1"))),
                };
                x.fading = Some((model, e.fading_b90_ts));
            }
        }
        #[allow(unreachable_patterns)]
        _ => return Err(unsupported(mode, "options")),
    }
    Ok(())
}

// ── Open, close, configure ────────────────────────────────────────────────

/// Build a decoder from C arguments: the shared body of
/// [`mfsk_decoder_open`] and an IQ channel's.
///
/// # Safety
/// `params` and `extras` must each be null or a valid struct of their type.
pub(crate) unsafe fn open_decoder(
    mode: u32,
    params: *const MfskParams,
    extras: *const MfskExtras,
) -> Result<FfiDecoder, (MfskStatus, String)> {
    let Some(m) = mode_of(mode) else {
        return Err((
            MfskStatus::InvalidArg,
            "not a mode this library knows".into(),
        ));
    };
    let Some(cm) = core_mode_of(m) else {
        return Err((
            MfskStatus::UnknownProtocol,
            "this mode has no decoder in this build".into(),
        ));
    };
    let base = params_from_core(&default_params(cm));
    let p = if params.is_null() {
        base
    } else {
        unsafe { read_prefixed(params, base) }
    };
    let core_params = params_to_core(m, &p).map_err(invalid)?;
    let mut any = AnyDecoder::new(cm, core_params);
    if !extras.is_null() {
        let e = unsafe { read_prefixed(extras, extras_unset()) };
        apply_extras(m, &mut any, &e, &None)?;
    }
    Ok(FfiDecoder {
        mode: m,
        any,
        last: Vec::new(),
        short_prefix: None,
        on_decode: None,
        on_decode_user: SyncUserData(ptr::null_mut()),
        budget: None,
        budget_user: SyncUserData(ptr::null_mut()),
        last_budget: MfskBudgetReport::default(),
        q65_callers: None,
        error: None,
    })
}

/// Open a decoder for `mode`.
///
/// `params` and `extras` may be NULL for the mode's defaults. **An option the
/// mode does not support is an error here**, not a field dropped at decode
/// time: `MFSK_STATUS_UNSUPPORTED`, with the reason in `mfsk_last_error()`.
/// Returns NULL and writes the status to `out_status` on failure.
///
/// # Safety
/// `params` and `extras` must each be null or a valid struct of their type;
/// `out_status` may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_open(
    mode: u32,
    params: *const MfskParams,
    extras: *const MfskExtras,
    out_status: *mut MfskStatus,
) -> *mut MfskDecoder {
    let report = |st: MfskStatus| {
        if !out_status.is_null() {
            unsafe { *out_status = st };
        }
    };
    match unsafe { open_decoder(mode, params, extras) } {
        Ok(d) => {
            report(MfskStatus::Ok);
            Box::into_raw(Box::new(d)) as *mut MfskDecoder
        }
        Err((st, msg)) => {
            set_error(format!("mfsk_decoder_open: {msg}"));
            report(st);
            ptr::null_mut()
        }
    }
}

/// Release a decoder. Null is a no-op.
///
/// # Safety
/// `dec` must be a handle from [`mfsk_decoder_open`], released once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_close(dec: *mut MfskDecoder) {
    if !dec.is_null() {
        drop(unsafe { Box::from_raw(dec as *mut FfiDecoder) });
    }
}

/// Whether a decode with the current mode, depth and extras runs the exact
/// delivery contract (`STREAMING.md` §3a): the callback of
/// `mfsk_decoder_set_on_decode` sees exactly the rows the call returns, once
/// each, in the same order. `false` is §3b — completion order, a transient
/// duplicate possible (FT8's `MFSK_STRATEGY_SINGLE_PASS` and sniper, FT4 at
/// `MFSK_DEPTH_FAST`, FST4, WSPR) — so a caller keeps its guard, pairing by
/// `MfskDecode::delivery`. Ask again after `mfsk_decoder_set_params` or
/// `mfsk_decoder_set_extras`. `false` for a null handle.
///
/// # Safety
/// `dec` must be a live handle or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_delivery_is_exact(dec: *const MfskDecoder) -> bool {
    match handle_ref(dec) {
        Some(d) => d.any.delivery_is_exact(),
        None => {
            set_error("mfsk_decoder_delivery_is_exact: null decoder handle");
            false
        }
    }
}

/// The last error recorded **on this handle**, or NULL. Prefer it to
/// `mfsk_last_error()` whenever you have a handle: the global one is a
/// `thread_local!`, which a Kotlin coroutine or a Swift `async` caller reads
/// as NULL after hopping threads. Valid until the next call on the handle.
///
/// # Safety
/// `dec` must be a live handle or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_last_error(dec: *const MfskDecoder) -> *const c_char {
    match handle_ref(dec) {
        Some(d) => d.error.as_ref().map(|s| s.as_ptr()).unwrap_or(ptr::null()),
        None => ptr::null(),
    }
}

/// Change the parameter block between periods, as the GUI rewrites it
/// before each one. What the decoder carries between periods is kept.
///
/// # Safety
/// `params` must be a valid [`MfskParams`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_set_params(
    dec: *mut MfskDecoder,
    params: *const MfskParams,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_set_params: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if params.is_null() {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_set_params: params is NULL",
        );
    }
    let current = params_from_core(d.any.params());
    let p = unsafe { read_prefixed(params, current) };
    match params_to_core(d.mode, &p) {
        Ok(c) => {
            *d.any.params_mut() = c;
            MfskStatus::Ok
        }
        Err(e) => d.fail(
            MfskStatus::InvalidArg,
            format!("mfsk_decoder_set_params: {e}"),
        ),
    }
}

/// Replace the library's options. What the block leaves unset goes back to
/// the depth's value; an option the mode lacks is `MFSK_STATUS_UNSUPPORTED`
/// and nothing changes. What the decoder carries between periods is kept.
///
/// # Safety
/// `extras` must be a valid [`MfskExtras`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_set_extras(
    dec: *mut MfskDecoder,
    extras: *const MfskExtras,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_set_extras: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if extras.is_null() {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_set_extras: extras is NULL",
        );
    }
    let e = unsafe { read_prefixed(extras, extras_unset()) };
    let callers = d.q65_callers.clone();
    match apply_extras(d.mode, &mut d.any, &e, &callers) {
        Ok(()) => MfskStatus::Ok,
        Err((st, msg)) => d.fail(st, format!("mfsk_decoder_set_extras: {msg}")),
    }
}

/// Q65 only: the contest callers heard (`q65_hist2`), so that with
/// `MFSK_CONTEST_GRID_EXCHANGE` they join the full-AP list (`ncontest = 1`).
/// The list is copied. Pass NULL to remove it. Survives
/// `mfsk_decoder_set_extras`.
///
/// # Safety
/// `callers` must be a live [`MfskQ65Callers`] or null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_set_q65_callers(
    dec: *mut MfskDecoder,
    callers: *const MfskQ65Callers,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_set_q65_callers: null decoder handle");
        return MfskStatus::NullPointer;
    };
    let new = if callers.is_null() {
        None
    } else {
        match q65_callers(callers as *mut MfskQ65Callers) {
            Some(c) => Some(c.clone()),
            None => {
                return d.fail(
                    MfskStatus::NullPointer,
                    "mfsk_decoder_set_q65_callers: bad handle",
                );
            }
        }
    };
    let AnyExtras::Q65(x) = d.any.extras_mut() else {
        return d.fail(
            MfskStatus::Unsupported,
            "mfsk_decoder_set_q65_callers: not a Q65 decoder",
        );
    };
    x.callers = new.clone();
    d.q65_callers = new;
    MfskStatus::Ok
}

/// Forget everything the decoder carries between periods (WSJT-X's "Clear
/// Avg" and `ndepth & 128`): the hash table, a7, averages.
///
/// # Safety
/// `dec` must be a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_clear(dec: *mut MfskDecoder) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_clear: null decoder handle");
        return MfskStatus::NullPointer;
    };
    d.any.clear();
    d.last.clear();
    MfskStatus::Ok
}

/// Teach the decoder a callsign, so a later period's `<...>` reference to it
/// resolves. Decoded messages populate the table by themselves; this is for
/// calls known from outside (a band map, a log). `MFSK_STATUS_UNSUPPORTED`
/// for a mode whose messages carry no hashed calls.
///
/// # Safety
/// `call` must be a valid NUL-terminated C string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_add_callsign(
    dec: *mut MfskDecoder,
    call: *const c_char,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_add_callsign: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if call.is_null() {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_add_callsign: call is NULL",
        );
    }
    let Ok(s) = (unsafe { CStr::from_ptr(call) }).to_str() else {
        return d.fail(
            MfskStatus::InvalidArg,
            "mfsk_decoder_add_callsign: call is not valid UTF-8",
        );
    };
    if d.any.learn_callsign(s) {
        MfskStatus::Ok
    } else {
        d.fail(
            MfskStatus::Unsupported,
            "mfsk_decoder_add_callsign: this mode's messages carry no hashed callsigns",
        )
    }
}

// ── Streaming rows and the budget ─────────────────────────────────────────

/// Deliver each row through `callback` **as it is found**, in addition to
/// the array at the end of the call. Pass NULL to stop.
///
/// Threading: with `rayon` (the `desktop` feature) the callback may fire from
/// a worker thread, several at once, in completion order; without it
/// (`mobile`) it fires on the calling thread in candidate order, a stronger
/// contract. Either way the array is the authoritative set.
///
/// # Safety
/// `callback`, if non-null, must be callable from any thread, any number of
/// times including zero, for as long as it is set; `user_data` must stay
/// valid for that time.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_set_on_decode(
    dec: *mut MfskDecoder,
    callback: MfskDecodeCallback,
    user_data: *mut c_void,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_set_on_decode: null decoder handle");
        return MfskStatus::NullPointer;
    };
    d.on_decode = callback;
    d.on_decode_user = SyncUserData(user_data);
    MfskStatus::Ok
}

/// Poll `check` during every subsequent decode; returning `false` stops the
/// search and returns what was found. NULL removes the budget. The check
/// runs once per candidate before it is claimed; a candidate already running
/// finishes. Only modes with `MFSK_CAP_BUDGET` take one:
/// `MFSK_STATUS_UNSUPPORTED` otherwise.
///
/// # Safety
/// `check`, if non-null, must be callable from any thread, any number of
/// times, for as long as it is set; `user_data` must stay valid for that time.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_set_budget(
    dec: *mut MfskDecoder,
    check: MfskBudgetCheck,
    user_data: *mut c_void,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_set_budget: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if check.is_some() && mfsk_mode_caps(d.mode as u32) & MFSK_CAP_BUDGET == 0 {
        let name = mode_index(d.mode).map(mode_name_str).unwrap_or("?");
        return d.fail(
            MfskStatus::Unsupported,
            format!("mfsk_decoder_set_budget: {name} does not publish MFSK_CAP_BUDGET"),
        );
    }
    d.budget = check;
    d.budget_user = SyncUserData(user_data);
    MfskStatus::Ok
}

/// What the budget cut short on the **last** decode. Zeroed when no budget
/// was set or it was never reached, so it can be read unconditionally.
///
/// # Safety
/// `out` must point to at least `out->size` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_last_budget(
    dec: *const MfskDecoder,
    out: *mut MfskBudgetReport,
) -> MfskStatus {
    let Some(d) = handle_ref(dec) else {
        set_error("mfsk_decoder_last_budget: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if out.is_null() {
        set_error("mfsk_decoder_last_budget: null out pointer");
        return MfskStatus::NullPointer;
    }
    unsafe { write_size_versioned(out, &d.last_budget) };
    MfskStatus::Ok
}

// ── Decoding ──────────────────────────────────────────────────────────────

pub(crate) fn row_of(mode: MfskMode, decoded: &Decoded, detail: &RowDetail) -> MfskDecode {
    let mut r = MfskDecode {
        size: core::mem::size_of::<MfskDecode>() as u32,
        mode,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz: decoded.freq_hz,
        dt_sec: decoded.dt_sec,
        snr_db: decoded.snr_db,
        sync_score: detail.sync_score.unwrap_or(0.0),
        sync_cv: detail.sync_cv.unwrap_or(0.0),
        hard_errors: detail.hard_errors.unwrap_or(0),
        info_bits: detail.info.len() as u16,
        pass: detail.pass,
        flags: 0,
        key_bits: 0,
        key: [0; MFSK_DECODE_KEY_LEN],
        delivery: detail.delivery.map_or(-1, |d| d as i32),
        stage: match detail.stage {
            Some(mfsk_core::decoder::Stage::Early) => MFSK_STAGE_EARLY,
            Some(_) => MFSK_STAGE_FINAL,
            None => MFSK_STAGE_NONE,
        },
    };
    write_field(&mut r.text, &decoded.text);
    // The message bits: the first 77 of the information block, packed.
    let bits = &detail.info[..detail.info.len().min(8 * MFSK_DECODE_KEY_LEN)];
    let bits = &bits[..bits.len().min(77)];
    r.key_bits = bits.len() as u8;
    for (i, &b) in bits.iter().enumerate() {
        r.key[i / 8] |= (b & 1) << (7 - i % 8);
    }
    if detail.hash_resolved {
        r.flags |= MFSK_DECODE_FLAG_HASH_RESOLVED;
    }
    if detail.copied_last_tx {
        r.flags |= MFSK_DECODE_FLAG_COPIED_LAST_TX;
    }
    // Which of the three numbers above are the mode's own (#594).
    if detail.sync_score.is_some() {
        r.flags |= MFSK_DECODE_FLAG_HAS_SYNC_SCORE;
    }
    if detail.sync_cv.is_some() {
        r.flags |= MFSK_DECODE_FLAG_HAS_SYNC_CV;
    }
    if detail.hard_errors.is_some() {
        r.flags |= MFSK_DECODE_FLAG_HAS_HARD_ERRORS;
    }
    r
}

/// Decode one period of `slot` and store the rows on the handle.
/// `prefix`: a `decode_prefix` call (#572) rather than a whole-period one.
fn run(d: &mut FfiDecoder, audio: Audio<'_>, period: i64, prefix: bool) -> Result<(), String> {
    d.short_prefix = None;
    let mut slot = SlotInput::new(audio);
    if period != MFSK_PERIOD_NONE {
        slot = slot.period(period);
    }
    let (check, user) = (d.budget, d.budget_user);
    let budget = check.map(|c| move || unsafe { c(user.ptr()) });
    let budget_ref = budget.as_ref().map(|b| b as &(dyn Fn() -> bool + Sync));
    if let Some(b) = budget_ref {
        slot = slot.budget(b);
    }
    let mode = d.mode;
    let (cb, cb_user) = (d.on_decode, d.on_decode_user);
    let deliver = move |dec: &Decoded, det: &RowDetail| {
        let row = row_of(mode, dec, det);
        if let Some(cb) = cb {
            unsafe { cb(&row, cb_user.ptr()) };
        }
    };
    let result = match (cb.is_some(), prefix) {
        (true, false) => d.any.decode_with(&slot, &deliver),
        (false, false) => d.any.decode(&slot),
        (true, true) => d.any.decode_prefix_with(&slot, &deliver),
        (false, true) => d.any.decode_prefix(&slot),
    };
    d.last_budget = budget_report(&result.budget);
    d.last = result.rows.into_iter().zip(result.details).collect();
    Ok(())
}

/// Copy the last decode's rows into the caller's array.
///
/// # Safety
/// `out` must be `cap` writable [`MfskDecode`], or null when `cap` is 0.
/// [`emit`], remembering a prefix call it refused for a short buffer.
unsafe fn emit_prefix(
    d: &mut FfiDecoder,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
    prefix: Option<(i64, usize)>,
) -> MfskStatus {
    let st = unsafe { emit(d, out, cap, out_len) };
    if st == MfskStatus::InvalidArg {
        d.short_prefix = prefix;
    }
    st
}

unsafe fn emit(
    d: &mut FfiDecoder,
    out: *mut MfskDecode,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    if !out_len.is_null() {
        unsafe { *out_len = d.last.len() };
    }
    if d.last.len() > cap || (out.is_null() && !d.last.is_empty()) {
        return d.fail(
            MfskStatus::InvalidArg,
            "decode: output buffer too small; *out_len is the count needed",
        );
    }
    for (i, (dec, det)) in d.last.iter().enumerate() {
        unsafe { write_size_versioned(out.add(i), &row_of(d.mode, dec, det)) };
    }
    MfskStatus::Ok
}

/// Decode one period of 16-bit PCM at `sample_rate` (resampled to 12 kHz if
/// it is not).
///
/// Rows go into `out[0..out_cap]`; `*out_len` always receives the number
/// found, so a short buffer returns `MFSK_STATUS_INVALID_ARG` with the
/// required count rather than a truncated answer you cannot detect.
/// `period` is the period's index on the UTC grid, or `MFSK_PERIOD_NONE`.
///
/// # Safety
/// `samples` must be `n_samples` readable `int16_t`; `out` must be
/// `out_cap` writable [`MfskDecode`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_decode_i16(
    dec: *mut MfskDecoder,
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    unsafe {
        decode_i16_impl(
            dec,
            samples,
            n_samples,
            sample_rate,
            period,
            out,
            out_cap,
            out_len,
            false,
        )
    }
}

/// Decode the period so far, keeping what this period has already found
/// (#572): call it as audio arrives, with **every sample of the period
/// received up to now**, and `period` set. The decoder infers the stage from
/// how much audio it is given. FT8 acts at 141 696 samples (checkpoint A,
/// `nzhsym` 41, ~11.8 s: its rows come back with
/// `MfskDecode::stage == MFSK_STAGE_EARLY`), at 162 432 (subtraction only;
/// no rows) and at the whole period, 180 000 samples, whose call returns the
/// period's complete set, the rows `mfsk_decoder_decode_i16` returns for the
/// same audio. Any other call returns no rows, as does every call of a mode
/// with no early decode until the whole period. A call for the same period
/// after the whole one returns the complete set again without decoding; a
/// call for another period starts afresh. With `MFSK_PERIOD_NONE` it is
/// `mfsk_decoder_decode_i16`. The callback set with
/// `mfsk_decoder_set_on_decode` sees each row once across the period's
/// calls, and `delivery` counts across them. At a rate other than 12 kHz each
/// prefix is resampled on its own, so the result is close to, not
/// byte-equal to, the whole-period decode.
///
/// # Safety
/// As [`mfsk_decoder_decode_i16`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_decode_prefix_i16(
    dec: *mut MfskDecoder,
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    unsafe {
        decode_i16_impl(
            dec,
            samples,
            n_samples,
            sample_rate,
            period,
            out,
            out_cap,
            out_len,
            true,
        )
    }
}

#[allow(clippy::too_many_arguments)]
unsafe fn decode_i16_impl(
    dec: *mut MfskDecoder,
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
    prefix: bool,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_decode_i16: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if samples.is_null() || (out.is_null() && out_cap != 0) {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_decode_i16: null buffer pointer",
        );
    }
    let pcm = unsafe { slice::from_raw_parts(samples, n_samples) };
    let resampled;
    let audio: &[i16] = if sample_rate == 12_000 {
        pcm
    } else {
        resampled = mfsk_core::engine::dsp::resample::resample_to_12k(pcm, sample_rate);
        &resampled
    };
    let retry = prefix && d.short_prefix.take() == Some((period, n_samples));
    if !retry && let Err(e) = in_pool_mut(|| run(d, Audio::I16(audio), period, prefix)) {
        return d.fail(MfskStatus::InvalidArg, e);
    }
    unsafe {
        emit_prefix(
            d,
            out,
            out_cap,
            out_len,
            prefix.then_some((period, n_samples)),
        )
    }
}

/// Decode one period of 32-bit float PCM, any level. At 12 kHz the float
/// goes to the decoder as it is: the modes whose engines work in `float`
/// (WSPR, JT9, JT65, Q65) never see 16 bits, and FT8, FT4 and FST4, which
/// take 16-bit audio as WSJT-X does, get it scaled to a fixed level, so a
/// caller never picks one. At another rate it is resampled and
/// peak-normalised first.
///
/// # Safety
/// As [`mfsk_decoder_decode_i16`], with `samples` as `float`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_decode_f32(
    dec: *mut MfskDecoder,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    unsafe {
        decode_f32_impl(
            dec,
            samples,
            n_samples,
            sample_rate,
            period,
            out,
            out_cap,
            out_len,
            false,
        )
    }
}

/// [`mfsk_decoder_decode_prefix_i16`] for `float` audio. The level of the
/// period's first prefix sets the gain for the rest of its calls.
///
/// # Safety
/// As [`mfsk_decoder_decode_f32`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_decode_prefix_f32(
    dec: *mut MfskDecoder,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    unsafe {
        decode_f32_impl(
            dec,
            samples,
            n_samples,
            sample_rate,
            period,
            out,
            out_cap,
            out_len,
            true,
        )
    }
}

#[allow(clippy::too_many_arguments)]
unsafe fn decode_f32_impl(
    dec: *mut MfskDecoder,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    period: i64,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
    prefix: bool,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_decode_f32: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if samples.is_null() || (out.is_null() && out_cap != 0) {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_decode_f32: null buffer pointer",
        );
    }
    let pcm = unsafe { slice::from_raw_parts(samples, n_samples) };
    if prefix && d.short_prefix.take() == Some((period, n_samples)) {
        return unsafe { emit_prefix(d, out, out_cap, out_len, Some((period, n_samples))) };
    }
    let r = if sample_rate == 12_000 {
        in_pool_mut(|| run(d, Audio::F32(pcm), period, prefix))
    } else {
        let audio = mfsk_core::engine::dsp::resample::resample_f32_to_12k(pcm, sample_rate);
        in_pool_mut(|| run(d, Audio::I16(&audio), period, prefix))
    };
    if let Err(e) = r {
        return d.fail(MfskStatus::InvalidArg, e);
    }
    unsafe {
        emit_prefix(
            d,
            out,
            out_cap,
            out_len,
            prefix.then_some((period, n_samples)),
        )
    }
}

/// FEC information bits for the `index`-th row of the last decode. The raw
/// bits are deliberately not a row field: they are 91 or 101 bytes and only a
/// caller doing subtraction or persistence wants them. `MfskDecode::info_bits`
/// says how many there are (0 for a mode that has none).
///
/// # Safety
/// `out` must be `cap` writable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_copy_info(
    dec: *const MfskDecoder,
    index: usize,
    out: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(d) = handle_ref(dec) else {
        set_error("mfsk_decoder_copy_info: null decoder handle");
        return MfskStatus::NullPointer;
    };
    let Some((_, det)) = d.last.get(index) else {
        set_error("mfsk_decoder_copy_info: index past the last decode's row count");
        return MfskStatus::InvalidArg;
    };
    if !out_len.is_null() {
        unsafe { *out_len = det.info.len() };
    }
    if det.info.len() > cap || (out.is_null() && !det.info.is_empty()) {
        set_error("mfsk_decoder_copy_info: buffer too small; *out_len is the size needed");
        return MfskStatus::InvalidArg;
    }
    unsafe { ptr::copy_nonoverlapping(det.info.as_ptr(), out, det.info.len()) };
    MfskStatus::Ok
}

/// Decode the stream's ready slot directly, without copying it out and back
/// in (FST4-300's slot is 3 600 000 samples), with the slot's own index as
/// the period. `MFSK_STATUS_UNSUPPORTED` with `*out_len = 0` when no slot is
/// ready yet, so a caller can poll this instead of
/// [`mfsk_stream_slot_ready`].
///
/// # Safety
/// As [`mfsk_decoder_decode_i16`], plus `stream` must be a live stream
/// opened for the same mode as `dec`; `out_period` and `out_slot_start_utc_ns`
/// may be null.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_decode_stream(
    dec: *mut MfskDecoder,
    stream: *mut MfskStream,
    out: *mut MfskDecode,
    out_cap: usize,
    out_len: *mut usize,
    out_period: *mut i64,
    out_slot_start_utc_ns: *mut i64,
) -> MfskStatus {
    let Some(d) = handle(dec) else {
        set_error("mfsk_decoder_decode_stream: null decoder handle");
        return MfskStatus::NullPointer;
    };
    let Some(st) = stream_inner(stream) else {
        return d.fail(
            MfskStatus::NullPointer,
            "mfsk_decoder_decode_stream: null stream handle",
        );
    };
    if st.mode != d.mode {
        return d.fail(
            MfskStatus::InvalidArg,
            "mfsk_decoder_decode_stream: the stream and the decoder are for different modes",
        );
    }
    if !out_len.is_null() {
        unsafe { *out_len = 0 };
    }
    let Some(slot) = st.ready.take() else {
        set_error("mfsk_decoder_decode_stream: no whole slot ready yet");
        return MfskStatus::Unsupported;
    };
    if !out_period.is_null() {
        unsafe { *out_period = slot.period };
    }
    if !out_slot_start_utc_ns.is_null() {
        unsafe { *out_slot_start_utc_ns = slot.utc_ns.unwrap_or(0) };
    }
    if let Err(e) = in_pool_mut(|| run(d, Audio::I16(&slot.audio), slot.period, false)) {
        return d.fail(MfskStatus::InvalidArg, e);
    }
    unsafe { emit(d, out, out_cap, out_len) }
}

/// As [`mfsk_unpack77`], resolving `<...>` callsigns against the decoder's
/// own callsign table.
///
/// # Safety
/// As [`mfsk_unpack77`]; `dec` must be a live decoder.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_unpack77(
    dec: *const MfskDecoder,
    message77: *const u8,
    out: *mut c_char,
    cap: usize,
    out_len: *mut usize,
) -> MfskStatus {
    let Some(d) = handle_ref(dec) else {
        set_error("mfsk_decoder_unpack77: null decoder handle");
        return MfskStatus::NullPointer;
    };
    if message77.is_null() {
        set_error("mfsk_decoder_unpack77: message77 is NULL");
        return MfskStatus::InvalidArg;
    }
    let bits = unsafe { slice::from_raw_parts(message77, 77) };
    unsafe {
        put_text(
            d.any_unpack77(bits),
            "mfsk_decoder_unpack77",
            out,
            cap,
            out_len,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mfsk_core::ProtocolId;

    const ALL: u8 = MFSK_DECODE_FLAG_HAS_SYNC_SCORE
        | MFSK_DECODE_FLAG_HAS_SYNC_CV
        | MFSK_DECODE_FLAG_HAS_HARD_ERRORS;

    /// A mode that reports none of the three numbers leaves their flags clear,
    /// so the `0` in the row is told from a measured `0` (#594).
    #[test]
    fn a_row_says_which_of_its_numbers_the_mode_reported() {
        let decoded = Decoded::new("K1ABC W9XYZ EN37", 1_100.0, 0.1, -12.0, ProtocolId::Jt65);

        // WSPR, JT9, JT65, Q65: nothing reported.
        let none = row_of(MfskMode::Jt65, &decoded, &RowDetail::default());
        assert_eq!(none.flags & ALL, 0, "flags {:#x}", none.flags);
        assert_eq!(
            (none.sync_score, none.sync_cv, none.hard_errors),
            (0.0, 0.0, 0)
        );

        // FT8: all three, and a clean decode is a real zero.
        let mut d = RowDetail::default();
        d.sync_score = Some(2.5);
        d.sync_cv = Some(0.1);
        d.hard_errors = Some(0);
        let all = row_of(MfskMode::Ft8, &decoded, &d);
        assert_eq!(all.flags & ALL, ALL);
        assert_eq!(
            (all.sync_score, all.sync_cv, all.hard_errors),
            (2.5, 0.1, 0)
        );

        // An a7 row: the error count, and no sync.
        let mut a7 = RowDetail::default();
        a7.hard_errors = Some(4);
        let r = row_of(MfskMode::Ft8, &decoded, &a7);
        assert_eq!(r.flags & ALL, MFSK_DECODE_FLAG_HAS_HARD_ERRORS);
    }
}
