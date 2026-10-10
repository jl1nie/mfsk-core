//! FST4W receive: `lib/fst4_decode.f90` at `v3.3.0-beta1` with `iwspr=1`.
//!
//! The front end is FST4's (whole-slot FFT, `get_candidates_fst4`,
//! `fst4_sync_search`, the bit-metric variants), shared through
//! `engine::pipeline::fst4_refined_candidates`. The per-candidate
//! ladder is its own — `fst4_decode.f90:648-812` — and not
//! `process_candidate_basic`'s, because it differs in shape, not in a knob:
//!
//! - per LLR variant (nsym 1, 2, 4, 8) it runs **Keff 66** (`decode240_74`
//!   with `maxosd=2, norder=3`) and, if that gave no message and the depth is 3,
//!   **Keff 50** (`maxosd=1, norder=4`) before moving to the next variant;
//! - Keff 50 has no CRC, so its message is accepted only if it contains a
//!   known `CALL GRID` ([`Fst4wState::wcalls`]), which Keff 66 decodes of
//!   type-1 messages add to;
//! - that list is state *between candidates* of one slot, so candidates are
//!   walked in order, one at a time (the FST4 pipeline runs them in parallel);
//! - the search window is `nfqso ± ntol`, there is no a-priori decoding, and
//!   the dedupe key is (message, unresolved 22-bit hash).
//!
//! What differs from the Fortran, deliberately:
//! - `dt_sec` is upstream's `xdt` (see `nspsec` below), where the FST4 pipeline reports
//!   `i0 / fs2 - 1`;
//! - the candidate gate is the FST4 pipeline's `nsync > 16` (upstream
//!   `nsync >= 16`, `get_fst4_bitmetrics.f90`); see `fst4::decode`'s `SYNC_Q_MIN`
//!   for the FST4 measurement. Not measured for FST4W yet;
//! - a message learns its callsigns into the hash table only once it is
//!   accepted, where upstream's `unpack77_for_state(..., nrx=1)` learns from
//!   every message that unpacks, accepted or not — so a Keff-50 word the list
//!   then refuses leaves the table alone;
//! - the survivors of the duplicate pass are the FST4 pipeline's (highest refined
//!   score wins), where upstream keeps the earlier one in list order.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::engine::Protocol;
use crate::engine::dsp::downsample::DownsampleCfg;
use crate::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra, sync_quality};
use crate::engine::pipeline::{
    BudgetCheck, BudgetReport, GenericPipelineProtocol, SnrCtx, fst4_refined_candidates,
};
use crate::engine::sync::{fine_sync_power_per_block, sync_power_cv};
use crate::engine::sync2d::freq_shift_cd0_into;
use crate::engine::tx::info_to_tones;
use crate::fec::Ldpc240_74;
use crate::fec::ldpc240_74::{Decoded74, Osd74Work, decode240_74};
use crate::fst4::decode::{
    FST4_120_DOWNSAMPLE, FST4_300_DOWNSAMPLE, FST4_900_DOWNSAMPLE, FST4_1800_DOWNSAMPLE,
};
use crate::msg::hash_table::CallsignHashTable;
use crate::msg::wsjt77::{register_callsigns, unpack77_with_hash};

use super::{Fst4w120, Fst4w300, Fst4w900, Fst4w1800, Fst4wMessage};

/// `get_candidates_fst4`'s `minsync` (`fst4_decode.f90:309`).
pub const MINSYNC: f32 = 1.20;
/// `candidates(MAXCAND=100, ...)` in `decode_kernel` for FST4W
/// (`fst4_decode.f90:270`); `get_candidates_fst4` itself stops at 200.
pub const MAX_CAND: usize = 100;
/// The FST4 pipeline's gate on the Costas sync count (`fst4::decode`'s
/// `SYNC_Q_MIN`).
const SYNC_Q_MIN: u32 = 16;
/// `MAXWCALLS` (`fst4_decode.f90:270`).
pub const MAX_WCALLS: usize = 100;
/// `character(len=20) :: wcalls(100)`.
const WCALL_LEN: usize = 20;

/// `fst4_decode.f90:592-621`, as FST4's: see `fst4::baseline`.
macro_rules! snr_via_fst4_baseline {
    ($($proto:ty),*) => {$(
        impl GenericPipelineProtocol for $proto {
            fn snr_db(ctx: SnrCtx<'_>) -> f32 {
                crate::fst4::baseline::fst4_snr_db::<$proto>(
                    ctx.itone,
                    ctx.cand_freq_hz,
                    ctx.refined_freq_hz,
                    ctx.i_start,
                    ctx.fft_cache,
                    ctx.ds_cfg,
                    <$proto>::SNR_CALFAC,
                )
            }
        }
    )*};
}
snr_via_fst4_baseline!(Fst4w120, Fst4w300, Fst4w900, Fst4w1800);

/// The downsampler configuration of a period.
pub(crate) trait Fst4wPeriod: GenericPipelineProtocol + Protocol<Fec = Ldpc240_74> {
    const DOWNSAMPLE: DownsampleCfg;
}
impl Fst4wPeriod for Fst4w120 {
    const DOWNSAMPLE: DownsampleCfg = FST4_120_DOWNSAMPLE;
}
impl Fst4wPeriod for Fst4w300 {
    const DOWNSAMPLE: DownsampleCfg = FST4_300_DOWNSAMPLE;
}
impl Fst4wPeriod for Fst4w900 {
    const DOWNSAMPLE: DownsampleCfg = FST4_900_DOWNSAMPLE;
}
impl Fst4wPeriod for Fst4w1800 {
    const DOWNSAMPLE: DownsampleCfg = FST4_1800_DOWNSAMPLE;
}

/// What FST4W keeps between periods: the callsign hash table and the Keff-50
/// call list (`this%wcalls`, `this%nwcalls`).
#[derive(Default)]
pub struct Fst4wState {
    pub(crate) table: CallsignHashTable,
    wcalls: Vec<String>,
    work: Osd74Work,
}

impl Fst4wState {
    /// The known-call list, oldest first: `get_known_calls`.
    pub fn wcalls(&self) -> &[String] {
        &self.wcalls
    }

    /// Replace the list: `set_known_calls` (`fst4_decode.f90:210-218`). An entry
    /// is kept to 20 characters, as `character(len=20)`; more than
    /// [`MAX_WCALLS`] is refused, as upstream's `status=-1`.
    pub fn set_wcalls(&mut self, calls: &[String]) -> Result<(), TooManyCalls> {
        if calls.len() > MAX_WCALLS {
            return Err(TooManyCalls);
        }
        self.wcalls = calls.iter().map(|c| fit_wcall(c)).collect();
        Ok(())
    }

    /// `fst4_decode.f90:757-780`: remember the `CALL GRID` of a type-1 message
    /// Keff 66 decoded, if no entry holds it.
    fn learn_wcall(&mut self, msg: &str) {
        let first = msg.find(' ').unwrap_or(msg.len());
        let rest = msg.get(first + 1..).unwrap_or("");
        let end = match rest.find(' ') {
            Some(q) => first + 1 + q,
            None => msg.len(),
        };
        let wpart = msg[..end].trim_end();
        // only type-1 messages
        if wpart.is_empty() || wpart.contains('/') || wpart.contains('<') {
            return;
        }
        if self.wcalls.iter().any(|w| w.contains(wpart)) {
            return;
        }
        let wpart = fit_wcall(wpart);
        if self.wcalls.len() < MAX_WCALLS {
            self.wcalls.push(wpart);
        } else {
            self.wcalls.remove(0);
            self.wcalls.push(wpart);
        }
    }

    /// `fst4_decode.f90:803-807`: a Keff-50 message is accepted if it holds any
    /// **non-blank** entry (beta1; rc1 matched blank ones as well).
    fn knows(&self, msg: &str) -> bool {
        self.wcalls
            .iter()
            .any(|w| !w.is_empty() && msg.contains(w.as_str()))
    }
}

/// An entry as `character(len=20)` holds it: cut to 20 characters, trailing
/// blanks gone.
fn fit_wcall(s: &str) -> String {
    let cut = s.get(..WCALL_LEN).unwrap_or(s);
    cut.trim_end().to_string()
}

/// [`Fst4wState::set_wcalls`] was given more than [`MAX_WCALLS`] entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TooManyCalls;

/// One FST4W decode.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Fst4wResult {
    /// The 74 information bits: 50 payload and the CRC-24 over them.
    pub info: Box<[u8]>,
    /// The message, `<...>` resolved against the decoder's hash table as it
    /// stood when the row was found.
    pub text: String,
    /// Tone-0 frequency, Hz.
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
    /// Hard errors against the input LLRs.
    pub hard_errors: u32,
    /// Refined `fst4_sync_search` score.
    pub sync_score: f32,
    pub sync_cv: f32,
    /// `Keff` of the rung that decoded: 66, or 50 (no CRC; the call list
    /// vouched for it).
    pub keff: u8,
    /// Which LLR variant: 0 nsym 1, 1 nsym 2, 2 nsym 4, 3 nsym 8.
    pub variant: u8,
    /// The unresolved 22-bit callsign hash, for a `<...>` message
    /// (`result%hash22`).
    pub hash22: Option<u32>,
}

/// The settings of one slot.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SlotSettings {
    pub nfqso: f32,
    pub ntol: f32,
    /// `ndepth`: 3 runs Keff 50 and the timing retry, 2 the retry only, 1 neither.
    pub ndepth: u32,
}

/// `unpack77` of 50 payload bits, `<...>` resolved against `table`.
fn text_of(payload: &[u8], table: &CallsignHashTable) -> Option<(String, [u8; 77])> {
    let b = Fst4wMessage::payload_to_77(payload)?;
    unpack77_with_hash(&b, table).map(|t| (t, b))
}

/// One LLR variant through the rungs: Keff 66, then (depth 3) Keff 50.
fn try_variant(
    state: &mut Fst4wState,
    llr: &[f32],
    k50: bool,
) -> Option<(Decoded74, u8, String, [u8; 77])> {
    if let Some(d) = decode240_74(&mut state.work, llr, 66, 2, 3, None) {
        // `count(cw.eq.1).eq.0`: the all-zero word verifies by construction.
        if d.cw.iter().all(|&b| b == 0) {
            return None;
        }
        if let Some((text, b77)) = text_of(&d.message74[..50], &state.table) {
            if k50 {
                state.learn_wcall(&text);
            }
            return Some((d, 66, text, b77));
        }
    }
    if k50 && let Some(d) = decode240_74(&mut state.work, llr, 50, 1, 4, None) {
        if d.cw.iter().all(|&b| b == 0) {
            return None;
        }
        if let Some((text, b77)) = text_of(&d.message74[..50], &state.table)
            && state.knows(&text)
        {
            return Some((d, 50, text, b77));
        }
    }
    None
}

/// Decode one slot of `P`. Rows are handed to `on_row` as they are found and
/// returned in that order.
pub(crate) fn decode_slot<P: Fst4wPeriod>(
    audio: &[i16],
    s: &SlotSettings,
    state: &mut Fst4wState,
    budget: Option<BudgetCheck<'_>>,
    on_row: Option<&(dyn Fn(&Fst4wResult) + Sync)>,
) -> (Vec<Fst4wResult>, BudgetReport) {
    let cfg = &P::DOWNSAMPLE;
    let mut report = BudgetReport::default();
    // `fst4_decode.f90:517-521`: the window is `nfqso ± ntol`, which
    // `fst4_coarse_sync` widens and shifts exactly as upstream does for the
    // signal and noise windows.
    let (survivors, fft_cache) = fst4_refined_candidates::<P>(
        audio,
        cfg,
        s.nfqso - s.ntol,
        s.nfqso + s.ntol,
        MINSYNC,
        MAX_CAND,
    );
    let k50 = s.ndepth >= 3;
    let jitters: &[i32] = if s.ndepth >= 2 { &[0, 1, -1] } else { &[0] };
    let nss = (P::NSPS / P::NDOWN) as i32;
    let n_sym = P::N_SYMBOLS as i32;
    let ds_rate = cfg.input_rate as f32 / P::NDOWN as f32;
    // `xdt=(isbest-nspsec)/fs2`, `nspsec=nint(fs2)` (`fst4_decode.f90:641`): the
    // frame is taken to start `nint(fs2)` samples in, not exactly one second,
    // which is 0.12 s at FST4W-1800's 3.57 Hz baseband.
    let nspsec = (ds_rate + 0.5) as i32;

    let mut rows: Vec<Fst4wResult> = Vec::new();
    // `decodes(i)`, `decoded_hashes(i)`: the dedupe key.
    let mut seen: Vec<(String, Option<u32>)> = Vec::new();
    let mut shifted: Vec<num_complex::Complex<f32>> = Vec::new();
    let total = survivors.len();

    for (n, (cand, cd0, freq_hz, i0, score)) in survivors.into_iter().enumerate() {
        if let Some(check) = budget
            && !check()
        {
            report.exhausted = true;
            report.candidates_skipped = (total - n) as u32;
            report.cut_at_score = Some(score);
            break;
        }
        report.stages_run += 1;
        // `isbest<0`: no start with a whole frame behind it.
        if score == f32::NEG_INFINITY {
            continue;
        }
        let dt_sec = (i0 - nspsec) as f32 / ds_rate;
        freq_shift_cd0_into(&cd0, freq_hz - cand.freq_hz, ds_rate, &mut shifted);
        let nfft2 = shifted.len() as i32;

        'jitter: for &off in jitters {
            let is0 = i0 + off;
            if is0 < 0 || is0 + n_sym * nss > nfft2 {
                continue;
            }
            let cs = symbol_spectra::<P>(&shifted, is0);
            if sync_quality::<P>(&cs) <= SYNC_Q_MIN {
                continue;
            }
            let sync_cv = sync_power_cv(&fine_sync_power_per_block::<P>(&shifted, is0));
            for variant in 0..4u8 {
                let llr: Vec<f32> = match variant {
                    0 => compute_llr_fast::<P, f32>(&cs).llra,
                    1 => compute_llr_partial::<P, f32, f32>(&cs, 2),
                    2 => compute_llr_partial::<P, f32, f32>(&cs, 4),
                    _ => compute_llr_partial::<P, f32, f32>(&cs, P::LLR_NSYM_MAX as usize),
                };
                let Some((d, keff, text, b77)) = try_variant(state, &llr, k50) else {
                    continue;
                };
                // an unresolved hash is told apart by its 22 bits
                let hash22 = text
                    .contains("<...>")
                    .then(|| b77[..22].iter().fold(0u32, |a, &b| (a << 1) | b as u32));
                if seen.iter().any(|(t, h)| *t == text && *h == hash22) {
                    break 'jitter;
                }
                seen.push((text.clone(), hash22));
                register_callsigns(&b77, &mut state.table);
                let itone = info_to_tones::<P>(&d.message74);
                let snr_db = P::snr_db(SnrCtx {
                    cs: &cs,
                    itone: &itone,
                    cd0: &shifted,
                    ds_rate_hz: ds_rate,
                    cand_score: cand.score,
                    cand_freq_hz: cand.freq_hz,
                    fft_cache: &fft_cache,
                    ds_cfg: cfg,
                    refined_freq_hz: freq_hz,
                    i_start: is0,
                });
                let row = Fst4wResult {
                    info: Box::from(&d.message74[..]),
                    text,
                    freq_hz,
                    dt_sec,
                    snr_db,
                    hard_errors: d.nharderror,
                    sync_score: score,
                    sync_cv,
                    keff,
                    variant,
                    hash22,
                };
                if let Some(cb) = on_row {
                    cb(&row);
                }
                rows.push(row);
                break 'jitter;
            }
        }
    }
    (rows, report)
}
