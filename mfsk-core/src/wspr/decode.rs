//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use alloc::vec::Vec;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::engine::pipeline::scan_dedup_match;
use crate::msg::WsprMessage;

use super::search::SearchParams;
#[cfg(any())]
use super::search::coarse_search;

/// One successful WSPR decode.
#[derive(Clone, Debug)]
pub struct WsprResult {
    /// Recovered message payload.
    pub message: WsprMessage,
    /// Base frequency (tone 0) used for demodulation.
    pub freq_hz: f32,
    /// Sample index at which symbol 0 started, in the *caller's* audio
    /// buffer. **Clamped to 0** when the signal actually started before
    /// the buffer (negative-dt case); use [`Self::dt_sec`] for the
    /// signed offset that matches wsprd's reporting.
    pub start_sample: usize,
    /// wsprd-equivalent `dt`: signal-start offset in seconds, relative
    /// to the WSPR nominal anchor (slot start + 1 s). Positive values
    /// = signal arrived late, negative = arrived early. Range that
    /// `decode_scan` can express: `−NEGATIVE_DT_PAD_SEC .. +∞`.
    pub dt_sec: f32,
    /// Linear drift the successful demodulation ran at, in Hz across
    /// the 110.6 s frame — wsprd's `drift1`.
    ///
    /// Load-bearing for subtraction, not decoration: wsprd hands this
    /// straight to `subtract_signal2` (`wsprd.c:1446`) so the replica
    /// it removes drifts the same way the signal did. Reconstructing a
    /// drifting signal as a stationary one leaves most of it behind.
    pub drift_hz: f32,
    /// 50-bit FEC info payload returned by Fano. Used by
    /// `decode_scan_subtract` to reconstruct the 162-channel-symbol
    /// stream and subtract the signal from the audio for SIC.
    pub info_bits: [u8; 50],
    /// wsprd-calibrated SNR estimate (dB, WSPR→2500 Hz reference
    /// bandwidth), from the coarse-search candidate that produced this
    /// decode — same `10·log10(smspec) − 26.3` formula wsprd itself
    /// reports next to a spot (`wsprd.c:1093`, see
    /// [`super::coarse_baseband::BasebandCandidate::snr_db`]). Set by
    /// [`decode_scan`] / [`decode_scan_subtract`], which always go
    /// through the coarse search; direct [`decode_at`] /
    /// [`decode_at_baseband`] / [`decode_at_baseband_nblocks`] calls
    /// have no coarse candidate to derive it from and leave this `0.0`.
    pub snr_db: f32,
}

/// Decode one WSPR frame at a known (freq, start_sample). Returns `None`
/// if the Fano decoder fails to converge or the message doesn't unpack.
///
/// Routes through the new 375 Hz baseband demod path
/// ([`super::demod::bit_metrics_from_audio`]) — port of WSJT-X
/// `wsprd.c::noncoherent_sequence_detection` at `nblock=1`. Per-symbol
/// explicit 4-tone Goertzel on the decimated baseband. Closes the
/// W5BIT and NM7J gaps that the previous 12 kHz / 8192-pt-FFT path
/// couldn't reach.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
) -> Option<WsprResult> {
    decode_at_with_drift(audio, sample_rate, start_sample, freq_hz, 0.0)
}

/// Callsigns confirmed by a **Fano** decode earlier in the same scan.
///
/// WSPR carries no CRC, so nothing in the codeword itself distinguishes
/// a real decode from a plausible-looking one. That is survivable for
/// Fano — a sequential decoder simply fails to converge on noise — but
/// not for OSD, which by construction synthesises a valid codeword for
/// *any* input and will happily emit a well-formed callsign built from
/// noise.
///
/// `wsprd.c:1396` guards it structurally rather than by threshold:
///
/// ```c
/// ihash = nhash(callsign, strlen(callsign), 146);
/// if (strncmp(hashtab + ihash*13, callsign, 13) == 0) { … not_decoded = 0; }
/// ```
///
/// — an OSD result is accepted only if that callsign is *already* in
/// the table, i.e. was confirmed by an earlier successful decode. OSD
/// can therefore re-find a known station under conditions Fano can't
/// reach, but can never invent a new one.
///
/// The threshold this replaces (`nhardmin ≤ 44`) cannot work, and that
/// is measurable rather than theoretical: on `150426_0918.wav` the one
/// genuine OSD decode (W3BI, -25 dB) lands at `nhardmin = 39` while
/// the phantoms land at 40, 40, 40, 41, 41, 41, 42, 42 — a separation
/// of one hard error. That gate let 8 phantoms through against 8 real
/// decodes; real `wsprd` reports 9 real and 0 phantom on the same file.
///
/// This crate stores callsign strings rather than `nhash` buckets:
/// same semantics without wsprd's hash-collision risk. Type 1
/// additionally requires the grid to match, mirroring wsprd's
/// `loctab` check.
#[derive(Debug, Clone, Default)]
pub struct WsprCallsignTable {
    /// `(callsign, grid)` — grid is `None` for Type 2 (compound call,
    /// no grid transmitted).
    entries: Vec<(String, Option<String>)>,
}

impl WsprCallsignTable {
    /// Empty table: every OSD result will be rejected.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a Fano-confirmed decode. Type 3 is never recorded — it
    /// carries a hash rather than a callsign, so it can confirm
    /// nothing.
    pub fn record(&mut self, msg: &crate::msg::WsprMessage) {
        let entry = match msg {
            crate::msg::WsprMessage::Type1 { callsign, grid, .. } => {
                (callsign.clone(), Some(grid.clone()))
            }
            crate::msg::WsprMessage::Type2 { callsign, .. } => (callsign.clone(), None),
            crate::msg::WsprMessage::Type3 { .. } => return,
        };
        if !self.entries.contains(&entry) {
            self.entries.push(entry);
        }
    }

    /// Whether an OSD-derived message may be accepted.
    pub fn accepts(&self, msg: &crate::msg::WsprMessage) -> bool {
        match msg {
            crate::msg::WsprMessage::Type1 { callsign, grid, .. } => self
                .entries
                .iter()
                .any(|(c, g)| c == callsign && g.as_deref() == Some(grid.as_str())),
            crate::msg::WsprMessage::Type2 { callsign, .. } => {
                self.entries.iter().any(|(c, _)| c == callsign)
            }
            // Type 3 is a hashed callsign with no table to resolve it
            // against — wsprd's own OSD path requires `itype` 1 or 2.
            crate::msg::WsprMessage::Type3 { .. } => false,
        }
    }
}

#[cfg(test)]
mod callsign_table_tests {
    use super::WsprCallsignTable;
    use crate::msg::WsprMessage;

    fn t1(callsign: &str, grid: &str) -> WsprMessage {
        WsprMessage::Type1 {
            callsign: callsign.into(),
            grid: grid.into(),
            power_dbm: 30,
        }
    }

    /// The OSD gate (`wsprd.c:1396`) admits a result only for a
    /// callsign an earlier Fano decode confirmed. Nothing else may
    /// open it — that is the entire reason OSD is safe to keep.
    #[test]
    fn accepts_only_recorded_callsigns() {
        let mut table = WsprCallsignTable::new();
        assert!(!table.accepts(&t1("W3BI", "FN20")), "empty table accepted");

        table.record(&t1("W3BI", "FN20"));
        assert!(table.accepts(&t1("W3BI", "FN20")));
        assert!(
            !table.accepts(&t1("ZZ9ZZZ", "AA00")),
            "unrecorded callsign accepted"
        );
    }

    /// Type 1 carries a grid, so wsprd requires both to match; Type 2
    /// carries a prefix/suffix instead and matches on callsign alone.
    #[test]
    fn type1_matches_on_grid_too_type2_does_not() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("W3BI", "FN20"));
        assert!(
            !table.accepts(&t1("W3BI", "AA00")),
            "Type 1 accepted a mismatched grid"
        );
        assert!(table.accepts(&WsprMessage::Type2 {
            callsign: "W3BI".into(),
            power_dbm: 30,
        }));
    }

    /// Type 3 is a hashed callsign with nothing to resolve it against,
    /// so wsprd's OSD path refuses it outright (`itype` must be 1 or 2).
    #[test]
    fn hashed_callsigns_are_never_accepted() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("W3BI", "FN20"));
        assert!(!table.accepts(&WsprMessage::Type3 {
            callsign_hash: 0x05c31,
            grid6: "FN20aa".into(),
            power_dbm: 30,
        }));
    }
}

/// Same as [`decode_at`] but with an explicit drift estimate
/// (`drift_hz` is total drift across the 110.6 s frame; matches
/// wsprd's `drift1`). The caller supplies the drift for now; a
/// future drift-search slice will sweep it inside the decode loop
/// like wsprd does.
/// Decode at known alignment using a pre-decimated baseband. Avoids
/// the O(NFFT1) decimation cost when many candidates share the same
/// audio (e.g. inside `decode_scan` / `decode_scan_subtract`).
///
/// `idat`, `qdat`: 46080-sample 375 Hz baseband from
/// [`super::baseband::decimate_to_baseband`].
/// `freq_hz`: tone-0 frequency in audio Hz (matches our coarse-search
/// convention; converted to wsprd's tone-center via `+1.5·df` inside).
/// `start_sample`: audio-rate sample where symbol 0 starts.
pub fn decode_at_baseband(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
) -> Option<WsprResult> {
    decode_at_baseband_nblocks(
        idat,
        qdat,
        sample_rate,
        start_sample,
        freq_hz,
        drift_hz,
        &[1],
    )
}

/// Variant of [`decode_at_baseband`] that tries multiple `nblock`
/// values (e.g. `&[1, 2, 3]`) for coherent block detection. The hot
/// loop scales O(`nblocks.len()`); used by pass 2 of `decode_scan`
/// where the noise-floor reduction makes the extra cost worth it.
pub fn decode_at_baseband_nblocks(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &[usize],
) -> Option<WsprResult> {
    decode_at_baseband_nblocks_gated(
        idat,
        qdat,
        sample_rate,
        start_sample,
        freq_hz,
        drift_hz,
        nblocks,
        None,
    )
}

/// [`decode_at_baseband_nblocks`] with an explicit OSD gate.
///
/// `confirmed` is the set of callsigns an earlier Fano decode already
/// established (see [`WsprCallsignTable`]). `None` — what the
/// un-gated wrappers pass — rejects every OSD result outright, which
/// is the only safe default for a caller with no scan-level context:
/// OSD synthesises a valid codeword for any input, so ungated it is a
/// phantom generator, not a sensitivity feature.
#[allow(clippy::too_many_arguments)]
pub fn decode_at_baseband_nblocks_gated(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
) -> Option<WsprResult> {
    decode_at_baseband_nblocks_gated_drift(
        idat,
        qdat,
        sample_rate,
        start_sample,
        freq_hz,
        drift_hz,
        nblocks,
        confirmed,
        true,
    )
}

/// wsprd's refine cascade (`wsprd.c:1221-1277`), in full. Each stage is
/// `sync_and_demodulate` in mode 0 (search lag, freq fixed) or mode 1
/// (search freq, lag fixed), keeping whichever cell scores highest on
/// the sync metric:
///
///   1. coarse lag   ±128 baseband samples, step 64
///   2. coarse freq  ±2 × 0.25 Hz
///   3. drift refine drift ± 0.5 (passes 0 and 1 only)
///   4. fine lag     ±32 baseband samples, step 16
///   5. fine freq    ±2 × 0.05 Hz
///
/// Stages 3-5 were missing here, and stage 2 ran at ±1.0 Hz in 0.5 Hz
/// steps rather than ±0.5 Hz in 0.25 Hz steps. That combination is why
/// handing this function the coarse stage's drift estimate used to
/// *hurt*: without stage 3 there is nothing to correct a drift the
/// ±4 Hz coarse grid got wrong, and without stages 4-5 the alignment
/// stays a coarse-grid cell away from the true one.
///
/// Extracted out of [`decode_at_baseband_nblocks_gated_drift`] so
/// [`debug_refined_sync`] can call it without duplicating the cascade
/// — the two need to agree on the exact value `minsync2` is compared
/// against, or a diagnostic built against a near-copy is answering a
/// slightly different question than the one being asked.
#[allow(clippy::type_complexity)]
fn refine_cascade(
    idat: &[f32],
    qdat: &[f32],
    f0_baseband_init: f32,
    lag_baseband_init: i32,
    drift_hz: f32,
    refine_drift: bool,
) -> (f32, i32, f32, f32, super::demod::IsQs) {
    super::instrument::bump(&super::instrument::CANDIDATES);

    let mut best_freq = f0_baseband_init;
    let mut best_lag = lag_baseband_init;
    let mut best_drift = drift_hz;

    // Ping-pong pair: `best_isqs` holds the current champion, `scratch`
    // is overwritten by every trial evaluation and swapped in when it
    // wins — no by-value `IsQs` return, no per-call-site local. Exactly
    // two 10 368 B buffers live for this whole 13-eval cascade, instead
    // of one fresh copy per call site (five distinct `let` bindings in
    // the old closure-based version) — see `tone_amplitudes_into`'s
    // doc comment for why that shape is what the embedded stack audit
    // (`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`, "Stack")
    // found dominating `refine_cascade`'s 93-103 KB peak.
    let mut best_isqs = super::demod::IsQs::zeroed();
    let mut scratch = super::demod::IsQs::zeroed();
    super::demod::tone_amplitudes_into(idat, qdat, best_freq, best_lag, best_drift, &mut best_isqs);
    let mut best_sync = super::demod::sync_score_isqs(&best_isqs);

    // Trial `(f, lag, drift)` into `scratch`; if it beats the current
    // champion, adopt its parameters and swap `scratch` in as the new
    // `best_isqs` (pointer/tag swap, no 10 368 B copy).
    macro_rules! try_cell {
        ($f:expr, $lag:expr, $drift:expr) => {{
            super::demod::tone_amplitudes_into(idat, qdat, $f, $lag, $drift, &mut scratch);
            let sync = super::demod::sync_score_isqs(&scratch);
            if sync > best_sync {
                best_sync = sync;
                best_freq = $f;
                best_lag = $lag;
                best_drift = $drift;
                core::mem::swap(&mut best_isqs, &mut scratch);
            }
        }};
    }

    // 1. Coarse lag.
    for &dlag in &[-128i32, -64, 64, 128] {
        try_cell!(best_freq, lag_baseband_init + dlag, best_drift);
    }

    // 2. Coarse freq — wsprd's `fstep = 0.25, ifmin = -2, ifmax = 2`.
    for i in -2i32..=2 {
        if i == 0 {
            continue;
        }
        try_cell!(f0_baseband_init + i as f32 * 0.25, best_lag, best_drift);
    }

    // 3. Drift refine — `wsprd.c:1236-1255`, passes 0 and 1 only. wsprd
    // tries `drift ± 0.5` and keeps the better, checking `+` first so a
    // tie goes to `+` exactly as the C's `else if` does.
    if refine_drift {
        for &dd in &[0.5f32, -0.5] {
            let before = best_sync;
            try_cell!(best_freq, best_lag, drift_hz + dd);
            if best_sync > before {
                break;
            }
        }
    }

    // Stages 4-5 run only above wsprd's `minsync1` (`wsprd.c:800,1259`);
    // below it the candidate is noise and the extra evals are wasted.
    const MINSYNC1: f32 = 0.10;
    if best_sync > MINSYNC1 {
        super::instrument::bump(&super::instrument::MINSYNC1_PASS);
        // 4. Fine lag — ±32, step 16.
        let centre_lag = best_lag;
        for &dlag in &[-32i32, -16, 16, 32] {
            try_cell!(best_freq, centre_lag + dlag, best_drift);
        }
        // 5. Fine freq — `fstep = 0.05, ifmin = -2, ifmax = 2`.
        let centre_freq = best_freq;
        for i in -2i32..=2 {
            if i == 0 {
                continue;
            }
            try_cell!(centre_freq + i as f32 * 0.05, best_lag, best_drift);
        }
    }

    (best_freq, best_lag, best_drift, best_sync, best_isqs)
}

/// Refined `best_sync` for one candidate — the exact value `minsync2`
/// gates on — without running Fano/OSD. Diagnostic-only: written to
/// answer "where does a known weak decode rank among the candidates
/// `minsync2` keeps", which needs the per-candidate refined sync, not
/// just the aggregate pass/fail counters `wspr::instrument` tracks.
///
/// `freq_hz`/`start_sample` follow [`decode_at_baseband_nblocks_gated_drift`]'s
/// own conventions (tone-0 audio Hz; audio-rate sample index).
#[cfg(any(test, feature = "internal-testing"))]
pub fn debug_refined_sync(
    idat: &[f32],
    qdat: &[f32],
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    refine_drift: bool,
) -> f32 {
    let f0_center_init = freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
    let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
    let lag_baseband_init = start_sample as i32 / 32;
    refine_cascade(
        idat,
        qdat,
        f0_baseband_init,
        lag_baseband_init,
        drift_hz,
        refine_drift,
    )
    .3
}

/// [`decode_at_baseband_nblocks_gated`] with wsprd's per-pass drift
/// switch.
///
/// `refine_drift` mirrors `wsprd.c:1236`'s `if (ipass < 2)`: the first
/// two passes try `drift ± 0.5` around the coarse estimate and keep
/// whichever scores better on sync, while the final pass does not —
/// it runs with `maxdrift = 0` throughout, trading drift tracking for
/// a lower-variance frequency estimate on the weak signals that are
/// all that remain by then.
#[allow(clippy::too_many_arguments)]
pub fn decode_at_baseband_nblocks_gated_drift(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
    refine_drift: bool,
) -> Option<WsprResult> {
    // `freq_hz` follows our tone-0 convention (matches `synthesize_audio`
    // and `coarse_search.freq_hz`); wsprd's `noncoherent_sequence_detection`
    // takes the signal CENTER, so we add 1.5·tone_spacing here and sweep
    // around that point.
    let f0_center_init = freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
    let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
    let lag_baseband_init = start_sample as i32 / 32;

    let (best_freq, best_lag, best_drift, best_sync, best_isqs) = refine_cascade(
        idat,
        qdat,
        f0_baseband_init,
        lag_baseband_init,
        drift_hz,
        refine_drift,
    );

    const MINSYNC2_EARLY: f32 = 0.12;
    const MINSYNC2_FINAL: f32 = 0.10;
    let minsync2 = if refine_drift {
        MINSYNC2_EARLY
    } else {
        MINSYNC2_FINAL
    };
    if best_sync <= minsync2 {
        super::instrument::bump(&super::instrument::MINSYNC2_REJECTED);
        return None;
    }

    decode_from_refined(
        idat,
        qdat,
        sample_rate,
        best_freq,
        best_lag,
        best_drift,
        &best_isqs,
        nblocks,
        confirmed,
        None,
    )
}

/// Mode 2 of the refine→demod cascade: bit metrics + Fano/OSD at an
/// *already*-refined alignment. Split out of
/// [`decode_at_baseband_nblocks_gated_drift`] so callers that need to
/// **rank candidates by refined sync before deciding which ones are
/// worth the ladder** — [`decode_pass2_top_n`]'s whole reason for
/// existing — can call [`refine_cascade`] once per candidate, sort,
/// and only pay this function's cost for the survivors, instead of
/// this always running unconditionally per candidate the way it did
/// when the two were one function.
///
/// `nblocks`/`confirmed` and the return value are unchanged from
/// [`decode_at_baseband_nblocks_gated_drift`]'s own contract; this is
/// a pure extraction, not a behaviour change — see that function's
/// remaining body for the minsync2 gate this is called after.
///
/// `budget`: checked before every (nblock, idt) position in the DT
/// peak-up ladder below; `Some(check)` returning `false` aborts the
/// ladder early, equivalent to this candidate simply failing to
/// decode rather than an error. Exists for the ladder's *failing*
/// path specifically — a candidate that never converges pays the
/// full sweep (up to 4 × 17 = 68 positions; ~48 s on CoreS3 for one
/// such candidate, see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Dual-core,
/// take two") since neither loop's wsprd-faithful exit condition
/// (`while (... && not_decoded)`) has any notion of giving up early.
/// `None` runs the exhaustive, wsprd-faithful sweep unconditionally —
/// every existing caller of this function passes `None`; the check is
/// additive, not a default behaviour change.
///
/// `&(dyn Fn() -> bool + Sync)`, not `FnMut`: callers build the check
/// from a fixed captured deadline (embedded: `esp_timer_get_time() <
/// deadline`) rather than mutable state, so it can be shared as-is
/// across [`decode_pass2_top_n`]'s `par_iter()` batch instead of
/// needing one exclusive borrow per candidate.
#[allow(clippy::too_many_arguments)]
fn decode_from_refined(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    best_freq: f32,
    best_lag: i32,
    best_drift: f32,
    best_isqs: &super::demod::IsQs,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> Option<WsprResult> {
    use crate::engine::{FecOpts, MessageCodec};
    let codec = crate::fec::ConvFano;
    // Pooled across the whole `nblocks` sweep below (up to 3 values in
    // pass 2) instead of `fano_decode` allocating a fresh `Vec<Node>`
    // scratch per call — see `fano::fano_decode_with_scratch`'s doc
    // comment for why reuse is safe here.
    let mut fano_scratch = crate::fec::conv::fano::FanoScratch::new();

    // Mode 2: bit metrics + Fano. wsprd does *not* stop at the refined
    // alignment — `wsprd.c:1329-1423` wraps the demod-and-decode step in
    // a **DT peak-up loop**, retrying at `shift1 ± 8k` baseband samples
    // for `k = 0 .. 8` (`iifac = 8`, `idt <= 128 / iifac`), in the order
    // 0, +8, −8, +16, −16, … ±64. Seventeen positions, and it is the
    // inner loop: every `ib` rung gets the full sweep.
    //
    // This is not a refinement of the refinement — the refine cascade
    // maximises *sync*, and the alignment that maximises sync is not
    // always the one Fano converges from. On the WSJT-X golden, wsprd
    // wins `G8VDQ IO91 37` at `shift1 + 16`, the third position it
    // tries, having already scored position 0 higher on sync.
    //
    // Both loops exit on the first success, exactly as the C's
    // `while (ib <= nblocksize && not_decoded)` /
    // `while (not_decoded && idt <= ...)` do, so the cost below is
    // what a *failing* candidate pays, not a succeeding one.
    let mut best_type1: Option<(u32, WsprResult)> = None;
    let mut best_other: Option<(u32, WsprResult)> = None;
    /// `iifac`, `wsprd.c:802` — step size in the final DT peak-up.
    const IIFAC: i32 = 8;
    /// `idt <= 128 / iifac`, `wsprd.c:1322`.
    const N_JITTER: i32 = 128 / IIFAC;
    // `nblock == 0` is this crate's spelling of wsprd's fourth ladder
    // rung (`ib == 4` → `blocksize = 1, bitmetric = 1`) — see
    // `demod::nblock1_bit_metrics_opt`.
    'rungs: for &nblock in nblocks {
        for idt in 0..=N_JITTER {
            if let Some(check) = budget
                && !check()
            {
                super::instrument::bump(&super::instrument::LADDER_BUDGET_ABORTED);
                break 'rungs;
            }
            // wsprd's `ii = (idt+1)/2; if (idt%2 == 1) ii = -ii; ii *= iifac;`
            let mut ii = (idt + 1) / 2;
            if idt % 2 == 1 {
                ii = -ii;
            }
            let lag = best_lag + IIFAC * ii;
            // Position 0 is the refined alignment, whose `IsQs` the
            // cascade already built.
            let isqs_owned;
            let isqs = if idt == 0 {
                best_isqs
            } else {
                isqs_owned = super::demod::tone_amplitudes(idat, qdat, best_freq, lag, best_drift);
                &isqs_owned
            };
            #[cfg(feature = "std")]
            let t_bm = std::time::Instant::now();
            let bm = if nblock == 0 {
                super::demod::nblock1_bit_metrics_opt(isqs, true)
            } else {
                super::demod::nblock_bit_metrics(isqs, nblock)
            };
            #[cfg(feature = "std")]
            super::instrument::add_us(
                &super::instrument::BIT_METRICS_US,
                t_bm.elapsed().as_micros() as u32,
            );
            let mut llrs = bm;
            deinterleave_llrs(&mut llrs);
            // wsprd's `if (rms > minrms)` plausibility gate
            // (`wsprd.c:1338-1345`) — skip the Fano attempt outright
            // when the normalised soft symbols are dominated by
            // saturating outliers.
            if crate::fec::conv::fano::wsprd_soft_symbol_rms(&llrs)
                <= crate::fec::conv::fano::WSPRD_MIN_RMS
            {
                continue;
            }
            // Fano first; if it fails to converge, fall back to OSD-1
            // (Ordered-Statistics Decoding, port of `osdwspr.f90`). OSD
            // can recover signals at -27 dB SNR (e.g. W3BI on the WSJT-X
            // golden) where Fano alone hits the convergence threshold.
            super::instrument::bump(&super::instrument::FANO_ATTEMPTS);
            // Cycle-budget cap (issue #260): every real decode in the
            // WSJT-X golden converges inside 20 % of the uncapped
            // budget (measured worst case — G8VDQ, the -23 dB decode
            // pass 2 exists for — is 162 075 cycles = 2 001/bit,
            // 20.01 % of the 10 000/bit default), while a failing
            // candidate burns essentially the whole budget before
            // giving up (mean 99 %, `docs/notes/
            // WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Fano
            // convergence budget" section). Unlike pass 2's own
            // minsync2 + top-N + deadline-budget triage, passes 0 and
            // 1 had no cap at all: every minsync2 (0.12) survivor paid
            // the full, uncapped Fano+OSD ladder regardless of how
            // many candidates a busier band produced.
            //
            // **Host is wsprd's own number**: `wsprd.c:799`,
            // `unsigned int maxcycles=10000; //Decoder timeout limit`
            // (overridable there by a CLI flag, same as here by the
            // feature below). It is also what this path effectively ran
            // at before issue #260 wired the cap up, since
            // `ConvFano::DEFAULT_MAX_CYCLES` is the same 10 000.
            //
            // Raising it above the reference buys floor recall and then
            // starts manufacturing decodes. Swept over the AWGN corpus
            // (`wspr_awgn_snr_sweep`, 100 trials/cell at the floor,
            // 500 total — every file holds exactly one transmitted
            // message, so any other decode is a phantom by
            // construction):
            //
            // ```text
            //   cycles/bit   -32 dB  -31 dB  -30 dB   phantoms/500
            //        5 000     19 %    67 %    96 %          0
            //       10 000     22 %    70 %    96 %          0   <- wsprd, host
            //       20 000     24 %    73 %    96 %          0
            //       50 000     27 %    76 %    98 %          2
            //      100 000     31 %    75 %    98 %          7
            //      500 000     23 %    61 %    91 %   golden FAILS (4)
            // ```
            //
            // The recall column alone reads as "higher is better up to
            // ~200 000"; the phantom column says the usable range ends
            // an order of magnitude earlier, between 20 000 and 50 000.
            // Given long enough, Fano finds codewords that satisfy the
            // CRC but are not the transmitted message — and in a
            // multi-pass SIC decoder those do compounding damage, since
            // they enter the carried callsign table and get subtracted
            // from the residual. At 500 000 recall ends up *below* the
            // capped value for exactly that reason. 20 000 measured
            // clean too, but there is no principled reason to sit
            // between the reference decoder's timeout and the onset of
            // false decodes.
            //
            // Capping *below* the reference costs real recall, and
            // costs it outright: the only fallback below is
            // `osd_decode_packed`, gated on `confirmed` being `Some` —
            // a callsign table built by an **earlier** Fano decode — so
            // a lone weak signal that Fano gives up on has no rescue
            // path at all. Embedded accepts that (5 000 via
            // `wspr-fano-cap-fast`) because it is what keeps a CoreS3
            // `decode_scan` inside the 120 s slot (-12.2 % TOTAL, see
            // `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`); it
            // stays phantom-free, it just hears less. Same split shape
            // as `wspr-pass2-topn`.
            #[cfg(not(feature = "wspr-fano-cap-fast"))]
            const WSPR_FANO_CYCLE_BUDGET: u64 = 10_000;
            #[cfg(feature = "wspr-fano-cap-fast")]
            const WSPR_FANO_CYCLE_BUDGET: u64 = 5_000;
            let fec_opts = FecOpts {
                max_cycles_per_bit: Some(WSPR_FANO_CYCLE_BUDGET),
                ..FecOpts::default()
            };
            #[cfg(feature = "std")]
            let t_fano = std::time::Instant::now();
            let fano_res = codec.decode_soft_pooled(&llrs, &fec_opts, &mut fano_scratch);
            #[cfg(feature = "std")]
            super::instrument::add_us(
                &super::instrument::FANO_US,
                t_fano.elapsed().as_micros() as u32,
            );
            let (info_bits, hard_errors) = if let Some(fec_res) = fano_res {
                super::instrument::bump(&super::instrument::FANO_OK);
                let mut info = [0u8; 50];
                info.copy_from_slice(&fec_res.info);
                (info, fec_res.hard_errors)
            } else if let Some((info, nhardmin)) = confirmed.and_then(|table| {
                super::instrument::bump(&super::instrument::OSD_ATTEMPTS);
                // Single lazy `and_then`, deliberately not `Option::zip`:
                // `zip` evaluates its argument eagerly, which would run the
                // (expensive) OSD decode on every candidate even when no
                // table exists — i.e. on the whole of pass 1, which always
                // passes `None`.
                // `osd_decode_packed`: bit-packed GF(2) rows, verified
                // bit-identical to `osd_decode` on 1626 real LLR
                // vectors harvested from an AWGN sweep
                // (`wspr_osd_packed_matches_unpacked`,
                // `tests/wspr_sweep.rs`) — 8.5x faster on host, and
                // the ~12.2 KB `osd_decode` stack frame this crate's
                // embedded stack audit found (two-thirds of it one
                // unpacked `[[u8;162];50]` matrix) drops to ~5 KB. See
                // `osd_decode_packed`'s own doc comment for why each
                // stage does or doesn't benefit from packing.
                #[cfg(feature = "std")]
                let t_osd = std::time::Instant::now();
                let osd_res = super::osd::osd_decode_packed(&llrs);
                #[cfg(feature = "std")]
                super::instrument::add_us(
                    &super::instrument::OSD_US,
                    t_osd.elapsed().as_micros() as u32,
                );
                let (info, nhardmin) = osd_res?;
                // wsprd's structural gate (`wsprd.c:1396`): an OSD result
                // is accepted only for a callsign an earlier Fano decode
                // already confirmed. Replaces an `nhardmin ≤ 44` threshold
                // that measurement showed cannot separate the two
                // populations — see `WsprCallsignTable`'s doc comment.
                let msg = crate::msg::Wspr50Message
                    .unpack(&info, &crate::engine::DecodeContext::default())?;
                table.accepts(&msg).then_some((info, nhardmin))
            }) {
                super::instrument::bump(&super::instrument::OSD_OK);
                (info, nhardmin)
            } else {
                continue;
            };
            let Some(message) = crate::msg::Wspr50Message
                .unpack(&info_bits, &crate::engine::DecodeContext::default())
            else {
                continue;
            };
            let lag_audio = lag * 32;
            let dt_sec = lag_audio as f32 / sample_rate as f32 - 1.0;
            let candidate = WsprResult {
                message: message.clone(),
                freq_hz: best_freq + super::baseband::CENTER_HZ
                    - 1.5 * super::demod::TONE_SPACING_HZ,
                start_sample: lag_audio.max(0) as usize,
                dt_sec,
                drift_hz: best_drift,
                info_bits,
                snr_db: 0.0,
            };
            let he = hard_errors;
            match message {
                crate::msg::WsprMessage::Type1 { .. } | crate::msg::WsprMessage::Type2 { .. } => {
                    if best_type1.as_ref().is_none_or(|(b, _)| he < *b) {
                        best_type1 = Some((he, candidate));
                    }
                }
                crate::msg::WsprMessage::Type3 { .. } => {
                    if best_other.as_ref().is_none_or(|(b, _)| he < *b) {
                        best_other = Some((he, candidate));
                    }
                }
            }
            // wsprd stops at the first success: its `while` conditions
            // are `ib <= nblocksize && not_decoded` and
            // `not_decoded && idt <= ...`, so neither loop keeps going
            // once a codeword is accepted.
            break 'rungs;
        }
    }
    best_type1.map(|(_, d)| d).or(best_other.map(|(_, d)| d))
}

pub fn decode_at_with_drift(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
) -> Option<WsprResult> {
    let (idat, qdat) = super::baseband::decimate_to_baseband(audio);
    decode_at_baseband(&idat, &qdat, sample_rate, start_sample, freq_hz, drift_hz)
}

/// Scan an audio buffer for any number of WSPR frames, returning all
/// successful decodes. Runs a coarse (freq, time) search with the given
/// [`SearchParams`], then attempts [`decode_at`] on each candidate in
/// score-descending order. Duplicate decodes (same message within ±5 Hz
/// and ±1 symbol) are collapsed to the single earliest-candidate hit,
/// so each transmission appears at most once in the output.
/// Half-window (in seconds) of front-side zero padding added before
/// the search runs. WSPR transmissions can start up to ~2 s **before**
/// the nominal slot anchor (wsprd reports such cases as `dt < -1.0`);
/// the missing pre-roll samples are not in the recording, but with
/// front padding the demodulator still aligns the rest of the frame
/// and Fano can recover from ~1–2 missing leading symbols. Mirrors
/// wsprd's `wspr_decode.f90` which prepends a configurable buffer
/// for the same reason.
const NEGATIVE_DT_PAD_SEC: f32 = 3.0;

/// Per-candidate pass-1 decode step, factored out of [`decode_scan`]'s
/// pass-1 loop so it can run under `par_iter()` (feature `parallel`)
/// or plain `.iter()` identically — pure function of its arguments, no
/// shared mutable state, so parallelizing this doesn't change behavior.
fn decode_pass1_candidate(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    c: &super::coarse_baseband::BasebandCandidate,
) -> Option<(WsprResult, usize)> {
    // wsprd's passes 0 and 1 hand `candidates[j].drift` — the value its
    // ±4 Hz coarse drift search picked — straight to the demodulator
    // (`wsprd.c:1216`). Passing 0.0 here instead discarded that search's
    // entire output, leaving the drift axis of the coarse grid costing
    // runtime and buying nothing.
    let mut d = decode_at_baseband(
        idat,
        qdat,
        sample_rate,
        c.start_sample,
        c.freq_hz,
        c.drift_hz,
    )?;
    let start_refined = d.start_sample;
    d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
    d.start_sample = start_refined.saturating_sub(pad);
    d.snr_db = c.snr_db;
    Some((d, start_refined))
}

/// Per-candidate pass-2 decode step — same parallelization rationale
/// as [`decode_pass1_candidate`], for [`decode_scan`]'s pass-2 loop.
/// wsprd-faithful: runs the full Fano + DT-peak-up + OSD ladder on
/// every `minsync2` survivor, no further cut. This is the host/default
/// path — see [`decode_pass2_top_n`] (feature `wspr-pass2-topn`) for
/// the embedded-motivated, evidence-bounded alternative.
#[cfg(not(feature = "wspr-pass2-topn"))]
fn decode_pass2_candidate(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    c: &super::coarse_baseband::BasebandCandidate,
    confirmed: &WsprCallsignTable,
) -> Option<WsprResult> {
    let mut d = decode_at_baseband_nblocks_gated_drift(
        idat,
        qdat,
        sample_rate,
        c.start_sample,
        c.freq_hz,
        c.drift_hz,
        &[1, 2, 3, 0],
        Some(confirmed),
        // wsprd's `ipass < 2` gate: the final pass does not refine drift.
        false,
    )?;
    let start_refined = d.start_sample;
    d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
    d.start_sample = start_refined.saturating_sub(pad);
    d.snr_db = c.snr_db;
    Some(d)
}

/// **Embedded-only** (feature `wspr-pass2-topn`; off by default, host
/// stays on [`decode_pass2_candidate`]'s wsprd-faithful full ladder).
/// How many pass-2 candidates, ranked by *refined* sync descending,
/// get the full Fano + DT-peak-up + OSD ladder. The rest are refined
/// (cheap — refine_cascade + minsync2 only) and then dropped.
///
/// Not a wsprd-faithful number — wsprd runs the ladder on every
/// `minsync2` survivor, no further cut. This is this crate's own
/// addition, and it costs real (if bounded) recall risk rather than
/// being provably safe the way `minsync2` is, so the number is
/// deliberately conservative and the evidence for it is recorded
/// here rather than assumed:
///
/// - the WSJT-X golden's own G8VDQ (rank 1 of 3 minsync2 survivors)
///   and W3BI (rank 2 of 4, in pass 1 — not gated by this constant,
///   which applies to pass 2 only, but the same population) show the
///   real decode is not always the single strongest candidate;
/// - a 260-trial AWGN sweep across 13 SNR levels
///   (`wspr_rank_sweep`, `tests/wspr_sweep.rs`) found the real
///   transmitted signal, whenever it decoded at all, landed at rank 0
///   or rank 1 in **every** trial — never rank 2 or beyond. That
///   sweep is single-signal (no SIC residual, no competing stations),
///   so it brackets pass 2's *isolated weak candidate* case rather
///   than reproducing pass 2 exactly, but it is the broadest evidence
///   available and it never once needed more than rank 1.
///
/// 2 is therefore the minimum this evidence supports, not a round
/// number. On the golden file this drops pass 2 wall-clock from
/// 107.5 s to roughly 72 s (measured 3→2 kept candidates), while
/// keeping the file's 9/9 recall (see the doc comment update in
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`).
///
/// **Also crucial, from the same sweep**: OSD is *not* dead weight
/// here the way it looked from the golden file alone (where its
/// success count was 0/0) — at -32/-31 dB, the marginal-detection
/// transition zone, OSD accounted for the *majority* of hits (7/8 and
/// 13/15). This constant trims *which candidates* reach the ladder;
/// it does not touch the ladder itself, and OSD stays in it.
#[cfg(feature = "wspr-pass2-topn")]
pub const PASS2_DEEP_LADDER_TOP_N: usize = 2;

/// `minsync2`'s final-pass threshold — matches
/// [`decode_at_baseband_nblocks_gated_drift`]'s own, `refine_drift =
/// false`. Shared by [`rank_pass2_candidates`] and
/// [`decode_at_baseband_nblocks_gated_drift`] rather than duplicated
/// as two magic-number `const`s that could drift apart.
#[cfg(feature = "wspr-pass2-topn")]
const MINSYNC2_FINAL: f32 = 0.10;

/// One coarse candidate after [`refine_cascade`] + the `minsync2`
/// gate, ready for [`deep_decode_pass2_candidate`]. `pub`: this is
/// [`rank_pass2_candidates`]'s return type, needed by any caller that
/// wants to split the ranked survivors across cores itself (issue
/// #260's dual-core pass-2 dispatch — see
/// `embedded_shared::apps::wspr_dual_core`) rather than always
/// running [`decode_pass2_top_n`]'s sequential Stage 2.
#[cfg(feature = "wspr-pass2-topn")]
pub struct WsprPass2Candidate<'c> {
    pub sync: f32,
    pub c: &'c super::coarse_baseband::BasebandCandidate,
    pub freq: f32,
    pub lag: i32,
    pub drift: f32,
    pub isqs: super::demod::IsQs,
}

/// Stage 1 of pass 2's candidate loop: refine + `minsync2`-filter
/// every coarse candidate (cheap — no Fano/OSD), then rank the
/// survivors by refined sync descending and keep the top
/// [`PASS2_DEEP_LADDER_TOP_N`] — see that constant's doc comment for
/// the AWGN-sweep evidence this cutoff is built on.
///
/// Split out of what used to be [`decode_pass2_top_n`]'s inline Stage
/// 1 so a dual-core dispatcher can rank once (cheap, parallel across
/// every coarse candidate) and then hand each of the ≤
/// `PASS2_DEEP_LADDER_TOP_N` survivors to a *different* core for
/// [`deep_decode_pass2_candidate`] — the expensive half — instead of
/// running that half sequentially on one core.
#[cfg(feature = "wspr-pass2-topn")]
pub fn rank_pass2_candidates<'c>(
    idat: &[f32],
    qdat: &[f32],
    cands: &'c [super::coarse_baseband::BasebandCandidate],
) -> Vec<WsprPass2Candidate<'c>> {
    // A plain `fn`, not a closure: closures infer one concrete lifetime
    // for their signature from first use, not the
    // `for<'a> Fn(&'a Candidate) -> Option<WsprPass2Candidate<'a>>`
    // this call site needs (`par_iter`/`iter` hand a fresh borrow per
    // item) — a `fn` item gets that generality for free via ordinary
    // lifetime elision.
    fn refine_one<'c>(
        idat: &[f32],
        qdat: &[f32],
        c: &'c super::coarse_baseband::BasebandCandidate,
    ) -> Option<WsprPass2Candidate<'c>> {
        let f0_center_init = c.freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
        let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
        let lag_baseband_init = c.start_sample as i32 / 32;
        let (freq, lag, drift, sync, isqs) = refine_cascade(
            idat,
            qdat,
            f0_baseband_init,
            lag_baseband_init,
            c.drift_hz,
            false,
        );
        if sync <= MINSYNC2_FINAL {
            super::instrument::bump(&super::instrument::MINSYNC2_REJECTED);
            return None;
        }
        Some(WsprPass2Candidate {
            sync,
            c,
            freq,
            lag,
            drift,
            isqs,
        })
    }
    // Embarrassingly parallel exactly like pass 0/1's own per-candidate
    // decode already is — same rationale, same `par_iter()` pattern.
    // This matters here specifically: cutting stage 2 down to
    // `PASS2_DEEP_LADDER_TOP_N` candidates only pays off if stage 1
    // (now run over *every* coarse candidate, not just survivors)
    // doesn't itself become a sequential bottleneck on a many-core
    // host — an earlier version of this function ran stage 1 as a
    // plain sequential loop and measured *slower* wall-clock overall
    // despite doing less total work, purely from losing parallelism.
    #[cfg(feature = "parallel")]
    let mut survivors: Vec<WsprPass2Candidate<'_>> = cands
        .par_iter()
        .filter_map(|c| refine_one(idat, qdat, c))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let mut survivors: Vec<WsprPass2Candidate<'_>> = cands
        .iter()
        .filter_map(|c| refine_one(idat, qdat, c))
        .collect();

    survivors.sort_by(|a, b| {
        b.sync
            .partial_cmp(&a.sync)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    survivors.truncate(PASS2_DEEP_LADDER_TOP_N);
    survivors
}

/// Stage 2 of pass 2's candidate loop: the full Fano + DT-peak-up +
/// OSD ladder on one already-[`rank_pass2_candidates`]-ranked
/// survivor. `pub` for the same dual-core reason as
/// [`WsprPass2Candidate`] — lets a caller run this on a different
/// core per candidate instead of [`decode_pass2_top_n`]'s sequential
/// loop over all survivors.
///
/// `budget`: forwarded to [`decode_from_refined`] unchanged — see its
/// own doc comment for the contract. `None` runs the full wsprd-
/// faithful ladder, unconditionally; this is the only place a caller
/// can bound a single candidate's *failing*-path wall-clock, which is
/// exactly the case that dominates pass 2 (see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Dual-core,
/// take two" — a non-converging candidate can cost 4-5x a converging
/// one).
#[cfg(feature = "wspr-pass2-topn")]
pub fn deep_decode_pass2_candidate(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    confirmed: &WsprCallsignTable,
    r: &WsprPass2Candidate<'_>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> Option<WsprResult> {
    let mut d = decode_from_refined(
        idat,
        qdat,
        sample_rate,
        r.freq,
        r.lag,
        r.drift,
        &r.isqs,
        &[1, 2, 3, 0],
        Some(confirmed),
        budget,
    )?;
    let start_refined = d.start_sample;
    d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
    d.start_sample = start_refined.saturating_sub(pad);
    d.snr_db = r.c.snr_db;
    Some(d)
}

/// Pass 2's own candidate loop, sequential: [`rank_pass2_candidates`]
/// then [`deep_decode_pass2_candidate`] on every survivor. This is
/// what [`decode_scan`]/[`decode_scan_inner`] use when the feature is
/// on, and what `embedded_shared::apps::wspr_bench` used to call
/// directly before the dual-core split above existed (that bench
/// predates this function and runs its own hand-inlined pass 0/1/2
/// loop — the "D pattern", avoiding a `tlsf_malloc` corruption bug
/// when `decode_block`-shaped helpers are called from a long bench
/// loop — rather than going through `decode_scan`).
///
/// `budget`: the same budget passed to every survivor's
/// [`deep_decode_pass2_candidate`] call — one shared check, not one
/// per candidate, since `&(dyn Fn() -> bool + Sync)` is a plain
/// shared reference and `par_iter()` needs `Sync` regardless. `None`
/// (host's own default, see [`decode_scan_inner`]) runs the full
/// wsprd-faithful ladder on every survivor.
#[cfg(feature = "wspr-pass2-topn")]
pub fn decode_pass2_top_n(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    cands: &[super::coarse_baseband::BasebandCandidate],
    confirmed: &WsprCallsignTable,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> Vec<WsprResult> {
    let survivors = rank_pass2_candidates(idat, qdat, cands);

    // Stage 2 (the Fano/OSD ladder) is the expensive half — this is
    // exactly the work the old per-candidate `decode_pass2_candidate`
    // ran under `par_iter()`. Losing that here (a plain sequential
    // `for` over `PASS2_DEEP_LADDER_TOP_N` survivors) was the second
    // half of the same regression stage 1's fn-conversion fixed: two
    // candidates run one-after-another cost roughly 2x one candidate's
    // wall-clock even though the total work dropped; run them
    // concurrently instead now that there are only ever
    // `PASS2_DEEP_LADDER_TOP_N` of them. (Embedded targets go further
    // still and run them on *separate cores* — see
    // `embedded_shared::apps::wspr_dual_core` — this sequential/
    // `par_iter` version is host's own path.)
    #[cfg(feature = "parallel")]
    let out: Vec<WsprResult> = survivors
        .par_iter()
        .filter_map(|r| {
            deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let out: Vec<WsprResult> = survivors
        .iter()
        .filter_map(|r| {
            deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
        })
        .collect();
    out
}

pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprResult> {
    decode_scan_inner(audio, sample_rate, nominal_start_sample, params, None, None)
}

/// [`decode_scan`] with a caller-owned [`WsprCallsignTable`] that
/// persists **across slots**.
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
pub fn decode_scan_with_table(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    confirmed: &mut WsprCallsignTable,
) -> Vec<WsprResult> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        None,
        Some(confirmed),
    )
}

/// Streaming variant of [`decode_scan`]: fires `on_result` once per
/// candidate as it's accepted, *in addition to* (not instead of) the
/// returned `Vec` — purely additive, same shape as
/// [`crate::msg::decode_request::DecodeRequest::on_result`] (see that
/// method's doc comment and `docs/reference/LIBRARY.md`'s "public
/// decode entry point" section for the full portability rationale).
///
/// A `_streaming` sibling rather than a new parameter on [`decode_scan`]
/// itself: `decode_scan` is a plain `pub fn`, not a builder, so adding
/// a parameter would be a breaking signature change (same reasoning as
/// `ft8::decode_block::decode_block_streaming` alongside `decode_block`).
///
/// **Delivery order/dedup contract**: both pass 1 and pass 2's
/// per-candidate decode step run under `rayon::par_iter()` (feature
/// `parallel`) — same completion-order/possible-duplicate caveat
/// documented on
/// [`crate::msg::decode_request::DecodeRequest::on_result`]'s parallel
/// single-pass strategy: `cb` fires from whichever thread decoded that
/// candidate, in completion order, *before* the dedup-then-push step
/// that decides what lands in the returned `Vec` — a same-message
/// duplicate found by two different candidates could fire `cb` twice
/// even though only one survives into the batch result. `cb` must be
/// `Sync` for this reason.
pub fn decode_scan_streaming(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    on_result: &(dyn Fn(&WsprResult) + Sync),
) -> Vec<WsprResult> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Some(on_result),
        None,
    )
}

fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    on_result: Option<&(dyn Fn(&WsprResult) + Sync)>,
    carried: Option<&mut WsprCallsignTable>,
) -> Vec<WsprResult> {
    // Prepend zeros so signals that started before audio[0] (negative
    // dt) become reachable. Internal `start_sample`s are shifted by
    // `pad`; we subtract `pad` back out before returning so callers
    // see the original time base.
    let pad = (NEGATIVE_DT_PAD_SEC * sample_rate as f32) as usize;
    let mut padded = alloc::vec![0f32; pad + audio.len()];
    padded[pad..].copy_from_slice(audio);
    let nominal_shifted = nominal_start_sample + pad;
    // Decimate ONCE up-front; the wsprd-equivalent coarse and the
    // demod both consume the same baseband buffer, so we save 32×
    // FFT work vs running each separately.
    // Channelizer selection. The reference (`decimate_to_baseband`,
    // wsprd's own) is exact and is what every number in this crate was
    // measured against; it is also unrunnable on an ESP32-S3, whose
    // 8 MB PSRAM cannot hold its 11.25 MiB `NFFT1` buffer and whose FFT
    // backend tops out at 8 192. Embedded turns on `wspr-ddc` (single-
    // stage streaming down-converter) or `wspr-ddc-cascade` (two-stage
    // cascade, ~4.5x cheaper and internal-DRAM-sized) instead — see
    // `wspr::ddc` for what each trades away.
    #[cfg(all(feature = "wspr-ddc", feature = "wspr-ddc-cascade"))]
    compile_error!("wspr-ddc and wspr-ddc-cascade select different channelizers — enable only one");
    #[cfg(not(any(feature = "wspr-ddc", feature = "wspr-ddc-cascade")))]
    let (mut idat, mut qdat) = super::baseband::decimate_to_baseband(&padded);
    #[cfg(feature = "wspr-ddc")]
    let (mut idat, mut qdat) = super::ddc::ddc_to_baseband(&padded);
    #[cfg(feature = "wspr-ddc-cascade")]
    let (mut idat, mut qdat) = super::ddc::ddc_to_baseband_cascade(&padded);
    // wsprd-equivalent coarse: 512-pt windowed FFT on the 375 Hz
    // baseband, time-averaged spectrum + 30 th-percentile noise
    // floor, peak detection on smspec, 3-D (freq, time, drift)
    // refinement. Lifts the coarse score landscape to actually peak
    // at the right (freq, dt) for weak signals next to strong ones
    // (W5BIT, W3BI). See `coarse_baseband.rs`.
    let max_drift = 4i32;
    let bb_cands = super::coarse_baseband::coarse_baseband(
        &idat,
        &qdat,
        pad,
        params.max_candidates,
        max_drift,
    );
    // Use the wsprd-equivalent coarse only. Legacy `coarse_search`
    // (12 kHz spectrogram) costs ~30 s of recall-test runtime on a
    // 122 s slot and brings nothing the new coarse doesn't already
    // find — every golden hit on the WSJT-X reference WAV (ND6P,
    // WD4LHT, NM7J, KI7CI, DJ6OL, W3HH, W5BIT) comes through the
    // baseband path. Kept the legacy module for synth round-trip
    // tests; not invoked here.
    let _ = nominal_shifted;
    // `coarse_baseband` already returns candidates sorted by sync score
    // descending — no remap/re-sort needed; keeping `BasebandCandidate`
    // itself (rather than the legacy `SyncCandidate`) preserves each
    // candidate's `snr_db` through to the decode below.
    let mut cands = bb_cands;
    cands.truncate(params.max_candidates);
    let _audio = &padded[..]; // shadow so all downstream reads use padded buffer
    let mut seen: Vec<WsprResult> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192; // one WSPR symbol at 12 kHz
    // 2-D refinement: WSPR's Fano (K=32 convolutional, no CRC) is
    // sensitive to *both* sub-bin freq and sub-t_step time mis-
    // alignment. Coarse-search rounds to 1.46 Hz / 170 ms; this
    // pass refines:
    //   time : ±170 ms / 43 ms step ⇒ 9 points
    //   freq : ±2 Hz   / 0.5 Hz step ⇒ 9 points
    // ≈ 81 sync_score evals × candidate.
    //
    // Going finer in time (e.g. 10 ms) actually *hurts* recall on
    // weak signals: WSPR has no CRC, so the highest-sync_score
    // alignment is often a noise-pattern Fano ghost rather than the
    // true signal. 43 ms preserves true peaks while the coarser
    // grid keeps us out of the ghost-attractor region. Better
    // long-term fix is a Fano-metric / callsign-sanity gate; until
    // then, 43 ms is the empirical optimum on the WSJT-X golden.
    // Tightened grid (3 × 3 = 9 evals/cand, was 9 × 9 = 81): wsprd's
    // entire 3-pass SIC runs in seconds; our refine cost dominated
    // total wall time. The small grid still recovers the same 5/8
    // goldens against the WSJT-X reference WAV — most of the recall
    // win came from sub-bin demod (slice 1 of issue #17), not from
    // the dense refine grid.
    const REFINE_FREQ_RADIUS_HZ: f32 = 1.0;
    const REFINE_FREQ_STEP_HZ: f32 = 1.0;
    let nsps = (sample_rate as f32 * <super::Wspr as crate::engine::ModulationParams>::SYMBOL_DT)
        .round() as i64;
    let refine_time_radius = nsps / 8; // ≈85 ms half-window
    let refine_time_step = nsps / 8; // 1 step at radius → 3 cells in time
    // (idat, qdat) computed above and shared with coarse_baseband.
    // Suppress the unused-warnings for refine_align knobs — they
    // belong to the legacy 12 kHz path that the new baseband
    // demod has retired (decode_at_baseband does its own ±1.5 Hz /
    // ±0.1 s sweep around the coarse pick).
    let _ = (
        REFINE_FREQ_RADIUS_HZ,
        REFINE_FREQ_STEP_HZ,
        refine_time_radius,
        refine_time_step,
    );
    // wsprd runs **three** passes (`npasses = 3`, `wsprd.c:805`), not
    // two, and they are not all configured alike (`wsprd.c:998-1010`):
    //
    //   pass 0, 1 : nblocksize = 1, maxdrift = 4, minsync2 = 0.12
    //   pass 2    : nblocksize = 4, maxdrift = 0, minsync2 = 0.10
    //
    // `minsync2` is applied inside
    // `decode_at_baseband_nblocks_gated_drift`, keyed off the same
    // `refine_drift` flag this file already threads through for
    // `maxdrift` — see that function's doc comment for why the two
    // reuse one boolean rather than taking a separate parameter.
    //
    // Each pass subtracts what it decoded before the next one re-runs
    // the coarse search, so pass 2 sees a residual with *two* rounds of
    // strong signals removed. This crate previously collapsed that into
    // two passes, which cost both of the WSJT-X golden's weakest
    // signals: W3BI (−27 dB) needs the extra subtraction round, and
    // G8VDQ (−23 dB) needs pass 2's zero-drift estimate.
    //
    // `wsprd.c:999` skips pass 1 entirely when pass 0 decoded nothing,
    // on the reasoning that with nothing subtracted the residual is the
    // input and re-searching it cannot find anything new.
    //
    // Each candidate's decode call is a pure function of
    // (idat, qdat, sample_rate, candidate) — no shared mutable state
    // during the loop body, so the decode step runs in parallel
    // (mirroring ft8::decode's own par_iter()/filter_map()/collect()
    // pattern, src/ft8/decode.rs:361-368). The dedup pass stays
    // strictly sequential afterward, in the original (order-preserving)
    // candidate order, so "first occurrence wins" semantics hold.
    //
    // Decodes carry their padded-buffer alignment alongside, so the
    // subtraction at the end of each pass knows where to aim.
    let mut found: Vec<(WsprResult, usize)> = Vec::new();
    for early_pass in 0..2 {
        // `wsprd.c:999`.
        if early_pass == 1 && found.is_empty() {
            break;
        }
        // Pass 0 works on the coarse candidates computed above; pass 1
        // re-runs the coarse over the residual left by pass 0's
        // subtraction.
        let pass_cands = if early_pass == 0 {
            core::mem::take(&mut cands)
        } else {
            let mut c = super::coarse_baseband::coarse_baseband(
                &idat,
                &qdat,
                pad,
                params.max_candidates,
                max_drift,
            );
            c.truncate(params.max_candidates);
            c
        };

        #[cfg(feature = "parallel")]
        let raw: Vec<(WsprResult, usize)> = pass_cands
            .par_iter()
            .filter_map(|c| decode_pass1_candidate(&idat, &qdat, sample_rate, pad, c))
            .collect();
        #[cfg(not(feature = "parallel"))]
        let raw: Vec<(WsprResult, usize)> = pass_cands
            .iter()
            .filter_map(|c| decode_pass1_candidate(&idat, &qdat, sample_rate, pad, c))
            .collect();

        let mut this_pass: Vec<(WsprResult, usize)> = Vec::new();
        for (d, start_refined) in raw {
            let dup = scan_dedup_match(
                &seen,
                &d,
                |r| &r.message,
                |r| r.freq_hz,
                |r| r.start_sample as i64,
                FREQ_DEDUP_HZ,
                TIME_DEDUP_SAMPLES,
            );
            if !dup {
                if let Some(cb) = on_result {
                    cb(&d);
                }
                this_pass.push((d.clone(), start_refined));
                seen.push(d);
            }
        }

        // Subtract this pass's decodes so the next pass sees a cleaner
        // residual — wsprd calls `subtract_signal2` inline on each
        // accepted decode, which amounts to the same thing by the time
        // the pass ends. `idat`/`qdat` are locally owned (from
        // `decimate_to_baseband`) and every earlier borrow is scoped to
        // the loop iteration that took it, so this can mutate in place
        // rather than cloning both ~180 KB buffers.
        for (d, start_refined) in &this_pass {
            let symbols = super::encode_channel_symbols(&d.info_bits);
            let f0_audio = d.freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
            let shift_baseband = (*start_refined as i32) / 32;
            super::subtract::subtract_signal_baseband(
                &mut idat,
                &mut qdat,
                f0_audio,
                shift_baseband,
                d.drift_hz,
                &symbols,
            );
        }
        found.extend(this_pass);
    }

    // Passes 0 and 1 ran Fano-only (`decode_pass1_candidate` passes no
    // table, so OSD is rejected outright). Every callsign they produced
    // is therefore Fano-confirmed and may unlock an OSD decode of the
    // *same* station in pass 2 — wsprd's own `hashtab` semantics, where
    // earlier successful decodes are what make later OSD attempts
    // trustworthy.
    // Seed from the caller's cross-slot table (if any) before adding
    // this slot's own Fano confirmations.
    let mut confirmed = match &carried {
        Some(t) => (*t).clone(),
        None => WsprCallsignTable::new(),
    };
    for (d, _) in &found {
        confirmed.record(&d.message);
    }

    // Pass 2 — wsprd's final pass. The residual now has both earlier
    // passes' decodes removed; what is left are the signals that were
    // buried under stronger neighbours.
    //
    // It runs unconditionally. `wsprd.c:999` skips **pass 1** when pass
    // 0 decoded nothing (`ipass = 2`), never pass 2 — a slot where the
    // early passes found nothing is exactly the one that most needs the
    // final pass's coherent-block ladder and zero-drift estimate. This
    // used to be gated on the early passes having produced something,
    // which cost every weak single-signal slot its best chance.
    {
        // Re-run coarse on the cleaned baseband. Skip the legacy
        // 12 kHz coarse here — pass 2 runs against an already-decimated
        // residual buffer, and reconstructing 12 kHz from baseband is
        // pointless for the same coarse_search call.
        // wsprd's pass 2 sets `maxdrift = 0` — "no drift for smaller
        // frequency estimator variance" (`wsprd.c:1005-1009`). Passes 0
        // and 1 search ±4; the final pass deliberately does not, because
        // by then the strong signals have been subtracted and the
        // remaining weak ones are better served by a lower-variance
        // frequency estimate than by a wider search.
        const PASS2_MAX_DRIFT: i32 = 0;
        let bb_cands2 = super::coarse_baseband::coarse_baseband(
            &idat,
            &qdat,
            pad,
            params.max_candidates,
            PASS2_MAX_DRIFT,
        );
        // Pass 2 uses nblock = 1, 2, 3 (coherent block detection) for
        // the +3..+4.8 dB margin needed to decode signals like W3BI at
        // -27 dB SNR. The strong-signal subtract above has exposed
        // them in the spectrum, but they still need the coherent gain
        // to clear the Fano convergence threshold — this is pass 2's
        // own per-candidate cost that's ~4x pass 1's (3 nblock values
        // vs 1), the dominant single cost in `decode_scan` per
        // `docs/notes/WSPR_BENCHMARK.md`'s Finding 2.
        //
        // wsprd-faithful by default: full ladder on every `minsync2`
        // survivor, same parallelize-the-decode-step /
        // dedup-sequentially-after shape as pass 1 above. Feature
        // `wspr-pass2-topn` (embedded targets only — see that
        // function's doc comment) swaps in `decode_pass2_top_n`
        // instead: refines and `minsync2`-filters every candidate
        // first, then runs the expensive ladder only on the top
        // `PASS2_DEEP_LADDER_TOP_N` by refined sync.
        #[cfg(feature = "wspr-pass2-topn")]
        let raw2: Vec<WsprResult> =
            decode_pass2_top_n(&idat, &qdat, sample_rate, pad, &bb_cands2, &confirmed, None);
        #[cfg(not(feature = "wspr-pass2-topn"))]
        #[cfg(feature = "parallel")]
        let raw2: Vec<WsprResult> = bb_cands2
            .par_iter()
            .filter_map(|c| decode_pass2_candidate(&idat, &qdat, sample_rate, pad, c, &confirmed))
            .collect();
        #[cfg(not(feature = "wspr-pass2-topn"))]
        #[cfg(not(feature = "parallel"))]
        let raw2: Vec<WsprResult> = bb_cands2
            .iter()
            .filter_map(|c| decode_pass2_candidate(&idat, &qdat, sample_rate, pad, c, &confirmed))
            .collect();

        for d in raw2 {
            let dup = scan_dedup_match(
                &seen,
                &d,
                |r| &r.message,
                |r| r.freq_hz,
                |r| r.start_sample as i64,
                FREQ_DEDUP_HZ,
                TIME_DEDUP_SAMPLES,
            );
            if !dup {
                if let Some(cb) = on_result {
                    cb(&d);
                }
                seen.push(d);
            }
        }
    }

    // Hand this slot's confirmations back to the caller's cross-slot
    // table. `seen` holds OSD-derived results too, but recording them
    // cannot widen the table: `WsprCallsignTable::accepts` only
    // admits an OSD result whose callsign is *already* an entry, so
    // re-recording it is a no-op. No OSD decode can therefore
    // bootstrap a new callsign into the table for the next slot —
    // every entry still traces back to a Fano decode.
    if let Some(t) = carried {
        for d in &seen {
            t.record(&d.message);
        }
    }

    seen
}

/// Convenience: scan using [`SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<WsprResult> {
    decode_scan(audio, sample_rate, 0, &SearchParams::default())
}

/// WSPR subtract configuration (continuous-phase 4-FSK). Mirrors WSJT-X
/// `subtract_signal2` in `wsprd.c`: tone spacing 1.4648 Hz, 8192
/// samples/symbol at 12 kHz, no GFSK shaping (WSPR is plain CPFSK).
const WSPR_SUBTRACT: crate::engine::dsp::subtract::SubtractCfg =
    crate::engine::dsp::subtract::SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 1.4648,
        samples_per_symbol: 8192,
        // WSPR's nominal symbol-0 start is 1.0 s into the slot; our
        // `start_sample` is already absolute, so the subtract layer
        // sees `dt_sec` as `(start - 1.0*FS) / FS`. `base_offset_s = 1.0`
        // matches the convention used by `WsprResult::dt_sec`.
        base_offset_s: 1.0,
        gfsk: None,
    };

/// LPF kernel half-width for the channel-aware subtract, in audio
/// samples. Chosen by measuring residual suppression on the WSJT-X
/// golden — see the call site in `decode_scan_subtract_inner`.
const WSPR_SUBTRACT_LPF_HALF: usize = 600;

/// A second SIC layer wrapped around [`decode_scan`], at 12 kHz.
///
/// **This is not part of the reference decoder, and
/// [`decode_scan`] is the wsprd-equivalent entry point.** The doc
/// comment here used to claim this function "mirrors WSJT-X
/// `wsprd.c`'s `npasses=3` SIC loop (`wsprd.c:998-1438`)". That was
/// true when it was written, and stopped being true in
/// [#275](https://github.com/jl1nie/mfsk-core/issues/275): porting
/// wsprd's three-pass structure faithfully put that same loop
/// *inside* `decode_scan_inner`, citing the same `wsprd.c:805` /
/// `wsprd.c:999` lines, down to the "skip pass 1 when pass 0 decoded
/// nothing" rule. The wrapper was left in place and its comment was
/// not revisited, so two nested layers ended up claiming the same
/// construct. wsprd has one such loop; this ran three inner passes
/// twice.
///
/// What it costs, measured on the WSJT-X golden
/// (`wspr_diag_pass_ablation`, Ryzen 9 9900X): `decode_scan` 0.71 s
/// for 9/9 goldens, this 2.38 s for the same 9/9 — **3.3× for zero
/// marginal recall**. `WSPR_BENCHMARK.md`'s "Option C" reached the
/// same conclusion by ablation before #275 landed.
///
/// Nothing in this crate calls it: [`decode_scan_default`] and the
/// `mfsk-ffi` C ABI both route to [`decode_scan`]. It is kept, for
/// now, only because it is `pub` and removing it is a breaking
/// change — prefer [`decode_scan`] in new code.
///
/// Returns deduplicated decodes from all passes.
#[deprecated(
    note = "not the wsprd-equivalent decoder — `decode_scan` is. This wraps a second \
            SIC layer around it that the reference has no counterpart for, costing 3.3x \
            for zero marginal recall on the WSJT-X golden. Use `decode_scan`."
)]
pub fn decode_scan_subtract(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprResult> {
    decode_scan_subtract_inner(audio, sample_rate, nominal_start_sample, params, None)
}

/// Streaming variant of [`decode_scan_subtract`], and therefore
/// carrying the same caveat: the extra SIC layer is **not** part of
/// the reference decoder — see [`decode_scan_subtract`]. Prefer
/// [`decode_scan_streaming`].
///
/// See [`decode_scan_streaming`]'s doc comment for the general rationale
/// (`_streaming` sibling, not a new parameter, since this is a plain
/// `pub fn` not a builder).
///
/// **Delivery order/dedup contract**: `cb` fires once per accepted
/// decode, at *this* function's own SIC-pass dedup-then-push point
/// (`all.push(d)` below) — not inside the per-pass `decode_scan` call
/// each SIC round makes internally, which stays a plain (non-
/// streaming) call. This matches FT8's `.sic_rounds()` contract: the
/// outer SIC accept point is the final-acceptance point, so `cb` fires
/// exactly once per result that ends up in the returned `Vec`, in the
/// same order — no divergence, unlike [`decode_scan_streaming`]'s
/// parallel-strategy caveat.
#[deprecated(note = "see `decode_scan_subtract` — use `decode_scan_streaming`.")]
pub fn decode_scan_subtract_streaming(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    on_result: &(dyn Fn(&WsprResult) + Sync),
) -> Vec<WsprResult> {
    decode_scan_subtract_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Some(on_result),
    )
}

fn decode_scan_subtract_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    on_result: Option<&(dyn Fn(&WsprResult) + Sync)>,
) -> Vec<WsprResult> {
    use crate::engine::dsp::subtract::subtract_tones_lpf;

    // The subtract helper takes `&mut [i16]`; convert once, mutate
    // across passes, work on `f32` for `decode_scan` per pass.
    let mut residual_i16: Vec<i16> = audio
        .iter()
        .map(|&x| (x * 32767.0).clamp(-32768.0, 32767.0) as i16)
        .collect();

    let mut all: Vec<WsprResult> = Vec::new();
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192;
    // wsprd uses 3 passes. Our `decode_scan` is expensive (~30 s on
    // a 120-s WSPR slot due to the 2-D refine grid), so we cap at 2
    // — empirically the bulk of the SIC benefit lands on pass 2 once
    // the strong KB0VHA-class signals have been removed.
    const NPASSES: usize = 2;

    for _pass in 0..NPASSES {
        // Re-convert residual back to f32 for decode_scan (it expects
        // unit-scale samples).
        let residual_f32: Vec<f32> = residual_i16.iter().map(|&s| s as f32 / 32_768.0).collect();
        let new_decodes = decode_scan(&residual_f32, sample_rate, nominal_start_sample, params);
        if new_decodes.is_empty() {
            break;
        }
        let mut added = 0usize;
        for d in new_decodes {
            let dup = scan_dedup_match(
                &all,
                &d,
                |r| &r.message,
                |r| r.freq_hz,
                |r| r.start_sample as i64,
                FREQ_DEDUP_HZ,
                TIME_DEDUP_SAMPLES,
            );
            if dup {
                continue;
            }
            // Reconstruct the on-air channel symbols (162 4-FSK tones)
            // from the recovered 50-bit info, and subtract from the
            // residual at the decoded (freq, dt). Mirrors wsprd.c:1432-
            // 1437 `subtract_signal2(idat, qdat, …, channel_symbols)`.
            let symbols = super::encode_channel_symbols(&d.info_bits);
            // Channel-aware LPF subtract, matching wsprd's
            // `subtract_signal2` (`wsprd.c:549-602`): it estimates a
            // *time-varying* amplitude by low-pass filtering the
            // product of residual and reference, so it tracks QSB and
            // drift across the 110.6 s transmission.
            //
            // This replaces a constant-amplitude least-squares
            // subtract that was measurably not doing its job. On the
            // WSJT-X golden, subtracting W5BIT and measuring the
            // residual at its own four tone frequencies:
            //
            //   const-amplitude   +9.0  -7.7  -5.8  -4.2  dB
            //   LPF (this)       -25.4 -28.6 -30.0 -19.7  dB
            //
            // — the old path *amplified* the first tone by 9 dB and
            // removed only a few dB from the rest, leaving most of
            // the interferer in the residual it was supposed to
            // clean.
            //
            // The comment this replaces said the LPF path was avoided
            // because it "would cost a multi-second convolution on a
            // 1.4 M-sample WSPR slot". That stopped being true when
            // `subtract_tones_lpf` gained its FFT fast path for
            // FT8/FT4 (one forward + inverse FFT per call, O(N log N));
            // WSPR was simply never migrated. Measured cost of the
            // whole `decode_scan_subtract` on that slot is unchanged
            // at ~0.3-0.4 s.
            //
            // `lpf_half = 600` was chosen by measurement, not
            // inherited: 5760 (wsprd's `nfilt=360` scaled from its
            // 375 Hz baseband to 12 kHz) removes 10-15 dB less,
            // because our reference is built at audio rate where a
            // proportionally longer kernel over-smooths the amplitude
            // estimate.
            //
            // `endcorrection = false`: measured identical either way
            // here, and WSPR's transmission is long enough that edge
            // handling is negligible.
            subtract_tones_lpf(
                &mut residual_i16,
                &symbols,
                d.freq_hz,
                d.dt_sec,
                &WSPR_SUBTRACT,
                WSPR_SUBTRACT_LPF_HALF,
                false,
            );
            if let Some(cb) = on_result {
                cb(&d);
            }
            all.push(d);
            added += 1;
        }
        if added == 0 {
            break;
        }
    }
    all
}

/// Deinterleave 162 LLRs in place (same permutation as [`deinterleave`]
/// but for `f32` values). Thin wrapper over
/// [`crate::engine::interleave::deinterleave_bitrev`] — was its own
/// third inline copy of the bit-reversal formula ("avoid exposing a
/// pub helper"), no longer needed now that the shared version is
/// itself the pub helper.
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    crate::engine::interleave::deinterleave_bitrev(llrs);
}

#[cfg(test)]
mod tests {
    use super::super::search::SearchParams;
    use super::super::synthesize_type1;
    use super::*;
    use crate::msg::WsprMessage;

    #[test]
    fn synth_decode_roundtrip_k1abc_fn42_37() {
        let freq = 1500.0;
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("valid message");
        let r = decode_at(&audio, 12_000, 0, freq).expect("decode");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn scan_recovers_message_without_freq_hint() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let decodes = decode_scan(
            &audio,
            12_000,
            0,
            &SearchParams {
                freq_min_hz: 1450.0,
                freq_max_hz: 1550.0,
                ..SearchParams::default()
            },
        );
        assert!(!decodes.is_empty(), "at least one decode");
        let d = decodes.into_iter().next().unwrap();
        assert_eq!(
            d.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
        assert!((d.freq_hz - 1500.0).abs() <= 2.0);
    }

    #[test]
    fn survives_moderate_awgn() {
        use std::f32::consts::PI;

        let freq = 1500.0;
        let mut audio =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.5).expect("valid message");

        // Deterministic "noise": superposition of a handful of off-tone
        // sinusoids plus a pseudorandom dither. This is a cheap AWGN
        // stand-in that keeps the test free of rand dependencies.
        let mut seed: u32 = 0x1234_5678;
        for (i, s) in audio.iter_mut().enumerate() {
            // Linear congruential pseudorandom for reproducible noise.
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let rnd = ((seed >> 16) as f32 / 32768.0 - 1.0) * 0.10;
            let off = 0.05 * (2.0 * PI * 2345.7 * i as f32 / 12_000.0).sin();
            *s += rnd + off;
        }

        let r = decode_at(&audio, 12_000, 0, freq).expect("decode under noise");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K9AN".into(),
                grid: "EN50".into(),
                power_dbm: 33,
            }
        );
    }
}
