//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use alloc::vec::Vec;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

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
    use crate::engine::{FecOpts, MessageCodec};
    // `freq_hz` follows our tone-0 convention (matches `synthesize_audio`
    // and `coarse_search.freq_hz`); wsprd's `noncoherent_sequence_detection`
    // takes the signal CENTER, so we add 1.5·tone_spacing here and sweep
    // around that point. A wide ±4 Hz sweep is needed because real-world
    // coarse picks can land 2-4 Hz off the true centre when nearby
    // signals or sub-bin offsets perturb the score landscape.
    // wsprd-style refine→demod cascade. Replaces the previous
    // brute-force 15×7 (Δf, Δdt) sweep, which ran Fano on every cell
    // and burnt seconds of CPU per candidate. Architecture follows
    // `wsprd.c:1217-1280`: mode 0 (lag refine) → mode 1 (freq refine)
    // → mode 2 (final demod), with Fano run only at the refined
    // alignment (not per cell).
    let f0_center_init = freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
    let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
    let lag_baseband_init = start_sample as i32 / 32;
    let codec = crate::fec::ConvFano;
    // Pooled across the whole `nblocks` sweep below (up to 3 values in
    // pass 2) instead of `fano_decode` allocating a fresh `Vec<Node>`
    // scratch per call — see `fano::fano_decode_with_scratch`'s doc
    // comment for why reuse is safe here.
    let mut fano_scratch = crate::fec::conv::fano::FanoScratch::new();

    // wsprd's refine cascade (`wsprd.c:1221-1277`), in full. Each
    // stage is `sync_and_demodulate` in mode 0 (search lag, freq
    // fixed) or mode 1 (search freq, lag fixed), keeping whichever
    // cell scores highest on the sync metric:
    //
    //   1. coarse lag   ±128 baseband samples, step 64
    //   2. coarse freq  ±2 × 0.25 Hz
    //   3. drift refine drift ± 0.5 (passes 0 and 1 only)
    //   4. fine lag     ±32 baseband samples, step 16
    //   5. fine freq    ±2 × 0.05 Hz
    //
    // Stages 3-5 were missing here, and stage 2 ran at ±1.0 Hz in
    // 0.5 Hz steps rather than ±0.5 Hz in 0.25 Hz steps. That
    // combination is why handing this function the coarse stage's
    // drift estimate used to *hurt*: without stage 3 there is nothing
    // to correct a drift the ±4 Hz coarse grid got wrong, and without
    // stages 4-5 the alignment stays a coarse-grid cell away from the
    // true one.
    let eval = |f: f32, lag: i32, drift: f32| {
        let isqs = super::demod::tone_amplitudes(idat, qdat, f, lag, drift);
        let sync = super::demod::sync_score_isqs(&isqs);
        (isqs, sync)
    };

    let mut best_freq = f0_baseband_init;
    let mut best_lag = lag_baseband_init;
    let mut best_drift = drift_hz;
    let (mut best_isqs, mut best_sync) = eval(best_freq, best_lag, best_drift);

    // 1. Coarse lag.
    for &dlag in &[-128i32, -64, 64, 128] {
        let (isqs, sync) = eval(best_freq, lag_baseband_init + dlag, best_drift);
        if sync > best_sync {
            best_sync = sync;
            best_lag = lag_baseband_init + dlag;
            best_isqs = isqs;
        }
    }

    // 2. Coarse freq — wsprd's `fstep = 0.25, ifmin = -2, ifmax = 2`.
    for i in -2i32..=2 {
        if i == 0 {
            continue;
        }
        let f = f0_baseband_init + i as f32 * 0.25;
        let (isqs, sync) = eval(f, best_lag, best_drift);
        if sync > best_sync {
            best_sync = sync;
            best_freq = f;
            best_isqs = isqs;
        }
    }

    // 3. Drift refine — `wsprd.c:1236-1255`, passes 0 and 1 only. wsprd
    // tries `drift ± 0.5` and keeps the better, checking `+` first so a
    // tie goes to `+` exactly as the C's `else if` does.
    if refine_drift {
        for &dd in &[0.5f32, -0.5] {
            let (isqs, sync) = eval(best_freq, best_lag, drift_hz + dd);
            if sync > best_sync {
                best_sync = sync;
                best_drift = drift_hz + dd;
                best_isqs = isqs;
                break;
            }
        }
    }

    // Stages 4-5 run only above wsprd's `minsync1` (`wsprd.c:800,1259`);
    // below it the candidate is noise and the extra evals are wasted.
    const MINSYNC1: f32 = 0.10;
    if best_sync > MINSYNC1 {
        // 4. Fine lag — ±32, step 16.
        let centre_lag = best_lag;
        for &dlag in &[-32i32, -16, 16, 32] {
            let (isqs, sync) = eval(best_freq, centre_lag + dlag, best_drift);
            if sync > best_sync {
                best_sync = sync;
                best_lag = centre_lag + dlag;
                best_isqs = isqs;
            }
        }
        // 5. Fine freq — `fstep = 0.05, ifmin = -2, ifmax = 2`.
        let centre_freq = best_freq;
        for i in -2i32..=2 {
            if i == 0 {
                continue;
            }
            let f = centre_freq + i as f32 * 0.05;
            let (isqs, sync) = eval(f, best_lag, best_drift);
            if sync > best_sync {
                best_sync = sync;
                best_freq = f;
                best_isqs = isqs;
            }
        }
    }

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
                &best_isqs
            } else {
                isqs_owned = super::demod::tone_amplitudes(idat, qdat, best_freq, lag, best_drift);
                &isqs_owned
            };
            let bm = if nblock == 0 {
                super::demod::nblock1_bit_metrics_opt(isqs, true)
            } else {
                super::demod::nblock_bit_metrics(isqs, nblock)
            };
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
            let (info_bits, hard_errors) = if let Some(fec_res) =
                codec.decode_soft_pooled(&llrs, &FecOpts::default(), &mut fano_scratch)
            {
                let mut info = [0u8; 50];
                info.copy_from_slice(&fec_res.info);
                (info, fec_res.hard_errors)
            } else if let Some((info, nhardmin)) = confirmed.and_then(|table| {
                // Single lazy `and_then`, deliberately not `Option::zip`:
                // `zip` evaluates its argument eagerly, which would run the
                // (expensive) OSD decode on every candidate even when no
                // table exists — i.e. on the whole of pass 1, which always
                // passes `None`.
                let (info, nhardmin) = super::osd::osd_decode(&llrs)?;
                // wsprd's structural gate (`wsprd.c:1396`): an OSD result
                // is accepted only for a callsign an earlier Fano decode
                // already confirmed. Replaces an `nhardmin ≤ 44` threshold
                // that measurement showed cannot separate the two
                // populations — see `WsprCallsignTable`'s doc comment.
                let msg = crate::msg::Wspr50Message
                    .unpack(&info, &crate::engine::DecodeContext::default())?;
                table.accepts(&msg).then_some((info, nhardmin))
            }) {
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
    let (mut idat, mut qdat) = super::baseband::decimate_to_baseband(&padded);
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
            let dup = seen.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
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
        // to clear the Fano convergence threshold. Same parallelize-
        // the-decode-step / dedup-sequentially-after shape as pass 1
        // above — this is pass 2's own per-candidate cost that's ~4x
        // pass 1's (3 nblock values vs 1), the dominant single cost in
        // `decode_scan` per `docs/notes/WSPR_BENCHMARK.md`'s Finding 2.
        #[cfg(feature = "parallel")]
        let raw2: Vec<WsprResult> = bb_cands2
            .par_iter()
            .filter_map(|c| decode_pass2_candidate(&idat, &qdat, sample_rate, pad, c, &confirmed))
            .collect();
        #[cfg(not(feature = "parallel"))]
        let raw2: Vec<WsprResult> = bb_cands2
            .iter()
            .filter_map(|c| decode_pass2_candidate(&idat, &qdat, sample_rate, pad, c, &confirmed))
            .collect();

        for d in raw2 {
            let dup = seen.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
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

/// Multi-pass WSPR decode with successive interference cancellation.
///
/// Mirrors WSJT-X `wsprd.c` `npasses=3` SIC loop (`wsprd.c:998-1438`):
/// each pass runs `decode_scan` on the current residual audio, decodes
/// every signal that survives Fano + (eventually) callsign sanity,
/// reconstructs the on-air channel-symbol sequence via
/// [`super::encode_channel_symbols`], and subtracts a channel-aware
/// LPF reference from the audio so subsequent passes can expose
/// previously-masked weak signals.
///
/// Returns deduplicated decodes from all passes.
pub fn decode_scan_subtract(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
) -> Vec<WsprResult> {
    decode_scan_subtract_inner(audio, sample_rate, nominal_start_sample, params, None)
}

/// Streaming variant of [`decode_scan_subtract`] — see
/// [`decode_scan_streaming`]'s doc comment for the general rationale
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
            let dup = all.iter().any(|prev| {
                prev.message == d.message
                    && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                    && (prev.start_sample as i64 - d.start_sample as i64).abs()
                        <= TIME_DEDUP_SAMPLES
            });
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
/// but for `f32` values).
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    let mut tmp = [0f32; 162];
    let mut p = 0u8;
    let mut i = 0u8;
    while p < 162 {
        // Inline the bit-reverse-8 to avoid exposing a pub helper.
        let i64 = i as u64;
        let j = ((((i64 * 0x8020_0802u64) & 0x0884_4221_10u64).wrapping_mul(0x0101_0101_01u64))
            >> 32) as u8 as usize;
        if j < 162 {
            tmp[p as usize] = llrs[j];
            p += 1;
        }
        i = i.wrapping_add(1);
    }
    *llrs = tmp;
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
