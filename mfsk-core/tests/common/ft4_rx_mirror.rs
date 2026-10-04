// SPDX-License-Identifier: GPL-3.0-only
//! The CoreS3 FT4 receiver's per-slot decode, reproduced call for call.
//!
//! `embedded-poc/embedded-shared/src/apps/ft4_rx.rs` is outside this
//! workspace and builds only under the `+esp` toolchain, so this is the
//! host stand-in: the same public `mfsk-core` entry points, in the same
//! order, with the same constants. Every consumer that needs to ask
//! "what would the board decode?" goes through here rather than
//! rebuilding the sequence, because the sequence is the thing under
//! test — `ft4_coarse_sync` over the whole 90 000-sample slot, which is
//! what the other FT4 tests use, produces a different candidate list
//! from the receiver's early-closed, incrementally-accumulated one.
//!
//! What it deliberately does **not** reproduce, and what the board is
//! therefore still the only instrument for: the deadline and its cut
//! (`TX_TURNAROUND_BUDGET_MS`), the two-core split, PSRAM and the
//! internal-DRAM threshold, and the slot grid.

use num_complex::Complex;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{Ft4SavgBuilder, ft4_coarse_sync_from_savg};
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeResult, DecodeStrictness, process_candidate_precomputed,
};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::{Ft4CoarsePhasors, ft4_sync_search_window_with};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::{SlotDecimator, candidate_baseband_half};
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

// ── the receiver's own constants ────────────────────────────────────
//
// Every one of these is a literal in `ft4_rx.rs`, repeated here because
// that crate cannot be linked from the host workspace.

/// `ft4_rx.rs`'s `SLOT_SAMPLES` — 7.5 s at 12 kHz.
pub const SLOT_SAMPLES: usize = 90_000;
/// `ft4_rx.rs`'s `CAPTURE_CLOSE_SAMPLES` — 6.775 s, where the receiver
/// stops taking audio because no candidate can reach past it.
pub const CAPTURE_CLOSE_SAMPLES: usize = 81_300;
pub const FREQ_MIN_HZ: f32 = 100.0;
pub const FREQ_MAX_HZ: f32 = 2_700.0;
pub const SYNC_MIN: f32 = 1.2;
pub const MAX_CAND: usize = 100;
/// `ft4::decode`'s own private `SYNC_Q_MIN`.
pub const SYNC_Q_MIN: u32 = 8;
/// `ft4_rx.rs`'s `WSJTX_WINDOW`, in `cd0` samples at 666.667 Hz.
pub const WSJTX_WINDOW: (i32, i32) = (-344, 1012);
/// The block size a UAC read produces, which is the cadence the board
/// feeds `SlotAccum` at. Block-independence is pinned elsewhere
/// (`slot_decimator_is_block_independent`); using the real one keeps
/// this a mirror.
pub const BLOCK: usize = 256;

/// The capture side: the periodogram and the shared half-rate stream,
/// advanced together a block at a time exactly as `SlotAccum` does.
pub fn capture(audio: &[i16]) -> (Vec<f32>, Vec<f32>) {
    let mut savg = Ft4SavgBuilder::new(CAPTURE_CLOSE_SAMPLES);
    let mut decim = SlotDecimator::new();
    let mut half: Vec<f32> = Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2 + 64);
    let mut fed = 0usize;
    while fed < CAPTURE_CLOSE_SAMPLES.min(audio.len()) {
        let take = BLOCK.min(CAPTURE_CLOSE_SAMPLES.min(audio.len()) - fed);
        let block = &audio[fed..fed + take];
        savg.push_with_rows(block, &mut |_row| {});
        decim.push_i16(block, &mut half);
        fed += take;
    }
    (savg.finish(), half)
}

/// The coarse stage, with the receiver's own band and caps.
pub fn coarse(savg: &[f32]) -> Vec<SyncCandidate> {
    ft4_coarse_sync_from_savg(savg, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND)
}

/// The RMS normalisation `process_candidate_basic_impl` applies to a
/// `cd0` it builds itself (WSJT-X `ft4_decode.f90:231-232`).
/// `candidate_baseband_half` deliberately does not, so the receiver
/// does it — `ft4_rx.rs`'s `rms_normalise` — and so does this.
pub fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// How a candidate's baseband gets built.
///
/// The receiver has exactly one of these today — `ft4::ddc`'s 101 + 263
/// taps — and the whole question behind the cheaper front ends is what
/// a blunter one would cost. Taking it as a parameter lets one test
/// answer that without a second copy of the pipeline.
pub type Producer = fn(&[f32], f32) -> Vec<Complex<f32>>;

/// What ships: mixer -> 101-tap ÷9 -> 263-tap ÷1 -> derotate.
pub fn fir_producer(half: &[f32], f0_hz: f32) -> Vec<Complex<f32>> {
    candidate_baseband_half(half, f0_hz)
}

/// Mix and boxcar-decimate by nine — `ft4::ddc`'s experimental
/// producer, so the host measurements and the board's run the same
/// code rather than two implementations of the same idea.
pub fn boxcar_producer(half: &[f32], f0_hz: f32) -> Vec<Complex<f32>> {
    mfsk_core::ft4::ddc::candidate_baseband_boxcar(half, f0_hz)
}

/// [`boxcar_producer`] written the obvious way, with a `cos`/`sin` per
/// sample. Kept as the reference the stepping version is checked
/// against — the same discipline `sync2d`'s own rotator carries.
pub fn boxcar_producer_reference(half: &[f32], f0_hz: f32) -> Vec<Complex<f32>> {
    use core::f32::consts::TAU;
    const DECIM: usize = 9;
    const HALF_WIN: usize = DECIM / 2;
    let in_rate = 6_000.0f32;
    let out_rate = in_rate / DECIM as f32;
    let centre = f0_hz + 31.25;
    // **Phase by accumulation, reduced each step.** The obvious
    // `-TAU * centre * n / rate` loses its low bits long before
    // `n = 40 609`: at f32, `cos` of ~4.4e4 radians is argument
    // reduction, not a cosine, and comparing the stepping version
    // against *that* measured the reference's error (2.4e-3 of full
    // scale) rather than the rotator's. Same discipline
    // `make_costas_ref` already uses.
    let dphi_in = -TAU * centre / in_rate;
    let dphi_out = TAU * 31.25 / out_rate;
    let mut mixed: Vec<Complex<f32>> = Vec::with_capacity(half.len());
    let mut phi = 0.0f32;
    for &x in half {
        mixed.push(Complex::new(phi.cos(), phi.sin()) * x);
        phi = (phi + dphi_in) % TAU;
    }
    let mut out = Vec::with_capacity(mfsk_core::ft4::ddc::CD0_LEN);
    let mut psi = 0.0f32;
    for j in 0..mfsk_core::ft4::ddc::CD0_LEN {
        let c = (j * DECIM) as i64;
        let mut acc = Complex::new(0.0f32, 0.0);
        for k in -(HALF_WIN as i64)..=(HALF_WIN as i64) {
            let n = c + k;
            if n < 0 || n as usize >= mixed.len() {
                continue;
            }
            acc += mixed[n as usize];
        }
        out.push(acc / DECIM as f32 * Complex::new(psi.cos(), psi.sin()));
        psi = (psi + dphi_out) % TAU;
    }
    out
}

/// Which of the two experimental arms a run uses. `Variant::SHIPPED`
/// is what the board does today.
#[derive(Clone, Copy)]
pub struct Variant {
    pub produce: Producer,
    /// Score the coarse Δt/Δf sweep over tone-demodulated bins instead
    /// of over every `cd0` sample.
    pub binned_search: bool,
    /// Snap each candidate's carrier to the coarse periodogram's bin
    /// centre before building its baseband.
    ///
    /// The precondition for building basebands *during* capture: a
    /// provisional candidate list, taken from a partial periodogram,
    /// interpolates each peak slightly differently from the final one,
    /// so a baseband mixed at the provisional carrier is not the one
    /// the final candidate would have built. Snapped to the bin grid,
    /// both land on the same carrier and can share it. The Δt/Δf search
    /// then starts from the bin centre and still sweeps ±12 Hz, which
    /// is what should make the snap cheap — the question this switch
    /// exists to measure.
    pub snap_to_bin: bool,
}

impl Variant {
    pub const SHIPPED: Self = Self {
        produce: fir_producer,
        binned_search: false,
        snap_to_bin: false,
    };
    pub const BOXCAR: Self = Self {
        produce: boxcar_producer,
        binned_search: false,
        snap_to_bin: false,
    };
    pub const BINNED_SEARCH: Self = Self {
        produce: fir_producer,
        binned_search: true,
        snap_to_bin: false,
    };
    pub const BOTH: Self = Self {
        produce: boxcar_producer,
        binned_search: true,
        snap_to_bin: false,
    };
    pub const SNAPPED: Self = Self {
        produce: fir_producer,
        binned_search: false,
        snap_to_bin: true,
    };
}

/// `ft4_coarse`'s periodogram bin width: `12 000 / NFFT1` = 5.208 Hz.
pub const COARSE_BIN_HZ: f32 = 12_000.0 / 2_304.0;

/// A carrier snapped to the nearest periodogram bin centre.
pub fn snap_to_bin(freq_hz: f32) -> f32 {
    (freq_hz / COARSE_BIN_HZ).round() * COARSE_BIN_HZ
}

/// `ft4_rx::decode_candidate`, call for call.
pub fn decode_candidate(
    half: &[f32],
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
) -> Option<String> {
    decode_candidate_with(half, cand, refs, Variant::SHIPPED)
}

/// [`decode_candidate`] over a chosen arm.
pub fn decode_candidate_with(
    half: &[f32],
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
    v: Variant,
) -> Option<String> {
    let snapped;
    let cand = if v.snap_to_bin {
        snapped = SyncCandidate {
            freq_hz: snap_to_bin(cand.freq_hz),
            ..*cand
        };
        &snapped
    } else {
        cand
    };
    let cd0 = (v.produce)(half, cand.freq_hz);
    decode_from_cd0(cd0, cand, refs, v.binned_search)
}

/// The part of a candidate's decode that comes after its baseband:
/// normalise, Δt/Δf search, LLR, BP, unpack. Split out so a caller that
/// built the baseband some other way — during capture, say — runs the
/// rest through the same code.
pub fn decode_from_cd0(
    mut cd0: Vec<Complex<f32>>,
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
    binned_search: bool,
) -> Option<String> {
    rms_normalise(&mut cd0);
    let s2 = if binned_search {
        mfsk_core::engine::sync2d::ft4_sync_search_window_binned::<Ft4>(
            &cd0,
            cand,
            WSJTX_WINDOW.0,
            WSJTX_WINDOW.1,
            refs,
        )
    } else {
        ft4_sync_search_window_with::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1, refs)
    };
    decode_after_search(cd0, cand, s2)
}

/// The decode after the Δt/Δf search: LLR, BP, unpack, from a
/// normalised `cd0` and the search's result.
pub fn decode_after_search(
    cd0: Vec<Complex<f32>>,
    cand: &SyncCandidate,
    s2: mfsk_core::engine::sync2d::Sync2dResult,
) -> Option<String> {
    let r: Option<DecodeResult> = process_candidate_precomputed::<Ft4>(
        cand,
        // FT4's `snr_db` reads the coarse candidate score, not a
        // wide-band cache, so the board passes an empty slice here and
        // so does this.
        &[],
        &FT4_DOWNSAMPLE,
        DecodeDepth::EMBEDDED,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
        (cd0, s2.freq_hz, s2.i0, s2.score),
        false,
        false,
    );
    unpack77(r?.message77())
}

/// One whole slot in, its distinct messages out, in candidate order —
/// which is descending coarse score, the order the receiver dedups in.
pub fn run_slot(audio: &[i16]) -> Vec<String> {
    run_slot_with(audio, Variant::SHIPPED)
}

/// [`run_slot`] over a chosen arm.
pub fn run_slot_with(audio: &[i16], v: Variant) -> Vec<String> {
    let (savg, half) = capture(audio);
    let cands = coarse(&savg);
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    let mut out: Vec<String> = Vec::new();
    for cand in &cands {
        if let Some(text) = decode_candidate_with(&half, cand, &refs, v)
            && !out.contains(&text)
        {
            out.push(text);
        }
    }
    out
}

/// The sample at which a Costas block's last symbol has arrived, for a
/// frame at nominal timing (DT = 0: first active symbol at
/// `TX_START_OFFSET_S`). `block` is 0..=3; 3 is the frame's last
/// active symbol.
///
/// Derived from the protocol rather than chosen: the periodogram the
/// coarse stage reads is a sum over a frame's tones, so a candidate's
/// peak is settled once its frame has arrived. For block 3 that is
/// `0.5 + 103 x 576 / 12 000` = 5.444 s. A station with positive DT
/// finishes later and simply falls to the build-after-close path.
pub fn costas_block_end_samples(block: usize) -> usize {
    use mfsk_core::engine::{FrameLayout, ModulationParams};
    let b = &Ft4::SYNC_MODE.blocks()[block];
    let end_symbol = b.start_symbol as usize + b.pattern.len();
    (Ft4::TX_START_OFFSET_S * 12_000.0) as usize + end_symbol * Ft4::NSPS as usize
}

/// Each buffer a baseband built during capture owns is allocated at
/// least this big: one byte over `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`
/// (2 048 on the CoreS3), so the allocator puts it in PSRAM.
pub const PIPELINED_MIN_ALLOC_BYTES: usize = 2_049;

/// When the pipelined receiver reads its provisional list: the last
/// active symbol of a nominally-timed frame (Costas block D's end).
pub fn provisional_samples() -> usize {
    costas_block_end_samples(3)
}

/// What the pipelined receiver did with its early basebands.
#[derive(Debug, Default, Clone, Copy)]
pub struct PipelineStats {
    /// Final candidates whose baseband was already built during capture.
    pub reused: usize,
    /// Final candidates with no provisional match, built after the close.
    pub fresh: usize,
    /// Provisional basebands the final list did not ask for.
    pub wasted: usize,
    /// Coarse block-cells the capture-time sweeps had scored by the
    /// close, and how many there are in all.
    pub sweep_done: usize,
    pub sweep_total: usize,
}

/// The receiver with its per-candidate DDC moved into capture time.
///
/// At `prov_samples` a provisional candidate list is read off the
/// running periodogram (`Ft4SavgBuilder::snapshot`) and a streaming
/// `CandidateDdc` starts for each — at the **snapped** carrier, so the
/// final list can share it — fed the half-rate backlog and then every
/// block as it arrives. At the close the final list takes a matching
/// baseband by bin, or builds one as today if the provisional list
/// missed it.
///
/// Must decode exactly what [`Variant::SNAPPED`] decodes: the carrier
/// is the same snapped value either way, and `FirStage` is
/// block-independent, so feeding the backlog and then the tail is the
/// same filter over the same samples as feeding them at once.
pub fn run_slot_pipelined(audio: &[i16], prov_samples: usize) -> (Vec<String>, PipelineStats) {
    run_slot_pipelined_with(audio, prov_samples, false)
}

/// [`run_slot_pipelined`], optionally with each candidate's coarse
/// Δt/Δf sweep also run during capture (`Ft4CoarseSweep`) as its
/// baseband grows, leaving only the rest of it and the fine pass for
/// after the close.
pub fn run_slot_pipelined_with(
    audio: &[i16],
    prov_samples: usize,
    sweep_in_capture: bool,
) -> (Vec<String>, PipelineStats) {
    use mfsk_core::engine::sync2d::{Ft4CoarseSweep, Ft4SweepScratch};
    use mfsk_core::ft4::ddc::{CD0_LEN, CandidateDdc};

    let bin_of = |f: f32| (f / COARSE_BIN_HZ).round() as i32;
    let close = CAPTURE_CLOSE_SAMPLES.min(audio.len());

    let mut savg = Ft4SavgBuilder::new(CAPTURE_CLOSE_SAMPLES);
    let mut decim = SlotDecimator::new();
    let mut half: Vec<f32> = Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2 + 64);
    /// One baseband being built during capture.
    struct Pipe {
        bin: i32,
        ddc: CandidateDdc,
        out: Vec<Complex<f32>>,
        /// Half-rate samples already fed.
        fed: usize,
        sweep: Option<Ft4CoarseSweep>,
    }
    let mut pipes: Vec<Pipe> = Vec::new();
    let mut started = false;
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    // Only when sweeping: a receiver holds one per worker, and the
    // DDC-only arrangement has none.
    let mut scratch = sweep_in_capture
        .then(|| Ft4SweepScratch::new_with_min_alloc::<Ft4>(PIPELINED_MIN_ALLOC_BYTES));

    let mut fed = 0usize;
    while fed < close {
        let take = BLOCK.min(close - fed);
        let block = &audio[fed..fed + take];
        savg.push_with_rows(block, &mut |_row| {});
        decim.push_i16(block, &mut half);
        fed += take;

        if !started && fed >= prov_samples {
            started = true;
            for c in coarse(&savg.snapshot()) {
                let f = snap_to_bin(c.freq_hz);
                let b = bin_of(f);
                if pipes.iter().any(|p| p.bin == b) {
                    continue;
                }
                pipes.push(Pipe {
                    bin: b,
                    // Just over the CoreS3's 2 048-byte internal-DRAM
                    // threshold, as the receiver will ask for — so the
                    // host counts the same placement the board makes.
                    ddc: CandidateDdc::new_half_rate_with_min_alloc(f, PIPELINED_MIN_ALLOC_BYTES),
                    out: Vec::with_capacity(CD0_LEN),
                    fed: 0,
                    sweep: sweep_in_capture.then(|| {
                        Ft4CoarseSweep::new(
                            WSJTX_WINDOW.0,
                            WSJTX_WINDOW.1,
                            CD0_LEN,
                            PIPELINED_MIN_ALLOC_BYTES,
                        )
                    }),
                });
            }
        }
        for p in pipes.iter_mut() {
            p.ddc.push_f32(&half[p.fed..], &mut p.out);
            p.fed = half.len();
            if let Some(sw) = p.sweep.as_mut() {
                let scratch = scratch.as_mut().expect("sweeping implies a scratch");
                sw.advance::<Ft4>(&p.out, CD0_LEN, scratch, &refs, usize::MAX);
            }
        }
    }

    let finals = coarse(&savg.finish());
    let mut stats = PipelineStats::default();
    for p in &pipes {
        if let Some(sw) = &p.sweep {
            let (d, t) = sw.progress();
            stats.sweep_done += d;
            stats.sweep_total += t;
        }
    }
    let mut used = vec![false; pipes.len()];
    let mut messages: Vec<String> = Vec::new();
    for cand in &finals {
        let f = snap_to_bin(cand.freq_hz);
        let snapped = SyncCandidate {
            freq_hz: f,
            ..*cand
        };
        let mut sweep = None;
        let cd0 = if let Some(k) = pipes.iter().position(|p| p.bin == bin_of(f)) {
            used[k] = true;
            stats.reused += 1;
            let p = &mut pipes[k];
            let mut cd0 = core::mem::take(&mut p.out);
            p.ddc.flush_to(CD0_LEN, &mut cd0);
            cd0.resize(CD0_LEN, Complex::new(0.0, 0.0));
            sweep = p.sweep.take();
            cd0
        } else {
            stats.fresh += 1;
            candidate_baseband_half(&half, f)
        };
        let decoded = match sweep {
            Some(sw) => {
                let mut sw = sw;
                let mut cd0 = cd0;
                let scratch = scratch.as_mut().expect("sweeping implies a scratch");
                sw.complete::<Ft4>(&cd0, scratch, &refs);
                rms_normalise(&mut cd0);
                let s2 = sw.fine::<Ft4>(&cd0, &snapped, scratch, &refs);
                decode_after_search(cd0, &snapped, s2)
            }
            None => decode_from_cd0(cd0, &snapped, &refs, false),
        };
        if let Some(text) = decoded
            && !messages.contains(&text)
        {
            messages.push(text);
        }
    }
    stats.wasted = used.iter().filter(|u| !**u).count();
    (messages, stats)
}
