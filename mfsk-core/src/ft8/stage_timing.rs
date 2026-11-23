// SPDX-License-Identifier: GPL-3.0-only
//! Where one FT8 decode call spends its time, by stage (#761).
//!
//! Off unless `MFSK_TRACE_STAGE_FT8` is set in the environment. Then every call of
//! `Decoder<Ft8>` (`decode`, `decode_prefix`) prints one line on stderr, `TRACE_STAGE_FT8_ALL`:
//! the call's wall time, the time attributed to stages, and each stage's time with the number of
//! times it ran, largest first.
//!
//! A stage's time is **exclusive**: a stage that runs inside another (OSD calls its own LLR and
//! BP, the SNR estimate calls the symbol spectra) is taken out of the outer one, so the figures
//! add up. They are summed over every thread that ran the stage, so with more than one rayon
//! thread they can exceed the wall time; use `RAYON_NUM_THREADS=1` for a split of one core.
//! What is not in any stage (candidate triage, sorting, allocation, the loops between stages) is
//! the difference between wall and attributed time.
//!
//! This is a measurement aid. It changes no result, and with the variable unset a stage costs one
//! relaxed atomic load. It exists because `perf` is not available on every machine this is run
//! on (WSL2, a Raspberry Pi without the tools, macOS without Instruments), and a profiler's
//! symbol-level answer is not the one a stage-level change needs.

/// A stage of an FT8 decode.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub(crate) enum Stage {
    /// The spectrogram `coarse_sync` searches.
    Spectrogram,
    /// `coarse_sync`: the sync search over the spectrogram.
    CoarseSync,
    /// The 192 000-point FFT of the whole audio every candidate's downsample reads.
    FftCache,
    /// A candidate's downsample to the 200 Hz baseband (an inverse FFT of its band).
    Downsample,
    /// `fine_refine_3stage`: the time and frequency refinement of a candidate.
    FineRefine,
    /// A candidate's 79 x 8 symbol spectra.
    SymbolSpectra,
    /// `llra`/`llrd` at one symbol (`compute_llr_fast_metric`).
    LlrFast,
    /// The two- and three-symbol metrics (`compute_llr_partial_metric`).
    LlrPartial,
    /// The `llre` metric.
    LlrE,
    /// All four metrics at once (`compute_llr_metric`).
    LlrFull,
    /// A belief-propagation run.
    Bp,
    /// OSD, apart from the LLR and BP it calls.
    Osd,
    /// Subtracting a decoded signal from the audio.
    Subtract,
    /// The baseline spectrum the SNR estimate reads.
    Baseline,
    /// The WSJT-X SNR estimate of a decode, apart from the spectra it takes.
    Xsnr,
}

#[allow(dead_code)]
const N: usize = 15;
#[allow(dead_code)]
const NAMES: [&str; N] = [
    "spectrogram",
    "coarse_sync",
    "fft_cache",
    "downsample",
    "fine_refine",
    "symbol_spectra",
    "llr_fast",
    "llr_partial",
    "llr_e",
    "llr_full",
    "bp",
    "osd",
    "subtract",
    "baseline",
    "xsnr",
];

#[cfg(feature = "std")]
mod imp {
    use super::{N, NAMES, Stage};
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
    use std::sync::OnceLock;
    use std::time::Instant;

    #[allow(clippy::declare_interior_mutable_const)]
    const Z: AtomicU64 = AtomicU64::new(0);
    static NANOS: [AtomicU64; N] = [Z; N];
    static COUNT: [AtomicU64; N] = [Z; N];
    static ACTIVE: AtomicBool = AtomicBool::new(false);

    fn enabled() -> bool {
        static ON: OnceLock<bool> = OnceLock::new();
        *ON.get_or_init(|| std::env::var_os("MFSK_TRACE_STAGE_FT8").is_some())
    }

    std::thread_local! {
        /// The stages this thread is inside, innermost last, each with the instant its current
        /// stretch began (a stretch ends when an inner stage starts, and begins again when it ends).
        static STACK: RefCell<Vec<(Stage, Instant)>> = const { RefCell::new(Vec::new()) };
    }

    /// Inside a stage until dropped.
    pub(crate) struct Scope(bool);

    pub(crate) fn scope(stage: Stage) -> Scope {
        if !enabled() || !ACTIVE.load(Relaxed) {
            return Scope(false);
        }
        STACK.with(|s| {
            let now = Instant::now();
            let mut s = s.borrow_mut();
            if let Some((outer, since)) = s.last() {
                NANOS[*outer as usize].fetch_add((now - *since).as_nanos() as u64, Relaxed);
            }
            s.push((stage, now));
        });
        Scope(true)
    }

    impl Drop for Scope {
        fn drop(&mut self) {
            if !self.0 {
                return;
            }
            STACK.with(|s| {
                let now = Instant::now();
                let mut s = s.borrow_mut();
                if let Some((stage, since)) = s.pop() {
                    NANOS[stage as usize].fetch_add((now - since).as_nanos() as u64, Relaxed);
                    COUNT[stage as usize].fetch_add(1, Relaxed);
                }
                if let Some(outer) = s.last_mut() {
                    outer.1 = now;
                }
            });
        }
    }

    /// One call of `Decoder<Ft8>`: starts the accounting, prints it when dropped.
    pub(crate) struct Call(Option<Instant>);

    impl Call {
        pub(crate) fn begin() -> Self {
            if !enabled() || ACTIVE.swap(true, Relaxed) {
                return Call(None); // off, or a call already being accounted (not nested)
            }
            for i in 0..N {
                NANOS[i].store(0, Relaxed);
                COUNT[i].store(0, Relaxed);
            }
            Call(Some(Instant::now()))
        }
    }

    impl Drop for Call {
        fn drop(&mut self) {
            let Some(start) = self.0 else { return };
            let wall = start.elapsed().as_secs_f64() * 1e3;
            ACTIVE.store(false, Relaxed);
            let mut rows: Vec<(f64, u64, &str)> = (0..N)
                .map(|i| {
                    (
                        NANOS[i].load(Relaxed) as f64 / 1e6,
                        COUNT[i].load(Relaxed),
                        NAMES[i],
                    )
                })
                .filter(|r| r.1 > 0)
                .collect();
            rows.sort_by(|a, b| b.0.total_cmp(&a.0));
            let attributed: f64 = rows.iter().map(|r| r.0).sum();
            let mut line =
                format!("TRACE_STAGE_FT8_ALL wall={wall:.1}ms attributed={attributed:.1}ms");
            for (ms, n, name) in rows {
                line += &format!(" {name}={ms:.1}ms({n})");
            }
            eprintln!("{line}");
        }
    }
}

#[cfg(not(feature = "std"))]
mod imp {
    use super::Stage;

    pub(crate) struct Scope;

    #[inline(always)]
    pub(crate) fn scope(_stage: Stage) -> Scope {
        Scope
    }

    pub(crate) struct Call;

    impl Call {
        #[inline(always)]
        pub(crate) fn begin() -> Self {
            Call
        }
    }
}

pub(crate) use imp::{Call, scope};
