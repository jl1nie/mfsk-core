//! The one waterfall feed every receiver shares.
//!
//! The panel is the same in every mode (`display::run_log_panel`), so
//! its waterfall is fed the same way in every mode: from the 12 kHz
//! audio itself, not from any decoder. FT8 used to derive its rows from
//! stage 1's spectra, FT4 from its coarse periodogram, and WSPR and FST4
//! had nothing — an empty waterfall on a panel otherwise identical.
//!
//! Two halves, on two tasks:
//!
//! - [`push`], from wherever audio enters: the UAC reader, the SIM
//!   feed, and the baked-recording sources that do not pass through
//!   `uac`. It only appends to a ring, with `try_lock`, so it can never
//!   make the audio path wait — the isochronous URB budget is 48 ms and
//!   nothing on that thread may spend it. When the lock is busy or the
//!   ring is full the samples are dropped here, and only here: the
//!   waterfall skips, the decoders do not.
//! - [`drain_to_ui`], from the panel loop: builds rows with
//!   `embedded_shared::waterfall::WfRowBuilder` (esp-dsp throughout)
//!   and pushes them into `ui::state::UI`. The panel is the lowest-
//!   priority thing on its core in every mode, so rows arrive in a burst
//!   after a decode has held the core; the ring holds [`CAP`] samples
//!   for that.
//!
//! **Slot marks come from UTC**, at the booted mode's slot period, and
//! are placed in the same sample stream the rows are built from. FT8's
//! used to come from its own stage-1 grid, which with no NTP follows
//! the air instead of the clock — so on a board that has only its RTC,
//! the rule now shows the clock's boundary, not the one the FT8 decoder
//! locked to.

use core::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;

use embedded_shared::waterfall::WfRowBuilder;
use mfsk_app_shared::boot_mode::BootMode;
use mfsk_app_shared::ui::state::UI;

/// Four seconds of 12 kHz audio: more than the longest stretch the
/// panel's core is held — an FT8 slot's decode, ~1.8 s — with room.
const CAP: usize = 48_000;

/// Samples lost to a busy lock or a full ring, for the periodic report.
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Slot boundaries the ring recorded, and rows the drain marked with one
/// — for the periodic report. A rule on the screen needs both.
static MARKS_SEEN: AtomicU32 = AtomicU32::new(0);
static MARKS_DRAWN: AtomicU32 = AtomicU32::new(0);

/// Slot period of the booted mode, ms; 0 draws no marks.
static PERIOD_MS: AtomicU32 = AtomicU32::new(0);

/// The row transform's length, real samples (`waterfall::WF_NFFT_*`).
static NFFT: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(embedded_shared::waterfall::WF_NFFT_FINE);

/// What a receiver asks of the feed.
#[derive(Clone, Copy, Debug)]
pub struct FeedConfig {
    /// Where the slot rules fall, ms; `0` draws none — a receiver with no
    /// slot, like JTTY, whose rules would mark nothing on the air.
    pub slot_period_ms: u32,
    /// `waterfall::WF_NFFT_FINE` for the slotted modes, or
    /// `WF_NFFT_COLUMN` where the panel's core has no slack to give
    /// (see those constants for what each costs and shows).
    pub nfft: usize,
}

impl FeedConfig {
    /// A receiver's own: the slotted modes their slot's rules and the
    /// finer transform; JTTY no rules (`BootMode::slot_rules_ms`) and the
    /// column-width transform, because its `Back` shares the panel's
    /// core with no slack to give (see `WF_NFFT_COLUMN`).
    pub fn for_mode(mode: BootMode) -> Self {
        Self {
            slot_period_ms: mode.slot_rules_ms(),
            nfft: match mode {
                BootMode::Jtty => embedded_shared::waterfall::WF_NFFT_COLUMN,
                _ => embedded_shared::waterfall::WF_NFFT_FINE,
            },
        }
    }
}

struct Ring {
    buf: Vec<i16>,
    /// Samples accepted into the ring in all — the stream the rows and
    /// the marks share.
    accepted: u64,
    /// Where the clock says the next slot boundary falls, in `accepted`.
    next_boundary: Option<u64>,
    /// Boundaries passed and not yet drawn.
    marks: heapless::Deque<u64, 8>,
}

static RING: Mutex<Option<Ring>> = Mutex::new(None);

struct Drain {
    builder: WfRowBuilder,
    /// Swapped with the ring's buffer on every drain, so neither side
    /// ever reallocates.
    spare: Vec<i16>,
    marks: heapless::Deque<u64, 8>,
    last_report_us: i64,
}

static DRAIN: Mutex<Option<Drain>> = Mutex::new(None);

/// Allocate the ring and set the mode's slot period. Called once from
/// `boot::run`; [`push`] is a no-op before it.
pub fn init(mode: BootMode) {
    init_with(FeedConfig::for_mode(mode));
}

/// [`init`] with the rules and the transform chosen explicitly.
pub fn init_with(cfg: FeedConfig) {
    PERIOD_MS.store(cfg.slot_period_ms, Ordering::Release);
    NFFT.store(cfg.nfft, Ordering::Release);
    if let Ok(mut g) = RING.lock() {
        *g = Some(Ring {
            buf: Vec::with_capacity(CAP),
            accepted: 0,
            next_boundary: None,
            marks: heapless::Deque::new(),
        });
    }
}

/// `MFSK_WF_FEED_OFF=1` builds without the feed — no ring, no rows — so
/// what it costs the decoders can be measured as a difference rather
/// than argued. Compile-time, like every other measurement knob here.
const FEED_OFF: bool = option_env!("MFSK_WF_FEED_OFF").is_some();

/// Offer audio to the waterfall. Never blocks; see the module doc.
pub fn push(samples: &[i16]) {
    if FEED_OFF {
        return;
    }
    let Ok(mut g) = RING.try_lock() else {
        DROPPED.fetch_add(samples.len() as u32, Ordering::Relaxed);
        return;
    };
    let Some(r) = g.as_mut() else {
        return;
    };
    let take = samples.len().min(CAP - r.buf.len());
    if take < samples.len() {
        DROPPED.fetch_add((samples.len() - take) as u32, Ordering::Relaxed);
    }
    r.buf.extend_from_slice(&samples[..take]);
    r.accepted += take as u64;
    let period = PERIOD_MS.load(Ordering::Acquire);
    if period == 0 {
        return;
    }
    if let Some(b) = r.next_boundary.filter(|&b| r.accepted >= b) {
        if r.marks.is_full() {
            let _ = r.marks.pop_front();
        }
        let _ = r.marks.push_back(b);
        MARKS_SEEN.fetch_add(1, Ordering::Relaxed);
    }
    // Re-read every block, so a clock step (NTP landing) moves the
    // rules with it.
    r.next_boundary = mfsk_app_shared::time_sync::samples_to_next_slot_12k_ms(period as u64)
        .map(|remain| r.accepted + remain as u64);
}

/// Turn whatever audio has arrived into waterfall rows. From the panel
/// loop, every frame.
pub fn drain_to_ui() {
    if FEED_OFF {
        return;
    }
    let Ok(mut dg) = DRAIN.lock() else {
        return;
    };
    let d = dg.get_or_insert_with(|| Drain {
        builder: WfRowBuilder::new(NFFT.load(Ordering::Acquire)),
        spare: Vec::with_capacity(CAP),
        marks: heapless::Deque::new(),
        last_report_us: now_us(),
    });
    {
        let Ok(mut g) = RING.lock() else {
            return;
        };
        let Some(r) = g.as_mut() else {
            return;
        };
        if r.buf.is_empty() {
            return;
        }
        core::mem::swap(&mut r.buf, &mut d.spare);
        while let Some(m) = r.marks.pop_front() {
            if d.marks.is_full() {
                let _ = d.marks.pop_front();
            }
            let _ = d.marks.push_back(m);
        }
    }
    let Drain {
        builder,
        spare,
        marks,
        ..
    } = d;
    builder.push(spare, &mut |row, pos| {
        let mut mark = false;
        while marks.front().is_some_and(|&b| b <= pos) {
            let _ = marks.pop_front();
            mark = true;
        }
        if mark {
            MARKS_DRAWN.fetch_add(1, Ordering::Relaxed);
        }
        if let Ok(mut ui) = UI.lock() {
            // `push_waterfall_at` rules a row whose slot index is 0 or
            // 1; `u8::MAX` is "not stated".
            ui.push_waterfall_at(row, if mark { 0 } else { u8::MAX });
        }
    });
    spare.clear();

    // The UI transform's cost, every ~10 s, as the builder measured it:
    // the esp-dsp part (window, FFT, real unfold, power) and the column
    // mapping separately, plus what was dropped on the way in.
    let now = now_us();
    if now - d.last_report_us >= 10_000_000 {
        let t = d.builder.take_timing();
        if t.rows > 0 {
            log::info!(
                "waterfall: {} rows — fft {} us/row (min {}) + map {} us/row (slowest row {} us), \
                 {} samples dropped, slot marks {} seen / {} drawn",
                t.rows,
                t.fft_us / t.rows as i64,
                t.min_fft_us,
                t.map_us / t.rows as i64,
                t.max_row_us,
                DROPPED.swap(0, Ordering::Relaxed),
                MARKS_SEEN.swap(0, Ordering::Relaxed),
                MARKS_DRAWN.swap(0, Ordering::Relaxed),
            );
        }
        d.last_report_us = now;
    }
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
