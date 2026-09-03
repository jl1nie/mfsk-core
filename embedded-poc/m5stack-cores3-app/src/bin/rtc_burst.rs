// SPDX-License-Identifier: GPL-3.0-or-later
//! Does a five-minute RTC drift measurement work? (issue #354)
//!
//! This bin exists to validate an **estimator**, not to produce a
//! drift number. `rtc_drift` spent 8.175 h to resolve 0.34 ppm, and
//! the reason it needed that long is not physics — it is that it
//! compares two endpoints and nothing in between, so its precision is
//! (edge-detection error) / (elapsed time) and the only lever is time.
//!
//! ## What the 8 h run measured about itself
//!
//! Its per-interval RTC-vs-crystal figures alternate between −48.9 and
//! −65.2 ppm across all 48 intervals. That spread is not the clock: at
//! 613.03 s per interval, 16.3 ppm is **10.0 ms** of edge-detection
//! error, appearing or not appearing depending on which side of a poll
//! the tick landed.
//!
//! 10 ms is exactly one FreeRTOS tick. `rtc_drift::wait_rtc_tick`
//! polls with `FreeRtos::delay_ms(4)`, and no `CONFIG_FREERTOS_HZ` is
//! set anywhere in this crate's `sdkconfig.defaults`, so the ESP-IDF
//! default of 100 Hz applies and every 4 ms request sleeps 10 ms. Its
//! own comment claims the edge is kept "to a few ms"; it is not.
//!
//! ## What this does instead
//!
//! Take an edge every second and fit a line, rather than differencing
//! two of them:
//!
//! - poll with `esp_rom_delay_us(200)` (a busy ROM delay, not a tick
//!   sleep), yielding for 900 ms after each accepted edge so the idle
//!   task still runs and the task watchdog stays quiet;
//! - accumulate `y = rtc_elapsed_us − mono_elapsed_us` against
//!   `x = mono_elapsed_s` and least-squares the slope. With `y` in µs
//!   and `x` in s the slope **is** ppm, no scaling;
//! - report its standard error and the residual RMS, so the run says
//!   how well it measured rather than leaving that to be assumed.
//!
//! Expected precision for N points over T seconds with per-edge error
//! σ: `SE ≈ σ·√12 / (T·√N)`. At σ = 1 ms, T = 300 s, N = 300 that is
//! **0.67 ppm** — i.e. five minutes buys what eight hours bought.
//!
//! ## The A/B, because the tick claim above is a diagnosis
//!
//! Burst 0 polls the old way (`FreeRtos::delay_ms(4)`) and bursts 1..
//! poll the new way. If the diagnosis is right, burst 0's residual RMS
//! lands near 10 ms/√12 = 2.9 ms and the others near a few hundred µs,
//! while all of them agree on the slope. That is the check this bin is
//! for; the drift number itself is already known from the 8 h run
//! (**−59.46 ppm** RTC vs crystal, ±0.34 ppm) and is what these bursts
//! are compared against.
//!
//! Nothing here touches NVS. The 8 h run's samples live in namespace
//! `rtcdrift` and are the only copy of a measurement that cost a day;
//! an estimator experiment has no business writing to that store.
//!
//! ## Why there is a panel and an NVS summary
//!
//! A first cut of this bin had neither, and the operator's objection
//! was immediate and correct: a board with a dark screen running a
//! 16-minute measurement is indistinguishable from a crashed one, and
//! the serial console — the only other channel — had already died
//! twice that day (`rtc_drift`'s `Broken pipe` at +1200 s of a 3600 s
//! run, and two DOWNLOAD-mode parks). Betting a whole run on it again
//! was the same mistake at a shorter timescale.
//!
//! So: the panel carries burst number, poll mode, `n`, the running fit
//! and a heartbeat that advances on every edge — one edge per second
//! *is* the liveness proof, no separate timer needed. And each burst's
//! summary is committed to NVS the moment it completes, so a console
//! that drops at minute 12 costs the raw per-second lines and nothing
//! else.
//!
//! The summaries go in namespace `rtcburst`, **not** `rtcdrift`. The
//! latter holds the 8 h run's only copy of a measurement that cost a
//! day, and an estimator experiment has no business writing there —
//! which is also how that store got a stray sample appended when a
//! flash-read rebooted the board mid-collection.
//!
//! ## Running it
//!
//! No WiFi, no NTP, no radio: RTC-vs-crystal needs neither. Which also
//! means nothing can void the run except the RTC itself stopping.
//!
//! ```sh
//! source ~/export-esp.sh
//! cargo build --release --bin rtc-burst
//! ../scripts/capture.sh target/xtensa-esp32s3-espidf/release/rtc-burst \
//!     logs/rtc_burst_$(date +%Y-%m-%d).log 1100 "burst 2 —"
//! ```

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::hal::delay::FreeRtos;
use mfsk_core_m5stack_cores3_app as app;

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use mipidsi::{models::ILI9342CRgb565, options::ColorInversion, Builder};

/// Seconds per burst. Five minutes at one edge per second is N = 300.
const BURST_S: u64 = match option_env!("MFSK_RTC_BURST_S") {
    Some(s) => parse_u64(s.as_bytes(), 300),
    None => 300,
};

/// How many bursts to run. Three is the smallest number that shows
/// repeatability rather than a single value with nothing to compare
/// against.
const BURSTS: u64 = match option_env!("MFSK_RTC_BURSTS") {
    Some(s) => parse_u64(s.as_bytes(), 3),
    None => 3,
};

/// Where burst summaries go. Deliberately not `rtcdrift` — see the
/// module doc.
const NVS_NS: &str = "rtcburst";

const fn parse_u64(b: &[u8], fallback: u64) -> u64 {
    let mut v = 0u64;
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c < b'0' || c > b'9' {
            return fallback;
        }
        v = v * 10 + (c - b'0') as u64;
        i += 1;
    }
    if v == 0 {
        fallback
    } else {
        v
    }
}

fn mono_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// How the seconds register is watched while waiting for it to change.
#[derive(Clone, Copy, PartialEq)]
enum Poll {
    /// What `rtc_drift` does: `FreeRtos::delay_ms(4)`, which on a
    /// 100 Hz tick is a 10 ms sleep. Kept so the fix has something to
    /// be better than.
    Tick,
    /// `esp_rom_delay_us(200)`, a busy ROM delay that does not go
    /// through the scheduler.
    Busy,
}

impl Poll {
    fn label(self) -> &'static str {
        match self {
            Poll::Tick => "tick",
            Poll::Busy => "busy",
        }
    }
}

/// Block until the RTC's seconds register changes; return its new
/// value and the monotonic time the change was observed.
fn wait_edge(
    i2c: &mut esp_idf_hal::i2c::I2cDriver<'static>,
    poll: Poll,
    timeout_ms: u64,
) -> Option<(i64, i64)> {
    let first = app::rtc::read_epoch(i2c)?;
    let deadline = mono_us() + (timeout_ms * 1_000) as i64;
    loop {
        if let Some(now) = app::rtc::read_epoch(i2c) {
            if now != first {
                return Some((now, mono_us()));
            }
        }
        if mono_us() > deadline {
            log::error!("rtc-burst: RTC did not tick within {timeout_ms} ms");
            return None;
        }
        match poll {
            Poll::Tick => FreeRtos::delay_ms(4),
            // SAFETY: a ROM busy-wait with no preconditions.
            Poll::Busy => unsafe { esp_idf_svc::sys::esp_rom_delay_us(200) },
        }
    }
}

/// Running least-squares of `y` (µs of RTC-minus-crystal error)
/// against `x` (seconds). Sums rather than a sample buffer: 300 points
/// need no allocation, and the same code then runs for an overnight
/// burst without a capacity to overflow.
#[derive(Default)]
struct Fit {
    n: f64,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
    syy: f64,
}

impl Fit {
    fn push(&mut self, x: f64, y: f64) {
        self.n += 1.0;
        self.sx += x;
        self.sy += y;
        self.sxx += x * x;
        self.sxy += x * y;
        self.syy += y * y;
    }

    /// `(ppm, standard error in ppm, residual RMS in µs)`.
    ///
    /// With `y` in µs and `x` in s, the slope is µs per second, which
    /// is ppm — the unit conversion is the identity.
    fn result(&self) -> Option<(f64, f64, f64)> {
        if self.n < 3.0 {
            return None;
        }
        let cxx = self.sxx - self.sx * self.sx / self.n;
        let cxy = self.sxy - self.sx * self.sy / self.n;
        let cyy = self.syy - self.sy * self.sy / self.n;
        if cxx <= 0.0 {
            return None;
        }
        let slope = cxy / cxx;
        let sse = (cyy - slope * cxy).max(0.0);
        let var = sse / (self.n - 2.0);
        Some((slope, (var / cxx).sqrt(), var.sqrt()))
    }
}

// ──────────────────────────────────────────────────────────────────
// The record that survives a dropped console

/// One completed burst, 32 bytes, append-only.
///
/// Only the fit is kept, not the 300 points behind it: the summary is
/// what the console might lose, and it is also everything the
/// comparison against the 8 h run needs.
#[derive(Clone, Copy)]
struct Record {
    run_id: u16,
    burst: u16,
    poll_busy: u8,
    n: u16,
    /// RTC's own epoch at the burst's first edge. There is no NTP in
    /// this bin, so this is the only wall-clock there is — it carries
    /// the RTC's absolute error and is for ordering runs, not for
    /// timing anything.
    rtc_start: i64,
    ppm: f32,
    se_ppm: f32,
    rms_us: f32,
}

impl Record {
    fn to_bytes(self) -> [u8; 32] {
        let mut b = [0u8; 32];
        b[0..2].copy_from_slice(&self.run_id.to_le_bytes());
        b[2..4].copy_from_slice(&self.burst.to_le_bytes());
        b[4] = self.poll_busy;
        b[6..8].copy_from_slice(&self.n.to_le_bytes());
        b[8..16].copy_from_slice(&self.rtc_start.to_le_bytes());
        b[16..20].copy_from_slice(&self.ppm.to_le_bytes());
        b[20..24].copy_from_slice(&self.se_ppm.to_le_bytes());
        b[24..28].copy_from_slice(&self.rms_us.to_le_bytes());
        b
    }

    fn from_bytes(b: &[u8; 32]) -> Self {
        let u16at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let f32at = |i: usize| f32::from_le_bytes(b[i..i + 4].try_into().expect("4 bytes"));
        Self {
            run_id: u16at(0),
            burst: u16at(2),
            poll_busy: b[4],
            n: u16at(6),
            rtc_start: i64::from_le_bytes(b[8..16].try_into().expect("8 bytes")),
            ppm: f32at(16),
            se_ppm: f32at(20),
            rms_us: f32at(24),
        }
    }
}

fn key_for(i: usize) -> heapless::String<8> {
    let mut k: heapless::String<8> = heapless::String::new();
    use core::fmt::Write as _;
    let _ = write!(&mut k, "b{i:03}");
    k
}

fn load_all(nvs: &EspNvs<NvsDefault>) -> heapless::Vec<Record, 200> {
    let mut out = heapless::Vec::new();
    let mut buf = [0u8; 32];
    for i in 0..out.capacity() {
        match nvs.get_blob(&key_for(i), &mut buf) {
            Ok(Some(_)) => {
                if out.push(Record::from_bytes(&buf)).is_err() {
                    break;
                }
            }
            _ => break,
        }
    }
    out
}

/// Print every stored burst. Runs before anything else, so attaching a
/// console at any later time collects what earlier runs concluded.
fn report(records: &[Record]) {
    if records.is_empty() {
        log::info!("rtc-burst: no stored bursts yet");
        return;
    }
    log::info!("rtc-burst: {} stored burst(s)", records.len());
    log::info!("rtc-burst: csv,idx,run,burst,poll,n,rtc_start,ppm,se_ppm,rms_us");
    for (i, r) in records.iter().enumerate() {
        log::info!(
            "rtc-burst: csv,{i},{},{},{},{},{},{:.3},{:.3},{:.1}",
            r.run_id,
            r.burst,
            if r.poll_busy == 1 { "busy" } else { "tick" },
            r.n,
            r.rtc_start,
            r.ppm,
            r.se_ppm,
            r.rms_us
        );
    }
}

// ──────────────────────────────────────────────────────────────────
// The panel
//
// Same bring-up `rtc_drift`/`lcd_minimal` use, inline because this bin
// has no business pulling in the app's display task.

struct Screen<D> {
    display: D,
}

impl<D> Screen<D>
where
    D: DrawTarget<Color = Rgb565>,
{
    fn line(&mut self, row: i32, text: &str, fg: Rgb565) {
        let y = row * 22;
        let _ = Rectangle::new(Point::new(0, y), Size::new(320, 22))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(&mut self.display);
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(fg)
            .background_color(Rgb565::BLACK)
            .build();
        let _ = Text::with_baseline(text, Point::new(4, y + 1), style, Baseline::Top)
            .draw(&mut self.display);
    }
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("=== rtc-burst === {BURSTS} burst(s) of {BURST_S} s, one edge per second");
    log::info!("rtc-burst: burst 0 polls the old way (tick), the rest busy-poll");

    let peripherals = Peripherals::take().expect("peripherals");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition");
    let nvs = EspNvs::new(nvs_part, NVS_NS, true).expect("burst NVS");

    // Everything earlier runs concluded, before this one can add to it.
    let stored = load_all(&nvs);
    report(&stored);
    let mut next_idx = stored.len();
    let run_id = stored.iter().map(|r| r.run_id).max().map_or(0, |m| m + 1);

    let mut i2c = match app::pmic::init(
        peripherals.i2c0,
        peripherals.pins.gpio12,
        peripherals.pins.gpio11,
    ) {
        Ok(i2c) => i2c,
        Err(e) => {
            log::error!("rtc-burst: PMIC/I2C init failed: {e:#} — no RTC to read");
            loop {
                FreeRtos::delay_ms(60_000);
            }
        }
    };

    let mut screen = {
        let driver = SpiDriver::new(
            peripherals.spi2,
            peripherals.pins.gpio36,
            peripherals.pins.gpio37,
            Option::<AnyIOPin>::None,
            &SpiDriverConfig::new(),
        )
        .expect("SPI2");
        let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
        let spi_dev =
            SpiDeviceDriver::new(driver, Some(peripherals.pins.gpio3), &spi_cfg).expect("SPI dev");
        let dc = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio35).expect("DC");
        let di = SPIInterface::new(spi_dev, dc);
        let mut delay = Ets;
        let display = Builder::new(ILI9342CRgb565, di)
            .display_size(320, 240)
            .invert_colors(ColorInversion::Inverted)
            .init(&mut delay)
            .expect("mipidsi init");
        Screen { display }
    };
    {
        use core::fmt::Write as _;
        let mut l: heapless::String<40> = heapless::String::new();
        let _ = write!(&mut l, "RTC BURST  run {run_id}");
        screen.line(0, &l, Rgb565::WHITE);
        let mut l2: heapless::String<40> = heapless::String::new();
        let _ = write!(&mut l2, "{BURSTS} x {BURST_S}s, {} stored", stored.len());
        screen.line(1, &l2, Rgb565::CSS_GRAY);
    }

    for burst in 0..BURSTS {
        let poll = if burst == 0 { Poll::Tick } else { Poll::Busy };
        log::info!(
            "rtc-burst: burst {burst} starting ({} poll, {BURST_S} s)",
            poll.label()
        );
        {
            use core::fmt::Write as _;
            let mut l: heapless::String<40> = heapless::String::new();
            let _ = write!(&mut l, "burst {}/{BURSTS} {} poll", burst + 1, poll.label());
            screen.line(2, &l, Rgb565::WHITE);
            screen.line(3, "waiting for first edge", Rgb565::CSS_GRAY);
            screen.line(4, "", Rgb565::WHITE);
        }

        let Some((rtc0, mono0)) = wait_edge(&mut i2c, poll, 5_000) else {
            screen.line(3, "RTC STOPPED - run void", Rgb565::RED);
            break;
        };
        let mut fit = Fit::default();
        let mut last_mono = mono0;
        loop {
            // Yield for most of the second so the idle task runs and
            // the task watchdog stays quiet; only the last ~100 ms is
            // spent polling hard.
            FreeRtos::delay_ms(900);
            let Some((rtc, mono)) = wait_edge(&mut i2c, poll, 5_000) else {
                break;
            };
            let x = (mono - mono0) as f64 / 1e6;
            let y = ((rtc - rtc0) * 1_000_000 - (mono - mono0)) as f64;
            fit.push(x, y);
            log::info!("rtc-burst: b{burst},{x:.6},{y:.0}");
            last_mono = mono;

            // The heartbeat *is* the sample count: one edge per second,
            // so a screen that stops advancing is a board that stopped
            // measuring. Nothing else needs to tick.
            {
                use core::fmt::Write as _;
                let mut l: heapless::String<40> = heapless::String::new();
                let _ = write!(
                    &mut l,
                    "n={:<4} t=+{:>4.0}s {}",
                    fit.n as u32,
                    x,
                    if (fit.n as u32) % 2 == 0 { "*" } else { " " }
                );
                screen.line(3, &l, Rgb565::WHITE);
                if let Some((ppm, se, rms)) = fit.result() {
                    let mut l2: heapless::String<40> = heapless::String::new();
                    let _ = write!(&mut l2, "{ppm:+.2} +-{se:.2} ppm",);
                    screen.line(4, &l2, Rgb565::CSS_YELLOW);
                    let mut l3: heapless::String<40> = heapless::String::new();
                    let _ = write!(&mut l3, "rms {:.2} ms", rms / 1000.0);
                    screen.line(5, &l3, Rgb565::CSS_GRAY);
                }
            }

            if x >= BURST_S as f64 {
                break;
            }
        }
        match fit.result() {
            Some((ppm, se, rms)) => {
                log::info!(
                    "rtc-burst: burst {burst} — {ppm:+.2} ppm ±{se:.2} (1σ), \
                     residual RMS {:.2} ms, n={}, span {:.1} s, poll {}",
                    rms / 1000.0,
                    fit.n as u32,
                    (last_mono - mono0) as f64 / 1e6,
                    poll.label(),
                );
                // Committed here, not at the end of the campaign: a
                // console that drops at minute 12 then costs the raw
                // per-second lines and nothing else.
                let rec = Record {
                    run_id,
                    burst: burst as u16,
                    poll_busy: u8::from(poll == Poll::Busy),
                    n: fit.n as u16,
                    rtc_start: rtc0,
                    ppm: ppm as f32,
                    se_ppm: se as f32,
                    rms_us: rms as f32,
                };
                if let Err(e) = nvs.set_blob(&key_for(next_idx), &rec.to_bytes()) {
                    log::warn!("rtc-burst: NVS write of burst {burst} failed: {e}");
                } else {
                    next_idx += 1;
                }
            }
            None => log::warn!("rtc-burst: burst {burst} — too few edges to fit"),
        }
    }

    log::info!("rtc-burst: done — compare against the 8 h run's -59.46 ppm (+-0.34)");
    report(&load_all(&nvs));
    screen.line(2, "DONE", Rgb565::GREEN);
    loop {
        FreeRtos::delay_ms(60_000);
    }
}
