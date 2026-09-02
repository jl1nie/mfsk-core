// SPDX-License-Identifier: GPL-3.0-or-later
//! How fast do this board's clocks drift? (issue #354)
//!
//! The operating model this measurement decides: **set the clock at
//! home over WiFi, then run a field session with no network at all.**
//! Whether that works is a number nobody in this repo has measured —
//! a "3 hour resync interval" was written into a draft on the strength
//! of an assumed 20 ppm and no evidence.
//!
//! Two clocks matter and they are not the same clock:
//!
//! - the **BM8563 RTC**, battery-backed, which is what survives a
//!   power cycle and therefore what a cold start in the field reads;
//! - the **system clock**, disciplined by NTP at boot and free-running
//!   on the 40 MHz crystal after that, which is what the slot grid
//!   actually counts on during a session.
//!
//! ## Why the RTC is read at a tick edge
//!
//! The BM8563 reports whole seconds. Comparing its value against a
//! reference twice an hour apart quantises to ±1 s, i.e. ±280 ppm over
//! an hour — useless for a question whose answer is in the tens of
//! ppm.
//!
//! So this polls the seconds register every few milliseconds and
//! records the monotonic timestamp at which it *changes*. That turns a
//! 1 s quantum into a ~10 ms one, and an hour's run resolves ~3 ppm.
//!
//! ## Method
//!
//! 1. WiFi up, NTP sync → the system clock is truth. Drop the SNTP
//!    handle so it cannot re-discipline behind the measurement.
//! 2. Catch an RTC tick edge: record `(rtc_epoch, mono_us, sys_us)`.
//! 3. Sleep, logging an intermediate reading every
//!    `MFSK_RTC_DRIFT_LOG_S` so a run cut short still says something.
//! 4. Catch another tick edge.
//! 5. NTP again → truth at the far end.
//!
//! Then, over the true interval:
//!
//! ```text
//! crystal ppm = (mono_elapsed  − true_elapsed) / true_elapsed × 1e6
//! rtc ppm     = (rtc_elapsed   − true_elapsed) / true_elapsed × 1e6
//! ```
//!
//! ## Running it
//!
//! Needs `cfg.toml` with WiFi credentials, and no radio attached (this
//! bin never installs the USB host, so the console stays).
//!
//! ```sh
//! source ~/export-esp.sh
//! MFSK_RTC_DRIFT_RUN_S=3600 cargo build --release --bin rtc-drift
//! ../scripts/capture.sh target/xtensa-esp32s3-espidf/release/rtc-drift \
//!     logs/rtc_drift_$(date +%Y-%m-%d).log 3900 "drift result"
//! ```

use std::time::{SystemTime, UNIX_EPOCH};

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
use mipidsi::{models::ILI9342CRgb565, options::ColorInversion, Builder};

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use mfsk_core_m5stack_cores3_app as app;

/// NVS namespace for the drift samples.
///
/// **Why the samples are persisted at all.** The first attempt logged
/// to the serial console and died at +1200 s of a 3 600 s run with
/// `Error: Broken pipe` — the board's USB CDC dropped, as it does, and
/// took an hour of measurement with it. A drift measurement is hours
/// long by nature: the resolution is NTP's accuracy divided by the
/// window, so the answer improves with time and the transport must not
/// be what ends the run.
///
/// NVS rather than LittleFS because `flash_log`/`adif` are Phase-0.5
/// placeholders whose own doc says the `EspLittleFs` API names were
/// never confirmed, and the `littlefs` partition has no component
/// registered against it. A 24-byte record every few minutes is a
/// handful of KB — NVS is the right size of tool, and it is already
/// mounted.
const NVS_NS: &str = "rtcdrift";

/// One tick-edge observation: the RTC's own second, the monotonic
/// timestamp of the edge, and the system clock at that moment.
#[derive(Clone, Copy)]
struct Sample {
    mono_us: i64,
    rtc_epoch: i64,
    sys_us: i64,
}

impl Sample {
    fn to_bytes(self) -> [u8; 24] {
        let mut b = [0u8; 24];
        b[0..8].copy_from_slice(&self.mono_us.to_le_bytes());
        b[8..16].copy_from_slice(&self.rtc_epoch.to_le_bytes());
        b[16..24].copy_from_slice(&self.sys_us.to_le_bytes());
        b
    }

    fn from_bytes(b: &[u8; 24]) -> Self {
        let g = |r: core::ops::Range<usize>| i64::from_le_bytes(b[r].try_into().expect("8 bytes"));
        Self {
            mono_us: g(0..8),
            rtc_epoch: g(8..16),
            sys_us: g(16..24),
        }
    }
}

fn key_for(i: usize) -> heapless::String<8> {
    let mut k: heapless::String<8> = heapless::String::new();
    use core::fmt::Write as _;
    let _ = write!(&mut k, "s{i:03}");
    k
}

fn store(nvs: &mut EspNvs<NvsDefault>, i: usize, s: Sample) {
    if let Err(e) = nvs.set_blob(&key_for(i), &s.to_bytes()) {
        log::warn!("rtc-drift: NVS write of sample {i} failed: {e}");
    }
}

/// Read back whatever a previous run left, stopping at the first gap.
fn load_all(nvs: &EspNvs<NvsDefault>) -> heapless::Vec<Sample, 256> {
    let mut out = heapless::Vec::new();
    let mut buf = [0u8; 24];
    for i in 0..out.capacity() {
        match nvs.get_blob(&key_for(i), &mut buf) {
            Ok(Some(_)) => {
                if out.push(Sample::from_bytes(&buf)).is_err() {
                    break;
                }
            }
            _ => break,
        }
    }
    out
}

/// Print a stored run as CSV, and what it adds up to.
///
/// Printed on **every** boot before a new run starts, so plugging the
/// console in at any later time is enough to collect the result — the
/// measurement does not have to be watched while it happens.
fn report(samples: &[Sample]) {
    if samples.len() < 2 {
        log::info!("rtc-drift: no stored run (need two samples to say anything)");
        return;
    }
    log::info!("rtc-drift: stored run — {} samples", samples.len());
    log::info!("rtc-drift: csv,idx,mono_us,rtc_epoch,sys_us");
    for (i, s) in samples.iter().enumerate() {
        log::info!(
            "rtc-drift: csv,{i},{},{},{}",
            s.mono_us,
            s.rtc_epoch,
            s.sys_us
        );
    }
    // **Reboots split the record.** `mono_us` restarts at zero on
    // every boot, so a run that spans a power cycle — moving the board
    // from a PC to a USB adapter, or a blip overnight — is several
    // segments, not one. Report the longest, which is the one that
    // measures anything.
    let mut segments: heapless::Vec<(usize, usize), 16> = heapless::Vec::new();
    let mut start = 0usize;
    for i in 1..samples.len() {
        if samples[i].mono_us < samples[i - 1].mono_us {
            let _ = segments.push((start, i));
            start = i;
        }
    }
    let _ = segments.push((start, samples.len()));
    if segments.len() > 1 {
        log::info!(
            "rtc-drift: {} segments (the board rebooted {} time(s) during the record)",
            segments.len(),
            segments.len() - 1
        );
    }
    let (lo, hi) = segments
        .iter()
        .copied()
        .max_by_key(|(lo, hi)| samples[hi - 1].mono_us - samples[*lo].mono_us)
        .unwrap_or((0, samples.len()));
    if hi - lo < 2 {
        log::info!("rtc-drift: longest segment has one sample — nothing to measure yet");
        return;
    }
    let a = samples[lo];
    let b = samples[hi - 1];
    let d_mono = b.mono_us - a.mono_us;
    let d_rtc = (b.rtc_epoch - a.rtc_epoch) * 1_000_000;
    let d_sys = b.sys_us - a.sys_us;
    let ppm = |x: i64, base: i64| -> f64 {
        if base == 0 {
            0.0
        } else {
            (x - base) as f64 / base as f64 * 1e6
        }
    };
    // `sys` is NTP-disciplined at both ends of a completed run, so it
    // is the reference; `mono` is the crystal alone.
    log::info!(
        "rtc-drift: over {} s — RTC {:+.2} ppm vs system, crystal {:+.2} ppm vs system, \
         RTC {:+.2} ppm vs crystal",
        d_sys / 1_000_000,
        ppm(d_rtc, d_sys),
        ppm(d_mono, d_sys),
        ppm(d_rtc, d_mono),
    );
    log::info!(
        "rtc-drift: an 8 h session would put the RTC {:+} ms out; one FT4 symbol is 48 ms",
        (ppm(d_rtc, d_sys) * 8.0 * 3600.0) as i64,
    );
}

/// How long to let the clocks run apart. Longer is better: the
/// resolution is set by NTP's own accuracy (tens of ms) divided by
/// this, so an hour resolves single-digit ppm and ten minutes does
/// not.
const RUN_S: u64 = match option_env!("MFSK_RTC_DRIFT_RUN_S") {
    Some(s) => parse_u64(s.as_bytes(), 3_600),
    None => 3_600,
};

/// Intermediate reading cadence, so a capture that is cut short still
/// carries data.
const LOG_S: u64 = match option_env!("MFSK_RTC_DRIFT_LOG_S") {
    Some(s) => parse_u64(s.as_bytes(), 300),
    None => 300,
};

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

/// What the screen has to say, and why it says it.
///
/// An 8 h measurement on a board with a dark screen is
/// indistinguishable from a crashed one, and — worse — **an NTP
/// failure invalidates the whole run**, which is exactly the kind of
/// thing that must not be discoverable only in a console that has
/// already dropped twice today. So the panel carries the two facts
/// that decide whether the run is worth anything (WiFi, NTP) above the
/// numbers it is collecting, and a heartbeat that moves every second
/// so "waiting for the next sample" cannot be mistaken for "hung".
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

fn mono_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn sys_us() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as i64)
        .unwrap_or(0)
}

/// Block until the RTC's seconds register changes, and report the
/// reading and the monotonic time at which it did.
///
/// Returns `None` if the RTC never ticks within `timeout_ms` — a flat
/// backup cell reads as a stopped clock rather than as an error.
fn wait_rtc_tick(
    i2c: &mut esp_idf_hal::i2c::I2cDriver<'static>,
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
            log::error!("rtc-drift: RTC did not tick within {timeout_ms} ms");
            return None;
        }
        // Fine enough to keep the edge to a few ms, coarse enough not
        // to hold the I2C bus solid.
        FreeRtos::delay_ms(4);
    }
}

fn sync_ntp(server: &str) -> bool {
    match mfsk_app_shared::ntp::start(server) {
        Ok(sntp) => {
            let ok = mfsk_app_shared::ntp::wait_synced(&sntp, 30_000);
            // Dropped here on purpose: SNTP polls again on its own
            // schedule, and a second discipline inside the measurement
            // window would erase exactly what is being measured.
            drop(sntp);
            ok
        }
        Err(e) => {
            log::error!("rtc-drift: NTP start failed: {e:#}");
            false
        }
    }
}

/// `MFSK_RTC_DRIFT_RESET=1` — discard a stored run and start fresh.
/// Without it a reboot continues appending, which is what makes an
/// interrupted overnight run recoverable rather than wasted.
const RESET: bool = match option_env!("MFSK_RTC_DRIFT_RESET") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("=== rtc-drift === run {RUN_S} s, sample every {LOG_S} s");

    let peripherals = Peripherals::take().expect("peripherals");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition");
    let mut nvs = EspNvs::new(nvs_part.clone(), NVS_NS, true).expect("drift NVS");

    // Whatever a previous run left, printed before anything else: the
    // console can be attached hours after the fact and still collect
    // the result.
    let stored = load_all(&nvs);
    report(&stored);
    let mut next_idx = if RESET {
        for i in 0..stored.len() {
            let _ = nvs.remove(&key_for(i));
        }
        log::info!(
            "rtc-drift: MFSK_RTC_DRIFT_RESET=1 — {} samples discarded",
            stored.len()
        );
        0
    } else {
        stored.len()
    };

    let mut i2c = match app::pmic::init(
        peripherals.i2c0,
        peripherals.pins.gpio12,
        peripherals.pins.gpio11,
    ) {
        Ok(i2c) => i2c,
        Err(e) => {
            log::error!("rtc-drift: PMIC/I2C init failed: {e:#} — no RTC to read");
            loop {
                FreeRtos::delay_ms(60_000);
            }
        }
    };

    // The panel. Same bring-up `lcd_minimal` uses, kept inline because
    // this bin has no business pulling in the app's display task.
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
    screen.line(0, "RTC DRIFT", Rgb565::WHITE);
    screen.line(1, "wifi: connecting", Rgb565::CSS_GRAY);
    screen.line(2, "ntp : waiting", Rgb565::CSS_GRAY);

    // NTP is what makes `sys_us` a reference rather than a guess. It is
    // wanted at both ends of the run and nowhere in between — SNTP is
    // dropped after each sync so it cannot re-discipline the clock
    // inside the window being measured.
    let ntp_server = {
        let n = mfsk_app_shared::settings::open_nvs(nvs_part.clone()).expect("settings NVS");
        mfsk_app_shared::settings::load(&n).ntp_server
    };
    let synced = bring_up_network_and_sync(peripherals.modem, nvs_part, &ntp_server);
    // The two lines that decide whether the run is worth anything.
    // Red, and left on the screen for the whole eight hours, because a
    // failure here is not something to discover afterwards.
    if app::WIFI_SSID.is_empty() {
        screen.line(1, "wifi: NO CREDENTIALS", Rgb565::RED);
    } else {
        screen.line(1, "wifi: up", Rgb565::GREEN);
    }
    if synced {
        screen.line(2, "ntp : SYNCED", Rgb565::GREEN);
    } else {
        screen.line(2, "ntp : FAILED - run is void", Rgb565::RED);
    }
    if !synced {
        log::warn!(
            "rtc-drift: no NTP — samples still land in NVS and RTC-vs-crystal is still \
             measurable, but nothing here can separate the two from true time"
        );
    }

    let mut waited = 0u64;
    loop {
        let Some((rtc_epoch, mono_us)) = wait_rtc_tick(&mut i2c, 5_000) else {
            log::error!("rtc-drift: RTC stopped ticking at +{waited}s");
            break;
        };
        let sample = Sample {
            mono_us,
            rtc_epoch,
            sys_us: sys_us(),
        };
        store(&mut nvs, next_idx, sample);
        let base = load_all(&nvs);
        if let Some(first) = base.first() {
            let d_mono = sample.mono_us - first.mono_us;
            let d_rtc = (sample.rtc_epoch - first.rtc_epoch) * 1_000_000;
            let ppm = if d_mono != 0 {
                (d_rtc - d_mono) as f64 / d_mono as f64 * 1e6
            } else {
                0.0
            };
            log::info!(
                "rtc-drift: sample {next_idx} at +{}s — rtc {rtc_epoch}, \
                 RTC-vs-crystal {ppm:+.1} ppm ({:+} ms)",
                d_mono / 1_000_000,
                (d_rtc - d_mono) / 1_000,
            );
            {
                use core::fmt::Write as _;
                let mut l: heapless::String<40> = heapless::String::new();
                let _ = write!(
                    &mut l,
                    "n={:<3} t=+{}h{:02}m",
                    next_idx + 1,
                    d_mono / 3_600_000_000,
                    (d_mono / 60_000_000) % 60
                );
                screen.line(3, &l, Rgb565::WHITE);
                let mut l2: heapless::String<40> = heapless::String::new();
                let _ = write!(
                    &mut l2,
                    "rtc {ppm:+.1} ppm ({:+} ms)",
                    (d_rtc - d_mono) / 1_000
                );
                screen.line(4, &l2, Rgb565::CSS_YELLOW);
            }
        }
        next_idx += 1;
        if next_idx >= 250 {
            log::info!("rtc-drift: sample store full — stopping");
            break;
        }
        if waited >= RUN_S {
            break;
        }
        let step = LOG_S.min(RUN_S - waited);
        // Second-by-second, so the panel proves the board is waiting
        // rather than hung. `delay_ms(600_000)` would leave it frozen
        // for ten minutes at a time.
        for left in (0..step).rev() {
            let mut l: heapless::String<40> = heapless::String::new();
            use core::fmt::Write as _;
            let _ = write!(
                &mut l,
                "next in {left:4}s {}",
                if left % 2 == 0 { "*" } else { " " }
            );
            screen.line(5, &l, Rgb565::CSS_GRAY);
            FreeRtos::delay_ms(1_000);
        }
        waited += step;
    }

    // Close the bracket: truth at the far end, then the summary the
    // whole run exists for.
    if synced && sync_ntp(&ntp_server) {
        let Some((rtc_epoch, mono_us)) = wait_rtc_tick(&mut i2c, 5_000) else {
            loop {
                FreeRtos::delay_ms(60_000);
            }
        };
        store(
            &mut nvs,
            next_idx,
            Sample {
                mono_us,
                rtc_epoch,
                sys_us: sys_us(),
            },
        );
    }
    report(&load_all(&nvs));

    loop {
        FreeRtos::delay_ms(60_000);
    }
}

/// WiFi up and one NTP sync, or `false` if either is unavailable.
fn bring_up_network_and_sync<M>(modem: M, nvs_part: EspDefaultNvsPartition, server: &str) -> bool
where
    M: esp_idf_svc::hal::modem::WifiModemPeripheral + 'static,
{
    if app::WIFI_SSID.is_empty() {
        log::warn!("rtc-drift: no cfg.toml WiFi credentials");
        return false;
    }
    let sysloop = EspSystemEventLoop::take().expect("sysloop");
    let mut driver = match mfsk_app_shared::wifi::wifi_driver_init(modem, sysloop, Some(nvs_part)) {
        Ok(d) => d,
        Err(e) => {
            log::error!("rtc-drift: WiFi driver init failed: {e:#}");
            return false;
        }
    };
    match mfsk_app_shared::wifi::connect_with_retry(
        &mut driver,
        app::WIFI_SSID,
        app::WIFI_PSK,
        Some(8),
    ) {
        Ok(info) => {
            log::info!("rtc-drift: WiFi up, ip {}", info.ip);
            // The driver has to outlive the association; this task
            // never returns, so leaking it is the whole lifetime.
            core::mem::forget(driver);
        }
        Err(e) => {
            log::error!("rtc-drift: no association: {e:#}");
            return false;
        }
    }
    sync_ntp(server)
}
