//! Transmit bring-up probes: the hardware questions to answer before a
//! transmit engine is built on top of them (plan stage P1).
//!
//! Compile-time opt-in, `MFSK_CORES3_TX_BRINGUP=<stage>`; unset, [`start`]
//! returns at once and nothing here runs. Each stage waits until the
//! receiver is streaming, so what it measures is the receiver *with* the
//! transmit side beside it, and logs `txb:` lines next to the reader's
//! own 1 Hz `uac: rx tick` (which already carries `sa/s`, RMS and the
//! internal heap's free total and largest block).
//!
//! - **`a` — OUT beside IN and CI-V, no RF.** Opens the radio's audio
//!   OUT interface as mono 16-bit 48 kHz, with two ring sizes in turn,
//!   streams one FT8 frame of digital silence through the real
//!   `GfskStream` + 4x hold path, holds the stream open and idle, and
//!   closes it. Never keys PTT, and forces amplitude 0 whatever
//!   `MFSK_CORES3_TX_AMPLITUDE` says. Answers: does the OUT pipe get a
//!   host channel with CI-V open (eight on this chip), what it costs in
//!   internal DRAM, and whether the receiver keeps 12 000 sa/s.
//! - **`b` — PTT only.** Keys the radio over CI-V with no audio, for
//!   15 s and then 35 s, and unkeys. Answers: how long the `FB` and the
//!   `1C 00` readback take, and what the IN stream does while the
//!   radio transmits — whether the reader's 3 s stall watchdog ends the
//!   session. **This transmits.** Run it only with the radio on a dummy
//!   load at minimum power, `PTT SOURCE` not `VOX`.
//!
//! Every key-down is bounded by [`MAX_KEY_S`] and unkeyed by a drop
//! guard, so a panic in between still sends PTT off.

use crate::uac;
use esp_idf_svc::sys;
use std::time::Duration;

const STAGE: Option<&str> = option_env!("MFSK_CORES3_TX_BRINGUP");

/// No key-down here lasts longer than this, whatever a stage asks for.
const MAX_KEY_S: u32 = 40;

/// How long to wait for the receiver, the OUT interface and (stage `b`)
/// CI-V before giving up. The radio is normally streaming ~10 s after
/// boot; CI-V opens after the IN stream by design (`civ_usb`).
const READY_TIMEOUT_S: u32 = 120;

/// Receiver ticks to log before touching anything, as the baseline the
/// rest is compared against.
const BASELINE_S: u64 = 20;

pub fn start() {
    let Some(stage) = STAGE else { return };
    // PSRAM stack: nothing here touches flash. `GfskStream`'s pulse and
    // the chunk buffer are heap.
    if let Err(e) = uac::spawn_psram_thread(c"tx_bringup", 8192, Some(4), None, move || run(stage))
    {
        log::error!("txb: spawn failed: {e}");
    }
}

fn run(stage: &str) {
    log::warn!("txb: bring-up stage {stage:?} armed — waiting for the receiver");
    let need_cat = stage == "b";
    if !wait_ready(need_cat) {
        log::error!("txb: not ready after {READY_TIMEOUT_S} s — stage {stage:?} not run");
        return;
    }
    log::warn!("txb: ready; {BASELINE_S} s of baseline");
    std::thread::sleep(Duration::from_secs(BASELINE_S));
    match stage {
        "a" => stage_a(),
        "b" => stage_b(),
        other => log::error!("txb: unknown stage {other:?} (a | b)"),
    }
    log::warn!("txb: stage {stage:?} done");
}

fn wait_ready(need_cat: bool) -> bool {
    for _ in 0..READY_TIMEOUT_S {
        let (st, sa, _) = uac::status();
        let rx = st == uac::UacState::Streaming && sa > 0;
        let out = uac::tx_iface().is_some();
        let cat = !need_cat || crate::civ_usb::is_open();
        if rx && out && cat {
            return true;
        }
        std::thread::sleep(Duration::from_secs(1));
    }
    false
}

fn now_us() -> i64 {
    unsafe { sys::esp_timer_get_time() }
}

/// One line: internal and PSRAM, free total and largest block.
fn log_heap(label: &str) {
    let (int_free, int_lrg, ps_free, ps_lrg) = unsafe {
        let int = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT;
        let ps = sys::MALLOC_CAP_SPIRAM;
        (
            sys::heap_caps_get_free_size(int),
            sys::heap_caps_get_largest_free_block(int),
            sys::heap_caps_get_free_size(ps),
            sys::heap_caps_get_largest_free_block(ps),
        )
    };
    log::warn!("txb: heap {label} int={int_free} lrg={int_lrg} ps={ps_free} pslrg={ps_lrg}");
}

/// One line per second for `secs`, from `uac::status()` — the reader's
/// own tick carries more, this is the summary to grep for.
fn watch_rx(label: &str, secs: u32) {
    for s in 0..secs {
        std::thread::sleep(Duration::from_secs(1));
        let (st, sa, rms) = uac::status();
        log::info!("txb: {label} t={} rx={st:?} {sa} sa/s rms {rms:?}", s + 1);
    }
}

// ---- stage a ------------------------------------------------------------

/// The ring sizes to try. The receiver's is 16 KB; the OUT ring holds
/// what the writer is ahead of the endpoint, and at 96 B/ms 4 KB is
/// 42 ms — two 20 ms chunks.
const OUT_RINGS: [u32; 2] = [4096, 8192];

/// Seconds to hold the stream open and empty after the frame: the
/// state a transmit engine would keep between transmissions if it did
/// not close.
const IDLE_OPEN_S: u32 = 30;

fn stage_a() {
    let Some((addr, iface)) = uac::tx_iface() else {
        log::error!("txb: a: no OUT interface");
        return;
    };
    for ring in OUT_RINGS {
        log::warn!("txb: a: ring {ring} B, addr={addr} iface={iface}");
        log_heap("before-open");
        let cfg = sys::uac::uac_host_device_config_t {
            addr,
            iface_num: iface,
            buffer_size: ring,
            buffer_threshold: ring / 4,
            callback: Some(uac::tx_device_event_cb),
            callback_arg: core::ptr::null_mut(),
        };
        let mut h: sys::uac::uac_host_device_handle_t = core::ptr::null_mut();
        let t0 = now_us();
        // SAFETY: `cfg` outlives the call; `h` is written on success.
        let err = unsafe { sys::uac::uac_host_device_open(&cfg, &mut h) };
        log::warn!("txb: a: open -> {err:#x} in {} us", now_us() - t0);
        if err != sys::ESP_OK {
            continue;
        }
        log_heap("after-open");
        let sc = sys::uac::uac_host_stream_config_t {
            channels: 1,
            bit_resolution: 16,
            sample_freq: 48_000,
            flags: 0,
        };
        let t0 = now_us();
        // SAFETY: `h` is open; `sc` outlives the call.
        let err = unsafe { sys::uac::uac_host_device_start(h, &sc) };
        // ESP_ERR_NOT_FOUND / NOT_SUPPORTED here with CI-V open, where
        // the 2026-09-20 probe (no CI-V) succeeded, is the host-channel
        // count — `hcd_pipe_alloc` logs "No more HCD channels".
        log::warn!(
            "txb: a: start 1ch/16b/48k -> {err:#x} in {} us",
            now_us() - t0
        );
        if err == sys::ESP_OK {
            log_heap("after-start");
            let msg77 = mfsk_core::msg::wsjt77::pack77(
                "CQ",
                crate::decode_pipeline::MY_CALL,
                crate::decode_pipeline::MY_GRID,
            )
            .unwrap_or([0u8; 77]);
            let done0 = uac::TX_DONE_EVENTS.load(core::sync::atomic::Ordering::Relaxed);
            // Amplitude 0 by construction, not by configuration.
            let (chunks, worst, total) = uac::write_ft8_frame(h, &msg77, 1_500.0, 0, 1);
            let done = uac::TX_DONE_EVENTS.load(core::sync::atomic::Ordering::Relaxed) - done0;
            log::warn!(
                "txb: a: frame {chunks}/632 ch worst {worst}us total {}ms/12640 tx_done {done}",
                total / 1_000
            );
            log_heap("after-frame");
            watch_rx("a idle-open", IDLE_OPEN_S);
            // SAFETY: started above.
            let err = unsafe { sys::uac::uac_host_device_stop(h) };
            log::warn!("txb: a: stop -> {err:#x}");
        }
        // SAFETY: opened above, not used after.
        let err = unsafe { sys::uac::uac_host_device_close(h) };
        log::warn!("txb: a: close -> {err:#x}");
        log_heap("after-close");
        watch_rx("a closed", 10);
    }
}

// ---- stage b ------------------------------------------------------------

/// Key-down lengths: an FT8 frame plus margin, then past the 30 s a
/// long JTTY message takes.
const KEY_DOWNS_S: [u32; 2] = [15, 35];
const _: () = assert!(KEY_DOWNS_S[0] <= MAX_KEY_S && KEY_DOWNS_S[1] <= MAX_KEY_S);

/// Readback wait per PTT call.
const PTT_WAIT_MS: u32 = 1_000;

/// Sends PTT off when dropped — on the normal path and on a panic.
struct Unkey;
impl Drop for Unkey {
    fn drop(&mut self) {
        match crate::civ_usb::ptt(false, PTT_WAIT_MS) {
            Ok(t) => log::warn!(
                "txb: b: unkey ack {:?}us readback {:?}us",
                t.ack_us,
                t.readback_us
            ),
            Err(e) => log::error!("txb: b: UNKEY FAILED: {e} — unkey the radio by hand"),
        }
    }
}

fn stage_b() {
    // Known state first; also measures an unkeyed round trip.
    match crate::civ_usb::ptt(false, PTT_WAIT_MS) {
        Ok(t) => log::warn!(
            "txb: b: initial off ack {:?}us readback {:?}us",
            t.ack_us,
            t.readback_us
        ),
        Err(e) => {
            log::error!("txb: b: {e} — not keying");
            return;
        }
    }
    for secs in KEY_DOWNS_S {
        let secs = secs.min(MAX_KEY_S);
        log::warn!("txb: b: KEY {secs} s");
        let guard = Unkey;
        let keyed = match crate::civ_usb::ptt(true, PTT_WAIT_MS) {
            Ok(t) => {
                log::warn!(
                    "txb: b: key ack {:?}us readback {:?}us",
                    t.ack_us,
                    t.readback_us
                );
                t.readback_us.is_some()
            }
            Err(e) => {
                log::error!("txb: b: key failed: {e}");
                false
            }
        };
        if !keyed {
            // Unkeyed by the guard; a radio that did not confirm is not
            // one to keep sending to.
            drop(guard);
            log::error!("txb: b: no PTT readback — stopping");
            return;
        }
        watch_rx("b keyed", secs);
        drop(guard);
        watch_rx("b after", 30);
    }
}
