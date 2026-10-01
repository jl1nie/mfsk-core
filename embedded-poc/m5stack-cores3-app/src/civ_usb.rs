//! CAT over USB: the IC-705's CI-V port (activator plan, stage 3).
//!
//! The IC-705 enumerates as one composite device, VID `0c26` PID
//! `0036`, carrying **two** CDC-ACM functions — control interfaces 0
//! and 2 (data 1 and 3) — beside the PCM2901 audio behind the same
//! internal hub (descriptor dump, `logs/udp_enumdump2_2026-09-20.log`).
//! CI-V is the first, "USB (A)": probed 2026-09-23
//! (`logs/udp_civ_probe_2026-09-23.log`), data interface 1 answered
//! `03` with 7.041 MHz and `26 00` with USB-D FIL1, and sent transceive
//! `00` frames as the dial turned. The radio's USB echo-back was off.
//!
//! What this task does with it:
//!
//! - reads the dial and mode on connect, and keeps the status bar's
//!   `rig_freq_hz` on whatever transceive reports after that — which
//!   is what `all.txt` and the ADIF log take their frequency from;
//! - sends a dial chosen on the FREQ page ([`request_freq`]) as `05`
//!   plus `26 00 01 01 01` (USB, data on, FIL1), and saves it so the
//!   next boot sends it again;
//! - closes on unplug and waits for the radio to come back;
//! - sends PTT off the moment it opens, and keys on request ([`ptt`]).
//!
//! It never touches DTR or RTS. The IC-705's "USB SEND" / "USB Keying"
//! settings can map either line to PTT or CW keying, and
//! `cdc_acm_host_open` leaves both alone unless asked; so does this.

use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys;
use esp_idf_svc::sys::cdc_acm as cdc;
use mfsk_app_shared::civ_frame::{self, Event, IC705_ADDR};
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

const IC705_VID: u16 = 0x0c26;
const IC705_PID: u16 = 0x0036;

/// USB (A)'s **data** interface, opened directly rather than through
/// its control interface (0).
///
/// The ESP32-S3's host controller has eight channels
/// (`OTG_NUM_HOST_CHAN`), one per open pipe. The IC-705 already takes
/// seven: EP0 for the hub, the Icom function and the PCM2901, the
/// hub's interrupt pipe, audio IN and audio OUT. Opening a control
/// interface adds its notification pipe beside the two bulk ones, and
/// on 2026-09-23 that starved the PCM2901, which then failed to
/// enumerate ("No more HCD channels available", `EXT_PORT: [1:4] Port
/// disabled`) and the board rebooted every ~33 s. Given the data
/// interface, `cdc_parse_interface_descriptor` finds no interrupt
/// endpoint and does not treat it as CDC-compliant, so only the two
/// bulk pipes are allocated. The cost is that the driver offers no
/// line-coding / control-line calls on the handle — which this does
/// not want anyway (see the module doc on DTR/RTS).
const CAT_IFACE: u8 = 1;

/// Below the audio path (`uac.rs` runs it at 8) — a CI-V reply that
/// waits a few milliseconds costs nothing, a missed isochronous frame
/// costs audio. Core 1, away from the USB interrupt and the decoder.
const DRIVER_TASK_PRIORITY: u32 = 5;
const DRIVER_TASK_STACK: usize = 4096;
const DRIVER_TASK_CORE: i32 = 1;

/// How often the task looks at its flags. A dial change waits at most
/// this long to go out; the status bar, at most this long to follow.
const POLL_MS: u64 = 200;

/// The dial the RX callback last parsed; 0 for none yet. The callback
/// runs in the class driver's task and only stores — the CAT task
/// carries it to the UI, so the driver never waits on the UI lock.
static RIG_HZ: AtomicU32 = AtomicU32::new(0);
/// A dial from the FREQ page, not yet sent; 0 for none. Latest wins.
static PENDING_HZ: AtomicU32 = AtomicU32::new(0);
/// The dial held in NVS, so an unchanged one is not written again.
static SAVED_HZ: AtomicU32 = AtomicU32::new(0);
static DISCONNECTED: AtomicBool = AtomicBool::new(false);
static NVS: OnceLock<Arc<Mutex<EspNvs<NvsDefault>>>> = OnceLock::new();

static READER: Mutex<civ_frame::Reader> = Mutex::new(civ_frame::Reader::new());

/// The open CI-V handle, null while there is none. Published so [`ptt`]
/// can send from the caller's task instead of waiting out [`POLL_MS`]:
/// `cdc_acm_host_data_tx_blocking` serialises its callers on the
/// device's own `out_mux`, so two tasks sending is safe.
static HDL: AtomicPtr<core::ffi::c_void> = AtomicPtr::new(core::ptr::null_mut());
/// Held by [`ptt`] for its whole exchange and by the CAT task around
/// `cdc_acm_host_close`, so a handle cannot be closed under a sender
/// that loaded it a moment before the unplug.
static HDL_USE: Mutex<()> = Mutex::new(());
/// The radio's last reported PTT state: 0 unknown, 1 receiving, 2
/// transmitting. Written by the RX callback.
static PTT_SEEN: AtomicU8 = AtomicU8::new(0);
/// Bumped on every `FB` from the radio, so a waiter can tell a fresh
/// acknowledgement from an old one.
static OK_SEQ: AtomicU32 = AtomicU32::new(0);
/// Bumped on every PTT readback, for the same reason.
static PTT_SEQ: AtomicU32 = AtomicU32::new(0);

/// Hand over the NVS handle before the host starts; the saved dial is
/// read from it once the radio connects.
pub fn set_nvs(nvs: Arc<Mutex<EspNvs<NvsDefault>>>) {
    let _ = NVS.set(nvs);
}

/// Send `hz` to the rig — the FREQ page's commit. Returns at once; the
/// CAT task sends it within [`POLL_MS`], or when the radio connects.
pub fn request_freq(hz: u32) {
    PENDING_HZ.store(hz, Ordering::Release);
}

/// Install the CDC-ACM class driver and start the CAT task. Call after
/// `usb_host_install` (i.e. after `uac::start_host` succeeds).
pub fn start() {
    let cfg = cdc::cdc_acm_host_driver_config_t {
        driver_task_stack_size: DRIVER_TASK_STACK,
        driver_task_priority: DRIVER_TASK_PRIORITY,
        xCoreID: DRIVER_TASK_CORE,
        new_dev_cb: None,
    };
    // SAFETY: the host library is installed; `cfg` outlives the call.
    let err = unsafe { cdc::cdc_acm_host_install(&cfg) };
    if err != sys::ESP_OK {
        log::warn!("cat: cdc_acm_host_install failed err={err:#x}");
        return;
    }
    // The saved dial, read off this (possibly PSRAM) stack.
    with_nvs(|nvs| {
        if let Some(hz) = mfsk_app_shared::freq_presets::read(nvs) {
            SAVED_HZ.store(hz, Ordering::Release);
            // A dial picked before the read landed stands.
            let _ = PENDING_HZ.compare_exchange(0, hz, Ordering::AcqRel, Ordering::Acquire);
            log::info!("cat: saved dial {hz} Hz — will send it on connect");
        }
    });
    if let Err(e) = crate::uac::spawn_psram_thread(c"cat", 4096, Some(4), None, cat_task) {
        log::warn!("cat: task spawn failed: {e}");
    }
}

/// Run `f` against NVS on a short-lived task with an internal-DRAM
/// stack. An NVS access stops the flash cache, which a PSRAM stack —
/// this task's, and the panel's — must not be holding; same reason as
/// `commit_config_and_restart`.
fn with_nvs(f: impl FnOnce(&EspNvs<NvsDefault>) + Send + 'static) {
    let Some(nvs) = NVS.get().cloned() else {
        log::warn!("cat: no NVS handle");
        return;
    };
    type Job = Box<dyn FnOnce() + Send>;
    let job: Job = Box::new(move || match nvs.lock() {
        Ok(nvs) => f(&nvs),
        Err(e) => log::warn!("cat: NVS lock poisoned: {e}"),
    });
    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `with_nvs` leaked exactly this pointer.
        let job = unsafe { Box::from_raw(arg as *mut Job) };
        job();
        // SAFETY: deletes the calling task; nothing runs after it.
        unsafe { sys::vTaskDelete(core::ptr::null_mut()) };
    }
    let ptr = Box::into_raw(Box::new(job)) as *mut core::ffi::c_void;
    // SAFETY: `entry` takes ownership of `ptr` and frees it.
    let created = unsafe {
        sys::xTaskCreatePinnedToCore(Some(entry), c"cat_nvs".as_ptr(), 4096, ptr, 3, core::ptr::null_mut(), 1)
    };
    if created != 1 {
        // SAFETY: the task was not created, so `ptr` is still ours.
        drop(unsafe { Box::from_raw(ptr as *mut Job) });
        log::warn!("cat: NVS task not created");
    }
}

unsafe extern "C" fn on_data(data: *const u8, len: usize, _arg: *mut core::ffi::c_void) -> bool {
    // SAFETY: the driver hands a buffer of `len` bytes valid for the
    // duration of the callback.
    let bytes = unsafe { core::slice::from_raw_parts(data, len) };
    let Ok(mut reader) = READER.lock() else {
        return true;
    };
    for &b in bytes {
        let Some(frame) = reader.push(b) else { continue };
        // Our own frames come back only with echo-back on; `parse`
        // drops them either way (they are from E0, not the radio).
        match civ_frame::parse(&frame, IC705_ADDR) {
            Some(Event::Freq(hz)) => RIG_HZ.store(hz, Ordering::Release),
            Some(Event::Ng) => log::warn!("cat: rig refused a command (NG)"),
            Some(Event::Ok) => {
                OK_SEQ.fetch_add(1, Ordering::AcqRel);
            }
            Some(Event::Ptt(on)) => {
                PTT_SEEN.store(if on { 2 } else { 1 }, Ordering::Release);
                PTT_SEQ.fetch_add(1, Ordering::AcqRel);
            }
            Some(Event::Mode { mode, data, filter }) => {
                log::info!("cat: mode {mode:#04x} data={data:?} fil{filter}")
            }
            _ => {}
        }
    }
    true
}

unsafe extern "C" fn on_event(ev: *const cdc::cdc_acm_host_dev_event_data_t, _arg: *mut core::ffi::c_void) {
    // SAFETY: the driver passes a valid event for the callback's span.
    let ty = unsafe { (*ev).type_ };
    if ty == cdc::cdc_acm_host_dev_event_t_CDC_ACM_HOST_DEVICE_DISCONNECTED {
        DISCONNECTED.store(true, Ordering::Release);
    } else {
        log::warn!("cat: device event {ty}");
    }
}

fn open() -> Option<cdc::cdc_acm_dev_hdl_t> {
    let cfg = cdc::cdc_acm_host_open_config_t {
        vid: IC705_VID,
        pid: IC705_PID,
        interface_idx: CAT_IFACE,
        dev_addr: 0,
        connection_timeout_ms: 10_000,
        out_buffer_size: 64,
        in_buffer_size: 0,
        event_cb: Some(on_event),
        data_cb: Some(on_data),
        user_arg: core::ptr::null_mut(),
    };
    let mut hdl: cdc::cdc_acm_dev_hdl_t = core::ptr::null_mut();
    // SAFETY: `cfg` outlives the call; `hdl` is written on success.
    let err = unsafe { cdc::cdc_acm_host_open_v2(&cfg, &mut hdl) };
    (err == sys::ESP_OK).then_some(hdl)
}

fn send(hdl: cdc::cdc_acm_dev_hdl_t, frame: &[u8]) {
    // SAFETY: `hdl` is open; `frame` outlives the blocking call.
    let err = unsafe { cdc::cdc_acm_host_data_tx_blocking(hdl, frame.as_ptr(), frame.len(), 200) };
    if err != sys::ESP_OK {
        log::warn!("cat: tx failed err={err:#x}");
    }
}

/// Whether CI-V is open — i.e. whether [`ptt`] can reach the radio.
pub fn is_open() -> bool {
    !HDL.load(Ordering::Acquire).is_null()
}

/// How a [`ptt`] call went, in microseconds from the moment the frame
/// was handed to the driver. `None` means it did not arrive inside the
/// wait.
#[derive(Clone, Copy, Debug)]
pub struct PttTiming {
    /// The `FB` acknowledging the `1C 00 0x` command.
    pub ack_us: Option<u32>,
    /// The `1C 00` readback reporting the requested state.
    pub readback_us: Option<u32>,
}

/// Key (`true`) or unkey the radio over CI-V, from the caller's task,
/// and wait up to `wait_ms` for the radio to confirm both that it
/// understood (`FB`) and that it is now in that state (`1C 00`
/// readback). `Err` when CI-V is not open — the caller must not assume
/// the radio did anything.
///
/// Blocks the caller for at most `wait_ms` plus two 200 ms send
/// timeouts. The readback is the confirmation that matters: an `FB`
/// means the frame parsed, not that the transmitter followed.
pub fn ptt(on: bool, wait_ms: u32) -> Result<PttTiming, &'static str> {
    let _held = HDL_USE.lock().map_err(|_| "CI-V lock poisoned")?;
    let h = HDL.load(Ordering::Acquire) as cdc::cdc_acm_dev_hdl_t;
    if h.is_null() {
        return Err("CI-V not open");
    }
    let now_us = || unsafe { sys::esp_timer_get_time() };
    let ok0 = OK_SEQ.load(Ordering::Acquire);
    let t0 = now_us();
    send(h, &civ_frame::ptt(IC705_ADDR, on));
    let mut ack_us = None;
    let deadline = t0 + wait_ms as i64 * 1_000;
    while now_us() < deadline {
        if OK_SEQ.load(Ordering::Acquire) != ok0 {
            ack_us = Some((now_us() - t0) as u32);
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    let want = if on { 2 } else { 1 };
    let mut readback_us = None;
    while now_us() < deadline {
        let seq0 = PTT_SEQ.load(Ordering::Acquire);
        send(h, &civ_frame::read_ptt(IC705_ADDR));
        let ask = now_us();
        // One reply per ask; re-ask after 50 ms, for a radio that has
        // not switched yet when the first one is answered.
        while now_us() < (ask + 50_000).min(deadline) {
            if PTT_SEQ.load(Ordering::Acquire) != seq0 && PTT_SEEN.load(Ordering::Acquire) == want {
                readback_us = Some((now_us() - t0) as u32);
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        if readback_us.is_some() {
            break;
        }
    }
    Ok(PttTiming { ack_us, readback_us })
}

fn cat_task() {
    // Audio first: open only once the PCM2901's IN stream has
    // enumerated, so a channel shortage can cost CAT and never the
    // receiver.
    let mut waited = 0u32;
    while crate::uac::RX_IFACE_SEEN.load(Ordering::Relaxed) < 0 && waited < 60 {
        std::thread::sleep(std::time::Duration::from_secs(1));
        waited += 1;
    }
    let mut shown_hz = 0u32;
    loop {
        let Some(h) = open() else {
            // `open` already waited `connection_timeout_ms`.
            continue;
        };
        DISCONNECTED.store(false, Ordering::Release);
        log::warn!("cat: IC-705 CI-V open (USB A)");
        // Unkey first. A board that reset while transmitting left the
        // radio keyed, and nothing else here would ever unkey it; this
        // is the first frame the radio hears from a new boot.
        send(h, &civ_frame::ptt(IC705_ADDR, false));
        send(h, &civ_frame::read_ptt(IC705_ADDR));
        HDL.store(h as *mut core::ffi::c_void, Ordering::Release);
        send(h, &civ_frame::read_freq(IC705_ADDR));
        send(h, &civ_frame::read_mode(IC705_ADDR));
        while !DISCONNECTED.load(Ordering::Acquire) {
            let want = PENDING_HZ.swap(0, Ordering::AcqRel);
            if want != 0 {
                log::warn!("cat: set dial {want} Hz, USB-D FIL1");
                send(h, &civ_frame::set_freq(IC705_ADDR, want));
                send(h, &civ_frame::set_mode_usb_data(IC705_ADDR));
                // Transceive reports a dial the rig *moved* to; ask, so
                // a refused `05` shows as the old dial rather than the
                // requested one.
                send(h, &civ_frame::read_freq(IC705_ADDR));
                if SAVED_HZ.swap(want, Ordering::AcqRel) != want {
                    with_nvs(move |nvs| {
                        if let Err(e) = mfsk_app_shared::freq_presets::write(nvs, want) {
                            log::warn!("cat: saving dial failed: {e}");
                        }
                    });
                }
            }
            let hz = RIG_HZ.load(Ordering::Acquire);
            if hz != 0 && hz != shown_hz {
                shown_hz = hz;
                if let Ok(mut ui) = mfsk_app_shared::ui::state::UI.lock() {
                    ui.update_status(|st| st.rig_freq_hz = Some(hz));
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(POLL_MS));
        }
        HDL.store(core::ptr::null_mut(), Ordering::Release);
        PTT_SEEN.store(0, Ordering::Release);
        log::warn!("cat: IC-705 unplugged — waiting for it");
        let _held = HDL_USE.lock();
        // SAFETY: opened above; the driver wants a close after a
        // disconnect to free the handle. No `ptt` caller holds it:
        // `HDL` is cleared and `HDL_USE` is ours.
        let err = unsafe { cdc::cdc_acm_host_close(h) };
        if err != sys::ESP_OK {
            log::warn!("cat: close failed err={err:#x}");
        }
    }
}
