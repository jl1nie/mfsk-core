//! The files this board keeps: `qso.adi` (the activator's contact log)
//! and `all.txt` (every decode and transmission, WSJT-X's `ALL.TXT`
//! layout), on the `littlefs` partition, and the one task that touches
//! them.
//!
//! **Why a task of its own.** A flash write or read disables the flash
//! cache, which also unmaps PSRAM, so ESP-IDF asserts that the caller's
//! stack is in internal DRAM (`esp_task_stack_is_sane_cache_disabled`,
//! `cache_utils.c:127`) — and aborts the board if not. The HTTP server
//! and the panel tasks run on PSRAM stacks on purpose, to leave internal
//! DRAM to the decoder; `boot_mode::commit_and_restart` is where this
//! was learned. Every read and write therefore goes over a channel to
//! [`TASK_NAME`], whose stack is internal DRAM by explicit caps.
//! Callers never block on the flash: appends are
//! fire-and-forget, and a full queue drops the batch and counts it
//! ([`dropped`]) rather than stall a decoder at its reply deadline.
//!
//! LittleFS's own buffers are put in PSRAM
//! (`CONFIG_LITTLEFS_MALLOC_STRATEGY_SPIRAM`); `esp_flash` copies
//! through an internal bounce buffer when the source or destination is
//! external, so that costs speed, not correctness.
//!
//! **When the flash is written — the one thing priorities cannot
//! arrange.** A LittleFS write stops the cache on both cores, and with
//! it every task and every interrupt not in IRAM, whatever their
//! priority: this priority-2 task's write stops the priority-8 USB
//! audio path, the decoder and a transmission alike. On an IC-705 an
//! `all.txt` flush mid-slot cost a skipped isochronous packet (3 ms of
//! receive audio, `logs/live_ft8_62e0bee6_2026-09-22.log`); a USB
//! transmit stream would lose the same. The flash's auto-suspend
//! (`SPI_FLASH_AUTO_SUSPEND`) would let the cache through, but IDF
//! detects this board's chip as `generic`, outside what it supports.
//!
//! So lines and records are held in PSRAM and written only at moments
//! chosen for it (see [`serve`] and [`next_rx_quiet_point`]):
//!
//! - a **quiet window** the transmit side hands over with
//!   [`quiet_window`] — the tail of our own transmit slot, after the
//!   audio has ended and before the next slot, when nothing on the
//!   board is doing anything a stall could cost. Nothing calls it
//!   until transmission exists;
//! - **receive-only**, when the buffer passes [`ALL_HIGH_WATER`], at the
//!   point in a slot after every station's transmission has ended and
//!   before the decode starts, [`ALL_RX_CHUNK`] at a time;
//! - just before a **read** (a download is a deliberate act; it takes
//!   what it costs).
//!
//! A power cut loses what is held: up to [`ALL_HIGH_WATER`] of
//! `all.txt` (~45 min of a busy FT8 band), and at most the contacts
//! since the last window — one, normally, since a contact is logged in
//! the transmit slot whose tail is the next window.

use std::io::{Read, Seek, SeekFrom, Write};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::Mutex;
use std::sync::OnceLock;
use std::time::Duration;

use mfsk_app_shared::boot_mode::BootMode;
use mfsk_app_shared::ui::state::{SlotDecode, UiState};

const MOUNT: &str = "/littlefs";
const PARTITION: &core::ffi::CStr = c"littlefs";
const TASK_NAME: &core::ffi::CStr = c"storage";
/// Measured high-water mark wanted from `board::log_task_stacks` before
/// this is trusted; the frames are LittleFS's path walk plus newlib's
/// VFS, neither of them deep.
const TASK_STACK: usize = 5120;
/// Below the decoder and the audio path: the flash op stalls both cores
/// whoever issues it, so the only thing priority buys is *when*.
const TASK_PRIO: u8 = 2;
/// Queued batches. A slot is one batch, so this is several slots of
/// slack for a stuck flash before anything is dropped.
const QUEUE: usize = 8;

/// `all.txt` rotates to `all.1.txt` past this; `qso.adi` never rotates.
/// Two of them is 4 MiB of the 6.125 MiB partition — roughly a day and
/// a half of a busy band at ~120 KB/h.
const ALL_MAX_BYTES: u64 = 2 * 1024 * 1024;

/// A file this module serves. The names are the HTTP paths too.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum File {
    Qso,
    All,
    AllOld,
}

impl File {
    pub const ALL_FILES: [File; 3] = [File::Qso, File::All, File::AllOld];

    pub fn name(self) -> &'static str {
        match self {
            File::Qso => "qso.adi",
            File::All => "all.txt",
            File::AllOld => "all.1.txt",
        }
    }

    pub fn from_name(name: &str) -> Option<File> {
        Self::ALL_FILES.into_iter().find(|f| f.name() == name)
    }

    fn path(self) -> String {
        format!("{MOUNT}/{}", self.name())
    }
}

enum Req {
    /// Lines for `all.txt`, and the slot period they came from — which
    /// decides where the receive-only write point falls.
    AppendAll { text: String, period_ms: u64 },
    /// Write what is held, finishing by this `esp_timer` time (µs).
    Quiet { until_us: i64 },
    /// A receiver with no slot says nothing it cares about is running:
    /// write what is held if it has waited [`NO_SLOT_QUIET_AGE_US`].
    QuietRx,
    /// Write everything held, then answer — before a restart.
    Flush { reply: SyncSender<()> },
    /// A record, and the header to write first if the file is new.
    AppendQso { record: String, header: String },
    Read {
        file: File,
        offset: u64,
        len: usize,
        reply: SyncSender<Option<Vec<u8>>>,
    },
}

static TX: OnceLock<SyncSender<Req>> = OnceLock::new();
/// The receiving end, from the first request until
/// [`start_if_requested`] hands it to the task.
static PENDING_RX: Mutex<Option<Receiver<Req>>> = Mutex::new(None);
static MOUNTED: AtomicBool = AtomicBool::new(false);
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Batches dropped because the queue was full or the filesystem is not
/// mounted.
pub fn dropped() -> u32 {
    DROPPED.load(Ordering::Relaxed)
}

pub fn mounted() -> bool {
    MOUNTED.load(Ordering::Acquire)
}

static ENABLED: AtomicBool = AtomicBool::new(false);
/// Something has been sent. The task is spawned only after this — the
/// channel exists from boot ([`enable`]), so its existence cannot be
/// the trigger, and was for a while (spawning on the panel's first
/// frame, ahead of the decoder's allocations).
static REQUESTED: AtomicBool = AtomicBool::new(false);

/// Allow the storage task to exist; it is spawned after the first
/// request, by [`start_if_requested`].
///
/// **Why lazily.** Spawned from `boot::run`, its 5 KB internal stack
/// was carved out of the block the decoder allocates from next: the
/// largest free internal block before the decode loop fell from
/// 40 960 B to 32 768 B, and with it FT8 went from 0 of 18 SIM slots
/// past key-up to 9-14 of 18 (2026-09-22, `logs/storage_off_ft8sim_*`
/// against `logs/storage_midslot_ft8sim_*`). The first request comes
/// from the first published slot, by which time the decoder has what
/// it needs.
///
/// The channel is made here, on the booting task, and not by the first
/// request: building an `mpsc` channel assembles its state on the
/// caller's stack before boxing it, and the first caller is a decoder.
/// Made by the first request, it left `ft4_slot` (8 KB) 444-464 B free in
/// FT4 SIM runs against 732 B with storage off (2026-09-22,
/// `logs/ft4sim_storage_*_2026-09-22.log`).
pub fn enable() {
    ENABLED.store(true, Ordering::Release);
    let _ = ensure_channel();
}

/// The channel: made once, by [`enable`].
fn ensure_channel() -> Option<&'static SyncSender<Req>> {
    if !ENABLED.load(Ordering::Acquire) {
        return None;
    }
    Some(TX.get_or_init(|| {
        let (tx, rx) = sync_channel::<Req>(QUEUE);
        if let Ok(mut p) = PENDING_RX.lock() {
            *p = Some(rx);
        }
        tx
    }))
}

/// Spawn the storage task once something has asked for it. Called by
/// the panel loop every frame; a no-op after the first spawn.
///
/// **Not from the requester**, which is a decoder: a thread spawn is
/// not a shallow call, and a decoder's stack is sized to its decode.
/// The panel's `main` task has ~10 KB free. Requests made before the
/// spawn wait in the channel, [`QUEUE`] deep.
pub fn start_if_requested() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    if ONCE.is_completed() || !REQUESTED.load(Ordering::Acquire) {
        return;
    }
    ONCE.call_once(|| {
        let rx = PENDING_RX.lock().ok().and_then(|mut p| p.take());
        if let Some(rx) = rx {
            spawn(rx);
        }
    });
}

/// Internal DRAM for the task's stack, taken at boot by a mode that
/// cannot count on finding [`TASK_STACK`] contiguous later.
///
/// Only JTTY calls it. Its receiver leaves 3-6 KB of internal DRAM free
/// once it runs, and there the lazy spawn failed with ENOMEM — no
/// `all.txt` that boot (`docs/notes/JTTY_CORES3_APP.md` §14). Every
/// other mode keeps the lazy pthread above, whose reason (not carving
/// the decoder's block at boot) still holds for them.
static RESERVED_STACK: core::sync::atomic::AtomicPtr<u8> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// Take the task's stack now. Call while the heap is whole, before
/// anything fragments it (`worker_arena`'s reason); the task still
/// spawns lazily, on the first request, onto this block.
pub fn reserve_stack() -> bool {
    use esp_idf_svc::sys;
    // SAFETY: a plain sized allocation, never freed — the task runs for
    // the life of the process.
    let p = unsafe { sys::heap_caps_malloc(TASK_STACK, sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT) }
        as *mut u8;
    if p.is_null() {
        log::error!("storage: could not reserve its {TASK_STACK} B stack");
        return false;
    }
    RESERVED_STACK.store(p, Ordering::Release);
    log::info!("storage: {TASK_STACK} B stack reserved at {p:p}");
    true
}

/// Spawn the storage task; it mounts the partition (formatting it the
/// first time) before taking requests.
///
/// A `std::thread` with its stack caps set to internal DRAM explicitly
/// — `uac::spawn_psram_thread`'s mechanism with the opposite caps — so
/// the channel below parks a pthread, as `std` expects. With a stack
/// from [`reserve_stack`] it is a static FreeRTOS task instead, as
/// JTTY's `Back` is, whose channel receive parks the same way.
fn spawn(rx: Receiver<Req>) {
    use esp_idf_svc::hal::cpu::Core;
    use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};

    let reserved = RESERVED_STACK.load(Ordering::Acquire);
    if !reserved.is_null() {
        spawn_static(rx, reserved);
        return;
    }
    let d = ThreadSpawnConfiguration::default();
    let cfg = ThreadSpawnConfiguration {
        name: Some(TASK_NAME),
        stack_size: TASK_STACK,
        stack_alloc_caps: MallocCap::Internal | MallocCap::Cap8bit,
        priority: TASK_PRIO,
        pin_to_core: Some(Core::Core1),
        ..d
    };
    if let Err(e) = cfg.set() {
        log::error!("storage: spawn config failed ({e:?}) — no logs this boot");
        return;
    }
    let spawned = std::thread::Builder::new()
        .stack_size(TASK_STACK)
        .spawn(move || task_main(rx));
    let _ = ThreadSpawnConfiguration::default().set();
    if let Err(e) = spawned {
        log::error!("storage: could not spawn the task ({e}) — no logs this boot");
    }
}

/// [`spawn`] onto the stack [`reserve_stack`] took: same name, priority
/// and core.
fn spawn_static(rx: Receiver<Req>, stack: *mut u8) {
    use esp_idf_svc::sys;
    static mut TCB: core::mem::MaybeUninit<sys::StaticTask_t> = core::mem::MaybeUninit::uninit();
    unsafe extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `arg` is the box leaked below, handed to this task alone.
        let rx = *unsafe { Box::from_raw(arg as *mut Receiver<Req>) };
        task_main(rx);
        // A FreeRTOS task must not return.
        unsafe { sys::vTaskDelete(core::ptr::null_mut()) };
    }
    let arg = Box::into_raw(Box::new(rx)) as *mut core::ffi::c_void;
    // SAFETY: the stack is the block `reserve_stack` took for this task
    // alone; the TCB is static and `start_if_requested` runs this once.
    let h = unsafe {
        sys::xTaskCreateStaticPinnedToCore(
            Some(entry),
            TASK_NAME.as_ptr(),
            TASK_STACK as u32,
            arg,
            u32::from(TASK_PRIO),
            stack,
            core::ptr::addr_of_mut!(TCB) as *mut sys::StaticTask_t,
            1,
        )
    };
    if h.is_null() {
        log::error!("storage: could not create the task on its reserved stack — no logs this boot");
        // SAFETY: the task never started, so the box is still ours.
        drop(unsafe { Box::from_raw(arg as *mut Receiver<Req>) });
    }
}

/// The task: mount, then serve requests and receive-only write points.
fn task_main(rx: Receiver<Req>) {
    if mount() {
        MOUNTED.store(true, Ordering::Release);
    }
    let mut st = Held::default();
    let ok = mounted();
    loop {
        // Wake for a request, or for the receive-only write
        // point when one is due.
        let req = match st.next_due_us() {
            Some(due) => {
                let wait = (due - now_us()).max(0) as u64;
                match rx.recv_timeout(Duration::from_micros(wait)) {
                    Ok(r) => Some(r),
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout) => None,
                    Err(_) => break,
                }
            }
            None => match rx.recv() {
                Ok(r) => Some(r),
                Err(_) => break,
            },
        };
        if !ok {
            if req.is_some() {
                DROPPED.fetch_add(1, Ordering::Relaxed);
            }
            continue;
        }
        match req {
            Some(r) => serve(r, &mut st),
            None => st.rx_quiet_point(),
        }
    }
}

fn mount() -> bool {
    use esp_idf_svc::sys;
    let mut conf = sys::esp_vfs_littlefs_conf_t {
        base_path: c"/littlefs".as_ptr(),
        partition_label: PARTITION.as_ptr(),
        ..Default::default()
    };
    conf.set_format_if_mount_failed(1);
    // SAFETY: `conf` and the strings it points at outlive the call; the
    // component copies what it keeps.
    let r = unsafe { sys::esp_vfs_littlefs_register(&conf) };
    if r != sys::ESP_OK {
        log::error!("storage: mount {MOUNT} failed ({r}) — no logs this boot");
        return false;
    }
    let (mut total, mut used) = (0usize, 0usize);
    // SAFETY: two valid out-pointers.
    unsafe { sys::esp_littlefs_info(PARTITION.as_ptr(), &mut total, &mut used) };
    log::info!(
        "storage: {MOUNT} mounted, {} of {} KB used",
        used / 1024,
        total / 1024
    );
    true
}

fn now_us() -> i64 {
    // SAFETY: no arguments.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn serve(req: Req, st: &mut Held) {
    match req {
        Req::AppendAll { text, period_ms } => {
            if st.all.pending.is_empty() {
                st.all_since_us = now_us();
            }
            st.all.hold(&text);
            st.period_ms = period_ms;
            st.schedule();
        }
        Req::QuietRx => {
            if st.period_ms == 0
                && !st.all.pending.is_empty()
                && now_us() - st.all_since_us >= NO_SLOT_QUIET_AGE_US
            {
                st.write_until(i64::MAX, usize::MAX, "quiet");
                st.schedule();
            }
        }
        Req::AppendQso { record, header } => {
            log::info!("storage: qso.adi held: {}", record.trim_end());
            if st.qso.is_empty() {
                st.qso_since_us = now_us();
            }
            st.qso.push((record, header));
            st.schedule();
        }
        Req::Quiet { until_us } => st.write_until(until_us, usize::MAX, "window"),
        Req::Flush { reply } => {
            st.write_until(i64::MAX, usize::MAX, "flush");
            let _ = reply.send(());
        }
        Req::Read {
            file,
            offset,
            len,
            reply,
        } => {
            // A download is a deliberate act; it takes what it costs.
            st.write_until(i64::MAX, usize::MAX, "read");
            let _ = reply.send(read(file, offset, len).ok());
        }
    }
}

/// What is held in PSRAM, and when the receive-only write is next due.
#[derive(Default)]
struct Held {
    all: AllTxt,
    /// ADIF records and the header each wants if the file is new.
    qso: Vec<(String, String)>,
    /// `esp_timer` µs the oldest held record arrived.
    qso_since_us: i64,
    /// The slot period the latest lines came from; 0 for a receiver with
    /// none (JTTY), which reports its own quiet moments instead.
    period_ms: u64,
    /// `esp_timer` µs the oldest held `all.txt` line arrived.
    all_since_us: i64,
    /// `esp_timer` µs of the next wake, and whether it is a
    /// receive-only write point (`true`) or only a time to decide again
    /// (`false`: a held contact reaching [`QSO_MAX_HOLD_US`]).
    due: Option<(i64, bool)>,
}

impl Held {
    fn next_due_us(&self) -> Option<i64> {
        self.due.map(|(t, _)| t)
    }

    /// Decide whether a receive-only write is wanted, and when.
    fn schedule(&mut self) {
        let now = now_us();
        let qso_expired = !self.qso.is_empty() && now - self.qso_since_us >= QSO_MAX_HOLD_US;
        if self.period_ms == 0 && !self.all.pending.is_empty() {
            // No slot, so no point in one to write at: the receiver sends
            // `QuietRx` when nothing it cares about is running, and this is
            // only the bound for a band that never goes quiet — written then
            // whatever it stalls.
            self.due = Some((self.all_since_us + NO_SLOT_MAX_AGE_US, true));
            return;
        }
        self.due = if self.all.pending.len() >= ALL_HIGH_WATER || qso_expired {
            next_rx_quiet_point(self.period_ms).map(|t| (t, true))
        } else if !self.qso.is_empty() {
            Some((self.qso_since_us + QSO_MAX_HOLD_US, false))
        } else {
            None
        };
    }

    /// A wake has come: write if it was a write point, then decide again.
    fn rx_quiet_point(&mut self) {
        if let Some((_, true)) = self.due {
            // Past the point by the time the flash is done is fine; this
            // is only where to *start*.
            self.write_until(i64::MAX, ALL_RX_CHUNK, "rx");
        }
        self.due = None;
        self.schedule();
    }

    /// Contacts first, then `all.txt` 4 KB at a time while there is
    /// time before `until_us` for another and no more than `budget`
    /// bytes have gone.
    fn write_until(&mut self, until_us: i64, budget: usize, why: &str) {
        let t0 = now_us();
        let at = slot_ms(self.period_ms);
        for (record, header) in self.qso.drain(..) {
            match write_qso(&record, &header) {
                Ok(()) => log::info!("storage: qso.adi += {}", record.trim_end()),
                // Loud: this is the contact log, not a convenience.
                Err(e) => log::error!(
                    "storage: qso.adi append FAILED: {e} — {}",
                    record.trim_end()
                ),
            }
        }
        let mut written = 0usize;
        while !self.all.pending.is_empty()
            && written < budget
            && until_us.saturating_sub(now_us()) > WRITE_BLOCK_WORST_US
        {
            match self.all.write_some(ALL_WRITE_BLOCK) {
                Ok(n) => written += n,
                Err(e) => {
                    log::warn!("storage: all.txt write failed: {e}");
                    break;
                }
            }
        }
        if written > 0 {
            let ms = (now_us() - t0) / 1_000;
            log::info!(
                "storage: [{why}] all.txt +{written} B in {ms} ms from slot +{at} ms, {} B still held, now {} KB",
                self.all.pending.len(),
                self.all.size() / 1024,
            );
        }
    }
}

/// Milliseconds into the current UTC slot, or -1 without a clock.
fn slot_ms(period_ms: u64) -> i64 {
    mfsk_app_shared::time_sync::utc_now_ms().map_or(-1, |t| (t % period_ms.max(1)) as i64)
}

/// The next point, as `esp_timer` µs, at which a receive-only write
/// stops no audio anything needs: every station's transmission in the
/// slot has ended and the decode has not started.
///
/// FT8: transmissions are 0.5 + 12.64 s, the decode starts at 14.0 s,
/// so 13.3 s leaves a station 0.16 s late its whole frame. FT4: 0.5 +
/// 5.04 s, capture closes at 6.775 s, so 5.8 s. WSPR: 1 + 110.6 s of a
/// 120 s slot, so 112 s. Anything else, 90 % of the way through. Now,
/// without a clock — there is no slot to place it in.
fn next_rx_quiet_point(period_ms: u64) -> Option<i64> {
    let offset_ms = match period_ms {
        15_000 => 13_300,
        7_500 => 5_800,
        120_000 => 112_000,
        p => p * 9 / 10,
    };
    let Some(now_ms) = mfsk_app_shared::time_sync::utc_now_ms() else {
        return Some(now_us());
    };
    let p = period_ms.max(1);
    let phase = now_ms % p;
    let wait_ms = (offset_ms + p - phase) % p;
    Some(now_us() + wait_ms as i64 * 1_000)
}

/// Put everything held on flash and wait for it, up to `timeout` —
/// before a restart, which would otherwise lose it. `false` if the task
/// did not answer in time (or storage is off).
pub fn flush_blocking(timeout: Duration) -> bool {
    // Nothing ever sent: nothing held, and no task to ask.
    if !REQUESTED.load(Ordering::Acquire) {
        return true;
    }
    let Some(tx) = ensure_channel() else {
        return false;
    };
    let (reply, rx) = sync_channel(1);
    if tx.send(Req::Flush { reply }).is_err() {
        return false;
    }
    rx.recv_timeout(timeout).is_ok()
}

/// Mark the next `until_us` (`esp_timer` µs) as free to write in: the
/// transmit side calls this once its audio for the slot has ended, with
/// the moment the next slot begins. What is held goes out then.
pub fn quiet_window(until_us: i64) {
    send(Req::Quiet { until_us });
}

/// **For a receiver with no slot** (JTTY): nothing it is receiving would
/// be hurt by a flash write now — no message is open and nothing waits
/// to be decoded. Held `all.txt` lines are written if the oldest has
/// waited [`NO_SLOT_QUIET_AGE_US`], so a quiet band does not stall the
/// board for every single line. Cheap to call often: it only queues.
pub fn quiet_rx() {
    let _ = ensure_channel().map(|tx| tx.try_send(Req::QuietRx));
}

/// No slot: how long `all.txt` lines wait for a quiet moment before one is
/// used — batching, so each flash stall (~85 ms, both cores) carries many.
const NO_SLOT_QUIET_AGE_US: i64 = 30_000_000;

/// No slot: the longest lines are held with no quiet moment at all, after
/// which they are written anyway. A power cut loses at most this much of
/// the log; a band busy enough never to go quiet for ten minutes pays one
/// stall for it.
const NO_SLOT_MAX_AGE_US: i64 = 600_000_000;

/// `all.txt` lines from a receiver with no slot: held like any other, but
/// written only at the moments [`quiet_rx`] reports, or past
/// [`NO_SLOT_MAX_AGE_US`].
pub fn append_all_txt_no_slot(text: String) {
    append_all_txt(text, 0);
}

/// Receive-only: write once `all.txt` holds this much. ~45 min of a
/// busy FT8 band (~65 B a line, ~15 lines a slot).
///
/// `MFSK_STORAGE_HIGH_WATER_KB=N` overrides it at build time — a bench
/// knob, so the receive-only write point can be seen in minutes rather
/// than most of an hour.
const ALL_HIGH_WATER: usize = match option_env!("MFSK_STORAGE_HIGH_WATER_KB") {
    Some(v) => parse_kb(v) * 1024,
    None => 48 * 1024,
};

const fn parse_kb(s: &str) -> usize {
    let b = s.as_bytes();
    let mut i = 0;
    let mut n = 0usize;
    while i < b.len() {
        n = n * 10 + (b[i] - b'0') as usize;
        i += 1;
    }
    n
}

/// Receive-only: at most this much per write point — four erase blocks,
/// ~120-450 ms at the 30-110 ms per 4 KB measured on 2026-09-22, inside
/// the 0.7 s between FT8's last symbol and its decode. The rest waits
/// for the next slot's point.
const ALL_RX_CHUNK: usize = 16 * 1024;

/// One write: one erase block's worth, so every `sync` commits whole
/// blocks (see [`ALL_WRITE_BLOCK`]'s history below).
const ALL_WRITE_BLOCK: usize = 4096;

/// Don't start another block with less than this before a window
/// closes: the slowest 4 KB flush measured, 146 ms, with a metadata
/// compaction in it.
const WRITE_BLOCK_WORST_US: i64 = 160_000;

/// A contact held longer than this with no window gets written at the
/// next receive-only point. The window follows the contact in the same
/// transmit slot, so this is a guard, not the path.
const QSO_MAX_HOLD_US: i64 = 30_000_000;

/// `all.txt` in memory and on flash.
///
/// **Written a block at a time.** LittleFS cannot program more into a
/// block after a `sync` has committed it, so every synced append copies
/// the file's tail block to a freshly erased one — the bytes already in
/// that block are written again each time. Traced on the CoreS3
/// (2026-09-22, `MFSK_STORAGE_TRACE`,
/// `logs/storage_trace_ft8sim_2026-09-22.log`), a 448 B slot append
/// cost one 4 KB erase plus 512 B to 4 096 B of programming, and roughly
/// one `sync` in ten also compacted the metadata log: 538 reads, 69 KB,
/// one erase, **147 ms**. [`ALL_WRITE_BLOCK`] per `sync` bounds the copy.
#[derive(Default)]
struct AllTxt {
    open: Option<(std::fs::File, u64)>,
    /// Lines not yet on flash. Reserved at [`ALL_HOLD_CAPACITY`] on
    /// first use, which puts it in PSRAM (anything over
    /// `SPIRAM_MALLOC_ALWAYSINTERNAL` goes there).
    pending: String,
}

/// Room reserved for held lines: the high-water mark plus a slot of
/// headroom, so holding never reallocates on the way up to it.
const ALL_HOLD_CAPACITY: usize = ALL_HIGH_WATER + 16 * 1024;

/// Most ever held: only reached if writes keep failing.
const ALL_HOLD_MAX: usize = 256 * 1024;

impl AllTxt {
    fn hold(&mut self, text: &str) {
        if self.pending.capacity() == 0 {
            self.pending.reserve(ALL_HOLD_CAPACITY);
        }
        self.pending.push_str(text);
        // Writes keep failing (a full or broken partition): drop the
        // oldest whole lines rather than grow without bound.
        if self.pending.len() > ALL_HOLD_MAX {
            let excess = self.pending.len() - ALL_HOLD_MAX;
            let cut = self.pending[excess..].find('\n').map_or(self.pending.len(), |i| excess + i + 1);
            self.pending.drain(..cut);
            DROPPED.fetch_add(1, Ordering::Relaxed);
            log::warn!("storage: all.txt held past {ALL_HOLD_MAX} B — dropped the oldest {cut} B");
        }
    }

    fn size(&self) -> u64 {
        self.open.as_ref().map_or(0, |(_, n)| *n)
    }

    /// Write up to `max` bytes of what is held (cut at a line end), and
    /// `sync`. Returns the bytes written.
    fn write_some(&mut self, max: usize) -> std::io::Result<usize> {
        if self.pending.is_empty() {
            return Ok(0);
        }
        let path = File::All.path();
        if self.open.is_none() {
            let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
            let f = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)?;
            self.open = Some((f, size));
        }
        let cut = if self.pending.len() <= max {
            self.pending.len()
        } else {
            // Whole lines only, so a file cut short by a power failure
            // still parses line by line.
            self.pending[..max].rfind('\n').map_or(max, |i| i + 1)
        };
        let size = self.size();
        if size + cut as u64 > ALL_MAX_BYTES {
            self.open = None; // close before the rename
            let old = File::AllOld.path();
            let _ = std::fs::remove_file(&old);
            std::fs::rename(&path, &old)?;
            log::info!("storage: all.txt rotated at {size} B");
            return self.write_some(max);
        }
        let (f, n) = self.open.as_mut().expect("opened above");
        #[cfg(storage_trace)]
        let s0 = trace::snapshot();
        let r = f
            .write_all(self.pending[..cut].as_bytes())
            .and_then(|()| f.sync_all());
        #[cfg(storage_trace)]
        log::info!("storage: trace write+sync: {}", trace::delta(s0, trace::snapshot()));
        if let Err(e) = r {
            // Reopen next time rather than keep writing through a handle
            // in an unknown state. The lines stay held.
            self.open = None;
            return Err(e);
        }
        *n += cut as u64;
        self.pending.drain(..cut);
        Ok(cut)
    }
}

fn write_qso(record: &str, header: &str) -> std::io::Result<()> {
    let path = File::Qso.path();
    let new = std::fs::metadata(&path).map_or(true, |m| m.len() == 0);
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)?;
    if new {
        f.write_all(header.as_bytes())?;
    }
    f.write_all(record.as_bytes())?;
    // The contact is on flash before the next transmission, which is the
    // guarantee a battery-powered log has to make.
    f.sync_all()
}

fn read(file: File, offset: u64, len: usize) -> std::io::Result<Vec<u8>> {
    let mut f = std::fs::File::open(file.path())?;
    f.seek(SeekFrom::Start(offset))?;
    let mut buf = vec![0u8; len];
    let mut n = 0;
    while n < len {
        match f.read(&mut buf[n..])? {
            0 => break,
            k => n += k,
        }
    }
    buf.truncate(n);
    Ok(buf)
}

fn send(req: Req) -> bool {
    // Queued even before the task exists: it mounts first and then
    // drains, and drops (and counts) what it cannot write.
    let ok = ensure_channel().is_some_and(|tx| tx.try_send(req).is_ok());
    if ok {
        REQUESTED.store(true, Ordering::Release);
    }
    if !ok {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// Append lines (each ending in `\n`) to `all.txt`.
pub fn append_all_txt(text: String, period_ms: u64) {
    if !text.is_empty() {
        send(Req::AppendAll { text, period_ms });
    }
}

/// Append one ADIF record to `qso.adi`, writing `header` first if the
/// file is new. Returns `false` if it could not even be queued — the
/// caller should say so on the panel, since a contact log that silently
/// misses a contact is worse than none.
pub fn append_qso(record: String, header: String) -> bool {
    send(Req::AppendQso { record, header })
}

/// Up to `len` bytes of `file` from `offset`; `Some(empty)` at the end,
/// `None` if the file is absent or the task did not answer. For the
/// HTTP download handlers, which run on a PSRAM stack and so must not
/// read the flash themselves.
pub fn read_chunk(file: File, offset: u64, len: usize) -> Option<Vec<u8>> {
    let tx = ensure_channel()?;
    let (reply, rx) = sync_channel(1);
    tx.send(Req::Read {
        file,
        offset,
        len,
        reply,
    })
    .ok()?;
    REQUESTED.store(true, Ordering::Release);
    rx.recv_timeout(Duration::from_secs(5)).ok().flatten()
}

/// The files, as the HTTP server serves them.
pub const HTTP_FILES: mfsk_app_shared::http_config::FileSource =
    mfsk_app_shared::http_config::FileSource {
        names: &["qso.adi", "all.txt", "all.1.txt"],
        read: |name, offset, len| read_chunk(File::from_name(name)?, offset, len),
    };

/// The UTC second the slot just decoded began, for a decode published
/// after its slot ended and before the next one did — true of every
/// receiver here at its publish point. `None` while the clock is unset:
/// a line stamped 1970 is worse than no line.
pub fn decoded_slot_unix(period_ms: u64) -> Option<i64> {
    let now = mfsk_app_shared::time_sync::utc_now_ms()?;
    let start_ms = (now / period_ms).checked_sub(1)? * period_ms;
    Some((start_ms / 1000) as i64)
}

/// Publish one slot — **the one call every receiver makes**: the rows
/// go to the station list and, from the same values, to `ALL.TXT`.
///
/// FT8, FT4, WSPR and FST4 each used to call `UiState::publish_slot`
/// and then build their own `ALL.TXT` tuples beside it, four times
/// over. The mode name and slot period come from the boot mode, and
/// the dial frequency from the status bar's `rig_freq_hz`, so what the
/// panel shows and what the log records cannot disagree — and a CAT
/// link that sets the status field puts the frequency into every
/// mode's log at once.
///
/// Takes the locked [`UiState`] because FT8 publishes from inside the
/// lock it decodes under. The `ALL.TXT` half only formats and queues;
/// the flash write happens on the storage task.
///
/// `slot_unix` is the UTC second the slot began, `None` while the
/// clock is unset (the log is skipped: a line stamped 1970 is worse
/// than no line). [`decoded_slot_unix`] gives it for receivers that
/// publish in the slot after the one decoded.
pub fn publish_slot(
    ui: &mut UiState,
    mode: BootMode,
    slot_unix: Option<i64>,
    decodes: &[SlotDecode<'_>],
) {
    ui.publish_slot(decodes.iter().copied());
    let Some(t) = slot_unix else { return };
    let name = mfsk_app_shared::ui::mode_picker::mode_name(mode).unwrap_or("?");
    let dial_hz = ui.status.rig_freq_hz.map(u64::from);
    let mut text = String::new();
    for d in decodes {
        let snr = if d.snr_db.is_finite() {
            d.snr_db.round() as i32
        } else {
            0
        };
        text.push_str(&mfsk_app_shared::all_txt::rx_line(
            t,
            dial_hz,
            name,
            snr,
            d.dt_sec,
            d.freq_hz.round() as i32,
            d.text,
        ));
    }
    append_all_txt(text, mode.slot_period_ms() as u64);
}

/// Counting wrappers around the flash calls LittleFS makes
/// (`littlefs_esp_part.c`: read, write, erase_range), linked in with
/// `-Wl,--wrap` when built with `MFSK_STORAGE_TRACE=1`. Everything
/// that touches a partition goes through them — NVS too — but in steady
/// state the storage task is the only writer, and [`Snapshot`] deltas
/// are taken around its own calls.
#[cfg(storage_trace)]
mod trace {
    use core::ffi::c_void;
    use core::sync::atomic::{AtomicU32, Ordering};
    use esp_idf_svc::sys::{esp_err_t, esp_partition_t, esp_timer_get_time};

    static N: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];
    static US: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];
    static BYTES: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];

    fn record(i: usize, t0: i64, size: usize) {
        // SAFETY: no arguments.
        let dt = (unsafe { esp_timer_get_time() } - t0) as u32;
        N[i].fetch_add(1, Ordering::Relaxed);
        US[i].fetch_add(dt, Ordering::Relaxed);
        BYTES[i].fetch_add(size as u32, Ordering::Relaxed);
    }

    extern "C" {
        fn __real_esp_partition_read(
            p: *const esp_partition_t,
            off: usize,
            dst: *mut c_void,
            size: usize,
        ) -> esp_err_t;
        fn __real_esp_partition_write(
            p: *const esp_partition_t,
            off: usize,
            src: *const c_void,
            size: usize,
        ) -> esp_err_t;
        fn __real_esp_partition_erase_range(
            p: *const esp_partition_t,
            off: usize,
            size: usize,
        ) -> esp_err_t;
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_read(
        p: *const esp_partition_t,
        off: usize,
        dst: *mut c_void,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_read(p, off, dst, size);
        record(0, t0, size);
        r
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_write(
        p: *const esp_partition_t,
        off: usize,
        src: *const c_void,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_write(p, off, src, size);
        record(1, t0, size);
        r
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_erase_range(
        p: *const esp_partition_t,
        off: usize,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_erase_range(p, off, size);
        record(2, t0, size);
        r
    }

    /// Counts, µs and bytes for read / write / erase.
    #[derive(Clone, Copy)]
    pub struct Snapshot([(u32, u32, u32); 3]);

    pub fn snapshot() -> Snapshot {
        let mut s = [(0, 0, 0); 3];
        for (i, e) in s.iter_mut().enumerate() {
            *e = (
                N[i].load(Ordering::Relaxed),
                US[i].load(Ordering::Relaxed),
                BYTES[i].load(Ordering::Relaxed),
            );
        }
        Snapshot(s)
    }

    /// `read 12x/1536B/3ms prog 4x/640B/2ms erase 1x/4096B/41ms`.
    pub fn delta(a: Snapshot, b: Snapshot) -> String {
        let name = ["read", "prog", "erase"];
        let mut out = String::new();
        for i in 0..3 {
            let (n, us, by) = (
                b.0[i].0.wrapping_sub(a.0[i].0),
                b.0[i].1.wrapping_sub(a.0[i].1),
                b.0[i].2.wrapping_sub(a.0[i].2),
            );
            out.push_str(&format!(
                "{} {n}x/{by}B/{:.1}ms ",
                name[i],
                us as f32 / 1000.0
            ));
        }
        out
    }
}
