//! `mfsk_runtime_configure` — the pool every decode runs on.
//!
//! The plan that introduced this called it the single most important
//! mobile fix, and there was no hook for it at any layer before. Even
//! with `parallel` on, decoding used rayon's **global** pool:
//! `num_cpus` threads with 2 MiB stacks, spawned lazily on the first
//! decode and never joined. On Android those threads are not attached
//! to ART, so a callback from one cannot touch a JNIEnv; on iOS they
//! sit outside GCD's QoS classes; on both they keep running after the
//! app is backgrounded.
//!
//! What is actually asserted here is that the configuration **takes
//! effect** — a pool that is accepted and then not used would look
//! identical from the outside, which is exactly the failure mode this
//! whole redesign exists to end.
//!
//! Single test function on purpose: the pool is process-global and can
//! only be built once, so the ordering has to be explicit rather than
//! left to the harness.
//!
//! And it is feature-split, because there are two contracts. Without
//! `parallel` there is no pool to configure — decoding is already
//! single-threaded, which is a *stronger* guarantee than the pool
//! provides, not a missing one — and the call must say so rather than
//! pretend to succeed. CI runs this suite under `--features mobile`
//! too, which is how the first version of this test was caught
//! asserting only half of what the library documents.

mod common;

use std::ffi::c_void;
use std::sync::atomic::{AtomicU32, Ordering};

use common::*;
use mfsk::*;

static STARTS: AtomicU32 = AtomicU32::new(0);
static STOPS: AtomicU32 = AtomicU32::new(0);
static USER_SEEN: AtomicU32 = AtomicU32::new(0);

const USER_TAG: u32 = 0xC0FFEE;

unsafe extern "C" fn on_start(_index: u32, user: *mut c_void) {
    if !user.is_null() && unsafe { *(user as *const u32) } == USER_TAG {
        USER_SEEN.fetch_add(1, Ordering::SeqCst);
    }
    STARTS.fetch_add(1, Ordering::SeqCst);
}

unsafe extern "C" fn on_stop(_index: u32, _user: *mut c_void) {
    STOPS.fetch_add(1, Ordering::SeqCst);
}

fn cfg(threads: u32, user: *mut c_void) -> MfskRuntimeConfig {
    MfskRuntimeConfig {
        size: std::mem::size_of::<MfskRuntimeConfig>() as u32,
        num_threads: threads,
        thread_stack_bytes: 512 * 1024,
        on_thread_start: Some(on_start),
        on_thread_stop: Some(on_stop),
        thread_user: user,
    }
}

/// Without `parallel`: nothing to configure, and it says so.
#[cfg(not(feature = "parallel"))]
#[test]
fn a_build_without_a_pool_says_so() {
    assert_eq!(
        unsafe { mfsk_runtime_configure(&cfg(4, std::ptr::null_mut())) },
        MfskStatus::Unsupported
    );
    let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }.to_string_lossy();
    assert!(
        msg.contains("single-threaded"),
        "the error should say decoding is already serial, not that something broke: {msg}"
    );
    assert_eq!(
        mfsk_runtime_thread_count(),
        1,
        "a build without `parallel` decodes on one thread"
    );

    // And a NULL config is the same answer, not a crash.
    assert_eq!(
        unsafe { mfsk_runtime_configure(std::ptr::null()) },
        MfskStatus::Unsupported
    );

    // The decode still works; it just does not use a pool.
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let dec = open(MfskMode::Ft8, None, None);
    let rows = decode_i16(dec, &slot);
    unsafe { mfsk_decoder_close(dec) };
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    assert_eq!(
        STARTS.load(Ordering::SeqCst),
        0,
        "no worker thread should have started"
    );
}

#[cfg(feature = "parallel")]
#[test]
fn the_configured_pool_is_the_one_decoding_runs_on() {
    // Before configuring, the count is rayon's global default.
    let before = mfsk_runtime_thread_count();
    assert!(before >= 1);

    let tag = USER_TAG;
    let user = &tag as *const u32 as *mut c_void;
    assert_eq!(
        unsafe { mfsk_runtime_configure(&cfg(3, user)) },
        MfskStatus::Ok
    );

    // **The configuration took effect**, not merely returned OK.
    assert_eq!(
        mfsk_runtime_thread_count(),
        3,
        "decoding should now report the pool it was told to build"
    );

    // A second call is refused rather than silently ignored: rayon
    // cannot rebuild a pool its threads may be parked in.
    assert_eq!(
        unsafe { mfsk_runtime_configure(&cfg(2, std::ptr::null_mut())) },
        MfskStatus::Unsupported
    );
    let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }.to_string_lossy();
    assert!(msg.contains("once"), "{msg}");
    assert_eq!(
        mfsk_runtime_thread_count(),
        3,
        "the refused call must not have changed anything"
    );

    // Decode on the pool and confirm the workers actually started —
    // which is what says the hooks reach a JNI consumer's
    // AttachCurrentThread.
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let dec = open(MfskMode::Ft8, None, None);
    let rows = decode_i16(dec, &slot);
    unsafe { mfsk_decoder_close(dec) };
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));

    let started = STARTS.load(Ordering::SeqCst);
    assert!(
        started > 0,
        "no worker thread ever started — the pool was built and then not used"
    );
    assert!(
        started <= 3,
        "started {started} threads for a pool configured with 3"
    );
    assert_eq!(
        USER_SEEN.load(Ordering::SeqCst),
        started,
        "every hook call should have carried the user pointer it was given"
    );

    // A NULL config is legal and means rayon's defaults — but it is
    // refused here because the pool already exists, which is the same
    // "once" rule.
    assert_eq!(
        unsafe { mfsk_runtime_configure(std::ptr::null()) },
        MfskStatus::Unsupported
    );

    // Size versioning: an older caller declaring a shorter struct must
    // be read as far as it declared and no further.
    let full = std::mem::size_of::<MfskRuntimeConfig>();
    let short = std::mem::offset_of!(MfskRuntimeConfig, thread_stack_bytes);
    let mut bytes = vec![0u8; full];
    bytes[..4].copy_from_slice(&(short as u32).to_ne_bytes());
    bytes[4..8].copy_from_slice(&7u32.to_ne_bytes());
    // Still refused (already configured), but it must not crash or read
    // past what the caller declared.
    assert_eq!(
        unsafe { mfsk_runtime_configure(bytes.as_ptr() as *const MfskRuntimeConfig) },
        MfskStatus::Unsupported
    );
}
