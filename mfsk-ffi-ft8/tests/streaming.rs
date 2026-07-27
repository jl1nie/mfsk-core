//! Host tests for the `mfsk_ft8_stream_*` C ABI.
//!
//! Exercises the streaming wrapper end-to-end: resampling 16/24/48 kHz
//! input down to 12 kHz, ring-buffer rolling-window semantics
//! (`peek_latest` after exceeding capacity), and chunk-boundary
//! continuity (the same input split into different chunk sizes must
//! produce the same buffered samples).
//!
//! Built only with the default `host` feature — these tests use the
//! caller-buffer i16 ABI, which is sufficient to validate the
//! streaming primitives without needing the FFT planner /
//! dot-product backend that the embedded path would require.

use mfsk_ft8::*;

#[test]
fn new_and_free_are_safe() {
    let s = mfsk_ft8_stream_new(16_000, 180_000);
    assert!(!s.is_null());
    unsafe { mfsk_ft8_stream_free(s) };

    // Null free is a no-op.
    unsafe { mfsk_ft8_stream_free(core::ptr::null_mut()) };

    // Bad parameters return null.
    assert!(mfsk_ft8_stream_new(0, 180_000).is_null());
    assert!(mfsk_ft8_stream_new(48_000, 0).is_null());
}

#[test]
fn passthrough_at_12k_is_lossless() {
    let s = mfsk_ft8_stream_new(12_000, 1024);
    let input: Vec<i16> = (0..1000).map(|i| (i as i16).wrapping_mul(7)).collect();
    unsafe {
        let st = mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());
        assert_eq!(st, MfskStatus::Ok);
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 1000);

        let mut out = vec![0i16; 1000];
        let n = mfsk_ft8_stream_peek_latest(s, out.as_mut_ptr(), 1000);
        assert_eq!(n, 1000);
        assert_eq!(out, input);

        mfsk_ft8_stream_free(s);
    }
}

#[test]
fn resample_48k_to_12k_decimates() {
    let s = mfsk_ft8_stream_new(48_000, 4096);
    // 4 s of 48 kHz audio = 192_000 samples → ~48_000 samples at 12 kHz.
    // Use a slowly-varying ramp so resampling errors stay small.
    let input: Vec<i16> = (0..4800).map(|i| (i / 4) as i16).collect();
    unsafe {
        mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());
        // 4800 / 4 = 1200 samples should be buffered at 12 kHz.
        let buf = mfsk_ft8_stream_buffered_samples(s);
        assert!(
            (buf as i64 - 1200).abs() <= 2,
            "expected ~1200 buffered, got {buf}"
        );
        mfsk_ft8_stream_free(s);
    }
}

#[test]
fn ring_buffer_overwrites_oldest() {
    // Capacity = 100 at 12 kHz. Push 250 samples at 12 kHz; only the
    // last 100 survive.
    let s = mfsk_ft8_stream_new(12_000, 100);
    let input: Vec<i16> = (0..250).map(|i| i as i16).collect();
    unsafe {
        mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 100);

        let mut out = vec![0i16; 100];
        let n = mfsk_ft8_stream_peek_latest(s, out.as_mut_ptr(), 100);
        assert_eq!(n, 100);
        // Last 100 of input are 150..249.
        let expected: Vec<i16> = (150..250).collect();
        assert_eq!(out, expected);

        mfsk_ft8_stream_free(s);
    }
}

#[test]
fn drain_advances_tail() {
    let s = mfsk_ft8_stream_new(12_000, 1000);
    let input: Vec<i16> = (0..500).collect();
    unsafe {
        mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 500);

        // Drain first 200; remaining buffered = 300.
        mfsk_ft8_stream_drain(s, 200);
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 300);

        // peek_latest(300) = last 300 of original = 200..499.
        let mut out = vec![0i16; 300];
        let n = mfsk_ft8_stream_peek_latest(s, out.as_mut_ptr(), 300);
        assert_eq!(n, 300);
        let expected: Vec<i16> = (200..500).collect();
        assert_eq!(out, expected);

        // Drain more than buffered: clamps to len.
        mfsk_ft8_stream_drain(s, 999_999);
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 0);

        mfsk_ft8_stream_free(s);
    }
}

#[test]
fn chunk_boundary_is_seamless() {
    // The streaming resampler must give the same buffered output
    // whether the input arrived in one big push or many small ones.
    let input: Vec<i16> = (0..16_000).map(|i| ((i * 13) % 32768) as i16).collect();

    // One-shot push.
    let s1 = mfsk_ft8_stream_new(16_000, 16_000);
    unsafe {
        mfsk_ft8_stream_push_i16(s1, input.as_ptr(), input.len());
    }

    // Chunked push (odd chunk size to land on resampler-internal
    // phase boundaries).
    let s2 = mfsk_ft8_stream_new(16_000, 16_000);
    let chunk = 137;
    unsafe {
        let mut pos = 0;
        while pos < input.len() {
            let end = (pos + chunk).min(input.len());
            mfsk_ft8_stream_push_i16(s2, input[pos..].as_ptr(), end - pos);
            pos = end;
        }
    }

    let n1 = unsafe { mfsk_ft8_stream_buffered_samples(s1) };
    let n2 = unsafe { mfsk_ft8_stream_buffered_samples(s2) };
    assert_eq!(n1, n2, "chunked push produced different buffered count");

    let mut out1 = vec![0i16; n1];
    let mut out2 = vec![0i16; n2];
    unsafe {
        mfsk_ft8_stream_peek_latest(s1, out1.as_mut_ptr(), n1);
        mfsk_ft8_stream_peek_latest(s2, out2.as_mut_ptr(), n2);
    }
    assert_eq!(out1, out2, "chunked push produced different samples");

    unsafe {
        mfsk_ft8_stream_free(s1);
        mfsk_ft8_stream_free(s2);
    }
}

#[test]
fn clear_resets_state() {
    let s = mfsk_ft8_stream_new(48_000, 1000);
    let input: Vec<i16> = (0..2000).map(|i| i as i16).collect();
    unsafe {
        mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());
        assert!(mfsk_ft8_stream_buffered_samples(s) > 0);

        mfsk_ft8_stream_clear(s);
        assert_eq!(mfsk_ft8_stream_buffered_samples(s), 0);

        // After clear, push the same input — buffered count must
        // match a fresh stream's count (resampler phase was reset).
        let s_fresh = mfsk_ft8_stream_new(48_000, 1000);
        mfsk_ft8_stream_push_i16(s_fresh, input.as_ptr(), input.len());
        mfsk_ft8_stream_push_i16(s, input.as_ptr(), input.len());

        assert_eq!(
            mfsk_ft8_stream_buffered_samples(s),
            mfsk_ft8_stream_buffered_samples(s_fresh),
        );

        mfsk_ft8_stream_free(s);
        mfsk_ft8_stream_free(s_fresh);
    }
}
