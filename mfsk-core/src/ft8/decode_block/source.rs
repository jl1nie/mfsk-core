// SPDX-License-Identifier: GPL-3.0-only
//! Where the audio comes from, for a decoder that cannot hold the slot.
//!
//! The ICAP radios hand an app 8 kHz audio that it has captured to the SD card (233 KB a slot), and the app
//! reads it back through `icap_read`: forward only, in calls of a few hundred samples, with no seek (a read
//! behind the file position closes and reopens the file and skips forward again; a read ahead of it skips by
//! reading). The decoder's data region is 240 KiB, so the slot cannot be resident, and *how many times and
//! how far the decoder reads* is a cost of its own (`apps/digi-common/digiapp.c`: on a slow card the call
//! overhead dominates).
//!
//! [`PcmSource`] is that interface: `read(start, out)`, zero outside the audio. [`SliceSource`] serves one
//! from memory (the host, the ESP32 drivers); [`CountingSource`] wraps a source and charges what the radio
//! would: every sample read, every sample skipped to get to a read ahead, every restart behind the position.
//! [`Window`] is the small forward cache a consumer reads through, so that overlapping reads (a spectrogram
//! frame is 1920 samples, 960 of them the previous frame's) are served from it, not the source.

use alloc::vec::Vec;

use super::types::AudioSample;

/// Audio read forward from a store. Samples outside `0..len()` read as zero.
pub trait PcmSource {
    /// Samples in the slot.
    fn len(&self) -> usize;

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Fill `out` with the samples `start..start + out.len()` (`start` may be negative).
    fn read(&mut self, start: i64, out: &mut [i16]);
}

/// A slot in memory.
pub struct SliceSource<'a, S: AudioSample>(pub &'a [S]);

impl<S: AudioSample> PcmSource for SliceSource<'_, S> {
    fn len(&self) -> usize {
        self.0.len()
    }

    fn read(&mut self, start: i64, out: &mut [i16]) {
        for (i, o) in out.iter_mut().enumerate() {
            let idx = start + i as i64;
            *o = if idx >= 0 && (idx as usize) < self.0.len() {
                self.0[idx as usize].to_i16()
            } else {
                0
            };
        }
    }
}

/// What a decoder read, as the radio would be charged for it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct ReadCost {
    /// `read` calls.
    pub calls: u64,
    /// Samples delivered to the caller from inside the audio.
    pub samples_read: u64,
    /// Samples the radio would read and throw away to reach a read ahead of the file position.
    pub samples_skipped: u64,
    /// Reads behind the file position: the file is closed, reopened and skipped through from the start.
    pub restarts: u64,
}

impl ReadCost {
    /// Samples the radio's SD reads move in all: delivered plus skipped.
    pub fn samples_moved(&self) -> u64 {
        self.samples_read + self.samples_skipped
    }

    /// Bytes moved (16-bit samples).
    pub fn bytes_moved(&self) -> u64 {
        2 * self.samples_moved()
    }
}

/// An [`PcmSource`] that counts what its reads would cost on the radio (see the module docs).
pub struct CountingSource<A: PcmSource> {
    inner: A,
    /// The file position after the last read.
    pos: u64,
    cost: ReadCost,
}

impl<A: PcmSource> CountingSource<A> {
    pub fn new(inner: A) -> Self {
        Self {
            inner,
            pos: 0,
            cost: ReadCost::default(),
        }
    }

    pub fn cost(&self) -> ReadCost {
        self.cost
    }

    pub fn into_inner(self) -> A {
        self.inner
    }
}

impl<A: PcmSource> PcmSource for CountingSource<A> {
    fn len(&self) -> usize {
        self.inner.len()
    }

    fn read(&mut self, start: i64, out: &mut [i16]) {
        let len = self.inner.len() as i64;
        // the part of the read inside the file
        let lo = start.max(0);
        let hi = (start + out.len() as i64).min(len);
        self.cost.calls += 1;
        if hi > lo {
            let (lo, hi) = (lo as u64, hi as u64);
            if lo < self.pos {
                // behind the file position: reopen, then skip to `lo` from the start
                self.cost.restarts += 1;
                self.pos = 0;
            }
            self.cost.samples_skipped += lo - self.pos;
            self.cost.samples_read += hi - lo;
            self.pos = hi;
        }
        self.inner.read(start, out);
    }
}

/// A forward cache over an [`PcmSource`]: reads are served from the last block read where they overlap
/// it, and fetched from the source in blocks of at least `BLOCK` samples, forward, otherwise.
pub struct Window<'a, A: PcmSource> {
    src: &'a mut A,
    buf: Vec<i16>,
    /// Absolute index of `buf[0]`.
    lo: i64,
}

/// Samples fetched from the source at a time (a multiple of the radio's 512-sample read).
const BLOCK: usize = 1024;

impl<'a, A: PcmSource> Window<'a, A> {
    pub fn new(src: &'a mut A) -> Self {
        Self {
            src,
            buf: Vec::new(),
            lo: 0,
        }
    }

    /// The source's length.
    pub fn len(&self) -> usize {
        self.src.len()
    }

    pub fn is_empty(&self) -> bool {
        self.src.is_empty()
    }

    /// Copy the samples `start..start + out.len()` into `out`.
    pub fn read_into(&mut self, start: i64, out: &mut [i16]) {
        let end = start + out.len() as i64;
        let have_end = self.lo + self.buf.len() as i64;
        if self.buf.is_empty() || start < self.lo || start > have_end {
            // nothing usable (first read, a restart, or a jump ahead): start the cache at `start`
            self.buf.clear();
            self.lo = start;
        } else if start > self.lo {
            // forward: drop what precedes `start`
            let drop = (start - self.lo) as usize;
            self.buf.drain(..drop.min(self.buf.len()));
            self.lo = start;
        }
        let have_end = self.lo + self.buf.len() as i64;
        if end > have_end {
            // fetch the rest forward, a whole number of blocks
            let need = (end - have_end) as usize;
            let n = need.div_ceil(BLOCK) * BLOCK;
            let at = self.buf.len();
            self.buf.resize(at + n, 0);
            self.src.read(have_end, &mut self.buf[at..]);
        }
        let off = (start - self.lo) as usize;
        out.copy_from_slice(&self.buf[off..off + out.len()]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp(n: usize) -> Vec<i16> {
        (0..n).map(|i| (i % 30000) as i16).collect()
    }

    #[test]
    fn a_slice_source_reads_zero_outside_the_audio() {
        let a = ramp(10);
        let mut s = SliceSource(&a[..]);
        let mut out = [7i16; 6];
        s.read(-2, &mut out);
        assert_eq!(out, [0, 0, 0, 1, 2, 3]);
        s.read(8, &mut out);
        assert_eq!(out, [8, 9, 0, 0, 0, 0]);
    }

    #[test]
    fn counting_charges_what_the_radio_would_move() {
        let a = ramp(10_000);
        let mut c = CountingSource::new(SliceSource(&a[..]));
        let mut b = [0i16; 100];
        c.read(0, &mut b); // 100 delivered
        c.read(100, &mut b); // contiguous: 100 more
        c.read(1000, &mut b); // ahead: skip 800, read 100
        assert_eq!(
            c.cost(),
            ReadCost {
                calls: 3,
                samples_read: 300,
                samples_skipped: 800,
                restarts: 0
            }
        );
        c.read(50, &mut b); // behind: reopen, skip 50 from the start, read 100
        assert_eq!(
            c.cost(),
            ReadCost {
                calls: 4,
                samples_read: 400,
                samples_skipped: 850,
                restarts: 1
            }
        );
        // the part of a read outside the file costs nothing
        c.read(9_950, &mut b);
        assert_eq!(c.cost().samples_read, 450);
    }

    #[test]
    fn a_window_serves_overlapping_forward_reads_from_one_pass() {
        let a = ramp(20_000);
        let mut c = CountingSource::new(SliceSource(&a[..]));
        {
            let mut w = Window::new(&mut c);
            let mut f = [0i16; 1920];
            // spectrogram frames: 1920 samples every 960
            for j in 0..18 {
                w.read_into(j * 960, &mut f);
                assert_eq!(&f[..], &a[j as usize * 960..j as usize * 960 + 1920]);
            }
        }
        let cost = c.cost();
        assert_eq!(cost.restarts, 0);
        assert_eq!(cost.samples_skipped, 0);
        // every sample of the span read once, plus at most one block of read-ahead
        assert!(cost.samples_read <= 17 * 960 + 1920 + 1024, "{cost:?}");
        assert!(cost.samples_read >= 17 * 960 + 1920, "{cost:?}");
    }

    #[test]
    fn a_window_reads_correctly_before_the_start_and_after_the_end() {
        let a = ramp(3000);
        let mut s = SliceSource(&a[..]);
        let mut w = Window::new(&mut s);
        let mut out = [9i16; 500];
        w.read_into(-100, &mut out);
        assert!(out[..100].iter().all(|&v| v == 0));
        assert_eq!(&out[100..], &a[..400]);
        w.read_into(2800, &mut out);
        assert_eq!(&out[..200], &a[2800..3000]);
        assert!(out[200..].iter().all(|&v| v == 0));
    }
}
