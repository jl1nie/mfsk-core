// SPDX-License-Identifier: GPL-3.0-only
//! Where a Q65 decode reads one symbol's samples from: [`SymbolWindows`].
//!
//! Every Q65 extraction (the symbol rows, the SNR's composite spectrum) reads **one symbol's `nsps` samples at a time**, in
//! increasing order. A slice serves that from memory; a [`PcmSource`] serves it from a store read forward (an SD card),
//! without ever holding the slot: [`PcmWindows`].

use alloc::vec::Vec;

use crate::engine::source::PcmSource;

/// `nsps`-sample windows of an audio, as f32 on the scale the Q65 decode works in (`i16 / 32768`).
pub(crate) trait SymbolWindows {
    /// Samples in the audio (a frame must fit in it).
    fn len(&self) -> usize;
    /// The `len` samples from `start`. The borrow ends before the next call, so a source may reuse one buffer.
    fn window(&mut self, start: usize, len: usize) -> &[f32];
}

impl SymbolWindows for &[f32] {
    fn len(&self) -> usize {
        <[f32]>::len(self)
    }

    fn window(&mut self, start: usize, len: usize) -> &[f32] {
        &self[start..start + len]
    }
}

/// The windows of a [`PcmSource`]: each read goes to the source (forward when the starts increase) and is converted with the
/// scale `decoder::q65` gives i16 audio, `1 / 32768`.
pub(crate) struct PcmWindows<'a, A: PcmSource> {
    src: &'a mut A,
    raw: Vec<i16>,
    buf: Vec<f32>,
}

impl<'a, A: PcmSource> PcmWindows<'a, A> {
    pub(crate) fn new(src: &'a mut A) -> Self {
        Self {
            src,
            raw: Vec::new(),
            buf: Vec::new(),
        }
    }
}

impl<A: PcmSource> SymbolWindows for PcmWindows<'_, A> {
    fn len(&self) -> usize {
        self.src.len()
    }

    fn window(&mut self, start: usize, len: usize) -> &[f32] {
        self.raw.resize(len, 0);
        self.src.read(start as i64, &mut self.raw);
        self.buf.clear();
        self.buf
            .extend(self.raw.iter().map(|&v| f32::from(v) / 32_768.0));
        &self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::source::SliceSource;

    /// A window of a source is `i16 / 32768`, exactly, and a slice's is the slice itself. (No decode output depends on the
    /// scale, so only a test of the windows can pin it.)
    #[test]
    fn a_source_window_is_the_i16_over_32768() {
        let raw = [16_384_i16, -32_768, 32_767, 0, 1];
        let mut src = SliceSource(&raw[..]);
        let mut w = PcmWindows::new(&mut src);
        assert_eq!(w.len(), 5);
        let got: Vec<f32> = w.window(0, 5).to_vec();
        assert_eq!(got, [0.5, -1.0, 32_767.0 / 32_768.0, 0.0, 1.0 / 32_768.0]);
        // a window is read from the start asked for, in any order a caller asks (forward or not)
        assert_eq!(w.window(2, 2), [32_767.0 / 32_768.0, 0.0]);
        assert_eq!(w.window(1, 1), [-1.0]);
    }

    #[test]
    fn a_slice_window_is_the_slice() {
        let data = [0.25_f32, -0.5, 0.75, 1.0];
        let mut w: &[f32] = &data;
        assert_eq!(SymbolWindows::len(&w), 4);
        assert_eq!(w.window(1, 2), [-0.5, 0.75]);
    }
}
