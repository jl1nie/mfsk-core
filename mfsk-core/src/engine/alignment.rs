// SPDX-License-Identifier: GPL-3.0-only
//! Where a candidate sits, for a decode that does no search of its own.

/// A candidate's place in the audio: the sample where its first symbol starts and the frequency of its tone 0, Hz.
///
/// What a decode of one known candidate takes (`wspr::decode_candidate`, `jt65::decode_candidate`, …): a search finds
/// the position, this carries it. Not `engine::sync::SyncCandidate`, which is a search's *result* (a time in seconds from
/// the nominal start, and a score).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Alignment {
    /// Sample index of symbol 0 in the audio the decode reads (the audio's own rate).
    pub start_sample: usize,
    /// Frequency of tone 0, Hz.
    pub freq_hz: f32,
}

impl Alignment {
    pub const fn new(start_sample: usize, freq_hz: f32) -> Self {
        Self {
            start_sample,
            freq_hz,
        }
    }
}
