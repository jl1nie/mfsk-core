// SPDX-License-Identifier: GPL-3.0-only
//
// Streaming capture — `mfsk_stream_*`. Audio goes in as it arrives, in chunks
// of any size; slots come out cut on the mode's UTC grid. A completed slot
// that is not taken before the next completes is replaced by it (and counted
// in ``CaptureStream/droppedSlots``), which is the right failure for a live
// receiver: the newest slot is the one worth decoding.

import CMfsk

/// What ``CaptureStream/setTime(utcNanoseconds:atSample:)`` did to the
/// stream's clock.
public enum ClockChange: Int32, Sendable {
    /// The first reading: the clock was anchored on it.
    case first = 0
    /// The clock moved towards the reading by at most its slew limit
    /// (400 ppm); open slots are unaffected.
    case slewed = 1
    /// The reading was more than a second away: the clock re-anchored and
    /// the slot that straddled the jump is dropped.
    case stepped = 2
}

/// A capture ring for a mode, fed from an audio callback.
///
/// **Time enters as a parameter.** The library reads no clock, which is what
/// keeps one type usable both from a phone that was backgrounded and from a
/// replayed recording: without a reading the grid free-runs from the first
/// sample, right for a recording; a live receiver tells the stream what time
/// it is as often as it has a reading, and the stream *follows* it.
///
/// Not thread-safe: push and take from one thread, or serialise.
public final class CaptureStream {
    let handle: OpaquePointer
    public let mode: Mode
    /// Samples in a full slot at 12 kHz, i.e. what ``takeSlot()`` returns.
    /// Asked of the mode rather than assumed: the five FST4 sub-modes differ
    /// by a factor of 30 here.
    public let slotSamples: Int

    public init(mode: Mode, sampleRate: UInt32 = 12_000) throws {
        var status = MFSK_STATUS_OK
        guard let opened = mfsk_stream_open(mode.rawValue, sampleRate, &status) else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
        self.mode = mode
        self.slotSamples = Int((try? mode.info.slotSamples12k) ?? 0)
    }

    deinit { mfsk_stream_close(handle) }

    /// Push 16-bit PCM at the rate the stream was opened with.
    public func push(_ samples: [Int16]) throws {
        guard !samples.isEmpty else { return }
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_stream_push_i16(handle, audio.baseAddress, UInt(audio.count)))
        }
    }

    /// Push 32-bit float PCM, nominally `-1.0...1.0`.
    public func push(_ samples: [Float]) throws {
        guard !samples.isEmpty else { return }
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_stream_push_f32(handle, audio.baseAddress, UInt(audio.count)))
        }
    }

    /// How many 12 kHz samples the stream has taken in: its clock, to pass
    /// as `atSample` to ``setTime(utcNanoseconds:atSample:)``.
    public var position: UInt64 { mfsk_stream_position(handle) }

    /// Whether a completed slot is waiting.
    public var isSlotReady: Bool { mfsk_stream_slot_ready(handle) }

    /// Completed slots a newer one replaced before they were taken.
    public var droppedSlots: UInt64 { mfsk_stream_dropped(handle) }

    /// The stream's sample `atSample` (12 kHz, as ``position`` counts) was at
    /// UTC `utcNanoseconds` (since the Unix epoch). nil for `atSample` is
    /// "the sample just pushed", i.e. ``position``.
    ///
    /// Call it as often as you have a reading: the stream follows readings at
    /// up to 400 ppm, so noisy readings and a drifting clock move slot
    /// boundaries by milliseconds and lose nothing. Only a jump of more than
    /// a second steps it, and drops the slot that straddled the jump.
    @discardableResult
    public func setTime(utcNanoseconds: Int64, atSample: UInt64? = nil) throws -> ClockChange {
        var change: Int32 = 0
        try check(mfsk_stream_set_time(handle, utcNanoseconds, atSample ?? position, &change))
        return ClockChange(rawValue: change) ?? .stepped
    }

    /// A slot taken out of the stream.
    public struct Slot: Sendable, Equatable {
        public let samples: [Int16]
        /// The slot's index on the mode's UTC grid.
        public let period: Int64
        /// UTC of its first sample, nanoseconds since the Unix epoch, or nil
        /// when no clock was set.
        public let startUTCNanoseconds: Int64?

        /// ``startUTCNanoseconds`` in seconds.
        public var startUTC: Double? { startUTCNanoseconds.map { Double($0) / 1e9 } }
    }

    /// Take the ready slot, with its index and (when a clock is set) its UTC
    /// start. Nil if no slot is ready.
    ///
    /// ``Decoder``'s stream decode decodes the same slot
    /// without this copy; take it only when the audio itself is wanted — to
    /// write a WAV, say.
    public func takeSlot() -> Slot? {
        guard slotSamples > 0 else { return nil }
        var samples = [Int16](repeating: 0, count: slotSamples)
        var period: Int64 = 0
        var utc: Int64 = 0
        let written = samples.withUnsafeMutableBufferPointer { buffer in
            mfsk_stream_take_slot_i16(handle, buffer.baseAddress, UInt(buffer.count), &period, &utc)
        }
        guard written > 0 else { return nil }
        return Slot(samples: Array(samples.prefix(Int(written))), period: period,
                    startUTCNanoseconds: utc == 0 ? nil : utc)
    }

    /// Drop the waiting slot and the one being cut, keeping the clock.
    public func clear() { mfsk_stream_clear(handle) }
}
