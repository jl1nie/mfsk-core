// SPDX-License-Identifier: GPL-3.0-or-later
//
// Streaming capture — `mfsk_stream_*`. The ring holds exactly one slot;
// pushing more than that before taking one overwrites the oldest audio,
// which is the right failure for a live receiver: the newest slot is
// the one worth decoding.

import CMfsk

/// A one-slot capture ring for a mode, fed from an audio callback.
///
/// **Time enters as a parameter.** The library reads no clock, which is
/// what keeps one type usable both from a phone that was backgrounded
/// and from a replayed recording — see ``setEpoch(utcSeconds:)``.
public final class CaptureStream {
    let handle: OpaquePointer
    public let mode: Mode
    /// Samples in a full slot at 12 kHz, i.e. what ``takeSlot()``
    /// returns. Asked of the mode rather than assumed: the five FST4
    /// sub-modes differ by a factor of 30 here.
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
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_stream_push_i16(handle, audio.baseAddress, UInt(audio.count)))
        }
    }

    /// Push 32-bit float PCM, nominally `-1.0...1.0`.
    public func push(_ samples: [Float]) throws {
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_stream_push_f32(handle, audio.baseAddress, UInt(audio.count)))
        }
    }

    /// How many 12 kHz samples are buffered.
    public var bufferedSamples: Int { Int(mfsk_stream_buffered(handle)) }

    /// Whether a whole slot is buffered and ready to take.
    public var isSlotReady: Bool { mfsk_stream_slot_ready(handle) }

    /// Tell the stream what UTC second the **next** sample pushed belongs
    /// to, so slot boundaries land where the protocol says.
    ///
    /// Without this the grid free-runs from the first sample, which is
    /// exactly right for replaying a recording and wrong for a live
    /// receiver. Call it whenever the clock is resynchronised; the grid
    /// re-anchors from that point rather than shifting what is already
    /// buffered.
    public func setEpoch(utcSeconds: Double) {
        mfsk_stream_set_epoch(handle, utcSeconds)
    }

    /// Take the buffered slot, with the UTC second its first sample fell
    /// on. Nil if no slot is ready.
    ///
    /// ``DecodeSession/decode(_:params:)-(CaptureStream,_)`` decodes the
    /// same slot without this copy; take it only when the audio itself
    /// is wanted — to write a WAV, say.
    public func takeSlot() -> (samples: [Int16], slotStartUTC: Double)? {
        guard slotSamples > 0 else { return nil }
        var samples = [Int16](repeating: 0, count: slotSamples)
        var slotStart: Double = 0
        let written = samples.withUnsafeMutableBufferPointer { buffer in
            mfsk_stream_take_slot_i16(handle, buffer.baseAddress, UInt(buffer.count), &slotStart)
        }
        guard written > 0 else { return nil }
        return (Array(samples.prefix(Int(written))), slotStart)
    }

    /// Drop everything buffered, keeping the epoch.
    public func clear() { mfsk_stream_clear(handle) }
}
