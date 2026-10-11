// SPDX-License-Identifier: GPL-3.0-only
//
// MSK144 — meteor scatter — `mfsk_msk144_*` (#497).
//
// WSJT-X's real-time receiver (`mskrtd`): audio goes in as it arrives and
// each ping comes out as soon as the 0.6 s window holding it is complete,
// instead of after the T/R period. That is why this is a class of its own
// rather than a ``Decoder`` mode: the mode's capabilities say
// ``Capabilities/streamReceiver``, not ``Capabilities/decodeHandle``.

import CMfsk

/// One MSK144 decode: a meteor ping.
public struct Msk144Decode: Sendable, Equatable {
    public let text: String
    /// dB, clamped to -8...24 (`mskrtd.f90`).
    public let snrDb: Int32
    public let frequencyHz: Float
    /// Seconds from the first sample pushed since the receiver was created
    /// or reset.
    public let seconds: Double

    init(_ raw: MfskMsk144Decode) {
        self.text = stringFromCArray(raw.text)
        self.snrDb = raw.snr_db
        self.frequencyHz = raw.freq_hz
        self.seconds = raw.t_s
    }
}

/// An MSK144 receiver: audio in as it arrives, in chunks of any size;
/// decodes out.
///
/// The noise floor carries over the session. Call ``newPeriod()`` at every
/// T/R period boundary the host keeps (15 s or 30 s): a repeat of the last
/// message is reported again after it, as upstream does.
///
/// **One thread at a time.**
public final class Msk144Receiver {
    let handle: OpaquePointer

    /// Open a receiver taking mono PCM at `sampleRate` (anything but
    /// 12 000 Hz is resampled, linearly). `frequencyHz` and `toleranceHz`
    /// are the search centre and half-width; `depth` is `ndepth`, 1...3.
    public init(sampleRate: UInt32 = 12_000, frequencyHz: Float = 1500,
                toleranceHz: Float = 100, depth: UInt32 = 3) throws {
        var status = MFSK_STATUS_OK
        guard let opened = mfsk_msk144_open(sampleRate, frequencyHz, toleranceHz, depth, &status)
        else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_msk144_close(handle) }

    /// Feed 16-bit PCM and return the decodes it produced, oldest first.
    @discardableResult
    public func push(_ samples: [Int16]) throws -> [Msk144Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_msk144_push_i16(handle, audio.baseAddress, UInt(audio.count)))
        }
        return try drain()
    }

    /// Feed 32-bit float PCM, nominally `-1.0...1.0`.
    @discardableResult
    public func push(_ samples: [Float]) throws -> [Msk144Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try check(mfsk_msk144_push_f32(handle, audio.baseAddress, UInt(audio.count)))
        }
        return try drain()
    }

    /// A new T/R period begins: a repeat of the last message is reported again.
    public func newPeriod() throws {
        try check(mfsk_msk144_new_period(handle))
    }

    /// Forget everything and start again at sample 0.
    public func reset() throws {
        try check(mfsk_msk144_reset(handle))
    }

    private func drain() throws -> [Msk144Decode] {
        var out: [Msk144Decode] = []
        while true {
            var raw = MfskMsk144Decode()
            raw.size = UInt32(MemoryLayout<MfskMsk144Decode>.size)
            let r = mfsk_msk144_poll(handle, &raw)
            if r == 1 {
                out.append(Msk144Decode(raw))
            } else if r == 0 {
                return out
            } else {
                throw MfskError(status: MfskStatus(rawValue: r), detail: globalLastError())
            }
        }
    }
}
