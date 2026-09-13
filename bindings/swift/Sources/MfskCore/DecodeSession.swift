// SPDX-License-Identifier: GPL-3.0-or-later
//
// The decode session — `mfsk_session_*`. "Session" rather than
// "decoder" because of what it owns: a callsign hash table and the
// previous slot's rows, both of which only mean anything across more
// than one call.

import CMfsk

/// A decoder for one mode, carrying the state that spans slots.
///
/// Modes with ``Capabilities/decodeHandle`` decode through this. The
/// others have their own entry points — ``WSPR``, ``JT9``, ``JT65`` —
/// because they are shaped differently, not because they are lesser.
public final class DecodeSession {
    let handle: OpaquePointer
    public let mode: Mode

    /// Open a session, validated against `params`.
    ///
    /// **A parameter the mode does not support is an error here**, not a
    /// field silently dropped at decode time.
    public init(mode: Mode, params: DecodeParams? = nil) throws {
        var status = MFSK_STATUS_OK
        var opened: OpaquePointer?
        if let params {
            opened = params.withC { mfsk_session_open(mode.rawValue, $0, &status) }
        } else {
            opened = mfsk_session_open(mode.rawValue, nil, &status)
        }
        guard let opened else {
            throw MfskError(status: status, detail: globalLastError())
        }
        self.handle = opened
        self.mode = mode
    }

    deinit { mfsk_session_close(handle) }

    /// The last error recorded **on this handle**.
    ///
    /// Prefer this over the thread-local global whenever a session is in
    /// hand: the global reads nil after an `async` caller hops threads
    /// between the failing call and the question.
    ///
    /// Not every failure lands here. `mfsk_session_copy_info` takes the
    /// handle as `const*`, so it has nowhere to write one and records
    /// only the thread-local — which is why every throw from this type
    /// falls back to that, rather than dropping the reason on the floor.
    public var lastError: String? {
        guard let p = mfsk_session_last_error(handle) else { return nil }
        return String(cString: p)
    }

    /// Whichever of the two error slots the failing call actually wrote.
    private func failureDetail() -> String? { lastError ?? globalLastError() }

    /// Decode one slot of 16-bit PCM.
    ///
    /// `params` overrides the session's own for this call only, and is
    /// validated the same way.
    public func decode(
        _ samples: [Int16],
        sampleRate: UInt32 = 12_000,
        params: DecodeParams? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_i16(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, p, out, capacity, found)
                }
            }
        }
    }

    /// Decode one slot of 32-bit float PCM, nominally `-1.0...1.0`.
    ///
    /// Not a lossier wrapper: the float path resamples in float and
    /// converts once at the end, where the decoders take i16 anyway.
    public func decode(
        _ samples: [Float],
        sampleRate: UInt32 = 12_000,
        params: DecodeParams? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_f32(handle, audio.baseAddress, UInt(audio.count),
                                            sampleRate, p, out, capacity, found)
                }
            }
        }
    }

    /// Decode the stream's buffered slot in place, or nil if no slot is
    /// ready yet.
    ///
    /// Fused on purpose: FST4-300's slot is 3 600 000 samples, and
    /// taking it out only to hand it back moves 7 MB for nothing.
    public func decode(
        _ stream: CaptureStream,
        params: DecodeParams? = nil
    ) throws -> (decodes: [Decode], slotStartUTC: Double)? {
        var slotStart: Double = 0
        do {
            let rows = try collectRows(errorDetail: { self.failureDetail() }) { out, capacity, found in
                withParams(params) { p in
                    mfsk_session_decode_stream(handle, stream.handle, p, out, capacity, found, &slotStart)
                }
            }
            return (rows, slotStart)
        } catch let error as MfskError where error.code == .unsupported {
            // The documented "no slot ready yet" answer, so a caller can
            // poll this instead of asking `slotReady` first.
            return nil
        }
    }

    /// Teach the session a callsign, so a later slot's `<...>` reference
    /// to it resolves.
    ///
    /// Decoded messages populate the table by themselves; this is for
    /// callsigns known from outside — a band map, an operator's log.
    public func addCallsign(_ call: String) throws {
        try check(mfsk_session_add_callsign(handle, call), detail: failureDetail())
    }

    /// The FEC information bits of the `index`-th row of the last
    /// decode. `Decode.informationBitCount` says how many there are.
    ///
    /// Deliberately not a row field: 91 or 101 bytes that only a caller
    /// doing subtraction or persistence wants.
    public func informationBits(at index: Int) throws -> [UInt8] {
        // 101 (CRC-24) is the widest the ABI documents; ask for 128 so a
        // wider block would come back as a short-buffer error rather
        // than a silently truncated answer.
        var bits = [UInt8](repeating: 0, count: 128)
        var written: UInt = 0
        let status = bits.withUnsafeMutableBufferPointer { buffer in
            mfsk_session_copy_info(handle, UInt(index), buffer.baseAddress, UInt(buffer.count), &written)
        }
        try check(status, detail: failureDetail())
        return Array(bits.prefix(Int(written)))
    }

    private func withParams<R>(
        _ params: DecodeParams?,
        _ body: (UnsafePointer<MfskDecodeParams>?) -> R
    ) -> R {
        guard let params else { return body(nil) }
        return params.withC { body($0) }
    }
}
