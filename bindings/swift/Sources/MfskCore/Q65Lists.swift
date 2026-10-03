// SPDX-License-Identifier: GPL-3.0-or-later
//
// The two lists WSJT-X keeps for Q65 (#466). The decoder is stateless about
// them, so they are the application's: feed them each decode, ask them for
// the DX station, and hand the callers to ``Decoder/setQ65Callers(_:)``.

import CMfsk

/// The 100 most recent Q65 decodes and their frequencies — WSJT-X's `q65_hist`.
/// It is how a "Decode Again" with no DX call entered finds the DX station, so
/// the full-AP list can be built without the operator typing the call.
///
/// The decoder is stateless, so the history is the application's: feed it each
/// decode with ``record(_:)``, ask with ``lookup(rxFrequencyHz:)``. **Not
/// thread-safe.**
public final class Q65History {
    let handle: OpaquePointer

    public init() throws {
        guard let opened = mfsk_q65_history_new() else {
            throw MfskError(status: MFSK_STATUS_INTERNAL, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_q65_history_free(handle) }

    /// The DX station ``lookup(rxFrequencyHz:)`` found — WSJT-X's `dxcall` and
    /// `dxgrid`.
    public struct DX: Sendable, Equatable {
        public var call: String
        public var grid: String?
    }

    /// Remember one decode at `frequencyHz` (tone 0); the 100 most recent are
    /// kept.
    public func push(frequencyHz: Float, message: String) throws {
        try check(mfsk_q65_history_push(handle, frequencyHz, message))
    }

    /// Remember every row of a decode, as `q65_decode.f90` calls `q65_hist`
    /// after each one.
    public func record(_ rows: [Decode]) throws {
        for row in rows { try push(frequencyHz: row.frequencyHz, message: row.text) }
    }

    public var count: Int { Int(mfsk_q65_history_len(handle)) }

    /// The DX station from the most recent decode within 10 Hz of
    /// `rxFrequencyHz` whose first word is 3 to 12 characters — so a `CQ ...`
    /// decode is passed over for an older one — or nil when nothing qualifies.
    public func lookup(rxFrequencyHz: Float) throws -> DX? {
        var raw = MfskQ65Dx()
        raw.size = UInt32(MemoryLayout<MfskQ65Dx>.size)
        let status = mfsk_q65_history_lookup(handle, rxFrequencyHz, &raw)
        if status == MFSK_STATUS_DECODE_FAILED { return nil }
        try check(status)
        return DX(call: stringFromCArray(raw.call),
                  grid: raw.has_grid != 0 ? stringFromCArray(raw.grid) : nil)
    }
}

/// The contest caller list — WSJT-X's `q65_hist2`: up to 50 stations that
/// called with a grid, from which the contest full-AP list is built
/// (``Decoder/setQ65Callers(_:)`` with ``DecodeParams/Contest/gridExchange``). Times are yours (Unix
/// seconds), since the library reads no clock. **Not thread-safe.**
public final class Q65Callers {
    let handle: OpaquePointer

    public init() throws {
        guard let opened = mfsk_q65_callers_new() else {
            throw MfskError(status: MFSK_STATUS_INTERNAL, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_q65_callers_free(handle) }

    /// One remembered station.
    public struct Caller: Sendable, Equatable {
        /// Up to six characters.
        public var call: String
        /// The four-character grid it sent.
        public var grid: String
        /// When it was last heard, as passed to ``record(frequencyHz:message:now:)``.
        public var lastHeard: UInt64
        /// Its audio frequency then, Hz.
        public var frequencyHz: Int32
    }

    /// Remember a decode at `frequencyHz` heard at `now`: a compound call is
    /// ignored, ` R ` is taken out, the second word is the caller and the next
    /// four characters its grid. A known caller is refreshed; a new one is
    /// added only if it sent a grid, the oldest making room once 50 are held.
    public func record(frequencyHz: Float, message: String, now: UInt64) throws {
        try check(mfsk_q65_callers_record(handle, frequencyHz, message, now))
    }

    /// Drop callers not heard for more than 24 hours. Call before each decode.
    public func expire(now: UInt64) throws {
        try check(mfsk_q65_callers_expire(handle, now))
    }

    /// Forget one caller (worked, say).
    public func remove(_ call: String) throws {
        try check(mfsk_q65_callers_remove(handle, call))
    }

    public var count: Int { Int(mfsk_q65_callers_len(handle)) }

    /// The stations, oldest first.
    public var callers: [Caller] {
        (0..<count).compactMap { index in
            var raw = MfskQ65Caller()
            raw.size = UInt32(MemoryLayout<MfskQ65Caller>.size)
            guard mfsk_q65_callers_get(handle, UInt(index), &raw) == MFSK_STATUS_OK else {
                return nil
            }
            return Caller(call: stringFromCArray(raw.call), grid: stringFromCArray(raw.grid),
                          lastHeard: raw.last_heard, frequencyHz: raw.freq_hz)
        }
    }
}
