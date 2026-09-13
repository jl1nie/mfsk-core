// SPDX-License-Identifier: GPL-3.0-or-later
//
// The decode row and the search parameters, both mirroring
// size-versioned C structs. Rows go into memory this binding owns:
// nothing here frees a pointer the library allocated, which is the
// category the v2 C surface removed from the decode path — and the one
// that makes a wrapper leak when an error unwinds between the call and
// the free.

import CMfsk

/// One decoded transmission.
public struct Decode: Sendable, Equatable {
    /// The **concrete sub-mode**, not the family: all five FST4 periods
    /// stay distinct here because the addressing model does.
    public let mode: Mode
    public let text: String
    public let frequencyHz: Float
    /// Time offset from the slot's `dt = 0` reference, seconds.
    public let dtSeconds: Float
    /// Estimated SNR in a 2500 Hz reference bandwidth, dB.
    public let snrDB: Float
    public let syncScore: Float
    /// Coefficient of variation of the per-block sync powers — near 0
    /// on a stable channel, elevated under QSB. The only fading
    /// indicator the row carries.
    public let syncCV: Float
    /// Hard-decision errors the FEC had to correct.
    public let hardErrors: UInt32
    /// Width of the FEC information block — 91 (CRC-14) or 101 (CRC-24).
    /// Says how many bits ``DecodeSession/informationBits(at:)`` returns.
    public let informationBitCount: UInt16
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things per mode, and are for diagnostics,
    /// not for logic.
    public let pass: UInt8
    /// The text needed the callsign hash table to resolve a `<...>`
    /// reference — `MFSK_DECODE_FLAG_HASH_RESOLVED`, which is a named
    /// constant in the header only because the Kotlin shim found it
    /// missing.
    public let usedHashTable: Bool

    init(_ raw: MfskDecode) {
        self.mode = Mode(rawValue: UInt32(raw.mode.rawValue)) ?? .ft8
        self.text = stringFromCArray(raw.text)
        self.frequencyHz = raw.freq_hz
        self.dtSeconds = raw.dt_sec
        self.snrDB = raw.snr_db
        self.syncScore = raw.sync_score
        self.syncCV = raw.sync_cv
        self.hardErrors = raw.hard_errors
        self.informationBitCount = raw.info_bits
        self.pass = raw.pass
        self.usedHashTable = raw.flags & UInt8(MFSK_DECODE_FLAG_HASH_RESOLVED) != 0
    }
}

/// Everything a decode can be asked to do.
///
/// Start from a mode's published defaults — `init(mode:)` is
/// `mfsk_decode_params_init`, and zeroing the fields by hand is *not*
/// equivalent: a zero `maxCandidates` or a zero band decodes nothing,
/// and "no frequency hint" is NaN rather than 0 because 0 Hz is a
/// frequency. That last one is why ``frequencyHintHz`` is an `Optional`
/// here and NaN there.
public struct DecodeParams: Sendable {
    /// Cost/recall rung.
    public enum Depth: UInt32, Sendable {
        /// Whatever the mode publishes as its default.
        case modeDefault = 0
        /// Full LLR-variant staircase + BP, no OSD fallback.
        case bpAll = 1
        /// Above + OSD fallback (host-only).
        case bpAllOSD = 2
    }

    /// Accept/reject threshold profile.
    public enum Strictness: UInt32, Sendable {
        case strict = 0
        /// WSJT-X's own ceiling for FT8; independently-tuned values for
        /// FT4/FST4.
        case normal = 1
        /// Deliberately exceeds WSJT-X's own FT8 ceiling; exploratory.
        case deep = 2
    }

    /// Equalisation. A property of the **input audio** — it flattens a
    /// passband an analogue filter has tilted — not of the search,
    /// which is why it lives on every decode rather than only on a
    /// narrow-band one. It costs decodes on flat synthetic audio.
    public enum Equalisation: UInt32, Sendable {
        case off = 0
        /// Per-signal equalisation using local Costas pilot tones.
        case local = 1
    }

    /// An a-priori hypothesis: who is transmitting, to whom, from where.
    /// Each field is truncated to the ABI's 16-byte inline capacity.
    public struct APHint: Sendable, Equatable {
        public var call1: String
        public var call2: String
        public var grid: String

        public init(call1: String = "", call2: String = "", grid: String = "") {
            self.call1 = call1
            self.call2 = call2
            self.grid = grid
        }
    }

    public var frequencyRangeHz: ClosedRange<Float>
    /// **Not comparable across modes** — see ``SyncScale``.
    public var syncMin: Float
    public var maxCandidates: UInt32
    public var depth: Depth
    public var strictness: Strictness
    public var equalisation: Equalisation
    /// Prioritise candidates near this frequency; nil for no hint.
    public var frequencyHintHz: Float?
    /// Successive-interference-cancellation rounds. Needs
    /// ``Capabilities/sicRounds``.
    public var sicRounds: UInt8
    /// Checkpoint-emulation early decode. Needs ``Capabilities/sicEarly``.
    public var sicEarly: Bool
    /// A-priori hint, or nil. Needs ``Capabilities/apWideband``
    /// (or ``Capabilities/apNarrow`` on a narrow-band call).
    public var apHint: APHint?
    /// Half-width of a narrow-band search, Hz; 0 for the mode's default.
    /// Only meaningful with ``Capabilities/sniper``.
    public var searchHalfWidthHz: Float

    /// This mode's published defaults, as the starting point the ABI
    /// insists on.
    public init(mode: Mode) throws {
        var raw = MfskDecodeParams()
        raw.size = UInt32(MemoryLayout<MfskDecodeParams>.size)
        try check(mfsk_decode_params_init(mode.rawValue, &raw))
        self.init(raw)
    }

    init(_ raw: MfskDecodeParams) {
        self.frequencyRangeHz = raw.freq_min_hz...max(raw.freq_min_hz, raw.freq_max_hz)
        self.syncMin = raw.sync_min
        self.maxCandidates = raw.max_cand
        self.depth = Depth(rawValue: raw.depth.rawValue) ?? .modeDefault
        self.strictness = Strictness(rawValue: raw.strictness.rawValue) ?? .normal
        self.equalisation = Equalisation(rawValue: raw.eq_mode.rawValue) ?? .off
        self.frequencyHintHz = raw.freq_hint_hz.isNaN ? nil : raw.freq_hint_hz
        self.sicRounds = raw.sic_rounds
        self.sicEarly = raw.sic_early
        self.apHint = raw.has_ap_hint
            ? APHint(call1: stringFromCArray(raw.ap_call1),
                     call2: stringFromCArray(raw.ap_call2),
                     grid: stringFromCArray(raw.ap_grid))
            : nil
        self.searchHalfWidthHz = raw.search_hz
    }

    /// Marshal into the C struct for the duration of `body`. One copy,
    /// not a sequence of fallible setter calls — which is one of the
    /// three reasons the ABI replaced its options handle with a struct.
    func withC<R>(_ body: (UnsafePointer<MfskDecodeParams>) throws -> R) rethrows -> R {
        var raw = MfskDecodeParams()
        raw.size = UInt32(MemoryLayout<MfskDecodeParams>.size)
        raw.freq_min_hz = frequencyRangeHz.lowerBound
        raw.freq_max_hz = frequencyRangeHz.upperBound
        raw.sync_min = syncMin
        raw.max_cand = maxCandidates
        raw.depth = MfskDecodeDepth(rawValue: depth.rawValue)
        raw.strictness = MfskStrictness(rawValue: strictness.rawValue)
        raw.eq_mode = MfskEqMode(rawValue: equalisation.rawValue)
        raw.freq_hint_hz = frequencyHintHz ?? Float.nan
        raw.sic_rounds = sicRounds
        raw.sic_early = sicEarly
        raw.has_ap_hint = apHint != nil
        if let hint = apHint {
            setCArray(&raw.ap_call1, to: hint.call1)
            setCArray(&raw.ap_call2, to: hint.call2)
            setCArray(&raw.ap_grid, to: hint.grid)
        }
        raw.search_hz = searchHalfWidthHz
        return try withUnsafePointer(to: &raw) { try body($0) }
    }
}

/// Run a call that fills caller-owned rows, growing the buffer if the
/// library says it was short.
///
/// The ABI's contract is that `*out_len` always receives the number of
/// decodes found, so a short buffer comes back as `INVALID_ARG` with
/// the required count rather than a truncated answer you cannot detect.
/// One retry is enough: the second buffer is sized from that count.
func collectRows(
    capacity: Int = 32,
    errorDetail: () -> String? = { globalLastError() },
    _ call: (UnsafeMutablePointer<MfskDecode>, UInt, UnsafeMutablePointer<UInt>) -> MfskStatus
) throws -> [Decode] {
    var template = MfskDecode()
    template.size = UInt32(MemoryLayout<MfskDecode>.size)
    var rows = [MfskDecode](repeating: template, count: max(capacity, 1))
    var found: UInt = 0
    let status = rows.withUnsafeMutableBufferPointer { buffer in
        call(buffer.baseAddress!, UInt(buffer.count), &found)
    }
    if status != MFSK_STATUS_OK {
        if status == MFSK_STATUS_INVALID_ARG, Int(found) > rows.count {
            return try collectRows(capacity: Int(found), errorDetail: errorDetail, call)
        }
        throw MfskError(status: status, detail: errorDetail())
    }
    return rows.prefix(Int(found)).map(Decode.init)
}
