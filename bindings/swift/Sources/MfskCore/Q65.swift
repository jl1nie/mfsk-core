// SPDX-License-Identifier: GPL-3.0-or-later
//
// Q65 — the family with its own function set rather than a decode
// session, because its addressing model is different: a sub-mode
// numbering of its own, four decode strategies that trade computation
// or extra inputs for threshold, and a callsign hash table passed in
// rather than owned.
//
// The sub-mode enums reach C because `cbindgen.toml` asks for them by
// name: every `mfsk_q65_*` function takes its sub-mode as `uint32_t`,
// deliberately — an out-of-range value is then a C int rather than an
// invalid Rust discriminant — so no signature mentions the enum and
// cbindgen would otherwise emit neither.

import CMfsk

/// Q65's ten wired sub-modes.
///
/// **These discriminants are Q65's own, not ``Mode``'s**, and they are
/// not in slot-length order: `a15` is 6 because it was appended rather
/// than inserted, so the numbers already in the ABI stayed put. Use
/// ``mode`` to get the `Mode` a decode row will report.
public enum Q65SubMode: UInt32, CaseIterable, Sendable {
    /// 30 s, ×1 spacing. Terrestrial weak-signal HF/VHF and
    /// ionoscatter; the most common sub-mode.
    case a30 = 0
    /// 60 s, ×1. 6 m EME.
    case a60 = 1
    /// 60 s, ×2. 70 cm / 23 cm EME.
    case b60 = 2
    /// 60 s, ×4. ~3 GHz microwave EME.
    case c60 = 3
    /// 60 s, ×8. 5.7 / 10 GHz EME — libration spread wants
    /// ``Q65/decode(_:subMode:sampleRate:fadingB90Ts:model:hashTable:)``.
    case d60 = 4
    /// 60 s, ×16. 24 GHz+ / extreme spread.
    case e60 = 5
    /// 15 s, ×1. The fastest wired sub-mode; stable paths that prefer a
    /// shorter T/R period over Q65-30A's sensitivity margin.
    case a15 = 6
    /// 120 s, ×8. 10 GHz rainscatter / troposcatter.
    case d120 = 7
    /// 120 s, ×16. 6 m ionoscatter with heavy spread.
    case e120 = 8
    /// 300 s, ×1. The deepest wired sub-mode.
    case a300 = 9

    /// The ``Mode`` this sub-mode addresses — the identity a decode row
    /// carries, and the one to ask for geometry.
    public var mode: Mode {
        switch self {
        case .a15: return .q65a15
        case .a30: return .q65a30
        case .a60: return .q65a60
        case .b60: return .q65b60
        case .c60: return .q65c60
        case .d60: return .q65d60
        case .e60: return .q65e60
        case .d120: return .q65d120
        case .e120: return .q65e120
        case .a300: return .q65a300
        }
    }

    /// The sub-mode a ``Mode`` names, or nil if it is not a Q65 mode.
    public init?(_ mode: Mode) {
        guard let match = Q65SubMode.allCases.first(where: { $0.mode == mode }) else { return nil }
        self = match
    }
}

/// Channel model for the fast-fading decoder.
public enum Q65FadingModel: UInt32, Sendable {
    /// Gaussian spread — libration-limited EME and most
    /// AWGN-with-jitter channels.
    case gaussian = 0
    /// Lorentzian spread — heavier tails; some ionoscatter and
    /// meteor-burst signatures.
    case lorentzian = 1
}

/// A table of callsigns, so a Q65 decode can expand the `<...>` hashed
/// half of a type-4 message.
///
/// Only the Q65 family takes one: ``DecodeSession`` owns its own table
/// and learns through ``DecodeSession/addCallsign(_:)``. Pass nil to
/// leave hashed callsigns as literal `<...>` — that changes how the
/// text renders and nothing else about the decode.
public final class CallsignHashTable {
    let handle: OpaquePointer

    public init() throws {
        guard let opened = mfsk_callsign_hash_table_new() else {
            throw MfskError(status: MFSK_STATUS_INTERNAL, detail: globalLastError())
        }
        self.handle = opened
    }

    deinit { mfsk_callsign_hash_table_free(handle) }

    /// Teach the table a callsign known from outside the decoder — a
    /// band map, an earlier slot, an operator's log.
    public func insert(_ call: String) throws {
        try check(mfsk_callsign_hash_table_insert(handle, call))
    }
}

/// Q65 — ten sub-modes, four decode strategies.
public enum Q65 {
    /// Synthesise a standard message. 12 kHz f32 PCM, one transmission.
    public static func encode(
        subMode: Q65SubMode,
        call1: String,
        call2: String,
        gridOrReport: String,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_q65(subMode.rawValue, call1, call2, gridOrReport, frequencyHz,
                            out, capacity, written)
        }
    }

    /// Plain AWGN scan-and-decode — the baseline every other strategy
    /// trades cost or inputs against.
    public static func decode(
        _ samples: [Float],
        subMode: Q65SubMode,
        sampleRate: UInt32 = 12_000,
        hashTable: CallsignHashTable? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows { out, capacity, found in
                mfsk_q65_decode(subMode.rawValue, audio.baseAddress, UInt(audio.count),
                                sampleRate, hashTable?.handle, out, capacity, found)
            }
        }
    }

    /// A-priori decode: up to four hints, each optional. Lifts the
    /// effective threshold by ~2 dB when the hints are correct.
    public static func decode(
        _ samples: [Float],
        subMode: Q65SubMode,
        sampleRate: UInt32 = 12_000,
        apHint: APHint,
        hashTable: CallsignHashTable? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try withOptionalCString(apHint.call1) { c1 in
                try withOptionalCString(apHint.call2) { c2 in
                    try withOptionalCString(apHint.grid) { grid in
                        try withOptionalCString(apHint.report) { report in
                            try collectRows { out, capacity, found in
                                mfsk_q65_decode_with_ap(
                                    subMode.rawValue, audio.baseAddress, UInt(audio.count),
                                    sampleRate, c1, c2, grid, report,
                                    hashTable?.handle, out, capacity, found)
                            }
                        }
                    }
                }
            }
        }
    }

    /// Fast-fading decode. Recovers the 5-8 dB the AWGN Bessel front
    /// end loses on Doppler-spread channels — required for microwave
    /// EME at 5.7 / 10 / 24 GHz.
    ///
    /// - Parameter fadingB90Ts: spread bandwidth × symbol period.
    ///   Typical values: 0.05 near-AWGN, 1.0 moderate, 5.0+ severe.
    public static func decode(
        _ samples: [Float],
        subMode: Q65SubMode,
        sampleRate: UInt32 = 12_000,
        fadingB90Ts: Float,
        model: Q65FadingModel = .gaussian,
        hashTable: CallsignHashTable? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try collectRows { out, capacity, found in
                mfsk_q65_decode_fading(subMode.rawValue, audio.baseAddress, UInt(audio.count),
                                       sampleRate, fadingB90Ts, model.rawValue,
                                       hashTable?.handle, out, capacity, found)
            }
        }
    }

    /// AP-list (template-matching) decode. Builds the standard
    /// 206-codeword candidate set from the call pair and picks the
    /// matching exchange — ~3 dB over plain BP when the truth is in the
    /// set, and nothing at all when it is not.
    public static func decode(
        _ samples: [Float],
        subMode: Q65SubMode,
        sampleRate: UInt32 = 12_000,
        apList: APList,
        hashTable: CallsignHashTable? = nil
    ) throws -> [Decode] {
        try samples.withUnsafeBufferPointer { audio in
            try withOptionalCString(apList.hisGrid) { grid in
                try collectRows { out, capacity, found in
                    mfsk_q65_decode_with_ap_list(
                        subMode.rawValue, audio.baseAddress, UInt(audio.count),
                        sampleRate, apList.myCall, apList.hisCall, grid,
                        hashTable?.handle, out, capacity, found)
                }
            }
        }
    }

    /// Hints for ``Q65/decode(_:subMode:sampleRate:apHint:hashTable:)``.
    ///
    /// **These are the message's fields in order, not roles.** `call1`
    /// is the first callsign field — `"CQ"` for a CQ, whatever the
    /// transmitting station is called — and it locks message bits
    /// 0-28; `call2` is the second and locks 29-57; `grid` and `report`
    /// both lock 58-73, so they are alternatives (see
    /// `mfsk_core::msg::ap`). A hint locks bits rather than steering a
    /// search, so **getting the order wrong does not degrade
    /// gracefully**: the wrong lock removes the decode entirely rather
    /// than costing a fraction of a dB. `Q65Tests` pins both directions.
    ///
    /// Each is nil when unknown; all four nil is a legal call and falls
    /// through to the plain strategy.
    public struct APHint: Sendable, Equatable {
        public var call1: String?
        public var call2: String?
        public var grid: String?
        public var report: String?

        public init(call1: String? = nil, call2: String? = nil,
                    grid: String? = nil, report: String? = nil) {
            self.call1 = call1
            self.call2 = call2
            self.grid = grid
            self.report = report
        }
    }

    /// The call pair an AP-list decode builds its templates from.
    /// `hisGrid` may be nil to skip the two grid-bearing templates.
    public struct APList: Sendable, Equatable {
        public var myCall: String
        public var hisCall: String
        public var hisGrid: String?

        public init(myCall: String, hisCall: String, hisGrid: String? = nil) {
            self.myCall = myCall
            self.hisCall = hisCall
            self.hisGrid = hisGrid
        }
    }
}

/// Call `body` with a C string, or with nil where the ABI reads nil as
/// "not supplied" — which for these entry points is not the same as an
/// empty string.
func withOptionalCString<R>(_ value: String?, _ body: (UnsafePointer<CChar>?) throws -> R) rethrows -> R {
    guard let value else { return try body(nil) }
    return try value.withCString { try body($0) }
}
