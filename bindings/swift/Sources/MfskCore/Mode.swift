// SPDX-License-Identifier: GPL-3.0-or-later
//
// Modes, and what each one can do. The point of the C surface's
// introspection family (`mfsk_mode_count` / `_at` / `_info` / `_caps` /
// `_defaults`) is that a consumer stops hardcoding a capability matrix,
// so this file asks the library rather than restating it: the only
// numbers written here are the `MfskMode` discriminants, which the ABI
// pins and never reuses, and `ModeTests` checks each one against the
// header's own constant.

import CMfsk

/// Every mode the ABI can address. Not every one is in every build —
/// protocols are feature-gated, so ask ``Mode/supported``.
public enum Mode: UInt32, CaseIterable, Sendable {
    case ft8 = 0
    case ft4 = 1
    case fst4s15 = 2
    case fst4s30 = 3
    case fst4s60 = 4
    case fst4s120 = 5
    case fst4s300 = 6
    case wspr = 7
    case jt9 = 8
    case jt65 = 9
    case q65a15 = 10
    case q65a30 = 11
    case q65a60 = 12
    case q65b60 = 13
    case q65c60 = 14
    case q65d60 = 15
    case q65e60 = 16
    case q65d120 = 17
    case q65e120 = 18
    case q65a300 = 19
    case msk144 = 20
    case uvRobust = 21
    case uvStandard = 22
    case uvUltraRobust = 23
    case uvExpress = 24

    /// The stable display name (`"FT8"`, `"FST4-120"`), which is also
    /// the key ``init(name:)`` accepts — the two round-trip. Answers for
    /// a mode this build lacks: the name is a property of the mode.
    public var name: String {
        guard let p = mfsk_mode_name(rawValue) else { return "mode-\(rawValue)" }
        return String(cString: p)
    }

    /// Look a mode up by its display name. Case-sensitive, matching the
    /// registry's own strings.
    ///
    /// Throws ``MfskError/Code/invalidArgument`` for a name that is not
    /// a mode and ``MfskError/Code/unknownProtocol`` for a real mode
    /// this build was compiled without — the distinction that tells a
    /// typo from a missing feature, which a failable initialiser would
    /// throw away.
    public init(name: String) throws {
        var out = MfskMode(rawValue: 0)
        try check(mfsk_mode_from_name(name, &out))
        guard let mode = Mode(rawValue: UInt32(out.rawValue)) else {
            throw MfskError(status: MFSK_STATUS_UNKNOWN_PROTOCOL,
                            detail: "the library returned mode \(out.rawValue), which this binding predates")
        }
        self = mode
    }

    /// The modes **this build** actually has, in the library's own order.
    public static var supported: [Mode] {
        (0..<mfsk_mode_count()).compactMap { index in
            var raw = MfskMode(rawValue: 0)
            guard mfsk_mode_at(index, &raw) == MFSK_STATUS_OK else { return nil }
            return Mode(rawValue: UInt32(raw.rawValue))
        }
    }

    /// Whether this build carries this mode.
    public var isSupported: Bool { (try? info) != nil }

    /// Geometry and capability. Throws ``MfskError/Code/unknownProtocol``
    /// if this build lacks the mode.
    public var info: ModeInfo {
        get throws {
            var raw = MfskModeInfo()
            raw.size = UInt32(MemoryLayout<MfskModeInfo>.size)
            try check(mfsk_mode_info(rawValue, &raw))
            return ModeInfo(raw)
        }
    }

    /// What the mode supports — the same word ``ModeInfo/capabilities``
    /// carries. 0 for a mode this build lacks, which is also a legal
    /// "supports nothing"; use ``info`` when the difference matters.
    public var capabilities: Capabilities {
        Capabilities(rawValue: mfsk_mode_caps(rawValue))
    }

    /// The mode's published default search, or throws
    /// ``MfskError/Code/unsupported`` for a mode with no wide-band
    /// search to describe.
    public var decodeDefaults: DecodeDefaults {
        get throws {
            var raw = MfskDecodeDefaults()
            raw.size = UInt32(MemoryLayout<MfskDecodeDefaults>.size)
            try check(mfsk_mode_defaults(rawValue, &raw))
            return DecodeDefaults(raw)
        }
    }
}

/// What a mode supports, as reported by `mfsk_mode_caps`.
public struct Capabilities: OptionSet, Sendable {
    public let rawValue: UInt64
    public init(rawValue: UInt64) { self.rawValue = rawValue }

    /// Drives the `DecodeRequest` builder, i.e. ``DecodeSession`` applies.
    /// Modes without this decode through their own entry point — see
    /// ``WSPR``, ``JT9``, ``JT65``.
    public static let decodeHandle = Capabilities(rawValue: 1 << 0)
    /// Narrow-band single-target search. FT8 only, by design: it is the
    /// receive half of narrowing a transceiver's analogue roofing filter.
    public static let sniper = Capabilities(rawValue: 1 << 1)
    /// A-priori hint on a targeted search.
    public static let apNarrow = Capabilities(rawValue: 1 << 2)
    /// A-priori hint on the wide-band search.
    public static let apWideband = Capabilities(rawValue: 1 << 3)
    /// Flat successive-interference cancellation.
    public static let sicRounds = Capabilities(rawValue: 1 << 4)
    /// Checkpoint-emulation early decode. FT8 only.
    public static let sicEarly = Capabilities(rawValue: 1 << 5)
    /// The OSD *switch* is honoured. Absent means "cannot be turned
    /// off", not "does not have it".
    public static let osd = Capabilities(rawValue: 1 << 6)
    /// Equalisation reaches the decoder.
    public static let equalisation = Capabilities(rawValue: 1 << 7)
    /// The strictness profile is honoured rather than accepted and dropped.
    public static let strictness = Capabilities(rawValue: 1 << 8)
    /// A caller-supplied budget predicate is polled.
    public static let budget = Capabilities(rawValue: 1 << 9)
    /// Known signals can be excluded from the reported results.
    public static let knownFilter = Capabilities(rawValue: 1 << 10)
    /// Known signals are subtracted from the audio. Strictly stronger
    /// than ``knownFilter``.
    public static let knownSubtract = Capabilities(rawValue: 1 << 11)
    /// A slot FFT can be handed back for a second pass.
    public static let fftCache = Capabilities(rawValue: 1 << 12)
    /// Results can be delivered through a callback as they are found.
    public static let onResult = Capabilities(rawValue: 1 << 13)
    /// The mode can synthesise as well as decode.
    public static let encode = Capabilities(rawValue: 1 << 14)
}

/// How a mode's sync threshold is measured — the trap
/// ``DecodeDefaults/syncMin`` exists alongside.
public enum SyncScale: UInt32, Sendable {
    /// Absolute Costas correlation score; a threshold is empirical.
    /// FT8, FST4.
    case costasAbsolute = 0
    /// The spectrum is divided by a fitted baseline first, so **noise
    /// sits at ~1.0 by construction** and any threshold at or below
    /// that admits every peak in the band. FT4 only — which is why
    /// FT4's 1.2 is a floor rather than a preference, and not a number
    /// to copy to another mode.
    case baselineNormalised = 1
}

/// A mode's published default search parameters.
public struct DecodeDefaults: Sendable {
    public let frequencyRangeHz: ClosedRange<Float>
    /// **Read ``syncScale`` before copying this number anywhere.**
    public let syncMin: Float
    public let maxCandidates: UInt32
    public let syncScale: SyncScale

    init(_ raw: MfskDecodeDefaults) {
        self.frequencyRangeHz = raw.freq_min_hz...max(raw.freq_min_hz, raw.freq_max_hz)
        self.syncMin = raw.sync_min
        self.maxCandidates = raw.max_cand
        self.syncScale = SyncScale(rawValue: UInt32(raw.sync_scale.rawValue)) ?? .costasAbsolute
    }
}

/// Geometry and capability for one mode.
public struct ModeInfo: Sendable {
    public let mode: Mode
    public let name: String
    public let toneCount: UInt32
    public let bitsPerSymbol: UInt32
    /// Samples per symbol at 12 kHz.
    public let samplesPerSymbol: UInt32
    public let symbolDuration: Float
    public let toneSpacingHz: Float
    /// Gaussian bandwidth-time product; 0 for plain FSK.
    public let gfskBT: Float
    public let gfskHmod: Float
    public let dataSymbols: UInt32
    /// Sync symbols per frame; 0 for interleaved-sync protocols.
    public let syncSymbols: UInt32
    public let totalSymbols: UInt32
    public let slotSeconds: Float
    /// Slot length in samples at 12 kHz — ``slotSeconds`` made exact.
    public let slotSamples12k: UInt32
    /// Seconds from the start of the slot buffer to the first frame
    /// symbol, i.e. the `dt = 0` reference. 0.5 for FT8, FT4 and
    /// FST4-15; 1.0 for the other FST4 sub-modes. A host that
    /// synthesises a slot has to know this — see
    /// ``Mode/synthesiseSlot(_:frequencyHz:amplitude:)``.
    public let txStartOffsetSeconds: Float
    /// FEC information bits — 91 (CRC-14) or 101 (CRC-24).
    public let fecK: UInt32
    public let fecN: UInt32
    public let payloadBits: UInt32
    /// Length of the forward FFT the decoder takes over the whole slot,
    /// 0 for a mode with its own front end. **The field that makes "one
    /// call shape for every mode" wrong as a memory story**: FT4 takes
    /// 92 160 points and FST4-300 takes 4 194 304.
    public let decodeFFT1Size: UInt32
    public let capabilities: Capabilities

    init(_ raw: MfskModeInfo) {
        self.mode = Mode(rawValue: UInt32(raw.mode.rawValue)) ?? .ft8
        self.name = stringFromCArray(raw.name)
        self.toneCount = raw.ntones
        self.bitsPerSymbol = raw.bits_per_symbol
        self.samplesPerSymbol = raw.nsps
        self.symbolDuration = raw.symbol_dt
        self.toneSpacingHz = raw.tone_spacing_hz
        self.gfskBT = raw.gfsk_bt
        self.gfskHmod = raw.gfsk_hmod
        self.dataSymbols = raw.n_data
        self.syncSymbols = raw.n_sync
        self.totalSymbols = raw.n_symbols
        self.slotSeconds = raw.t_slot_s
        self.slotSamples12k = raw.slot_samples_12k
        self.txStartOffsetSeconds = raw.tx_start_offset_s
        self.fecK = raw.fec_k
        self.fecN = raw.fec_n
        self.payloadBits = raw.payload_bits
        self.decodeFFT1Size = raw.decode_fft1_size
        self.capabilities = Capabilities(rawValue: raw.caps)
    }
}
