// SPDX-License-Identifier: GPL-3.0-or-later
//
// Q65 — decoded through ``Decoder`` like every other slot mode (open it with
// a Q65 ``Mode``; Pileup, Max Drift, fading and the rest are ``Extras``,
// the EME delay is ``DecodeParams/emeDelay``, the AP list is the QSO context
// plus ``Decoder/setQ65Callers(_:)``). What stays here is what the decoder
// does not cover: the transmit side, whose sub-mode numbering is Q65's own.
//
// The sub-mode enums reach C because `cbindgen.toml` asks for them by
// name: every `mfsk_encode_q65*` function takes its sub-mode as `uint32_t`,
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
    /// ``Extras/fading``.
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

/// Channel model for the fast-fading decoder (``Extras/Fading``).
public enum Q65FadingModel: UInt32, Sendable {
    /// Gaussian spread — libration-limited EME and most
    /// AWGN-with-jitter channels.
    case gaussian = 0
    /// Lorentzian spread — heavier tails; some ionoscatter and
    /// meteor-burst signatures.
    case lorentzian = 1
}

/// Q65 — ten sub-modes. Decode with a ``Decoder``; this is the transmit side.
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

    /// ``encode(subMode:call1:call2:gridOrReport:frequencyHz:)`` with Pileup's
    /// "copied last Tx" flag: `copiedLastTx` sets the spare 78th payload bit,
    /// which a Pileup receiver reports as ``Decode/copiedLastTx``.
    public static func encode(
        subMode: Q65SubMode,
        call1: String,
        call2: String,
        gridOrReport: String,
        copiedLastTx: Bool,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_q65_flagged(subMode.rawValue, call1, call2, gridOrReport,
                                    copiedLastTx ? 1 : 0, frequencyHz, out, capacity, written)
        }
    }
}
