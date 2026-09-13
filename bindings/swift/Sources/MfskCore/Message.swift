// SPDX-License-Identifier: GPL-3.0-or-later
//
// Transmit, in the three stages the C surface exposes: pack a message
// to 77 bits, turn those into channel symbols, render the symbols as
// PCM. Each stage writes into a buffer sized from the mode
// (`mfsk_symbol_count` / `mfsk_synth_output_len`), so **there is no
// pointer to free** — the category that makes a wrapper leak when an
// error unwinds between the call and the free.

import CMfsk

/// A packed WSJT message: 77 bits, one per byte, as every stage-2 call
/// takes them.
public struct Message: Sendable, Equatable {
    /// The ABI's payload width, and the length of ``bits``.
    public static let bitCount = 77

    public let bits: [UInt8]

    /// Wrap bits that already came from somewhere else (a log, a test
    /// vector). 77 of them, each 0 or 1.
    public init(bits: [UInt8]) throws {
        guard bits.count == Message.bitCount else {
            throw MfskError(status: MFSK_STATUS_INVALID_ARG,
                            detail: "a packed message is \(Message.bitCount) bits, got \(bits.count)")
        }
        self.bits = bits
    }

    private init(unchecked bits: [UInt8]) { self.bits = bits }

    /// `call1 call2 report` — the standard exchange (WSJT type 1/2).
    public static func standard(call1: String, call2: String, report: String) throws -> Message {
        try packed { mfsk_pack77(call1, call2, report, $0) }
    }

    /// `call1 call2 grid` — a type-1 message.
    public static func type1(call1: String, call2: String, grid: String) throws -> Message {
        try packed { mfsk_pack77_type1(call1, call2, grid, $0) }
    }

    /// Up to 13 characters of free text.
    public static func freeText(_ text: String) throws -> Message {
        try packed { mfsk_pack77_free_text(text, $0) }
    }

    /// One non-standard callsign in full, plus a **hashed** reference to
    /// the standard one.
    ///
    /// The hashed half decodes as `<...>` unless the receiving session
    /// has seen that callsign — see ``DecodeSession/addCallsign(_:)``.
    public static func type4(
        nonStandardCall: String,
        standardCall: String,
        report: String,
        isCQ: Bool
    ) throws -> Message {
        try packed { mfsk_pack77_type4(nonStandardCall, standardCall, report, isCQ, $0) }
    }

    /// Render as text.
    ///
    /// Pass the session whose table should resolve hashed `<...>`
    /// callsigns; without one they stay unresolved, which changes only
    /// how the text renders.
    public func text(resolvedBy session: DecodeSession? = nil) throws -> String {
        var out = [CChar](repeating: 0, count: Int(MFSK_DECODE_TEXT_LEN))
        var written: UInt = 0
        let status = bits.withUnsafeBufferPointer { packed in
            out.withUnsafeMutableBufferPointer { buffer in
                mfsk_unpack77(session?.handle, packed.baseAddress,
                              buffer.baseAddress, UInt(buffer.count), &written)
            }
        }
        try check(status)
        return String(cString: out)
    }

    /// Stage 2: this message as `mode`'s channel symbols.
    ///
    /// Throws ``MfskError/Code/unsupported`` for a mode with no tone
    /// stage — WSPR, JT9, JT65 and Q65 synthesise from their own message
    /// codecs in one step instead; see ``WSPR/encode(call:grid:powerDBM:frequencyHz:)``.
    public func tones(for mode: Mode) throws -> [UInt8] {
        let capacity = mode.symbolCount
        guard capacity > 0 else {
            throw MfskError(status: MFSK_STATUS_UNSUPPORTED,
                            detail: "\(mode.name) has no tone stage")
        }
        var tones = [UInt8](repeating: 0, count: capacity)
        var written: UInt = 0
        let status = bits.withUnsafeBufferPointer { packed in
            tones.withUnsafeMutableBufferPointer { buffer in
                mfsk_message_to_tones(mode.rawValue, packed.baseAddress,
                                      buffer.baseAddress, UInt(buffer.count), &written)
            }
        }
        try check(status)
        return Array(tones.prefix(Int(written)))
    }

    private static func packed(_ call: (UnsafeMutablePointer<UInt8>) -> MfskStatus) throws -> Message {
        var bits = [UInt8](repeating: 0, count: Message.bitCount)
        let status = bits.withUnsafeMutableBufferPointer { call($0.baseAddress!) }
        try check(status)
        return Message(unchecked: bits)
    }
}

extension Mode {
    /// Channel symbols per frame, or 0 for a mode with no tone stage.
    /// A non-zero answer is what says ``Message/tones(for:)`` and
    /// ``synthesiseFrame(_:frequencyHz:amplitude:)-(Message,_,Int16)`` apply.
    public var symbolCount: Int { Int(mfsk_symbol_count(rawValue)) }

    /// Samples a full frame synthesises to at 12 kHz.
    ///
    /// **Ask rather than assume**: the five FST4 sub-modes differ by a
    /// factor of 30 (720 → 21 504 samples per symbol), so a constant
    /// baked for 60A is silently wrong for the other four.
    public var synthesisedFrameLength: Int { Int(mfsk_synth_output_len(rawValue)) }

    /// Stage 3: channel symbols as 16-bit PCM at 12 kHz.
    public func synthesiseFrame(tones: [UInt8], frequencyHz: Float, amplitude: Int16 = 8000) throws -> [Int16] {
        var out = [Int16](repeating: 0, count: synthesisedFrameLength)
        var written: UInt = 0
        let status = tones.withUnsafeBufferPointer { itone in
            out.withUnsafeMutableBufferPointer { buffer in
                mfsk_tones_to_i16(rawValue, itone.baseAddress, UInt(itone.count), frequencyHz,
                                  amplitude, buffer.baseAddress, UInt(buffer.count), &written)
            }
        }
        try check(status)
        return Array(out.prefix(Int(written)))
    }

    /// Stage 3, in float: channel symbols as 32-bit PCM at 12 kHz.
    public func synthesiseFrame(tones: [UInt8], frequencyHz: Float, amplitude: Float) throws -> [Float] {
        var out = [Float](repeating: 0, count: synthesisedFrameLength)
        var written: UInt = 0
        let status = tones.withUnsafeBufferPointer { itone in
            out.withUnsafeMutableBufferPointer { buffer in
                mfsk_tones_to_f32(rawValue, itone.baseAddress, UInt(itone.count), frequencyHz,
                                  amplitude, buffer.baseAddress, UInt(buffer.count), &written)
            }
        }
        try check(status)
        return Array(out.prefix(Int(written)))
    }

    /// All three stages: a message, placed in a full slot at this mode's
    /// own TX offset, ready to hand to ``DecodeSession`` or to a sound
    /// card.
    ///
    /// The offset is ``ModeInfo/txStartOffsetSeconds`` — 0.5 s for FT8,
    /// FT4 and FST4-15, 1.0 s for the other FST4 sub-modes — and getting
    /// it from the mode rather than a constant is the whole reason the
    /// field is published.
    public func synthesiseSlot(
        _ message: Message,
        frequencyHz: Float,
        amplitude: Int16 = 8000
    ) throws -> [Int16] {
        let info = try info
        let frame = try synthesiseFrame(tones: message.tones(for: self),
                                        frequencyHz: frequencyHz,
                                        amplitude: amplitude)
        var slot = [Int16](repeating: 0, count: Int(info.slotSamples12k))
        let start = Int(info.txStartOffsetSeconds * 12_000)
        for (i, sample) in frame.enumerated() where start + i < slot.count {
            slot[start + i] = sample
        }
        return slot
    }

    /// The common case in one call: pack `call1 call2 report`, tone it,
    /// and place it in a slot.
    public func synthesiseSlot(
        call1: String,
        call2: String,
        report: String,
        frequencyHz: Float,
        amplitude: Int16 = 8000
    ) throws -> [Int16] {
        try synthesiseSlot(Message.standard(call1: call1, call2: call2, report: report),
                           frequencyHz: frequencyHz,
                           amplitude: amplitude)
    }
}
