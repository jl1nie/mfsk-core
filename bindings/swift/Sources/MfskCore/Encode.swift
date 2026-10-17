// SPDX-License-Identifier: GPL-3.0-only
//
// The transmit side over the mode: `mfsk_encode` and `mfsk_encode_info`.
// One call for every mode that has a text encoder, as ``Decoder`` is the one
// receive call; the per-mode `encode` functions (``WSPR``, ``JT9``, ``JT65``,
// ``Q65``, ``JTTY``, ``MSK144``) keep their own shapes.

import CMfsk

/// What ``Mode/encodeInfo(_:copiedLastTx:)`` returns: the channel symbols, what
/// a receiver will print, and the calls the message sends as hashes.
public struct EncodedMessage: Equatable, Sendable {
    /// One tone index per channel symbol.
    public let tones: [UInt8]
    /// The message as a receiver unpacks it (WSJT-X's Tx box): case and
    /// spacing normalised, a call sent as a hash written `<CALL>`.
    public let text: String
    /// The calls the message sends as hashes. Give each to
    /// ``Decoder/addCallsign(_:)`` so the `<...>` of the answer resolves.
    /// Empty for WSPR, JT9 and JT65.
    public let hashedCalls: [String]
}

/// `MFSK_ENCODE_Q65_COPIED_LAST_TX`, written out: a `#define` of `(1 << 0)`
/// reaches Swift as `Int32`, and the `flags` parameter is `UInt32`.
private let q65CopiedLastTxFlag: UInt32 = 1 << 0

extension Mode {
    /// Encode `text` and synthesise it: f32 PCM at unit amplitude, from sample
    /// 0 of the frame (the caller places it in the period). `frequencyHz` is
    /// the frequency of tone 0. `copiedLastTx` is Q65 only (WSJT-X 3.2's Pileup
    /// "copied last Tx" bit) and ignored by every other mode.
    ///
    /// Throws ``MfskError/Code/invalidArgument`` for a message the mode cannot
    /// send, ``MfskError/Code/unsupported`` for a form it has but the library
    /// does not encode, and ``MfskError/Code/unknownProtocol`` for a mode with
    /// no text encoder (MSK144, JTTY, uvpacket) or one absent from this build.
    public func encode(
        _ text: String,
        frequencyHz: Float,
        sampleRate: UInt32 = 12_000,
        copiedLastTx: Bool = false
    ) throws -> [Float] {
        let flags: UInt32 = copiedLastTx ? q65CopiedLastTxFlag : 0
        var needed: UInt = 0
        let probe = mfsk_encode(rawValue, text, flags, sampleRate, frequencyHz, nil, 0, &needed)
        // A null buffer is "too small" by design and reports the size; a
        // refusal reports nothing, so a zero is the real error.
        guard needed > 0 else {
            try check(probe)
            throw MfskError(status: MFSK_STATUS_DECODE_FAILED, detail: globalLastError())
        }
        var out = [Float](repeating: 0, count: Int(needed))
        var written: UInt = 0
        let status = out.withUnsafeMutableBufferPointer {
            mfsk_encode(rawValue, text, flags, sampleRate, frequencyHz,
                        $0.baseAddress, UInt($0.count), &written)
        }
        try check(status)
        return Array(out.prefix(Int(written)))
    }

    /// The tones, the receiver's text and the hashed calls of `text`, without
    /// synthesising it. Throws as ``encode(_:frequencyHz:sampleRate:copiedLastTx:)``.
    public func encodeInfo(_ text: String, copiedLastTx: Bool = false) throws -> EncodedMessage {
        let flags: UInt32 = copiedLastTx ? q65CopiedLastTxFlag : 0
        var count: UInt = 0, textSize: UInt = 0, hashedSize: UInt = 0
        // Null buffers report all three sizes (the strings including the NUL);
        // a refusal reports nothing, so a zero is the real error.
        let probe = mfsk_encode_info(rawValue, text, flags, nil, 0, &count,
                                     nil, 0, &textSize, nil, 0, &hashedSize)
        guard count > 0, textSize > 0, hashedSize > 0 else {
            try check(probe)
            throw MfskError(status: MFSK_STATUS_DECODE_FAILED, detail: globalLastError())
        }
        var tones = [UInt8](repeating: 0, count: Int(count))
        var textOut = [CChar](repeating: 0, count: Int(textSize))
        var hashedOut = [CChar](repeating: 0, count: Int(hashedSize))
        var textWritten: UInt = 0, hashedWritten: UInt = 0
        let status = tones.withUnsafeMutableBufferPointer { t in
            textOut.withUnsafeMutableBufferPointer { x in
                hashedOut.withUnsafeMutableBufferPointer { h in
                    mfsk_encode_info(rawValue, text, flags, t.baseAddress, UInt(t.count), &count,
                                     x.baseAddress, UInt(x.count), &textWritten,
                                     h.baseAddress, UInt(h.count), &hashedWritten)
                }
            }
        }
        try check(status)
        return EncodedMessage(
            tones: Array(tones.prefix(Int(count))),
            text: String(cString: textOut),
            hashedCalls: String(cString: hashedOut).split(separator: " ").map(String.init)
        )
    }
}
