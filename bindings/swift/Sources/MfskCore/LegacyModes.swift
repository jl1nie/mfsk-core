// SPDX-License-Identifier: GPL-3.0-or-later
//
// WSPR, JT9 and JT65: the transmit side. Their message codecs synthesise in
// one step rather than through a tone stage, so each has an `encode` of its
// own. They *decode* through ``Decoder`` like every other slot mode
// (`Decoder(mode: .wspr)`), a 120 s, 60 s and 60 s period respectively.

import CMfsk

/// WSPR — 120 s slot, 4-FSK, convolutional r=½ K=32 + Fano.
public enum WSPR {
    /// Synthesise a beacon: callsign, 4-character grid, power in dBm.
    /// 12 kHz f32 PCM, one full slot.
    public static func encode(
        call: String,
        grid: String,
        powerDBM: Int32,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_wspr(call, grid, powerDBM, frequencyHz, out, capacity, written)
        }
    }
}

/// JT9 — 60 s slot, 9-FSK.
public enum JT9 {
    public static func encode(
        call1: String,
        call2: String,
        gridOrReport: String,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_jt9(call1, call2, gridOrReport, frequencyHz, out, capacity, written)
        }
    }
}

/// JT65 — 60 s slot, 65-FSK, Reed-Solomon(63,12).
public enum JT65 {
    public static func encode(
        call1: String,
        call2: String,
        gridOrReport: String,
        frequencyHz: Float
    ) throws -> [Float] {
        try encodeAudio { out, capacity, written in
            mfsk_encode_jt65(call1, call2, gridOrReport, frequencyHz, out, capacity, written)
        }
    }
}

/// The `mfsk_encode_*` family's shared shape: call once with a null
/// buffer to learn the length, then once more to fill it. Two calls
/// rather than a guess because a transmission runs from 15 s to 300 s
/// of audio depending on the mode — Q65's family shares this, which is
/// why it is `internal` rather than `private`.
func encodeAudio(
    _ call: (UnsafeMutablePointer<Float>?, UInt, UnsafeMutablePointer<UInt>) -> MfskStatus
) throws -> [Float] {
    var needed: UInt = 0
    _ = call(nil, 0, &needed)
    guard needed > 0 else {
        throw MfskError(status: MFSK_STATUS_DECODE_FAILED, detail: globalLastError())
    }
    var out = [Float](repeating: 0, count: Int(needed))
    var written: UInt = 0
    let status = out.withUnsafeMutableBufferPointer { buffer in
        call(buffer.baseAddress, UInt(buffer.count), &written)
    }
    try check(status)
    return Array(out.prefix(Int(written)))
}

extension Array where Element == Float {
    /// Nominal `-1.0...1.0` float PCM as 16-bit, which is what the
    /// `mfsk_encode_*` family produces and what ``Decoder`` takes. Clamped, not wrapped: a sample at ±1.0 is
    /// full-scale, and anything past it is the caller's gain problem,
    /// not a sign flip in the middle of a transmission.
    public func asPCM16(scale: Float = 32767) -> [Int16] {
        map { Int16(Swift.max(-32768, Swift.min(32767, ($0 * scale).rounded()))) }
    }
}
