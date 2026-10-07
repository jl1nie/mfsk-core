// SPDX-License-Identifier: GPL-3.0-only
//
// Early decode through the push-driven handles (#601): the IQ receiver
// decodes an FT8 channel early by default, and a capture stream does when it
// is given the decoder's prefix points. The Rust side is
// `mfsk-ffi/tests/iq_ffi.rs` and `decoder_ffi.rs`.

import XCTest
@testable import MfskCore

final class EarlyDecodeTests: XCTestCase {
    private let text = "CQ JA1ABC PM95"

    private func ft8Slot() throws -> [Int16] {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        return try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                           frequencyHz: 1500)
    }

    func testTheDecoderNamesItsPrefixPoints() throws {
        _ = try ft8Slot()
        XCTAssertEqual(try Decoder(mode: .ft8).prefixPoints, [141_696, 162_432])
        if Mode.ft4.isSupported {
            XCTAssertEqual(try Decoder(mode: .ft4).prefixPoints, [], "FT4 has no checkpoints")
        }
    }

    func testAStreamWithPrefixPointsDecodesEarly() throws {
        let slot = try ft8Slot()
        let decoder = try Decoder(mode: .ft8)
        let stream = try CaptureStream(mode: .ft8)
        try stream.setPrefixPoints(decoder.prefixPoints)
        let audio = slot + [Int16](repeating: 0, count: 12_000)
        var calls: [(whole: Bool, rows: [Decode])] = []
        var pos = 0
        while pos < audio.count {
            let end = min(pos + 4_801, audio.count)
            try stream.push(Array(audio[pos..<end]))
            pos = end
            while stream.isSlotReady {
                let whole = stream.isSlotWhole
                let got = try XCTUnwrap(try decoder.decode(stream))
                calls.append((whole, got.decodes))
            }
        }
        XCTAssertEqual(calls.map(\.whole), [false, false, true], "A, B and the whole slot")
        XCTAssertTrue(calls[0].rows.contains { $0.text == text && $0.stage == .early })
        XCTAssertTrue(calls[1].rows.isEmpty, "B returns nothing")
        XCTAssertEqual(calls[2].rows.filter { $0.text == text }.count, 1)
        XCTAssertEqual(stream.droppedSlots, 0)
    }

    /// The FT8 slot as double-sideband IQ at 48 kS/s, `cf32`, on a dial 6 kHz
    /// above the centre: 12 kHz up to 48 kHz by linear interpolation (the
    /// channel's filter removes the images), as the Kotlin test builds it.
    private func iqBytes(_ slot: [Int16], rate: Int, offsetHz: Double) -> [UInt8] {
        let n = 15 * rate + rate / 2
        let w = 2 * Double.pi * offsetHz / Double(rate)
        var re = [Float](repeating: 0, count: n)
        var im = [Float](repeating: 0, count: n)
        var peak: Float = 0
        for i in 0..<n {
            let x = Double(i) / 4
            let k = Int(x)
            let a = k < slot.count ? Double(slot[k]) : 0
            let b = k + 1 < slot.count ? Double(slot[k + 1]) : 0
            let v = a + (b - a) * (x - Double(k))
            re[i] = Float(v * cos(w * Double(i)))
            im[i] = Float(v * sin(w * Double(i)))
            peak = max(peak, abs(re[i]), abs(im[i]))
        }
        var bytes: [UInt8] = []
        bytes.reserveCapacity(n * 8)
        for i in 0..<n {
            for v in [re[i] * 0.7 / peak, im[i] * 0.7 / peak] {
                withUnsafeBytes(of: v.bitPattern.littleEndian) { bytes.append(contentsOf: $0) }
            }
        }
        return bytes
    }

    func testAnIQRowArrivesBeforeTheSlotIsWhole() throws {
        let slot = try ft8Slot()
        let rate = 48_000
        let center = 14_077_000.0
        let bytes = iqBytes(slot, rate: rate, offsetHz: 6_000)
        let split = 13 * rate * 8 // 13 s: past checkpoint A, short of the slot
        for early in [true, false] {
            let rx = try IQReceiver(sampleRate: UInt32(rate), centerHz: center)
            let channel = try rx.addChannel(dialHz: center + 6_000, mode: .ft8)
            if !early { try rx.setEarly(false, forChannel: channel) }
            try rx.setTime(utcNanoseconds: 1_700_000_100 * 1_000_000_000, atSample: 0)
            try rx.push(Array(bytes[..<split]))
            let before = try rx.drain().filter { $0.text == text }
            try rx.push(Array(bytes[split...]))
            let after = try rx.drain().filter { $0.text == text }
            if early {
                XCTAssertEqual(before.count, 1, "the row before the slot is whole")
                XCTAssertEqual(before.first?.stage, .early)
                XCTAssertEqual(after.count, 0, "and not queued again")
            } else {
                XCTAssertEqual(before.count, 0, "early decode is off")
                XCTAssertEqual(after.count, 1)
                XCTAssertNil(after.first?.stage, "a plain decode")
            }
        }
    }

    func testSetEarlyNeedsAChannel() throws {
        _ = try ft8Slot()
        let rx = try IQReceiver(sampleRate: 48_000, centerHz: 14_077_000)
        XCTAssertThrowsError(try rx.setEarly(true, forChannel: 9)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
    }
}
