// SPDX-License-Identifier: GPL-3.0-or-later
//
// The capture ring, driven the way a USB-audio reader drives it: small
// chunks in, one slot out, with the clock supplied rather than read. The
// scenario is `decoder_ffi.rs`'s
// `a_stream_cuts_on_the_grid_and_the_decoder_reads_the_period`.

import XCTest
@testable import MfskCore

final class CaptureStreamTests: XCTestCase {
    /// A 15 s FT8 boundary.
    private static let boundaryNs: Int64 = 1_700_000_010 * 1_000_000_000

    /// Pushes a quarter-second lead-in, tells the stream the lead-in began
    /// 250 ms before the boundary, then pushes `audio` in odd chunks.
    private func fill(_ stream: CaptureStream, with audio: [Int16]) throws {
        try stream.push([Int16](repeating: 0, count: 3_000))
        XCTAssertEqual(try stream.setTime(utcNanoseconds: Self.boundaryNs - 250_000_000, atSample: 0),
                       .first)
        for chunk in stride(from: 0, to: audio.count, by: 7_777) {
            try stream.push(Array(audio[chunk..<min(chunk + 7_777, audio.count)]))
        }
    }

    func testPushUntilReadyThenDecodeInPlaceWithTheSlotsPeriod() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        var audio = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                                frequencyHz: 1500)
        audio += [Int16](repeating: 0, count: 12_000)
        let stream = try CaptureStream(mode: .ft8)
        XCTAssertFalse(stream.isSlotReady, "a fresh stream has no slot")
        try fill(stream, with: audio)
        XCTAssertTrue(stream.isSlotReady, "a full slot was pushed")

        let decoder = try Decoder(mode: .ft8)
        let result = try XCTUnwrap(try decoder.decode(stream))
        XCTAssertTrue(result.decodes.contains { $0.text.contains("JA1ABC") })
        XCTAssertEqual(result.period, 1_700_000_010 / 15, "the boundary the lead-in ends on")
        XCTAssertEqual(result.slotStartUTCNanoseconds, result.period * 15_000_000_000)
        XCTAssertEqual(result.slotStartUTC ?? 0, Double(result.period * 15), accuracy: 0.001)
        XCTAssertFalse(stream.isSlotReady, "decoding the slot consumes it")
        XCTAssertNil(try decoder.decode(stream), "nothing more is ready")
    }

    func testNoSlotReadyIsNilRatherThanAnError() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        let decoder = try Decoder(mode: .ft8)
        XCTAssertNil(try decoder.decode(stream),
                     "polling an empty stream is a legal way to drive this")
    }

    func testAStreamAndADecoderOfDifferentModesDoNotMix() throws {
        guard Mode.ft8.isSupported, Mode.ft4.isSupported else { throw XCTSkip("needs FT8 and FT4") }
        let stream = try CaptureStream(mode: .ft4)
        let decoder = try Decoder(mode: .ft8)
        XCTAssertThrowsError(try decoder.decode(stream)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
    }

    func testAModeThatIsNotCutIntoSlotsHasNoStream() {
        XCTAssertThrowsError(try CaptureStream(mode: .jtty))
    }

    func testTakeSlotHandsBackTheAudioItsPeriodAndItsTimestamp() throws {
        guard let info = try? Mode.ft8.info else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        XCTAssertNil(stream.takeSlot())
        try fill(stream, with: [Int16](repeating: 0, count: Int(info.slotSamples12k) + 12_000))
        let taken = try XCTUnwrap(stream.takeSlot())
        XCTAssertEqual(taken.samples.count, Int(info.slotSamples12k))
        XCTAssertEqual(taken.period, 1_700_000_010 / 15)
        XCTAssertEqual(taken.startUTCNanoseconds, taken.period * 15_000_000_000)
        XCTAssertFalse(stream.isSlotReady, "taking the slot consumes it")
    }

    func testWithoutAClockThePeriodStillCountsAndUTCIsAbsent() throws {
        guard let info = try? Mode.ft8.info else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        try stream.push([Float](repeating: 0, count: Int(info.slotSamples12k) + 12_000))
        let taken = try XCTUnwrap(stream.takeSlot())
        XCTAssertNil(taken.startUTCNanoseconds, "a free-running grid has no UTC")
        XCTAssertEqual(stream.position, UInt64(info.slotSamples12k) + 12_000)
    }

    func testClearDropsTheAudioAndKeepsTheClock() throws {
        guard let info = try? Mode.ft8.info else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        try fill(stream, with: [Int16](repeating: 0, count: Int(info.slotSamples12k) + 12_000))
        XCTAssertTrue(stream.isSlotReady)
        stream.clear()
        XCTAssertFalse(stream.isSlotReady)
        // The clock survived: the next slot still lands on the grid.
        try stream.push([Int16](repeating: 0, count: Int(info.slotSamples12k) + 12_000))
        let taken = try XCTUnwrap(stream.takeSlot())
        XCTAssertNotNil(taken.startUTCNanoseconds)
    }
}
