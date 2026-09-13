// SPDX-License-Identifier: GPL-3.0-or-later
//
// The capture ring, driven the way a USB-audio reader drives it: small
// chunks in, one slot out, with the clock supplied rather than read.

import XCTest
@testable import MfskCore

final class CaptureStreamTests: XCTestCase {
    private static let epoch = 1_700_000_000.0

    func testPushUntilReadyThenDecodeInPlace() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                               frequencyHz: 1500)
        let stream = try CaptureStream(mode: .ft8)
        stream.setEpoch(utcSeconds: Self.epoch)
        XCTAssertFalse(stream.isSlotReady, "a fresh stream has no slot")

        // 1920 samples is 160 ms at 12 kHz — a plausible UAC callback.
        for chunk in stride(from: 0, to: slot.count, by: 1920) {
            try stream.push(Array(slot[chunk..<min(chunk + 1920, slot.count)]))
        }
        XCTAssertTrue(stream.isSlotReady, "a full slot was pushed")
        XCTAssertGreaterThanOrEqual(stream.bufferedSamples, slot.count)

        let session = try DecodeSession(mode: .ft8)
        let result = try XCTUnwrap(try session.decode(stream))
        XCTAssertTrue(result.decodes.contains { $0.text.contains("JA1ABC") })
        XCTAssertEqual(result.slotStartUTC, Self.epoch, accuracy: 0.001,
                       "the slot should be stamped from the epoch it was told, not a clock")
    }

    func testNoSlotReadyIsNilRatherThanAnError() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        let session = try DecodeSession(mode: .ft8)
        XCTAssertNil(try session.decode(stream),
                     "polling an empty stream is a legal way to drive this")
    }

    func testTakeSlotHandsBackTheAudioAndItsTimestamp() throws {
        guard let info = try? Mode.ft8.info else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        stream.setEpoch(utcSeconds: Self.epoch)
        XCTAssertNil(stream.takeSlot())
        try stream.push([Int16](repeating: 0, count: Int(info.slotSamples12k)))
        let taken = try XCTUnwrap(stream.takeSlot())
        XCTAssertEqual(taken.samples.count, Int(info.slotSamples12k))
        XCTAssertEqual(taken.slotStartUTC, Self.epoch, accuracy: 0.001)
        XCTAssertFalse(stream.isSlotReady, "taking the slot consumes it")
    }

    func testFloatPushIsAcceptedAndClearDoesNotRewindTheGrid() throws {
        guard let info = try? Mode.ft8.info else { throw XCTSkip("no FT8 in this build") }
        let stream = try CaptureStream(mode: .ft8)
        stream.setEpoch(utcSeconds: Self.epoch)
        let halfSlot = Int(info.slotSamples12k) / 2
        try stream.push([Float](repeating: 0, count: halfSlot))
        XCTAssertGreaterThan(stream.bufferedSamples, 0)
        stream.clear()
        XCTAssertEqual(stream.bufferedSamples, 0)

        // `clear()` drops the audio and keeps the epoch — as an anchor,
        // which is not the same as rewinding to it. The 7.5 s that were
        // pushed and dropped still happened, so the next slot is stamped
        // 7.5 s later. That is the behaviour a live receiver wants
        // (time does not run backwards when you discard a buffer), and
        // it is the one thing about `clear()` a caller can get wrong.
        try stream.push([Int16](repeating: 0, count: Int(info.slotSamples12k)))
        let taken = try XCTUnwrap(stream.takeSlot())
        XCTAssertEqual(taken.slotStartUTC,
                       Self.epoch + Double(halfSlot) / 12_000.0,
                       accuracy: 0.001)
    }
}
