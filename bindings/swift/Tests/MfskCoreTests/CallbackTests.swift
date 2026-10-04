// SPDX-License-Identifier: GPL-3.0-only
//
// `Decoder.onDecode` — rows delivered as they are found, on top
// of the array the call returns. The handler can fire from a rayon
// worker on a `desktop` build, so everything these tests collect goes
// through a lock.

import XCTest
@testable import MfskCore

final class CallbackTests: XCTestCase {
    private func ft8Slot(frequencyHz: Float = 1500) throws -> [Int16] {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        return try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                           frequencyHz: frequencyHz)
    }

    func testTheHandlerSeesWhatTheReturnedArrayHolds() throws {
        let slot = try ft8Slot()
        let collected = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { collected.append($0) }

        let returned = try decoder.decode(slot)
        XCTAssertTrue(returned.contains { $0.text.contains("JA1ABC") })

        // The array is the authoritative set; the callback is the same
        // rows arriving earlier. For one clean candidate the two agree
        // exactly — order is not asserted, because with rayon this is
        // completion order rather than the array's.
        let streamed = collected.snapshot
        XCTAssertEqual(streamed.count, returned.count)
        XCTAssertEqual(Set(streamed.map(\.text)), Set(returned.map(\.text)))
        XCTAssertEqual(streamed.first?.frequencyHz ?? 0, 1500, accuracy: 5)
    }

    func testTheHandlerSurvivesTheCallThatSetIt() throws {
        // The box lives on the decoder, not on the stack frame that
        // installed it — an easy thing to get wrong, and it would
        // present as a crash inside the decode rather than at the call
        // that set the handler.
        let slot = try ft8Slot()
        let decoder = try Decoder(mode: .ft8)
        let collected = Collected()
        try autoreleasepool {
            let sink = collected           // captured, then this scope ends
            try decoder.onDecode { sink.append($0) }
        }
        _ = try decoder.decode(slot)
        XCTAssertFalse(collected.snapshot.isEmpty)
    }

    func testAHandlerAppliesToEverySubsequentDecode() throws {
        let slot = try ft8Slot()
        let collected = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { collected.append($0) }
        _ = try decoder.decode(slot)
        let afterFirst = collected.snapshot.count
        _ = try decoder.decode(slot)
        XCTAssertEqual(collected.snapshot.count, afterFirst * 2,
                       "the handler is set on the decoder, not on one call")
    }

    func testNilStopsDelivery() throws {
        let slot = try ft8Slot()
        let collected = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { collected.append($0) }
        _ = try decoder.decode(slot)
        let before = collected.snapshot.count
        XCTAssertGreaterThan(before, 0)

        try decoder.onDecode(nil)
        let returned = try decoder.decode(slot)
        XCTAssertEqual(collected.snapshot.count, before, "nil should stop delivery")
        XCTAssertFalse(returned.isEmpty, "and must not stop decoding")
    }

    func testReplacingTheHandlerSwitchesDelivery() throws {
        let slot = try ft8Slot()
        let first = Collected()
        let second = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { first.append($0) }
        _ = try decoder.decode(slot)
        try decoder.onDecode { second.append($0) }
        _ = try decoder.decode(slot)

        XCTAssertEqual(first.snapshot.count, second.snapshot.count,
                       "the first handler should have stopped where the second began")
    }

    func testAHandlerPassedToOneCallIsForThatCallOnly() throws {
        let slot = try ft8Slot()
        let persistent = Collected()
        let once = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { persistent.append($0) }

        let returned = try decoder.decode(slot, handler: { once.append($0) })
        XCTAssertEqual(once.snapshot.count, returned.count,
                       "the per-call handler saw what the array holds")
        XCTAssertEqual(persistent.snapshot.count, 0,
                       "while it was in place it stood in for the persistent one")

        _ = try decoder.decode(slot)
        XCTAssertEqual(once.snapshot.count, returned.count, "and it did not outlive the call")
        XCTAssertGreaterThan(persistent.snapshot.count, 0, "the persistent handler came back")
    }

    func testTheRowIsCopiedOutOfTheCallbackWindow() throws {
        // The C row pointer is valid only for the duration of the call,
        // so the value kept here has to be a copy — including the text,
        // which lives in a fixed-size array inside the row.
        let slot = try ft8Slot(frequencyHz: 1650)
        let collected = Collected()
        let decoder = try Decoder(mode: .ft8)
        try decoder.onDecode { collected.append($0) }
        _ = try decoder.decode(slot)

        let kept = collected.snapshot
        XCTAssertFalse(kept.isEmpty)
        // Read the copies well after every callback has returned.
        XCTAssertTrue(kept.contains { $0.text.contains("JA1ABC") })
        XCTAssertTrue(kept.allSatisfy { $0.mode == .ft8 && $0.frequencyHz > 1000 })
    }

    func testFT8AdvertisesTheCapability() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        XCTAssertTrue(Mode.ft8.capabilities.contains(.onResult),
                      "the bit that says this entry point applies")
    }
}
