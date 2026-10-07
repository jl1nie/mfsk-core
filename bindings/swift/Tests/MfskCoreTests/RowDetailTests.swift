// SPDX-License-Identifier: GPL-3.0-only
//
// What a row says beyond its text (#592, #594): a number the mode does not
// report is nil, the message key travels with the row, a returned row names
// the delivery it was, and the budget report counts SicEarly's subtractions.

import XCTest
@testable import MfskCore

final class RowDetailTests: XCTestCase {
    private func ft8Slot() throws -> [Int16] {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        return try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                           frequencyHz: 1500)
    }

    func testFT8ReportsItsNumbersAndItsKey() throws {
        let rows = try Decoder(mode: .ft8).decode(try ft8Slot())
        let row = try XCTUnwrap(rows.first { $0.text == "CQ JA1ABC PM95" })
        XCTAssertNotNil(row.syncScore)
        XCTAssertNotNil(row.syncCV)
        XCTAssertNotNil(row.hardErrors)
        XCTAssertEqual(row.keyBits, 77)
        XCTAssertEqual(row.key.count, 10)
        XCTAssertNil(row.delivery, "no handler, no delivery")
    }

    func testAReturnedRowNamesTheDeliveryItWas() throws {
        let decoder = try Decoder(mode: .ft8)
        XCTAssertTrue(decoder.deliveryIsExact, "FT8's default is SicEarly")
        let collected = Collected()
        try decoder.onDecode { collected.append($0) }
        let returned = try decoder.decode(try ft8Slot())
        let streamed = collected.snapshot.sorted { ($0.delivery ?? 0) < ($1.delivery ?? 0) }
        XCTAssertEqual(streamed.compactMap(\.delivery), Array(0..<UInt32(streamed.count)))
        for row in returned {
            let at = try XCTUnwrap(row.delivery)
            XCTAssertEqual(streamed[Int(at)].key, row.key)
        }
    }

    func testEveryModeWithADecoderTakesABudget() throws {
        guard Mode.wspr.isSupported else { throw XCTSkip("no WSPR in this build") }
        XCTAssertTrue(Mode.wspr.capabilities.contains(.budget))
        let wspr = try Decoder(mode: .wspr)
        try wspr.setBudget { true }
        XCTAssertFalse(wspr.deliveryIsExact, "WSPR is the parallel contract")
    }

    func testTheReportCountsSicEarlySubtractions() throws {
        let decoder = try Decoder(mode: .ft8)
        try decoder.setBudget { true }
        let rows = try decoder.decode(try ft8Slot())
        XCTAssertFalse(decoder.lastBudget.exhausted)
        XCTAssertEqual(Int(decoder.lastBudget.rowsSubtracted), rows.count)
    }
}
