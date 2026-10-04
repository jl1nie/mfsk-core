// SPDX-License-Identifier: GPL-3.0-only
//
// The two lists WSJT-X keeps for Q65 (#466): the caller list (`q65_hist2`)
// and the history (`q65_hist`). Both are the application's, not the
// decoder's.

import XCTest
@testable import MfskCore

final class Q65ListsTests: XCTestCase {
    func testTheCallerListRemembersExpiresAndForgets() throws {
        let c = try Q65Callers()
        try c.record(frequencyHz: 1500, message: "K1ABC W9XYZ EN37", now: 100)
        try c.record(frequencyHz: 1510, message: "K1ABC JA1ABC R PM95", now: 100)
        try c.record(frequencyHz: 1520, message: "K1ABC VK3ABC -15", now: 100)   // no grid
        try c.record(frequencyHz: 1530, message: "K1ABC W9XYZ/R EN37", now: 100) // compound
        XCTAssertEqual(c.count, 2)
        XCTAssertEqual(c.callers[0], Q65Callers.Caller(call: "W9XYZ", grid: "EN37",
                                                       lastHeard: 100, frequencyHz: 1500))
        XCTAssertEqual(c.callers[1], Q65Callers.Caller(call: "JA1ABC", grid: "PM95",
                                                       lastHeard: 100, frequencyHz: 1510))
        try c.record(frequencyHz: 1600, message: "K1ABC W9XYZ RR73", now: 500)
        XCTAssertEqual(c.callers[0].lastHeard, 500, "a known caller is refreshed")
        try c.expire(now: 100 + 24 * 3600 + 1)
        XCTAssertEqual(c.count, 1)
        try c.remove("W9XYZ")
        XCTAssertEqual(c.count, 0)
    }

    func testTheHistoryFindsTheDXStationNearTheRxFrequency() throws {
        let h = try Q65History()
        XCTAssertNil(try h.lookup(rxFrequencyHz: 1500))
        try h.push(frequencyHz: 1500, message: "K1ABC JA1ABC PM95")
        XCTAssertEqual(try h.lookup(rxFrequencyHz: 1508),
                       Q65History.DX(call: "JA1ABC", grid: "PM95"))
        XCTAssertNil(try h.lookup(rxFrequencyHz: 1520), "outside 10 Hz")
        try h.push(frequencyHz: 1500, message: "CQ VK3ABC QF22")
        XCTAssertEqual(try h.lookup(rxFrequencyHz: 1500),
                       Q65History.DX(call: "JA1ABC", grid: "PM95"), "a later CQ is passed over")
        try h.push(frequencyHz: 1500, message: "K1ABC W9XYZ -15")
        XCTAssertEqual(try h.lookup(rxFrequencyHz: 1500), Q65History.DX(call: "W9XYZ", grid: nil))
        for i in 0..<120 { try h.push(frequencyHz: 2000 + Float(i), message: "K1ABC JA1ABC PM95") }
        XCTAssertEqual(h.count, 100, "only the 100 most recent are kept")
    }

    func testTheHistoryRecordsADecodesRows() throws {
        guard Mode.q65a30.isSupported else { throw XCTSkip("no Q65 in this build") }
        let frame = try Q65.encode(subMode: .a30, call1: "K1ABC", call2: "JA1ABC",
                                   gridOrReport: "PM95", frequencyHz: 1500)
        var extras = Extras()
        extras.earlyToleranceSeconds = 1
        extras.lateToleranceSeconds = 1
        let decoder = try Decoder(mode: .q65a30, extras: extras)
        let rows = try decoder.decode(placeInSlot(frame, offsetSeconds: 0.5, slotSeconds: 30))
        XCTAssertFalse(rows.isEmpty)
        let h = try Q65History()
        try h.record(rows)
        XCTAssertEqual(h.count, rows.count)
        XCTAssertEqual(try h.lookup(rxFrequencyHz: 1500)?.call, "JA1ABC")
    }

    func testTheCallerListIsCopiedIntoTheDecoderAtTheCall() throws {
        guard Mode.q65a30.isSupported else { throw XCTSkip("no Q65 in this build") }
        let callers = try Q65Callers()
        try callers.record(frequencyHz: 1500, message: "K1ABC W9XYZ EN37", now: 1_000)
        let decoder = try Decoder(mode: .q65a30)
        XCTAssertNoThrow(try decoder.setQ65Callers(callers))
        // Survives a replaced options block, and can be removed.
        XCTAssertNoThrow(try decoder.setExtras(Extras()))
        XCTAssertNoThrow(try decoder.setQ65Callers(nil))
    }
}
