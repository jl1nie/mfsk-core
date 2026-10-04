// SPDX-License-Identifier: GPL-3.0-only
//
// The decode budget. (The known-signal carry and the slot-FFT cache of the
// old decoder handle are gone from the ABI: the decoder keeps what WSJT-X
// keeps between periods, and nothing else.)

import XCTest
@testable import MfskCore

final class StrategyTests: XCTestCase {
    func testWithoutABudgetTheReportIsEmpty() throws {
        let audio = try twoStationFT8Slot()
        let decoder = try Decoder(mode: .ft8)
        XCTAssertFalse(try decoder.decode(audio).isEmpty)

        let report = decoder.lastBudget
        XCTAssertFalse(report.exhausted)
        XCTAssertEqual(report.candidatesSkipped, 0)
        // Absent, not zero: -1 and NaN in C, nil here, because 0 is a
        // real sync count and 0.0 a real score.
        XCTAssertNil(report.cutAtSync)
        XCTAssertNil(report.cutAtScore)
    }

    /// Pinned to the single pass: its scheduler polls per candidate, which is
    /// what `candidatesSkipped` and `cutAtSync` describe. FT8's default since
    /// 0.12.0 (`sicEarly`) polls per stage.
    func testABudgetThatRefusesEverythingCutsTheSearch() throws {
        let audio = try twoStationFT8Slot()
        var extras = Extras()
        extras.strategy = .singlePass
        let decoder = try Decoder(mode: .ft8, extras: extras)
        let full = try decoder.decode(audio).count
        XCTAssertGreaterThan(full, 0)

        let polls = Counter()
        try decoder.setBudget { polls.increment(); return false }
        let cut = try decoder.decode(audio)
        XCTAssertGreaterThan(polls.value, 0, "the predicate was never polled")
        XCTAssertLessThan(cut.count, full)

        let report = decoder.lastBudget
        XCTAssertTrue(report.exhausted)
        XCTAssertGreaterThan(report.candidatesSkipped, 0)
        XCTAssertNotNil(report.cutAtSync, "FT8 ranks by Costas sync, so the cut point is knowable")

        try decoder.setBudget(nil)
        XCTAssertEqual(try decoder.decode(audio).count, full, "removing it restores the search")
        XCTAssertFalse(decoder.lastBudget.exhausted)
    }

    func testAGenerousBudgetChangesNothing() throws {
        let audio = try twoStationFT8Slot()
        let decoder = try Decoder(mode: .ft8)
        let full = try decoder.decode(audio).count
        try decoder.setBudget { true }
        XCTAssertEqual(try decoder.decode(audio).count, full)
        XCTAssertFalse(decoder.lastBudget.exhausted)
    }

    func testTheBudgetBitAndTheEntryPointAgree() throws {
        for mode in Mode.supported {
            guard let decoder = try? Decoder(mode: mode) else { continue }
            var budgetAccepted = true
            do { try decoder.setBudget { true } } catch { budgetAccepted = false }
            XCTAssertEqual(budgetAccepted, mode.capabilities.contains(.budget), "\(mode.name): budget")
            try? decoder.setBudget(nil)
        }
    }
}
