// SPDX-License-Identifier: GPL-3.0-or-later
//
// The three strategies the capability word advertised before any C
// entry point could reach them: a decode budget, known-signal carry,
// and slot-FFT reuse.

import XCTest
@testable import MfskCore

final class StrategyTests: XCTestCase {
    /// Two stations in one slot, so a budget has something to cut and a
    /// known list something to remove.
    private func twoStationSlot() throws -> [Int16] {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        let a = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                            frequencyHz: 1200)
        let b = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "VK3NV", report: "QF22",
                                            frequencyHz: 1800)
        return zip(a, b).map { $0.addingReportingOverflow($1).partialValue }
    }

    func testWithoutABudgetTheReportIsEmpty() throws {
        let audio = try twoStationSlot()
        let session = try DecodeSession(mode: .ft8)
        XCTAssertFalse(try session.decode(audio).isEmpty)

        let report = session.lastBudget
        XCTAssertFalse(report.exhausted)
        XCTAssertEqual(report.candidatesSkipped, 0)
        // Absent, not zero: -1 and NaN in C, nil here, because 0 is a
        // real sync count and 0.0 a real score.
        XCTAssertNil(report.cutAtSync)
        XCTAssertNil(report.cutAtScore)
    }

    func testABudgetThatRefusesEverythingCutsTheSearch() throws {
        let audio = try twoStationSlot()
        let session = try DecodeSession(mode: .ft8)
        let full = try session.decode(audio).count
        XCTAssertGreaterThan(full, 0)

        let polls = Counter()
        try session.setBudget { polls.increment(); return false }
        let cut = try session.decode(audio)
        XCTAssertGreaterThan(polls.value, 0, "the predicate was never polled")
        XCTAssertLessThan(cut.count, full)

        let report = session.lastBudget
        XCTAssertTrue(report.exhausted)
        XCTAssertGreaterThan(report.candidatesSkipped, 0)
        XCTAssertNotNil(report.cutAtSync, "FT8 ranks by Costas sync, so the cut point is knowable")

        try session.setBudget(nil)
        XCTAssertEqual(try session.decode(audio).count, full, "removing it restores the search")
        XCTAssertFalse(session.lastBudget.exhausted)
    }

    func testAGenerousBudgetChangesNothing() throws {
        let audio = try twoStationSlot()
        let session = try DecodeSession(mode: .ft8)
        let full = try session.decode(audio).count
        try session.setBudget { true }
        XCTAssertEqual(try session.decode(audio).count, full)
        XCTAssertFalse(session.lastBudget.exhausted)
    }

    func testKnownSignalsAreNotReportedTwice() throws {
        let audio = try twoStationSlot()
        let session = try DecodeSession(mode: .ft8)
        try session.keepKnown(true)
        XCTAssertEqual(session.knownCount, 0)

        let first = try session.decode(audio)
        XCTAssertTrue(first.contains { $0.text.contains("JA1ABC") })
        XCTAssertEqual(session.knownCount, first.count)

        XCTAssertTrue(try session.decode(audio).isEmpty,
                      "a second pass over the same slot re-reported known signals")

        try session.keepKnown(false)
        XCTAssertEqual(session.knownCount, 0, "off also drops what was held")
        XCTAssertEqual(try session.decode(audio).count, first.count)
    }

    func testTheFFTCacheDoesNotChangeTheAnswer() throws {
        let audio = try twoStationSlot()
        let plain = try DecodeSession(mode: .ft8).decode(audio).map(\.text).sorted()

        let session = try DecodeSession(mode: .ft8)
        try session.keepFFTCache(true)
        XCTAssertEqual(try session.decode(audio).map(\.text).sorted(), plain)
        XCTAssertEqual(try session.decode(audio).map(\.text).sorted(), plain,
                       "the reused transform must decode the same slot the same way")
    }

    func testAStaleCacheIsNotReused() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        // The failure this guards is silent: a slot transform of other
        // audio gives a confident wrong answer.
        let first = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                                frequencyHz: 1200)
        let second = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "VK3NV", report: "QF22",
                                                 frequencyHz: 1800)
        let session = try DecodeSession(mode: .ft8)
        try session.keepFFTCache(true)
        XCTAssertTrue(try session.decode(first).contains { $0.text.contains("JA1ABC") })
        let rows = try session.decode(second)
        XCTAssertTrue(rows.contains { $0.text.contains("VK3NV") },
                      "got \(rows.map(\.text)) — a stale cache would look exactly like this")
        XCTAssertFalse(rows.contains { $0.text.contains("JA1ABC") })
    }

    func testTheCapabilityBitsAndTheEntryPointsAgree() throws {
        for mode in Mode.supported where mode.capabilities.contains(.decodeHandle) {
            let session = try DecodeSession(mode: mode)
            let caps = mode.capabilities

            var budgetAccepted = true
            do { try session.setBudget { true } } catch { budgetAccepted = false }
            XCTAssertEqual(budgetAccepted, caps.contains(.budget), "\(mode.name): budget")
            try? session.setBudget(nil)

            var knownAccepted = true
            do { try session.keepKnown(true) } catch { knownAccepted = false }
            XCTAssertEqual(knownAccepted, caps.contains(.knownFilter), "\(mode.name): known")

            var cacheAccepted = true
            do { try session.keepFFTCache(true) } catch { cacheAccepted = false }
            XCTAssertEqual(cacheAccepted, caps.contains(.fftCache), "\(mode.name): fft cache")
        }
    }
}

/// The budget predicate can be polled from rayon workers, so even a
/// counter needs a lock.
private final class Counter {
    private let lock = NSLock()
    private var count = 0

    func increment() {
        lock.lock()
        defer { lock.unlock() }
        count += 1
    }

    var value: Int {
        lock.lock()
        defer { lock.unlock() }
        return count
    }
}
