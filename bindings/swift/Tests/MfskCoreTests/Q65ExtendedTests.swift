// SPDX-License-Identifier: GPL-3.0-or-later
//
// `Q65.decode(_:mode:params:...)` and the two lists WSJT-X keeps (#466). Each
// setting has to change what is decoded: a field that is accepted and ignored
// looks identical to one that works. The scenarios are the ones
// `mfsk-ffi/tests/q65_ex_ffi.rs` and the Kotlin JVM test use.

import CMfsk
import XCTest
@testable import MfskCore

final class Q65ExtendedTests: XCTestCase {
    private let sub = Q65SubMode.a30
    private let want = "K1ABC JA1ABC -15"

    private func requireQ65() throws {
        guard sub.mode.isSupported else { throw XCTSkip("no Q65 in this build") }
    }

    /// A 30 s slot with `signal` starting `start` seconds in.
    private func slot(_ signal: [Float], start: Float) -> [Float] {
        var out = [Float](repeating: 0, count: 30 * 12_000)
        let at = Int((start * 12_000).rounded())
        for (i, s) in signal.enumerated() where at + i < out.count { out[at + i] = s }
        return out
    }

    private func frame(_ a: String, _ b: String, _ c: String, flagged: Bool = false) throws -> [Float] {
        try Q65.encode(subMode: sub, call1: a, call2: b, gridOrReport: c,
                       copiedLastTx: flagged, frequencyHz: 1500)
    }

    func testTheDefaultsAreTheLibrarysOwnAndSayWhereDtZeroIs() throws {
        try requireQ65()
        let p = try Q65.Params(mode: .q65a30)
        XCTAssertEqual(p.frequencyRangeHz, 200...3_000)
        // WSJT-X's ±1 s window, not the wide one the strategy functions scan.
        XCTAssertEqual(p.earlyToleranceSeconds, 1)
        XCTAssertEqual(p.lateToleranceSeconds, 1)
        XCTAssertEqual(p.nominalStartSeconds, 0.5, "Q65-30A's frame starts 0.5 s in")
        XCTAssertEqual(p.ftolHz, 10)
        XCTAssertNil(p.rxFrequencyHz)
        XCTAssertNil(p.fading)
        XCTAssertNil(p.list)
        XCTAssertNil(p.apHint)
        XCTAssertThrowsError(try Q65.Params(mode: .ft8), "FT8 is not a Q65 mode")
    }

    func testASettingThatCannotBeHonouredIsRefusedNotDropped() throws {
        try requireQ65()
        let audio = [Float](repeating: 0, count: 30 * 12_000)
        let base = try Q65.Params(mode: .q65a30)
        func refused(_ what: String, _ needle: String, _ p: Q65.Params,
                     callers: Q65Callers? = nil) {
            XCTAssertThrowsError(try Q65.decode(audio, mode: .q65a30, params: p, callers: callers),
                                 what) { error in
                guard let error = error as? MfskError else { return XCTFail("wrong error type") }
                XCTAssertTrue(error.detail.contains(needle),
                              "\(what): the refusal should name \(needle), got \(error.detail)")
            }
        }
        var p = base
        p.pileup = true
        refused("Pileup with no hint", "AP hint", p)
        p = base
        p.rxFrequencyHz = 1500
        refused("an Rx frequency with no list", "ap_list", p)
        p = base
        p.list = .standard(myCall: "K1ABC", hisCall: "JA1ABC")
        p.fading = Q65.Fading(b90Ts: 1)
        refused("a list with fading", "mutually exclusive", p)
        p = base
        p.maxDrift = 10
        p.fading = Q65.Fading(b90Ts: 1)
        refused("drift with fading", "fast-fading", p)
        p = base
        p.maxDrift = 51
        refused("drift past 50", "max_drift", p)
        p = base
        p.list = .contest(myCall: "K1ABC")
        refused("the contest list with no callers", "MfskQ65Callers", p)
    }

    func testDtIsMeasuredFromTheNominalStartAndTheFlaggedRowSaysSo() throws {
        try requireQ65()
        let plain = try frame("K1ABC", "JA1ABC", "-15")
        for (start, dt) in [(Float(0.5), Float(0)), (0.9, 0.4), (0.2, -0.3)] {
            let rows = try Q65.decode(slot(plain, start: start), mode: .q65a30)
            XCTAssertEqual(rows.map(\.text), [want], "start \(start)")
            XCTAssertEqual(rows.first?.dtSeconds ?? 99, dt, accuracy: 0.05)
            XCTAssertEqual(rows.first?.mode, .q65a30)
            XCTAssertEqual(rows.first?.copiedLastTx, false)
        }
        let flagged = try frame("K1ABC", "JA1ABC", "-15", flagged: true)
        XCTAssertNotEqual(flagged, plain, "the flag changes the transmission")
        let rows = try Q65.decode(slot(flagged, start: 0.5), mode: .q65a30)
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.copiedLastTx, true)
    }

    func testTheEMEDelayReachesAFrameTheDefaultWindowDoesNot() throws {
        try requireQ65()
        let late = slot(try frame("K1ABC", "JA1ABC", "-15"), start: 3.5)
        var p = try Q65.Params(mode: .q65a30)
        XCTAssertTrue(try Q65.decode(late, mode: .q65a30, params: p).isEmpty)
        p.emeDelay = true
        let rows = try Q65.decode(late, mode: .q65a30, params: p)
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.dtSeconds ?? 99, 3.0, accuracy: 0.05)
    }

    func testPileupFreesThe78thBitForAMyCallDxCallHint() throws {
        try requireQ65()
        var p = try Q65.Params(mode: .q65a30)
        p.apHint = Q65.APHint(call1: "K1ABC", call2: "JA1ABC")
        p.pileup = true
        let flagged = slot(try frame("K1ABC", "JA1ABC", "-15", flagged: true), start: 0.5)
        let rows = try Q65.decode(flagged, mode: .q65a30, params: p)
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.copiedLastTx, true)
        // An unflagged reply decodes under Pileup too.
        let plain = slot(try frame("K1ABC", "JA1ABC", "-15"), start: 0.5)
        XCTAssertEqual(try Q65.decode(plain, mode: .q65a30, params: p).map(\.text), [want])
    }

    func testQ3FindsAListMessageAtTheRxFrequency() throws {
        try requireQ65()
        let audio = slot(try frame("K1ABC", "JA1ABC", "-15"), start: 0.5)
        var p = try Q65.Params(mode: .q65a30)
        // A window that holds nothing else, so only q3 can find it.
        p.frequencyRangeHz = 3_900...3_950
        p.list = .standard(myCall: "K1ABC", hisCall: "JA1ABC", hisGrid: "PM95")
        XCTAssertTrue(try Q65.decode(audio, mode: .q65a30, params: p).isEmpty,
                      "template matching over the scan window finds nothing here")
        p.rxFrequencyHz = 1500
        XCTAssertEqual(try Q65.decode(audio, mode: .q65a30, params: p).map(\.text), [want])
        p.rxFrequencyHz = 1600
        XCTAssertTrue(try Q65.decode(audio, mode: .q65a30, params: p).isEmpty,
                      "100 Hz off the Rx frequency is outside F Tol")
    }

    func testTheContestListDecodesACallerItRemembers() throws {
        try requireQ65()
        let audio = slot(try frame("K1ABC", "W9XYZ", "RR73"), start: 0.5)
        var p = try Q65.Params(mode: .q65a30)
        p.frequencyRangeHz = 3_900...3_950
        p.rxFrequencyHz = 1500
        p.list = .contest(myCall: "K1ABC")
        let callers = try Q65Callers()
        XCTAssertTrue(try Q65.decode(audio, mode: .q65a30, params: p, callers: callers).isEmpty,
                      "nobody listed: nothing to find")
        try callers.record(frequencyHz: 1500, message: "K1ABC W9XYZ EN37", now: 1_000)
        XCTAssertEqual(
            try Q65.decode(audio, mode: .q65a30, params: p, callers: callers).map(\.text),
            ["K1ABC W9XYZ RR73"])
    }

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
}
