// SPDX-License-Identifier: GPL-3.0-only
//
// Q65 through the decoder — and the one thing Q65 keeps of its own, the
// sub-mode numbering. The point is not sensitivity (a clean signal decodes on
// any setting); it is that each ``Extras`` / ``DecodeParams`` field reaches
// the right place in the library, and that each combination the engine could
// not honour is refused with its reason, not dropped.

import XCTest
@testable import MfskCore

final class Q65Tests: XCTestCase {
    private let sub = Q65SubMode.a30
    private let want = "K1ABC JA1ABC -15"

    private func requireQ65() throws {
        guard sub.mode.isSupported else { throw XCTSkip("no Q65 in this build") }
    }

    /// A 30 s period with `signal` starting `start` seconds in.
    private func period(_ signal: [Float], start: Float) -> [Float] {
        placeInSlot(signal, offsetSeconds: start, slotSeconds: 30)
    }

    private func frame(_ a: String, _ b: String, _ c: String, flagged: Bool = false) throws -> [Float] {
        try Q65.encode(subMode: sub, call1: a, call2: b, gridOrReport: c,
                       copiedLastTx: flagged, frequencyHz: 1500)
    }

    /// WSJT-X's own ±1 s window, set explicitly so a change of the library's
    /// default would not move these tests.
    private func window() -> Extras {
        var extras = Extras()
        extras.earlyToleranceSeconds = 1
        extras.lateToleranceSeconds = 1
        return extras
    }

    func testSubModeNumberingIsItsOwn() {
        // Q65's discriminants are not MfskMode's, and not in slot-length
        // order: a15 is 6 because it was appended rather than inserted. A
        // wrapper that assumed either would address the wrong sub-mode with
        // nothing to say so.
        XCTAssertEqual(Q65SubMode.a15.rawValue, 6)
        XCTAssertEqual(Q65SubMode.a15.mode, .q65a15)
        XCTAssertNotEqual(Q65SubMode.a15.rawValue, Mode.q65a15.rawValue)
        for sub in Q65SubMode.allCases {
            XCTAssertEqual(Q65SubMode(sub.mode), sub, "\(sub) did not round-trip through Mode")
        }
        XCTAssertNil(Q65SubMode(.ft8))
    }

    func testEncodeLengthMatchesTheModesGeometry() throws {
        try requireQ65()
        let pcm = try Q65.encode(subMode: sub, call1: "CQ", call2: "K1ABC",
                                 gridOrReport: "FN42", frequencyHz: 1000)
        let info = try sub.mode.info
        XCTAssertEqual(pcm.count, Int(info.totalSymbols * info.samplesPerSymbol))
    }

    func testPlainDecodeRoundTripsAndReportsTheConcreteSubMode() throws {
        try requireQ65()
        let signal = try frame("CQ", "K1ABC", "FN42")
        let decoder = try Decoder(mode: sub.mode, extras: window())
        let rows = try decoder.decode(period(signal, start: 0.5))
        XCTAssertTrue(rows.contains { $0.text.contains("K1ABC") }, "\(rows.map(\.text))")
        // Every row reports the concrete sub-mode, which is how a caller
        // tells Q65-30A's rows from Q65-60B's in one list.
        XCTAssertTrue(rows.allSatisfy { $0.mode == .q65a30 })
        XCTAssertEqual(rows.first?.frequencyHz ?? 0, 1500, accuracy: 5)
    }

    func testDtIsMeasuredFromTheNominalStartAndTheFlaggedRowSaysSo() throws {
        try requireQ65()
        let plain = try frame("K1ABC", "JA1ABC", "-15")
        let decoder = try Decoder(mode: sub.mode, extras: window())
        for (start, dt) in [(Float(0.5), Float(0)), (0.9, 0.4), (0.2, -0.3)] {
            let rows = try decoder.decode(period(plain, start: start))
            XCTAssertEqual(rows.map(\.text), [want], "start \(start)")
            XCTAssertEqual(rows.first?.dtSeconds ?? 99, dt, accuracy: 0.05)
            XCTAssertEqual(rows.first?.copiedLastTx, false)
        }
        let flagged = try frame("K1ABC", "JA1ABC", "-15", flagged: true)
        XCTAssertNotEqual(flagged, plain, "the flag changes the transmission")
        let rows = try decoder.decode(period(flagged, start: 0.5))
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.copiedLastTx, true)
    }

    func testTheEMEDelayReachesAFrameTheDefaultWindowDoesNot() throws {
        try requireQ65()
        let late = period(try frame("K1ABC", "JA1ABC", "-15"), start: 3.5)
        let plain = try Decoder(mode: sub.mode, extras: window())
        XCTAssertTrue(try plain.decode(late).isEmpty)

        var params = try DecodeParams(mode: sub.mode)
        params.emeDelay = true
        let eme = try Decoder(mode: sub.mode, params: params, extras: window())
        let rows = try eme.decode(late)
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.dtSeconds ?? 99, 3.0, accuracy: 0.05)
    }

    func testPileupFreesThe78thBitForAMyCallDxCallHint() throws {
        try requireQ65()
        var extras = window()
        extras.apHint = Extras.APHint(call1: "K1ABC", call2: "JA1ABC")
        extras.pileup = true
        let decoder = try Decoder(mode: sub.mode, extras: extras)
        let flagged = period(try frame("K1ABC", "JA1ABC", "-15", flagged: true), start: 0.5)
        let rows = try decoder.decode(flagged)
        XCTAssertEqual(rows.map(\.text), [want])
        XCTAssertEqual(rows.first?.copiedLastTx, true)
        // An unflagged reply decodes under Pileup too.
        let plain = period(try frame("K1ABC", "JA1ABC", "-15"), start: 0.5)
        XCTAssertEqual(try decoder.decode(plain).map(\.text), [want])
    }

    func testASettingThatCannotBeHonouredIsRefusedNotDropped() throws {
        try requireQ65()
        func refused(_ what: String, _ expected: MfskError.Code, _ build: (inout Extras) -> Void) {
            var extras = Extras()
            build(&extras)
            XCTAssertThrowsError(try Decoder(mode: sub.mode, extras: extras), what) { error in
                guard let error = error as? MfskError else { return XCTFail("wrong error type") }
                XCTAssertEqual(error.code, expected, what)
                XCTAssertFalse(error.detail.isEmpty, "\(what): the refusal should say why")
            }
        }
        refused("Pileup with no hint", .unsupported) { $0.pileup = true }
        refused("drift past 50 bins", .invalidArgument) { $0.maxDrift = 51 }
        // Options of other modes.
        refused("an FT8 strategy", .unsupported) { $0.strategy = .sicRounds(2) }
        refused("a noise blanker", .unsupported) { $0.noiseBlanker = .percent(5) }
        refused("a7", .unsupported) { $0.a7 = true }
        // And FT8 has no Q65 option.
        if Mode.ft8.isSupported {
            var extras = Extras()
            extras.maxDrift = 10
            XCTAssertThrowsError(try Decoder(mode: .ft8, extras: extras)) { error in
                XCTAssertEqual((error as? MfskError)?.code, .unsupported)
            }
        }
    }

    func testFadingAndMaxDriftAreAcceptedOnQ65() throws {
        try requireQ65()
        var extras = window()
        extras.fading = Extras.Fading(b90Ts: 0.1, model: .lorentzian)
        let signal = period(try frame("CQ", "K1ABC", "FN42"), start: 0.5)
        let fading = try Decoder(mode: sub.mode, extras: extras)
        XCTAssertTrue(try fading.decode(signal).contains { $0.text.contains("K1ABC") })

        var drift = window()
        drift.maxDrift = 2
        let decoder = try Decoder(mode: sub.mode, extras: drift)
        XCTAssertTrue(try decoder.decode(signal).contains { $0.text.contains("K1ABC") })
    }

    func testAnAPHintWithTheFieldsInTheWrongOrderIsAWrongHint() throws {
        try requireQ65()
        // A hint locks message bits, so the right callsigns in the wrong
        // fields are not a weaker hint but a wrong one. Every candidate is
        // tried without AP first (as `jt9 -3` does, since #555), so a clean
        // signal still decodes either way; what the wrong order costs is the
        // AP gain on a weak one. Both orders must at least be accepted.
        let signal = period(try frame("CQ", "K1ABC", "FN42"), start: 0.5)
        for hint in [Extras.APHint(call1: "CQ", call2: "K1ABC", grid: "FN42"),
                     Extras.APHint(call1: "K1ABC", call2: "CQ", grid: "FN42")] {
            var extras = window()
            extras.apHint = hint
            let decoder = try Decoder(mode: sub.mode, extras: extras)
            XCTAssertTrue(try decoder.decode(signal).contains { $0.text.contains("K1ABC") },
                          "\(hint)")
        }
    }

    func testQ65AdvertisesNoDecodeHandleBitYetDecodesThroughTheDecoder() throws {
        try requireQ65()
        // The capability bit says which modes drive the FT8/FT4/FST4
        // `DecodeRequest` builder; it is not the test for whether a Decoder
        // opens.
        XCTAssertFalse(sub.mode.capabilities.contains(.decodeHandle))
        XCTAssertNoThrow(try Decoder(mode: sub.mode))
    }
}
