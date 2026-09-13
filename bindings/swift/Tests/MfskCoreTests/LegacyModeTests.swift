// SPDX-License-Identifier: GPL-3.0-or-later
//
// WSPR, JT9 and JT65 — the modes with no decode handle. Each is a
// one-call encode and a one-call decode, and each slot is 60 s or 120 s
// of audio, so these are the slowest tests here by a wide margin.

import XCTest
@testable import MfskCore

final class LegacyModeTests: XCTestCase {
    func testWSPRRoundTrips() throws {
        guard let info = try? Mode.wspr.info else { throw XCTSkip("no WSPR in this build") }
        XCTAssertFalse(Mode.wspr.capabilities.contains(.decodeHandle),
                       "WSPR decodes through its own entry point, not the session")
        let audio = try WSPR.encode(call: "K1ABC", grid: "FN42", powerDBM: 37, frequencyHz: 1500)
        // The encoder returns the transmission, not a padded slot: 162
        // symbols x 8192 samples is 110.6 s of the 120 s period, and the
        // silence either side is the caller's to add.
        XCTAssertEqual(audio.count, Int(info.totalSymbols * info.samplesPerSymbol))
        XCTAssertLessThan(audio.count, Int(info.slotSamples12k))
        let rows = try WSPR.decode(audio.asPCM16())
        XCTAssertTrue(rows.contains { $0.text.contains("K1ABC") },
                      "expected K1ABC in \(rows.map(\.text))")
        XCTAssertTrue(rows.allSatisfy { $0.mode == .wspr })
    }

    func testJT9RoundTrips() throws {
        guard (try? Mode.jt9.info) != nil else { throw XCTSkip("no JT9 in this build") }
        // 1350 Hz rather than 1500 on purpose: the pre-v2 ABI hardcoded
        // 1500 with no way to say otherwise, and the frequency argument
        // is the thing being exercised.
        let audio = try JT9.encode(call1: "CQ", call2: "K1ABC", gridOrReport: "FN42",
                                   frequencyHz: 1350)
        let rows = try JT9.decode(audio.asPCM16(), frequencyHz: 1350)
        XCTAssertTrue(rows.contains { $0.text.contains("K1ABC") },
                      "expected K1ABC in \(rows.map(\.text))")
    }

    func testJT65RoundTrips() throws {
        guard (try? Mode.jt65.info) != nil else { throw XCTSkip("no JT65 in this build") }
        let audio = try JT65.encode(call1: "CQ", call2: "K1ABC", gridOrReport: "FN42",
                                    frequencyHz: 1270)
        let rows = try JT65.decode(audio.asPCM16(), frequencyHz: 1270)
        XCTAssertTrue(rows.contains { $0.text.contains("K1ABC") },
                      "expected K1ABC in \(rows.map(\.text))")
    }

    func testPCM16ConversionClampsRatherThanWrapping() {
        let converted: [Int16] = [-2.0, -1.0, 0.0, 0.5, 1.0, 2.0].asPCM16()
        XCTAssertEqual(converted, [-32768, -32767, 0, 16384, 32767, 32767])
    }
}
