// SPDX-License-Identifier: GPL-3.0-only
//
// FST4W through the Swift binding (#649): pack, tones, slot, decode, the
// unresolved-hash field, and the Keff-50 known-call list. The same shape as
// `mfsk-ffi/tests/fst4w_ffi.rs`.

import XCTest
@testable import MfskCore

final class FST4WTests: XCTestCase {
    func testPackTransmitDecodeRoundTrips() throws {
        let slot = try Mode.fst4w120.synthesiseSlot(Message.fst4w("K1ABC FN42 37"), frequencyHz: 1500)
        let decoder = try Decoder(mode: .fst4w120)
        let rows = try decoder.decode(slot)
        XCTAssertEqual(rows.map(\.text), ["K1ABC FN42 37"])
        XCTAssertEqual(rows.first?.mode, .fst4w120)
        XCTAssertEqual(rows.first?.informationBitCount, 74)
        XCTAssertEqual(rows.first?.keyBits, 50)
        XCTAssertNil(rows.first?.hash22)
    }

    func testAnUnresolvedHashReachesTheRow() throws {
        let slot = try Mode.fst4w120.synthesiseSlot(Message.fst4w("<JA1XYZ> PM95AA"), frequencyHz: 1500)
        let rows = try Decoder(mode: .fst4w120).decode(slot)
        XCTAssertEqual(rows.map(\.text), ["<...> PM95AA"])
        XCTAssertNotNil(rows.first?.hash22)
    }

    func testTheKnownCallListRoundTripsAndIsLearned() throws {
        let decoder = try Decoder(mode: .fst4w120)
        XCTAssertEqual(try decoder.wcalls(), [])
        try decoder.setWcalls(["JA1XYZ PM95", "VK3NV QF22"])
        XCTAssertEqual(try decoder.wcalls(), ["JA1XYZ PM95", "VK3NV QF22"])
        XCTAssertThrowsError(try decoder.setWcalls((0..<101).map { "C\($0)" })) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
        try decoder.setWcalls([])
        let slot = try Mode.fst4w120.synthesiseSlot(Message.fst4w("K1ABC FN42 37"), frequencyHz: 1500)
        XCTAssertEqual(try decoder.decode(slot).map(\.text), ["K1ABC FN42 37"])
        XCTAssertEqual(try decoder.wcalls(), ["K1ABC FN42"])
    }

    func testOtherModesHaveNoList() throws {
        let decoder = try Decoder(mode: .ft8)
        XCTAssertThrowsError(try decoder.wcalls()) { error in
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }

    func testAMessageFST4WCannotSendIsRefused() {
        for bad in ["CQ K1ABC FN42", "hello", "K1ABC FN42 38"] {
            XCTAssertThrowsError(try Message.fst4w(bad), bad) { error in
                XCTAssertEqual((error as? MfskError)?.code, .decodeFailed)
            }
        }
    }
}
