// SPDX-License-Identifier: GPL-3.0-or-later
//
// The decoder handle's own behaviour, ported from
// `mfsk-ffi/tests/decoder_ffi.rs`: state kept between periods, the hash
// table that is the decoder's own, the other modes through the same handle.

import XCTest
@testable import MfskCore

final class DecoderTests: XCTestCase {
    func testAHashedCallResolvesInTheDecoderThatHeardIt() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        // Period 0: VK3NV is sent in full (a plain message). Period 1:
        // VK3NV is only a hash, in a type-4 message from a non-standard call.
        let heard = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "VK3NV", report: "QF22",
                                                frequencyHz: 1500)
        let type4 = try Message.type4(nonStandardCall: "JA1ABC/QRP", standardCall: "VK3NV",
                                      report: "", isCQ: false)
        let hashed = try Mode.ft8.synthesiseSlot(type4, frequencyHz: 1500)

        let a = try Decoder(mode: .ft8)
        let b = try Decoder(mode: .ft8)
        XCTAssertTrue(try a.decode(heard, period: 10).contains { $0.text.contains("VK3NV") })
        let with = try a.decode(hashed, period: 11)
        let without = try b.decode(hashed, period: 11)
        // `a` learned VK3NV in period 10; `b` never heard it.
        XCTAssertTrue(without.contains { $0.text.contains("<...>") }, "\(without.map(\.text))")
        XCTAssertTrue(with.contains { $0.text.contains("<VK3NV>") }, "\(with.map(\.text))")
        XCTAssertTrue(with.contains { $0.usedHashTable })

        // Teaching it from outside works too, and clearing forgets.
        let c = try Decoder(mode: .ft8)
        try c.addCallsign("VK3NV")
        XCTAssertTrue(try c.decode(hashed).contains { $0.text.contains("<VK3NV>") })
        try c.clear()
        XCTAssertTrue(try c.decode(hashed).contains { $0.text.contains("<...>") })
    }

    func testAModeWhoseMessagesCarryNoHashesSaysSo() throws {
        guard Mode.wspr.isSupported else { throw XCTSkip("no WSPR in this build") }
        let decoder = try Decoder(mode: .wspr)
        XCTAssertThrowsError(try decoder.addCallsign("VK3NV")) { error in
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }

    func testTheOtherModesDecodeThroughTheSameHandle() throws {
        // WSPR: the frame starts 1 s into the 120 s period.
        if Mode.wspr.isSupported {
            let frame = try WSPR.encode(call: "K1ABC", grid: "FN42", powerDBM: 37, frequencyHz: 1500)
            let decoder = try Decoder(mode: .wspr)
            let rows = try decoder.decode(placeInSlot(frame, offsetSeconds: 1, slotSeconds: 120))
            XCTAssertTrue(rows.contains { $0.text.contains("K1ABC FN42 37") }, "\(rows.map(\.text))")
            XCTAssertTrue(rows.allSatisfy { $0.mode == .wspr })
        }
        // JT9 and JT65: the frame starts the period.
        if Mode.jt9.isSupported {
            let frame = try JT9.encode(call1: "CQ", call2: "K1ABC", gridOrReport: "FN42",
                                       frequencyHz: 1500)
            let decoder = try Decoder(mode: .jt9)
            let rows = try decoder.decode(placeInSlot(frame, offsetSeconds: 0, slotSeconds: 60))
            XCTAssertTrue(rows.contains { $0.text.contains("CQ K1ABC FN42") }, "\(rows.map(\.text))")
        }
        if Mode.jt65.isSupported {
            let frame = try JT65.encode(call1: "CQ", call2: "K1ABC", gridOrReport: "FN42",
                                        frequencyHz: 1500)
            let decoder = try Decoder(mode: .jt65)
            let rows = try decoder.decode(placeInSlot(frame, offsetSeconds: 0, slotSeconds: 60))
            XCTAssertTrue(rows.contains { $0.text.contains("CQ K1ABC FN42") }, "\(rows.map(\.text))")
        }
        // Q65-30A, 0.5 s in, with the late-and-early window the Rust test sets.
        if Mode.q65a30.isSupported {
            let frame = try Q65.encode(subMode: .a30, call1: "CQ", call2: "K1ABC",
                                       gridOrReport: "FN42", frequencyHz: 1500)
            var extras = Extras()
            extras.earlyToleranceSeconds = 1
            extras.lateToleranceSeconds = 1
            let decoder = try Decoder(mode: .q65a30, extras: extras)
            let rows = try decoder.decode(placeInSlot(frame, offsetSeconds: 0.5, slotSeconds: 30))
            XCTAssertTrue(rows.contains { $0.text.contains("CQ K1ABC FN42") }, "\(rows.map(\.text))")
            XCTAssertTrue(rows.allSatisfy { $0.mode == .q65a30 })
        }
    }

    func testTheWSPREncoderReturnsTheTransmissionNotAPaddedSlot() throws {
        guard let info = try? Mode.wspr.info else { throw XCTSkip("no WSPR in this build") }
        let audio = try WSPR.encode(call: "K1ABC", grid: "FN42", powerDBM: 37, frequencyHz: 1500)
        // 162 symbols x 8192 samples is 110.6 s of the 120 s period, and the
        // silence either side is the caller's to add.
        XCTAssertEqual(audio.count, Int(info.totalSymbols * info.samplesPerSymbol))
        XCTAssertLessThan(audio.count, Int(info.slotSamples12k))
    }

    func testPCM16ConversionClampsRatherThanWrapping() {
        let converted: [Int16] = [-2.0, -1.0, 0.0, 0.5, 1.0, 2.0].asPCM16()
        XCTAssertEqual(converted, [-32768, -32767, 0, 16384, 32767, 32767])
    }

    func testAnOptionCanBeReplacedBetweenPeriods() throws {
        guard Mode.fst4s60.isSupported else { throw XCTSkip("no FST4 in this build") }
        let decoder = try Decoder(mode: .fst4s60)
        var extras = Extras()
        extras.noiseBlanker = .percent(5)
        XCTAssertNoThrow(try decoder.setExtras(extras))
        extras.noiseBlanker = .percent(99)
        XCTAssertThrowsError(try decoder.setExtras(extras)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
        // And the failed replacement left the decoder usable.
        XCTAssertNoThrow(try decoder.setExtras(Extras()))
    }

    func testAnUnsupportedOptionNamesItself() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        let decoder = try Decoder(mode: .ft8)
        var extras = Extras()
        extras.noiseBlanker = .percent(5)          // FT8 has no blanker
        XCTAssertThrowsError(try decoder.setExtras(extras)) { error in
            guard let error = error as? MfskError else { return XCTFail("wrong error type") }
            XCTAssertEqual(error.code, .unsupported)
            XCTAssertTrue(error.detail.contains("noise blanker"), "got '\(error.detail)'")
        }
    }

    func testTheCallsListedBySetQ65CallersIsQ65Only() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        let decoder = try Decoder(mode: .ft8)
        XCTAssertThrowsError(try decoder.setQ65Callers(try Q65Callers())) { error in
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }
}
