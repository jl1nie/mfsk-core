// SPDX-License-Identifier: GPL-3.0-or-later
//
// The pack77 family and the tone stage.

import XCTest
@testable import MfskCore

final class MessageTests: XCTestCase {
    func testStandardExchangeRendersBackAsText() throws {
        let message = try Message.standard(call1: "CQ", call2: "JA1ABC", report: "PM95")
        XCTAssertEqual(message.bits.count, 77)
        XCTAssertEqual(try message.text(), "CQ JA1ABC PM95")
    }

    func testFreeTextRendersBackAsText() throws {
        let message = try Message.freeText("HELLO WORLD")
        XCTAssertTrue(try message.text().contains("HELLO WORLD"))
    }

    func testAHashedCallsignResolvesOnlyOnceTheSessionKnowsIt() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        // Type 4 carries one non-standard call in full and a *hash* of
        // the standard one, so the text depends on what the receiving
        // session has been taught.
        // The report is empty on purpose: type 4 spends its payload on
        // the non-standard call in full plus the hash, and
        // `mfsk_pack77_type4` refuses a signal report on top of that
        // with "the message does not fit this format".
        let message = try Message.type4(nonStandardCall: "JA1ABC/QRP", standardCall: "JL1NIE",
                                        report: "", isCQ: false)
        let unresolved = try message.text()
        XCTAssertTrue(unresolved.contains("<...>"), "got '\(unresolved)'")

        let session = try DecodeSession(mode: .ft8)
        try session.addCallsign("JL1NIE")
        XCTAssertTrue(try message.text(resolvedBy: session).contains("JL1NIE"))
    }

    func testAnInvalidCallsignFailsToPack() {
        XCTAssertThrowsError(try Message.standard(call1: "CQ", call2: "!!!not a call!!!",
                                                  report: "PM95"))
    }

    func testMessageBitsMustBe77() {
        XCTAssertThrowsError(try Message(bits: [0, 1, 0])) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
    }

    func testToneCountMatchesWhatTheModePublishes() throws {
        let message = try Message.standard(call1: "CQ", call2: "JA1ABC", report: "PM95")
        for mode in Mode.supported where mode.symbolCount > 0 {
            let tones = try message.tones(for: mode)
            XCTAssertEqual(tones.count, mode.symbolCount, "\(mode.name)")
            let info = try mode.info
            XCTAssertTrue(tones.allSatisfy { UInt32($0) < info.toneCount },
                          "\(mode.name): a tone index outside 0..<\(info.toneCount)")
        }
    }

    func testAModeWithoutAToneStageSaysSoRatherThanGuessing() throws {
        guard (try? Mode.wspr.info) != nil else { throw XCTSkip("no WSPR in this build") }
        XCTAssertEqual(Mode.wspr.symbolCount, 0)
        let message = try Message.standard(call1: "CQ", call2: "JA1ABC", report: "PM95")
        XCTAssertThrowsError(try message.tones(for: .wspr)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }

    func testFrameLengthIsAskedOfTheModeNotAssumed() throws {
        // The FST4 sub-modes differ by a factor of 30 here, which is the
        // reason `mfsk_synth_output_len` exists at all.
        let lengths = Mode.supported
            .filter { $0.symbolCount > 0 }
            .map { ($0, $0.synthesisedFrameLength) }
        for (mode, length) in lengths {
            XCTAssertGreaterThan(length, 0, "\(mode.name)")
            let info = try mode.info
            XCTAssertEqual(length, Int(info.totalSymbols * info.samplesPerSymbol), "\(mode.name)")
        }
    }
}
