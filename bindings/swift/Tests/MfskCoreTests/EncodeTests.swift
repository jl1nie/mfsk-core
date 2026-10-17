// SPDX-License-Identifier: GPL-3.0-only
//
// `Mode.encode` / `Mode.encodeInfo` over `mfsk_encode` / `mfsk_encode_info`.

import XCTest
@testable import MfskCore

final class EncodeTests: XCTestCase {
    private func requireFT8() throws {
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
    }

    func testEncodeAgreesWithTheStagedPipeline() throws {
        try requireFT8()
        let pcm = try Mode.ft8.encode("CQ JA1ABC PM95", frequencyHz: 1500)
        let info = try Mode.ft8.info
        XCTAssertEqual(pcm.count, Int(info.totalSymbols * info.samplesPerSymbol))

        let message = try Message.standard(call1: "CQ", call2: "JA1ABC", report: "PM95")
        XCTAssertEqual(try Mode.ft8.encodeInfo("CQ JA1ABC PM95").tones,
                       try message.tones(for: .ft8))
    }

    func testInfoSaysWhatAReceiverPrintsAndWhichCallsAreHashed() throws {
        try requireFT8()
        let plain = try Mode.ft8.encodeInfo("CQ JA1ABC PM95")
        XCTAssertEqual(plain.text, "CQ JA1ABC PM95")
        XCTAssertEqual(plain.hashedCalls, ["JA1ABC"])

        let hashed = try Mode.ft8.encodeInfo("<VK3NV> JA1ABC/QRP RR73")
        XCTAssertEqual(hashed.text, "<VK3NV> JA1ABC/QRP RR73")
        XCTAssertTrue(hashed.hashedCalls.contains("VK3NV"), "\(hashed.hashedCalls)")
        let decoder = try Decoder(mode: .ft8)
        for call in hashed.hashedCalls { try decoder.addCallsign(call) }
    }

    func testAnUnpackableTextIsInvalidArgument() throws {
        try requireFT8()
        let tooLong = "THIS TEXT IS FAR TOO LONG TO SEND"
        XCTAssertThrowsError(try Mode.ft8.encode(tooLong, frequencyHz: 1500)) {
            XCTAssertEqual(($0 as? MfskError)?.code, .invalidArgument)
        }
        XCTAssertThrowsError(try Mode.ft8.encodeInfo(tooLong)) {
            XCTAssertEqual(($0 as? MfskError)?.code, .invalidArgument)
        }
    }

    func testTheQ65FlagChangesTheTones() throws {
        guard Mode.q65a60.isSupported else { throw XCTSkip("no Q65 in this build") }
        let plain = try Mode.q65a60.encodeInfo("CQ JA1ABC PM95")
        let flagged = try Mode.q65a60.encodeInfo("CQ JA1ABC PM95", copiedLastTx: true)
        XCTAssertEqual(plain.tones.count, flagged.tones.count)
        XCTAssertNotEqual(plain.tones, flagged.tones)
    }
}
