// SPDX-License-Identifier: GPL-3.0-or-later
//
// `transmitFrequencyHz` (FT8's `nftx`) and FST4's noise blanker, added to
// `DecodeParams` after `searchHalfWidthHz` (#466). Both are refused on a mode that lacks
// them, like every other option, rather than dropped.

import XCTest
@testable import MfskCore

final class TransmitFrequencyAndBlankerTests: XCTestCase {
    private func refused(_ mode: Mode, _ mutate: (inout DecodeParams) -> Void,
                         file: StaticString = #filePath, line: UInt = #line) throws {
        guard (try? mode.info) != nil else { throw XCTSkip("no \(mode.name) in this build") }
        var params = try DecodeParams(mode: mode)
        mutate(&params)
        XCTAssertThrowsError(try DecodeSession(mode: mode, params: params), file: file, line: line) {
            XCTAssertEqual(($0 as? MfskError)?.code, .unsupported, file: file, line: line)
        }
    }

    func testTheNewFieldsAreUnsetByDefault() throws {
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        let params = try DecodeParams(mode: .ft8)
        XCTAssertNil(params.transmitFrequencyHz, "the ABI's unset is NaN, which must arrive as nil")
        XCTAssertNil(params.noiseBlanker)
    }

    func testCapabilitiesSayWhichModeHasWhat() throws {
        guard (try? Mode.ft8.info) != nil, (try? Mode.fst4s60.info) != nil else {
            throw XCTSkip("no FT8 or FST4 in this build")
        }
        XCTAssertTrue(Mode.ft8.capabilities.contains(.transmitFrequency))
        XCTAssertFalse(Mode.ft8.capabilities.contains(.noiseBlanker))
        XCTAssertTrue(Mode.fst4s60.capabilities.contains(.noiseBlanker))
        XCTAssertFalse(Mode.fst4s60.capabilities.contains(.transmitFrequency))
    }

    func testTransmitFrequencyIsFT8Only() throws {
        try refused(.ft4) { $0.transmitFrequencyHz = 1500 }
        try refused(.fst4s60) { $0.transmitFrequencyHz = 1500 }
        guard (try? Mode.ft8.info) != nil else { throw XCTSkip("no FT8 in this build") }
        var params = try DecodeParams(mode: .ft8)
        params.transmitFrequencyHz = 1500
        _ = try DecodeSession(mode: .ft8, params: params)
    }

    func testTheBlankerIsFST4Only() throws {
        try refused(.ft8) { $0.noiseBlanker = .percent(5) }
        try refused(.ft4) { $0.noiseBlanker = .percent(5) }
        try refused(.fst4s60) { $0.noiseBlanker = .percent(26) }
        try refused(.fst4s60) { $0.noiseBlanker = .sweep(step: 3) }
        // a sweep looks only near the hint, so it needs one
        try refused(.fst4s60) { $0.noiseBlanker = .sweep(step: 5) }
    }

    func testTheBlankerOpensWhereItIsWanted() throws {
        guard (try? Mode.fst4s60.info) != nil else { throw XCTSkip("no FST4 in this build") }
        var params = try DecodeParams(mode: .fst4s60)
        params.noiseBlanker = .percent(5)
        _ = try DecodeSession(mode: .fst4s60, params: params)
        params.frequencyHintHz = 1500
        params.noiseBlanker = .sweep(step: 5, toleranceHz: 20)
        _ = try DecodeSession(mode: .fst4s60, params: params)
    }
}
