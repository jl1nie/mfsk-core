// SPDX-License-Identifier: GPL-3.0-only
//
// Synthesise a known message, hand the PCM back to the decoder, and
// check the text survives the trip — the same end-to-end shape as
// `mfsk-ffi/examples/cpp_smoke/main.cpp`, which is what makes this a
// check of the binding rather than of one function at a time.

import XCTest
@testable import MfskCore

final class RoundTripTests: XCTestCase {
    private let call1 = "CQ"
    private let call2 = "JA1ABC"
    private let grid = "PM95"

    private func requireSupported(_ mode: Mode) throws -> ModeInfo {
        try XCTUnwrap(try? mode.info, "this build has no \(mode.name); nothing to test")
    }

    private func ft8Slot() throws -> [Int16] {
        _ = try requireSupported(.ft8)
        return try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid, frequencyHz: 1500)
    }

    func testFT8SlotRoundTrips() throws {
        let slot = try ft8Slot()
        let decoder = try Decoder(mode: .ft8)
        let rows = try decoder.decode(slot)
        XCTAssertTrue(rows.contains { $0.text.contains(call2) },
                      "expected \(call2) in \(rows.map(\.text))")
        for row in rows {
            XCTAssertEqual(row.mode, .ft8)
            XCTAssertEqual(row.frequencyHz, 1500, accuracy: 5)
        }
        XCTAssertEqual(rows.first?.informationBitCount, 91)
    }

    func testFT4SlotRoundTrips() throws {
        _ = try requireSupported(.ft4)
        let slot = try Mode.ft4.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        let decoder = try Decoder(mode: .ft4)
        XCTAssertTrue(try decoder.decode(slot).contains { $0.text.contains(call2) })
    }

    func testFloatAndIntegerPathsAgree() throws {
        let slot = try ft8Slot()
        let floats = slot.map { Float($0) / 32768.0 }
        let decoder = try Decoder(mode: .ft8)
        let fromInts = try decoder.decode(slot).map(\.text).sorted()
        let fromFloats = try decoder.decode(floats).map(\.text).sorted()
        XCTAssertEqual(fromInts, fromFloats,
                       "the f32 entry point is not a lossier wrapper; it should find the same rows")
    }

    func testFloatAudioReachesTheEnginesAtAnyLevel() throws {
        // A quiet float buffer, as a radio adapter at a low volume gives.
        let quiet = try ft8Slot().map { Float($0) / 32768.0 * 0.002 }
        let decoder = try Decoder(mode: .ft8)
        XCTAssertTrue(try decoder.decode(quiet).contains { $0.text.contains(call2) })
    }

    func testAFrequencyBandThatExcludesTheSignalFindsNothing() throws {
        let slot = try ft8Slot()
        var params = try DecodeParams(mode: .ft8)
        params.bandHz = 2500...2900
        let decoder = try Decoder(mode: .ft8, params: params)
        XCTAssertFalse(try decoder.decode(slot).contains { $0.text.contains(call2) },
                       "a 2500-2900 Hz search still found a 1500 Hz signal")
    }

    func testParametersChangeBetweenPeriods() throws {
        let slot = try ft8Slot()
        let decoder = try Decoder(mode: .ft8)
        var params = try DecodeParams(mode: .ft8)
        params.bandHz = 2500...2900
        try decoder.setParams(params)
        XCTAssertFalse(try decoder.decode(slot).contains { $0.text.contains(call2) })
        try decoder.setParams(try DecodeParams(mode: .ft8))
        XCTAssertTrue(try decoder.decode(slot).contains { $0.text.contains(call2) })
    }

    func testAnAPHintIsAcceptedWhereTheModeAdvertisesOne() throws {
        _ = try requireSupported(.ft8)
        XCTAssertTrue(Mode.ft8.capabilities.contains(.apWideband))
        let slot = try ft8Slot()
        // Message-field order — `CQ JA1ABC PM95` means call1 is "CQ".
        // AP is one rung of FT8's ladder, so this decodes either way on a
        // clean signal.
        var extras = Extras()
        extras.apHint = Extras.APHint(call1: call1, call2: call2, grid: grid)
        var params = try DecodeParams(mode: .ft8)
        params.rxFrequencyHz = 1500
        let decoder = try Decoder(mode: .ft8, params: params, extras: extras)
        XCTAssertTrue(try decoder.decode(slot).contains { $0.text.contains(call2) })
    }

    func testAnOptionTheModeCannotHonourFailsAtOpenAsUnsupported() throws {
        // `decoder_ffi.rs`'s `an_option_the_mode_lacks_is_unsupported`:
        // FST4 has no subtraction, FT4 no a7, WSPR no AP, FT8 no blanker.
        func code(_ mode: Mode, _ build: (inout Extras) -> Void) throws -> MfskError.Code? {
            guard mode.isSupported else { throw XCTSkip("no \(mode.name) in this build") }
            var extras = Extras()
            build(&extras)
            do {
                _ = try Decoder(mode: mode, extras: extras)
                return nil
            } catch let error as MfskError {
                XCTAssertFalse(error.detail.isEmpty, "the refusal should say why")
                return error.code
            }
        }
        XCTAssertNil(try code(.ft8) { $0.strategy = .sicRounds(2) })
        XCTAssertEqual(try code(.fst4s60) { $0.strategy = .sicRounds(2) }, .unsupported)
        XCTAssertEqual(try code(.wspr) { $0.strategy = .sicRounds(2) }, .unsupported)

        XCTAssertNil(try code(.ft8) { $0.a7 = true })
        XCTAssertEqual(try code(.ft4) { $0.a7 = true }, .unsupported)

        XCTAssertNil(try code(.ft8) { $0.apHint = Extras.APHint(call1: "CQ") })
        XCTAssertEqual(try code(.wspr) { $0.apHint = Extras.APHint(call1: "CQ") }, .unsupported)
        XCTAssertEqual(try code(.jt9) { $0.apHint = Extras.APHint(call1: "CQ") }, .unsupported)

        XCTAssertNil(try code(.fst4s60) { $0.noiseBlanker = .percent(5) })
        XCTAssertEqual(try code(.ft8) { $0.noiseBlanker = .percent(5) }, .unsupported)

        // An out-of-range value is the caller's mistake, not a missing option.
        XCTAssertEqual(try code(.fst4s60) { $0.noiseBlanker = .percent(99) }, .invalidArgument)
        XCTAssertEqual(try code(.fst4s60) { $0.noiseBlanker = .sweep(step: 3, toleranceHz: 20) },
                       .invalidArgument)
        XCTAssertEqual(try code(.fst4s60) { $0.noiseBlanker = .sweep(step: 5, toleranceHz: 0) },
                       .invalidArgument)
    }

    func testAParameterBlockThatIsNotUsableIsRefusedNotClamped() throws {
        _ = try requireSupported(.ft8)
        var params = try DecodeParams(mode: .ft8)
        params.bandHz = 1000...1000          // an empty band
        XCTAssertThrowsError(try Decoder(mode: .ft8, params: params)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
        params = try DecodeParams(mode: .ft8)
        params.station.call = "A-CALLSIGN-THAT-IS-FAR-TOO-LONG"
        XCTAssertThrowsError(try Decoder(mode: .ft8, params: params), "does not fit the field") { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
    }

    func testTheTransmitFrequencyIsAcceptedByFT8() throws {
        _ = try requireSupported(.ft8)
        XCTAssertTrue(Mode.ft8.capabilities.contains(.transmitFrequency))
        var params = try DecodeParams(mode: .ft8)
        params.txFrequencyHz = 1500
        XCTAssertNoThrow(try Decoder(mode: .ft8, params: params))
    }

    func testAnEmptyBufferDecodesNothing() throws {
        _ = try requireSupported(.ft8)
        let decoder = try Decoder(mode: .ft8)
        XCTAssertEqual(try decoder.decode([Int16]()).count, 0)
        XCTAssertEqual(try decoder.decode([Float]()).count, 0)
    }

    func testInformationBitsOutsideTheLastDecodeAreRefused() throws {
        _ = try requireSupported(.ft8)
        let decoder = try Decoder(mode: .ft8)
        XCTAssertThrowsError(try decoder.informationBits(at: 99)) { error in
            guard let error = error as? MfskError else { return XCTFail("wrong error type") }
            XCTAssertEqual(error.code, .invalidArgument)
            XCTAssertTrue(error.detail.contains("index"), "got '\(error.detail)'")
        }
    }

    func testInformationBitsComeBackForTheRowsJustDecoded() throws {
        let decoder = try Decoder(mode: .ft8)
        let rows = try decoder.decode(try ft8Slot())
        let index = try XCTUnwrap(rows.firstIndex { $0.text.contains(call2) })
        let bits = try decoder.informationBits(at: index)
        XCTAssertEqual(bits.count, Int(rows[index].informationBitCount))
        XCTAssertEqual(bits.count, 91)
        XCTAssertTrue(bits.allSatisfy { $0 <= 1 }, "these are bits, one per byte")
    }

    func testAModeWithNoSlotDecoderCannotBeOpened() throws {
        for mode in [Mode.msk144, .jtty] {
            XCTAssertThrowsError(try Decoder(mode: mode), mode.name) { error in
                XCTAssertFalse((error as? MfskError)?.detail.isEmpty ?? true,
                               "\(mode.name) should say why it has no decoder")
            }
        }
    }
}
