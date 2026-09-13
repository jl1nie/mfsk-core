// SPDX-License-Identifier: GPL-3.0-or-later
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

    func testFT8SlotRoundTrips() throws {
        _ = try requireSupported(.ft8)
        let slot = try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        let session = try DecodeSession(mode: .ft8)
        let rows = try session.decode(slot)
        XCTAssertTrue(rows.contains { $0.text.contains(call2) },
                      "expected \(call2) in \(rows.map(\.text))")
        for row in rows {
            XCTAssertEqual(row.mode, .ft8)
            XCTAssertEqual(row.frequencyHz, 1500, accuracy: 5)
        }
    }

    func testFT4SlotRoundTrips() throws {
        _ = try requireSupported(.ft4)
        let slot = try Mode.ft4.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        let session = try DecodeSession(mode: .ft4)
        XCTAssertTrue(try session.decode(slot).contains { $0.text.contains(call2) })
    }

    func testFloatAndIntegerPathsAgree() throws {
        _ = try requireSupported(.ft8)
        let slot = try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        let floats = slot.map { Float($0) / 32768.0 }
        let session = try DecodeSession(mode: .ft8)
        let fromInts = try session.decode(slot).map(\.text).sorted()
        let fromFloats = try session.decode(floats).map(\.text).sorted()
        XCTAssertEqual(fromInts, fromFloats,
                       "the f32 entry point is not a lossier wrapper; it should find the same rows")
    }

    func testAFrequencyBandThatExcludesTheSignalFindsNothing() throws {
        _ = try requireSupported(.ft8)
        let slot = try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        var params = try DecodeParams(mode: .ft8)
        params.frequencyRangeHz = 2500...2900
        let session = try DecodeSession(mode: .ft8)
        let rows = try session.decode(slot, params: params)
        XCTAssertFalse(rows.contains { $0.text.contains(call2) },
                       "a 2500-2900 Hz search still found a 1500 Hz signal")
    }

    func testAnAPHintIsAcceptedWhereTheModeAdvertisesOne() throws {
        _ = try requireSupported(.ft8)
        XCTAssertTrue(Mode.ft8.capabilities.contains(.apWideband))
        let slot = try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        var params = try DecodeParams(mode: .ft8)
        params.apHint = DecodeParams.APHint(call1: call2, call2: "CQ", grid: grid)
        params.frequencyHintHz = 1500
        let session = try DecodeSession(mode: .ft8)
        XCTAssertTrue(try session.decode(slot, params: params).contains { $0.text.contains(call2) })
    }

    func testAParameterTheModeCannotHonourFailsAtOpen() throws {
        // "A parameter the mode does not support is an error here, not a
        // field silently dropped at decode time" — so pick a mode
        // without the capability and check it actually refuses.
        guard let victim = Mode.supported.first(where: {
            $0.capabilities.contains(.decodeHandle) && !$0.capabilities.contains(.sicEarly)
        }) else {
            throw XCTSkip("no handle-driven mode in this build lacks sicEarly")
        }
        var params = try DecodeParams(mode: victim)
        params.sicEarly = true
        XCTAssertThrowsError(try DecodeSession(mode: victim, params: params)) { error in
            guard let error = error as? MfskError else { return XCTFail("wrong error type") }
            XCTAssertFalse(error.detail.isEmpty, "the refusal should say why")
        }
    }

    func testTooLittleAudioDecodesNothingRatherThanFailing() throws {
        _ = try requireSupported(.ft8)
        let session = try DecodeSession(mode: .ft8)
        // `MfskStatus`' doc lists "an audio buffer too short for the
        // protocol's slot length" under invalidArgument, but the session
        // path does not length-check: 1000 samples of silence decode to
        // nothing and report success. Pinned as it behaves, because a
        // caller feeding a partial slot needs to know which of the two
        // it gets.
        XCTAssertEqual(try session.decode([Int16](repeating: 0, count: 1000)).count, 0)
        XCTAssertNil(session.lastError)
    }

    func testAnEmptyBufferDecodesNothing() throws {
        _ = try requireSupported(.ft8)
        let session = try DecodeSession(mode: .ft8)
        // Not an error either — `n_samples == 0` is a legal call, and
        // the null-pointer check it might have tripped does not fire
        // because Swift hands an empty array a non-null address.
        XCTAssertEqual(try session.decode([Int16]()).count, 0)
    }

    func testInformationBitsOutsideTheLastDecodeAreRefused() throws {
        _ = try requireSupported(.ft8)
        let session = try DecodeSession(mode: .ft8)
        XCTAssertThrowsError(try session.informationBits(at: 99)) { error in
            guard let error = error as? MfskError else { return XCTFail("wrong error type") }
            XCTAssertEqual(error.code, .invalidArgument)
            XCTAssertTrue(error.detail.contains("index"), "got '\(error.detail)'")
        }
        // And it is the *thread-local* slot that carries it: this one
        // call takes the handle as `const*`, so it has nowhere to record
        // a per-handle error. The binding falls back to the global for
        // exactly this case.
        XCTAssertNil(session.lastError)
    }

    func testAModeWithNoDecodeHandleCannotOpenASession() throws {
        guard let victim = Mode.supported.first(where: {
            !$0.capabilities.contains(.decodeHandle)
        }) else { throw XCTSkip("every mode in this build drives the decode handle") }
        XCTAssertThrowsError(try DecodeSession(mode: victim)) { error in
            XCTAssertFalse((error as? MfskError)?.detail.isEmpty ?? true,
                           "\(victim.name) should say why it has no session")
        }
    }

    func testInformationBitsComeBackForTheRowsJustDecoded() throws {
        _ = try requireSupported(.ft8)
        let slot = try Mode.ft8.synthesiseSlot(call1: call1, call2: call2, report: grid,
                                               frequencyHz: 1500)
        let session = try DecodeSession(mode: .ft8)
        let rows = try session.decode(slot)
        let index = try XCTUnwrap(rows.firstIndex { $0.text.contains(call2) })
        let bits = try session.informationBits(at: index)
        XCTAssertEqual(bits.count, Int(rows[index].informationBitCount))
        XCTAssertTrue(bits.allSatisfy { $0 <= 1 }, "these are bits, one per byte")
    }
}
