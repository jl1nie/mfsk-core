// SPDX-License-Identifier: GPL-3.0-or-later
//
// Q65's four decode strategies, each against audio this test
// synthesised. The point is not sensitivity — a clean signal decodes
// on any of them — it is that each entry point is wired to the right C
// function with its arguments in the right order, which is the failure
// mode a thin binding actually has.

import CMfsk
import XCTest
@testable import MfskCore

final class Q65Tests: XCTestCase {
    // Q65-30A: the terrestrial workhorse, and at 30 s the cheapest
    // sub-mode to decode four times in one test file.
    private let subMode = Q65SubMode.a30
    private let myCall = "JL1NIE"
    private let hisCall = "K1ABC"
    private let grid = "FN42"
    private let frequency: Float = 1000

    private func audio() throws -> [Float] {
        guard subMode.mode.isSupported else { throw XCTSkip("no Q65 in this build") }
        return try Q65.encode(subMode: subMode, call1: "CQ", call2: hisCall,
                              gridOrReport: grid, frequencyHz: frequency)
    }

    func testSubModeNumberingIsItsOwn() {
        // Q65's discriminants are not MfskMode's, and not in
        // slot-length order: a15 is 6 because it was appended rather
        // than inserted. A wrapper that assumed either would address
        // the wrong sub-mode with nothing to say so.
        XCTAssertEqual(Q65SubMode.a15.rawValue, 6)
        XCTAssertEqual(Q65SubMode.a15.mode, .q65a15)
        XCTAssertNotEqual(Q65SubMode.a15.rawValue, Mode.q65a15.rawValue)
        for sub in Q65SubMode.allCases {
            XCTAssertEqual(Q65SubMode(sub.mode), sub, "\(sub) did not round-trip through Mode")
        }
        XCTAssertNil(Q65SubMode(.ft8))
    }

    func testPlainDecodeRoundTrips() throws {
        let pcm = try audio()
        let info = try subMode.mode.info
        XCTAssertEqual(pcm.count, Int(info.totalSymbols * info.samplesPerSymbol))

        let rows = try Q65.decode(pcm, subMode: subMode)
        XCTAssertTrue(rows.contains { $0.text.contains(hisCall) },
                      "expected \(hisCall) in \(rows.map(\.text))")
        // Every row reports the concrete sub-mode, which is how a caller
        // tells Q65-30A's rows from Q65-60B's in one list.
        XCTAssertTrue(rows.allSatisfy { $0.mode == .q65a30 })
        XCTAssertEqual(rows.first?.frequencyHz ?? 0, frequency, accuracy: 5)
    }

    func testAPHintDecodeRoundTrips() throws {
        let pcm = try audio()
        // Message-field order: the transmission is `CQ K1ABC FN42`, so
        // call1 is "CQ". Not "the transmitting station".
        let hint = Q65.APHint(call1: "CQ", call2: hisCall, grid: grid)
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, apHint: hint)
            .contains { $0.text.contains(hisCall) })
    }

    func testASwappedAPHintRemovesTheDecodeRatherThanCostingSensitivity() throws {
        // The trap the field order is worth documenting for: a hint
        // locks message bits, so hinting the *right* callsigns in the
        // *wrong* fields is not a weaker hint, it is a wrong one — and
        // a clean signal that decodes four other ways decodes not at
        // all. This is the assertion that made the binding's doc
        // comment say "fields in order, not roles".
        let pcm = try audio()
        let swapped = Q65.APHint(call1: hisCall, call2: "CQ", grid: grid)
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, apHint: swapped).isEmpty)
    }

    func testAPHintWithNoHintsIsALegalCall() throws {
        // All four fields nil: the ABI reads NULL as "not supplied",
        // which is not the same as an empty string, and the call still
        // has to work.
        let pcm = try audio()
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, apHint: Q65.APHint())
            .contains { $0.text.contains(hisCall) })
    }

    func testFadingDecodeRoundTrips() throws {
        let pcm = try audio()
        // 0.1 is the near-AWGN end of b90_ts; the argument is what is
        // under test, not the fading gain.
        let rows = try Q65.decode(pcm, subMode: subMode, fadingB90Ts: 0.1, model: .gaussian)
        XCTAssertTrue(rows.contains { $0.text.contains(hisCall) })
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, fadingB90Ts: 0.1, model: .lorentzian)
            .contains { $0.text.contains(hisCall) })
    }

    func testAPListDecodeRoundTrips() throws {
        let pcm = try audio()
        // The transmission is `CQ K1ABC FN42`, so the template set built
        // from the pair has to contain the CQ form for this to decode.
        let rows = try Q65.decode(pcm, subMode: subMode,
                                  apList: Q65.APList(myCall: myCall, hisCall: hisCall, hisGrid: grid))
        XCTAssertTrue(rows.contains { $0.text.contains(hisCall) },
                      "expected \(hisCall) in \(rows.map(\.text))")
    }

    func testAHashTableIsAcceptedByEveryStrategy() throws {
        let pcm = try audio()
        let table = try CallsignHashTable()
        try table.insert(myCall)
        try table.insert(hisCall)

        // A standard message carries no hashed half, so the table cannot
        // change this text — what is under test is that the handle
        // crosses correctly and is not confused with a null. Resolution
        // itself needs a type-4 Q65 message, which `mfsk_encode_q65`
        // does not pack (it synthesises standard messages only), so it
        // is covered on the Rust side in `q65_ffi.rs` instead.
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, hashTable: table)
            .contains { $0.text.contains(hisCall) })
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode,
                                     apHint: Q65.APHint(call1: "CQ"), hashTable: table)
            .contains { $0.text.contains(hisCall) })
        XCTAssertTrue(try Q65.decode(pcm, subMode: subMode, fadingB90Ts: 0.1, hashTable: table)
            .contains { $0.text.contains(hisCall) })
    }

    func testQ65HasNoDecodeSession() throws {
        guard subMode.mode.isSupported else { throw XCTSkip("no Q65 in this build") }
        // The capability bit is what says which modes the session
        // drives; Q65 is addressed through this file instead.
        XCTAssertFalse(subMode.mode.capabilities.contains(.decodeHandle))
        XCTAssertThrowsError(try DecodeSession(mode: subMode.mode))
    }

    func testABogusSubModeIsRefusedWithADetail() throws {
        // Not reachable through `Q65SubMode`, which is the point of the
        // enum — but the ABI takes `uint32_t`, so the refusal has to
        // exist for a value that arrives from a config file.
        var rows = [MfskDecode](repeating: MfskDecode(), count: 4)
        var found: UInt = 0
        // A real (if useless) audio buffer, so the sub-mode is what
        // fails: the null-pointer check runs first and would otherwise
        // mask it.
        let silence = [Float](repeating: 0, count: 1200)
        let status = silence.withUnsafeBufferPointer { audio in
            rows.withUnsafeMutableBufferPointer { buffer in
                mfsk_q65_decode(9999, audio.baseAddress, UInt(audio.count), 12_000, nil,
                                buffer.baseAddress, UInt(buffer.count), &found)
            }
        }
        XCTAssertEqual(status, MFSK_STATUS_INVALID_ARG)
        XCTAssertTrue((globalLastError() ?? "").contains("sub-mode"), "got '\(globalLastError() ?? "")'")
    }
}
