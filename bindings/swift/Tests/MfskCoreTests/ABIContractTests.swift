// SPDX-License-Identifier: GPL-3.0-or-later
//
// The numbers this binding restates, checked against the header that
// owns them. `Mode`'s discriminants and `Capabilities`' bit positions
// are the only ABI values written out in Swift — everything else is
// asked of the library at runtime — so these are the only two places a
// silent divergence could hide.

import CMfsk
import XCTest
@testable import MfskCore

final class ABIContractTests: XCTestCase {
    func testModeDiscriminantsMatchTheHeader() {
        let pairs: [(Mode, MfskMode)] = [
            (.ft8, MFSK_MODE_FT8), (.ft4, MFSK_MODE_FT4),
            (.fst4s15, MFSK_MODE_FST4S15), (.fst4s30, MFSK_MODE_FST4S30),
            (.fst4s60, MFSK_MODE_FST4S60), (.fst4s120, MFSK_MODE_FST4S120),
            (.fst4s300, MFSK_MODE_FST4S300), (.wspr, MFSK_MODE_WSPR),
            (.jt9, MFSK_MODE_JT9), (.jt65, MFSK_MODE_JT65),
            (.q65a15, MFSK_MODE_Q65A15), (.q65a30, MFSK_MODE_Q65A30),
            (.q65a60, MFSK_MODE_Q65A60), (.q65b60, MFSK_MODE_Q65B60),
            (.q65c60, MFSK_MODE_Q65C60), (.q65d60, MFSK_MODE_Q65D60),
            (.q65e60, MFSK_MODE_Q65E60), (.q65d120, MFSK_MODE_Q65D120),
            (.q65e120, MFSK_MODE_Q65E120), (.q65a300, MFSK_MODE_Q65A300),
            (.msk144, MFSK_MODE_MSK144),
            (.uvRobust, MFSK_MODE_UV_ROBUST), (.uvStandard, MFSK_MODE_UV_STANDARD),
            (.uvUltraRobust, MFSK_MODE_UV_ULTRA_ROBUST), (.uvExpress, MFSK_MODE_UV_EXPRESS),
        ]
        // Every case, not a sample: a mode missing from this list is a
        // mode nothing checks.
        XCTAssertEqual(pairs.count, Mode.allCases.count)
        for (swift, c) in pairs {
            XCTAssertEqual(swift.rawValue, UInt32(c.rawValue), "\(swift) is not MfskMode \(c.rawValue)")
        }
    }

    func testCapabilityBitsMatchTheHeader() {
        let pairs: [(Capabilities, Int32)] = [
            (.decodeHandle, MFSK_CAP_DECODE_HANDLE), (.sniper, MFSK_CAP_SNIPER),
            (.apNarrow, MFSK_CAP_AP_NARROW), (.apWideband, MFSK_CAP_AP_WIDEBAND),
            (.sicRounds, MFSK_CAP_SIC_ROUNDS), (.sicEarly, MFSK_CAP_SIC_EARLY),
            (.osd, MFSK_CAP_OSD), (.equalisation, MFSK_CAP_EQ_MODE),
            (.strictness, MFSK_CAP_STRICTNESS), (.budget, MFSK_CAP_BUDGET),
            (.knownFilter, MFSK_CAP_KNOWN_FILTER), (.knownSubtract, MFSK_CAP_KNOWN_SUBTRACT),
            (.fftCache, MFSK_CAP_FFT_CACHE), (.onResult, MFSK_CAP_ON_RESULT),
            (.encode, MFSK_CAP_ENCODE),
        ]
        for (swift, c) in pairs {
            XCTAssertEqual(swift.rawValue, UInt64(c))
        }
    }

    func testQ65DiscriminantsMatchTheHeader() {
        // Q65's own numbering, which is not MfskMode's and not in
        // slot-length order. It reaches C only because `cbindgen.toml`
        // asks for the enum by name — every `mfsk_q65_*` function takes
        // it as `uint32_t`, so no signature mentions it.
        let pairs: [(Q65SubMode, MfskQ65SubMode)] = [
            (.a30, MFSK_Q65_SUB_MODE_A30), (.a60, MFSK_Q65_SUB_MODE_A60),
            (.b60, MFSK_Q65_SUB_MODE_B60), (.c60, MFSK_Q65_SUB_MODE_C60),
            (.d60, MFSK_Q65_SUB_MODE_D60), (.e60, MFSK_Q65_SUB_MODE_E60),
            (.a15, MFSK_Q65_SUB_MODE_A15), (.d120, MFSK_Q65_SUB_MODE_D120),
            (.e120, MFSK_Q65_SUB_MODE_E120), (.a300, MFSK_Q65_SUB_MODE_A300),
        ]
        XCTAssertEqual(pairs.count, Q65SubMode.allCases.count)
        for (swift, c) in pairs {
            XCTAssertEqual(swift.rawValue, UInt32(c.rawValue), "\(swift)")
        }
        XCTAssertEqual(Q65FadingModel.gaussian.rawValue,
                       UInt32(MFSK_Q65_FADING_MODEL_GAUSSIAN.rawValue))
        XCTAssertEqual(Q65FadingModel.lorentzian.rawValue,
                       UInt32(MFSK_Q65_FADING_MODEL_LORENTZIAN.rawValue))
    }

    func testStatusCodesMatchTheHeader() {
        XCTAssertEqual(MfskError.Code.nullPointer.rawValue, MFSK_STATUS_NULL_POINTER.rawValue)
        XCTAssertEqual(MfskError.Code.invalidArgument.rawValue, MFSK_STATUS_INVALID_ARG.rawValue)
        XCTAssertEqual(MfskError.Code.unknownProtocol.rawValue, MFSK_STATUS_UNKNOWN_PROTOCOL.rawValue)
        XCTAssertEqual(MfskError.Code.decodeFailed.rawValue, MFSK_STATUS_DECODE_FAILED.rawValue)
        XCTAssertEqual(MfskError.Code.internalError.rawValue, MFSK_STATUS_INTERNAL.rawValue)
        XCTAssertEqual(MfskError.Code.unsupported.rawValue, MFSK_STATUS_UNSUPPORTED.rawValue)
    }

    func testRowFlagBitsMatchTheHeader() {
        // The only other bit this binding names. `flags` is a `uint8_t`,
        // so the cast is the check as much as the comparison is.
        XCTAssertEqual(UInt8(MFSK_DECODE_FLAG_HASH_RESOLVED), 1 << 0)
    }

    func testPackedMessageWidthMatchesTheHeader() {
        // 77 is the payload the pack77 family writes, one bit per byte.
        XCTAssertEqual(Message.bitCount, 77)
        // And the row's text capacity, which `text(resolvedBy:)` sizes from.
        XCTAssertEqual(Int(MFSK_DECODE_TEXT_LEN), 64)
    }

    func testSizeVersionedStructsAreWhatTheLibraryExpects() throws {
        // Each of these is passed with `size = sizeof(...)`; if the
        // header and the linked library ever disagreed, the library
        // would write only the prefix the caller declared. Nothing here
        // can detect that — but a mode's defaults coming back usable
        // proves the round trip works at the size this binding sends.
        let params = try DecodeParams(mode: .ft8)
        XCTAssertGreaterThan(params.maxCandidates, 0)
        XCTAssertGreaterThan(params.frequencyRangeHz.upperBound, params.frequencyRangeHz.lowerBound)
        XCTAssertNil(params.frequencyHintHz, "the ABI's 'unset' hint is NaN, which must arrive as nil")
    }
}
