// SPDX-License-Identifier: GPL-3.0-or-later
//
// Written the way a consumer would have to write it: enumerate what the
// build has, ask each mode what it supports, and assert on the answer.
// Anything checked against a list written here would be testing this
// file, not the library.

import XCTest
@testable import MfskCore

final class ModeIntrospectionTests: XCTestCase {
    func testTheBuildDeclaresModes() throws {
        let modes = Mode.supported
        XCTAssertFalse(modes.isEmpty, "this build claims to support no modes at all")
        XCTAssertGreaterThanOrEqual(Runtime.abiVersion, 2,
                                    "the introspection surface landed in ABI v2")
    }

    func testNamesRoundTrip() throws {
        for mode in Mode.supported {
            let name = mode.name
            XCTAssertFalse(name.isEmpty)
            XCTAssertEqual(try Mode(name: name), mode, "\(name) did not round-trip")
            XCTAssertEqual(try mode.info.name, name,
                           "mfsk_mode_name disagrees with MfskModeInfo::name")
        }
    }

    func testAnUnknownNameIsDistinguishedFromAnAbsentMode() {
        // The distinction the ABI documents: a typo is invalidArgument,
        // a real mode this build lacks is unknownProtocol. A failable
        // initialiser would have collapsed both to nil.
        do {
            _ = try Mode(name: "definitely-not-a-mode")
            XCTFail("a nonsense name should not resolve")
        } catch let error as MfskError {
            XCTAssertEqual(error.code, .invalidArgument)
        } catch {
            XCTFail("unexpected error: \(error)")
        }
    }

    func testEveryHandleModePublishesAUsableSearch() throws {
        for mode in Mode.supported where mode.capabilities.contains(.decodeHandle) {
            let defaults = try mode.decodeDefaults
            XCTAssertGreaterThan(defaults.frequencyRangeHz.upperBound,
                                 defaults.frequencyRangeHz.lowerBound, "\(mode.name)")
            XCTAssertGreaterThan(defaults.maxCandidates, 0, "\(mode.name)")
            // The trap `syncScale` exists for: on a baseline-normalised
            // scale, noise sits at ~1.0 by construction, so a threshold
            // at or below that admits every peak in the band.
            if defaults.syncScale == .baselineNormalised {
                XCTAssertGreaterThan(defaults.syncMin, 1.0, "\(mode.name)")
            }
            XCTAssertGreaterThan(try mode.info.decodeFFT1Size, 0,
                                 "\(mode.name) drives the decode handle but reports no slot transform")
        }
    }

    func testTheSniperIsFT8Only() throws {
        for mode in Mode.supported where mode.capabilities.contains(.sniper) {
            XCTAssertEqual(mode, .ft8, "\(mode.name) advertises a sniper, which is FT8-only by design")
        }
    }

    func testGeometryIsSelfConsistent() throws {
        for mode in Mode.supported {
            let info = try mode.info
            XCTAssertEqual(info.mode, mode)
            XCTAssertGreaterThan(info.slotSeconds, 0, "\(mode.name)")
            XCTAssertEqual(info.slotSamples12k, UInt32((info.slotSeconds * 12_000).rounded()),
                           "\(mode.name): slot_samples_12k should be t_slot_s made exact")
            if info.capabilities.contains(.encode), mode.symbolCount > 0 {
                // A tone stage means both halves of stage 3 are sized.
                XCTAssertGreaterThan(mode.synthesisedFrameLength, 0, "\(mode.name)")
            }
        }
    }

    func testAModeThisBuildLacksIsReportedAsSuch() throws {
        // MSK144 is addressed by the ABI and dispatched specially; it has
        // no registry entry, so whether it is "supported" is a property
        // of the build. Either answer is legal — what must hold is that
        // `isSupported` and `info` agree.
        for mode in Mode.allCases {
            XCTAssertEqual(mode.isSupported, (try? mode.info) != nil, "\(mode.name)")
        }
    }
}
