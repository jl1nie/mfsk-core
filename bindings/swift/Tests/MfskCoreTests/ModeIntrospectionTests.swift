// SPDX-License-Identifier: GPL-3.0-only
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
        XCTAssertGreaterThanOrEqual(Runtime.abiVersion, 3,
                                    "the one-decoder-handle surface is ABI v3")
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

    func testEveryModeWithADecoderPublishesItsDefaults() throws {
        for mode in Mode.supported {
            // MSK144 and JTTY have no slot decoder; everything else that the
            // build carries does, and says what it searches by default.
            guard let params = try? DecodeParams(mode: mode) else { continue }
            XCTAssertGreaterThan(params.bandHz.upperBound, params.bandHz.lowerBound, "\(mode.name)")
            XCTAssertNoThrow(try Decoder(mode: mode, params: params), "\(mode.name)")
            if mode.capabilities.contains(.decodeHandle) {
                XCTAssertGreaterThan(try mode.info.decodeFFT1Size, 0,
                                     "\(mode.name) drives the decode handle but reports no slot transform")
            }
        }
    }

    func testTheDefaultsAreTheGUIsAndTheModesOwn() throws {
        // Values the Rust side pins in `decoder_ffi.rs`.
        guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
        let ft8 = try DecodeParams(mode: .ft8)
        XCTAssertEqual(ft8.depth, .deep, "the GUI's default")
        XCTAssertEqual(ft8.ap, .off, "FT8's Enable AP box starts unchecked")
        XCTAssertEqual(ft8.bandHz, 200...4000)
        XCTAssertNil(ft8.rxFrequencyHz)
        XCTAssertNil(ft8.toleranceHz)
        XCTAssertNil(ft8.txFrequencyHz)
        if Mode.ft4.isSupported {
            XCTAssertEqual(try DecodeParams(mode: .ft4).ap, .full)
        }
        if Mode.fst4s60.isSupported {
            XCTAssertEqual(try DecodeParams(mode: .fst4s60).bandHz, 600...1400)
        }
    }

    func testAModeWithNoSlotHasNoParameterBlock() throws {
        // MSK144 is not decoded as a slot; JTTY has no slot at all.
        for mode in [Mode.msk144, .jtty] {
            XCTAssertThrowsError(try DecodeParams(mode: mode), mode.name) { error in
                XCTAssertNotNil(error as? MfskError)
            }
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
