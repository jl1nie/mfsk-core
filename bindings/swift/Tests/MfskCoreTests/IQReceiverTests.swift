// SPDX-License-Identifier: GPL-3.0-only
//
// The IQ receiver's control surface, from `mfsk-ffi/tests/iq_ffi.rs`'s
// `errors_are_statuses_not_crashes`. The signal-in, decode-out path is not
// repeated here: building a double-sideband IQ scene takes a polyphase
// resampler this package does not have, and the Rust test already drives that
// through every wire format.

import XCTest
@testable import MfskCore

final class IQReceiverTests: XCTestCase {
    private let rate: UInt32 = 48_000
    private let center = 14_077_000.0
    private var dial: Double { center + 6_000 }

    private func requireFT8AndWSPR() throws {
        guard Mode.ft8.isSupported, Mode.wspr.isSupported else {
            throw XCTSkip("needs FT8 and WSPR in this build")
        }
    }

    func testABadOpenIsAStatusNotACrash() {
        func code(_ build: () throws -> IQReceiver) -> MfskError.Code? {
            do { _ = try build(); return nil } catch { return (error as? MfskError)?.code }
        }
        XCTAssertEqual(code { try IQReceiver(sampleRate: 8_000, centerHz: center) }, .invalidArgument,
                       "below 12 kHz")
        XCTAssertEqual(code { try IQReceiver(sampleRate: rate, centerHz: .nan) }, .invalidArgument,
                       "a non-finite centre")
        XCTAssertEqual(code { try IQReceiver(sampleRate: 30_000, centerHz: center, channelizer: .pfb) },
                       .invalidArgument, "the bank at a rate none fits")
    }

    func testAChannelThatCannotBePlacedIsRefused() throws {
        try requireFT8AndWSPR()
        let rx = try IQReceiver(sampleRate: rate, centerHz: center)
        func code(_ dial: Double, _ mode: Mode) -> MfskError.Code? {
            do { try rx.addChannel(dialHz: dial, mode: mode); return nil }
            catch { return (error as? MfskError)?.code }
        }
        XCTAssertEqual(code(center - 1_000, .ft8), .invalidArgument, "DC inside the audio window")
        XCTAssertEqual(code(center + 40_000, .ft8), .invalidArgument, "past the band edge")
        XCTAssertEqual(code(dial, .msk144), .invalidArgument, "a mode the receiver does not carry")
        XCTAssertEqual(code(.nan, .ft8), .invalidArgument, "a non-finite dial")
    }

    func testARetunePausesAndResumesAChannel() throws {
        try requireFT8AndWSPR()
        let rx = try IQReceiver(sampleRate: rate, centerHz: center)
        let channel = try rx.addChannel(dialHz: dial, mode: .wspr)
        XCTAssertEqual(rx.state(ofChannel: channel), .active)

        let away = try rx.retune(centerHz: center + 200_000)
        XCTAssertEqual(away.paused, 1)
        XCTAssertEqual(away.resumed, 0)
        XCTAssertEqual(rx.state(ofChannel: channel), .paused)

        let back = try rx.retune(centerHz: center)
        XCTAssertEqual(back.paused, 0)
        XCTAssertEqual(back.resumed, 1)
        XCTAssertEqual(rx.state(ofChannel: channel), .active)

        try rx.removeChannel(channel)
        XCTAssertNil(rx.state(ofChannel: channel))
        XCTAssertThrowsError(try rx.removeChannel(channel)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
    }

    func testTheChannelsDecoderIsBorrowedAndConfigurable() throws {
        try requireFT8AndWSPR()
        let rx = try IQReceiver(sampleRate: rate, centerHz: center)
        let channel = try rx.addChannel(dialHz: dial, mode: .ft8)
        let decoder = try XCTUnwrap(rx.decoder(forChannel: channel))
        XCTAssertTrue(decoder === rx.decoder(forChannel: channel), "one wrapper per channel")
        XCTAssertEqual(decoder.mode, .ft8)
        XCTAssertNoThrow(try decoder.addCallsign("VK3NV"))
        XCTAssertNoThrow(try decoder.clear())
        var params = try DecodeParams(mode: .ft8)
        params.bandHz = 300...2_800
        XCTAssertNoThrow(try decoder.setParams(params))
        XCTAssertNil(rx.decoder(forChannel: channel + 7), "no such channel")
    }

    func testAnOptionTheModeLacksIsRefusedWhenTheChannelIsAdded() throws {
        try requireFT8AndWSPR()
        let rx = try IQReceiver(sampleRate: rate, centerHz: center)
        var extras = Extras()
        extras.noiseBlanker = .percent(5)
        XCTAssertThrowsError(try rx.addChannel(dialHz: dial, mode: .ft8, extras: extras)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .unsupported)
        }
    }

    func testSilenceDecodesNothingAndTheClockCounts() throws {
        try requireFT8AndWSPR()
        let rx = try IQReceiver(sampleRate: rate, centerHz: center, format: .cs16)
        try rx.addChannel(dialHz: dial, mode: .ft8)
        XCTAssertNil(try rx.poll())
        XCTAssertEqual(rx.pending, 0)
        XCTAssertNoThrow(try rx.push([]))
        // Two seconds of silence, 4 bytes per cs16 complex sample.
        try rx.push([UInt8](repeating: 0, count: Int(rate) * 2 * 4))
        XCTAssertEqual(rx.samplesIn, UInt64(rate) * 2)
        XCTAssertEqual(try rx.drain().count, 0)
        XCTAssertNoThrow(try rx.gap(lostSamples: 1_000))
        XCTAssertEqual(rx.samplesIn, UInt64(rate) * 2 + 1_000, "a gap advances the clock")
        let change = try rx.setTime(utcNanoseconds: 1_700_000_000 * 1_000_000_000)
        XCTAssertEqual(change, .first)
    }
}
