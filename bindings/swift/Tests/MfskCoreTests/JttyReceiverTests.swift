// SPDX-License-Identifier: GPL-3.0-or-later
//
// The JTTY receiver, fed upstream's own sample recording in chunks the
// way an audio callback would. There is no JTTY transmit path in the
// ABI, so the audio is the vendored golden file rather than synthesised.

import Foundation
import XCTest
@testable import MfskCore

final class JttyReceiverTests: XCTestCase {
    private static let expected = "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!"

    /// `bindings/swift/Tests/MfskCoreTests/<this file>` → the repository root.
    private static func recording() throws -> [Int16] {
        var url = URL(fileURLWithPath: #filePath)
        for _ in 0..<5 { url.deleteLastPathComponent() }
        url.appendPathComponent("embedded-poc/assets/golden/jtty/260807_134110.wav")
        let bytes = [UInt8](try Data(contentsOf: url))
        let tag = Array("data".utf8)
        guard let at = (0..<(bytes.count - 8)).first(where: { Array(bytes[$0..<($0 + 4)]) == tag })
        else {
            XCTFail("no data chunk in \(url.path)")
            return []
        }
        let body = at + 8
        return (0..<((bytes.count - body) / 2)).map { i in
            Int16(bitPattern: UInt16(bytes[body + 2 * i]) | (UInt16(bytes[body + 2 * i + 1]) << 8))
        }
    }

    private func requireJtty() throws {
        guard (try? Mode.jtty.info) != nil else { throw XCTSkip("no JTTY in this build") }
    }

    func testTheModeIsAStreamReceiverWithNoSlotDecode() throws {
        try requireJtty()
        let info = try Mode.jtty.info
        XCTAssertEqual(info.name, "JTTY")
        XCTAssertEqual(info.slotSamples12k, 22_656, "one 1.888 s frame at 12 kHz")
        XCTAssertTrue(info.capabilities.contains(.streamReceiver))
        XCTAssertFalse(info.capabilities.contains(.decodeHandle))
        XCTAssertEqual(try Mode(name: "JTTY"), .jtty)
    }

    func testTheRecordingsMessageComesOutWhenFedInChunks() throws {
        try requireJtty()
        let pcm = try Self.recording()
        XCTAssertGreaterThan(pcm.count, 12_000)
        let receiver = try JttyReceiver()
        var latest: [UInt64: JttyUpdate] = [:]
        for start in stride(from: 0, to: pcm.count, by: 4096) {
            for u in try receiver.push(Array(pcm[start..<min(start + 4096, pcm.count)])) {
                latest[u.id] = u
            }
        }
        XCTAssertTrue(try receiver.poll().isEmpty, "push drains after itself")
        let message = try XCTUnwrap(latest.values.first { $0.text == Self.expected },
                                    "got \(latest.values.map(\.text))")
        XCTAssertTrue(message.isComplete)
        XCTAssertEqual(message.frequencyHz, 1507, accuracy: 3)
    }

    func testAnotherRateGoesThroughTheResamplerAndFloatIsAccepted() throws {
        try requireJtty()
        let pcm = try Self.recording()
        var up = [Int16](repeating: 0, count: pcm.count * 2)
        for i in 0..<(pcm.count - 1) {
            up[2 * i] = pcm[i]
            up[2 * i + 1] = Int16((Int32(pcm[i]) + Int32(pcm[i + 1])) / 2)
        }
        let receiver = try JttyReceiver(sampleRate: 24_000)
        var seen: [JttyUpdate] = []
        for start in stride(from: 0, to: up.count, by: 8192) {
            let chunk = up[start..<min(start + 8192, up.count)].map { Float($0) / 32768 }
            seen += try receiver.push(chunk)
        }
        XCTAssertTrue(seen.contains { $0.text == Self.expected })
    }

    func testFinishReportsWhatWasLeftOpenAndResetStartsOver() throws {
        try requireJtty()
        let pcm = try Self.recording()
        let receiver = try JttyReceiver()
        // 12 s of a 22 s message: open, not complete.
        try receiver.push(Array(pcm[0..<(12 * 12_000)]))
        let left = try receiver.finish()
        XCTAssertTrue(left.contains { $0.text.hasPrefix("RAN ALL NIGHT") && !$0.isComplete },
                      "got \(left.map(\.text))")
        try receiver.reset()
        let again = try receiver.push(pcm)
        XCTAssertTrue(again.contains { $0.text == Self.expected && $0.isComplete })
    }

    func testBadParametersAreRefused() throws {
        try requireJtty()
        var params = JttyParams()
        params.bandHz = 2000...2000
        XCTAssertThrowsError(try JttyReceiver(params: params)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
        XCTAssertThrowsError(try JttyReceiver(sampleRate: 0)) { error in
            XCTAssertEqual((error as? MfskError)?.code, .invalidArgument)
        }
        let receiver = try JttyReceiver()
        params.bandHz = 200...2800
        params.toleranceHz = -1
        XCTAssertThrowsError(try receiver.setParams(params))
    }
}
