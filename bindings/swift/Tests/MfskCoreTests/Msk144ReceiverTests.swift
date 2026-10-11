// SPDX-License-Identifier: GPL-3.0-only
//
// The MSK144 receiver (#497), fed WSJT-X's own sample recording in chunks
// the way an audio callback would. WSJT-X decodes two pings from it.

import Foundation
import XCTest
@testable import MfskCore

final class Msk144ReceiverTests: XCTestCase {
    /// `bindings/swift/Tests/MfskCoreTests/<this file>` → the repository root.
    private static func recording() throws -> [Int16] {
        var url = URL(fileURLWithPath: #filePath)
        for _ in 0..<5 { url.deleteLastPathComponent() }
        url.appendPathComponent("embedded-poc/assets/golden/msk144/181211_120800.wav")
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

    private func requireMsk144() throws {
        guard Mode.msk144.isSupported else { throw XCTSkip("no MSK144 in this build") }
    }

    func testTheModeIsAStreamReceiverWithNoSlotDecode() throws {
        try requireMsk144()
        XCTAssertTrue(Mode.msk144.capabilities.contains(.streamReceiver))
        XCTAssertFalse(Mode.msk144.capabilities.contains(.decodeHandle))
    }

    func testTheRecordingsPingsComeOutWhenFedInChunks() throws {
        try requireMsk144()
        let pcm = try Self.recording()
        let receiver = try Msk144Receiver(frequencyHz: 1477, toleranceHz: 60, depth: 3)
        var got: [Msk144Decode] = []
        for start in stride(from: 0, to: pcm.count, by: 1000) {
            got += try receiver.push(Array(pcm[start..<min(start + 1000, pcm.count)]))
        }
        let w4imd = try XCTUnwrap(got.first { $0.text == "CQ W4IMD EM84" }, "got \(got.map(\.text))")
        XCTAssertEqual(w4imd.seconds, 4.6, accuracy: 0.3)
        XCTAssertTrue(got.contains { $0.text == "CQ KD9VV EN71" }, "got \(got.map(\.text))")
        try receiver.newPeriod()
        try receiver.reset()
    }

    func testABadDepthIsRefused() throws {
        try requireMsk144()
        XCTAssertThrowsError(try Msk144Receiver(depth: 9)) { error in
            XCTAssertNotNil(error as? MfskError)
        }
    }
}
