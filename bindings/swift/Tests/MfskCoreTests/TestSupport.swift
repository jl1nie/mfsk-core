// SPDX-License-Identifier: GPL-3.0-or-later
//
// Helpers shared by the decoder tests.

import Foundation
import XCTest
@testable import MfskCore

/// `frame` placed `offsetSeconds` into a silent period of `slotSeconds`,
/// 12 kHz — for the modes (WSPR, JT9, JT65, Q65) whose encoders return the
/// transmission and not a padded period.
func placeInSlot(_ frame: [Float], offsetSeconds: Float, slotSeconds: Int) -> [Float] {
    var slot = [Float](repeating: 0, count: slotSeconds * 12_000)
    let at = Int((offsetSeconds * 12_000).rounded())
    for (i, sample) in frame.enumerated() where at + i < slot.count {
        slot[at + i] += sample
    }
    return slot
}

/// A thread-safe sink, because the delivery contract explicitly allows
/// concurrent callbacks from worker threads.
final class Collected {
    private let lock = NSLock()
    private var rows: [Decode] = []

    func append(_ row: Decode) {
        lock.lock()
        defer { lock.unlock() }
        rows.append(row)
    }

    var snapshot: [Decode] {
        lock.lock()
        defer { lock.unlock() }
        return rows
    }
}

/// The budget predicate can be polled from rayon workers, so even a counter
/// needs a lock.
final class Counter {
    private let lock = NSLock()
    private var count = 0

    func increment() {
        lock.lock()
        defer { lock.unlock() }
        count += 1
    }

    var value: Int {
        lock.lock()
        defer { lock.unlock() }
        return count
    }
}

/// Two FT8 stations in one period, so a budget has something to cut.
func twoStationFT8Slot() throws -> [Int16] {
    guard Mode.ft8.isSupported else { throw XCTSkip("no FT8 in this build") }
    let a = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JA1ABC", report: "PM95",
                                        frequencyHz: 1200)
    let b = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "VK3NV", report: "QF22",
                                        frequencyHz: 1800)
    return zip(a, b).map { $0.addingReportingOverflow($1).partialValue }
}
